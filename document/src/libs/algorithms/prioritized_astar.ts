import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 prioritized planning. python/mrmp/mapf/prioritized_astar.py의
// 정확한 미러 — 고정 이웃 순서(up/down/left/right/wait), (f, seq) lexicographic
// tie-break, stay-at-goal 점유, pop 시점 goal guard, 정적 flood-fill 사전 검사로
// 유한화한 시간축까지 같아 trace가 필드 단위로 일치한다. 시각화용이 아니라
// parity 검증의 한 축이다 (scripts/check-engine-parity.mjs).

interface HeapNode {
    f: number;
    // push 순서 카운터 — f 동률을 발생 순서로 깨는 Python heapq 튜플의 두 번째 항.
    seq: number;
    cell: Cell;
    t: number;
}

// (f, seq) lexicographic min-heap. 이진 힙이고 비교가 전순위가 아니라도 된다 —
// pop 순서만 같으면 되므로.
class Heap {
    private a: HeapNode[] = [];

    get size(): number { return this.a.length }

    push(n: HeapNode): void {
        const a = this.a;
        a.push(n);
        let i = a.length - 1;
        while (i > 0) {
            const p = (i - 1) >> 1;
            if (this.less(a[i], a[p])) {
                [a[i], a[p]] = [a[p], a[i]];
                i = p;
            } else break;
        }
    }

    pop(): HeapNode {
        const a = this.a;
        const top = a[0];
        const last = a.pop()!;
        if (a.length > 0) {
            a[0] = last;
            let i = 0;
            for (;;) {
                const l = 2 * i + 1, r = l + 1;
                let m = i;
                if (l < a.length && this.less(a[l], a[m])) m = l;
                if (r < a.length && this.less(a[r], a[m])) m = r;
                if (m === i) break;
                [a[i], a[m]] = [a[m], a[i]];
                i = m;
            }
        }
        return top;
    }

    private less(x: HeapNode, y: HeapNode): boolean {
        return x.f !== y.f ? x.f < y.f : x.seq < y.seq;
    }
}

const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];

// 도착 후에도 그 셀을 계속 점유하는 Python과 동일한 stay-at-goal 점유 판정.
const occupiedAt = (path: Cell[], t: number): Cell =>
    t < path.length ? path[t] : path[path.length - 1];

export function runPrioritizedAStar(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "prioritized_astar", params});

    const W = map.width, H = map.height;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    // 고정 이웃 순서: up/down/left/right (통과 가능만), 마지막에 wait self-loop.
    const neighbors = (c: Cell): Cell[] => {
        const out: Cell[] = [];
        for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
            const n: Cell = [c[0] + dr, c[1] + dc];
            if (free(n)) out.push(n);
        }
        out.push([c[0], c[1]]);
        return out;
    };

    // 정적 flood fill — 예약과 무관한 도달 가능 셀 집합. set.size가 유한 horizon의
    // |reachable|이고, 순서는 결과 집합에 영향이 없다 (FIFO로 고정).
    const reachable = (start: Cell): Set<string> => {
        const seen = new Set<string>([`${start[0]},${start[1]}`]);
        const queue: Cell[] = [start];
        while (queue.length > 0) {
            const cell = queue.shift()!;
            for (const succ of neighbors(cell)) {
                const key = `${succ[0]},${succ[1]}`;
                if (!seen.has(key)) {
                    seen.add(key);
                    queue.push(succ);
                }
            }
        }
        return seen;
    };

    // (cell, t) 상태 하나의 space-time A*. 반환: 성공 시 space-time 경로, 실패 null.
    const planOne = (start: Cell, goal: Cell, planned: Cell[][], agent: number):
        { path: Cell[] | null; expanded: number } => {
        // 앞선 경로가 t=0에 이 agent의 시작 셀을 점유하면 되돌릴 수 없는 joint conflict.
        for (const p of planned) {
            if (cellEq(occupiedAt(p, 0), start)) return {path: null, expanded: 0};
        }
        const reach = reachable(start);
        if (!reach.has(`${goal[0]},${goal[1]}`)) return {path: null, expanded: 0};

        let frozen = 0;
        for (const p of planned) frozen = Math.max(frozen, pathLen(p) - 1);
        const horizon = frozen + reach.size;

        const frontier = new Heap();
        let counter = 0;
        frontier.push({f: manhattan(start, goal), seq: counter++, cell: start, t: 0});
        const seen = new Set<string>([`${start[0]},${start[1]}|0`]);
        const parent = new Map<string, Cell>();
        let expanded = 0;

        while (frontier.size > 0) {
            const cur = frontier.pop();
            // 상태는 정확히 한 번 push되므로 pop이 곧 확장이다 (lazy deletion 불필요).
            expanded += 1;
            emit({event: "node_expanded", state: [cur.cell[0], cur.cell[1]], cost: cur.t, agent, t: cur.t});

            // goal guard: stay-at-goal 반향으로 앞선 경로가 시각 >= t에 goal을
            // 계속 점유하면 pop을 받아들인다.
            let guard = true;
            for (const p of planned) {
                for (let tt = cur.t; tt < pathLen(p); tt++) {
                    if (cellEq(occupiedAt(p, tt), goal)) {
                        guard = false;
                        break;
                    }
                }
                if (!guard) break;
            }
            if (cellEq(cur.cell, goal) && guard) {
                const path: Cell[] = [[cur.cell[0], cur.cell[1]]];
                let curCell: Cell = [cur.cell[0], cur.cell[1]];
                let curT = cur.t;
                while (curT > 0) {
                    const prev = parent.get(`${curCell[0]},${curCell[1]}|${curT}`)!;
                    path.push([prev[0], prev[1]]);
                    curCell = [prev[0], prev[1]];
                    curT -= 1;
                }
                path.reverse();
                return {path, expanded};
            }
            if (cur.t === horizon) continue;

            const t2 = cur.t + 1;
            for (const succ of neighbors(cur.cell)) {
                // step t+1에 succ로의 이동이 합법이려면: 어떤 앞선 경로도 succ를 t+1에
                // 점유하면 안 되고(vertex conflict) 그 cell과 자리를 맞바꾸지도 못한다(edge).
                let blocked = false;
                for (const p of planned) {
                    if (cellEq(occupiedAt(p, t2), succ)
                        || (cellEq(occupiedAt(p, cur.t), succ) && cellEq(occupiedAt(p, t2), cur.cell))) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;
                const key = `${succ[0]},${succ[1]}|${t2}`;
                if (seen.has(key)) continue;
                seen.add(key);
                parent.set(key, [cur.cell[0], cur.cell[1]]);
                frontier.push({f: t2 + manhattan(succ, goal), seq: counter++, cell: succ, t: t2});
            }
        }
        return {path: null, expanded};
    };

    const paths: Cell[][] = [];
    let expanded = 0;
    for (let agent = 0; agent < tasks.length; agent++) {
        const r = planOne(tasks[agent][0], tasks[agent][1], paths, agent);
        expanded += r.expanded;
        if (r.path === null) {
            // 뒤 agent가 막히면 계획 전체가 실패 — 지표는 정직하게 0.
            emit({
                event: "planning_finished", success: false,
                metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
            });
            return events;
        }
        emit({event: "path_found", path: r.path, agent});
        paths.push(r.path);
    }
    let cost = 0;
    let makespan = 0;
    for (const p of paths) {
        cost += pathLen(p);
        makespan = Math.max(makespan, pathLen(p));
    }
    emit({
        event: "planning_finished", success: true,
        metrics: {expanded_nodes: expanded, makespan, sum_of_costs: cost},
    });
    return events;
}

// 한 agent의 비용 = 경로 스텝 수 (len(path) - 1).
const pathLen = (path: Cell[]): number => path.length - 1;

const manhattan = (a: Cell, b: Cell): number => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);
