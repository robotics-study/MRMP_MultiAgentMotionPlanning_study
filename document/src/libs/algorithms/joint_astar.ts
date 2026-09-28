import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 joint-space A*. python/mrmp/mapf/joint_astar.py의 정확한
// 미러 — lazy-deletion A*, (f, seq) tie-break(seq는 1부터), itertools.product와
// 동일한 successor 순서(마지막 agent가 가장 빨리 변함), 도착한 agent의 self-loop
// 고정까지 그대로라 trace가 필드 단위로 일치한다. 시각화용이 아니라 parity 검증의
// 한 축이다 (scripts/check-engine-parity.mjs).

interface HeapNode {
    f: number;
    // push 순서 카운터 — f 동률을 발생 순서로 깨는 Python heapq 튜플의 두 번째 항.
    // joint 미러는 count(1)이므로 첫 entry(seq=1)부터 시작한다.
    seq: number;
    state: Cell[];
}

// (f, seq) lexicographic min-heap. 이진 힙이고 비교가 전순위가 아니라도 된다 —
// pop 순서만 같으면 되므로. seq가 유일하므로 state는 비교된 적이 없다.
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

// joint state의 키 — 평면화된 [r,c|r,c|...] 문자열. Python의 tuple 동등성과 동일.
const keyOf = (state: Cell[]): string => state.map((c) => `${c[0]},${c[1]}`).join("|");

const manhattan = (a: Cell, b: Cell): number => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);

export function runJointAStar(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "joint_astar", params});

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

    const starts = tasks.map(([s]) => s);
    const goals = tasks.map(([, g]) => g);

    // t=0에 두 agent가 같은 셀을 공유하면 unsolvable한 instance — joint 상태 공간에
    // 그들을 떨어뜨릴 수 있는 conflict-free 상태는 존재하지 않는다. 탐색은 시작조차 안 한다.
    for (let i = 0; i < starts.length; i++) {
        for (let j = i + 1; j < starts.length; j++) {
            if (cellEq(starts[i], starts[j])) {
                emit({
                    event: "planning_finished", success: false,
                    metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0},
                });
                return events;
            }
        }
    }

    // agent별 Manhattan 거리의 합 — admissible하고 consistent.
    const h = (state: Cell[]): number => {
        let total = 0;
        for (let k = 0; k < state.length; k++) total += manhattan(state[k], goals[k]);
        return total;
    };

    const frontier = new Heap();
    let counter = 1; // Python의 itertools.count(1) — 시작 노드가 seq 1을 받는다.
    frontier.push({f: h(starts), seq: counter++, state: starts});
    const gScore = new Map<string, number>([[keyOf(starts), 0]]);
    const parent = new Map<string, Cell[]>();
    const closed = new Set<string>();
    let expanded = 0;

    while (frontier.size > 0) {
        const cur = frontier.pop();
        const key = keyOf(cur.state);
        // stale heap entry — 이미 pop된 상태는 절대 다시 확장되지 않는다.
        if (closed.has(key)) continue;
        closed.add(key);
        expanded += 1;
        const gNow = gScore.get(key)!;
        emit({event: "node_expanded", state: cur.state.flat(), cost: gNow});

        if (cur.state.every((c, k) => cellEq(c, goals[k]))) {
            // parent chain을 goal-state → start로 거슬러 올라가 역전시키면 chain[t]가
            // 스텝 t의 joint 위치. agent k의 경로는 FIRST 도착까지다 — 그 뒤로 고정됐으니.
            const chain: Cell[][] = [cur.state];
            while (!chain[chain.length - 1].every((c, k) => cellEq(c, starts[k]))) {
                chain.push(parent.get(keyOf(chain[chain.length - 1]))!);
            }
            chain.reverse();
            const paths: Cell[][] = [];
            for (let k = 0; k < tasks.length; k++) {
                let arrival = 0;
                while (!cellEq(chain[arrival][k], goals[k])) arrival++;
                paths.push(chain.slice(0, arrival + 1).map((s) => s[k]));
            }
            let cost = 0, makespan = 0;
            for (const p of paths) {
                cost += p.length - 1;
                makespan = Math.max(makespan, p.length - 1);
            }
            for (let k = 0; k < paths.length; k++) emit({event: "path_found", path: paths[k], agent: k});
            emit({
                event: "planning_finished", success: true,
                metrics: {expanded_nodes: expanded, makespan, sum_of_costs: cost},
            });
            return events;
        }

        // 한 joint 스텝: 도착하지 않은 agent마다 정확히 행동 하나(이동 또는 wait
        // self-loop)씩 취하고 각각 스텝 하나를 지불한다. 도착한 agent는 그 자리에
        // 고정되고 아무것도 지불하지 않는다. step cost = 아직 도착하지 않은 agent 수.
        let stepCost = 0;
        const actions: Cell[][] = [];
        for (let k = 0; k < cur.state.length; k++) {
            if (cellEq(cur.state[k], goals[k])) {
                actions.push([cur.state[k]]); // 도착: self-loop만, 지불 없음.
            } else {
                stepCost += 1;
                actions.push(neighbors(cur.state[k]));
            }
        }

        // cartesian product를 agent-index 순서로, 마지막 agent가 가장 빨리 변하도록
        // 열거한다 — itertools.product의 출력 순서와 정확히 동일. odometer idx가
        // agent마다 digit을 가진 mixed-radix 세기다 (agent 0명 → 빈 combo 하나).
        const idx = new Array<number>(actions.length).fill(0);
        for (;;) {
            const succ: Cell[] = actions.map((acts, k) => acts[idx[k]]);

            // vertex conflict: 새 스텝에서 두 agent가 한 셀 위. edge conflict: 짝이
            // 스텝을 사이에 두고 자리를 맞바꿈. 둘 중 하나라도면 combo 폐기.
            let vertex = false, edge = false;
            for (let i = 0; i < succ.length && !vertex; i++) {
                for (let j = i + 1; j < succ.length; ++j) {
                    if (cellEq(succ[i], succ[j])) { vertex = true; break; }
                }
            }
            for (let i = 0; i < cur.state.length && !edge; i++) {
                for (let j = i + 1; j < cur.state.length; ++j) {
                    if (cellEq(succ[i], cur.state[j]) && cellEq(cur.state[i], succ[j])) { edge = true; break; }
                }
            }

            if (!vertex && !edge) {
                const g2 = gNow + stepCost;
                const sk = keyOf(succ);
                const prev = gScore.get(sk);
                // 개선 없음 — heuristic이 consistent하므로 그 상태는 더 나은 g로
                // (이미 또는 나중에) settle된다. push를 건너뛴다.
                if (prev === undefined || g2 < prev) {
                    parent.set(sk, cur.state);
                    gScore.set(sk, g2);
                    frontier.push({f: g2 + h(succ), seq: counter++, state: succ});
                }
            }

            // odometer tick: 마지막 agent의 digit이 가장 빨리 증가하고 왼쪽으로 올림.
            let advanced = false;
            for (let k = idx.length; k > 0;) {
                --k;
                if (idx[k] + 1 < actions[k].length) { idx[k] += 1; advanced = true; break; }
                idx[k] = 0;
            }
            if (!advanced) break;
        }
    }

    // frontier 고갈: joint 계획이 존재하지 않는다 (complete 탐색 — 이건 instance에
    // 대한 판정이지 어떤 우선순위 순서에 대한 게 아니다). 정직하게 보고한다.
    emit({
        event: "planning_finished", success: false,
        metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
    });
    return events;
}
