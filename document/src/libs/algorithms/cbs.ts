import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 CBS. python/mrmp/search/cbs.py의 정확한 미러 — 고정 이웃
// 순서(up/down/left/right/wait), (f, seq) lexicographic tie-break(low level)와
// (cost, seq)(constraint tree queue), earliest-conflict tie-break(cell row/col
// 그다음 pair i<j), canonical edge constraint까지 trace가 필드 단위로 일치한다.
// 시각화용이 아니라 parity 검증의 한 축이다 (scripts/check-engine-parity.mjs).

type ConflictKind = "vertex" | "edge";

// 하나의 CT 제약. vertex: 시각 t에 cell 점유 금지. edge: 시각 t 스텝에서 cell과
// to 사이를 (어느 방향으로든) 지나는 것 금지 — 쌍은 정준화되어 방향이 없다.
export interface Constraint {
    kind: ConflictKind;
    cell: Cell;
    t: number;
    to?: Cell;
}

// CT 노드: agent별 제약 목록 + 각각의 개별 최적 경로.
interface CtNode {
    constraints: Constraint[][];
    paths: Cell[][];
}

interface SpaceTimeNode {
    f: number;
    seq: number;
    cell: Cell;
    t: number;
}

// (f, seq) lexicographic min-heap — low level space-time A*용.
class SpaceTimeHeap {
    private a: SpaceTimeNode[] = [];

    get size(): number { return this.a.length }

    push(n: SpaceTimeNode): void {
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

    pop(): SpaceTimeNode {
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

    private less(x: SpaceTimeNode, y: SpaceTimeNode): boolean {
        return x.f !== y.f ? x.f < y.f : x.seq < y.seq;
    }
}

interface CtEntry {
    cost: number;
    seq: number;
    node: CtNode;
}

// (cost, seq) lexicographic min-heap — high level constraint tree queue.
class CtHeap {
    private a: CtEntry[] = [];

    get size(): number { return this.a.length }

    push(n: CtEntry): void {
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

    pop(): CtEntry {
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

    private less(x: CtEntry, y: CtEntry): boolean {
        return x.cost !== y.cost ? x.cost < y.cost : x.seq < y.seq;
    }
}

const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
// Cell의 정렬 순서 — conflict 선택의 tie-break (row, 그다음 col).
const cellLt = (a: Cell, b: Cell): boolean => a[0] !== b[0] ? a[0] < b[0] : a[1] < b[1];

// 도착 후에도 그 셀을 계속 점유하는 Python과 동일한 stay-at-goal 점유 판정.
const occupiedAt = (path: Cell[], t: number): Cell =>
    t < path.length ? path[t] : path[path.length - 1];

export function runCbs(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    const maxCt = params.max_ct_expansions as number;
    emit({event: "planning_started", algorithm: "cbs", params});

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

    // 정적 flood fill — 제약과 무관한 도달 가능 셀 집합. size가 유한 horizon의
    // |reachable|이고, 순서는 결과 집합에 영향이 없다 (FIFO로 고정).
    const reachable = (start: Cell): Set<string> => {
        const seen = new Set<string>([`${start[0]},${start[1]}`]);
        const queue: Cell[] = [[start[0], start[1]]];
        while (queue.length > 0) {
            const cell = queue.shift()!;
            for (const succ of neighbors(cell)) {
                const key = `${succ[0]},${succ[1]}`;
                if (!seen.has(key)) {
                    seen.add(key);
                    queue.push([succ[0], succ[1]]);
                }
            }
        }
        return seen;
    };

    // edge 제약은 양방향 금지: 정준화된 쌍 {cell, to}를 어느 방향으로든 넘는 것을 막는다.
    const matchesEdge = (c: Constraint, fromCell: Cell, toCell: Cell): boolean =>
        (cellEq(c.cell, fromCell) && c.to !== undefined && cellEq(c.to, toCell))
        || (cellEq(c.cell, toCell) && c.to !== undefined && cellEq(c.to, fromCell));

    // agent 한 명의 space-time A* — wave 1의 서브 탐색이지만 예약 대신 제약을 쓴다.
    const planOne = (start: Cell, goal: Cell, constraints: Constraint[], agent: number):
        { path: Cell[] | null; expanded: number } => {
        // t=0에서 start 셀에 vertex 제약이 있으면 불가피 — 태어난 셀이 금지 셀이라는
        // 사실은 어떤 이동으로도 되돌릴 수 없다.
        for (const c of constraints) {
            if (c.kind === "vertex" && cellEq(c.cell, start) && c.t === 0) return {path: null, expanded: 0};
        }
        const reach = reachable(start);
        if (!reach.has(`${goal[0]},${goal[1]}`)) return {path: null, expanded: 0};
        let constrainedUntil = 0;
        for (const c of constraints) constrainedUntil = Math.max(constrainedUntil, c.t);
        const horizon = constrainedUntil + reach.size;

        const frontier = new SpaceTimeHeap();
        let counter = 0;
        frontier.push({f: manhattan(start, goal), seq: counter++, cell: [start[0], start[1]], t: 0});
        const seen = new Set<string>([`${start[0]},${start[1]}|0`]);
        const parent = new Map<string, Cell>();
        let expanded = 0;

        while (frontier.size > 0) {
            const cur = frontier.pop();
            // 상태는 정확히 한 번 push되므로 pop이 곧 확장이다 (lazy deletion 불필요).
            expanded += 1;
            emit({event: "node_expanded", state: [cur.cell[0], cur.cell[1]], cost: cur.t, agent, t: cur.t});

            // goal guard: 도착 후에도 goal을 계속 점유하므로, goal cell에 대한 vertex
            // 제약이 시각 >= t 어디에서도 유효하지 않을 때에만 이 pop을 받아들인다.
            let guard = true;
            for (const c of constraints) {
                if (c.kind === "vertex" && cellEq(c.cell, goal) && c.t >= cur.t) {
                    guard = false;
                    break;
                }
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
                // step t+1에 succ로의 이동이 합법이려면: (succ, t+1) vertex 제약이 없고
                // {cell, succ} 쌍을 t+1 스텝에서 금지하는 edge 제약도 없어야 한다.
                let blocked = false;
                for (const c of constraints) {
                    if ((c.kind === "vertex" && cellEq(c.cell, succ) && c.t === t2)
                        || (c.kind === "edge" && c.t === t2 && matchesEdge(c, cur.cell, succ))) {
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

    // 모든 agent 쌍에 대한 가장 이른 conflict; 동률은 (cell row, col) 그다음 pair i<j.
    const firstConflict = (paths: Cell[][]):
        { kind: ConflictKind; cell: Cell; t: number; agents: [number, number]; to?: Cell } | null => {
        let horizon = 0;
        for (const p of paths) horizon = Math.max(horizon, p.length - 1);
        for (let t = 0; t <= horizon; t++) {
            const kinds = new Map<string, {kind: ConflictKind; to?: Cell}>();
            let best: { cell: Cell; i: number; j: number } | null = null;
            for (let i = 0; i < paths.length; i++) {
                for (let j = i + 1; j < paths.length; j++) {
                    const nowI = occupiedAt(paths[i], t), nowJ = occupiedAt(paths[j], t);
                    if (cellEq(nowI, nowJ)) {
                        const key = `${nowI[0]},${nowI[1]}|${i}|${j}`;
                        kinds.set(key, {kind: "vertex"});
                        if (best === null || cellLt(nowI, best.cell)
                            || (cellEq(nowI, best.cell) && (i < best.i || (i === best.i && j < best.j)))) {
                            best = {cell: [nowI[0], nowI[1]], i, j};
                        }
                        continue;
                    }
                    if (t >= 1) {
                        const prevI = occupiedAt(paths[i], t - 1), prevJ = occupiedAt(paths[j], t - 1);
                        // 스텝 (t-1 -> t)를 가로질러 쌍이 셀을 맞바꿈: edge conflict.
                        if (cellEq(nowI, prevJ) && cellEq(nowJ, prevI)) {
                            const lo: Cell = cellLt(nowI, nowJ) ? [nowI[0], nowI[1]] : [nowJ[0], nowJ[1]];
                            const hi: Cell = cellLt(nowI, nowJ) ? [nowJ[0], nowJ[1]] : [nowI[0], nowI[1]];
                            const key = `${lo[0]},${lo[1]}|${i}|${j}`;
                            kinds.set(key, {kind: "edge", to: hi});
                            if (best === null || cellLt(lo, best.cell)
                                || (cellEq(lo, best.cell) && (i < best.i || (i === best.i && j < best.j)))) {
                                best = {cell: lo, i, j};
                            }
                        }
                    }
                }
            }
            if (best !== null) {
                const k = kinds.get(`${best.cell[0]},${best.cell[1]}|${best.i}|${best.j}`)!;
                return {kind: k.kind, cell: best.cell, t, agents: [best.i, best.j], to: k.to};
            }
        }
        return null;
    };

    // Root 확장(예산의 첫 번째): 모든 agent를 제약 없이 index 순서로 계획한다.
    let expanded = 0;
    const rootPaths: Cell[][] = [];
    for (let k = 0; k < tasks.length; k++) {
        const r = planOne(tasks[k][0], tasks[k][1], [], k);
        expanded += r.expanded;
        if (r.path === null) {
            // 제약 없는 서브 탐색이 실패하면 goal은 정적으로 도달 불가 — 어떤 제약
            // 집합도 도움이 될 수 없으므로 전체 instance가 여기서 실패한다.
            emit({
                event: "planning_finished", success: false,
                metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
            });
            return events;
        }
        rootPaths.push(r.path);
    }

    // Best-first CT queue: sum-of-costs 최소순, 동률은 생성 순서 FIFO tie-break.
    const queue = new CtHeap();
    let rootCost = 0;
    for (const p of rootPaths) rootCost += p.length - 1;
    let counter = 0;
    queue.push({cost: rootCost, seq: counter++, node: {constraints: tasks.map(() => []), paths: rootPaths}});
    let ctExpansions = 1;

    while (queue.size > 0 && ctExpansions < maxCt) {
        const top = queue.pop();
        ctExpansions += 1;
        const cost = top.cost;
        const node = top.node;

        const conflict = firstConflict(node.paths);
        if (conflict === null) {
            // best-first가 최적을 pop했다: 개별 최적 경로들이 충돌까지 없으면 jointly 최적.
            let makespan = 0;
            for (const p of node.paths) makespan = Math.max(makespan, p.length - 1);
            node.paths.forEach((p, k) => emit({event: "path_found", path: p, agent: k}));
            emit({
                event: "planning_finished", success: true,
                metrics: {expanded_nodes: expanded, makespan, sum_of_costs: cost},
            });
            return events;
        }

        emit({
            event: "conflict_found", kind: conflict.kind, cell: [conflict.cell[0], conflict.cell[1]],
            t: conflict.t, agents: conflict.agents, to: conflict.to,
        });

        // 분기: 충돌한 agent마다 자식 하나, pair 순서대로. 각 자식은 자기 agent만
        // 이번 충돌에서 금지하고 재계획하며, 서브 탐색이 실패한 자식은 죽고 push되지 않는다.
        for (const agent of [conflict.agents[0], conflict.agents[1]]) {
            const childConstraints = node.constraints.map((c) => c.slice());
            childConstraints[agent].push({
                kind: conflict.kind, cell: conflict.cell, t: conflict.t, to: conflict.to,
            });
            emit({
                event: "constraint_added", agent, kind: conflict.kind,
                cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t, to: conflict.to,
            });
            const r = planOne(tasks[agent][0], tasks[agent][1], childConstraints[agent], agent);
            expanded += r.expanded;
            if (r.path === null) continue;
            const childPaths = node.paths.slice();
            const oldCost = childPaths[agent].length - 1;
            childPaths[agent] = r.path;
            queue.push({
                cost: cost - oldCost + (r.path.length - 1), seq: counter++,
                node: {constraints: childConstraints, paths: childPaths},
            });
        }
    }

    // 예산 소진 전 큐가 비면: 모든 가지가 자기 제약으로 죽었다 — 진짜 unsolvability
    // 판정이다. (예산 소진은 "예산 내 해 없음"일 뿐 — 모듈 문서 참조.)
    emit({
        event: "planning_finished", success: false,
        metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
    });
    return events;
}

const manhattan = (a: Cell, b: Cell): number => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);
