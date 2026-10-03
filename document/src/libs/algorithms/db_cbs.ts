import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 db-CBS. python/mrmp/kinodynamic/db_cbs.py의 정확한 미러 —
// 상태가 (cell, velocity)인 저수준 A*(정수 속도 격자, 축별 가속 1)와 CBS high level을
// 같은 정준 순서로 돈다: successor는 (row, col, v_row, v_col) 사전식으로 정렬된 집합,
// 힙은 (f, seq)/(cost, seq) lexicographic tie-break, conflict 선택은 가장 이른 스텝의
// 공재(cell row → col, 그다음 pair i<j — 이 갈래에 edge conflict는 없다), 제약은 부피
// (cell,t)에서 Manhattan floor(δ) 이내 금지. 라더 [delta_start, delta_end]는 독립적인
// 새 트리이고 마지막 성공 칸의 해가 답이며 expanded_nodes는 전 rung에 누적된다.
// trace가 필드 단위로 일치하는 게 parity 검증의 근거다 (scripts/check-engine-parity.mjs).

interface DbConstraint {
    cell: Cell;
    t: number;
}

interface CtNode {
    constraints: DbConstraint[][];
    paths: Cell[][];
}

// 저수준 힙 노드: g == t라 f = t + h, push 순서는 정수 seq — Python heapq의 (f, seq)와
// 동일하게 float 우선, 그다음 생성 순서(유일하므로 페이는 비교된 적 없다).
interface StateNode {
    f: number;
    seq: number;
    cell: Cell;
    v: Cell;
    t: number;
}

class StateHeap {
    private a: StateNode[] = [];

    get size(): number { return this.a.length }

    push(n: StateNode): void {
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

    pop(): StateNode {
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

    private less(x: StateNode, y: StateNode): boolean {
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
const manhattan = (a: Cell, b: Cell): number => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);

// 도착 후에도 그 셀을 계속 점유하는 Python과 동일한 stay-at-goal 점유 판정.
const occupiedAt = (path: Cell[], t: number): Cell =>
    t < path.length ? path[t] : path[path.length - 1];

export function runDbCbs(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    vmax: number[],
    params: Record<string, unknown>,
): TraceEvent[] {
    const deltaStart = params.delta_start as number;
    const deltaEnd = params.delta_end as number;
    const maxCt = params.max_ct_expansions as number;

    // Python이 plan()에서 던지는 순서 그대로 — 파라미터 쌍이 먼저, 그다음 vmax 루프.
    // 라이브 sandbox의 vmax 칩은 0.25/0.5로 순환할 수 있고 그때 엔진은 Python처럼 던진다
    // (속도 격자는 정수만 양자화한다). 의도된 차이 하나: Python 데모는 plan() 전에
    // planning_started를 싣므로 거부된 실행의 trace 파일에는 그 이벤트 하나만 남고, 여기서는
    // 이벤트를 아예 싣지 않는다 — 거부된 실행은 어떤 트레이스도 수출되지 않고 라이브는 error
    // 카드를 보이니 관찰 가능한 차이는 없다.
    if (deltaEnd > deltaStart) {
        throw new Error(`param error: db_cbs: delta_end (${deltaEnd}) must not exceed `
            + `delta_start (${deltaStart}) — the ladder tightens, never loosens`);
    }
    for (const v of vmax) {
        if (!Number.isInteger(v) || v < 1) {
            throw new Error(`param error: db_cbs: vmax must be a positive integer (got ${v}) `
                + "— the velocity lattice quantizes integers only");
        }
    }

    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    // Python의 demo가 plan() 전에 싣는 planning_started — vmax의 존재가 timed 선언이다.
    emit({event: "planning_started", algorithm: "db_cbs", params, vmax});

    const W = map.width, H = map.height;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    // 상태 키 — (cell, velocity, step). seen/parent는 이 문자열로 키를 잡는다.
    const keyOf = (cell: Cell, v: Cell, t: number): string =>
        `${cell[0]},${cell[1]}|${v[0]},${v[1]}|${t}`;

    // goal까지 스텝은 Chebyshev / (vmax + floor(δ))보다 작을 수 없다 — 어떤 합법 이동도
    // 어느 축을 그 이상으로 옮기지 못한다. admissible하고 consistent하다.
    const heuristic = (cell: Cell, goal: Cell, vmaxK: number, f: number): number =>
        Math.ceil(Math.max(Math.abs(cell[0] - goal[0]), Math.abs(cell[1] - goal[1])) / (vmaxK + f));

    // 한 합법 전이로 도달 가능한 모든 상태를 정준 사전식 순서(row, col, v_row, v_col)로
    // 만든다 — Python의 sorted(set(...))와 동일. 전이 (cell,v) → (x', v')는 어떤 동작 u가
    // |u−v| ≤ 1(축별), |u| ≤ vmax(축별)를 만족하고 착지가 x'에서 Manhattan f 이내,
    // 속도가 v'에서 f 이내일 때만 합법. f = 0이면 정확한 이중 적분자로 줄어든다.
    const successors = (cell: Cell, v: Cell, vmaxK: number, f: number): Array<{ cell: Cell; v: Cell }> => {
        const out: Array<{ cell: Cell; v: Cell }> = [];
        for (let ur = Math.max(-vmaxK, v[0] - 1); ur <= Math.min(vmaxK, v[0] + 1); ur++) {
            for (let uc = Math.max(-vmaxK, v[1] - 1); uc <= Math.min(vmaxK, v[1] + 1); uc++) {
                // 이상적 착지 x+u에서 Manhattan f 이내의 착지 셀(중복 허용 — 아래서 dedup).
                for (let dr = -f; dr <= f; dr++) {
                    for (let dc = -(f - Math.abs(dr)); dc <= (f - Math.abs(dr)); dc++) {
                        const land: Cell = [cell[0] + ur + dr, cell[1] + uc + dc];
                        if (!free(land)) continue;
                        // 착지 상태는 동작에서 f 이내의 모든 속도를 싣는다.
                        for (let vr = Math.max(-vmaxK, ur - f); vr <= Math.min(vmaxK, ur + f); vr++) {
                            for (let vc = Math.max(-vmaxK, uc - f); vc <= Math.min(vmaxK, uc + f); vc++) {
                                out.push({cell: land, v: [vr, vc]});
                            }
                        }
                    }
                }
            }
        }
        // sorted(set(...))의 사전식 순서 — 정렬 후 인접 중복을 지운다.
        out.sort((a, b) => a.cell[0] - b.cell[0] || a.cell[1] - b.cell[1] || a.v[0] - b.v[0] || a.v[1] - b.v[1]);
        const uniq: Array<{ cell: Cell; v: Cell }> = [];
        for (const s of out) {
            const lastU = uniq[uniq.length - 1];
            if (lastU && cellEq(lastU.cell, s.cell) && cellEq(lastU.v, s.v)) continue;
            uniq.push(s);
        }
        return uniq;
    };

    // 이동 관계 자체에 대한 flood fill — 도달한 셀에서 축별 거리 reach_r 이내의 모든 통과
    // 가능 셀이 전이 하나의 착지다. 결과는 집합이라 fill 순서는 무관하지만 FIFO로 고정.
    const reachable = (start: Cell, reachR: number): Set<string> => {
        const seen = new Set<string>([`${start[0]},${start[1]}`]);
        const queue: Cell[] = [[start[0], start[1]]];
        while (queue.length > 0) {
            const cell = queue.shift()!;
            for (let dr = -reachR; dr <= reachR; dr++) {
                for (let dc = -reachR; dc <= reachR; dc++) {
                    const succ: Cell = [cell[0] + dr, cell[1] + dc];
                    const key = `${succ[0]},${succ[1]}`;
                    if (!seen.has(key) && free(succ)) {
                        seen.add(key);
                        queue.push(succ);
                    }
                }
            }
        }
        return seen;
    };

    // agent 한 명의 db-A*: (cell, velocity) 상태 위 A*를 절대 시간에 한다. g == t는
    // 결코 개선되지 않으므로 pop이 곧 확장이고 모든 상태는 정확히 한 번 push된다.
    const planOne = (start: Cell, goal: Cell, vmaxK: number, constraints: DbConstraint[],
                     f: number, agent: number): { path: Cell[] | null; expanded: number } => {
        // 금지 부피 안에서 태어나는 것은 되돌릴 수 없다 — 스텝 0을 지우는 이동은 없다.
        for (const c of constraints) {
            if (c.t === 0 && manhattan(c.cell, start) <= f) return {path: null, expanded: 0};
        }
        const reach = reachable(start, vmaxK + f);
        if (!reach.has(`${goal[0]},${goal[1]}`)) return {path: null, expanded: 0}; // 정적 도달 불가
        let constrainedUntil = 0;
        for (const c of constraints) constrainedUntil = Math.max(constrainedUntil, c.t);
        // 유한성: 상태 공간은 정확히 |reach| × (2vmax+1)²이고 마지막 제약 스텝 너머엔
        // 아무것도 조이지 않으므로 어떤 계획도 그보다 한 스텝 더를 필요로 하지 않는다.
        const horizon = constrainedUntil + reach.size * (2 * vmaxK + 1) ** 2;

        const frontier = new StateHeap();
        let counter = 0;
        frontier.push({f: heuristic(start, goal, vmaxK, f), seq: counter++, cell: [start[0], start[1]], v: [0, 0], t: 0});
        const seen = new Set<string>([keyOf([start[0], start[1]], [0, 0], 0)]);
        const parent = new Map<string, { cell: Cell; v: Cell }>();
        let expanded = 0;

        while (frontier.size > 0) {
            const cur = frontier.pop();
            expanded += 1;
            emit({event: "node_expanded", state: [cur.cell[0], cur.cell[1]], cost: cur.t, agent, t: cur.t});
            // goal guard: 도착 후에도 goal을 계속 점유하므로 goal 셀의 제약이 시각 >= t
            // 어디에서도 유효하지 않을 때에만 이 pop을 받아들인다(속도는 관찰되지 않는다).
            let guard = true;
            for (const c of constraints) {
                if (c.t >= cur.t && manhattan(c.cell, goal) <= f) {
                    guard = false;
                    break;
                }
            }
            if (cellEq(cur.cell, goal) && guard) {
                const path: Cell[] = [];
                let cell: Cell = [cur.cell[0], cur.cell[1]];
                let v: Cell = [cur.v[0], cur.v[1]];
                let tt = cur.t;
                for (;;) {
                    path.push(cell);
                    // parent는 predecessor 상태를 저장한다 — 스텝은 정확히 하나 아래(전이는 항상 1).
                    const prev = parent.get(keyOf(cell, v, tt));
                    if (prev === undefined) break;
                    cell = [prev.cell[0], prev.cell[1]];
                    v = [prev.v[0], prev.v[1]];
                    tt -= 1;
                }
                path.reverse(); // index 0 == step 0
                return {path, expanded};
            }
            if (cur.t === horizon) continue;

            const t2 = cur.t + 1;
            for (const s of successors(cur.cell, cur.v, vmaxK, f)) {
                let blocked = false;
                for (const c of constraints) {
                    if (c.t === t2 && manhattan(c.cell, s.cell) <= f) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;
                const key = keyOf(s.cell, s.v, t2);
                if (seen.has(key)) continue;
                seen.add(key);
                parent.set(key, {cell: [cur.cell[0], cur.cell[1]], v: [cur.v[0], cur.v[1]]});
                frontier.push({f: t2 + heuristic(s.cell, goal, vmaxK, f), seq: counter++, cell: s.cell, v: s.v, t: t2});
            }
        }
        return {path: null, expanded};
    };

    // 모든 agent 쌍의 가장 이른 공재(같은 스텝 같은 셀); 동률은 (cell row, col) 그다음
    // pair i<j. 스텝을 가로지르는 swap은 conflict가 아니다 — 정수 스텝에서 표본되는 점
    // 로봇들은 서로를 통과한다.
    const firstConflict = (paths: Cell[][]): { cell: Cell; t: number; agents: [number, number] } | null => {
        let horizon = 0;
        for (const p of paths) horizon = Math.max(horizon, p.length - 1);
        for (let t = 0; t <= horizon; t++) {
            let best: { cell: Cell; i: number; j: number } | null = null;
            for (let i = 0; i < paths.length; i++) {
                for (let j = i + 1; j < paths.length; j++) {
                    const nowI = occupiedAt(paths[i], t), nowJ = occupiedAt(paths[j], t);
                    if (!cellEq(nowI, nowJ)) continue;
                    // candidates의 min과 동일: (row, col)이 먼저, 그다음 pair 사전식.
                    if (best === null || cellLt(nowI, best.cell)
                        || (cellEq(nowI, best.cell) && (i < best.i || (i === best.i && j < best.j)))) {
                        best = {cell: [nowI[0], nowI[1]], i, j};
                    }
                }
            }
            if (best !== null) return {cell: best.cell, t, agents: [best.i, best.j]};
        }
        return null;
    };

    // 한 칸 = delta 하나의 새 constraint tree 전체. root 확장은 모든 agent를 제약 없이
    // index 순서로 — 서브 탐색이 실패하면 이 rung은 죽는다(확장 수는 누적).
    const rung = (delta: number): { paths: Cell[][] | null; expanded: number } => {
        const f = Math.floor(delta); // 격자 여유: <1이면 정확, >=1이면 그만큼 칸
        let expanded = 0;
        const rootPaths: Cell[][] = [];
        for (let k = 0; k < tasks.length; k++) {
            const r = planOne(tasks[k][0], tasks[k][1], vmax[k], [], f, k);
            expanded += r.expanded;
            if (r.path === null) return {paths: null, expanded};
            rootPaths.push(r.path);
        }

        // best-first CT queue: sum-of-arrival-steps 최소순, 동률은 생성 순서.
        const queue = new CtHeap();
        let rootCost = 0;
        for (const p of rootPaths) rootCost += p.length - 1;
        let counter = 0;
        queue.push({cost: rootCost, seq: counter++, node: {constraints: tasks.map(() => [] as DbConstraint[]), paths: rootPaths}});
        let ctExpansions = 1;

        while (queue.size > 0 && ctExpansions < maxCt) {
            const top = queue.pop();
            ctExpansions += 1;
            const cost = top.cost;
            const node = top.node;

            const conflict = firstConflict(node.paths);
            if (conflict === null) {
                // best-first가 개별 최적 경로들의 충돌 없는 조합을 pop했다 — 이 rung의 δ에서
                // jointly 최적이다. 끝.
                return {paths: node.paths, expanded};
            }

            emit({
                event: "conflict_found", kind: "vertex",
                cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t, agents: conflict.agents,
            });

            // 분기: 충돌한 agent마다 자식 하나, pair 순서대로. 각 자식은 자기 agent만 이번
            // 충돌 부피에서 금지하고 재계획하며, 저수준이 실패한 자식은 죽고 push되지 않는다.
            for (const agent of [conflict.agents[0], conflict.agents[1]]) {
                const childConstraints = node.constraints.map((c) => c.slice());
                childConstraints[agent].push({cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t});
                emit({
                    event: "constraint_added", agent, kind: "vertex",
                    cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t,
                });
                const r = planOne(tasks[agent][0], tasks[agent][1], vmax[agent], childConstraints[agent], f, agent);
                expanded += r.expanded;
                if (r.path === null) continue;
                const childPaths = node.paths.slice();
                const oldCost = childPaths[agent].length - 1;
                childPaths[agent] = r.path;
                queue.push({cost: cost - oldCost + (r.path.length - 1), seq: counter++,
                    node: {constraints: childConstraints, paths: childPaths}});
            }
        }

        return {paths: null, expanded};
    };

    // 라더: 파라미터 쌍이 만드는 사다리 — 좁아지기만 한다. 각 칸은 독립적인 새 트리다
    // (느슨한 rung은 제약 부피도 살찌우므로 단조 개선이 아니다)와 마지막 성공 칸의 해가
    // 답이고 expanded_nodes는 전 rung에 누적된다. 생존 없이는 정직한 실패 — 그때 delta는
    // 시도된 가장 좁은 칸(모든 가지가 죽은 모델).
    const ladder = deltaStart === deltaEnd ? [deltaStart] : [deltaStart, deltaEnd];
    let expanded = 0;
    let solved: Cell[][] | null = null;
    let answeredDelta = 0.0;
    for (const delta of ladder) {
        const r = rung(delta);
        expanded += r.expanded;
        if (r.paths === null) continue;
        solved = r.paths;
        answeredDelta = delta;
    }

    if (solved === null) {
        emit({
            event: "planning_finished", success: false,
            metrics: {delta: ladder[ladder.length - 1], expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
        });
        return events;
    }

    let cost = 0;
    let makespan = 0;
    for (const p of solved) {
        cost += p.length - 1;
        makespan = Math.max(makespan, p.length - 1);
    }
    solved.forEach((p, k) => emit({event: "path_found", path: p.map((c) => [c[0], c[1]] as Cell), agent: k}));
    emit({
        event: "planning_finished", success: true,
        metrics: {delta: answeredDelta, expanded_nodes: expanded, makespan, sum_of_costs: cost},
    });
    return events;
}
