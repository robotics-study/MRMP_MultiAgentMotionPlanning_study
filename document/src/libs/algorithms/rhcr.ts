import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 RHCR. python/mrmp/search/rhcr.py의 정확한 미러 — 창 달린 CBS를
// 매 갱신 주기 h마다 실제 위치에서 다시 돌린다(창 w 안의 도착 스텝만 충돌로 본다).
// 고정 이웃 순서(up/down/left/right/wait), 시공간 A*의 (f, seq) tie-break, CT 큐의
// (cost, seq) 사전순 순서, earliest-conflict 선택(cell row/col 그다음 pair i<j),
// canonical edge constraint까지 trace가 필드 단위로 일치한다(parity: check-engine-parity).
// 창이 호라이즌을 덮으면 일반 CBS와 필드 단위로 완전히 같아진다 — cbs.ts가 바로 그
// 극한이고, 여기서는 그 위를 시간축이 굴린다.

type ConflictKind = "vertex" | "edge";

// 하나의 CT 제약. vertex: 시각 t에 cell 점유 금지. edge: 시각 t 스텝에서 cell과
// to 사이를 (어느 방향으로든) 지나는 것 금지 — 쌍은 정준화되어 방향이 없다.
interface Constraint {
    kind: ConflictKind;
    cell: Cell;
    t: number;
    to?: Cell;
}

// 에피소드 하나의 CT 노드: agent별 제약 목록 + 각각의 개별 최적 절대-인덱스 경로.
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

// (cost, seq) lexicographic min-heap — 에피소드 안의 constraint tree 큐. 에피소드가
// 바뀌면 counter도 새로 돈다(Python의 itertools.count가 에피소드마다 생성된다).
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

const ck = (c: Cell): string => `${c[0]},${c[1]}`;
const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
// Cell의 정렬 순서 — conflict 선택의 tie-break (row, 그다음 col).
const cellLt = (a: Cell, b: Cell): boolean => a[0] !== b[0] ? a[0] < b[0] : a[1] < b[1];
const manhattan = (a: Cell, b: Cell): number => Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);

// 도착 후에도 그 셀을 계속 점유하는 Python과 동일한 stay-at-goal 점유 판정.
const occupiedAt = (path: Cell[], t: number): Cell =>
    t < path.length ? path[t] : path[path.length - 1];

// edge 제약은 양방향 금지: 정준화된 쌍 {cell, to}를 어느 방향으로든 넘는 것을 막는다.
const matchesEdge = (c: Constraint, fromCell: Cell, toCell: Cell): boolean =>
    (cellEq(c.cell, fromCell) && c.to !== undefined && cellEq(c.to, toCell))
    || (cellEq(c.cell, toCell) && c.to !== undefined && cellEq(c.to, fromCell));

export function runRhcr(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "rhcr", params});

    const window = params.window as number;
    const period = params.replan_period as number;
    const maxSteps = params.max_steps as number;
    // 파라미터 스키마가 표현할 수 없는 교차 제약: h > w는 어떤 창도 결코 덮지 않는
    // 스텝을 실행한다는 뜻 — 더 어려운 문제가 아니라 안전하지 않은 입력이다.
    // Python은 여기서 ParamError를 던지고(아직 이벤트 없음), 라이브 데모의 칩은
    // period=1 고정이라 이 가드가 발동할 수 없다. 그래도 미러는 던진다.
    if (period > window) {
        throw new Error(`param error: replan_period ${period} exceeds window ${window} `
            + "(every executed step must fall inside some resolved window)");
    }

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
        const seen = new Set<string>([ck(start)]);
        const queue: Cell[] = [[start[0], start[1]]];
        let head = 0;
        while (head < queue.length) {
            const cell = queue[head++];
            for (const succ of neighbors(cell)) {
                if (!seen.has(ck(succ))) {
                    seen.add(ck(succ));
                    queue.push([succ[0], succ[1]]);
                }
            }
        }
        return seen;
    };

    const starts = tasks.map(([s]) => [s[0], s[1]] as Cell);
    const goals = tasks.map(([, g]) => [g[0], g[1]] as Cell);
    // Well-formed instance: distinct starts, distinct goals, passable cells. A
    // t=0 collision is history rather than a conflict the window could resolve —
    // an ill-formed input is not an instance of this problem at all.
    if (new Set(starts.map(ck)).size < starts.length
        || new Set(goals.map(ck)).size < goals.length
        || [...starts, ...goals].some((c) => !free(c))) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    // agent 한 명의 절대 시공간 A* — cbs.ts의 서브 탐색과 같고 시계만 에피소드
    // 시작(t_now)으로 옮겨 놓은 것. 상태는 정확히 한 번 push되므로 pop이 곧 확장이다.
    const planOne = (start: Cell, goal: Cell, constraints: Constraint[], tNow: number, agent: number):
        {path: Cell[] | null; expanded: number} => {
        // t_now에서 start 셀에 vertex 제약이 있으면 불가피 — 태어난 셀이 금지 셀이라는
        // 사실은 어떤 이동으로도 되돌릴 수 없다. (구성상 제약은 항상 스텝 > t_now를
        // 싣고 이 가드는 발동하지 않는다 — cbs.py와의 정확성을 위해 미러링.)
        for (const c of constraints) {
            if (c.kind === "vertex" && cellEq(c.cell, start) && c.t <= tNow) return {path: null, expanded: 0};
        }
        const reach = reachable(start);
        if (!reach.has(ck(goal))) {
            // 현재 위치에서 정적으로 도달 불가: 어떤 제약 집합도 도울 수 없으므로 이
            // 에피소드(그리고 전체 instance)가 여기서 정직하게 실패한다.
            return {path: null, expanded: 0};
        }
        let constrainedUntil = tNow;
        for (const c of constraints) constrainedUntil = Math.max(constrainedUntil, c.t);
        const horizon = constrainedUntil + reach.size;

        const frontier = new SpaceTimeHeap();
        let counter = 0;
        frontier.push({f: tNow + manhattan(start, goal), seq: counter++, cell: [start[0], start[1]], t: tNow});
        const seen = new Set<string>([`${ck(start)}|${tNow}`]);
        const parent = new Map<string, Cell>();
        let expanded = 0;

        while (frontier.size > 0) {
            const cur = frontier.pop();
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
                while (curT > tNow) {
                    const prev = parent.get(`${ck(curCell)}|${curT}`)!;
                    path.push([prev[0], prev[1]]);
                    curCell = [prev[0], prev[1]];
                    curT -= 1;
                }
                path.reverse(); // index 0 == 스텝 t_now; 호출자가 절대 인덱스로 되돌린다
                return {path, expanded};
            }
            if (cur.t === horizon) continue;

            const t2 = cur.t + 1;
            for (const succ of neighbors(cur.cell)) {
                let blocked = false;
                for (const c of constraints) {
                    if ((c.kind === "vertex" && cellEq(c.cell, succ) && c.t === t2)
                        || (c.kind === "edge" && c.t === t2 && matchesEdge(c, cur.cell, succ))) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;
                const key = `${ck(succ)}|${t2}`;
                if (seen.has(key)) continue;
                seen.add(key);
                parent.set(key, [cur.cell[0], cur.cell[1]]);
                frontier.push({f: t2 + manhattan(succ, goal), seq: counter++, cell: succ, t: t2});
            }
        }
        return {path: null, expanded};
    };

    // 창 안에 도착한 가장 이른 conflict; 동률은 (cell row, col) 그다음 pair i<j.
    // 창 너머에서는 아무것도 conflict가 아니다 — 창이 바로 그것이다.
    const firstConflict = (paths: Cell[][], tNow: number):
        {kind: ConflictKind; cell: Cell; t: number; agents: [number, number]; to?: Cell} | null => {
        let horizon = -1;
        for (const p of paths) horizon = Math.max(horizon, p.length - 1);
        horizon = Math.min(tNow + window, horizon);
        for (let t = tNow + 1; t <= horizon; t++) {
            const kinds = new Map<string, {kind: ConflictKind; to?: Cell}>();
            let best: { cell: Cell; i: number; j: number } | null = null;
            for (let i = 0; i < paths.length; i++) {
                for (let j = i + 1; j < paths.length; j++) {
                    const nowI = occupiedAt(paths[i], t), nowJ = occupiedAt(paths[j], t);
                    if (cellEq(nowI, nowJ)) {
                        kinds.set(`${ck(nowI)}|${i}|${j}`, {kind: "vertex"});
                        if (best === null || cellLt(nowI, best.cell)
                            || (cellEq(nowI, best.cell) && (i < best.i || (i === best.i && j < best.j)))) {
                            best = {cell: [nowI[0], nowI[1]], i, j};
                        }
                        continue;
                    }
                    const prevI = occupiedAt(paths[i], t - 1), prevJ = occupiedAt(paths[j], t - 1);
                    // 스텝 (t-1 -> t)를 가로질러 쌍이 셀을 맞바꿈: edge conflict.
                    if (cellEq(nowI, prevJ) && cellEq(nowJ, prevI)) {
                        const lo: Cell = cellLt(nowI, nowJ) ? [nowI[0], nowI[1]] : [nowJ[0], nowJ[1]];
                        const hi: Cell = cellLt(nowI, nowJ) ? [nowJ[0], nowJ[1]] : [nowI[0], nowI[1]];
                        kinds.set(`${ck(lo)}|${i}|${j}`, {kind: "edge", to: hi});
                        if (best === null || cellLt(lo, best.cell)
                            || (cellEq(lo, best.cell) && (i < best.i || (i === best.i && j < best.j)))) {
                            best = {cell: lo, i, j};
                        }
                    }
                }
            }
            if (best !== null) {
                const k = kinds.get(`${ck(best.cell)}|${best.i}|${best.j}`)!;
                return {kind: k.kind, cell: [best.cell[0], best.cell[1]], t, agents: [best.i, best.j], to: k.to};
            }
        }
        return null;
    };

    // 에피소드 하나 = 창 달린 CBS 한 그루. root는 실제 위치에서의 제약 없는 계획이고,
    // 큐가 비면 이 instance의 이 에피소드는 정직하게 실패다(창 안의 서브 탐색이 죽었다).
    const episode = (executed: Cell[][], tNow: number): {paths: Cell[][] | null; expanded: number} => {
        let expanded = 0;
        const rootPaths: Cell[][] = [];
        for (let i = 0; i < goals.length; i++) {
            const r = planOne(executed[i][tNow], goals[i], [], tNow, i);
            expanded += r.expanded;
            if (r.path === null) return {paths: null, expanded};
            rootPaths.push(executed[i].slice(0, tNow).concat(r.path));
        }

        let counter = 0; // 에피소드마다 새로 도는 CT 큐의 FIFO tie-break 카운터
        let rootCost = 0;
        for (const p of rootPaths) rootCost += p.length - 1;
        const queue = new CtHeap();
        queue.push({cost: rootCost, seq: counter++, node: {constraints: goals.map(() => [] as Constraint[]), paths: rootPaths}});

        while (queue.size > 0) {
            const top = queue.pop();
            const conflict = firstConflict(top.node.paths, tNow);
            if (conflict === null) return {paths: top.node.paths, expanded}; // 창-청결 — 이 에피소드의 해

            emit({
                event: "conflict_found", kind: conflict.kind, cell: [conflict.cell[0], conflict.cell[1]],
                t: conflict.t, agents: conflict.agents, to: conflict.to,
            });
            for (const agent of [conflict.agents[0], conflict.agents[1]]) {
                const childConstraints = top.node.constraints.map((c) => c.slice());
                childConstraints[agent].push({
                    kind: conflict.kind, cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t, to: conflict.to,
                });
                emit({
                    event: "constraint_added", agent, kind: conflict.kind,
                    cell: [conflict.cell[0], conflict.cell[1]], t: conflict.t, to: conflict.to,
                });
                const r = planOne(executed[agent][tNow], goals[agent], childConstraints[agent], tNow, agent);
                expanded += r.expanded;
                if (r.path === null) continue; // 서브 탐색이 죽은 자식은 push되지 않는다
                const childPaths = top.node.paths.slice();
                const oldCost = childPaths[agent].length - 1;
                childPaths[agent] = executed[agent].slice(0, tNow).concat(r.path);
                queue.push({
                    cost: top.cost - oldCost + (tNow + r.path.length - 1), seq: counter++,
                    node: {constraints: childConstraints, paths: childPaths},
                });
            }
        }
        return {paths: null, expanded}; // 큐 고갈 — 정직한 실패(판정이 아니라 예산 같은 증거)
    };

    // 성공: 모든 궤적을 동시 착지 스텝에서 자르고 보고한다. 각 agent의 비용은
    // FIRST 도착(논문의 flowtime)이고 makespan은 그 동시 착지 스텝이다.
    const finish = (executed: Cell[][], doneAt: number, calls: number): void => {
        const paths = executed.map((path) => path.slice(0, doneAt + 1));
        let cost = 0;
        for (let i = 0; i < paths.length; i++) {
            for (let s = 0; ; s++) {
                if (cellEq(paths[i][s], goals[i])) { cost += s; break; }
            }
        }
        paths.forEach((p, k) => emit({event: "path_found", path: p.map((c) => [c[0], c[1]] as Cell), agent: k}));
        emit({
            event: "planning_finished", success: true,
            metrics: {expanded_nodes: calls, makespan: doneAt, sum_of_costs: cost},
        });
    };

    const fail = (calls: number): void => {
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: calls, makespan: 0, sum_of_costs: 0},
        });
    };

    // 롤링 루프 — 매 주기 창 달린 CBS 에피소드를 다시 돌리고 h 스텝을 커밋한다.
    const run = (): void => {
        const k = tasks.length;
        const executed: Cell[][] = starts.map((s) => [[s[0], s[1]] as Cell]); // index t = 시각 t의 셀
        let tNow = 0;
        let calls = 0; // 모든 에피소드의 low-level pop 총합 — 롤링의 정직한 대가

        for (;;) {
            let allDone = true;
            for (let i = 0; i < k; i++) if (!cellEq(executed[i][tNow], goals[i])) { allDone = false; break }
            if (allDone) { finish(executed, tNow, calls); return } // 동시 착지 — 여기서 모두가 머문다
            if (tNow >= maxSteps) { fail(calls); return } // 정직한 예산 고갈 — RHCR은 그 이상을 말할 수 없다

            const ep = episode(executed, tNow);
            calls += ep.expanded;
            if (ep.paths === null) { fail(calls); return }

            // h 스텝 커밋: 실행된 실제 위치가 다음 에피소드의 start가 된다(pliable).
            let doneAt = -1;
            for (let s = tNow + 1; s <= tNow + period; s++) {
                for (let i = 0; i < k; i++) executed[i].push(occupiedAt(ep.paths[i], s));
                if (doneAt < 0) {
                    let all = true;
                    for (let i = 0; i < k; i++) if (!cellEq(executed[i][s], goals[i])) { all = false; break }
                    if (all) doneAt = s;
                }
            }
            if (doneAt >= 0) { finish(executed, doneAt, calls); return }
            tNow += period;
        }
    };
    run();
    return events;
}
