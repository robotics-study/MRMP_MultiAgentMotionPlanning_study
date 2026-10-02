import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 winPIBT. python/mrmp/decentralized/winpibt.py의 정확한 미러 —
// 고정 이웃 순서(up/down/left/right), ε_i=(k-1-i)/k(IEEE-754 나눗셈은 언어 간 동일),
// 이상 경로 탐색의 힙도 Python heapq와 동일한 전순서(f 오름차순, 동률은 더 늦은 스텝
// 우선, 그다음이 삽입 순서 FIFO). 시각화용이 아니라 parity 검증의 한 축이다
// (scripts/check-engine-parity.mjs).

interface Sim {
    // 자유 셀 판정. 경계 밖은 자유 셀 집합에 애초부터 없다 (Python의 frozenset 멤버십과 동일).
    free: (c: Cell) => boolean;
    goals: Cell[];
    // dists[k]는 goal k에서 정적 BFS 거리 테이블 — 키가 없으면 도달 불가(무한대로 취급).
    dists: Array<Map<string, number>>;
    // paths[i]는 agent i의 잠정 경로 — ℓ_i(ell[i])까지 등록된 셀들. t 시각 위치는 paths[i][t].
    paths: Cell[][];
    ell: number[];
    eps: number[];
    // winpibt() 결정 절차 호출 총합 — 이 알고리즘엔 search frontier가 없으니 expanded_nodes metric이 곧 이것이다.
    calls: number;
}

const ck = (c: Cell): string => `${c[0]},${c[1]}`;
const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
// 도달 불가 거리의 값 — Python의 float('inf')와 동일하게 모든 도달 가능 거리 뒤에 정렬된다.
const INF = Number.POSITIVE_INFINITY;

// 고정 순서 4-이웃. Python의 _nbrs(free, c)와 동일한 순서다. wait self-loop은 이
// 알고리즘의 액션이 아니다 — 제자리는 이상 경로가 셀을 반복해 표현한다(논문의 wait padding).
const nbrs = (free: (c: Cell) => boolean, c: Cell): Cell[] => {
    const out: Cell[] = [];
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
        const n: Cell = [c[0] + dr, c[1] + dc];
        if (free(n)) out.push(n);
    }
    return out;
};

// goal에서 시작하는 정적 BFS 거리 테이블(고정 이웃 순서). 도달성만 — 거리는 설계상 다른 모든 agent를 무시한다.
const bfs = (sim: Sim, goal: Cell): Map<string, number> => {
    const dist = new Map<string, number>([[ck(goal), 0]]);
    const queue: Cell[] = [goal];
    let head = 0;
    while (head < queue.length) {
        const c = queue[head++];
        for (const n of nbrs(sim.free, c)) {
            if (!dist.has(ck(n))) {
                dist.set(ck(n), dist.get(ck(c))! + 1);
                queue.push(n);
            }
        }
    }
    return dist;
};

interface HeapEntry {
    f: number;
    negT: number;  // -step — 동률에서 더 늦은 스텝이 먼저 popped된다 (Python 튜플의 두 번째 항)
    seq: number;   // 삽입 순서 — 마지막 tie-break(FIFO)
    cell: Cell;
}

// (f, negT, seq) 사전식 min-heap — Python heapq의 튜플 비교와 동일한 전순서.
class Heap {
    private a: HeapEntry[] = [];

    get size(): number { return this.a.length }

    push(n: HeapEntry): void {
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

    pop(): HeapEntry {
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

    private less(x: HeapEntry, y: HeapEntry): boolean {
        if (x.f !== y.f) return x.f < y.f;
        if (x.negT !== y.negT) return x.negT < y.negT;
        return x.seq < y.seq;
    }
}

// 유효성 규칙 — τ에 v2로 들어가는 스텝은, ℓ_j ≥ τ인 다른 agent j가 v2를 [τ, min(β, ℓ_j)]
// 어떤 시각에서도 점유하면 invalid하고, 완전히 확보된 이동과 swap해도 invalid하다. ℓ_j 너머엔
// 아무것도 결정되지 않았다: 제약이 없고, v2에 아직 선 사람은 그 스텝이 확보될 때 상속으로 끌려온다.
const validStep = (sim: Sim, i: number, v1: Cell, v2: Cell, tau: number, beta: number): boolean => {
    for (let j = 0; j < sim.paths.length; j++) {
        if (j === i) continue;
        const ellJ = sim.ell[j];
        if (ellJ >= tau) {
            const pathJ = sim.paths[j];
            for (let x = tau; x <= Math.min(beta, ellJ); x++) {
                if (cellEq(pathJ[x], v2)) return false;  // 확보된 예약은 눈에 보이게 끝날 때까지 독이다
            }
            if (cellEq(pathJ[tau], v1) && cellEq(pathJ[tau - 1], v2)) return false;  // swap
        }
    }
    return true;
};

// 세그먼트의 모든 스텝이 disentangled해야 한다.
const valid = (sim: Sim, i: number, seg: Cell[], t1: number, beta: number): boolean => {
    for (let j = 1; j < seg.length; j++) {
        if (!validStep(sim, i, seg[j - 1], seg[j], t1 + j, beta)) return false;
    }
    return true;
};

// 경로가 t1 시각에 v에서 시작해 β까지 모든 VISIBLE 예약과 disentangled로 걷히는 이상 경로 —
// validPath + registerPath를 하나의 탐색으로 (논문이 "이상적 경로를 계산"에 남긴 자유도를 여기서 고정한다).
// (셀, 스텝) 상태 위 A*: f = 스텝 + goal까지 정적 BFS 거리, 동률은 더 늦은 스텝 우선 그다음 삽입 순서.
// pop된 모든 상태에서 고정 정적 경로를 새로 시도하고(성공 = β까지 생존), 실패하면 한 스텝 더 퍼진다.
const idealPath = (sim: Sim, i: number, t1: number, beta: number): Cell[] | null => {
    const start = sim.paths[i][t1];
    const dist = sim.dists[i];
    let seq = 0;
    const heap = new Heap();
    // parent는 상태 (셀, 스텝) → 부모 상태. 시작의 부모는 null — 체인의 끝.
    const parent = new Map<string, { cell: Cell; t: number } | null>();
    parent.set(`${ck(start)},${t1}`, null);
    const closed = new Set<string>();
    heap.push({f: dist.has(ck(start)) ? dist.get(ck(start))! : INF, negT: -t1, seq: seq++, cell: start});

    // 체인을 시작 스텝까지 거슬러 오른다 — 인덱스 0이 시작 스텝이다.
    const chain = (state: { cell: Cell; t: number }): Cell[] => {
        const cells: Cell[] = [];
        let cur: { cell: Cell; t: number } | null = state;
        while (cur !== null && cur !== undefined) {
            cells.push(cur.cell);
            cur = parent.get(`${ck(cur.cell)},${cur.t}`)!;
        }
        return cells.reverse();
    };

    while (heap.size > 0) {
        const e = heap.pop();
        const gT = -e.negT;
        const v = e.cell;
        if (gT >= beta) return chain({cell: v, t: gT});  // 지평선까지 생존 — 체인을 되짚는다
        // 고정 정적 경로: start에서 goal까지 BFS parent 체인(wait로 pad / β에 truncate).
        let seg: Cell[] | null = null;
        if (dist.has(ck(v))) {
            const cells: Cell[] = [v];
            let c = v;
            while (!cellEq(c, sim.goals[i])) {
                const d = dist.get(ck(c))!;
                // 고정 순서가 parent를 결정 — 구성상 고정된다. 도달 가능 non-goal은 항상 거리 감소 이웃이 있다.
                const nxt = nbrs(sim.free, c).find((n) => (dist.has(ck(n)) ? dist.get(ck(n))! : INF) === d - 1)!;
                c = nxt;
                cells.push(c);
            }
            while (cells.length - 1 + gT < beta) cells.push(sim.goals[i]);
            while (cells.length - 1 + gT > beta) cells.pop();
            seg = cells;
        }
        if (seg !== null && valid(sim, i, seg, gT, beta)) {
            const out = chain({cell: v, t: gT});
            for (let s = 1; s < seg.length; s++) out.push(seg[s]);
            return out;
        }
        closed.add(`${ck(v)},${gT}`);
        const moves = nbrs(sim.free, v);
        moves.push(v);  // 고정 순서, wait은 마지막
        for (const m of moves) {
            const stateKey = `${ck(m)},${gT + 1}`;
            if (closed.has(stateKey) || parent.has(stateKey)) continue;
            if (!validStep(sim, i, v, m, gT + 1, beta)) continue;
            parent.set(stateKey, {cell: v, t: gT});
            heap.push({
                f: (gT + 1) + (dist.has(ck(m)) ? dist.get(ck(m))! : INF),
                negT: -(gT + 1), seq: seq++, cell: m,
            });
        }
    }
    return null;
};

// t 시각에 v에서 경로가 t보다 전에 끝나는 첫 agent(index 순서) — 스텝 t에 확보되는 칸을 비켜야 할 사람.
// 등록 경로가 t를 덮는 agent는 대상이 아니다: 그 예약이 이미 유효성을 지배한다.
const target = (sim: Sim, v: Cell, t: number): number | null => {
    for (let j = 0; j < sim.paths.length; j++) {
        if (sim.ell[j] < t && cellEq(sim.paths[j][sim.ell[j]], v)) return j;
    }
    return null;
};

export function runWinpibt(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "winpibt", params});

    const W = map.width, H = map.height;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    const starts = tasks.map(([s]) => [s[0], s[1]] as Cell);
    const goals = tasks.map(([, g]) => [g[0], g[1]] as Cell);
    // 논문의 인스턴스는 정의상 well-formed (단사 start/goal, 통과 가능 셀). 위반 입력은 이 문제의 인스턴스가 아니다.
    if (new Set(starts.map(ck)).size < starts.length
        || new Set(goals.map(ck)).size < goals.length
        || [...starts, ...goals].some((c) => !free(c))) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    const k = tasks.length;
    const w = params.window as number;
    const maxSteps = params.max_steps as number;
    const sim: Sim = {free, goals, dists: [], paths: [], ell: new Array(k).fill(0), eps: [], calls: 0};
    for (const g of goals) sim.dists.push(bfs(sim, g));
    for (const s of starts) sim.paths.push([[s[0], s[1]]]);
    // 고정 우선순위: agent 0 최고 (ε가 1에 가장 가까움). 구성상 서로 다르다.
    for (let i = 0; i < k; i++) sim.eps.push((k - 1 - i) / k);

    // winpibt(i, alpha) — 논문의 Algorithm 1. α까지 경로를 연장하고 새 스텝을 하나씩 확보한다.
    const winpibt = (i: number, alpha: number): boolean => {
        sim.calls += 1;
        const ellI = sim.ell[i];
        if (ellI >= alpha) return true;  // 이미 α 너머까지 등록됐다
        // 진입에서 고정되는 예언적 시각 β
        let beta = alpha;
        for (const p of sim.paths) beta = Math.max(beta, p.length - 1);

        const path = idealPath(sim, i, ellI, beta);
        if (path === null) {
            // valid walk 없음 — copeStuck이 α까지 wait을 못 박는다
            const v = sim.paths[i][ellI];
            for (let s = 0; s < alpha - ellI; s++) sim.paths[i].push(v);
            sim.ell[i] = alpha;
            return false;
        }
        // hard mode: 이상 경로 전체를 한 번에 등록하고, 확보된 접두어(ℓ_i)만 남에게 보인다.
        for (let s = 1; s <= alpha - ellI; s++) sim.paths[i].push(path[s]);

        let t = ellI + 1;
        while (t <= alpha) {
            const v = sim.paths[i][t];
            sim.ell[i] = t;  // 확보된 스텝은 지금 이 순간에야 눈에 보인다
            // retroactive: ℓ_j < t-1로 v에 서 있던 사람은 한 스텝씩 연장해 t-1 너머나 v 밖으로 나간다 (반환값 무시).
            let j = target(sim, v, t - 1);
            while (j !== null) {
                winpibt(j, sim.ell[j] + 1);
                j = target(sim, v, t - 1);
            }
            // PIBT 핵심: 정확히 스텝 t-1에 서 있던 점유자가 상속받아 t까지 비켜야 하고, 실패하면 이 호출의 연장을 backtrack한다.
            j = target(sim, v, t);
            if (j !== null) {
                if (!winpibt(j, t)) {
                    while (sim.paths[i].length > t) sim.paths[i].pop();  // Π_i(t..α) 회수
                    sim.ell[i] = t - 1;
                    const replan = idealPath(sim, i, t - 1, beta);
                    if (replan === null) {  // copeStuck: 마지막 확보 셀에 wait을 못 박는다
                        const v2 = sim.paths[i][t - 1];
                        for (let s = 0; s < alpha - (t - 1); s++) sim.paths[i].push(v2);
                        sim.ell[i] = alpha;
                        return false;
                    }
                    for (let s = 1; s <= alpha - t + 1; s++) sim.paths[i].push(replan[s]);
                    continue;  // 재계획된 경로에 대해 스텝 t 확보를 다시 시도
                }
            }
            if (cellEq(v, sim.goals[i]) && t < alpha) {
                // hard mode 고정 goal: 도착이 확보된 뒤 tail을 goal에서 다시 계획한다 — 대기가
                // disentangled되는 순간 그 위에 주차하고 β까지 돌아다니지 않는다. 이 재계획은 결코 실패할 수 없다.
                const replan = idealPath(sim, i, t, beta);
                if (replan === null) throw new Error("winpibt: goal-tail replan failed (impossible by construction)");
                for (let s = 1; s <= alpha - t; s++) sim.paths[i][t + s] = replan[s];
            }
            t += 1;
        }
        return true;
    };

    let t = 0;
    let kappa = 0;  // κ: 낮은 우선순위가 아직 예약할 수 있는 상한 (Algorithm 2 line 14/15)
    let p = [...sim.eps];  // 우선순위는 ε에서 시작한다 (η는 0에서 시작)
    for (;;) {
        if (sim.paths.some((path, i) => !cellEq(path[t], sim.goals[i]))) {
            if (t >= maxSteps) {
                // 정직한 budget 소진 — unsolvability의 증명이 아니다.
                emit({event: "planning_finished", success: false,
                    metrics: {expanded_nodes: sim.calls, makespan: 0, sum_of_costs: 0}});
                return events;
            }
            // 우선순위 갱신 (Algorithm 2 line 3): goal 위면 ε로 리셋, 이동 중이면 +1.
            p = p.map((pi, i) => (cellEq(sim.paths[i][t], sim.goals[i]) ? sim.eps[i] : pi + 1.0));
            // 내림차순 우선순위; 값이 서로 달라 순서는 전순서다 (안정 정렬이라 동률은 index 순서).
            const order = [...Array(k).keys()].sort((a, b) => p[b] - p[a]);
            for (let j = 0; j < order.length; j++) {
                const i = order[j];
                if (sim.ell[i] <= t) {  // 아직 현재 스텝 너머로 등록되지 않았다 (line 7)
                    const alpha = j === 0 ? t + w : Math.min(t + w, kappa);
                    winpibt(i, alpha);  // top-level 호출 — 판정은 여기서 끝난다
                }
                kappa = j === 0 ? sim.ell[i] : Math.min(kappa, sim.ell[i]);
            }
            t += 1;
        } else break;
    }

    let cost = 0;
    const paths: Cell[][] = [];
    for (const path of sim.paths) {
        const full = path.slice(0, t + 1);
        for (let s = 1; s <= t; s++) if (!cellEq(full[s], full[s - 1])) cost += 1;
        paths.push(full);
    }
    for (let i = 0; i < paths.length; i++) emit({event: "path_found", path: paths[i], agent: i});
    emit({event: "planning_finished", success: true,
        metrics: {expanded_nodes: sim.calls, makespan: t, sum_of_costs: cost}});
    return events;
}
