import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 PIBT. python/mrmp/decentralized/pibt.py의 정확한 미러 — 고정
// 이웃 순서(up/down/left/right), ε_i=(k-1-i)/k(IEEE-754 나눗셈은 언어 간 동일), 후보
// 정렬은 (goal 거리, 점유, row-major) 튜플 키 — 완전한 전순서라 Python의 stable sort와
// 결과 순서가 구조적으로 같다. 시각화용이 아니라 parity 검증의 한 축이다
// (scripts/check-engine-parity.mjs).

interface Sim {
    // 자유 셀 판정. 경계 밖은 자유 셀 집합에 애초부터 없다 (Python의 frozenset 멤버십과 동일).
    free: (c: Cell) => boolean;
    goals: Cell[];
    // dists[k]는 goal k에서 정적 BFS 거리 테이블 — 키가 없으면 도달 불가(순위에서 +inf로
    // 모든 도달 가능 셀 뒤에 정렬된다).
    dists: Array<Map<string, number>>;
    // pos는 π[t](현재 셀), nxt는 부분적으로 결정된 π[t+1] (null = 미결정 — 아직
    // 결정되지 않은 agent는 상속 사슬에 끌려갈 수 있다).
    pos: Cell[];
    nxt: Array<Cell | null>;
    // decision procedure 호출 총합 — 이 알고리즘엔 search frontier가 없으니 expanded_nodes metric이 곧 이것이다.
    calls: number;
}

const ck = (c: Cell): string => `${c[0]},${c[1]}`;
const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];

// 고정 순서 4-이웃. Python의 _nbrs(free, c)와 동일한 순서다. wait self-loop은 이
// 알고리즘의 액션이 아니다 — 제자리는 후보 목록에 항상 들어가는 현재 셀 자체다.
const nbrs = (free: (c: Cell) => boolean, c: Cell): Cell[] => {
    const out: Cell[] = [];
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
        const n: Cell = [c[0] + dr, c[1] + dc];
        if (free(n)) out.push(n);
    }
    return out;
};

// goal에서 시작하는 정적 BFS 거리 테이블(고정 이웃 순서). 도달성만 — 거리는 설계상
// 다른 모든 agent를 무시한다.
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

// 논문의 재귀 절차. agent i가 다음 셀을 고른다; UNDECIDED agent k가 점유한 후보는
// 상속을 유발하고(k가 i의 claim을 상속받아 비켜야 하거나 이 claim을 실패시켜야 한다),
// claimant의 현재 셀은 i의 후보에서 제외되므로 swap은 결코 성립하지 않는다. 이미 아무나
// 주장한 후보는 그냥 건너뛴다 — vertex conflict는 구조적으로 불가능하다.
const decide = (sim: Sim, i: number, fromAgent: number | null): boolean => {
    sim.calls += 1;
    const cur = sim.pos[i];
    const dist = sim.dists[i];

    // 후보: 현재 셀 + 자유 이웃(고정 순서), 고정 순위로 정렬 — goal 거리 오름차순
    // (도달 불가 = +inf는 전부 뒤), tie 시 미점유 우선, 최종 tie-break row-major.
    const candidates = nbrs(sim.free, cur);
    candidates.push(cur);
    const rank = (u: Cell): [number, number, number, number] => {
        const d = dist.get(ck(u));
        let occupied = 0;
        for (const pos of sim.pos) if (cellEq(pos, u)) occupied = 1;
        return [d === undefined ? Infinity : d, occupied, u[0], u[1]];
    };
    // 키는 셀마다 완전하게 서로 다르므로 이 비교는 전순서다 (뺄셈이 아니라 비교 —
    // Infinity끼리의 뺄셈은 NaN이라 쓰지 않는다).
    candidates.sort((a, b) => {
        const ra = rank(a), rb = rank(b);
        for (let j = 0; j < 4; j++) if (ra[j] !== rb[j]) return ra[j] < rb[j] ? -1 : 1;
        return 0;
    });

    for (const v of candidates) {
        let claimed = false;
        for (const nxt of sim.nxt) if (nxt !== null && cellEq(nxt, v)) claimed = true;
        if (claimed) continue;  // 이미 이 스텝에서 주장됨 — 정의상 vertex conflict
        if (fromAgent !== null && cellEq(sim.pos[fromAgent], v)) continue;  // swap 금지 가드
        sim.nxt[i] = v;  // 재귀 전 투기적 claim — 이것이 점유자를 비켜야 하게 만든다
        let occ: number | null = null;
        for (let j = 0; j < sim.pos.length; j++) if (cellEq(sim.pos[j], v)) { occ = j; break; }
        if (occ !== null && occ !== i && sim.nxt[occ] === null) {
            if (decide(sim, occ, i)) return true;
            sim.nxt[i] = null;  // backtrack — claim은 일어나지 않았다
            continue;
        }
        return true;  // 비어 있거나 점유자가 이미 비기로 했다: 확정
    }
    sim.nxt[i] = cur;  // 막혔다: 제자리에 머물고 윗선에 invalid를 알린다
    return false;
};

export function runPibt(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "pibt", params});

    const W = map.width, H = map.height;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    const starts = tasks.map(([s]) => [s[0], s[1]] as Cell);
    const goals = tasks.map(([, g]) => [g[0], g[1]] as Cell);
    // 논문의 인스턴스는 정의상 well-formed (단사 start/goal, 통과 가능 셀). 위반 입력은
    // 이 문제의 인스턴스가 아니다 — 계획 없음으로 정직하게 보고한다.
    if (new Set(starts.map(ck)).size < starts.length
        || new Set(goals.map(ck)).size < goals.length
        || [...starts, ...goals].some((c) => !free(c))) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    const k = tasks.length;
    const maxSteps = params.max_steps as number;
    const sim: Sim = {free, goals, dists: [], pos: [...starts], nxt: [], calls: 0};
    // 정적 거리 테이블 (논문: goal별 BFS를 사전에 한 번).
    for (const g of goals) sim.dists.push(bfs(sim, g));

    // 고정 우선순위: agent 0 최고 (ε가 1에 가장 가까움). 구성상 서로 다르다.
    const eps = [...Array(k).keys()].map((i) => (k - 1 - i) / k);
    let p = [...eps];
    const paths: Cell[][] = starts.map((s) => [[s[0], s[1]] as Cell]);

    let t = 0;
    for (;;) {
        if (sim.pos.some((pos, i) => !cellEq(pos, goals[i]))) {
            if (t >= maxSteps) {
                // 정직한 budget 소진 — unsolvability의 증명이 아니다.
                emit({event: "planning_finished", success: false,
                    metrics: {expanded_nodes: sim.calls, makespan: 0, sum_of_costs: 0}});
                return events;
            }
            // 우선순위 갱신 (논문 7행): goal 위면 ε로 리셋, 이동 중이면 +1.
            p = p.map((pi, i) => (cellEq(sim.pos[i], goals[i]) ? eps[i] : pi + 1.0));
            // 내림차순 우선순위; 값이 서로 달라 순서는 전순서다.
            const order = [...Array(k).keys()].sort((a, b) => p[b] - p[a]);
            sim.nxt = new Array<Cell | null>(k).fill(null);
            for (const i of order) if (sim.nxt[i] === null) decide(sim, i, null);
            t += 1;
            // 동시 커밋: 모든 결정은 시각 t의 위치에 대해 내려지고, 전부 결정된 뒤에
            // 위치가 전진한다 (top-level 호출은 실패하지 않는다 — Lemma 1).
            for (let i = 0; i < k; i++) {
                const next = sim.nxt[i];
                if (next === null) throw new Error("pibt: top-level decision failed (Lemma 1 violated)");
                paths[i].push([next[0], next[1]]);
                sim.pos[i] = [next[0], next[1]];
            }
        } else break;
    }

    let cost = 0;
    for (const path of paths) {
        for (let s = 1; s < path.length; s++) if (!cellEq(path[s], path[s - 1])) cost += 1;
    }
    for (let i = 0; i < paths.length; i++) emit({event: "path_found", path: paths[i], agent: i});
    emit({event: "planning_finished", success: true,
        metrics: {expanded_nodes: sim.calls, makespan: t, sum_of_costs: cost}});
    return events;
}
