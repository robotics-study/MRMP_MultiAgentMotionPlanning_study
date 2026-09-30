import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 Push and Swap. python/mrmp/search/push_and_swap.py의 정확한
// 미러 — 고정 이웃 순서(up/down/left/right, wait self-loop는 이 알고리즘의 액션이
// 아니다), BFS parent Map의 삽입 순서가 곧 인입(dequeue) 순서(Python dict와 동일)라
// 구멍 선택과 후보 반복이 언어 간에 일치한다. 시각화용이 아니라 parity 검증의 한 축이다
// (scripts/check-engine-parity.mjs).

// 실행된 이동 하나 (agent, from, to) — swap이 rollback을 위해 기록하는 단위.
type Move = [number, Cell, Cell];

interface Sim {
    // 자유 셀 판정. 경계 밖은 자유 셀 집합에 애초부터 없다 (Python의 frozenset 멤버십과 동일).
    free: (c: Cell) => boolean;
    // 현재 배정(agent index → 셀)과 목표 배정.
    A: Cell[];
    T: Cell[];
    // 이미 goal에 도착한 agent들의 goal 셀 집합 — push의 hole 찾기 BFS는 이 셀들을 벽으로 본다.
    U: Set<string>;
    // 실행 기록. 이동마다 새 배정이 append되고, swap rollback은 이 목록을 자른다.
    Pi: Cell[][];
    // 모든 shortest-path/hole BFS 호출의 dequeue 총합 — 이 알고리즘엔 자체 search
    // frontier가 없으니 expanded_nodes metric이 곧 이것이다.
    expandedNodes: number;
}

const ck = (c: Cell): string => `${c[0]},${c[1]}`;
const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];

// 고정 순서 4-이웃. Python의 _nbrs(free, c)와 동일한 순서다.
const nbrs = (sim: Sim, c: Cell): Cell[] => {
    const out: Cell[] = [];
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
        const n: Cell = [c[0] + dr, c[1] + dc];
        if (sim.free(n)) out.push(n);
    }
    return out;
};

// 셀을 점유하는 agent의 index (배정은 단사), 없으면 null.
const occupant = (sim: Sim, c: Cell): number | null => {
    for (let i = 0; i < sim.A.length; i++) if (cellEq(sim.A[i], c)) return i;
    return null;
};

// 한 행동: agent가 빈 자유 셀 to로 한 칸 옮기고 새 배정이 Pi에 붙는다. 투기적 swap
// 후보 안에서는 지역 segment에도 기록돼 시도 전체를 되돌릴 수 있다.
const move = (sim: Sim, segment: Move[] | null, agent: number, to: Cell): void => {
    const frm = sim.A[agent];
    // 모든 호출 지점이 구성하는 불변식 — 검사하고 위반 시 던진다(복구하지 않는다).
    if (cellEq(frm, to) || !sim.free(to) || occupant(sim, to) !== null) {
        throw new Error("push_and_swap: move invariant violated");
    }
    sim.A[agent] = to;
    sim.Pi.push([...sim.A]);
    if (segment !== null) segment.push([agent, frm, to]);
};

interface BfsResult {
    // 삽입 순서 = BFS 인입 순서 = dequeue 순서. Python의 parent dict가 parent 지도와
    // 순서 목록 역할을 겸하므로, 여기서는 순서 배열을 명시적으로 따로 둔다.
    order: Cell[];
    parent: Map<string, Cell | null>;
}

// SHORTEST_PATH와 hole 찾기 공용 BFS — blocked를 피해 고정 이웃 순서로 돈다.
const bfsParent = (sim: Sim, start: Cell, blocked: Set<string>): BfsResult => {
    const res: BfsResult = {order: [], parent: new Map()};
    if (blocked.has(ck(start))) return res;  // 빈 BFS — 확장은 0으로 계산된다
    res.order.push(start);
    res.parent.set(ck(start), null);
    const queue: Cell[] = [start];
    let head = 0;
    while (head < queue.length) {
        const c = queue[head++];
        for (const n of nbrs(sim, c)) {
            if (res.parent.has(ck(n)) || blocked.has(ck(n))) continue;
            res.parent.set(ck(n), c);
            res.order.push(n);
            queue.push(n);
        }
    }
    // 모든 인입 셀은 정확히 한 번 dequeue되므로 order.length가 이 호출의 확장 수다.
    sim.expandedNodes += res.order.length;
    return res;
};

// 정적 BFS 최단경로 start → goal (도달 불가면 null). parent 체인을 거슬러 복원한다.
const pathTo = (sim: Sim, start: Cell, goal: Cell): Cell[] | null => {
    const res = bfsParent(sim, start, new Set());
    if (!res.parent.has(ck(goal))) return null;
    const out: Cell[] = [];
    let cur: Cell = goal;
    for (;;) {
        out.push(cur);
        const p = res.parent.get(ck(cur))!;  // root의 parent는 null — 그 전까진 항상 존재
        if (p === null) break;
        cur = p;
    }
    return out.reverse();
};

// 알고리즘 2(PUSH): r을 A[r]에서 T[r]까지 정적 최단경로를 따라 걷히고, 경로를 막는
// 점유자는 chain-push로 가장 가까운 빈 셀에 밀어 넣는다. hole 찾기 BFS의 blocked 집합은
// 정확히 {A[r]} ∪ U — 주차된 agent는 결코 밀릴 수 없다.
const push = (sim: Sim, r: number): boolean => {
    if (cellEq(sim.A[r], sim.T[r])) return true;  // 공리적 완료 (방어적 — 호출자가 A[r] != T[r]에서 루프)
    const pStar = pathTo(sim, sim.A[r], sim.T[r]);
    if (pStar === null) return false;  // 정적으로 도달 불가 → 상류에서 정직한 실패
    let idx = 1;
    while (!cellEq(sim.A[r], sim.T[r])) {
        // r은 p_star[idx-1] 위에 서 있고 ≠ T[r] == p_star[last] — 다음 셀은 존재한다.
        const v = pStar[idx];
        if (occupant(sim, v) === null) {
            move(sim, null, r, v);
            idx += 1;
            continue;
        }
        // v에서 막혔다. 주차된(U) 점유자는 자기 자신을 blocked로 표시하므로 BFS가 시작조차
        // 못 한다. 아니면 인입 순서상 첫 빈 셀이 hole이고 chain의 점유자들이 r에서 먼 쪽부터
        // 한 칸씩 hole을 향해 밀린다.
        const blocked = new Set<string>(sim.U);
        blocked.add(ck(sim.A[r]));
        if (!chainPush(sim, null, v, blocked)) return false;
    }
    return true;
};

// PUSH와 MULTIPUSH가 공유하는 이동(논문 알고리즘 2의 9~22행): blocker 셀에서 BFS로
// blocked를 피해, 인입/dequeue 순서상 첫 빈 셀을 hole으로 고르고 blocker→hole parent
// chain 위의 점유자들이 r에서 먼 쪽부터 한 칸씩 밀린다.
const chainPush = (sim: Sim, segment: Move[] | null, blocker: Cell, blocked: Set<string>): boolean => {
    const res = bfsParent(sim, blocker, blocked);
    let vEmpty: Cell | null = null;
    for (const c of res.order) {  // 삽입 순서 == BFS 인입 순서 == dequeue 순서
        if (occupant(sim, c) === null) {
            vEmpty = c;
            break;
        }
    }
    if (vEmpty === null) return false;
    const chain: Cell[] = [];
    let cur: Cell = vEmpty;
    for (;;) {
        chain.push(cur);
        const p = res.parent.get(ck(cur))!;
        if (p === null) break;
        cur = p;
    }
    chain.reverse();  // blocker 셀이 먼저, hole이 마지막
    // r에서 먼 점유자가 먼저 움직여 각 대상 셀이 이미 비어 있게 만든다.
    for (let k = chain.length - 2; k >= 0; k--) {
        const a = occupant(sim, chain[k]);
        if (a !== null) move(sim, segment, a, chain[k + 1]);
    }
    return true;
};

// 알고리즘 3(SWAP): r의 첫 수를 막는 agent s와 r을 맞바꾼다. 논문은 swap 후보를 모든
// degree ≥ 3 vertex로 두고 POP() 순서를 비웠고, 여기선 A[r]에서 BFS dequeue 순서
// (가장 가까운 후보 먼저)로 고정한다. 격자에서 swap은 물리적으로 빈 2×2 block을 필요로
// 하고, 그것이 바로 clearAndSwap이 판정하는 것이다. 실패한 후보는 투기적 — 이동들을
// rollback하고, 성공한 swap의 EXECUTE_SWAP 이동들만 solution에 직행한다.
const swap = (sim: Sim, r: number): boolean => {
    if (cellEq(sim.A[r], sim.T[r])) return true;  // 공리적 완료 (방어적)
    const pStar = pathTo(sim, sim.A[r], sim.T[r]);
    if (pStar === null || pStar.length < 2) return false;
    const sOpt = occupant(sim, pStar[1]);
    if (sOpt === null) return false;  // r의 첫 수를 막는 것이 없다 — push가 실패했을 리 없다
    const s = sOpt;
    for (const v of bfsParent(sim, sim.A[r], new Set()).order) {
        const before = [...sim.A];
        const piLen = sim.Pi.length;
        const segment: Move[] = [];
        if (!multipush(sim, segment, r, s, v)) {
            sim.A = before;
            sim.Pi.length = piLen;  // 투기적 이동들을 trace에서도 되돌린다
            continue;
        }
        if (!clearAndSwap(sim, segment, r, s)) {
            sim.A = before;
            sim.Pi.length = piLen;
            continue;
        }
        // 성공. 순방향 segment는 이미 Pi에 이동 단위로 쌓였고 EXECUTE_SWAP 이동들은
        // 직행했다. 거꾸로 replay한 segment가 r/s 역할을 바꿔 모든 밀려난 agent —
        // U의 주차된 agent 포함 — 를 제자리에 되돌린다.
        for (let i = segment.length - 1; i >= 0; i--) {
            const [agent, frm, to] = segment[i];
            const who = agent === r ? s : (agent === s ? r : agent);
            if (!cellEq(sim.A[who], to)) throw new Error("push_and_swap: swap replay does not land");
            move(sim, null, who, frm);
        }
        // 맞바꿔 쫓겨난 s가 이미 goal에 있던 agent면(U) 둘 다 제자리로 돌려보낸다.
        if (sim.U.has(ck(sim.T[s]))) return resolve(sim, r, s);
        return true;
    }
    return false;
};

// 복합 짝 이동(논문의 MULTIPUSH): r이 p = SHORTEST_PATH(A[r], v)를 따라 lead하고 s가
// 비워진 셀을 follow로 채운다. 경로 위 제3자 점유자는 chain-push로 치우되 U는 무시한다
// — swap은 주차된 agent도 건드릴 수 있고 replay가 되돌린다. 그 BFS에서는 짝의 셀 둘 다 blocked.
const multipush = (sim: Sim, segment: Move[], r: number, s: number, v: Cell): boolean => {
    const p = pathTo(sim, sim.A[r], v);  // p는 A[lead]에서 시작 — lead는 항상 r
    if (p === null || p.length < 2) return false;
    for (const c of p) {
        if (cellEq(c, sim.A[s])) {
            // s가 r 앞 경로 위에 앉아 있으면 복합체가 v에 닿을 수 없고 후보는 실패한다.
            return false;
        }
    }
    const lead = r, follow = s;
    let adjacent = false;
    for (const n of nbrs(sim, sim.A[lead])) if (cellEq(n, sim.A[follow])) adjacent = true;
    if (!adjacent) return false;  // 방어적 — 구성상 짝은 인접해 있다
    let i = 0;  // p가 A[lead]에서 시작하니 lead의 index는 0
    while (!cellEq(sim.A[lead], v)) {
        const w = p[i + 1];
        const a = occupant(sim, w);
        if (a !== null && a !== follow) {
            if (!chainPush(sim, segment, w, new Set([ck(sim.A[lead]), ck(sim.A[follow])]))) return false;
        }
        move(sim, segment, lead, w);
        i += 1;
        const vacated = p[i - 1];  // lead가 방금 비운 셀 — 구성상 follow에 인접
        if (occupant(sim, vacated) === null) move(sim, segment, follow, vacated);
    }
    let stillAdjacent = false;
    for (const n of nbrs(sim, sim.A[r])) if (cellEq(n, sim.A[s])) stillAdjacent = true;
    return cellEq(sim.A[r], v) && stillAdjacent;
};

// CLEAR + EXECUTE_SWAP 융합(논문의 case 1/2): r이 v = A[r]에, s가 그 이웃 w1 = A[s]에
// 있다. w2 ∈ N(v)\\{w1}과 w4 ∈ N(w1)\\{v} ∩ N(w2)가 clearable(점유자는 자기 첫 빈 이웃으로
// 한 칸 물러남)이면 교환이 존재한다. 이후 네 이동 r:v→w2, s:w1→v, r:w2→w4, r:w4→w1이
// 짝을 맞바꾼다 — 이 이동들은 Pi에 직행하고 그 앞 segment만 가역이다.
const clearAndSwap = (sim: Sim, segment: Move[], r: number, s: number): boolean => {
    const v = sim.A[r], w1 = sim.A[s];
    const nbrsV = nbrs(sim, v);
    let adjacent = false;
    for (const n of nbrsV) if (cellEq(n, w1)) adjacent = true;
    if (!adjacent) return false;
    for (const w2 of nbrsV.filter((w) => !cellEq(w, w1))) {
        if (occupant(sim, w2) !== null && !clearCell(sim, segment, w2)) continue;
        const nbrsW1 = nbrs(sim, w1);
        for (const w4 of nbrsW1.filter((w) => !cellEq(w, v) && nbrs(sim, w2).some((x) => cellEq(x, w)))) {
            if (occupant(sim, w4) !== null && !clearCell(sim, segment, w4)) continue;
            // w4를 비우는 동작이 이미 비워 둔 w2를 다시 채울 수 있다 — 밀려난 점유자의 첫 빈
            // 이웃이 하필 방금 비운 그 w2인 경우(3명 이상에서 도달 가능). 이 시도는 다른 clear
            // 실패와 동일하게 실패한다(continue; 모든 시도가 실패하면 후보 전체가 실패하고 rollback).
            if (occupant(sim, w2) !== null) continue;
            // EXECUTE_SWAP: r이 v를 비워 s에게 내주고 block을 돌아 w1으로 들어간다.
            move(sim, null, r, w2);
            move(sim, null, s, v);
            move(sim, null, r, w4);
            move(sim, null, r, w1);
            return true;
        }
    }
    return false;
};

// vertex 하나 비우기: 점유자가 고정 순서상 자기 첫 빈 이웃으로 한 칸 물러난다.
// 빈 이웃이 없으면 이 swap 후보는 실패다.
const clearCell = (sim: Sim, segment: Move[], x: Cell): boolean => {
    for (const n of nbrs(sim, x)) {
        if (occupant(sim, n) === null) {
            const a = occupant(sim, x);
            if (a === null) throw new Error("push_and_swap: clearCell on an empty cell");  // 호출자는 점유된 셀에서만 부른다
            move(sim, segment, a, n);
            return true;
        }
    }
    return false;
};

// 맞바꿔 쫓겨난 s가 이미 goal에 도착해 있던 agent였던 경우(논문 case 1/2): r을
// T[s]에서 밀어내고 같은 방식으로 s를 집으로 보낸다. 여기서 swap이 실패하면 원래
// swap까지 무효화된다 — 정직한 실패.
const resolve = (sim: Sim, r: number, s: number): boolean => {
    while (cellEq(sim.A[r], sim.T[s])) {
        if (!push(sim, r) && !swap(sim, r)) return false;
    }
    while (!cellEq(sim.A[s], sim.T[s])) {
        if (!push(sim, s) && !swap(sim, s)) return false;
    }
    return true;
};

export function runPushAndSwap(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "push_and_swap", params});

    const W = map.width, H = map.height;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    const starts = tasks.map(([s]) => [s[0], s[1]] as Cell);
    const goals = tasks.map(([, g]) => [g[0], g[1]] as Cell);
    // 배정은 정의상 단사이고 모든 관련 셀은 통과 가능해야 한다. 위반 입력은 이 문제의
    // 인스턴스가 아니다 — 계획 없음으로 정직하게 보고한다.
    if (new Set(starts.map(ck)).size < starts.length
        || new Set(goals.map(ck)).size < goals.length
        || [...starts, ...goals].some((c) => !free(c))) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    const sim: Sim = {free, A: [...starts], T: goals, U: new Set<string>(), Pi: [[...starts]], expandedNodes: 0};

    // 알고리즘 1: agent를 index 순서로. PUSH가 먼저고, push가 길을 못 열 때만 SWAP,
    // 둘 다 실패하면 정직한 실패다.
    for (let r = 0; r < tasks.length; r++) {
        while (!cellEq(sim.A[r], sim.T[r])) {
            if (!push(sim, r) && !swap(sim, r)) {
                // 정직한 실패(prioritized 갈래의 관행): 경로 없음, 비용 0. 그때까지 센
                // BFS 확장은 그대로 보고한다.
                emit({event: "planning_finished", success: false,
                    metrics: {expanded_nodes: sim.expandedNodes, makespan: 0, sum_of_costs: 0}});
                return events;
            }
        }
        sim.U.add(ck(sim.T[r]));
    }

    // full-horizon 시공간 경로: paths[k][t]는 시각 t에 agent k가 차지하는 셀. U의
    // agent들은 뒤늦은 swap 중에 밀려나고 replay로 집으로 돌아타니 자른 경로는 실체를 왜곡한다.
    const paths: Cell[][] = [];
    let cost = 0;
    for (let k = 0; k < tasks.length; k++) {
        const p = sim.Pi.map((a) => a[k]);
        for (let t = 1; t < p.length; t++) if (!cellEq(p[t], p[t - 1])) cost += 1;
        paths.push(p);
    }
    for (let k = 0; k < paths.length; k++) emit({event: "path_found", path: paths[k], agent: k});
    emit({event: "planning_finished", success: true,
        metrics: {expanded_nodes: sim.expandedNodes, makespan: sim.Pi.length - 1, sum_of_costs: cost}});
    return events;
}
