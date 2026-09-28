import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 MA-RRT*. python/mrmp/sampling/ma_rrt_star.py의 정확한 미러 —
// MINSTD Lehmer PRNG(16807·s mod 2³¹−1)가 같은 seed에서 byte-단위로 동일한 표본 열을
// 뽑고, GREEDY 조향의 고정 이웃 순서(up/down/left/right/wait)와 첫 strict-min tie-break,
// 삽입 순서가 NEAREST/NEAR scan 순서를 겸하는 것까지 그대로 옮긴다. r_n은 1/64 격자로
// 양자화해 libm log/pow의 ULP 차이를 무력화한다. 시각화용이 아니라 parity 검증의 한 축이다
// (scripts/check-engine-parity.mjs).

// GREEDY가 돌려주는 부분 구간: agent별 셀 목록(두 끝점 모두 포함)과 그 primitive
// 비용 합, 그리고 target에 실제로 닿았는지 여부.
interface Segment {
    cells: Cell[][];
    cost: number;    // primitive 비용 합: 자기 goal에서 대기는 0, 나머지 1
    reached: boolean;
}

const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
const jointEq = (a: Cell[], b: Cell[]): boolean =>
    a.every((c, i) => cellEq(c, b[i]));
// joint 상태의 map key — Python tuple hash 대신 쓰는 문자열 키.
const keyOf = (s: Cell[]): string => s.map((c) => `${c[0]},${c[1]}`).join(";");

// Euclidean 거리 — 정수 차이의 제곱 합이라 sqrt가 correctly-rounded로 같은 비트를 낸다.
const euclid = (a: Cell, b: Cell): number => {
    const dr = a[0] - b[0], dc = a[1] - b[1];
    return Math.sqrt(dr * dr + dc * dc);
};

export function runMaRrtStar(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    const seed = params.seed as number;
    const gamma = params.gamma as number;
    const pGoalProb = params.goal_sampling_probability as number;
    const cMax = params.greedy_cost_budget as number;
    const maxIterations = params.max_iterations as number;
    emit({event: "planning_started", algorithm: "ma_rrt_star", params});

    const goals = tasks.map(([, goal]) => goal);
    const start = tasks.map(([s]) => s);
    const k = tasks.length;

    // t=0에 같은 셀을 공유하는 agent가 있으면 불가피 instance — conflict-free joint
    // 계획은 어떤 방식으로도 그들을 영원히 분리할 수 없다.
    let dupStart = false;
    for (let i = 0; i < k && !dupStart; i++) {
        for (let j = i + 1; j < k; j++) if (cellEq(start[i], start[j])) dupStart = true;
    }
    if (dupStart) {
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0},
        });
        return events;
    }

    // MINSTD Lehmer PRNG — Python/C++의 int64 산술과 bit-identical.
    let s = seed;
    const uniform = (): number => {
        s = (16807 * s) % 2147483647;
        return s / 2147483647.0;
    };

    // motion graph의 vertex 집합 W — row-major 정순서.
    const cells: Cell[] = [];
    for (let r = 0; r < map.height; r++) {
        for (let c = 0; c < map.width; c++) {
            if (!map.occupied[r * map.width + c]) cells.push([r, c]);
        }
    }
    const dim = 2 * k; // joint 상태 공간의 차원 d — agent마다 평면 waypoint 하나
    const mMin = 1.0; // 가장 긴 primitive: 단위 시간당 셀 한 칸

    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < map.height && c[1] >= 0 && c[1] < map.width
        && !map.occupied[c[0] * map.width + c[1]];

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

    // Σ_i Euclidean(x_i, y_i) — paper의 joint 거리(speed 1). agent 순서대로 합쳐서
    // 모든 언어가 동일하게 반올림한다.
    const distJoint = (a: Cell[], b: Cell[]): number => {
        let total = 0;
        for (let i = 0; i < k; i++) total += euclid(a[i], b[i]);
        return total;
    };

    // GREEDY(G_M, s, d) (paper Alg 4): 모든 agent가 동시에 한 칸씩 자기 좌표로
    // 접근한다. 각자 Euclidean 거리가 첫 strict-min인 child(fixed-order tie-break).
    // 셀 중복(vertex conflict)이나 쌍 맞교환(edge conflict)이 나는 스텝은 abort —
    // 구간은 직전 snapshot에서 끝난다. 모든 agent의 argmin이 wait을 고르는 스텝은
    // 상태를 바꾸지 않고 target에도 닿을 수 있으므로 여기서 멈춘다. 반복 비용이
    // c_max를 넘으면 중단하고 reached=false를 돌려준다.
    const greedy = (x: Cell[], target: Cell[]): Segment => {
        const seg: Cell[][] = x.map((c) => [c]);
        let cur = x.slice();
        let cost = 0;
        while (!jointEq(cur, target) && cost <= cMax) {
            const prev = cur;
            const nxt: Cell[] = [];
            for (let i = 0; i < k; i++) {
                let bestChild: Cell | null = null;
                let bestD = Infinity;
                for (const child of neighbors(cur[i])) {
                    const d = euclid(child, target[i]);
                    if (d < bestD) {   // 첫 strict min = fixed-order tie-break
                        bestD = d;
                        bestChild = child;
                    }
                }
                nxt.push(bestChild!);  // wait self-loop는 항상 후보라 non-null
            }
            // 새 스텝에 대한 conflict 검사 (이전 스텝은 추가될 때 이미 검사됐다).
            for (let i = 0; i < k; i++) {
                for (let j = i + 1; j < k; j++) {
                    if (cellEq(nxt[i], nxt[j])
                        || (cellEq(nxt[i], prev[j]) && cellEq(nxt[j], prev[i]))) {
                        return {cells: seg, cost, reached: false};
                    }
                }
            }
            // 모든 agent의 argmin이 wait을 골랐으면 상태는 그대로 — target에도
            // 닿지 못하므로 여기서 종료한다.
            if (nxt.every((c, i) => cellEq(c, prev[i]))) {
                return {cells: seg, cost, reached: false};
            }
            let stepCost = 0;
            for (let i = 0; i < k; i++) {
                // primitive 비용: start = end = 자기 goal이면 0(자기 goal에서 대기는
                // 공짜), 아니면 단위 시간 1.
                if (!(cellEq(nxt[i], prev[i]) && cellEq(prev[i], goals[i]))) stepCost += 1;
                seg[i].push(nxt[i]);
            }
            cur = nxt;
            cost += stepCost;
        }
        return {cells: seg, cost, reached: jointEq(cur, target)};
    };

    // 트리: 삽입 순서로 인덱싱된 병렬 배열 (삽입 순서가 NEAREST tie-break이자
    // NEAR scan 순서다). root = start tuple.
    const states: Cell[][] = [start];
    const parents: number[] = [-1];
    const segments: Cell[][][] = [start.map((c) => [c])];
    const segCosts: number[] = [0];
    const indexOf = new Map<string, number>([[keyOf(start), 0]]);

    let goalIndex: number | null = jointEq(start, goals) ? 0 : null;
    let bestCost = goalIndex !== null ? 0 : Infinity;

    const flat = (state: Cell[]): number[] => state.flatMap((c) => [c[0], c[1]]);
    emit({event: "node_expanded", state: flat(start), cost: 0});

    // chain root→노드까지의 primitive 비용 합. 구간 비용은 생성 후 불변이라
    // (rewiring은 어떤 구간이 chain에 속하는지만 바꾼다) parent chain을 거슬러
    // 더하는 것이 정확하다.
    const chainCost = (idx: number): number => {
        let total = 0;
        let node = idx;
        while (parents[node] !== -1) {
            total += segCosts[node];
            node = parents[node];
        }
        return total;
    };

    for (let iteration = 0; iteration < maxIterations; iteration++) {
        // --- SAMPLE: draw 하나 bias를 정하고, unbiased면 agent 순서로 waypoint 하나씩.
        let sample: Cell[];
        if (uniform() < pGoalProb) {
            sample = goals;
        } else {
            sample = [];
            for (let i = 0; i < k; i++) sample.push(cells[Math.floor(uniform() * cells.length)]);
        }

        // --- EXTEND. NEAREST는 트리 전체, 동률은 최소 삽입 index.
        let nearestIdx = 0;
        let nearestD = distJoint(states[0], sample);
        for (let idx = 1; idx < states.length; idx++) {
            const d = distJoint(states[idx], sample);
            if (d < nearestD) {
                nearestD = d;
                nearestIdx = idx;
            }
        }

        const seg = greedy(states[nearestIdx], sample);
        // new_state가 이미 트리에 있으면 no-op — paper의 set-union 줄이 duplicate를
        // parent edge에 대해 무엇을 의미하는지 말하지 않고, 트리로 남기는 유일한 읽기다.
        const newState = seg.cells.map((c) => c[c.length - 1]);
        if (seg.cells[0].length === 1 || indexOf.has(keyOf(newState))) continue;

        // NEAR 반지름: r_n = max{γ(log n/n)^(1/d), m}. n은 삽입 전 |V|. 1/64 격자
        // 양자화는 libm log/pow의 ULP 차이를 삼킨다.
        const n = states.length;
        const rN = Math.max(Math.floor(gamma * Math.pow(Math.log(n) / n, 1 / dim) * 64.0) / 64.0, mMin);

        // parent 선택: 기본은 x_nearest. X_near를 삽입 순서로 훑으며 strict 개선만 받는다.
        let bestParent = nearestIdx;
        let costNew = chainCost(nearestIdx) + seg.cost;
        let bestSeg = seg.cells;
        let bestSegCost = seg.cost;
        const nearIndices: number[] = [];
        for (let idx = 0; idx < states.length; idx++) {
            if (distJoint(states[idx], newState) <= rN) nearIndices.push(idx);
        }
        for (const idx of nearIndices) {
            const cand = greedy(states[idx], newState);
            if (!cand.reached) continue;   // x' ≠ x_new — candidate parent가 아니다
            const cPrime = chainCost(idx) + cand.cost;
            if (cPrime < costNew) {
                bestParent = idx;
                costNew = cPrime;
                bestSeg = cand.cells;
                bestSegCost = cand.cost;
            }
        }

        // §4.4 informed pruning: chain 비용 + goal까지 metric lower bound가 incumbent를
        // 넘으면 추가하지 않는다(strict >).
        if (costNew + distJoint(newState, goals) > bestCost) continue;

        const newIdx = states.length;
        states.push(newState);
        parents.push(bestParent);
        segments.push(bestSeg);
        segCosts.push(bestSegCost);
        indexOf.set(keyOf(newState), newIdx);
        emit({event: "node_expanded", state: flat(newState), cost: costNew});

        // Rewiring (Alg 2 lines 18–25): near 노드가 새 노드를 통해 더 싸게 도달하면
        // strict 개선일 때만 parent를 옮긴다.
        for (const idx of nearIndices) {
            if (idx === bestParent) continue;
            const rew = greedy(newState, states[idx]);
            if (!rew.reached) continue;
            const oldCost = chainCost(idx);
            if (oldCost > costNew + rew.cost) {
                parents[idx] = newIdx;
                segments[idx] = rew.cells;
                segCosts[idx] = rew.cost;
            }
        }

        // incumbent 비용은 (rewiring 이후의) goal chain을 정확히 다시 계산한다.
        if (goalIndex === null && jointEq(newState, goals)) goalIndex = newIdx;
        if (goalIndex !== null) bestCost = chainCost(goalIndex);
    }

    // --- 결과 ----------------------------------------------------------------
    if (goalIndex === null) {
        // 예산 소진 — MA-RRT*는 probabilistically complete일 뿐, instance의 판정이 아니다.
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: states.length, makespan: 0, sum_of_costs: 0},
        });
        return events;
    }

    // chain root→goal을 agent별로 이어붙인다(중복된 각 구간의 head는 첫 구간만 생략).
    // 각 agent의 마지막 이동 이후는 잘라낸다 — trailing wait은 자기 goal에서의 공짜 대기다.
    const chainIndices: number[] = [];
    for (let node = goalIndex; node !== -1; node = parents[node]) chainIndices.push(node);
    chainIndices.reverse();
    const paths: Cell[][] = Array.from({length: k}, () => []);
    for (const node of chainIndices) {
        const seg = segments[node];
        for (let i = 0; i < k; i++) {
            if (paths[i].length === 0) paths[i].push(...seg[i]);
            else for (let t = 1; t < seg[i].length; t++) paths[i].push(seg[i][t]);
        }
    }
    let makespan = 0;
    const trimmed: Cell[][] = [];
    for (const path of paths) {
        let lastMove = 0;
        for (let t = 1; t < path.length; t++) {
            if (!cellEq(path[t], path[t - 1])) lastMove = t;
        }
        trimmed.push(path.slice(0, lastMove + 1));
        makespan = Math.max(makespan, trimmed[trimmed.length - 1].length - 1);
    }
    const cost = chainCost(goalIndex);
    for (let i = 0; i < k; i++) emit({event: "path_found", path: trimmed[i], agent: i});
    emit({
        event: "planning_finished", success: true,
        metrics: {expanded_nodes: states.length, makespan, sum_of_costs: cost},
    });
    return events;
}
