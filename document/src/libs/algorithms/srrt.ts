import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 sRRT. python/mrmp/sampling/srrt.py의 정확한 미러 — MINSTD
// Lehmer PRNG(16807·s mod 2³¹−1)가 같은 seed에서 byte-단위로 동일한 표본 열을 뽑고,
// BFS 정책 트리의 고정 이웃 순서(up/down/left/right/wait), 첫 strict-min tie-break,
// 삽입 순서가 NEAREST tie-break를 겸하는 것까지 그대로 옮긴다. 이 알고리즘의 모든
// 결정은 정수 산술(Manhattan 합, 셀 동등) — float는 rng 추첨 외에는 개입하지 않아
// 언어 간 parity가 구조적으로 정확하다. 시각화용이 아니라 parity 검증의 한 축이다
// (scripts/check-engine-parity.mjs).

const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
const jointEq = (a: Cell[], b: Cell[]): boolean =>
    a.every((c, i) => cellEq(c, b[i]));
// joint 상태의 map key — Python tuple hash 대신 쓰는 문자열 키.
const keyOf = (s: Cell[]): string => s.map((c) => `${c[0]},${c[1]}`).join(";");

// Manhattan 거리 — 정수 산술이라 모든 언어에서 동일하다.
const manhattan = (a: Cell, b: Cell): number =>
    Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);

export function runSrrt(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    const seed = params.seed as number;
    const pGoalProb = params.goal_sampling_probability as number;
    const maxIterations = params.max_iterations as number;
    emit({event: "planning_started", algorithm: "srrt", params});

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

    // --- 개별 정책 (§II) — φᵢ = goal에서 뒤로 자란 BFS 트리. parent가 곧 다음
    // 스텝이고, goal의 parent는 자기 자신(자기 goal에서 대기). start가 트리에 없으면
    // goal이 start에서 도달 불가 — instance 판정.
    const policies: Map<string, Cell>[] = [];
    for (let i = 0; i < k; i++) {
        const tree = new Map<string, Cell>([[keyOf([goals[i]]), goals[i]]]);
        let frontier: Cell[] = [goals[i]];
        while (frontier.length > 0) {
            const nxtFrontier: Cell[] = [];
            for (const c of frontier) {
                for (const n of neighbors(c)) {
                    if (!tree.has(keyOf([n]))) {   // 고정 순서 ⇒ 첫 parent가 승리 (BFS tie-break)
                        tree.set(keyOf([n]), c);
                        nxtFrontier.push(n);
                    }
                }
            }
            frontier = nxtFrontier;
        }
        if (!tree.has(keyOf([start[i]]))) {
            emit({
                event: "planning_finished", success: false,
                metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0},
            });
            return events;
        }
        policies.push(tree);
    }

    // Σ_i Manhattan — NEAREST가 쓰는 joint 거리. agent 순서대로 합친다.
    const distJoint = (a: Cell[], b: Cell[]): number => {
        let total = 0;
        for (let i = 0; i < k; i++) total += manhattan(a[i], b[i]);
        return total;
    };

    // --- joint 트리 T_f — 병렬 배열은 삽입 순서로 인덱싱(삽입 순서가 NEAREST
    // tie-break). root = start tuple, collision set ∅.
    const states: Cell[][] = [start];
    const parentsJoint: number[] = [-1];
    const colsets: Set<number>[] = [new Set<number>()];
    const segments: Cell[][][] = [start.map((c) => [c])];
    const indexOf = new Map<string, number>([[keyOf(start), 0]]);

    let goalIndex: number | null = jointEq(start, goals) ? 0 : null;

    const flat = (state: Cell[]): number[] => state.flatMap((c) => [c[0], c[1]]);
    emit({event: "node_expanded", state: flat(start)});

    for (let iteration = 0; iteration < maxIterations; iteration++) {
        if (goalIndex !== null) break;   // goal tuple이 트리에 들어왔다 — 성장 종료.

        // --- SAMPLE: 확률 p_goal에 goal tuple 자체(repo의 informed-sampling
        // 관례), 아니면 agent 순서로 vertex 집합에서 균일 한 칸씩.
        let sample: Cell[];
        if (uniform() < pGoalProb) {
            sample = goals;
        } else {
            sample = [];
            for (let i = 0; i < k; i++) sample.push(cells[Math.floor(uniform() * cells.length)]);
        }

        // --- NEAREST 트리 전체, 동률은 최소 삽입 index.
        let nearestIdx = 0;
        let nearestD = distJoint(states[0], sample);
        for (let idx = 1; idx < states.length; idx++) {
            const d = distJoint(states[idx], sample);
            if (d < nearestD) {
                nearestD = d;
                nearestIdx = idx;
            }
        }

        // --- LOCAL PLANNER (식 (2) + lockstep walk). pinned robot은 정책 스텝
        // 정확히 한 칸, free robot은 sampled cell까지 greedy step(첫 strict-min),
        // 도착하면 대기. Manhattan local minimum에 막힌 robot은 영원히 못 닿고
        // expansion은 정직하게 실패한다.
        let colset = new Set(colsets[nearestIdx]);
        const targets: Cell[] = [];
        for (let i = 0; i < k; i++) {
            // lookup은 항상 hitts: walk이 도달하는 모든 셀은 start와 같은 free
            // component에 있고 BFS 트리는 그 전체를 덮는다.
            targets.push(colset.has(i) ? sample[i] : policies[i].get(keyOf([states[nearestIdx][i]]))!);
        }
        let cur = states[nearestIdx];
        const seg: Cell[][] = cur.map((c) => [c]);   // agent별 셀, 두 끝점 포함
        let aborted = false;
        while (!jointEq(cur, targets)) {
            const prev = cur;
            const nxtList: Cell[] = [];
            let stuck = false;
            for (let i = 0; i < k && !stuck; i++) {
                if (cellEq(cur[i], targets[i])) {
                    nxtList.push(cur[i]);   // 도착 — lockstep 스텝은 대기
                    continue;
                }
                let bestChild: Cell | null = null;
                let bestD = Infinity;
                for (const child of neighbors(prev[i])) {
                    const d = manhattan(child, targets[i]);
                    if (d < bestD) {   // 첫 strict min = fixed-order tie-break
                        bestD = d;
                        bestChild = child;
                    }
                }
                if (bestChild !== null && cellEq(bestChild, prev[i])) {
                    stuck = true;   // local minimum: 이 target은 여기서 도달 불가
                    break;
                }
                nxtList.push(bestChild!);   // wait self-loop은 항상 후보라 non-null
            }
            if (stuck) {
                aborted = true;
                break;
            }
            const nxt = nxtList;
            // 이 스텝의 conflict 검사(vertex + swap), agent 쌍별.
            const involved = new Set<number>();
            for (let i = 0; i < k; i++) {
                for (let j = i + 1; j < k; j++) {
                    if (cellEq(nxt[i], nxt[j])
                        || (cellEq(nxt[i], prev[j]) && cellEq(prev[i], nxt[j]))) {
                        involved.add(i);
                        involved.add(j);
                    }
                }
            }
            if (involved.size > 0) {
                // 어떤 conflict든 local path의 expansion을 죽인다(paper: 노드는
                // "conflict가 없으면"만 추가된다). INFORMATIVE(관련 robot 중 아직
                // C_r 밖이 있는) 것만 set을 키운다: 관련 robot 전부 C_r에 합류하고
                // ancestor가 이미 자기를 목록에 올릴 때까지 parent chain을 거슬러
                // back-propagate한다. 다음 sample이 확대된 set으로 재-expansion한다.
                // 이미 free인 robot끼리의 conflict는 정보가 없다 — 아무것도 갱신되지
                // 않고 expansion만 실패한다.
                let informative = false;
                for (const robot of involved) if (!colset.has(robot)) informative = true;
                if (informative) {
                    const newSet = new Set(colset);
                    for (const robot of involved) newSet.add(robot);
                    colsets[nearestIdx] = newSet;
                    colset = newSet;
                    let node = parentsJoint[nearestIdx];
                    while (node !== -1) {
                        let coversAll = true;
                        for (const robot of involved) {
                            if (!colsets[node].has(robot)) coversAll = false;
                        }
                        if (coversAll) break;
                        const updated = new Set(colsets[node]);
                        for (const robot of involved) updated.add(robot);
                        colsets[node] = updated;
                        node = parentsJoint[node];
                    }
                }
                aborted = true;
                break;
            }
            for (let i = 0; i < k; i++) seg[i].push(nxt[i]);
            cur = nxt;
        }
        if (aborted) continue;

        const newState = cur;
        if (indexOf.has(keyOf(newState))) continue;   // duplicate vertex — no-op(트리로 남긴다)
        const newIdx = states.length;
        states.push(newState);
        parentsJoint.push(nearestIdx);
        colsets.push(colset);   // nearest 노드의 갱신 후 set 그대로
        segments.push(seg);
        indexOf.set(keyOf(newState), newIdx);
        emit({event: "node_expanded", state: flat(newState)});

        // goal check — joint goal tuple이 정확히 vertex가 됐다. 루프 상단이
        // goalIndex를 보자마자 빠져나가므로 여기서 즉시 break해도 동일하다.
        if (jointEq(newState, goals)) {
            goalIndex = newIdx;
            break;
        }
    }

    // --- 결과 ----------------------------------------------------------------
    if (goalIndex === null) {
        // 예산 소진 — sRRT의 completeness는 미증명(논문이 직접 말한다). instance 판정이 아니다.
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: states.length, makespan: 0, sum_of_costs: 0},
        });
        return events;
    }

    // chain root→goal을 agent별로 이어붙인다(중복된 각 구간의 head는 첫 구간만 생략).
    // 각 agent의 마지막 이동 이후는 잘라낸다 — trailing wait은 자기 goal에서의 대기다.
    const chainIndices: number[] = [];
    for (let node = goalIndex; node !== -1; node = parentsJoint[node]) chainIndices.push(node);
    chainIndices.reverse();
    const paths: Cell[][] = Array.from({length: k}, () => []);
    for (const node of chainIndices) {
        const seg = segments[node];
        for (let i = 0; i < k; i++) {
            if (paths[i].length === 0) paths[i].push(...seg[i]);
            else for (let t = 1; t < seg[i].length; t++) paths[i].push(seg[i][t]);
        }
    }
    let cost = 0;
    const trimmed: Cell[][] = [];
    for (let i = 0; i < k; i++) {
        let lastMove = 0;
        for (let t = 1; t < paths[i].length; t++) {
            if (!cellEq(paths[i][t], paths[i][t - 1])) lastMove = t;
        }
        trimmed.push(paths[i].slice(0, lastMove + 1));
        // agent별 비용: 자기 goal에서 대기를 제외하고 모든 action이 시간 스텝 1
        // (모든 planner가 보고하는 같은 metric).
        for (let t = 1; t < trimmed[i].length; t++) {
            if (!(cellEq(trimmed[i][t], trimmed[i][t - 1]) && cellEq(trimmed[i][t], goals[i]))) cost += 1;
        }
    }
    let makespan = 0;
    for (const p of trimmed) makespan = Math.max(makespan, p.length - 1);
    for (let i = 0; i < k; i++) emit({event: "path_found", path: trimmed[i], agent: i});
    emit({
        event: "planning_finished", success: true,
        metrics: {expanded_nodes: states.length, makespan, sum_of_costs: cost},
    });
    return events;
}
