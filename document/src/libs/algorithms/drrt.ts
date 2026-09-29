import {freePoint, GridMap, segmentFree} from "../grid";
import {movingPairDistance, pointSegmentDistance} from "../geometry";
import {Point, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 dRRT. python/mrmp/sampling/drrt.py의 정확한 미러 — Solovey,
// Salzman & Halperin 2016의 implicit roadmap: robot마다 개별 PRM G_i(연속 free space
// 위, start/goal이 항상 vertex)를 깔고 그 tensor product를 트리 T로 탐색한다. 모든
// float 판정(point-segment 거리, cosine oracle, moving-pair 거리, free/segment predicate,
// 고정 순서 산술)과 MINSTD Lehmer 추첨 열이 Python/C++과 bit-identical — parity 검증의
// 한 축이다 (scripts/check-engine-parity.mjs).

const dist = (a: Point, b: Point): number => {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    return Math.sqrt(dx * dx + dy * dy);
};

// oracle의 각도 ranking: ρ(c,q)와 ρ(c,v) ray 사이 코사인. 고정 산술 순서이고
// argmax(cos) == argmin(angle). 퇴화 ray(양 끝점이 같은 점 — start가 goal과 같을 때만
// 가능)는 코사인 0.0 고정: strict > 아래 승자 없음 = 오름차순 순서가 동률을 겸한다.
const cosine = (q: Point, c: Point, v: Point): number => {
    const dx1 = q[0] - c[0];
    const dy1 = q[1] - c[1];
    const dx2 = v[0] - c[0];
    const dy2 = v[1] - c[1];
    const denom = Math.sqrt(dx1 * dx1 + dy1 * dy1) * Math.sqrt(dx2 * dx2 + dy2 * dy2);
    if (denom === 0.0) return 0.0;
    return (dx1 * dx2 + dy1 * dy2) / denom;
};

export function runDrrt(
    map: GridMap,
    tasks: Array<[Point, Point]>,
    radius: number[],
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    const seed = params.seed as number;
    const samplesPerRobot = params.samples_per_robot as number;
    const kFanout = params.roadmap_k as number;
    const maxRounds = params.max_rounds as number;

    const m = tasks.length;
    const radii = tasks.map((_, i) => radius[i]);
    const starts = tasks.map(([s]) => s);
    const goals = tasks.map(([, g]) => g);

    emit({event: "planning_started", algorithm: "drrt", params, coords: "world", radius});

    const fail = (expanded: number): void => {
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
        });
    };

    // Instance 판정(예산 아님): disc이 obstacle과 겹치는 start/goal은 불가피 instance.
    // pairwise 겹침은 양쪽 끝에서 모두 final — start끼리 겹치면 유효한 초기
    // configuration 자체가 없고, goal끼리 겹치면 두 disc을 동시에 점유하는 최종
    // configuration이 영원히 없다. 남의 goal과 자기 start가 겹치는 건 판정이 아니다 —
    // i가 먼저 떠나면 j가 도착할 수 있다.
    for (let i = 0; i < m; i++) {
        if (!freePoint(map, starts[i], radii[i]) || !freePoint(map, goals[i], radii[i])) {
            fail(0);
            return events;
        }
    }
    for (let i = 0; i < m; i++) {
        for (let j = i + 1; j < m; j++) {
            if (dist(starts[i], starts[j]) < radii[i] + radii[j] ||
                dist(goals[i], goals[j]) < radii[i] + radii[j]) {
                fail(0);
                return events;
            }
        }
    }

    // MINSTD Lehmer — Python/C++ int64와 integer-exact하고 IEEE double에서도 정확하다.
    let s = seed;
    const uniform = (): number => {
        s = (16807 * s) % 2147483647;
        return s / 2147483647.0;
    };
    // world extent — python OccupancyGrid2D.extent()와 같은 산술 순서.
    const xMin = map.origin[0];
    const yMin = map.origin[1];
    const xMax = map.origin[0] + map.width * map.resolution;
    const yMax = map.origin[1] + map.height * map.resolution;

    // --- 개별 roadmap G_i: vertex는 [start, goal] 먼저, 그 다음 rejection sample. ----
    const vertices: Point[][] = [];
    const adjacency: number[][][] = [];
    for (let i = 0; i < m; i++) {
        const vi: Point[] = [starts[i], goals[i]];
        while (vi.length < samplesPerRobot + 2) {
            const sx = xMin + uniform() * (xMax - xMin);
            const sy = yMin + uniform() * (yMax - yMin);
            if (freePoint(map, [sx, sy], radii[i])) vi.push([sx, sy]);
        }
        // k-nearest 후보는 (거리 오름차순, index 오름차순) — pair는 먼저 삽입된 끝점에서
        // 정확히 한 번 평가되므로 (min,max)로 정규화해 중복 평가가 없다.
        const edgeSet: Array<[number, number]> = [];
        const hasPair = (a: number, b: number): boolean =>
            edgeSet.some(([x, y]) => x === a && y === b);
        for (let v = 0; v < vi.length; v++) {
            const cands: Array<[number, number]> = [];
            for (let w = 0; w < vi.length; w++) {
                if (w !== v) cands.push([dist(vi[v], vi[w]), w]);
            }
            cands.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
            for (const [, w] of cands.slice(0, kFanout)) {
                const pair: [number, number] = v < w ? [v, w] : [w, v];
                if (hasPair(pair[0], pair[1])) continue;
                if (segmentFree(map, vi[pair[0]], vi[pair[1]], radii[i])) edgeSet.push(pair);
            }
        }
        // 인접 목록은 정점 index 오름차순 — oracle의 동률 처리가 이 순서를 읽는다.
        const adjI: number[][] = vi.map(() => [] as number[]);
        edgeSet.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
        for (const [a, b] of edgeSet) {
            adjI[a].push(b);
            adjI[b].push(a);
        }
        vertices.push(vi);
        adjacency.push(adjI);
        emit({event: "roadmap_built", agent: i, vertices: vi, edges: edgeSet.map(([a, b]) => [a, b] as [number, number])});
    }

    // joint vertex는 agent별 vertex index의 tuple. root는 전 agent start(index 0),
    // goal tuple은 고정 index가 아니라 각 roadmap vertex 1의 점들이다.
    const root: number[] = new Array<number>(m).fill(0);
    const goalPoints: Point[] = vertices.map((vi) => vi[1]);

    const jointDistance = (a: number[], q: Point[]): number => {
        let total = 0.0;
        for (let i = 0; i < m; i++) {
            const dx = vertices[i][a[i]][0] - q[i][0];
            const dy = vertices[i][a[i]][1] - q[i][1];
            total += dx * dx + dy * dy;
        }
        return Math.sqrt(total);
    };

    const flatten = (state: number[]): number[] => {
        const flat: number[] = [];
        for (let i = 0; i < m; i++) flat.push(vertices[i][state[i]][0], vertices[i][state[i]][1]);
        return flat;
    };

    const states: number[][] = [root];
    const parentsJoint: number[] = [-1];
    const indexOf = new Map<string, number>([[root.join(","), 0]]);
    emit({event: "node_expanded", state: flatten(root)});

    // O_D: robot마다 인접 목록(오름차순)에서 코사인 argmax — strict > 라 동률은 낮은 index.
    // 어느 robot이든 이웃이 없으면 O_D = ∅이고 표본은 무시된다.
    const oracle = (state: number[], q: Point[]): number[] | null => {
        const candidate: number[] = [];
        for (let i = 0; i < m; i++) {
            const cIdx = state[i];
            const nbrs = adjacency[i][cIdx];
            if (nbrs.length === 0) return null;
            const cPoint = vertices[i][cIdx];
            const qPoint = q[i];
            let bestIdx = -1;
            let bestCos = 0.0;
            let first = true;
            for (const w of nbrs) {
                const cos = cosine(qPoint, cPoint, vertices[i][w]);
                if (first || cos > bestCos) {
                    bestCos = cos;
                    bestIdx = w;
                    first = false;
                }
            }
            candidate.push(bestIdx);
        }
        return candidate;
    };

    // tensor edge: 전원 동시에 움직인다 — pair마다 moving-pair 거리가 반지름 합 이상이어야 유효.
    const edgeValid = (prev: number[], nxt: number[]): boolean => {
        for (let i = 0; i < m; i++) {
            for (let j = i + 1; j < m; j++) {
                const d = movingPairDistance(
                    vertices[i][prev[i]], vertices[i][nxt[i]],
                    vertices[j][prev[j]], vertices[j][nxt[j]],
                );
                if (d < radii[i] + radii[j]) return false;
            }
        }
        return true;
    };

    // hop-최단 BFS를 고정 오름차순 이웃 순서로 걷고 parent chain을 goal에서 거슬러
    // 올라가 되감는다 — 반환 경로는 node vertex에서 시작해 goal vertex에서 끝난다.
    const bfsLocal = (startIdx: number, agent: number): Point[] | null => {
        const tree = new Map<number, number>([[startIdx, startIdx]]);
        let frontier: number[] = [startIdx];
        while (!tree.has(1)) {
            const nxtFrontier: number[] = [];
            for (const c of frontier) {
                for (const w of adjacency[agent][c]) {
                    if (!tree.has(w)) {
                        tree.set(w, c);
                        nxtFrontier.push(w);
                    }
                }
            }
            if (nxtFrontier.length === 0) return null;   // goal vertex 도달 불가
            frontier = nxtFrontier;
        }
        const chain: number[] = [];
        let node = 1;
        for (;;) {
            chain.push(node);
            if (tree.get(node) === node) break;
            node = tree.get(node)!;
        }
        chain.reverse();   // parent walk는 goal→start 순서 — 움직임은 반대다.
        return chain.map((idx) => vertices[agent][idx]);
    };

    const pathHitsPoint = (path: Point[], p: Point, radiusSum: number): boolean => {
        for (let t = 0; t < path.length - 1; t++) {
            if (pointSegmentDistance(p, path[t], path[t + 1]) < radiusSum) return true;
        }
        return false;
    };

    // §4.2 우선순위 분해: robot별 π_i + priority DAG. acyclic이면 Kahn 순서(lowest
    // index first)로 실행 — i가 움직일 때 π_i에 닿을 수 있는 parked disc은 이미 떠난 뒤다.
    // cycle이면 이 candidate는 실패하고 다음 candidate이 시도된다.
    const connect = (qState: number[]): Point[][] | null => {
        const pathsLocal: Point[][] = [];
        for (let i = 0; i < m; i++) {
            const pi = bfsLocal(qState[i], i);
            if (pi === null) return null;
            pathsLocal.push(pi);
        }
        // deps[i]: i보다 먼저 움직여야 하는 robot들.
        const deps: Set<number>[] = Array.from({length: m}, () => new Set<number>());
        for (let i = 0; i < m; i++) {
            for (let j = i + 1; j < m; j++) {
                const rSum = radii[i] + radii[j];
                if (pathHitsPoint(pathsLocal[i], vertices[j][qState[j]], rSum)) deps[i].add(j);
                if (pathHitsPoint(pathsLocal[j], vertices[i][qState[i]], rSum)) deps[j].add(i);
                if (pathHitsPoint(pathsLocal[i], goals[j], rSum)) deps[j].add(i);
                if (pathHitsPoint(pathsLocal[j], goals[i], rSum)) deps[i].add(j);
            }
        }
        const order: number[] = [];
        const done = new Set<number>();
        while (order.length < m) {
            let ready = -1;
            for (let r = 0; r < m; r++) {
                if (done.has(r)) continue;
                if ([...deps[r]].every((d) => done.has(d))) { ready = r; break; }
            }
            if (ready === -1) return null;   // cycle — candidate 실패
            order.push(ready);
            done.add(ready);
        }
        const paths: Point[][] = vertices.map((_, a) => [vertices[a][qState[a]]]);
        for (const mover of order) {
            const pi = pathsLocal[mover];
            for (let t = 1; t < pi.length; t++) {
                for (let a = 0; a < m; a++) paths[a].push(a === mover ? pi[t] : paths[a][paths[a].length - 1]);
            }
        }
        return paths;
    };

    // --- main loop (Algorithm 1) -------------------------------------------------
    let connectorPaths: Point[][] | null = null;
    let chainNodes: number[] = [];
    for (let roundI = 1; roundI <= maxRounds && connectorPaths === null; roundI++) {
        // EXPAND: N = 2^round_i 반복 — 파라미터 없는 doubling schedule.
        for (let it = 0; it < 2 ** roundI; it++) {
            const qRand: Point[] = [];
            for (let i = 0; i < m; i++) {
                qRand.push([xMin + uniform() * (xMax - xMin), yMin + uniform() * (yMax - yMin)]);
            }
            let nearIdx = 0;
            let nearD = jointDistance(states[0], qRand);
            for (let idx = 1; idx < states.length; idx++) {
                const d = jointDistance(states[idx], qRand);
                if (d < nearD) { nearD = d; nearIdx = idx; }   // strict < — 최소 삽입 index 승리
            }
            const candidate = oracle(states[nearIdx], qRand);
            if (candidate === null || indexOf.has(candidate.join(","))) continue;
            if (!edgeValid(states[nearIdx], candidate)) continue;
            states.push(candidate);
            parentsJoint.push(nearIdx);
            indexOf.set(candidate.join(","), states.length - 1);
            emit({event: "node_expanded", state: flatten(candidate)});
        }
        // CONNECT TO_TARGET: goal tuple까지 K = round_i candidate(거리, index 순).
        const ranked = states
            .map((state, idx) => [jointDistance(state, goalPoints), idx] as [number, number])
            .sort((a, b) => a[0] - b[0] || a[1] - b[1])
            .slice(0, roundI);
        for (const [, nodeIdx] of ranked) {
            const connected = connect(states[nodeIdx]);
            if (connected === null) continue;
            // RETRIEVE PATH: tree chain root → q(agent별로 이어붙이고)에 connector의
            // 순차 tick을 splice한다(중복 head는 건너뛴다).
            const chain: number[] = [];
            for (let node = nodeIdx; node !== -1; node = parentsJoint[node]) chain.push(node);
            chain.reverse();
            chainNodes = chain;
            connectorPaths = connected;
            break;
        }
    }

    if (connectorPaths === null) {
        // 예산 소진 — "예산 안에서 해를 찾지 못했다"이지 "unsolvable"이 아니다.
        fail(states.length);
        return events;
    }

    // tree phase: 모든 tree edge는 전 robot을 동시에 움직인다(tensor, self-loop 없음).
    const paths: Point[][] = vertices.map((_, a) => [vertices[a][root[a]]]);
    for (const nodeIdx of chainNodes.slice(1)) {
        const state = states[nodeIdx];
        for (let a = 0; a < m; a++) paths[a].push(vertices[a][state[a]]);
    }
    for (let a = 0; a < m; a++) {
        for (let t = 1; t < connectorPaths[a].length; t++) paths[a].push(connectorPaths[a][t]);
    }

    // 각 agent의 마지막 이동 이후를 자르고 unit-cost metric(자기 goal 대기는 비용 0).
    let cost = 0.0;
    const trimmed: Point[][] = [];
    for (let i = 0; i < m; i++) {
        let lastMove = 0;
        for (let t = 1; t < paths[i].length; t++) {
            if (paths[i][t][0] !== paths[i][t - 1][0] || paths[i][t][1] !== paths[i][t - 1][1]) lastMove = t;
        }
        const path = paths[i].slice(0, lastMove + 1);
        trimmed.push(path);
        for (let t = 1; t < path.length; t++) {
            if (!(path[t][0] === path[t - 1][0] && path[t][1] === path[t - 1][1]
                && path[t][0] === goals[i][0] && path[t][1] === goals[i][1])) cost += 1.0;
        }
    }
    let makespan = 0;
    for (const p of trimmed) makespan = Math.max(makespan, p.length - 1);
    for (let i = 0; i < m; i++) emit({event: "path_found", path: trimmed[i], agent: i});
    emit({
        event: "planning_finished", success: true,
        metrics: {expanded_nodes: states.length, makespan, sum_of_costs: cost},
    });
    return events;
}
