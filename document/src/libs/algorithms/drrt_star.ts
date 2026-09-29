import {freePoint, GridMap, segmentFree} from "../grid";
import {movingPairDistance} from "../geometry";
import {Point, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 dRRT*. python/mrmp/sampling/drrt_star.py의 정확한 미러 —
// Shome, Solovey, Dobson, Halperin & Bekris 2020: dRRT와 같은 implicit tensor roadmap이되
// 개별 roadmap은 k-nearest 대신 PRM* connection radius r(n) = (1+eta)^2 * sqrt(mu(C_f)
// log n / (2 pi n))로 연결되고(self-loop 포함 — 대기가 graph의 edge다), tree 탐색은
// cost-to-come rewiring + branch-and-bound. oracle은 I_d: goal-biased draw는 robot마다
// argmin-H(goal vertex까지의 최단경로 길이), unbiased draw는 균일 random neighbor. 모든
// float 판정과 MINSTD Lehmer 추첨 열, heapq의 (distance, index) lex pop 순서까지
// Python/C++과 bit-identical — parity 검증의 한 축이다 (scripts/check-engine-parity.mjs).

const dist = (a: Point, b: Point): number => {
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    return Math.sqrt(dx * dx + dy * dy);
};

// (distance, index)를 Python heapq의 tuple 순서와 동일하게 lex로 비교하는 min-heap —
// 거리가 정확히 같으면 낮은 index가 먼저 pop된다. Dijkstra의 pop 순서가 언어 간에
// bit-identical로 일치해야 해서 필요하다(완전한 전순서라 heap 내부 구조와 무관하게 최소 원소는 유일).
type HeapItem = [number, number];
class PairHeap {
    private items: HeapItem[] = [];
    get size(): number { return this.items.length; }
    push(item: HeapItem): void {
        const a = this.items;
        a.push(item);
        let i = a.length - 1;
        while (i > 0) {
            const parent = (i - 1) >> 1;
            if (this.less(a[i], a[parent])) {
                [a[i], a[parent]] = [a[parent], a[i]];
                i = parent;
            } else break;
        }
    }
    pop(): HeapItem {
        const a = this.items;
        const top = a[0];
        const last = a.pop() as HeapItem;
        if (a.length > 0) {
            a[0] = last;
            let i = 0;
            for (;;) {
                const l = 2 * i + 1, r = l + 1;
                let smallest = i;
                if (l < a.length && this.less(a[l], a[smallest])) smallest = l;
                if (r < a.length && this.less(a[r], a[smallest])) smallest = r;
                if (smallest === i) break;
                [a[i], a[smallest]] = [a[smallest], a[i]];
                i = smallest;
            }
        }
        return top;
    }
    private less(a: HeapItem, b: HeapItem): boolean {
        return a[0] !== b[0] ? a[0] < b[0] : a[1] < b[1];
    }
}

export function runDrrtStar(
    map: GridMap,
    tasks: Array<[Point, Point]>,
    radius: number[],
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    const seed = params.seed as number;
    const nSamples = params.samples_per_robot as number;
    const eta = params.eta as number;
    const goalSampleRate = params.goal_sample_rate as number;
    const maxIterations = params.max_iterations as number;

    const m = tasks.length;
    const radii = tasks.map((_, i) => radius[i]);
    const starts = tasks.map(([s]) => s);
    const goals = tasks.map(([, g]) => g);

    emit({event: "planning_started", algorithm: "drrt_star", params, coords: "world", radius});

    const fail = (expanded: number): TraceEvent[] => {
        emit({
            event: "planning_finished", success: false,
            metrics: {expanded_nodes: expanded, makespan: 0, sum_of_costs: 0},
        });
        return events;
    };

    // Instance 판정(예산 아님): obstacle과 겹치는 start/goal disc, start끼리의 겹침,
    // goal끼리의 겹침. 남의 goal과 자기 start의 겹침은 판정이 아니다 — i가 먼저 떠날 수 있다.
    for (let i = 0; i < m; i++) {
        if (!freePoint(map, starts[i], radii[i]) || !freePoint(map, goals[i], radii[i])) return fail(0);
    }
    for (let i = 0; i < m; i++) {
        for (let j = i + 1; j < m; j++) {
            if (dist(starts[i], starts[j]) < radii[i] + radii[j]) return fail(0);
            if (dist(goals[i], goals[j]) < radii[i] + radii[j]) return fail(0);
        }
    }

    // MINSTD Lehmer — Python/C++ int64와 integer-exact하고 IEEE double에서도 정확하다.
    let s = seed;
    const uniform = (): number => {
        s = (16807 * s) % 2147483647;
        return s / 2147483647.0;
    };
    // world extent와 mu(C_f) — python OccupancyGrid2D.extent()/area()와 같은 산술 순서
    // ((count * res) * res).
    const xMin = map.origin[0];
    const yMin = map.origin[1];
    const xMax = map.origin[0] + map.width * map.resolution;
    const yMax = map.origin[1] + map.height * map.resolution;
    let freeCells = 0;
    for (const occupiedCell of map.occupied) if (!occupiedCell) freeCells++;
    const area = (freeCells * map.resolution) * map.resolution;

    // Theorem 1의 radius, d = 2 (zeta_2 = pi), 고정 산술 순서 그대로.
    const nf = nSamples;
    const radiusR = ((1.0 + eta) * (1.0 + eta)) * Math.sqrt((area * Math.log(nf)) / (2 * Math.PI * nf));

    // --- 개별 roadmap G_i: [start, goal]이 먼저, rejection sample이 그다음. edge는
    // dist < r(n)이고 swept-disc 판정은 먼저 삽입된 끝점에서 한 번만 평가된다.
    // 인접 목록은 오름차순 index이고 자기 자신(self-loop)을 포함한다 — 대기가 edge다. ---
    const vertices: Point[][] = [];
    const adjacency: number[][][] = [];
    for (let i = 0; i < m; i++) {
        const vi: Point[] = [starts[i], goals[i]];
        while (vi.length < nSamples + 2) {
            const sx = xMin + uniform() * (xMax - xMin);
            const sy = yMin + uniform() * (yMax - yMin);
            if (freePoint(map, [sx, sy], radii[i])) vi.push([sx, sy]);
        }
        const edgeSet: Array<[number, number]> = [];
        for (let v = 0; v < vi.length; v++) {
            for (let w = v + 1; w < vi.length; w++) {
                if (dist(vi[v], vi[w]) < radiusR && segmentFree(map, vi[v], vi[w], radii[i])) {
                    edgeSet.push([v, w]);
                }
            }
        }
        const adjI: number[][] = [];
        for (let v = 0; v < vi.length; v++) {
            const nbrs: number[] = [];
            for (let w = 0; w < vi.length; w++) {
                const lo = Math.min(v, w), hi = Math.max(v, w);
                if (w === v || edgeSet.some(([a, b]) => a === lo && b === hi)) nbrs.push(w);
            }
            adjI.push(nbrs);
        }
        vertices.push(vi);
        adjacency.push(adjI);
        emit({event: "roadmap_built", agent: i, vertices: vi, edges: edgeSet.map(([a, b]) => [a, b] as [number, number])});
    }

    // H_i(v): G_i 위에서 goal vertex(index 1)까지의 최단경로 길이. heapq와 동일한
    // (distance, index) lex 순서로 pop하고 stale 항목은 버린다. 도달 불가면 +inf —
    // all-inf argmin은 다른 모든 동률과 같이 낮은 index로 해결된다.
    const dijkstraH = (i: number): number[] => {
        const nI = vertices[i].length;
        const distH: number[] = new Array<number>(nI).fill(Infinity);
        distH[1] = 0.0;
        const queue = new PairHeap();
        queue.push([0.0, 1]);
        while (queue.size > 0) {
            const [dU, u] = queue.pop();
            if (dU > distH[u]) continue;   // stale heap 항목
            for (const w of adjacency[i][u]) {
                const wLen = dist(vertices[i][u], vertices[i][w]);
                if (dU + wLen < distH[w]) {
                    distH[w] = dU + wLen;
                    queue.push([distH[w], w]);
                }
            }
        }
        return distH;
    };
    const hTable: number[][] = [];
    for (let i = 0; i < m; i++) hTable.push(dijkstraH(i));

    const root: number[] = new Array<number>(m).fill(0);
    const goalTuple: number[] = new Array<number>(m).fill(1);
    const keyOf = (state: number[]): string => state.join(",");

    const flatten = (state: number[]): number[] => {
        const flat: number[] = [];
        for (let i = 0; i < m; i++) flat.push(vertices[i][state[i]][0], vertices[i][state[i]][1]);
        return flat;
    };

    // concatenated 좌표 위 Euclidean — 제곱 차이는 agent 순서대로 합산, sqrt 하나.
    const jointDistance = (a: number[], q: Point[]): number => {
        let total = 0.0;
        for (let i = 0; i < m; i++) {
            const dx = vertices[i][a[i]][0] - q[i][0];
            const dy = vertices[i][a[i]][1] - q[i][1];
            total += dx * dx + dy * dy;
        }
        return Math.sqrt(total);
    };

    // w(u, v) = sum_i ‖v_i − u_i‖ — agent 오름차순, 고정.
    const edgeWeight = (a: number[], b: number[]): number => {
        let total = 0.0;
        for (let i = 0; i < m; i++) total += dist(vertices[i][a[i]], vertices[i][b[i]]);
        return total;
    };

    // tensor edge 유효성: 움직이는 pair 어디에서도 disc이 strict overlap하면 안 된다
    // (움직이지 않는 robot은 자기 점을 mover의 세그먼트에 대어 검사된다).
    const edgeValid = (a: number[], b: number[]): boolean => {
        for (let i = 0; i < m; i++) {
            for (let j = i + 1; j < m; j++) {
                const d = movingPairDistance(
                    vertices[i][a[i]], vertices[i][b[i]],
                    vertices[j][a[j]], vertices[j][b[j]],
                );
                if (d < radii[i] + radii[j]) return false;
            }
        }
        return true;
    };

    // composite heuristic: robot별 최단경로 길이의 합(cost 함수도 합이다). inf는 전파되고 inf < inf는 false.
    const hOf = (state: number[]): number => {
        let total = 0.0;
        for (let i = 0; i < m; i++) total += hTable[i][state[i]];
        return total;
    };

    const states: number[][] = [root];
    const parents: number[] = [-1];
    const costs: number[] = [0.0];
    const children: number[][] = [[]];
    const indexOf = new Map<string, number>([[keyOf(root), 0]]);
    emit({event: "node_expanded", state: flatten(root)});

    // node를 new_parent 아래로 re-parent하고(strict improvement) moved subtree의
    // 비용을 위에서 아래로 재계산. children 목록은 오름차순을 유지하므로 이 BFS 순서는
    // 결정적이고, 삼각부등식 때문에 upward rewire는 구조적으로 불가능하다(cycle 없음).
    const rewire = (node: number, newParent: number): void => {
        const old = parents[node];
        if (old >= 0) children[old].splice(children[old].indexOf(node), 1);
        parents[node] = newParent;
        const siblings = children[newParent];
        let pos = 0;
        while (pos < siblings.length && siblings[pos] < node) pos++;
        siblings.splice(pos, 0, node);
        costs[node] = costs[newParent] + edgeWeight(states[newParent], states[node]);
        const queue: number[] = [node];
        for (let head = 0; head < queue.length; head++) {
            const x = queue[head];
            for (const ch of children[x]) {
                costs[ch] = costs[x] + edgeWeight(states[x], states[ch]);
                queue.push(ch);
            }
        }
    };

    let incumbentCost = Infinity;
    let bestPaths: Point[][] | null = null;
    let vLast = 0;   // Algorithm 6 line 1: V_last ← S (greedy first). -1은 ∅.

    for (let iteration = 0; iteration < maxIterations; iteration++) {
        let qRand: Point[];
        let nearIdx: number;
        if (vLast < 0) {
            // exploration: bias draw 하나가 결정하고, unbiased면 agent 순서 x-then-y 좌표를
            // 그리고 각 robot이 자기 uniform neighbor pick을 뽑는다. biased draw는 goal tuple 그 자체다.
            const uBias = uniform();
            if (uBias < goalSampleRate) {
                qRand = goals.slice();
            } else {
                qRand = [];
                for (let i = 0; i < m; i++) {
                    const sx = xMin + uniform() * (xMax - xMin);
                    const sy = yMin + uniform() * (yMax - yMin);
                    qRand.push([sx, sy]);
                }
            }
            nearIdx = 0;
            let nearD = jointDistance(states[0], qRand);
            for (let idx = 1; idx < states.length; idx++) {
                const d = jointDistance(states[idx], qRand);
                if (d < nearD) { nearD = d; nearIdx = idx; }   // strict < — 최소 삽입이 동률 승리
            }
        } else {
            // greedy child propagation: sample은 goal tuple이고 V_near은 V_last 자체.
            qRand = goals.slice();
            nearIdx = vLast;
        }

        // I_d (Alg. 8), robot 오름차순: goal과 정확히 같으면 argmin H로 전환(strict <라 동률은
        // 낮은 index, all-inf neighbourhood도 lowest index로 해결된다). 아니면 균일 random neighbor.
        const vNew: number[] = [];
        for (let i = 0; i < m; i++) {
            const adj = adjacency[i][states[nearIdx][i]];
            if (qRand[i][0] === goals[i][0] && qRand[i][1] === goals[i][1]) {
                let bestIdx = adj[0];
                let bestH = hTable[i][adj[0]];
                for (let w = 1; w < adj.length; w++) {
                    const h = hTable[i][adj[w]];
                    if (h < bestH) { bestH = h; bestIdx = adj[w]; }
                }
                vNew.push(bestIdx);
            } else {
                const pick = Math.floor(uniform() * adj.length);   // u ∈ (0,1) → 유효 index
                vNew.push(adj[pick]);
            }
        }

        // N = Adj(V_new, G_hat) ∩ T — tree node를 삽입 순서대로 훑는다. 인접은 robot별이고
        // 반사적이다(자기 index에 self-loop).
        const cand: number[] = [];
        for (let uIdx = 0; uIdx < states.length; uIdx++) {
            let adjacent = true;
            for (let i = 0; i < m && adjacent; i++) {
                if (!adjacency[i][states[uIdx][i]].includes(vNew[i])) adjacent = false;
            }
            if (adjacent) cand.push(uIdx);
        }

        // V_best = 유효한 candidate 중 c(U) + w(U, V_new)의 argmin. strict <라 동률은 먼저
        // 삽입된 candidate이 이긴다. 유효한 candidate이 없으면 expansion은 거부된다.
        let bestParent = -1;
        let bestCost = Infinity;
        for (const uIdx of cand) {
            const uState = states[uIdx];
            if (!edgeValid(uState, vNew)) continue;
            const cCand = costs[uIdx] + edgeWeight(uState, vNew);
            if (cCand < bestCost) { bestCost = cCand; bestParent = uIdx; }   // strict < — 동률은 최소 index
        }
        if (bestParent < 0) { vLast = -1; continue; }
        if (bestCost > incumbentCost) { vLast = -1; continue; }   // branch-and-bound (strict)

        const inserted = !indexOf.has(keyOf(vNew));
        let idxNew: number;
        if (inserted) {
            idxNew = states.length;
            states.push(vNew);
            parents.push(bestParent);
            costs.push(bestCost);
            children.push([]);
            children[bestParent].push(idxNew);   // 오름차순: idxNew는 가장 큰 index
            indexOf.set(keyOf(vNew), idxNew);
            emit({event: "node_expanded", state: flatten(vNew)});
        } else {
            idxNew = indexOf.get(keyOf(vNew)) as number;
            if (bestCost < costs[idxNew]) rewire(idxNew, bestParent);
        }

        // rewire pass: V_new가 이웃한 tree node들이 strict하게 더 싸지면 V_new 아래로
        // re-parent된다. upward는 삼각부등식 때문에 불가능하다(cycle 없음).
        for (const uIdx of cand) {
            if (uIdx === idxNew) continue;
            const uState = states[uIdx];
            if (edgeValid(vNew, uState)) {
                const w2 = edgeWeight(vNew, uState);
                if (costs[idxNew] + w2 < costs[uIdx]) rewire(uIdx, idxNew);
            }
        }

        // child promotion: 이번 iteration에 GENERATED된 node가 parent보다 H를 개선하면
        // 다음 V_last가 된다. 이미 있던 state의 재확장은 아무것도 생성하지 않았으므로
        // 항상 ∅로 리셋 — goal tuple이 T에 들어간 뒤에도 exploration이 이어지는 이유다.
        if (inserted && hOf(vNew) < hOf(states[bestParent])) {
            vLast = idxNew;
        } else {
            vLast = -1;
        }

        // Connect to Target: goal tuple이 tree node이거나 해가 아직 없는 것이다. cost(π)는 c(goal node).
        const gIdx = indexOf.get(keyOf(goalTuple));
        if (gIdx !== undefined && costs[gIdx] < incumbentCost) {
            const chain: number[] = [];
            let node = gIdx;
            while (node !== -1) {
                chain.push(node);
                node = parents[node];
            }
            chain.reverse();
            // chain은 NODE index를 root → goal node 순으로 담는다. agent i의 웨이포인트는
            // 그 node에서 index tuple이 고르는 vertex다.
            const rawPaths: Point[][] = [];
            for (let i = 0; i < m; i++) rawPaths.push([]);
            for (const nodeIdx of chain) {
                const state = states[nodeIdx];
                for (let i = 0; i < m; i++) rawPaths[i].push(vertices[i][state[i]]);
            }
            // 각 agent의 마지막 이동 이후를 자른다(트레일링 대기는 arc length 0).
            const trimmed: Point[][] = [];
            for (const points of rawPaths) {
                let lastMove = 0;
                for (let t = 1; t < points.length; t++) {
                    if (points[t][0] !== points[t - 1][0] || points[t][1] !== points[t - 1][1]) lastMove = t;
                }
                trimmed.push(points.slice(0, lastMove + 1));
            }
            incumbentCost = costs[gIdx];
            bestPaths = trimmed;
        }
    }

    if (bestPaths === null) {
        // 예산 소진 — "예산 안에서 해를 찾지 못했다"이지 "unsolvable"이 아니다.
        return fail(states.length);
    }

    // geometric metric(논문의 cost 함수): agent별 arc length의 합과 max. traced path에서
    // 고정 순서로 재계산한다(search를 굴린 c(goal node)와 같은 양을 edge-first로 묶은 것 —
    // float associativity가 lower bits를 다르게 만들고, 둘 다 고정이고 둘 다 더 참이지 않는다).
    const lengths: number[] = [];
    for (const path of bestPaths) {
        let length = 0.0;
        for (let t = 1; t < path.length; t++) length += dist(path[t - 1], path[t]);
        lengths.push(length);
    }
    let sumOfCosts = 0.0;
    let makespan = 0.0;
    for (let i = 0; i < m; i++) {
        sumOfCosts += lengths[i];
        if (lengths[i] > makespan) makespan = lengths[i];
    }
    for (let i = 0; i < m; i++) emit({event: "path_found", path: bestPaths[i], agent: i});
    emit({
        event: "planning_finished", success: true,
        metrics: {expanded_nodes: states.length, makespan, sum_of_costs: sumOfCosts},
    });
    return events;
}
