import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";
import {cbsPlan} from "./cbs";

// 브라우저 라이브 데모용 MAPF-POST. python/mrmp/kinodynamic/mapf_post.py의 정확한 미러 —
// 계획을 새로 만들지 않는다. search 갈래의 CBS(cbsPlan을 emit 없이 조용히 호출)가 이산
// 계획을 풀고, 그 위에 Temporal Plan Graph를 STN으로 바꿔 최대 완화를 푼다: TPG 꼭짓점은
// 이벤트 하나(agent j가 셀 s에 t에 진입 — 대기는 제거)이고, type-1 간선은 agent 자신의
// 체인(단위 간선을 delta / 1−2·delta / delta로 쪼개고 안전 마커를 심는다), type-2 간선은
// 공유 셀을 먼저 지나간 쪽의 마커에서 나중에 지나갈 쪽의 마커로의 precedence. STN 제약은
// [length/v_k, inf] (type-2는 [0, inf]), 출발 이벤트는 [0, 0]으로 고정. DAG라 STN은 항상
// 일관되고 최대 완화 Bellman-Ford의 고정점이 곧 가장 빠른 스케줄이다. expanded_nodes는
// 라벨을 실제로 개선한 완화 횟수(self metric) — 기반 탐색의 확장이 아니다.
// delta와 모든 데모 vmax가 이진 유리수라 산술이 정확하고, 같은 순서로 계산된 float64는
// Python/C++과 비트 단위로 동일하다 (parity 검증의 근거).

interface StnEdge {
    // 꼬리 꼭짓점의 시각에 lb를 더한 값은 머리 꼭짓점의 시각 이하 — 단순 시간 제약.
    src: number;
    dst: number;
    lb: number;
}

const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];

export function runMapfPost(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    vmax: number[],
    params: Record<string, unknown>,
): TraceEvent[] {
    const delta = params.delta as number;
    // config 범위는 이미 [0.0, 0.5]로 클램프하지만 논문의 조건은 엄격한 양수다 —
    // delta = 0은 모든 안전 마커를 셀 중심과 겹치게 해 안전 거리 보장을 무효화한다.
    if (!(delta > 0)) throw new Error(`mapf_post: delta must be > 0 (got ${delta})`);

    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    // Python의 demo가 plan() 전에 싣는 planning_started — vmax의 존재가 timed 선언이다.
    emit({event: "planning_started", algorithm: "mapf_post", params, vmax});

    // 기반 이산 계획: CBS는 조용히 돈다 (emit 없음) — 이 갈래의 trace는 스케줄만 싣고
    // 지표도 자기 것(완화 횟수)을 센다. 예산 소진/트리 고갈은 "후처리할 계획 없음"으로
    // 그대로 정직하게 상속된다.
    const base = cbsPlan(map, tasks, params.max_ct_expansions as number);
    if (!base.success) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    // Route 추출: 대기를 버린다(같은 셀 연속은 첫 개만 유지). 남은 이벤트는 자기 이산
    // 스텝 번호를 지킨다 — 그 스텝은 type-2 간선의 순서를 정하는 데만 쓰이고 시각이 아니다.
    const routes: Cell[][] = [];
    const steps: number[][] = [];
    for (const path of base.paths) {
        const route: Cell[] = [];
        const kept: number[] = [];
        for (let t = 0; t < path.length; t++) {
            if (route.length === 0 || !cellEq(path[t], route[route.length - 1])) {
                route.push([path[t][0], path[t][1]]);
                kept.push(t);
            }
        }
        routes.push(route);
        steps.push(kept);
    }

    // 증강 TPG를 평면 간선 목록으로 — 꼭짓점 id는 구성 순서로 고정된다: agent index
    // 순서, 각 agent 안에서는 이벤트(스텝) 순서, 그 다음 마커를 체인 순서대로.
    const n = routes.length;
    const eventIds: number[][] = [];
    let nextId = 0;
    for (const route of routes) {
        const ids: number[] = [];
        for (let i = 0; i < route.length; i++) ids.push(nextId++);
        eventIds.push(ids);
    }
    const edges: StnEdge[] = [];
    const m1Of: number[][] = [];
    const m2Of: number[][] = [];
    for (let j = 0; j < n; j++) {
        const vj = vmax[j];
        const ids = eventIds[j];
        const m1j: number[] = [];
        const m2j: number[] = [];
        // type-1: 단위 간선마다 안전 마커와 함께 세 조각. parked agent(단일 셀)는
        // 이벤트 하나만 내고 나가는 간선이 없으니 마커도 없다.
        for (let i = 0; i < ids.length - 1; i++) {
            const m1 = nextId, m2 = nextId + 1;
            nextId += 2;
            m1j.push(m1);
            m2j.push(m2);
            edges.push({src: ids[i], dst: m1, lb: delta / vj});
            edges.push({src: m1, dst: m2, lb: (1.0 - 2.0 * delta) / vj});
            edges.push({src: m2, dst: ids[i + 1], lb: delta / vj});
        }
        m1Of.push(m1j);
        m2Of.push(m2j);
    }
    // type-2 precedence: agent j의 각 이벤트마다 다른 agent의 RAW 경로를 그 스텝보다
    // 나중에 스캔 — 첫 번째 나중 방문이 이긴다(나중에 것은 전이성으로 함의된다). 마지막
    // 이벤트는 parked이므로 나가는 간선이 없어 type-2의 출발지가 될 수 없다.
    for (let j = 0; j < n; j++) {
        for (let i = 0; i < eventIds[j].length - 1; i++) {
            const cell = routes[j][i];
            const step = steps[j][i];
            for (let k = 0; k < n; k++) {
                if (k === j) continue;
                let found = -1;
                for (let t = step + 1; t < base.paths[k].length; t++) {
                    if (cellEq(base.paths[k][t], cell)) {
                        found = t;
                        break;
                    }
                }
                if (found < 0) continue;
                const idxK = steps[k].indexOf(found);
                // 나중 방문은 스텝 >= 1에서 일어나므로 들어오는 간선(과 그 m2 마커)은 존재한다.
                edges.push({src: m1Of[j][i], dst: m2Of[k][idxK - 1], lb: 0.0});
            }
        }
    }
    const labels = new Array<number>(nextId).fill(-Infinity);
    for (let j = 0; j < n; j++) labels[eventIds[j][0]] = 0.0; // X_S → 모든 source 이벤트 [0, 0]

    const expanded = relax(labels, edges);
    const timesByAgent = eventIds.map((ids) => ids.map((id) => labels[id]));
    let makespan = -Infinity;
    let cost = 0;
    for (const times of timesByAgent) {
        const last = times[times.length - 1];
        if (last > makespan) makespan = last;
        cost += last;
    }

    for (let k = 0; k < routes.length; k++) {
        emit({event: "schedule_found", agent: k, cells: routes[k], times: timesByAgent[k]});
    }
    emit({event: "planning_finished", success: true,
        metrics: {expanded_nodes: expanded, makespan, sum_of_costs: cost}});
    return events;
}

// 제약 간선에 대한 최대 완화 Bellman-Ford — DAG의 유일한 고정점. 모든 패스는 간선을
// 구성 순서 그대로 읽고, 개선이 없는 패스가 루프를 멈춘다. -Infinity + lb는 -Infinity라
// 완화되지 않은 꼭짓점에서 나가는 간선은 결코 발화하지 않는다. 반환값은 라벨을 실제로
// 개선한 완화 횟수 — 이 알고리즘의 expanded_nodes.
function relax(labels: number[], edges: StnEdge[]): number {
    let expanded = 0;
    let changed = true;
    while (changed) {
        changed = false;
        for (const e of edges) {
            const cand = labels[e.src] + e.lb;
            if (cand > labels[e.dst]) {
                labels[e.dst] = cand;
                expanded += 1;
                changed = true;
            }
        }
    }
    return expanded;
}
