import {Cell, Point, TraceEvent} from "./types";

// trace 이벤트 열을 재생 가능한 타임라인으로 접는다. step은 이벤트 index와 같고,
// 렌더러는 "step 이하"의 이벤트만 그린다 — 스크러버로 임의 시점 탐색이 가능하다.
// MRMP는 joint state 확장을 agent별로 풀어 expanded에 담고(색 구분), path_found의
// space-time 경로는 실행 재생(execStep)에 쓰인다. discrete trace는 상태가 셀,
// continuous trace(coords=world)는 world point이고 roadmap_built를 추가로 실는다.
// timed(kinodynamic) trace는 schedule_found로 route+도착 시각을 싣고 vmax의 존재가
// timed를 선언한다 — 실행 재생은 소수 τ에서 uniform velocity model을 따른다.

// agent 인덱스별 hue (순환) — tools/viz/replay.py의 팔레트와 동일해야 GIF와
// 브라우저 재생이 같은 실행으로 읽힌다.
export const AGENT_COLORS = ["#0d9488", "#c2179b", "#2563eb", "#ca8a04", "#7c3aed", "#e5484d"];

// conflict 마커 색 — 모든 agent hue와 구분되는 빨강 (replay.py와 동일).
export const CONFLICT_COLOR = "#dc2626";

export interface MultiTimeline {
    steps: number;                                       // 마지막 step (= 이벤트 수)
    algorithm: string;
    // 상태 페어의 해석: cell(기본)이면 [row, col], world면 미터 단위 점.
    coords: "cell" | "world";
    // continuous trace의 disc 반지름(미터) — 실행 재생 디스크 크기가 여기서 온다.
    radius?: number[];
    // timed(kinodynamic) trace — planning_started의 vmax 존재가 선언한다. 실행 재생은
    // schedules를 uniform velocity model로 재생하고 makespan은 이산 스텝이 아니라 가장
    // 늦은 도착 시각(소수 가능)이다.
    timed: boolean;
    // agent별 속도 한계(칸/시간, agent index 순서) — timed 실행 재생의 속도다.
    vmax?: number[];
    // joint state 확장은 agent별로 풀어 담는다 — 좌표와 그 agent 인덱스.
    expanded: Array<{step: number; point: Point; agent: number}>;
    // agent별 space-time 경로 (path[t] = 시각 t의 상태). step = path_found 이벤트 순번.
    paths: Array<{step: number; agent: number; path: Point[]}>;
    // agent별 timed route(wait 제거 셀)와 각 retained 위치의 도착 시각. step =
    // schedule_found 이벤트 순번.
    schedules: Array<{step: number; agent: number; cells: Cell[]; times: number[]}>;
    // continuous(dRRT 계열)만: agent별 개별 roadmap(정점 삽입 순서 + 인덱스 쌍 간선).
    roadmaps: Array<{step: number; agent: number; vertices: Point[]; edges: Array<[number, number]>}>;
    conflicts: Array<{step: number; cells: Cell[]; agents: [number, number]; t: number}>;
    constraints: Array<{step: number; cell: Cell; to?: Cell; agent: number;
        kind: "vertex" | "edge"; t: number}>;
    makespan: number;                                    // discrete는 마지막 도착 스텝, timed는 가장 늦은 도착 시각
    params?: Record<string, unknown>;
    metrics?: Record<string, number>;
    success?: boolean;
}

const asPair = (state?: number[]): Point | null =>
    state && state.length >= 2 ? [state[0], state[1]] : null

export function buildTimeline(events: TraceEvent[]): MultiTimeline {
    const timeline: MultiTimeline = {
        steps: events.length,
        algorithm: "",
        coords: "cell",
        timed: false,
        expanded: [],
        paths: [],
        schedules: [],
        roadmaps: [],
        conflicts: [],
        constraints: [],
        makespan: 0,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "planning_started":
                timeline.algorithm = ev.algorithm ?? ""
                timeline.params = ev.params
                // planning_started는 항상 첫 이벤트 — 좌표 해석·반지름·timed 선언은 여기서 확정된다.
                if (ev.coords) timeline.coords = ev.coords
                if (ev.radius) timeline.radius = ev.radius
                if (ev.vmax) {
                    timeline.timed = true
                    timeline.vmax = ev.vmax
                }
                break
            case "roadmap_built": {
                if (ev.agent === undefined || !ev.vertices) break
                timeline.roadmaps.push({
                    step, agent: ev.agent, vertices: ev.vertices, edges: ev.edges ?? [],
                })
                break
            }
            case "node_expanded": {
                const state = ev.state ?? []
                if (ev.agent !== undefined) {
                    // 개별 탐색(prioritized / CBS low level): 페어 하나.
                    const point = asPair(state)
                    if (point) timeline.expanded.push({step, point, agent: ev.agent})
                } else {
                    // joint state 확장: 평면화된 상태에서 agent쌍마다 풀어 낸다.
                    for (let k = 0; k + 1 < state.length; k += 2) {
                        timeline.expanded.push({step, point: [state[k], state[k + 1]], agent: k})
                    }
                }
                break
            }
            case "path_found": {
                const path = (ev.path ?? []) as Point[]
                if (ev.agent === undefined) break
                timeline.paths.push({step, agent: ev.agent, path})
                timeline.makespan = Math.max(timeline.makespan, path.length - 1)
                break
            }
            case "schedule_found": {
                if (ev.agent === undefined || !ev.cells || !ev.times) break
                const cells = ev.cells.map((c) => [c[0], c[1]] as Cell)
                timeline.schedules.push({step, agent: ev.agent, cells, times: ev.times})
                for (const t of ev.times) timeline.makespan = Math.max(timeline.makespan, t)
                break
            }
            case "conflict_found": {
                const cell = asPair(ev.cell)
                if (!cell || !ev.agents) break
                const cells: Cell[] = [cell]
                const to = asPair(ev.to)
                if (to) cells.push(to)
                timeline.conflicts.push({step, cells, agents: ev.agents, t: ev.t ?? 0})
                break
            }
            case "constraint_added": {
                const cell = asPair(ev.cell)
                if (!cell || ev.agent === undefined) break
                timeline.constraints.push({
                    step, cell, to: asPair(ev.to) ?? undefined,
                    agent: ev.agent, kind: ev.kind ?? "vertex", t: ev.t ?? 0,
                })
                break
            }
            case "planning_finished":
                timeline.metrics = ev.metrics
                timeline.success = ev.success
                break
        }
    })
    return timeline
}
