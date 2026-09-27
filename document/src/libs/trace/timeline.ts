import {Cell, TraceEvent} from "./types";

// trace 이벤트 열을 재생 가능한 타임라인으로 접는다. step은 이벤트 index와 같고,
// 렌더러는 "step 이하"의 이벤트만 그린다 — 스크러버로 임의 시점 탐색이 가능하다.
// MRMP는 joint state 확장을 agent별로 풀어 expanded에 담고(색 구분), path_found의
// space-time 경로는 실행 재생(execStep)에 쓰인다.

// agent 인덱스별 hue (순환) — tools/viz/replay.py의 팔레트와 동일해야 GIF와
// 브라우저 재생이 같은 실행으로 읽힌다.
export const AGENT_COLORS = ["#0d9488", "#c2179b", "#2563eb", "#ca8a04", "#7c3aed", "#e5484d"];

// conflict 마커 색 — 모든 agent hue와 구분되는 빨강 (replay.py와 동일).
export const CONFLICT_COLOR = "#dc2626";

export interface MultiTimeline {
    steps: number;                                       // 마지막 step (= 이벤트 수)
    algorithm: string;
    // joint state 확장은 agent별로 풀어 담는다 — cell과 그 agent 인덱스.
    expanded: Array<{step: number; cell: Cell; agent: number}>;
    // agent별 space-time 경로 (path[t] = 시각 t의 셀). step = path_found 이벤트 순번.
    paths: Array<{step: number; agent: number; path: Cell[]}>;
    conflicts: Array<{step: number; cells: Cell[]; agents: [number, number]; t: number}>;
    constraints: Array<{step: number; cell: Cell; to?: Cell; agent: number;
        kind: "vertex" | "edge"; t: number}>;
    makespan: number;                                    // 마지막 도착 스텝
    params?: Record<string, unknown>;
    metrics?: Record<string, number>;
    success?: boolean;
}

const asCell = (state?: number[]): Cell | null =>
    state && state.length >= 2 ? [state[0], state[1]] : null

export function buildTimeline(events: TraceEvent[]): MultiTimeline {
    const timeline: MultiTimeline = {
        steps: events.length,
        algorithm: "",
        expanded: [],
        paths: [],
        conflicts: [],
        constraints: [],
        makespan: 0,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "planning_started":
                timeline.algorithm = ev.algorithm ?? ""
                timeline.params = ev.params
                break
            case "node_expanded": {
                const state = ev.state ?? []
                if (ev.agent !== undefined) {
                    // 개별 탐색(prioritized / CBS low level): 셀 하나.
                    const cell = asCell(state)
                    if (cell) timeline.expanded.push({step, cell, agent: ev.agent})
                } else {
                    // joint state 확장: 평면화된 상태에서 agent쌍마다 풀어 낸다.
                    for (let k = 0; k + 1 < state.length; k += 2) {
                        timeline.expanded.push({step, cell: [state[k], state[k + 1]], agent: k})
                    }
                }
                break
            }
            case "path_found": {
                const path = (ev.path ?? []) as Cell[]
                if (ev.agent === undefined) break
                timeline.paths.push({step, agent: ev.agent, path})
                timeline.makespan = Math.max(timeline.makespan, path.length - 1)
                break
            }
            case "conflict_found": {
                const cell = asCell(ev.cell)
                if (!cell || !ev.agents) break
                const cells: Cell[] = [cell]
                const to = asCell(ev.to)
                if (to) cells.push(to)
                timeline.conflicts.push({step, cells, agents: ev.agents, t: ev.t ?? 0})
                break
            }
            case "constraint_added": {
                const cell = asCell(ev.cell)
                if (!cell || ev.agent === undefined) break
                timeline.constraints.push({
                    step, cell, to: asCell(ev.to) ?? undefined,
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
