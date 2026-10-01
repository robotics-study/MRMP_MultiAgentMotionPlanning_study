import {Fragment, useMemo, useRef} from "react";
import {Circle, Group, Layer, Line, Rect, Shape, Stage, Text} from "react-konva";
import Konva from "konva";
import {GridMap, worldToCellFloat} from "../../libs/grid";
import {AGENT_COLORS, CONFLICT_COLOR, MultiTimeline} from "../../libs/trace/timeline";
import {Cell, Point} from "../../libs/trace/types";
import {useCanvasColors} from "../../libs/useTheme";

// agent 색은 AGENT_COLORS(공용 팔레트)를 쓰고, conflict 마커만 구분되는 빨강이다.
// endpoint 핸들은 항상 셀에 스냅된다 — world 모드에서도 드래그는 셀 단위로 움직이고
// 변환(cellToWorld)이 같은 픽셀을 가리킨다.
export interface AgentMarker {
    start: Cell;
    goal: Cell;
}

export interface GridCanvasProps {
    map: GridMap;
    // 가장 긴 변의 픽셀 크기. 셀 크기는 여기서 유도된다.
    panel: number;
    timeline?: MultiTimeline;
    // 이 step 이하의 이벤트만 그린다 (탐색 재생/스크럽).
    step?: number;
    // 실행 재생 스텝 τ — null이면 탐색 단계(디스크 숨김). discrete는 정수 스텝,
    // world(timeline.coords="world")는 소수 τ: 웨이포인트 사이 선형 보간으로 디스크가 미끄러진다.
    execStep?: number | null;
    // sandbox: agent별 endpoint (start 점 + goal 링). 발표된 경로와 무관하게 항상 보인다 —
    // 계획 실패 때도 핸들을 잡아 옮겨야 하므로.
    agents?: Array<AgentMarker>;
    // sandbox 상호작용 — 핸들러가 있을 때만 활성화된다.
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveAgent?: (agent: number, which: "start" | "goal", cell: Cell) => void;
}

const GridCanvas = ({map, panel, timeline, step = Infinity, execStep = null,
                    agents, onPaintCell, onMoveAgent}: GridCanvasProps) => {
    const colors = useCanvasColors();
    const cell = panel / Math.max(map.width, map.height);
    const stageW = Math.round(cell * map.width);
    const stageH = Math.round(cell * map.height);
    // world 모드: 상태 페어는 world point이고 디스크는 진짜 반지름(미터→픽셀)을 그린다.
    const world = timeline?.coords === "world";

    // 상태 페어 → 픽셀. discrete는 셀 중심, world는 (x-origin)/res 변환 — cell center의
    // world 점은 정확히 같은 픽셀에 착지하므로 두 모드가 한 캔버스에서 섞이지 않아도 된다.
    const toXY = (p: Point): [number, number] => {
        if (!world) return [(p[1] + 0.5) * cell, (p[0] + 0.5) * cell];
        const [rf, cf] = worldToCellFloat(map, p);
        return [cf * cell, rf * cell];
    }

    // 셀별 확장 이벤트(step 이하) — discrete는 한 Shape로 배치 드로잉, world는 점들.
    const expandedByAgent = useMemo(() => {
        if (!timeline) return [] as Array<{color: string; points: Point[]}>
        const buckets = new Map<number, Point[]>()
        for (const e of timeline.expanded) {
            if (e.step > step) continue
            const k = e.agent % AGENT_COLORS.length
            const arr = buckets.get(k) ?? []
            arr.push(e.point)
            buckets.set(k, arr)
        }
        return Array.from(buckets.entries())
            .sort((a, b) => a[0] - b[0])
            .map(([k, points]) => ({color: AGENT_COLORS[k], points}))
    }, [timeline, step])

    // world 모드만: agent별 roadmap(간선 + 정점 점) — 트리 탐색과 개별 PRM을 구분해 그린다.
    const roadmapsByAgent = useMemo(() => {
        if (!world || !timeline) return []
        return timeline.roadmaps
            .filter((r) => r.step <= step)
            .map((r) => ({color: AGENT_COLORS[r.agent % AGENT_COLORS.length], ...r}))
    }, [timeline, step, world])

    // 발표된 space-time 경로와 timed route (start/goal 마커는 agents가 그리므로 여기서는 선만).
    const visiblePaths = useMemo(
        () => (timeline ? timeline.paths.filter((p) => p.step <= step) : []),
        [timeline, step],
    )
    const visibleSchedules = useMemo(
        () => (timeline ? timeline.schedules.filter((s) => s.step <= step) : []),
        [timeline, step],
    )
    const visibleConflicts = useMemo(
        () => (timeline ? timeline.conflicts.filter((c) => c.step <= step) : []),
        [timeline, step],
    )
    const visibleConstraints = useMemo(
        () => (timeline ? timeline.constraints.filter((c) => c.step <= step) : []),
        [timeline, step],
    )

    // 벽 페인팅: pointer down 시 첫 셀의 반전값을 붓 값으로 삼아 드래그 내내 유지한다.
    // endpoint 가드는 셀 일치가 아니라 픽셀 거리로 잰다 — 마커가 셀 경계에 걸치면
    // 클릭 셀과 마커 셀이 어긋나, 마커를 잡으려는 클릭이 페인팅으로 새기기 때문이다.
    const nearMarker = (c: Cell | undefined, px: number, py: number): boolean => {
        if (!c) return false
        const [mx, my] = toXY(c)
        return Math.hypot(px - mx, py - my) <= cell * 0.75
    }
    const nearAnyMarker = (px: number, py: number): boolean => {
        if (!agents) return false
        return agents.some((a) => nearMarker(a.start, px, py) || nearMarker(a.goal, px, py))
    }
    const paintValue = useRef<boolean | null>(null)
    const cellAt = (stage: Konva.Stage | null): Cell | null => {
        const pos = stage?.getPointerPosition()
        if (!pos) return null
        const col = Math.floor(pos.x / cell)
        const row = Math.floor(pos.y / cell)
        if (row < 0 || row >= map.height || col < 0 || col >= map.width) return null
        return [row, col]
    }
    const paint = (c: Cell) => {
        if (!onPaintCell || paintValue.current === null) return
        const [px, py] = [(c[1] + 0.5) * cell, (c[0] + 0.5) * cell]
        if (nearAnyMarker(px, py)) return
        onPaintCell(c[0], c[1], paintValue.current)
    }

    // endpoint 핸들 — draggable이면 드래그로 옮긴다. 스냅은 부모 상태 갱신이 담당한다
    // (원래 중심으로 되돌려 이중 이동을 막는다). Konva의 Shape은 Container가 아니라
    // 자식을 가질 수 없다(Node.prototype.add 없음) — 번호 라벨은 디스크와 나란한
    // 형제 Text로 두고, 함께 움직여야 하므로 Group으로 묶는다.
    const marker = (a: AgentMarker, i: number) => {
        const color = AGENT_COLORS[i % AGENT_COLORS.length]
        return (
            <Fragment key={`m${i}`}>
                {[("start" as const), ("goal" as const)].map((which) => {
                    const [x, y] = toXY(a[which])
                    const isStart = which === "start"
                    return (
                        <Group key={which} x={x} y={y} draggable={!!onMoveAgent}
                               onDragEnd={(e) => {
                                   const col = Math.floor(e.target.x() / cell)
                                   const row = Math.floor(e.target.y() / cell)
                                   e.target.position({x, y})
                                   onMoveAgent?.(i, which, [
                                       Math.max(0, Math.min(map.height - 1, row)),
                                       Math.max(0, Math.min(map.width - 1, col)),
                                   ])
                               }}>
                            <Circle x={0} y={0} radius={isStart ? cell * 0.3 : cell * 0.36}
                                    fill={isStart ? color : "none"} stroke={color}
                                    strokeWidth={Math.max(1.2, cell * (isStart ? 0.09 : 0.08))}
                                    dash={isStart ? undefined : [cell * 0.3, cell * 0.24]}
                                    hitStrokeWidth={cell * 0.9}/>
                            {isStart && (
                                <Text text={String(i)} width={cell * 0.6} height={cell * 0.6}
                                      x={-cell * 0.3} y={-cell * 0.3} align="center" verticalAlign="middle"
                                      fontSize={Math.max(8, cell * 0.34)} fill="#ffffff" fontStyle="bold"
                                      listening={false}/>
                            )}
                        </Group>
                    )
                })}
            </Fragment>
        )
    }

    // world 실행 재생: 소수 τ에서 웨이포인트 사이 선형 보간 (연속 계획의 움직임은 직선).
    const pathAt = (path: Point[], tau: number): Point => {
        if (tau <= 0) return path[0]
        if (tau >= path.length - 1) return path[path.length - 1]
        const t = Math.floor(tau)
        const f = tau - t
        const a = path[t], b = path[t + 1]
        return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
    }

    // timed 실행 재생 — uniform velocity model(replay.py의 timed_position 미러): departure
    // (D_i = times[i+1] − 1/vmax)까지 cells[i]에 체류하고 그 다음에만 정확히 vmax로 이동.
    // departure 이전에는 외삽 없이 셀 그대로다.
    const scheduleAt = (cells: Cell[], times: number[], v: number, tau: number): Point => {
        if (tau <= times[0]) return [cells[0][0], cells[0][1]]
        for (let i = 0; i < times.length - 1; i++) {
            const arrive = times[i + 1]
            const depart = arrive - 1 / v
            if (tau < depart) return [cells[i][0], cells[i][1]]
            if (tau < arrive) {
                const f = (tau - depart) * v
                const a = cells[i], b = cells[i + 1]
                return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
            }
        }
        const last = cells[cells.length - 1]
        return [last[0], last[1]]
    }

    return (
        <Stage width={stageW} height={stageH}
               className="bg-surface border border-border rounded-lg overflow-hidden w-fit"
               onPointerDown={(e) => {
                   if (!onPaintCell) return
                   const pos = e.target.getStage()?.getPointerPosition()
                   // endpoint 근처에서 드래그를 시작하면 페인팅이 아니라 핸들 이동이다.
                   if (!pos || nearAnyMarker(pos.x, pos.y)) return
                   const c = cellAt(e.target.getStage())
                   if (!c) return
                   paintValue.current = !map.occupied[c[0] * map.width + c[1]]
                   paint(c)
               }}
               onPointerMove={(e) => {
                   if (paintValue.current === null) return
                   const c = cellAt(e.target.getStage())
                   if (c) paint(c)
               }}
               onPointerUp={() => { paintValue.current = null }}
               onPointerLeave={() => { paintValue.current = null }}>
            <Layer>
                {/* 벽 */}
                {map.occupied.map((occ, i) => occ && (
                    <Rect key={`w${i}`} x={(i % map.width) * cell} y={Math.floor(i / map.width) * cell}
                          width={cell} height={cell} fill={colors.text} opacity={0.78} listening={false}/>
                ))}
                {/* 격자 선 — 셀이 충분히 클 때만 (작으면 노이즈). world 모드는 점/선 위주라 더 드물게. */}
                {(!world && cell >= 9) && Array.from({length: map.width + 1}, (_, k) => (
                    <Line key={`gv${k}`} points={[k * cell, 0, k * cell, stageH]} listening={false}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {(!world && cell >= 9) && Array.from({length: map.height + 1}, (_, k) => (
                    <Line key={`gh${k}`} points={[0, k * cell, stageW, k * cell]} listening={false}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {/* world 모드: agent별 개별 roadmap — 옅은 간선 + 정점 점 (tree 탐색의 무대) */}
                {roadmapsByAgent.map((rm) => (
                    <Shape key={`re${rm.color}`} listening={false}
                           sceneFunc={(ctx, shape) => {
                               for (const [a, b] of rm.edges) {
                                   const pa = toXY(rm.vertices[a])
                                   const pb = toXY(rm.vertices[b])
                                   ctx.beginPath()
                                   ctx.moveTo(pa[0], pa[1])
                                   ctx.lineTo(pb[0], pb[1])
                                   // fill 미설정 — stroke만 shape에 적용된다.
                                   ctx.fillStrokeShape(shape)
                               }
                           }}
                           stroke={rm.color} opacity={0.25} strokeWidth={Math.max(0.6, cell * 0.05)}/>
                ))}
                {roadmapsByAgent.map((rm) => (
                    <Shape key={`rv${rm.color}`} listening={false}
                           sceneFunc={(ctx, shape) => {
                               for (const v of rm.vertices) {
                                   const [x, y] = toXY(v)
                                   ctx.beginPath()
                                   ctx.arc(x, y, Math.max(1.2, cell * 0.1), 0, Math.PI * 2)
                                   ctx.fillStrokeShape(shape)
                               }
                           }}
                           fill={rm.color} opacity={0.5}/>
                ))}
                {/* agent별 확장 — discrete는 셀 채움, world는 점 (joint 확장은 풀어 전 agent 색) */}
                {expandedByAgent.map(({color, points}) => (
                    <Shape key={`e${color}`} listening={false}
                           sceneFunc={(ctx, shape) => {
                               for (const p of points) {
                                   if (!world) {
                                       ctx.fillRect(p[1] * cell, p[0] * cell, cell, cell)
                                       continue
                                   }
                                   const [x, y] = toXY(p)
                                   ctx.beginPath()
                                   ctx.arc(x, y, Math.max(1.6, cell * 0.18), 0, Math.PI * 2)
                                   ctx.fillStrokeShape(shape)
                               }
                           }}
                           fill={color} opacity={world ? 0.95 : 0.28}/>
                ))}
                {/* 발표된 space-time 경로와 timed route (start/goal 마커는 agents가 그린다) */}
                {visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const pts = p.path.flatMap((pt) => toXY(pt))
                    return (
                        <Line key={`p${p.agent}`} points={pts} stroke={color} listening={false}
                              strokeWidth={Math.max(1.6, cell * 0.18)} opacity={0.9}
                              lineCap="round" lineJoin="round"/>
                    )
                })}
                {visibleSchedules.map((s) => {
                    const color = AGENT_COLORS[s.agent % AGENT_COLORS.length]
                    const pts = s.cells.flatMap((c) => toXY([c[0], c[1]]))
                    return (
                        <Line key={`s${s.agent}`} points={pts} stroke={color} listening={false}
                              strokeWidth={Math.max(1.6, cell * 0.18)} opacity={0.9}
                              lineCap="round" lineJoin="round"/>
                    )
                })}
                {/* 제약: agent 색 점선 사각형 (edge 제약은 양끝 모두) */}
                {visibleConstraints.map((c, i) => {
                    const color = AGENT_COLORS[c.agent % AGENT_COLORS.length]
                    return [c.cell, ...(c.to ? [c.to] : [])].map((cc, j) => (
                        <Rect key={`c${i}-${j}`} x={cc[1] * cell} y={cc[0] * cell}
                              width={cell} height={cell} fill="none" stroke={color} listening={false}
                              strokeWidth={1.4} dash={[cell * 0.3, cell * 0.24]}/>
                    ))
                })}
                {/* conflict: 다투는 셀마다 빨간 X */}
                {visibleConflicts.map((c, i) => c.cells.map((cc, j) => {
                    const [x1, y1] = [cc[1] * cell, cc[0] * cell]
                    return (
                        <Line key={`x${i}-${j}`} points={[x1 + cell * 0.2, y1 + cell * 0.2,
                            x1 + cell * 0.8, y1 + cell * 0.8, x1 + cell * 0.8, y1 + cell * 0.2,
                            x1 + cell * 0.2, y1 + cell * 0.8]} stroke={CONFLICT_COLOR} listening={false}
                              strokeWidth={Math.max(1.6, cell * 0.14)} lineCap="round"/>
                    )
                }))}
                {/* agent endpoint 핸들: 번호가 적힌 start 점 + goal 링 (sandbox) */}
                {agents?.map((a, i) => marker(a, i))}
                {/* 실행 재생: agent 디스크 — world는 진짜 반지름(미터), discrete/timed는 셀 크기.
                    경로 끝에 도착하면 거기 고정이고, world의 소수 τ는 웨이포인트 선형 보간,
                    timed는 scheduleAt의 dwell→traverse (속도는 timeline.vmax). */}
                {execStep !== null && visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const rPx = world && timeline?.radius
                        ? timeline.radius[p.agent % timeline.radius.length] * (cell / map.resolution)
                        : cell * 0.38
                    const [x, y] = toXY(pathAt(p.path, execStep))
                    return (
                        <Fragment key={`d${p.agent}`}>
                            <Circle x={x} y={y} radius={rPx} fill={color} listening={false}
                                    stroke={colors.bg} strokeWidth={Math.max(1, rPx * 0.25)}/>
                            <Text text={String(p.agent)} width={rPx * 2} height={rPx * 2}
                                  x={x - rPx} y={y - rPx} align="center" verticalAlign="middle"
                                  fontSize={Math.max(8, rPx * 1.05)} fill="#ffffff" fontStyle="bold" listening={false}/>
                        </Fragment>
                    )
                })}
                {execStep !== null && timeline?.timed && visibleSchedules.map((s) => {
                    const color = AGENT_COLORS[s.agent % AGENT_COLORS.length]
                    const vmax = timeline.vmax ?? [1]
                    const rPx = cell * 0.38
                    const [x, y] = toXY(scheduleAt(s.cells, s.times,
                        vmax[s.agent % vmax.length], execStep))
                    return (
                        <Fragment key={`sd${s.agent}`}>
                            <Circle x={x} y={y} radius={rPx} fill={color} listening={false}
                                    stroke={colors.bg} strokeWidth={Math.max(1, rPx * 0.25)}/>
                            <Text text={String(s.agent)} width={rPx * 2} height={rPx * 2}
                                  x={x - rPx} y={y - rPx} align="center" verticalAlign="middle"
                                  fontSize={Math.max(8, rPx * 1.05)} fill="#ffffff" fontStyle="bold" listening={false}/>
                        </Fragment>
                    )
                })}
            </Layer>
        </Stage>
    )
}

export default GridCanvas
