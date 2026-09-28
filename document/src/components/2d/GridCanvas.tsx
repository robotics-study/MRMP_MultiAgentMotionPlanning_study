import {Fragment, useMemo, useRef} from "react";
import {Circle, Group, Layer, Line, Rect, Shape, Stage, Text} from "react-konva";
import Konva from "konva";
import {GridMap} from "../../libs/grid";
import {AGENT_COLORS, CONFLICT_COLOR, MultiTimeline} from "../../libs/trace/timeline";
import {Cell} from "../../libs/trace/types";
import {useCanvasColors} from "../../libs/useTheme";

// agent 색은 AGENT_COLORS(공용 팔레트)를 쓰고, conflict 마커만 구분되는 빨강이다.
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
    // 실행 재생 스텝 τ — null이면 탐색 단계(디스크 숨김), 아니면 각 agent가
    // 자기 space-time 경로의 min(τ, 끝) 셀에 디스크로 서 있다.
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

    // 셀별 확장 이벤트(step 이하) — 한 Shape로 배치 드로잉한다.
    const expandedByAgent = useMemo(() => {
        if (!timeline) return [] as Array<{color: string; cells: Cell[]}>
        const buckets = new Map<number, Cell[]>()
        for (const e of timeline.expanded) {
            if (e.step > step) continue
            const k = e.agent % AGENT_COLORS.length
            const arr = buckets.get(k) ?? []
            arr.push(e.cell)
            buckets.set(k, arr)
        }
        return Array.from(buckets.entries())
            .sort((a, b) => a[0] - b[0])
            .map(([k, cells]) => ({color: AGENT_COLORS[k], cells}))
    }, [timeline, step])

    const center = (c: Cell): [number, number] => [(c[1] + 0.5) * cell, (c[0] + 0.5) * cell]

    // 발표된 space-time 경로 (step 이하). start/goal 마커는 agents에서 따로 그리므로
    // 여기서는 선만 그린다.
    const visiblePaths = useMemo(
        () => (timeline ? timeline.paths.filter((p) => p.step <= step) : []),
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
        const [mx, my] = center(c)
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
        const px = (c[1] + 0.5) * cell
        const py = (c[0] + 0.5) * cell
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
                    const [x, y] = center(a[which])
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
                {/* 격자 선 — 셀이 충분히 클 때만 (작으면 노이즈) */}
                {cell >= 9 && Array.from({length: map.width + 1}, (_, k) => (
                    <Line key={`gv${k}`} points={[k * cell, 0, k * cell, stageH]} listening={false}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {cell >= 9 && Array.from({length: map.height + 1}, (_, k) => (
                    <Line key={`gh${k}`} points={[0, k * cell, stageW, k * cell]} listening={false}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {/* agent별 확장 셀 — 자기 hue로 반투명 채움 (joint 확장은 풀어서 전 agent 색) */}
                {expandedByAgent.map(({color, cells}) => (
                    <Shape key={`e${color}`} listening={false}
                           sceneFunc={(ctx, shape) => {
                               for (const c of cells) {
                                   ctx.fillRect(c[1] * cell, c[0] * cell, cell, cell)
                               }
                               ctx.fillStrokeShape(shape)
                           }}
                           fill={color} opacity={0.28}/>
                ))}
                {/* 발표된 space-time 경로 (start/goal 마커는 agents가 그린다) */}
                {visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const pts = p.path.flatMap((c) => center(c))
                    return (
                        <Line key={`p${p.agent}`} points={pts} stroke={color} listening={false}
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
                {/* 실행 재생: agent 디스크 (번호 표시) — 경로 끝에 도착하면 거기 고정. */}
                {execStep !== null && visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const [x, y] = center(p.path[Math.min(execStep, p.path.length - 1)])
                    return (
                        <Fragment key={`d${p.agent}`}>
                            <Circle x={x} y={y} radius={cell * 0.38} fill={color} listening={false}
                                    stroke={colors.bg} strokeWidth={Math.max(1, cell * 0.06)}/>
                            <Text text={String(p.agent)} width={cell * 0.76} height={cell * 0.76}
                                  x={x - cell * 0.38} y={y - cell * 0.38} align="center" verticalAlign="middle"
                                  fontSize={Math.max(8, cell * 0.4)} fill="#ffffff" fontStyle="bold" listening={false}/>
                        </Fragment>
                    )
                })}
            </Layer>
        </Stage>
    )
}

export default GridCanvas
