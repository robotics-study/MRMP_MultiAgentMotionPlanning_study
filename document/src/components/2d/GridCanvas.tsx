import {Fragment, useMemo} from "react";
import {Circle, Layer, Line, Rect, Shape, Stage, Text} from "react-konva";
import {GridMap} from "../../libs/grid";
import {AGENT_COLORS, CONFLICT_COLOR, MultiTimeline} from "../../libs/trace/timeline";
import {Cell} from "../../libs/trace/types";
import {useCanvasColors} from "../../libs/useTheme";

// agent 색은 AGENT_COLORS(공용 팔레트)를 쓰고, conflict 마커만 구분되는 빨강이다.
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
}

const GridCanvas = ({map, panel, timeline, step = Infinity, execStep = null}: GridCanvasProps) => {
    const colors = useCanvasColors();
    const cell = panel / Math.max(map.width, map.height);
    const stageW = Math.round(cell * map.width);
    const stageH = Math.round(cell * map.height);

    // agent 인덱스별 확장 셀 (step 이하만) — 한 Shape로 배치 드로잉한다.
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

    // 발표된 space-time 경로 (step 이하) — 시작 점 / 목표 링은 경로와 함께 드러난다.
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

    return (
        <Stage width={stageW} height={stageH}
               className="bg-surface border border-border rounded-lg overflow-hidden w-fit">
            <Layer>
                {/* 벽 */}
                {map.occupied.map((occ, i) => occ && (
                    <Rect key={`w${i}`} x={(i % map.width) * cell} y={Math.floor(i / map.width) * cell}
                          width={cell} height={cell} fill={colors.text} opacity={0.78}/>
                ))}
                {/* 격자 선 — 셀이 충분히 클 때만 (작으면 노이즈) */}
                {cell >= 9 && Array.from({length: map.width + 1}, (_, k) => (
                    <Line key={`gv${k}`} points={[k * cell, 0, k * cell, stageH]}
                          stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                ))}
                {cell >= 9 && Array.from({length: map.height + 1}, (_, k) => (
                    <Line key={`gh${k}`} points={[0, k * cell, stageW, k * cell]}
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
                {/* 발표된 space-time 경로 + 시작 점/목표 링 */}
                {visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const pts = p.path.flatMap((c) => center(c))
                    return (
                        <Line key={`p${p.agent}`} points={pts} stroke={color}
                              strokeWidth={Math.max(1.6, cell * 0.18)} opacity={0.9}
                              lineCap="round" lineJoin="round"/>
                    )
                })}
                {visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const [sx, sy] = center(p.path[0])
                    return <Circle key={`s${p.agent}`} x={sx} y={sy} radius={cell * 0.22} fill={color}/>
                })}
                {visiblePaths.map((p) => {
                    const color = AGENT_COLORS[p.agent % AGENT_COLORS.length]
                    const [gx, gy] = center(p.path[p.path.length - 1])
                    return <Circle key={`g${p.agent}`} x={gx} y={gy} radius={cell * 0.36} fill="none"
                                  stroke={color} strokeWidth={Math.max(1.2, cell * 0.09)}/>
                })}
                {/* 제약: agent 색 점선 사각형 (edge 제약은 양끝 모두) */}
                {visibleConstraints.map((c, i) => {
                    const color = AGENT_COLORS[c.agent % AGENT_COLORS.length]
                    return [c.cell, ...(c.to ? [c.to] : [])].map((cc, j) => (
                        <Rect key={`c${i}-${j}`} x={cc[1] * cell} y={cc[0] * cell}
                              width={cell} height={cell} fill="none" stroke={color}
                              strokeWidth={1.4} dash={[cell * 0.3, cell * 0.24]}/>
                    ))
                })}
                {/* conflict: 다투는 셀마다 빨간 X */}
                {visibleConflicts.map((c, i) => c.cells.map((cc, j) => {
                    const [x1, y1] = [cc[1] * cell, cc[0] * cell]
                    return (
                        <Line key={`x${i}-${j}`} points={[x1 + cell * 0.2, y1 + cell * 0.2,
                            x1 + cell * 0.8, y1 + cell * 0.8, x1 + cell * 0.8, y1 + cell * 0.2,
                            x1 + cell * 0.2, y1 + cell * 0.8]} stroke={CONFLICT_COLOR}
                              strokeWidth={Math.max(1.6, cell * 0.14)} lineCap="round"/>
                    )
                }))}
                {/* 실행 재생: agent 디스크 (번호 표시) — 경로 끝에 도착하면 거기 고정.
                    Konva의 Shape(Circle)은 Container가 아니라 자식을 가질 수 없다
                    (Node.prototype.add 없음) — 라벨은 디스크와 나란한 형제 Text로 둔다. */}
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
