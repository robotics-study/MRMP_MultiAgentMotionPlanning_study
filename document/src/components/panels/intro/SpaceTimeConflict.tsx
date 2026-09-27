import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {AGENT_COLORS, CONFLICT_COLOR} from "../../../libs/trace/timeline";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";

// MAPF의 핵심 그림: 각자 최단인 두 경로가 같은 시각·같은 셀에서 만난다(vertex conflict).
// 시간 슬라이더로 재생하고, CBS 풍의 해결(한 agent가 한 스텝 대기) 토글을 제공한다.
const N = 7;
const PANEL = 300;

// agent A: 좌→우, agent B: 상→하. 해결 모드에서 B는 한 스텝 기다린다.
const pathA: Array<[number, number]> = [[3, 0], [3, 1], [3, 2], [3, 3], [3, 4], [3, 5], [3, 6]];
const pathB: Array<[number, number]> = [[0, 3], [1, 3], [2, 3], [3, 3], [4, 3], [5, 3], [6, 3]];
const pathBWait: Array<[number, number]> = [[0, 3], [1, 3], [2, 3], [2, 3], [3, 3], [4, 3], [5, 3], [6, 3]];

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [time, setTime] = useState(0)
    const [resolved, setResolved] = useState(false)

    const bPath = resolved ? pathBWait : pathB
    const maxT = Math.max(pathA.length, bPath.length) - 1
    const cell = PANEL / N
    const at = (path: Array<[number, number]>, tm: number) => path[Math.min(tm, path.length - 1)]

    const a = at(pathA, time)
    const b = at(bPath, time)
    const conflict = a[0] === b[0] && a[1] === b[1]

    const cx = (c: number) => (c + 0.5) * cell
    const cy = (r: number) => (r + 0.5) * cell

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`v${k}`} points={[k * cell, 0, k * cell, PANEL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`h${k}`} points={[0, k * cell, PANEL, k * cell]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    <Line points={pathA.flatMap(([r, c]) => [cx(c), cy(r)])}
                          stroke={AGENT_COLORS[0]} strokeWidth={2} opacity={0.35}/>
                    <Line points={bPath.flatMap(([r, c]) => [cx(c), cy(r)])}
                          stroke={AGENT_COLORS[1]} strokeWidth={2} opacity={0.35}/>
                    {conflict && (
                        <Rect x={a[1] * cell} y={a[0] * cell} width={cell} height={cell}
                              fill={CONFLICT_COLOR} opacity={0.35}/>
                    )}
                    <Circle x={cx(a[1])} y={cy(a[0])} radius={cell * 0.3} fill={AGENT_COLORS[0]}/>
                    <Circle x={cx(b[1])} y={cy(b[0])} radius={cell * 0.3} fill={AGENT_COLORS[1]}/>
                    <Text x={6} y={6} fontSize={12} fill={conflict ? CONFLICT_COLOR : colors.muted}
                          text={conflict
                              ? t("vertex conflict!", "vertex conflict!")
                              : `t = ${time}`}/>
                </Layer>
            </Stage>
            <div className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: PANEL}}>
                <input type="range" min={0} max={maxT} value={Math.min(time, maxT)}
                       onChange={(e) => setTime(parseInt(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("time", "time")}/>
                <button type="button" onClick={() => setResolved((r) => !r)}
                        className={cn(
                            "px-2 py-0.5 rounded border",
                            resolved
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {resolved ? t("constraint on", "constraint on") : t("resolve", "resolve")}
                </button>
            </div>
            <div className="text-xs text-muted text-center">
                {resolved
                    ? t("red agent waits one step — one constraint removes the conflict",
                        "빨간 agent가 한 스텝 기다린다. 제약 하나가 conflict를 없앤다")
                    : t("both paths are individually optimal, and they meet at t = 3",
                        "두 경로는 각자 최적이지만 t = 3에서 만난다")}
            </div>
        </div>
    )
}

const SpaceTimeConflict = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Two individually optimal paths, one shared cell: scrub time to find the vertex conflict, then resolve it with a single wait constraint",
            "각자 최적인 두 경로와 하나의 공유 셀. 시간을 움직여 vertex conflict를 찾고, 대기 제약 하나로 해결해 보라",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default SpaceTimeConflict
