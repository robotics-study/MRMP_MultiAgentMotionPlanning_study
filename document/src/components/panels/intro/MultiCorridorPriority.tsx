import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {AGENT_COLORS, CONFLICT_COLOR} from "../../../libs/trace/timeline";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";

// MAPF의 두 번째 핵심 그림: 폭 1의 통로에서 마주 오는 두 로봇은 각자 최단으로는
// 정면 충돌한다. 우선순위를 매기면 낮은 쪽이 옆 pocket으로 비켜 높은 쪽을 먼저 보낸다.
// (SpaceTimeConflict의 열린 격자 교차와 달리 여기서는 회피 공간이 하나뿐이다.)
const COLS = 7;
const ROWS = 4;
const CELL = 40;
const W = COLS * CELL;
const H = ROWS * CELL;
const CORRIDOR = 2;               // 통로 행
const POCKET: [number, number] = [1, 4];   // 옆으로 비킬 수 있는 유일한 칸 (row, col)

// agent 0(우선순위 높음): 왼→오. agent 1(우선순위 낮음): 오→왼.
const pathA: Array<[number, number]> = [[2, 0], [2, 1], [2, 2], [2, 3], [2, 4], [2, 5], [2, 6]];
const pathBNaive: Array<[number, number]> = [[2, 6], [2, 5], [2, 4], [2, 3], [2, 2], [2, 1], [2, 0]];
// 비킴: pocket으로 올라가 agent 0이 지나가길 기다렸다가 다시 통로로.
const pathBYield: Array<[number, number]> = [
    [2, 6], [2, 5], [2, 4], [1, 4], [1, 4], [2, 4], [2, 3], [2, 2], [2, 1], [2, 0],
];

const isWall = (r: number, c: number) =>
    r !== CORRIDOR && !(r === POCKET[0] && c === POCKET[1]);

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [time, setTime] = useState(0)
    const [yield_, setYield] = useState(false)

    const bPath = yield_ ? pathBYield : pathBNaive
    const maxT = Math.max(pathA.length, bPath.length) - 1
    const at = (p: Array<[number, number]>, tm: number) => p[Math.min(tm, p.length - 1)]
    const a = at(pathA, time)
    const b = at(bPath, time)
    const conflict = a[0] === b[0] && a[1] === b[1]

    const cx = (c: number) => (c + 0.5) * CELL
    const cy = (r: number) => (r + 0.5) * CELL

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {Array.from({length: ROWS}, (_, r) =>
                        Array.from({length: COLS}, (_, c) =>
                            isWall(r, c) ? (
                                <Rect key={`w${r}-${c}`} x={c * CELL} y={r * CELL}
                                      width={CELL} height={CELL}
                                      fill={colors.text} opacity={0.82}/>
                            ) : (
                                <Rect key={`c${r}-${c}`} x={c * CELL} y={r * CELL}
                                      width={CELL} height={CELL}
                                      stroke={colors.border} strokeWidth={0.5}/>
                            )))}
                    <Line points={pathA.flatMap(([r, c]) => [cx(c), cy(r)])}
                          stroke={AGENT_COLORS[0]} strokeWidth={2} opacity={0.35}/>
                    <Line points={bPath.flatMap(([r, c]) => [cx(c), cy(r)])}
                          stroke={AGENT_COLORS[1]} strokeWidth={2} opacity={0.35}/>
                    {conflict && (
                        <Rect x={a[1] * CELL} y={a[0] * CELL} width={CELL} height={CELL}
                              fill={CONFLICT_COLOR} opacity={0.4}/>
                    )}
                    <Circle x={cx(a[1])} y={cy(a[0])} radius={CELL * 0.3} fill={AGENT_COLORS[0]}/>
                    <Circle x={cx(b[1])} y={cy(b[0])} radius={CELL * 0.3} fill={AGENT_COLORS[1]}/>
                    <Text x={cx(a[1]) - 4} y={cy(a[0]) - 5} text="0" fontSize={12}
                          fill={colors.bg} fontStyle="bold"/>
                    <Text x={cx(b[1]) - 4} y={cy(b[0]) - 5} text="1" fontSize={12}
                          fill={colors.bg} fontStyle="bold"/>
                    <Text x={6} y={6} fontSize={12} fill={conflict ? CONFLICT_COLOR : colors.muted}
                          text={conflict
                              ? t("head-on collision!", "정면 충돌!")
                              : `t = ${time}`}/>
                </Layer>
            </Stage>
            <div className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: W}}>
                <input type="range" min={0} max={maxT} value={Math.min(time, maxT)}
                       onChange={(e) => setTime(parseInt(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("time", "time")}/>
                <button type="button" onClick={() => setYield((y) => !y)}
                        className={cn(
                            "px-2 py-0.5 rounded border whitespace-nowrap",
                            yield_
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {yield_ ? t("priority on", "우선순위 on") : t("prioritize", "우선순위 부여")}
                </button>
            </div>
            <div className="text-xs text-muted text-center">
                {yield_
                    ? t("agent 1 (lower priority) steps into the pocket and lets agent 0 pass",
                        "agent 1(낮은 우선순위)이 pocket으로 비켜 agent 0을 먼저 보낸다")
                    : t("both take the shortest path and meet head-on in the one-wide corridor",
                        "둘 다 최단으로 가다 폭 1 통로에서 정면으로 만난다")}
            </div>
        </div>
    )
}

const MultiCorridorPriority = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "In a one-wide corridor the only way past a head-on conflict is for one robot to yield. Prioritized planning fixes an order; the lower-priority agent treats the higher one as a moving obstacle and ducks into the pocket.",
            "폭 1 통로에서 정면 충돌을 피하는 길은 한 로봇이 양보하는 것뿐이다. Prioritized planning은 순서를 정하고, 낮은 우선순위 agent가 높은 쪽을 움직이는 장애물로 보고 pocket으로 비킨다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default MultiCorridorPriority
