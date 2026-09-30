import {ReactNode, useEffect, useMemo, useState} from "react";
import {GridMap} from "../../libs/grid";
import {MultiTimeline} from "../../libs/trace/timeline";
import {Cell} from "../../libs/trace/types";
import GridCanvas, {AgentMarker} from "../2d/GridCanvas";
import {useTr} from "../../libs/i18n";

// 재생은 하나의 시계다. tick이 steps보다 작은 동안은 탐색 phase(이벤트 prefix를
// 고정 배속으로 압축 — 이벤트 수와 무관하게 체감 속도 동일)이고, 그 뒤는 실행
// phase: 각 agent가 자기 space-time 경로를 스텝마다 걸어간다. continuous trace
// (coords=world)는 같은 속도로 1/5 마이크로 스텝씩 — 웨이포인트 사이를 디스크가
// 선형 보간으로 미끄러지므로 소수 τ가 재생의 기본 단위다. 슬라이더 하나가 두
// phase 전체를 스크럽하고, ⏮/⏭은 정지 상태에서 한 tick(discrete는 이동 한 칸,
// world는 보간 마이크로 스텝)씩 옮긴다 — 알고리즘의 동작은 실행 phase에서 보이므로
// 그 구간을 되감아 볼 수 있어야 한다.
const SEARCH_MS = 3000;
const TICK_MS = 30;
const EXEC_STEP_MS = 250;
const WORLD_SUBSTEPS = 5;

interface TracePlayerProps {
    map: GridMap;
    timeline: MultiTimeline;
    autoPlay?: boolean;
    // sandbox 모드: agent별 endpoint 핸들과 상호작용 핸들러를 캔버스로 관통시킨다.
    agents?: Array<AgentMarker>;
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveAgent?: (agent: number, which: "start" | "goal", cell: Cell) => void;
    // 있으면 리셋 버튼이 붙는다. preset의 맵/endpoint를 되돌린다.
    onReset?: () => void;
    panel?: number;
    // 플레이어 아래 추가 컨트롤 (demo 페이지의 설명, sandbox의 agent 추가 등).
    footer?: ReactNode;
}

const Btn = ({onClick, label, children}: {
    onClick: () => void; label: string; children: ReactNode
}) => (
    <button type="button" onClick={onClick} aria-label={label}
            className="px-1.5 py-1 rounded border border-border hover:bg-surface leading-none">
        {children}
    </button>
)

const TracePlayer = ({map, timeline, autoPlay = true, agents, onPaintCell, onMoveAgent,
                     onReset, panel = 340, footer}: TracePlayerProps) => {
    const t = useTr()
    // 단일 시계 — step과 execStep은 tick에서 유도된다 (위 주석 참조).
    const sub = timeline.coords === "world" ? WORLD_SUBSTEPS : 1
    const total = timeline.steps + Math.ceil(timeline.makespan * sub)
    const [tick, setTick] = useState(autoPlay ? 0 : total)
    const [playing, setPlaying] = useState(autoPlay)

    const searching = tick < timeline.steps
    const step = Math.min(tick, timeline.steps)
    const execStep = searching ? null : Math.min((tick - timeline.steps) / sub, timeline.makespan)
    const finished = tick >= total

    // 타임라인이 바뀌면(입력 변경/트레이스 교체) 시계를 처음으로 되돌려 다시 재생한다.
    useEffect(() => {
        setTick(autoPlay ? 0 : total)
        setPlaying(autoPlay)
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [timeline, autoPlay])

    // 탐색 phase: step을 고정 배속으로 굴린다.
    useEffect(() => {
        if (!playing || !searching) return
        const perTick = Math.max(1, Math.round(timeline.steps / (SEARCH_MS / TICK_MS)))
        const timer = window.setInterval(() => {
            setTick((v) => Math.min(timeline.steps, v + perTick))
        }, TICK_MS)
        return () => window.clearInterval(timer)
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [playing, searching, timeline])

    // 실행 phase: τ를 고정 스텝 속도로 굴린다 (makespan 1 스텝당 EXEC_STEP_MS).
    useEffect(() => {
        if (!playing || searching) return
        const timer = window.setInterval(() => {
            setTick((v) => Math.min(total, v + sub))
        }, EXEC_STEP_MS / sub)
        return () => window.clearInterval(timer)
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [playing, searching, timeline, sub, total])

    // 전체 타임라인이 끝나면 자동 재생을 멈춘다.
    useEffect(() => {
        if (playing && tick >= total) setPlaying(false)
    }, [playing, tick, total])

    const replay = () => {
        setTick(0)
        setPlaying(true)
    }
    // 정지 상태에서 한 tick씩 — 탐색 중엔 이벤트 하나, 실행 중엔 스텝 하나.
    const stepBy = (d: number) => {
        setPlaying(false)
        setTick((v) => Math.max(0, Math.min(total, v + d)))
    }

    const expandedCount = useMemo(
        () => timeline.expanded.filter((e) => e.step <= step).length,
        [timeline, step],
    )
    const pathsShown = useMemo(
        () => timeline.paths.filter((p) => p.step <= step).length,
        [timeline, step],
    )

    return (
        <div className="flex flex-col gap-2 items-center">
            <GridCanvas map={map} panel={panel} timeline={timeline} step={step} execStep={execStep}
                        agents={agents} onPaintCell={onPaintCell} onMoveAgent={onMoveAgent}/>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: panel}}>
                <Btn onClick={() => stepBy(-1)} label={t("step back one step", "한 스텝 뒤로")}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                        <path d="M7 5h2v14H7zM19 5v14l-8-7 8-7z"/>
                    </svg>
                </Btn>
                {playing
                    ? <Btn onClick={() => setPlaying(false)} label={t("pause", "일시정지")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M7 5h4v14H7zM13 5h4v14h-4z"/>
                        </svg>
                    </Btn>
                    : <Btn onClick={finished ? replay : () => setPlaying(true)} label={t("play", "재생")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M8 5v14l11-7z"/>
                        </svg>
                    </Btn>}
                <Btn onClick={() => stepBy(1)} label={t("step forward one step", "한 스텝 앞으로")}>
                    <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                        <path d="M5 5l8 7-8 7V5zM15 5h2v14h-2z"/>
                    </svg>
                </Btn>
                <input type="range" min={0} max={total} value={Math.min(tick, total)}
                       onChange={(e) => {
                           setPlaying(false)
                           setTick(parseInt(e.target.value))
                       }}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("timeline — search events, then execution steps",
                                    "타임라인 — 탐색 이벤트, 이어서 실행 스텝")}/>
                {onReset && (
                    <Btn onClick={onReset} label={t("reset the sandbox", "sandbox 초기화")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="none"
                             stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round"
                             aria-hidden="true">
                            <path d="M3 12a9 9 0 1 0 3-6.7"/>
                            <path d="M3 4v5h5"/>
                        </svg>
                    </Btn>
                )}
            </div>

            <div className="text-xs text-muted text-center tabular-nums">
                {t("expanded", "확장한 노드")}{" "}
                <span className="font-semibold" style={{color: "var(--accent)"}}>{expandedCount}</span>
                {" · "}{t("paths", "경로")}{" "}
                <span className="font-semibold">
                    {pathsShown}/{timeline.paths.length}
                </span>
                {!searching && timeline.makespan > 0 && (
                    <>
                        {" · "}{t("execution t", "실행 t")}{" "}
                        <span className="font-semibold">{execStep}/{timeline.makespan}</span>
                    </>
                )}
                {!searching && timeline.success === false && (
                    <>
                        {" · "}<span className="font-semibold">{t("no path", "경로 없음")}</span>
                    </>
                )}
            </div>

            {footer}
        </div>
    )
}

export default TracePlayer
