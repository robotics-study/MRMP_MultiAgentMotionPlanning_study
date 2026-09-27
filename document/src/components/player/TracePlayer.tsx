import {ReactNode, useEffect, useMemo, useState} from "react";
import {GridMap} from "../../libs/grid";
import {MultiTimeline} from "../../libs/trace/timeline";
import GridCanvas from "../2d/GridCanvas";
import {useTr} from "../../libs/i18n";
import cn from "../../libs/cn";

// 탐색 재생: 이벤트 수와 무관하게 체감 속도를 맞춘다 (고정 배속). 탐색이 끝나면
// 실행 재생으로 넘어가 각 agent가 자기 space-time 경로를 스텝마다 걸어간다.
const SEARCH_MS = 3000;
const TICK_MS = 30;
const EXEC_STEP_MS = 250;

interface TracePlayerProps {
    map: GridMap;
    timeline: MultiTimeline;
    autoPlay?: boolean;
    // 플레이어 아래 추가 컨트롤 (demo 페이지의 설명 등).
    footer?: ReactNode;
}

const Btn = ({onClick, label, children, active}: {
    onClick: () => void; label: string; children: ReactNode; active?: boolean
}) => (
    <button type="button" onClick={onClick} aria-label={label}
            className={cn(
                "px-1.5 py-1 rounded border border-border hover:bg-surface leading-none",
                active && "border-[var(--accent)] text-[var(--accent)]",
            )}>
        {children}
    </button>
)

const TracePlayer = ({map, timeline, autoPlay = true, footer}: TracePlayerProps) => {
    const t = useTr()
    // step: 이벤트 prefix(탐색 재생). execStep: 실행 재생 스텝 τ (null = 탐색 단계).
    const [step, setStep] = useState(autoPlay ? 0 : timeline.steps)
    const [playing, setPlaying] = useState(autoPlay)
    const [execStep, setExecStep] = useState<number | null>(autoPlay ? null : timeline.makespan)
    const finished = step >= timeline.steps

    // 타임라인이 바뀌면(트레이스 교체) 처음부터 다시 재생한다.
    useEffect(() => {
        setStep(autoPlay ? 0 : timeline.steps)
        setPlaying(autoPlay)
        setExecStep(autoPlay ? null : timeline.makespan)
    }, [timeline, autoPlay])

    // 탐색 단계: step을 고정 배속으로 굴린다.
    useEffect(() => {
        if (!playing || finished) return
        const perTick = Math.max(1, Math.round(timeline.steps / (SEARCH_MS / TICK_MS)))
        const timer = window.setInterval(() => {
            setStep((s) => Math.min(timeline.steps, s + perTick))
        }, TICK_MS)
        return () => window.clearInterval(timer)
    }, [playing, finished, timeline])

    // 실행 단계: 탐색이 끝나면 τ를 0 → makespan으로 고정 스텝 속도로 굴린다.
    useEffect(() => {
        if (!playing || !finished) return
        let cur = 0
        const timer = window.setInterval(() => {
            cur += 1
            if (cur >= timeline.makespan) {
                window.clearInterval(timer)
                setPlaying(false)
                setExecStep(timeline.makespan)
            } else {
                setExecStep(cur)
            }
        }, EXEC_STEP_MS)
        return () => window.clearInterval(timer)
    }, [playing, finished, timeline])

    const expandedCount = useMemo(
        () => timeline.expanded.filter((e) => e.step <= step).length,
        [timeline, step],
    )
    const pathsShown = useMemo(
        () => timeline.paths.filter((p) => p.step <= step).length,
        [timeline, step],
    )

    const replay = () => {
        setStep(0)
        setExecStep(null)
        setPlaying(true)
    }

    return (
        <div className="flex flex-col gap-2 items-center">
            <GridCanvas map={map} panel={340} timeline={timeline} step={step} execStep={execStep}/>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: 340}}>
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
                <input type="range" min={0} max={timeline.steps} value={Math.min(step, timeline.steps)}
                       onChange={(e) => {
                           setPlaying(false)
                           const v = parseInt(e.target.value)
                           setStep(v)
                           // 탐색 끝까지 스크럽하면 실행 재생은 최종 자세로 고정한다.
                           setExecStep(v >= timeline.steps ? timeline.makespan : null)
                       }}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("search progress", "탐색 진행")}/>
            </div>

            <div className="text-xs text-muted text-center tabular-nums">
                {t("expanded", "확장한 노드")}{" "}
                <span className="font-semibold" style={{color: "var(--accent)"}}>{expandedCount}</span>
                {" · "}{t("paths", "경로")}{" "}
                <span className="font-semibold">
                    {pathsShown}/{timeline.paths.length}
                </span>
                {execStep !== null && timeline.makespan > 0 && (
                    <>
                        {" · "}{t("execution t", "실행 t")}{" "}
                        <span className="font-semibold">{execStep}/{timeline.makespan}</span>
                    </>
                )}
                {finished && timeline.success === false && (
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
