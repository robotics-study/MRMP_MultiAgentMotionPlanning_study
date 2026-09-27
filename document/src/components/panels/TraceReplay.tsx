import {useEffect, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../CanvasFigure";
import TracePlayer from "../player/TracePlayer";
import {loadGridMap, loadTrace} from "../../libs/trace/load";
import {buildTimeline, MultiTimeline} from "../../libs/trace/timeline";
import {GridMap} from "../../libs/grid";
import {useTr} from "../../libs/i18n";
import cn from "../../libs/cn";

// 저장소의 실제 데모가 방출한 trace를 재생하는 공용 패널. 어떤 알고리즘이든
// spec/trace_schema.json 이벤트 계약만 지키면 이 하나로 재생된다 — 시각화 계층은
// 알고리즘 내부를 전혀 만지지 않는다. C++/Python 데모는 동일한 이벤트 열을 방출하므로
// 웹에는 한 벌(py)만 싣는다.
//
// nav_study와 달리 trace는 시나리오 이름으로 키를 잡는다 (한 맵에 여러 시나리오).
// 각 시나리오는 자기 맵 JSON을 데려온다 — 데모가 world 좌표를 cell로 변환한 결과가
// 그대로 재생 대상이 된다.
export interface ScenarioSource {
    // traces/<algo>/<scenario>.py.jsonl.gz 의 scenario 이름.
    scenario: string;
    // 그 시나리오가 실행된 맵 (data/maps/<map>.json).
    map: string;
}

interface Loaded {
    map: GridMap;
    timeline: MultiTimeline;
}

const ReplayScene = ({algo, sources, panel = 340}: {
    algo: string; sources: ScenarioSource[]; panel?: number;
}) => {
    const t = useTr()
    const [scenario, setScenario] = useState(sources[0].scenario)
    const [loaded, setLoaded] = useState<Loaded | null>(null)
    const [error, setError] = useState<string | null>(null)

    useEffect(() => {
        let cancelled = false
        setLoaded(null)
        setError(null)
        const source = sources.find((s) => s.scenario === scenario)!
        Promise.all([
            loadGridMap(`data/maps/${source.map}.json`),
            loadTrace(`data/traces/${algo}/${scenario}.py.jsonl.gz`),
        ]).then(([map, events]) => {
            if (cancelled) return
            setLoaded({map, timeline: buildTimeline(events)})
        }).catch((e: unknown) => {
            if (!cancelled) setError(e instanceof Error ? e.message : String(e))
        })
        return () => {
            cancelled = true
        }
        // sources는 페이지에서 고정 리터럴로 넘긴다 — 시나리오 선택만 재로딩을 촉발한다.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [algo, scenario])

    // 시나리오 선택자 — 버튼 라벨은 scenario 이름 그대로 (파일명과 동일).
    const selector = (
        <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
            {sources.length > 1 && sources.map((s) => (
                <button key={s.scenario} type="button" onClick={() => setScenario(s.scenario)}
                        className={cn(
                            "px-2 py-0.5 rounded border font-mono",
                            scenario === s.scenario
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {s.scenario}
                </button>
            ))}
        </div>
    )

    if (error || !loaded) {
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                {error ? `${t("failed to load trace", "trace 로드 실패")}: ${error}` : "Loading…"}
            </div>
            {selector}
        </div>
    }
    return <TracePlayer map={loaded.map} timeline={loaded.timeline} footer={selector}/>
}

const TraceReplay = ({algo, sources, label}: {
    algo: string; sources: ScenarioSource[]; label: string;
}) => {
    // 시나리오마다 맵 크기가 다를 수 있어 aspect는 1 고정 (모두 정사각 격자).
    const size = modalCanvasSize(1)
    return <CanvasFigure
        label={label}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<ReplayScene algo={algo} sources={sources} panel={Math.min(size.width, 640)}/>}
    >
        <ReplayScene algo={algo} sources={sources}/>
    </CanvasFigure>
}

export default TraceReplay
