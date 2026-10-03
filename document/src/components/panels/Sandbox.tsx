import {ReactNode, useEffect, useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../CanvasFigure";
import TracePlayer from "../player/TracePlayer";
import {loadGridMap} from "../../libs/trace/load";
import {buildTimeline, MultiTimeline} from "../../libs/trace/timeline";
import {Cell, TraceEvent} from "../../libs/trace/types";
import {cellToWorld, freePoint, GridMap} from "../../libs/grid";
import {useTr} from "../../libs/i18n";
import {Localized} from "../../../types/global";

// 라이브 sandbox — 페이지의 알고리즘을 브라우저에서 직접 돌린다. 벽을 그리고,
// agent의 start/goal 핸들을 드래그하고, agent를 더하고 빼면 엔진이 즉시 다시
// 계획하고 재생이 처음부터 돈다. 저장소의 Python/C++ 구현과 필드 단위로 동일한
// 미러 엔진(libs/algorithms)이 그대로 실행된다 — recorded trace 재생은 어디에도 없다.

export interface ScenarioPreset {
    // 버튼 라벨 (수출 trace 시나리오 이름과 동일).
    name: string;
    // data/maps/<map>.json.
    map: string;
    // agent index 순서 그대로의 [start, goal] 셀 — 핸들도 항상 셀에 스냅되고
    // runLive 래퍼가 cellToWorld로 세계 좌표로 바꾼다(연속 알고리즘도 같은 픽셀).
    agents: Array<[Cell, Cell]>;
    // 연속 preset(dRRT 계열): 모든 agent disc의 반지름(미터). 있으면 핸들 드래그와
    // agent 추가가 디스크가 장애물과 겹치는 셀을 거부한다 (planner의 instance 판정 규칙).
    radius?: number;
    // timed preset(kinodynamic): agent index 순서별 속도 한계(칸/시간) — run에 그대로
    // 전달되고 agent 추가 시 1이 늘어난다. 칩 클릭으로 순환 변경하고, discrete 알고리즘은
    // run이 vmax를 무시한다.
    vmax?: number[];
    // 창 달린 알고리즘(winPIBT, RHCR)의 window 파라미터 기본값 — 있으면 창 칩이 렌더되고
    // 클릭마다 1 → 2 → 3으로 순환한다. 없는 알고리즘은 run이 네 번째 인자를 무시한다.
    window?: number;
    // db-CBS의 불연속성 경계 δ 기본값 — 있으면 δ 칩이 렌더되고 클릭마다 사다리를 순환한다.
    // floor(δ)가 값의 전부다: 0.5와 1은 같은 격자(floor 0 — 정확한 이중 적분자)이고
    // 1.5부터 착지 여유 한 칸이 열린다. 없는 알고리즘은 run이 다섯 번째 인자를 무시한다.
    delta?: number;
}

// 창 사다리 — 칩 클릭마다 순환. winPIBT에서 이 값이 곧 갈래의 축이다: w=1은 PIBT로
// 돌아가고, 커지면 prioritized planning에 가까워진다(pocket 시나리오가 칼날이 되는 지점).
// RHCR에서는 창 자체가 알고리즘의 축이다 — 좁히면 근시안이 연구 대상이 된다.
const WINDOW_LADDER = [1, 2, 3]

// db-CBS의 δ 사다리 — 칩 클릭마다 순환. floor(δ)가 값의 전부라 0.5 → 1은 동작이 전혀
// 같고(둘 다 floor 0), 1.5에서야 격자가 바뀐다 — 그 자체가 교훈이다.
const DELTA_LADDER = [0.5, 1, 1.5]

// timed preset의 속도 한계 사다리 — 칩 클릭마다 ×2로 순환(4를 넘으면 0.25로 돌아간다).
// 전부 이진 유리수라 모든 시간이 정확히 표현 가능한 double에 착지한다는 관례와 같다.
const VMAX_LADDER = [0.25, 0.5, 1, 2, 4]
const SUBSCRIPT = ["₀", "₁", "₂", "₃", "₄", "₅"]

export interface SandboxProps {
    presets: ScenarioPreset[];
    // 라이브 엔진 — Python/C++ planner의 정확한 미러 (libs/algorithms). timed 알고리즘은
    // 세 번째 인자(agent별 속도 한계)를, 창 달린 알고리즘은 네 번째 인자(창 크기)를,
    // db-CBS는 다섯 번째 인자(불연속성 경계 δ)를 쓰고 나머지는 무시한다. 엔진이 던지면
    // (db-CBS의 정수 vmax 거부처럼) 크래시가 아니라 error 카드로 보인다.
    run: (map: GridMap, tasks: Array<[Cell, Cell]>, vmax: number[], win: number,
          delta: number) => TraceEvent[];
    // agent 수 상한. 라이브 실행은 동기 호출이라 coupled(joint-space) 탐색은
    // agent가 늘면 |V|^k로 폭발한다 — 그 페이지는 2로 막는다 (그 자체가 교훈).
    maxAgents?: number;
    label: string;
    // 창 칩이 있는 알고리즘만 쓰는 칩 설명(언어 쌍) — 창이 무엇을 노브인지 각 페이지가 쓴다.
    windowHint?: Localized<string>;
    // δ 칩이 있는 알고리즘(db-CBS)만 쓰는 칩 설명 — 불연속성 경계가 무엇을 노브인지.
    deltaHint?: Localized<string>;
}

export const SandboxScene = ({presets, run, maxAgents = 6, panel = 340, windowHint, deltaHint}: {
    presets: ScenarioPreset[];
    run: (map: GridMap, tasks: Array<[Cell, Cell]>, vmax: number[], win: number,
          delta: number) => TraceEvent[];
    maxAgents?: number;
    panel?: number;
    windowHint?: Localized<string>;
    deltaHint?: Localized<string>;
}) => {
    const t = useTr()
    const [presetName, setPresetName] = useState(presets[0].name)
    // preset은 페이지가 고정 리터럴로 넘긴다 — presetName 선택만 재로딩을 촉발한다.
    const preset = presets.find((p) => p.name === presetName) ?? presets[0]
    const [map, setMap] = useState<GridMap | null>(null)
    const [error, setError] = useState<string | null>(null)
    // 리셋은 presetName이 이미 같아도 effect를 다시 촉발해야 하므로 별도 nonce를 돈다.
    const [nonce, setNonce] = useState(0)
    const copy = (a: Array<[Cell, Cell]>): Array<[Cell, Cell]> =>
        a.map(([s, g]) => [[s[0], s[1]], [g[0], g[1]]])
    const [agents, setAgents] = useState<Array<[Cell, Cell]>>(() => copy(preset.agents))
    // agent별 속도 한계 — agents와 같은 길이로 유지된다 (추가 시 1, 제거 시 함께 감소).
    // discrete 알고리즘은 run이 vmax를 읽지 않으므로 값은 무관하다.
    const [vmax, setVmax] = useState<number[]>(() => preset.vmax ?? preset.agents.map(() => 1))
    // 창 달린 알고리즘의 window — 나머지 알고리즘은 run이 네 번째 인자를 읽지 않는다.
    const [win, setWin] = useState<number>(() => preset.window ?? 1)
    // db-CBS의 δ — 나머지 알고리즘은 run이 다섯 번째 인자를 읽지 않는다.
    const [delta, setDelta] = useState<number>(() => preset.delta ?? 0.5)

    // preset 전환: 그 맵 JSON을 다시 읽고 endpoint·속도 한계·창·δ를 preset 기본값으로 되돌린다.
    useEffect(() => {
        let cancelled = false
        const p = presets.find((q) => q.name === presetName) ?? presets[0]
        setAgents(copy(p.agents))
        setVmax(p.vmax ?? p.agents.map(() => 1))
        if (p.window !== undefined) setWin(p.window)
        if (p.delta !== undefined) setDelta(p.delta)
        loadGridMap(`data/maps/${p.map}.json`).then((m) => {
            if (!cancelled) setMap(m)
        }).catch((e: unknown) => {
            if (!cancelled) setError(e instanceof Error ? e.message : String(e))
        })
        return () => {
            cancelled = true
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [presetName, nonce])

    // 라이브: map/agents/창/δ가 바뀔 때마다 엔진을 다시 돌리고 재생이 0부터 돈다. 엔진이
    // 던지는 거부(db-CBS의 비정수 vmax 같은)는 크래시가 아니라 error 카드다 — 거부된
    // 실행도 정직한 결과다.
    const outcome = useMemo<{ timeline: MultiTimeline } | { message: string } | null>(() => {
        if (!map) return null
        try {
            return {timeline: buildTimeline(run(map, agents, vmax, win, delta))}
        } catch (e) {
            return {message: e instanceof Error ? e.message : String(e)}
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [map, run, agents, vmax, win, delta])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            if (!prev) return prev
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }

    // 벽 위로는 옮길 수 없다 (캔버스의 페인팅 가드와 같은 규칙을 여기서도). 연속
    // preset은 disc가 장애물 셀과 strict overlap하는 위치도 거부 — planner의 instance
    // 판정을 드래그로 유도하지 않는다.
    const moveAgent = (agent: number, which: "start" | "goal", cell: Cell) => {
        if (!map || map.occupied[cell[0] * map.width + cell[1]]) return
        if (preset.radius !== undefined && !freePoint(map, cellToWorld(map, cell), preset.radius)) return
        setAgents((prev) => prev.map((a, k) => k !== agent ? a
            : (which === "start"
                ? [[cell[0], cell[1]] as Cell, a[1]]
                : [a[0], [cell[0], cell[1]] as Cell])))
    }

    // agent 추가: start는 행 우선로에서 endpoint로 쓰이지 않은 첫 빈 셀, goal은
    // 반대쪽 끝에서 같은 식으로 고른다(연속 preset은 disc가 들어가는 셀만). agent
    // 제거는 마지막을 뺀다 (최소 1명).
    const freeUnused = (fromEnd: boolean): Cell | null => {
        if (!map) return null
        const taken = new Set(agents.flat().map(([r, c]) => `${r},${c}`))
        const n = map.occupied.length
        for (let i = 0; i < n; i++) {
            const idx = fromEnd ? n - 1 - i : i
            if (map.occupied[idx]) continue
            const cell: Cell = [Math.floor(idx / map.width), idx % map.width]
            if (preset.radius !== undefined && !freePoint(map, cellToWorld(map, cell), preset.radius)) continue
            if (!taken.has(`${cell[0]},${cell[1]}`)) return cell
        }
        return null
    }
    const addAgent = () => {
        if (agents.length >= maxAgents) return
        const start = freeUnused(false)
        const goal = freeUnused(true)
        if (!start || !goal) return
        setAgents((prev) => [...prev, [start, goal]])
        setVmax((prev) => [...prev, 1])
    }
    const removeAgent = () => {
        setAgents((prev) => prev.length > 1 ? prev.slice(0, -1) : prev)
        setVmax((prev) => prev.length > 1 ? prev.slice(0, -1) : prev)
    }

    // 속도 칩 클릭 — 사다리를 한 칸 순환 (×2, 4 → 0.25로 wrap). 사다리 밖의 값은 1로 스냅.
    const cycleVmax = (agent: number) => setVmax((prev) => prev.map((v, k) => {
        if (k !== agent) return v
        const i = VMAX_LADDER.indexOf(v)
        return i < 0 ? 1 : VMAX_LADDER[(i + 1) % VMAX_LADDER.length]
    }))
    // 창 칩 클릭 — 사다리 [1, 2, 3]를 한 칸 순환.
    const cycleWin = () => setWin((prev) => {
        const i = WINDOW_LADDER.indexOf(prev)
        return i < 0 ? 1 : WINDOW_LADDER[(i + 1) % WINDOW_LADDER.length]
    })
    // δ 칩 클릭 — 사다리 [0.5, 1, 1.5]를 한 칸 순환 (floor가 값의 전부: 0.5와 1은 같은 격자).
    const cycleDelta = () => setDelta((prev) => {
        const i = DELTA_LADDER.indexOf(prev)
        return i < 0 ? 0.5 : DELTA_LADDER[(i + 1) % DELTA_LADDER.length]
    })

    const controls: ReactNode = (
        <div className="flex flex-col items-center gap-1.5 text-xs text-muted">
            <div className="flex items-center justify-center gap-1.5 flex-wrap">
                {presets.map((p) => (
                    <button key={p.name} type="button" onClick={() => setPresetName(p.name)}
                            className="px-2 py-0.5 rounded border font-mono"
                            style={p.name === presetName
                                ? {borderColor: "var(--accent)", color: "var(--accent)", fontWeight: 600}
                                : undefined}>
                        {p.name}
                    </button>
                ))}
                <span className="mx-1" aria-hidden="true">·</span>
                <button type="button" onClick={removeAgent} disabled={agents.length <= 1}
                        aria-label={t("remove an agent", "agent 하나 제거")}
                        className="px-2 py-0.5 rounded border border-border hover:bg-surface disabled:opacity-40">
                    −
                </button>
                <button type="button" onClick={addAgent} disabled={agents.length >= maxAgents}
                        aria-label={t("add an agent", "agent 하나 추가")}
                        className="px-2 py-0.5 rounded border border-border hover:bg-surface disabled:opacity-40">
                    +
                </button>
            </div>
            {preset.window !== undefined && (
                <div className="flex items-center justify-center gap-1.5 flex-wrap">
                    <button type="button" onClick={cycleWin}
                            aria-label={t("cycle the planning window (1 → 2 → 3)",
                                          "계획 창 순환 변경 (1 → 2 → 3)")}
                            className="px-1.5 py-0.5 rounded border border-border font-mono tabular-nums hover:bg-surface">
                        {`w=${win}`}
                    </button>
                </div>
            )}
            {preset.vmax !== undefined && (
                <div className="flex items-center justify-center gap-1.5 flex-wrap">
                    {vmax.map((v, i) => (
                        <button key={i} type="button" onClick={() => cycleVmax(i)}
                                aria-label={t(`cycle agent ${i}'s velocity limit (×2 per click)`,
                                              `agent ${i}의 속도 한계를 순환 변경 (클릭마다 ×2)`)}
                                className="px-1.5 py-0.5 rounded border border-border font-mono tabular-nums hover:bg-surface">
                            {`v${SUBSCRIPT[i]}=${v}`}
                        </button>
                    ))}
                </div>
            )}
            <div className="text-xs text-muted text-center">
                {t("drag cells to draw walls · drag the numbered dot and ring to move an agent's start/goal — " +
                    "playback is one clock over both phases: scrub anywhere with the slider, or step one event / one move at a time",
                    "셀을 드래그해 벽을 그리고, 번호가 적힌 점과 링을 끌어 agent의 start/goal을 옮겨 보라 — " +
                    "재생은 탐색과 실행 두 phase를 하나의 시계로 관통합니다. 슬라이더로 어디든 스크럽하고, ⏮/⏭으로 이벤트 하나·이동 한 칸씩 직접 밟아 보세요")}
            </div>
            {preset.vmax !== undefined && (
                <div className="text-xs text-muted text-center">
                    {t("velocity chips: every click doubles that agent's limit (4 wraps back to 0.25) — " +
                        "scale every vmax by the same factor and every time scales by its inverse; there is no absolute clock",
                        "속도 칩: 클릭할 때마다 그 agent의 한계가 두 배가 됩니다 (4를 넘으면 0.25로 돌아간다) — " +
                        "모든 vmax에 같은 인자를 걸면 모든 시간이 그 역수로만 스케일됩니다. 절대 시계는 없습니다")}
                </div>
            )}
            {preset.window !== undefined && windowHint && (
                <div className="text-xs text-muted text-center">{t(windowHint.en, windowHint.ko)}</div>
            )}
            {preset.delta !== undefined && (
                <div className="flex items-center justify-center gap-1.5 flex-wrap">
                    <button type="button" onClick={cycleDelta}
                            aria-label={t("cycle the discontinuity bound (0.5 → 1 → 1.5)",
                                          "불연속성 경계를 순환 변경 (0.5 → 1 → 1.5)")}
                            className="px-1.5 py-0.5 rounded border border-border font-mono tabular-nums hover:bg-surface">
                        {`δ=${delta}`}
                    </button>
                </div>
            )}
            {preset.delta !== undefined && deltaHint && (
                <div className="text-xs text-muted text-center">{t(deltaHint.en, deltaHint.ko)}</div>
            )}
        </div>
    )

    if (error || !map || !outcome) {
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                {error ? `${t("failed to load map", "맵 로드 실패")}: ${error}` : "Loading…"}
            </div>
            {controls}
        </div>
    }
    if ("message" in outcome) {
        // 엔진이 실행을 거부했다 (db-CBS의 정수 vmax 거부처럼) — 크래시가 아니라 결과다.
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                {`${t("the planner refused", "planner가 거부했습니다")}: ${outcome.message}`}
            </div>
            {controls}
        </div>
    }
    const timeline = outcome.timeline
    return <TracePlayer map={map} timeline={timeline} panel={panel}
                        agents={agents.map(([start, goal]) => ({start, goal}))}
                        onPaintCell={paintCell} onMoveAgent={moveAgent}
                        onReset={() => { setPresetName(preset.name); setNonce((n) => n + 1) }}
                        footer={controls}/>
}

const Sandbox = ({presets, run, maxAgents, label, windowHint, deltaHint}: SandboxProps) => {
    const size = modalCanvasSize(1)
    return <CanvasFigure
        label={label}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<SandboxScene presets={presets} run={run} maxAgents={maxAgents} windowHint={windowHint}
                             deltaHint={deltaHint} panel={Math.min(size.width, 640)}/>}
    >
        <SandboxScene presets={presets} run={run} maxAgents={maxAgents} windowHint={windowHint}
                      deltaHint={deltaHint}/>
    </CanvasFigure>
}

export default Sandbox
