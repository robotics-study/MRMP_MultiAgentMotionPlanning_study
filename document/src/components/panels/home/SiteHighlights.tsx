import {useTr} from "../../../libs/i18n";

// 홈에서 "이 사이트가 무엇인가"를 사실만으로 세우는 카드 줄.
const SiteHighlights = () => {
    const t = useTr()

    const cards: Array<{kicker: string; title: string; desc: string}> = [
        {
            kicker: t("MAPF", "MAPF"),
            title: t("Paths that coexist", "공존하는 경로들"),
            desc: t(
                "Each agent gets its own space-time path: no two agents share a cell at the " +
                "same step, and nobody swaps cells across an edge. Prioritized planning, " +
                "joint-space search, and CBS — read in that lineage order.",
                "agent마다 자기 시공간(space-time) 경로가 있습니다. 같은 스텝에 두 agent가 " +
                "같은 셀을 차지하지 않고, 간선을 사이에 두고 자리를 맞바꾸지도 않습니다. " +
                "우선순위 계획, joint-space 탐색, CBS를 이 계보 순서로 읽습니다.",
            ),
        },
        {
            kicker: t("proofs", "증명"),
            title: t("Every property, proven", "모든 성질에 증명"),
            desc: t(
                "Optimality, completeness, and complexity claims come with step-by-step " +
                "proofs, not hand-waving — including why prioritized planning is incomplete.",
                "최적성·완전성·복잡도 주장은 말로 얼버무리지 않고 단계별 증명으로 뒷받침합니다. " +
                "우선순위 계획이 왜 불완전한지에 대한 증명까지요.",
            ),
        },
        {
            kicker: t("live demos", "라이브 데모"),
            title: t("Draw a wall. Watch the plan change.", "벽을 그리면 계획이 바뀝니다"),
            desc: t(
                "Every page runs the planner live in your browser — the exact mirror of the " +
                "Python/C++ engine. Draw walls, drag an agent's start or goal, add agents: every " +
                "edit re-plans and replays, showing expansions per agent, the vertex and edge " +
                "conflicts found, the constraints added, then every agent walking its space-time path.",
                "모든 페이지에서 planner를 브라우저에서 직접 돌립니다. 엔진은 Python/C++ 구현과 " +
                "필드 단위로 동일한 미러예요. 벽을 그리고, agent의 start/goal을 끄고, agent를 " +
                "추가하면 매 편집이 즉시 재계획·재생됩니다. agent별 확장, 발견된 vertex·edge " +
                "conflict, 걸리는 제약, 모든 agent가 시공간 경로를 걸어가는 순간까지.",
            ),
        },
        {
            kicker: t("full source", "전체 소스"),
            title: t("Read the real implementation", "실제 구현을 그대로 읽기"),
            desc: t(
                "Each page ends with the complete C++ and Python source that the explanations " +
                "describe. The live sandbox engine is a third mirror of the same planner, and all " +
                "three emit byte-identical traces on every scenario, verified on every build.",
                "각 페이지 끝에는 설명이 가리키는 C++·Python 구현 전체가 그대로 붙어 있습니다. " +
                "라이브 sandbox 엔진은 같은 planner의 세 번째 미러이고, 셋 모두 모든 시나리오에서 " +
                "필드 단위로 동일한 trace를 방출합니다. 빌드마다 검증됩니다.",
            ),
        },
    ]

    return (
        <div className="grid gap-4 sm:grid-cols-2 mb-12">
            {cards.map((c) => (
                <div key={c.title}
                     className="flex flex-col gap-2 rounded-[var(--radius)] border border-border bg-surface p-5 shadow-card">
                    <span className="text-xs font-bold uppercase tracking-wider"
                          style={{color: "var(--accent)"}}>{c.kicker}</span>
                    <span className="font-semibold" style={{fontSize: "1.02rem"}}>{c.title}</span>
                    <p className="m-0 text-sm text-muted leading-relaxed">{c.desc}</p>
                </div>
            ))}
        </div>
    )
}

export default SiteHighlights
