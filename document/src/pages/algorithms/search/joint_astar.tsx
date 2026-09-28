import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runJointAStar} from "../../../libs/algorithms/joint_astar";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/search/joint_astar.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/search/joint_astar.hpp?raw";
import cppImpl from "../../../../../cpp/src/search/joint_astar.cpp?raw";

const REPO = "https://github.com/robotics-study/MRMP_MultiAgentMotionPlanning_study"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 이 planner는 파라미터가 없다.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runJointAStar(map, tasks, {})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. agent 수는 2로
// 고정이다: 상태 공간이 |V|^k로 폭발하는 planner를 브라우저에서 실시간으로 돌리는
// 상한이 바로 그 지수다 (3번째 agent를 못 고르는 게 아니라, 그게 페이지의 논지).
const PRESETS: ScenarioPreset[] = [
    {name: "maze01_two", map: "maze01", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
    {name: "open01_cross", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
]

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const JointAStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The previous page rescued the single-robot planner with an <em>order</em>, and paid
                    for it: a fixed priority is neither optimal nor complete. What does the honest
                    baseline look like — the thing every decoupled and hybrid method is measured
                    against? Take the very same A* from the single-robot pages and refuse to plan one
                    agent at all. Instead, treat <InlineMath math="k"/> positions as <em>one</em>{" "}
                    state, search over that product space once, and let a single optimal search decide
                    who waits and who moves. This is <em>joint-space A*</em>: complete and sum-of-costs
                    optimal by construction, paid for with a state space of{" "}
                    <InlineMath math="|V|^k"/>. It has no founding paper — it is what MAPF search is
                    when you apply A* (Hart, Nilsson & Raphael, 1968) to the product graph directly,
                    the coupled pole of the survey's taxonomy.
                </p>}
                ko={<p>
                    이전 페이지는 단일 robot planner를 <strong>순서</strong>로 살렸다. 그리고 대가를
                    치렀다. 고정된 우선순위는 최적도 완전도 아니다. 그러면 정직한 baseline은 어떤 모습일까.
                    모든 decoupled와 hybrid가 여기에 대해 저울질되는 그것. 이전 페이지들의 그 A*를 그대로
                    가져오되, 아예 agent별로 계획하는 것을 포기한다. <InlineMath math="k"/>개의 위치를
                    <em>하나</em>의 상태로 다루고, 그 product space를 한 번 탐색하고, 누가 기다리고 누가
                    갈지는 단일 최적 탐색에 맡긴다. 이것이 <strong>joint-space A*</strong>이다. 구성상
                    완전하고 sum-of-costs 최적이며, 대가는 <InlineMath math="|V|^k"/>의 상태 공간이다.
                    창간 논문은 없다. A*(Hart, Nilsson & Raphael, 1968)를 product graph에 그대로 적용한
                    것이 곧 이것이고, survey 분류 체계에서 coupled 극단이 그것이다.
                </p>}
            />

            <h2>{t("From Priorities to the Joint State", "우선순위에서 joint 상태로")}</h2>
            <T
                en={<>
                    <p>
                        The state of the search is no longer a cell but a tuple. Write{" "}
                        <InlineMath math="s = (c_0, \dots, c_{k-1}) \in V^k"/> for the positions of all{" "}
                        <InlineMath math="k"/> agents at one instant. A joint action moves every agent
                        simultaneously — each unarrived agent takes one move or wait, and a transition is
                        legal only if no two agents land on the same cell (vertex conflict) and no pair
                        swaps cells across the step (edge conflict):
                    </p>
                    <BlockMath math="s \to s' \iff \bigl(\forall i \ne j:\; c_i' \ne c_j'\bigr) \;\wedge\; \neg\bigl(\exists i \ne j:\; c_i' = c_j \wedge c_j' = c_i\bigr)"/>
                    <Terms items={[
                        ["s", <>joint 상태. 좌표가 한 칸이라도 다른 순간은 서로 다른 상태다. 상태 공간은{" "}
                            <InlineMath math="|V|^k"/>이고, 이것이 이 페이지의 모든 것이다</>],
                        ["\\text{goal}", <>goal tuple <InlineMath math="(g_0, \dots, g_{k-1})"/>. 좌표가
                            자기 goal에 도달한 agent는 <em>arrived</em>이고, 그 순간부터 유일한 행동은
                            self-loop이다 (stay-at-goal이 구조로 인코딩된다)</>],
                        ["\\delta_i", <>agent <InlineMath math="i"/>의 부분 행동. 도착 전이면 up/down/left/right/wait
                            중 하나, 도착한 뒤에는 자기 셀에서의 self-loop 하나</>],
                    ]}/>
                    <p>
                        Cost needs one more decision, and it decides what “optimal” means. Every step,
                        each still-unarrived agent pays 1; an arrived agent pinned on its goal pays
                        nothing. One joint step therefore costs the number of unarrived agents, which
                        telescopes into exactly the same objective as the prioritized page: the sum of
                        individual arrival times <InlineMath math="\sum_k T_k"/>. The two algorithms now
                        optimize literally the same function — what separates them is only which plans
                        they are able to find.
                    </p>
                </>}
                ko={<>
                    <p>
                        탐색의 상태가 더 이상 셀 하나가 아니라 tuple이다. 한 순간에 모든 agent{" "}
                        <InlineMath math="k"/>개의 위치를 <InlineMath math="s = (c_0, \dots, c_{k-1}) \in V^k"/>로
                        쓴다. joint action은 모든 agent를 동시에 움직이게 한다. 도착하지 않은 agent마다
                        이동이나 대기를 하나씩 취하고, 전이는 두 agent가 같은 셀에 착지하지 않고(vertex
                        conflict) 짝이 스텝을 사이에 두고 셀을 맞바꾸지도 않을 때(edge conflict)에만 합법이다:
                    </p>
                    <BlockMath math="s \to s' \iff \bigl(\forall i \ne j:\; c_i' \ne c_j'\bigr) \;\wedge\; \neg\bigl(\exists i \ne j:\; c_i' = c_j \wedge c_j' = c_i\bigr)"/>
                    <Terms items={[
                        ["s", <>joint 상태. 좌표가 한 칸이라도 다른 순간은 서로 다른 상태다. 상태 공간은{" "}
                            <InlineMath math="|V|^k"/>이고, 이것이 이 페이지의 전부다</>],
                        ["\\text{goal}", <>goal tuple <InlineMath math="(g_0, \dots, g_{k-1})"/>. 자기 goal에
                            도달한 agent는 <em>arrived</em>이고 그 순간부터 유일한 행동은 self-loop이다
                            (stay-at-goal이 구조로 인코딩된다)</>],
                        ["\\delta_i", <>agent <InlineMath math="i"/>의 부분 행동. 도착 전이면 up/down/left/right/wait
                            중 하나, 도착 뒤에는 자기 셀에서의 self-loop 하나</>],
                    ]}/>
                    <p>
                        비용에 결정이 하나 남았고, 그것이 “최적”의 의미를 결정한다. 모든 스텝에서 아직
                        도착하지 않은 agent가 1을 지불하고, 도착해서 goal에 고정된 agent는 아무것도 지불하지
                        않는다. 따라서 joint 스텝 하나의 비용은 도착하지 않은 agent의 수이고, 이는 telescope되어
                        우선순위 페이지와 정확히 같은 목적함수로 수렴한다: 개별 도착 시간의 합{" "}
                        <InlineMath math="\sum_k T_k"/>. 이제 두 알고리즘이 문자 그대로 같은 함수를 최적화한다.
                        둘을 가르는 것은 오직 어떤 계획을 찾을 <em>수 있느냐</em>뿐이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Complete.</strong> The product graph is finite and the search explores
                            it exhaustively. If a conflict-free joint plan exists, one is found; if the
                            frontier empties, that is a verdict that no plan exists — not an artifact of a
                            lucky or unlucky priority order.
                        </li>
                        <li>
                            <strong>Sum-of-costs optimal.</strong> A* with an admissible heuristic on the
                            product graph returns a shortest legal joint path, and by the cost identity
                            above that is exactly the minimum of{" "}
                            <InlineMath math="\sum_k T_k"/>. The demo scenarios prove the point: every
                            scenario where prioritized planning got lucky has the same cost here — now as
                            a theorem rather than luck.
                        </li>
                        <li>
                            <strong>Exponential in the number of agents.</strong> The state space is{" "}
                            <InlineMath math="|V|^k"/>: two agents on this site's 20×20 map already mean a
                            few hundred free cells squared. On the open map with a perfect heuristic the
                            coupled search expands fewer nodes than two sequential searches (18 vs 35), but
                            in the corridor maze it explodes — <em>10,344</em> joint expansions against
                            prioritized planning's 1,375, for the exact same cost of 66. That is the whole
                            trade: optimality and completeness, paid for in state space.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>완전 (complete).</strong> product graph는 유한하고 탐색은 그것을 남김없이
                            탐색한다. conflict-free joint 계획이 존재하면 반드시 찾는다. frontier가 비면 그건
                            계획이 없다는 <em>판정</em>이다. 운 좋은/나쁜 우선순위 순서의 산물이 아니다.
                        </li>
                        <li>
                            <strong>Sum-of-costs 최적.</strong> admissible heuristic을 쓴 A*가 product graph에서
                            최단 합법 joint 경로를 반환하고, 위의 비용 항등식에 의해 그것은 정확히{" "}
                            <InlineMath math="\sum_k T_k"/>의 최소값이다. 데모 시나리오가 이를 증명한다:
                            우선순위 계획이 운 좋게 이겼던 모든 시나리오의 비용이 여기서도 동일하다. 이제 그건
                            운이 아니라 정리다.
                        </li>
                        <li>
                            <strong>agent 수에 지수.</strong> 상태 공간은 <InlineMath math="|V|^k"/>다. 이 사이트의
                            20×20 맵에서 agent 두 대만 해도 자유 셀 몇백 개의 제곱이다. heuristic이 완벽한 열린
                            맵에서는 coupled 탐색이 순서대로 두 번 탐색하는 것보다 적게 확장한다(18 vs 35). 그러나
                            통로 미로에서는 폭발한다. 정확히 같은 비용 66에 대해 prioritized의 1,375개 대비{" "}
                            <em>10,344</em>개의 joint 확장. 거래 전체가 이렇다: 최적성과 완전성을 상태 공간과 맞바꾼다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The procedure is one A* loop over joint states — the multi-agent content lives entirely
                    in what a state and a legal transition <em>are</em>, not in the search machinery. Note
                    that time never appears in the state: arrival times live only in the reconstructed
                    paths, which is why the trace's expansions carry no timestep field.
                </p>}
                ko={<p>
                    절차는 joint 상태 위의 A* 루프 하나다. multi-agent 내용은 오직 상태와 합법 전이가
                    <em>무엇이냐</em>에 들어 있고, 탐색 메커니즘에는 아무것도 없다. 시간은 상태에 전혀
                    등장하지 않는다는 것에 주목하라. 도착 시간은 오직 재구성된 경로에만 살고, 그래서 trace의
                    확장 이벤트에는 시각 필드가 없다.
                </p>}
            />
            <Pseudocode code={`if start_i = start_j for some i ≠ j: return failure                      # 1
s ← (start_0, …, start_{k−1});  g[s] ← 0;  OPEN ← {(h(s), seq₁)}   # 2
while OPEN not empty:
    s ← pop_min(OPEN); if s ∈ CLOSED skip, else CLOSED += {s}       # 3
    emit node_expanded(flatten(s), g[s])                             # 4
    if s = (goal_0, …, goal_{k−1}): reconstruct chain, return paths  # 5
    actions_i ← {c_i}          if c_i = goal_i   (arrived: pin)      # 6
              ← neighbors(c_i) otherwise         (up/down/left/right/wait)
    for δ ∈ actions_0 × ⋯ × actions_{k−1} in product order           # 7
        skip δ on vertex or edge conflict                            # 8
        g₂ ← g[s] + #{i : c_i ≠ goal_i}                              # 9
        push (g₂ + h(succ), next seq) iff g₂ < g[succ]               # 10`}
    />
            <T
                en={<ol>
                    <li>Two agents sharing a cell at <InlineMath math="t=0"/> is an unsolvable instance:
                        no joint state can separate them. The search reports failure without expanding
                        anything — a verdict on the instance, not on the algorithm.</li>
                    <li>The start state is the tuple of starts;{" "}
                        <InlineMath math="h(s) = \sum_k d(c_k, g_k)"/> (Manhattan per agent). The frontier
                        is a min-heap on <InlineMath math="(f, \text{seq})"/> — ties broken by push order,{" "}
                        exactly as in every other page of this repository.</li>
                    <li>Lazy deletion: a state popped for the first time is expanded once and never again.
                        The heuristic is consistent, so the first pop already carries optimal{" "}
                        <InlineMath math="g"/> (proof below).</li>
                    <li>The expansion emits one trace event carrying the flattened joint state — no agent
                        field, no timestep field: a joint state has neither.</li>
                    <li>Popping the goal tuple ends the search. The parent chain, reversed, is the joint
                        path; agent <InlineMath math="k"/>’s space-time path is its coordinate sequence up
                        to its first arrival (after which it pinned anyway).</li>
                    <li>An arrived agent's action list is the single self-loop on its goal cell. This is
                        stay-at-goal semantics made structural — no guard needed, pinning is the only move.</li>
                    <li>Successors are the cartesian product of the per-agent action lists, enumerated in
                        a fixed order (agent index outermost, last agent fastest) so every language replays
                        identical expansions.</li>
                    <li>A combination that repeats a cell (vertex) or swaps a pair across the step (edge)
                        is not an edge of the product graph. It is skipped, not repaired.</li>
                    <li>The step cost is the number of still-unarrived agents — the sum-of-costs objective
                        accumulated one step at a time.</li>
                    <li>A popped state's <InlineMath math="g"/> never improves afterwards (consistency), so
                        a successor already settled at a better <InlineMath math="g"/> is simply not pushed.
                        The frontier may still hold stale entries; they are skipped at pop, which is what
                        “lazy deletion” means.</li>
                </ol>}
                ko={<ol>
                    <li><InlineMath math="t=0"/>에 두 agent가 한 셀을 공유하면 unsolvable한 instance다. 어떤
                        joint 상태도 그들을 분리할 수 없다. 탐색은 아무것도 확장하지 않고 실패를 보고한다.
                        알고리즘의 문제가 아니라 instance에 대한 판정이다.</li>
                    <li>시작 상태는 start들의 tuple이고{" "}
                        <InlineMath math="h(s) = \sum_k d(c_k, g_k)"/> (agent별 Manhattan). frontier는{" "}
                        <InlineMath math="(f, \text{seq})"/> 기준 min-heap. 동률은 push 순서로 깨고, 이 저장소의
                        다른 모든 페이지와 정확히 같다.</li>
                    <li>lazy deletion: 처음으로 pop된 상태는 한 번 확장되고 다시는 오지 않는다. heuristic이
                        consistent하므로 첫 pop이 이미 최적 <InlineMath math="g"/>를 싣고 있다 (증명은 아래).</li>
                    <li>확장은 평면화된 joint 상태를 실은 trace 이벤트 하나를 방출한다. agent 필드도 시각
                        필드도 없다. joint 상태에는 그 둘 다 없으니까.</li>
                    <li>goal tuple의 pop이 탐색을 끝낸다. parent chain을 역전시키면 joint 경로이고, agent{" "}
                        <InlineMath math="k"/>의 시공간 경로는 FIRST 도착까지의 좌표 수열이다 (그 뒤로 어차피
                        고정됐으므로).</li>
                    <li>도착한 agent의 행동 목록은 goal 셀 위의 self-loop 하나뿐이다. stay-at-goal semantics가
                        구조로 인코딩된 것이다. 가드 같은 것은 필요 없다. 고정이 유일한 행동이다.</li>
                    <li>후속 상태는 agent별 행동 목록들의 cartesian product이고 고정 순서(agent index가 바깥,
                        마지막 agent가 가장 빨리)로 열거한다. 그래서 모든 언어가 동일한 확장을 재생산한다.</li>
                    <li>셀을 반복하는 combo(vertex)나 짝이 스텝을 가로질러 맞바꾸는 combo(edge)는 product graph의
                        간선이 아니다. 수리하지 않고 건너뛴다.</li>
                    <li>스텝 비용은 아직 도착하지 않은 agent의 수. sum-of-costs 목적함수를 스텝마다 쌓아간다.</li>
                    <li>pop된 상태의 <InlineMath math="g"/>는 이후로 절대 개선되지 않는다 (consistency). 그래서
                        더 나은 <InlineMath math="g"/>로 settle된 후속은 그냥 push되지 않는다. frontier에 stale
                        항목이 남아 있을 수 있고 pop 때 건너뛰는데, 이것이 “lazy deletion”의 뜻이다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    This is the baseline, so the guarantees are the textbook ones and the failure is only
                    scalability. Expand the proofs for why the lazy-deletion search is sound without ever
                    re-opening a state, and why per-step “count the unarrived” really accumulates to the sum
                    of arrival times that the prioritized page optimized.
                </p>}
                ko={<p>
                    baseline이니 보장은 교과서적인 것이고 실패는 확장성 하나뿐이다. 접힌 증명을 펼치면, 상태를
                    다시 열지 않는 lazy-deletion 탐색이 왜 sound인지, 그리고 스텝마다 “도착하지 않은 수를 세는”
                    것이 어떻게 우선순위 페이지가 최적화한 도착 시간의 합으로 정확히 쌓이는지 알 수 있다.
                </p>}
            />
            <Proof title={t("Theorem (optimality and completeness)", "정리 (최적성과 완전성)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> The search returns a plan minimizing{" "}
                            <InlineMath math="\sum_k T_k"/> over all conflict-free joint plans, or reports
                            failure only when no such plan exists.
                        </p>
                        <p>
                            <strong>Proof.</strong> The legal transitions define an implicit graph on{" "}
                            <InlineMath math="V^k"/> whose edges carry the step costs above; a conflict-free
                            joint plan is exactly a path from the start tuple to the goal tuple, and its
                            cost telescopes: each agent contributes 1 per step until it arrives and 0 after,
                            so a path of joint steps costs{" "}
                            <InlineMath math="\sum_k T_k"/>. On that graph,{" "}
                            <InlineMath math="h(s) = \sum_k d(c_k, g_k)"/> is admissible (each unarrived agent
                            needs at least its Manhattan distance in further steps, each paying 1) and
                            consistent (one joint step changes any single coordinate by at most one cell). A*
                            with a consistent heuristic pops every state at its optimal{" "}
                            <InlineMath math="g"/> on the first pop — that is exactly why lazy deletion needs
                            no re-opening: a later discovery can never undercut the settled value. So popping
                            the goal tuple yields an optimal joint path, and reconstructing through the parent
                            chain converts it back into per-agent space-time paths with identical cost. If the
                            frontier empties instead, every reachable joint state was settled; a plan would be
                            a path to the goal tuple, hence its states were reachable — contradiction. The
                            empty frontier is therefore sound evidence of unsolvability.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 이 탐색은 conflict-free joint 계획 전체에 대해{" "}
                            <InlineMath math="\sum_k T_k"/>를 최소화하는 계획을 반환하고, 그런 계획이 없을 때에만
                            실패를 보고한다.
                        </p>
                        <p>
                            <strong>증명.</strong> 합법 전이는 위의 스텝 비용을 실은 암시적 그래프를{" "}
                            <InlineMath math="V^k"/> 위에 정의하고, conflict-free joint 계획은 시작 tuple에서 goal
                            tuple까지의 경로와 정확히 같다. 그리고 그 비용은 telescope한다: 각 agent는 도착할 때까지
                            스텝당 1을 내고 이후 0을 내므로, joint 스텝들의 경로의 비용은{" "}
                            <InlineMath math="\sum_k T_k"/>다. 그 그래프에서{" "}
                            <InlineMath math="h(s) = \sum_k d(c_k, g_k)"/>는 admissible하다 (도착하지 않은 agent마다
                            자기 Manhattan 거리만큼의 스텝이 더 필요하고 각 스텝은 1을 지불한다). 그리고 consistent하다
                            (joint 스텝 하나는 어떤 좌표 하나를 최대 한 칸 바꾼다). consistent heuristic의 A*는 모든
                            상태를 첫 pop에서 최적 <InlineMath math="g"/>로 pop한다. 그래서 lazy deletion은 재확장이
                            필요 없다: 나중 발견이 settle된 값을 절대 밑돌 수 없으니까. 따라서 goal tuple의 pop은 최적
                            joint 경로를 주고, parent chain을 거슬러 재구성하면 비용이 같은 agent별 시공간 경로로
                            돌아온다. 반대로 frontier가 비면, 도달 가능한 모든 joint 상태가 settle된 것이다. 계획이
                            있었다면 goal tuple까지의 경로가 있었고 그 상태들은 도달 가능했을 테니 모순. 따라서 빈
                            frontier는 unsolvability의 sound한 증거다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The same three scenarios as the prioritized page, now live — and this sandbox is capped
                    at two agents on purpose. The state space is <InlineMath math="|V|^k"/>: a third agent
                    isn't a missing feature, it's what the exponential means. Watch the difference in what
                    “solving” means: every agent moves inside one search at once (the expansion bloom covers
                    all agents simultaneously — a joint state is being expanded, not an individual path), and
                    the finished paths are jointly optimal by construction. The costs match the prioritized
                    page exactly (66 / 33 / 30): on these three scenarios the priority order happened to
                    already be optimal. What differs is the price — <code>maze01_two</code> needed 10,344
                    joint expansions where prioritized needed 1,375. On <code>open01_cross</code>, where the
                    Manhattan heuristic points straight down both corridors, the coupled search is actually
                    cheaper (18 vs 35). The lesson isn't “worse”; it's that coupling pays in lockstep with
                    how much the agents must reason about each other.
                </p>}
                ko={<p>
                    우선순위 페이지와 동일한 세 시나리오를 이제 라이브로 돌린다 — 그리고 이 sandbox는
                    일부러 agent 2명으로 막아 두었다. 상태 공간이 <InlineMath math="|V|^k"/>이기 때문이다.
                    agent 3번을 못 고르게 한 게 기능이 아니라, 그게 지수의 의미다. “푼다”는 것의 의미가
                    어떻게 다른지 지켜보라. 모든 agent가 하나의 탐색 안에서 동시에 움직인다. 확장 bloom이
                    모든 agent에 동시에 피어나고(개별 경로가 joint 상태가 확장되는 것이다), 완성된 경로는 구성상
                    jointly optimal이다. 비용은 우선순위 페이지와 정확히 일치한다(66 / 33 / 30). 이 세 시나리오에서
                    우선순위 순서는 우연히 이미 최적이었다. 다른 것은 대가다. <code>maze01_two</code>는 joint 확장
                    10,344개가 필요했고 prioritized는 1,375개였다. Manhattan heuristic이 두 통로를 곧장 가리키는{" "}
                    <code>open01_cross</code>에서는 오히려 coupled 탐색이 더 싸다(18 vs 35). 교훈은 “더 나쁘다”가
                    아니다. 결합은 agent들이 서로에 대해 얼마나 추론해야 하는지에 비례해서 대가를 청구한다는 것이다.
                </p>}
            />
            <Sandbox maxAgents={2} label={t(
                "Live joint_astar sandbox — byte-identical to the Python/C++ planner, capped at two agents because |V|^k is the lesson. Draw walls, drag endpoints; every edit re-plans and replays",
                "라이브 joint_astar sandbox — Python/C++ planner와 바이트 단위로 동일하고, |V|^k가 논지이므로 agent 2명으로 막혀 있습니다. 벽을 그리고 endpoint를 끄면 모든 편집이 즉시 재계획·재생됩니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    Python and C++ mirror each other line for line, and the browser engine that powers the
                    live sandbox above — and backs parity checking — is a third mirror: same heap tie-breaks
                    (the push counter starts at 1), same{" "}
                    <InlineMath math="(\text{agent order}, \text{last fastest})"/> product enumeration, same
                    flattened joint-state events — so all three produce byte-identical traces on every
                    scenario, which <code>check-engine-parity</code> verifies on every build. The code below
                    is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이
                    세 번째 미러다. parity 검사도 이 엔진이 뒷받침한다. 동일한 heap tie-break(push 카운터는 1부터 시작), 동일한{" "}
                    <InlineMath math="(\text{agent 순서}, \text{마지막이 가장 빠름})"/> product 열거, 동일한 평면화된
                    joint 상태 이벤트. 그래서 셋 모두 모든 시나리오에서 바이트 단위로 동일한 trace를 만들고,{" "}
                    <code>check-engine-parity</code>가 빌드마다 이를 검증한다. 아래 코드는 발췌가 아니라 실제 소스
                    그대로다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/search/joint_astar.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/search/joint_astar.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/search/joint_astar.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/search/joint_astar.hpp`,
                            },
                            {
                                name: "cpp/src/search/joint_astar.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/search/joint_astar.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The planner and its C++ mirror, embedded from the repository sources",
                    "planner와 그 C++ 미러. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <T en={<p>
                The joint-state formulation has no founding paper — it is A* applied to the product of the
                per-agent graphs, which is why this page cites the search itself and the MAPF survey that
                names this approach “coupled”.
            </p>} ko={<p>
                joint 상태 공식화의 창간 논문은 없다. agent별 그래프의 product에 A*를 적용한 것이 전부이기
                때문이다. 그래서 이 페이지는 탐색 자체와, 이 접근을 survey에서 “coupled”라 부른 논문을 인용한다.
            </p>} />
            <ol>
                <li>
                    P. E. Hart, N. J. Nilsson, B. Raphael,{" "}
                    <a href="https://doi.org/10.1109/TSSC.1968.300136" target="_blank" rel="noopener noreferrer">
                        <em>A Formal Basis for the Heuristic Determination of Minimum Cost Paths</em>
                    </a>,
                    IEEE Transactions on Systems Science and Cybernetics, 1968.
                </li>
                <li>
                    R. Stern et al.,{" "}
                    <a href="https://doi.org/10.1609/socs.v10i1.18510" target="_blank" rel="noopener noreferrer">
                        <em>Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks</em>
                    </a>,
                    SoCS, 2019.
                </li>
            </ol>
        </>
    )
}

export default JointAStar
