import {ReactNode} from "react";
import {T, useTr} from "../../libs/i18n";
import Terms from "../../components/math/Terms";
import {BlockMath, InlineMath} from "../../components/math/Tex";
import TraceReplay from "../../components/panels/TraceReplay";
import CodeTabs from "../../components/CodeTabs";
import Pseudocode from "../../components/Pseudocode";
import pyImpl from "../../../../python/mrmp/mapf/prioritized_astar.py?raw";
import cppHeader from "../../../../cpp/include/mrmp/mapf/prioritized_astar.hpp?raw";
import cppImpl from "../../../../cpp/src/mapf/prioritized_astar.cpp?raw";

const REPO = "https://github.com/robotics-study/MRMP_MultiAgentMotionPlanning_study"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const PrioritizedAStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every planner on the single-robot side of this family solves one robot's map.
                    Multi-agent planning breaks because those individually-perfect paths collide.
                    The cheapest way to rescue a single-robot planner is an <strong>order</strong>:
                    pick who goes first, let that agent plan as if it were alone, then make every
                    later agent treat the finished paths as moving obstacles and route around them.
                    Erdmann and Lozano-Pérez formalized exactly this in 1987 —{" "}
                    <em>prioritized planning</em>, the first entry of this repository's genealogy and
                    the decoupled pole against which everything else is measured.
                </p>}
                ko={<p>
                    이 계열의 단일 로봇 planner는 전부 로봇 한 대의 지도를 푼다. 여러 대가 모이면
                    각자 완벽한 경로들이 서로 부딪혀 문제가 된다. 단일 robot planner를 살리는 가장
                    값싼 방법은 <strong>순서</strong>다. 먼저 갈 agent를 정하고 그 agent는 혼자 있는
                    것처럼 계획하게 한다. 이후의 모든 agent는 완성된 경로들을 움직이는 장애물로 취급해
                    그 주위를 돌아간다. Erdmann과 Lozano-Pérez가 1987년에 바로 이 방법을 공식화했다.
                    이것이 이 저장소 계보의 첫 항목이며, 다른 모든 접근이 여기에 대해 저울질되는
                    decoupled 극단이다.
                </p>}
            />

            <h2>{t("From A* to Prioritized Planning", "A*에서 우선순위 계획으로")}</h2>
            <T
                en={<>
                    <p>
                        Once agent <InlineMath math="j"/> has finished, its path is no longer a
                        geometric curve but a <em>time-parameterized obstacle</em>: at every step{" "}
                        <InlineMath math="t"/> it occupies exactly one cell, and after arriving it
                        stands on its goal forever (stay-at-goal):
                    </p>
                    <BlockMath math="O_j(t) \;=\; p_j\bigl[\min(t,\; T_j)\bigr], \qquad T_j = \operatorname{len}(p_j) - 1"/>
                    <Terms items={[
                        ["p_j", <>agent <InlineMath math="j"/>의 완성된 space-time 경로. <InlineMath math="p_j[t]"/>는 시각 <InlineMath math="t"/>에 차지하는 셀</>],
                        ["T_j", <>경로 길이. 마지막 스텝 이후에도 agent는 goal에 서 있으므로 경로는 더 이상 자라지 않아도 점유를 계속한다</>],
                        ["O_j(t)", <>시각 <InlineMath math="t"/>에 agent <InlineMath math="j"/>가 점유하는 셀. 이 함수가 곧 제약이다</>],
                    ]}/>
                    <p>
                        With reservations in hand, the multi-agent problem collapses into{" "}
                        <em>one single-robot search per agent</em>. The state is a space-time pair{" "}
                        <InlineMath math="(c, t)"/>: cell and timestep. Every action (four moves plus
                        wait) costs exactly one step, so <InlineMath math="g = t"/> always, the
                        heuristic is Manhattan distance, and the search is the plain optimal A* you
                        already know — just running on a grid × time lattice where a successor{" "}
                        <InlineMath math="(c', t+1)"/> simply doesn't exist if some earlier agent's{" "}
                        <InlineMath math="O_j(t+1) = c'"/>, or if the two would swap cells across the step.
                    </p>
                    <p>
                        Priority is not a parameter here. It <em>is</em> the agent index: agent 0 plans
                        against nothing but the static map, agent 1 reserves agent 0's finished path, and
                        so on. The order is fixed up front, which is also what makes the whole pipeline
                        deterministic — the same input always produces byte-identical traces in every
                        language this repository implements it in.
                    </p>
                </>}
                ko={<>
                    <p>
                        agent <InlineMath math="j"/>가 계획을 끝내면 그 경로는 더 이상 기하학적 곡선이
                        아니라 <em>시간이 매개변수화된 장애물</em>이다. 모든 스텝 <InlineMath math="t"/>에
                        정확히 한 셀을 차지하고, 도착한 뒤에도 goal 위에 계속 서 있다 (stay-at-goal):
                    </p>
                    <BlockMath math="O_j(t) \;=\; p_j\bigl[\min(t,\; T_j)\bigr], \qquad T_j = \operatorname{len}(p_j) - 1"/>
                    <Terms items={[
                        ["p_j", <>agent <InlineMath math="j"/>의 완성된 space-time 경로. <InlineMath math="p_j[t]"/>는 시각 <InlineMath math="t"/>에 차지하는 셀</>],
                        ["T_j", <>경로 길이. 마지막 스텝 이후에도 agent가 goal에 서 있으므로, 경로는 더 자라지 않아도 점유를 계속한다</>],
                        ["O_j(t)", <>시각 <InlineMath math="t"/>에 agent <InlineMath math="j"/>가 점유하는 셀. 이 함수가 곧 제약이다</>],
                    ]}/>
                    <p>
                        예약이 확보되면 multi-agent 문제는 <em>agent당 단일 로봇 탐색 한 번</em>으로
                        무너진다. 상태는 시공간 쌍 <InlineMath math="(c, t)"/>: 셀과 시각의 짝. 모든
                        행동(4방향 이동 + 대기)은 정확히 스텝 하나 비용이라 항상{" "}
                        <InlineMath math="g = t"/>이고, heuristic은 Manhattan 거리다. 그래서 탐색은 이미
                        알고 있는 최적 A* 그대로이며, 격자 × 시간 격자 위에서 돌릴 뿐이다. 어떤 앞선
                        agent의 <InlineMath math="O_j(t+1) = c'"/>이면 후속 상태{" "}
                        <InlineMath math="(c', t{+}1)"/>는 존재하지 않고, 두 agent가 스텝을 사이에 두고
                        셀을 맞바꾸는 이동도 존재하지 않는다.
                    </p>
                    <p>
                        우선순위는 여기서 파라미터가 아니다. 우선순위<InlineMath math="="/> agent index다.
                        agent 0은 정적 지도만 상대로 계획하고, agent 1은 agent 0의 완성된 경로를 예약으로
                        깔고, 그다음이 이어진다. 순서는 처음부터 고정이고, 이것이 파이프라인 전체를
                        결정론적으로 만드는 근거이기도 하다. 같은 입력은 이 저장소가 구현한 모든 언어에서
                        바이트 단위로 동일한 trace를 만든다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Individually optimal.</strong> Each agent's returned path is a{" "}
                            <em>shortest</em> feasible space-time path against the fixed reservations
                            of its predecessors — the sub-search is exact A*, no weighting, no shortcuts.
                        </li>
                        <li>
                            <strong>Not jointly optimal.</strong> The joint plan can cost more than the
                            true optimum because an early agent never detours to help a later one.
                            Priority order is a heuristic standing in for the joint search.
                        </li>
                        <li>
                            <strong>Not complete.</strong> A fixed order can box a later agent in even
                            when some joint plan exists (an earlier path may wall off the only pocket).
                            The planner reports <InlineMath math="\text{success} = \text{false"/> honestly
                            instead of pretending otherwise.
                        </li>
                        <li>
                            <strong>Cost:</strong> one A* per agent over at most{" "}
                            <InlineMath math="|V| \times T"/> space-time states with{" "}
                            <InlineMath math="T"/> bounded by the finite-horizon argument below. Total work
                            is linear in the number of agents — that is the whole trade: no exponential{" "}
                            <InlineMath math="|V|^k"/> joint space, paid for with optimality and completeness.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>개별 최적 (individually optimal).</strong> 각 agent가 돌려받는 경로는
                            앞선 agent들의 고정된 예약에 대해 <em>최단</em> feasible 시공간 경로다. 하위
                            탐색은 정확한 A* 그 자체이고 가중치도 근사도 없다.
                        </li>
                        <li>
                            <strong>전체(joint) 최적 아님.</strong> 앞선 agent가 뒤를 위해 돌아가는 법이
                            없으므로 joint 계획은 참최적보다 비쌀 수 있다. 우선순위 순서가 joint 탐색을
                            대신하는 heuristic인 셈이다.
                        </li>
                        <li>
                            <strong>불완전 (not complete).</strong> 고정된 순서는 뒤의 agent를, joint 계획이
                            존재함에도 갇히게 할 수 있다 (앞선 경로가 유일한 pocket을 막아버릴 수 있다).
                            planner는 거짓말하지 않고 <InlineMath math="\text{success} = \text{false}"/>를
                            정직하게 보고한다.
                        </li>
                        <li>
                            <strong>비용:</strong> agent마다 A* 한 번. 상태는 최대{" "}
                            <InlineMath math="|V| \times T"/>개의 시공간 상태이고{" "}
                            <InlineMath math="T"/>는 아래 유한 horizon 논의로 제한된다. 총 작업량은 agent
                            수에 선형. 이게 거래 전체다: 지수적 <InlineMath math="|V|^k"/> joint 공간을
                            최적성과 완전성하고 맞바꾼다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The procedure is one loop over agents, and inside each iteration a textbook A* on
                    the space-time lattice. Note where the multi-agent content hides: it is entirely in
                    which successors <em>exist</em>, never in the search machinery itself.
                </p>}
                ko={<p>
                    절차는 agent에 대한 루프 하나이고, 각 반복 안은 시공간 격자 위의 교과서적 A*다.
                    multi-agent 내용이 어디에 숨어 있는지 주목하라: 탐색 메커니즘 자체에는 아무것도 없고,
                    오직 어떤 후속 상태가 <em>존재하느냐</em>에만 들어 있다.
                </p>}
            />
            <Pseudocode code={`for each agent i in index order (priority = index):                     # 1
    if some reserved path occupies start_i at t = 0: return failure   # 2
    R ← flood_fill(free cells reachable from start_i)                 # 3
    if goal_i ∉ R: return failure                                     # 4
    horizon ← max_j (len(p_j) − 1) + |R|                              # 5
    OPEN ← min-heap keyed by (f, push-seq); push (h(start), seq₀)     # 6
    while OPEN not empty:                                             # 7
        (cell, t) ← pop_min; expanded += 1
        if cell = goal and no reservation visits goal at any step ≥ t:
            return reconstruct(parent)                                # 8
        if t = horizon: skip expansion
        for succ in fixed order up/down/left/right/wait:              # 9
            blocked if O_j(t+1) = succ  or  (O_j(t) = succ ∧ O_j(t+1) = cell)
            push (t+1 + h(succ), next seq) — each state exactly once  # 10`}
            />
            <T
                en={<ol>
                    <li>Agents are planned strictly in index order. Earlier paths become reservations;
                        the priority is fixed, never re-decided mid-run.</li>
                    <li>An earlier path standing on the later agent's start at{" "}
                        <InlineMath math="t=0"/> is an unavoidable joint conflict. Fail honestly — no
                        move can undo it.</li>
                    <li>A plain flood fill over free cells decides static unreachability up front, and
                        its size feeds the horizon bound in step 5.</li>
                    <li>If the goal isn't statically reachable there is nothing to search.</li>
                    <li>The time axis must be made finite somehow. Once every reservation has parked
                        (after <InlineMath math="\max_j T_j"/>), only simple static paths matter, so no
                        minimal feasible plan needs states beyond{" "}
                        <InlineMath math="T_{\text{frozen}} + |R|"/>.</li>
                    <li>The frontier is a min-heap on <InlineMath math="(f, \text{seq})"/>: priority by{" "}
                        <InlineMath math="f = t + h"/>, ties broken by push order. This exact tie-break is
                        part of the cross-language contract.</li>
                    <li>Pop is expansion (every state is pushed exactly once — with{" "}
                        <InlineMath math="g = t"/> a discovered cost never improves, so there is nothing
                        to relax and no lazy deletion needed).</li>
                    <li>Popping the goal isn't enough: under stay-at-goal semantics the finished agent
                        occupies its goal forever, so the pop counts only when no earlier path visits the
                        goal at any step ≥ <InlineMath math="t"/>.</li>
                    <li>Successors are generated in one fixed order (up/down/left/right, then wait) —
                        determinism is a feature, not an accident.</li>
                    <li>A move into <InlineMath math="c'"/> at step <InlineMath math="t+1"/> is legal only
                        if no reservation occupies <InlineMath math="c'"/> at{" "}
                        <InlineMath math="t+1"/> (vertex conflict) nor swaps it with the current cell across
                        that step (edge conflict). Survivors are pushed once, keyed by state.</li>
                </ol>}
                ko={<ol>
                    <li>agent는 index 순서대로만 계획한다. 앞선 경로는 예약이 되고, 우선순위는 고정이다.
                        도중에 다시 정하지 않는다.</li>
                    <li><InlineMath math="t=0"/>에 앞선 경로가 뒤 agent의 시작 셀 위에 있으면 되돌릴 수 없는
                        joint conflict다. 정직하게 실패를 반환한다. 어떤 수도 되돌릴 수 없다.</li>
                    <li>자유 셀에 대한 단순 flood fill로 정적 도달 불가를 먼저 판정하고, 그 크기가 5단계의
                        horizon 상한에 들어간다.</li>
                    <li>goal이 정적으로도 도달 불가면 탐색할 것이 없다.</li>
                    <li>시간축은 유한화해야 한다. 모든 예약이 주차된(<InlineMath math="\max_j T_j"/> 이후)
                        뒤에는 단순한 정적 경로만 의미 있으므로, 최소의 feasible 계획은{" "}
                        <InlineMath math="T_{\text{frozen}} + |R|"/> 너머 상태를 필요로 하지 않는다.</li>
                    <li>frontier는 <InlineMath math="(f, \text{seq})"/> 기준 min-heap이다. 우선순위는{" "}
                    <InlineMath math="f = t + h"/>이고 동률은 push 순서로 깬다. 이 정확한 tie-break가
                        언어 간 계약의 일부다.</li>
                    <li>pop이 곧 확장이다. 모든 상태는 정확히 한 번 push된다. <InlineMath math="g = t"/>라
                        발견 시점의 비용은 절대 개선되지 않으므로 relaxation도 lazy deletion도 필요 없다.</li>
                    <li>goal을 pop하는 것만으로는 부족하다. stay-at-goal 반향으로 완료된 agent는 goal을
                        영원히 점유하므로, 어떤 앞선 경로도 시각 ≥ <InlineMath math="t"/>에서 goal을 방문하지
                        않을 때에만 그 pop이 유효하다.</li>
                    <li>후속 상태는 고정 순서(up/down/left/right, 마지막에 wait)로 생성한다. 결정론은
                        사고가 아니라 기능이다.</li>
                    <li><InlineMath math="t+1"/> 스텝의 <InlineMath math="c'"/>로의 이동은 어떤 예약도{" "}
                        <InlineMath math="c'"/>를 <InlineMath math="t+1"/>에 점유하지 않고(vertex conflict)
                        현재 셀과 자리를 맞바꾸지도 않을 때(edge conflict)에만 합법이다. 남은 것들은 상태로
                        키화되어 정확히 한 번 push된다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    Two guarantees and two failures, precisely. The guarantee is per-agent optimality and
                    joint validity of the returned set; the failure modes are exactly the ones the design
                    admits: a priority order can lose against the joint optimum, and it can fail outright
                    where another order would have succeeded. Expand the proofs for the formal statements —
                    including why the finite horizon is sound rather than an arbitrary cutoff.
                </p>}
                ko={<p>
                    보장 둘과 실패 둘을 정확히 구분하자. 보장은 agent별 최적성과 반환된 경로 집합의 joint
                    타당성이다. 실패는 설계가 스스로 인정한 것들뿐이다: 우선순위 순서는 joint 최적에 질 수
                    있고, 다른 순서라면 성공했을 상황에서 통째로 실패할 수도 있다. 형식적 서술은 증명을
                    펼쳐 보라. 임의 컷오프가 아닌 유한 horizon이 타당한 이유도 들어 있다.
                </p>}
            />
            <Proof title={t("Theorem (individual optimality, joint validity)", "정리 (개별 최적성, joint 타당성)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> For each agent <InlineMath math="i"/> in order, the
                            returned path is a shortest space-time path from{" "}
                            <InlineMath math="\text{start}_i"/> to <InlineMath math="\text{goal}_i"/> among
                            the paths legal against the fixed reservations{" "}
                            <InlineMath math="O_0, \dots, O_{i-1}"/>, and the returned set is pairwise
                            conflict-free.
                        </p>
                        <p>
                            <strong>Optimality.</strong> Fix <InlineMath math="i"/> and freeze all earlier
                            paths. What remains is an ordinary static graph: states{" "}
                            <InlineMath math="(c, t)"/> with edges to legal successors at{" "}
                            <InlineMath math="t+1"/>. The Manhattan heuristic is admissible for this graph
                            (every real step costs 1 and Manhattan never overestimates the remaining steps),
                            so A* popping the goal at{" "}
                            <InlineMath math="(goal, t)"/> — with the stay-at-goal guard ensuring the path
                            stays legal after arrival — returns a shortest feasible path. Any cheaper legal
                            path would put some state of its own in OPEN with{" "}
                            <InlineMath math="f \le C^* < t"/> at pop time, contradiction.
                        </p>
                        <p>
                            <strong>Validity.</strong> A move into a reserved cell is pruned at expansion
                            (vertex), as is any swap across an edge; the goal guard extends each reservation
                            forever. So no pair of returned paths shares a cell at a step or swaps ends —
                            conflicts are excluded by construction, not detected afterwards.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 순서상 각 agent <InlineMath math="i"/>에 대해 반환되는
                            경로는, 고정된 예약 <InlineMath math="O_0, \dots, O_{i-1}"/>에 대해 합법인 경로
                            중 <InlineMath math="\text{start}_i"/>에서{" "}
                            <InlineMath math="\text{goal}_i"/>까지의 최단 시공간 경로다. 그리고 반환된
                            집합은 쌍별로 충돌이 없다.
                        </p>
                        <p>
                            <strong>최적성.</strong> <InlineMath math="i"/>를 고정하고 앞선 경로를 전부
                            얼린다. 남는 것은 평범한 정적 그래프다: 상태는 <InlineMath math="(c, t)"/>이고
                            간선은 <InlineMath math="t+1"/>의 합법 후속으로 간다. Manhattan heuristic은 이
                            그래프에 admissible하다(실제 스텝은 항상 비용 1이고 Manhattan은 남은 스텝을
                            과대평가하지 않는다). 따라서 A*가 <InlineMath math="(goal, t)"/>를 pop하는 순간
                            반환되는 경로는 feasible 최단이다. stay-at-goal 가드가 도착 이후의 합법성까지
                            보장하기 때문이다. 더 싼 합법 경로가 있었다면 그 경로의 어떤 상태가 pop 시점에{" "}
                            <InlineMath math="f \le C^* < t"/>로 OPEN에 있었을 테니 모순.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            <strong>타당성.</strong> 예약된 셀로의 이동은 확장에서 잘려나가고(vertex),
                            간선을 가로지르는 맞교환도 잘린다. goal 가드는 각 예약을 영원히 연장한다.
                            그러니 반환된 어떤 경로 짝도 스텝 공유 셀을 갖지 않고 끝을 맞바꾸지도 않는다.
                            충돌은 사후 탐지가 아니라 구성 자체로 배제된다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Lemma (why the horizon is sound)", "보조정리 (horizon이 타당한 이유)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Let <InlineMath math="T_f = \max_j T_j"/> over the frozen
                            reservations and{" "}
                            <InlineMath math="R"/> the static reachable set from the start. If any feasible
                            path to the goal exists at all, one exists of length{" "}
                            <InlineMath math="\le T_f + |R|"/>, so refusing to expand states past that
                            horizon loses no solutions.
                        </p>
                        <p>
                            <strong>Proof.</strong> After step <InlineMath math="T_f"/> every earlier agent is
                            parked on its goal forever, so the time-varying constraints are constant: a cell
                            is either permanently blocked or permanently free. On a static map, any feasible
                            suffix can be simplified — deleting loops from the waiting part leaves a simple
                            (self-avoiding) path through permanently-free cells, and a simple path visits at
                            most <InlineMath math="|R|"/> cells. Concatenating that suffix after step{" "}
                            <InlineMath math="T_f"/> yields a feasible plan of length at most{" "}
                            <InlineMath math="T_f + |R|"/>. States beyond the horizon therefore cannot lie on
                            any minimal feasible plan, and since A* pops by increasing{" "}
                            <InlineMath math="f = g + h \ge g"/>, a successful search terminates long before
                            the cutoff is ever consulted. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 얼린 예약들에 대해{" "}
                            <InlineMath math="T_f = \max_j T_j"/>, <InlineMath math="R"/>은 시작에서 도달
                            가능한 정적 셀 집합이라 하자. goal까지 feasible 경로가 어딘가에 존재한다면,{" "}
                            <InlineMath math="\le T_f + |R|"/> 길이인 것도 하나 존재한다. 그래서 그 horizon
                            너머 상태를 확장하지 않아도 해를 잃지 않는다.
                        </p>
                        <p>
                            <strong>증명.</strong> 스텝 <InlineMath math="T_f"/> 이후 모든 앞선 agent는 goal에
                            영원히 주차되므로, 시간에 따라 변하던 제약은 상수가 된다: 셀은 영구 차단이거나
                            영구 자유다. 정적 지도에서 어떤 feasible 접미식이든 단순화할 수 있다. 대기
                            부분의 루프를 지우면 영구 자유 셀을 지나는 단순(self-avoiding) 경로가 남고, 단순
                            경로는 최대 <InlineMath math="|R|"/>개 셀을 방문한다. 그 접미식을 스텝{" "}
                            <InlineMath math="T_f"/> 뒤에 이어붙이면 길이{" "}
                            <InlineMath math="\le T_f + |R|"/>의 feasible 계획이 나온다. 그러니 horizon 너머
                            상태는 어떤 최소 feasible 계획 위에도 있을 수 없고, A*는{" "}
                            <InlineMath math="f = g + h \ge g"/>가 증가하는 순서로 pop하므로 성공하는 탐색은
                            컷오프가 필요해지기 훨씬 전에 끝난다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The player below replays recorded traces from the repository's real demos — click a
                    scenario button to switch. Watch each agent's expansion bloom in its own color (the
                    space-time search, flattened back onto the grid), then the execution replay walk every
                    finished path step by step. The three scenarios are chosen for what priority does:{" "}
                    <code>maze01_two</code> threads both agents through one corridor gap at staggered times
                    so neither pays any detour: each still achieves its unconstrained shortest cost;
                    <code>open01_cross</code> times a crossing apart instead of slowing down; and in
                    <code>open01_swap</code> agent 0's straight path is a moving wall that agent 1 must
                    route around — parity makes cost 15 impossible, so the detour costs exactly 16.
                </p>}
                ko={<p>
                    아래 플레이어는 이 저장소의 실제 demo가 방출한 기록된 trace를 재생한다. 시나리오 버튼을
                    눌러 전환하라. 각 agent의 확장이 자기 색으로 피어나는 것(시공간 탐색을 격자에 다시
                    펼친 것)이 보이고, 이어 실행 재생이 완성된 경로를 스텝마다 걸어간다. 세 시나리오는
                    우선순위가 무엇을 하는지로 골랐다: <code>maze01_two</code>는 두 agent를 폭 1 통로 gap을
                    통해 시간을 어긋나게 통과시켜 둘 모두 우회 없이 자기 최소 비용을 유지한다.{" "}
                    <code>open01_cross</code>는 교차점에서 속도를 줄이는 대신 타이밍으로 비킨다. 그리고{" "}
                    <code>open01_swap</code>에서 agent 0의 직진 경로는 움직이는 벽이 되고 agent 1은 그 주위를
                    돌아가야 한다. parity 때문에 비용 15가 불가능해서 우회 비용은 정확히 16이다.
                </p>}
            />
            <TraceReplay algo="prioritized_astar" label={t(
                "Recorded traces from the repository's prioritized_astar demo (per-agent expansion + execution replay)",
                "저장소의 prioritized_astar demo가 방출한 실제 trace(agent별 확장 + 실행 재생)",
            )} sources={[
                {scenario: "maze01_two", map: "maze01"},
                {scenario: "open01_cross", map: "open01"},
                {scenario: "open01_swap", map: "open01"},
            ]}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the
                    browser engine that powers parity checking is a third mirror. Same fixed neighbor order,
                    same <InlineMath math="(f, \text{seq})"/> tie-break, same horizon argument, same event
                    stream — so all three produce byte-identical traces on every scenario, which{" "}
                    <code>check-engine-parity</code> verifies on every build. The code below is the actual
                    source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, parity 검사를 뒷받침하는 브라우저 엔진이
                    세 번째 미러다. 동일한 고정 이웃 순서, 동일한 <InlineMath math="(f, \text{seq})"/>
                    tie-break, 동일한 horizon 논의, 동일한 이벤트 스트림. 그래서 셋 모두 모든 시나리오에서
                    바이트 단위로 동일한 trace를 만들고, 빌드마다 <code>check-engine-parity</code>가 이를
                    검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/mapf/prioritized_astar.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/mapf/prioritized_astar.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/mapf/prioritized_astar.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/mapf/prioritized_astar.hpp`,
                            },
                            {
                                name: "cpp/src/mapf/prioritized_astar.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/mapf/prioritized_astar.cpp`,
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
            <ol>
                <li>
                    H. Erdmann, T. Lozano-Pérez,{" "}
                    <a href="https://doi.org/10.1007/BF00665363" target="_blank" rel="noopener noreferrer">
                        <em>On Multiple Moving Objects</em>
                    </a>,
                    Algorithmica, 1987.
                </li>
            </ol>
        </>
    )
}

export default PrioritizedAStar
