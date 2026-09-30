import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runSrrt} from "../../../libs/algorithms/srrt";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/sampling/srrt.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/sampling/srrt.hpp?raw";
import cppImpl from "../../../../../cpp/src/sampling/srrt.cpp?raw";

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 파라미터는 저장소의 configs/sampling/srrt.yaml
// 기본값과 동일: seed 42, goal biasing 0.5, 반복 예산 2500. gamma와 c_max는 없다 —
// sRRT에는 수축 반지름도 greedy 예산도 없다.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runSrrt(map, tasks, {seed: 42, goal_sampling_probability: 0.5, max_iterations: 2500})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. agent 상한은 4.
// cross와 maze는 policy 경로가 시간차로 어긋나 결합이 일어나지 않는 경우, swap은
// 정면 충돌로 결합이 반드시 일어나는 경우다.
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

const SubdimensionalRrt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    MA-RRT* fought the curse of dimension by searching the joint space harder. Wagner, Kang and
                    Choset refuse the premise: why search the joint space at all when most robots, most of the
                    time, have nothing to coordinate about? Their framework is <em>subdimensional expansion</em> —
                    plan for each robot separately first (an <em>individual policy</em>: the optimal action at every
                    point of that robot's own space, ignoring everyone else), combine those paths into a
                    one-dimensional search space embedded in the full configuration space, and raise the local
                    dimensionality only where robots actually collide. This page is what they get when they run that
                    framework on an RRT instead of graph search: sRRT. The joint tree still grows forward from{" "}
                    <InlineMath math="q_I"/>, but every node carries a <em>collision set</em> — the robots whose
                    policy has been shown insufficient — and robots outside it never stop obeying their policy.
                    It is CBS's trick wearing a sampling coat: couple only where you must, decouple by branching
                    off an ancestor whose set is smaller.
                </p>}
                ko={<p>
                    MA-RRT*는 차원의 저주를 joint 공간을 더 세게 탐색해서 싸웠다. Wagner, Kang, Choset는 전제
                    자체를 거부한다. 대부분의 로봇은 대부분의 시간 조율할 게 아무것도 없는데 왜 joint 공간을
                    탐색하느냐고. 그들의 프레임워크는 <em>subdimensional expansion</em>이다. 각 robot을 먼저 혼자
                    계획하고(개별 <em>individual policy</em>: 남들은 전부 무시한 채 자기 공간의 모든 점에서 최적
                    action을 돌려주는 함수), 그 경로들을 합쳐 전체 configuration space에 박힌 1차원 탐색 공간을
                    만들고, robot들이 실제로 충돌하는 곳에서만 국소 차원을 올린다. 이 페이지는 그 프레임워크를
                    graph search 대신 RRT 위에 돌렸을 때 얻는 것이다. sRRT. joint 트리는 여전히{" "}
                    <InlineMath math="q_I"/>에서 앞으로 자라지만 모든 node가 <em>collision set</em>을 싣고 간다.
                    policy가 부족하다는 것이 입증된 robot들의 집합이다. 그 밖의 robot은 policy를 절대 그만두지
                    않는다. CBS의 속임수를 sampling 코트에 입힌 것이다. 반드시 필요한 곳에서만 결합하고 더 작은
                    set을 가진 ancestor에서 가지치기로 다시 분리된다.
                </p>}
            />

            <h2>{t("From Joint Trees to Individual Policies", "joint 트리에서 개별 policy로")}</h2>
            <T
                en={<>
                    <p>
                        The framework rests on two objects. The first is the <em>individual policy</em>:{" "}
                        <InlineMath math="\varphi_i"/> maps every point of robot <InlineMath math="i"/>'s own
                        configuration space to the optimal action from there to its goal, neglecting all other
                        robots. In the paper's high-dimensional settings a policy can only be approximated by an RRT
                        grown backward from the goal; on this repository's grid the individually optimal thing is
                        exactly constructible — a BFS tree from the goal whose parent pointer <InlineMath math="p"/>{" "}
                        names the next step, with the goal looping at itself (a robot at its own goal waits). The
                        paper says so itself: “we prefer to use optimal individual policies”, and only reach for
                        RRTs because constructing them is infeasible when the individual space has high dimension.
                        Here it isn't.
                    </p>
                    <p>
                        The second object is the <em>collision set</em>. A node of the joint tree{" "}
                        <InlineMath math="T_f"/> is a triple — configuration, predecessor pointer, and the set{" "}
                        <InlineMath math="C_k"/> of robots for which the planner has <em>found a path through that
                        node into a collision</em>. Robots outside <InlineMath math="C_k"/> can safely obey their
                        policy; robots inside it must have every local path considered. That is what makes the
                        search space variable-dimensional: from a node <InlineMath math="q_r"/> with{" "}
                        <InlineMath math="C_r = \varnothing"/> the reachable set is just the one joint path the two
                        policies already trace, and it thickens into full joint freedom exactly where a collision
                        has been found. A sample lands in the local search space by projection:
                    </p>
                    <BlockMath math="q'_s = \prod_i \begin{cases} \varphi_i(q^i_r) & r_i \notin C_r \\ q^i_s & r_i \in C_r \end{cases}"/>
                    <Terms items={[
                        ["\\varphi_i", <>robot <InlineMath math="i"/>'s individual policy — on this grid, the BFS tree grown backward from its goal;{" "}
                            <InlineMath math="\varphi_i(q^i_r)"/> is one policy step from where robot <InlineMath math="i"/> stands in <InlineMath math="q_r"/></>],
                        ["C_r", <>the collision set of node <InlineMath math="q_r"/>: robots whose policy has been shown insufficient somewhere on the chain below. Outside the set means “obey the policy”, inside means “steer by samples”</>],
                        ["q'_s", <>the projected sample — pinned coordinates replaced by one policy step, free coordinates kept from the raw draw</>],
                    ]}/>
                    <p>
                        Then the machinery is RRT-shaped: NEAREST picks the tree node whose joint state sits closest
                        to the (projected) sample, a local planner walks from that node to the projection — pinned
                        robots take their one policy step and wait, free robots greedy-step toward their sampled cell
                        in lockstep — and if the walk is collision-free its endpoint joins the tree carrying{" "}
                        <InlineMath math="C_r"/> unchanged. A robot-robot collision on a local path is information:
                        every involved robot joins <InlineMath math="C_r"/>, and that addition back-propagates up
                        the predecessor chain until an ancestor already lists the robot, so future expansions from{" "}
                        <em>any</em> affected node steer it by samples instead of policy. A collision between robots
                        both already free carries no information: the expansion just fails, like any collision would.
                    </p>
                </>}
                ko={<>
                    <p>
                        프레임워크는 두 객체 위에 서 있다. 첫번째는 <em>individual policy</em>다.{" "}
                        <InlineMath math="\varphi_i"/>는 robot <InlineMath math="i"/>의 자기 공간의 모든 점을, 다른
                        모든 robot을 무시했을 때 거기서 goal까지의 최적 action에 사상한다. 논문의 고차원 설정에서는
                        policy를 goal에서 뒤로 자란 RRT로만 근사할 수 있지만 이 저장소의 격자에서는 개별적으로
                        최적적인 것이 정확히 구성 가능하다. goal에서 자란 BFS 트리이고 parent 포인터{" "}
                        <InlineMath math="p"/>가 다음 스텝을 가리키며 goal은 자기 자신에서 순환한다(자기 goal에
                        있는 robot은 대기한다). 논문이 직접 말한다. “optimal individual policies를 선호한다”. RRT를
                        쓰는 이유는 개별 공간의 차원이 높으면 구성이 불가능하기 때문일 뿐이다. 여기서는 아니다.
                    </p>
                    <p>
                        두번째 객체는 <em>collision set</em>이다. joint 트리 <InlineMath math="T_f"/>의 node는
                        triple이다. configuration, predecessor 포인터, 그리고 planner가 “그 node를 지나 충돌에
                        이르는 경로를 발견한” robot들의 집합 <InlineMath math="C_k"/>. <InlineMath math="C_k"/> 밖의
                        robot은 policy를 안전하게 따르고 안의 robot은 모든 국소 경로가 고려돼야 한다. 이게 바로
                        탐색 공간을 차원 가변으로 만드는 것이다. <InlineMath math="C_r = \varnothing"/>인 node{" "}
                        <InlineMath math="q_r"/>에서 도달 가능 집합은 두 policy가 이미 그어 놓은 joint 경로 하나뿐이고
                        충돌이 발견된 곳에서 정확히 전체 joint 자유도로 두꺼워진다. 표본은 투사로 국소 탐색 공간에
                        도착한다:
                    </p>
                    <BlockMath math="q'_s = \prod_i \begin{cases} \varphi_i(q^i_r) & r_i \notin C_r \\ q^i_s & r_i \in C_r \end{cases}"/>
                    <Terms items={[
                        ["\\varphi_i", <>robot <InlineMath math="i"/>의 individual policy. 이 격자에서는 goal에서 뒤로 자란 BFS 트리이고{" "}
                            <InlineMath math="\varphi_i(q^i_r)"/>은 <InlineMath math="q_r"/>에서 robot <InlineMath math="i"/>가 서 있는 곳에서 나가는 policy 스텝 하나다</>],
                        ["C_r", <>node <InlineMath math="q_r"/>의 collision set. 아래 chain에서 policy가 부족함이 입증된 robot들의 집합. 밖은 “policy를 따라라”, 안은 “표본으로 조향해라”</>],
                        ["q'_s", <>투사된 표본. pinned 좌표는 policy 스텝 하나로 대체되고 free 좌표는 날것 draw가 유지된다</>],
                    ]}/>
                    <p>
                        그 뒤의 기계는 RRT 모양이다. NEAREST가 (투사된) 표본과 가장 가까운 트리 node를 고르고 local
                        planner가 그 node에서 투사점까지 걷는다. pinned robot은 policy 스텝 하나만 하고 대기하고 free
                        robot은 sampled cell을 향해 lockstep으로 greedy하게 간다. walk이 conflict-free하면 끝점이{" "}
                        <InlineMath math="C_r"/>을 그대로 얹어 트리에 합류한다. local path 위의 robot-robot 충돌은
                        정보다: 관련된 robot 전원이 <InlineMath math="C_r"/>에 합류하고 그 추가가 predecessor chain을
                        거슬러 back-propagate되어 이미 그 robot을 올린 ancestor에서 멈춘다. 그래서 <em>어떤</em>
                        영향받은 node에서의 future expansion도 policy 대신 표본으로 그 robot을 조향한다. 이미 둘 다
                        free인 robot들 사이의 충돌은 정보가 아니다. expansion은 그냥 실패할 뿐이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>No guarantee is claimed, and that is the paper's own honesty.</strong> The sRRT
                            paper explicitly leaves even probabilistic completeness unproven — standard proofs fail
                            because a planner that never explores off-policy where no collision occurred will{" "}
                            <em>never</em> cover the full configuration space. There is no optimality at all: the
                            policies are individually optimal, but everything past coupling detours around any
                            optimum. What this repository can prove about its implementation goes in the collapsibles
                            below — emitted plans are valid, and a run that never couples returns a provably optimal
                            solution. Everything else is honest sampling: budget exhausted without a goal vertex is
                            “no solution found”, never “unsolvable”.
                        </li>
                        <li>
                            <strong>Dimension is paid only where robots actually meet.</strong> While no collision has
                            fired, the tree grows along one deterministic joint path — every expansion moves every
                            pinned robot exactly one policy step — and the sample stream decides nothing about where
                            the chain goes, only when it extends. On <code>open01_cross</code> and{" "}
                            <code>maze01_two</code> that is the whole story: the two BFS policies pass their shared
                            cells at different times, no collision ever fires, and the returned cost is exactly the
                            sum of the individually-optimal lengths (33 = 16 + 17, and 66 = 33 + 33) — which{" "}
                            <em>is</em> the joint optimum on those scenarios. That is timing luck on this geometry,
                            not a property of sRRT. On <code>open01_swap</code> the policies walk head-on into each
                            other; coupling fires, and from there sRRT is what it becomes when fully coupled: a plain
                            RRT on the joint state — MA-RRT* without rewiring — steered by goal-biased samples,
                            landing honestly suboptimal (44 against the exact 30).
                        </li>
                        <li>
                            <strong>Coupling is permanent on a branch.</strong> A robot that enters{" "}
                            <InlineMath math="C_r"/> never leaves it: sets only grow along their chain, and
                            dimensionality drops again only by branching off an ancestor whose set is smaller. The
                            framework's graph-search sibling (M*) can decouple groups of robots involved in widely
                            separated collisions; the paper names that as future work for sRRT, so this implementation
                            honestly does not. That is also why the coupled regime here degrades to plain joint-space
                            sampling rather than back to decoupled optimality.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>보장은 아무것도 걸려 있지 않고 그건 논문 자신의 정직함이다.</strong> sRRT 논문은
                            확률적 완전성조차 미증명으로 남긴다. 표준 증명들은 실패한다. 충돌이 나지 않은 길에서는
                            절대 policy 밖으로 탐색하지 않는 planner가 전체 configuration space를 결코 cover하지 않기
                            때문이다. 최적성은 아예 없다. policy는 개별적으로 최적 but 결합 이후의 모든 것은 어떤
                            최적성이든 빙판처럼 돌아간다. 이 저장소가 구현체에 대해 증명할 수 있는 것은 아래 접힌
                            증명의 두 정리다. 방출되는 계획은 합법이고 결합이 한 번도 일어나지 않은 실행은 증명 가능하게
                            최적이라는 것. 나머지는 정직한 sampling이다. 예산이 goal vertex 없이 끝나면 “예산 안에서
                            해를 찾지 못했다”이지 절대 “unsolvable”이 아니다.
                        </li>
                        <li>
                            <strong>차원은 robot들이 실제로 만나는 곳에서만 지불된다.</strong> 충돌이 한 번도 안 난
                            동안 트리는 결정적인 joint 경로 하나를 따라 자란다. 모든 expansion은 pinned robot마다
                            정확히 policy 스텝 하나씩 움직이고 표본 열은 체인이 어디로 가는지 아무것도 정하지 않고
                            언제 확장하기만 정한다. <code>open01_cross</code>와 <code>maze01_two</code>에서는 그게
                            이야기의 전부다. 두 BFS policy가 공유 셀을 다른 시각에 지나고 충돌은 한 번도 안 나고
                            반환 비용은 정확히 개별 최적 길이의 합이다(33 = 16 + 17 그리고 66 = 33 + 33). 그리고 그건
                            그 시나리오에서 joint 최적값“이다”. 이건 이 geometry에 대한 타이밍 운이고 sRRT의 성질이
                            아니다. <code>open01_swap</code>에서는 policy들이 정면으로 서로 걸어 들어가고 결합이
                            발화하고 거기서부터 sRRT는 완전히 결합됐을 때 자신이 되는 것이 된다. joint 상태 위의 plain
                            RRT — rewiring 없는 MA-RRT* — goal biasing된 표본에 조향되어 정직하게 suboptimal한 30
                            정확한 답 앞에서 44에 착지한다.
                        </li>
                        <li>
                            <strong>결합은 branch 위에서 영구적이다.</strong> <InlineMath math="C_r"/>에 들어간 robot은
                            절대 나가지 않는다. set은 chain을 따라 커지기만 하고 차원은 더 작은 set을 가진 ancestor에서
                            가지를 칠 때만 내려간다. 프레임워크의 graph search 형제(M*)는 멀리 떨어진 충돌들에 관여한
                            robot group들을 분리할 수 있고 논문은 그것을 sRRT의 future work로 이름 그대로 지목한다. 그래서
                            이 구현은 정직하게 그것을 하지 않는다. 결합된 영역이 여기에서 decoupled 최적성으로 돌아가는
                            대신 plain joint-space sampling으로 퇴화하는 이유도 그것이다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One loop, and the multi-agent content lives inside it as two data structures: k policy trees
                    built once up front, and a collision set carried by every joint-tree node. Everything else is an
                    RRT with a projection in front of NEAREST. The paper leaves several things ambiguous (how a
                    policy's on-demand extension behaves here, what happens to an expansion whose collision involves
                    only already-free robots, how ties break); every choice is fixed below and identical in all three
                    language engines. Goal biasing — sampling the goal tuple outright with probability{" "}
                    <InlineMath math="p_{\text{goal}}"/> — is not from the paper (it draws uniform joint samples
                    only) but is this repository's informed-sampling convention, and on a grid it is what makes the
                    coupled regime reach the goal at finite budget.
                </p>}
                ko={<>
                    루프 하나이고 multi-agent 내용은 그 안의 두 데이터 구조로 살아 있다. 미리 한 번 지어지는 k개의
                    policy 트리와 모든 joint node가 싣고 가는 collision set. 나머지는 전부 NEAREST 앞에 투사가 붙은
                    RRT다. 논문은 여러 가지를 모호하게 남긴다(policy의 on-demand extension이 여기서 어떻게 동작하는지,
                    이미 free인 robot만 관련된 충돌을 만난 expansion은 어떻게 되는지, 동률은 어떻게 깨는지)를 아래에서
                    모두 고정했고 세 언어 엔진에 동일하다. Goal biasing — 확률 <InlineMath math="p_{\text{goal}}"/>로
                    goal tuple을 그대로 샘플하는 것 — 은 논문 것이 아니다(논문은 균일 joint 표본만 뽑는다). 하지만 이건
                    이 저장소의 informed-sampling 관례이고 격자에서 유한한 예산 안에 결합된 영역이 goal에 닿게 만드는
                    것이 바로 그것이다.
                </>}
            />
            <Pseudocode code={`# φ_i = BFS tree grown backward from q_F^i (fixed neighbor order as tie-break;   # 1
#        the goal's own policy step is a wait at itself)                             # 2
# start ∉ φ_i의 트리 → instance 판정: 그 agent의 goal은 도달 불가                    # 3
T_f = {(q_I, no parent, C = ∅)}                                                      # 4
repeat max_iterations times:
    u ← uniform(); x ← q_F w.p. p_goal else k independent uniform waypoints          # 5
    x_near ← argmin_{v ∈ T_f} Σ_i Manhattan(v^i, x^i)   (ties → lowest insertion index)  # 6
    C_r := C_near; targets: i ∉ C_r → φ_i(q^near^i), i ∈ C_r → x^i                    # 7
    walk lockstep from q_near to targets: pinned robots take their one policy step,   # 8
        free robots greedy-step (first strict min of Manhattan) and wait on arrival;
        a robot stuck at a local minimum honestly fails the expansion
    collision on any step? abort the expansion; if informative (some involved robot  # 9
        outside C_r), enlarge C_r and back-propagate up until an ancestor lists them
    endpoint already a vertex? no-op. else insert with inherited set; goal check      # 10
result: chain root → goal concatenated per agent, trimmed after each last move       # 11`}
            />
            <T
                en={<ol>
                    <li>The individual policies are built once, before the tree exists: a BFS from each goal over
                        free cells, fixed neighbor order (up/down/left/right) as parent tie-break. The paper grows
                        policies with RRTs and extends them on demand; on a grid the BFS tree covers every reachable
                        cell by construction, so “extend until covered” is vacuous — and if a start cell is missing
                        from its policy tree, the goal was unreachable from it at all. That is an instance verdict,
                        delivered immediately with zero expansions.</li>
                    <li>The root node is the start tuple with empty collision set; nodes live in insertion order, and
                        that order is load-bearing — it breaks NEAREST ties exactly like everywhere else in this
                        repository.</li>
                    <li>SAMPLE: one draw decides goal biasing first; otherwise one waypoint per agent, in agent
                        order. The paper draws uniform samples from the full configuration space; the bias is the
                        repository's convention on top.</li>
                    <li>NEAREST scans the whole tree on <InlineMath math="\sum_i \lvert x_i - y_i\rvert"/> (integer
                        arithmetic — no sqrt anywhere in this planner), ties to the lowest insertion index.</li>
                    <li>The projection (formula 2 of the paper): robots outside C_r have their coordinate replaced by
                        one policy step from where they stand in the nearest node; robots inside keep the sampled
                        cell. The walk then moves every robot in lockstep — pinned robots arrive after exactly one
                        step and wait, free robots descend strictly toward their target (a strict-min step strictly
                        decreases Manhattan distance, so a walk that cannot progress is stuck forever and honestly
                        fails instead of looping).</li>
                    <li>A vertex conflict or swap on any step of the walk kills the expansion — a node is added only
                        when no collision is found. If some involved robot was outside C_r the collision is
                        informative: every involved robot joins C_r and the addition propagates to ancestors until
                        one already lists that robot (its ancestors list it too, by induction). Otherwise nothing
                        updates; the expansion just fails like any collision would.</li>
                    <li>A projected sample whose walk lands on an existing vertex is a no-op — the only reading that
                        keeps T_f a tree. Accepted nodes inherit the nearest node's post-update set unchanged.</li>
                    <li>The answer: chain root → goal, concatenated per agent with duplicated heads skipped, trimmed
                        after each agent's last move (trailing waits sit at their own goal and cost 0). The loop
                        stops the moment the goal tuple becomes a vertex — there is no anytime refinement to keep
                        running for.</li>
                </ol>}
                ko={<ol>
                    <li>Individual policy는 트리가 존재하기 전에 한 번 지어진다: goal에서 자유 셀들을 향해 뒤로 자란
                        BFS, 고정 이웃 순서(up/down/left/right)가 parent 동률 처리다. 논문은 policy를 RRT로 기르고
                        필요할 때 확장하지만 격자에서는 BFS 트리가 구성상 도달 가능한 모든 셀을 덮는다. “cover될 때까지
                        확장”은 공허해지고 start cell이 그 policy 트리에 없으면 goal은 거기서 완전히 도달 불가였다는
                        뜻이다. 이건 instance 판정이고 확장 0으로 즉시 전달된다.</li>
                    <li>Root node는 collision set이 빈 start tuple이고 node들은 삽입 순서로 살고 그 순서는 기능을 한다.
                        MA-RRT* 때와 정확히 같은 방식으로 NEAREST 동률을 깬다.</li>
                    <li>SAMPLE: draw 하나가 먼저 goal biasing을 정하고 아니면 agent 순서대로 waypoint 하나씩. 논문은
                        전체 configuration space에서 균일 표본을 뽑고 bias는 그 위에 얹은 저장소의 관례다.</li>
                    <li>NEAREST는 트리를 <InlineMath math="\sum_i"/> Manhattan으로 훑고(정수 산술 — 이 planner 어디에도
                        sqrt는 없다) 동률은 최소 삽입 index로 깬다.</li>
                    <li>투사(논문의 식 2): C_r 밖의 robot은 좌표가 nearest node에서의 policy 스텝 하나로 대체되고 안의
                        robot은 sampled cell을 유지한다. walk은 모든 robot을 lockstep으로 움직인다. pinned는 정확히
                        스텝 하나 후 도착해 대기하고 free는 target으로 strict-min을 따라 내려간다(strict-min 스텝은
                        Manhattan 거리를 엄격히 줄이므로 진행 불가에 막힌 walk은 영원히 못 닿으니 무한 루프 대신 정직하게
                        실패한다).</li>
                    <li>walk의 어떤 스텝에서든 vertex conflict나 swap은 expansion을 죽인다. node는 conflict가 없을 때만
                        추가된다. 관련된 robot 중 일부가 C_r 밖이었으면 충돌은 informative이다: 관련 robot 전원이
                        C_r에 합류하고 그 추가가 ancestor를 거슬러 올라가 이미 그 robot을 올린 ancestor에서 멈춘다(그
                        ancestor의 ancestor들도 귀납으로 올린다). 아니면 아무것도 갱신되지 않고 expansion은 그냥 실패한다.</li>
                    <li>투사된 표본의 walk이 기존 vertex에 착지하면 no-op — T_f를 트리로 남기는 유일한 읽기다. 받아들여진
                        node는 nearest node의 갱신 후 set을 그대로 상속한다.</li>
                    <li>답: root → goal chain을 agent별로 이어붙이고 중복 head를 건너뛰고 각 agent의 마지막 이동 이후를
                        자른다(trailing wait은 자기 goal에서 0이다). goal tuple이 vertex가 되는 순간 루프는 멈춘다. 계속
                        돌릴 anytime 개선이 없으니까.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<>
                    <p>
                        The paper guarantees nothing about outcomes — it explicitly leaves probabilistic completeness
                        unproven and never claims optimality; standard probabilistic-completeness proofs fail here
                        because a tree that follows policies where no collision occurred never covers the off-policy
                        regions of the configuration space. What survives as theorem is narrower but provable, and
                        both halves are worth reading: every emitted plan is valid, and a run whose chain never
                        couples returns something provably optimal. Between those two lies everything sRRT actually
                        is: dimension that grows exactly where robots meet, and nothing you can promise about when
                        they do.
                    </p>
                </>}
                ko={<>
                    <p>
                        논문은 결과에 대해 아무것도 보장하지 않는다. 확률적 완전성을 명시적으로 미증명으로 남기고
                        최적성은 아예 주장하지 않는다. 표준 확률적 완전성 증명들은 여기서 실패한다. 충돌이 나지 않은
                        곳에서 policy를 따른 트리는 configuration space의 policy 밖 영역을 절대 cover하지 않기 때문이다.
                        정리로 살아남은 것은 좁지만 증명 가능하고 두 쪽 모두 읽을 가치가 있다. 방출되는 모든 계획은
                        합법이고 결합이 한 번도 일어나지 않은 실행은 증명 가능하게 최적인 것을 돌려준다. 그 둘 사이에
                        sRRT가 실제로 무엇인지의 전부가 있다. robot들이 만나는 곳에서 정확히 자라는 차원, 그리고 언제
                        만나는지 약속할 수 있는 게 아무것도 없다는 것.
                    </p>
                </>}
            />
            <Proof title={t("Theorem (every emitted plan is valid)", "정리 (방출되는 모든 계획은 합법이다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Every path pair the planner emits is collision-free, and its cost
                            equals the summed arrival times of what was actually driven.
                        </p>
                        <p>
                            <strong>Proof.</strong> A tree edge exists only for a walk that passed the step-wise check:
                            no two agents land on one cell (vertex conflict) and no pair swaps cells across a step
                            (edge conflict). Concatenating segments along a parent chain therefore concatenates
                            step-wise conflict-free motion — consecutive segments meet exactly at the shared joint
                            state, so the concatenation is itself a legal joint path. Trimming cuts each agent's path
                            after its last move; every cut step was a wait at that agent's own goal, which costs 0,
                            so neither the cost nor any other agent's motion changes. The emitted per-agent paths are
                            the coordinates of that joint path and their lengths minus one are exactly what the summed
                            metric charged for them. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 이 planner가 내보내는 모든 경로 쌍은 conflict-free이고 비용은
                            실제로 주행한 것의 도착 시각의 합과 같다.
                        </p>
                        <p>
                            <strong>증명.</strong> 트리 edge는 스텝별 검사를 통과한 walk에 대해서만 존재한다. 두 agent가
                            한 셀에 착지하지 않고(vertex) 쌍이 스텝을 넘어 셀을 맞바꾸지도 않는다(edge). 그래서 parent
                            chain을 따라 구간들을 이어붙이면 스텝별로 conflict-free한 움직임들이 이어붙여진다. 연속한
                            구간은 공유된 joint 상태에서 정확히 만나니 연결 자체가 합법적인 joint 경로다. 자르기는 각
                            agent의 마지막 이동 이후를 잘라내고 잘린 모든 스텝은 자기 goal에서의 대기였는데 그 비용은
                            0이다. 그래서 비용도 다른 agent의 움직임도 변하지 않는다. 방출된 agent별 경로는 그 joint
                            경로의 좌표들이고 길이 빼기 1이 합산 metric이 정확히 계산한 값이다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Theorem (never coupled means provably optimal)", "정리 (결합이 없으면 증명 가능하게 최적)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If no informative collision ever fires on the chain from root to
                            goal, the returned joint path has cost{" "}
                            <InlineMath math="\sum_i \ell_i"/> where <InlineMath math="\ell_i"/> is agent{" "}
                            <InlineMath math="i"/>'s individually-optimal path length — and no feasible joint solution
                            can cost less.
                        </p>
                        <p>
                            <strong>Proof.</strong> With an empty collision set every expansion pins every robot to its
                            policy: each accepted step of the chain moves agent <InlineMath math="i"/> exactly along
                            one BFS-tree edge toward its goal, so the concatenated path is exactly a shortest
                            obstacle-respecting path from start to goal — length <InlineMath math="\ell_i"/> minus the
                            free waits at the goal. Any feasible joint path is in particular a valid single-agent path
                            for each agent, and every step of it that leaves one agent's cell costs 1 under this
                            repository's metric, so any joint solution costs at least{" "}
                            <InlineMath math="\sum_i \ell_i"/>. The returned path achieves the bound, hence is jointly
                            optimal. Note what this does not say: nothing guarantees a run never couples — on{" "}
                            <code>open01_swap</code> coupling is unavoidable, and there sRRT lands above the optimum.
                            The theorem says the decoupled case is free, not easy to arrange. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> root에서 goal까지의 chain에서 informative collision이 한 번도 안
                            나면 반환된 joint 경로의 비용은 <InlineMath math="\sum_i \ell_i"/>이고{" "}
                            <InlineMath math="\ell_i"/>는 agent <InlineMath math="i"/>의 개별 최적 경로 길이이며 더
                            싼 합법 joint 해는 존재할 수 없다.
                        </p>
                        <p>
                            <strong>증명.</strong> 빈 collision set에서는 모든 expansion이 모든 robot을 policy에
                            고정한다. chain의 각 받아들여진 스텝은 agent <InlineMath math="i"/>를 BFS tree edge 하나만큼
                            goal로 움직이게 하니 이어붙인 경로는 정확히 start에서 goal까지의 장애물 존중 최단 경로이고
                            길이는 goal에서의 공짜 대기를 뺀 <InlineMath math="\ell_i"/>다. 모든 합법 joint 경로는 특히
                            각 agent의 단일-agent 경로이고 스텝 중 agent의 cell을 벗어나는 것은 이 metric에서 1이라 어떤
                            joint 해든 최소 <InlineMath math="\sum_i \ell_i"/>다. 반환된 경계가 bound를 달성하니 joint로
                            최적이다. 이게 말하지 않는 것에 주목하라: 실행이 결합하지 않음을 아무도 보장하지 않는다.{" "}
                            <code>open01_swap</code>에서는 결합이 불가피하고 거기서 sRRT는 optimum 위에 착지한다. 정리는
                            decoupled 경우가 공짜라고 말하지 만들기가 쉽다고 말하지 않는다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what
                    the Python/C++ code below emits. Watch the two regimes. On <code>open01_cross</code> and{" "}
                    <code>maze01_two</code> no collision ever fires: every expansion moves both robots one policy
                    step at once (watch both agents' cells bloom simultaneously), the tree is a single line along
                    the policies, and the cost is exactly the sum of individual optima. Drag an endpoint so the two
                    policy paths meet head-on — <code>open01_swap</code> is that scenario — and watch the behavior
                    change character: the informative collision fires, both robots join every ancestor's set, and from
                    there the tree grows like a plain joint-space RRT steered by goal-biased samples. The seed is part
                    of a run's identity: all three engines replay identical sample streams from it.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과
                    바이트 단위로 같은 엔진입니다. 두 영역을 지켜보세요. <code>open01_cross</code>와{" "}
                    <code>maze01_two</code>에서는 충돌이 한 번도 안 납니다. 모든 expansion이 두 robot을 동시에 policy
                    스텝 하나씩 움직이고 트리는 policy들을 따른 하나의 선이며 비용은 정확히 개별 최적들의 합입니다.
                    endpoint를 끌어 두 policy 경로가 정면으로 만나게 하세요. <code>open01_swap</code>이 그 시나리오입니다.
                    그리고 동작이 성질이 바뀌는 것을 보세요. informative collision이 발화하고 두 robot이 모든 ancestor의
                    set에 합류하고 거기서부터 트리는 goal-biased 표본에 조향되는 plain joint-space RRT처럼 자랍니다.
                    seed는 실행 정체성의 일부입니다. 세 엔진 모두에서 동일한 표본 열이 재생됩니다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live srrt sandbox — byte-identical to the Python/C++ planner. Draw walls, drag endpoints; every edit re-plans and replays: while no collision fires, one expansion moves both agents along their policies at once, and where a collision does fire the affected robots join the collision set and get steered by samples instead",
                "라이브 srrt sandbox — Python/C++ planner와 바이트 단위로 동일합니다. 벽을 그리고 endpoint를 끄면 모든 편집이 재계획과 재생으로 이어집니다: 충돌이 없으면 expansion 하나가 두 agent를 policy를 따라 동시에 움직이고 충돌이 나면 관련된 robot들이 collision set에 합류되어 표본 조향으로 넘어갑니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine
                    that powers the live sandbox above is a third mirror. This time byte-identity needs no lattice
                    trickery: unlike MA-RRT*'s near-ball radius (where libm's log/pow had to be quantized away), every
                    decision in this planner is integer arithmetic — Manhattan sums, cell equality, BFS tie-breaks by
                    fixed neighbor order. The only float anywhere is the PRNG draw itself: MINSTD Lehmer{" "}
                    <InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/> with{" "}
                    <InlineMath math="u = s/(2^{31}-1)"/>, integer-exact in Python and C++ int64 and exact in IEEE
                    doubles everywhere. Parity is exact by construction. The repo conventions layered on the paper
                    (goal biasing, insertion-order tie-breaks, the fixed neighbor order) are part of the contract, and{" "}
                    <code>check-engine-parity</code> verifies on every build that all three engines produce identical
                    traces on every scenario. The code below is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째
                    미러입니다. 이번엔 byte-identity가 격자 양자화 트릭을 필요로 하지 않습니다. MA-RRT*의 near-ball
                    반지름(libm의 log/pow를 양자로 무력화해야 했다)과 달리 이 planner의 모든 결정은 정수 산술입니다.
                    Manhattan 합, 셀 동등, 고정 이웃 순서로의 BFS tie-break. 어디에도 있는 유일한 float는 PRNG draw
                    자체입니다. MINSTD Lehmer <InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/>,{" "}
                    <InlineMath math="u = s/(2^{31}-1)"/>은 Python과 C++ int64에서 integer-exact하고 모든 곳에서 IEEE
                    double로 정확합니다. parity는 구성상 정확합니다. 논문 위에 얹은 저장소 관례들(goal biasing, 삽입
                    순서 tie-break, 고정 이웃 순서)이 계약의 일부이고 <code>check-engine-parity</code>는 빌드마다 세
                    엔진이 모든 시나리오에서 동일한 trace를 내뱉는지 검증합니다. 아래 코드는 발췌가 아니라 실제 소스
                    그대로입니다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/sampling/srrt.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/sampling/srrt.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/sampling/srrt.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/sampling/srrt.hpp`,
                            },
                            {
                                name: "cpp/src/sampling/srrt.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/sampling/srrt.cpp`,
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
                sRRT was introduced by Wagner, Kang and Choset at ICRA 2012; the subdimensional-expansion framework it
                instantiates — individual policies, collision sets, the projection that constructs a variable-
                dimensionality search space — is their own AIJ 2015 treatment of M*'s idea. The paper's own conclusion
                names decoupling (what rM* does for graph search) as sRRT's future work; this repository implements
                the 2012 algorithm exactly as published, grid-discretized like its sibling MA-RRT*.
            </p>} ko={<p>
                sRRT는 Wagner, Kang, Choset가 ICRA 2012에 도입했다. 그것이 구현한 subdimensional expansion
                프레임워크(individual policies, collision sets, 차원 가변 탐색 공간을 구성하는 투사)는 M*의 아이디어를
                그들이 다룬 AIJ 2015 논문이다. 논문 자신의 결론은 decoupling(rM*이 graph search에 대해 하는 것)을
                sRRT의 future work로 이름 그대로 지목한다. 이 저장소는 2012 알고리즘 MA-RRT*의 형제처럼 격자 이산화해
                출판된 그대로 구현한다.
            </p>} />
            <ol>
                <li>
                    G. Wagner, M. Kang, H. Choset,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2012.6225297" target="_blank" rel="noopener noreferrer">
                        <em>Probabilistic path planning for multiple robots with subdimensional expansion</em>
                    </a>,
                    ICRA, 2012.
                </li>
                <li>
                    G. Wagner, H. Choset,{" "}
                    <a href="https://doi.org/10.1016/j.artint.2014.11.001" target="_blank" rel="noopener noreferrer">
                        <em>Subdimensional expansion for multirobot path planning</em>
                    </a>,
                    Artificial Intelligence 219, 2015.
                </li>
            </ol>
        </>
    )
}

export default SubdimensionalRrt
