import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runMaRrtStar} from "../../../libs/algorithms/ma_rrt_star";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/sampling/ma_rrt_star.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/sampling/ma_rrt_star.hpp?raw";
import cppImpl from "../../../../../cpp/src/sampling/ma_rrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 파라미터는 저장소의 configs/sampling/ma_rrt_star.yaml
// 기본값과 동일: seed 42, γ=5, goal biasing 0.5, greedy 예산 200, 반복 예산 300.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runMaRrtStar(map, tasks, {
        seed: 42, gamma: 5, goal_sampling_probability: 0.5,
        greedy_cost_budget: 200, max_iterations: 300,
    })

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. agent 상한은 4:
// 표본이 여전히 통째로 joint 상태라는 사실(차원의 저주)은 페이지의 논지이고,
// 반복당 비용은 트리에만 선형이라 라이브 실행은 실시간으로 남는다.
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

const MaRrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    CBS was the search branch's last word, and it is still enumeration — cleverly
                    scoped, but a space-time graph whose cells are all visited in principle. Now remove
                    enumeration itself. The single-robot escape from enumeration (the sister repository's
                    whole lineage) is to throw samples at the space and connect what sees each other.
                    Karaman & Frazzoli added <em>rewiring</em> to that idea and turned probabilistic
                    completeness into asymptotic optimality: RRT*. This page is what Čáp, Novák, Vokřínek
                    and Pěchouček get when they lift RRT* to k agents: MA-RRT*, one tree over the joint
                    state space — the sampling branch's own coupled pole, the exact counterpart of
                    joint-space A*. The paper discretizes its own waypoints onto a grid (its G-RRT*), so
                    this first member of the sampling branch lives on the very same motion graph as the
                    search branch. What changes is not the space. It is how it gets explored: nothing here
                    ever enumerates a state — states arrive by <em>sample</em>, and optimality arrives by{" "}
                    <em>rewiring</em>.
                </p>}
                ko={<p>
                    CBS는 search 갈래의 마지막 말이었고, 그래도 여전히 열거다. 간섭 지점만 건드릴 뿐인데도
                    space-time 그래프의 칸을 원리적으로는 전부 방문한다. 이제 열거 자체를 걷어 내자. 단일 robot
                    시절 열거에서 벗어나는 탈출구는 자매 저장소의 계보 전체였다. 공간에 표본을 던지고 서로
                    보이는 것들을 잇는 것이다. Karaman과 Frazzoli는 그 생각에 <em>rewiring</em>을 더해서
                    확률적 완전성을 점근 최적성으로 바꿔 놓았다. RRT*다. 이 페이지는 Čáp, Novák, Vokřínek,
                    Pěchouček이 RRT*를 k명의 agent로 끌어올릴 때 얻는 것이다. joint 상태 공간 위의 트리 하나
                    MA-RRT*, sampling 갈래 자신의 coupled 극단이며 joint-space A*의 그대로의 대응물이다. 논문은
                    자기 waypoint를 스스로 격자에 이산화한다(논문의 G-RRT*). 그래서 sampling 갈래의 첫 회원이
                    search 갈래와 정확히 같은 motion graph 위에 산다. 바뀌는 것은 공간이 아니다. 어떻게
                    탐색하느냐다. 여기서는 어떤 상태도 열거되지 않는다. 상태는 <em>표본</em>으로 도착하고,
                    최적성은 <em>rewiring</em>으로 도착한다.
                </p>}
            />

            <h2>{t("From Joint States to Random Trees", "joint 상태에서 random tree로")}</h2>
            <T
                en={<>
                    <p>
                        The state is the same tuple as the joint-space page:{" "}
                        <InlineMath math="x = (x_0, \dots, x_{k-1}) \in V^k"/>, one waypoint per agent over
                        the motion graph <InlineMath math="G_M = (V, E)"/> whose vertices are free cells and
                        whose primitives are the unit moves plus the wait self-loop. The cost metric is the
                        same too: a primitive costs 0 exactly when start = end = that agent's goal, so a
                        chain's cost is again summed arrival time. What was enumerated before is now sampled —
                        and that flips every ingredient of the algorithm.
                    </p>
                    <p>
                        Plain RRT connects each sample to its nearest tree vertex and never looks back; the
                        tree is probabilistically complete but its cost converges to nothing. RRT*'s move is
                        to stop trusting “nearest”: a new vertex scans all tree vertices within a{" "}
                        <em>shrinking</em> radius, takes whichever parent reaches it cheapest, and offers
                        every near vertex a cheaper route through itself:
                    </p>
                    <BlockMath math="r_n = \max\left\{\gamma \left(\frac{\log n}{n}\right)^{1/d},\; m\right\}"/>
                    <Terms items={[
                        ["x", <>the joint state — the same tuple as the joint-space page's, every coordinate a
                            free cell of the motion graph <InlineMath math="G_M"/></>],
                        ["r_n", <>the shrinking radius. With n tree nodes it selects which vertices are rewiring
                            candidates; it shrinks as n grows but never below m</>],
                        ["\\gamma", <>the shrinkage constant the paper leaves a free parameter; this repository
                            uses 5.0. Larger rewires more aggressively</>],
                        ["d", <>dimensionality of the joint state space, <InlineMath math="2k"/> — each agent's
                            planar waypoint adds one dimension. The exponent <InlineMath math="1/d"/> is where
                            the curse of dimensionality lives</>],
                        ["m", <>the floor: the longest primitive (one cell per unit time). Below one edge length
                            the ball would kill rewiring, so it gets a floor</>],
                    ]}/>
                    <p>
                        MA-RRT* lifts every ingredient to the joint state. A sample is a whole tuple: with
                        probability <InlineMath math="p_{\text{goal}}"/> it is the goal tuple itself (goal
                        biasing), otherwise each agent's waypoint is drawn uniformly from{" "}
                        <InlineMath math="V"/>, in agent order. Steering is the paper's GREEDY: every agent
                        steps simultaneously toward its own coordinate of the sample, and a step that would
                        collide aborts the segment — conflicts are never steered through, they end the walk.
                        NEAREST and NEAR run on the joint distance{" "}
                        <InlineMath math="\sum_i \lVert x_i - y_i \rVert"/>, which is exactly a lower bound on
                        the cost of any joint transition between the two states. The tree's vertices are joint
                        states, its edges are whole conflict-free segments, and when the goal tuple itself
                        becomes a vertex, the chain from root to it <em>is</em> the answer — concatenated per
                        agent and trimmed after each agent's last move.
                    </p>
                </>}
                ko={<>
                    <p>
                        상태는 joint-space 페이지와 같은 tuple이다. agent마다 waypoint 하나씩,{" "}
                        <InlineMath math="x = (x_0, \dots, x_{k-1}) \in V^k"/>. vertex가 자유 셀이고 primitive가
                        단위 이동 + wait self-loop인 motion graph <InlineMath math="G_M = (V, E)"/> 위의 tuple이다.
                        비용 척도도 같다. start = end = 자기 goal일 때만 primitive 비용이 0이니 chain의 비용은
                        다시 도착 시각의 합이다. 전에 열거되던 것이 이제 표본으로 온다. 그리고 그 한 번이 알고리즘의
                        모든 재료를 뒤집는다.
                    </p>
                    <p>
                        plain RRT는 각 표본을 최근접 트리 vertex에 연결하고 돌아보지 않는다. 트리는 확률적으로
                        완전하지만 비용은 아무 데도 수렴하지 않는다. RRT*의 수는 “최근접”을 믿는 것을 그만두는 것이다.
                        새 vertex는 <em>수축하는</em> 반지름 안의 트리 vertex를 전부 훑어서 가장 싸게 도달하는 parent를
                        고르고, 가까운 vertex마다 자기를 통하는 더 싼 경로를 제안한다:
                    </p>
                    <BlockMath math="r_n = \max\left\{\gamma \left(\frac{\log n}{n}\right)^{1/d},\; m\right\}"/>
                    <Terms items={[
                        ["x", <>joint 상태. joint-space 페이지와 같은 tuple이고 각 좌표는 motion graph{" "}
                            <InlineMath math="G_M"/>의 자유 셀이다</>],
                        ["r_n", <>수축하는 반지름. 트리 노드 n개일 때 rewiring 후보를 고르는 반지름. n이 커지면
                            줄지만 m 아래로는 내려가지 않는다</>],
                        ["\\gamma", <>수축 상수. 논문이 자유 파라미터로 남긴 값이고 이 저장소는 5.0을 쓴다. 크게
                            쓰면 더 공격적으로 rewire한다</>],
                        ["d", <>joint 상태 공간의 차원 <InlineMath math="2k"/>. agent마다 평면 waypoint 하나가
                            차원을 하나씩 더한다. 지수 <InlineMath math="1/d"/>가 차원의 저주가 사는 자리다</>],
                        ["m", <>하한 = 가장 긴 primitive의 길이(단위 시간당 셀 한 칸). 반지름이 간선 하나보다
                            작아지면 rewiring이 죽으므로 마룻값을 깐다</>],
                    ]}/>
                    <p>
                        MA-RRT*는 이 재료를 전부 joint 상태로 끌어올린다. 표본이 통째로 tuple이다. 확률{" "}
                        <InlineMath math="p_{\text{goal}}"/>로 goal tuple 자체이고, 아니면 agent 순서대로 각
                        waypoint를 <InlineMath math="V"/>에서 균일하게 뽑는다. 조향은 논문의 GREEDY다. 모든 agent가
                        표본의 자기 좌표로 동시에 한 칸씩 가고, 충돌할 스텝은 구간을 중단시킨다. conflict는 조향을
                        통과하는 게 아니라 조향을 끝낸다. NEAREST와 NEAR은 joint 거리{" "}
                        <InlineMath math="\sum_i \lVert x_i - y_i \rVert"/> 위에서 돈다. 이 거리가 두 상태 사이 어떤
                        joint 전이의 비용보다도 작다는 것은 정확히 아래에서 증명한다. 트리의 vertex는 joint 상태이고
                        edge는 통째로 conflict-free한 구간이며, goal tuple 자체가 vertex가 되는 순간 root에서 그까지의
                        chain이 그대로 답이다. agent별로 이어붙이고 각 agent의 마지막 이동 이후를 잘라 낸다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Probabilistically complete, asymptotically optimal.</strong> The paper's
                            guarantees are RRT*'s lifted to the joint space: a solution is found almost surely
                            if one exists, and the tree's cost converges to the optimum as iterations go to
                            infinity. Both are limit statements. A finite budget ends in an honest “no solution
                            found within budget”, never an unsolvability verdict — the same honesty CBS's
                            expansion budget buys. And asymptotic is not now: on <code>open01_cross</code> this
                            planner hits cost 33, exactly joint-space A*'s optimum, but on{" "}
                            <code>maze01_two</code> it reports 118 where the exact answer is 66. The tree was
                            still improving when its budget ran out.
                        </li>
                        <li>
                            <strong>The same objective as the search branch.</strong> A primitive costs 0 exactly
                            at one's own goal and 1 per step otherwise, so a chain's cost is summed arrival time{" "}
                            <InlineMath math="\sum_k T_k"/> — literally the number every search-branch page
                            reports. That is what makes 118 vs. 66 a fair sentence to write.
                        </li>
                        <li>
                            <strong>Paid in dimension, not in enumeration.</strong> Per iteration this planner
                            touches the tree it has grown, never{" "}
                            <InlineMath math="|V|^k"/>: on <code>maze01_two</code> it expanded 102 joint states
                            where joint-space A* needed 10,344. But dimension is not free either — every agent
                            added doubles down on the exponent in{" "}
                            <InlineMath math="(\log n/n)^{1/2k}"/>, and a sample must land near a whole joint
                            state that can still reach the goal. The paper's own experiments show exactly this
                            trade: sampling scales better than forward search precisely where maps are large
                            and sparse, which is where enumeration drowns.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>확률적으로 완전하고 점근적으로 최적이다.</strong> 논문의 보장은 RRT*의 보장을
                            joint 상태로 끌어올린 것이다. 해가 존재하면 거의 확실히 찾고, 반복이 무한으로 가면 트리의
                            비용이 최적에 수렴한다. 둘 다 극한 명제다. 유한한 예산은 정직하게 “예산 안에서 해를 찾지
                            못했다”로 끝나고 unsolvability의 판정이 절대 아니다. CBS의 확장 예산이 산 것과 같은 정직함이다.
                            그리고 점근은 지금이 아니다. <code>open01_cross</code>에서는 비용 33, 정확히 joint-space A*의
                            최적값을 맞히지만 <code>maze01_two</code>에서는 정확한 답이 66인 곳에 118을 보고한다. 예산이
                            떨어질 때 트리는 아직 개선 중이었다.
                        </li>
                        <li>
                            <strong>목적함수는 search 갈래와 동일하다.</strong> primitive는 자기 goal에서 정확히 0,
                            그 외 스텝당 1이다. 그래서 chain의 비용은 도착 시각의 합{" "}
                            <InlineMath math="\sum_k T_k"/>이고, 이건 말 그대로 search 갈래 모든 페이지가 보고해 온 그
                            숫자다. 118 대 66이라는 문장을 공정하게 쓸 수 있는 이유가 이것이다.
                        </li>
                        <li>
                            <strong>대가의 통화는 열거가 아니라 차원이다.</strong> 반복마다 이 planner가 건드리는 것은
                            자라난 트리뿐이고 <InlineMath math="|V|^k"/>를 건드리지 않는다. <code>maze01_two</code>에서
                            joint 상태 102개 확장으로 끝났고 joint-space A*는 10,344개가 필요했다. 그러나 차원도 공짜가
                            아니다. agent가 한 대 늘 때마다 <InlineMath math="(\log n/n)^{1/2k}"/>의 지수가 두 배로
                            걸리고, 표본은 goal에 도달 가능한 joint 상태 근처 전체에 떨어져야 한다. 논문의 실험이 정확히
                            이 교환을 보여 준다. 샘플링은 맵이 크고 sparse한 곳에서 forward search보다 잘 확장되고,
                            정확히 그곳에서 열거가 잠긴다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One loop, and the multi-agent content lives inside two of its steps: SAMPLE draws a whole
                    joint state, and GREEDY steers every agent at once. Everything else is RRT* verbatim —
                    nearest, near-ball, parent selection, rewiring — run on joint states with joint distances.
                    The paper leaves three things ambiguous (which point GREEDY starts from during rewiring,
                    what a duplicate sample means, how greedy's ties break); every choice is fixed below and
                    identical in all three language engines.
                </p>}
                ko={<p>
                    루프 하나이고 multi-agent 내용은 그중 두 단계 안에 들어 있다. SAMPLE은 통째로 joint 상태를
                    뽑고 GREEDY는 모든 agent를 동시에 조향한다. 나머지는 그대로 RRT*다. 최근접, near ball, parent
                    선택, rewiring. 전부 joint 상태 위에서 joint 거리로 돈다. 논문이 모호하게 남긴 세 가지(rewiring에서
                    GREEDY가 어디서 시작하는지, duplicate 표본이 의미하는 것, greedy의 동률 처리)를 아래에서 모두
                    고정했고 세 언어 엔진에 동일하다.
                </p>}
            />
            <Pseudocode code={`# T = (V, E): V는 joint 상태의 집합 (삽입 순서가 순서다), root = start tuple        # 1
repeat max_iterations times:                                                     # 2
    u ← uniform(); x ← goal w.p. p_goal else k independent uniform waypoints     # 3
    x_near ← argmin_{v ∈ V} d_joint(v, x)          (ties → lowest insertion index)   # 4
    σ ← GREEDY(x_near, x): every agent steps toward its own coordinate of x;      # 5
        a step that collides (or waits everywhere) ends the segment; cost ≤ c_max # 6
    if σ empty or endpoint already in V: continue                                 # 7
    X_near = {v ∈ V : d_joint(v, x_new) ≤ r_n},  r_n = max{γ(log n/n)^{1/d}, m}   # 8
    parent: default x_near; strict cost improvement scan over X_near              # 9
    skip if chain(x_new) + d_joint(x_new, goal) > incumbent (informed pruning)    # 10
    insert x_new; rewire every v ∈ X_near through it on strict improvement;       # 11
    incumbent := exact cost of the goal chain (recomputed after every rewiring)   # 12`}
            />
            <T
                en={<ol>
                    <li>The tree is the answer. Vertices are joint states indexed by insertion order, and that
                        order is load-bearing: it breaks NEAREST ties and orders the NEAR scan. The root is the
                        start tuple; its segment has no steps, so it contributes 0 to every chain cost.</li>
                    <li>The loop runs a fixed budget of iterations — “until interrupted” made finite. The tree
                        only ever improves (see step 12), so more iterations never hurt a found solution; they
                        only refine it.</li>
                    <li>SAMPLE: one draw decides the bias first. With probability p_goal the sample is the goal
                        tuple outright (§4.3's informed sampling); otherwise every agent's waypoint is drawn
                        uniformly from V, in agent order — k draws, not one joint draw.</li>
                    <li>NEAREST scans the whole tree on the joint distance; ties break to the lowest insertion
                        index, which is why insertion order is part of the algorithm's identity, not an
                        implementation detail.</li>
                    <li>GREEDY (the paper's Algorithm 4): every agent simultaneously steps to the child that
                        first strictly minimizes Euclidean distance to its own coordinate of the target — ties
                        broken by the map's fixed neighbor order (up/down/left/right, then wait).</li>
                    <li>A step that would repeat a cell (vertex conflict) or swap a pair (edge conflict) aborts:
                        the segment ends at the pre-step snapshot. A step where every agent's argmin picked wait
                        changes no state and can never reach the target either, so it ends the walk too. The
                        accumulated primitive cost is capped at c_max.</li>
                    <li>An empty segment means the sample was already a vertex — and so does a partial segment
                        landing on an existing vertex. The paper's set-union line never says what a duplicate
                        means for the parent edge; not adding one is the only reading that keeps T a tree.</li>
                    <li>NEAR: every vertex within r_n of x_new, scanned in insertion order. n is |V| before this
                        insertion. The radius is floored to a 1/64 lattice so libm log/pow rounding cannot move
                        the ball across language boundaries.</li>
                    <li>Parent selection defaults to x_near (the vertex GREEDY extended from) and takes a
                        candidate parent only on strict cost improvement: chain(x′) + cost(GREEDY(x′, x_new)){" "}
                        <InlineMath math="\lt"/> current. A candidate whose greedy walk does not reach x_new is
                        not a candidate at all.</li>
                    <li>Informed pruning (§4.4): if the new vertex's chain cost plus the metric lower bound to
                        the goal tuple exceeds the incumbent solution, the vertex is dead weight — it is not
                        added and nothing rewires through it. Before the first solution the incumbent is infinite,
                        so nothing can be pruned.</li>
                    <li>Rewiring: a near vertex re-parents through the new vertex only on strict improvement.
                        The paper's Algorithm 2 line 19 names GREEDY(G_M, x, x_near) with x still naming the
                        sample; that is read as a typo for x_new — reaching x_near <em>through</em> the new
                        vertex is the whole point of rewiring.</li>
                    <li>The incumbent is recomputed exactly (chain walk over immutable segment costs) after every
                        iteration once the goal tuple has become a vertex. The answer itself: chain root → goal,
                        concatenated per agent with each duplicated head skipped, trimmed after each agent's last
                        move — trailing waits sit at their own goal and cost 0, so trimming cannot change the cost.</li>
                </ol>}
                ko={<ol>
                    <li>트리가 답이다. vertex는 joint 상태이고 index는 삽입 순서다. 이 순서는 기능을 한다. NEAREST
                        동률을 깨고 NEAR scan의 순서를 정한다. root는 start tuple이고 그 구간은 스텝이 없으니 모든
                        chain 비용에 0을 더한다.</li>
                    <li>루프는 반복 예산을 고정하고 돈다. “interrupt될 때까지”를 유한하게 만든 것뿐이다. 트리는
                        좋아지기만 하므로(12단계 참고) 반복이 해를 망칠 일은 없고 개선만 한다.</li>
                    <li>SAMPLE: draw 하나가 먼저 bias를 정한다. 확률 p_goal로 표본은 goal tuple 자체고(§4.3의 informed
                        sampling), 아니면 agent 순서대로 각 waypoint를 V에서 균일하게 뽑는다. joint draw 하나가
                            아니라 k개의 draw다.</li>
                    <li>NEAREST는 트리를 joint 거리로 훑고 동률은 최소 삽입 index로 깬다. 그래서 삽입 순서가 구현
                        디테일이 아니라 알고리즘 정체성의 일부다.</li>
                    <li>GREEDY(논문의 Algorithm 4): 모든 agent가 동시에, 자기 좌표와의 Euclidean 거리를 처음 strict-min으로
                        줄이는 child로 한 칸 간다. 동률은 맵의 고정 이웃 순서(up/down/left/right, 그다음 wait)로 깬다.</li>
                    <li>셀을 반복하는 스텝(vertex conflict)이나 쌍이 셀을 맞바꾸는 스텝(edge conflict)은 abort다. 구간은
                        직전 snapshot에서 끝난다. 모든 agent의 argmin이 wait을 고른 스텝은 상태를 안 바꾸고 target에도
                        닿지 못하니 역시 중단한다. 누적 primitive 비용은 c_max에서 끊긴다.</li>
                    <li>빈 구간은 표본이 이미 vertex였다는 뜻이고, 기존 vertex에 도착한 부분 구간도 같다. 논문의
                        set-union 줄은 duplicate가 parent edge에 무슨 의미인지 말하지 않는다. 트리를 트리로 남기는 유일한
                        읽기가 추가하지 않는 것이다.</li>
                    <li>NEAR: x_new에서 r_n 이내의 모든 vertex를 삽입 순서로 훑는다. n은 이 삽입 전 |V|다. 반지름을
                        1/64 격자로 내림해서 libm log/pow의 반올림이 언어 경계를 넘어 공을 움직이지 못하게 한다.</li>
                    <li>parent 선택의 기본값은 x_near(GREEDY가 출발한 vertex)이고 strict 개선에서만 후보를 받는다:
                        chain(x′) + cost(GREEDY(x′, x_new)) &lt; 현재. x_new에 도달하지 못하는 GREEDY는 후보가 아니다.</li>
                    <li>informed pruning(§4.4): 새 vertex의 chain 비용 + goal까지 metric lower bound가 incumbent를
                        넘으면 그 vertex는 dead weight다. 추가되지 않고 아무것도 그로 rewire하지 않는다. 첫 해 이전에는
                        incumbent가 무한이라 아무것도 prune될 수 없다.</li>
                    <li>Rewiring: 가까운 vertex는 strict 개선에서만 새 vertex로 parent를 옮긴다. 논문 Algorithm 2의
                        19번째 줄은 GREEDY(G_M, x, x_near)에서 x를 여전히 표본을 가리키게 쓰는데, 이건 x_new의 오타로
                        읽는다. 새 vertex를 <em>통해서</em> x_near에 도달하는 것이 rewiring의 전부다.</li>
                    <li>incumbent는 goal tuple이 vertex가 된 뒤 매 반복 정확히 다시 계산된다(chain을 불변 구간 비용 위로
                        거슬러 합산). 답 자체도 그 chain이다: root → goal을 agent별로 이어붙이고 중복된 각 구간의 head를
                        건너뛰고, 각 agent의 마지막 이동 이후를 잘라 낸다. trailing wait은 자기 goal에서 0이므로 자르기가
                        비용을 바꿀 수 없다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<>
                    <p>
                        What it guarantees is the paper's pair of limit statements: probabilistic completeness and
                        asymptotic optimality, RRT*'s guarantees lifted to the joint space. What it cannot do is
                        promise anything about a finite run — the budget always ends, and when it ends without the
                        goal tuple ever having become a vertex, the honest report is “no solution found within
                        budget”, never “unsolvable”. Two things are provable here, though: every plan this planner
                        does emit is valid, and the informed pruning can never hide a better solution. Expand for
                        both.
                    </p>
                </>}
                ko={<>
                    <p>
                        보장하는 것은 논문의 극한 명제 쌍이다. 확률적 완전성과 점근 최적성, joint 상태로 끌어올려진
                        RRT*의 보장. 못 하는 것은 유한한 실행에 대해 무엇도 약속하지 못하는 것이다. 예산은 항상 끝나고
                        goal tuple이 끝내 vertex가 되지 못한 채 끝나면 정직한 보고는 “예산 안에서 해를 찾지 못했다”이지
                        “unsolvable”이 아니다. 그래도 여기서 증명 가능한 두 가지는 있다. 이 planner가 실제로 내보내는
                        모든 계획은 합법이고, informed pruning은 더 나은 해를 절대 숨길 수 없다. 둘 다 접힌 증명을 편다.
                    </p>
                </>}
            />
            <Proof title={t("Theorem (every emitted plan is valid)", "정리 (방출되는 모든 계획은 합법이다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Every path pair the planner emits is collision-free, and its
                            cost equals the summed arrival times of what was actually driven.
                        </p>
                        <p>
                            <strong>Proof.</strong> A tree edge exists only for a segment GREEDY accepted, and
                            GREEDY accepts a step only after checking it: no two agents land on one cell (vertex
                            conflict) and no pair swaps cells across the step (edge conflict). Concatenating
                            segments along a parent chain therefore concatenates step-wise conflict-free motion —
                            consecutive segments meet exactly at the shared joint state, so the concatenation is
                            itself a legal joint path. Trimming cuts each agent's path after its last move; every
                            cut step was a wait at that agent's own goal, which costs 0, so neither the cost nor
                            any other agent's motion changes. The emitted per-agent paths are the coordinates of
                            that joint path, and their lengths minus one are exactly what the summed chain cost
                            charged for them. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 이 planner가 내보내는 모든 경로 쌍은 conflict-free이고 비용은
                            실제로 주행한 것의 도착 시각의 합과 같다.
                        </p>
                        <p>
                            <strong>증명.</strong> 트리 edge는 GREEDY가 받아들인 구간에 대해서만 존재하고 GREEDY는
                            스텝을 검사한 뒤에만 받아들인다. 두 agent가 한 셀에 착지하지 않고(vertex conflict) 쌍이
                            스텝을 넘어 셀을 맞바꾸지도 않는다(edge conflict). 그래서 parent chain을 따라 구간들을
                            이어붙이면 스텝별로 conflict-free한 움직임들이 이어붙여진다. 연속한 구간은 공유된 joint
                            상태에서 정확히 만나니 연결 자체가 합법적인 joint 경로다. 자르기는 각 agent의 마지막 이동
                            이후를 잘라내고 잘린 모든 스텝은 자기 goal에서의 대기였는데 그 비용은 0이다. 그래서 비용도
                            다른 agent의 움직임도 변하지 않는다. 방출된 agent별 경로는 그 joint 경로의 좌표들이고 길이
                            빼기 1이 chain 비용이 정확히 계산한 값이다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Theorem (pruning cannot hide a better solution)", "정리 (pruning은 더 나은 해를 숨길 수 없다)")}>
                <T
                en={<>
                    <p>
                        <strong>Claim.</strong> Skipping a candidate whose chain cost plus the metric lower bound
                        exceeds the incumbent can never discard every chain that would have beaten the incumbent.
                    </p>
                    <p>
                        <strong>Proof.</strong> A joint path from{" "}
                        <InlineMath math="x_{\text{new}}"/> to the goal tuple moves each agent{" "}
                        <InlineMath math="i"/> along some walk; every step that changes agent <InlineMath math="i"/>
                        's cell costs 1 and moves it exactly one cell, while a wait at its own goal costs 0, so any
                        continuation out of <InlineMath math="x_{\text{new}}"/> costs at least{" "}
                        <InlineMath math="\sum_i \lVert x_{{\text{new}},i} - g_i \rVert"/> — the metric is admissible.
                        Every chain through <InlineMath math="x_{\text{new}}"/> therefore costs at least{" "}
                        <InlineMath math="\mathrm{cost}(x_{\text{new}}) + d_{\text{joint}}(x_{\text{new}}, g)"/>, and
                        when that already exceeds the incumbent strictly, no chain through the candidate could tie or
                        beat it. The strict inequality is what makes this safe: an alternative exactly as good as the
                        incumbent survives pruning. And before any solution exists the incumbent is infinite, so the
                        tree grows unpruned until the first goal vertex lands — which is also why the pruning can never
                        cause a failure to find a plan at all. <InlineMath math="\blacksquare"/>
                    </p>
                </>}
                ko={<>
                    <p>
                        <strong>주장.</strong> chain 비용 + metric lower bound가 incumbent를 초과하는 candidate을
                        건너뛰는 것은 incumbent를 이겼을 모든 chain을 폐기할 수 없다.
                    </p>
                    <p>
                        <strong>증명.</strong> <InlineMath math="x_{\text{new}}"/>에서 goal tuple로 가는 joint 경로는
                        agent <InlineMath math="i"/>를 어떤 walk를 따라 이동시키고, cell을 바꾸는 스텝마다 비용 1에
                        정확히 한 칸을 움직이고 자기 goal에서의 대기는 0이다. 그래서{" "}
                        <InlineMath math="x_{\text{new}}"/>에서 나가는 모든 연속 경로는 최소{" "}
                        <InlineMath math="\sum_i \lVert x_{{\text{new}},i} - g_i \rVert"/>이고 metric은 admissible하다.
                        따라서 <InlineMath math="x_{\text{new}}"/>를 지나는 모든 chain은 최소{" "}
                        <InlineMath math="\mathrm{cost}(x_{\text{new}}) + d_{\text{joint}}(x_{\text{new}}, g)"/>이고
                        그게 이미 incumbent를 strict하게 초과하면 candidate을 지나는 어떤 chain도 동률 이상일 수 없다.
                        strict 부등식이 이걸 안전하게 만드는 부분이다. incumbent와 정확히 같은 해는 pruning에서 살아남는다.
                        그리고 해가 존재하기 전에는 incumbent가 무한이라 첫 goal vertex가 도착할 때까지 트리는 prune 없이
                        자란다. 그래서 pruning이 계획 찾기를 실패하게 만들 수도 없다. <InlineMath math="\blacksquare"/>
                    </p>
                </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what
                    the Python/C++ code below emits. The seed is part of a run's identity: all three engines replay
                    identical sample streams from it, and the presets below are exactly the scenarios the search
                    branch solved optimally, so you can watch what sampling costs. On <code>open01_cross</code>{" "}
                    the goal tuple itself gets sampled on the very first iteration and GREEDY's simultaneous descent
                    is conflict-free: two expanded nodes, cost 33 — exactly joint-space A*’s optimum for one
                    hundredth of its expansions. On <code>maze01_two</code> the tree needs 102 nodes to find a plan
                    at all (joint-space A* enumerated 10,344 states) and its cost, 118 against the exact 66, is what
                    asymptotic means before the limit. Drag endpoints and add agents: watch expansions bloom at both
                    agents' cells at once (a joint state is being expanded, not an individual path), watch the
                    incumbent improve after the first solution — rewiring keeps working until the budget ends.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과
                    바이트 단위로 같은 엔진입니다. seed는 실행 정체성의 일부입니다. 세 엔진 모두에서 동일한 표본 열이
                    재생되고 아래 preset은 search 갈래가 최적적으로 푼 바로 그 시나리오들이니 sampling의 대가를 직접
                    비교할 수 있습니다. <code>open01_cross</code>에서는 첫 반복에서 goal tuple 자체가 샘플되고 GREEDY의
                    동시 하강이 conflict-free합니다. 확장 2개, 비용 33 — joint-space A* 최적값과 정확히 같고 확장은
                    1/50입니다. <code>maze01_two</code>에서는 트리가 계획을 찾기 위해만 102개가 필요하고(joint-space A*는
                    상태를 10,344개 열거했습니다) 비용 118은 정확한 답 66 앞에서 극한 이전의 점근이 무엇인지 보여 줍니다.
                    endpoint를 끌어 agent를 추가해 보라. 확장이 두 agent의 셀에 동시에 피어나는 것(개별 경로가 joint
                    상태가 확장되는 것이다)과 첫 해 이후에도 incumbent가 계속 개선되는 것을 보라. rewiring은 예산이 끝날
                    때까지 일한다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live ma_rrt_star sandbox — byte-identical to the Python/C++ planner. Draw walls, drag endpoints; every edit re-plans and replays: joint-state expansions bloom on all agents' cells at once, the goal chain appears when sampled, rewiring refines it after",
                "라이브 ma_rrt_star sandbox — Python/C++ planner와 바이트 단위로 동일합니다. 벽을 그리고 endpoint를 끄면 모든 편집이 재계획과 재생으로 이어집니다: joint 상태 확장이 모든 agent의 셀에 동시에 피어나고 goal chain은 샘플되는 순간 나타나고 rewiring이 그 뒤로 다듬습니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser
                    engine that powers the live sandbox above is a third mirror. Byte-identical trees from one seed
                    need three cross-language coincidences, and all three are engineered: the PRNG is MINSTD Lehmer
                    (<InlineMath math="s \leftarrow 16807 s \bmod (2^{31}-1)"/>), integer-exact in Python and C++
                    int64 and exact in IEEE doubles everywhere; distances are sums of correctly-rounded sqrt over
                    integer deltas, which already agree bit-for-bit; and{" "}
                    <InlineMath math="r_n"/> is floored to a 1/64 lattice so that libm's log/pow cannot disagree at
                    the ULP level. Every tie-break the paper leaves open (first-min child in fixed neighbor order,
                    lowest insertion index) is part of the contract, and <code>check-engine-parity</code> verifies on
                    every build that all three engines produce identical traces on every scenario. The code below is
                    the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째
                    미러다. seed 하나로 바이트 단위로 동일한 트리가 내려면 언어를 넘는 일치 세 가지가 필요하고 셋 다
                    엔지니어링된 것입니다. PRNG는 MINSTD Lehmer(<InlineMath math="s \leftarrow 16807 s \bmod (2^{31}-1)"/>)이고
                    Python과 C++ int64에서 integer-exact하며 IEEE double 어디서나 정확합니다. 거리는 정수 차이의
                    correctly-rounded sqrt의 합이라 원래 비트 단위로 일치하고 <InlineMath math="r_n"/>은 1/64 격자로
                    내림해서 libm의 log/pow가 ULP 수준으로 어긋날 수 없게 합니다. 논문이 열어 둔 모든 tie-break(고정
                    이웃 순서의 첫 strict-min, 최소 삽입 index)이 계약의 일부이고 <code>check-engine-parity</code>는
                    빌드마다 세 엔진이 모든 시나리오에서 동일한 trace를 만드는지 검증합니다. 아래 코드는 발췌가 아니라
                    실제 소스 그대로입니다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/sampling/ma_rrt_star.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/sampling/ma_rrt_star.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/sampling/ma_rrt_star.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/sampling/ma_rrt_star.hpp`,
                            },
                            {
                                name: "cpp/src/sampling/ma_rrt_star.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/sampling/ma_rrt_star.cpp`,
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
                MA-RRT* was introduced for cooperative pathfinding by Čáp, Novák, Vokřínek and Pěchouček at
                AAMAS 2013; the asymptotic-optimality machinery it lifts — the shrinking near-ball, rewiring, the
                analysis this page's honesty section leans on — is Karaman and Frazzoli's RRT*.
            </p>} ko={<p>
                MA-RRT*는 cooperative pathfinding를 위해 Čáp, Novák, Vokřínek, Pěchouček가 AAMAS 2013에 도입했다.
                이 끌어올린 점근 최적성 메커니즘(수축하는 near ball, rewiring, 이 페이지의 정직함 섹션이 기대는 그
                해석)은 Karaman과 Frazzoli의 RRT*다.
            </p>} />
            <ol>
                <li>
                    M. Čáp, P. Novák, J. Vokřínek, M. Pěchouček,{" "}
                    <a href="https://doi.org/10.5555/2484920.2485174" target="_blank" rel="noopener noreferrer">
                        <em>Multi-agent RRT*: sampling-based cooperative pathfinding</em>
                    </a>,
                    AAMAS, 2013.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank" rel="noopener noreferrer">
                        <em>Sampling-based algorithms for optimal motion planning</em>
                    </a>,
                    International Journal of Robotics Research, 2011.
                </li>
            </ol>
        </>
    )
}

export default MaRrtStar
