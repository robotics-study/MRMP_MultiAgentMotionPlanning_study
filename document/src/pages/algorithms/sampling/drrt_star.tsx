import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runDrrtStar} from "../../../libs/algorithms/drrt_star";
import {cellToWorld, GridMap} from "../../../libs/grid";
import {Cell, Point, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/sampling/drrt_star.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/sampling/drrt_star.hpp?raw";
import cppImpl from "../../../../../cpp/src/sampling/drrt_star.cpp?raw";

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 파라미터는 저장소의 configs/sampling/drrt_star.yaml
// 기본값과 동일: seed 42, robot당 120 rejection sample, radius bound의 eta 2.0,
// goal bias 0.1, expansion 예산 300. disc 반지름도 시나리오와 같은 0.2 (핸들 드래그의
// 셀 스냅은 그대로고, runLive가 cellToWorld로 세계 좌표로 바꾼다 — 데모/parity와 동일한 좌표계).
const RADIUS = 0.2

const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runDrrtStar(
        map,
        tasks.map(([s, g]) => [cellToWorld(map, s), cellToWorld(map, g)] as [Point, Point]),
        tasks.map(() => RADIUS),
        {seed: 42, samples_per_robot: 120, eta: 2.0, goal_sample_rate: 0.1, max_iterations: 300},
    )

// 시나리오 preset — 셀 좌표는 데모/parity의 world 좌표와 같은 지점의 셀 중심이다
// (open01은 resolution 0.5, origin [0,0]이라 cell (10,1)의 중심이 정확히 (0.75, 4.75)).
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross_discs", map: "open01", radius: RADIUS, agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap_discs", map: "open01", radius: RADIUS, agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
]

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const DrrtStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    dRRT was probabilistically complete and that was the whole promise — its own conclusion named
                    asymptotic optimality as future work. Shome, Solovey, Dobson, Halperin and Bekris take exactly
                    that future work and hand it back. Two changes, both surgical. The individual roadmaps switch
                    from <em>k</em>-nearest fanout to a connection radius <InlineMath math="r(n)"/> that grows with the
                    sample count — sampling density is now inside a theorem instead of a knob tuned by feel. And the
                    tree search switches from oracle growth plus a decoupled local connector to RRT*-style rewiring:
                    every joint vertex carries a cost-to-come, candidates re-parent when they become cheaper, and
                    branch-and-bound refuses any expansion that cannot beat the incumbent. Waiting becomes native —
                    adjacency lists include self-loops, so a tensor edge may move a subset of the robots — and dRRT's
                    priority-DAG connector disappears entirely; the goal tuple is reached by tree search alone. The
                    oracle becomes <InlineMath math="I_d"/>: a goal-biased sample switches every robot to argmin over
                    that robot's own shortest-path-to-goal length, an unbiased sample picks uniformly random neighbors.
                    Same implicit tensor roadmap as dRRT, now searched so the tree converges to the optimal path —
                    the coupled pole of the sampling branch at its sharpest.
                </p>}
                ko={<p>
                    dRRT는 probabilistically complete였고 그게 약속의 전부였다 — 논문 결론 자체가 asymptotic optimality를
                    future work로 지목했다. Shome, Solovey, Dobson, Halperin, Bekris가 바로 그 future work를 가져와 되돌려
                    준다. 변화는 둘이고 둘 다 정밀하다. 개별 roadmap이 <InlineMath math="k"/>-nearest fanout에서 표본 수와
                    함께 자라는 connection radius <InlineMath math="r(n)"/>으로 바뀌고 — 표본 밀도가 감으로 튜닝하는 knob 대신
                    정리 안에 들어간다 — tree 탐색은 oracle growth + decoupled connector에서 RRT*식 rewiring으로 바뀐다:
                    모든 joint vertex가 cost-to-come을 갖고, candidate이 더 싸지면 re-parent되고, branch-and-bound가 incumbent를
                    이길 수 없는 expansion을 거부한다. 대기가 native가 된다 — 인접 목록에 self-loop이 들어가니 tensor edge가
                    robot 부분집합만 움직일 수 있고 — dRRT의 priority-DAG connector는 통째로 사라진다. goal tuple은 tree 탐색만으로
                    도달된다. oracle은 <InlineMath math="I_d"/>가 된다: goal-biased 표본이면 모든 robot이 자기 goal까지 최단경로
                    길이에 대한 argmin으로 전환되고, unbiased 표본이면 균일 random neighbor를 고른다. dRRT와 같은 implicit tensor
                    roadmap을 이제 tree가 최적 경로에 수렴하도록 탐색한다 — sampling 갈래의 coupled 극점이 가장 날카로운 지점이다.
                </p>}
            />

            <h2>{t("From Implicit Roadmaps to Asymptotic Optimality", "implicit roadmap에서 점근적 최적성으로")}</h2>
            <T
                en={<>
                    <p>
                        What breaks optimality in dRRT is the fanout. A fixed <InlineMath math="k"/>-nearest graph has a
                        fixed geometric quality no matter how many samples you pour into it — more vertices thicken the
                        graph but never straighten its paths toward the optimum. The fix is the PRM* radius: robot{" "}
                        <InlineMath math="i"/>'s roadmap <InlineMath math="G_i = (V_i, E_i)"/> now connects every pair of
                        vertices closer than <InlineMath math="r(n)"/>, with{" "}
                        <InlineMath math="n"/> rejection samples per robot and a bound taken from the paper's Theorem 1
                        verbatim.
                    </p>
                    <BlockMath math="r(n) \ge r^*(n) = \gamma \left(\frac{\log n}{n}\right)^{1/d}, \qquad \gamma = (1+\eta)^2 \left(\frac{1}{d}\right)^{1/d} \left(\frac{\mu(C_f)}{\zeta_d}\right)^{1/d}"/>
                    <p>
                        This repository runs the bound at <InlineMath math="d = 2"/> with{" "}
                        <InlineMath math="\zeta_2 = \pi"/> (unit-disc area), so the factor in force is{" "}
                        <InlineMath math="\gamma = (1+\eta)^2 \sqrt{\mu(C_f)/(2\pi)}"/>, where{" "}
                        <InlineMath math="\mu(C_f)"/> is this raster's own free-cell measure. An edge exists iff the two
                        vertices sit strictly inside that radius and the swept disc stays clear — evaluated once per pair
                        from the earlier-inserted endpoint, exactly as before. The second change is what happens on top of
                        the graph. Every joint vertex now carries a cost-to-come:{" "}
                        <InlineMath math="c(v)"/> is the sum of per-robot arc lengths along the tree chain,{" "}
                        <InlineMath math="\sum_i \|\sigma_i\|"/>, one of the paper's three cost functions (the second,
                        <InlineMath math="\max_i \|\sigma_i\|"/>, becomes this repository's makespan metric). Candidates
                        are re-parented when cheaper, expansions that cannot beat the incumbent are refused outright, and —
                        the insight the whole informed search rests on — the tensor structure itself supplies a heuristic:
                    </p>
                    <BlockMath math="H_i(v) = \text{shortest-path length on } G_i \text{ from } v \text{ to } t_i"/>
                    <p>
                        Each robot's own roadmap already knows how far every vertex is from its goal; precompute those
                        lengths once per robot (Dijkstra from the goal vertex — the paper suggests Johnson's algorithm for
                        all-pairs, but only single-source-to-goal values are ever read here) and the oracle becomes a
                        heuristic. A goal-biased sample switches every robot to argmin of <InlineMath math="H_i"/> over its
                        adjacency; an unbiased sample leaves each robot on a uniformly random neighbor. And because{" "}
                        <InlineMath math="\mathrm{Adj}(v_i, G_i)"/> now includes <InlineMath math="v_i"/> itself — the
                        paper's own words: “this ensures that it is
                        possible for a robot to stay static during an edge expansion” — waiting is a graph edge. dRRT's local connector and its priority DAG are gone; the
                        goal tuple is reached by tree search alone, and greedy child propagation dives at it as soon as any
                        generated node improves <InlineMath math="H"/> over its parent.
                    </p>
                    <Terms items={[
                        ["r(n)", <>the PRM* connection radius in force — <InlineMath math="(1+\eta)^2 \sqrt{\mu(C_f) \log n / (2\pi n)}"/> at <InlineMath math="d=2,\ \zeta_2=\pi"/>. An edge exists iff the pair sits strictly inside it and the swept disc stays clear</>],
                        ["c(v)", <>cost-to-come: the sum of per-robot arc lengths along the tree chain root → v. Edge weight is <InlineMath math="w(u,v) = \sum_i \|v_i - u_i\|"/>; re-parenting requires strict improvement and recomputes the moved subtree</>],
                        ["H_i(v)", <>the heuristic — shortest-path length on robot i's own roadmap from v to that robot's goal vertex, precomputed by Dijkstra before the tree exists. <InlineMath math="+\infty"/> when unreachable; an all-inf argmin resolves to the lowest index like every other tie here</>],
                        ["I_d", <>the informed oracle. Exact equality with its own goal switches a robot to argmin H over its ascending adjacency (ties → lowest index); otherwise it picks uniformly at random among neighbors — self-loop included, so waiting is one of the choices</>],
                        ["\\mathrm{Adj}(v_i, G_i)", <>the neighbor list <em>including v_i itself</em>. A tensor edge may move a subset of robots; waiting is native to the graph now, not sequenced by a connector</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        dRRT에서 최적성을 깨는 건 fanout이다. 고정 <InlineMath math="k"/>-nearest graph는 표본을 아무리
                        부어도 기하 품질이 고정된다 — vertex만 두꺼워지고 경로는 절대 최적으로 곧게 뻗지 않는다. 교정이 PRM*
                        radius다. robot <InlineMath math="i"/>의 roadmap <InlineMath math="G_i = (V_i, E_i)"/>은 이제{" "}
                        <InlineMath math="r(n)"/>보다 가까운 모든 vertex 쌍을 연결하고, <InlineMath math="n"/>은 robot당
                        rejection sample 수이며 bound는 논문 Theorem 1을 그대로 쓴다.
                    </p>
                    <BlockMath math="r(n) \ge r^*(n) = \gamma \left(\frac{\log n}{n}\right)^{1/d}, \qquad \gamma = (1+\eta)^2 \left(\frac{1}{d}\right)^{1/d} \left(\frac{\mu(C_f)}{\zeta_d}\right)^{1/d}"/>
                    <p>
                        이 저장소는 bound를 <InlineMath math="d = 2"/>, <InlineMath math="\zeta_2 = \pi"/> (unit disc 넓이)에서
                        돌리니 실제로 작동하는 인자는 <InlineMath math="\gamma = (1+\eta)^2 \sqrt{\mu(C_f)/(2\pi)}"/>이고{" "}
                        <InlineMath math="\mu(C_f)"/>는 이 raster 자신의 free-cell measure다. edge는 두 vertex가 그 반지름 안에
                        strict하게 있고 swept disc가 clear할 때만 존재하고 — pair마다 먼저 삽입된 끝점에서 한 번 평가된다, 전에
                        그랬던 것처럼. 두 번째 변화는 graph 위에서 벌어지는 일이다. 이제 모든 joint vertex에 cost-to-come이
                        붙는다: <InlineMath math="c(v)"/>는 tree chain을 따라 간 robot별 arc length의 합{" "}
                        <InlineMath math="\sum_i \|\sigma_i\|"/>이고, 논문 세 비용 함수 중 첫째다 (둘째{" "}
                        <InlineMath math="\max_i \|\sigma_i\|"/>는 이 저장소의 makespan metric이 된다). candidate은 더 싸지면
                        re-parent되고, incumbent를 이길 수 없는 expansion은 즉시 거부되며 — informed search 전체가 서 있는 통찰은
                        tensor 구조 자체가 heuristic을 공급한다는 것이다:
                    </p>
                    <BlockMath math="H_i(v) = \text{shortest-path length on } G_i \text{ from } v \text{ to } t_i"/>
                    <p>
                        각 robot의 roadmap은 이미 모든 vertex에서 자기 goal까지 거리를 안다. 그 길이를 tree가 존재하기 전에
                        robot마다 한 번씩 Dijkstra로 미리 계산하고(논문은 all-pairs에 Johnson을 제안하지만 여기서 읽히는 건
                        goal 정점 기준 single-source 값뿐이다) oracle이 heuristic이 된다. goal-biased 표본이면 모든 robot이{" "}
                        <InlineMath math="H_i"/> argmin으로 전환되고, unbiased 표본이면 각 robot이 균일 random neighbor를 고른다.
                        그리고 <InlineMath math="\mathrm{Adj}(v_i, G_i)"/>가 이제 자기 자신 <InlineMath math="v_i"/>를 포함하니 — 논문
                        본문 그대로 “this ensures that it is possible for a robot to stay static during an edge expansion” —
                        대기가 graph의 edge다. dRRT의
                        local connector와 priority DAG는 사라지고 goal tuple은 tree 탐색만으로 도달되며, 생성된 node가 parent보다{" "}
                        <InlineMath math="H"/>를 개선하는 순간 greedy child propagation이 goal로 다이브한다.
                    </p>
                    <Terms items={[
                        ["r(n)", <>작동 중인 PRM* connection radius — <InlineMath math="(1+\eta)^2 \sqrt{\mu(C_f) \log n / (2\pi n)}"/> at <InlineMath math="d=2,\ \zeta_2=\pi"/>. pair가 반지름 안에 strict하게 있고 swept disc가 clear할 때만 edge가 존재한다</>],
                        ["c(v)", <>cost-to-come: tree chain root → v를 따라 간 robot별 arc length의 합. edge weight은 <InlineMath math="w(u,v) = \sum_i \|v_i - u_i\|"/>이고 re-parenting은 strict 개선을 요구하며 옮겨진 subtree 비용을 재계산한다</>],
                        ["H_i(v)", <>heuristic — tree가 존재하기 전에 Dijkstra로 미리 계산한, robot i의 roadmap에서 v부터 그 robot의 goal vertex까지 최단경로 길이. 도달 불가면 <InlineMath math="+\infty"/>이고 all-inf argmin도 다른 모든 동률처럼 낮은 index로 해소된다</>],
                        ["I_d", <>informed oracle. 자기 goal과 정확히 일치하면 robot이 오름차순 인접 목록 위 H의 argmin으로 전환되고(동률은 낮은 index), 아니면 neighbor 중 균일 random 선택 — self-loop 포함이라 대기가 선택지 하나다</>],
                        ["\\mathrm{Adj}(v_i, G_i)", <>자기 자신 <InlineMath math="v_i"/>을 포함한 이웃 목록. tensor edge가 robot 부분집합만 움직일 수 있고 대기는 이제 graph에 native이며 connector가 순서화하지 않는다</>],
                    ]}/>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Probabilistically complete <em>and</em> asymptotically optimal — that is the whole
                            point.</strong> The paper proves this in two halves. Theorem 1: if the connection radius obeys{" "}
                            <InlineMath math="r(n) \ge r^*(n)"/>, the implicit tensor roadmap <InlineMath math="\hat{G}"/>{" "}
                            itself contains, with probability tending to one, paths whose cost approaches the robust optimum.
                            Theorem 2: given such a graph, the online tree search eventually finds the shortest path on it —
                            an absorbing-Markov-chain argument, since every step of the optimal chain has a positive
                            probability of being sampled and greedily propagated into the tree. Together:{" "}
                            <InlineMath math="\lim_{n,m \to \infty} \Pr[\mathrm{cost}(\Sigma(n,m)) \le (1+\epsilon) c^*] = 1"/>.
                            A budget exhausted here is honestly “no solution found within budget”, never “unsolvable”.
                            Instance verdicts <em>are</em> final though: a start or goal disc overlapping an obstacle cell,
                            and pairwise overlap at BOTH ends — two starts means no valid initial configuration exists at
                            all, two goals means no valid final one ever will. A start overlapping another robot's goal is
                            deliberately not a verdict: that robot can vacate before the other arrives.
                        </li>
                        <li>
                            <strong>Anytime by construction.</strong> The loop never stops on first success. Every iteration
                            grows and rewires the tree while branch-and-bound refuses any candidate whose cost exceeds the
                            incumbent, so what you read out at iteration 20 is strictly no worse than what you read out at
                            iteration 10 — the returned chain is the best found within the budget, converging toward the
                            optimum of <InlineMath math="\hat{G}"/> and (in the limit) of the free space. The paper's outer/inner
                            loop split degenerates here to one iteration per solution check; that is a pinning choice, not
                            an approximation.
                        </li>
                        <li>
                            <strong>The cost functions are the paper's own.</strong> A composite trajectory{" "}
                            <InlineMath math="\Sigma = (\sigma_1, \dots, \sigma_m)"/> is scored by sum of arc lengths{" "}
                            <InlineMath math="\sum_i \|\sigma_i\|"/> and by maximum arc length <InlineMath math="\max_i \|\sigma_i\|"/>;
                            both are reparameterization-invariant, which is exactly why the execution replay's tick grid stays
                            display-only. The optimum being converged to is the <em>robust</em> one:{" "}
                            <InlineMath math="c^*"/> is the infimum over costs that some collision-free path with a fixed
                            clearance margin <InlineMath math="\delta > 0"/> achieves — the margin is what makes the sampling
                            argument survive perturbations at all.
                        </li>
                        <li>
                            <strong>What this repository fixes.</strong> The paper leaves free: sample count{" "}
                            <InlineMath math="n"/> (here rejection samples over the map's world extent),{" "}
                            <InlineMath math="\eta"/>, goal-bias rate, iteration budget. Every tie in this implementation —
                            nearest-node ties, argmin-H ties, candidate ties, all-inf heuristics — breaks to the lower
                            insertion index so all three language engines replay identical trees from one seed. The paper's
                            additional focused resampling after a solution is found (informed sampling à la Informed RRT*)
                            is <em>not</em> implemented here: plain uniform samples plus branch-and-bound, honestly reported.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>Probabilistically complete <em>그리고</em> asymptotically optimal — 그게 요점 전체다.</strong>{" "}
                            논문은 두 절반으로 증명한다. Theorem 1: connection radius가 <InlineMath math="r(n) \ge r^*(n)"/>을
                            지키면 implicit tensor roadmap <InlineMath math="\hat{G}"/> 자체가, 확률 1로 수렴하며, costs가 robust
                            optimum에 접근하는 경로들을 담고 있다. Theorem 2: 그런 graph가 주어지면 online tree 탐색이 그 위의
                            최단 경로를 결국 찾는다 — absorbing Markov chain 논증이고, 최적 chain의 각 단계가 표본에 뽑혀 greedy로
                            트리에 전파될 양의 확률이 있으니까. 합치면{" "}
                            <InlineMath math="\lim_{n,m \to \infty} \Pr[\mathrm{cost}(\Sigma(n,m)) \le (1+\epsilon) c^*] = 1"/>.
                            여기서 예산 소진은 정직하게 “예산 안에서 해를 찾지 못했다”이지 절대 “unsolvable”이 아니다. instance
                            판정만은 최종적이다: obstacle과 겹치는 start/goal disc, 그리고 양쪽 끝에서 모두 pairwise 겹침 —
                            start끼리 겹치면 유효한 초기 configuration 자체가 없고, goal끼리 겹치면 두 disc을 동시에 점유하는 최종
                            configuration이 영원히 없다. 남의 goal과 자기 start의 겹침은 일부러 판정이 아니다. i가 먼저 떠나면 j가
                            도착할 수 있다.
                        </li>
                        <li>
                            <strong>구성상 anytime이다.</strong> 루프는 첫 성공에 절대 멈추지 않는다. 매 반복이 트리를 키우고
                            rewire하고, branch-and-bound는 costs가 incumbent을 넘는 candidate을 거부하니 — 20반복에서 읽는 것은
                            10반복에서 읽은 것보다 절대 나쁘지 않고, 반환되는 chain은 예산 안에서 찾은 최선이며{" "}
                            <InlineMath math="\hat{G}"/>의 optimum (그리고 극한에선 free space의 optimum)으로 수렴한다. 논문의
                            outer/inner loop 분리는 여기서 solution check당 반복 하나로 퇴화하고, 이건 pinning 선택이지 근사가 아니다.
                        </li>
                        <li>
                            <strong>비용 함수는 논문 자신의 것이다.</strong> composite trajectory{" "}
                            <InlineMath math="\Sigma = (\sigma_1, \dots, \sigma_m)"/>는 arc length의 합{" "}
                            <InlineMath math="\sum_i \|\sigma_i\|"/>와 최대 arc length <InlineMath math="\max_i \|\sigma_i\|"/>로
                            점수 매겨지고, 둘 다 reparameterization 불변이다 — 실행 재생의 tick 격자가 display 전용으로 남는 이유가
                            정확히 그것이다. 수렴하는 optimum은 <em>robust</em>한 쪽이고: <InlineMath math="c^*"/>는 고정 clearance
                            margin <InlineMath math="\delta > 0"/>를 지키는 어떤 collision-free 경로가 달성하는 비용들의 infimum이고,
                            그 margin이 sampling 논증을 perturbations에 대해 살아있게 만드는 것이다.
                        </li>
                        <li>
                            <strong>이 저장소가 고정한 것들.</strong> 논문이 자유로이 남긴 것: 표본 수 <InlineMath math="n"/> (여기에선
                            맵의 world extent 위 rejection sample), <InlineMath math="\eta"/>, goal bias, 반복 예산. 이 구현의 모든 동률 —
                            nearest-node, argmin-H, candidate, all-inf heuristic — 낮은 삽입 index로 깨져서 세 언어 엔진이 같은 seed에서
                            동일한 트리를 재생한다. 논문이 해답 발견 후 추가했다는 focused resampling (Informed RRT* 식 informed sampling)은
                            여기에 구현하지 <em>않았다</em>: plain uniform 표본에 branch-and-bound뿐이고 정직하게 보고한다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One loop, and the multi-agent content lives inside it as two data structures: per-robot roadmaps with
                    radius-bound connectivity built once before the tree exists, and a precomputed heuristic table per robot.
                    Everything else is RRT* on the implicit tensor graph — rewire, branch-and-bound, goal biasing — and every
                    choice the paper leaves open (how ties break, what “random neighbor” draws from, when greedy mode ends) is
                    fixed below and identical in all three language engines. The seed is part of a run's identity: roadmap
                    samples draw first (agent 0's points, then agent 1's), the tree's joint samples after them, from one shared
                    stream; a greedy iteration draws nothing at all.
                </p>}
                ko={<p>
                    루프 하나이고 multi-agent 내용은 그 안의 두 데이터 구조로 살아 있다. 트리가 존재하기 전에 한 번 지어지는
                    radius-bound 연결성의 robot별 roadmap과, robot마다 미리 계산해 둔 heuristic 표. 나머지는 implicit tensor graph
                    위의 RRT*다 — rewire, branch-and-bound, goal biasing — 그리고 논문이 열어둔 모든 선택(동률은 어떻게 깨지,
                    “random neighbor”가 뭐에서 뽑나, greedy mode는 언제 끝나나)은 아래에서 고정됐고 세 언어 엔진에 동일하다. seed는
                    실행 정체성의 일부다: roadmap 표본이 먼저(agent 0의 점들, 그다음 agent 1의) tree의 joint 표본이 그다음에 하나의
                    shared stream에서 뽑히고, greedy 반복은 아무것도 뽑지 않는다.
                </p>}
            />
            <Pseudocode code={`# G_i: V_i = [s_i, t_i] then n rejection samples over the extent; an edge iff    # 1
#        dist < r(n) (strict) and the swept disc stays clear from the             #
#        earlier-inserted endpoint. Adjacency ascending INCLUDING self-loop.       #
H_i ← shortest-path lengths on G_i to t_i by Dijkstra (+inf unreachable);         # 2
T ← root S = (0, ..., 0), c(root) = 0; incumbent ← ∞; V_last ← S
for iteration = 1 .. max_iterations:                                              # 3
    V_last = ∅ → exploration: one bias draw decides. Unbiased sample draws         # 4
        per-agent coords x-then-y, then each robot picks a uniform neighbor of
        its current vertex; V_near ← NEAREST tree node by concatenated Euclidean
        (strict <, earliest insertion wins ties). V_last ≠ ∅ → greedy: q_rand IS
        the goal tuple, V_near ← V_last itself, and NOTHING is drawn that iteration.
    I_d per robot ascending: exact goal equality switches to argmin H over Adj     # 5
        (strict < keeps lowest index; an all-inf neighbourhood resolves lowest too);
        otherwise a uniform random neighbor pick int(u · |Adj_i|).
    N ← Adj(V_new, G_hat) ∩ T scanned in ascending insertion order. A candidate    # 6
        edge is valid iff every pair's moving-pair distance ≥ r_i + r_j (a
        motionless robot contributes its point against the mover's segment).
    V_best ← argmin over VALID candidates of c(U) + w(U, V_new), strict < so       # 7
        earliest-inserted wins exact ties. No valid candidate → refuse.
        best_cost > incumbent (strict) → refuse — branch-and-bound.
    V_new ∉ T: append under V_best. V_new ∈ T: re-parent iff STRICTLY cheaper,     # 8
        moved subtree's costs recomputed top-down; then every U ∈ N is re-parented
        under V_new when that strictly improves c(U).
    promotion: inserted AND H(V_new) < H(parent) → next V_last ← V_new;            # 9
        anything else resets V_last ← ∅ — re-expanding an EXISTING state never
        promotes, and that is what ends greedy mode once the goal tuple is in T.
    Connect to Target every iteration: the goal tuple is a tree node or the        # 10
        solution does not exist yet; chain root → goal concatenated per agent,
        trimmed after each robot's last move.`}
    />
            <T
                en={<ol>
                    <li>The individual roadmaps are built once, before the tree exists. Vertices in insertion order:
                        start first, goal second, then <InlineMath math="n"/> rejection samples — uniform draws over the
                        map's world extent kept only where the disc is free. An edge connects a pair exactly when their
                        distance is strictly below <InlineMath math="r(n)"/> and the swept-disc check passes from the
                        earlier-inserted endpoint; adjacency lists keep ascending index order and include the self-loop,
                        so every vertex can also wait in place.</li>
                    <li>Before any sampling: one Dijkstra per robot over its own roadmap from that robot's goal vertex —
                        that is <InlineMath math="H_i"/>, evaluated at vertices by arc-length sums. The tree starts as the
                        root tuple (every robot at its own vertex 0) with cost 0, the incumbent solution is infinite, and{" "}
                        <InlineMath math="V_{last} \leftarrow S"/> — so the very first expansion is already greedy.</li>
                    <li>The loop runs a fixed budget of iterations; there is no early stop on success. The paper's inner
                        loop count degenerates to one: every iteration ends by checking whether the goal tuple is in the
                        tree with cost below the incumbent.</li>
                    <li>Exploration draws one bias draw first: below the goal-sample rate, the joint sample IS the goal
                        tuple and every robot takes the guided branch. Otherwise each agent's coordinates are drawn
                        (x-then-y, agent order) and each robot still on the random branch draws its uniform neighbor pick.{" "}
                        <InlineMath math="V_{near}"/> is then the nearest tree node by Euclidean distance over concatenated
                        coordinates — strict comparison, earliest insertion wins ties. In greedy mode there are no draws at
                        all: the sample is the goal tuple and <InlineMath math="V_{near}"/> is <InlineMath math="V_{last}"/> itself.</li>
                    <li>The oracle answers per robot in ascending order: a coordinate exactly equal to that robot's goal
                        switches to argmin <InlineMath math="H_i"/> over the ascending adjacency (strict comparison keeps the
                        lowest index on ties — and an all-infinite neighborhood, where this vertex's component never reaches
                        the goal, is one big tie that resolves to the lowest index too); otherwise a uniform pick among the
                        neighbors. The picks are vertex indices; together they form the candidate joint state{" "}
                        <InlineMath math="V_{new}"/>.</li>
                    <li>Candidates are the tree nodes adjacent to <InlineMath math="V_{new}"/> on the implicit graph, scanned
                        in ascending insertion order — and a candidate edge must be collision-free for every pair: the moving-pair
                        distance of the two simultaneous motions stays at or above <InlineMath math="r_i + r_j"/>, with a motionless
                        robot contributing its point against the mover's segment.</li>
                    <li><InlineMath math="V_{best}"/> is the argmin of <InlineMath math="c(U) + w(U, V_{new})"/> over valid
                        candidates (strict comparison — earliest inserted wins exact ties); no valid candidate refuses the whole
                        expansion. Branch-and-bound: a candidate cost strictly above the incumbent refuses it too. Insertion appends
                        under <InlineMath math="V_{best}"/>; re-parenting an existing state requires strict improvement and recomputes
                        the moved subtree's costs top-down — cycles are structurally impossible because an ancestor's cost already
                        dominates the direct edge, so the strict test can never fire upward.</li>
                    <li>The rewiring pass then visits every tree neighbor of <InlineMath math="V_{new}"/> and re-parents it under{" "}
                        <InlineMath math="V_{new}"/> when that strictly improves its cost-to-come — RRT*'s own move, on the implicit graph.</li>
                    <li>Child promotion is the paper's prose pinned exactly: only a node GENERATED this iteration becomes the next{" "}
                        <InlineMath math="V_{last}"/>, and only if it improved <InlineMath math="H"/> over its chosen parent. Re-expanding
                        an existing state never generates anything, so it always resets <InlineMath math="V_{last} \leftarrow \varnothing"/> —
                        which is precisely what ends greedy mode once the goal tuple itself sits in the tree, letting plain exploration
                        resume and the anytime phase continue.</li>
                    <li>Retrieval: the solution is the tree chain root → goal node; each agent's waypoint list is trimmed after that
                        robot's last move (trailing waits add zero arc length) and both metrics — sum and max of per-agent arc lengths —
                        are recomputed from the traced paths in fixed order.</li>
                </ol>}
                ko={<ol>
                    <li>개별 roadmap은 트리가 존재하기 전에 한 번 지어진다. vertex는 삽입 순서: start가 먼저, goal이 그다음,{" "}
                        <InlineMath math="n"/>개 rejection sample — 맵의 world extent에 균일 draw를 disc가 free인 곳에만 남긴다. edge는
                        두 vertex 거리가 <InlineMath math="r(n)"/> 아래 strict하고 swept disc 검사가 먼저 삽입된 끝점에서 통과할 때만
                        연결되고, 인접 목록은 오름차순 index 순서에 self-loop까지 포함하니 모든 vertex는 제자리에 대기하는 선택지도
                        가진다.</li>
                    <li>표본 채취 전에: robot마다 자기 roadmap 위 자기 goal 정점 기준 Dijkstra 하나 — 그게{" "}
                        <InlineMath math="H_i"/>이고 vertex에서 arc length 합으로 평가된다. 트리는 root tuple(전 robot이 자기 vertex 0)을
                        비용 0으로 시작하고, incumbent 해는 무한대이고, <InlineMath math="V_{last} \leftarrow S"/> — 그래서 첫 expansion부터
                        이미 greedy다.</li>
                    <li>루프는 고정 예산만큼 돌고 성공에 조기 종료하지 않는다. 논문의 inner loop 개수는 하나로 퇴화한다: 매 반복마다 goal
                        tuple이 트리에 incumbent보다 작은 비용으로 있는지 확인까지 끝낸다.</li>
                    <li>탐색은 bias draw 하나부터: goal-sample rate 아래면 joint 표본이 곧 goal tuple이고 모든 robot이 guided branch로 간다.
                        아니면 agent별 좌표를 뽑고(x-then-y, agent 순서) 여전히 random branch인 robot은 균일 neighbor pick을 뽑는다. 그다음{" "}
                        <InlineMath math="V_{near}"/>는 concatenated 좌표 위 Euclidean 거리상 가장 가까운 tree node — strict 비교라 동률은 최소
                        삽입 index가 이긴다. greedy mode에선 draw가 아예 없다: 표본이 goal tuple이고 <InlineMath math="V_{near}"/>가{" "}
                        <InlineMath math="V_{last}"/> 자기 자신이다.</li>
                    <li>Oracle은 robot별 오름차순으로 답한다: 좌표가 그 robot의 goal과 정확히 일치하면 오름차순 인접 목록 위{" "}
                        <InlineMath math="H_i"/> argmin으로 전환(strict 비교라 동률은 낮은 index 유지 — goal에 도달 못하는 component의 all-inf
                        neighborhood도 큰 동률이라 낮은 index로 해소), 아니면 neighbor 중 균일 선택. 고른 것은 vertex index이고 합쳐 candidate
                        joint state <InlineMath math="V_{new}"/>를 이룬다.</li>
                    <li>Candidate은 implicit graph에서 <InlineMath math="V_{new}"/>에 인접한 tree node들을 오름차순으로 훑고 — candidate edge는
                        모든 pair에 대해 collision-free여야 한다: 두 simultaneous motion의 moving-pair 거리가{" "}
                        <InlineMath math="r_i + r_j"/> 이상, 움직이지 않는 robot은 mover의 segment에 자기 점을 대어 기여한다.</li>
                    <li><InlineMath math="V_{best}"/>는 valid candidate 중 <InlineMath math="c(U) + w(U, V_{new})"/>의 argmin(strict 비교 —
                        먼저 삽입된 것이 정확 동률을 이긴다); valid가 없으면 expansion 전체가 거부된다. Branch-and-bound: candidate 비용이
                        incumbent보다 strict하게 크면 역시 거부. 삽입은 <InlineMath math="V_{best}"/> 아래에 append되고 기존 state의 re-parenting은
                        strict 개선을 요구하며 옮겨진 subtree 비용을 top-down으로 재계산한다 — ancestor의 비용이 직접 edge를 이미 dominate하니 strict
                        검사가 위로 절대 발동하지 않아 cycle은 구조적으로 불가능하다.</li>
                    <li>Rewiring pass는 <InlineMath math="V_{new}"/>의 tree 이웃 전부 돌아다니며 그걸로 re-parent하면 costs가 strict하게 개선될 때
                        걸고 다닌다 — implicit graph 위의 RRT* 자신의 동작이다.</li>
                    <li>Child promotion은 논문 문장을 그대로 pin한 것이다: 이번 반복에서 생성된 node만 다음 <InlineMath math="V_{last}"/>이 되고,{" "}
                        <InlineMath math="H"/>를 선택된 parent보다 개선해야 한다. 기존 state 재-expansion은 아무것도 생성하지 않으니 항상{" "}
                        <InlineMath math="V_{last} \leftarrow \varnothing"/>로 리셋 — 그리고 그게 정확히 goal tuple이 트리에 앉은 뒤 greedy mode가
                        끝나는 메커니즘이고, plain exploration이 이어지며 anytime 단계가 계속된다.</li>
                    <li>Retrieval: 해는 tree chain root → goal node이고, agent별 waypoint 목록은 그 robot의 마지막 이동 이후를 자르고(뒤따르는
                        대기는 arc length 0), 두 metric — agent별 arc length의 합과 최대 —이 고정 순서로 traced 경로에서 재계산된다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<>
                    <p>
                        The paper's guarantee is a pair of theorems: the sampled tensor roadmap itself converges to the robust
                        optimum (Theorem 1), and the online tree search eventually finds the shortest path on whatever graph it
                        got (Theorem 2). What survives as theorem for this implementation is narrower but provable: every emitted
                        plan is valid — no disc ever overlaps another or an obstacle, at any instant — and a budget exhaustion is
                        honestly reported as failure rather than impossibility. Everything else is honest sampling: the roadmap's
                        connectivity at <InlineMath math="n"/> samples decides what is reachable at all, and{" "}
                        <InlineMath math="\eta"/> only buys slack in an asymptotic bound. The theorems themselves are worth
                        opening — they are why this page exists where dRRT's ends.
                    </p>
                </>}
                ko={<>
                    <p>
                        논문의 보장은 정리 쌍이다: 샘플링된 tensor roadmap 자체가 robust optimum에 수렴하고(Theorem 1), online tree
                        탐색이 결국 그렇게 얻은 graph 위의 최단 경로를 찾는다(Theorem 2). 이 구현체에 대해 정리로 살아남은 것은 더
                        좁지만 증명 가능하다: 방출되는 모든 계획은 합법이고 — disc가 어떤 순간에도 다른 disc이나 obstacle과 겹치지
                        않는다 — 예산 소진은 불가능성의 실패로 정직하게 보고된다. 나머지는 전부 정직한 sampling이다. <InlineMath math="n"/>개
                        표본에서 roadmap의 연결성이 애초에 무엇을 도달 가능하게 하는지 결정하고, <InlineMath math="\eta"/>는 점근적 bound에
                        여유를 살 뿐이다. 정리 자체는 열어볼 가치가 있다 — dRRT 페이지가 끝나는 자리에 이 페이지가 있는 이유가 그것이다.
                    </p>
                </>}
            />
            <Proof title={t("Theorem 1 (the sampled roadmap converges to the robust optimum)", "정리 1 (샘플된 roadmap이 robust optimum에 수렴한다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If the connection radius obeys{" "}
                            <InlineMath math="r(n) \ge r^*(n) = \gamma (\log n / n)^{1/d}"/> (with{" "}
                            <InlineMath math="\gamma = (1+\eta)^2 (1/d)^{1/d} (\mu(C_f)/\zeta_d)^{1/d}"/> and any fixed{" "}
                            <InlineMath math="\eta > 0"/>), then for every fixed <InlineMath math="\epsilon > 0"/> the sampled
                            tensor roadmap <InlineMath math="\hat{G}"/> contains a composite path whose cost is at most{" "}
                            <InlineMath math="(1+\epsilon) c^*"/> asymptotically almost surely — where{" "}
                            <InlineMath math="c^*"/> is the robust optimum: the infimum over costs that some path with a fixed
                            clearance margin <InlineMath math="\delta > 0"/> achieves.
                        </p>
                        <p>
                            <strong>Proof sketch.</strong> Fix <InlineMath math="\epsilon"/> and take a robust path{" "}
                            <InlineMath math="\Sigma = (\sigma_1, \dots, \sigma_m)"/> with margin <InlineMath math="\delta"/> and
                            cost at most <InlineMath math="(1+\epsilon) c^*"/>; it suffices to show{" "}
                            <InlineMath math="\mathrm{cost}(\Sigma(n)) \le (1+o(1))\,\mathrm{cost}(\Sigma)"/> a.a.s. First the
                            robustness transfers: fix robot <InlineMath math="i"/>, time <InlineMath math="\tau"/>, and any{" "}
                            <InlineMath math="q_i"/> in the parameterized forbidden space{" "}
                            <InlineMath math="C_i^o(\tau) = C_i^o \cup \bigcup_{j \neq i} I_i^j(\sigma_j(\tau))"/> — configurations
                            of robot i that would collide with obstacle or with another robot parked along its path. The composite
                            configuration differing from <InlineMath math="\Sigma(\tau)"/> only in coordinate{" "}
                            <InlineMath math="i"/> still sits at distance at least <InlineMath math="\delta"/> from{" "}
                            <InlineMath math="\Sigma(\tau)"/>, and that distance is exactly one summand of the Euclidean norm, so{" "}
                            <InlineMath math="\|\sigma_i(\tau) - q_i\| \ge \delta"/>: a robust composite path is robust coordinate-wise.
                            Now apply the single-robot theory (Janson, Schmerling, Clark and Pavone's FMT* Theorem 4.1): each{" "}
                            <InlineMath math="G_i"/>, built at exactly this radius, a.a.s. contains a path{" "}
                            <InlineMath math="\sigma_i^{(n)}"/> from <InlineMath math="s_i"/> to <InlineMath math="t_i"/> with length{" "}
                            <InlineMath math="\|\sigma_i^{(n)}\| \le (1+o(1))\|\sigma_i\|"/> whose image stays within{" "}
                            <InlineMath math="r^*(n)"/> of <InlineMath math="\sigma_i"/>. For a cost that is a linear combination of arc
                            lengths the convergence trivially sums: <InlineMath math="\sum_i \|\sigma_i^{(n)}\| \le (1+o(1)) \sum_i \|\sigma_i\|"/>.
                            What remains is robot-robot collisions — individual paths hugging individual curves can still cross each other.
                            The repair is reparameterization: take the vertex chains <InlineMath math="V_i"/> traversed by each{" "}
                            <InlineMath math="\sigma_i^{(n)}"/>, give every vertex a timestamp — the time its curve passes closest,{" "}
                            <InlineMath math="\tau^j_i = \arg\min_\tau \|v^j_i - \sigma_i(\tau)\|"/> — merge all timestamps into one ordered
                            global list, and replay each robot's chain at those shared times. Anything a robot does near{" "}
                            <InlineMath math="\sigma_i^{(n)}"/>'s image stays within{" "}
                            <InlineMath math="r^*(n) \to 0"/> of its own curve, and coordinate-wise robustness with a fixed margin is exactly
                            what keeps the re-timed composite path out of the obstacle space. The composite path exists, costs what it should,
                            and is collision-free. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> connection radius가{" "}
                            <InlineMath math="r(n) \ge r^*(n) = \gamma (\log n / n)^{1/d}"/> (여기서{" "}
                            <InlineMath math="\gamma = (1+\eta)^2 (1/d)^{1/d} (\mu(C_f)/\zeta_d)^{1/d}"/>이고 임의의 고정{" "}
                            <InlineMath math="\eta > 0"/>)을 지키면, 모든 고정 <InlineMath math="\epsilon > 0"/>에 대해 샘플된 tensor
                            roadmap <InlineMath math="\hat{G}"/>는 costs가 <InlineMath math="(1+\epsilon) c^*"/> 이하인 composite 경로를
                            a.a.s. 담고 — 여기서 <InlineMath math="c^*"/>는 robust optimum, 즉 고정 clearance margin{" "}
                            <InlineMath math="\delta > 0"/>을 지키는 어떤 경로가 달성하는 비용들의 infimum이다.
                        </p>
                        <p>
                            <strong>증명 스케치.</strong> <InlineMath math="\epsilon"/>을 고정하고 margin <InlineMath math="\delta"/>에
                            costs <InlineMath math="(1+\epsilon) c^*"/> 이하인 robust 경로 <InlineMath math="\Sigma = (\sigma_1, \dots, \sigma_m)"/>를
                            잡는다. <InlineMath math="\mathrm{cost}(\Sigma(n)) \le (1+o(1))\,\mathrm{cost}(\Sigma)"/>이 a.a.s.임을 보이면 충분하다.
                            먼저 robustness가 전달된다: robot <InlineMath math="i"/>, 시각 <InlineMath math="\tau"/>, parameterized forbidden space{" "}
                            <InlineMath math="C_i^o(\tau) = C_i^o \cup \bigcup_{j \neq i} I_i^j(\sigma_j(\tau))"/>의 임의의 점 — obstacle이나 경로를
                            따라 주차된 다른 robot과 충돌하는 robot i의 configuration — 을 잡으면, <InlineMath math="\Sigma(\tau)"/>에서 좌표{" "}
                            <InlineMath math="i"/>만 다른 composite configuration은 여전히 거리가 <InlineMath math="\delta"/> 이상이고 그 거리는 Euclidean
                            norm의 항 하나 그 자체니 <InlineMath math="\|\sigma_i(\tau) - q_i\| \ge \delta"/>: robust composite 경로는 좌표별로도 robust다.
                            이제 single-robot 이론(Janson, Schmerling, Clark, Pavone의 FMT* Theorem 4.1)을 적용한다. 정확히 이 radius로 지어진 각{" "}
                            <InlineMath math="G_i"/>는 a.a.s. 길이 <InlineMath math="\|\sigma_i^{(n)}\| \le (1+o(1))\|\sigma_i\|"/>이고 image가{" "}
                            <InlineMath math="\sigma_i"/>에서 <InlineMath math="r^*(n)"/> 안에 있는 경로 <InlineMath math="\sigma_i^{(n)}"/>을{" "}
                            <InlineMath math="s_i"/>에서 <InlineMath math="t_i"/>까지 담고 있다. arc length들의 선형 결합인 비용엔 수렴이 그대로 합으로 더해진다:{" "}
                            <InlineMath math="\sum_i \|\sigma_i^{(n)}\| \le (1+o(1)) \sum_i \|\sigma_i\|"/>. 남은 건 robot 간 충돌이고 — 각 곡선에 밀착한 개별
                            경로들은 서로 교차할 수 있다. 수선은 reparameterization이다: 각 <InlineMath math="\sigma_i^{(n)}"/>이 지나는 vertex chain{" "}
                            <InlineMath math="V_i"/>에서 모든 vertex에 timestamp을 주고 — 자기 곡선이 가장 가까이 지나간 시각{" "}
                            <InlineMath math="\tau^j_i = \arg\min_\tau \|v^j_i - \sigma_i(\tau)\|"/> — 전체 timestamp를 하나의 순서 있는 global 목록으로 병합하고,
                            그 shared 시간에서 각 robot의 chain을 다시 재생한다. robot이 <InlineMath math="\sigma_i^{(n)}"/>의 image 근처에서 하는 어떤 일은
                            자기 곡선에서 <InlineMath math="r^*(n) \to 0"/> 안에 머물고, 고정 margin의 좌표별 robustness가 정확히 time-rewind된 composite 경로를
                            obstacle 공간 밖으로 유지시킨다. composite 경로가 존재하고, 비용을 맞춰야 할 만큼 내고, collision-free다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Theorem 2 (the online search finds the graph's shortest path)", "정리 2 (online 탐색이 graph의 최단 경로를 찾는다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If <InlineMath math="r(n) > r^*(n)"/> then for every fixed{" "}
                            <InlineMath math="\epsilon > 0"/>,{" "}
                            <InlineMath math="\lim_{n,m \to \infty} \Pr[\mathrm{cost}(\Sigma(n,m)) \le (1+\epsilon) c^*] = 1"/> —
                            where <InlineMath math="m"/> is the iteration budget: with the graph already AO by Theorem 1, it suffices to
                            show that for fixed <InlineMath math="n"/> and a fixed instantiation of <InlineMath math="\hat{G}"/>, the tree
                            search eventually contains the shortest path on it.
                        </p>
                        <p>
                            <strong>Proof sketch.</strong> Let <InlineMath math="v_1, \dots, v_t"/> be the vertices of{" "}
                            <InlineMath math="\hat{G}"/> along its optimal path, with <InlineMath math="v_t"/> absorbing. The search is an
                            absorbing Markov chain: each state's transition probability splits into staying put (the sample lands somewhere
                            else) and stepping to the next vertex on the chain. For plain exploration the step{" "}
                            <InlineMath math="v_i \to v_{i+1}"/> happens whenever a sample lands in the region where{" "}
                            <InlineMath math="v_i"/> is nearest — positive measure under general position, exactly as in dRRT's completeness
                            argument — and greedy child propagation only adds transitions: once a node improves{" "}
                            <InlineMath math="H"/> over its parent it becomes the next expansion point, driving the tree along the graph's own
                            shortest-path structure. Every non-absorbing state eventually reaches the absorbing one with probability 1 (the
                            chain-theorem this rests on is Grinstead &amp; Snell's), so the optimal path enters the tree; and once it is in,
                            rewiring under strict improvement plus branch-and-bound mean the incumbent only ever moves down toward that path's cost.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> <InlineMath math="r(n) > r^*(n)"/>이면 모든 고정 <InlineMath math="\epsilon > 0"/>에 대해{" "}
                            <InlineMath math="\lim_{n,m \to \infty} \Pr[\mathrm{cost}(\Sigma(n,m)) \le (1+\epsilon) c^*] = 1"/> — 여기서{" "}
                            <InlineMath math="m"/>은 반복 예산이고, graph는 정리 1로 이미 AO이므로, 고정된 <InlineMath math="n"/>과{" "}
                            <InlineMath math="\hat{G}"/>의 고정 인스턴스에 대해 tree 탐색이 결국 그 위의 최단 경로를 담음을 보이면 충분하다.
                        </p>
                        <p>
                            <strong>증명 스케치.</strong> <InlineMath math="\hat{G}"/>의 최적 경로를 따라 가는 vertex들을{" "}
                            <InlineMath math="v_1, \dots, v_t"/>라 하고 <InlineMath math="v_t"/>를 absorbing state로 둔다. 탐색은 absorbing Markov
                            chain이고 각 state의 전이 확률은 제자리에 머묾(표본이 다른 어딘가에 떨어짐)과 chain의 다음 vertex로 이동으로 갈라진다.
                            plain exploration에서 이동 <InlineMath math="v_i \to v_{i+1}"/>은 표본이 <InlineMath math="v_i"/>가 nearest인 영역에
                            떨어질 때 일어나고 — general position 아래 양의 measure이고, dRRT 완결성 논증과 정확히 같다 — greedy child propagation은
                            전이를 추가하기만 한다: node가 parent보다 <InlineMath math="H"/>를 개선하면 다음 expansion 지점이 되어 트리를 graph 자신의
                            최단경로 구조를 따라 밀어간다. absorbing이 아닌 모든 state는 확률 1로 absorbing에 도달하고(이 논증이 기대는 chain 정리가
                            Grinstead &amp; Snell), 그래서 최적 경로가 트리에 들어가고, 일단 들어가면 strict 개선 아래 rewire와 branch-and-bound 덕분에
                            incumbent은 그 경로의 비용을 향해 내려가기만 한다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what the Python/C++
                    code below emits. Both scenarios are open rooms with two disc robots of radius 0.2: <code>open01_cross_discs</code>{" "}
                    crosses perpendicular corridors and <code>open01_swap_discs</code> swaps head-on along one corridor; at the default
                    budget both solve near-optimally where dRRT's fixed fanout left slack (sum of arc lengths 16.55 and 14.53 against
                    dRRT's 19.0 and 20.0 on the same scenarios). Faint lines are the individual roadmaps — every sampled vertex and every
                    pair strictly inside <InlineMath math="r(n)"/> whose swept disc stayed clear, the implicit graph's visible half — bright
                    dots are the tree's revealed joint vertices, and execution replays at true disc radius with linear interpolation between
                    waypoints. Watch what waiting looks like here: a tensor edge may move one robot while the other sits still (the self-loop
                    is an ordinary edge now), so there is no priority DAG sequencing anything — coordination is just which edges the tree found.
                    And watch the anytime behavior: raise the iteration budget and the incumbent only improves, never degrades; at seed 42 with
                    a stingy roadmap (<InlineMath math="n = 40"/>, <InlineMath math="\eta = 0.5"/>) both scenarios honestly fail to connect —
                    an honest failure is also data. Drag an endpoint: endpoints snap to cells, and a start or goal whose disc overlaps a wall
                    is an instance verdict — planning fails at once with zero expansions. The seed is part of a run's identity: all three engines
                    replay identical sample streams from it.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과 바이트 단위로 같은 엔진입니다.
                    두 시나리오 모두 반지름 0.2 disc robot 둘이 열린 방을 지나는 경우고 — <code>open01_cross_discs</code>는 수직 통로 교차,{" "}
                    <code>open01_swap_discs</code>는 한 통로를 따라 정면 swap이고 — 기본 예산에서 둘 다 dRRT의 고정 fanout이 여유를 남긴 자리에
                    최적 근처로 풀립니다(같은 시나리오에서 arc length 합 16.55와 14.53 대 dRRT의 19.0과 20.0). 옅은 선은 개별 roadmap입니다 —
                    샘플링된 모든 vertex와 <InlineMath math="r(n)"/> 안에 strict하게 있고 swept disc이 clear했던 모든 pair, implicit graph의 보이는
                    절반이고, 밝은 점은 트리가 드러낸 joint vertex이며, 실행 재생은 진짜 disc 반지름과 웨이포인트 사이 선형 보간으로 굴러갑니다.
                    여기서 대기가 어떻게 생겼는지 보세요: tensor edge가 robot 하나만 움직이고 나머지는 앉아 있을 수 있고(self-loop이 이제 평범한
                    edge) 그래서 순서화하는 priority DAG는 없습니다 — 조율은 그냥 트리가 찾은 edge들이라는 겁니다. 그리고 anytime 행동을 보세요:
                    반복 예산을 올리면 incumbent은 개선되기만 하고 절대 나빠지지 않고, seed 42에 인색한 roadmap(<InlineMath math="n = 40"/>,{" "}
                    <InlineMath math="\eta = 0.5"/>)에서는 두 시나리오가 정직하게 연결에 실패합니다 — 정직한 실패도 데이터입니다. endpoint를
                    끌어보세요: endpoint는 셀에 스냅되고 벽과 겹치는 disc의 start/goal은 instance 판정이고 즉시 확장 0으로 실패합니다. seed는 실행
                    정체성의 일부입니다. 세 엔진 모두에서 동일한 표본 열이 재생됩니다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live drrt_star sandbox — byte-identical to the Python/C++ planner. Draw walls, drag endpoints; every edit re-plans and replays: faint lines are each robot's individual PRM* roadmap at the radius bound, bright dots are the joint tree's revealed vertices, discs move simultaneously on tensor edges (a motionless robot is a self-loop edge), and the incumbent path only improves as iterations pile up",
                "라이브 drrt_star sandbox — Python/C++ planner와 바이트 단위로 동일합니다. 벽을 그리고 endpoint를 끄면 모든 편집이 재계획과 재생으로 이어집니다: 옅은 선은 radius bound에서 각 robot의 개별 PRM* roadmap, 밝은 점은 joint 트리가 드러낸 vertex이고, disc들은 tensor edge로 동시에 움직이며(움직이지 않는 robot은 self-loop edge), 반복이 쌓일수록 incumbent 경로가 개선되기만 합니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that powers
                    the live sandbox above is a third mirror. Like dRRT's, every decision here is floating-point: Euclidean distances,
                    swept-disc validity via segment-to-segment distance, arc-length sums — so bit-identity comes from pinning the
                    expression order everywhere, every sum of squared differences and every <InlineMath math="\sqrt{\cdot}"/> written in
                    one fixed order that all three languages evaluate on identical IEEE-754 doubles (the C++ build compiles with{" "}
                    <code>-ffp-contract=off</code> so no fused multiply-add sneaks into a comparison). The per-robot Dijkstra pops its heap
                    by lexicographic <InlineMath math="(distance, index)"/> order — Python's heapq tuple order, mirrored exactly in C++ and JS —
                    because the tie order of equal-distance pops is part of a run's identity. The PRNG is the same MINSTD Lehmer generator as
                    the other samplers here (<InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/>, <InlineMath math="u = s/(2^{31}-1)"/>),
                    integer-exact in Python and C++ and exact in JS doubles; roadmap samples draw first, tree draws after, and greedy iterations
                    draw nothing. Trace floats cross languages by parsed value, never bytes, and <code>check-engine-parity</code> verifies on every
                    build that all three engines produce identical results on every scenario. The code below is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러입니다. dRRT처럼 여기선
                    모든 결정이 floating-point입니다: Euclidean 거리, segment-to-segment 거리로 묻는 swept disc 판정, arc length 합. 그래서
                    bit-identity는 산술 순서를 고정해서 나옵니다 — 제곱합 하나하나 <InlineMath math="\sqrt{\cdot}"/> 하나하나가 세 언어에서 동일한
                    IEEE-754 double에 동일하게 평가되는 순서로 쓰이고(C++ 빌드는 <code>-ffp-contract=off</code>라 fused multiply-add가 비교에 끼어들지
                    못한다), robot별 Dijkstra는 heap을 사전순 <InlineMath math="(distance, index)"/> 순으로 pop하고 — Python heapq의 tuple 순서를 C++과
                    JS에서 그대로 미러링 — 같은 거리 pop의 순서까지 실행 정체성의 일부이기 때문이다. PRNG는 다른 sampler들과 같은 MINSTD Lehmer({" "}
                    <InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/>, <InlineMath math="u = s/(2^{31}-1)"/>)이고 Python과 C++ int64에서
                    integer-exact하고 JS double에서도 정확하며, roadmap 표본이 먼저 뽑히고 tree 표본이 그다음에, greedy 반복은 아무것도 뽑지 않는다.
                    trace의 float은 바이트가 아니라 parsed 값으로 언어를 넘나들고 <code>check-engine-parity</code>는 빌드마다 세 엔진이 모든 시나리오에서
                    동일한 결과를 내는지 검증합니다. 아래 코드는 발췌가 아니라 실제 소스 그대로입니다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/sampling/drrt_star.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/sampling/drrt_star.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/sampling/drrt_star.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/sampling/drrt_star.hpp`,
                            },
                            {
                                name: "cpp/src/sampling/drrt_star.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/sampling/drrt_star.cpp`,
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
                dRRT* is Shome, Solovey, Dobson, Halperin and Bekris's Autonomous Robots 2020 journal paper — itself the extension of
                Dobson, Solovey, Shome, Halperin and Bekris's MRS 2017 conference version; both carry the same algorithm analyzed above,
                and it is this page's algorithm. The implicit-roadmap foundation is dRRT (Solovey, Salzman and Halperin, IJRR 2016) —
                the previous page of this genealogy. The optimality machinery underneath Theorem 1 is single-robot PRM* theory: Karaman
                and Frazzoli's asymptotic-optimality framework supplies the radius bound and the spanner property, and Lemma 1 is quoted
                from Janson, Schmerling, Clark and Pavone's FMT* paper (their Theorem 4.1) — a reminder that dRRT*'s multi-robot guarantee
                is exactly single-robot optimality plus reparameterization. The absorbing-Markov-chain step of Theorem 2 is Grinstead &amp;
                Snell. This repository implements the algorithm as published (single-source Dijkstra for H instead of all-pairs Johnson, and
                no focused resampling after the first solution — both noted on the page); the paper's own experiments ran on polygonal and
                polyhedral environments including manipulators — the geometry layer changes here, the algorithm does not.
            </p>} ko={<p>
                dRRT*는 Shome, Solovey, Dobson, Halperin, Bekris의 Autonomous Robots 2020 저널 논문이고 — 그 자체로 Dobson, Solovey, Shome,
                Halperin, Bekris의 MRS 2017 conference 버전의 확장이며, 둘 다 위에서 분석한 같은 알고리즘을 싣고 있고 그게 이 페이지의 알고리즘이다.
                implicit roadmap 기반은 dRRT(Solovey, Salzman, Halperin, IJRR 2016)이고 — 이 계보의 이전 페이지다. Theorem 1 아래를 받치는 최적성
                기계장치는 single-robot PRM* 이론이고: Karaman과 Frazzoli의 asymptotic-optimality 프레임워크가 radius bound와 spanner 성질을 공급하고,
                Lemma 1은 Janson, Schmerling, Clark, Pavone의 FMT* 논문(Theorem 4.1)에서 인용됐다 — dRRT*의 multi-robot 보장이 정확히 single-robot
                최적성 + reparameterization임을 일깨우는 표지다. Theorem 2의 absorbing Markov chain 단계는 Grinstead &amp; Snell이다. 이 저장소는
                알고리즘 출판된 그대로 구현하고(H는 all-pairs Johnson 대신 single-source Dijkstra, 첫 해 이후 focused resampling 없음 — 둘 다 페이지에
                명시), 논문 실험은 manipulator를 포함한 polygon/polyhedron 환경이었고 — geometry layer가 바뀌고 알고리즘은 안 바뀐다.
            </p>} />
            <ol>
                <li>
                    R. Shome, K. Solovey, A. Dobson, D. Halperin, K. Bekris,{" "}
                    <a href="https://doi.org/10.1007/s10514-019-09832-9" target="_blank" rel="noopener noreferrer">
                        <em>dRRT*: Scalable and informed asymptotically-optimal multi-robot motion planning</em>
                    </a>,
                    Autonomous Robots 44(3–4):443–467, 2020. Conference version: Dobson, Solovey, Shome, Halperin &amp; Bekris,{" "}
                    <em>Scalable asymptotically-optimal multi-robot motion planning</em>, IEEE International Symposium on Multi-Robot
                    and Multi-Agent Systems (MRS), 2017. arXiv:1706.09932.
                </li>
                <li>
                    K. Solovey, O. Salzman, D. Halperin,{" "}
                    <a href="https://doi.org/10.1177/0278364915615688" target="_blank" rel="noopener noreferrer">
                        <em>Finding a needle in an exponential haystack: discrete RRT for exploration of implicit
                        roadmaps in multi-robot motion planning</em>
                    </a>,
                    IJRR 35(5):501–513, 2016. arXiv:1305.2889 — the implicit tensor roadmap this page searches.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank" rel="noopener noreferrer">
                        <em>Sampling-based algorithms for optimal motion planning</em>
                    </a>,
                    IJRR 30(7):846–894, 2011. arXiv:1105.1186 — the RRT*/PRM* machinery Theorem 1's radius bound comes from.
                </li>
                <li>
                    L. Janson, E. Schmerling, A. Clark, M. Pavone,{" "}
                    <a href="https://doi.org/10.1177/0278364915577958" target="_blank" rel="noopener noreferrer">
                        <em>Fast marching tree: a fast marching sampling-based method for optimal motion planning in many dimensions</em>
                    </a>,
                    IJRR 34(7):883–921, 2015 — the single-robot spanner result (their Theorem 4.1) quoted as Lemma 1. arXiv:1405.5904.
                </li>
                <li>
                    C. M. Grinstead, J. L. Snell, <em>Introduction to Probability</em>, American Mathematical Society,
                    Providence, RI, 2012 — the absorbing-Markov-chain theorem Theorem 2's proof leans on.
                </li>
            </ol>
        </>
    )
}

export default DrrtStar
