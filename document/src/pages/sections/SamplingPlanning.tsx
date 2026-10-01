import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";

// sampling 갈래 소개 페이지 — 연속 configuration space의 표본 채취 갈래와 그 안에서
// 다시 나타나는 결합 축을 소개한다. MA-RRT*(논문 자체의 이산화 G-RRT*로)와 sRRT가
// 집필·구현됐고 dRRT 계열은 planned 상태다.
const SamplingPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The search branch shares one premise: the state space is discrete — cells, time
                    steps, enumeration. Take the premise away. Robots have geometry — disks of
                    radius <InlineMath math="r"/> moving in a continuous workspace — and their
                    positions are real numbers, not cells. The question does not change:{" "}
                    <InlineMath math="k"/> robots, each with its own start and goal, paths that must
                    coexist. What changes is what <em>searching</em> can even mean once there is
                    nothing left to enumerate.
                </p>}
                ko={<p>
                    search 갈래는 하나의 전제를 공유한다: 상태 공간이 이산이라는 것 — 칸과 시각,
                    그리고 열거. 그 전제를 걷어 내 보자. 로봇은 기하를 가진다 — 연속적인 작업
                    공간을 지름 <InlineMath math="r"/>인 원판으로 지나간다 — 그리고 위치는 칸이
                    아니라 실수다. 질문은 그대로다: <InlineMath math="k"/>대의 로봇, 각자의
                    시작·목표, 공존해야 하는 경로. 바뀌는 것은 열거할 것이 없어진 뒤{" "}
                    <em>탐색</em>이 무슨 뜻이냐는 것뿐이다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Multi-Robot Motion Planning over continuous configuration spaces: the workspace{" "}
                    <InlineMath math="\mathcal{W} \subseteq \mathbb{R}^2"/> and obstacles give each
                    robot its own configuration space <InlineMath math="\mathcal{C}_i"/> (obstacles
                    inflated by the robot's radius), a robot's state is a point{" "}
                    <InlineMath math="x_i \in \mathcal{C}_i"/>, and the joint state lives in the
                    product <InlineMath math="\mathcal{C}_1 \times \cdots \times \mathcal{C}_k"/>.
                    Find one collision-free path per robot between its start and goal. Discretize
                    space <em>and</em> time and this is exactly the MAPF problem of the search
                    branch — which is why the two branches are the same problem fought on two kinds
                    of space. Here the space stays continuous, so no cell boundary exists to check:
                    a collision is geometry overlapping, and there are uncountably many places it can
                    happen.
                </p>}
                ko={<p>
                    연속적인 configuration space 위의 Multi-Robot Motion Planning: 작업 공간{" "}
                    <InlineMath math="\mathcal{W} \subseteq \mathbb{R}^2"/>과 장애물이 각 로봇의
                    configuration space <InlineMath math="\mathcal{C}_i"/>(장애물을 로봇 반경만큼
                    부풀려 만든다)를 주고, 로봇의 상태는 점 <InlineMath math="x_i \in
                    \mathcal{C}_i"/>이며, joint 상태는 곱{" "}
                    <InlineMath math="\mathcal{C}_1 \times \cdots \times \mathcal{C}_k"/>에 산다.
                    로봇마다 시작에서 목표까지 충돌 없는 경로 하나씩을 찾는다. 공간과 시각을 모두
                    이산화하면 이것은 정확히 search 갈래의 MAPF 문제다 — 두 갈래가 같은 문제를 두
                    종류의 공간에서 싸우는 이유다. 여기서는 공간이 연속인 채로 남으니 충돌을 확인할
                    칸 경계라는 것이 없다: 충돌은 기하가 겹치는 일이고, 그럴 자리는 셀 수 없이 많다.
                </p>}
            />

            <h2>{t("Why Sample", "왜 sampling인가")}</h2>
            <T
                en={<p>
                    A grid over a continuous space is a lie with a resolution parameter: fine enough
                    and the state space explodes past any enumeration; coarse and the robot scrapes
                    walls. And every added robot adds two more dimensions to the joint state —
                    enumeration dies twice over, in resolution and in agents. The escape is the one
                    single-robot idea that never enumerates: throw samples at the space and connect
                    what can see each other (the RRT/PRM lineage, covered in the sister repository).
                    Everything in this branch lifts that idea to many robots — either by growing{" "}
                    <em>one</em> motion tree over the joint state directly, or by giving every robot
                    its own tree or roadmap and then solving the coordination the individual trees
                    cannot see. The guarantees weaken honestly: completeness becomes{" "}
                    <em>probabilistic</em>, optimality becomes <em>asymptotic</em>.
                </p>}
                ko={<p>
                    연속 공간을 격자로 쪼개는 것은 해상도라는 매개변수를 단 거짓말이다. 충분히
                    가늘게면 상태 공간이 폭발해서 열거는 죽고, 굵으면 로봇은 벽을 긁는다. 게다가
                    로봇이 한 대 늘 때마다 joint 상태의 차원은 2개씩 더 는다. 열거는 해상도와 agent
                    수 두 번 죽는다. 탈출구는 단일 로봇 시절부터 열거를 몰랐던 그 아이디어 하나:
                    공간에 표본을 던지고 서로 보이는 것들을 잇는다(자매 저장소의 RRT/PRM 계보). 이
                    갈래의 모든 것은 그 생각을 여러 로봇으로 끌어올린다 — joint 상태에서{" "}
                    <em>하나의</em> motion tree를 직접 키우거나, 로봇마다 트리와 roadmap을 주고 개별
                    트리가 못 보는 조율을 따로 푸는 길. 보장은 정직하게 약해진다: 완전성은{" "}
                    <em>확률적</em>이 되고 최적성은 <em>점근적</em>이 된다.
                </p>}
            />

            <h2>{t("The Same Coupling Axis, Again", "다시 나타난 결합의 축")}</h2>
            <T
                en={<>
                    <p>
                        What makes this branch multi-robot rather than k single-robot planners is
                        the same axis as before — and it reappears almost unchanged:
                    </p>
                    <ul>
                        <li>
                            <strong>Coupled.</strong> One motion tree over the joint state space: a
                            sample is a whole joint state, and extension steers one agent at a time
                            along its own manifold (MA-RRT*). Asymptotically optimal, hopeless in
                            dimension — the exact analogue of joint-space A*.
                        </li>
                        <li>
                            <strong>In between.</strong> Grow the coupling only where it is needed:
                            every robot keeps its own tree or roadmap, and subdimensional expansion
                            couples them only while paths conflict (sRRT) — CBS's trick wearing a
                            sampling coat. The implicit-roadmap family pushes this further: build one
                            roadmap per robot and search their tensor product{" "}
                            <em>implicitly</em>, steering bootstrap samples through it (dRRT), then
                            rewire toward asymptotic optimality (dRRT*).
                        </li>
                    </ul>
                    <p>
                        The decoupled pole needs no new invention here — planning one robot at a time
                        around finished paths is exactly what the search branch's prioritized
                        planning already means. What sampling adds to this branch is everything the
                        grid could not give: geometry, continuous space, and guarantees that survive
                        without discretization.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 갈래를 "단일 로봇 planner k대"가 아니라 multi-robot으로 만드는 것은
                        아까 그 축 그대로다 — 그리고 거의 그대로 다시 나타난다:
                    </p>
                    <ul>
                        <li>
                            <strong>Coupled.</strong> joint 상태 공간 위의 motion tree 하나. 표본이
                            통째로 joint 상태이고, extension은 각 agent를 자기 매니폴드를 따라 한
                            대씩 조향한다(MA-RRT*). 점근 최적이며 차원에서는 절망적 — joint-space
                            A*의 그대로의 대응물이다.
                        </li>
                        <li>
                            <strong>그 사이.</strong> 결합이 필요한 곳에서만 결합을 키운다: 로봇마다
                            자기 트리와 roadmap을 유지하고, 경로가 충돌하는 동안에만 subdimensional
                            expansion으로 그것들을 결합한다(sRRT) — sampling 옷을 입은 CBS의 속임수.
                            implicit roadmap 계열은 한 발 더 간다: 로봇마다 roadmap을 세우고 그 tensor
                            product를 <em>암묵적으로</em> 탐색해 bootstrap 표본을 그 위로 조향하고
                            (dRRT), 이후 rewiring으로 점근 최적성을 쫓는다(dRRT*).
                        </li>
                    </ul>
                    <p>
                        decoupled 극단은 이 갈래에서 새 발명이 필요 없다 — 한 대씩 완성된 경로
                        주위를 돌아가며 계획하는 것은 search 갈래의 우선순위 계획이 이미 뜻하는
                        그대로다. 격자가 줄 수 없었던 모든 것 — 기하, 연속 공간, 이산화 없이 살아남는
                        보장 — 그것이 sampling이 이 계보에 더하는 것이다.
                    </p>
                </>}
            />

            <h2>{t("All Four, in Reading Order", "네 회원 전부, 읽는 순서")}</h2>
            <T
                en={<p>
                    All four members have landed, and the reading order kept following the coupling
                    axis from one end to the other. <strong>MA-RRT*</strong> (Čáp et al., 2013) — one
                    RRT* on the joint state space, coupled from the start — arrived through its own
                    paper's discretization, waypoints on a grid, so this branch began on the very same
                    maps and cost metric as its search sibling; sRRT (Wagner, Kang & Choset, 2012) —
                    subdimensional expansion over motion trees — landed on the same grid: the paper
                    prefers optimal individual policies anyway, and a BFS tree from the goal is exactly
                    that. Then came the real geometry: <strong>dRRT</strong> (Solovey, Salzman &
                    Halperin 2016) — per-robot roadmaps over continuous free space, their tensor
                    product searched <em>implicitly</em> by steering bootstrap samples through it — and{" "}
                    <strong>dRRT*</strong> (Shome et al. 2020), asymptotic optimality on top. Each
                    landed as its own page with derivation, proof, and live sandbox. The coupling axis
                    has now run its full course: from one joint tree, through per-robot trees coupled
                    only while paths conflict, to roadmaps never built at all — and the genealogy's
                    last stop is the sibling section where the plan itself disappears.
                </p>}
                ko={<p>
                    네 회원이 모두 도착했고, 읽는 순서는 계속 결합 축을 끝까지 따라갔다.
                    <strong>MA-RRT*</strong>(Čáp 외, 2013) — joint 상태 공간 위의 RRT* 하나, 처음부터
                    coupled — 는 논문 자체의 이산화(격자 위 waypoint)를 통해 도착했고, 그래서 이 갈래는
                    자매 섹션과 정확히 같은 맵과 같은 비용 척도에서 시작했다. sRRT(Wagner, Kang &
                    Choset, 2012) — motion tree 위의 subdimensional expansion — 도 같은 격자 위에
                    도착했다. 논문 어차피 optimal individual policies를 선호하고 goal에서 BFS 트리가
                    정확히 그거다. 그리고 진짜 기하가 왔다: <strong>dRRT</strong>(Solovey, Salzman &
                    Halperin 2016) — 연속 자유 공간 위의 로봇별 roadmap, 그 tensor product를 bootstrap
                    표본을 조향해 <em>암묵적으로</em> 탐색하고 — 그리고 그 위 점근 최적성을 얹은{" "}
                    <strong>dRRT*</strong>(Shome 외 2020). 각각 유도·증명·라이브 sandbox와 함께 각자의
                    페이지로 도착했다. 결합 축은 이제 그 전 과정을 달렸다: joint 트리 하나에서, 경로가
                    충돌하는 동안에만 결합되는 로봇별 트리를 지나, 아예 구성조차 되지 않는 roadmap까지 —
                    그리고 계보의 마지막 정류장은 계획 자체 사라지는 자매 섹션이다.
                </p>}
            />
        </>
    )
}

export default SamplingPlanning
