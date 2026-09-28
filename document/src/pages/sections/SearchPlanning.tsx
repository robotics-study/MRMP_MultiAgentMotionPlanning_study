import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import SpaceTimeConflict from "../../components/panels/intro/SpaceTimeConflict";
import MultiCorridorPriority from "../../components/panels/intro/MultiCorridorPriority";
import MultiConstraintTree from "../../components/panels/intro/MultiConstraintTree";

// search 갈래 소개 페이지 — 계보의 첫 갈래(그래프 위 열거 탐색)와 그 안의 결합 축을 소개한다.
const SearchPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    One robot with a path is a solved problem. A warehouse with three hundred robots
                    sharing the same aisles is not: paths that are individually perfect can be
                    collectively impossible. Multi-agent planning is about finding paths
                    that <em>coexist</em>.
                </p>}
                ko={<p>
                    경로를 가진 로봇 한 대는 풀린 문제다. 같은 통로를 공유하는 로봇 삼백 대의
                    창고는 그렇지 않다: 각자로서는 완벽한 경로들이 모이면 불가능해질 수 있다.
                    Multi-agent planning은 <em>공존하는</em> 경로들을 찾는 일이다.
                </p>}
            />

            <T
                en={<>
                    <p>
                        The field sorts itself by what the planner does: survey taxonomies split
                        multi-robot planners into search-based, sampling-based, and learning-guided
                        families (Bui 2023), and this site follows that axis. <em>This</em> section is
                        the search branch — discrete states enumerated exactly. Its sibling{" "}
                        <em>sampling-based</em> section probes continuous configuration space with
                        random samples instead.
                    </p>
                    <p>
                        Inside each branch runs the same second axis, the one that makes planning
                        for many robots different in kind: how the agents are coupled. This branch
                        is where all three coupling strategies were invented, and it is written here
                        in exactly that order.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 분야는 planner가 무엇을 하는지로 스스로를 정리한다: survey 분류들은
                        multi-robot planner를 search 기반, sampling 기반, 학습 기반으로 나눈다
                        (Bui 2023). 이 사이트도 그 축을 따른다. <em>이</em> 섹션은 search 갈래다 —
                        이산 상태를 정확하게 열거한다. 자매 섹션인 <em>sampling-based</em>는 대신
                        연속적인 configuration space를 무작위 표본으로 찔러 본다.
                    </p>
                    <p>
                        각 갈래 안에는 같은 두 번째 축이 흐른다. 여러 로봇의 계획을 종류가
                        다르게 만드는 바로 그 축, agent들이 어떻게 결합되는가다. 세 결합 전략이 모두
                        이 갈래에서 발명됐고, 이 섹션은 정확히 그 순서대로 읽는다.
                    </p>
                </>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Multi-Agent Path Finding (MAPF): given one shared map
                    and <InlineMath math="k"/> agents, each with its own start and goal, find a
                    collision-free path <em>per agent</em> such that no two agents occupy the same
                    cell at the same time (vertex conflict) or swap cells across an edge (edge
                    conflict). Solution quality is measured by <strong>sum of costs</strong> (total
                    of all path lengths) or <strong>makespan</strong> (when the last agent
                    arrives).
                </p>}
                ko={<p>
                    Multi-Agent Path Finding (MAPF): 공유 지도 하나와 각자 시작·목표를 가진{" "}
                    <InlineMath math="k"/> 개의 agent가 주어졌을 때, 어떤 두 agent도 같은 시각에
                    같은 셀을 차지하지 않고(vertex conflict) 간선을 사이에 두고 자리를 맞바꾸지
                    않도록(edge conflict), <em>agent 별</em> 충돌 없는 경로를 찾는다. 해의 품질은{" "}
                    <strong>sum of costs</strong>(전체 경로 길이 합) 또는{" "}
                    <strong>makespan</strong>(마지막 agent의 도착 시각)으로 잰다.
                </p>}
            />

            <SpaceTimeConflict/>

            <h2>{t("Why It Is Hard", "왜 어려운가")}</h2>
            <T
                en={<p>
                    The honest search space is the <em>joint</em> space: the state is the tuple of
                    all agent positions, so its size is roughly <InlineMath math="|V|^k"/> — a
                    20×20 grid that A* clears in milliseconds becomes, for ten agents, a state space
                    of ~<InlineMath math="10^{26}"/>. Optimal MAPF is NP-hard. Every practical
                    algorithm is a strategy for <em>not</em> searching that joint space naively,
                    trading between solution quality, completeness, and how much coupling between
                    agents it is willing to consider.
                </p>}
                ko={<p>
                    탐색 공간을 곧이곧대로 잡으면 <em>joint</em> 공간이 된다. 상태가 모든 agent 위치의 튜플이라
                    크기가 대략 <InlineMath math="|V|^k"/>다. A*가 밀리초에 끝내는 20×20
                    격자도 agent 열 대면 ~<InlineMath math="10^{26}"/> 상태 공간이 된다. 최적 MAPF는
                    NP-hard다. 실용적인 알고리즘은 전부 이 joint 공간을 순진하게 탐색하지{" "}
                    <em>않기</em> 위한 전략이며, 해 품질·완전성·agent 간 결합을 얼마나 고려할지를
                    맞바꾼다.
                </p>}
            />

            <T
                en={<p>
                    The joint space is not just large, it is full of traps. Two robots whose
                    shortest paths point straight at each other in a one-wide corridor cannot both
                    win: someone must give way. The cheapest fix is an <strong>order</strong> —
                    decide who has priority, and let the loser treat the winner as a moving obstacle
                    it must route around.
                </p>}
                ko={<p>
                    joint 공간은 넓기만 한 게 아니라 함정투성이다. 폭 1 통로에서 최단 경로가 서로를
                    정면으로 겨누는 두 로봇은 둘 다 이길 수 없다. 누군가는 비켜야 한다. 가장 값싼
                    해법은 <strong>순서</strong>를 정하는 것이다. 우선순위를 정하고, 진 쪽이 이긴
                    쪽을 돌아가야 할 움직이는 장애물로 취급하게 한다.
                </p>}
            />
            <MultiCorridorPriority/>

            <h2>{t("Decoupled, Coupled, and In Between", "Decoupled, Coupled, 그리고 그 사이")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Decoupled — Prioritized A*.</strong> Order the agents; each plans
                            with single-agent A* in space-time, treating earlier agents' paths as
                            moving obstacles. Fast and scalable, but incomplete: a bad priority order
                            can paint later agents into corners. The planned <strong>Push and Swap</strong>
                            keeps per-agent planning yet drops the frozen reservations: push clears a
                            blocker off the high-priority agent's shortest path, swap exchanges two
                            agents outright — complete, it claims, once at least two cells per component
                            stay free. Whether that claim holds is exactly what its successor audits.
                        </li>
                        <li>
                            <strong>Coupled — Joint-space A*.</strong> Search the joint space
                            directly. Complete and optimal, and hopeless beyond a handful of agents —
                            it exists here mostly as the baseline that explains why everything else
                            exists.
                        </li>
                        <li>
                            <strong>In between — CBS.</strong> Conflict-Based Search plans each agent
                            independently (low level), detects conflicts, and branches on
                            constraints "agent <InlineMath math="i"/> may not be at{" "}
                            <InlineMath math="v"/> at time <InlineMath math="t"/>" (high level).
                            Optimal, yet it only ever couples agents that actually conflict — the
                            standard modern approach.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>Decoupled (Prioritized A*).</strong> agent에 순서를 매기고, 각자
                            앞선 agent들의 경로를 움직이는 장애물로 취급하며 시공간 A*로
                            계획한다. 빠르고 확장성 있지만 불완전하다: 우선순위를 잘못 매기면 뒤의
                            agent를 구석에 가둘 수 있다. 이 극단의 다음 장은 planned인 Push and Swap이다.
                            예약 동결을 포기하지 않으면서 lower-priority agent를 push로 밀어내고 swap으로
                            자리를 맞바꾼다. component당 빈 셀이 2개 이상이면 항상 성공한다고 주장하고,
                            그 주장이 참인지 검증하는 것이 후속 Push and Rotate다.
                        </li>
                        <li>
                            <strong>Coupled (Joint-space A*).</strong> joint 공간을 직접 탐색한다.
                            완전하고 최적이지만 agent 몇 대만 넘어도 감당이 안 된다. 여기서는 다른
                            접근들이 왜 필요한지 보여 주는 baseline 역할에 그친다.
                        </li>
                        <li>
                            <strong>그 사이 (CBS).</strong> Conflict-Based Search는 agent를 각자
                            계획하고(low level), conflict를 찾아 "agent <InlineMath math="i"/>는
                            시각 <InlineMath math="t"/>에 <InlineMath math="v"/>에 있을 수 없다"는
                            제약으로 분기한다(high level). 최적이면서도 실제로 충돌한 agent만
                            결합한다. 현대의 표준 접근이다.
                        </li>
                    </ul>
                </>}
            />

            <T
                en={<p>
                    CBS is worth seeing as a picture. Its high-level search is a binary tree of
                    constraints: the root plans every agent alone, and whenever two agents collide
                    at some cell and time, it splits into two children — one forbidding the first
                    agent from that cell-time, the other forbidding the second. Each child replans
                    only the constrained agent. The tree grows solely along branches that still
                    have conflicts, which is why CBS couples agents only when they actually
                    interfere.
                </p>}
                ko={<p>
                    CBS는 그림으로 보면 이해가 빠르다. high-level 탐색은 제약의 이진 트리다. root는
                    모든 agent를 각자 계획하고, 두 agent가 어떤 칸·시각에서 충돌하면 두 자식으로
                    갈라진다. 한쪽은 첫 번째 agent를, 다른쪽은 두 번째 agent를 그 칸·시각에서
                    금지한다. 각 자식은 제약이 걸린 agent만 다시 계획한다. 트리는 아직 충돌이 남은
                    가지를 따라서만 자라며, 그래서 CBS는 실제로 간섭하는 agent만 결합한다.
                </p>}
            />
            <MultiConstraintTree/>

            <h2>{t("All Three Poles, in Reading Order", "세 극단, 읽는 순서")}</h2>
            <T
                en={<p>
                    All three poles have a written representative now. Read{" "}
                    <strong>Prioritized A*</strong> first (the decoupled pole), then{" "}
                    <strong>Joint-Space A*</strong> — the coupled baseline that everything else is
                    measured against — and finally <strong>CBS</strong>, the hybrid in between. The
                    priority pole's completion, <strong>Push and Swap</strong> and its fix{" "}
                    <strong>Push and Rotate</strong>, sits on the roadmap. Each gets the same
                    derivation, proof, and live interactive sandbox treatment as the single-robot
                    pages. When this branch is read through, the genealogy continues in the sibling
                    section: the same coupling axis, re-fought with motion trees over continuous
                    configuration space.
                </p>}
                ko={<p>
                    세 극단이 모두 대표 알고리즘을 갖췄다. 먼저 <strong>Prioritized A*</strong>
                    (decoupled 극단)를 읽고, 이어서 모든 것이 여기에 대해 저울질되는 coupled baseline{" "}
                    <strong>Joint-Space A*</strong>를 읽고, 마지막으로 그 사이 어딘가의 hybrid{" "}
                    <strong>CBS</strong>를 읽어라. priority 극단의 완성인 <strong>Push and Swap</strong>와
                    그 보완 <strong>Push and Rotate</strong>는 로드맵에 있다. 각각 단일 로봇 페이지와 같은
                    유도·증명과 라이브 interactive sandbox로 다룬다. 이 갈래를 다 읽으면 계보는 자매
                    섹션으로 이어진다.
                    같은 결합 축을 연속적인 configuration space 위의 motion tree로 다시 싸우는 갈래.
                </p>}
            />
        </>
    )
}

export default SearchPlanning
