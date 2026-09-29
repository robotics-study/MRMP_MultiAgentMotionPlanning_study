import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runDrrt} from "../../../libs/algorithms/drrt";
import {cellToWorld, GridMap} from "../../../libs/grid";
import {Cell, Point, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/sampling/drrt.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/sampling/drrt.hpp?raw";
import cppImpl from "../../../../../cpp/src/sampling/drrt.cpp?raw";

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 파라미터는 저장소의 configs/sampling/drrt.yaml
// 기본값과 동일: seed 42, robot당 40 rejection sample, fanout k = 6, round 예산 6.
// disc 반지름도 시나리오와 같은 0.2 (핸들 드래그의 셀 스냅은 그대로고, runLive가
// cellToWorld로 세계 좌표로 바꾼다 — 데모/parity와 동일한 좌표계).
const RADIUS = 0.2

const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runDrrt(
        map,
        tasks.map(([s, g]) => [cellToWorld(map, s), cellToWorld(map, g)] as [Point, Point]),
        tasks.map(() => RADIUS),
        {seed: 42, samples_per_robot: 40, roadmap_k: 6, max_rounds: 6},
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

const Drrt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    MA-RRT* fought the curse of dimension by searching the joint space harder, and sRRT folded
                    the dimensionality down with individual policies. Solovey, Salzman and Halperin take a third
                    road: they refuse to search the continuous configuration space directly at all. They lay one
                    PRM per robot and make their <em>tensor product</em> the composite roadmap — except this
                    exponentially large graph is never built explicitly. It is <em>implicit</em>. A vertex is a
                    collision-free placement tuple, an edge is a move where every robot moves simultaneously, and
                    an RRT does the pathfinding on top of that abstract graph: discrete-RRT, literally. Samples
                    are still points in joint space, NEAREST finds the closest tree vertex, and a direction oracle
                    answers “which neighbor of that vertex heads toward the sample.” In this repository's genealogy
                    this is the coupled pole of the sampling branch dropped into continuous space — MA-RRT* sampled
                    the joint state directly on a grid, sRRT folded dimensionality away, dRRT grinds the space into
                    a graph and feels that graph out with samples.
                </p>}
                ko={<p>
                    MA-RRT*는 차원의 저주를 joint 공간을 더 세게 탐색해서 싸웠고, sRRT는 개별 policy로 차원을 접었다.
                    Solovey, Salzman과 Halperin은 세 번째 길을 간다. continuous configuration space를 직접 탐색하는
                    것 자체를 거부하고, robot마다 PRM을 하나씩 깔고 그 <em>tensor product</em>를 composite roadmap으로
                    삼는다. 다만 그 지수적으로 큰 graph는 절대 명시적으로 구성되지 않는다. <em>implicit</em>이다.
                    vertex는 collision-free placement tuple이고 edge는 모든 robot이 동시에 움직이는 간선이며, 그 추상적
                    graph 위의 pathfinding을 RRT가 한다. 이름 그대로 discrete-RRT다. 표본은 여전히 joint 공간의 점이고
                    NEAREST는 tree에서 가장 가까운 vertex를 찾고 방향 oracle이 “그 vertex의 이웃 중 표본 방향으로 향하는
                    것”을 알려준다. 이 저장소의 계보에서 이건 sampling 갈래의 coupled 극점이 continuous 공간으로 내려온
                    자리다. MA-RRT*가 격자 위에서 joint 상태를 직접 샘플했고 sRRT가 차원을 접었다면, dRRT는 공간을
                    graph로 갈아엎고 그 graph를 표본으로 더듬어 드러낸다.
                </p>}
            />

            <h2>{t("From Joint Trees to Implicit Roadmaps", "joint 트리에서 implicit roadmap으로")}</h2>
            <T
                en={<>
                    <p>
                        Two objects make the construction. The first is the <em>individual roadmap</em>{" "}
                        <InlineMath math="G_i = (V_i, E_i)"/>: an ordinary PRM for robot <InlineMath math="i"/> alone
                        on the continuous free space of the same raster. Its vertices are collision-free placements —
                        a disc of radius <InlineMath math="r_i"/> that overlaps no obstacle cell — and{" "}
                        <InlineMath math="s_i, t_i \in V_i"/> hold by construction (start and goal are the first two
                        vertices). Edges are straight segments whose swept disc stays clear, drawn to each vertex's{" "}
                        <InlineMath math="k"/> nearest neighbors. The second object is how they combine: the composite
                        roadmap <InlineMath math="G = (V, E)"/> has as its vertices every pairwise collision-free
                        placement tuple <InlineMath math="C = (v_1, \dots, v_m)"/>, and — this is the tensor product,
                        not the Cartesian one — an edge connects <InlineMath math="C"/> to <InlineMath math="C'"/>{" "}
                        exactly when <em>every</em> robot moves along its own roadmap edge at the same time without
                        discs overlapping. Waiting does not exist on a tensor edge. It exists only later, inside the
                        local connector.
                    </p>
                    <BlockMath math="(C, C') \in E \iff \forall i: (v_i, v'_i) \in E_i \;\wedge\; \forall i \neq j:\ \mathrm{dist}\big((v_i \to v'_i), (v_j \to v'_j)\big) \ge r_i + r_j"/>
                    <Terms items={[
                        ["G_i = (V_i, E_i)", <>robot <InlineMath math="i"/>의 개별 PRM. vertex는 disc가 obstacle 셀과 strict overlap하지 않는 placement이고 start/goal이 항상 처음 두 vertex다</>],
                        ["k", <>개별 roadmap의 fanout. 각 vertex는 거리상 가장 가까운 이웃 최대 k개를 가진다(동률은 낮은 삽입 index)</>],
                        ["C = (v_1, \\dots, v_m)", <>composite roadmap의 vertex — pairwise collision-free placement tuple. root <InlineMath math="S=(s_1,\\dots,s_m)"/>과 goal <InlineMath math="T=(t_1,\\dots,t_m)"/>도 구성상 vertex다</>],
                        ["\\mathrm{dist}((v_i \\to v'_i), (v_j \\to v'_j))", <>robot <InlineMath math="i"/>가 <InlineMath math="v_i"/>에서 <InlineMath math="v'_i"/>로, 동시에 <InlineMath math="j"/>가 <InlineMath math="v_j"/>에서 <InlineMath math="v'_j"/>로 미끄러질 때 중심 간 최소 거리. 상대 운동이 또 다른 세그먼트라 시간 이산화 없이 정확하다</>],
                    ]}/>
                    <p>
                        A graph that exists only as a construction rule cannot be searched by walking it — you need
                        the neighbors, and they are exactly what is not enumerated. That is what the{" "}
                        <em>direction oracle</em> <InlineMath math="O_D"/> answers. Given a joint vertex{" "}
                        <InlineMath math="C = (c_1, \dots, c_m)"/> and a sample point <InlineMath math="q"/>, it
                        answers per robot: which neighbor of <InlineMath math="c_i"/> points most toward{" "}
                        <InlineMath math="q_i"/>. The paper defines the answer by angle; this repository computes it as
                        argmax cosine over the ascending-ordered adjacency list (strict comparison, so ties keep the
                        lower insertion index) — argmax cosine is argmin angle, and a degenerate ray has no direction,
                        which is handled honestly below.
                    </p>
                    <BlockMath math="O_D(C, q) = \big(O_D(c_1, q_1),\ \dots,\ O_D(c_m, q_m)\big), \qquad O_D(c_i, q_i) = \arg\max_{v \in N(c_i)} \cos\angle\big(\rho(c_i, q_i),\ \rho(c_i, v)\big)"/>
                    <Terms items={[
                        ["\\rho(v, v')", <>the ray starting at v that goes through v'. The oracle ranks neighbors by the angle between two rays</>],
                        ["N(c_i)", <>the neighbor list of vertex <InlineMath math="c_i"/> in the individual roadmap (ascending order). If any robot's neighbor list is empty, <InlineMath math="O_D = \varnothing"/> and the sample is ignored</>],
                    ]}/>
                    <p>
                        The candidate tuple joins the tree only if the tensor edge is valid: for every pair, the
                        moving-pair distance of the two simultaneous motions must not drop below{" "}
                        <InlineMath math="r_i + r_j"/>. And when a tree node finally sits near the goal tuple, no
                        oracle saves you — connecting to the goal means actually sequencing real motion. That is the{" "}
                        <em>local connector</em>, and it is van den Berg's prioritized planning: each robot gets a
                        hop-shortest path on its own roadmap from where it stands to its goal, priorities are derived
                        from which paths cross whose parked discs, and if the priority graph is acyclic the robots
                        move one at a time. The paper itself ran this as an experiment and reported it beat using
                        bounded-coupling M* as the connector.
                    </p>
                </>}
                ko={<>
                    <p>
                        구성을 만드는 객체는 둘이다. 첫번째는 <em>개별 roadmap</em>{" "}
                        <InlineMath math="G_i = (V_i, E_i)"/>. 같은 raster의 continuous free space 위에서 robot{" "}
                        <InlineMath math="i"/> 혼자를 위한 평범한 PRM이다. vertex는 collision-free placement, 즉 disc{" "}
                        <InlineMath math="r_i"/>가 obstacle 셀과 strict overlap하지 않는 위치이고{" "}
                        <InlineMath math="s_i, t_i \in V_i"/>는 구성상 항상 성립한다(start와 goal이 처음 두 vertex다).
                        edge는 swept disc가 clear한 직선 구간이고 거리상 가장 가까운 이웃 <InlineMath math="k"/>개에게
                        긋는다. 두번째 객체는 그것들의 결합 방식이다. composite roadmap <InlineMath math="G = (V, E)"/>의
                        vertex는 pairwise collision-free인 placement tuple 전부이고, 그리고 이건 Cartesian이 아니라 tensor
                        product다 — edge가 <InlineMath math="C"/>와 <InlineMath math="C'"/>를 연결하는 건 <em>모든</em>{" "}
                        robot이 동시에 자기 roadmap 간선을 따라 겹침 없이 움직일 때뿐이다. tensor edge에 대기는 존재하지
                        않는다. 대기는 나중에 local connector 안에서만 존재한다.
                    </p>
                    <BlockMath math="(C, C') \in E \iff \forall i: (v_i, v'_i) \in E_i \;\wedge\; \forall i \neq j:\ \mathrm{dist}\big((v_i \to v'_i), (v_j \to v'_j)\big) \ge r_i + r_j"/>
                    <Terms items={[
                        ["G_i = (V_i, E_i)", <>robot <InlineMath math="i"/>의 개별 PRM. vertex는 disc가 obstacle 셀과 strict overlap하지 않는 placement이고 start/goal이 항상 처음 두 vertex다</>],
                        ["k", <>개별 roadmap의 fanout. 각 vertex는 거리상 가장 가까운 이웃 최대 k개를 가진다(동률은 낮은 삽입 index)</>],
                        ["C = (v_1, \\dots, v_m)", <>composite roadmap의 vertex — pairwise collision-free placement tuple. root <InlineMath math="S=(s_1,\\dots,s_m)"/>과 goal <InlineMath math="T=(t_1,\\dots,t_m)"/>도 구성상 vertex다</>],
                        ["\\mathrm{dist}((v_i \\to v'_i), (v_j \\to v'_j))", <>robot <InlineMath math="i"/>가 <InlineMath math="v_i"/>에서 <InlineMath math="v'_i"/>로, 동시에 <InlineMath math="j"/>가 <InlineMath math="v_j"/>에서 <InlineMath math="v'_j"/>로 미끄러질 때 중심 간 최소 거리. 상대 운동이 또 다른 세그먼트라 시간 이산화 없이 정확하다</>],
                    ]}/>
                    <p>
                        구성 규칙으로만 존재하는 graph는 걸어서 탐색할 수 없다. 이웃이 필요한데 그 이웃이 바로
                        열거되지 않는 것이기 때문이다. 그래서 <em>direction oracle</em> <InlineMath math="O_D"/>가 있다.
                        joint vertex <InlineMath math="C = (c_1, \dots, c_m)"/>와 표본 <InlineMath math="q"/>를 받으면
                        robot마다 답한다: <InlineMath math="c_i"/>의 이웃 중 어느 것이 <InlineMath math="q_i"/> 방향으로
                        향하는가. 논문은 답을 각도로 정의하고 이 저장소는 argmax cosine으로 계산한다(오름차순 인접 목록에서
                        strict 비교, 동률은 낮은 삽입 index). argmax cosine은 argmin angle이고 퇴화한 ray는 방향이 없으니
                        그건 아래에서 정직하게 다룬다.
                    </p>
                    <BlockMath math="O_D(C, q) = \big(O_D(c_1, q_1),\ \dots,\ O_D(c_m, q_m)\big), \qquad O_D(c_i, q_i) = \arg\max_{v \in N(c_i)} \cos\angle\big(\rho(c_i, q_i),\ \rho(c_i, v)\big)"/>
                    <Terms items={[
                        ["\\rho(v, v')", <>v에서 시작해 <InlineMath math="v'"/>을 지나는 ray. oracle은 두 ray 사이 각도로 이웃을 랭킹한다</>],
                        ["N(c_i)", <>개별 roadmap에서 vertex <InlineMath math="c_i"/>의 이웃 목록(오름차순). 어느 robot의 이웃이라도 비어 있으면 <InlineMath math="O_D = \varnothing"/>이고 표본은 무시된다</>],
                    ]}/>
                    <p>
                        후보 tuple은 tensor edge가 유효할 때만 트리에 합류한다. 모든 pair에 대해 두 simultaneous motion의
                        moving-pair 거리가 <InlineMath math="r_i + r_j"/> 아래로 내려가지 않아야 한다. 그리고 tree node가
                        마침내 goal tuple 근처에 앉으면 oracle은 더 이상 구해지지 않는다 — goal에 연결한다는 건 실제 움직임을
                        실제로 순서화하는 것이고, 그건 van den Berg의 prioritized planning이다. 각 robot은 자기 roadmap에서
                        지금 위치에서 goal까지 hop-최단 경로를 받고, 우선순위는 어떤 경로가 누구의 parked disc를 지나는지에서
                        유도되고, priority graph가 acyclic하면 robot들은 한 번에 하나씩 움직인다. 논문이 실험으로 돌려보고
                        bounded-coupling M*보다 이게 낫다고 보고한 그 connector다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Probabilistically complete, and that is the whole promise.</strong> The paper's
                            theorem: if the composite roadmap <InlineMath math="G"/> is connected (its vertices in
                            general position), then with high probability every vertex of{" "}
                            <InlineMath math="G"/> gets revealed given enough rounds — so any feasible instance whose
                            sampled roadmaps connect will eventually be solved. There is no optimality claim anywhere
                            in the paper, and its own conclusion names asymptotic optimality as future work. That
                            future work has a name: dRRT* (Shome et al.), the next page of this genealogy. A budget
                            exhausted here is honestly “no solution found within budget”, never “unsolvable”. Two
                            verdicts <em>are</em> final though: a start or goal disc overlapping an obstacle, or two
                            start discs overlapping each other — no valid initial configuration exists at all.
                        </li>
                        <li>
                            <strong>Coupling is total on tree edges.</strong> Every tensor edge moves every robot at
                            once, so the tree phase has no waiting and no decoupling: a sample extends the tree only
                            when all <InlineMath math="m"/> robots can move simultaneously without overlapping.
                            Sequential motion exists only inside the connector, and the connector's priority DAG is
                            exactly sRRT-style prioritization — if it cycles, this candidate fails and the next of the{" "}
                            <InlineMath math="K"/> nearest candidates is tried. Coupling here is not adaptive like
                            sRRT's collision sets; it is structural, and what adapts is only which roadmap edges the
                            tree has revealed so far.
                        </li>
                        <li>
                            <strong>The paper's parameter-free schedule survives verbatim.</strong> Round{" "}
                            <InlineMath math="i"/> runs <InlineMath math="N = 2^i"/> expand iterations (the doubling
                            that removes the iteration-count knob) and then one connect attempt with{" "}
                            <InlineMath math="K = i"/> candidates. What the paper leaves free, this repository fixes:
                            each individual roadmap gets <InlineMath math="n"/> rejection samples over the map's world
                            extent (rejection sampling — a sample whose disc overlaps an obstacle is simply dropped),
                            fanout <InlineMath math="k"/>, and every tie in this implementation breaks to the lower
                            insertion index so all three language engines replay identical trees from one seed.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>Probabilistically complete이고 그게 약속의 전부다.</strong> 논문 정리: composite
                            roadmap <InlineMath math="G"/>가 connected하고(vertex들이 general position) 충분히 많은
                            round가 주어지면 w.h.p. <InlineMath math="G"/>의 모든 vertex가 드러난다. 그래서 sampled
                            roadmap이 연결되는 feasible instance는 결국 풀린다. 최적성 주장은 어디에도 없고 논문 결론이
                            asymptotic optimality를 future work로 직접 지목한다. 그 future work엔 이름이 있다. dRRT*
                            (Shome et al.), 이 계보의 다음 페이지다. 여기서 예산 소진은 정직하게 “예산 안에서 해를 찾지
                            못했다”이지 절대 “unsolvable”이 아니다. 단 두 판정만 최종적이다. obstacle과 겹치는 start/goal
                            disc, 그리고 서로 겹치는 start disc — 유효한 초기 configuration 자체가 없다.
                        </li>
                        <li>
                            <strong>Tree edge에서 결합은 전체적이다.</strong> 모든 tensor edge가 전 robot을 동시에
                            움직이므로 tree 단계엔 대기 없이 결합뿐이다. 표본이 트리를 확장하는 건 <InlineMath math="m"/>명
                            전원이 겹침 없이 동시에 움직일 수 있을 때뿐이다. 순차 움직임은 connector 안에서만 존재하고,
                            connector의 priority DAG는 정확히 sRRT식 prioritization이다. cycle이면 이 candidate은 실패하고{" "}
                            <InlineMath math="K"/>개 후보 중 다음 것이 시도된다. 결합이 여기서 sRRT의 collision set처럼
                            adaptive하지 않다. 구조적으로 전체고, adaptive한 건 tree가 지금까지 얼마나 드러냈는지뿐이다.
                        </li>
                        <li>
                            <strong>논문의 파라미터 없는 스케줄이 그대로 살아 있다.</strong> round{" "}
                            <InlineMath math="i"/>에 <InlineMath math="N = 2^i"/> expansion 반복(반복 횟수 knob을 없애는
                            doubling)과 <InlineMath math="K = i"/> 후보로 한 번의 connect 시도. 논문이 자유로이 남긴 것들은
                            이 저장소가 고정한다: 개별 roadmap마다 world extent 위에 rejection sample{" "}
                            <InlineMath math="n"/>개(rejection sampling — disc가 obstacle과 겹치는 표본은 그냥 버려진다),
                            fanout <InlineMath math="k"/>, 그리고 이 구현의 모든 동률은 낮은 삽입 index로 깨져서 세 언어
                            엔진이 같은 seed에서 동일한 트리를 재생한다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One loop, and the multi-agent content lives inside it as two data structures: per-robot roadmaps
                    built once before the tree exists, and a priority DAG inside every connect attempt. Everything else
                    is an RRT whose NEAREST queries a geometrically embedded graph through an oracle. The paper leaves
                    several things unspecified (how to build <InlineMath math="G_i"/>, how ties break, what “find a
                    path on <InlineMath math="G_i"/>” means concretely); every choice is fixed below and identical in
                    all three language engines. The seed is part of a run's identity: roadmap samples draw first
                    (agent 0's points, then agent 1's), the tree's joint samples after them, from one shared stream.
                </p>}
                ko={<p>
                    루프 하나이고 multi-agent 내용은 그 안의 두 데이터 구조로 살아 있다. 트리가 존재하기 전에 한 번
                    지어지는 robot별 roadmap과, 모든 connect 시도 안에 들어 있는 priority DAG. 나머지는 NEAREST가
                    geometrically embedded graph를 oracle으로 조회하는 RRT다. 논문은 여러 가지를 unspecified로 남긴다
                    (<InlineMath math="G_i"/>를 어떻게 구성하는지, 동률은 어떻게 깨는지, “<InlineMath math="G_i"/>에서
                    경로를 찾아라”가 구체적으로 뭔지). 아래에서 모두 고정했고 세 언어 엔진에 동일하다. seed는 실행
                    정체성의 일부다. roadmap 표본이 먼저(agent 0의 점들, 그다음 agent 1의) tree의 joint 표본이 그다음에
                    하나의 shared stream에서 뽑힌다.
                </p>}
            />
            <Pseudocode code={`# G_i: V_i = [s_i, t_i] then n rejection samples over the extent; edges to the k      # 1
#        nearest vertices (ties → lower insertion index); an edge exists iff the    #
#        swept disc stays clear — evaluated once per pair from the earlier vertex.  #
T ← root S = (s_1, ..., s_m)                                                        # 2
for round i = 1 .. max_rounds:
    repeat N = 2^i times:                                                           # 3
        q ← joint sample (per-robot x,y draws from the one shared stream)
        c ← NEAREST tree node by Euclidean distance over concatenated coords        # 4
            (ties → earliest insertion)
        c' ← O_D(c, q): per robot argmax cosine over ascending adjacency;           # 5
             ∅ if any robot has no neighbors. accept iff new and VALID: for every   #
             pair i<j the moving-pair distance of both motions ≥ r_i + r_j           #
    for each of K = i tree nodes nearest to T (distance, then insertion index):     # 6
        π_j ← hop-shortest BFS on G_j from the node's vertex to t_j                 #
        priority DAG: π_i hits j parked at v_j → i after j; π_i hits j parked        #
             at its goal t_j → i before j; a cycle fails this candidate              #
    execute in Kahn order (lowest index first among ready): mover walks π one       # 7
        roadmap edge per tick, everyone else waits in place
success → RETRIEVE PATH: tree chain root→q concatenated per agent, connector        # 8
          ticks spliced on, trimmed after each robot's last move`}
    />
            <T
                en={<ol>
                    <li>The individual roadmaps are built once, before the tree exists. Vertices in insertion order:
                        start first, goal second, then <InlineMath math="n"/> rejection samples — uniform draws over the
                        map's world extent kept only where the disc is free. Edges connect each vertex to its{" "}
                        <InlineMath math="k"/> nearest by Euclidean distance (ties: lower insertion index), evaluated
                        exactly once per pair from the earlier-inserted endpoint, so both languages evaluate
                        bit-identical expressions.</li>
                    <li>The tree's root is the start tuple — every robot at its own vertex 0. The goal tuple is not a
                        fixed index tuple; it is the joint point of all goals, and a node sitting on it ranks first
                        (distance 0) in every connect ranking.</li>
                    <li>EXPAND: round <InlineMath math="i"/> draws <InlineMath math="2^i"/> joint samples. Each draw is
                        per-robot coordinate pairs from one shared PRNG stream — the roadmap's samples consumed this
                        same stream first, so the seed fixes everything.</li>
                    <li>NEAREST scans the whole tree on Euclidean distance over concatenated coordinates (the paper's
                        nearest neighbor in <InlineMath math="\mathbb{R}^d"/>), ties to the earliest insertion. The
                        tie-break is load-bearing: it is what makes three languages replay one tree.</li>
                    <li>The oracle answers per robot by argmax cosine over the ascending adjacency list, strict{" "}
                        <InlineMath math=">"/> so ties keep the lower index; a degenerate ray (start equal to goal)
                        contributes cosine 0. A candidate joint vertex is accepted only if it is new and every pair's
                        moving-pair distance stays at or above <InlineMath math="r_i + r_j"/> — that single check also
                        keeps accepted vertices pairwise collision-free, since the relative-motion segment covers both
                        endpoints.</li>
                    <li>CONNECT TO TARGET ranks the <InlineMath math="K = i"/> nearest tree nodes to the goal tuple
                        (distance, then insertion index) and tries the local connector on each until one succeeds. A
                        robot's path <InlineMath math="\pi_j"/> is a hop-shortest BFS on its own roadmap with fixed
                        ascending neighbor order — first discovery is the parent.</li>
                    <li>The priority rule is the paper's: if moving along <InlineMath math="\pi_i"/> would collide with
                        robot <InlineMath math="j"/> parked at its connector-start vertex, i moves after j; if it would
                        collide with j parked at its goal, i moves before j. A cycle fails the candidate (the next of
                        the K is tried). Execution is Kahn topological order, lowest index first among ready robots: one
                        robot walks one roadmap edge per tick while everyone else waits.</li>
                    <li>Retrieval concatenates the tree chain root → q per agent (every tree tick moved all robots —
                        tensor edges), splices the connector's sequential ticks after it, and trims each path after that
                        robot's last move. The cost metric is the repository-wide one: a step costs 1 unless it is a
                        wait at one's own goal.</li>
                </ol>}
                ko={<ol>
                    <li>개별 roadmap은 트리가 존재하기 전에 한 번 지어진다. vertex는 삽입 순서: start가 먼저, goal이
                        그다음, 그다음 rejection sample <InlineMath math="n"/>개 — 맵의 world extent에 균일 draw를 disc가
                        free인 곳에만 남긴다. edge는 각 vertex에서 Euclidean 거리상 k-nearest(동률은 낮은 삽입 index)이고
                        pair마다 먼저 삽입된 끝점에서 정확히 한 번 평가된다. 그래서 두 언어가 bit-identical 식을 평가한다.</li>
                    <li>트리의 root는 start tuple — 전 robot이 자기 vertex 0에 있다. goal tuple은 고정 index tuple이 아니고
                        모든 goal의 joint 점이고, 거기에 앉은 node는 connect ranking마다 거리 0으로 먼저 온다.</li>
                    <li>EXPAND: round <InlineMath math="i"/>에 joint 표본 <InlineMath math="2^i"/>개. 각 draw는 하나의
                        shared stream에서 agent별 좌표 페어이고 — roadmap의 표본이 같은 stream을 먼저 소진했으므로 seed가
                        전부를 고정한다.</li>
                    <li>NEAREST는 트리를 concatenated 좌표 위 Euclidean 거리로 훑고(논문 표현대로{" "}
                        <InlineMath math="\mathbb{R}^d"/>의 nearest neighbor) 동률은 최소 삽입 index. tie-break는 기능을
                        한다. 세 언어가 하나의 트리를 재생하게 만드는 것이 이것이다.</li>
                    <li>Oracle은 robot마다 오름차순 인접 목록 위 argmax cosine으로 답하고 strict 비교라 동률은 낮은 index를
                        남긴다. 퇴화한 ray(start가 goal과 같을 때만 가능)는 cosine 0에 기여한다. 후보 joint vertex는 새 것이고
                        모든 pair의 moving-pair 거리가 <InlineMath math="r_i + r_j"/> 이상일 때만 받아들여진다 — 상대 운동
                        세그먼트가 두 끝점까지 커버하므로 이 검사 하나가 accepted vertex를 pairwise로 collision-free하게도
                        유지한다.</li>
                    <li>CONNECT TO TARGET은 goal tuple에 가까운 K = i개 tree node를 (거리, 삽입 index) 순으로 놓고 local
                        connector를 성공할 때까지 시도한다. robot의 경로 <InlineMath math="\pi_j"/>는 자기 roadmap 위 고정
                        오름차순 이웃 순서의 hop-최단 BFS이고 첫 발견이 parent다.</li>
                    <li>우선순위 규칙은 논문의 것이다. <InlineMath math="\pi_i"/>를 따라 움직이는 게 connector 시작 vertex에
                        parked된 j와 충돌하면 i는 j 뒤에, goal에 parked된 j와 충돌하면 i가 j 앞에 움직인다. cycle이면
                        candidate이 실패하고 K개 중 다음 것이 시도된다. 실행은 Kahn topological order — 준비된 중 낮은 index
                        먼저, 한 robot이 tick마다 roadmap edge 하나를 걷는 동안 나머지는 제자리에 대기한다.</li>
                    <li>Retrieval은 tree chain root → q를 agent별로 이어붙이고(tensor edge라 모든 tree tick이 전원을 움직였다)
                        connector의 순차 tick을 splice한 뒤, 각 robot의 마지막 이동 이후를 자른다. 비용 metric은 저장소 공통:
                        자기 goal에서의 대기가 아니면 스텝당 1이다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<>
                    <p>
                        The paper's guarantee is probabilistic completeness via a Voronoi argument, and it needs the
                        vertices of <InlineMath math="G"/> to be in general position — no two vertices embedded at the
                        same point, no three collinear. What survives as theorem for this implementation is narrower but
                        provable: every emitted plan is valid (no disc ever overlaps another or an obstacle), and a
                        budget exhaustion is honestly reported as failure rather than impossibility. Everything else is
                        honest sampling: the roadmap's connectivity at <InlineMath math="n"/> samples decides what is
                        reachable at all, and the doubling schedule only decides how long you wait for the Voronoi-cell
                        argument to bite.
                    </p>
                </>}
                ko={<>
                    <p>
                        논문의 보장은 Voronoi 논증으로 가는 probabilistic completeness이고 general position을 필요로 한다 —
                        두 vertex가 같은 점에 박히지 않고 세 점이 collinear하지 않다. 이 구현체에 대해 정리로 살아남은 것은 더
                        좁지만 증명 가능하다. 방출되는 모든 계획은 합법이고(disc가 다른 disc나 obstacle과 겹치는 순간이 없다) 예산
                        소진은 불가능성의 실패로 정직하게 보고된다. 나머지는 전부 정직한 sampling이다. <InlineMath math="n"/>개
                        표본에서 roadmap의 연결성이 애초에 무엇을 도달 가능하게 하는지 결정하고, doubling 스케줄은 Voronoi cell
                        논증이 작동할 때까지 얼마나 기다릴지만 결정한다.
                    </p>
                </>}
            />
            <Proof title={t("Theorem (every emitted plan is valid)", "정리 (방출되는 모든 계획은 합법이다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Every path tuple the planner emits keeps every disc off every
                            obstacle and off every other disc at every instant.
                        </p>
                        <p>
                            <strong>Proof.</strong> Obstacle-clearance is by construction: a vertex enters{" "}
                            <InlineMath math="V_i"/> only when its disc overlaps no cell, an edge enters{" "}
                            <InlineMath math="E_i"/> only when the swept segment stays clear, and every motion in the
                            output walks vertices and edges of these graphs. Robot-robot clearance splits by phase. On a
                            tree edge, both robots move simultaneously along their segments and acceptance required{" "}
                            <InlineMath math="\mathrm{dist} \ge r_i + r_j"/> over the whole relative-motion segment —
                            which includes both endpoints, so consecutive edges chain without overlap at the joints. In
                            the connector phase only one robot moves per tick: when i moves along{" "}
                            <InlineMath math="\pi_i"/>, any robot j parked at its start vertex with a disc touching{" "}
                            <InlineMath math="\pi_i"/> satisfies “i after j”, so j already moved; and no robot that has
                            already moved can park on a point of <InlineMath math="\pi_i"/>, because then the rule would
                            have demanded i move before it. Both directions are exactly what acyclicity of the priority
                            DAG guarantees — which is why a cycle fails the candidate instead. The final concatenation
                            splices at equal joint states, so nothing overlaps across the splice.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> planner가 내보내는 모든 경로 tuple은 disc을 모든 obstacle과 다른 disc으로부터
                            모든 순간에 떨어뜨린다.
                        </p>
                        <p>
                            <strong>증명.</strong> obstacle 회피는 구성상이다. vertex는 disc이 셀과 겹치지 않을 때만{" "}
                            <InlineMath math="V_i"/>에 들어가고 edge는 swept segment가 clear할 때만 <InlineMath math="E_i"/>에
                            들어가고, 출력의 모든 움직임은 이 graph들의 vertex와 edge를 걷는다. robot 간 회피는 단계로 나뉜다. tree
                            edge에서 둘은 자기 세그먼트를 동시에 미끄러지고 acceptance는 상대 운동 세그먼트 전체에{" "}
                            <InlineMath math="\mathrm{dist} \ge r_i + r_j"/>를 요구하는데 — 이건 두 끝점까지 커버하니 연속한 edge가
                            이음에서 겹치지 않고 이어진다. connector 단계에선 tick마다 robot 하나만 움직인다. i가{" "}
                            <InlineMath math="\pi_i"/>를 따라 움직일 때, 시작 vertex에 parked된 채 disc이 <InlineMath math="\pi_i"/>에
                            닿는 j는 “i는 j 뒤”를 만족하니 이미 움직인 뒤이고, 이미 움직인 robot이 <InlineMath math="\pi_i"/>의 점에
                            park할 수는 없는 게 그러면 규칙이 i가 그 전에 움직이길 요구했을 것이다. 두 방향이 정확히 priority DAG의
                            acyclicity가 보장하는 것이고 그래서 cycle이면 candidate이 실패한다. 마지막 연결은 같은 joint 상태에서
                            splice되니 이음에서도 겹침이 없다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Theorem (probabilistic completeness, paper's Theorem 1)", "정리 (probabilistic completeness — 논문의 Theorem 1)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If <InlineMath math="G"/> is connected and its vertices are in general
                            position, then with probability tending to one, every vertex of{" "}
                            <InlineMath math="G"/> is eventually revealed — so a feasible instance is solved given enough
                            rounds.
                        </p>
                        <p>
                            <strong>Proof sketch.</strong> Take an unrevealed vertex <InlineMath math="v^*"/> adjacent to a
                            revealed one v, and assume for simplicity that v is the only revealed neighbor of{" "}
                            <InlineMath math="v^*"/>. For the tree to grow across that edge, a joint sample must land in{" "}
                            <InlineMath math="\mathrm{Vor}(v) \cap \mathrm{Vor}'(v, v^*)"/>: the Voronoi cell of site{" "}
                            <InlineMath math="v"/> among revealed vertices (so NEAREST picks v), intersected with the
                            ray-Voronoi cell of <InlineMath math="\rho(v, v^*)"/> among the rays leaving v toward its
                            already-revealed neighbors (so the oracle points at <InlineMath math="v^*"/>). General position
                            gives both cells positive measure — a ball of radius <InlineMath math="r > 0"/> around v
                            inside the first, a cone of solid angle <InlineMath math="\alpha > 0"/> at v inside the second
                            — and their intersection is non-empty: otherwise v and <InlineMath math="v^*"/> share a point. A
                            fixed positive-measure region under a uniform density means each round's{" "}
                            <InlineMath math="2^i"/> draws hit it with probability bounded away from zero, so the probability
                            of never crossing that edge decays geometrically in the rounds. The connector plays no role in
                            this half of the argument — once both endpoints are revealed and their connecting edge is accepted,
                            retrieval has a chain to work with. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> <InlineMath math="G"/>가 connected하고 vertex들이 general position이면 w.h.p.{" "}
                            <InlineMath math="G"/>의 모든 vertex가 결국 드러나고, 그래서 feasible instance는 충분한 round에서 풀린다.
                        </p>
                        <p>
                            <strong>증명 스케치.</strong> 드러나지 않은 <InlineMath math="v^*"/>와 인접한 드러난 v를 잡고 단순화를 위해{" "}
                            <InlineMath math="v^*"/>의 드러난 이웃이 v 하나만이라 하자. 트리가 그 edge를 건너려면 joint 표본이{" "}
                            <InlineMath math="\mathrm{Vor}(v) \cap \mathrm{Vor}'(v, v^*)"/>에 떨어져야 한다 — NEAREST가 v를 고르게 하는
                            revealed vertex들 속 site v의 Voronoi cell과, v에서 이미 드러난 이웃들로 나가는 ray들의 ray-Voronoi diagram에서{" "}
                            <InlineMath math="\rho(v, v^*)"/>의 cell이 교차하는 곳이다. general position이 두 cell에 양의 measure을 주고 —
                            첫것 안에 반지름 <InlineMath math="r > 0"/>인 공, 둘째 안에 꼭짓점 각도 <InlineMath math="\alpha > 0"/>인
                            cone — 교집합은 non-empty다. 아니면 v와 <InlineMath math="v^*"/>가 같은 점을 공유하게 된다. 균일 density 아래
                            고정된 양의 measure 영역이니 각 round의 <InlineMath math="2^i"/>개 draw가 그걸 확률 0보다 떨어진 상한으로 때리고,
                            영원히 못 건널 확률은 round에 기하급수로 줄어든다. connector는 이 논증 절반에서 역할이 없다 — 양 끝점이 드러나고
                            연결 edge가 accepted되면 retrieval은 작업할 chain을 갖는다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what the
                    Python/C++ code below emits. Both scenarios are open rooms with two disc robots of radius 0.2:{" "}
                    <code>open01_cross_discs</code> crosses perpendicular corridors and <code>open01_swap_discs</code>{" "}
                    swaps head-on along one corridor. Faint lines are the individual roadmaps (every sampled vertex, every
                    swept-clear edge — the implicit graph's visible half), bright dots are the tree's revealed joint
                    vertices, and execution replays at true disc radius with linear interpolation between waypoints. Watch
                    what coupling looks like here: on the cross scenario a tensor edge whose two motions would overlap is
                    rejected outright (the crossing cannot be crossed together) and the priority DAG sequences who crosses
                    first; on the swap scenario the roadmap itself must contain the detour, because no pair of straight
                    edges passes two discs through each other. Drag an endpoint — endpoints snap to cells, and a start or
                    goal whose disc overlaps a wall is an honest instance verdict: planning fails at once with zero
                    expansions. The seed is part of a run's identity: all three engines replay identical sample streams
                    from it.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과 바이트 단위로
                    같은 엔진입니다. 두 시나리오 모두 반지름 0.2 disc robot 둘이 열린 방을 지나는 경우다.{" "}
                    <code>open01_cross_discs</code>는 수직 통로 교차, <code>open01_swap_discs</code>는 한 통로를 따라 정면
                    swap입니다. 옅은 선은 개별 roadmap입니다. 샘플링된 모든 vertex와 swept-clear했던 edge 전부, implicit graph의
                    보이는 절반이고, 밝은 점은 트리가 드러낸 joint vertex이며, 실행 재생은 진짜 disc 반지름과 웨이포인트 사이 선형
                    보간으로 굴러갑니다. 여기서 결합이 어떻게 생겼는지 보세요. 교차 시나리오에서 두 움직임이 겹칠 tensor edge는 그대로
                    거부되고(교차점을 같이 지나갈 수 없다) priority DAG가 누가 먼저 지나는지 순서화합니다. swap에서는 roadmap 자체가
                    우회를 담고 있어야 합니다. 직선 edge 쌍으로는 두 disc가 서로를 통과할 수 없으니까요. endpoint를 끌어보세요.
                    endpoint는 셀에 스냅되고 벽과 겹치는 disc의 start/goal은 정직한 instance 판정입니다. 즉시 확장 0으로 실패합니다.
                    seed는 실행 정체성의 일부입니다. 세 엔진 모두에서 동일한 표본 열이 재생됩니다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live drrt sandbox — byte-identical to the Python/C++ planner. Draw walls, drag endpoints; every edit re-plans and replays: faint lines are each robot's individual roadmap, bright dots are the joint tree's revealed vertices, and the discs move simultaneously on tensor edges until a pair would overlap — then the priority DAG sequences them",
                "라이브 drrt sandbox — Python/C++ planner와 바이트 단위로 동일합니다. 벽을 그리고 endpoint를 끄면 모든 편집이 재계획과 재생으로 이어집니다. 옅은 선은 각 robot의 개별 roadmap, 밝은 점은 joint 트리가 드러낸 vertex이고, disc들은 tensor edge로 동시에 움직이다가 pair가 겹칠 때가 되면 priority DAG가 순서화합니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that
                    powers the live sandbox above is a third mirror. Unlike the discrete branch, this planner's every
                    decision is floating-point: cosine similarities, Euclidean distances over concatenated coordinates,
                    segment-to-segment distances for swept-disc validity. Bit-identity therefore comes from pinning the
                    expression order everywhere — every sum of squared differences, every <InlineMath math="\sqrt{\cdot}"/>,
                    every clamp written in one fixed order that all three languages evaluate on identical IEEE-754 doubles.
                    The PRNG is the same MINSTD Lehmer generator as the other samplers here (
                    <InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/>, <InlineMath math="u = s/(2^{31}-1)"/>),
                    integer-exact in Python and C++ and exact in JS doubles. Trace floats cross languages by parsed value,
                    never bytes. The repo conventions layered on the paper (insertion-order tie-breaks, rejection sampling
                    over the extent, the unit-cost metric) are part of the contract, and <code>check-engine-parity</code>{" "}
                    verifies on every build that all three engines produce identical results on every scenario. The code
                    below is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러입니다.
                    discrete 갈래와 달리 이 planner의 모든 결정은 floating-point입니다. cosine similarity, concatenated 좌표 위
                    Euclidean 거리, swept disc 판정의 segment-to-segment 거리. 그래서 bit-identity는 산술 순서를 고정해서 나옵니다.
                    제곱합 하나하나, <InlineMath math="\sqrt{\cdot}"/> 하나하나, clamp 하나가 세 언어에서 동일한 IEEE-754 double에
                    동일하게 평가되는 순서로 씁니다. PRNG는 다른 sampler들과 같은 MINSTD Lehmer(<InlineMath math="s \leftarrow 16807\, s \bmod (2^{31}-1)"/>,{" "}
                    <InlineMath math="u = s/(2^{31}-1)"/>)이고 Python과 C++ int64에서 integer-exact하고 JS double에서도 정확합니다.
                    trace의 float은 바이트가 아니라 parsed 값으로 언어를 넘나듭니다. 논문 위에 얹은 저장소 관례들(삽입 순서 tie-break,
                    extent 위 rejection sampling, unit-cost metric)이 계약의 일부이고 <code>check-engine-parity</code>는 빌드마다 세
                    엔진이 모든 시나리오에서 동일한 결과를 내는지 검증합니다. 아래 코드는 발췌가 아니라 실제 소스 그대로입니다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/sampling/drrt.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/sampling/drrt.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/sampling/drrt.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/sampling/drrt.hpp`,
                            },
                            {
                                name: "cpp/src/sampling/drrt.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/sampling/drrt.cpp`,
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
                dRRT is Solovey, Salzman and Halperin's IJRR 2016 paper (the 2013 arXiv preprint carries the same title);
                its local connector is van den Berg, Snoeyink, Lin and Manocha's prioritized decoupling from RSS 2009,
                which the paper itself chose over bounded-coupling M* after experimenting with both. The
                asymptotically-optimal successor this page keeps pointing at — informed, on the same implicit roadmap — is
                dRRT* by Shome, Solovey, Dobson, Halperin and Bekris; it is the next page of this genealogy, not this
                one's algorithm. This repository implements the 2016 algorithm exactly as published, discretized onto the
                same raster its siblings use (the paper's own experiments ran on polygonal and polyhedral environments —
                the geometry layer changes, the algorithm does not).
            </p>} ko={<p>
                dRRT는 Solovey, Salzman, Halperin의 IJRR 2016 논문이고(2013 arXiv preprint가 같은 제목이다), local connector는
                van den Berg, Snoeyink, Lin, Manocha의 RSS 2009 prioritized decoupling이고 논문이 두 가지를 실험해 본 뒤 직접
                골랐다. 이 페이지가 계속 가리키는 asymptotically-optimal 후속은 Shome, Solovey, Dobson, Halperin, Bekris의
                dRRT*이고 같은 implicit roadmap 위의 informed 버전이며, 이건 이 계보의 다음 페이지고 이 페이지의 알고리즘이
                아니다. 이 저장소는 2016 알고리즘을 출판된 그대로 구현하고 형제들이 쓰는 같은 raster에 이산화한다(논문 실험은
                polygon/polyhedron 환경이었고 — geometry layer가 바뀌고 알고리즘은 안 바뀐다).
            </p>} />
            <ol>
                <li>
                    K. Solovey, O. Salzman, D. Halperin,{" "}
                    <a href="https://doi.org/10.1177/0278364915615688" target="_blank" rel="noopener noreferrer">
                        <em>Finding a needle in an exponential haystack: discrete RRT for exploration of implicit
                        roadmaps in multi-robot motion planning</em>
                    </a>,
                    IJRR 35(5):501–513, 2016. arXiv:1305.2889.
                </li>
                <li>
                    J. van den Berg, J. Snoeyink, M. Lin, D. Manocha,{" "}
                    <a href="https://doi.org/10.15607/RSS.2009.V.018" target="_blank" rel="noopener noreferrer">
                        <em>Centralized path planning for multiple robots: optimal decoupling into sequential plans</em>
                    </a>,
                    RSS V, 2009.
                </li>
                <li>
                    R. Shome, K. Solovey, A. Dobson, D. Halperin, K. Bekris,{" "}
                    <a href="https://doi.org/10.1007/s10514-019-09832-9" target="_blank" rel="noopener noreferrer">
                        <em>dRRT*: Scalable and informed asymptotically-optimal multi-robot motion planning</em>
                    </a>,
                    Autonomous Robots 44(3–4):443–467, 2020. (planned — 다음 wave)
                </li>
            </ol>
        </>
    )
}

export default Drrt
