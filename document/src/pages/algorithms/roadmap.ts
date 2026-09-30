import {AlgoSection, Localized} from "../../../types/global";

// 홈 카드가 쓰는 알고리즘 한 줄 소개. 페이지를 집필하면 여기에 함께 적는다
// (집필 전 planned 항목은 blurb 없이 제목만).
export interface AlgoBlurb {
    slug: string;
    blurb: Localized<string>;
}

export const ALGO_BLURBS: AlgoBlurb[] = [
    {
        slug: "prioritized_astar",
        blurb: {
            en: "Order the agents, let each plan a space-time A* around the finished paths as " +
                "moving obstacles: fast and scalable, individually optimal, honestly incomplete.",
            ko: "agent에 순서를 매기고 각자 시공간 A*로 완성된 경로를 움직이는 장애물로 " +
                "돌아가기: 빠르고 확장성 있고 개별 최적이며, 불완전함을 정직하게 고백한다.",
        },
    },
    {
        slug: "push_and_swap",
        blurb: {
            en: "Keep the order, drop the frozen reservations: each agent drives its static shortest path and " +
                "whatever blocks it is chain-pushed into the nearest reachable hole — or swapped around through a " +
                "free 2×2 block. Parameter-free, honest about the tree-shaped maps where no swap site exists.",
            ko: "순서는 유지하고 얼린 예약은 버린다: 각 agent는 정적 최단경로를 스스로 운전하고, 막히는 것은 " +
                "chain-push로 가장 가까운 도달 가능 빈 셀에 밀어 넣고, 그것도 안 되면 빈 2×2 block을 통해 swap으로 " +
                "맞바꾼다. 파라미터 무의존이고, swap 자리가 없는 트리 모양 맵에서 정직하게 실패한다.",
        },
    },
    {
        slug: "push_and_rotate",
        blurb: {
            en: "Read the map's structure before planning: decompose the free graph into biconnected " +
                "subgraphs joined by planks, derive a priority order between pieces, and extend the swap " +
                "into a rotate on degree-3 junctions — a decision procedure that also settles width-1 " +
                "corridors, reporting unsolvable when none exists.",
            ko: "계획 전에 맵의 구조를 읽는다: 자유 그래프를 plank로 이어진 biconnected subgraph로 분해하고, " +
                "조각들 사이에 우선순위 순서를 유도하고, swap을 degree-3 junction의 rotate로 확장한다 — " +
                "폭 1 통로까지 해결하는 판정 절차이고, 해가 없으면 불가능하다고 보고한다.",
        },
    },
    {
        slug: "joint_astar",
        blurb: {
            en: "Treat all k positions as one state and run a single A* over that product space: " +
                "complete, sum-of-costs optimal by construction — paid for in |V|^k. The baseline " +
                "every other approach exists to escape.",
            ko: "k개의 위치 전부를 하나의 상태로 다루고 그 product space 위에서 A* 하나를 돌린다: " +
                "완전하고 구성상 sum-of-costs 최적이며, 대가는 |V|^k다. 다른 모든 접근이 벗어나려 " +
                "존재하는 baseline.",
        },
    },
    {
        slug: "cbs",
        blurb: {
            en: "Plan each agent alone against explicit constraints and branch a constraint tree on " +
                "every conflict: optimal like coupled search without ever touching |V|^k, complete up " +
                "to an honest budget — paid in interference, not in agents.",
            ko: "모든 agent를 명시적 constraint에 대해 혼자 계획하고 conflict마다 constraint tree를 " +
                "분기한다: |V|^k를 한 번도 건드리지 않고 coupled처럼 최적, 정직한 예산까지 완전. " +
                "대가는 agent 수가 아니라 간섭만큼.",
        },
    },
    {
        slug: "ma_rrt_star",
        blurb: {
            en: "One RRT* grown on the joint state space of motion graphs: samples are whole joint " +
                "states, steering is simultaneous greedy descent, rewiring chases optimality — " +
                "probabilistically complete, asymptotically optimal, paid in dimension.",
            ko: "motion graph의 joint 상태 위에 RRT* 하나를 기른다: 표본은 통째로 joint 상태이고, " +
                "조향은 동시 greedy 하강이며, rewiring으로 최적성을 쫓는다 — 확률적으로 완전하고 " +
                "점근적으로 최적이며, 대가는 차원으로 지불한다.",
        },
    },
    {
        slug: "subdimensional_rrt",
        blurb: {
            en: "Refuse the joint space until it is needed: every robot obeys its own BFS-tree policy " +
                "until a collision proves the policy insufficient — then the involved robots join a " +
                "collision set and get steered by samples. Dimension grows exactly where robots meet, " +
                "and nothing about optimality is promised.",
            ko: "필요할 때까지 joint 공간을 거부한다: 모든 robot은 충돌이 policy의 부족함을 입증할까지 " +
                "자기 BFS-tree policy를 따른다 — 그러면 관련된 robot들이 collision set에 합류해 표본 " +
                "조향으로 넘어간다. 차원은 robot들이 만나는 곳에서 정확히 자라고 최적성에 대해서는 아무것도 " +
                "약속하지 않는다.",
        },
    },
    {
        slug: "drrt",
        blurb: {
            en: "Stop searching the continuous space and grind it into a graph instead: one PRM per robot, " +
                "their tensor product as the composite roadmap — never built, always queried through a " +
                "direction oracle. An RRT on that implicit graph, with prioritized planning as the local " +
                "connector. Probabilistically complete, no optimality claim.",
            ko: "continuous 공간을 탐색하는 대신 graph로 갈아엎는다: robot마다 PRM 하나, 그 tensor product가 " +
                "composite roadmap이고 절대 구성되지 않으면서 direction oracle으로만 조회된다. 그 implicit " +
                "graph 위의 RRT이고 local connector는 prioritized planning. 확률적으로 완전하고 최적성 주장은 없다.",
        },
    },
    {
        slug: "drrt_star",
        blurb: {
            en: "The same implicit tensor roadmap, searched so it converges: k-nearest becomes the PRM* radius " +
                "bound, the oracle becomes goal-biased argmin over precomputed shortest-path heuristics, and " +
                "RRT*-style rewiring with branch-and-bound chases the optimum — probabilistically complete AND " +
                "asymptotically optimal, anytime.",
            ko: "같은 implicit tensor roadmap을 수렴하도록 탐색한다: k-nearest가 PRM* radius bound가 되고, " +
                "oracle이 미리 계산한 최단경로 heuristic 위 goal-biased argmin이 되며, branch-and-bound를 갖춘 " +
                "RRT*식 rewiring이 optimum을 쫓는다 — 확률적으로 완전하고 점근적으로 최적이며 anytime.",
        },
    },
];

// 대분류 — 홈의 큰 섹션이자 사이드바 disclosure 단위. 갈래는 survey(Bui 2023)가
// planner 타입으로 나누는 그대로다: search-based(그래프 위 열거 탐색)와
// sampling-based(연속 configuration space의 표본 채취). 각 섹션 안의 알고리즘 배치는
// 항상 계보순 — 결합 축을 따라 decoupled → coupled → hybrid 순서로 읽는다. 소스 코드
// 트리(python/mrmp/<section>/)와 configs(<section>/<slug>.yaml)도 이 구분을 따른다.
export const SECTIONS: Array<{
    key: AlgoSection;
    title: Localized<string>;
    desc: Localized<string>;
}> = [
    {
        key: "search",
        title: {en: "Search-Based Planning", ko: "Search-Based Planning"},
        desc: {
            en: "Point robots on one shared graph — Multi-Agent Path Finding proper. Enumerate " +
                "states exactly, and read the branch along its coupling axis: prioritized planning " +
                "and its decentralized per-agent completion via push/swap primitives, coupled " +
                "joint-space search, and the hybrid in between (CBS).",
            ko: "하나의 그래프를 공유하는 점 로봇들 — Multi-Agent Path Finding 그 자체. 상태를 " +
                "정확하게 열거하고, 결합 축을 따라 읽는다: 우선순위 계획과 push/swap primitive로 " +
                "완성되는 decentralized 계열, coupled joint-space 탐색, 그리고 그 사이 어딘가의 " +
                "hybrid(CBS).",
        },
    },
    {
        key: "sampling",
        title: {en: "Sampling-Based Planning", ko: "Sampling-Based Planning"},
        desc: {
            en: "Robots with geometry in continuous configuration space: sample instead of " +
                "enumerate — joint-state motion trees, subdimensional expansion, implicit " +
                "roadmaps. The same coupling axis reappears: MA-RRT* (in the paper's own grid " +
                "discretization) is written and implemented; sRRT and the dRRT family follow.",
            ko: "기하를 가진 로봇을 연속적인 configuration space에서 계획한다 — 열거 대신 " +
                "샘플링. joint 상태의 motion tree, subdimensional expansion, implicit roadmap. 같은 " +
                "결합 축이 다시 나타난다: MA-RRT*(논문 자체의 격자 이산화로)가 집필·구현됐고, " +
                "sRRT와 dRRT 계열이 뒤를 잇는다.",
        },
    },
];
