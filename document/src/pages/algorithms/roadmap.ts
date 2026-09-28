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
                "states exactly, and read the branch along its coupling axis: decoupled " +
                "prioritized planning, coupled joint-space search, and the hybrid in between (CBS).",
            ko: "하나의 그래프를 공유하는 점 로봇들 — Multi-Agent Path Finding 그 자체. 상태를 " +
                "정확하게 열거하고, 결합 축을 따라 읽는다: decoupled 우선순위 계획, coupled " +
                "joint-space 탐색, 그리고 그 사이 어딘가의 hybrid(CBS).",
        },
    },
    {
        key: "sampling",
        title: {en: "Sampling-Based Planning", ko: "Sampling-Based Planning"},
        desc: {
            en: "Robots with geometry in continuous configuration space: sample instead of " +
                "enumerate — joint-state motion trees, subdimensional expansion, implicit " +
                "roadmaps. The same coupling axis reappears; this branch is still unwritten.",
            ko: "기하를 가진 로봇을 연속적인 configuration space에서 계획한다 — 열거 대신 " +
                "샘플링. joint 상태의 motion tree, subdimensional expansion, implicit roadmap. 같은 " +
                "결합 축이 다시 나타나며, 이 갈래는 아직 집필 중이다.",
        },
    },
];
