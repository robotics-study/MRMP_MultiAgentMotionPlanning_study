import {lazy} from "react";
import {IAlgoData} from "../../../types/global";

// 알고리즘 메타데이터는 여기서만 관리한다. 콘텐츠 모듈은 lazy import로 분리해
// 홈/목록 화면에서는 불러오지 않는다 (초기 번들 축소).
// sections의 en/ko 문자열은 각 언어로 렌더된 본문 h2 헤딩과 정확히 일치해야
// 사이드바/TOC/검색 앵커(slug)가 맞는다.
// 배열 순서가 사이드바·pager의 진행 순서다 — 섹션별로 계보순: search 갈래는
// decoupled/priority 갈래(Prioritized A* → Push and Swap → Push and Rotate — 우선순위
// 계획을 local primitive로 완성하는 decentralized 계열) → coupled(Joint-space A*, 모든 것의
// baseline) → hybrid(CBS), sampling 갈래는 coupled(joint 상태 motion tree) → subdimensional
// → implicit roadmap 순서.
// 집필된 페이지만 멀티라인 리터럴로 올린다 (멀티라인 리터럴 규약 — prerender/sitemap이
// contents 있는 블록만 파싱한다). 콘텐츠 모듈은 pages/algorithms/<section>/<slug>.tsx.
const data: IAlgoData[] = [
    {
        slug: "prioritized_astar",
        title: {en: "Prioritized A*", ko: "Prioritized A*"},
        section: "search",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./search/prioritized_astar")),
        sections: [
            {en: "From A* to Prioritized Planning", ko: "A*에서 우선순위 계획으로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // priority 갈래의 완성 — 예약을 동결하지 않고 push/swap primitive로 끝난 agent를 치운다.
    // 파라미터 무의존 plan-and-repair이고, component당 빈 셀 ≥2이면 완전하다는 주장이
    // 격자에서 어떻게 무너지는지(폭 1 통로)가 후속 Push and Rotate의 출발점이다.
    {
        slug: "push_and_swap",
        title: {en: "Push and Swap", ko: "Push and Swap"},
        section: "search",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./search/push_and_swap")),
        sections: [
            {en: "From Frozen Reservations to Push and Swap", ko: "얼린 예약에서 Push and Swap으로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // priority 갈래가 판정 절차로 완성되는 지점 — free graph를 biconnected subgraph와
    // plank로 분해하고, 맞교환은 degree-3 junction의 rotate로 확장된다. 폭 1 통로도 해친다.
    {
        slug: "push_and_rotate",
        title: {en: "Push and Rotate", ko: "Push and Rotate"},
        section: "search",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./search/push_and_rotate")),
        sections: [
            {en: "From Push and Swap to Push and Rotate", ko: "Push and Swap에서 Push and Rotate로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {
        slug: "joint_astar",
        title: {en: "Joint-Space A*", ko: "Joint-Space A*"},
        section: "search",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./search/joint_astar")),
        sections: [
            {en: "From Priorities to the Joint State", ko: "우선순위에서 joint 상태로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    {
        slug: "cbs",
        title: {en: "Conflict-Based Search", ko: "Conflict-Based Search"},
        section: "search",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./search/cbs")),
        sections: [
            {en: "From Joint States to Constraint Trees", ko: "joint 상태에서 constraint tree로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // --- sampling 갈래 — 집필 순서도 결합 축을 따른다: coupled → subdimensional →
    // implicit roadmap. 첫 회원 MA-RRT*는 논문 자체의 이산화(G-RRT*)로 DiscreteSpace
    // 위에서 구현됐다 — search 갈래와 같은 맵, 같은 비용 척도로 비교 가능하다.
    {
        slug: "ma_rrt_star",
        title: {en: "MA-RRT*", ko: "MA-RRT*"},
        section: "sampling",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./sampling/ma_rrt_star")),
        sections: [
            {en: "From Joint States to Random Trees", ko: "joint 상태에서 random tree로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // subdimensional 갈래의 첫 회원 — joint 공간을 통째로 탐색하는 대신 개별 policy를
    // 먼저 세우고 충돌이 나는 곳에서만 차원을 올린다. 논문 자체의 이산화 없이도
    // 격자 위에서 정확히 구성 가능(BFS tree가 optimal individual policy)하다.
    {
        slug: "subdimensional_rrt",
        title: {en: "sRRT", ko: "sRRT"},
        section: "sampling",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./sampling/subdimensional_rrt")),
        sections: [
            {en: "From Joint Trees to Individual Policies", ko: "joint 트리에서 개별 policy로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // implicit roadmap 갈래의 첫 회원 — 개별 PRM의 tensor product를 표본으로 더듬는다.
    // 논문 자체의 continuous 설정을 그대로 따른다(저장소의 raster 위에 disc robot).
    {
        slug: "drrt",
        title: {en: "dRRT", ko: "dRRT"},
        section: "sampling",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./sampling/drrt")),
        sections: [
            {en: "From Joint Trees to Implicit Roadmaps", ko: "joint 트리에서 implicit roadmap으로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
    // 같은 implicit roadmap 위의 informed asymptotically-optimal 후속 (Shome, Solovey,
    // Dobson, Halperin & Bekris 2020) — 계보상 dRRT의 다음 장. 개별 roadmap이 k-nearest에서
    // PRM* connection radius로 바뀌고, tree 탐색이 oracle growth + decoupled connector에서
    // cost-to-come rewiring + branch-and-bound + heuristic guidance로 바뀐다.
    {
        slug: "drrt_star",
        title: {en: "dRRT*", ko: "dRRT*"},
        section: "sampling",
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./sampling/drrt_star")),
        sections: [
            {en: "From Implicit Roadmaps to Asymptotic Optimality", ko: "implicit roadmap에서 점근적 최적성으로"},
            {en: "Properties and Complexity", ko: "성질과 복잡도"},
            {en: "The Algorithm", ko: "알고리즘"},
            {en: "What It Guarantees, What It Cannot", ko: "보장하는 것, 못 하는 것"},
            {en: "Demo", ko: "Demo"},
            {en: "Implementation", ko: "Implementation"},
            {en: "References", ko: "References"},
        ],
    },
];

export default data;
