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
];

// 대분류 — 홈의 큰 섹션이자 사이드바 disclosure 단위. 이 저장소는 MAPF 하나가
// 전부다 (파일 구조 pages/algorithms/<slug>/ 도 slug만 공유한다). 알고리즘 배치는
// 항상 계보순: decoupled(Prioritized A*) → coupled(Joint-space A*, 모든 것의 baseline)
// → hybrid(CBS) — CBS가 앞 둘을 "각자 계획"과 "전체 최적"의 양극으로 소개하므로
// 그 두 극을 먼저 보여 준다.
export const SECTIONS: Array<{
    key: AlgoSection;
    title: Localized<string>;
    desc: Localized<string>;
}> = [
    {
        key: "mapf",
        title: {en: "Multi-Agent Path Finding", ko: "Multi-Agent Path Finding"},
        desc: {
            en: "Planning paths for several robots on one shared map: a collision-free " +
                "space-time path per agent. Decoupled prioritized planning, coupled " +
                "joint-space search, and the hybrid in between (CBS).",
            ko: "여러 로봇이 하나의 지도를 공유할 경로를 계획한다: agent마다 충돌 없는 " +
                "시공간 경로. decoupled 우선순위 계획, coupled joint-space 탐색, 그리고 " +
                "그 사이 어딘가의 CBS.",
        },
    },
];
