import {lazy} from "react";
import {IAlgoData} from "../../../types/global";

// 알고리즘 메타데이터는 여기서만 관리한다. 콘텐츠 모듈은 lazy import로 분리해
// 홈/목록 화면에서는 불러오지 않는다 (초기 번들 축소).
// sections의 en/ko 문자열은 각 언어로 렌더된 본문 h2 헤딩과 정확히 일치해야
// 사이드바/TOC/검색 앵커(slug)가 맞는다.
// 배열 순서가 사이드바·pager의 진행 순서다 — 계보순: decoupled(Prioritized A*) →
// coupled(Joint-space A*, 모든 것의 baseline) → hybrid(CBS). 집필된 페이지만 올린다 (멀티라인 리터럴
// 규약 — prerender/sitemap이 contents 있는 블록만 파싱한다). 콘텐츠 모듈은
// pages/algorithms/<slug>.tsx. 미집필 planned 항목은 한 줄 항목으로 둔다.
const data: IAlgoData[] = [
    {
        slug: "prioritized_astar",
        title: {en: "Prioritized A*", ko: "Prioritized A*"},
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./prioritized_astar")),
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
    {
        slug: "joint_astar",
        title: {en: "Joint-Space A*", ko: "Joint-Space A*"},
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./joint_astar")),
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
        supportedExample: {python: true, "c++": true},
        contents: lazy(() => import("./cbs")),
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
];

export default data;
