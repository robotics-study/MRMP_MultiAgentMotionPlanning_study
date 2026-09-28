import {ComponentType, lazy} from "react";
import {AlgoSection, Localized} from "../../../types/global";

// 대분류 소개 페이지 레지스트리. 알고리즘 각론 전에 "이 갈래가 무엇인가"류의 개념
// 설명을 담는다. sections의 en/ko 문자열은 렌더된 본문 h2 헤딩과 정확히 일치해야
// TOC/검색 앵커(slug)가 맞는다. 집필된 페이지는 멀티라인 리터럴로 쓴다 (스크립트
// 파싱 규약).
export interface ISectionIntro {
    key: AlgoSection;
    contents: ComponentType;
    sections: Localized[];
}

const data: ISectionIntro[] = [
    {
        key: "search",
        contents: lazy(() => import("./SearchPlanning")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Why It Is Hard", ko: "왜 어려운가"},
            {en: "Decoupled, Coupled, and In Between", ko: "Decoupled, Coupled, 그리고 그 사이"},
            {en: "All Three Poles, in Reading Order", ko: "세 극단, 읽는 순서"},
        ],
    },
    {
        key: "sampling",
        contents: lazy(() => import("./SamplingPlanning")),
        sections: [
            {en: "The Problem", ko: "문제 정의"},
            {en: "Why Sample", ko: "왜 sampling인가"},
            {en: "The Same Coupling Axis, Again", ko: "다시 나타난 결합의 축"},
            {en: "What Is Coming", ko: "구현 예정"},
        ],
    },
]

export default data
