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
            {en: "All Four, in Reading Order", ko: "네 회원 전부, 읽는 순서"},
        ],
    },
    // 세 번째 갈래 — 계획(plan)이라는 매개체 자체를 버린다. search 갈래의 우선순위
    // 계보가 완성한 규율을 실행 시간의 협상으로만 다시 세운다 (PIBT).
    {
        key: "decentralized",
        contents: lazy(() => import("./DecentralizedPlanning")),
        sections: [
            {en: "The Problem, Rebuilt at the Time Step", ko: "스텝에서 다시 세운 문제"},
            {en: "Why Drop the Plan", ko: "계획을 버리는 이유"},
            {en: "What Survives Without a Plan", ko: "계획 없이 남는 것"},
            {en: "What Is Written, and What Comes Next", ko: "집필된 것과 그다음"},
        ],
    },
    // 네 번째 갈래 — 계보의 종착점. 계획을 되돌려 받고 시간(속도 한계·dwell)을 실는다.
    // search 갈래의 CBS가 아래에서 조용히 계속 돌고, MAPF-POST가 TPG → STN 변환으로
    // 가장 빠른 실행 스케줄을 낸다.
    {
        key: "kinodynamic",
        contents: lazy(() => import("./KinodynamicPlanning")),
        sections: [
            {en: "The Problem, With a Clock", ko: "시계를 얹은 문제"},
            {en: "Why Post-Process", ko: "왜 후처리인가"},
            {en: "What the Search Branch Hands Over", ko: "search 갈래가 넘겨주는 것"},
            {en: "What Is Written, and Where the Genealogy Ends", ko: "집필된 것과 계보가 끝나는 지점"},
        ],
    },
]

export default data
