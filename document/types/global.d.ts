import {ComponentType} from "react";

// 영/한 두 언어 문자열 쌍. 알고리즘 제목·섹션 등 언어에 따라 바뀌는 메타데이터에 쓴다.
export interface Localized<T = string> {
    en: T,
    ko: T,
}

// 대분류(section) — 계보의 갈래 그 자체. search/sampling 은 survey 의 planner 타입 분류
// 축이고, decentralized 는 그 위에 선 세 번째 갈래 — 오프라인 계획이라는 매개체 자체를
// 버리고 실행 시간의 스텝별 협상만 남긴다. 네 번째 갈래 kinodynamic 은 반대로 닫는다 —
// 계획을 되돌려 받고 그 위에 속도 한계와 dwell semantics 을 실고, 아래에선 search 갈래의
// CBS 가 조용히 계속 돈다. 알고리즘 레지스트리·사이드바·홈이 공유하고, 소스 코드
// 디렉토리(python/mrmp/<section>)와 1:1 로 대응한다. 단일 로봇 내비게이션은
// 자매 저장소(navigation study)가 다룬다.
export type AlgoSection = "search" | "sampling" | "decentralized" | "kinodynamic";

export interface ISupportedExample {
    python?: boolean,
    "c++"?: boolean,
}

export interface IAlgoData {
    // URL 경로(/algo/<slug>)이자 configs/<section>/<slug>.yaml, 소스 파일명과 동일한 식별자.
    slug: string,
    title: Localized,
    // 이 알고리즘이 속한 계보 갈래 — 홈/사이드바가 섹션별로 묶고 코드 트리도 같은 이름이다.
    section: AlgoSection,
    supportedExample?: ISupportedExample,
    // 지연 로딩(React.lazy)된 컴포넌트일 수 있다. contents 가 없으면 아직 집필되지 않은 페이지.
    contents?: ComponentType,
    // 본문 major 섹션(h2) 제목 목록 — 사이드바/TOC/검색 인덱스가 공유한다.
    // 렌더된 헤딩 텍스트(현재 언어)와 문자열이 일치해야 앵커(slug)가 맞는다.
    sections?: Localized[],
}
