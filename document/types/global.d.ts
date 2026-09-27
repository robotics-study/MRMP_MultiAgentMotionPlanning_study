import {ComponentType} from "react";

// 영/한 두 언어 문자열 쌍. 알고리즘 제목·섹션 등 언어에 따라 바뀌는 메타데이터에 쓴다.
export interface Localized<T = string> {
    en: T,
    ko: T,
}

// 이 저장소의 유일한 대분류(section) — 알고리즘 레지스트리·사이드바·홈이 공유한다.
// 단일 로봇 내비게이션은 자매 저장소(navigation study)가 다루고, 여기는 MAPF만 다룬다.
export type AlgoSection = "mapf";

export interface ISupportedExample {
    python?: boolean,
    "c++"?: boolean,
}

export interface IAlgoData {
    // URL 경로(/algo/<slug>)이자 configs/mapf/<slug>.yaml, 소스 파일명과 동일한 식별자.
    slug: string,
    title: Localized,
    supportedExample?: ISupportedExample,
    // 지연 로딩(React.lazy)된 컴포넌트일 수 있다. contents 가 없으면 아직 집필되지 않은 페이지.
    contents?: ComponentType,
    // 본문 major 섹션(h2) 제목 목록 — 사이드바/TOC/검색 인덱스가 공유한다.
    // 렌더된 헤딩 텍스트(현재 언어)와 문자열이 일치해야 앵커(slug)가 맞는다.
    sections?: Localized[],
}
