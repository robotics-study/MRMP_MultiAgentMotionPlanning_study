import {IAlgoData} from "../../../types/global";

// 알고리즘 메타데이터는 여기서만 관리한다. 콘텐츠 모듈은 lazy import로 분리해
// 홈/목록 화면에서는 불러오지 않는다 (초기 번들 축소).
// sections의 en/ko 문자열은 각 언어로 렌더된 본문 h2 헤딩과 정확히 일치해야
// 사이드바/TOC/검색 앵커(slug)가 맞는다.
// 배열 순서가 사이드바·pager의 진행 순서다 — 계보순: decoupled(Prioritized A*) →
// coupled(Joint-space A*) → hybrid(CBS). 집필된 페이지만 올린다 (멀티라인 리터럴
// 규약 — prerender/sitemap이 contents 있는 블록만 파싱한다). 콘텐츠 모듈은
// pages/algorithms/<slug>.tsx.
const data: IAlgoData[] = [];

export default data;
