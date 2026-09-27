import {useCallback} from "react";
import {useLocation, useNavigate, useSearchParams} from "react-router-dom";
import {AlgoSection} from "../../types/global";

// 페이지 전환 단일 진입점. 알고리즘은 /algo/<slug>, 대분류 소개는 /section/<key> 경로로,
// 언어는 ?lang=ko 쿼리로, 섹션 딥링크는 #anchor 해시로 표현한다. 경로 기반이라 빌드 시
// 페이지별 정적 셸을 미리 만들 수 있고, 크롤러가 페이지마다 다른 메타를 본다. go(null)은
// 홈(랜딩). anchor가 있으면 TOC가 헤딩에 id를 부여한 뒤 해당 위치로 스크롤하고, 없으면
// 최상단으로 올린다.
export function useAlgoNav() {
    const navigate = useNavigate()
    const location = useLocation()
    const [params] = useSearchParams()
    const algoMatch = location.pathname.match(/^\/algo\/([a-z0-9_]+)/)
    const sectionMatch = location.pathname.match(/^\/section\/([a-z]+)/)
    const current = algoMatch ? algoMatch[1] : null
    const currentSection = sectionMatch ? (sectionMatch[1] as AlgoSection) : null

    const navigateTo = useCallback((pathname: string, anchor?: string) => {
        // 언어 선택은 URL로 유지한다 (공유·크롤링 시 언어 변형이 살아남도록).
        const sp = new URLSearchParams()
        const lang = params.get("lang")
        if (lang) sp.set("lang", lang)
        const search = sp.toString()
        navigate({
            pathname,
            search: search ? `?${search}` : "",
            hash: anchor ? `#${anchor}` : "",
        })
        if (!anchor) window.scrollTo({top: 0})
    }, [navigate, params])

    const go = useCallback((slug: string | null, anchor?: string) => {
        navigateTo(slug ? `/algo/${slug}` : "/", anchor)
    }, [navigateTo])

    const goSection = useCallback((key: AlgoSection, anchor?: string) => {
        navigateTo(`/section/${key}`, anchor)
    }, [navigateTo])

    return {current, currentSection, go, goSection}
}
