import {createContext, ReactNode, useCallback, useContext, useEffect} from "react";
import {useSearchParams} from "react-router-dom";
import {Localized} from "../../types/global";

export type Lang = "en" | "ko"

const STORE_KEY = "mr-lang"

// 언어 결정 우선순위: URL ?lang → localStorage 저장값 → 브라우저 언어 → 영어.
// (URL 값은 LangProvider가 별도로 우선 처리하므로 여기서는 저장값/브라우저만 본다.)
function detectLang(): Lang {
    try {
        const stored = localStorage.getItem(STORE_KEY)
        if (stored === "ko" || stored === "en") return stored
    } catch {
        // private 모드 등 localStorage 접근 불가 — 브라우저 언어로 폴백.
    }
    return navigator.language?.toLowerCase().startsWith("ko") ? "ko" : "en"
}

interface LangContextValue {
    lang: Lang
    setLang: (lang: Lang) => void
}

const LangContext = createContext<LangContextValue>({lang: "en", setLang: () => {}})

export function LangProvider({children}: {children: ReactNode}) {
    const [params, setParams] = useSearchParams()
    const raw = params.get("lang")
    const urlLang: Lang | null = raw === "ko" || raw === "en" ? raw : null
    const lang: Lang = urlLang ?? detectLang()

    // URL에 lang이 없으면 감지값을 주소에 반영해 공유 가능한 링크로 만든다.
    // 영어는 기본값이라 파라미터를 생략(깔끔한 URL), 한국어일 때만 ?lang=ko를 붙인다.
    useEffect(() => {
        if (!urlLang && lang === "ko") {
            const next = new URLSearchParams(params)
            next.set("lang", "ko")
            setParams(next, {replace: true})
        }
    }, [urlLang, lang, params, setParams])

    // <html lang>을 실제 언어에 맞춰 접근성/크롤러 신호를 정확히 준다. 선택값도 저장.
    useEffect(() => {
        document.documentElement.lang = lang
        try {
            localStorage.setItem(STORE_KEY, lang)
        } catch {
            // 저장 불가 환경 — URL/감지로 계속 동작하므로 무시.
        }
    }, [lang])

    const setLang = useCallback((next: Lang) => {
        const sp = new URLSearchParams(params)
        if (next === "ko") sp.set("lang", "ko")
        else sp.delete("lang")
        try {
            localStorage.setItem(STORE_KEY, next)
        } catch {
            // 무시 — URL 파라미터만으로도 전환이 반영된다.
        }
        setParams(sp)
    }, [params, setParams])

    return <LangContext.Provider value={{lang, setLang}}>{children}</LangContext.Provider>
}

export const useLang = () => useContext(LangContext)

// 문자열 prop(캡션·placeholder·aria-label 등) 번역용. t("English", "한국어").
export const useTr = () => {
    const {lang} = useLang()
    return (en: string, ko: string) => (lang === "ko" ? ko : en)
}

// Localized 메타데이터에서 현재 언어 값을 뽑는다 (컨텍스트 밖의 순수 함수 — search/seo 용).
export function pick<T>(lang: Lang, value: Localized<T>): T {
    return lang === "ko" ? value.ko : value.en
}

// 본문 블록(문단·헤딩·리스트) 번역용. 언어에 맞는 노드만 렌더한다.
export const T = ({en, ko}: {en: ReactNode; ko: ReactNode}) => {
    const {lang} = useLang()
    return <>{lang === "ko" ? ko : en}</>
}
