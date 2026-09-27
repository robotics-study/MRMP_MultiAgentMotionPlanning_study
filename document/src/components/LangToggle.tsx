import {useLang} from "../libs/i18n";
import cn from "../libs/cn";

// nav_study 헤더의 언어 토글을 그대로 이식 — 글로브 아이콘 + KO / EN 세그먼트.
// 정적 사이트는 언어별 URL로 이동하지만, SPA라 <a> 대신 <button>으로 컨텍스트만 바꾼다.
const LangToggle = () => {
    const {lang, setLang} = useLang()

    return (
        <div className="lang-toggle" role="group" aria-label="language">
            <svg className="g-ico" width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor"
                 strokeWidth="1.9" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
                <circle cx="12" cy="12" r="9"/>
                <path d="M3 12h18"/>
                <path d="M12 3c2.5 2.5 3.8 5.6 3.8 9s-1.3 6.5-3.8 9c-2.5-2.5-3.8-5.6-3.8-9S9.5 5.5 12 3Z"/>
            </svg>
            <button type="button" className={cn(lang === "ko" && "active")}
                    aria-pressed={lang === "ko"} onClick={() => setLang("ko")}>KO
            </button>
            <span className="l-sep" aria-hidden="true">/</span>
            <button type="button" className={cn(lang === "en" && "active")}
                    aria-pressed={lang === "en"} onClick={() => setLang("en")}>EN
            </button>
        </div>
    )
}

export default LangToggle
