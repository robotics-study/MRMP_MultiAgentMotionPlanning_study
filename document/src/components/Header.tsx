import BrandLogo from "./BrandLogo";
import SearchBox from "./SearchBox";
import LangToggle from "./LangToggle";
import {useAlgoNav} from "../libs/nav";
import {useTr} from "../libs/i18n";

const REPO = "https://github.com/robotics-study/MRMP-Multi-Agent-Motion-Planning--study"
// 상위 학습 아카이브(robotics-study.github.io). 이 앱은 그 하위 프로젝트라 브랜드만으로는
// 허브로 되돌아갈 방법이 없어, 브레드크럼 부모 링크로 탈출로를 제공한다.
const HUB = "https://robotics-study.github.io/"

const ExtIcon = () => (
    <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
         strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <path d="M7 17 17 7M8 7h9v9"/>
    </svg>
)

// 스티키 상단 바 — 브랜드 · 네비 · 검색. 테마 토글은 없다(시스템 설정을 그대로 따름).
const Header = ({onMenu, showMenu}: { onMenu: () => void; showMenu: boolean }) => {
    const {go} = useAlgoNav()
    const t = useTr()

    return (
        <header className="topbar">
            {showMenu && (
                <button className="iconbtn menu-btn" onClick={onMenu} aria-label="menu">
                    <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"
                         strokeLinecap="round" aria-hidden="true">
                        <path d="M4 7h16M4 12h16M4 17h16"/>
                    </svg>
                </button>
            )}

            <div className="brandcrumb">
                <a className="brand hub" href={HUB} aria-label="robotics-study home">
                    <BrandLogo gradId="navBrandLogo"/>
                    <span className="wm">robotics<span className="wm-dim"> study</span></span>
                </a>
                <span className="crumb-sep" aria-hidden="true">/</span>
                <button className="brand" onClick={() => go(null)} aria-label="Home">
                    <span className="wm">mrmp</span>
                </button>
            </div>

            <nav className="topnav">
                <a onClick={() => go(null)}>{t("Overview", "개요")}</a>
                <a href={REPO} target="_blank" rel="noopener noreferrer">GitHub<ExtIcon/></a>
            </nav>

            <span className="spacer"/>
            <SearchBox/>
            <LangToggle/>
        </header>
    )
}

export default Header
