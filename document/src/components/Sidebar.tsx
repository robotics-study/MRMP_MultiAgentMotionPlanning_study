import {useEffect, useState} from "react";
import algorithms from "../pages/algorithms";
import {SECTIONS} from "../pages/algorithms/roadmap";
import {AlgoSection} from "../../types/global";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";
import cn from "../libs/cn";

const Chevron = () => (
    <svg className="sb-chev" width="11" height="11" viewBox="0 0 24 24" fill="none"
         stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round"
         aria-hidden="true">
        <path d="m9 6 6 6-6 6"/>
    </svg>
)

// 좌측 알고리즘 네비게이션 — 대분류 disclosure 안에 학습 순서대로 알고리즘을 묶는다.
// 미집필 항목은 "+ n more soon"으로 접어, 집필된 페이지 위주로 보여 준다.
const Sidebar = ({open: mobileOpen, onNavigate}: { open?: boolean; onNavigate?: () => void }) => {
    const {current, currentSection: introSection, go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    // 홈에서는 첫 대분류를 펼쳐 목차 역할을 하게 한다.
    const defaultOpen = introSection ?? SECTIONS[0].key
    const [opened, setOpened] = useState<Set<AlgoSection>>(() => new Set([defaultOpen]))
    const [plannedShown, setPlannedShown] = useState(false)

    // 검색 등 외부 경로로 페이지가 바뀌면 그 대분류를 펼친다 (사용자 토글은 유지).
    useEffect(() => {
        if (!introSection) return
        setOpened((prev) => {
            if (prev.has(introSection)) return prev
            return new Set(prev).add(introSection)
        })
    }, [introSection])

    const toggleSection = (key: AlgoSection) => setOpened((prev) => {
        const next = new Set(prev)
        if (next.has(key)) next.delete(key)
        else next.add(key)
        return next
    })

    const open = (slug: string) => {
        go(slug)
        onNavigate?.()
    }

    return (
        <aside className={cn("sidebar", mobileOpen && "open")}>
            <h4>{t("Overview", "개요")}</h4>
            <a className={cn(!current && !introSection && "active")} onClick={() => {
                go(null)
                onNavigate?.()
            }}>{t("Home", "홈")}</a>

            {SECTIONS.map((sec) => {
                const items = algorithms
                const ready = items.filter((a) => a.contents).length
                const isOpen = opened.has(sec.key)
                return (
                    <div key={sec.key} className={cn("sb-group", isOpen && "open")}>
                        <button type="button" className="sb-head" onClick={() => toggleSection(sec.key)}
                                aria-expanded={isOpen}>
                            <Chevron/>
                            {pick(lang, sec.title)}
                            <span className="sb-count">{ready}/{items.length}</span>
                        </button>
                        <div className="sb-body">
                            <div>
                                <a className={cn(introSection === sec.key && "active")}
                                   onClick={() => {
                                       goSection(sec.key)
                                       onNavigate?.()
                                   }}>
                                    Introduction
                                </a>
                                {/* 펼침 상태에서는 레지스트리(학습) 순서를 유지한 채 미집필을 dim으로 끼워 넣는다 */}
                                {(plannedShown ? items : items.filter((a) => a.contents)).map((a) => a.contents
                                    ? (
                                        <a key={a.slug}
                                           className={cn(current === a.slug && "active")}
                                           onClick={() => open(a.slug)}>
                                            {pick(lang, a.title)}
                                        </a>
                                    ) : (
                                        <span key={a.slug} className="planned">
                                            {pick(lang, a.title)}
                                        </span>
                                    ))}
                                {items.some((a) => !a.contents) && (
                                    <button type="button" className="sb-more"
                                            onClick={() => setPlannedShown((v) => !v)}
                                            aria-expanded={plannedShown}>
                                        {plannedShown ? t("show less", "show less") : t("show all", "전체 보기")}
                                    </button>
                                )}
                            </div>
                        </div>
                    </div>
                )
            })}
        </aside>
    )
}

export default Sidebar
