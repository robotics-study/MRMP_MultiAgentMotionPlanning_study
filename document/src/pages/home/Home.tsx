import {ISupportedExample, Localized} from "../../../types/global";
import algorithms from "../algorithms";
import {ALGO_BLURBS, SECTIONS} from "../algorithms/roadmap";
import BrandLogo from "../../components/BrandLogo";
import HeroSearch from "../../components/panels/HeroSearch";
import SiteHighlights from "../../components/panels/home/SiteHighlights";
import {useAlgoNav} from "../../libs/nav";
import {useLang, useTr, pick} from "../../libs/i18n";
import cn from "../../libs/cn";

const REPO = "https://github.com/robotics-study/MRMP_MultiAgentMotionPlanning_study"

const AlgoCard = ({slug, title, blurb, supportedExample, onOpen}: {
    slug: string
    title: Localized
    blurb?: Localized
    supportedExample?: ISupportedExample
    onOpen?: () => void
}) => {
    const {lang} = useLang()
    const t = useTr()
    const langs = supportedExample
        ? Object.entries(supportedExample).filter(([, v]) => v).map(([codeLang]) => codeLang)
        : []
    return (
        <div className={cn("doc-card", onOpen ? "clickable" : "dim")}
             role={onOpen ? "button" : undefined} tabIndex={onOpen ? 0 : undefined}
             onClick={onOpen}
             onKeyDown={(e) => onOpen && (e.key === "Enter" || e.key === " ") && onOpen()}>
            <div className="dc-head">
                <span className="dc-title">{pick(lang, title)}</span>
                {!onOpen && <span className="soon">soon</span>}
            </div>
            {blurb && <p className="dc-blurb">{pick(lang, blurb)}</p>}
            {onOpen && langs.length > 0 && (
                <div className="chips">
                    {langs.map((codeLang) => {
                        // 저장소 소스 경로 — 알고리즘 파일은 python/mrmp/mapf/ · cpp/include/mrmp/mapf/ 에 있다.
                        const file = codeLang === "c++"
                            ? `cpp/include/mrmp/mapf/${slug}.hpp`
                            : `python/mrmp/mapf/${slug}.py`
                        return (
                            <a key={codeLang} className="mini-chip" target="_blank" rel="noreferrer"
                               onClick={(e) => e.stopPropagation()}
                               href={`${REPO}/blob/main/${file}`}>
                                {t(`${codeLang} code`, `${codeLang} 코드`)}
                            </a>
                        )
                    })}
                </div>
            )}
        </div>
    )
}

const Home = () => {
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()
    const ready = algorithms.filter((a) => a.contents)
    const first = ready[0]?.slug
    const blurbOf = (slug: string) => ALGO_BLURBS.find((b) => b.slug === slug)?.blurb

    return (
        <main className="lander">
            <div className="lander-top">
                <BrandLogo size={54} gradId="navLanderLogo"/>
                <h1>mrmp<span className="wm-dim"> study</span></h1>
                <p className="sub">
                    {t(
                        "A study of multi-agent motion planning (MAPF) — each algorithm derived, " +
                        "proven, and shown as the real C++ and Python source that implements it.",
                        "multi-agent motion planning(MAPF) 알고리즘을 하나하나 유도하고 증명하고, " +
                        "그것을 구현한 실제 C++·Python 소스까지 함께 읽는 학습 노트.",
                    )}
                </p>
                <div className="lander-chips">
                    <span className="chip">MAPF</span>
                    <span className="chip">Prioritized A*</span>
                    <span className="chip">Joint-Space A*</span>
                    <span className="chip">CBS</span>
                    <span className="chip">C++ / Python</span>
                </div>
                <div className="lander-btns">
                    {first && (
                        <button className="btn btn-primary" onClick={() => go(first)}>
                            {t("Start reading", "학습 시작")}
                        </button>
                    )}
                    <a className="btn btn-ghost" href={REPO} target="_blank" rel="noopener noreferrer">GitHub</a>
                </div>
            </div>

            <HeroSearch/>

            <SiteHighlights/>

            {SECTIONS.map((sec) => (
                <div key={sec.key} className="lander-cat">
                    <div className="part-head">
                        <h3>
                            <span className="part-index">I</span>
                            {pick(lang, sec.title)}
                            <a className="part-intro" onClick={() => goSection(sec.key)}>
                                Introduction →
                            </a>
                        </h3>
                        <p className="part-desc">{pick(lang, sec.desc)}</p>
                    </div>
                    <div className="card-grid">
                        {algorithms.map((a) => (
                            <AlgoCard key={a.slug} slug={a.slug} title={a.title}
                                      blurb={blurbOf(a.slug)}
                                      supportedExample={a.supportedExample}
                                      onOpen={a.contents ? () => go(a.slug) : undefined}/>
                        ))}
                    </div>
                </div>
            ))}
        </main>
    )
}

export default Home
