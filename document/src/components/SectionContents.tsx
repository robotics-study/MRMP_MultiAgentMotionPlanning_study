import 'katex/dist/katex.min.css';
import algorithms from "../pages/algorithms";
import {ISectionIntro} from "../pages/sections";
import {SECTIONS} from "../pages/algorithms/roadmap";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";

// 대분류 소개 페이지 래퍼 — AlgorithmContents와 같은 본문 프레임에 소개 콘텐츠를 담는다.
// pager의 다음은 그 대분류의 첫 집필 알고리즘이다.
const SectionContents = ({intro}: {intro: ISectionIntro}) => {
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    const section = SECTIONS.find((s) => s.key === intro.key)!
    const Contents = intro.contents
    const firstAlgo = algorithms.find((a) => a.contents)

    // 이전은 순서상 앞 대분류의 소개, 첫 대분류면 홈.
    const idx = SECTIONS.findIndex((s) => s.key === intro.key)
    const prevSection = idx > 0 ? SECTIONS[idx - 1] : undefined

    return (
        <main className="content">
            <article className="content-inner">
                <p className="eyebrow">{t("Overview", "개요")}</p>
                <h1>{pick(lang, section.title)}</h1>

                <Contents/>

                <nav className="pager">
                    {prevSection
                        ? <a onClick={() => goSection(prevSection.key)}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">{pick(lang, prevSection.title)}</div>
                        </a>
                        : <a onClick={() => go(null)}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">{t("Home", "홈")}</div>
                        </a>}
                    {firstAlgo
                        ? <a className="next" onClick={() => go(firstAlgo.slug)}>
                            <div className="dir">{t("Next →", "다음 →")}</div>
                            <div className="ttl">{pick(lang, firstAlgo.title)}</div>
                        </a>
                        : <span/>}
                </nav>
            </article>
        </main>
    )
}

export default SectionContents
