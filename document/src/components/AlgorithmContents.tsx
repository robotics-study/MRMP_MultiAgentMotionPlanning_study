import {IAlgoData} from "../../types/global";
import 'katex/dist/katex.min.css';
import algorithms from "../pages/algorithms";
import {SECTIONS} from "../pages/algorithms/roadmap";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// supportedExample(python/c++) → 실제 구현 파일로 가는 chip 링크. 알고리즘 파일은
// python/mrmp/search/<slug>.py · cpp/include/mrmp/search/<slug>.hpp 에 있다.
const codeLinkFor = (algo: IAlgoData, language: string): string | null => {
    if (language === "c++") return `${REPO}/blob/main/cpp/include/mrmp/search/${algo.slug}.hpp`
    if (language === "python") return `${REPO}/blob/main/python/mrmp/search/${algo.slug}.py`
    return null
}

const AlgorithmContents = (algo: IAlgoData) => {
    const {title, slug, contents: Contents, supportedExample} = algo
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    // pager는 집필된 페이지 사이만 오간다 (registry 배열 순서 = 학습 순서).
    const ready = algorithms.filter((a) => a.contents)
    const idx = ready.findIndex((a) => a.slug === slug)
    const prev = idx > 0 ? ready[idx - 1] : undefined
    const next = idx >= 0 && idx < ready.length - 1 ? ready[idx + 1] : undefined

    // eyebrow: 대분류 이름 (이 저장소는 MAPF 하나뿐).
    const secTitle = SECTIONS[0].title

    const codeLinks = supportedExample
        ? Object.entries(supportedExample)
            .filter(([, v]) => v)
            .map(([language]) => ({language, href: codeLinkFor(algo, language)}))
            .filter((l): l is {language: string; href: string} => !!l.href)
        : []

    return (
        <main className="content">
            <article className="content-inner">
                <p className="eyebrow">{pick(lang, secTitle)}</p>
                <h1>{pick(lang, title)}</h1>

                {codeLinks.length > 0 && (
                    <div className="code-links">
                        <span className="cl-label">{t("Source code", "소스 코드")}</span>
                        {codeLinks.map(({language, href}) => (
                            <a key={language} href={href} target="_blank" rel="noopener noreferrer">
                                {language}
                            </a>
                        ))}
                        <a href={`${REPO}/blob/main/python/demos/demo_${slug}.py`}
                           target="_blank" rel="noopener noreferrer">demo</a>
                    </div>
                )}

                {Contents ? <Contents/> : null}

                <nav className="pager">
                    {prev
                        ? <a onClick={() => go(prev.slug)}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">{pick(lang, prev.title)}</div>
                        </a>
                        : <a onClick={() => goSection(SECTIONS[0].key)}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">
                                {pick(lang, secTitle)} · Introduction
                            </div>
                        </a>}
                    {next
                        ? <a className="next" onClick={() => go(next.slug)}>
                            <div className="dir">{t("Next →", "다음 →")}</div>
                            <div className="ttl">{pick(lang, next.title)}</div>
                        </a>
                        : <span/>}
                </nav>
            </article>
        </main>
    )
}

export default AlgorithmContents
