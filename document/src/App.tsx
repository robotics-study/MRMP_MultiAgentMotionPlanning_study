import {Suspense, useCallback, useEffect, useMemo, useState} from "react";
import algorithms from "./pages/algorithms";
import sectionIntros from "./pages/sections";
import {applyPageMeta, algoMeta, sectionMeta} from "./libs/seo";
import Header from "./components/Header";
import Sidebar from "./components/Sidebar";
import Toc from "./components/Toc";
import Footer from "./components/Footer";
import Home from "./pages/home/Home";
import AlgorithmContents from "./components/AlgorithmContents";
import SectionContents from "./components/SectionContents";
import {BASE_PATH} from "./libs/url";
import {BrowserRouter, Routes, Route} from "react-router-dom";
import {useAlgoNav} from "./libs/nav";
import cn from "./libs/cn";
import {LangProvider, useLang} from "./libs/i18n";

const PageSelector = () => {
    const {lang} = useLang()
    const {current, currentSection} = useAlgoNav()
    const [menuOpen, setMenuOpen] = useState(false)
    const closeMenu = useCallback(() => setMenuOpen(false), [])

    // 집필되지 않은 slug(planned 항목 클릭 등)은 intro도 없어 그대로 홈이 렌더된다.
    const currentAlgo = useMemo(
        () => algorithms.find((item) => item.slug === current && item.contents),
        [current],
    )

    const currentIntro = useMemo(
        () => sectionIntros.find((item) => item.key === currentSection),
        [currentSection],
    )

    // 페이지 전환·언어 전환마다 제목·메타를 현재 뷰에 맞춘다 (SPA 이므로 크롤러/프리뷰용 갱신).
    useEffect(() => {
        applyPageMeta(
            currentIntro ? sectionMeta(lang, currentIntro) : algoMeta(lang, currentAlgo))
    }, [currentAlgo, currentIntro, lang])

    const inDoc = !!(currentAlgo || currentIntro)
    const loading = (
        <main className="content">
            <div className="grid place-items-center py-24 text-muted text-sm">Loading…</div>
        </main>
    )

    return (
        <>
            <Header onMenu={() => setMenuOpen((o) => !o)} showMenu={inDoc}/>
            {inDoc ? (
                <>
                    <div className="layout">
                        <Sidebar open={menuOpen} onNavigate={closeMenu}/>
                        <Suspense fallback={loading}>
                            {currentIntro
                                ? <SectionContents intro={currentIntro}/>
                                : <AlgorithmContents {...currentAlgo!}/>}
                        </Suspense>
                        <Toc pageKey={currentIntro ? `section:${currentIntro.key}` : currentAlgo!.slug}/>
                    </div>
                    <div className={cn("backdrop", menuOpen && "open")} onClick={closeMenu}/>
                </>
            ) : (
                <Home/>
            )}
            <Footer/>
        </>
    )
}

const App = () => {
    return <BrowserRouter basename={BASE_PATH || "/"}
                          future={{v7_startTransition: true, v7_relativeSplatPath: true}}>
        <LangProvider>
            <Routes>
                <Route path={"/"} element={<PageSelector/>}/>
                <Route path={"/algo/:slug"} element={<PageSelector/>}/>
                <Route path={"/section/:key"} element={<PageSelector/>}/>
                <Route path={"*"} element={<PageSelector/>}/>
            </Routes>
        </LangProvider>
    </BrowserRouter>
}

export default App
