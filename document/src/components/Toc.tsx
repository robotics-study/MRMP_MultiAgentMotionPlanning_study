import {useEffect, useState} from "react";
import {slugify} from "../libs/slug";
import {useLang, useTr} from "../libs/i18n";
import cn from "../libs/cn";

interface TocItem {
    id: string
    text: string
    h3: boolean
}

// 우측 "On this page" — 렌더된 본문의 h2/h3를 스캔해 id 부여 + 스크롤스파이.
// 본문(특히 lazy 페이지·KaTeX·캔버스)이 비동기로 붙으므로 헤딩이 나타날 때까지 rAF로 재시도한다.
const Toc = ({pageKey}: { pageKey: string }) => {
    const {lang} = useLang()
    const t = useTr()
    const [items, setItems] = useState<TocItem[]>([])
    const [active, setActive] = useState("")

    useEffect(() => {
        let raf = 0
        let tries = 0
        const scan = () => {
            const inner = document.querySelector(".content-inner")
            const heads = inner ? Array.from(inner.querySelectorAll("h2, h3")) : []
            if (heads.length < 1) {
                if (tries++ < 60) raf = requestAnimationFrame(scan)
                return
            }
            const list: TocItem[] = heads.map((h) => {
                const text = (h.textContent ?? "").replace("¶", "").trim()
                if (!h.id) h.id = slugify(text)
                return {id: h.id, text, h3: h.tagName === "H3"}
            })
            setItems(list)

            // 검색/직접 진입의 #anchor 딥링크 — 헤딩이 생긴 지금 스크롤한다.
            const hash = decodeURIComponent(window.location.hash.replace("#", ""))
            if (hash) document.getElementById(hash)?.scrollIntoView()
        }
        setItems([])
        scan()
        return () => cancelAnimationFrame(raf)
        // 언어 전환 시 헤딩 텍스트가 바뀌므로 다시 스캔한다.
    }, [pageKey, lang])

    useEffect(() => {
        if (!items.length) return
        const spy = () => {
            let cur = ""
            for (const it of items) {
                const el = document.getElementById(it.id)
                if (el && el.getBoundingClientRect().top <= 120) cur = it.id
            }
            setActive(cur)
        }
        window.addEventListener("scroll", spy, {passive: true})
        spy()
        return () => window.removeEventListener("scroll", spy)
    }, [items])

    if (items.length < 2) return <aside className="toc" aria-hidden="true"/>

    return (
        <aside className="toc">
            <h4>{t("On this page", "목차")}</h4>
            {items.map((it) => (
                <a key={it.id}
                   href={`#${it.id}`}
                   className={cn(it.h3 && "h3", active === it.id && "active")}
                   onClick={(e) => {
                       e.preventDefault()
                       document.getElementById(it.id)?.scrollIntoView()
                       history.replaceState(null, "", `#${it.id}`)
                   }}>
                    {it.text}
                </a>
            ))}
        </aside>
    )
}

export default Toc
