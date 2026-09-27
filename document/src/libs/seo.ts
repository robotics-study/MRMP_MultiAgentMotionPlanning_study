// SPA 라우트 변경마다 제목·메타를 갱신한다 — 크롤러/링크 프리뷰가 현재 뷰를 반영하도록.
// document.title, description, Open Graph, canonical, hreflang, JSON-LD를 클라이언트에서
// 갱신한다. index.html에 정적으로 심어 둔 태그를 찾아 값만 바꾸고, 없으면 만든다.
// 설명문은 마케팅 문구가 아니라 학습 내용(주제·개념) 중심으로 쓴다.

import {Lang, pick} from "./i18n";
import {IAlgoData} from "../../types/global";
import {ALGO_BLURBS, SECTIONS} from "../pages/algorithms/roadmap";
import {ISectionIntro} from "../pages/sections";

const ORIGIN = "https://robotics-study.github.io";
const BASE_PATH = "/MRMP_MultiAgentMotionPlanning_study/";

const SITE: Record<Lang, string> = {
    en: "MRMP · Study",
    ko: "MRMP · Study",
}

function upsertMeta(attr: "name" | "property", key: string, content: string) {
    let el = document.head.querySelector<HTMLMetaElement>(`meta[${attr}="${key}"]`)
    if (!el) {
        el = document.createElement("meta")
        el.setAttribute(attr, key)
        document.head.appendChild(el)
    }
    el.setAttribute("content", content)
}

function upsertLink(rel: string, href: string, hreflang?: string) {
    const selector = hreflang
        ? `link[rel="${rel}"][hreflang="${hreflang}"]`
        : `link[rel="${rel}"]`
    let el = document.head.querySelector<HTMLLinkElement>(selector)
    if (!el) {
        el = document.createElement("link")
        el.rel = rel
        if (hreflang) el.hreflang = hreflang
        document.head.appendChild(el)
    }
    el.href = href
}

function upsertJsonLd(id: string, data: object) {
    let el = document.head.querySelector<HTMLScriptElement>(`script#${id}`)
    if (!el) {
        el = document.createElement("script")
        el.id = id
        el.type = "application/ld+json"
        document.head.appendChild(el)
    }
    el.textContent = JSON.stringify(data)
}

function clamp(text: string, max = 155): string {
    return text.length <= max ? text : text.slice(0, max - 1).trimEnd() + "…"
}

// 해시·잡다한 파라미터를 뺀 정규화 URL. subpath는 "algo/<slug>" | "section/<key>" | 없음(홈),
// 언어 변형은 ?lang=ko.
export function pageUrl(lang: Lang, subpath?: string): string {
    const path = subpath !== undefined ? `${subpath}/` : ""
    const qs = lang === "ko" ? "?lang=ko" : ""
    return `${ORIGIN}${BASE_PATH}${path}${qs}`
}

declare global {
    interface Window {
        gtag?: (...args: unknown[]) => void
    }
}

// SPA 라우트 변경마다 GA4 page_view를 직접 보낸다 (index.html은 send_page_view: false).
// 로컬 개발 트래픽은 집계를 오염시키므로 배포 호스트에서만 보낸다.
function trackPageView(title: string) {
    if (!window.location.hostname.endsWith("github.io")) return
    window.gtag?.("event", "page_view", {
        page_title: title,
        page_location: window.location.href,
        page_path: window.location.pathname + window.location.search,
    })
}

export interface PageMeta {
    title: string
    description: string
    lang: Lang
    // 정규화 URL의 하위 경로 ("algo/prioritized_astar" 등). 없으면 홈.
    subpath?: string
    // TechArticle JSON-LD의 about 목록 (본문 h2 제목들).
    topics?: string[]
}

export function applyPageMeta({title, description, lang, subpath, topics}: PageMeta) {
    const desc = clamp(description)
    const canonical = pageUrl(lang, subpath)
    document.title = title
    document.documentElement.lang = lang
    upsertMeta("name", "description", desc)
    upsertMeta("property", "og:title", title)
    upsertMeta("property", "og:description", desc)
    upsertMeta("property", "og:url", canonical)
    upsertMeta("property", "og:locale", lang === "ko" ? "ko_KR" : "en_US")
    upsertMeta("name", "twitter:title", title)
    upsertMeta("property", "twitter:description", desc)
    upsertLink("canonical", canonical)
    trackPageView(title)
    // 언어별 대체 URL: 같은 페이지의 en/ko 쌍.
    upsertLink("alternate", pageUrl("en", subpath), "en")
    upsertLink("alternate", pageUrl("ko", subpath), "ko")
    upsertLink("alternate", pageUrl("en", subpath), "x-default")
    // 페이지 구조화 데이터.
    if (subpath) {
        upsertJsonLd("page-jsonld", {
            "@context": "https://schema.org",
            "@type": "TechArticle",
            headline: title,
            description: desc,
            inLanguage: lang,
            url: canonical,
            isPartOf: {
                "@type": "WebSite",
                name: SITE[lang],
                url: pageUrl(lang),
            },
            about: topics ?? [],
        })
    } else {
        upsertJsonLd("page-jsonld", {
            "@context": "https://schema.org",
            "@type": "LearningResource",
            name: SITE[lang],
            description: desc,
            url: canonical,
            inLanguage: ["en", "ko"],
            learningResourceType: "Study notes",
            about: [
                "Robotics", "Multi-Agent Path Finding", "MAPF", "Multi-Robot Motion Planning",
                "Prioritized Planning", "Joint Space Search", "Conflict-Based Search", "CBS",
            ],
        })
    }
}

const HOME_DESC: Record<Lang, string> = {
    en:
        "Study notes on multi-agent motion planning (MAPF): prioritized planning, joint-space " +
        "search, and Conflict-Based Search (CBS), with interactive space-time conflict " +
        "visualizations and C++/Python implementations.",
    ko:
        "multi-agent motion planning(MAPF) 알고리즘 학습 노트: 우선순위 계획, joint-space " +
        "탐색, Conflict-Based Search(CBS). 충돌과 time step을 함께 재생하는 인터랙티브 " +
        "시각화와 C++/Python 구현.",
}

// 알고리즘 → 페이지 메타. 설명은 한 줄 소개(내용 요약) + 주요 절 제목으로 만든다.
export function algoMeta(lang: Lang, algo?: IAlgoData): PageMeta {
    if (!algo) {
        return {title: SITE[lang], description: HOME_DESC[lang], lang}
    }
    const title = pick(lang, algo.title)
    const blurb = ALGO_BLURBS.find((b) => b.slug === algo.slug)?.blurb
    const topicList = (algo.sections ?? []).map((s) => pick(lang, s))
    const topics = topicList.join(", ")
    const intro = blurb ? pick(lang, blurb) : ""
    const body = lang === "ko"
        ? `${intro} ${topics ? `주요 내용: ${topics}.` : ""}`.trim()
        : `${intro} ${topics ? `Topics: ${topics}.` : ""}`.trim()
    return {
        title: `${title} · ${SITE[lang]}`,
        description: body,
        lang,
        subpath: `algo/${algo.slug}`,
        topics: topicList,
    }
}

// 대분류 소개 → 페이지 메타. 설명은 대분류 한 줄 소개 + 본문 h2 제목으로 만든다.
export function sectionMeta(lang: Lang, intro: ISectionIntro): PageMeta {
    const section = SECTIONS.find((s) => s.key === intro.key)!
    const title = pick(lang, section.title)
    const topicList = intro.sections.map((s) => pick(lang, s))
    const topics = topicList.join(", ")
    const body = lang === "ko"
        ? `${pick(lang, section.desc)} 주요 내용: ${topics}.`
        : `${pick(lang, section.desc)} Topics: ${topics}.`
    return {
        title: `${title} · ${SITE[lang]}`,
        description: body,
        lang,
        subpath: `section/${intro.key}`,
        topics: topicList,
    }
}
