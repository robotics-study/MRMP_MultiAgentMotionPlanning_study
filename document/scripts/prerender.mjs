// vite build 뒤에 실행되어 페이지별 정적 HTML 셸을 만든다: dist/algo/<slug>/index.html.
// 각 셸에는 그 페이지의 title, description, canonical, hreflang, Open Graph, JSON-LD 가
// 미리 박혀 있어서 크롤러가 JS 를 실행하기 전에도 페이지별 메타를 본다. 본문은 SPA 가
// 부팅하며 그린다 (asset 경로는 vite base 가 절대 경로라 하위 디렉토리에서도 동작).
// 페이지 데이터는 소스의 리터럴을 파싱한다: pages/algorithms/index.ts (title·sections),
// pages/algorithms/roadmap.ts (한 줄 소개 blurb). 집필된 페이지는 멀티라인 리터럴로 쓰는
// 것이 레지스트리 컨벤션이라, 그 블록만 잡으면 미집필 한 줄 항목은 자연히 제외된다.
import {cpSync, mkdirSync, readFileSync, writeFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const ORIGIN = "https://robotics-study.github.io";
const BASE = "/mrmp_introduction/";
const SITE = "MRMP · Study";

// {en: "..", ko: ".."} 리터럴에서 en 문자열을 뽑는다 ("a" + "b" 연결 포함).
const enOf = (block) => {
    const m = block.match(/en:\s*((?:"(?:[^"\\]|\\.)*"\s*\+?\s*)+)/);
    if (!m) return "";
    return [...m[1].matchAll(/"((?:[^"\\]|\\.)*)"/g)].map((x) => x[1]).join("");
};

// index.ts 를 페이지 블록으로 잘라 slug, 제목(en), 절 제목(en) 목록을 얻는다.
const parseAlgos = () => {
    const src = readFileSync(join(root, "src/pages/algorithms/index.ts"), "utf-8");
    const blocks = src.split(/\n\s*\{\s*\n\s*slug:/).slice(1);
    return blocks
        .filter((raw) => raw.includes("contents:"))
        .map((raw) => {
            const slug = raw.match(/^\s*"([a-z0-9_]+)"/)[1];
            const title = enOf(raw.match(/title:\s*\{[\s\S]*?\}/)[0]);
            const sectionsBlock = raw.match(/sections:\s*\[([\s\S]*?)\]/);
            const sections = sectionsBlock
                ? [...sectionsBlock[1].matchAll(/\{en:\s*"((?:[^"\\]|\\.)*)"/g)].map((m) => m[1])
                : [];
            return {slug, title, sections};
        });
};

// roadmap.ts 에서 페이지별 한 줄 소개(en)를 얻는다.
const parseBlurbs = () => {
    const src = readFileSync(join(root, "src/pages/algorithms/roadmap.ts"), "utf-8");
    const out = new Map();
    for (const raw of src.split(/\n\s*\{\s*\n\s*slug:/).slice(1)) {
        const slugMatch = raw.match(/^\s*"([a-z0-9_]+)"/);
        if (!slugMatch) continue;
        const blurbBlock = raw.match(/blurb:\s*\{[\s\S]*?\n\s*\}/);
        if (blurbBlock) out.set(slugMatch[1], enOf(blurbBlock[0]));
    }
    return out;
};

// roadmap.ts 의 SECTIONS 에서 대분류 제목·소개(en)를 얻는다.
const parseSectionMeta = () => {
    const src = readFileSync(join(root, "src/pages/algorithms/roadmap.ts"), "utf-8");
    const sectionsBlock = src.match(/SECTIONS[\s\S]*$/)[0];
    const out = new Map();
    for (const raw of sectionsBlock.split(/\n\s*\{\s*\n\s*key:/).slice(1)) {
        const key = raw.match(/^\s*"([a-z]+)"/)[1];
        const title = enOf(raw.match(/title:\s*\{[\s\S]*?\}/)[0]);
        const descBlock = raw.match(/desc:\s*\{[\s\S]*?\n\s*\}/);
        out.set(key, {title, desc: descBlock ? enOf(descBlock[0]) : ""});
    }
    return out;
};

// 소개 페이지 레지스트리(pages/sections)에서 key 와 본문 h2 제목(en)을 얻는다.
const parseIntros = (file) => {
    const src = readFileSync(join(root, file), "utf-8");
    return src
        .split(/\n\s*\{\s*\n\s*key:/)
        .slice(1)
        .filter((raw) => raw.includes("contents:"))
        .map((raw) => {
            const key = raw.match(/^\s*"([a-z]+)"/)[1];
            const sectionsBlock = raw.match(/sections:\s*\[([\s\S]*?)\]/);
            const sections = sectionsBlock
                ? [...sectionsBlock[1].matchAll(/\{en:\s*"((?:[^"\\]|\\.)*)"/g)].map((m) => m[1])
                : [];
            return {key, sections};
        });
};

const clamp = (t, max = 155) => (t.length <= max ? t : t.slice(0, max - 1).trimEnd() + "…");
const escAttr = (t) => t.replaceAll("&", "&amp;").replaceAll('"', "&quot;");
const escHtml = (t) => t.replaceAll("&", "&amp;").replaceAll("<", "&lt;");

const replaceTag = (html, pattern, replacement, what) => {
    if (!pattern.test(html)) throw new Error(`prerender: pattern not found for ${what}`);
    return html.replace(pattern, replacement);
};

const algos = parseAlgos();
const blurbs = parseBlurbs();
const sectionMeta = parseSectionMeta();
const template = readFileSync(join(root, "dist/index.html"), "utf-8");

// subpath: "algo/<slug>" | "section/<key>". 페이지별 메타를 박은 정적 셸을 쓴다.
const writeShell = ({subpath, pageTitle, desc, topics}) => {
    const urlEn = `${ORIGIN}${BASE}${subpath}/`;
    const urlKo = `${urlEn}?lang=ko`;
    const jsonld = {
        "@context": "https://schema.org",
        "@type": "TechArticle",
        headline: pageTitle,
        description: desc,
        inLanguage: "en",
        url: urlEn,
        isPartOf: {"@type": "WebSite", name: SITE, url: `${ORIGIN}${BASE}`},
        about: topics,
    };

    let html = template;
    html = replaceTag(html, /<title>[\s\S]*?<\/title>/, `<title>${escHtml(pageTitle)}</title>`, "title");
    html = replaceTag(html, /(<meta name="description"\s+content=")[^"]*(")/,
        `$1${escAttr(desc)}$2`, "description");
    html = replaceTag(html, /(<meta property="og:title" content=")[^"]*(")/,
        `$1${escAttr(pageTitle)}$2`, "og:title");
    html = replaceTag(html, /(<meta property="og:description"\s+content=")[^"]*(")/,
        `$1${escAttr(desc)}$2`, "og:description");
    html = replaceTag(html, /(<meta property="og:url" content=")[^"]*(")/,
        `$1${escAttr(urlEn)}$2`, "og:url");
    html = replaceTag(html, /(<meta name="twitter:title" content=")[^"]*(")/,
        `$1${escAttr(pageTitle)}$2`, "twitter:title");
    html = replaceTag(html, /(<meta name="twitter:description"\s+content=")[^"]*(")/,
        `$1${escAttr(desc)}$2`, "twitter:description");
    html = replaceTag(html, /(<link rel="canonical" href=")[^"]*(")/,
        `$1${escAttr(urlEn)}$2`, "canonical");
    html = replaceTag(html, /(<link rel="alternate" hreflang="en" href=")[^"]*(")/,
        `$1${escAttr(urlEn)}$2`, "hreflang en");
    html = replaceTag(html, /(<link rel="alternate" hreflang="ko" href=")[^"]*(")/,
        `$1${escAttr(urlKo)}$2`, "hreflang ko");
    html = replaceTag(html, /(<link rel="alternate" hreflang="x-default" href=")[^"]*(")/,
        `$1${escAttr(urlEn)}$2`, "hreflang x-default");
    html = replaceTag(html, /(<script id="page-jsonld" type="application\/ld\+json">)[\s\S]*?(<\/script>)/,
        `$1${JSON.stringify(jsonld)}$2`, "jsonld");

    const dir = join(root, "dist", subpath);
    mkdirSync(dir, {recursive: true});
    writeFileSync(join(dir, "index.html"), html);
};

for (const {slug, title, sections} of algos) {
    const blurb = blurbs.get(slug) ?? "";
    writeShell({
        subpath: `algo/${slug}`,
        pageTitle: `${title} · ${SITE}`,
        desc: clamp(`${blurb} Topics: ${sections.join(", ")}.`.trim()),
        topics: sections,
    });
}

const intros = parseIntros("src/pages/sections/index.ts");
for (const {key, sections} of intros) {
    const meta = sectionMeta.get(key) ?? {title: key, desc: ""};
    writeShell({
        subpath: `section/${key}`,
        pageTitle: `${meta.title} · ${SITE}`,
        desc: clamp(`${meta.desc} Topics: ${sections.join(", ")}.`),
        topics: sections,
    });
}

// 알 수 없는 경로도 SPA 로 부팅하도록 404 셸을 둔다 (GitHub Pages 관례).
cpSync(join(root, "dist/index.html"), join(root, "dist/404.html"));
console.log(`prerender: ${algos.length} algo + ${intros.length} section shells + 404.html`);
