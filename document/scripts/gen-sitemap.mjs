// 빌드 전에 public/sitemap.xml 을 생성한다. 페이지 목록은 pages/algorithms/index.ts 와
// pages/sections/index.ts 에서 읽는다: 집필된 페이지는 멀티라인 리터럴(`{\n slug: ...` +
// contents:)로 쓰는 것이 레지스트리 컨벤션이라, 그 블록만 잡으면 미집필 한 줄 항목은
// 자연히 제외된다. 알고리즘 페이지가 하나도 없어도(초기 스캐폴드) 홈+섹션으로 생성한다.
// 각 URL 에 en/ko hreflang 대체 링크를 함께 적는다.
import {readFileSync, writeFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const ORIGIN = "https://robotics-study.github.io";
const BASE = "/MRMP-Multi-Agent-Motion-Planning--study/";

const indexTs = readFileSync(join(root, "src/pages/algorithms/index.ts"), "utf-8");
const slugs = indexTs
    .split(/\n\s*\{\s*\n\s*slug:/)
    .slice(1)
    .filter((block) => block.includes("contents:"))
    .map((block) => block.match(/^\s*"([a-z0-9_]+)"/)[1]);

const introKeys = (file) => readFileSync(join(root, file), "utf-8")
    .split(/\n\s*\{\s*\n\s*key:/)
    .slice(1)
    .filter((block) => block.includes("contents:"))
    .map((block) => block.match(/^\s*"([a-z]+)"/)[1]);
const sectionKeys = introKeys("src/pages/sections/index.ts");

// subpath: "algo/<slug>" | "section/<key>" | undefined(홈)
const url = (lang, subpath) => {
    const path = subpath !== undefined ? `${subpath}/` : "";
    const qs = lang === "ko" ? "?lang=ko" : "";
    return `${ORIGIN}${BASE}${path}${qs}`;
};

const esc = (s) => s.replaceAll("&", "&amp;");
const today = new Date().toISOString().slice(0, 10);

const entry = (lang, subpath, priority) => `  <url>
    <loc>${esc(url(lang, subpath))}</loc>
    <lastmod>${today}</lastmod>
    <priority>${priority}</priority>
    <xhtml:link rel="alternate" hreflang="en" href="${esc(url("en", subpath))}"/>
    <xhtml:link rel="alternate" hreflang="ko" href="${esc(url("ko", subpath))}"/>
    <xhtml:link rel="alternate" hreflang="x-default" href="${esc(url("en", subpath))}"/>
  </url>`;

const entries = [
    entry("en", undefined, "1.0"),
    entry("ko", undefined, "1.0"),
    ...sectionKeys.flatMap((k) => [entry("en", `section/${k}`, "0.9"), entry("ko", `section/${k}`, "0.9")]),
    ...slugs.flatMap((s) => [entry("en", `algo/${s}`, "0.8"), entry("ko", `algo/${s}`, "0.8")]),
];

const xml = `<?xml version="1.0" encoding="UTF-8"?>
<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9"
        xmlns:xhtml="http://www.w3.org/1999/xhtml">
${entries.join("\n")}
</urlset>
`;

writeFileSync(join(root, "public/sitemap.xml"), xml);
console.log(`sitemap.xml: ${entries.length} URLs (sections: ${sectionKeys.join(", ")} / pages: ${slugs.join(", ") || "-"})`);
