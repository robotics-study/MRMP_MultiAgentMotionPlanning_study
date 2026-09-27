import {TraceEvent} from "./types";
import {GridMap, GridMapJson, parseGridMap} from "../grid";
import {resolvePath} from "../url";

// 데모 trace는 용량 때문에 gzip(.jsonl.gz)으로 커밋된다. GitHub Pages는 원본 바이트를
// 그대로 주지만, vite dev 서버는 Content-Encoding: gzip으로 서빙해 브라우저가 이미 해제한
// 본문을 준다. 확장자 대신 gzip magic byte(1f 8b)로 판별해 두 환경 모두에서 동작하게 한다.
async function fetchText(url: string): Promise<string> {
    const res = await fetch(url)
    if (!res.ok) throw new Error(`fetch failed: ${url} (${res.status})`)
    const buf = await res.arrayBuffer()
    const head = new Uint8Array(buf, 0, Math.min(2, buf.byteLength))
    if (head.length === 2 && head[0] === 0x1f && head[1] === 0x8b) {
        const stream = new Blob([buf]).stream().pipeThrough(new DecompressionStream("gzip"))
        return new Response(stream).text()
    }
    return new TextDecoder().decode(buf)
}

export async function loadTrace(path: string): Promise<TraceEvent[]> {
    const text = await fetchText(resolvePath(path))
    return text
        .split("\n")
        .filter((line) => line.trim().length > 0)
        .map((line) => JSON.parse(line) as TraceEvent)
}

export async function loadGridMap(path: string): Promise<GridMap> {
    const res = await fetch(resolvePath(path))
    if (!res.ok) throw new Error(`fetch failed: ${path} (${res.status})`)
    return parseGridMap(await res.json() as GridMapJson)
}
