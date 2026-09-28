import {GridMap, GridMapJson, parseGridMap} from "../grid";
import {resolvePath} from "../url";

// 라이브 sandbox가 맵 JSON을 가져온다 (data/maps/<name>.json — tools/web_export가
// PGM에서 내보낸 것). 알고리즘 실행 자체는 브라우저 엔진(libs/algorithms)이 한다 —
// recorded trace는 더 이상 페이지가 읽지 않고, check-engine-parity의 대조 자료로만 쓰인다.
export async function loadGridMap(path: string): Promise<GridMap> {
    const res = await fetch(resolvePath(path))
    if (!res.ok) throw new Error(`fetch failed: ${path} (${res.status})`)
    return parseGridMap(await res.json() as GridMapJson)
}
