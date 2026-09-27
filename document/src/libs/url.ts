// 배포(GitHub Pages)에서는 /MRMP-Multi-Agent-Motion-Planning--study 하위 경로로 서빙된다. dev에서는 루트.
// vite.config의 base 설정과 동일한 규칙 — 라우터 basename·정적 자산 경로가 이 값을 공유한다.
export const BASE_PATH = import.meta.env.PROD ? "/MRMP-Multi-Agent-Motion-Planning--study" : ""

export function resolvePath(path: string) {
    return `${BASE_PATH}/${path}`
}
