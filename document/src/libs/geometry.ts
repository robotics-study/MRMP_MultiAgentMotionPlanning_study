// python/mrmp/core/geometry.py의 정확한 미러. 모든 함수는 고정 IEEE-754 double
// 산술식이고 연산 순서가 계약의 일부다 — Python/C++과 bit-identical 판정.
// 충돌 semantics: obstacle 셀은 닫힌 정사각형이고 disc는 거리가 radius보다 엄격히
// 작을 때 blocked(접촉은 free). 두 robot disc는 중심 거리가 반지름 합보다 엄격히
// 작을 때 충돌한다.
import {Point} from "./trace/types";

// 점 p에서 세그먼트 a→b까지의 거리 — clamp한 t로 closest point를 재구성하는 것까지
// 계약의 일부다(python point_segment_distance와 동일 순서).
export function pointSegmentDistance(p: Point, a: Point, b: Point): number {
    const dx = b[0] - a[0]
    const dy = b[1] - a[1]
    const wx = p[0] - a[0]
    const wy = p[1] - a[1]
    const len2 = dx * dx + dy * dy
    let t: number
    if (len2 === 0.0) {
        t = 0.0
    } else {
        t = (wx * dx + wy * dy) / len2
        if (t < 0.0) t = 0.0
        else if (t > 1.0) t = 1.0
    }
    const cx = a[0] + t * dx
    const cy = a[1] + t * dy
    const ex = p[0] - cx
    const ey = p[1] - cy
    return Math.sqrt(ex * ex + ey * ey)
}

// (b-a) x (c-a) — 부호가 a→b→c의 orientation.
function orient(a: Point, b: Point, c: Point): number {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

// p는 a→b와 collinear(호출자가 orient == 0 확인): 세그먼트 위에 있는가?
function onSegment(a: Point, b: Point, p: Point): boolean {
    return Math.min(a[0], b[0]) <= p[0] && p[0] <= Math.max(a[0], b[0])
        && Math.min(a[1], b[1]) <= p[1] && p[1] <= Math.max(a[1], b[1])
}

// 닫힌 세그먼트 교차 판정(접촉 포함). collinear overlap은 on-segment 케이스로 떨어진다.
export function segmentsIntersect(a: Point, b: Point, c: Point, d: Point): boolean {
    const o1 = orient(a, b, c)
    const o2 = orient(a, b, d)
    const o3 = orient(c, d, a)
    const o4 = orient(c, d, b)
    if (o1 === 0.0 && onSegment(a, b, c)) return true
    if (o2 === 0.0 && onSegment(a, b, d)) return true
    if (o3 === 0.0 && onSegment(c, d, a)) return true
    if (o4 === 0.0 && onSegment(c, d, b)) return true
    return (o1 > 0.0) !== (o2 > 0.0) && (o3 > 0.0) !== (o4 > 0.0)
}

// robot 1이 a1→b1을, 동시에 robot 2가 a2→b2를 같은 단위 시간 동안 같은 속도로
// 미끄러질 때의 최소 중심 간 거리 — 상대 운동 w + t·d 자체가 세그먼트라 원점에서
// 그 세그먼트까지의 거리일 뿐(시간 이산화 없음).
export function movingPairDistance(a1: Point, b1: Point, a2: Point, b2: Point): number {
    const wx = a1[0] - a2[0]
    const wy = a1[1] - a2[1]
    const dx = (b1[0] - a1[0]) - (b2[0] - a2[0])
    const dy = (b1[1] - a1[1]) - (b2[1] - a2[1])
    return pointSegmentDistance([0.0, 0.0], [wx, wy], [wx + dx, wy + dy])
}
