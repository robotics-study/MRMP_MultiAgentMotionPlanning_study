// 웹 패널이 쓰는 occupancy grid 모델. 저장소 컨벤션과 동일하게 셀 인덱스는 (row, col)
// 이며 row 0이 이미지 최상단이다. world 좌표 변환(cellToWorld/worldToCellFloat)도
// 여기 있다 — 저장소의 좌표계 규칙처럼 좌표 프레임은 맵 레이어만 소유한다.
// freePoint/segmentFree는 python maps/occupancy_grid.py의 ContinuousSpace predicates와
// bit-identical한 부동소수 판정이다(연산 순서를 그대로 미러했다).
import {Cell, Point} from "./trace/types";
import {pointSegmentDistance, segmentsIntersect} from "./geometry";

export interface GridMap {
    name: string;
    width: number;    // cols
    height: number;   // rows
    // meters per cell — continuous planners와 world 좌표 렌더링이 쓴다.
    resolution: number;
    // world pose of the bottom-left pixel [x, y] (theta는 저장소에서도 버린다).
    origin: [number, number];
    // row-major 점유 여부. index = row * width + col.
    occupied: boolean[];
}

// tools/web_export가 만드는 맵 JSON: rows는 '#'(occupied)/'.'(free) 문자열 배열.
export interface GridMapJson {
    name: string;
    width: number;
    height: number;
    resolution: number;
    origin: [number, number];
    rows: string[];
}

export function parseGridMap(json: GridMapJson): GridMap {
    const occupied: boolean[] = new Array(json.width * json.height).fill(false)
    json.rows.forEach((row, r) => {
        for (let c = 0; c < json.width; c++) occupied[r * json.width + c] = row[c] === "#"
    })
    return {
        name: json.name, width: json.width, height: json.height,
        resolution: json.resolution, origin: json.origin, occupied,
    }
}

// world point → 셀 float 좌표 [row_f, col_f](셀 중심은 반 칸 오프셋). python
// cell_to_world의 역함수라 discrete 렌더링과 world 렌더링이 같은 픽셀에 착지한다:
// cell (r,c)의 중심 world 점은 여기에서 정확히 [r+0.5, c+0.5]로 돌아온다.
export const worldToCellFloat = (map: GridMap, p: Point): [number, number] =>
    [map.height - (p[1] - map.origin[1]) / map.resolution,
     (p[0] - map.origin[0]) / map.resolution]

// python OccupancyGrid2D.cell_to_world의 미러 — 같은 산술 순서.
export const cellToWorld = (map: GridMap, c: Cell): Point => [
    map.origin[0] + (c[1] + 0.5) * map.resolution,
    map.origin[1] + ((map.height - 1 - c[0]) + 0.5) * map.resolution,
]

// 한 셀의 world 직사각형 (x_lo, y_lo, x_hi, y_hi) — python _cell_rect의 미러.
const cellRect = (map: GridMap, row: number, col: number):
    [number, number, number, number] => [
    map.origin[0] + col * map.resolution,
    map.origin[1] + (map.height - 1 - row) * map.resolution,
    map.origin[0] + (col + 1) * map.resolution,
    map.origin[1] + (map.height - row) * map.resolution,
]

// python _point_rect_distance의 미러.
function pointRectDistance(p: Point, rect: [number, number, number, number]): number {
    const dx = Math.max(rect[0] - p[0], p[0] - rect[2], 0.0)
    const dy = Math.max(rect[1] - p[1], p[1] - rect[3], 0.0)
    return Math.sqrt(dx * dx + dy * dy)
}

// python _segment_rect_distance의 미러: 닫힌 교차면 0, 아니면 끝점-rect 거리와
// 코너-세그먼트 거리의 min (두 세그먼트가 disjoint면 최단 쌍은 항상 끝점을 지난다).
function segmentRectDistance(a: Point, b: Point, rect: [number, number, number, number]): number {
    const c0: Point = [rect[0], rect[1]]
    const c1: Point = [rect[2], rect[1]]
    const c2: Point = [rect[2], rect[3]]
    const c3: Point = [rect[0], rect[3]]
    const edges: Array<[Point, Point]> = [[c0, c1], [c1, c2], [c2, c3], [c3, c0]]
    for (const e of edges) {
        if (segmentsIntersect(a, b, e[0], e[1])) return 0.0
    }
    let best = Math.min(pointRectDistance(a, rect), pointRectDistance(b, rect))
    for (const corner of [c0, c1, c2, c3]) {
        const d = pointSegmentDistance(corner, a, b)
        if (d < best) best = d
    }
    return best
}

// python OccupancyGrid2D.free_point의 미러: disc가 obstacle 셀과 strict overlap하면
// blocked. row-major 순서와 dx lower-bound 스킵까지 같은 순서로 돈다.
export function freePoint(map: GridMap, q: Point, radius: number): boolean {
    for (let row = 0; row < map.height; row++) {
        for (let col = 0; col < map.width; col++) {
            if (!map.occupied[row * map.width + col]) continue
            const rect = cellRect(map, row, col)
            const dx = Math.max(rect[0] - q[0], q[0] - rect[2], 0.0)
            // dx > radius guarantees dist >= dx > radius: not blocked by this cell.
            if (dx > radius) continue
            const dy = Math.max(rect[1] - q[1], q[1] - rect[3], 0.0)
            const dist = Math.sqrt(dx * dx + dy * dy)
            if (dist < radius || dist === 0.0) return false
        }
    }
    return true
}

// python OccupancyGrid2D.segment_free의 미러 — swept disc 판정.
export function segmentFree(map: GridMap, a: Point, b: Point, radius: number): boolean {
    const minX = Math.min(a[0], b[0])
    const maxX = Math.max(a[0], b[0])
    const minY = Math.min(a[1], b[1])
    const maxY = Math.max(a[1], b[1])
    for (let row = 0; row < map.height; row++) {
        for (let col = 0; col < map.width; col++) {
            if (!map.occupied[row * map.width + col]) continue
            const rect = cellRect(map, row, col)
            // Lower bound on dist(segment, rect): 한 축의 분리만으로도 radius를 넘으면 스킵.
            if (Math.max(rect[0] - maxX, minX - rect[2], 0.0) > radius) continue
            if (Math.max(rect[1] - maxY, minY - rect[3], 0.0) > radius) continue
            const dist = segmentRectDistance(a, b, rect)
            if (dist < radius || dist === 0.0) return false
        }
    }
    return true
}
