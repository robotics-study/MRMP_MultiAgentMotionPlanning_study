// 웹 패널이 쓰는 occupancy grid 모델. 저장소 컨벤션과 동일하게 셀 인덱스는 (row, col)
// 이며 row 0이 이미지 최상단이다 (패널은 셀 좌표만 그린다 — world 좌표 변환은 없다).
export interface GridMap {
    name: string;
    width: number;    // cols
    height: number;   // rows
    // row-major 점유 여부. index = row * width + col.
    occupied: boolean[];
}

// tools/web_export가 만드는 맵 JSON: rows는 '#'(occupied)/'.'(free) 문자열 배열.
export interface GridMapJson {
    name: string;
    width: number;
    height: number;
    rows: string[];
}

export function parseGridMap(json: GridMapJson): GridMap {
    const occupied: boolean[] = new Array(json.width * json.height).fill(false)
    json.rows.forEach((row, r) => {
        for (let c = 0; c < json.width; c++) occupied[r * json.width + c] = row[c] === "#"
    })
    return {name: json.name, width: json.width, height: json.height, occupied}
}
