// spec/trace_schema.json의 TypeScript 대응. C++/Python 데모가 방출한 trace와
// 브라우저 라이브 엔진이 만드는 이벤트가 같은 타입을 공유한다 — 패널/플레이어 코드는 하나다.
// MRMP trace에는 wall-clock 시간이 없다: 재생은 seq 순서만으로 진행하고, 유일한
// 시간은 space-time 이벤트의 이산 시각 필드 t과, timed(kinodynamic) trace가 실어
// 나르는 schedule_found의 도착 시각뿐이다.
//
// 상태 페어의 해석은 coords가 선언한다(생략 = cell 기본값 — 기존 discrete trace 바이트
// 불변). world는 연속 알고리즘(dRRT 계열)의 world point이고, radius는 그 disc 반지름이다.
export type TraceEventType =
    | "planning_started"
    | "roadmap_built"
    | "node_expanded"
    | "path_found"
    | "schedule_found"
    | "conflict_found"
    | "constraint_added"
    | "planning_finished";

// 상태 페어는 항상 숫자 쌍: discrete는 [row, col](int), continuous는 world point(float).
export type Cell = [number, number];
export type Point = [number, number];

export interface TraceEvent {
    seq: number;
    event: TraceEventType;
    // node_expanded: 확장한 노드의 상태. agent가 있으면 개별 탐색(페어 하나), 없으면
    // joint state — 모든 agent의 상태를 coords 해석의 페어로 평면화한 것.
    state?: number[];
    // node_expanded 선택: 그 노드의 g-value.
    cost?: number;
    // 사건이 속한 agent 인덱스 (path_found/roadmap_built/constraint_added는 필수, joint state 확장 없음).
    agent?: number;
    // space-time 이벤트의 이산 시각 (스텝).
    t?: number;
    // path_found 전용: 그 agent의 space-time 경로. path[t]가 시각 t에 차지하는 상태
    // (discrete: 셀, continuous: world point — 인접 웨이포인트 사이는 직선 이동).
    path?: Point[];
    // roadmap_built 전용(연속 trace): 개별 roadmap의 정점(삽입 순서 그대로, [start, goal]
    // 이 먼저)와 정점 인덱스 쌍 [i, j] (i < j, (min,max) 순).
    vertices?: Point[];
    edges?: Array<[number, number]>;
    // schedule_found 전용(timed trace): wait이 제거된 route(cells, 연속 셀은 인접)와
    // 각 retained 위치의 earliest arrival 시각(times). 실행 재생은 uniform velocity
    // model — departure(D_i = times[i+1] − 1/vmax)까지 셀에 체류하고 그 다음에만 이동.
    cells?: number[][];
    times?: number[];
    // planning_started 전용.
    algorithm?: string;
    map?: string;
    params?: Record<string, unknown>;
    // 상태 페어의 해석 선언 — 생략이면 cell(이산 기본값), world는 연속 planner의 점.
    coords?: "cell" | "world";
    // planning_started 선택(연속 trace 전용): agent index 순서대로의 disc 반지름(미터).
    radius?: number[];
    // planning_started 선택(timed trace 전용): agent별 속도 한계(칸/시간). vmax의 존재
    // 자체가 timed 선언이다 — schedule_found가 이 값으로 실행을 재생한다.
    vmax?: number[];
    // conflict_found/constraint_added 전용: vertex = 같은 셀 동시 점유, edge = 두 agent가
    // 간선 양끝을 맞바꿈 (cell/to가 양끝, 방향은 인코딩되지 않는다).
    kind?: "vertex" | "edge";
    cell?: number[];
    to?: number[];
    agents?: [number, number];
    success?: boolean;
    metrics?: Record<string, number>;
}
