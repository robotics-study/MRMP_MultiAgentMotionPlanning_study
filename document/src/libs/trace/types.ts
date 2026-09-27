// spec/trace_schema.json의 TypeScript 대응. C++/Python 데모가 방출한 trace와
// 브라우저 라이브 엔진이 만드는 이벤트가 같은 타입을 공유한다 — 패널/플레이어 코드는 하나다.
// MRMP trace에는 wall-clock 시간이 없다: 재생은 seq 순서만으로 진행하고, 유일한
// 시간은 space-time 이벤트의 이산 시각 필드 t다.
export type TraceEventType =
    | "planning_started"
    | "node_expanded"
    | "path_found"
    | "conflict_found"
    | "constraint_added"
    | "planning_finished";

// 격자 셀 (row, col) — row 0이 이미지 최상단.
export type Cell = [number, number];

export interface TraceEvent {
    seq: number;
    event: TraceEventType;
    // node_expanded: 확장한 노드의 상태. agent가 있으면 개별 탐색(셀 하나), 없으면
    // joint state — 모든 agent의 셀을 [r0,c0,r1,c1,...]로 평면화한 것.
    state?: number[];
    // node_expanded 선택: 그 노드의 g-value.
    cost?: number;
    // 사건이 속한 agent 인덱스 (path_found/constraint_added는 필수, joint state 확장 없음).
    agent?: number;
    // space-time 이벤트의 이산 시각 (스텝).
    t?: number;
    // path_found 전용: 그 agent의 space-time 경로. path[t]가 시각 t에 차지하는 셀.
    path?: Cell[];
    // planning_started 전용.
    algorithm?: string;
    map?: string;
    params?: Record<string, unknown>;
    // conflict_found/constraint_added 전용: vertex = 같은 셀 동시 점유, edge = 두 agent가
    // 간선 양끝을 맞바꿈 (cell/to가 양끝, 방향은 인코딩되지 않는다).
    kind?: "vertex" | "edge";
    cell?: number[];
    to?: number[];
    agents?: [number, number];
    success?: boolean;
    metrics?: Record<string, number>;
}
