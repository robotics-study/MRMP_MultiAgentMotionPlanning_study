# 맵 파일 포맷

언어 공용 계약. C++/Python 의 `maps/` 로더는 이 문서를 기준으로 구현하며, 포맷 변경은 이 문서를 먼저 갱신한 뒤 양 언어에 반영한다.

공통 규칙:
- 모든 맵 파일은 최상위에 `type` 필드를 가진다 (`occupancy_grid` 만 지원). 로더는 이 필드로 맵 타입을 판별한다 (확장자/디렉토리에 의존하지 않는다).
- world 좌표는 미터 단위 float `(x, y)`. grid 인덱스는 `(row, col)` int이며 `row 0 = 이미지 최상단`.
- 경로 참조(이미지 등)는 맵 파일 기준 상대 경로.

## occupancy_grid (`maps/grid/`)

ROS map_server 스타일. yaml + 그레이스케일 이미지(pgm).

```yaml
type: occupancy_grid
image: maze01.pgm          # 0(검정)=occupied, 255(흰색)=free
resolution: 0.5            # meters / pixel
origin: [0.0, 0.0, 0.0]    # 이미지 좌하단 픽셀의 world pose [x, y, theta]
occupied_thresh: 0.65      # 시그니처 호환용 (판정에는 free_thresh 만 쓴다)
free_thresh: 0.196         # (1 - pixel/255) <= 이 값 → free (사이 값은 unknown = 통행 불가 취급)
```

- capability: `DiscreteSpace` — 이동은 4-connected + wait(self-loop), 모든 action 비용 1 스텝.
  대각선 이동은 없으므로 corner-cut 규칙도 없다. 휴리스틱은 Manhattan.

## scenario (`maps/scenarios/`)

맵 위에서 실행할 **다중 agent** 문제 정의. 단일 start/goal 이 아니라 `agents:` 목록을 쓴다 —
`agents:` 없는 시나리오(단일 로봇 형식)는 이 저장소의 로더가 거부한다.

```yaml
map: ../grid/maze01.yaml   # scenario 파일 기준 상대 경로
agents:                    # 목록 순서가 agent index (0부터) — trace 의 agent 필드와 일치
  - start: [0.75, 1.25]    # world 좌표. 데모 드라이버가 grid.world_to_cell 로 셀로 변환한다
    goal: [8.25, 7.25]
  - start: [8.25, 1.25]
    goal: [0.75, 7.25]
```

- agent 수에 상한은 없지만, joint-space 탐색의 상태 공간은 agent 수에 대해 지수로 커진다 —
  시나리오는 보통 2~3명.
