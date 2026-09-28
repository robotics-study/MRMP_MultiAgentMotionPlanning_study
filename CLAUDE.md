# mrmp

Multi-robot planning 알고리즘 구현체 + demo 모음 — 계보의 두 갈래(search-based MAPF, sampling-based MRMP)를 C++ / Python 독립 이중 구현으로.

## 프로젝트 개요

**여러 로봇의 조율(multi-agent motion planning)** 알고리즘만 다룬다. 계보의 1차 분류 축은 planner 타입(survey: Bui et al. 2023): search 기반은 모든 agent가 하나의 shared grid 위에서 이산 상태를 열거해 시공간(space-time) 경로를 찾고, sampling 기반은 연속 configuration space에서 표본 채취로 계획한다. 단일 로봇 navigation 알고리즘은 이 저장소의 범위가 아니다 — 자매 저장소 [nav_study](https://github.com/robotics-study/navigation_basic) 에서 다룬다.

| 섹션 (= 코드 디렉토리) | 알고리즘 (⏳ = planned) | 베이스 클래스 |
|---|---|---|
| `search` | Prioritized A*, Push and Swap ⏳, Push and Rotate ⏳, Joint-space A*, CBS | `MultiAgentPlanner` |
| `sampling` | MA-RRT*, sRRT, dRRT ⏳, dRRT* ⏳ | `MultiAgentPlanner` — MA-RRT*는 논문의 자체 이산화(G-RRT*)로, sRRT는 individual policy(BFS tree)가 격자에서 정확히 구성되므로 `DiscreteSpace` 위에서 구현; 연속 공간 capability은 dRRT wave와 함께 |

계보 순서: 각 갈래 안에서 결합 축을 따라 decoupled/priority → coupled → hybrid. search 갈래는 Prioritized A*(Erdmann & Lozano-Pérez 1987) → Joint-space A*(모든 것의 baseline) → CBS(Sharon et al. 2015)로 집필·구현 완료. priority 갈래의 완성인 decentralized 계열 — Push and Swap(Luna & Bekris, IJCAI 2011) → Push and Rotate(de Wilde, ter Mors & Witteveen, JAIR 2014) — 은 planned(예약 동결 대신 push/swap primitive로 치우며, component당 빈 셀 ≥2이면 완전하다고 주장 — 그 주장을 검증·보완하는 것이 Push and Rotate). sampling 갈래는 MA-RRT*(Čáp et al. 2013, coupled — 논문 자체의 이산화 G-RRT*로 DiscreteSpace 위에서 구현 완료) → sRRT(Wagner, Kang & Choset 2012, subdimensional — individual policy + collision set, 같은 DiscreteSpace 위에서 구현 완료) → dRRT(Solovey, Salzman & Halperin 2016) ⏳ → dRRT*(Dobson et al. 2017) ⏳ 순서. 새 알고리즘도 이 계보 위치에 끼워 넣는다.

모든 알고리즘은 추상 클래스 기반으로 다음 세 가지가 자동으로 성립해야 한다:
1. **Performance estimate** — 공통 metric(sum_of_costs, makespan, expanded nodes, success)을 benchmark runner가 수집.
2. **Step-by-step visualization** — 알고리즘이 방출하는 trace 이벤트(공용 JSON 포맷)를 시각화 도구가 재생.
3. **Demo** — 맵 + 시나리오 + 파라미터 파일만 지정하면 실행되는 데모.

## 저장소 구조

```
.
├── CLAUDE.md
├── spec/                        # 언어 공용 계약 (구현보다 우선하는 single source of truth)
│   ├── trace_schema.json        #   step-by-step trace 이벤트 JSON Schema
│   ├── param_schema.json        #   알고리즘 파라미터 선언(name/type/range/default) 스키마
│   └── map_formats.md           #   맵 파일 포맷 정의 (occupancy grid pgm/yaml + scenario)
├── maps/                        # 공용 벤치마크 맵 데이터
│   ├── grid/                    #   occupancy grid (ROS 스타일 yaml + pgm)
│   └── scenarios/               #   agents(start/goal world 좌표) 시나리오 (yaml, 맵 참조)
├── configs/<section>/           # 알고리즘별 파라미터 yaml (언어 공용) — section ∈ {search, sampling}
├── cpp/
│   ├── CMakeLists.txt
│   ├── include/mrmp/
│   │   ├── core/                # planner.hpp, params.hpp, trace.hpp, types.hpp, capabilities.hpp
│   │   ├── maps/                # occupancy_grid.hpp, pgm.hpp, loader.hpp
│   │   ├── search/              #   알고리즘 헤더 (사이트 섹션과 1:1)
│   │   └── sampling/            #   MA-RRT* 등 — 사이트 섹션과 1:1
│   ├── src/                     # include/와 동일 구조의 구현
│   ├── demos/                   # demo_<algo>.cpp — 실행 시 trace 파일 출력
│   └── tests/                   # GoogleTest
├── python/
│   ├── pyproject.toml
│   ├── mrmp/
│   │   ├── core/                # planner.py, params.py, trace.py, types.py, capabilities.py
│   │   ├── maps/                # cpp include/mrmp/maps/ 와 1:1 미러
│   │   ├── search/              #   알고리즘 모듈 (사이트 섹션과 1:1)
│   │   └── sampling/            #   MA-RRT* 등 — 사이트 섹션과 1:1
│   ├── demos/                   # demo_<algo>.py — demo_common.run(name, factory) 조립만
│   └── tests/                   # pytest
└── tools/                       # Python. mrmp 패키지에 의존 (설치 후 사용)
    ├── viz/                     # trace 재생기: replay.py (matplotlib step-by-step / 애니메이션 저장)
    ├── bench/                   # matrix runner: (scenario × algorithm) 조합 실행 + 리포트
    └── web_export/              # 문서 사이트용 맵 JSON + gzip trace 내보내기
```

## 아키텍처 원칙

### 의존 방향 (위반은 리뷰 Critical)
- `core` 는 stdlib(+ Eigen / numpy)만 의존한다. 알고리즘·맵 모듈을 알지 못한다.
- `maps` 는 `core` 만 의존한다.
- 알고리즘 모듈(`search`, 이후 `sampling`)은 `core` 의 추상 인터페이스에만 의존한다. **구체 맵 클래스 직접 참조 금지.**
- `tools/viz`, `tools/bench`, `tools/web_export` 는 trace/param/map 포맷(spec)과 `core`/`maps` 로더에만 의존한다. 알고리즘 내부 상태 접근 금지 — 시각화에 필요한 모든 정보는 trace 이벤트로 방출되어야 한다.
- `demos` 는 최상위 조립 계층: 알고리즘 + maps + configs 를 묶기만 한다. 로직 금지.

### 언어 미러링
- C++과 Python은 **같은 설계를 각자 idiomatic 하게** 구현한다. 클래스/메서드 개념 이름, 파라미터 이름, trace 이벤트는 동일해야 한다 (표기만 언어 컨벤션).
- 알고리즘 추가/변경은 원칙적으로 두 언어 동시 반영. 한쪽만 구현된 상태는 README parity 표에 명시하고 남겨두지 않는 것을 원칙으로 한다.
- 언어 간 공유물(trace schema, param yaml, map 데이터, 시나리오)은 반드시 `spec/`, `configs/`, `maps/` 에 두고 양쪽에서 로드한다. 언어 디렉토리 안에 복제 금지.

### 맵 추상화 — capability 모델
알고리즘은 구체 맵 타입이 아니라 **capability 인터페이스**를 요구한다. MRMP는 현재 capability가 하나뿐이다:

| capability | 핵심 메서드 | 요구 알고리즘 |
|---|---|---|
| `DiscreteSpace` | `neighbors(state) -> [(state, cost)]`, `heuristic(a, b)`, `cells()` | Prioritized A*, Joint-space A*, CBS, MA-RRT* |

- `neighbors`는 4-connected 이동 + wait(self-loop)를 **고정 순서**(up, down, left, right, wait)로 반환하고 모든 action 비용은 1.0 — g-value가 곧 경과 시각이고 언어 간 tie-breaking을 동일하게 유지해야 한다.
- `heuristic`은 Manhattan (단위 비용 4-connected에 admissible + consistent).
- Planner 는 `required_capabilities()` 를 선언하고, demo/bench 는 실행 전 `map.supports(capability)` 로 호환성을 검사한다.
- 새 맵 타입 추가 시 기존 알고리즘 코드는 수정되지 않아야 한다 (OCP).

### 파라미터 추상화
- 각 알고리즘은 자신의 `ParamSet` 을 선언한다: 파라미터 이름, 타입, 기본값, 유효 범위/제약. 선언 형식은 `spec/param_schema.json` 을 따른다.
- 값은 `configs/search/<algorithm>.yaml` 에서 로드하고 로드 시점에 선언 기반 검증(범위 밖 → 에러)을 수행한다. 코드에 매직 넘버로 파라미터를 심지 않는다.
- 같은 yaml 을 C++/Python 양쪽이 그대로 읽는다.

### Trace (step-by-step 시각화의 계약)
- trace 는 JSON Lines 파일. 한 줄이 이벤트 하나이고 `seq` 순서가 내러티브다 — wall-clock 시간은 담지 않는다. 유일한 시간 필드는 space-time 이벤트의 이산 시각 `t`.
- 이벤트: `planning_started`(algorithm/map/params 스냅샷, demo가 방출), `node_expanded`(state = 셀 또는 평면화 joint state `[r0,c0,r1,c1,...]`, agent 필드가 있으면 개별 탐색 노드), `path_found`(agent 필수), `conflict_found`(vertex/edge), `constraint_added`, `planning_finished`(success/metrics).
- float 직렬화는 byte-equality가 아니라 **parse 후 수치 equality** 계약 (Python `5.0` vs C++ `5` 는 같은 값).
- trace 방출은 demo·viz 시 on. hot loop 에서 recorder 가 null 이면 zero-cost 여야 한다.
- **데모 산출물 형식 (룰)**: 모든 알고리즘의 demo trace 는 `replay.py` 로 (1) 애니메이션 **GIF** (`--gif`, 탐색 진행 + 실행 재생) 와 (2) 탐색 중간 과정 **PNG 스냅샷** 세트 (`--snapshots`, 진행률 균등 분할) 로 렌더링 가능해야 한다. 산출물은 두 언어 데모 각각에 대해 `out/viz/<algo>/py/`, `out/viz/<algo>/cpp/` 아래에 둔다 (`out/` 은 gitignore — 커밋하지 않는다).

### Benchmark
- `tools/bench/run_matrix.py` 는 (scenario × algorithm) 조합을 실행하고 metric 을 수집한다: success, sum_of_costs, makespan, expanded_nodes. 알고리즘 열은 `configs/search/<algo>.yaml` + `python/demos/demo_<algo>.py` 가 둘 다 존재할 때 발견된다.
- C++ demo 는 같은 CLI 인자로 같은 trace 를 출력하므로 언어 비교가 가능하다.

## 빌드 / 테스트 / 실행

```bash
# C++ (C++20, CMake ≥ 3.20, GoogleTest)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build

# Python (≥ 3.10)
cd python && pip install -e ".[dev,viz]" && cd ..
PYTHONPATH=$PWD/python .venv/bin/python -m pytest python/tests -q
PYTHONPATH=$PWD/python .venv/bin/python -m ruff check python tools
PYTHONPATH=$PWD/python .venv/bin/python -m mypy python/mrmp tools

# Demo (예시 — 두 언어가 동일한 인자 형태를 갖는다)
python python/demos/demo_prioritized_astar.py --map maps/grid/maze01.yaml \
    --scenario maps/scenarios/maze01_two.yaml \
    --params configs/search/prioritized_astar.yaml --trace out/trace.jsonl
./cpp/build/demos/demo_prioritized_astar --map ... --scenario ... \
    --params ... --trace out/trace.cpp.jsonl

# 시각화 / 벤치마크 / 웹 내보내기
python tools/viz/replay.py out/trace.jsonl                                        # interactive 재생
python tools/viz/replay.py out/trace.jsonl --gif out/viz/x.gif --snapshots out/snaps/
python tools/bench/run_matrix.py --out out/report.md
PYTHONPATH=$PWD/python python tools/web_export/export_web_assets.py \
    --maps maze01,open01 --scenario maze01_two --algos prioritized_astar          # 사이트용 자산
```

## 새 알고리즘 추가 체크리스트

1. `configs/search/<algo>.yaml` 에 파라미터 선언 + 기본값 작성.
2. `MultiAgentPlanner` 를 상속해 C++/Python 양쪽 구현 (`required_capabilities()` 선언 포함). 계보 순서 유지.
3. 탐색 단계마다 trace 이벤트 방출. 새 이벤트 타입이 필요하면 `spec/trace_schema.json` 먼저 갱신.
4. 두 언어 각각 demo 추가 (`demo_common.run(name, factory)` 패턴 — 조립만, 로직 금지).
5. 단위 테스트: 최소 (a) 알려진 시나리오에서 최적/유효 경로 검증 (sum_of_costs/makespan 기준값), (b) 충돌 없는 실행 보장, (c) 경로 없음 케이스, (d) 파라미터 검증 실패 케이스. Python과 C++ 테스트가 같은 계약을 검증한다.
6. `tools/bench/run_matrix.py` 매트릭스에서 전 시나리오 1회 실행 확인.
7. demo trace 를 `replay.py --gif` / `--snapshots` 로 렌더링해 GIF 애니메이션 + 중간 과정 PNG 가 정상 생성되는지 확인.
8. `export_web_assets.py` 로 사이트 자산 갱신 (`document/public/data/`). README parity 표 갱신.

새 **맵 타입** 추가 시: `spec/map_formats.md` 에 포맷 정의 → 양 언어 `maps/` 에 로더 + capability adapter 구현 → `maps/` 에 샘플 데이터 → capability 매트릭스 표 갱신. 알고리즘 코드는 수정하지 않는다.

## 코딩 컨벤션

- **C++**: C++20. 헤더는 `include/mrmp/`, 구현은 `src/` 동일 경로. 네임스페이스 `mrmp::<module>`. 소유권은 `unique_ptr`/값 타입 우선, raw new/delete 금지. 예외는 로드/검증 단계에서만, planning hot path 에서는 사용하지 않는다.
- **Python**: 전 함수 type hint 필수. `numpy` 기반 좌표 연산. 추상 클래스는 `abc.ABC`. `any`/무타입 dict 전달 금지 — 파라미터는 `ParamSet`, 상태는 `types.py` 의 dataclass 를 쓴다.
- 좌표계: world 좌표 (x, y) 는 float, grid 인덱스는 (row, col) int — 변환은 맵 클래스만 담당한다. agent index 는 리스트 위치이며 우선순위와 무관하다 (우선순위는 알고리즘 파라미터). 이 구분을 흐리는 코드 금지.
- 주석은 WHY 만. 알고리즘 수식/휴리스틱 선택 근거는 논문 인용(저자, 연도)으로 남긴다.

## 문서 사이트 (document/)

React 18 + Vite + TS + Tailwind SPA. 2D 는 Konva, 수식은 KaTeX, 이중언어는 `<T en ko>`. 사이트 섹션(`search` / `sampling`)은 저장소 코드 디렉토리와 1:1 미러 — 알고리즘 페이지는 `pages/algorithms/<section>/<slug>.tsx`, 카드·사이드바 그룹핑도 같은 섹션 키를 쓴다. 빌드/검증: `cd document && yarn build`, dev 서버 `yarn dev`.

### 알고리즘 페이지 규칙 (순서 고정)

인트로 → 개념/유도 → **Properties and Complexity** → **The Algorithm** → 증명(collapsible) → (반례 등 이론 보조) → **Demo** → **Implementation** → **References**. registry `sections[]` 도 같은 순서로.

- **알고리즘 배치는 항상 계보순**: registry 배열(= 사이드바·pager·홈 카드 순서)은 decoupled(Prioritized A*) → coupled(Joint-space A*) → hybrid(CBS). 새 알고리즘도 자기 계보 위치에 끼워 넣는다 (끝에 append 금지).
- **The Algorithm**: 자료구조·루프 요약 문단 → `Pseudocode` 블록(`# 1~n` 스텝 마커) → 바로 아래 "1. ~한다" 번호 목록으로 각 스텝의 무엇/왜 해설 (vertex/edge conflict 판정 시점 같은 함정 포함).
- **증명**: 산문 서술 금지. 가정 → BlockMath 부등식 체인 → 모순/결론의 단계형.
- **수식 항 설명 필수 (`Terms` 컴포넌트)**: 모든 display 수식(BlockMath) 바로 아래에 `components/math/Terms`로 기호별 설명을 붙인다. **모든 기호를 그 자리에서 정의한다** — 이전 페이지에서 정의한 기호도 다시 적어, 독자가 페이지를 왔다 갔다 하지 않게 한다.
- **Parameters 섹션 금지** — 웹은 알고리즘 설명이지 코드 문서가 아니다. parameter 개념은 이론 산문에서 다룬다.
- **Demo**: 라이브 sandbox(`components/panels/Sandbox.tsx`) — 페이지가 모듈 상수 `runLive`(libs/algorithms 의 TS 엔진 = Python 구현의 정확한 미러)와 preset 시나리오를 넘기면, 브라우저에서 직접 실행하고 벽 페인팅·endpoint 드래그·agent 추가마다 재계획한다. 재생은 TracePlayer(탐색 phase 스크러버 + 실행 phase τ 재생), 배속 버튼 없이 고정. TS 엔진은 `scripts/check-engine-parity.mjs` 에 등록해 python trace 와 필드 단위 parity 를 매 빌드 검증하고, 수출 trace(`<algo>/<scenario>.jsonl.gz`)는 parity 의 기준 자료로만 쓰인다 (페이지는 더 이상 trace 를 읽지 않는다).
- **Implementation**: 실제 저장소 소스를 vite `?raw` 로 embed (사본 금지), python/c++ 탭 토글 + 파일별 GitHub 링크.
- **References**: 실제 논문 링크(DOI) 필수.
- **시각 자료 적극 배치**: 페이지·소개마다 Konva figure (CanvasFigure 래핑, 테마 색은 useCanvasColors). agent 색상은 `AGENT_COLORS`, 충돌 색은 `CONFLICT_COLOR` (모두 `libs/trace/timeline.ts` 정의 — replay.py 와 공유). 데이터 표는 가운데 정렬(전역 CSS 처리됨).

### 한국어 작문 규칙

- 기술 용어 과잉 번역 금지: 헤딩·UI 는 Demo / References 처럼 영어 유지. "인터랙티브 데모" 같은 음차 금지.
- **직역 금지**: 한국어는 영어 번역이 아니라 같은 내용을 한국어로 새로 쓴다. 번역투 표현을 자연스러운 표현으로.
- 한국어 산문에서 em-dash 삽입구("A — B — C") 금지. 문장 분리나 쉼표/괄호로 재구성.
- **조사는 선행 영어 토큰에 붙인다**: "CBS 는" ❌ → "CBS는" ✅, "frontier 를" ❌ → "frontier를" ✅. 수식 컴포넌트(`<InlineMath/>`) 뒤 조사도 동일.
- 세미콜론 문장 연결("~한다; ~한다") 금지 — 마침표로 분리.

### PR 워크플로우

- **머지된 브랜치에 후속 커밋을 push 하지 않는다.** push 전에 해당 브랜치 PR 상태를 확인하고, 이미 머지됐으면 main 에서 새 브랜치를 파서 새 PR 로 올린다.
- 알고리즘 wave 는 `feat/<section>-<slug>` 브랜치 → PR (간단한 계획 포함) → 리뷰 코멘트 → 수정 → merge → 브랜치 삭제. base 는 main.
