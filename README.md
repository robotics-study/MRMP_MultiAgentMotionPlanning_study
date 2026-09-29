<div align="center">

# 🤖 MRMP · Multi-Agent Motion Planning study

### 🌐 [robotics-study.github.io/mrmp_introduction](https://robotics-study.github.io/mrmp_introduction/)

문서 사이트가 라이브입니다 — 알고리즘마다 유도·증명·라이브 sandbox 페이지. (한국어/English 토글 내장)

**Multi-agent motion planning 의 계보 — search 기반(MAPF)과 sampling 기반(MRMP)을 C++ / Python 독립 이중 구현으로 스터디**

같은 추상화 설계를 두 언어로 미러링하고, 언어 공용 trace 포맷으로 탐색 과정을 기록하며,<br>
브라우저 라이브 sandbox 로 직접 돌려보고, (scenario × algorithm) 매트릭스로 벤치마크한다.
단일 로봇 navigation 은 자매 저장소<br>
[nav_study](https://github.com/robotics-study/navigation_basic) 에서 다룬다.

*The genealogy of multi-agent motion planning — the search-based branch (MAPF) and the
sampling-based branch — mirrored in C++20 and Python, with step-by-step visualization, live
in-browser sandboxes running the same engines, and a benchmark matrix. Single-robot navigation
lives in the sibling nav_study repo.*

![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)
![CMake](https://img.shields.io/badge/CMake-%E2%89%A53.20-064F8C.svg)
![Tests](https://img.shields.io/badge/tests-110%20py%20%2B%20103%20cpp-brightgreen.svg)

</div>

---

## ✨ 특징

- **📐 공통 추상화** — 모든 planner 는 `MultiAgentPlanner` 를 상속하고, 구체 맵이 아닌 **capability 인터페이스**(`DiscreteSpace`: 4-connected 이동 + wait, Manhattan heuristic)만 요구한다. 새 맵 타입을 추가해도 알고리즘 코드는 바뀌지 않는다.
- **🪞 언어 미러링** — C++ 과 Python 이 같은 설계·같은 파라미터·같은 trace 이벤트를 각자 idiomatic 하게 구현한다. 공유 계약(`spec/`)·파라미터(`configs/`)·맵(`maps/`)은 언어 밖에 두고 양쪽에서 로드한다.
- **🎬 Trace 기반 시각화** — 알고리즘은 탐색 진행을 JSON Lines 이벤트로 방출하고, 재생기는 언어당 하나가 아니라 **하나**(`tools/viz/replay.py`)다. GIF 애니메이션 + 중간 과정 PNG 스냅샷을 만든다.
- **📊 벤치마크 매트릭스** — `tools/bench/run_matrix.py` 가 (scenario × algorithm) 전 조합을 실행해 성공 여부·sum_of_costs·makespan·expanded_nodes 를 수집하고 리포트를 쓴다.
- **🌐 인터랙티브 문서 사이트** — 알고리즘마다 유도·성질·증명·라이브 sandbox(벽을 그리고 endpoint 를 끌어 편집하면 브라우저 엔진이 즉시 재계획)·실제 구현 소스를 한 페이지에 담는다. 브라우저 엔진은 planner 의 세 번째 미러이며, 저장소가 수출한 trace 는 parity 검증의 기준 자료로만 쓰인다.

## 📚 문서 사이트

**[📖 robotics-study.github.io/mrmp_introduction](https://robotics-study.github.io/mrmp_introduction/)** — 우상단 토글로 한국어/English 전환.

알고리즘별 페이지: 개념 유도 + 성질(완전성·최적성·복잡도) 증명 + pseudocode 해설 +
라이브 sandbox 데모(편집하면 즉시 재계획) + 실제 C++/Python 소스 + **원 논문 레퍼런스(DOI)**.

> 사이트 소스는 `document/` (React + Vite SPA). `main` 에 push 되면 GitHub Actions 가
> 빌드해 GitHub Pages 로 배포한다 (`.github/workflows/deploy.yml`).
>
> ```bash
> cd document && yarn install && yarn dev    # 로컬 개발 서버
> yarn build                                 # 배포 번들 (prerender + sitemap 포함)
> node scripts/check-engine-parity.mjs       # 브라우저 데모 엔진 ↔ python trace parity 검증
> ```

## 🗺️ 구현 현황 (parity)

| 섹션 | 알고리즘 | C++ | Python | 원 논문 |
|---|---|:---:|:---:|---|
| search | Prioritized A* | ✅ | ✅ | Erdmann & Lozano-Pérez (1987) |
| search | Push and Swap | ⏳ | ⏳ | Luna & Bekris (IJCAI 2011) |
| search | Push and Rotate | ⏳ | ⏳ | de Wilde, ter Mors & Witteveen (JAIR 2014) |
| search | Joint-space A* | ✅ | ✅ | joint-state search (관행적 baseline) |
| search | CBS | ✅ | ✅ | Sharon, Stern, Felner & Sturtevant (2015) |
| sampling | MA-RRT* | ✅ | ✅ | Čáp, Novák, Vokřínek & Pěchouček (2013) |
| sampling | sRRT | ✅ | ✅ | Wagner, Kang & Choset (2012) |
| sampling | dRRT | ✅ | ✅ | Solovey, Salzman & Halperin (2016) |
| sampling | dRRT* | ✅ | ✅ | Shome, Solovey, Dobson, Halperin & Bekris (Autonomous Robots 2020) |

각 갈래 안에서 계보순(decoupled/priority → coupled → hybrid; priority 갈래는 Push and Swap → Push and Rotate 로 완성 예정 — decentralized 계열)으로 wave 단위로 구현. ✅ done 이 되면 각 알고리즘 페이지의 References 에 원 논문 링크가 붙는다. sampling 갈래의 첫 회원 MA-RRT* 는 논문 자체의 이산화(G-RRT*)로 DiscreteSpace 위에서 구현됐으므로 새 맵 타입 없이 들어왔고, sRRT 도 개별 policy 가 격자에서 BFS tree 로 정확히 구성되므로 같은 DiscreteSpace 위에 들어왔다. dRRT 는 연속 configuration space 용 새 capability ContinuousSpace 위에서 구현됐다. 같은 raster 를 그대로 쓰되 robot 을 반지름 있는 disc 로 다루고, 부풀려진 obstacle 은 쓰지 않는다. dRRT* 는 같은 ContinuousSpace 위의 informed asymptotically-optimal 후속 — 개별 roadmap 이 k-nearest 에서 PRM* connection radius 로, tree 탐색이 oracle growth + decoupled connector 에서 cost-to-come rewiring + branch-and-bound 로 바뀐다. 단일 로컬 planner(VO/RVO/ORCA 등)는 자매 저장소 nav_study 의 local_planning 범위.

## 🚀 빠른 시작

```bash
# Python (>= 3.10) — mrmp 패키지 + viz/dev extras
cd python && pip install -e ".[dev,viz]" && cd ..
PYTHONPATH=$PWD/python .venv/bin/python -m pytest python/tests -q   # 110 passed

# C++ (C++20, CMake >= 3.20, GoogleTest 는 FetchContent 자동)
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build -j
ctest --test-dir cpp/build     # 103 tests
```

### 데모 실행 — 두 언어가 동일한 CLI 인자 (알고리즘 구현 시 활성화)

```bash
# Python
python python/demos/demo_prioritized_astar.py \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_two.yaml \
  --params configs/search/prioritized_astar.yaml --trace out/trace.jsonl

# C++ (동일 인자)
./cpp/build/demos/demo_prioritized_astar \
  --map maps/grid/maze01.yaml --scenario maps/scenarios/maze01_two.yaml \
  --params configs/search/prioritized_astar.yaml --trace out/trace.cpp.jsonl
```

stdout 에 한 줄 JSON metric(`sum_of_costs`·`makespan`·`expanded_nodes`), `--trace` 경로에 step-by-step JSONL trace 가 남는다.

### 시각화 — C++/Python trace 를 같은 도구로 재생

```bash
python tools/viz/replay.py out/trace.jsonl                    # interactive 재생
python tools/viz/replay.py out/trace.jsonl --gif out/x.gif --snapshots out/snaps/
```

### 벤치마크

```bash
python tools/bench/run_matrix.py --out out/report.md
```

## 📁 저장소 구조

```
├── spec/          # 언어 공용 계약 — trace/param 스키마, 맵 포맷 (single source of truth)
├── maps/          # 벤치마크 grid 맵 (pgm/yaml) + agents(start/goal) 시나리오
├── configs/       # 알고리즘별 파라미터 yaml — C++/Python 이 같은 파일을 읽는다
├── cpp/           # C++20 구현 (include + src + demos + GoogleTest)
├── python/        # Python 구현 (mrmp 패키지 + demos + pytest)
├── tools/         # viz(trace 재생기) · bench(매트릭스 러너) · web_export(사이트 맵 + parity trace 수출)
└── document/      # 문서 사이트 (React + Vite SPA → GitHub Pages)
```

아키텍처 원칙(의존 방향, capability 모델, trace 계약)은 [CLAUDE.md](CLAUDE.md) 참고.

## 🧭 새 알고리즘 추가

1. `configs/search/<algo>.yaml` 파라미터 선언 → 2. 두 언어 구현 (`required_capabilities()` 포함)
→ 3. trace 이벤트 방출 → 4. 두 언어 demo → 5. 단위 테스트 (최적성/충돌 없음/no-path/param 검증)
→ 6. bench 매트릭스 통과 → 7. `replay.py --gif/--snapshots` 렌더 확인 → 8. parity 표 + 문서 사이트 페이지 갱신.

상세 체크리스트는 [CLAUDE.md](CLAUDE.md) 참고.
