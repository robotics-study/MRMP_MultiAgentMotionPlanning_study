import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runMapfPost} from "../../../libs/algorithms/mapf_post";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/kinodynamic/mapf_post.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/kinodynamic/mapf_post.hpp?raw";
import cppImpl from "../../../../../cpp/src/kinodynamic/mapf_post.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents/vmax
// 변경에만 재실행한다. 파라미터는 config 기본값 그대로: 안전 마커 δ = 1/4(모든 데모 vmax가
// 이진 유리수라 시간이 정확하다)와 상속된 CBS 예산 256.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>, vmax: number[]): TraceEvent[] =>
    runMapfPost(map, tasks, vmax, {delta: 0.25, max_ct_expansions: 256})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계이고 vmax가 timed 시나리오의
// 네 번째 입력이다(전부 이진 유리수: 1, 2, 1/4).
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross_timed", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]], vmax: [1, 2]},
    {name: "open01_swap_timed", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]], vmax: [1, 1]},
    {name: "pocket01_swap_timed", map: "pocket01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
    {name: "tee01_head_on_timed", map: "tee01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
    {name: "corridor01_head_on_timed", map: "corridor01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
    {name: "maze01_two_timed", map: "maze01", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]], vmax: [1, 0.25]},
]

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const MapfPost = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every page so far ended with a plan: a space-time path per agent, cell sequences on a grid.
                    MAPF-POST — Multi-Agent Path Finding with Kinematic Constraints (Hönig, Kumar, Cohen, Ma, Xu,
                    Ayanian & Koenig, ICAPS 2016) — plans nothing new. It takes the search branch's collision-free
                    discrete plan (CBS runs underneath, silently) and converts it into an execution schedule: each
                    agent's route with waits deleted plus one earliest arrival time per retained location, so a real
                    robot dwells on its cell until its departure instant and traverses at exactly its own velocity
                    limit. This is where the genealogy closes on itself. The search branch's artifact becomes this
                    branch's input, and the decentralized branch's lesson — that motion lives in continuous time, not
                    in step indices — gets bolted onto the plan instead of replacing it. A cell stop is no longer a
                    moment: it is an open protected cloud of radius <InlineMath math="\delta"/> around a point, and a
                    move is no longer instantaneous: it takes <InlineMath math="1/v_k"/> at the agent's own limit.
                </p>}
                ko={<p>
                    지금까지의 모든 페이지는 계획으로 끝났다: agent별 space-time 경로, 격자 위 셀 수열.
                    MAPF-POST(Multi-Agent Path Finding with Kinematic Constraints — Hönig·Kumar·Cohen·Ma·Xu·Ayanian &
                    Koenig, ICAPS 2016)는 아무것도 새로 계획하지 않는다. search 갈래의 충돌 없는 이산 계획(아래에서
                    CBS가 조용히 돈다)을 받아 실행 스케줄로 바꾼다: 대기을 지운 route에 유지된 각 위치의 가장 빠른
                    도착 시각 하나씩을 얹는다. 그래서 실제 로봇은 출발 시각까지 셀에 머물다가 자기 속도 한계 그대로
                    이동한다. 계보가 여기서 자기 자신으로 닫힌다. search 갈래의 생산물이 이 갈래의 입력이 되고,
                    decentralized 갈래의 교훈, 운동은 스텝 번호가 아니라 연속 시간에 산다는 것을, 계획을 대체하는 대신
                    계획 위에 얹는다. 셀 정지는 더 이상 순간이 아니다: 반지름 <InlineMath math="\delta"/>의 열린
                    보호 구름이고, 이동은 더 이상 즉각적이지 않다: 자기 한계에서 <InlineMath math="1/v_k"/>가 걸린다.
                </p>}
            />

            <h2>{t("From Plans to Schedules", "계획에서 스케줄로")}</h2>
            <T
                en={<>
                    <p>
                        Read what changes when time gets real. A discrete plan says agent 0 occupies cell{" "}
                        <InlineMath math="c"/> at step 7; it says nothing about when the robot enters, dwells, or
                        leaves, and a real robot cannot track a cell sequence anyway — it accelerates, cruises, and
                        decelerates. The conversion deletes every wait (the first occurrence of each retained location
                        survives) and attaches one earliest arrival time per surviving location. Execution is then
                        fixed by one rule: dwell until the departure instant{" "}
                        <InlineMath math="D_i = t(c_{i+1}) - 1/v_k"/>, then traverse at exactly{" "}
                        <InlineMath math="v_k"/>. The discrete step indices survive only as an ordering — they decide
                        which visitor of a shared cell was earlier, and nothing else.
                    </p>
                    <p>
                        Why post-process instead of planning in continuous space directly? Because the search branch's
                        guarantees come along for free. The discrete plan underneath is still CBS's — sum-of-costs
                        optimal on the grid, complete up to an honest budget — and the conversion itself is exact: a
                        Temporal Plan Graph becomes a Simple Temporal Network whose consistency is guaranteed by
                        construction, so the earliest schedule always exists once a plan does. What this branch inherits,
                        it inherits whole: where CBS branches itself into exhaustion (the width-1 corridor), there is
                        simply no plan to post-process, and the failure comes through with zeroed metrics rather than an
                        invented schedule.
                    </p>
                </>}
                ko={<>
                    <p>
                        시간이 실재가 되면 무엇이 바뀌는지 읽어라. 이산 계획은 "agent 0이 스텝 7에 셀{" "}
                        <InlineMath math="c"/>를 점유한다"고 말할 뿐이다 — 로봇이 언제 진입하고 머물러서 떠나고는 말하지
                        않고, 실제 로봇은 어차피 셀 수열을 추적할 수 없다. 가속하고 순항하고 감속하기 때문이다. 변환은
                        모든 대기를 지운다(유지된 위치의 첫 발생만 남는다) 그리고 살아남은 위치마다 가장 빠른 도착 시각
                        하나를 얹는다. 실행은 그때부터 하나의 규칙으로 확정된다: 출발 시각{" "}
                        <InlineMath math="D_i = t(c_{i+1}) - 1/v_k"/>까지 머물고 그다음 정확히 <InlineMath math="v_k"/>로
                        이동한다. 이산 스텝 번호는 순서 정하는 것까지만 살아남는다 — 공유 셀의 두 방문자 중 누가 먼저였는지를
                        결정하고, 그 외의 것은 아무것도 아니다.
                    </p>
                    <p>
                        왜 연속 공간에서 직접 계획하지 않고 후처리하는가? search 갈래의 보장이 공짜로 따라오기 때문이다.
                        아래 이산 계획은 여전히 CBS 것이다 — 격자에서 sum-of-costs 최적, 정직한 예산까지 완전. 그리고 변환
                        자체는 정확하다: Temporal Plan Graph가 구성상 일관성이 보장되는 Simple Temporal Network가 되고,
                        그래서 계획이 존재하기만 하면 가장 빠른 스케줄은 항상 존재한다. 상속받는 것은 통째로 상속받는다:
                        CBS가 스스로를 고갈까지 분기시키는 곳(폭 1 통로)에서는 후처리할 계획이 그냥 없고, 실패는 지표를
                        0으로 만들어 발명된 스케줄 대신 그대로 통과한다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>A post-processor, not a planner.</strong> The plan underneath is CBS's — optimal on
                            the grid, complete up to the inherited budget — and this branch adds no search of its own.
                            Where the discrete instance has no plan, the honest output is "no plan to post-process" with
                            every metric zeroed, never an invented schedule. The inherited incompleteness is inherited
                            whole: <InlineMath math="corridor01"/> head-on still fails here for exactly the reason it
                            failed in the search branch.
                        </li>
                        <li>
                            <strong>The STN is always consistent.</strong> The constraint graph built from a
                            collision-free plan is acyclic by construction, so no negative cycle can exist and the
                            earliest schedule exists for every feasible instance (Theorem 1 below). There is no
                            infeasibility branch to handle — the only failure mode is the inherited one.
                        </li>
                        <li>
                            <strong>Safety with a positive margin.</strong> Point agents executing a consistent
                            schedule keep graph distance at least{" "}
                            <InlineMath math="2\delta \cdot v_{\min}/v_{\max} > 0"/> (Theorem 2 below) — never merely
                            "not colliding at the same step". The bound is exactly tight where a Type-2 precedence
                            binds and equal velocities make the two markers symmetric.
                        </li>
                        <li>
                            <strong>Earliest, not optimal.</strong> What gets computed is the paper's Algorithm 1: the
                            earliest schedule on the given plan. The paper's LP variants (minimize flow time, maximize{" "}
                            <InlineMath math="v_{\min}"/>) are not implemented here — <code>sum_of_costs</code> is the
                            resulting flow-time analogue (goal arrival times summed), an outcome, not an optimized
                            objective.
                        </li>
                        <li>
                            <strong>Cost:</strong> max-relaxation Bellman-Ford over the constraint edges — linear in
                            vertices per pass, quadratic overall at worst on a graph this small, and{" "}
                            <code>expanded_nodes</code> counts exactly the relaxations that strictly improved a label.
                            That is this branch's own work metric; the base search's expansions are deliberately not
                            counted (CBS runs silently).
                        </li>
                        <li>
                            <strong>The unit-velocity baseline.</strong> At{" "}
                            <InlineMath math="v_k = 1"/> for every agent and shared cells a whole step apart, every
                            Type-2 constraint asks for less than the chain already provides — the schedule reproduces
                            the discrete plan's own costs exactly. And because Type-2 bounds carry lower bound{" "}
                            <InlineMath math="0"/>, scaling every velocity by one factor scales all times by its
                            inverse and nothing else: there is no absolute clock in the construction.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>후처리기이지 planner가 아니다.</strong> 아래 계획은 CBS 것이고(격자에서 최적, 정직한
                            예산까지 완전), 이 갈래는 자기 탐색을 하나도 보태지 않는다. 이산 인스턴스에 계획이 없으면
                            정직한 출력은 "후처리할 계획 없음"이고 모든 지표가 0이며, 발명된 스케줄이 결코 아니다. 상속된
                            불완전성은 통째로 상속된다: <InlineMath math="corridor01"/> 정면 교환은 search 갈래에서 실패한
                            바로 그 이유로 여기서도 실패한다.
                        </li>
                        <li>
                            <strong>STN은 항상 일관된다.</strong> 충돌 없는 계획에서 구성되는 제약 그래프는 구성상 acyclic이고,
                            그래서 음수 cycle이 존재할 수 없어 가장 빠른 스케줄은 feasible한 모든 인스턴스에 존재한다(아래 정리 1).
                            처리해야 할 비실행성 분기가 없다 — 유일한 실패 모드는 상속된 그것뿐이다.
                        </li>
                        <li>
                            <strong>양수 여유가 있는 안전.</strong> 일관된 스케줄을 실행하는 점 agent는 그래프 거리를 항상{" "}
                            <InlineMath math="2\delta \cdot v_{\min}/v_{\max} > 0"/> 이상으로 유지한다(아래 정리 2) — 단순히
                            "같은 스텝에 안 겹침"이 아니다. Type-2 precedence가 조여지고 같은 속도가 두 마커를 대칭시킬 때
                            bound는 정확히 tight해진다.
                        </li>
                        <li>
                            <strong>가장 빠를 뿐 최적은 아니다.</strong> 계산되는 것은 논문의 Algorithm 1이다: 주어진 계획 위의
                            가장 빠른 스케줄. 논문의 LP 변형(flow time 최소화,<InlineMath math="v_{\min}"/> 최대화)은 여기서
                            구현되지 않는다 — <code>sum_of_costs</code>는 그 결과로 생긴 flow-time 유사값(goal 도착 시각들의 합)이고,
                            최적화한 목적 함수가 아니다.
                        </li>
                        <li>
                            <strong>비용:</strong> 제약 간선에 대한 최대 완화 Bellman-Ford — 패스당 정점 선형이고 이 정도 크기의
                            그래프에서 최악에도 이차, 그리고 <code>expanded_nodes</code>는 라벨을 엄격히 개선한 완화만 센다.
                            그게 이 갈래의 자기 작업 지표다. 기반 탐색의 확장은 의도적으로 세지 않다(CBS는 조용히 돈다).
                        </li>
                        <li>
                            <strong>단위 속도 기준선.</strong> 모든 agent가 <InlineMath math="v_k = 1"/>에 공유 셀이 스텝 하나
                            이상 떨어져 있으면 모든 type-2 제약이 체인이 이미 주는 것보다 적은 것을 요구한다 — 스케줄은 이산
                            계획의 자기 비용을 그대로 재현한다. 그리고 type-2 하한은 <InlineMath math="0"/>이라 모든 속도에 같은
                            인자를 걸면 모든 시간이 역수로만 스케일된다: 구성에 절대 시계는 없다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The procedure is a construction, then one fixed-point pass. Construction: every retained event —
                        agent <InlineMath math="j"/> enters cell <InlineMath math="c"/> at discrete step{" "}
                        <InlineMath math="s"/> — becomes a vertex, and each unit move on agent{" "}
                        <InlineMath math="j"/>'s chain splits into three constraints through two fresh marker vertices,{" "}
                        <InlineMath math="\delta"/> before the cell, <InlineMath math="\delta"/> after it. Every shared
                        cell contributes one precedence: from the earlier visitor's outgoing-edge marker to the later
                        visitor's incoming-edge marker. Solving: every edge is a simple temporal constraint{" "}
                        <InlineMath math="[\mathrm{LB}, \infty]"/>, sources are pinned, and max-relaxation Bellman-Ford
                        over the edges in construction order lands on the unique fixed point — the earliest schedule.
                    </p>
                    <BlockMath math="\big(u \xrightarrow{\;\delta/v_k\;} m_1 \xrightarrow{\;(1-2\delta)/v_k\;} m_2 \xrightarrow{\;\delta/v_k\;} v\big), \qquad m_1^{j} \xrightarrow{\;[0,\infty]\;} m_2^{k}, \qquad X_S \xrightarrow{\;[0,0]\;} v_{\mathrm{first}}"/>
                    <Terms items={[
                        ["u, v", <>event vertices — one event each: "agent enters cell at discrete step s". Waits are deleted, so there is exactly one per retained location</>],
                        ["m_1, m_2", <>safety markers — virtual points on the move edge: <InlineMath math="m_1"/> sits <InlineMath math="\delta"/> past the cell (on the outgoing edge), <InlineMath math="m_2"/> sits <InlineMath math="\delta"/> before its cell (on the incoming edge)</>],
                        ["\\delta", <>safety-marker distance. The condition is <InlineMath math="0 < \delta \le 1/2"/> (here 1/4) — clouds are open so boundaries may touch, and <InlineMath math="\delta = 0"/> would collapse every marker onto its cell and void the safety bound, which the planner refuses</>],
                        ["v_k", <>agent k's velocity limit (cells per time unit). Every temporal constraint's lower bound is a length divided by that agent's own speed</>],
                        ["X_S \\xrightarrow{[0,0]} v_{\\mathrm{first}}", <>every agent's first event pinned to the fixed source. The interval <InlineMath math="[0,0]"/> is two-sided, so the time is nailed to exactly 0</>],
                        ["[\\mathrm{LB}, \\infty]", <>a simple temporal constraint: the tail's time plus the lower bound never exceeds the head's time. The three type-1 pieces carry length divided by speed; type-2's lower bound is simply 0</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        절차는 구성 하나와 고정점 패스 하나다. 구성: 유지된 이벤트마다 — agent <InlineMath math="j"/>가
                        이산 스텝 <InlineMath math="s"/>에 셀 <InlineMath math="c"/>에 진입한다는 사건 — 꼭짓점이 생기고,
                        agent <InlineMath math="j"/>의 체인 위 단위 이동마다 새 마커 꼭짓점 둘을 거쳐 세 제약으로 쪼개진다:
                        셀에서 <InlineMath math="\delta"/> 전, 그리고 <InlineMath math="\delta"/> 후. 공유 셀은 precedence를
                        하나씩 보탠다: 먼저 지나간 쪽의 나가는 변 마커에서 나중에 지나갈 쪽의 들어오는 변 마커로. 풀이: 모든
                        간선은 단순 시간 제약 <InlineMath math="[\mathrm{LB}, \infty]"/>이고 출발점은 못 박히며, 구성 순서
                        그대로의 최대 완화 Bellman-Ford가 유일한 고정점에 착지한다 — 가장 빠른 스케줄이다.
                    </p>
                    <BlockMath math="\big(u \xrightarrow{\;\delta/v_k\;} m_1 \xrightarrow{\;(1-2\delta)/v_k\;} m_2 \xrightarrow{\;\delta/v_k\;} v\big), \qquad m_1^{j} \xrightarrow{\;[0,\infty]\;} m_2^{k}, \qquad X_S \xrightarrow{\;[0,0]\;} v_{\mathrm{first}}"/>
                    <Terms items={[
                        ["u, v", <>이벤트 꼭짓점 — "agent가 셀에 이산 스텝에 진입한다"는 사건 하나. 대기는 제거되므로 유지된 위치마다 정확히 하나씩이다</>],
                        ["m_1, m_2", <>안전 마커 — 이동 간선의 가상 점. <InlineMath math="m_1"/>은 셀에서 <InlineMath math="\delta"/> 지난 곳(나가는 변 위), <InlineMath math="m_2"/>는 도달하기 <InlineMath math="\delta"/> 전(들어오는 변 위)</>],
                        ["\\delta", <>안전 마커 거리. 조건은 <InlineMath math="0 < \delta \le 1/2"/>(여기선 1/4) — 구름이 열려 있어 경계가 겹칠 수 없고, <InlineMath math="\delta = 0"/>은 마커를 셀과 겹치게 해 안전 보장을 무효화하므로 플래너가 거부한다</>],
                        ["v_k", <>agent k의 속도 한계(칸/시간). 모든 시간 제약 하한은 길이 나누기 자기 속도</>],
                        ["X_S \\xrightarrow{[0,0]} v_{\\mathrm{first}}", <>모든 agent의 첫 이벤트가 고정된 출발 꼭짓점. 구간 <InlineMath math="[0,0]"/>은 양방향이라 시각이 정확히 0에 못 박힌다</>],
                        ["[\\mathrm{LB}, \\infty]", <>단순 시간 제약: 꼬리 시각에 하한을 더한 값은 머리 시각 이하. type-1 세 조각의 하한은 길이 나누기 속도, type-2는 그냥 0</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── per instance: build the STN once ───────────────────────────────────────
1  routes ← CBS paths with waits deleted (keep the first cell of every run); a retained
   event keeps its discrete step — steps only ORDER type-2 edges, they are never times
2  vertices: one per event, ids in agent-index order then step order; markers take the
   remaining ids in chain order. A parked agent is a single pinned vertex with no edge out
3  type-1: every unit move u→v splits into u→m1 [δ/v_k], m1→m2 [(1−2δ)/v_k], m2→v' [δ/v_k]
4  type-2: for each retained event of j at step s, scan every other agent's RAW path beyond
   s; the FIRST later visit t wins (later ones follow by transitivity):
   edge m1_j → m2_k with bound [0, ∞]. A parked final event is never a type-2 source.
5  pin: X_S → each agent's first event, bound [0, 0]
# ── solve: the DAG's unique fixed point ────────────────────────────────────
6  max-relaxation Bellman-Ford over the edge list in construction order; a pass that
   improves nothing stops the loop. expanded_nodes counts strict improvements only.
   (−∞ + lb stays −∞ — an edge out of an unrelaxed vertex never fires.)
# ── execute: uniform velocity model ────────────────────────────────────────
7  dwell at c_i until D_i = t(c_{i+1}) − 1/v_k, then traverse at exactly v_k;
   a final cell is dwelled in forever`}
            />
            <T
                en={<ol>
                    <li>Route extraction deletes waits by keeping the first occurrence of every run of identical cells.
                        The retained events keep their discrete step indices — but those steps now only decide which
                        visitor of a shared cell came earlier. They are never read as times.</li>
                    <li>Vertex ids ascend in construction order (agent index, then step; markers take the rest), and
                        edge enumeration is fixed the same way. That is what makes three languages produce bit-identical
                        traces: every float below is a dyadic rational, so relaxation sums land on identical doubles.</li>
                    <li>The split is where safety lives. A unit move becomes three edges whose lower bounds are{" "}
                        <InlineMath math="\delta/v_k"/>, <InlineMath math="(1-2\delta)/v_k"/>,{" "}
                        <InlineMath math="\delta/v_k"/>: passing the first marker means being <InlineMath math="\delta"/>{" "}
                        past the cell, and the second is where a later visitor's precedence attaches.</li>
                    <li>The type-2 scan reads the other agent's raw path (waits included) but its found occurrence is a
                        retained arrival by construction — the step before it held a different cell, or the discrete plan
                        would have collided there. A parked final event has no outgoing move edge, so it can never be a
                        type-2 source; and it needs none, because nobody visits a parked agent's cell later without a
                        discrete conflict.</li>
                    <li>Solving is one fixed-point pass: relax every edge in construction order until a pass improves
                        nothing. The graph is acyclic (proof below), so the fixed point is unique, every label lands
                        finite, and it is exactly{" "}
                        <InlineMath math="t(v) = -\mathrm{dist}(v, X_S)"/> — the earliest schedule.</li>
                    <li>Execution mirrors the constraints: dwell until the departure instant (next arrival minus own
                        traversal time), traverse at exactly <InlineMath math="v_k"/>, park forever on the final cell.
                        The paper's non-holonomic extension (orientation vertices, rotate actions) is out of scope —
                        this repository's robots stay point agents on a grid.</li>
                </ol>}
                ko={<ol>
                    <li>route 추출은 같은 셀 연속에서 첫 발생만 남겨 대기를 지운다. 남은 이벤트는 이산 스텝 번호를 지키고,
                        그 스텝은 이제 공유 셀의 두 방문자 중 누가 먼저였는지 결정하는 데만 쓰인다 — 시각으로는 결코 읽히지
                        않는다.</li>
                    <li>꼭짓점 id는 구성 순서(agent index, 그다음 스텝, 마커는 나머지)로 오르고 간선 열거도 같은 식으로 고정된다.
                        이게 세 언어가 비트 단위로 동일한 trace를 만드는 이유다 — 아래 모든 float은 이진 유리수라 완화 합산이
                        동일한 double에 착지한다.</li>
                    <li>쪼개기가 안전이 사는 자리다. 단위 이동이 하한 <InlineMath math="\delta/v_k"/>,{" "}<InlineMath math="(1-2\delta)/v_k"/>,{" "}
                        <InlineMath math="\delta/v_k"/>인 세 간선이 된다: 첫 마커를 지났다는 건 셀에서 <InlineMath math="\delta"/>만큼
                        지났다는 뜻이고, 두 번째 마커가 나중에 올 방문자의 precedence가 붙는 자리다.</li>
                    <li>type-2 스캔은 다른 agent의 raw 경로(대기 포함)를 읽지만 발견된 발생은 구성상 유지된 도착이다 — 바로 전
                        스텝이 다른 셀을 잡고 있었지 않았으면 이산 계획이 거기서 충돌했다. parked 최종 이벤트는 나가는 이동 간선이
                        없어 type-2 출발지가 될 수 없고, 필요도 없다: 아무도 discrete conflict 없이 parked 셀에 나중에 오지 않는다.</li>
                    <li>풀이는 고정점 패스 하나다: 개선 없는 패스가 나올 때까지 구성 순서 그대로 모든 간선을 완화한다. 그래프는
                        acyclic하고(증명은 아래), 그래서 고정점은 유일하고 모든 라벨이 유한한 값에 착지하며, 그게 정확히{" "}
                        <InlineMath math="t(v) = -\mathrm{dist}(v, X_S)"/> — 가장 빠른 스케줄이다.</li>
                    <li>실행은 제약을 그대로 흉내 낸다: 출발 시각(다음 도착 빼기 자기 이동 시간)까지 머물고, 정확히{" "}
                        <InlineMath math="v_k"/>로 이동하고, 마지막 셀에는 영구히 머문다. 논문의 non-holonomic 확장(orientation
                        꼭짓점, rotate 동작)은 범위 밖이다 — 이 저장소의 로봇은 격자 위 점 agent로 남는다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    The guarantee is conditional and exact: given a collision-free discrete plan, the construction above
                    always yields a schedule, executing it keeps every pair of agents at graph distance{" "}
                    <InlineMath math="2\delta \cdot v_{\min}/v_{\max}"/> or more, and among all schedules respecting the
                    constraints this one is earliest. What it cannot: no plan-making. Where the discrete instance has no
                    plan — head-on in a width-1 corridor — the honest output is failure with zeroed metrics, and the
                    branch inherits that incompleteness without complaint. And "earliest" is not "optimal": the paper's
                    flow-time-minimizing LP variants are not implemented here, so a wait the discrete plan wasted stays
                    in the schedule unless the markers' slack lets it shrink (on <InlineMath math="tee01"/> it does —
                    11 becomes 10.5; elsewhere it stands). Read the two proofs for why consistency is structural rather
                    than lucky, and where the safety margin comes from.
                </p>}
                ko={<p>
                    보장은 조건부이고 정확하다: 충돌 없는 이산 계획이 주어지면 위 구성은 항상 스케줄을 내고, 그것을 실행하면
                    모든 agent 쌍의 그래프 거리가 <InlineMath math="2\delta \cdot v_{\min}/v_{\max}"/> 이상이고, 제약을 지키는
                    모든 스케줄 중 이것이 가장 빠르다. 못 하는 것: 계획 만들기. 이산 인스턴스에 계획이 없는 곳 — 폭 1 통로의
                    정면 교환 — 정직한 출력은 지표가 0인 실패이고, 이 갈래는 그 불완전성을 불평 없이 상속받는다. 그리고 "가장
                    빠름"은 "최적"이 아니다: 논문의 flow-time 최소화 LP 변형은 여기서 구현되지 않아, 이산 계획이 낭비한 대기는
                    마커의 여유가 그것을 줄여주지 않는 한 스케줄에 남는다(<InlineMath math="tee01"/>에서는 줄어든다 — 11이 10.5가
                    된다. 다른 곳에서는 그대로 선다). 일관성이 운이 아니라 구조인 이유와 안전 여유가 어디서 나오는지 두 증명을
                    펼쳐 읽어라.
                </p>}
            />
            <Proof title={t("Theorem 1 (the STN is always consistent)", "정리 1 (STN은 항상 일관된다)")}>
                <T
                    en={<p>
                        Order the vertices by discrete time: an event at step <InlineMath math="s"/> ranks{" "}
                        <InlineMath math="2s"/>, and a marker on the segment leaving step <InlineMath math="s"/> ranks{" "}
                        <InlineMath math="2s+1"/>. Type-1 edges go strictly forward in this order along their own chain.
                        A type-2 edge leaves the earlier visitor's marker (rank <InlineMath math="2s+1"/>) and lands on
                        the later visitor's incoming marker, whose event has step{" "}
                        <InlineMath math="t > s"/> — so it lands at rank ≥ <InlineMath math="2s+1"/> inside or after the
                        same interval. Inside one interval the only edges are into a marker whose sole successor is that
                        next event itself: an edge back would require the later visitor to vacate what the earlier one
                        enters in the same step — exactly the swap a collision-free plan cannot contain. So every edge
                        points forward in a total order, the graph is acyclic, and a negative-cost cycle is impossible.
                        An STN with no negative cycle is consistent: every vertex chains from a pinned source,{" "}
                        <InlineMath math="t(v) = -\mathrm{dist}(v, X_S)"/> lands finite everywhere, and the earliest
                        schedule exists.<InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        꼭짓점을 이산 시간으로 정렬한다: 스텝 <InlineMath math="s"/>의 이벤트는 순위 <InlineMath math="2s"/>,
                        스텝 <InlineMath math="s"/>에서 나가는 변 위의 마커는 순위 <InlineMath math="2s+1"/>. type-1 간선은
                        자기 체인을 따라 이 순서로 엄격히 전진한다. type-2 간선은 먼저 지나간 쪽의 마커(순위{" "}
                        <InlineMath math="2s+1"/>)에서 나가 나중에 지나갈 쪽의 들어오는 마커에 착지하는데, 그 이벤트의 스텝은{" "}
                        <InlineMath math="t > s"/> — 그래서 같은 구간 안이나 그 이후, 순위 ≥ <InlineMath math="2s+1"/>에
                        착지한다. 한 구간 안에서 간선은 후임이 곧 다음 이벤트인 마커로만 들어간다. 되돌아가는 간선이 있으려면
                        나중에 온 방문자가 먼저 온 agent가 들어가는 셀을 같은 스텝에 비워야 한다 — 충돌 없는 계획이 결코 담을 수
                        없는 swap 그거다. 그래서 모든 간선은 전순서에서 앞을 향하고, 그래프는 acyclic이고, 음수 비용 cycle은
                        불가능하다. 음수 cycle이 없는 STN은 일관된다: 모든 꼭짓점이 못 박힌 출발점에서 체인으로 이어지고,{" "}
                        <InlineMath math="t(v) = -\mathrm{dist}(v, X_S)"/>가 모든 곳에서 유한한 값에 착지하며, 가장 빠른 스케줄이
                        존재한다.<InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>
            <Proof title={t("Theorem 2 (distance stays above 2δ·v_min/v_max)", "정리 2 (거리는 항상 2δ·v_min/v_max 이상)")}>
                <T
                    en={<>
                    <p>
                        Take a cell <InlineMath math="c"/> both routes visit, earlier visitor <InlineMath math="j"/>,
                        later <InlineMath math="k"/>. First, the two edges through <InlineMath math="c"/> are distinct —
                        hence perpendicular — because a same-edge pass-through is impossible in a collision-free plan:
                        if <InlineMath math="j"/> leaves <InlineMath math="c"/> toward <InlineMath math="n"/> and{" "}
                        <InlineMath math="k"/> enters <InlineMath math="c"/> from <InlineMath math="n"/>, arriving at step{" "}
                        <InlineMath math="s+1"/> is exactly an edge conflict, and arriving any later only leaves{" "}
                        <InlineMath math="k"/> still standing on <InlineMath math="n"/> when <InlineMath math="j"/>
                        arrives there. So the two markers sit at distance <InlineMath math="\delta"/> from{" "}
                        <InlineMath math="c"/> along different axes, exactly <InlineMath math="2\delta"/> apart in
                        Manhattan. Now unroll the constraint chain:
                    </p>
                    <BlockMath math="t_k(c) \;\ge\; T(m_2^{k}) + \frac{\delta}{v_k} \;\ge\; T(m_1^{j}) + \frac{\delta}{v_k} = D_j(c) + \frac{\delta}{v_j} + \frac{\delta}{v_k}"/>
                    <p>
                        The later visitor reaches <InlineMath math="c"/> only after the earlier one passed its own marker,{" "}
                        <InlineMath math="\delta"/> gone — equivalently, the open clouds of radius <InlineMath math="\delta"/>
                        around <InlineMath math="c"/> are disjoint in time (open sets may touch). At the touching instant
                        both sit exactly <InlineMath math="\delta"/> from <InlineMath math="c"/> on perpendicular axes:
                        Manhattan distance exactly <InlineMath math="2\delta"/>, and equal velocities make that instant
                        the minimum — which is what <InlineMath math="tee01"/> pins at 0.5 with{" "}
                        <InlineMath math="\delta = 0.25"/>. With unequal velocities the later one's own marker segment takes{" "}
                        <InlineMath math="\delta/v_k"/> to cross while the earlier keeps moving, and the worst case over all
                        configurations lands at <InlineMath math="2\delta \cdot v_{\min}/v_{\max}"/> — still strictly positive. That is the whole safety story: not "no collision at step granularity" but
                        a positive geometric margin, tight exactly where a precedence binds.<InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                    <p>
                        두 route가 모두 지나는 셀 <InlineMath math="c"/>, 먼저 지나간 쪽 <InlineMath math="j"/>, 나중에{" "}
                        <InlineMath math="k"/>. 첫째, <InlineMath math="c"/>를 지나는 두 변은 다르다 — 그래서 수직이다 — 같은
                        간선 관통이 충돌 없는 계획에서 불가능하기 때문이다: <InlineMath math="j"/>가 <InlineMath math="c"/>에서{" "}
                        <InlineMath math="n"/>으로 나가고 <InlineMath math="k"/>가 <InlineMath math="n"/>에서{" "}
                        <InlineMath math="c"/>로 들어오는데, 스텝 <InlineMath math="s+1"/>에 도착하는 건 정확히 edge conflict이고,
                        그보다 늦게 도착하는 건 <InlineMath math="j"/>이 거기 도착했을 때 <InlineMath math="k"/>가 아직{" "}
                        <InlineMath math="n"/>에 서 있다는 뜻이다. 그래서 두 마커는 <InlineMath math="c"/>에서 다른 축 방향으로
                        거리 <InlineMath math="\delta"/>에 놓이고 Manhattan으로 정확히 <InlineMath math="2\delta"/> 떨어진다. 이제
                        제약 체인을 풀자:
                    </p>
                    <BlockMath math="t_k(c) \;\ge\; T(m_2^{k}) + \frac{\delta}{v_k} \;\ge\; T(m_1^{j}) + \frac{\delta}{v_k} = D_j(c) + \frac{\delta}{v_j} + \frac{\delta}{v_k}"/>
                    <p>
                        나중에 온 방문자는 먼저 지나간 쪽이 자기 마커(셀에서 <InlineMath math="\delta"/>만큼 떨어진 곳)를 지난
                        뒤에야 <InlineMath math="c"/>에 도달한다. 동치로, <InlineMath math="c"/> 반지름 <InlineMath math="\delta"/>의
                        열린 구름들이 시간으로 서로 disjoint하다(열려 있어 경계는 닿을 수 있다). 닿는 순간 둘은 정확히{" "}
                        <InlineMath math="c"/>에서 수직 축 방향으로 <InlineMath math="\delta"/> 떨어진 지점에 서 있고 Manhattan
                        거리는 정확히 <InlineMath math="2\delta"/>다. 속도가 같으면 그 순간이 최소가 되고,<InlineMath math="tee01"/>이{" "}
                        <InlineMath math="\delta = 0.25"/>에서 0.5로 고정해 둔 게 그것이다. 속도가 다르면 나중에 온 쪽의 마커 구간을
                        건너는 데 <InlineMath math="\delta/v_k"/>가 걸리는 동안 먼저 간 쪽은 계속 움직여, 모든 구성에 걸친 최악이{" "}
                        <InlineMath math="2\delta \cdot v_{\min}/v_{\max}"/>에 착지한다 — 여전히 엄격히
                        양수다. 그게 안전의 전부다: "스텝 단위로 안 겹침"이 아니라 양수 기하 여유이고, precedence가 조여지는 자리에
                        정확히 tight해진다.<InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what the
                    Python/C++ code below emits. Draw walls, drag a numbered dot or its ring to move an agent's start/goal,
                    add agents; every edit re-plans instantly and replays from time 0 (the replay compresses planning into
                    the first seconds, then runs the schedule in a fixed window — dwell is standing still on a cell, traverse
                    is gliding at exactly that agent's velocity). The presets are the whole argument of this page.{" "}
                    <code>open01_swap_timed</code>: both agents at one cell per time unit replay their discrete plan exactly
                    (sum 30, makespan 16 — CBS's own numbers) with a head-on swap resolved by yielding.{" "}
                    <code>open01_cross_timed</code>: agent 1 is twice as fast, and the schedule is no longer any rescaling of
                    a discrete plan — the fast agent dwells at (9,9) until 7.875 and crosses (10,9) at exactly{" "}
                    <InlineMath math="8 + \delta/1 + \delta/2 = 8.375"/> while the slow one never waits at all; sum 28.375
                    against the discrete plan's 33,
                    makespan 16 because the slow clock is the makespan. <code>tee01_head_on_timed</code>: compression — the
                    discrete plan wasted a whole step waiting at (1,2), here agent 1 departs (1,3) at t = 2, its marker
                    clears at 2.25 and agent 0 arrives at 2.5 instead of step 3: sum 10.5 against 11, and the minimum
                    distance is exactly <InlineMath math="0.5 = 2\delta"/> at <InlineMath math="t = 2"/>, tight where the
                    precedence binds. <code>pocket01_swap_timed</code>:
                    the pocket detour at unit velocity (sum 10) — double both vmax in the sandbox and every time halves,
                    because nothing in the construction is an absolute clock. <code>maze01_two_timed</code>: v = 1 against
                    v = 1/4 through one gap; the slow agent's clock dominates (makespan 132 = 4 × 33) and the fast one's
                    arrival at the shared cell slides from discrete step 23 to 89.25. And <code>corridor01_head_on_timed</code>:
                    nothing to post-process — every metric zero, honestly.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다 — 아래 Python/C++ 코드가 내뱉는 것과 바이트 단위로
                    같은 엔진입니다. 벽을 그리고, 번호가 적힌 점이나 그 링을 끌어 start/goal을 옮기고, agent를 더하면 모든 편집이
                    즉시 재계획되고 재생은 시각 0부터 다시 돕니다(재생은 계획을 첫 몇 초로 압축한 뒤 고정 창에서 스케줄을 굴립니다 —
                    dwell은 셀에 그냥 서 있는 것이고 traverse는 정확히 그 agent의 속도로 미끄러지는 것입니다). preset들은 이 페이지의
                    논지 전체를 담았습니다. <code>open01_swap_timed</code>: 둘 다 시간당 한 칸이면 이산 계획이 그대로 재현됩니다
                    (합 30, makespan 16 — CBS 자신의 숫자)이고 정면 맞교환은 양보로 풀립니다. <code>open01_cross_timed</code>:
                    agent 1이 두 배 빠르고, 스케줄은 더 이상 이산 계획의 어떤 재스케일링도 아닙니다 — 빠른 쪽은 (9,9)에서 7.875까지
                    머물다 정확히 <InlineMath math="8 + \delta/1 + \delta/2 = 8.375"/>에 (10,9)를 지나고, 느린 쪽은 한 번도 서지
                    않습니다. 합 28.375 대 이산 계획의
                    33, makespan은 느린 시계의 16. <code>tee01_head_on_timed</code>: 압축 — 이산 계획은 (1,2)에서 스텝 하나를 대기로
                    낭비했고, 여기서 agent 1은 t = 2에 (1,3)을 떠나 마커가 2.25에 비켜 agent 0이 스텝 3 대신 2.5에 도착합니다: 합
                    10.5 대 11, 그리고 최소 거리는 <InlineMath math="t = 2"/>에서 정확히 <InlineMath math="0.5 = 2\delta"/> —
                    precedence가 조여지는 자리에 tight합니다.{" "}
                    <code>pocket01_swap_timed</code>: 단위 속도의 주머니 우회(합 10) — sandbox에서 vmax 둘 다 두 배로 걸면 모든 시간이
                    절반이 됩니다. 구성에 절대 시계가 없으니까요. <code>maze01_two_timed</code>: v = 1 대 v = 1/4가 단일 gap을
                    지나고, 느린 agent의 시계가 지배해 makespan 132 = 4 × 33이고 빠른 쪽의 공유 셀 도착은 이산 스텝 23에서 89.25로
                    밀립니다. 그리고 <code>corridor01_head_on_timed</code>: 후처리할 게 없습니다 — 모든 지표 0, 정직하게.
                </p>}
            />
            <Sandbox label={t(
                "Live mapf_post sandbox — the browser engine is a byte-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; every edit re-plans and replays",
                "라이브 mapf_post sandbox. 브라우저 엔진은 Python/C++ planner와 바이트 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어집니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that
                    powers the live sandbox above is a third mirror — same vertex numbering (agent index, then step order),
                    same edge enumeration, same max-relaxation loop counting strict improvements, so all three produce
                    field-identical traces on every scenario, which <code>check-engine-parity</code> verifies on every
                    build. The code below is the actual source, not an excerpt. Two implementation notes worth knowing:{" "}
                    <InlineMath math="\delta"/> and every demo velocity are dyadic rationals (1/4, 1, 2), so every bound,
                    sum, and division lands on an
                    exactly representable double — the strict-improvement counts and arrival times match bit-for-bit across
                    languages; and the underlying CBS runs with no recorder at all, because this branch's trace carries only
                    schedules (<code>schedule_found</code> per agent) and its own metric counts STN relaxations, never the
                    base search's expansions. The replay follows the execution semantics literally: dwell means standing on
                    the cell until the departure instant, traverse means gliding at exactly <InlineMath math="v_k"/>, and
                    fractional time is first-class in the player.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러다 —
                    같은 꼭짓점 번호 매기기(agent index, 그다음 스텝 순서), 같은 간선 열거, 엄격한 개선만 세는 같은 최대 완화 루프.
                    그래서 셋 모두 모든 시나리오에서 필드 단위로 동일한 trace를 만들고 빌드마다 <code>check-engine-parity</code>가
                    검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘 구현 노트 둘: <InlineMath math="\delta"/>와
                    모든 데모 속도가 이진 유리수(1/4, 1, 2)라
                    모든 하한·합·나눗셈이 정확히 표현 가능한 double에 착지하고 — 엄격 개선 횟수와 도착 시각이 언어 간 비트 단위로 일치한다.
                    그리고 기반 CBS는 recorder 없이 완전히 조용히 돌고 — 이 갈래의 trace는 스케줄(agent마다 <code>schedule_found</code>)만
                    싣고 자기 지표는 STN 완화를 세지 기반 탐색의 확장을 세지 않는다. 재생은 실행 semantics를 그대로 따른다: dwell은 출발
                    시각까지 셀에 서 있는 것이고 traverse는 정확히 <InlineMath math="v_k"/>로 미끄러지는 것이며, 소수 시간은 플레이어의
                    1급 시민이다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/kinodynamic/mapf_post.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/kinodynamic/mapf_post.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/kinodynamic/mapf_post.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/kinodynamic/mapf_post.hpp`,
                            },
                            {
                                name: "cpp/src/kinodynamic/mapf_post.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/kinodynamic/mapf_post.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The planner and its C++ mirror, embedded from the repository sources",
                    "planner와 그 C++ 미러. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    W. Hönig, N. Kumar, A. Cohen, H. Ma, C. Xu, N. Ayanian & W. Koenig,{" "}
                    <a href="https://ojs.aaai.org/doi/10.1609/icaps.v26i1.13796" target="_blank" rel="noopener noreferrer">
                        <em>Multi-Agent Path Finding with Kinematic Constraints</em>
                    </a>,
                    ICAPS 26 (2016) — DOI 10.1609/icaps.v26i1.13796.
                </li>
            </ol>
        </>
    )
}

export default MapfPost
