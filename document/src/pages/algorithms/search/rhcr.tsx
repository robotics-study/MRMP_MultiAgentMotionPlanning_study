import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runRhcr} from "../../../libs/algorithms/rhcr";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/search/rhcr.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/search/rhcr.hpp?raw";
import cppImpl from "../../../../../cpp/src/search/rhcr.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents/window
// 변경에만 재실행한다. 파라미터는 저장소의 configs/search/rhcr.yaml과 같은 값: 창 w(칩으로
// 1→2→3 순환)와 h = 1 고정, 그리고 정직한 budget max_steps = 64 스텝.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>, _vmax: number[], win: number): TraceEvent[] =>
    runRhcr(map, tasks, {window: win, replan_period: 1, max_steps: 64})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. corridor01_head_on은 목록에
// 없다: 어떤 창에서도 정직하게 실패하지만(테스트로 고정), 스텝마다 전체 재계획이라 라이브
// 실행의 트레이스가 터진다 — 부정이 고정되는 곳은 테스트다. agent 상한 4는 CBS 페이지와
// 같은 선이다: 매 스텝 통째로 창 달린 CBS가 다시 도니 라이브 실행을 실시간에 남게 막는다.
const PRESETS: ScenarioPreset[] = [
    {name: "open01_swap", map: "open01", window: 2, agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
    {name: "open01_cross", map: "open01", window: 2, agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "pocket01_swap", map: "pocket01", window: 2, agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "tee01_head_on", map: "tee01", window: 2, agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "maze01_two", map: "maze01", window: 2, agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
]

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Rhcr = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    CBS resolves every collision over the whole horizon in one sitting — its hidden assumption is that all
                    tasks are known at{" "}
                    <InlineMath math="t = 0"/> and the horizon itself is finite. RHCR — Li, Tinka, Kiesel, Durham, Kumar &
                    Koenig (AAAI-21, arXiv:2005.07371) — folds that hybrid pole into the time axis itself: run the same CBS
                    on a <em>window</em> of width{" "}
                    <InlineMath math="w"/>, execute the first{" "}
                    <InlineMath math="h \le w"/> steps, and replan from the <em>actual</em> positions. A collision whose
                    arrival step falls beyond the window is not deferred — it simply does not exist yet. This repository
                    implements the paper's batch adaptation: every agent carries exactly one real task, and once an agent
                    stands on its goal its task sequence is empty (the paper's own degenerate case — a dummy task whose goal
                    is the current cell, i.e. stay put). At{" "}
                    <InlineMath math="w = \infty"/> this is literally CBS again — same tie-breaks, pinned field-for-field by
                    test; narrow the window and myopia becomes the object under study instead of a caveat.
                </p>}
                ko={<p>
                    CBS는 호라이즌 전체의 충돌을 한 번에 해소하고 — 그 숨은 가정은 모든 task가{" "}
                    <InlineMath math="t = 0"/>에 알려 있고 호라이즌 자체가 유한하다는 것이다. RHCR(Li, Tinka, Kiesel, Durham,
                    Kumar & Koenig, AAAI-21, arXiv:2005.07371)은 그 hybrid 극단을 시간축 자체로 접는다: 같은 CBS를 폭{" "}
                    <InlineMath math="w"/>인 <em>창</em> 위에서 돌리고, 첫{" "}
                    <InlineMath math="h \le w"/>스텝을 실행한 뒤, <em>실제</em> 위치에서 다시 계획한다. 도착 스텝이 창 너머인
                    충돌은 연기되는 게 아니라 — 아직 존재하지 않는다. 이 저장소는 논문의 batch adaptation을 구현한다: 모든
                    agent는 정확히 하나의 real task를 들고, agent가 goal 위에 서면 그 task 순서는 비어 있다(논문 자신의 퇴화
                    경우 — dummy task의 goal이 현재 셀, 즉 그자리에 머무르기).{" "}
                    <InlineMath math="w = \infty"/>에서 이건 문자 그대로 다시 CBS이고 — 같은 tie-break, 테스트로 필드 단위로
                    고정된다. 창을 좁히면 근시안은 주의 사항이 아니라 연구 대상이 된다.
                </p>}
            />

            <h2>{t("From Constraint Trees to the Rolling Horizon", "constraint tree에서 롤링 호라이즌으로")}</h2>
            <T
                en={<>
                    <p>
                        Read CBS again and notice what was frozen by construction: the horizon is known from step zero, and
                        every collision on it — however far in the future — is branched away before anyone moves. RHCR thaws
                        exactly that decision. The instance decomposes into a sequence of <em>Windowed MAPF</em> instances:
                        episode{" "}
                        <InlineMath math="T"/> re-runs plain CBS over absolute space-time, but its conflict scan only sees
                        arrival steps in{" "}
                        <InlineMath math="(T,\; T + w]"/>; the first{" "}
                        <InlineMath math="h"/> executed steps become the next episode's starts. Everything else is the hybrid
                        pole verbatim — same space-time A* low level, same canonicalized vertex/edge constraints, same
                        best-first constraint tree with the same tie-breaks. Only the horizon moved.
                    </p>
                    <p>
                        And the plan stays <em>pliable</em>, never frozen: executed history carries forward only through each
                        agent's current position — no old path segment is reserved. A parked agent can be constrained off its
                        own goal by a later window and must leave and come back; that is exactly the flexibility that makes
                        RHCR beat endpoint-holding variants in the paper's own comparison, and it is the same hard-mode
                        semantics the decentralized branch pinned for PIBT from the other side. The generalization pins its
                        own limit: when the window covers an instance's whole horizon, a single episode IS plain CBS — same
                        paths, same expansion counts down to the unit — and rolling at{" "}
                        <InlineMath math="h = 1"/> on top of that replays the very same trajectory, because sum-of-costs
                        decomposes over time. What the window hides is the other half of the deal: a collision beyond{" "}
                        <InlineMath math="w"/> is invisible forever. Not optimal, not complete — by design, not as a caveat.
                    </p>
                </>}
                ko={<>
                    <p>
                        CBS를 다시 읽어라 — 구성상 얼려져 있던 것이 보인다: 호라이즌은 스텝 0에서 알려지고, 그 위의 충돌은
                        아무리 먼 미래든 아무도 움직이기 전에 분기로 지워진다. RHCR은 정확히 그 결정을 녹인다. instance이{" "}
                        <em>Windowed MAPF</em> 인스턴스들의 수열로 분해된다: 에피소드{" "}
                        <InlineMath math="T"/>는 절대 시공간 위에서 plain CBS를 다시 돌리지만, 그 conflict scan은 도착 스텝이{" "}
                        <InlineMath math="(T,\; T + w]"/>에 떨어지는 것만 본다. 실행된 첫{" "}
                        <InlineMath math="h"/>스텝이 다음 에피소드의 start가 된다. 나머지는 그대로 hybrid 극단이다 — 같은
                        시공간 A* low level, 같은 정준화된 vertex/edge constraint, 같은 tie-break의 best-first constraint tree.
                        움직인 건 호라이즌뿐이다.
                    </p>
                    <p>
                        그리고 계획은 <em>pliable</em>로 남고 결코 얼지 않는다: 실행된 역사는 각 agent의 현재 위치를 통해서만
                        이어지고, 오래된 경로 구간은 예약되지 않는다. 주차된 agent도 나중 창이 자기 goal에 제약을 걸면 밀려나서
                        나갔다 다시 와야 한다 — 논문 자체의 method-3 비교에서 RHCR을 endpoint-holding보다 앞서게 만든 정확히 그
                        유연함이고, decentralized 갈래가 PIBT 반대편에서 고정해 둔 것과 같은 hard-mode semantics다. 이 일반화는
                        자기 극한도 고정한다: 창이 instance의 호라이즌 전체를 덮으면 단일 에피소드는 문자 그대로 plain CBS다 —
                        같은 경로에 tie-break까지 동일한 확장 수. 그리고 그 위에 h = 1로 롤링하면 정확히 같은 궤적을 재현한다.
                        sum-of-costs가 시간에 따라 분해되니까. 창이 숨기는 것이 거래의 다른 절반이다: w 너머의 충돌은 영구히
                        보이지 않는다. 최적 아님, 완전 아님 — 주의 사항이 아니라 설계로.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>The window is the axis, pinned on the same maps.</strong>{" "}
                            <InlineMath math="open01\_swap"/> at{" "}
                            <InlineMath math="w = 2"/> lands on CBS's own optimum — [16, 14] = 30, makespan 16; at{" "}
                            <InlineMath math="w = 1"/> every conflict arrives one step late, the only resolution left is a
                            wait that defers itself into the next window, and the alternating waits cost a step nobody
                            recovers: 31 with makespan 17. On <InlineMath math="pocket01\_swap"/> and{" "}
                            <InlineMath math="tee01\_head\_on"/> the swap resolves at{" "}
                            <InlineMath math="w = 2"/> ([4, 6]; costs 10 and 11) and honestly stalls at{" "}
                            <InlineMath math="w = 1"/> — both maps pinning the same stall price of 1895 pops.{" "}
                            <InlineMath math="open01\_cross"/> is the control: timing already separates the pair at (10, 9),
                            no window ever sees a conflict, and every width pins the SAME run down to its expansion count.
                        </li>
                        <li>
                            <strong>Rolling is never free.</strong> The same swap solved in 574 low-level pops by one wide
                            episode costs 3112 when rolled at{" "}
                            <InlineMath math="h = 1"/> — the trajectory is bit-identical, only the recomputation differs. On{" "}
                            <InlineMath math="maze01\_two"/> every window pins 66/33 and the rolling re-solve pays 13,958 pops
                            where CBS spent 2753. Every executed step re-runs a whole windowed CBS; that is what foresight
                            costs when it is recomputed instead of planned once.
                        </li>
                        <li>
                            <strong>No verdict failure mode — only the budget.</strong> Every windowed instance here is
                            satisfiable: parking past the horizon always fits, so the constraint tree never empties and no
                            episode ever proves unsolvability. A head-on corridor stalls at every window (the same tree-graph
                            argument Push and Swap pinned) and the loop just keeps paying until step 64; a budget stop is
                            evidence, never proof — cut the budget to 1 on the solvable cross and it stops after exactly the
                            root's two unconstrained A* searches.
                        </li>
                        <li>
                            <strong>Three parameters, everything else pinned.</strong> The window{" "}
                            <InlineMath math="w"/>, the replan period{" "}
                            <InlineMath math="h \le w"/> (violating it raises — executing a step no resolved window covered is
                            not harder, it is unsafe), and an honest step budget. What the paper leaves free or random —
                            neighbor order, every tie-break of both searches, conflict selection order — is pinned
                            deterministically so all three language mirrors produce identical traces.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>창이 축이고, 같은 맵 위에서 수치로 고정된다.</strong>{" "}
                            <InlineMath math="open01\_swap"/>은 <InlineMath math="w = 2"/>에서 CBS 자신의 최적에 착지한다 —{" "}
                            [16, 14] = 30, makespan 16; <InlineMath math="w = 1"/>에서는 모든 conflict가 한 스텝 늦게 도착하고
                            남은 해소책은 다음 창으로 자기 자신을 연기하는 대기뿐이며, 교대로 밀린 대기가 아무도 회복 못 하는
                            스텝 하나를 더 쓴다: 31에 makespan 17. <InlineMath math="pocket01\_swap"/>과{" "}
                            <InlineMath math="tee01\_head\_on"/>에서 교환은 <InlineMath math="w = 2"/>에서 풀리고([4, 6]; 비용
                            10과 11) <InlineMath math="w = 1"/>에서는 정직하게 교착한다 — 두 맵 모두 같은 1895 pop의 교착 가격을
                            고정한다. <InlineMath math="open01\_cross"/>는 대조군이다: 타이밍이 이미 (10, 9)에서 둘을 갈라 어떤
                            창도 conflict를 보지 않고, 모든 폭이 확장 수까지 같은 실행을 고정한다.
                        </li>
                        <li>
                            <strong>롤링은 결코 공짜가 아니다.</strong> 넓은 에피소드 하나로 574 low-level pop에 풀린 같은 교환이{" "}
                            <InlineMath math="h = 1"/>로 롤링하면 3112가 든다 — 궤적은 비트 단위로 동일한데 재계산만 다르다.{" "}
                            <InlineMath math="maze01\_two"/>에서는 모든 창이 66/33을 고정하고 롤링 재계획이 CBS가 2753을 쓴 곳에
                            13,958 pops를 지불한다. 실행된 매 스텝이 창 달린 CBS 통째로 다시 돈다; 한 번에 계획하는 대신 다시
                            계산되는 선견의 가격이 그거다.
                        </li>
                        <li>
                            <strong>판정 실패 모드는 없다 — budget만 있다.</strong> 여기 모든 windowed instance은 만족 가능하다:
                            호라이즌 너머로 주차하는 건 항상 가능하므로 constraint tree는 결코 비지 않고 어떤 에피소드도
                            불가능성을 증명하지 않는다. 폭 1 통로의 정면 대치는 모든 창에서 교착하고(Push and Rotate가 고정했던 그
                            트리-그래프 논증), 루프는 스텝 64까지 계속 지불할 뿐이다. budget 소진은 증거이지 증명이 아니다 — 풀 수
                            있는 cross에서 budget을 1로 줄이면 정확히 root의 무제약 A* 두 번(35 pop)에서 멈춘다.
                        </li>
                        <li>
                            <strong>파라미터 셋, 나머지는 전부 고정.</strong> 창{" "}
                            <InlineMath math="w"/>, 갱신 주기{" "}
                            <InlineMath math="h \le w"/>(위반하면 clamp가 아니라 raise — 어떤 창도 덮지 않은 스텝을 실행하는 건
                            더 어려운 게 아니라 안전하지 않은 것), 그리고 정직한 step budget. 논문이 자유롭거나 랜덤으로 남긴 것 —
                            이웃 순서, 두 탐색의 모든 tie-break, conflict 선택 순서 — 은 전부 결정론적으로 고정돼 세 언어 미러가
                            동일한 trace를 만든다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The paper's batch adaptation, mirrored line for line: a rolling loop of Windowed MAPF episodes, each
                        one the hybrid pole with its clock shifted. Well-formedness (distinct starts, distinct goals, passable
                        cells) is checked up front — a{" "}
                        <InlineMath math="t = 0"/> collision is history, not a conflict any window could resolve.
                    </p>
                    <Terms items={[
                        ["w", <>the window: an episode at step T resolves collisions whose ARRIVAL step falls in (T, T + w] and ignores every collision beyond — that is what the window IS</>],
                        ["h \\le w", <>the replan period: execute the first h steps, then replan from the actual positions. Safety needs h ≤ w — every executed step must fall inside some resolved window; the pair raises instead of clamping</>],
                        ["\\text{pliable}", <>executed history carries forward only through each agent's CURRENT position; no old path segment stays reserved, so a parked agent can be constrained off its own goal by a later window and must leave and come back</>],
                        ["\\text{flowtime}", <>the reported cost sums each agent's FIRST arrival at its final goal (the paper's objective); the makespan is the simultaneous-co-presence step where execution stops. Paths are the EXECUTED trajectories, padded to that step like the decentralized branch's page</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        논문의 batch adaptation을 라인 그대로 미러링했다: 시계를 옮긴 hybrid 극단으로서의 Windowed MAPF
                        에피소드들을 굴리는 롤링 루프. well-formedness(서로 다른 start, 서로 다른 goal, 통과 가능 셀)는 미리
                        검사한다 — <InlineMath math="t = 0"/> 충돌은 어떤 창도 해소할 수 있는 conflict가 아니라 역사다.
                    </p>
                    <Terms items={[
                        ["w", <>창: 스텝 T의 에피소드는 도착 스텝이 (T, T + w]에 떨어지는 충돌만 해소하고 그 너머의 충돌은 무시한다 — 창이란 바로 그것이다</>],
                        ["h \\le w", <>갱신 주기: 첫 h스텝을 실행하고 실제 위치에서 다시 계획한다. 안전은 h ≤ w를 필요로 한다 — 실행되는 모든 스텝은 어딘가 해소된 창 안에 떨어져야 하고, 쌍을 위반하면 clamp 대신 raise</>],
                        ["\\text{pliable}", <>실행된 역사는 각 agent의 현재 위치를 통해서만 이어진다; 오래된 경로 구간은 예약으로 남지 않는다 — 주차된 agent도 나중 창에 자기 goal에서 밀려나 나갈 수 있다</>],
                        ["\\text{flowtime}", <>보고되는 비용은 각 agent의 FIRST 도착의 합(논문의 목적함수)이고 makespan은 실행이 멈추는 동시 착지 스텝이다. 경로는 실행된 궤적이고 decentralized 갈래 페이지처럼 그 스텝까지 pad된다</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── per instance, once ───────────────────────────────────────────────
w (window), h ≤ w (replan period), step budget; well-formedness: distinct starts/goals, passable cells
executed[i] ← [start_i];  t_now ← 0;  calls ← 0
# ── the rolling loop — one Windowed MAPF episode per replan period ───
loop:
    if every agent stands on its final goal at step t_now: finish (cost = each agent's FIRST arrival)
    if t_now ≥ budget: stop honestly — a window never proves unsolvability, ever
    episode(t_now):                                        # plain CBS with the clock shifted
        root: for i in agent-index order — space-time A* from executed[i][t_now] toward g_i over steps ≥ t_now
              (fixed tie-breaks; a pop counts as arrival only if NO vertex constraint on g_i binds at any step ≥ t)
        CT queue keyed by (Σ absolute arrivals, creation order), best-first:
            conflict = earliest collision whose ARRIVAL step lies in (t_now, t_now + w]   # beyond the window nothing is a conflict
            branch on both agents; each child re-plans that agent from its CURRENT position at t_now
        queue empty → honest failure (an unreachable goal — never a verdict about the instance)
    commit h steps: executed[i] grows by what the resolved paths occupy at those steps
    first step where ALL agents simultaneously stand on their final goals → finish there
    t_now ← t_now + h            # pliable: only CURRENT positions carry forward, nothing old stays reserved`}
    />
            <T
                en={<ol>
                    <li>The episode is plain CBS with the clock shifted: absolute space-time A* from each agent's actual
                        position, canonicalized constraints, best-first CT — and a conflict scan that goes blind past{" "}
                        <InlineMath math="t_{now} + w"/>. The tree always terminates: every constraint binds at some step ≤{" "}
                        <InlineMath math="t_{now} + w"/>, so beyond that horizon nothing binds.</li>
                    <li>Committing a step means appending what each resolved path occupies at that absolute step; the first
                        simultaneous co-presence ends execution and fixes both metrics — flowtime (first arrivals) and the
                        makespan step.</li>
                    <li>The CT queue is keyed by the sum of ABSOLUTE arrival steps, so best-first pops minimize exactly the
                        quantity this repository reports as sum_of_costs; root expansions run in agent-index order and every
                        tie-break (f then insertion order inside A*, earliest conflict step then cell row/col then pair i &lt; j
                        between conflicts) is pinned so Python, C++ and the browser agree bit-for-bit.</li>
                    <li>Rolling re-solves from scratch — no constraint carries across episodes. That is what makes the{" "}
                        <InlineMath math="w = \infty"/> limit exactly CBS (same tie-breaks ⇒ same tree ⇒ same expansions, pinned
                        by test) and every narrower window a strictly local view: the same collision gets rediscovered and
                        re-resolved in episode after episode until its step is behind everyone.</li>
                </ol>}
                ko={<ol>
                    <li>에피소드는 시계를 옮긴 plain CBS다: 각 agent의 실제 위치에서 절대 시공간 A*, 정준화된 constraint,
                        best-first CT — 그리고 <InlineMath math="t_{now} + w"/>너머에서는 맹인이 되는 conflict scan. 트리는
                        항상 종료한다: 모든 constraint는 어딘가 스텝 ≤{" "}
                        <InlineMath math="t_{now} + w"/>에서 묶이고, 그 호라이즌 너머엔 아무것도 묶이지 않는다.</li>
                    <li>스텝을 커밋한다는 건 해소된 경로들이 그 절대 스텝에서 점유하는 것을 덧붙인다는 뜻이다. 모든 agent가 최종
                        goal에 동시에 선 첫 스텝이 실행을 끝내고 두 지표를 고정한다 — flowtime(첫 도착들의 합)과 makespan 스텝.</li>
                    <li>CT 큐는 절대 도착 스텝들의 합을 키로 쓰므로 best-first가 정확히 이 저장소가 sum_of_costs로 보고하는 그
                        양을 최소화해 pop하고; root 확장은 agent index 순서로, 모든 tie-break(A* 안의 f→삽입 순서, conflict
                        사이의 가장 이른 도착 스텝→cell row/col→pair i &lt; j)은 Python·C++·브라우저가 비트 단위로 일치하게
                        고정된다.</li>
                    <li>롤링은 매번 처음부터 다시 푼다 — constraint는 에피소드를 가로질러 이어지지 않는다. 그래서{" "}
                        <InlineMath math="w = \infty"/> 극한이 정확히 CBS가 되고(같은 tie-break ⇒ 같은 트리 ⇒ 같은 확장 수,
                        테스트로 고정) 모든 더 좁은 창은 철저히 국소적인 시야가 된다: 같은 충돌도 그 스텝이 모두의 뒤로 지나갈
                        때까지 에피소드마다 다시 발견되고 다시 해소된다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    Nothing about optimality — a collision beyond{" "}
                    <InlineMath math="w"/> is invisible forever, and the pinned swap shows the price: 31 where CBS spends 30.
                    Nothing about completeness either — narrow windows deadlock honestly on head-on corridors and the loop
                    stops at the budget, never at a proof. What it does guarantee is safety in exactly one direction,{" "}
                    <InlineMath math="h \le w"/>: every executed step falls inside some window that was already resolved
                    collision-free, so committed motion can never collide — and when the window does cover an instance's whole
                    horizon, everything CBS promised survives verbatim. The paper's own framing is worth keeping straight: this
                    is lifelong MAPF batched into one real task per agent; the rolling replan from actual positions (the
                    paper's pliable mode) is what lets a parked agent be routed off its own goal by a later window — the
                    flexibility that endpoint-holding variants pay for with deadlocks. Everything else — optimality,
                    completeness, even feasibility — is exactly as strong as the window you chose, and the demo chip lets you
                    turn that knob yourself.
                </p>}
                ko={<p>
                    최적성에 대해서는 아무것도 보장하지 않는다 —{" "}
                    <InlineMath math="w"/> 너머의 충돌은 영구히 보이고, 고정된 swap이 대가를 보여준다: CBS가 30인 곳에 31.
                    완전성도 보장하지 않는다 — 좁은 창은 정면 통로에서 정직하게 교착하고 루프는 증명이 아니라 budget에서 멈춘다.
                    보장은 정확히 한 방향으로만 성립한다,{" "}
                    <InlineMath math="h \le w"/>: 실행되는 모든 스텝은 이미 충돌 없이 해소된 창 안에 떨어지므로, 커밋된 이동은
                    결코 충돌할 수 없다 — 그리고 창이 실제로 instance의 호라이즌 전체를 덮으면 CBS가 약속한 것이 그대로 살아남는다.
                    논문 자신의 프레이밍을 바로 잡아 둘 가치가 있다: 이건 agent당 실제 task 하나로 배치된 lifelong MAPF이고, 실제
                    위치에서 롤링 재계획하는 pliable 모드가 바로 주차된 agent를 나중 창이 자기 goal에서 밀어내도록 만드는 유연함이며
                    endpoint-holding 변형들이 교착으로 지불하는 바로 그것이다. 나머지는 전부 — 최적성, 완전성, 심지어 실행 가능성까지 —
                    당신이 고른 창만큼만 강하고, 데모의 칩으로 그 노브를 직접 돌릴 수 있다.
                </p>}
            />
            <Proof title="왜 windowed instance는 항상 만족 가능한가(그래서 판정 실패가 없는가)">
                <T
                    en={<p>
                        Every constraint an episode ever creates carries a step at most{" "}
                        <InlineMath math="t_{now} + w"/> — conflicts beyond the window are never even looked at, so no
                        constraint can bind there. Past that horizon nothing binds anything: any feasible plan has an
                        equivalent one of length ≤ max(constrained steps) + |reachable| (park anywhere reachable until every
                        constrained step has passed, then walk), which is also what makes each sub-search finite. So as long as
                        the goal is statically reachable from the current position — the only honest failure a sub-search can
                        still report — the windowed instance is satisfiable and the constraint tree never empties. That is why
                        this planner has no verdict mode: on the corridor, every episode sees a locally-solvable instance (both
                        agents simply wait past the horizon), the loop creeps or stalls until step 64, and what it reports is
                        "no execution within budget" — evidence, never proof. The safety direction needs one line: an executed
                        step s satisfies{" "}
                        <InlineMath math="t_{now} < s \le t_{now} + h \le t_{now} + w"/>, and the episode at{" "}
                        <InlineMath math="t_{now}"/> returned window-clean paths — no vertex conflict, no swap anywhere in
                        that range. QED.<InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        에피소드가 만들어내는 모든 constraint는 스텝이 최대{" "}
                        <InlineMath math="t_{now} + w"/>를 싣는다 — 창 너머의 충돌은 쳐다보지도 않으니 거기 묶일 constraint도 없다.
                        그 호라이즌 너머에선 아무것도 아무것도 묶지 않는다: 모든 실행 가능 계획은 길이 ≤ max(묶인 스텝들) +
                        |reachable|인 동치 계획이 있다(모든 묶인 스텝이 지나갈 때까지 도달 가능한 어딘가에 주차하고 걷기). 그게
                        각 서브 탐색을 유한하게 만드는 이유이기도 하다. 그러니 goal이 현재 위치에서 정적으로 도달 가능하기만 하면 —
                        서브 탐색이 보고할 수 있는 유일한 정직한 실패 — windowed instance는 만족 가능하고 constraint tree는 결코
                        비지 않는다. 그래서 이 planner에게 판정 모드가 없다: 통로에서 모든 에피소드는 국소적으로 풀리는 인스턴스를
                        본다(둘 다 호라이즌 너머로 대기)하고, 루프는 스텝 64까지 기어가거나 교착하며 보고하는 건 "budget 내 실행
                        없음" — 증거이지 증명이 아니다. 안전 방향은 한 줄이다: 실행된 스텝 s는{" "}
                        <InlineMath math="t_{now} < s \le t_{now} + h \le t_{now} + w"/>를 만족하고, 그 에피소드는 창-청결한
                        경로들을 반환했다 — 그 범위 어디에도 vertex conflict 없고 swap도 없다. ∎
                    </p>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, field-for-field what the
                    Python/C++ code below emits — and its window chip is the whole argument of this page.{" "}
                    <code>open01_swap</code>: at w = 2 the conflict enters a window while both agents can still route around
                    each other and you get CBS's own optimum ([16, 14] = 30); click to w = 1 and every conflict arrives one
                    step late — the alternating waits cost exactly one more step (31, makespan 17).{" "}
                    <code>pocket01_swap</code> and <code>tee01_head_on</code> resolve at w = 2 and pin an honest deadlock at
                    w = 1 (both stall at 1895 pops); watch the replayed trajectories freeze. <code>open01_cross</code>{" "}
                    doesn't move at all — timing already separates the pair, so no window ever sees a conflict and even the
                    rolling re-solve pins [16, 17] = 33 (paying 323 pops where CBS spent 35: that is what rolling costs when
                    there is nothing to resolve). <code>maze01_two</code> pins 66/33 at every window for the same reason. The
                    player replays the executed trajectory as one clock over both phases; the expansion blooms still show per
                    agent with absolute steps, and on a rolling run you can watch the SAME collision rediscovered episode
                    after episode until its step falls behind everyone. <code>corridor01_head_on</code> is deliberately not a
                    preset: it stalls at every window (pinned in tests at 976 pops with budget 8) and re-running a whole CBS
                    per step explodes the trace — draw your own corridor if you want to watch an honest stall. The chip cycles
                    w = 1 → 2 → 3; h stays pinned at 1 and the budget at 64 steps.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다 — 아래 Python/C++ 코드가 내뱉는 것과 필드 단위로
                    같은 엔진이고 — 그리고 그 창 칩이 이 페이지의 논지 전체입니다. <code>open01_swap</code>: w = 2에서는
                    conflict가 둘 다 아직 서로를 돌아갈 수 있는 창 안에 도착하고 CBS 자신의 최적([16, 14] = 30)에 착지합니다;
                    w = 1으로 클릭하면 모든 conflict가 한 스텝 늦게 도착하고 — 교대로 밀린 대기가 정확히 스텝 하나를 더 씁니다
                    (31, makespan 17). <code>pocket01_swap</code>과 <code>tee01_head_on</code>은 w = 2에서 풀리고 w = 1에서는
                    정직한 교착을 고정합니다(둘 다 1895 pops에서 멈추고); 재생된 궤적이 얼어붙는 것을 보라.{" "}
                    <code>open01_cross</code>는 아예 움직이지 않는다 — 타이밍이 이미 둘을 갈라서 어떤 창도 conflict를 보지
                    않고, 롤링 재계획조차 [16, 17] = 33을 고정한다(CBS가 35를 쓴 곳에 323 pops를 지불한다: 해소할 게 없을 때
                    롤링의 가격이다). <code>maze01_two</code>도 같은 이유로 모든 창에서 66/33을 고정한다. 플레이어는 실행된
                    ꤀적을 탐색과 실행 두 phase 위에 하나의 시계로 재생하고, 확장 bloom은 여전히 agent별·절대 스텝으로 피어나며,
                    롤링 실행에서는 같은 충돌이 그 스텝이 모두의 뒤로 지나갈 때까지 에피소드마다 다시 발견되는 것을 볼 수 있다.{" "}
                    <code>corridor01_head_on</code>은 일부러 preset이 아니다: 모든 창에서 교착하고(테스트는 budget 8에서 976
                    pops로 고정) 스텝마다 CBS 통째로 다시 돌려 트레이스가 터진다 — 정직한 교착을 보고 싶으면 통로를 직접 그려
                    보라. 칩은 w = 1 → 2 → 3을 순환하고; h는 1, budget은 64스텝으로 고정이다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live rhcr sandbox — the browser engine is a field-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; the chip cycles the window w = 1 → 2 → 3 (h stays 1) and every edit re-plans and replays",
                "라이브 rhcr sandbox — 브라우저 엔진은 Python/C++ planner와 필드 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어지고, 칩은 창 w = 1 → 2 → 3을 순환합니다(h는 1 고정)",
            )} windowHint={{
                en: "window chip: the planning window w (the replan period stays h = 1, the budget 64 steps) — every click widens what each episode can see; on open01_swap that alone flips between CBS's optimum and a one-step myopia, and on pocket/tee it is the difference between resolution and honest deadlock",
                ko: "창 칩: 계획 창 w (갱신 주기 h = 1, budget 64스텝 고정) — 클릭마다 각 에피소드가 볼 수 있는 범위가 넓어지고; open01_swap에서는 그것만으로 CBS의 최적과 스텝 하나의 근시안이 뒤집히고, pocket/tee에서는 해소와 정직한 교착의 차이 그 자체다",
            }} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that
                    powers the live sandbox above is a third mirror — same fixed neighbor order (up/down/left/right-then-wait),
                    same space-time A* frontier (f ascending, ties by insertion order), same CT queue keyed by (sum of absolute
                    arrivals, creation sequence), same conflict order (earliest arrival step; ties by cell row, col, then pair
                    i &lt; j), so all three produce field-identical traces on every scenario, which{" "}
                    <code>check-engine-parity</code> verifies on every build. The code below is the actual source, not an
                    excerpt. Two implementation notes worth knowing: trace events carry ABSOLUTE steps (the same collision
                    reappears as fresh conflict_found events episode after episode — that repetition is the algorithm, not a
                    rendering artifact), and <code>expanded_nodes</code> counts every low-level pop across ALL episodes — the
                    honest price of rolling, which is exactly why the wide-window runs above cost an order of magnitude more.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러다 —
                    동일한 고정 이웃 순서(up/down/left/right 그다음 wait), 동일한 시공간 A* frontier(f 오름차순, 동률은 삽입
                    순서), 동일한 (절대 도착들의 합, 생성 순서) 키의 CT 큐, 동일한 conflict 순서(가장 이른 도착 스텝; 동률은 cell
                    row, col 그다음 pair i &lt; j) — 그래서 셋 모두 모든 시나리오에서 필드 단위로 동일한 trace를 만들고 빌드마다{" "}
                    <code>check-engine-parity</code>가 검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘 구현 노트
                    둘: trace 이벤트는 절대 스텝을 싣고(같은 충돌이 에피소드가 지나갈 때마다 새 conflict_found로 다시 나타난다 —
                    그 반복이 알고리즘이지 렌더링 부산물이 아니다), 그리고 <code>expanded_nodes</code>는 모든 에피소드의 low-level
                    pop 전부를 센다 — 롤링의 정직한 대가이고, 위에서 넓은 창 실행들이 한 자릿수 더 비싼 이유가 정확히 그거다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/search/rhcr.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/search/rhcr.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/search/rhcr.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/search/rhcr.hpp`,
                            },
                            {
                                name: "cpp/src/search/rhcr.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/search/rhcr.cpp`,
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
            <T en={<p>
                RHCR is Li, Tinka, Kiesel, Durham, Kumar and Koenig's answer to lifelong MAPF at warehouse scale (AAAI 2021);
                this page implements its batch adaptation — one real task per agent. The pole it wraps is the page before
                this one: CBS, Sharon, Stern, Felner & Sturtevant.
            </p>} ko={<p>
                RHCR은 창고 규모의 lifelong MAPF에 대한 Li, Tinka, Kiesel, Durham, Kumar와 Koenig의 답이다(AAAI 2021); 이
                페이지는 그 batch adaptation을 구현한다 — agent당 실제 task 하나. 그것이 감싼 극단은 바로 앞 페이지다: CBS,
                Sharon, Stern, Felner & Sturtevant.
            </p>} />
            <ol>
                <li>
                    J. Li, M. Tinka, S. Kiesel, J. K. Durham, V. Kumar, S. Koenig,{" "}
                    <a href="https://arxiv.org/abs/2005.07371" target="_blank" rel="noopener noreferrer">
                        <em>Lifelong Multi-Agent Path Finding in Large-Scale Warehouses</em>
                    </a>,
                    AAAI-21 (arXiv:2005.07371).
                </li>
                <li>
                    G. Sharon, R. Stern, A. Felner, N. R. Sturtevant,{" "}
                    <a href="https://doi.org/10.1016/j.artint.2014.11.006" target="_blank" rel="noopener noreferrer">
                        <em>Conflict-based search for optimal multi-agent pathfinding</em>
                    </a>,
                    Artificial Intelligence, 2015 — the hybrid pole this page folds into time.
                </li>
            </ol>
        </>
    )
}

export default Rhcr
