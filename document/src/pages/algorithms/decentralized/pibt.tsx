import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runPibt} from "../../../libs/algorithms/pibt";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/decentralized/pibt.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/decentralized/pibt.hpp?raw";
import cppImpl from "../../../../../cpp/src/decentralized/pibt.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents 변경에만
// 재실행한다. 파라미터는 정직한 budget 하나뿐이다 (max_steps).
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runPibt(map, tasks, {max_steps: 500})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다.
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
    {name: "pocket01_swap", map: "pocket01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "tee01_head_on", map: "tee01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "corridor01_head_on", map: "corridor01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "maze01_two", map: "maze01", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
]

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Pibt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Everything before this page produced a plan before anyone moved. PIBT — Priority Inheritance
                    with Backtracking, Okumura, Machida, Défago & Tamura (IJCAI 2019 / Artificial Intelligence 310,
                    2022) — is the same priority discipline with the artifact deleted: there is no path until it has
                    been walked. Every timestep, each agent picks its next cell from its neighborhood ranked by
                    distance to its own goal, in decreasing priority order; a cell still occupied is not taken but{" "}
                    <strong>claimed</strong>, and the occupant inherits that claim — it must vacate into one of its
                    own candidates or make the claim fail. A swap is structurally impossible (an inheriting agent may
                    never step into its claimant's cell), a vertex conflict is structurally impossible (every claimed
                    cell is excluded from every later choice), and the top-level decision can never fail — so the
                    group never deadlocks by construction. What kills PIBT instead is topology, exactly where Push and
                    Rotate built machinery: an edge on no cycle gives the blocked agent nowhere to vacate to. The
                    paper says its own lineage plainly — push and swap/rotate "partly influenced" PIBT, which can be
                    regarded as a combination of safe <em>push</em> operations; this page is where that genealogy
                    closes on itself.
                </p>}
                ko={<p>
                    이 페이지 이전의 모든 것은 누가 움직이기 전에 계획을 생산했다. PIBT — Priority Inheritance with
                    Backtracking, Okumura, Machida, Défago & Tamura (IJCAI 2019 / Artificial Intelligence 310, 2022) —
                   는 산출물을 지워버린 같은 우선순위 규율이다: 걷기 전까지 경로는 존재하지 않는다. 매 스텝 각 agent가
                    자기 goal까지 거리로 순서 매겨진 이웃에서 다음 칸을 고르고, 결정은 우선순위 내림차순으로 이뤄지며,
                    아직 점유된 칸은 취하지 않고 <strong>claim</strong>하고 점유자가 그 claim을 상속받는다 — 자기 후보
                    중 하나로 비켜야 하거나 claim을 실패시켜야 한다. swap은 구조적으로 불가능하고(상속받은 agent는
                    claimant의 현재 셀에 결코 들어갈 수 없다), vertex conflict도 구조적으로 불가능하며(claim된 모든 칸은
                    이후 모든 선택지에서 제외), top-level 결정은 결코 실패하지 않는다 — 그래서 구성상 그룹 교착이 없다.
                    PIBT를 죽이는 것은 대신 위상이다 — Push and Rotate가 기계장치를 지었던 바로 그 자리: 어떤 cycle에도
                    속하지 않는 간선은 막힌 agent에게 물러날 곳을 주지 않는다. 논문은 자기 계보를 스스로 밝힌다 — push
                    and swap/rotate가 PIBT에 "부분적으로 영향을 줬고", PIBT는 안전한 <em>push</em> 동작들의 조합으로 볼
                    수 있다; 이 페이지에서 그 계보가 자기 자신으로 닫힌다.
                </p>}
            />

            <h2>{t("From Planned Paths to Per-Step Negotiation", "계획된 경로에서 스텝별 협상으로")}</h2>
            <T
                en={<>
                    <p>
                        Read the search branch's priority discipline again and notice what was always accidental:
                        prioritized planning serves agents one at a time, treats finished paths as immovable, and its
                        incompleteness came from freezing those paths into reservations nobody may edit. PIBT is what
                        that discipline becomes when you re-plan every step instead of once. The fixed serving order
                        becomes floating-point priorities: standing on the goal resets{" "}
                        <InlineMath math="p_i"/> to a base value <InlineMath math="\varepsilon_i \in [0, 1)"/>, still
                        travelling increments it by one each step — so any active agent always outranks any parked one,{" "}
                        <em>until</em> the parked one is pushed off its goal and becomes active again at the bottom of the
                        order. The planned path is replaced by nothing more than a static BFS distance field from each
                        goal, computed once up front (the paper's own suggestion against the on-demand-A* bottleneck).
                        And the reservation becomes single-step: a claim expires the moment it is honored or refused, and
                        nobody reserves anything for tomorrow.
                    </p>
                    <p>
                        The search branch's three named maneuvers collapse into one rule. A claimed cell's occupant
                        inherits the claimant's priority and must vacate — that is push, generalized. Swap cannot
                        happen: the inheriting agent may never move into its claimant's current cell. And rotate survives
                        only where it physically can: a deeper member of an inheritance chain <em>may</em> step into the
                        top agent's vacated cell, so cyclic rotations still occur on graphs with cycles — but there is no
                        machinery to manufacture one. On <InlineMath math="tee01"/> the same head-on swap that Push and
                        Rotate solved with a degree-3 junction deadlocks here honestly, because the retreating agent's own
                        cell has no escape branch; on <InlineMath math="pocket01"/> the pocket exists and pure
                        negotiation resolves the same exchange cheaper than either primitive did.
                    </p>
                </>}
                ko={<>
                    <p>
                        search 갈래의 우선순위 규율을 다시 읽어 보면 우연히 그랬던 것들이 보인다: 우선순위 계획은 agent를
                        한 대씩 서비스하고 완성된 경로를 움직일 수 없는 것으로 취급하며, 그 경로를 아무도 편집 못 할 예약으로
                        얼린 데서 불완전성이 왔다. PIBT는 그 규율을 한 번이 아니라 매 스텝 다시 계획했을 때의 모습이다.
                        고정된 서비스 순서는 부동소수점 우선순위가 된다: goal 위에 서 있으면 <InlineMath math="p_i"/>를{" "}
                        <InlineMath math="\varepsilon_i \in [0, 1)"/> 기본값으로 리셋, 아직 이동 중이면 스텝마다 둘을 더하니
                        active는 parked에게 항상 이기고 — parked가 goal에서 밀려나 다시 active가 되어 순서 맨 아래로 내려가기
                        전까지는. 계획된 경로는 goal에서 오는 정적 BFS 거리 필드로 대체된다(온디맨드 A* 병목에 대한 논문 자체의
                        제안대로 사전에 한 번). 그리고 예약은 스텝 하나가 된다: claim은 받아들여지거나 거절되는 순간 만료되고,
                        아무도 내일을 위해 뭐도 예약하지 않는다.
                    </p>
                    <p>
                        search 갈래의 이름 붙은 세 기동이 규칙 하나로 압축된다. claim된 칸의 점유자는 claimant의 우선순위를
                        상속받고 비켜야 한다 — 그게 push의 일반화다. swap은 일어날 수 없다: 상속받는 agent는 claimant의 현재
                        셀에 결코 들어가지 못한다. rotate은 물리적으로 가능한 곳에서만 살아남는다: 상속 체인의 더 깊은 구성원은
                        top agent가 비운 셀에 <em>들어갈 수 있고</em>, 그래서 cycle이 있는 그래프에서는 순환 rotation이 여전히
                        일어난다 — 하지만 그걸 만들어낼 기계장치는 없다. <InlineMath math="tee01"/>에서 Push and Rotate가
                        degree-3 junction으로 푼 그 정면 교환은 여기서 정직하게 교착한다: 물러나는 agent의 현재 셀에 escape
                        branch가 없기 때문이다. <InlineMath math="pocket01"/>에서는 주머니가 존재하고, 순수 협상이 primitive
                        둘보다 싸게 같은 맞교환을 해결한다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Sub-optimal by design, occasionally optimal by luck.</strong> An agent always steps
                            toward its own goal; nobody ever computes a cheaper joint detour. And yet on these maps the
                            negotiation lands on the joint optimum anyway: <InlineMath math="open01\_swap"/> costs{" "}
                            <InlineMath math="14 + 16 = 30"/> — exactly what CBS searched to prove optimal, and cheaper
                            than Push and Rotate's rigid 36 — because the yielding agent's greedy detour happens to be
                            the optimal one. That is geometry cooperating, never a guarantee.
                        </li>
                        <li>
                            <strong>Reachability, not completeness.</strong> The paper's Theorem 1: if every adjacent pair
                            of free cells lies on a simple cycle, every agent reaches its goal within{" "}
                            <InlineMath math="\mathrm{diam}(G) \cdot |A|"/> steps regardless of priority order. That is
                            reachability — the paper itself notes it never ensures all agents reach their goals{" "}
                            <em>simultaneously</em>, which is what one-shot MAPF demands. This repository runs to
                            simultaneous occupancy or to the step budget, and a budget exhaustion is honestly reported as
                            "no solution found within budget", never as a proof of unsolvability (same convention as
                            CBS's tree budget).
                        </li>
                        <li>
                            <strong>Cost per timestep:</strong> at most{" "}
                            <InlineMath math="|A|"/> decision calls, each sorting at most{" "}
                            <InlineMath math="\Delta + 1"/> candidates — with the distance tables precomputed once by BFS
                            from each goals (<InlineMath math="O(|A| \cdot |E|)"/> overhead), one step costs{" "}
                            <InlineMath math="O(\Delta \log \Delta)"/> per agent and no joint space is ever built. The{" "}
                            <code>expanded_nodes</code> metric counts exactly those decision calls — with two agents it is
                            twice the makespan on every successful run (34 = 2 × 17 on open01_cross), because every agent
                            is decided exactly once per step, via inheritance or top-level.
                        </li>
                        <li>
                            <strong>Parameter-free except one honest budget.</strong> Everything the paper leaves free or
                            random — the <InlineMath math="\varepsilon_i"/> values, candidate tie-breaks — is pinned
                            deterministically here (agent 0 highest; unoccupied before occupied; row-major final
                            tie-break) so all three language mirrors produce identical traces. The only knob is{" "}
                            <code>max_steps</code>.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>설계상 sub-optimal, 운 좋으면 최적.</strong> agent는 항상 자기 goal로 한 칸 나가고,
                            아무도 더 싼 joint 우회를 계산하지 않는다. 그런데 이 맵들에서는 협상이 어쨌든 joint optimum에
                            착지한다: <InlineMath math="open01\_swap"/>은 <InlineMath math="14 + 16 = 30"/> — CBS가 최적임을
                            증명하러 탐색해 낸 그 값이고, Push and Rotate의 딱딱한 36보다 싸다 — 양보하는 agent의 greedy
                            우회가 우연히 최적였기 때문이다. 이건 기하가 협조한 것이지 결코 보장이 아니다.
                        </li>
                        <li>
                            <strong>완전성이 아니라 reachability.</strong> 논문의 정리 1: 인접한 free 칸 쌍이 모두 어떤 simple
                            cycle 위에 놓이면, 모든 agent가 우선순위 순서와 무관하게 <InlineMath math="\mathrm{diam}(G) \cdot
                            |A|"/> 스텝 안에 goal에 도착한다. 그건 reachability다 — 논문 스스로 보장하지 못한다고 밝힌다:
                            모든 agent가 goal에 <em>동시에</em> 도달하는 것, one-shot MAPF가 요구하는 것이 바로 그것인데. 이
                            저장소는 동시 점유까지 달리거나 step budget으로 멈추고, budget 소진은 정직하게 "budget 내 해 없음"으로
                            보고되지 절대 unsolvability의 증거로가 아니다(CBS의 tree budget과 같은 관례).
                        </li>
                        <li>
                            <strong>스텝당 비용:</strong> 스텝마다 결정 호출 최대 <InlineMath math="|A|"/>개, 각각{" "}
                            <InlineMath math="\Delta + 1"/>개 이하 후보를 정렬 — 거리 표를 goal별 BFS로 한 번 사전 계산하면({" "}
                            <InlineMath math="O(|A| \cdot |E|)"/> overhead) 스텝당 agent당 <InlineMath math="O(\Delta \log
                            \Delta)"/>이고 joint 공간은 결코 구성되지 않는다. <code>expanded_nodes</code> metric은 정확히 그
                            결정 호출들을 센다 — agent 둘에서 성공한 모든 실행의 값은 makespan의 두 배(34 = 2 × 17,
                            open01_cross): 매 스텝 모든 agent가 상속이든 top-level이로 정확히 한 번 결정되기 때문이다.
                        </li>
                        <li>
                            <strong>정직한 budget 하나를 빼고 파라미터 무의존.</strong> 논문이 자유롭거나 랜덤으로 남긴 것 —{" "}
                            <InlineMath math="\varepsilon_i"/> 값, candidate tie-break — 전부 여기에 결정론적으로 고정된다
                            (agent 0 최고; 미점유가 점유보다 우선; 최종 tie-break은 row-major) 그래서 세 언어 미러가 동일한
                            trace를 만든다. 노브는 <code>max_steps</code> 하나뿐이다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The procedure runs one synchronized round per timestep; everything else is recursion inside a
                        single round. Priorities are floats, distinct by construction, and their order is total:
                        travelling increments, goal reset — that alone keeps every active agent above every parked one.
                    </p>
                    <BlockMath math="\varepsilon_i = \frac{k - 1 - i}{k}, \qquad p_i[t] = \begin{cases} \varepsilon_i & \pi_i[t] = g_i \\ p_i[t-1] + 1 & \text{otherwise} \end{cases}"/>
                    <Terms items={[
                        ["\\pi[t]", <>the joint assignment — agent index to cell at time t. The trace's full-horizon paths are read off these rows after the run; a parked agent CAN be pushed off its goal by an active one, which is exactly what the priority reset encodes</>],
                        ["C_i", <>agent i's candidate list: <InlineMath math="\pi_i[t]"/> and its free neighbors, sorted by (distance-table value ascending — unreachable sorts last; unoccupied before occupied; row-major cell order as final tie-break)</>],
                        ["claim", <>a tentative assignment for the current step. Claimed cells are excluded from every later choice this round — vertex conflicts are impossible by construction — and an invalid claim is undone, exactly as if it never happened</>],
                        ["inheritance", <>occupant of a claimed cell decides next, with the claimant recorded: stepping into that claimant's current cell is skipped (swap guard), so the chain can rotate around a cycle but never trade two cells pairwise</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        절차는 스텝마다 동기화된 라운드 하나이고, 나머지는 전부 한 라운드 안의 재귀다. 우선순위는 float이고
                        구성상 서로 달라서 순서가 전순서가 된다: 이동 중 증가, goal에서 리셋 — 그것만으로 모든 active가 모든
                        parked 위에 있고, active끼리는 고정된 index 순서가 군다.
                    </p>
                    <BlockMath math="\varepsilon_i = \frac{k - 1 - i}{k}, \qquad p_i[t] = \begin{cases} \varepsilon_i & \pi_i[t] = g_i \\ p_i[t-1] + 1 & \text{그 외} \end{cases}"/>
                    <Terms items={[
                        ["\\pi[t]", <>joint 배정 — 시각 t에서 agent index → 셀. trace의 full-horizon 경로는 실행 후 이 행들에서 읽는다; parked인 agent도 active에게 goal에서 밀려날 수 있고, 바로 그걸 우선순위 리셋이 인코딩한다</>],
                        ["C_i", <>agent i의 후보 목록: <InlineMath math="\pi_i[t]"/>와 자유 이웃 — (거리 표 값 오름차순, 도달 불가가 나중; 미점유가 점유보다 우선; 최종 tie-break은 row-major)으로 정렬</>],
                        ["claim", <>이번 스텝의 투기적 배정. claim된 칸은 이 라운드 이후 모든 선택지에서 제외된다 — vertex conflict는 구성상 불가능 — 그리고 invalid한 claim은 취소되고, 정확히 일어나지 않은 것처럼 된다</>],
                        ["inheritance", <>claim된 칸의 점유자가 다음으로 결정하고 claimant이 기록된다: claimant의 현재 셀에 들어가는 건 건너뛰기(swap guard) — 그래서 체인은 cycle 주위를 rotation할 수 있지만 두 칸을 맞교환하는 일은 결코 없다</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── per instance, once ───────────────────────────────────────────────
D1  DIST[g]: BFS distance table from each goal over free cells   # computed once up front
D2  ε_i ← (k−1−i)/k                                              # distinct base priorities, agent 0 highest
# ── every timestep t: one synchronized round ─────────────────────────
while some agent is not on its goal and the budget holds:
    p_i ← ε_i if π_i[t] = g_i else p_i + 1                        # priority update
    for i in agents sorted by DECREASING p (values distinct ⇒ order total):
        if i still undecided: DECIDE(i, from = none)              # top-level call — never fails
# DECIDE(i, from): the paper's recursive procedure
    calls += 1; candidates ← {π_i[t]} ∪ Neigh(π_i[t]) sorted by (DIST[g_i] ↑, unoccupied first, row-major)
    for v in candidates:
        if v already claimed this round: continue     # vertex conflict impossible by construction
        if v = π_from[t]: continue                    # swap guard — a pairwise trade can never form
        claim v for i                                 # tentative — BEFORE recursing
        if v unoccupied, or its occupant already decided: return valid   # settled
        if DECIDE(occupant(v), from = i): return valid          # inheritance
        undo the claim; continue                                  # backtrack — v never happened
    π_i[t+1] ← π_i[t]; return invalid   # stuck: stay put, report invalid upward (only inherited calls reach this)
commit: every π_i[t+1] is now set; positions advance together`}
            />
            <T
                en={<ol>
                    <li>Priorities are updated first, then agents decide in decreasing order. Values are distinct by
                        construction (<InlineMath math="\varepsilon_i"/> distinct, every step adds the same +1 to active
                        agents and resets parked ones below all active ones), so the order is total — no tie-break needed
                        at the top level.</li>
                    <li>An undecided agent in that order calls the recursive procedure. Its candidates are its current
                        cell plus free neighbors, sorted by the static distance table to its own goal; unoccupied sorts
                        before occupied (the paper's own tie-break, to avoid unnecessary inheritance), and row-major cell
                        order settles what the paper leaves random.</li>
                    <li>The claim is tentative and placed BEFORE recursing — that is what forces an occupant to vacate. If
                        the occupant has already decided this round, the claim simply stands; if undecided, it inherits:
                        decide(occupant, from = me) runs now, and its claimant's current cell is excluded from the
                        occupant's candidates — the swap guard.</li>
                    <li>If the inherited call fails (the occupant has nowhere to go), the claim is undone as if it never
                        happened and the claimant tries its next candidate. Exhausting all candidates makes an inherited
                        call report invalid upward; a top-level call can never fail, because any earlier claim on the
                        decider's own cell would already have routed through it — that is Lemma 1 in one sentence.</li>
                    <li>All decisions are collected before positions advance: every agent moves at step t+1 or stays, and
                        the round is over. The paper's Algorithm 1 line for line; the pinned details (fixed ε, fixed
                        candidate order) are ours so three languages agree bit-for-bit.</li>
                </ol>}
                ko={<ol>
                    <li>우선순위를 먼저 갱신하고, agent는 내림차순 순서로 결정한다. 값은 구성상 서로 다르다({" "}
                        <InlineMath math="\varepsilon_i"/>가 서로 다르고, 매 스텝 active엔 같은 +1이 더해지고 parked는 모든
                        active 아래로 리셋) — 그래서 top-level에서 순서가 전순서이고 tie-break가 필요 없다.</li>
                    <li>그 순서에서 미결정 agent가 재귀 절차를 호출한다. 후보는 자기 현재 셀 plus 자유 이웃, 자기 goal까지의
                        정적 거리 표로 정렬; 미점유가 점유보다 먼저 정렬되고(불필요한 상속을 피하는 논문 자체의 tie-break),
                        논문이 랜덤으로 남긴 마지막 tie-break은 row-major가 대신 고정한다.</li>
                    <li>claim은 투기적이고 재귀 <em>전에</em> 놓인다 — 그게 점유자를 비게 만드는 것이다. 점유자가 이미 이번
                        라운드에 결정했다면 claim은 그냥 성립하고, 미결정이면 상속받는다: decide(occupant, from = 나)가 지금
                        실행되고, claimant의 현재 셀은 점유자의 후보에서 제외된다 — swap guard.</li>
                    <li>상속된 호출이 실패하면(점유자가 갈 데가 없다) claim은 일어나지 않은 것처럼 취소되고 claimant는 다음
                        후보를 시도한다. 후보를 다 소진하면 상속된 호출은 위로 invalid를 보고하고; top-level 호출은 결코
                        실패하지 않는다 — 결정자의 자기 셀에 대한 이전 claim은 이미 그 agent를 통과해 지나갔을 것이기 때문 —
                        Lemma 1을 한 문장으로 줄이면 이거다.</li>
                    <li>모든 결정이 위치가 전진하기 전에 수집된다: 스텝 t+1에 모든 agent가 움직이거나 머물고, 라운드는 끝난다.
                        논문의 Algorithm 1을 라인 그대로; 고정된 디테일(고정 ε, 고정 후보 순서)은 세 언어가 비트 단위로
                        일치하게 우리가 고정했다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    The guarantee is local and the consequence is global: on graphs where every adjacent pair of free
                    cells lies on a simple cycle, the highest-priority agent always gets its preferred cell, so it walks
                    home step by step — and one agent at a time, everyone goes home. Where an edge sits on no cycle, the
                    same machinery deadlocks: the paper's own failure figure is our <InlineMath math="corridor01"/>
                    head-on, and our <InlineMath math="tee01"/> shows the pocket in the wrong place is the same fact.
                    Both fail honestly at the budget here. Expand the proofs for why the cycle condition buys progress,
                    and what exactly its failure says.
                </p>}
                ko={<p>
                    보장은 국소적이고 결과는 전역이다: 인접한 free 칸 쌍이 모두 어떤 simple cycle 위에 놓이는 그래프에서,
                    최고 우선순위 agent는 항상 선호하는 칸을 받고 — 그래서 스텝마다 집으로 걷고, 한 대씩 그렇게 모두가 집에
                    간다. 간선이 어떤 cycle에도 안 속하는 곳에서 같은 기계가 교착한다: 논문 자신의 실패 그림이 우리의{" "}
                    <InlineMath math="corridor01"/> 정면 교환이고, 우리 <InlineMath math="tee01"/>은 주머니가 잘못된 자리에
                    있으면 같은 사실임을 보여준다. 둘 다 여기서 budget에 정직하게 실패한다. cycle 조건이 진보를 사는 이유와 그
                    실패가 정확히 무엇을 말하는지 증명을 펼쳐 보라.
                </p>}
            />
            <Proof title={t("Lemma 1 (why the top agent always gets its cell)", "보조정리 1 (왜 top agent는 항상 자기 칸을 받는가)")}>
                <T
                    en={<p>
                        Let <InlineMath math="a_1"/> be the highest-priority agent at time <InlineMath math="t"/> and{" "}
                        <InlineMath math="v^{*}"/> its nearest-to-goal neighbor. When <InlineMath math="\mathsf{decide}(a_1, \bot)"/>
                        {" "}runs, nothing is claimed yet — so if <InlineMath math="v^{*}"/> is free,{" "}
                        <InlineMath math="a_1"/> takes it and the claim stands. If a second agent{" "}
                        <InlineMath math="a_2"/> occupies <InlineMath math="v^{*}"/>, inheritance fires, and here the cycle
                        condition does its only job: if edge <InlineMath math="(\pi_1[t], v^{*})"/> lies on a simple cycle,{" "}
                        <InlineMath math="v^{*}"/> has a neighbor besides <InlineMath math="\pi_1[t]"/> — and the swap guard
                        excludes exactly one cell, the claimant's. So <InlineMath math="a_2"/>'s candidate list is non-empty
                        after exclusions and it vacates; if its own refuge is occupied by{" "}
                        <InlineMath math="a_3"/> the chain recurses, and only fails if some member has no unclaimed escape at
                        all. Note what the guard does <em>not</em> forbid: a deeper chain member may step into{" "}
                        <InlineMath math="\pi_1[t]"/> — rotations around cycles are allowed; only pairwise trades are excluded.
                        Induction along the chain is the paper's proof that on such graphs the top agent's claim never fails.
                        <InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        <InlineMath math="a_1"/>을 시각 <InlineMath math="t"/>의 최고 우선순위 agent,{" "}
                        <InlineMath math="v^{*}"/>를 자기 goal에 가장 가까운 이웃이라 하자.{" "}
                        <InlineMath math="\mathsf{decide}(a_1, \bot)"/>이 실행될 때 아직 아무것도 claim되지 않았다 — 그래서{" "}
                        <InlineMath math="v^{*}"/>가 비어 있으면 <InlineMath math="a_1"/>이 취하고 claim은 성립한다. 두 번째
                        agent <InlineMath math="a_2"/>가 <InlineMath math="v^{*}"/>를 점유하면 상속이 발동하고, 여기서 cycle
                        조건이 유일한 임무를 한다: 간선 <InlineMath math="(\pi_1[t], v^{*})"/>가 simple cycle 위에 있으면{" "}
                        <InlineMath math="v^{*}"/>는 <InlineMath math="\pi_1[t]"/> 외에 이웃을 하나 더 가지고 — swap guard는
                        정확히 셀 하나, claimant의 셀만 제외한다. 그래서 <InlineMath math="a_2"/>의 후보 목록은 제외 후에도
                        비어 있지 않고 점유자는 비켜나고, 그 피신처가 <InlineMath math="a_3"/>에게 점유되면 체인은 재귀되고,
                        어떤 구성원이도 미claim 탈출로를 갖지 않을 때만 실패한다. guard가 금지하지 않는 것을 주목하라: 더 깊은
                        체인 구성원은 <InlineMath math="\pi_1[t]"/>에 들어갈 수 있다 — cycle 주위 rotation은 허용되고, pairwise
                        교환만 배제된다. 체인을 따른 귀납이, 그런 그래프에서 top agent의 claim이 결코 실패하지 않는다는 논문의
                        증명이다.<InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>
            <Proof title={t("Theorem 1 (reachability within diam(G)·|A| — and why that is not completeness)", "정리 1 (diam(G)·|A| 안의 reachability — 그리고 그게 왜 완전성이 아닌가)")}>
                <T
                    en={<p>
                        While any agent has never yet reached its goal, exactly one of those agents holds the highest
                        priority among them; by Lemma 1 it gets its nearest-to-goal neighbor every single step, so its
                        distance strictly falls and it stands on its goal within <InlineMath math="\mathrm{diam}(G)"/> steps.
                        Standing on the goal resets its priority below all still-active agents — the next active agent takes
                        over the guarantee, and a parked agent pushed off its goal simply re-enters as an active agent at the
                        bottom of the order. Round after round, every agent has stood on its goal within{" "}
                        <InlineMath math="\mathrm{diam}(G) \cdot |A|"/> steps.<InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        어떤 agent도 아직 goal에 도달한 적이 없는 동안, 그들 중 정확히 하나가 최고 우선순위를 갖고; 보조정리 1로
                        그 agent는 스텝마다 자기 goal에 가장 가까운 이웃을 받고, 그래서 거리가 엄격히 떨어져 <InlineMath math="\mathrm{diam}(G)"/>
                        스텝 안에 goal 위에 선다. goal 위에 서면 우선순위가 아직 active인 모든 agent 아래로 리셋되고 — 다음 active가
                        보장을 이어받고, goal에서 밀려난 parked는 순서 맨 아래에서 그냥 다시 active가 된다. 라운드가 반복되면 각 agent는{" "}
                        <InlineMath math="\mathrm{diam}(G) \cdot |A|"/> 스텝 안에 한 번은 goal 위에 선다.<InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>
            <T
                en={<p>
                    Read that bound honestly: it says every agent visits its goal, not that all agents end up standing on
                    their goals together — the paper itself lists "all agents reach their goals simultaneously is never
                    ensured" as a failure category of one-shot MAPF. That gap is why this repository runs the negotiation to{" "}
                    <em>simultaneous</em> occupancy and stops honestly at the budget otherwise; on tree-shaped maps like{" "}
                    <InlineMath math="tee01"/> and <InlineMath math="corridor01"/> the head-on swap has no escape node for the
                    retreating agent — the paper's Figure 5 exactly: inheritance returns invalid, the claimant re-picks its own
                    cell as "next nearest", and nothing ever moves again. A budget exhaustion is evidence of that geometry, not
                    a proof about it: PIBT never certifies unsolvability. That is the branch's bargain in one sentence — cheap,
                    reactive, provably moving on cyclic graphs; silent about everything else.
                </p>}
                ko={<p>
                    그 bound를 정직하게 읽어라: 모든 agent가 goal을 <em>방문</em>한다는 것이지, 모든 agent가 함께 goal 위에
                    서 있다는 게 아니다 — 논문 스스로 one-shot MAPF의 실패 범주로 "모든 agent가 동시에 goal에 도달하는 것은 결코
                    보장되지 않는다"를 올린다. 그 틈 때문에 이 저장소는 협상을 <em>동시</em> 점유까지 달리고 아니면 budget에 정직하게
                    멈춘다; <InlineMath math="tee01"/>이나 <InlineMath math="corridor01"/> 같은 트리 모양 맵에서 정면 교환은 물러나는
                    agent에게 escape node가 없다 — 논문의 Figure 5 그대로다: 상속은 invalid를 반환하고, claimant은 자기 셀을 "다음으로
                    가까운" 것으로 다시 고르고, 두 번 다시 아무것도 움직이지 않는다. budget 소진은 그 기하의 증거지 그 기하에 대한 증명이
                    아니다: PIBT는 unsolvability를 결코 인증하지 않는다. 이 갈래의 거래를 한 문장으로 — 싸고, 반응적이고, cycle 그래프에서
                    움직임이 증명되어 있고; 나머지에 대해서는 침묵한다.
                </p>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what the
                    Python/C++ code below emits. Draw walls, drag a numbered dot or its ring to move an agent's start/goal,
                    add agents; every edit re-plans instantly and replays from step 0 (the replay compresses planning into the
                    first seconds, then steps the executed paths at one cell per tick). The presets are the whole argument of
                    this page. <code>open01_cross</code>: two crossings that never interfere — both walk their unconstrained
                    shortest paths (16 + 17 = 33) and the negotiation is invisible except in the metric, exactly twice per step.{" "}
                    <code>open01_swap</code>: the head-on swap on open ground — agent 0 walks straight through at full speed and
                    agent 1 yields by detouring around row 9 (it may not trade cells: swap guard), landing on 30 = the joint
                    optimum CBS searched for. <code>pocket01_swap</code>: the same exchange where a pocket exists — agent 1 ducks
                    into cell (0,4) and comes back behind, 4 + 6 = 10 moves against Push and Swap's rigid 14 and Push and Rotate's
                    12. <code>tee01_head_on</code>: the pocket one column too early — the retreating agent's own cell has no
                    escape branch, inheritance returns invalid, both re-pick their own cells, and the run deadlocks honestly at
                    the budget (1000 = 2 × 500 decision calls). <code>corridor01_head_on</code>: the same failure with nothing to
                    blame but the corridor. And <code>maze01_two</code> threads the single gap for exactly the joint optimum, 66.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다 — 아래 Python/C++ 코드가 내뱉는 것과 바이트 단위로 같은
                    엔진입니다. 벽을 그리고, 번호가 적힌 점이나 그 링을 끌어 start/goal을 옮기고, agent를 더하면 모든 편집이 즉시 재계획되고
                    재생은 0스텝부터 다시 돕니다(재생은 계획을 첫 몇 초로 압축한 뒤, 실행된 경로를 tick당 한 칸로 스텝합니다). preset들은 이
                    페이지의 논지 전체를 담았습니다. <code>open01_cross</code>: 서로 간섭이 없는 두 교차 — 둘 다 제약 없는 최단경로를 걷고
                    (16 + 17 = 33) 협상은 metric에서나 보입니다(스텝마다 정확히 두 번). <code>open01_swap</code>: 열린 땅에서의 정면
                    맞교환 — agent 0은 풀속도로 똑바로 걷고 agent 1은 row 9로 우회해 양보합니다(칸 맞교환은 불가 — swap guard), 그리고 CBS가
                    탐색으로 찾아낸 joint optimum 30에 착지합니다. <code>pocket01_swap</code>: 주머니가 있는 곳에서의 같은 교환 — agent 1이
                    (0,4)로 몸을 숙여 뒤에서 돌아오고 4 + 6 = 10 이동, Push and Swap의 딱딱한 14와 Push and Rotate의 12 대비.{" "}
                    <code>tee01_head_on</code>: 주머니가 한 칸 이른 자리 — 물러나는 agent의 현재 셀에 escape branch가 없고, 상속은 invalid를
                    돌려주고, 둘 다 자기 셀을 다시 고르고, 실행은 budget(1000 = 2 × 500 결정 호출)에 정직하게 교착합니다.{" "}
                    <code>corridor01_head_on</code>: 통로 말고 blamed할 게 없는 같은 실패. 그리고 <code>maze01_two</code>는 단일 gap을 정확히
                    joint optimum 66으로 통과합니다.
                </p>}
            />
            <Sandbox label={t(
                "Live pibt sandbox — the browser engine is a byte-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; every edit re-plans and replays",
                "라이브 pibt sandbox. 브라우저 엔진은 Python/C++ planner와 바이트 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어집니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that
                    powers the live sandbox above is a third mirror — same fixed neighbor order (up/down/left/right), same{" "}
                    <InlineMath math="\varepsilon_i = (k-1-i)/k"/> (IEEE-754 division is identical across languages), same
                    candidate ranking, same backtrack semantics, so all three produce field-identical traces on every scenario,
                    which <code>check-engine-parity</code> verifies on every build. The code below is the actual source, not an
                    excerpt. Two implementation notes worth knowing: the candidate rank is a complete total order (distance, then
                    occupied-flag, then row-major) so sorting can't disagree across languages — and in TypeScript the comparator
                    compares with <code>&lt;</code> rather than subtracting, because Infinity minus Infinity is NaN; and paths are
                    full-horizon like the priority branch's, every agent's cell at every step up to the makespan, because a parked
                    agent CAN be pushed off its goal by an active one — that is what the priority reset encodes. The trace replays
                    those rows tick by tick; <code>sum_of_costs</code> counts actual moves (waits cost nothing).
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러다 — 동일한 고정
                    이웃 순서(up/down/left/right), 동일한 <InlineMath math="\varepsilon_i = (k-1-i)/k"/>(IEEE-754 나눗셈은 언어 간 동일),
                    동일한 후보 순위, 동일한 backtrack semantics. 그래서 셋 모두 모든 시나리오에서 필드 단위로 동일한 trace를 만들고 빌드마다{" "}
                    <code>check-engine-parity</code>가 검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘 구현 노트 둘: 후보
                    rank는 완전한 전순서(거리, 그다음 점유 플래그, 그다음 row-major)라서 정렬이 언어 간에 어긋날 수 없고 — TypeScript에서는
                    비교기가 뺄셈 대신 <code>&lt;</code>로 비교한다, Infinity − Infinity가 NaN이기 때문이다. 그리고 경로는 priority 갈래처럼
                    full-horizon이다: makespan까지 모든 스텝의 모든 agent 셀 — parked인 agent도 active에게 goal에서 밀려날 수 있고, 그게 바로
                    우선순위 리셋이 인코딩한 것이다. trace는 그 행들을 tick마다 재생하고, <code>sum_of_costs</code>는 실제 이동을 센다(대기는 비용 0).
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/decentralized/pibt.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/decentralized/pibt.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/decentralized/pibt.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/decentralized/pibt.hpp`,
                            },
                            {
                                name: "cpp/src/decentralized/pibt.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/decentralized/pibt.cpp`,
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
                    K. Okumura, M. Machida, F. Défago, Y. Tamura,{" "}
                    <a href="https://arxiv.org/abs/1901.11282" target="_blank" rel="noopener noreferrer">
                        <em>Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding</em>
                    </a>,
                    Artificial Intelligence 310 (2022) 103752 — conference version IJCAI 2019.
                </li>
            </ol>
        </>
    )
}

export default Pibt
