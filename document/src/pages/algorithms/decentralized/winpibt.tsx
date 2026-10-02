import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runWinpibt} from "../../../libs/algorithms/winpibt";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/decentralized/winpibt.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/decentralized/winpibt.hpp?raw";
import cppImpl from "../../../../../cpp/src/decentralized/winpibt.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents/window
// 변경에만 재실행한다. 파라미터는 창 w(칩으로 1→2→3 순환)과 정직한 budget max_steps 둘.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>, _vmax: number[], win: number): TraceEvent[] =>
    runWinpibt(map, tasks, {window: win, max_steps: 500})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. 창 칩 기본값은 2 (저장소 config의
// 기본값과 동일): pocket01_swap에서 이 값이 칼날이다 (w ≤ 2 성공, w ≥ 3 정직한 교착).
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross", map: "open01", window: 2, agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", window: 2, agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
    {name: "pocket01_swap", map: "pocket01", window: 2, agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "tee01_head_on", map: "tee01", window: 2, agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "corridor01_head_on", map: "corridor01", window: 2, agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
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

const Winpibt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    PIBT negotiates one cell per timestep — and that single step is its myopia: an agent cannot see
                    around a corner, cannot anticipate an oncoming crowd, and reacts only when a cell is already being
                    claimed. winPIBT — Okumura, Tamura & Défago (IJCAI-20 MAPF workshop, arXiv:1905.10149) — is the
                    same priority discipline generalized along the one axis PIBT left frozen: <em>time</em>. Every agent
                    now holds a provisional space-time path{" "}
                    <InlineMath math="\Pi_i"/> and extends it{" "}
                    <InlineMath math="w"/> steps ahead at a time, securing the new steps one by one in priority order;
                    lower priorities may never reserve beyond what higher ones have already secured. At{" "}
                    <InlineMath math="w = 1"/> this is PIBT again — it reproduces PIBT's pinned totals on every shared
                    scenario here — and as the window grows, reservations freeze further and further ahead, until the
                    greedy window behaves exactly like prioritized planning. The same head-on swap that negotiation
                    resolves at <InlineMath math="w \le 2"/> deadlocks honestly at{" "}
                    <InlineMath math="w \ge 3"/>: this page shows the branch's two failure modes as one continuous knob.
                </p>}
                ko={<p>
                    PIBT는 스텝마다 칸 하나를 협상하고 — 그 스텝 하나가 근시안이다: agent는 모퉁이를 못 보고 몰려오는
                    무리를 예측하지 못하고, 칸이 이미 주장되기 시작해야만 반응한다. winPIBT(Okumura·Tamura·Défago,
                    IJCAI-20 MAPF workshop, arXiv:1905.10149)는 PIBT가 얼려 둔 유일한 축 — <em>시간</em> —을 따라 일반화한
                    같은 우선순위 규율이다. 이제 모든 agent는 잠정 시공간 경로 <InlineMath math="\Pi_i"/>를 들고 그것을{" "}
                    <InlineMath math="w"/> 스텝씩 연장하며 새 스텝을 우선순위 순서로 하나씩 확보하고, 낮은 우선순위는 높은
                    우선순위가 이미 확보한 것 너머를 결코 예약할 수 없다. <InlineMath math="w = 1"/>에서 이건 다시 PIBT다 —
                    여기서 공유하는 모든 시나리오의 고정된 합을 그대로 재현한다 — 그리고 창이 커질수록 예약은 점점 더 먼
                    미래까지 얼어붙고, 탐욕스러운 창은 결국 prioritized planning과 똑같이 군다. <InlineMath math="w \le 2"/>에서
                    협상이 풀어낸 그 정면 교환이 <InlineMath math="w \ge 3"/>에서는 정직하게 교착한다: 이 페이지는 그 갈래의
                    실패 두 종류를 하나의 연속된 노브로 보여준다.
                </p>}
            />

            <h2>{t("From Per-Step Negotiation to the Time Window", "스텝별 협상에서 시간 창으로")}</h2>
            <T
                en={<>
                    <p>
                        Read PIBT again and notice what was frozen by construction: a claim expires the moment it is
                        honored, nobody reserves anything for tomorrow. winPIBT thaws that decision. Each agent keeps a
                        provisional space-time path — the static BFS distance route from its current secured cell toward
                        its goal, padded with waits — and extends it{" "}
                        <InlineMath math="w"/> steps ahead at a time. The extension is registered whole but stays{" "}
                        <em>invisible</em> step by step: only the secured prefix up to{" "}
                        <InlineMath math="\ell_i"/> constrains anyone else, and securing step{" "}
                        <InlineMath math="t"/> drags whoever still stands on that cell forward — retroactively, one step at a
                        time — before the step is ever secured. Priority inheritance survives untouched; what changes is its
                        reach. A claim now covers a whole future interval: entering cell{" "}
                        <InlineMath math="v"/> at step{" "}
                        <InlineMath math="\tau"/> is invalid iff some other agent's <em>secured</em> path occupies{" "}
                        <InlineMath math="v"/> anywhere in{" "}
                        <InlineMath math="[\tau, \min(\beta, \ell_j)]"/>, or swaps with a fully-secured move.
                    </p>
                    <p>
                        And the window is capped by its own discipline: agent{" "}
                        <InlineMath math="i"/> may extend only to{" "}
                        <InlineMath math="\kappa"/>, the running minimum of what higher priorities have already secured —
                        lower priorities can never reserve beyond what higher ones covered. That single cap is why a big{" "}
                        <InlineMath math="w"/> behaves like prioritized planning: with a wide window the highest-priority
                        agent freezes its whole route before anyone else speaks, and the negotiation degenerates into
                        exactly the frozen reservations the search branch's first member suffered from. The knob is
                        continuous between the two regimes — at{" "}
                        <InlineMath math="w = 1"/> it is literally PIBT again (same pinned totals on every shared scenario),
                        and one click of the window chip on the demo below moves the same map between cooperation and
                        deadlock.
                    </p>
                </>}
                ko={<>
                    <p>
                        PIBT를 다시 읽어라 — 구성상 얼려져 있던 것이 보인다: claim은 받아들여지거나 거절되는 순간 만료되고,
                        아무도 내일을 위해 뭐도 예약하지 않는다. winPIBT는 그 결정을 녹인다. 각 agent는 잠정 시공간 경로를
                        들고 자기 현재 확보 셀에서 goal까지의 정적 BFS 거리 경로를 wait으로 채운 것 — 그리고 그것을{" "}
                        <InlineMath math="w"/> 스텝씩 연장한다. 연장은 통째로 등록되지만 스텝 단위로만 <em>보이게</em> 된다:
                        {" "}<InlineMath math="\ell_i"/>까지의 확보된 접두어만 남을 제약하고,{" "}
                        <InlineMath math="t"/>스텝을 확보하는 것은 그 칸에 아직 선 사람을 소급으로, 스텝씩, 확보되기 전에
                        끌어낸다. 우선순위 상속은 그대로 살아남고, 바뀌는 건 그 사정거리다. 이제 claim은 미래 구간 전체를
                        덮는다: 시각{" "}
                        <InlineMath math="\tau"/>에 칸{" "}
                        <InlineMath math="v"/>로 들어가는 것은 다른 agent의 <em>확보된</em> 경로가 어디든{" "}
                        <InlineMath math="[\tau, \min(\beta, \ell_j)]"/>에서{" "}
                        <InlineMath math="v"/>를 점유하거나 완전히 확보된 이동과 swap할 때만 invalid하다.
                    </p>
                    <p>
                        그리고 창은 자기 규율로 제한된다: agent{" "}
                        <InlineMath math="i"/>는 높은 우선순위들이 이미 확보한 것들의 running minimum{" "}
                        <InlineMath math="\kappa"/>까지밖에 연장할 수 없다 — 낮은 우선순위는 높은 쪽이 덮은 것을 결코
                        넘겨 예약하지 못한다. 이 제한 하나가 큰{" "}
                        <InlineMath math="w"/>를 prioritized planning처럼 만드는 이유다: 창이 넓으면 최고 우선순위 agent가
                        남들이 말하기 전에 경로 전체를 얼리고, 협상은 search 갈래 첫 회원이 앓았던 그 얼린 예약으로
                        퇴화한다. 노브는 두 국면 사이를 연속으로 잇는다 —{" "}
                        <InlineMath math="w = 1"/>에서 문자 그대로 다시 PIBT이고(공유 시나리오 전부 고정된 합 동일), 데모의 창
                        칩 클릭 한 번이 같은 맵을 협력과 교착 사이로 옮긴다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>The window is the axis, pinned on the same maps.</strong>{" "}
                            <InlineMath math="open01\_cross"/> pins [16, 17] = 33 at every window — when negotiation alone
                            suffices, foresight buys nothing. On <InlineMath math="open01\_swap"/> the cost total stays 30 at
                            every window but the makespan tightens from 17 (exactly PIBT's run) to 16 at{" "}
                            <InlineMath math="w = 2"/>: the window buys foresight, not speed — agent 1 ducks into row 9 one
                            step earlier because it can already see the reservation arriving. On{" "}
                            <InlineMath math="pocket01\_swap"/> the totals match PIBT's 4 + 6 = 10 at{" "}
                            <InlineMath math="w \le 2"/> (only the pocket route differs) and the same map honestly deadlocks
                            at <InlineMath math="w \ge 3"/> — greedy reservations wall off the pocket entrance before its
                            owner can reach it. Same map, same priorities; only the window moved.
                        </li>
                        <li>
                            <strong>Cheaper per step than PIBT's negotiation.</strong> One{" "}
                            <InlineMath math="\mathsf{winpibt}"/> call covers a whole window: on{" "}
                            <InlineMath math="pocket01\_swap"/> the pinned run spends 9 decision calls where PIBT spent 14
                            (2 × 7 — one per agent per step). The metric counts exactly these calls, top-level and inherited;
                            there is still no search frontier of its own — only a fixed tie-break space-time A* inside one
                            call, and the static BFS distance fields computed once up front.
                        </li>
                        <li>
                            <strong>Reachability, not completeness — same condition as PIBT.</strong> On graphs where every
                            adjacent pair of free cells lies on a simple cycle, with a finite window, every agent reaches its
                            goal in finite time (the paper's Theorem 4.3) — for the same reason PIBT survives: an edge on no
                            cycle gives an agent with a reserved cell nowhere to retreat to. <InlineMath math="tee01"/> and{" "}
                            <InlineMath math="corridor01"/> fail honestly at every window, and budget exhaustion is evidence,
                            never proof (the pinned corridor run reports its 749 calls honestly).
                        </li>
                        <li>
                            <strong>Two parameters, everything else pinned.</strong> The window{" "}
                            <InlineMath math="w"/> and the honest step budget are the only knobs; what the paper leaves free
                            or random — the{" "}
                            <InlineMath math="\varepsilon_i"/> values, the ideal path's tie-breaks, the search heap's order —
                            is pinned deterministically so all three language mirrors produce identical traces.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>창이 축이고, 같은 맵 위에서 수치로 고정된다.</strong>{" "}
                            <InlineMath math="open01\_cross"/>는 모든 창에서 [16, 17] = 33 — 협상만으로 충분한 곳에서 선견은
                            아무것도 사지 않는다. <InlineMath math="open01\_swap"/>에서 비용 합은 모든 창에서 30인데 makespan은
                            17(PIBT 실행과 정확히 동일)에서 <InlineMath math="w = 2"/>에서 16으로 조여진다: 창이 산 것은 속도가
                            아니라 선견이다 — agent 1은 도착할 예약을 미리 볼 수 있어서 한 스텝 일찍 row 9로 몸을 낮춘다.{" "}
                            <InlineMath math="pocket01\_swap"/>에서 합은 <InlineMath math="w \le 2"/>에서 PIBT의 4 + 6 = 10과
                            같고(주머니 경로만 다르고) 같은 맵이 <InlineMath math="w \ge 3"/>에서 정직하게 교착한다 — 탐욕스러운
                            예약들이 주인이 도달하기 전에 주머니 입구를 봉쇄한다. 같은 맵, 같은 우선순위; 움직인 건 창뿐이다.
                        </li>
                        <li>
                            <strong>스텝당 비용은 PIBT의 협상보다 싸다.</strong> <InlineMath math="\mathsf{winpibt}"/> 호출
                            하나는 창 전체를 커버한다: <InlineMath math="pocket01\_swap"/>에서 고정된 실행은 결정 호출 9회를
                            쓰고 PIBT는 14회(2 × 7 — 스텝마다 agent마다 한 번)를 썼다. metric은 정확히 이 호출들(top-level과
                            상속 전부)을 센다; 여전히 자체 search frontier는 없고 — 호출 안의 고정 tie-break 시공간 A*, 그리고
                            사전에 한 번 계산되는 정적 BFS 거리 필드뿐이다.
                        </li>
                        <li>
                            <strong>완전성이 아니라 reachability — 조건은 PIBT와 같다.</strong> 인접한 free 칸 쌍이 모두 simple
                            cycle 위에 있는 그래프에서, 유한한 창이면, 모든 agent가 유한한 시간에 goal에 도달한다(논문의 정리
                            4.3) — 그리고 이유는 PIBT와 같다: 어떤 cycle에도 없는 간선은 예약된 칸을 가진 agent에게 물러날 곳을
                            주지 않는다. <InlineMath math="tee01"/>과 <InlineMath math="corridor01"/>은 모든 창에서 정직하게
                            실패하고, budget 소진은 증거이지 증명이 아니다(고정된 corridor 실행은 749 호출을 정직하게 보고한다).
                        </li>
                        <li>
                            <strong>파라미터 둘, 나머지는 전부 고정.</strong> 창{" "}
                            <InlineMath math="w"/>와 정직한 step budget이 유일한 노브이고, 논문이 자유롭거나 랜덤으로 남긴 것({" "}
                            <InlineMath math="\varepsilon_i"/> 값, 이상적 경로의 tie-break, 탐색 힙의 순서)은 전부 결정론적으로
                            고정돼 세 언어 미러가 동일한 trace를 만든다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The paper's Algorithms 1 and 2, mirrored line for line. Priorities are PIBT's exactly — standing on
                        the goal resets{" "}
                        <InlineMath math="p_i"/> to{" "}
                        <InlineMath math="\varepsilon_i"/>, travelling increments it — and agents extend their provisional
                        paths in decreasing priority order, each capped by what higher priorities already secured.
                    </p>
                    <BlockMath math="\varepsilon_i = \frac{k - 1 - i}{k}, \qquad p_i[t] = \begin{cases} \varepsilon_i & \pi_i[t] = g_i \\ p_i[t-1] + 1 & \text{otherwise} \end{cases}"/>
                    <Terms items={[
                        ["\\Pi_i", <>agent i's provisional space-time path — cells registered up to the secured length <InlineMath math="\ell_i"/>. Positions at time t are <InlineMath math="\Pi_i[t]"/>; beyond{" "}
                            <InlineMath math="\ell_i"/> nothing is decided yet, and that is exactly what the disentangled condition encodes</>],
                        ["\\alpha = \\min(t + w, \\kappa)", <>the extension target for this round: the window opens to t + w but is capped by κ, the running minimum of higher priorities' secured lengths — lower priorities may never reserve beyond what higher ones covered. The highest-priority agent's α is always exactly t + w</>],
                        ["\\beta", <>the prophetic timestep fixed at entry: max(α, every registered path length). Validity of a new step checks other agents' reservations only up to min(β, ℓ_j) — beyond what they secured nothing exists to collide with</>],
                        ["ideal path", <>what the paper leaves free; pinned here as the static BFS parent-chain route padded with waits at the goal (truncated to the horizon), and if that is not disentangled from the visible reservations, a space-time A* over (cell, step) states with fixed tie-breaks</>],
                        ["secure", <>registering makes a step exist; securing makes it visible. Securing step t forces anyone parked on that cell out of it first — retroactively for occupants whose secured end is earlier than t − 1, by inheritance for the one parked exactly at t − 1. A failed inheritance retracts the unsecured suffix and replans from the last secured step</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        논문의 Algorithm 1과 2를 라인 그대로 옮겼다. 우선순위는 정확히 PIBT의 것이다 — goal 위에 서면{" "}
                        <InlineMath math="p_i"/>가{" "}
                        <InlineMath math="\varepsilon_i"/>로 리셋되고 이동 중엔 증가하고 — agent들은 잠정 경로를 우선순위
                        내림차순으로 연장하고, 각 연장은 높은 우선순위가 이미 확보한 것으로 제한된다.
                    </p>
                    <BlockMath math="\varepsilon_i = \frac{k - 1 - i}{k}, \qquad p_i[t] = \begin{cases} \varepsilon_i & \pi_i[t] = g_i \\ p_i[t-1] + 1 & \text{그 외} \end{cases}"/>
                    <Terms items={[
                        ["\\Pi_i", <>agent i의 잠정 시공간 경로 — 확보된 길이 <InlineMath math="\ell_i"/>까지 등록된 셀들. 시각 t의 위치는 <InlineMath math="\Pi_i[t]"/>;{" "}
                            <InlineMath math="\ell_i"/>너머에서는 아직 아무것도 결정되지 않았고, 바로 그게 disentangled 조건의 내용이다</>],
                        ["\\alpha = \\min(t + w, \\kappa)", <>이번 라운드 연장 목표: 창은 t + w까지 열리지만 κ로 제한된다 — 높은 우선순위들의 확보된 길이의 running minimum. 낮은 우선순위는 높은 쪽이 덮은 것을 넘겨 결코 예약하지 못한다. 최고 우선순위의 α는 항상 정확히 t + w</>],
                        ["\\beta", <>진입에서 고정되는 예언적 시각: max(α, 모든 등록 경로 길이 − 1). 새 스텝의 유효성은 다른 agent의 예약을 min(β, ℓ_j)까지만 검사한다 — 그들이 확보한 것 너머에는 충돌할 게 아직 존재하지 않는다</>],
                        ["ideal path", <>논문이 자유로 남긴 자리; 여기서 고정하면 goal까지의 정적 BFS parent-chain 경로를 wait으로 채운 것(지평선까지 자른다)이고, 그게 보이는 예약들과 disentangled가 아니면 고정된 tie-break의 (셀, 스텝) 상태 위 시공간 A*</>],
                        ["secure", <>등록은 스텝을 존재하게 하고, 확보는 보이게 한다. 스텝 t를 확보하면 그 칸에 선 사람을 먼저 밀어낸다 — 확보된 끝이 t − 1보다 이른 사람은 소급으로 한 스텝씩, 정확히 t − 1에 선 사람은 상속으로. 실패한 상속은 확보되지 않은 접미어를 되돌리고 마지막 확보 스텝에서 다시 계획한다</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── per instance, once ───────────────────────────────────────────────
D1  DIST[g]: static BFS distance table from each goal over free cells   # computed once, ignores all agents
D2  ε_i ← (k−1−i)/k                                                     # distinct base priorities, agent 0 highest
# ── every round t: one synchronized extension round (Algorithm 2) ─────
while some agent is not on its goal and the budget holds:
    p_i ← ε_i if π_i[t] = g_i else p_i + 1                    # priority update — PIBT's rule verbatim
    κ ← 0
    for i in agents sorted by DECREASING p (values distinct ⇒ order total):
        if ℓ_i ≤ t:                                           # path not registered beyond the current step
            α ← t + w                     (highest priority)   # the window opens
            α ← min(t + w, κ)             (everyone else)      # capped by what higher priorities secured
            winpibt(i, α)                                     # top-level call — its verdict ends here
        κ ← ℓ_i (highest) else min(κ, ℓ_i)                     # running minimum of secured lengths
    t ← t + 1
# winpibt(i, α): the paper's Algorithm 1
    calls += 1; if ℓ_i ≥ α: return valid                       # already registered beyond α
    β ← max(α, every registered path length − 1)               # prophetic timestep, FIXED at entry
    Π ← ideal path from (π_i[ℓ_i], step ℓ_i): the static BFS parent chain padded with waits —
        if disentangled from every VISIBLE reservation; else a pinned space-time A* over (cell, step)
    if no valid walk exists: pin waits to α and return invalid  # copeStuck — the claimant backtracks
    register Π's steps ℓ_i+1 .. α at once                      # only the secured prefix is visible
    for t from ℓ_i + 1 up to α:                                # secure the new steps one by one
        ℓ_i ← t                                                # the step becomes visible right now, not before
        while some agent j (index order) ends on π_i[t] with ℓ_j < t−1:
            winpibt(j, ℓ_j + 1)                                # retroactive: extend them one step (verdict ignored)
        if occupant j is parked exactly at step t − 1 on π_i[t]:
            if not winpibt(j, t): pop the unsecured suffix, replan from the last secured step, retry step t
        if π_i[t] = g_i and t < α: re-plan the tail FROM the goal   # hard mode — goals are fixed
    return valid`}
            />
            <T
                en={<ol>
                    <li>Priorities update first (goal resets to ε, travelling increments), then agents whose secured path
                        does not yet cover step t extend in decreasing priority order. The highest-priority extension opens
                        the full window; everyone else is capped by κ — that cap is what makes a wide window behave like
                        prioritized planning.</li>
                    <li>A call fixes β at entry and computes an ideal path from the agent's secured end: the static BFS
                        parent-chain route padded with waits, tried first at every popped state; only when it collides with
                        visible reservations does the pinned space-time search fan out. Success means surviving to step β.</li>
                    <li>The whole extension registers at once but secures step by step — visibility is the secured length ℓ_j,
                        not registration. Entering v at τ is invalid iff a secured path occupies v anywhere in [τ, min(β, ℓ_j)]
                        or swaps with a fully-secured move; beyond ℓ_j nothing exists to collide with yet.</li>
                    <li>Securing step t drags occupants forward: anyone whose secured end sits on the cell earlier than t − 1
                        is extended one step at a time (their verdict ignored — they just need to be off that cell), and the
                        occupant parked exactly at t − 1 inherits the claim like PIBT. A failed inheritance retracts the
                        unsecured suffix, replans from the last secured step, and retries the same step.</li>
                    <li>Goals are fixed (classical MAPF, not iterative): an agent pushed off its goal re-issues the task —
                        once arrival is secured, the tail is re-planned FROM the goal so it parks as soon as waiting is
                        disentangled. And like PIBT, running out of budget is honestly "no solution found within budget",
                        never a proof.</li>
                </ol>}
                ko={<ol>
                    <li>우선순위를 먼저 갱신하고(goal에서 ε로 리셋, 이동 중엔 증가), 확보된 경로가 아직 스텝 t를 덮지 않는
                        agent들이 우선순위 내림차순으로 연장한다. 최고 우선순위의 연장은 창을 끝까지 열고 나머지는 κ로 제한된다 —
                        그 제한이 넓은 창을 prioritized planning처럼 만드는 것이다.</li>
                    <li>호출은 진입에서 β를 고정하고 확보된 끝에서 이상 경로를 계산한다: wait으로 채운 정적 BFS parent-chain
                        경로를 pop된 모든 상태에서 먼저 시도하고, 보이는 예약과 충돌할 때만 고정된 시공간 탐색이 퍼진다. 성공은
                        스텝 β까지 생존이다.</li>
                    <li>연장은 통째로 등록되지만 스텝씩 확보되고 — 가시성은 등록이 아니라 확보 길이 ℓ_j다. τ에 v로 들어가는 것은
                        확보된 경로가 [τ, min(β, ℓ_j)] 어딘가에서 v를 점유하거나 완전히 확보된 이동과 swap할 때만 invalid하고,
                        ℓ_j 너머엔 충돌할 게 아직 없다.</li>
                    <li>스텝 t를 확보하면 점유자들을 앞으로 끌어당긴다: 확보된 끝이 t − 1보다 이른 칸에 그 칸이 있으면 소급으로
                        스텝씩 연장되고(판정은 무시 — 그냥 그 칸에서 벗어나면 된다), 정확히 t − 1에 선 점유자는 PIBT처럼 claim을
                        상속받는다. 실패한 상속은 확보 안 된 접미어를 되돌리고 마지막 확보 스텝에서 다시 계획해 같은 스텝을 재시도한다.</li>
                    <li>goal은 고정이다(classical MAPF이고 iterative가 아니다): goal에서 밀려난 agent는 작업을 다시 내고 — 도착이
                        확보되면 tail을 goal에서 다시 계획해 대기가 disentangled되는 순간 그 위에 주차한다. 그리고 PIBT처럼,
                        budget 소진은 정직하게 "budget 내 해 없음"이지 결코 증명이 아니다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    The guarantee is the same shape as PIBT's and its proof has the same spine: on dodgeable graphs — every
                    adjacent pair of free cells on a simple cycle — with a finite window, every agent eventually stands on
                    its goal. Once an agent holds the highest priority, no lower-priority agent can reserve anything beyond
                    what is already secured ahead of it (that is exactly what κ caps), so the reservations in front of the
                    top agent freeze; and at each step of its frozen route the cycle condition hands the occupant somewhere
                    to go. What the theorem still does not promise is simultaneity — everyone visits their goal, not
                    everyone ends there together — which is why this repository runs to simultaneous occupancy or honestly
                    stops at the budget. And what the window adds is a second failure mode layered on top of the first:
                    widen it and greedy reservations wall off pockets before their owners arrive (the demo's{" "}
                    <InlineMath math="pocket01\_swap"/> deadlock at{" "}
                    <InlineMath math="w \ge 3"/> is prioritized planning's failure, arriving continuously as the knob turns).
                </p>}
                ko={<p>
                    보장은 PIBT와 같은 모양이고 증명은 같은 줄기를 갖는다: dodgeable 그래프 — 인접한 free 칸 쌍이 모두 simple
                    cycle 위에 — 유한한 창에서, 모든 agent가 결국 goal 위에 선다. 어떤 agent가 최고 우선순위를 잡으면 낮은
                    우선순위들은 그 앞에 이미 확보된 것 너머를 예약할 수 없고(정확히 그게 κ의 제한이다) 그래서 top agent 앞의
                    예약들이 얼고, 얼린 경로의 각 스텝에서 cycle 조건이 점유자에게 갈 곳을 건넨다. 정리가 여전히 약속하지 않는
                    것은 동시성이다 — 모두가 goal을 <em>방문</em>할 뿐 함께 거기 머물진 않는다 — 그래서 이 저장소는 동시 점유까지
                    달리거나 budget에 정직하게 멈춘다. 그리고 창이 더하는 건 첫 번째 위에 겹치는 두 번째 실패 모드다: 넓히면
                    탐욕스러운 예약들이 주인이 도착하기 전에 주머니를 봉쇄하고(데모의 <InlineMath math="pocket01\_swap"/> 교착이{" "}
                    <InlineMath math="w \ge 3"/>에서 정확히 그거다 — 노브를 돌리면 prioritized planning의 실패가 연속적으로 도착한다).
                </p>}
            />
            <Proof title={t("Theorem 4.3 (reachability under a finite window)", "정리 4.3 (유한 창에서의 reachability)")}>
                <T
                    en={<p>
                        While some agent is not yet on its goal, exactly one of the active agents holds the highest priority.
                        When that agent extends, it extends to t + w along its static route — and here κ does the quiet work:
                        every lower-priority extension from now on is capped at what the top agent already secured, so no new
                        reservation can ever appear beyond the frozen horizon. Each step of the top agent's route then gets
                        secured one by one; where a cell it needs is occupied, the occupant inherits and must vacate — and on
                        a dodgeable graph the edge into that cell lies on a simple cycle, so the occupant has an escape that
                        is not the claimant's own cell (the same geometry as PIBT's Lemma 1, applied at every step of the
                        extension). The top agent walks home step by step; standing on its goal resets its priority below all
                        active agents and the next one takes over. Finite time follows because each phase ends within a
                        bounded number of rounds. What is <em>not</em> guaranteed: that everyone stays there — simultaneous
                        occupancy is what this repository runs to, honestly stopping at the budget when geometry offers no
                        cycle to retreat along.<InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        아직 goal에 안 도달한 agent가 있는 동안 active 중 정확히 하나가 최고 우선순위를 갖는다. 그 agent가
                        연장하면 t + w까지 정적 경로를 따라 연장하고 — 여기서 κ가 조용히 일한다: 이제부터 모든 낮은 우선순위
                        연장은 top agent가 이미 확보한 것으로 제한되므로 얼린 지평선 너머로 새 예약이 결코 생길 수 없다. 그러면
                        top agent 경로의 각 스텝이 하나씩 확보되고, 필요한 칸이 점유되면 점유자가 상속받아 비켜야 하고 — dodgeable
                        그래프에서 그 칸으로 들어가는 간선은 simple cycle 위에 있으니 점유자에게 claimant 자기 칸이 아닌 탈출구가
                        있다(연장의 매 스텝에 PIBT 보조정리 1의 그 기하가 적용된다). top agent는 스텝마다 집으로 걷고, goal 위에
                        서면 우선순위가 모든 active 아래로 리셋되어 다음 agent가 바통을 받는다. 각 단계가 유한한 라운드 안에 끝나니
                        유한한 시간이다. 보장되지 않는 건 모두가 거기 머무는 것이다 — 동시 점유는 이 저장소가 달리는 목표이고, 기하가
                        물러날 cycle을 주지 않을 때 budget에 정직하게 멈춘다.<InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, field-for-field what the
                    Python/C++ code below emits — and its window chip is the whole argument of this page.{" "}
                    <code>pocket01_swap</code>: at w = 2 agent 1 reaches the pocket entrance before agent 0's shorter window
                    reserves it, ducks into (0,4) and walks out the far side — [4, 6], PIBT's totals with a different route
                    inside the pocket; click the chip to w = 3 and watch the same map deadlock honestly, because agent 0's
                    three-cell window secures (1,2), (1,3), (1,4) before agent 1 can reach the entrance.{" "}
                    <code>open01_swap</code>: at w = 1 the run replays PIBT exactly ([14, 16], makespan 17); at w = 2 the
                    cost total is still 30 but the makespan tightens to 16 — foresight, not speed. <code>open01_cross</code>{" "}
                    pins [16, 17] at every window: where negotiation alone suffices the window changes nothing.{" "}
                    <code>tee01_head_on</code> and <code>corridor01_head_on</code> deadlock at every window — an edge on no
                    cycle defeats every member of this branch, wide windows only make the deadlock greedier. And{" "}
                    <code>maze01_two</code> threads the single gap for 66 again, in 35 calls where PIBT spent 34.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다 — 아래 Python/C++ 코드가 내뱉는 것과 필드 단위로
                    같은 엔진이고 — 그리고 그 창 칩이 이 페이지의 논지 전체입니다. <code>pocket01_swap</code>: w = 2에서는 agent
                    1이 agent 0의 짧은 창이 예약하기 전에 주머니 입구에 도달해 (0,4)로 몸을 숙여 반대편으로 나갑니다 — [4, 6],
                    PIBT의 합에 주머니 안 경로는 다릅니다. 칩을 w = 3으로 클릭하면 같은 맵이 정직하게 교착하는 걸 볼 수 있습니다 —
                    agent 0의 칸 3개 창이 agent 1이 입구에 도달하기 전에 (1,2), (1,3), (1,4)를 확보해 버리니까.{" "}
                    <code>open01_swap</code>: w = 1에서는 실행이 PIBT를 그대로 재현하고([14, 16], makespan 17), w = 2에서 비용 합은
                    여전히 30인데 makespan만 16으로 조여집니다 — 속도가 아니라 선견. <code>open01_cross</code>는 모든 창에서 [16, 17]에
                    고정됩니다: 협상만으로 충분한 곳에서 창은 아무것도 바꾸지 않습니다. <code>tee01_head_on</code>과{" "}
                    <code>corridor01_head_on</code>은 모든 창에서 교착합니다 — 어떤 cycle에도 없는 간선은 이 갈래의 모든 회원을
                    물리치고, 넓은 창은 교착을 더 탐욕스럽게 만들 뿐입니다. 그리고 <code>maze01_two</code>는 단일 gap을 다시 66으로
                    통과하고, PIBT가 34를 쓴 곳에 호출 35회를 씁니다.
                </p>}
            />
            <Sandbox label={t(
                "Live winpibt sandbox — the browser engine is a field-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; the chip cycles the window w = 1 → 2 → 3 and every edit re-plans and replays",
                "라이브 winpibt sandbox. 브라우저 엔진은 Python/C++ planner와 필드 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어지고, 칩은 창 w = 1 → 2 → 3을 순환합니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that
                    powers the live sandbox above is a third mirror — same fixed neighbor order (up/down/left/right), same{" "}
                    <InlineMath math="\varepsilon_i = (k-1-i)/k"/>, same BFS parent chain for the ideal path, same space-time
                    search heap ordered by f ascending then later step first then insertion order, so all three produce
                    field-identical traces on every scenario, which <code>check-engine-parity</code> verifies on every build.
                    The code below is the actual source, not an excerpt. Two implementation notes worth knowing: the search
                    heap's tie-break is part of the pinned contract (the paper leaves "compute the ideal path" free — this
                    repository pins it so three languages agree bit-for-bit), and paths are full-horizon like PIBT's page:
                    every agent's cell at every step up to the makespan, with waits as repeated cells;{" "}
                    <code>sum_of_costs</code> counts actual moves and <code>expanded_nodes</code> counts winpibt() calls.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러다 —
                    동일한 고정 이웃 순서(up/down/left/right), 동일한 <InlineMath math="\varepsilon_i = (k-1-i)/k"/>, 이상 경로용
                    동일한 BFS parent chain, f 오름차순 → 더 늦은 스텝 우선 → 삽입 순서로 정렬된 동일한 시공간 탐색 힙. 그래서 셋
                    모두 모든 시나리오에서 필드 단위로 동일한 trace를 만들고 빌드마다 <code>check-engine-parity</code>가 검증한다.
                    아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘 구현 노트 둘: 탐색 힙의 tie-break는 고정 계약의 일부다
                    (논문은 "이상적 경로를 계산"을 자유로 남기고 — 이 저장소는 세 언어가 비트 단위로 일치하게 그것을 고정한다). 그리고
                    경로는 PIBT 페이지처럼 full-horizon이다: makespan까지 모든 스텝의 모든 agent 셀, 대기는 반복된 셀이고,{" "}
                    <code>sum_of_costs</code>는 실제 이동을 세고 <code>expanded_nodes</code>는 winpibt() 호출을 센다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/decentralized/winpibt.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/decentralized/winpibt.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/decentralized/winpibt.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/decentralized/winpibt.hpp`,
                            },
                            {
                                name: "cpp/src/decentralized/winpibt.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/decentralized/winpibt.cpp`,
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
                    K. Okumura, Y. Tamura, F. Défago,{" "}
                    <a href="https://arxiv.org/abs/1905.10149" target="_blank" rel="noopener noreferrer">
                        <em>winPIBT: Extended Prioritized Algorithm for Iterative Multi-agent Path Finding</em>
                    </a>,
                    IJCAI-20 MAPF workshop (arXiv:1905.10149).
                </li>
                <li>
                    K. Okumura, M. Machida, F. Défago, Y. Tamura,{" "}
                    <a href="https://arxiv.org/abs/1901.11282" target="_blank" rel="noopener noreferrer">
                        <em>Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding</em>
                    </a>,
                    Artificial Intelligence 310 (2022) 103752 — the base algorithm this page generalizes.
                </li>
            </ol>
        </>
    )
}

export default Winpibt
