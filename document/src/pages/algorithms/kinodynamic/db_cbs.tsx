import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runDbCbs} from "../../../libs/algorithms/db_cbs";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/kinodynamic/db_cbs.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/kinodynamic/db_cbs.hpp?raw";
import cppImpl from "../../../../../cpp/src/kinodynamic/db_cbs.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents/vmax/δ에
// 반응해 다시 돈다. 라이브는 단일 rung(사다리 없이 지금 고른 δ 하나)로 돌리고, 수출 트레이스는
// config 기본 사다리 [1.5, 0.5]를 쓴다 — 답은 항상 0.5 칸이라 고정 수치와 일치한다.
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>, vmax: number[], _win: number,
                 delta: number): TraceEvent[] =>
    runDbCbs(map, tasks, vmax, {delta_start: delta, delta_end: delta, max_ct_expansions: 256})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계. 다섯 전부 timed이고 vmax가 정수인
// 것만 쓴다(maze01_two_timed의 0.25는 이 갈래가 거부한다). cross는 agent 1이 두 배 빨라 관성이
// 보이는 시나리오, 넷은 둘 다 단위 속도.
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross_timed", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]], vmax: [1, 2]},
    {name: "open01_swap_timed", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]], vmax: [1, 1]},
    {name: "pocket01_swap_timed", map: "pocket01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
    {name: "tee01_head_on_timed", map: "tee01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
    {name: "corridor01_head_on_timed", map: "corridor01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]], vmax: [1, 1]},
]

const REPO = "https://github.com/robotics-study/mrmp_introduction"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const DbCbs = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The kinodynamic branch's second member closes the genealogy by folding it: instead of bolting time onto an
                    existing plan, db-CBS — Discontinuity-Bounded Conflict-Based Search (Moldagalieva, Ortiz-Haro, Toussaint &
                    Hönig, arXiv 2309.16445) — makes the low level itself kinodynamic. The high level is still CBS verbatim:
                    best-first over a constraint tree, earliest conflict first, both children constrained. What changes is what a
                    single agent searches over. A state is no longer a cell but a pair <InlineMath math="(x, v)"/> — position and
                    velocity — and a transition lands where the action points, with acceleration bounded by one cell per step
                    squared per axis. Momentum enters the search itself, not a post-processing pass. And the paper's namesake knob,
                    the discontinuity bound <InlineMath math="\delta"/>, decides how strictly physics binds: at{" "}
                    <InlineMath math="\delta < 1"/> it is exact (the double integrator — momentum is law), and every step of slack
                    above that lets a landing sit further from the ideal landing and a velocity jump harder. The outer loop
                    tightens the bound rung by rung, exactly as the paper iterates; each rung is a fresh tree.
                </p>}
                ko={<p>
                    kinodynamic 갈래의 두 번째 회원은 계보를 접어서 닫는다: 기존 계획에 시간을 얹는 대신 db-CBS(Discontinuity-Bounded
                    Conflict-Based Search — Moldagalieva·Ortiz-Haro·Toussaint & Hönig, arXiv 2309.16445)는 저수준 자체를 kinodynamic으로
                    만든다. high level은 여전히 CBS 그대로다: constraint tree 위 최고가 우선, 가장 이른 충돌 먼저, 두 자녀에 제약. 바뀌는
                    것은 agent 하나가 무엇을 탐색하느냐다. 상태는 더 이상 셀이 아니라 쌍 <InlineMath math="(x, v)"/> — 위치와 속도 —이고
                    전이는 동작이 가리키는 곳에 착지하며 가속은 축별 스텝²당 한 칸으로 제한된다. 운동량이 후처리가 아니라 탐색 자체에
                    들어온다. 그리고 논문의 이름이 된 노브, 불연속성 경계 <InlineMath math="\delta"/>가 물리가 얼마나 강하게 조이는지
                    정한다: <InlineMath math="\delta < 1"/>에서 정확하고(이중 적분자 — 관성은 법), 그 위로 여유 한 칸씩은 착지를 이상적
                    착지에서 더 멀리 떨어뜨리고 속도 점프를 더 격렬하게 만든다. 바깥 루프는 논문이 반복 그대로 경계를 칸마다 조인다; 각
                    칸은 새로운 트리다.
                </p>}
            />

            <h2>{t("From Schedules to Motion as State", "스케줄에서 운동의 상태로")}</h2>
            <T
                en={<>
                    <p>
                        Read what changes when the low level gains momentum. A state is a pair{" "}
                        <InlineMath math="(x, v)"/>: a passable cell and an integer velocity bounded per axis by that agent's{" "}
                        <InlineMath math="v_{\max}"/> (the same field MAPF-POST reads — but here it must be an integer, because the
                        lattice quantizes integers only). From state <InlineMath math="(x, v)"/> a transition is legal iff some
                        action <InlineMath math="u"/> exists with <InlineMath math="|u - v| \le 1"/> per axis and{" "}
                        <InlineMath math="|u| \le v_{\max}"/>, landing within Manhattan <InlineMath math="\lfloor \delta \rfloor"/> of
                        the target cell with velocity within <InlineMath math="\lfloor \delta \rfloor"/> of the target velocity. At{" "}
                        <InlineMath math="\delta = 0.5"/> both slack terms vanish and this collapses to the exact double integrator
                        (proof below): land exactly at <InlineMath math="x + v'"/>, and since <InlineMath math="v_{\max} = 1"/> makes
                        acceleration vacuous, every 8-neighbor move plus wait is legal — momentum only exists at{" "}
                        <InlineMath math="v_{\max} \ge 2"/>.
                    </p>
                    <p>
                        The bound does double duty, and that is the honest reading of "discontinuity". It governs motion (how far a
                        landing may sit from the ideal landing) and it fattens constraints: a constraint{" "}
                        <InlineMath math="\langle c, t \rangle"/> now means "keep your position Manhattan-farther than{" "}
                        <InlineMath math="\lfloor \delta \rfloor"/> from cell <InlineMath math="c"/> at step <InlineMath math="t"/> — a
                        volume, not a point. At the tight rung it is exactly CBS's vertex constraint; looser rungs fatten it. And
                        conflicts themselves are sampled-state co-presence (same cell, same step) — point robots have no shape to
                        overlap, so edge and swap conflicts do not exist in this branch at any <InlineMath math="\delta"/>. That is the
                        page's sharpest lesson: a head-on swap that the search branch must detour becomes an honest pass-through here,
                        and the width-1 corridor that CBS honestly fails becomes solvable.
                    </p>
                </>}
                ko={<>
                    <p>
                        저수준에 운동량이 생기면 무엇이 바뀌는지 읽어라. 상태는 쌍 <InlineMath math="(x, v)"/>: 통과 가능한 셀과 축별{" "}
                        <InlineMath math="v_{\max}"/>로 제한된 정수 속도(MAPF-POST가 읽는 그 필드지만 여기선 정수여야 한다 — 격자가 정수만
                        양자화하니까). 상태 <InlineMath math="(x, v)"/>에서 전이는 어떤 동작 <InlineMath math="u"/>가 축별{" "}
                        <InlineMath math="|u - v| \le 1"/>이고 축별 <InlineMath math="|u| \le v_{\max}"/>를 만족하고 착지가 목표 셀에서
                        Manhattan으로 <InlineMath math="\lfloor \delta \rfloor"/> 이내, 속도가 목표 속도에서{" "}
                        <InlineMath math="\lfloor \delta \rfloor"/> 이내일 때만 합법이다. <InlineMath math="\delta = 0.5"/>에서 두 여유 항이
                        모두 사라지면 이건 정확한 이중 적분자로 줄어든다(증명은 아래): 정확히 <InlineMath math="x + v'"/>에 착지하고,{" "}
                        <InlineMath math="v_{\max} = 1"/>은 가속을 공전으로 만들어 모든 8-이웃 이동과 대기가 합법이다 — 관성은{" "}
                        <InlineMath math="v_{\max} \ge 2"/>에서야 온다.
                    </p>
                    <p>
                        이 경계는 이중 노동을 하고, 그게 "불연속성"의 정직한 읽기다. 운동(착지가 이상적 착지에서 얼마나 떨어져도 되는지)을
                        지배하고 동시에 제약을 부풀린다: 제약 <InlineMath math="\langle c, t \rangle"/>은 이제 "스텝 <InlineMath math="t"/>에서
                        셀 <InlineMath math="c"/>에서 Manhattan으로 <InlineMath math="\lfloor \delta \rfloor"/>보다 멀리 떨어지라" — 점이 아닌
                        부피. 꽉 조인 칸에서는 정확히 CBS의 vertex 제약이고, 느슨한 칸은 그것을 부풀린다. 그리고 충돌 자체는 표본 상태 공재(같은
                        셀, 같은 스텝)다 — 점 로봇은 겹칠 모양이 없으니 edge와 swap 충돌은 이 갈래에 어떤 <InlineMath math="\delta"/>에서도
                        존재하지 않는다. 이게 이 페이지의 가장 날카로운 교훈이다: search 갈래가 우회해야 했던 정면 맞교환이 여기선 정직한 통과가
                        되고, CBS가 정직하게 실패하던 폭 1 통로가 solvable이 된다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>A different motion model, not a refinement of the old one.</strong> At{" "}
                            <InlineMath math="v_{\max} = 1"/> and <InlineMath math="\delta < 1"/> this is an eight-connected grid with
                            waits — not the search branch's four-connected grid. Swaps pass through at every{" "}
                            <InlineMath math="\delta"/>; a swap that CBS must detour (cost 30 on <code>open01_swap</code>) slides past here
                            for 28, and <code>corridor01_head_on</code>, honestly unsolvable in the search branch, is solvable here.
                        </li>
                        <li>
                            <strong>Momentum is real at <InlineMath math="v_{\max} = 2"/>.</strong> Acceleration stays one cell per step²
                            per axis no matter how fast the agent may go: the first step out of rest covers one cell, only later steps two.
                            On <code>open01_cross_timed</code> that ramp is what times the crossing.
                        </li>
                        <li>
                            <strong>The ladder is not a refinement.</strong> A looser bound fattens the constraint volume too — so rungs are
                            independent attempts, never monotone refinements of one another. Each rung starts a fresh tree from an
                            unconstrained root (constraints never cross a <InlineMath math="\delta"/> boundary), and the answer is whatever
                            the last succeeding rung solved; a ladder whose every rung died fails honestly with zeroed metrics.{" "}
                            <code>expanded_nodes</code> accumulates across every rung's tree, because every rung really paid its own
                            expansions.
                        </li>
                        <li>
                            <strong>The paper's third level is out of scope.</strong> The joint-space trajectory optimization (DDP seeded
                            with the search result, iterated under the shrinking bound) is not implemented here — exactly as MAPF-POST's LP
                            variants were not. And because the lattice never refines, this repository's honest claim ends at "complete on the
                            discretized model at <InlineMath math="\delta < 1"/>": tightening the ladder tightens dynamics and constraint
                            volume, never resolution — the paper's asymptotic-optimality limit is out of reach by construction.
                        </li>
                        <li>
                            <strong>The goal reads position only.</strong> A pop counts as the goal when no constraint binds the goal cell at
                            any step <InlineMath math="\ge t"/> — arrival is arrival, and the velocity component at arrival is unobserved (a
                            deliberate narrowing of the paper's state-valued <InlineMath math="x_f"/>). Occupancy persists after arrival: a
                            parked agent keeps its cell forever.
                        </li>
                        <li>
                            <strong>Honest stopping.</strong> An empty queue is the unsolvability verdict; budget exhaustion is not — it
                            reports "no solution within budget" and says so, with zeroed metrics. Being born inside a forbidden volume (a
                            step-0 constraint on your own start cell) is likewise unfixable: that instance is malformed for that rung.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>옛날 것의 개선이 아니라 다른 운동 모델.</strong> <InlineMath math="v_{\max} = 1"/>에{" "}
                            <InlineMath math="\delta < 1"/>에서 이건 대기가 있는 8-연결 격자다 — search 갈래의 4-연결이 아니다. swap은 어떤{" "}
                            <InlineMath math="\delta"/>에서도 통과하고, CBS가 우회해야 했던 맞교환(<code>open01_swap</code>에서 비용 30)은 여기서
                            28로 미끄러져 지나가며, search 갈래가 정직하게 실패하던 <code>corridor01_head_on</code>은 여기서 solvable이다.
                        </li>
                        <li>
                            <strong><InlineMath math="v_{\max} = 2"/>에서 관성은 실재다.</strong> agent가 얼마나 빨리 가든 가속은 축별 스텝²당 한
                            칸 그대로다: 정지에서 첫 스텝은 한 칸, 이후에만 두 칸. <code>open01_cross_timed</code>에서 그 램프가 교차의 때를 맞춘다.
                        </li>
                        <li>
                            <strong>사다리는 세밀화가 아니다.</strong> 느슨한 경계는 제약 부피도 부풀린다 — 그래서 칸들은 독립적인 시도이고 서로의
                            단조로운 개선이 결코 아니다. 각 칸은 제약 없는 출발점에서 새 트리를 시작하고(제약은 <InlineMath math="\delta"/> 경계를 넘지
                            않는다), 답은 성공한 마지막 칸이 풀어낸 것이고, 모든 칸이 죽은 사다리는 지표를 0으로 정직하게 실패를 보고한다.{" "}
                            <code>expanded_nodes</code>는 칸마다 자기가 실제로 낸 확장이므로 누적된다.
                        </li>
                        <li>
                            <strong>논문의 세 번째 층은 범위 밖.</strong> 조인트 공간 궤적 최적화(탐색 결과로 씨앗을 뿌린 DDP를 줄어드는 경계로 반복)는
                            여기서 구현되지 않는다 — MAPF-POST의 LP 변형이 그랬던 것과 똑같이. 그리고 격자는 결코 가늘어지지 않으므로 이 저장소의 정직한
                            주장은 "<InlineMath math="\delta < 1"/>의 이산화된 모델에서 완전"에서 끝난다: 사다리를 조이는 것은 운동과 제약 부피를
                            조이는 것이지 해석도를 조이는 것이 아니고 — 논문의 점근 최적성 극한은 구성상 손이 닿지 않는다.
                        </li>
                        <li>
                            <strong>goal은 위치만 읽는다.</strong> 어떤 스텝 <InlineMath math="\ge t"/>에서도 goal 셀을 조이는 제약이 없으면 pop이 goal이다
                            — 도착은 도착이고 도착 시의 속도 성분은 관찰되지 않는다(논문의 상태값 <InlineMath math="x_f"/>를 의도적으로 좁힌 것). 점유는
                            도착 후에도 지속된다: 정차한 agent는 자기 셀을 영원히 지킨다.
                        </li>
                        <li>
                            <strong>정직한 정지.</strong> 빈 큐는 불가능 판정이고 예산 소진은 아니다 — 예산 소진은 "예산 내 해 없음"을 그대로 보고하고
                            (지표 0) 그렇게 말한다. 태어날 때부터 금지된 부피 안에 있는 것(step 0의 start 셀 제약) 역시 되돌릴 수 없다: 그 인스턴스는 그
                            칸에서 malformed다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The procedure is the paper's iteration, read literally. For each rung of the ladder{" "}
                        <InlineMath math="[\delta_{\mathrm{start}}, \delta_{\mathrm{end}}]"/> — a fresh constraint tree; plan every agent
                        unconstrained on the state space, expand the earliest conflict (ties by cell row, col, then pair{" "}
                        <InlineMath math="i < j"/>) into both agents' children, and branch until a node's paths are pairwise
                        conflict-free. The low level is A* over states <InlineMath math="(x, v, t)"/> in absolute time:{" "}
                        <InlineMath math="g = t"/> never improves so a pop is the expansion,{" "}
                        <InlineMath math="f = t + \lceil \mathrm{Chebyshev}(x, \mathrm{goal}) / (v_{\max} + \lfloor \delta \rfloor) \rceil"/> —
                        admissible and consistent because no legal move shifts either axis by more than that — and every state is pushed
                        exactly once. Finiteness comes from a horizon no plan needs (proof below): past the last constrained step nothing
                        binds anymore, and every feasible continuation shortens to a simple state-path.
                    </p>
                    <BlockMath math="(x, v) \;\to\; (x', v') \iff \exists u:\ |u - v|_\infty \le 1 \;\wedge\; |u|_\infty \le v_{\max} \;\wedge\; \max\big(\mathrm{Manhattan}(x', x + u),\ \|v' - u\|_\infty\big) \le \lfloor \delta \rfloor"/>
                    <Terms items={[
                        ["(x, v)", <>the state — a passable cell and an integer velocity pair. The velocity never leaves the search: the executed artifact stays a cell sequence indexed by absolute step</>],
                        ["u", <>the action — a velocity command. Legal when it deviates from the current velocity by at most 1 per axis (acceleration) and stays inside ±v_max per axis; at floor(δ) = 0 the landing is exactly x + u and u becomes v'</>],
                        ["\\delta", <>the discontinuity bound. Its floor is the whole value: 0.5 and 1 share one lattice (floor 0 — exact double integrator), 1.5 opens a cell of landing slack and a velocity jump of one. The chip cycles 0.5 → 1 → 1.5, and the first click does nothing — that is the lesson</>],
                        ["\\langle c, t \\rangle", <>a constraint: keep your position Manhattan-farther than floor(δ) from cell c at step t. At the tight rung exactly CBS's vertex constraint; the volume fattens as the bound loosens</>],
                        ["h = \\lceil \\mathrm{Chebyshev} / (v_{\\max} + f) \\rceil", <>the heuristic — steps to the goal cell can never undercut it, because no legal move shifts either axis by more than vmax + floor(δ). Admissible and consistent</>],
                        ["\\mathrm{reachable}", <>flood fill over the move relation itself: from a reached cell every passable cell within per-axis distance reach_r = vmax + floor(δ) is one transition's landing away. At floor 0 and vmax 1 that is exactly the 8-neighborhood; wider bounds hop cells — flying over walls between sampled endpoints is legal by construction</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        절차는 논문의 반복을 문자 그대로 읽은 것이다. 사다리{" "}
                        <InlineMath math="[\delta_{\mathrm{start}}, \delta_{\mathrm{end}}]"/>의 각 칸마다 — 새로운 constraint tree, 모든 agent를
                        상태 공간에서 제약 없이 계획하고, 가장 이른 충돌(tie-break은 셀 row, 그다음 col, 그다음 쌍 <InlineMath math="i < j"/>)을 두
                        agent의 자녀로 분기하고, 한 노드의 경로들이 쌍별로 충돌 없어질 때까지 확장한다. 저수준은 절대 시간 위 상태{" "}
                        <InlineMath math="(x, v, t)"/> 위의 A*다: <InlineMath math="g = t"/>는 결코 개선되지 않으니 pop이 곧 확장이고,{" "}
                        <InlineMath math="f = t + \lceil \mathrm{Chebyshev}(x, \mathrm{goal}) / (v_{\max} + \lfloor \delta \rfloor) \rceil"/> — 어떤
                        합법 이동도 어느 축을 그 이상으로 옮기지 못해 admissible하고 consistent하다. 그리고 모든 상태는 정확히 한 번 push된다. 유한성은
                        계획이 필요 없는 지평선에서 온다(증명은 아래): 마지막 제약 스텝 너머에선 아무것도 조이지 않고, 모든 실행 가능한 연속은 단순 state-path로
                        줄어든다.
                    </p>
                    <BlockMath math="(x, v) \;\to\; (x', v') \iff \exists u:\ |u - v|_\infty \le 1 \;\wedge\; |u|_\infty \le v_{\max} \;\wedge\; \max\big(\mathrm{Manhattan}(x', x + u),\ \|v' - u\|_\infty\big) \le \lfloor \delta \rfloor"/>
                    <Terms items={[
                        ["(x, v)", <>상태 — 통과 가능한 셀과 정수 속도 쌍. 속도는 탐색을 떠나지 않지만 실행되는 산물은 절대 스텝으로 인덱스된 셀 수열로 남는다</>],
                        ["u", <>동작 — 속도 명령. 현재 속도에서 축별 최대 1만큼 벗어나고(가속) 축별 ±vmax 안에 머물면 합법; floor(δ) = 0이면 착지는 정확히 x + u이고 u가 v'가 된다</>],
                        ["\\delta", <>불연속성 경계. floor가 값의 전부다: 0.5와 1은 같은 격자(floor 0 — 정확한 이중 적분자)를 공유하고, 1.5는 착지 여유 한 칸과 속도 점프 하나를 연다. 칩은 0.5 → 1 → 1.5를 순환하고 첫 클릭은 아무 일도 하지 않는다 — 그게 교훈이다</>],
                        ["\\langle c, t \\rangle", <>제약: 스텝 t에서 위치를 셀 c에서 Manhattan으로 floor(δ)보다 멀리 유지하라. 꽉 조인 칸에서는 정확히 CBS의 vertex 제약이고, 경계가 느슨해질수록 부피가 부풀어 오른다</>],
                        ["h = \\lceil \\mathrm{Chebyshev} / (v_{\\max} + f) \\rceil", <>휴리스틱 — goal 셀까지의 스텝은 결코 이보다 작을 수 없다. 어떤 합법 이동도 어느 축을 vmax + floor(δ) 이상으로 옮기지 못한다. admissible하고 consistent하다</>],
                        ["\\mathrm{reachable}", <>전이 관계 자체 위의 flood fill: 도달한 셀에서 축별 거리 reach_r = vmax + floor(δ) 이내의 모든 통과 가능한 셀이 전이 한 번의 착지다. floor 0에 vmax 1이면 정확히 8-이웃; 더 넓은 경계는 셀을 건너뛴다 — 표본 endpoint 사이 벽 위를 날아가는 것은 구성상 합법이다</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── the ladder: one fresh CBS per rung, left to right ──────────────────────
1  for delta in [delta_start, delta_end]:   # the ladder tightens, never loosens
2    f = floor(delta)                       # lattice slack: <1 -> exact, >=1 that many cells
3    fresh CT; root plans every agent with _plan_one at this rung's f
4    best-first: pop lowest (sum-of-arrival-steps, creation order); no conflict -> solved
5    conflict = earliest step with co-presence (ties: cell row, col, pair i < j) — vertex only
6    branch on both agents; a child whose sub-search dies never enters the queue
7  the LAST succeeding rung's solution is the answer (anytime); all-died is an honest failure
# ── _plan_one: db-A* over (cell, velocity) in absolute time ─────────────────
8  born inside a forbidden volume (step-0 constraint on the start cell)? -> this rung dies
9  horizon = constrained_until + |reachable| x (2*vmax+1)^2   # no plan needs one step more
10 frontier heap keyed (f, seq); f = t + ceil(Chebyshev(cell, goal) / (vmax + f))
11 pop: expanded += 1; goal iff cell == goal AND no constraint binds goal at ANY step >= t
12 successors: every state reachable in one legal transition, canonical sorted order —
   deduped set, each pushed exactly once (g == t never improves)
13 a successor state is blocked when any constraint binds its cell at its step`}
            />
            <T
                en={<ol>
                    <li>The high level is CBS verbatim — the same best-first tree as the search branch's CBS, keyed by sum-of-arrival-steps
                        with creation-order tie-breaks. The first conflict-free node popped is jointly optimal at this rung's model, for the
                        same old reason: every node's paths are individually optimal under their constraints.</li>
                    <li>The low level reads a state, not a cell. A successor enumeration returns the whole deduplicated set of states one
                        legal transition away, in canonical lexicographic order (row, col, v_row, v_col) — that fixed order is part of what
                        makes Python, C++ and the browser engine produce field-identical traces.</li>
                    <li>The goal guard reads position only: a pop counts when no constraint binds the goal cell at any step ≥ t. Occupancy
                        persists after arrival (a parked agent keeps its cell), so late constraints still bite — that is what a vertex
                        constraint on a parked agent's cell means here.</li>
                    <li>Conflicts are sampled-state co-presence, ties broken by cell row, then col, then pair i &lt; j. There is no edge kind
                        at any δ: two agents trading cells across one step pass through each other, honestly and legally.</li>
                    <li>The velocity component never leaks into the output: the artifact stays a cell sequence indexed by absolute step — but
                        which sequences are findable is now decided by the dynamics. The replay interpolates those cells at each agent's vmax
                        exactly like MAPF-POST's playback.</li>
                </ol>}
                ko={<ol>
                    <li>high level은 CBS 그대로다 — search 갈래의 CBS와 같은 최고가 우선 트리, sum-of-arrival-steps 키에 생성 순서 tie-break. 처음
                        충돌 없는 노드가 pop되면 그 칸의 모델에서 jointly optimal이다. 같은 오래된 이유다: 모든 노드의 경로가 자기 제약 아래 개별적으로
                        최적이다.</li>
                    <li>저수준은 셀이 아니라 상태를 읽는다. successor 열거는 합법 전이 한 번으로 도달 가능한 상태들의 중복 없는 집합을 정준 사전식 순서(row,
                        col, v_row, v_col)로 돌려준다 — 그 고정된 순서가 Python·C++·브라우저 엔진이 필드 단위로 동일한 trace를 만드는 근거 중 하나다.</li>
                    <li>goal guard는 위치만 읽는다: 어떤 스텝 ≥ t에서도 goal 셀을 조이는 제약이 없으면 pop이 goal이다. 점유는 도착 후에도 지속되니(정차한
                        agent는 셀을 지킨다) 늦은 제약도 여전히 조인다 — 정차한 agent의 셀에 거는 vertex 제약이 여기서 무슨 뜻인지가 그것이다.</li>
                    <li>충돌은 표본 상태 공재이고 tie-break은 셀 row, 그다음 col, 그다음 쌍 i &lt; j. 어떤 δ에서도 edge 종류는 없다: 스텝 하나를 사이에 두고
                        셀을 맞바꾸는 둘은 서로를 통과한다 — 정직하게, 그리고 합법적으로.</li>
                    <li>속도 성분은 출력으로 새지 않는다: 산물은 절대 스텝으로 인덱스된 셀 수열로 남는다 — 다만 어떤 수열이 발견 가능한지를 이제 운동이 결정한다.
                        재생은 MAPF-POST의 재생과 똑같이 그 셀들을 agent별 vmax로 보간한다.</li>
                </ol>}
            />
            <Proof title={t("Proposition 1 (at floor(δ) = 0 the transition is the exact double integrator)", "정리 1 (floor(δ) = 0에서 전이는 정확한 이중 적분자다)")}>
                <T
                    en={<p>
                        At <InlineMath math="\lfloor \delta \rfloor = 0"/> the legality condition reads{" "}
                        <InlineMath math="\max(\mathrm{Manhattan}(x', x + u), |v' - u|_\infty) \le 0"/>. A max of two non-negative terms is at
                        most 0 only when both vanish: the Manhattan term forces <InlineMath math="x' = x + u"/> exactly, and{" "}
                        <InlineMath math="|v' - u|_\infty \le 0"/> forces <InlineMath math="v' = u"/>. Substituting, a transition is legal iff{" "}
                        <InlineMath math="|v' - v|_\infty \le 1"/> and <InlineMath math="|v'|_\infty \le v_{\max}"/>, and then{" "}
                        <InlineMath math="x' = x + v'"/> — position integrates velocity, velocity changes by at most one per axis per step. That
                        is the discrete double integrator verbatim; nothing is approximated, which is why the tight rung's plans are honest
                        executions and not sketches of them.<InlineMath math="\blacksquare"/>
                    </p>}
                    ko={<p>
                        <InlineMath math="\lfloor \delta \rfloor = 0"/>에서 합법 조건은{" "}
                        <InlineMath math="\max(\mathrm{Manhattan}(x', x + u), |v' - u|_\infty) \le 0"/>으로 읽힌다. 음이 아닌 항 두 개의 max가 0 이하인
                        것은 둘 다 0일 때뿐: Manhattan 항은 <InlineMath math="x' = x + u"/>를 정확히 강제하고 <InlineMath math="|v' - u|_\infty \le 0"/>은{" "}
                        <InlineMath math="v' = u"/>를 강제한다. 대입하면 전이는 <InlineMath math="|v' - v|_\infty \le 1"/>이고{" "}
                        <InlineMath math="|v'|_\infty \le v_{\max}"/>일 때만 합법이고 그때 <InlineMath math="x' = x + v'"/> — 위치는 속도를 적분하고
                        속도는 스텝마다 축당 최대 하나 변한다. 그게 이산 이중 적분자 그 자체고 아무것도 근사되지 않으므로, 꽉 조인 칸의 계획은 스케치가 아닌
                        정직한 실행이다.<InlineMath math="\blacksquare"/>
                    </p>}
                />
            </Proof>
            <Proof title={t("Proposition 2 (the horizon: no plan needs one step more)", "정리 2 (지평선: 어떤 계획도 한 스텝 더를 필요로 하지 않는다)")}>
                <T
                en={<p>
                    Fix a rung and one agent's constraint list, and let <InlineMath math="T"/> be the latest constrained step. Two facts make
                    the search finite without losing completeness. First: every visited cell lies in the flood fill — by induction from{" "}
                    <InlineMath math="x_0"/>, a landing is always a passable cell within per-axis distance{" "}
                    <InlineMath math="v_{\max} + f"/> of the current one, and the fill closes exactly that relation; if the goal is outside the
                    fill, no state-path reaches it at any speed and the rung fails honestly. Second: take any feasible state-path and let{" "}
                    <InlineMath math="\tau > G"/> be its first guarded arrival (<InlineMath math="G"/> = latest step whose constraint binds near
                    the goal — beyond <InlineMath math="T"/> no constraint binds anything, so a late enough arrival always passes). If{" "}
                    <InlineMath math="\tau > T + |S|"/> where <InlineMath math="S"/> is the whole state space (fill cells times{" "}
                    <InlineMath math="(2v_{\max}+1)^2"/> velocities), then the segment from step <InlineMath math="T"/> to step{" "}
                    <InlineMath math="\tau"/> holds more than <InlineMath math="|S|"/> states, so some state repeats:{" "}
                    <InlineMath math="(x, v)"/> at steps <InlineMath math="t_1 < t_2"/>, both ≥ <InlineMath math="T"/>. Cut the loop — delete the
                    segment between the repeats and splice: every constraint lives at a step ≤ <InlineMath math="T \le t_1"/>, so nothing that was
                    satisfied moves, and the spliced path is still legal because the state at <InlineMath math="t_2"/> equals the state at{" "}
                    <InlineMath math="t_1"/>. The arrival lands at <InlineMath math="\tau - (t_2 - t_1) \ge t_1 + 1 > T \ge G"/> — still guarded. So a
                    feasible path of length ≤ <InlineMath math="T + |S|"/> exists, the search explores every state up to exactly that horizon, and
                    completeness on the discretized model follows.<InlineMath math="\blacksquare"/>
                </p>}
                ko={<p>
                    한 칸과 agent 하나의 제약 목록을 고정하고 <InlineMath math="T"/>를 가장 늦은 제약 스텝이라 하자. 두 사실이 탐색을 완전성을 잃지 않고
                    유한하게 만든다. 첫째: 모든 방문 셀은 flood fill에 있다 — 귀납으로, 착지는 항상 현재 셀에서 축별 거리{" "}
                    <InlineMath math="v_{\max} + f"/> 이내의 통과 가능한 셀이고 fill은 정확히 그 관계를 닫고; goal이 fill 밖에 있으면 어떤 state-path도
                    어떤 속도으로도 거기에 닿지 못하므로 칸은 정직하게 실패한다. 둘째: 임의의 실행 가능한 state-path와 그 첫 guard 통과 도착{" "}
                    <InlineMath math="\tau > G"/>를 잡자(<InlineMath math="G"/> = goal 근처를 조이는 제약의 가장 늦은 스텝 —{" "}
                    <InlineMath math="T"/> 너머로는 아무것도 조이지 않으므로 충분히 늦은 도착은 항상 통과한다). 만약{" "}
                    <InlineMath math="\tau > T + |S|"/>이고 <InlineMath math="S"/>가 전체 상태 공간(fill의 셀 ×{" "}
                    <InlineMath math="(2v_{\max}+1)^2"/> 속도)이면 스텝 <InlineMath math="T"/>부터 <InlineMath math="\tau"/>까지 구간에{" "}
                    <InlineMath math="|S|"/>보다 많은 상태가 있으니 어떤 상태가 반복된다: 스텝 <InlineMath math="t_1 < t_2"/>(둘 다 ≥{" "}
                    <InlineMath math="T"/>)에서 같은 <InlineMath math="(x, v)"/>. 루프를 자른다 — 반복 사이 구간을 지우고 이어붙이면 모든 제약은 스텝 ≤{" "}
                    <InlineMath math="T \le t_1"/>에 있으니 만족된 것은 움직이지 않고, <InlineMath math="t_2"/>의 상태가 <InlineMath math="t_1"/>의
                    상태와 같으니 이어붙인 경로도 합법이다. 도착은 <InlineMath math="\tau - (t_2 - t_1) \ge t_1 + 1 > T \ge G"/>에 착지하고 — 여전히
                    guard를 통과한다. 따라서 길이 ≤ <InlineMath math="T + |S|"/>인 실행 가능한 경로가 존재하고, 탐색은 정확히 그 지평선까지 모든 상태를
                    훑으므로 이산화된 모델 위의 완전성이 따라온다.<InlineMath math="\blacksquare"/>
                </p>}
                />
            </Proof>

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    The guarantee is exactness on a discretized model and honesty about the rest. At δ &lt; 1 the motion model is the exact
                    discrete double integrator and the search is complete on it (the proof above): an empty queue means genuinely unsolvable for
                    that rung's physics, and the answer at each rung is optimal under exactly those dynamics — which is why{" "}
                    <code>open01_swap_timed</code> costs 28 where four-connected CBS pays 30 (the swap needs no wait here) while{" "}
                    <code>tee01_head_on_timed</code> costs 9: decelerating into the stem and back out costs a real step, and momentum does not
                    negotiate. What it cannot: no trajectory optimization — the paper's third level is out of scope, so nothing smooths the
                    staircase; no resolution refinement — tightening δ tightens dynamics and constraint volume, never the lattice, so the paper's
                    asymptotic-optimality limit is out of reach by construction; and no non-integer velocities — the lattice quantizes integers
                    only, and the sandbox says so on an error card when you spin a vmax chip to 0.25. The ladder's non-monotonicity is part of the
                    honesty too: looser rungs are not easier (their constraints are fatter), so a rung that fails can be preceded by one that
                    succeeds, and the answer always names the δ it was solved at.
                </p>}
                ko={<p>
                    보장은 이산화된 모델 위의 정확성과 나머지에 대한 정직함이다. δ &lt; 1에서 운동 모델은 정확한 이산 이중 적분자이고 탐색은 그 위에서 완전하다(증명은
                    위 정리 2): 빈 큐는 그 칸의 물리에서 진짜로 불가능이라는 뜻이고, 각 칸의 답은 바로 그 동역학 아래 최적이다 — 그래서 4-연결 CBS가 30을 내는{" "}
                    <code>open01_swap_timed</code>이 여기서 28이고(여기선 맞교환에 대기가 필요 없다), <code>tee01_head_on_timed</code>은 9다: 줄기로 감속해
                    들어가고 다시 나오는 데 실제 스텝 하나가 들고, 관성은 협상하지 않으니까. 못 하는 것: 궤적 최적화 없음 — 논문의 세 번째 층은 범위 밖이라 계단을
                    매끄럽게 하는 것은 아무것도 없고; 해석도 세밀화 없음 — δ를 조이면 운동과 제약 부피가 조여지지 격자가 가늘어지지는 않으므로 논문의 점근 최적성 극한은
                    구성상 손이 닿지 않고; 정수가 아닌 속도 없음 — 격자는 정수만 양자화하고, vmax 칩을 0.25로 돌리면 엔진은 error 카드로 그렇게 말한다. 사다리의
                    비단조성도 정직함의 일부다: 느슨한 칸이 더 쉽지는 않다(제약도 더 뚱뚱해진다) 그래서 실패한 칸 앞에 성공한 칸이 올 수 있고, 답은 언제나 풀린 δ를
                    이름으로 적시한다.
                </p>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what the Python/C++ code below emits.
                    Draw walls, drag endpoints, add agents; every edit re-plans instantly and replays from step 0 (the replay interpolates each
                    cell-to-cell move at that agent's vmax). The δ chip cycles the bound 0.5 → 1 → 1.5 — and notice the first click does nothing:
                    floor(δ) is the whole value, so 0.5 and 1 share one lattice; only 1.5 opens a cell of landing slack. The vmax chips cycle ×2 per
                    click (4 wraps to 0.25), and spinning one to a non-integer makes the engine refuse — honestly, on an error card, not by crashing.
                    The presets are the whole argument. <code>open01_cross_timed</code>: agent 1 at vmax 2 ramps out of rest (one cell per step, then
                    two) and the two individually-optimal state-paths already miss each other in time-space — no conflict fires at all; sum 25,
                    makespan 16. <code>open01_swap_timed</code>: the head-on swap the search branch detours for 30 slides through here on diagonally
                    offset lanes without a single wait — cost 28 = 14 + 14. <code>pocket01_swap_timed</code>: agent 1 zigzags through the pocket cells
                    at full speed (cost 8, no wait anywhere). <code>tee01_head_on_timed</code>: the same swap where deceleration into the stem and back
                    out costs a real step — one dwell, cost 9. And <code>corridor01_head_on_timed</code>: the width-1 corridor the search branch honestly
                    fails is solvable here — swapping is legal, but in width 1 someone must still yield a step; cost 9, makespan 5. Spin any vmax chip to
                    2 and watch the ramps bend every path: momentum is not negotiable at δ = 0.5.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다 — 아래 Python/C++ 코드가 내뱉는 것과 바이트 단위로 같은 엔진입니다. 벽을 그리고, endpoint를
                    끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획되고 재생은 스텝 0부터 다시 돕니다(재생은 셀-에서-셀 이동을 agent별 vmax로 보간합니다). δ 칩은
                    경계를 0.5 → 1 → 1.5로 순환하고 — 첫 클릭이 아무 일도 안 한다는 걸 눈으로 확인하세요: floor가 값의 전부라 0.5와 1은 같은 격자이고, 1.5에서만 착지
                    여유 한 칸이 열립니다. vmax 칩은 클릭마다 ×2 순환(4를 넘으면 0.25)이고 정수가 아닌 값으로 돌리면 엔진이 거부합니다 — 크래시가 아니라 error 카드로
                    정직하게. preset들이 이 페이지의 논지 전체입니다. <code>open01_cross_timed</code>: vmax 2인 agent 1은 정지에서 램프하고(스텝당 한 칸, 그다음 두 칸)
                    개별 최적 상태 경로 둘은 시공간에서 이미 서로를 비껴가 충돌이 아예 발생하지 않습니다 — 합 25, makespan 16. <code>open01_swap_timed</code>: search
                    갈래가 30을 내고 우회하던 정면 맞교환이 여기서 대각으로 어긋난 레인으로 대기 없이 미끄러져 지나갑니다 — 비용 28 = 14 + 14.{" "}
                    <code>pocket01_swap_timed</code>: agent 1이 주머니 셀을 전속력으로 지그재그 관통합니다(비용 8, 어디에도 대기 없음).{" "}
                    <code>tee01_head_on_timed</code>: 줄기로 감속해 들어가고 다시 나오는 데 실제 스텝 하나가 드는 같은 맞교환 — 대기 하나, 비용 9. 그리고{" "}
                    <code>corridor01_head_on_timed</code>: search 갈래가 정직하게 실패하던 폭 1 통로가 여기서 solvable입니다 — swap은 합법이지만 폭 1에서는 누군가는
                    여전히 스텝 하나를 양보해야 합니다; 비용 9, makespan 5. vmax 칩을 2로 돌리고 램프가 모든 경로를 굽히는 것을 보세요: δ = 0.5에서 관성은 협상 대상이
                    아닙니다.
                </p>}
            />
            <Sandbox label={t(
                "Live db-CBS sandbox — the browser engine is a byte-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; cycle δ and vmax live",
                "라이브 db-CBS sandbox. 브라우저 엔진은 Python/C++ planner와 바이트 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하고, δ와 vmax를 라이브로 순환해 보세요",
            )} presets={PRESETS} run={runLive} deltaHint={{
                en: "δ chips cycle 0.5 → 1 → 1.5 — and floor(δ) is the whole value: 0.5 and 1 behave identically (floor 0, the exact double integrator); only 1.5 opens a cell of landing slack and a one-step velocity jump",
                ko: "δ 칩은 0.5 → 1 → 1.5를 순환합니다 — 그리고 floor(δ)가 값의 전부입니다: 0.5와 1은 똑같이 동작하고(floor 0, 정확한 이중 적분자), 1.5에서만 착지 여유 한 칸과 스텝 하나 속도 점프가 열립니다",
            }}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser engine that powers the live sandbox
                    above is a third mirror — same canonical successor order (row, col, v_row, v_col), same heap tie-breaks by creation order, same
                    earliest-conflict rule (cell row, then col, then pair i &lt; j), so all three produce field-identical traces on every scenario,
                    which <code>check-engine-parity</code> verifies on every build. The code below is the actual source, not an excerpt. Two
                    implementation notes worth knowing: every pinned number in this repository lives on dyadic-rational ground — velocities are
                    integers and steps are integers, so heap keys and metrics land on exactly representable doubles across languages; and the recorded
                    traces run the default ladder [1.5, 0.5] while the live sandbox runs your single chosen δ — the answer always comes from a succeeding
                    rung (the exact one last), and <code>expanded_nodes</code> honestly accumulates what every rung paid.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째 미러다 — 같은 정준 successor 순서(row, col,
                    v_row, v_col), 같은 생성 순서 heap tie-break, 같은 가장 이른 충돌 규칙(셀 row, 그다음 col, 그다음 쌍 i &lt; j). 그래서 셋 모두 모든 시나리오에서
                    필드 단위로 동일한 trace를 만들고 빌드마다 <code>check-engine-parity</code>가 검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘 노트:
                    이 저장소의 모든 고정 수치는 이진 유리수 위에 산다 — 속도는 정수이고 스텝은 정수라 heap 키와 지표가 언어 간 정확히 표현 가능한 double에 착지한다. 그리고
                    기록된 trace는 기본 사다리 [1.5, 0.5]를 굴리고 라이브 sandbox는 당신이 고른 단일 δ를 굴린다 — 답은 언제나 성공한 칸(정확한 칸이 마지막)에서 나오고,{" "}
                    <code>expanded_nodes</code>는 모든 칸이 낸 만큼을 정직하게 누적한다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/kinodynamic/db_cbs.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/kinodynamic/db_cbs.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/kinodynamic/db_cbs.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/kinodynamic/db_cbs.hpp`,
                            },
                            {
                                name: "cpp/src/kinodynamic/db_cbs.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/kinodynamic/db_cbs.cpp`,
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
                    R. Moldagalieva, A. Ortiz-Haro, M. Toussaint & W. Hönig,{" "}
                    <a href="https://arxiv.org/abs/2309.16445" target="_blank" rel="noopener noreferrer">
                        <em>db-CBS: Discontinuity-Bounded Conflict-Based Search for Multi-Robot Kinodynamic Motion Planning</em>
                    </a>,
                    arXiv:2309.16445 (2023).
                </li>
            </ol>
        </>
    )
}

export default DbCbs
