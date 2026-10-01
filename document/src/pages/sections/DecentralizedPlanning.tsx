import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";

// decentralized 갈래 소개 페이지 — 계획이라는 매개체 자체를 버린 자리에서 문제를 다시
// 세운다. search 갈래가 우선순위 규율을 어떻게 실행 시간의 협상으로 다시 세우는지(PIBT),
// 무엇이 남고 무엇이 사라지는지 다룬다.
const DecentralizedPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The search branch ended with a decision procedure — an algorithm that always terminates
                    with a plan or with proof of impossibility. But notice what every algorithm in both previous
                    branches shares, from prioritized planning to dRRT*: the <em>plan</em> is the product. A path
                    exists before it is walked; execution merely replays what search already decided. This branch
                    drops that artifact entirely. Here nothing is planned offline — at each time step every agent
                    negotiates its next cell against the other agents' current cells, and whatever was decided a
                    moment ago has already become history. The question stops being "what path should this agent
                    follow" and becomes "who moves where, right now, and who must get out of the way".
                </p>}
                ko={<p>
                    search 갈래는 판정 절차로 끝났다 — 항상 계획으로 끝나거나 불가능함의 증거로 끝나는 알고리즘. 그런데
                    이전 두 갈래의 모든 알고리즘이 공유한 것을 주목하라: <em>계획</em>이 생산물이다. 경로는 걷기 전에
                    이미 존재하고, 실행은 탐색이 이미 결정한 것을 되돌릴 뿐이다. 이 갈래는 그 매개체를 통째로 버린다.
                    여기서는 오프라인으로 아무것도 계획되지 않는다 — 각 스텝마다 모든 agent가 다른 agent들의 현재 칸에
                    맞서 다음 칸을 협상하고, 잠깐 전에 결정된 것은 이미 과거가 된다. 질문은 더 이상 "이 agent는 어떤
                    경로를 따라야 하는가"가 아니고 "지금 이 순간 누가 어디로 가고, 누가 비켜야 하는가"다.
                </p>}
            />

            <h2>{t("The Problem, Rebuilt at the Time Step", "스텝에서 다시 세운 문제")}</h2>
            <T
                en={<>
                    <p>
                        The stage is the same graph <InlineMath math="G = (V, E)"/> with{" "}
                        <InlineMath math="k"/> agents and fixed goals — MAPF exactly as the search branch
                        defined it. What changes is what counts as a solution. There is no tuple of paths to
                        produce; there is a decision rule that, given every agent's current cell and current
                        priority at time <InlineMath math="t"/>, produces everyone's cell at{" "}
                        <InlineMath math="t + 1"/> in one synchronized round. Two invariants hold by
                        construction rather than by search: two agents can never claim the same cell (an already-claimed
                        cell is simply not selectable), and a swap — two agents trading cells along an edge in one
                        step — is ruled out by design, because an agent forced to vacate may never step into its
                        claimant's cell. The instance counts as solved when every agent simultaneously stands on its
                        goal; the run then stops and the executed paths are read off the history.
                    </p>
                    <p>
                        One honesty note that changes what "complete" can even mean here: the search branch's
                        decision procedure could certify impossibility. A per-step negotiator cannot — running out
                        of step budget is evidence, not proof. What this branch guarantees instead is a weaker,
                        local property with a global consequence: on graphs where every adjacent pair of free cells
                        lies on a common cycle, negotiation alone gets everyone home regardless of priority order;
                        on tree-shaped maps the same run deadlocks and says so honestly.
                    </p>
                </>}
                ko={<>
                    <p>
                        무대는 같은 그래프 <InlineMath math="G = (V, E)"/>에 <InlineMath math="k"/>대의 agent,
                        고정된 goal — search 갈래가 정의한 그대로의 MAPF다. 바뀌는 것은 무엇이 해로 인정되느냐다.
                        생산할 경로들의 튜플이 없다. 대신 시간 <InlineMath math="t"/>에서 모든 agent의 현재 칸과
                        현재 우선순위가 주어지면 모두의 <InlineMath math="t + 1"/> 칸을 하나의 동기화된 라운드로
                        만들어내는 결정 규칙이 있을 뿐이다. 두 불변식이 탐색이 아니라 구성으로 성립한다: 두 agent가
                        같은 칸을 주장하는 일은 불가능하고(이미 주장된 칸은 선택지에서 그냥 없다), swap(한 스텝에
                        두 agent가 간선을 따라 칸을 맞바꾸는 것)은 설계로 배제된다. 비키라는 지시를 받은 agent가
                        자기 claimant의 칸으로 들어가는 일이 허용되지 않기 때문이다. 모든 agent가 동시에 goal 위에
                        서면 인스턴스는 풀린 것이고, 실행은 멈추고 실행된 경로들이 역사에서 읽힌다.
                    </p>
                    <p>
                        "완전"이 여기서 무슨 뜻일 수 있는지 바꾸는 정직 노트 하나: search 갈래의 판정 절차는 불가능함을
                        인증할 수 있었다. 스텝별 협상자는 못 한다 — 스텝 예산이 바닥나는 것은 증거지 증명이 아니다.
                        이 갈래가 대신 보장하는 것은 약하지만 국소적인 성질이고 그건 전역 결과를 낳는다: 인접한 free
                        칸 쌍이 모두 공통 cycle 위에 있는 그래프에서는 우선순위 순서와 무관하게 협상만으로 모두가 집에
                        도착하고, 트리 모양 맵에서는 같은 실행이 교착에 걸리고 그걸 정직하게 말한다.
                    </p>
                </>}
            />

            <h2>{t("Why Drop the Plan", "계획을 버리는 이유")}</h2>
            <T
                en={<p>
                    Because a plan is exactly what the search branch pays for in full. Joint-space search explores{" "}
                    <InlineMath math="|V|^{k}"/>; CBS branches a constraint tree on every conflict; even the
                    decoupled pole still produces whole space-time paths before anyone moves, and its incompleteness
                    came from freezing those paths into reservations nobody may edit. Dropping the plan cuts the
                    knot: per step, negotiation costs a constant number of decisions per agent — no joint space is
                    ever built, nothing is reserved for the future, and an agent's "plan" is just the distance field
                    it walks down. What that buys is real: cost linear in agents per step, reactions to whatever the
                    world turned out to be, no offline phase at all. What it costs is equally real: optimality is off
                    the table by design (an agent always steps toward its own goal; nobody ever computes a cheaper
                    joint detour), and completeness survives only as that cycle condition — where geometry offers no
                    pocket to duck into, this branch does not fail gracefully with a plan, it simply stops. The two
                    branches are complements in the honest sense: search trades cost for guarantees, negotiation
                    trades guarantees for cheapness.
                </p>}
                ko={<p>
                    계획이야말로 search 갈래가 대가를 통째로 치르는 것이 정확히 그거이기 때문이다. joint-space 탐색은{" "}
                    <InlineMath math="|V|^{k}"/>를 탐색하고, CBS는 conflict마다 constraint tree를 분기하며, decoupled
                    극단조차 아무도 움직이기 전에 space-time 경로 전체를 생산하고 그 경로를 아무도 편집 못 할 예약으로
                    얼려버린 데서 불완전성이 왔다. 계획을 버리면 매듭이 잘린다: 스텝마다 협상은 agent당 상수 개의
                    결정이고, joint 공간은 구성되지 않으며 미래에 뭐가 예약되지 않고, agent의 "계획"은 그냥 자기가
                    내려가는 거리 필드다. 그게 사는 것은 진짜다 — 스텝당 agent에 선형인 비용, 세상이 어떻게 됐든 하는
                    반응, 오프라인 단계의 부재. 대가도 똑같이 진짜다: 최적성은 설계로 테이블에서 내려온다(agent는 항상
                    자기 goal을 향해 한 칸 나가고, 아무도 더 싼 joint 우회를 계산하지 않는다). 그리고 완전성은 그 cycle
                    조건으로만 살아남는다 — 기하가 몸을 피울 주머니를 제공하지 않는 곳에서, 이 갈래는 계획으로 우아하게
                    실패하는 게 아니라 그냥 멈춘다. 두 갈래는 정직한 의미로 보완재다: search는 보장을 비용과 바꾸고,
                    협상은 비용을 보장과 바꾼다.
                </p>}
            />

            <h2>{t("What Survives Without a Plan", "계획 없이 남는 것")}</h2>
            <T
                en={<p>
                    Read carefully and the search branch's priority discipline survives intact — it just stops being a
                    preprocessing step. The fixed serving order becomes floating-point priorities: an agent standing
                    on its goal resets to a small base value, one still travelling increments every step, so any
                    active agent always outranks any parked one and within each group the pinned index order rules.
                    The planned path is replaced by nothing more than a static BFS distance field from each goal —
                    an agent's "preference" at a step is just which neighbor cell sits closest to its goal. And the
                    branch's whole primitive machinery collapses into one inheritance rule: claim a cell, and whoever
                    still occupies it inherits your priority and must vacate or make your claim fail. Push, swap,
                    rotate — three named maneuvers with their biconnected decompositions and junction clearings — all
                    compress into "vacate or the claim backtracks". What doesn't survive is exactly what those
                    primitives were built for: on a width-1 corridor with no pocket anywhere, head-on agents still
                    deadlock here. The search branch repaired around that geometry with machinery; this branch lets
                    it stand, and reports honestly.
                </p>}
                ko={<p>
                    자세히 읽으면 search 갈래의 우선순위 규율은 그대로 살아남는다 — 전처리 단계가 아니게 됐을 뿐이다.
                    고정된 서비스 순서는 부동소수점 우선순위가 된다: goal 위에 선 agent는 작은 기본값으로 리셋되고 아직
                    이동 중인 agent는 스텝마다 증가하니, active인 agent는 parked인 agent에게 항상 이기고 각 그룹 안에서는
                    고정된 index 순서가 군다. 계획된 경로는 goal에서 오는 정적 BFS 거리 필드 하나로 대체된다 — 한 스텝에서
                    agent의 "선호"는 그냥 어느 이웃 칸이 자기 goal에 가장 가까운지일 뿐이다. 그리고 갈래 전체의 primitive
                    기계장치 하나는 상속 규칙 하나로 압축된다: 칸을 주장하면, 아직 그 위에 있는 점유자가 네 우선순위를
                    상속받아 비켜야 하거나 네 주장을 실패시켜야 한다. push, swap, rotate — biconnected 분해와 junction
                    정리까지 동원된 세 개의 이름난 기동이 전부 "비키거나 주장이 backtrack되거나"로 줄어든다. 살아남지 못한
                    것은 정확히 그 primitive들이 지어졌던 것을 위한 자리다: 어디에도 주머니 없는 폭 1 통로에서 정면으로
                    만난 agent는 여기서도 교착된다. search 갈래는 그 기하를 기계장치로 돌아 수리했고, 이 갈래는 그대로
                    두되 정직하게 보고한다.
                </p>}
            />

            <h2>{t("What Is Written, and What Comes Next", "집필된 것과 그다음")}</h2>
            <T
                en={<p>
                    One member is written and implemented: <strong>PIBT</strong> — Priority Inheritance with
                    Backtracking (Okumura, Machida, Défago & Tamura) — the priority discipline of the search branch
                    rebuilt at the time step, with inheritance in place of every primitive. Read it after Push and
                    Rotate: the same maps (<InlineMath math="open01"/>, <InlineMath math="pocket01"/>,{" "}
                    <InlineMath math="corridor01"/>), the same head-on swap at their centers, and now the comparison
                    is direct — machinery that repairs around geometry versus negotiation that lets geometry win.
                    The genealogy's terminus this branch foreshadowed is written too: <strong>MAPF-POST</strong> —
                    plans again, but plans carrying time: routes with the waits deleted and one earliest arrival
                    time per retained location, executed at each agent's own velocity limit. What it inherits from
                    here is exactly what this branch inherited from prioritized planning: the view that time belongs
                    to the problem itself, not merely to its execution.
                </p>}
                ko={<p>
                    <strong>PIBT</strong>(Priority Inheritance with Backtracking, Okumura·Machida·Défago &
                    Tamura)가 한 회원으로 집필·구현됐다: search 갈래의 우선순위 규율을 스텝에서 다시 세우고 모든 primitive
                    자리에 상속을 놓은 것. Push and Rotate 뒤에 이것을 읽어라: 같은 맵(<InlineMath math="open01"/>,{" "}
                    <InlineMath math="pocket01"/> 그리고 <InlineMath math="corridor01"/>), 그 한가운데 같은 정면 교환,
                    그리고 이제 비교가 직접 가능해진다 — 기하를 기계장치로 돌아 수리하는 쪽과, 기하가 이기게 두는 협상.
                    이 갈래가 예고한 계보의 종착점도 집필됐다: <strong>MAPF-POST</strong> — 계획이 다시 등장하지만
                    이번엔 시간을 실은 계획이다. 대기를 지운 route와 유지된 위치마다 가장 빠른 도착 시각 하나씩, 각 agent의
                    자기 속도 한계로 실행된다. 여기서 상속받는 것은 정확히 이 갈래가 우선순위 계획에서 상속한 것이다: 시간은
                    실행만의 것이 아니라 문제 자체의 것이라는 관점.
                </p>}
            />
        </>
    )
}

export default DecentralizedPlanning
