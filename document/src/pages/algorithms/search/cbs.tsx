import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runCbs} from "../../../libs/algorithms/cbs";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/search/cbs.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/search/cbs.hpp?raw";
import cppImpl from "../../../../../cpp/src/search/cbs.cpp?raw";

const REPO = "https://github.com/robotics-study/MRMP_MultiAgentMotionPlanning_study"

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 파라미터는 저장소의 configs/search/cbs.yaml과
// 같은 값: CT 확장 예산 64 (root가 첫 번째 확장으로 계산된다).
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runCbs(map, tasks, {max_ct_expansions: 64})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다. agent 상한은 4:
// coupled 탐색처럼 |V|^k가 아니라도 라이브 실행이 실시간으로 남게 하려는 선이다.
const PRESETS: ScenarioPreset[] = [
    {name: "maze01_two", map: "maze01", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
    {name: "open01_cross", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
]

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Cbs = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The previous two pages are the poles of this genealogy. Decoupled planning is cheap
                    but incomplete — a fixed order can only hope individually-optimal paths happen to
                    coexist. Coupled search is optimal and complete, but pays{" "}
                    <InlineMath math="|V|^k"/> for the privilege. CBS refuses both fates: it never plans
                    the agents jointly, yet stays optimal and complete. The trick is to make the
                    <em> conflicts themselves</em> the unit of search. Plan every agent alone against
                    explicit constraints (the low level is literally wave 1's space-time A*), keep each
                    joint solution as a node of a <strong>constraint tree</strong>, and when the cheapest
                    node's solution still collides, branch: one child forbids the first agent at that
                    cell-and-step, the other forbids the second. Agents are coupled only at the exact
                    cell and step where they actually interfere — nowhere else, ever.
                </p>}
                ko={<p>
                    이전 두 페이지는 이 계보의 두 극단이다. decoupled 계획은 싸지만 불완전하다. 고정된
                    순서는 개별 최적 경로들이 우연히 공존하기를 바랄 뿐이다. coupled 탐색은 최적이고
                    완전하지만 대가가 <InlineMath math="|V|^k"/>다. CBS는 두 운명을 모두 거부한다. agent들을
                    하나로 묶어 계획하지 않으면서도 최적이고 완전하다. 트릭은 <em>conflict 자체</em>를 탐색의 단위로
                    만드는 것이다. 모든 agent를 명시적 constraint에 대해 혼자 계획하고(low level은 그대로
                    wave 1의 시공간 A*), 각 joint 해를 <strong>constraint tree</strong>의 노드로 보관한다.
                    가장 싼 노드의 해가 여전히 충돌하면 분기한다. 한 자식은 첫 agent를 그 칸·시각에서
                    금지하고, 다른 자식은 두 번째를 금지한다. 결합은 agent들이 실제로 간섭하는 바로 그 칸과
                    스텝에서만 일어난다. 다른 어디에서도 아니다.
                </p>}
            />

            <h2>{t("From Joint States to Constraint Trees", "joint 상태에서 constraint tree로")}</h2>
            <T
                en={<>
                    <p>
                        A node of the constraint tree (CT) is not a joint state — it is a whole{" "}
                        <em>solution</em>: one space-time path per agent, plus the constraints those paths
                        obey. The root's constraint sets are empty, so its solution is every agent's
                        unconstrained optimum, planned in index order. The high level then does exactly one
                        thing: pop the cheapest node, scan its solution for a conflict, and either stop or
                        branch.
                    </p>
                    <BlockMath math="N = (\{C_0, \dots, C_{k-1}\},\; (p_0, \dots, p_{k-1})), \qquad \mathrm{cost}(N) = \sum_k \operatorname{len}(p_k) - 1"/>
                    <Terms items={[
                        ["C_k", <>agent <InlineMath math="k"/>의 constraint 집합. vertex constraint{" "}
                            <InlineMath math="(c, t)"/>는 시각 <InlineMath math="t"/>에 셀 <InlineMath math="c"/>를
                            금지하고, edge constraint는 정준화된 쌍 <InlineMath math="\{c, d\}"/>을 스텝에서
                            어느 방향으로든 지나는 것을 금지한다</>],
                        ["p_k", <>agent <InlineMath math="k"/>의 개별 최적 경로. 자기 constraint를 지키는
                            경로 중 가장 짧은 것. wave 1의 시공간 A*가 찾는다</>],
                        ["\\mathrm{cost}", <>노드 비용은 sum-of-costs. queue는 이 값 기준 best-first이고,
                            동률은 생성 순서(FIFO)로 깬다. 이 저장소의 다른 모든 탐색과 같은 관례다</>],
                    ]}/>
                    <p>
                        A conflict is selected the same way every time: the earliest step with any conflict,{" "}
                        <em>all agent pairs scanned at once</em> (there is no priority among agents here —
                        that was wave 1's move, and it survives only as a tie-break), ties broken by cell
                        row then column, then by pair order. The chosen constraint is canonicalized: the
                        pair's lexicographically smaller cell is <InlineMath math="c"/>, the larger is{" "}
                        <InlineMath math="d"/>, and direction is not encoded — a child that forbids agent{" "}
                        <InlineMath math="i"/> from the pair forbids traversing it in either direction.
                    </p>
                </>}
                ko={<>
                    <p>
                        constraint tree(CT)의 노드는 joint 상태가 아니다. 노드 하나는 해(solution) 전체다:
                        agent별 시공간 경로 하나씩, 그리고 그 경로들이 지키는 constraint들. root의 constraint
                        집합은 비어 있으므로 root의 해는 모든 agent의 무제약 최적경로이고, index 순서대로
                        계획된다. high level은 이후 딱 한 가지만 한다. 가장 싼 노드를 pop하고, 해에서
                        conflict을 스캔하고, 끝나면 멈추거나 분기한다.
                    </p>
                    <BlockMath math="N = (\{C_0, \dots, C_{k-1}\},\; (p_0, \dots, p_{k-1})), \qquad \mathrm{cost}(N) = \sum_k \operatorname{len}(p_k) - 1"/>
                    <Terms items={[
                        ["C_k", <>agent <InlineMath math="k"/>의 constraint 집합. vertex constraint{" "}
                            <InlineMath math="(c, t)"/>는 시각 <InlineMath math="t"/>에 셀 <InlineMath math="c"/>를
                            금지하고, edge constraint는 정준화된 쌍 <InlineMath math="\{c, d\}"/>을 스텝에서
                            어느 방향으로든 지나는 것을 금지한다</>],
                        ["p_k", <>agent <InlineMath math="k"/>의 개별 최적 경로. 자기 constraint를 지키는
                            경로 중 가장 짧은 것. wave 1의 시공간 A*가 찾는다</>],
                        ["\\mathrm{cost}", <>노드 비용은 sum-of-costs. queue는 이 값 기준 best-first이고 동률은
                            생성 순서(FIFO)로 깬다. 이 저장소의 다른 모든 탐색과 같은 관례다</>],
                    ]}/>
                    <p>
                        conflict 선택은 매번 동일하다: 어떤 pair에서든 conflict이 생긴 가장 이른 시각. 이때
                        모든 agent 쌍을 동시에 훑는다. 여기엔 agent 간 우선순위가 없다. 그건 wave 1의
                        수법이었고, 여기서는 tie-break로만 살아남는다. 동률은 cell의 row, 그다음 col, 그다음
                        pair 순서로 깬다. 선택된 constraint는 정준화된다: 쌍 중 사전식 작은 셀이{" "}
                        <InlineMath math="c"/>, 큰 셀이 <InlineMath math="d"/>이고 방향은 인코딩되지 않는다.
                        agent <InlineMath math="i"/>에게 pair를 금지한 자식은 그 쌍을 어느 방향으로든 지나는
                        것까지 금지당한다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Sum-of-costs optimal.</strong> The first conflict-free node popped is
                            provably optimal (proof below), so CBS delivers exactly what joint-space search
                            delivers — the same 66 / 33 / 30 on the three demo scenarios — without ever
                            touching <InlineMath math="V^k"/>.
                        </li>
                        <li>
                            <strong>Complete, but only semi-decidable.</strong> An empty queue is a sound
                            verdict: every branch died on its own constraints, so no joint plan exists at
                            all. But an unsolvable instance does not always produce that verdict — some
                            instances just migrate the conflict to later and later steps forever. That is
                            why this planner has one parameter where its siblings have none: a budget of{" "}
                            <InlineMath math="64"/> constraint-tree expansions (the root counts as the
                            first). Hitting the budget honestly means “no solution found within budget”,
                            never “unsolvable”.
                        </li>
                        <li>
                            <strong>Paid in interference, not in{" "}
                            <InlineMath math="|V|^k"/>.</strong> Every expansion is a single-agent search,
                            and the tree branches only where agents actually collide. On the corridor maze
                            CBS spends 2,753 single-agent expansions against joint-space search's 10,344
                            joint states — for the same cost of 66. But look at <code>open01_swap</code>:
                            head-on in a corridor is exactly where coupled search shines, and there the
                            joint state space expands 92 times while CBS spends 574. CBS is not a winner
                            everywhere; it pays in proportion to how much the agents actually interfere.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>Sum-of-costs 최적.</strong> conflict-free로 pop된 첫 노드는 증명 가능하게
                            최적이다(증명은 아래). 그래서 CBS는 joint-space 탐색이 내주는 것과 정확히 같은 것을
                            내준다. 세 시나리오에서 역시 66 / 33 / 30이고, 그걸 <InlineMath math="V^k"/>를
                            한 번도 건드리지 않고 해낸다.
                        </li>
                        <li>
                            <strong>완전하지만 semi-decidable에 불과하다.</strong> queue가 비는 것은 sound한
                            판정이다. 모든 가지가 자기 constraint로 죽었다는 뜻이고, 그럼 joint 계획은 존재하지
                            않는 것이다. 그런데 unsolvable한 instance가 항상 그 판정을 만들어 내지는 않는다.
                            어떤 instance은 conflict을 그냥 계속 더 나중 시각으로 옮기기만 한다. 그래서 이 planner만
                            파라미터를 가진다(형제들은 없다): constraint tree 확장 예산{" "}
                            <InlineMath math="64"/>(root가 첫 번째 확장으로 계산된다). 예산 소진은 정직하게는
                            “예산 안에서 해를 찾지 못했다”일 뿐이고, “unsolvable”의 증거가 절대 아니다.
                        </li>
                        <li>
                            <strong><InlineMath math="|V|^k"/>가 아니라 간섭만큼 지불한다.</strong> 확장 하나는
                            언제나 agent 혼자만의 탐색이고, 트리는 agent들이 실제로 부딪히는 곳에서만 갈라진다.
                            통로 미로에서 CBS는 single-agent 확장 2,753개를 쓰고, joint-space 탐색은 joint
                            상태를 10,344개 확장한다. 비용은 둘 다 66으로 같다. 그런데 <code>open01_swap</code>을
                            보라. 통로에서 정면으로 마주치는 것은 coupled 탐색이 딱 빛나는 순간이고, 거기서는
                            joint 상태 공간이 92번 확장되는 동안 CBS는 574개를 쓴다. CBS는 어디서나 이기는
                            알고리즘이 아니다. agent들이 실제로 얼마나 간섭하는지에 비례해서 대가를 청구한다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Two nested loops, and the multi-agent content lives entirely in the outer one. The inner
                    search is wave 1's space-time A* with reservations replaced by constraints; everything
                    that makes this CBS — branching, best-first order, conflict selection — happens above it.
                </p>}
                ko={<p>
                    중첩된 루프 두 개. multi-agent 내용은 전부 바깥 루프에 들어 있고, 안쪽 탐색은 wave 1의
                    시공간 A*에서 reservation을 constraint로 바꿔치기 한 것이다. CBS를 CBS답게 만드는 모든 것은
                    그 위에서 일어난다. 분기, best-first 순서, conflict 선택이 그렇다.
                </p>}
            />
            <Pseudocode code={`# root expansion (counts as the budget's first pop): every agent planned unconstrained
for k = 0 … K−1: p_k ← A*(start_k, goal_k, C_k = ∅)                      # 1
if any p_k = None: return failure                                        # 2
queue ← [(Σₖ len(p_k) − 1, seq₁)];  ct ← 1                               # 3
while queue ≠ ∅ and ct < budget:                                         # 4
    (cost, _, N) ← pop_min(queue);  ct ← ct + 1                          # 5
    conflict ← earliest over all pairs in N.paths                        # 6
    if none: emit paths in agent order; return success                   # 7
    for a ∈ (i, j): C_a += constraint(kind, cell, to, t)                  # 8
        p′_a ← A*(start_a, goal_a, C_a)                                   # 9
        if p′_a ≠ None: push (cost − len(p_a) + len(p′_a), seq_next)      # 10
return failure — queue empty = verdict; budget hit = budget              # 11`}
            />
            <T
                en={<ol>
                    <li>The root is expanded at construction: every agent planned unconstrained, in index
                        order.</li>
                    <li>If one sub-search fails the whole instance fails right there. A goal no
                        constraint-free path can reach is unreachable under <em>every</em> constraint set,
                        because constraints only ever remove paths — and a vertex constraint on a start cell
                        at <InlineMath math="t=0"/> always kills its agent: no move can undo being born
                        inside a forbidden cell.</li>
                    <li>The queue key is (sum-of-costs, creation sequence): best-first with a FIFO tie-break,
                        exactly the convention every other page of this repository uses. The root counts as
                        the budget's first expansion.</li>
                    <li>The loop runs while the queue is non-empty and the budget unspent. The budget is the
                        honest admission of semi-decidability — why no loop condition can replace it is the
                        second proof below.</li>
                    <li>Popping takes the cheapest node (ties by creation order) and counts against the
                        budget. <code>expanded_nodes</code> counts every low-level pop across all
                        sub-searches, dead branches included — that is the honest price of optimality, and
                        the sandbox counter shows it live.</li>
                    <li>Conflict scan: earliest step with any conflict over all pairs at once; ties by cell
                        row, then column, then pair order <InlineMath math="i \lt j"/>. Vertex and edge kinds
                        are kept distinct; an edge conflict carries the canonicalized pair (direction is not
                        encoded).</li>
                    <li>No conflict means the node's solution is jointly conflict-free, and best-first makes
                        it optimal (proof below). Emit the paths in agent-index order; done.</li>
                    <li>Branching creates one child per conflicting agent, in pair order. Each child adds
                        exactly one constraint to exactly one agent.</li>
                    <li>The constrained agent alone is re-planned against its full constraint list — wave 1's
                        space-time A* again, horizon included: past the last constrained step nothing binds
                        anymore, so every feasible plan has an equivalent of length ≤ (last constrained step)
                        + |reachable|.</li>
                    <li>A child whose sub-search fails dies and is never pushed. The surviving child's cost
                        updates by exchange — the other agents' paths are untouched, so subtract the replaced
                        path's cost and add the new one's — and rides the queue with the next sequence
                        number.</li>
                    <li>Leaving the loop means failure, and which exit fired matters: an empty queue is a
                        verdict of unsolvability; a spent budget only means “no solution found within
                        budget”.</li>
                </ol>}
                ko={<ol>
                    <li>root는 생성 시점에 확장된다: 모든 agent를 무제약으로, index 순서대로 계획한다.</li>
                    <li>sub-search 하나가 실패하면 instance 전체가 즉시 실패한다. constraint 없는 경로로 못 가는
                        goal은 <em>모든</em> constraint 집합 아래에서도 도달 불가다. constraint는 경로를 지우기만 하고
                        늘리지 않으니까. 특히 start 셀에 <InlineMath math="t=0"/>인 vertex constraint는 그 agent를
                        항상 죽인다. 금지된 셀 안에서 태어나는 것은 어떤 이동으로도 되돌릴 수 없다.</li>
                    <li>queue의 키는 (sum-of-costs, 생성 순서): FIFO tie-break이 있는 best-first이고, 이 저장소의
                        다른 모든 페이지가 쓰는 관례와 정확히 같다. root가 예산의 첫 번째 확장으로 계산된다.</li>
                    <li>루프는 queue가 비지 않고 예산이 남았을 동안 돈다. 예산은 semi-decidability를 정직하게
                        인정한 것이고, 어떤 루프 조건으로도 그것을 대체할 수 없는 이유가 아래 두 번째 증명이다.</li>
                    <li>pop은 가장 싼 노드(동률은 생성 순서)를 꺼내고 예산을 소진시킨다.
                        <code>expanded_nodes</code>는 죽은 가지를 포함해 모든 sub-search의 모든 low-level pop을
                        센다. 그게 최적성의 정직한 대고, sandbox의 카운터가 그걸 라이브로 보여 준다.</li>
                    <li>conflict 스캔: 모든 pair를 동시에 훑어 가장 이른 시각의 conflict. 동률은 cell의 row,
                        col, pair 순서 <InlineMath math="i \lt j"/>로 깬다. vertex와 edge 종류는 구분해 유지하고,
                        edge conflict는 정준화된 쌍을 그대로 실어 나른다(방향은 인코딩되지 않는다).</li>
                    <li>conflict가 없으면 그 노드의 해는 joint하게 conflict-free이고, best-first가 그것을 최적이라
                        만든다(증명은 아래). 경로를 agent index 순서로 방출하고 끝낸다.</li>
                    <li>분기는 conflict한 agent마다 자식 하나씩, pair 순서로. 각 자식은 정확히 한 agent에게
                        정확히 하나의 constraint를 더한다.</li>
                    <li>제약된 agent만 자기 constraint 목록 전체에 대해 다시 계획된다. wave 1의 시공간 A*가
                        다시 쓰이는 것이고 horizon도 그대로다: 마지막 제약 스텝 너머에서는 아무것도 속박하지 않으니,
                        모든 합법 계획은 (마지막 제약 스텝) + |reachable| 길이 이하의 동치 계획을 가진다.</li>
                    <li>sub-search가 실패한 자식은 죽고 push되지 않는다. 살아남은 자식의 비용은 exchange로
                        갱신한다. 다른 agent들의 경로는 손대지 않았으니 교체되는 경로의 비용을 빼고 새 경로의
                        비용을 더하면 되고, 다음 순서 번호로 queue에 올라탄다.</li>
                    <li>루프를 빠져나오면 실패이고, 어느 출구로 나갔는지가 중요하다. 빈 queue는 unsolvability의
                        판정이고, 소진된 예산은 그냥 “예산 안에서 해를 찾지 못했다”일 뿐이다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    CBS buys the coupled pole's guarantees at the decoupled pole's price — and pays for that
                    magic with one honest limitation: no budget-free termination on every instance. Expand the
                    proofs for both halves of that claim.
                </p>}
                ko={<p>
                    CBS는 coupled 극단의 보장을 decoupled 극단의 가격으로 산다. 그리고 그 마법의 대가로 정직한
                    한계를 하나 샀다: 모든 instance에 예산 없는 종결이 없다는 것. 그 주장의 양쪽을 접힌 증명에서
                    편다.
                </p>}
            />
            <Proof title={t("Theorem (optimality)", "정리 (최적성)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If CBS pops a node whose solution is conflict-free, that
                            solution minimizes <InlineMath math="\sum_k T_k"/> over all joint plans; and if
                            a joint plan exists at all, some node eventually has one.
                        </p>
                        <p>
                            <strong>Proof.</strong> Fix an optimal joint plan <InlineMath math="S^{*}"/> of
                            cost <InlineMath math="\mathrm{OPT}"/> and call a CT node{" "}
                            <em>faithful</em> when every constraint in it is satisfied by{" "}
                            <InlineMath math="S^{*}"/>. The root is faithful, and its cost is the sum of the
                            unconstrained per-agent optima — at most <InlineMath math="\mathrm{OPT}"/>, since{" "}
                            <InlineMath math="S^{*}"/> itself supplies feasible individual paths. A faithful
                            node never dies: agent <InlineMath math="i"/>’s constrained sub-search always has
                            a feasible path left (follow <InlineMath math="S_i^{*}"/>, then run straight to
                            the goal past the last constrained step), so a child that survives is always
                            pushed. And when a faithful node pops with a conflict at{" "}
                            <InlineMath math="(x,t)"/> between agents <InlineMath math="i"/> and{" "}
                            <InlineMath math="j"/>, at least one child stays faithful: if both children were
                            unfaithful, then <InlineMath math="S^{*}"/> itself would occupy the pair cells in
                            the conflicting way — a conflict inside <InlineMath math="S^{*}"/> itself,
                            contradiction. So under non-termination a faithful chain grows forever, each pop
                            adding one constraint that strictly changes some agent’s constrained-optimal path
                            (the new constraint forbids exactly what the old solution did). Costs along the
                            chain are non-decreasing and bounded by <InlineMath math="\mathrm{OPT}"/>, hence
                            eventually constant — but a constant-cost infinite chain needs infinitely many
                            distinct path tuples of cost ≤ <InlineMath math="\mathrm{OPT}"/>, and there are
                            only finitely many. Contradiction: on a solvable instance the search terminates,
                            so some node is popped conflict-free. Best-first pops in non-decreasing cost, and
                            at that moment a faithful node of cost ≤ <InlineMath math="\mathrm{OPT}"/> still
                            sits in the queue, so the popped cost is ≤ <InlineMath math="\mathrm{OPT}"/>;
                            being conflict-free it is a feasible joint plan, so its cost is also ≥{" "}
                            <InlineMath math="\mathrm{OPT}"/>. Equality: the solution is optimal.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> CBS가 conflict-free 노드를 pop하면 그 해는 모든 joint 계획에
                            대해 <InlineMath math="\sum_k T_k"/>를 최소화하고, joint 계획이 어쨌든 존재하면 어떤
                            노드는 언젠가 conflict-free로 pop된다.
                        </p>
                        <p>
                            <strong>증명.</strong> 비용이 <InlineMath math="\mathrm{OPT}"/>인 최적 joint 계획{" "}
                            <InlineMath math="S^{*}"/>를 고정하고, 자기 constraint가 전부 <InlineMath math="S^{*}"/>에
                            의해 지켜지는 노드를 <em>faithful</em>이라 부르자. root는 faithful이고 그 비용은 무제약
                            agent별 최적의 합이므로 <InlineMath math="S^{*}"/> 자체가 합법적인 개별 경로를 공급하므로{" "}
                            <InlineMath math="\mathrm{OPT}"/> 이하다. faithful 노드는 죽지 않는다: agent{" "}
                            <InlineMath math="i"/>의 constraint가 걸린 sub-search에는 항상 합법 경로가 남는다
                            (마지막 제약 스텝 너머에서는 <InlineMath math="S_i^{*}"/>를 따라가면 되니). 그래서 살아남는
                            자식이 항상 push된다. 그리고 faithful 노드가 <InlineMath math="(x,t)"/>의 conflict로 pop되면,
                            적어도 한 자식은 faithful로 남는다: 두 자식 모두 unfaithful하면 <InlineMath math="S^{*}"/>
                            자신이 pair 셀을 충돌하는 방식으로 차지하고 있다는 뜻이고, 그건 <InlineMath math="S^{*}"/>
                            안에 conflict이 있다는 모순이다. 따라서 무종결을 가정하면 faithful chain이 영원히 자라는데,
                            pop마다 constraint가 하나씩 추가되고 그 새 constraint는 이전 해가 정확히 하던 일을 금지하므로
                            어떤 agent의 개별 최적 경로가 엄격히 바뀐다. chain을 따라 비용은 단조 증가하고{" "}
                            <InlineMath math="\mathrm{OPT}"/>로 상한되므로 결국 상수인데, 비용 상수의 무한 chain은 비용이{" "}
                            <InlineMath math="\mathrm{OPT}"/> 이하인 서로 다른 경로 tuple을 무한히 요구한다. 유한하다.
                            모순. 그래서 solvable instance에서 탐색은 종결하고, 어떤 노드가 conflict-free로 pop된다.
                            best-first는 비용이 감소하지 않는 순서로 pop하고, 그 순간에도 비용이 <InlineMath math="\mathrm{OPT}"/>
                            이하인 faithful 노드가 queue에 남아 있으므로 pop된 비용은 ≤ <InlineMath math="\mathrm{OPT}"/>.
                            conflict-free이니 합법 joint 계획이고, 따라서 비용은 ≥ <InlineMath math="\mathrm{OPT}"/>.
                            일치한다. 최적이다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Theorem (the empty queue is a verdict — and why you still need the budget)", "정리 (빈 queue는 판정이다. 그래도 예산이 필요한 이유)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> If the queue empties, no joint plan exists. But the converse
                            fails: some unsolvable instances never empty the queue, which is why the search
                            runs on an expansion budget.
                        </p>
                        <p>
                            <strong>Proof of the verdict.</strong> Suppose a joint plan{" "}
                            <InlineMath math="S^{*}"/> exists while the queue empties. By the chain argument of
                            the optimality proof, a faithful node is always in the queue and never dies — its
                            constrained sub-searches keep succeeding because{" "}
                            <InlineMath math="S^{*}"/>'s own paths are feasible under those constraints. A live
                            faithful node means a non-empty queue. Contradiction: an empty queue proves no plan
                            exists, which is why the demo's walled-off goal reports failure honestly.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            <strong>Why not always.</strong> On a genuinely unsolvable instance — two agents
                            that can never pass in a corridor of width one — every branch merely migrates the
                            conflict: constraining agent <InlineMath math="i"/> at cell <InlineMath math="x"/>
                            and step <InlineMath math="t"/> makes it wait or detour, and the same head-on
                            meeting reappears one step later on both children. Both sub-searches keep succeeding
                            (waiting is always feasible), so no branch ever dies, and the tree grows forever
                            while conflicts crawl toward infinity. A search that must answer cannot wait for
                            that tree: with the budget spent, the honest report is “no solution found within
                            budget”, never a proof of unsolvability. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> queue가 비면 joint 계획이 존재하지 않는다. 그러나 역은 성립하지
                            않는다. 어떤 unsolvable instance는 queue를 절대 비우지 않고, 그래서 이 탐색은 확장 예산
                            위에서 돈다.
                        </p>
                        <p>
                            <strong>판정의 증명.</strong> queue가 비었는데 joint 계획 <InlineMath math="S^{*}"/>가
                            존재한다고 하자. 최적성 증명의 chain 논증에 의해 faithful 노드는 항상 queue에 있고 절대
                            죽지 않는다. 그 constraint들 아래에서 <InlineMath math="S^{*}"/>의 경로들이 여전히 합법이니
                            sub-search는 계속 성공한다. 살아 있는 faithful 노드가 있는데 queue가 비어 있다. 모순.
                            그래서 빈 queue는 계획이 없다는 것을 증명하고, demo에서 goal을 벽으로 막았을 때 정직하게
                            실패를 보고하는 이유가 이거다. <InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            <strong>왜 항상은 아닌가.</strong> 진짜 unsolvable한 instance, 예를 들어 폭 1 통로에서 절대 서로
                            지나갈 수 없는 두 agent에서는 모든 분기가 conflict을 옮기기만 한다. agent{" "}
                            <InlineMath math="i"/>를 셀 <InlineMath math="x"/> 시각 <InlineMath math="t"/>에서
                            금지하면 그 agent는 기다리거나 돌아가고, 같은 정면 대치가 한 스텝 뒤 양쪽 자식에서 다시
                            살아난다. 두 sub-search는 계속 성공한다(대기는 항상 합법). 그래서 아무 가지도 죽지 않고,
                            conflict가 무한으로 기어가는 동안 트리는 영원히 자란다. 답해야 하는 탐색이 그 트리를 기다릴
                            수는 없다. 예산을 다 쓴다면 정직한 보고는 “예산 안에서 해를 찾지 못했다”이지, unsolvability의
                            증거가 아니다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte
                    what the Python/C++ code below emits. Watch what the tree does on the three presets. On{" "}
                    <code>open01_cross</code> the root's two unconstrained paths already miss each other at
                    the crossing (agent 0 passes cell (10,9) one step before agent 1 arrives), so there is no
                    conflict to branch on: zero branching, and the expansion count — 35 — is exactly what two
                    plain A* searches cost. On <code>maze01_two</code> both root paths thread the same corridor
                    and swap cells (2,9)↔(3,9) at step 23: one edge conflict, one branch, and the popped child's
                    reroute keeps length 33 — cost stays exactly 66. On <code>open01_swap</code> the head-on
                    meeting has no clever timing left: watch the conflict migrate down the corridor (vertex at{" "}
                    <InlineMath math="(10,9)@7"/>, then an edge swap, then more vertices) while the tree grows
                    through seven conflicts and 574 expansions — where joint-space search needed only 92. The
                    red X marks each selected conflict (both cells for a swap), dashed rectangles mark the
                    constraints it spawns, and expansion blooms still show per agent: two single-agent searches
                    at a time, never one joint space. Drag endpoints to create your own conflicts; the budget
                    is 64 CT expansions, and on a corridor you cannot solve by editing, watch it stop honestly.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과
                    바이트 단위로 같은 엔진입니다. 세 preset에서 트리가 무엇을 하는지 보라. <code>open01_cross</code>{" "}
                   에서는 root의 무제약 경로 두 개가 교차점에서 이미 서로를 비껴간다(agent 0이 (10,9)를 agent 1이
                    도착하기 한 스텝 전에 지난다). 분기할 conflict가 없다. 분기 0회, 확장 수 35는 그냥 A* 두 번의
                    값이다. <code>maze01_two</code>에서는 root의 두 경로가 같은 통로를 지나고 스텝 23에서 (2,9)↔(3,9)를
                    맞바꾼다. edge conflict 하나, 분기 하나, 그리고 pop된 자식의 재경로는 길이 33을 유지하고 비용은
                    정확히 66이다. <code>open01_swap</code>에서는 정면 대치에 남은 영리한 타이밍이 없다. conflict가
                    통로를 따라 이동하는 것을 보라(<InlineMath math="(10,9)@7"/>의 vertex, 그다음 edge swap, 또 여러
                    vertex). 트리는 conflict 7개와 확장 574개로 자라고, joint-space 탐색은 92면 끝났다. 빨간 X는 선택된
                    각 conflict(swap이면 두 셀 모두), 점선 사각형이 그 conflict에서 파생된 constraint이고, 확장 bloom은 여전히
                    agent별로 피어난다. 동시에 두 개의 single-agent 탐색이지, joint 상태 공간 같은 것은 아니다. endpoint를
                    끌어 직접 conflict을 만들어 보라. 예산은 CT 확장 64고, 편집으로도 풀 수 없는 통로를 만들면 정직하게
                    멈추는 것을 볼 수 있다.
                </p>}
            />
            <Sandbox maxAgents={4} label={t(
                "Live cbs sandbox — byte-identical to the Python/C++ planner. Draw walls, drag endpoints; every edit re-plans and replays: expansions per agent, conflicts as red X, constraints as dashed rectangles",
                "라이브 cbs sandbox — Python/C++ planner와 바이트 단위로 동일합니다. 벽을 그리고 endpoint를 끄면 모든 편집이 재계획과 재생으로 이어집니다: agent별 확장, 빨간 X의 conflict, 점선 사각형의 constraint",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the
                    browser engine that powers the live sandbox above is a third mirror: same fixed neighbor
                    order, same <InlineMath math="(f, \text{seq})"/> tie-break inside every sub-search, same{" "}
                    <InlineMath math="(\mathrm{cost}, \text{seq})"/> queue order on top, same canonicalized
                    edge constraints — so all three produce byte-identical traces on every scenario, which{" "}
                    <code>check-engine-parity</code> verifies on every build. Trace chronology is part of the
                    contract: root expansions come first in agent-index order, each pop emits its{" "}
                    <code>conflict_found</code> before its children's <code>constraint_added</code> and re-plan
                    expansions, and <code>path_found</code> appears only for the final solution paths. The code
                    below is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이
                    세 번째 미러다. 동일한 고정 이웃 순서, 모든 sub-search 안의 동일한{" "}
                    <InlineMath math="(f, \text{seq})"/> tie-break, 그 위의 동일한{" "}
                    <InlineMath math="(\mathrm{cost}, \text{seq})"/> queue 순서, 동일한 정준화 edge constraint.
                    그래서 셋 모두 모든 시나리오에서 바이트 단위로 동일한 trace를 만들고,{" "}
                    <code>check-engine-parity</code>가 빌드마다 이를 검증한다. trace 연대기도 계약의 일부다.
                    root 확장이 agent index 순서로 먼저 오고, 각 pop은 자기 <code>conflict_found</code>를 자식들의{" "}
                    <code>constraint_added</code>와 재확장보다 앞에 방출하고, <code>path_found</code>는 최종 해의
                    경로에만 나타난다. 아래 코드는 발췌가 아니라 실제 소스 그대로다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/search/cbs.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/search/cbs.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/search/cbs.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/search/cbs.hpp`,
                            },
                            {
                                name: "cpp/src/search/cbs.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/search/cbs.cpp`,
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
                CBS was introduced for MAPF by Sharon, Stern, Felner and Sturtevant — first at AAAI 2012,
                then in the journal version this repository implements; the survey is the same one that names
                the genealogy's three poles.
            </p>} ko={<p>
                CBS는 Sharon, Stern, Felner, Sturtevant가 MAPF를 위해 도입했다. 2012년 AAAI에 먼저 실렸고, 이
                저장소가 구현한 것은 저널 판이다. survey는 이 계보의 세 극단에 이름을 붙여 준 그 논문이다.
            </p>} />
            <ol>
                <li>
                    G. Sharon, R. Stern, A. Felner, N. R. Sturtevant,{" "}
                    <a href="https://doi.org/10.1016/j.artint.2014.11.006" target="_blank" rel="noopener noreferrer">
                        <em>Conflict-based search for optimal multi-agent pathfinding</em>
                    </a>,
                    Artificial Intelligence, 2015.
                </li>
                <li>
                    R. Stern et al.,{" "}
                    <a href="https://doi.org/10.1609/socs.v10i1.18510" target="_blank" rel="noopener noreferrer">
                        <em>Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks</em>
                    </a>,
                    SoCS, 2019.
                </li>
            </ol>
        </>
    )
}

export default Cbs
