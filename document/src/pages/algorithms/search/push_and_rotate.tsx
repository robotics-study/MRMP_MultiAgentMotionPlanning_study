import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runPushAndRotate} from "../../../libs/algorithms/push_and_rotate";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/search/push_and_rotate.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/search/push_and_rotate.hpp?raw";
import cppImpl from "../../../../../cpp/src/search/push_and_rotate.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이 map/agents 변경에만
// 재실행한다. 이 planner는 파라미터가 없다 (파라미터 무 dependence가 논문의 판매 포인트다).
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runPushAndRotate(map, tasks, {})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다.
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
    {name: "pocket01_swap", map: "pocket01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "corridor01_head_on", map: "corridor01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "tee01_head_on", map: "tee01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
    {name: "pocket01_rotate", map: "pocket01",
        agents: [[[1, 5], [1, 2]], [[1, 3], [1, 1]], [[1, 1], [1, 3]]]},
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

const PushAndRotate = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Push and Swap fails on a corridor, and it fails for a geometric reason: an exchange needs
                    room to turn around, and a width-1 corridor has none. Push and Rotate is de Wilde, ter Mors
                    and Witteveen's 2014 answer — the same priority discipline, but built from the ground up to
                    be a <strong>decision procedure</strong>: with at least two empty cells it finds a move
                    sequence whenever one exists, and reports unsolvable when none does. It gets there by
                    reading the map's structure before planning: the free graph decomposes into biconnected
                    subgraphs joined by planks, an agent whose start and goal live in different pieces makes
                    the instance unsolvable on the spot, and between pieces a priority order is derived so
                    whole subgraphs are cleared one at a time. Where Push and Swap's swap needed a 2×2 block,
                    this algorithm's exchange needs only a degree-3 junction — and where the walker's next step
                    lands on a trail that is still open, ROTATE rotates every rider on the resulting cycle one
                    cell forward instead of giving up.
                </p>}
                ko={<p>
                    Push and Swap은 통로에서 실패하고, 그건 기하학적 이유 때문이다. 맞교환에는 몸을 돌릴 공간이
                    필요하고 폭 1 통로엔 그게 없다. Push and Rotate는 de Wilde, ter Mors, Witteveen의 2014년
                    답이다 — 같은 우선순위 규율 위에 처음부터 <strong>판정 절차(decision procedure)</strong>로
                    설계됐다. 빈 셀이 둘 이상이면 이동 수열이 존재할 때 항상 찾고, 없으면 불가능하다고 보고한다.
                    계획 전에 맵의 구조를 읽는 데서 출발한다: 자유 그래프를 biconnected subgraph와 plank로 분해하고,
                    agent의 start와 goal이 다른 조각에 있으면 그 순간 인스턴스 불가능으로 판정하며, 조각들 사이엔
                    우선순위 순서가 유도되어 subgraph 전체를 하나씩 비운다. Push and Swap의 swap이 2×2 block을
                    필요로 했던 자리에서 이 알고리즘의 exchange는 degree-3 junction만 필요로 하고, 걷고 있는 agent의
                    다음 발이 아직 열린 trail 위에 닿으면 포기하는 대신 rotate가 그 사이클 위의 모든 rider를 한 칸
                    순환시킨다.
                </p>}
            />

            <h2>{t("From Push and Swap to Push and Rotate", "Push and Swap에서 Push and Rotate로")}</h2>
            <T
                en={<>
                    <p>
                        Everything the previous page built survives here: agents are still served one at a time,
                        each walks a static BFS shortest path from its current cell to its goal, blockers get
                        chain-pushed into reachable empty cells, and an exchange still replays every disturbed
                        bystander home. What changes is everything about <em>when repair is possible</em>. Push
                        and Swap asked for a free 2×2 block at the blocking site — a grid fact, not an algorithmic
                        choice — so on any tree-shaped map it simply had no answer. The fix is structural: before
                        moving anyone, decompose the free graph into its <strong>subgraphs</strong> (nontrivial
                        biconnected components plus singleton junctions), merge pairs whose join vertices sit within{" "}
                        <InlineMath math="m-2"/> of each other (<InlineMath math="m"/> empty cells — the paper's
                        Kornhauser-style feasibility condition), and assign every agent to a subgraph by its start
                        <em>and</em> by its goal. An agent whose two ends land in different pieces is not an
                        inconvenience here; it is proof the instance cannot be solved, and the planner says so
                        before moving anything.
                    </p>
                    <p>
                        That structure buys three things. First, priority becomes sound: relations between
                        subgraphs come from which goals sit where, transitively close into an order — and if that
                        order ever closes on itself, the instance is again honestly unsolvable. Second, swap gets
                        generalized: with a junction to work on, the exchange needs only two free neighbors of the
                        junction vertex (Algorithm 12 clears them in four escalating stages), not a whole square.
                        Third — and this is the actual completion — when the walker's next step lands on a cell
                        that belongs to an already-resolving agent's still-open trail, there is no empty cell to
                        push into and no pair to swap: <strong>rotate</strong> fires instead, cycling every rider
                        on the closed loop one cell forward so the walker's cell opens from behind. On a pure tree
                        none of this helps — no subgraph hosts a junction, no cycle ever closes — which is exactly
                        why the corridor instance stays unsolvable here too. The two algorithms meet at{" "}
                        <InlineMath math="tee01"/>: one added junction cell turns the same head-on swap from
                        impossible into 8 + 8 = 16 moves.
                    </p>
                </>}
                ko={<>
                    <p>
                        이전 페이지까지의 것은 전부 여기에도 살아 있다. agent는 여전히 한 대씩 처리되고, 각자 현재
                        셀에서 goal까지 정적 BFS 최단경로를 걷고, 막히는 점유자는 도달 가능한 빈 셀로 chain-push되고,
                        맞교환은 밀려난 구경꾼들을 전부 replay로 집으로 되돌린다. 바뀐 건 <em>복구가 언제 가능한가</em>에
                        대한 전부다. Push and Swap은 막힌 자리에서 빈 2×2 block을 요구했고 — 그건 알고리즘 선택이 아니라
                        격자의 기하학이었다 — 그래서 트리 모양 맵에서는 답이 없었다. 해법은 구조적이다. 아무도 움직이기
                        전에 자유 그래프를 <strong>subgraph</strong>(nontrivial biconnected component에 singleton junction)로
                        분해하고, join vertex가 서로 거리 <InlineMath math="m-2"/> 안에 있는 쌍을 병합하고(<InlineMath math="m"/>은
                        빈 셀 수 — 논문의 Kornhauser식 가능 조건), 모든 agent를 start<em>와</em> goal 양쪽으로 subgraph에
                        배정한다. 양 끝이 다른 조각에 떨어진 agent는 여기서부터 불편함이 아니라 인스턴스가 풀 수 없다는
                        증거이고, planner는 아무것도 움직이기 전에 그걸로 판정한다.
                    </p>
                    <p>
                        그 구조는 세 가지를 산다. 첫째, 우선순위가 sound해진다. 조각들 사이의 관계는 어느 goal이 어디에
                        있는지에서 나오고 추이적으로 닫혀 순서가 되며 — 그 순서가 자기 자신으로 닫히면 인스턴스는 역시
                        정직하게 불가능이다. 둘째, swap이 일반화된다. junction이 있으면 exchange는 junction vertex의 빈
                        이웃 둘만 필요하고(알고리즘 12가 네 단계로 늘려가며 비운다), 사각형 전체는 필요 없다. 셋째 — 그리고
                        이게 진짜 완성인데 — 걷는 agent의 다음 발이 이미 resolving 중인 agent의 아직 열린 trail에 닿을 때,
                        밀어 넣을 빈 셀도 맞바꿀 짝도 없다. 그때 <strong>rotate</strong>가 발동해서 닫힌 루프 위의 모든
                        rider를 한 칸 순환시키고, 그러면 walker의 셀이 뒤에서 열린다. 순수한 트리에서는 이 전부가 소용이
                        없다 — 어떤 subgraph도 junction을 담지 않고 사이클은 결코 닫히지 않는다 — 그래서 통로 인스턴스가
                        여기서도 불가능으로 남는 것이 정확히 그 이유다. 두 알고리즘은 <InlineMath math="tee01"/>에서 만난다:
                        갈림길 셀 하나가 같은 정면 교환을 불가능에서 8 + 8 = 16 이동으로 바꾼다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>A decision procedure, not just a planner.</strong> The paper's Theorem 1 is a
                            two-sided guarantee: with{" "}
                            <InlineMath math="n \le |V| - m"/> … precisely, with at least two empty vertices, the
                            algorithm finds a solution whenever one exists — and its unsolvable verdicts are true
                            statements about the instance (an agent split across subgraphs, or a cyclic priority
                            order), not search failures. The corridor head-on stays unsolvable here too; the same
                            swap on <InlineMath math="tee01"/>'s single junction is solvable and gets solved.
                        </li>
                        <li>
                            <strong>Parameter-free.</strong> Like its predecessor: no knob, and every choice the
                            algorithm must still make (neighbor order, row-major vertex scans, BFS enqueue-order
                            tie-breaks, first-clearable cycle vertex) is pinned deterministically in code.
                        </li>
                        <li>
                            <strong>Not optimal — and on tight maps, not even close.</strong> The rotate machinery
                            buys solvability, not speed: the pocket01_rotate scenario costs 35 moves where a
                            swap-only run of the same instance spends 47 (rotation is cheaper there), while
                            open01_swap costs 36 against a joint optimum of 30 and maze01_two costs 72 against 66.
                        </li>
                        <li>
                            <strong>Cost:</strong> every primitive is a BFS over free cells; the decomposition,
                            assignment and priority passes are polynomial scans over already-known cells. The
                            expanded_nodes metric counts BFS dequeues across path-finding and clear_vertex calls —
                            the algorithm still has no search frontier of its own.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>planner가 아니라 판정 절차(decision procedure)다.</strong> 논문의 정리 1은 양쪽
                            보장이었다: 빈 vertex가 둘 이상이면, 해가 존재할 때 항상 찾고 — 그리고 불가능 판정은
                            인스턴스에 대한 참 명제다(agent의 두 끝이 다른 subgraph에 있거나, 우선순위 순서가 자기로
                            닫히거나). 검색 실패가 아니다. 통로 정면 교환은 여기서도 불가능으로 남고,{" "}
                            <InlineMath math="tee01"/>의 갈림길 하나 위 같은 교환은 가능하고 실제로 푼다.
                        </li>
                        <li>
                            <strong>파라미터 무의존.</strong> 이전 알고리즘처럼: 노브는 없고, 알고리즘이 결국 정해야
                            하는 모든 선택(이웃 순서, row-major vertex 스캔, BFS 인입 순서 tie-break, 첫 clearable
                            cycle vertex)은 코드에 결정론적으로 고정된다.
                        </li>
                        <li>
                            <strong>최적과 무관하다 — 좁은 맵에서는 한참 멀리.</strong> rotate 기계는 해결 가능성을
                            사고 속도를 안 산다. pocket01_rotate 시나리오는 35 이동이고 같은 인스턴스를 swap만으로
                            돌리면 47이다(여기선 rotation이 더 싸다) — open01_swap은 최적 30에 36, maze01_two는 최적
                            66에 72를 쓴다.
                        </li>
                        <li>
                            <strong>비용:</strong> 모든 primitive는 자유 셀 위 BFS이고, 분해·배정·우선순위 패스는 이미
                            알려진 셀 위의 다항 스캔이다. expanded_nodes metric은 path-finding과 clear_vertex 호출들의
                            dequeue 총합을 센다 — 이 알고리즘에도 여전히 자체 search frontier는 없다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<>
                    <p>
                        The pipeline runs once per instance; the solve loop runs per agent. Decomposition, assignment
                        and priority are pure graph computations — everything after that is a primitive: push walks,
                        swap exchanges at junctions, rotate cycles, and clear_vertex feeds them all by making any
                        vertex empty along a parent chain.
                    </p>
                    <BlockMath math="A \in V^{n} \text{ injective}, \qquad T_k \text{ fixed goals}, \qquad m = |V| - n \text{ empty cells}"/>
                    <Terms items={[
                        ["A", <>current assignment — agent index to cell, injective; every executed move appends the new assignment to the solution path Pi</>],
                        ["T", <>goal assignment. Injective like A; an endpoint outside free is not a valid instance</>],
                        ["m", <>number of empty cells. The merge condition for two subgraphs is join-vertex distance ≤ m − 2</>],
                        ["q", <>the trail — every cell the walker has stepped on stays open in q until its agent finishes; a step onto another rider's open trail closes a cycle and fires rotate</>],
                    ]}/>
                </>}
                ko={<>
                    <p>
                        파이프라인은 인스턴스당 한 번, solve 루프는 agent마다 돈다. 분해·배정·우선순위는 순수 그래프
                        계산이고, 그 뒤 전부는 primitive다: push는 걷기, swap은 junction에서의 맞교환, rotate은 사이클
                        순환이고, clear_vertex가 parent 체인을 따라 임의의 vertex를 비워 전부에게 밥을 준다.
                    </p>
                    <BlockMath math="A \in V^{n} \text{ injective}, \qquad T_k \text{ fixed goals}, \qquad m = |V| - n \text{ 빈 셀 수}"/>
                    <Terms items={[
                        ["A", <>현재 배정 — agent index → 셀의 단사 함수. 실행된 모든 이동은 새 배정을 solution path Pi에 append 한다</>],
                        ["T", <>goal 배정. A처럼 단사여야 하고, free 밖의 endpoint는 유효한 인스턴스가 아니다</>],
                        ["m", <>빈 셀의 수. 두 subgraph의 병합 조건은 join vertex 거리 ≤ m − 2</>],
                        ["q", <>trail — walker가 밟은 셀은 그 agent가 끝날 때까지 열린 채로 q에 남는다. 다음 발이 다른 rider의 열린 trail 위에 닿으면 사이클이 닫히고 rotate이 발동한다</>],
                    ]}/>
                </>}
            />
            <Pseudocode code={`# ── per instance, once ───────────────────────────────────────────────
S1  SUBGRAPHS(G): Hopcroft–Tarjan biconnected components + singleton
    junctions; merge pairs whose join vertices sit within m−2      # Alg 1
S2  ASSIGN: agent → subgraph of its start AND of its goal;
    an agent split across pieces ⇒ UNSOLVABLE                      # Alg 2
S3  PRIORITY: Si < Sj when a plank leaving Si reaches a cell whose
    goal-agent belongs to Sj; close transitively; self-relation ⇒ UNSOLVABLE  # Alg 3
# ── per agent, inside solve ────────────────────────────────────────
for each unfinished r (assigned first by priority, lowest index wins):   # Alg 8
    p ← BFS path from A[r] to T[r]        (finished agents block it only on polygons)
    while A[r] ≠ T[r]:
        v ← next cell of p
        if v ∈ q: ROTATE(cycle of q)       # every rider steps one cell forward  # Alg 6
        else if not PUSH(r, v): SWAP(r, occupant(v))   # junction exchange         # Alg 4/5
        append v to the trail q
    r is finished; unwind q: send resolving agents home, hand off on demand
PUSH(r, v): clear_vertex(v) first when occupied — scan unoccupied cells row-major,
    first whose BFS reaches v wins, chain occupants shift toward it                # Alg 10
SWAP(r, s): candidates = degree ≥ 3 vertices of r's subgraph in BFS-dequeue order; # Alg 5
    MULTIPUSH walks the pair there (follower fills vacated cells),                  # Alg 11
    CLEAR frees two neighbors of v in four escalating stages,                        # Alg 12
    EXECUTE_SWAP rounds the junction; failed candidates roll back                    # Alg 13`}
            />
            <T
                en={<ol>
                    <li>The free graph is decomposed once: nontrivial biconnected components (a cycle's worth of
                        mutually reachable cells) plus every uncovered degree-≥3 cell as a singleton, then merged
                        while join vertices sit within <InlineMath math="m-2"/> — the paper's condition for when
                        two pieces are effectively one workspace.</li>
                    <li>Every agent is assigned by both endpoints. Start and goal landing in different subgraphs
                        (or an endpoint unassignable) is not routed around — it is reported as unsolvable, because
                        no maneuver can carry an agent between merged-far-apart pieces at this level of abstraction.</li>
                    <li>Priority relations come from the geometry: a plank leaving <InlineMath math="S_i"/> that
                        reaches a cell whose goal belongs to <InlineMath math="S_j"/> means{" "}
                        <InlineMath math="S_i"/> must be cleared first. The transitive closure of those relations is
                        the serving order; a self-relation contradicts itself, and the instance honestly fails.</li>
                    <li>Solve walks agents one at a time: assigned agents first (lowest index among those whose
                        subgraph has no unfinished predecessor), unassigned ones last. The walker's path BFS treats
                        finished agents' cells as walls only on polygon graphs — elsewhere push may shove even a
                        parked agent, and the unwind phase walks it back home.</li>
                    <li>The trail <InlineMath math="q"/> is what makes this complete rather than merely greedy:
                        every cell the walker has stepped on stays open in q until r finishes. When the next step
                        lands inside another resolving agent's still-open trail, the two trails close a cycle and
                        ROTATE rotates it — cascade from the first empty cycle cell; fully occupied cycles push one
                        rider off at the first clearable vertex, swap the pair onto the junction, and replay.</li>
                    <li>PUSH is the old chain shift with generalized blocking sets; SWAP is the old exchange with
                        its 2×2-block requirement replaced by Algorithm 12's four-stage clearing of two free
                        neighbors around a degree-3 vertex — which is exactly what a corridor cannot offer and a
                        junction can.</li>
                    <li>Every speculative maneuver records its moves into a segment; failure rolls the assignment
                        and the trace history back, success replays the segment reversed with roles exchanged.
                        The paper's Algorithm 9 (move smoothing) is not implemented — costs are raw move counts.</li>
                </ol>}
                ko={<ol>
                    <li>자유 그래프를 한 번 분해한다: nontrivial biconnected component(사이클 분량의 상호 도달 셀들)에
                        뒤덮이지 않은 degree ≥ 3 셀은 singleton으로 더하고, join vertex가 <InlineMath math="m-2"/> 안에
                        있는 쌍부터 병합 — 두 조각이 실질적으로 같은 작업공간인 조건이 그거다.</li>
                    <li>모든 agent를 양 끝으로 배정한다. start와 goal이 다른 subgraph에 떨어지면(또는 끝이 배정
                        불가) 우회하지 않는다 — 불가능으로 보고한다. 이 추상화 수준에서는 어떤 기동도 agent를 멀리
                        떨어진 조각들 사이로 나르지 못하기 때문이다.</li>
                    <li>우선순위 관계는 기하에서 나온다: <InlineMath math="S_i"/>에서 나가는 plank가 goal이{" "}
                        <InlineMath math="S_j"/> 소속인 셀에 닿으면 <InlineMath math="S_i"/>를 먼저 비워야 한다는 뜻이다.
                        그 관계들의 추이 닫힘이 서비스 순서가 되고, 자기 자신과의 관계는 모순이므로 인스턴스는 정직하게
                        실패한다.</li>
                    <li>solve는 agent를 한 대씩 걷힌다: 배정된 agent가 우선(끝난 predecessor가 없는 subgraph 중 index
                        최소), 미배정은 나중. walker의 경로 BFS는 polygon 그래프에서만 끝난 agent들의 셀을 벽으로 취급하고,
                        아니면 주차된 agent도 밀릴 수 있으며 unwind 단계가 그걸 집으로 되돌린다.</li>
                    <li>trail <InlineMath math="q"/>가 이 알고리즘을 탐욕이 아니라 완전하게 만드는 부분이다. walker가 밟은
                        셀은 r이 끝날 때까지 q에 열린 채로 남는다. 다음 발이 아직 resolving 중인 agent의 열린 trail 위에
                        닿으면 두 trail이 사이클을 닫고 rotate이 순환한다 — 첫 빈 cycle 셀에서 캐스케이드, 가득 찬
                        사이클은 첫 clearable vertex에서 rider 하나를 밀어내고 짝을 junction에 swap한 뒤 replay로 되돌린다.</li>
                    <li>PUSH는 blocked 집합만 일반화된 옛 chain shift이고, SWAP은 2×2 block 요건을 degree-3 vertex 주변
                        빈 이웃 둘의 4단계 clear(알고리즘 12)로 바꾼 옛 exchange다 — 통로가 못 주고 갈림길이 줄 수 있는
                        바로 그것.</li>
                    <li>모든 투기적 기동은 이동들을 segment에 기록하고, 실패하면 배정과 trace 역사를 rollback으로 되돌린다.
                        성공하면 segment를 역순 + role 교환으로 replay한다. 논문의 알고리즘 9(이동 스무딩)는 구현하지
                        않았다 — 비용은 날것 이동 수다.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    This is the branch's endgame: a procedure that terminates with a plan or with proof-shaped
                    evidence of impossibility. Both verdicts are exact here, and both are cheap — everything runs
                    in BFS-and-scan territory on the original graph, never in joint space. Expand the proofs for
                    why the subgraph condition is the right one, and what the honest failure actually says.
                </p>}
                ko={<p>
                    이게 우선순위 갈래의 결승점이다: 계획으로 끝나거나 불가능함의 증거-형태 판정으로 끝나는 절차. 여기선
                    두 판정 모두 정확하고, 둘 다 싸게 나온다 — 전부 joint 공간이 아니라 원래 그래프 위의 BFS와 스캔
                    영역에서 돈다. 왜 subgraph 조건이 올바른 조건인지, 정직한 실패가 실제로 무엇을 말하는지 증명을
                    펼쳐 보라.
                </p>}
            />
            <Proof title={t("Proposition (why m − 2 and why the assignment is decidable)", "명제 (왜 m − 2이고 왜 배정이 판정 가능한가)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Two pieces joined by a plank are one workspace exactly when the
                            empty-cell budget reaches the join: with <InlineMath math="m"/> empty cells, two
                            subgraphs whose closest members sit within <InlineMath math="m-2"/> can be treated as
                            one — an agent parked on either side can be walked across and back without stranding
                            anyone. An agent whose start sits inside one merged piece and whose goal lies outside
                            every reachable piece cannot be delivered by any sequence of push/swap/rotate moves,
                            because every move keeps the agent inside its current piece: push and swap never cross
                            a plank (they only ever step along edges that already exist inside the piece), and
                            rotate only rotates cycles, which live inside one biconnected piece.
                        </p>
                        <p>
                            That is the whole completeness argument in miniature: inside a merged piece every local
                            maneuver exists (junctions host exchanges, cycles host rotations), so whatever the
                            instance demands is locally executable; between pieces nothing can move anything, so an
                            agent split across pieces makes the instance unsolvable — and the planner's{" "}
                            <InlineMath math="f \neq f'"/> check says exactly that. The merge radius{" "}
                            <InlineMath math="m-2"/> is where "locally executable" stops being a promise: with two
                            empty cells you can always park one agent out of the way of the other's crossing.
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> plank으로 이어진 두 조각이 하나의 작업공간인 조건은 정확히 빈 셀
                            예산이 join에 닿는 것이다. 빈 셀이 <InlineMath math="m"/>개일 때, 가장 가까운 구성원이
                            거리 <InlineMath math="m-2"/> 안에 있는 두 subgraph는 하나로 취급할 수 있다 — 어느 쪽에
                            주차된 agent도 다른 쪽으로 건너고 돌아올 수 있고 그 누구도 고립되지 않는다. 한 병합 조각
                            안에 start가 있고 goal이 도달 가능한 어떤 조각에도 없는 agent는 push/swap/rotate 어떤
                            이동 수열로도 데려갈 수 없다. 모든 이동은 agent를 현재 조각 안에 머물게 하기 때문이다:
                            push와 swap은 plank를 건너지 않고(조각 안에 이미 있는 간선을 따라 걸을 뿐), rotate은
                            사이클만 순환시키고 그 사이클은 하나의 biconnected 조각 안에 산다.
                        </p>
                        <p>
                            완전성 논증 전체가 이걸로 축소된다: 병합된 조각 안에서는 모든 국소 기동이 존재하고(갈림길은
                            exchange를, 사이클은 rotation을 담는다), 인스턴스가 요구하는 것은 국소적으로 실행 가능하고;
                            조각들 사이에서는 아무것도 아무것도 움직일 수 없으니, 조각이 갈라진 agent는 인스턴스를
                            불가능하게 만들고 planner의 <InlineMath math="f \neq f'"/> 검사(양쪽 배정 비교)가 정확히
                            그걸 말한다. 병합 반경 <InlineMath math="m-2"/>는 "국소적으로 실행 가능"이 더 이상 약속이
                            아니게 되는 지점이다: 빈 셀 둘이면 항상, 다른 쪽 agent의 건너기를 한 agent로 비킬 수 있다.
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Proposition (the corridor stays unsolvable)", "명제 (통로는 계속 불가능하다)")}>
                <T
                    en={<p>
                        On a width-1 corridor the decomposition produces no nontrivial subgraph at all — there are
                        no cycles, so every degree-≥3 cell would be a singleton junction and a pure corridor has
                        none. With no subgraphs there is no exchange site anywhere (Algorithm 12 needs two free
                        neighbors around a junction) and no cycle to rotate, so both agents swapping ends of the
                        corridor make each other's start unassignable-to-goal: the assignment check fires before
                        any move is made and the planner reports <InlineMath math="\text{success} = \text{false}"/>.
                        That is not a search giving up — it is the same fact the 2×2-block lemma stated locally,
                        now seen globally: the geometry never offered a place to turn around.
                    </p>}
                    ko={<p>
                        폭 1 통로에서 분해는 nontrivial subgraph를 하나도 만들지 않는다 — 사이클이 없으니 모든
                        degree ≥ 3 셀은 singleton junction이어야 하고 순수한 통로엔 그게 없다. subgraph가 없으면
                        어디에도 exchange 자리(알고리즘 12는 junction 주변의 빈 이웃 둘이 필요하다)가 없고 순환할
                        사이클도 없다. 그래서 통로의 끝을 맞바꾸려는 두 agent는 서로의 start를 goal-배정 불가능하게
                        만들고, 배정 검사가 어떤 이동보다 먼저 발동해 planner는 <InlineMath math="\text{success} = \text{false}"/>를
                        보고한다. 그건 검색이 포기한 게 아니다 — 2×2 block 보조정리가 국소적으로 말한 그 사실이 전역으로
                        보인 것뿐이다: 기하가 몸을 돌릴 자리를 결코 제공하지 않았다.
                    </p>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what
                    the Python/C++ code below emits. Draw walls, drag a numbered dot or its ring to move an agent's
                    start/goal, add agents; every edit re-plans instantly and replays from step 0. The presets are
                    the whole argument of this page: <code>corridor01_head_on</code> is the head-on swap Push and
                    Swap could not do anything about — here it fails at the assignment check, before a single move.{" "}
                    <code>tee01_head_on</code> adds one junction cell to that corridor and solves the same request
                    in 8 + 8 = 16 moves: the pair walks to the junction and exchanges on it. <code>pocket01_swap</code>{" "}
                    is Push and Swap's own showcase re-solved here cheaper (6 + 6 = 12 against 14 — a generalized
                    exchange beats the rigid block maneuver). In <code>pocket01_rotate</code> three agents crowd
                    the pocket corridor: agent 0 walks left through (1,3), and when its path crosses the still-open
                    trails of the two resolving agents a cycle closes — rotate cascades every rider one cell and
                    cuts 47 moves down to 35. <code>open01_cross</code> never interferes at all (16 + 17, both at
                    unconstrained shortest cost), and <code>maze01_two</code> threads the single gap for 72 against
                    an optimum of 66. Watch the replay of pocket01_rotate closely: that is what a rotation looks
                    like — riders stepping one cell in sequence around a closed loop, nobody swapping at all.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과 바이트
                    단위로 같은 엔진입니다. 벽을 그리고, 번호가 적힌 점이나 그 링을 끌어 start/goal을 옮기고, agent를
                    더하면 모든 편집이 즉시 재계획되고 재생은 0스텝부터 다시 돕니다. preset들은 이 페이지의 논지 전체를
                    담았습니다. <code>corridor01_head_on</code>은 Push and Swap이 아무것도 못 했던 정면 교환이고, 여기서는
                    배정 검사에서 이동 하나 없이 실패합니다. <code>tee01_head_on</code>은 그 통로에 갈림길 셀을 하나
                    더해 같은 요청을 8 + 8 = 16 이동으로 풉니다 — 짝이 갈림길까지 걷고 위에서 맞교환합니다.{" "}
                    <code>pocket01_swap</code>은 Push and Swap의 쇼케이스를 여기서 더 싸게 푼 것입니다(14가 아니라 6 + 6 = 12
                    — 일반화된 exchange가 딱딱한 block 기동을 이긴다). <code>pocket01_rotate</code>에서는 agent 셋이
                    pocket 통로를 붐비고, agent 0이 (1,3)을 지나 왼쪽으로 걷는 경로가 아직 resolving 중인 둘의 열린
                    trail과 만나면 사이클이 닫히고 rotate이 모든 rider를 한 칸씩 순환시켜 47이 35로 줄어듭니다.{" "}
                    <code>open01_cross</code>는 간섭이 아예 없고(16 + 17, 둘 다 제약 없는 최단), <code>maze01_two</code>는
                    단일 gap을 72에 통과합니다(최적 66). pocket01_rotate의 재생을 잘 보세요: rotation이 저렇게 생겼습니다 —
                    닫힌 루프 위를 rider들이 순서대로 한 칸씩 밟고, 아무도 맞교환하지 않습니다.
                </p>}
            />
            <Sandbox label={t(
                "Live push_and_rotate sandbox — the browser engine is a byte-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; every edit re-plans and replays",
                "라이브 push_and_rotate sandbox. 브라우저 엔진은 Python/C++ planner와 바이트 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어집니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser
                    engine that powers the live sandbox above is a third mirror — same fixed neighbor order, same
                    row-major scans, same BFS-enqueue-order tie-breaks, same rollback semantics, so all three
                    produce byte-identical traces on every scenario, which <code>check-engine-parity</code>{" "}
                    verifies on every build. The code below is the actual source, not an excerpt. Two
                    implementation notes worth knowing: Python's parent dict doubles as parent map and enqueue-order
                    list (dict insertion order IS discovery order), so the C++ mirror carries both a parent map and
                    an explicit order vector; and the subgraph decomposition's merge step re-scans pairs from the
                    top after every merge — pinned, because which pair merges first changes what everything else
                    sees.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째
                    미러다 — 동일한 고정 이웃 순서, 동일한 row-major 스캔, 동일한 BFS 인입 순서 tie-break, 동일한
                    rollback semantics. 그래서 셋 모두 모든 시나리오에서 바이트 단위로 동일한 trace를 만들고 빌드마다{" "}
                    <code>check-engine-parity</code>가 검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다. 알아 둘
                    구현 노트 둘: Python의 parent dict는 parent 지도와 인입 순서 목록을 겸합니다(dict 삽입 순서가 곧
                    발견 순서) — 그래서 C++ 미러는 parent 지도와 명시적 order 벡터를 따로 들고 갑니다. 그리고 분해의
                    병합 단계는 병합마다 쌍 스캔을 맨 위에서 다시 시작한다 — 고정된 이유: 어느 쌍이 먼저 병합되느냐가
                    나머지가 무엇을 보게 될지를 바꾸기 때문이다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/search/push_and_rotate.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/search/push_and_rotate.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/search/push_and_rotate.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/search/push_and_rotate.hpp`,
                            },
                            {
                                name: "cpp/src/search/push_and_rotate.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/search/push_and_rotate.cpp`,
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
                    R. de Wilde, A. ter Mors, C. Witteveen,{" "}
                    <a href="https://doi.org/10.1613/jair.4447" target="_blank" rel="noopener noreferrer">
                        <em>Push and Rotate: a Complete Multi-agent Pathfinding Algorithm</em>
                    </a>,
                    Journal of Artificial Intelligence Research 51 (2014) 443–492.
                </li>
            </ol>
        </>
    )
}

export default PushAndRotate
