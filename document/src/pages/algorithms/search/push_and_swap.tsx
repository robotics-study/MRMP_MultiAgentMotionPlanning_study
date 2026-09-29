import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Sandbox, {ScenarioPreset} from "../../../components/panels/Sandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import {runPushAndSwap} from "../../../libs/algorithms/push_and_swap";
import {GridMap} from "../../../libs/grid";
import {Cell, TraceEvent} from "../../../libs/trace/types";
import pyImpl from "../../../../../python/mrmp/search/push_and_swap.py?raw";
import cppHeader from "../../../../../cpp/include/mrmp/search/push_and_swap.hpp?raw";
import cppImpl from "../../../../../cpp/src/search/push_and_swap.cpp?raw";

// 라이브 sandbox의 엔진 — 모듈 상수여야 identity가 안정적이라 SandboxScene이
// map/agents 변경에만 재실행한다. 이 planner는 파라미터가 없다 (파라미터 무 dependence가
// 논문의 판매 포인트).
const runLive = (map: GridMap, tasks: Array<[Cell, Cell]>): TraceEvent[] =>
    runPushAndSwap(map, tasks, {})

// 시나리오 preset — cell 좌표는 데모/parity와 동일한 좌표계다.
const PRESETS: ScenarioPreset[] = [
    {name: "open01_cross", map: "open01", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
    {name: "open01_swap", map: "open01", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
    {name: "pocket01_swap", map: "pocket01", agents: [[[1, 1], [1, 5]], [[1, 5], [1, 1]]]},
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

const PushAndSwap = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Prioritized planning fails in one specific way: an early agent parks on its goal,
                    a later agent's only route runs straight through that parked body, and a frozen
                    path has no answer. Push and Swap is Luna and Bekris's 2011 completion of the
                    priority branch — the same one-agent-at-a-time discipline, but finished agents are
                    no longer frozen in place. When agent <InlineMath math="r"/> reaches a cell that an
                    earlier agent occupies, the planner does not give up. It either <strong>pushes</strong>:
                    chain-shoves the blocker (and anyone standing between it and the nearest empty cell)
                    out of the way — or it <strong>swaps</strong>, exchanging <InlineMath math="r"/> with
                    the blocker through a local maneuver and restoring every bystander that maneuver
                    disturbed. There is no window, no tuning knob, no re-ordering heuristic. Parameter-free
                    operation is literally the paper's selling point.
                </p>}
                ko={<p>
                    우선순위 계획은 한 가지 방식으로 실패한다. 앞선 agent가 goal에 주차되고, 뒤 agent의
                    유일한 경로가 그 주차된 몸을 정면으로 관통하며, 얼려진 경로는 답이 없다. Push and Swap은
                    Luna와 Bekris가 2011년에 완성한 우선순위 갈래의 마지막 장이다. agent를 한 대씩 처리하는
                    같은 규율을 쓰되, 끝난 agent를 더 이상 제자리에 얼리지 않는다. agent <InlineMath math="r"/>이
                    앞선 agent가 점유한 셀에 닿으면 planner는 포기하지 않는다. <strong>push</strong>로 막는
                    점유자를(그와 가장 가까운 빈 셀 사이를 가로막는 모든 점유자까지) 사슬처럼 밀어내거나, 그래도
                    안 되면 <strong>swap</strong>으로 <InlineMath math="r"/>과 점유자를 국소 기동으로 맞바꾸고
                    기동에 밀려난 구경꾼들을 전부 제자리로 되돌린다. 창(window)도, 튜닝 노브도, 순서 재배열
                    heuristic도 없다. 파라미터 무의존이 말 그대로 논문의 판매 포인트다.
                </p>}
            />

            <h2>{t("From Frozen Reservations to Push and Swap", "얼린 예약에서 Push and Swap으로")}</h2>
            <T
                en={<>
                    <p>
                        Prioritized planning froze a finished agent's whole space-time path. Here nothing is
                        frozen in time — what survives of the priority discipline is only the order: agents
                        are still served one at a time in index order, and each agent drives its own static
                        BFS shortest path from its current cell to its goal. The state of the world is just
                        an assignment <InlineMath math="A"/> (one cell per agent) plus the set{" "}
                        <InlineMath math="U"/> of goal cells whose agents have finished:
                    </p>
                    <BlockMath math="A \in V^{n} \text{ injective}, \qquad U = \{\, T_k : \text{agent } k \text{ has reached its goal}\,\, \}"/>
                    <Terms items={[
                        ["A", <>현재 배정. agent index → 셀의 단사 함수이고, 모든 이동은 이 배정을 갱신한다</>],
                        ["T", <>목표 배정. A와 마찬가지로 단사여야 한다. 위반 입력은 문제의 인스턴스가 아니다</>],
                        ["U", <>goal에 이미 도착한 agent들의 goal 셀 집합. push의 hole 찾기 BFS는 이 셀들을 벽으로 취급한다. 주차된 agent는 결코 밀릴 수 없다</>],
                    ]}/>
                    <p>
                        Two primitives do all the work, and both are pure graph operations on the grid. PUSH
                        walks <InlineMath math="r"/> along its shortest path; when the next cell is occupied,
                        a BFS from the blocker (treating <InlineMath math="\{A[r]\} \cup U"/> as walls) finds
                        the nearest reachable empty cell and every occupant on that chain steps one cell
                        toward the hole, farthest-from-<InlineMath math="r"/> first. SWAP is what happens when
                        push cannot clear the way — typically because the blocker is a parked agent in{" "}
                        <InlineMath math="U"/>, which by construction can never be shoved. The paper leaves
                        POP() order over swap vertices unspecified; this repository pins it to BFS-dequeue
                        order from <InlineMath math="A[r]"/> so every run is byte-identical across languages.
                    </p>
                    <p>
                        One grid fact changes the geometry of the swap: grid graphs are bipartite, they have
                        no triangles, so exchanging two adjacent agents physically requires a free 2×2 block.
                        The paper's Figure-1 T-junction sketch is schematic — on a grid only the square case
                        exists. That single fact is also what makes completeness delicate, as the next section
                        shows with a one-cell-wide corridor that cannot be solved at all.
                    </p>
                </>}
                ko={<>
                    <p>
                        우선순위 계획은 끝난 agent의 시공간 경로 전체를 얼렸다. 여기서는 시간 위에 얼려진 것이
                        없다. 우선순위 규율에서 남는 것은 순서뿐이다. agent는 여전히 index 순서로 한 대씩
                        처리하고, 각 agent는 자기 현재 셀에서 goal까지의 정적 BFS 최단경로를 스스로 운전한다.
                        세계의 상태는 배정 <InlineMath math="A"/>(agent당 셀 하나)과 끝난 agent들의 goal 셀
                        집합 <InlineMath math="U"/>뿐이다:
                    </p>
                    <BlockMath math="A \in V^{n} \text{ injective}, \qquad U = \{\, T_k : \text{agent } k \text{ has reached its goal}\,\, \}"/>
                    <Terms items={[
                        ["A", <>현재 배정. agent index → 셀의 단사 함수이고, 모든 이동은 이 배정을 갱신한다</>],
                        ["T", <>목표 배정. A와 마찬가지로 단사여야 한다. 위반 입력은 문제의 인스턴스가 아니다</>],
                        ["U", <>goal에 이미 도착한 agent들의 goal 셀 집합. push의 hole 찾기 BFS는 이 셀들을 벽으로 취급한다. 주차된 agent는 결코 밀릴 수 없다</>],
                    ]}/>
                    <p>
                        두 primitive가 모든 작업을 한다. 둘 다 격자 위의 순수 그래프 연산이다. PUSH는{" "}
                        <InlineMath math="r"/>을 최단경로를 따라 걷히고, 다음 셀이 점유돼 있으면 blocker에서
                        BFS(<InlineMath math="\{A[r]\} \cup U"/>를 벽으로 취급)로 가장 가까운 도달 가능 빈 셀을
                        찾아 그 체인 위의 점유자들이 r에서 먼 쪽부터 한 칸씩 hole을 향해 밀린다. SWAP은 push가
                        길을 못 열 때다. 보통 blocker가 <InlineMath math="U"/>의 주차된 agent이기 때문인데,
                        구성상 주차된 agent는 밀 수 없다. 논문은 swap 후보의 POP() 순서를 비워뒀고, 이 저장소는
                        이를 <InlineMath math="A[r]"/>에서 BFS dequeue 순서(index가 아니라 거리순이 아니라 탐색
                        순서)로 고정해 모든 실행이 언어 간 바이트 단위로 동일하다.
                    </p>
                    <p>
                        격자라는 사실이 swap의 기하를 하나 바꾼다: 격자 그래프는 이분그래프이고 삼각형이 없다.
                        그래서 이웃 둘을 맞바꾸는 것은 물리적으로 빈 2×2 block을 필요로 한다. 논문의 Figure-1
                        T자 접합 스케치는 개략도일 뿐이고, 격자에서는 사각형 경우만 존재한다. 이 사실 하나가
                        완전성도 미묘하게 만든다. 다음 섹션이 폭 1 통로가 아무것도 못 푸는 사례로 보여준다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Parameter-free.</strong> No window, no priority re-decision, no tuning.
                            Every choice the algorithm must still make (neighbor order, hole selection, swap
                            candidate iteration) is pinned deterministically in code — that is a design
                            decision of the paper, not an implementation shortcut.
                        </li>
                        <li>
                            <strong>Not optimal — not even close.</strong> The cost is whatever the push and
                            swap dance happens to cost. On the open01_swap scenario below this planner spends
                            38 moves where the joint optimum is 30; on maze01_two it spends 74 against 66.
                            Priority already gave up joint optimality; plan-and-repair gives up even
                            individual optimality, and buys repair ability in exchange.
                        </li>
                        <li>
                            <strong>Complete only under a condition grids don't satisfy.</strong> The paper
                            proves completeness for at most{" "}
                            <InlineMath math="|V|-2"/> agents on graphs with enough connectivity. A grid tree
                            (a width-1 corridor) has no swap site anywhere, so even two agents swapping ends
                            fail honestly here — see the corridor01_head_on preset. The pinned semantics are
                            the faithful algorithm, not an omniscient repair oracle.
                        </li>
                        <li>
                            <strong>Cost:</strong> every primitive is a BFS over free cells and every move is
                            one step, so per-agent work is polynomial in{" "}
                            <InlineMath math="|V|"/> — no joint space anywhere. The expanded_nodes metric of
                            this implementation counts total BFS dequeues across every shortest-path and
                            hole-finding call; the algorithm has no search frontier of its own.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>파라미터 무의존.</strong> 창도, 순서 재결정도, 튜닝도 없다. 알고리즘이
                            결국 정해야 하는 모든 선택(이웃 순서, hole 선택, swap 후보 반복 순서)은 코드에
                            결정론적으로 고정된다. 이건 논문의 설계 판단이지 구현의 편법이 아니다.
                        </li>
                        <li>
                            <strong>최적과 무관하다. 그것도 아주 멀리.</strong> 비용은 push와 swap 춤이 우연히 소모한
                            만큼이다. 아래 open01_swap 시나리오에서 이 planner는 joint 최적 30에 대해 38을
                            쓰고, maze01_two에서는 66에 대해 74를 쓴다. 우선순위가 이미 joint 최적성을 포기했고,
                            plan-and-repair은 개별 최적성까지 포기한다. 그 대가로 복구 능력을 산다.
                        </li>
                        <li>
                            <strong>완전성은 격자가 만족하지 않는 조건 아래에서만.</strong> 논문은 충분한 연결성을
                            가진 그래프에서 agent 수 ≤{" "}
                            <InlineMath math="|V|-2"/>일 때 완전성을 증명한다. 격자 트리(폭 1 통로)는 어디에도
                            swap 자리가 없으므로 폭 1 통로의 정면 교환 두 agent도 여기서 정직하게 실패한다.
                            corridor01_head_on preset을 보라. 고정된 semantics는 faithful한 알고리즘이지 전지전능한
                            복구 oracle이 아니다.
                        </li>
                        <li>
                            <strong>비용:</strong> 모든 primitive는 자유 셀 위 BFS이고 모든 이동은 한 스텝이라,
                            agent당 작업량은 <InlineMath math="|V|"/>에 대해 다항이다. joint 공간은 어디에도 없다.
                            이 구현의 expanded_nodes metric은 모든 최단경로·hole 탐색 호출의 dequeue 총합을 센다.
                            이 알고리즘엔 자체 search frontier가 없기 때문이다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The procedure is one loop over agents in index order, and inside it a push attempt first,
                    a swap attempt second. Everything else — the chain shift, the pair walk, the four-move
                    exchange, the rollback of failed candidates — is machinery in service of those two verbs.
                </p>}
                ko={<p>
                    절차는 index 순서로 agent를 도는 루프 하나이고, 그 안에서 push를 먼저 시도하고 swap을 나중에
                    시도한다. 나머지 전부(사슬 밀기, 짝 걷기, 네 이동 교환, 실패한 후보의 rollback)는 이 두
                    동사를 위한 장치다.
                </p>}
            />
            <Pseudocode code={`for each agent r in index order:                                        # 1
    while A[r] ≠ T[r]:
        PUSH(r):                                                 # 2
            p* ← BFS shortest path from A[r] to T[r]             # 3
            walk r along p*; when the next cell v is occupied:   # 4
                BFS from the blocker avoiding {A[r]} ∪ U;        # 5
                first empty cell in enqueue order = hole, and    #
                the chain's occupants shift toward it            #
        if PUSH failed → SWAP(r):                                # 6
            s ← occupant of p*[1] (the blocker of r's first step)
            for each candidate vertex v in BFS-dequeue order:    # 7
                MULTIPUSH: r leads along path(A[r], v), s follows,
                third-party occupants are chain-pushed away      # 8
                CLEAR + EXECUTE_SWAP on a free 2×2 block:        # 9
                    r:v→w2, s:w1→v, r:w2→w4, r:w4→w1             #
            if T[s] ∈ U: RESOLVE — push/swap r off T[s], send s home   # 10`}
            />
            <T
                en={<ol>
                    <li>Agents are served in index order — the priority survives as ordering only. When agent{" "}
                        <InlineMath math="r"/> reaches its goal, that cell joins{" "}
                        <InlineMath math="U"/> and becomes unshovable.</li>
                    <li>Push is always tried first. It succeeds unless the blocker simply cannot be moved out
                        of the way — which in practice means the blocker is a parked agent from{" "}
                        <InlineMath math="U"/>, since the hole-finding BFS treats{" "}
                        <InlineMath math="\{A[r]\} \cup U"/> as walls and cannot even start inside them.</li>
                    <li>The shortest path is plain BFS over free cells in fixed neighbor order (up/down/left/
                        right; the wait self-loop is not an action here). The parent chain doubles as the push
                        chain.</li>
                    <li>r walks while the next cell is empty. The moment it is occupied, PUSH turns into a
                        chain shift and r waits one step while the column ahead rearranges.</li>
                    <li>The hole is the first empty cell in BFS enqueue order from the blocker — pinned so that
                        every language picks the same hole. Occupants on the blocker→hole parent chain move farthest-first,
                        so each target cell is already empty when its mover steps.</li>
                    <li>SWAP exchanges r with s, the occupant of the very first step of{" "}
                        <InlineMath math="p^*"/>. Failed candidates are speculative: their moves land on a local
                        segment that is rolled back (assignment and trace history both); only a successful swap's
                        EXECUTE_SWAP moves join the solution directly.</li>
                    <li>Candidates are iterated in BFS-dequeue order from{" "}
                        <InlineMath math="A[r]"/> — nearest first, deterministic. A candidate fails if s sits on
                        the path to it (the pair can never walk past its own member) or if CLEAR cannot free a block.</li>
                    <li>MULTIPUSH walks the pair: r leads along the path, s follows into each vacated cell, and
                        third-party occupants of that path are chain-pushed away — U is ignored here, because a
                        swap may disturb parked agents and the replay restores them.</li>
                    <li>CLEAR + EXECUTE_SWAP fused: with r on v and s on its neighbor w1, the exchange exists iff
                        some <InlineMath math="w_2 \in N(v) \setminus \{w_1\}"/> and{" "}
                        <InlineMath math="w_4 \in N(w_1) \setminus \{v\} \cap N(w_2)"/> are clearable (an occupant is
                        cleared by stepping into its own first free neighbor). Then four moves round the block.</li>
                    <li>If the swapped-away agent s was already at its goal, both must be restored: push/swap r off{" "}
                        <InlineMath math="T[s]"/>, then send s home the same way. A failing swap here invalidates
                        the original swap — honest failure.</li>
                </ol>}
                ko={<ol>
                    <li>agent는 index 순서로 처리한다. 우선순위는 순서로서만 남는다. agent{" "}
                        <InlineMath math="r"/>이 goal에 도착하면 그 셀은 <InlineMath math="U"/>에 합류하고
                        밀 수 없는 셀이 된다.</li>
                    <li>push를 항상 먼저 시도한다. push는 blocker를 길에서 치울 수 없을 때만 실패하는데, 실질적으로는
                        blocker가 <InlineMath math="U"/>의 주차된 agent일 때다. hole 찾기 BFS가{" "}
                        <InlineMath math="\{A[r]\} \cup U"/>를 벽으로 취급해 그 안에서는 시작조차 못 하기 때문이다.</li>
                    <li>최단경로는 고정 이웃 순서(up/down/left/right)의 자유 셀 plain BFS다. wait self-loop는 여기선
                        액션이 아니다. parent 체인이 곧 push 체인이 된다.</li>
                    <li>빈 셀인 동안 r은 걷는다. 다음 셀이 점유되는 순간 push는 사슬 밀기로 바뀌고, r은 앞 열이
                        재배열되는 동안 한 스텝 대기한다.</li>
                    <li>hole은 blocker에서 BFS 인입 순서상 첫 빈 셀이다. 언어마다 같은 hole을 고르도록 고정된다.
                        blocker→hole parent 체인의 점유자들은 r에서 먼 쪽부터 움직여서 각 대상 셀이 이동자가 밟기
                        전에 이미 비어 있게 된다.</li>
                    <li>SWAP은 r과 s(<InlineMath math="p^*"/>의 첫 수를 막는 점유자)를 맞바꾼다. 실패한 후보는
                        투기적이다: 그 이동들은 지역 segment에 쌓이고 rollback으로 되돌려진다(배정도 trace도).
                        성공한 swap의 EXECUTE_SWAP 이동들만 solution에 직행한다.</li>
                    <li>후보는 <InlineMath math="A[r]"/>에서 BFS dequeue 순서, 즉 가까운 후보부터 결정론적으로 반복된다.
                        s가 그 후보로 가는 경로 위에 있으면(짝이 자기 구성원을 지나갈 수 없다) 후보는 실패하고,
                        CLEAR가 block을 비우지 못해도 실패한다.</li>
                    <li>MULTIPUSH는 짝을 함께 걷힌다: r이 경로를 따라 lead하고 s가 비워진 셀마다 follow로 들어가며,
                        그 경로 위의 제3자 점유자는 chain-push로 치운다. swap은 주차된 agent도 건드릴 수 있고 replay가
                        되돌려주므로 여기서는 U를 무시한다.</li>
                    <li>CLEAR + EXECUTE_SWAP 융합: r이 v에, s의 이웃 w1에 있을 때, 교환은 어떤{" "}
                        <InlineMath math="w_2 \in N(v) \setminus \{w_1\}"/>과{" "}
                        <InlineMath math="w_4 \in N(w_1) \setminus \{v\} \cap N(w_2)"/>이 clearable할 때만 존재한다
                        (점유자는 자기 첫 빈 이웃으로 한 칸 물러남). 그러면 네 이동이 block을 돈다.</li>
                    <li>맞바꿔 쫓겨난 s가 이미 goal에 있던 agent면 둘 다 되돌려야 한다. r을 <InlineMath math="T[s]"/>에서
                        밀어내고 같은 방식으로 s를 집으로 보낸다. 여기서 swap이 실패하면 원래 swap까지 무효화된다.
                        정직한 실패.</li>
                </ol>}
            />

            <h2>{t("What It Guarantees, What It Cannot", "보장하는 것, 못 하는 것")}</h2>
            <T
                en={<p>
                    The guarantee is conditional completeness — enough free cells on a connected-enough graph and
                    the planner always finds a plan. The failure modes are equally precise: a grid tree has no
                    swap site at all, so the completeness condition fails not in the proof but in the geometry.
                    Expand the proofs for why a swap needs a 2×2 block on grids, and for what the honest failure
                    actually is here.
                </p>}
                ko={<p>
                    보장은 조건부 완전성이다. 충분히 연결된 그래프에 빈 셀이 충분하면 planner는 항상 계획을 찾는다.
                    실패도 똑같이 정확하다: 격자 트리에는 swap 자리가 아예 없으므로, 완전성 조건은 증명에서가 아니라
                    기하에서 무너진다. 왜 격자의 swap에 2×2 block이 필요한지, 여기서 정직한 실패가 실제로 무엇인지
                    증명을 펼쳐 보라.
                </p>}
            />
            <Proof title={t("Lemma (a grid swap needs a free 2×2 block)", "보조정리 (격자의 swap에 빈 2×2 block이 필요한 이유)")}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> Two adjacent agents on cells <InlineMath math="v, w_1"/> can
                            exchange cells using only single-cell moves through empty intermediate cells iff some{" "}
                            <InlineMath math="w_2 \in N(v) \setminus \{w_1\}"/> and{" "}
                            <InlineMath math="w_4 \in N(w_1) \setminus \{v\} \cap N(w_2)"/> exist. On a 4-connected
                            grid that is exactly a free 2×2 block.
                        </p>
                        <p>
                            <strong>Proof.</strong> An exchange is a cycle: r leaves v toward some neighbor{" "}
                            <InlineMath math="w_2 \neq w_1"/>, s must then enter v, and to reach v without standing
                            on w1's cell s must arrive from a common neighbor{" "}
                            <InlineMath math="w_4"/> of v and w2 — so the four cells form a 4-cycle. Grid graphs are
                            bipartite (color by parity of row+col): every cycle has length ≥ 4, no triangle hosts an
                            exchange, and the smallest possible witness is exactly the square{" "}
                            <InlineMath math="\{v, w_1, w_2, w_4\}"/>. A width-1 corridor contains no 4-cycle at all,
                            which is why corridor01_head_on is unsolvable for this algorithm even though{" "}
                            <InlineMath math="n = |V|-2"/> holds there. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 이웃 셀 <InlineMath math="v, w_1"/>에 있는 두 agent가 빈 중간
                            셀을 통한 한 칸 이동만으로 맞바꿀 수 있을 필요충분조건은 어떤{" "}
                            <InlineMath math="w_2 \in N(v) \setminus \{w_1\}"/>과{" "}
                            <InlineMath math="w_4 \in N(w_1) \setminus \{v\} \cap N(w_2)"/>이 존재하는 것이다.
                            4연결 격자에서 그것은 정확히 빈 2×2 block이다.
                        </p>
                        <p>
                            <strong>증명.</strong> 교환은 사이클이다: r이 <InlineMath math="w_2 \neq w_1"/>로
                            떠나고, s는 이어 v에 들어가야 하고, s가 w1의 셀을 밟지 않고 v에 닿으려면 v와 w2의 공통
                            이웃 <InlineMath math="w_4"/>에서 와야 한다. 네 셀이 4-cycle을 이룬다. 격자 그래프는
                            이분그래프이고(row+col 홀짝으로 색칠), 모든 사이클은 길이 ≥4이며 삼각형은 어떤 교환도
                            담지 못한다. 가능한 최소 목격자는 정확히 사각형{" "}
                            <InlineMath math="\{v, w_1, w_2, w_4\}"/>이다. 폭 1 통로는 4-cycle이 아예 없으므로 이
                            알고리즘에게 corridor01_head_on은 <InlineMath math="n = |V|-2"/>가 성립해도 불가능하다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Proposition (honest failure, not a bug)", "명제 (버그가 아니라 정직한 실패)")}>
                <T
                    en={<p>
                        <strong>Claim.</strong> When the free cells form a tree and two agents must exchange ends,
                        this planner reports failure — and that is the faithful reading of the paper, not an
                        implementation gap.
                    </p>}
                    ko={<p>
                        <strong>주장.</strong> 자유 셀이 트리를 이루고 두 agent가 끝을 맞바꿔야 할 때 이 planner는
                        실패를 보고한다. 그리고 그것은 논문의 faithful한 해석이지 구현의 구멍이 아니다.
                    </p>}
                />
                <T
                    en={<p>
                        On a tree every vertex of degree ≥ 3 still has no 4-cycle anywhere, so by the lemma no swap
                        exists at any site; push cannot move a parked agent by construction because{" "}
                        <InlineMath math="U"/> cells are walls for its hole-finding BFS. The paper's completeness
                        theorem assumes connectivity that trees do not have. An implementation that "solved" the
                        corridor head-on case would be doing something other than Push and Swap, so this one reports{" "}
                        <InlineMath math="\text{success} = \text{false}"/> with the expansions counted up to the
                        failure — honestly, exactly like the prioritized branch before it.
                    </p>}
                    ko={<p>
                        트리에서는 degree ≥ 3 vertex도 어디에도 4-cycle이 없으므로, 보조정리에 의해 어떤 자리에서도
                        swap이 존재하지 않는다. push는 주차된 agent를 구성상 밀 수 없다. hole 찾기 BFS가{" "}
                        <InlineMath math="U"/> 셀을 벽으로 취급하기 때문이다. 논문의 완전성 정리는 트리가 가지지 않는
                        연결성을 가정한다. 통로 정면 교환을 "풀었다"고 하는 구현은 Push and Swap가 아닌 다른 것을
                        하고 있는 것이고, 그래서 이 구현은 실패까지 센 확장 수와 함께{" "}
                        <InlineMath math="\text{success} = \text{false}"/>를 정직하게 보고한다. 앞선 우선순위 갈래와
                        똑같이.
                    </p>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs this planner live in your browser — the same engine, byte-for-byte what
                    the Python/C++ code below emits. Draw walls, drag a numbered dot or its ring to move an agent's
                    start/goal, add agents; every edit re-plans instantly and replays from step 0. The presets are
                    chosen for what each primitive is: <code>open01_cross</code> never interferes at all — agent 0
                    walks row 10 first and parks, agent 1 crosses the same cell later, so both keep their
                    unconstrained shortest costs (16 + 17). In <code>pocket01_swap</code> a width-1 corridor with one
                    pocket shows the whole swap anatomy: push shoves the blocker into the pocket, and when the second
                    agent's route runs through the parked goal, the pair walks together to the block corner and the
                    four-move rotation exchanges them (8 + 6 moves). <code>corridor01_head_on</code> is the same head-on
                    swap with the pocket removed — no 2×2 block exists anywhere on that tree, and the planner fails
                    honestly. In <code>open01_swap</code> the swap happens mid-corridor: agent 0's parked body at{" "}
                    <InlineMath math="(10,16)"/> forces a real exchange around the wall corner, costing 38 where the
                    joint optimum is 30. And <code>maze01_two</code> threads both agents through one corridor gap in
                    opposite directions — 74 moves against an optimum of 66. Watch what the replay shows: this planner
                    has no search frontier, so there are no expansion blooms — only execution, including every push
                    shove and swap rotation that a prioritized planner could never have performed at all.
                </p>}
                ko={<p>
                    아래 sandbox는 이 planner를 브라우저에서 직접 실행합니다. 아래 Python/C++ 코드가 내뱉는 것과
                    바이트 단위로 같은 엔진입니다. 벽을 그리고, 번호가 적힌 점이나 그 링을 끌어 agent의 start/goal을
                    옮기고, agent를 더하면 모든 편집이 즉시 재계획되고 재생은 스텝 0부터 다시 돕니다. preset들은 각
                    primitive가 무엇인지로 골랐습니다. <code>open01_cross</code>는 충돌이 아예 없습니다. agent 0이
                    row 10을 먼저 걷고 주차하고, agent 1이 같은 셀을 나중에 지나므로 둘 다 제약 없는 최단 비용을
                    유지합니다(16 + 17). <code>pocket01_swap</code>은 폭 1 통로에 pocket 하나가 붙은 맵으로 swap의
                    해부학 전부를 보여줍니다. push가 blocker를 pocket으로 밀어 넣고, 두 번째 agent의 경로가 주차된
                    goal을 관통하자 짝이 block 모서리까지 함께 걷고 네 이동 rotation이 둘을 맞바꿉니다(8 + 6 이동).
                    <code>corridor01_head_on</code>은 pocket을 지운 같은 정면 교환이고, 그 트리 어디에도 2×2 block이
                    없으므로 planner는 정직하게 실패합니다. <code>open01_swap</code>에서는 swap이 통로 중간에서
                    일어납니다. agent 0의 주차된 몸이 <InlineMath math="(10,16)"/>에 자리 잡아 벽 모서리에서의 실제
                    교환을 강제하고, joint 최적 30에 38을 씁니다. <code>maze01_two</code>는 두 agent를 폭 1 통로 gap으로
                    반대 방향으로 통과시키고, 최적 66에 74를 씁니다. 재생이 보여주는 것을 보세요: 이 planner엔 search
                    frontier가 없어서 확장 bloom이 없고, 실행만 흐릅니다. 우선순위 planner라면 절대 할 수 없었을 밀기와
                    rotation을 포함해서.
                </p>}
            />
            <Sandbox label={t(
                "Live push_and_swap sandbox — the browser engine is a byte-identical mirror of the Python/C++ planner. Draw walls, drag endpoints, add agents; every edit re-plans and replays",
                "라이브 push_and_swap sandbox. 브라우저 엔진은 Python/C++ planner와 바이트 단위로 동일한 미러입니다. 벽을 그리고, endpoint를 끌어 옮기고, agent를 더하면 모든 편집이 즉시 재계획과 재생으로 이어집니다",
            )} presets={PRESETS} run={runLive}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The Python and C++ implementations are line-for-line mirrors of each other, and the browser
                    engine that powers the live sandbox above is a third mirror. Same fixed neighbor order, same
                    BFS-enqueue-order tie-breaks for hole selection and candidate iteration, same rollback semantics —
                    so all three produce byte-identical traces on every scenario, which{" "}
                    <code>check-engine-parity</code> verifies on every build. The code below is the actual source, not
                    an excerpt. One implementation note worth knowing: Python's parent dict doubles as parent map and
                    enqueue-order list (dict insertion order IS discovery order), so the C++ mirror carries both a
                    parent map and an explicit order vector — forgetting either half breaks every "first empty cell in
                    enqueue order" scan.
                </p>}
                ko={<p>
                    Python과 C++ 구현은 서로 줄 대 줄 미러이고, 위 라이브 sandbox를 움직이는 브라우저 엔진이 세 번째
                    미러다. 동일한 고정 이웃 순서, 동일한 hole 선택·후보 반복의 BFS 인입 순서 tie-break, 동일한 rollback
                    semantics. 그래서 셋 모두 모든 시나리오에서 바이트 단위로 동일한 trace를 만들고, 빌드마다{" "}
                    <code>check-engine-parity</code>가 이를 검증한다. 아래 코드는 발췌가 아니라 실제 소스 그대로다.
                    알아 둘 구현 노트 하나: Python의 parent dict는 parent 지도와 인입 순서 목록을 겸합니다(dict 삽입
                    순서가 곧 발견 순서). 그래서 C++ 미러는 parent 지도와 명시적 order 벡터를 따로 들고 가고, 어느 한쪽을
                    빼먹으면 "인입 순서상 첫 빈 셀" 스캔이 전부 무너진다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/mrmp/search/push_and_swap.py",
                                code: pyImpl,
                                href: `${REPO}/blob/main/python/mrmp/search/push_and_swap.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/mrmp/search/push_and_swap.hpp",
                                code: cppHeader,
                                href: `${REPO}/blob/main/cpp/include/mrmp/search/push_and_swap.hpp`,
                            },
                            {
                                name: "cpp/src/search/push_and_swap.cpp",
                                code: cppImpl,
                                href: `${REPO}/blob/main/cpp/src/search/push_and_swap.cpp`,
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
                    M. Luna, K. Bekris,{" "}
                    <a href="https://doi.org/10.5591/978-1-57735-516-8/IJCAI11-059" target="_blank" rel="noopener noreferrer">
                        <em>Push and Swap: Pushing the Limits of Multi-Robot Path Planning</em>
                    </a>,
                    IJCAI 2011.
                </li>
            </ol>
        </>
    )
}

export default PushAndSwap
