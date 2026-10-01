import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";

// kinodynamic 갈래 소개 페이지 — 계보의 종착점. 계획이라는 매개체가 다시 등장하지만 이번엔
// 시간을 실는다: search 갈래의 이산 계획을 입력으로 받아 각 agent의 속도 한계에 맞는 실행
// 스케줄로 변환한다(MAPF-POST).앞의 두 갈래의 생산물과 decentralized 갈래의 관점이 여기서 만난다.
const KinodynamicPlanning = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    The genealogy ends where it started, with a plan — but a different object than every plan before it.
                    All three earlier branches produce cell sequences indexed by discrete steps: the search branch
                    enumerates them exactly, the sampling branch samples them out of continuous space, and the
                    decentralized branch deleted the artifact entirely and negotiated one step at a time. This branch
                    takes the discrete plan back as <em>input</em> and asks the question none of them answered: when, in
                    continuous time, does each robot actually enter, dwell on, and leave each location — given that it
                    moves at its own finite velocity limit? The artifact is no longer a path but a schedule: routes with
                    waits deleted plus one earliest arrival time per retained location. The plan's collision-freeness
                    becomes precedence constraints between time intervals, and the robot stops being a cell-hopper: it
                    dwells until its departure instant and traverses at exactly <InlineMath math="v_k"/>.
                </p>}
                ko={<p>
                    계보는 계획에서 시작했던 자리로 돌아와 끝난다 — 다만 이전의 모든 계획과는 다른 물건이다. 앞의 세 갈래는
                    전부 이산 스텝으로 인덱스된 셀 수열을 생산했다: search 갈래는 그것을 정확히 열거하고, sampling 갈래는
                    연속 공간에서 그것으로 샘플링하고, decentralized 갈래는 매개체 자체를 지워버리고 스텝별로 협상했다. 이
                    갈래는 이산 계획을 <em>입력</em>으로 되돌려 받고 그들 아무도 답하지 않은 질문을 묻는다: 각 로봇이 자기
                    속도 한계에서 실제로 언제 각 위치로 진입하고 머물고 떠나느냐. 생산물은 더 이상 경로가 아니라 스케줄이다:
                    대기를 지운 route에 유지된 위치마다 가장 빠른 도착 시각 하나씩. 계획의 충돌 없음은 시간 구간들 사이의
                    precedence 제약이 되고, 로봇은 셀 점프꾼을 그만둔다: 출발 시각까지 머물고 정확히 <InlineMath math="v_k"/>로
                    이동한다.
                </p>}
            />

            <h2>{t("The Problem, With a Clock", "시계를 얹은 문제")}</h2>
            <T
                en={<>
                    <p>
                        The stage is still the search branch's graph <InlineMath math="G = (V, E)"/> with{" "}
                        <InlineMath math="k"/> point agents and fixed goals — what changes is what counts as a solution.
                        Each agent now carries a velocity limit <InlineMath math="v_k"/>, and a solution is a route per
                        agent plus arrival times: execution dwells on cell <InlineMath math="c_i"/> until{" "}
                        <InlineMath math="D_i = t(c_{i+1}) - 1/v_k"/>, then traverses at exactly{" "}
                        <InlineMath math="v_k"/>. Safety is no longer "no two agents name the same cell in the same
                        step" but a positive geometric margin: every protected neighborhood around a cell stays time-disjoint
                        from every other visitor's, so agents keep graph distance strictly above zero at every instant.
                    </p>
                    <p>
                        And the failure mode changes character. This branch plans nothing by itself — it post-processes.
                        Given a collision-free discrete plan it always produces a schedule (the constraint network is
                        guaranteed consistent), and where no discrete plan exists there is simply nothing to post-process:
                        the inherited solver's exhaustion comes through as an honest failure with zeroed metrics, never as
                        an invented trajectory. The question this branch answers is "given that you already have a plan,
                        what does executing it at real velocities cost and guarantee" — not "can these agents be routed".
                    </p>
                </>}
                ko={<>
                    <p>
                        무대는 여전히 search 갈래의 그래프 <InlineMath math="G = (V, E)"/>에 <InlineMath math="k"/>대의 점
                        agent와 고정 goal — 바뀌는 것이 해로 무엇이 인정되느냐다. 이제 각 agent는 속도 한계{" "}
                        <InlineMath math="v_k"/>를 들고 다니고, 해는 agent별 route에 도착 시각들이다: 실행은 셀{" "}
                        <InlineMath math="c_i"/>에 출발 시각 <InlineMath math="D_i = t(c_{i+1}) - 1/v_k"/>까지 머물다가
                        그다음 정확히 <InlineMath math="v_k"/>로 이동한다. 안전은 더 이상 "두 agent가 같은 스텝에 같은 셀을
                        부르지 않는다"가 아니라 양수 기하 여유다: 셀 주변의 모든 보호 근방이 다른 방문자의 시간 구간과 서로
                        disjoint라서, 어떤 순간에나 agent 간 그래프 거리가 0보다 엄격히 위에 유지된다.
                    </p>
                    <p>
                        실패 모드도 성격이 바뀐다. 이 갈래는 혼자서 아무것도 계획하지 않는다 — 후처리만 한다. 충돌 없는 이산
                        계획이 주어지면 제약 네트워크가 구성상 일관돼 있어 스케줄은 항상 생산되고, 이산 계획 자체가 없는 곳에서는
                        후처리할 게 그냥 없다: 상속된 solver의 고갈은 발명된 trajectory가 아니라 지표를 0으로 만든 정직한 실패로
                        그대로 통과한다. 이 갈래가 답하는 질문은 "계획을 이미 가졌다면, 실제 속도로 실행했을 때 그게 얼마이고 무엇을
                        보장하는가"이지 "이 agent들을 라우팅할 수 있는가"가 아니다.
                    </p>
                </>}
            />

            <h2>{t("Why Post-Process", "왜 후처리인가")}</h2>
            <T
                en={<p>
                    Because discretization's lie has a cheap fix and an expensive one, and this is the cheap one. The
                    expensive fix is the sampling branch: leave the grid entirely and plan in continuous configuration
                    space, paying dimension for the privilege. The cheap fix keeps everything enumeration already bought —
                    exactness, optimality on the grid, completeness up to a budget — and repairs only the lie itself:
                    steps become intervals, cell occupancy becomes protected clouds with positive radius, and simultaneous
                    step-indexing becomes precedence between those intervals. The repair is exact because it introduces no
                    approximation: a Simple Temporal Network over the plan's own event graph has an earliest schedule for
                    every feasible instance, computed by one fixed-point pass. What you get is honest about both sides —
                    the discrete plan's optimality survives as the schedule's cost floor, and the discrete plan's failures
                    survive unchanged. The decentralized branch answered discretization by abandoning the plan; this branch
                    answers it by refusing to let the plan pretend time doesn't exist. Two opposite repairs, and the
                    genealogy needs both on the record.
                </p>}
                ko={<p>
                    이산화의 거짓말에는 값싼 고침과 비싼 고침이 있고, 이건 값싼 쪽이기 때문이다. 비싼 고침은 sampling 갈래다:
                    격자를 완전히 떠나 연속 configuration space에서 계획하고, 특권의 대가로 차원을 지불한다. 값싼 고침은 열거가
                    이미 산 것 — 정확성, 격자 위 최적성, 예산까지의 완전성 —을 전부 지키고 거짓말 자체만 수리한다: 스텝은 구간이
                        되고, 셀 점유는 양수 반지름의 보호 구름이 되고, 같은 스텝 번호 공유는 구간들 사이의 precedence가 된다. 수리는
                    근사가 없어서 정확하다: 계획 자기 이벤트 그래프 위의 Simple Temporal Network는 feasible한 모든 인스턴스에 가장
                    빠른 스케줄을 갖고 있고 그게 고정점 패스 하나로 계산된다. 얻는 것은 양쪽 모두에게 정직하다 — 이산 계획의 최적성은
                    스케줄 비용 하한으로 살아남고, 이산 계획의 실패는 그대로 살아남는다. decentralized 갈래가 이산화에 대한 답으로
                    계획을 버렸고, 이 갈래는 계획이 시간이 존재하는 척하지 못하게 하는 것으로 답한다. 정반대의 두 수리이고, 계보는
                    둘 다 기록에 남겨야 한다.
                </p>}
            />

            <h2>{t("What the Search Branch Hands Over", "search 갈래가 넘겨주는 것")}</h2>
            <T
                en={<p>
                    The handover is literal: the underlying solver in this branch's implementation is the search branch's
                    CBS, running silently with no trace of its own, and everything it was — sum-of-costs optimal on the
                    grid, complete up to an honest budget, incomplete exactly where geometry defeats it — arrives attached
                    to the schedule. The same five maps run here as everywhere else in this repository; the same head-on
                    swap that CBS resolves now gets scheduled instead of re-planned, and the width-1 corridor that defeats
                    CBS still produces nothing here. What the handover changes is what a conflict <em>means</em>: in the
                    search branch two agents naming the same cell at different steps is a non-event, and here it becomes
                    the entire content of the problem — an ordering constraint between protected clouds, whose slack is
                    measured in seconds instead of steps. The search branch optimized cost; this branch reads that cost
                    as a floor and asks what executing it actually looks like when robots have velocity limits.
                </p>}
                ko={<p>
                    인수는 문자 그대로다: 이 갈래 구현의 기반 solver는 search 갈래의 CBS이고, 자기 trace도 없이 조용히 돌아가며,
                    그것이었던 모든 것 — 격자에서 sum-of-costs 최적, 정직한 예산까지 완전, 기하가 이기는 곳에서 정확히 불완전 —이
                    스케줄에 붙어 온다. 이 저장소 어디나와 같은 다섯 맵이 여기서도 돌아가고, CBS가 풀던 그 정면 교환은 이제 재계획
                    대신 스케줄링되며, CBS를 무너뜨린 폭 1 통로는 여기서도 아무것도 생산하지 않는다. 인수가 바꾸는 것은 conflict가
                    <em>의미하는 것</em>이다: search 갈래에서 두 agent가 다른 스텝에 같은 셀을 부르는 건 무사건이었고, 여기서는 그게
                    문제의 전부가 된다 — 보호 구름들 사이의 순서 제약이고, 그 여유는 스텝이 아니라 초로 측정된다. search 갈래는 비용을
                    최적화했고, 이 갈래는 그 비용을 하한으로 읽은 뒤, 로봇에게 속도 한계가 있을 때 실행이 실제로 어떻게 보이는지를 묻는다.
                </p>}
            />

            <h2>{t("What Is Written, and Where the Genealogy Ends", "집필된 것과 계보가 끝나는 지점")}</h2>
            <T
                en={<p>
                    One member is written and implemented: <strong>MAPF-POST</strong> — Multi-Agent Path Finding with
                    Kinematic Constraints (Hönig, Kumar, Cohen, Ma, Xu, Ayanian & Koenig) — the Temporal Plan Graph to
                    STN conversion described above, in three mirrored implementations. Read it after CBS: same maps, same
                    plans underneath, and now the comparison is direct — what a schedule makes of a plan you already
                    proved optimal, what unequal velocities do to a crossing, and where waiting was wasted. The paper's
                    non-holonomic extension (orientation vertices, rotate actions) stays out of scope: this repository's
                    robots remain point agents on a grid, so the kinodynamic layer here is exactly velocity limits and
                    dwell semantics. With this branch the genealogy is complete at four: enumerate, sample, abandon the
                    plan, or hand it back a clock — every position the survey's axes admit, each implemented in Python,
                    C++, and the browser engine you can run on every page of this site.
                </p>}
                ko={<p>
                    한 회원이 집필·구현됐다: <strong>MAPF-POST</strong>(Multi-Agent Path Finding with Kinematic
                    Constraints, Hönig·Kumar·Cohen·Ma·Xu·Ayanian & Koenig) — 위에서 설명한 Temporal Plan Graph에서 STN으로의
                    변환, 세 개 미러 구현으로. CBS 뒤에 이것을 읽어라: 같은 맵, 아래 같은 계획, 그리고 이제 비교가 직접
                    가능해진다 — 최적임을 증명한 계획으로 스케줄이 무엇을 만드는지, 서로 다른 속도가 교차에 무엇을 하는지, 어디에서
                    대기가 낭비였는지. 논문의 non-holonomic 확장(orientation 꼭짓점, rotate 동작)은 범위 밖으로 남는다: 이 저장소의
                    로봇은 격자 위 점 agent로 남아 있고, 그래서 여기의 kinodynamic 층은 정확히 속도 한계와 dwell semantics다. 이
                    갈래로 계보가 넷으로 완성된다: 열거하거나, 샘플링하거나, 계획을 버리거나, 계획에 시계를 되돌려주거나 — survey의
                    축들이 허용하는 모든 자리 각각을 Python과 C++와, 이 사이트 모든 페이지에서 직접 돌릴 수 있는 브라우저 엔진으로.
                </p>}
            />
        </>
    )
}

export default KinodynamicPlanning
