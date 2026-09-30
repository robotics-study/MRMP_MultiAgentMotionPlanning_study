import {GridMap} from "../grid";
import {Cell, TraceEvent} from "../trace/types";

// 브라우저 라이브 데모용 Push and Rotate. python/mrmp/search/push_and_rotate.py의 정확한
// 미러 — 고정 이웃 순서(up/down/left/right), row-major 스캔(sorted(free) = 행→열 순회),
// Map 삽입 순서가 곧 BFS 인입(dequeue) 순서(Python dict와 동일)이라 모든 타이브레이크가
// 언어 간에 일치한다. 시각화용이 아니라 parity 검증의 한 축이다(scripts/check-engine-parity.mjs).

// 실행된 이동 하나 (agent, from, to) — rotate/swap이 rollback 전에 기록하는 단위.
type Move = [number, Cell, Cell];

interface Sim {
    // 자유 셀 판정. 경계 밖은 자유 셀 집합에 애초부터 없다(Python의 frozenset 멤버십과 동일).
    free: (c: Cell) => boolean;
    // 그리드 크기 — free 셀의 row-major 스캔(Python의 sorted(free))이 전체 좌표를 훑는다.
    w: number;
    h: number;
    // 현재 배정(agent index → 셀)과 목표 배정. 배정은 단사다.
    A: Cell[];
    T: Cell[];
    // 실행 기록. 이동마다 새 배정이 append되고, 투기적 swap/rotate 시도의 rollback은 이 목록을 자른다.
    Pi: Cell[][];
    // 모든 path/clear_vertex BFS의 dequeue 총합 — 이 알고리즘엔 자체 search frontier가 없으니
    // expanded_nodes metric이 곧 이것이다(분해 DFS와 merge BFS는 세지 않는다).
    expandedNodes: number;
}

const ck = (c: Cell): string => `${c[0]},${c[1]}`;
const cellEq = (a: Cell, b: Cell): boolean => a[0] === b[0] && a[1] === b[1];
// "r,c" 키를 행 우선 수치 순서로 되돌린다 — Python의 sorted(free)/sorted(vi_set) 스캔 대응.
const parseKey = (key: string): Cell => {
    const [r, c] = key.split(",").map(Number);
    return [r, c];
};

// 고정 순서 4-이웃. Python의 _nbrs(free, c)와 동일한 순서다.
const nbrs = (sim: Sim, c: Cell): Cell[] => {
    const out: Cell[] = [];
    for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
        const n: Cell = [c[0] + dr, c[1] + dc];
        if (sim.free(n)) out.push(n);
    }
    return out;
};

// 모든 자유 셀을 row-major 순서로 — Python의 sorted(free) 스캔(clear_vertex hole 후보,
// DFS root, subgraph vertex 순회)과 동일한 순서다.
const freeCellsRowMajor = (sim: Sim): Cell[] => {
    const out: Cell[] = [];
    for (let r = 0; r < sim.h; r++) {
        for (let c = 0; c < sim.w; c++) {
            if (sim.free([r, c])) out.push([r, c]);
        }
    }
    return out;
};

// 셀을 점유하는 agent의 index(배정은 단사), 없으면 null.
const occupant = (sim: Sim, c: Cell): number | null => {
    for (let i = 0; i < sim.A.length; i++) if (cellEq(sim.A[i], c)) return i;
    return null;
};

// 한 행동: agent가 자기 셀에서 빈 자유 셀 to로 한 칸 옮기고 새 배정이 Pi에 붙는다.
// 투기적 swap 후보 안에서는 지역 segment에도 기록돼 시도 전체를 되돌릴 수 있다.
// 모든 호출 지점이 구성하는 불변식 — 검사하고 위반 시 던진다(복구하지 않는다).
const move = (sim: Sim, segment: Move[] | null, agent: number, to: Cell): void => {
    const frm = sim.A[agent];
    if (cellEq(frm, to) || !sim.free(to) || occupant(sim, to) !== null) {
        throw new Error("push_and_rotate: move invariant violated");
    }
    sim.A[agent] = to;
    sim.Pi.push([...sim.A]);
    if (segment !== null) segment.push([agent, frm, to]);
};

interface BfsResult {
    // 삽입 순서 = BFS 인입 순서 = dequeue 순서. Python의 parent dict가 parent 지도와
    // 순서 목록 역할을 겸하므로, 여기서는 순서 배열을 명시적으로 따로 둔다.
    order: Cell[];
    parent: Map<string, Cell | null>;
}

// 알고리즘 10이 쓰는 공용 BFS — blocked를 피해 고정 이웃 순서로 돈다. order.length가
// 이 호출의 확장 수다(모든 인입 셀은 정확히 한 번 dequeue된다).
const bfsParent = (sim: Sim, start: Cell, blocked: Set<string>): BfsResult => {
    const res: BfsResult = {order: [], parent: new Map()};
    if (blocked.has(ck(start))) return res;  // 빈 BFS — 확장은 0으로 계산된다
    res.order.push(start);
    res.parent.set(ck(start), null);
    const queue: Cell[] = [start];
    let head = 0;
    while (head < queue.length) {
        const c = queue[head++];
        for (const n of nbrs(sim, c)) {
            if (res.parent.has(ck(n)) || blocked.has(ck(n))) continue;
            res.parent.set(ck(n), c);
            res.order.push(n);
            queue.push(n);
        }
    }
    sim.expandedNodes += res.order.length;
    return res;
};

// 정적 BFS 최단경로 start → goal (도달 불가면 null). parent 체인을 거슬러 복원한다.
const pathTo = (sim: Sim, start: Cell, goal: Cell, blocked: Set<string>): Cell[] | null => {
    const res = bfsParent(sim, start, blocked);
    if (!res.parent.has(ck(goal))) return null;
    const out: Cell[] = [];
    let cur: Cell = goal;
    for (;;) {
        out.push(cur);
        const p = res.parent.get(ck(cur))!;  // root의 parent는 null — 그 전까진 항상 존재
        if (p === null) break;
        cur = p;
    }
    return out.reverse();
};

// 알고리즘 10(clear_vertex): 빈 셀을 row-major로 스캔해, U를 피해 v에 도달하는 BFS를 가진
// 첫 셀 u를 고르고, 그 parent chain 위의 점유자들이 r에서 먼 쪽부터 한 칸씩 u를 향해 밀린다.
const clearVertex = (sim: Sim, segment: Move[] | null, v: Cell, blocked: Set<string>): boolean => {
    for (const u of freeCellsRowMajor(sim)) {
        if (occupant(sim, u) !== null) continue;  // 빈 vertex만 hole 후보로 스캔된다
        const res = bfsParent(sim, u, blocked);
        if (!res.parent.has(ck(v))) continue;
        const chain: Cell[] = [];
        let cur: Cell = v;
        for (;;) {
            chain.push(cur);
            const p = res.parent.get(ck(cur))!;
            if (p === null) break;
            cur = p;
        }
        chain.reverse();  // [u = x0, ..., xk = v]
        let xp: Cell | null = null;
        for (const x of chain) {
            if (xp !== null) {
                const a = occupant(sim, x);
                if (a !== null) move(sim, segment, a, xp);
            }
            xp = x;
        }
        return true;
    }
    return false;
};

// 알고리즘 4(push): r을 v로 한 칸 걷힌다. v가 점유돼 있으면 먼저 clear_vertex — blocked 집합은
// 끝난 agent들의 셀에 r 자기 셀까지(주차된 agent는 walker가 그 cell을 unreachable-as-a-hole로
// 만든 뒤에야 밀린다).
const push = (sim: Sim, r: number, v: Cell, uSet: Set<string>): boolean => {
    if (occupant(sim, v) !== null) {
        const blocked = new Set(uSet);
        blocked.add(ck(sim.A[r]));
        if (!clearVertex(sim, null, v, blocked)) return false;
    }
    move(sim, null, r, v);
    return true;
};

// 알고리즘 11(multipush): 인접 짝 (r0, s0)을 후보 vertex v로 데려간다. 가까운 쪽이 lead로
// 정적 BFS 경로를 걷고 다른 쪽이 비워진 셀마다 뒤따른다. 고정된 해석은 p[0]을 건너뛴다
// (lead의 시작 셀을 문자 그대로 비우면 lead를 자기 시작 셀에서 밀어내기 때문).
const multipush = (sim: Sim, segment: Move[], r0: number, s0: number, v: Cell): boolean => {
    const pr = pathTo(sim, sim.A[r0], v, new Set());
    const ps = pathTo(sim, sim.A[s0], v, new Set());
    // 가까운 쪽이 lead; 도달 불가 = 무한히 멀음; tie는 첫 번째 인자(r0).
    let r: number, s: number, p: Cell[];
    if (pr !== null && (ps === null || pr.length <= ps.length)) {
        r = r0; s = s0; p = pr;
    } else if (ps !== null) {
        r = s0; s = r0; p = ps;
    } else {
        return false;
    }
    for (let i = 1; i < p.length; i++) {  // 고정: 시작 셀은 건너뛴다
        const vr = sim.A[r], vs = sim.A[s];
        const x = p[i];
        if (occupant(sim, x) !== null) {
            if (!clearVertex(sim, segment, x, new Set([ck(vr), ck(vs)]))) return false;
        }
        move(sim, segment, r, x);
        move(sim, segment, s, vr);
    }
    return cellEq(sim.A[r], v) && nbrs(sim, sim.A[r]).some((n) => cellEq(n, sim.A[s]));
};

// 알고리즘 12(clear): 후보 vertex v의 이웃 둘을 비운다. r은 v 위에 선 짝의 구성원, s는 다른 쪽이고
// v' = A(s). stage 1은 점유된 이웃들의 점유자를 밀어내고, stage 2-4는 Luna와 Bekris가 생략한
// 정교한 경우들이다. stage 2/3는 후보를 고정 순서로 시도하고, 첫 clear_vertex를 통과한 뒤
// 두 번째에서 실패하면 break(다음 stage로)한다.
const clearPair = (sim: Sim, segment: Move[], rp: number, sp: number, v: Cell): boolean => {
    const r = cellEq(sim.A[rp], v) ? rp : sp;  // line 1 (position으로 재도출)
    const s = r === rp ? sp : rp;
    const vp = sim.A[s];  // v' — 구성상 v에 인접한 s의 셀

    let eSet: Cell[] = nbrs(sim, v).filter((n) => occupant(sim, n) === null);  // line 2 (고정 순서)
    if (eSet.length >= 2) return true;  // line 3 — 빈 이웃이 이미 둘: 비울 것이 없다
    const inE = (c: Cell): boolean => eSet.some((e) => cellEq(e, c));

    // Stage 1 (lines 5-9): 점유된 이웃의 점유자를 밀어낸다. 첫 성공이 E를 씨뿌리고 둘째가 끝낸다.
    const stage1 = nbrs(sim, v).filter((n) => !inE(n) && !cellEq(n, vp));
    for (const n of stage1) {
        const blocked = new Set(eSet.map(ck));
        blocked.add(ck(v));
        blocked.add(ck(vp));
        if (clearVertex(sim, segment, n, blocked)) {
            if (eSet.length >= 1) return true;
            eSet.push(n);
        }
    }
    if (eSet.length === 0) return false;  // lines 10-11
    const eps = eSet[0];  // line 12: 유일한 빈 이웃

    // Stage 2 (lines 13-19): 후보 n마다 복사본에서 시도(snapshot/rollback). 바깥 clear_vertex가
    // 실패하면 그냥 다음 후보로 넘어가고, n은 비워졌는데 eps가 안 비우면(line 19) stage 2를 그만 둔다.
    const cands = nbrs(sim, v).filter((n) => !cellEq(n, vp) && !inE(n));
    for (const n of cands) {
        const before = [...sim.A];
        const piLen = sim.Pi.length;
        const seg2: Move[] = [];
        let ok = false;
        try {
            ok = clearVertex(sim, seg2, n, new Set([ck(v), ck(vp)]));
            if (ok && !clearVertex(sim, seg2, eps, new Set([ck(v), ck(vp), ck(n)]))) {
                sim.A = before;  // line 19: rollback(복사본은 끝내 커밋되지 않았다)
                sim.Pi.length = piLen;
                break;  // 안쪽 실패가 stage 2를 멈춘다 — stage 3으로 넘어간다
            }
            if (ok) return true;  // lines 17-18: 커밋은 암시적(라이브 상태가 이미 갱신됐다)
        } catch {
            ok = false;  // 구성상 불가능 — 바깥 clear 실패로 취급한다
        }
        if (!ok) {
            sim.A = before;
            sim.Pi.length = piLen;
        }
    }

    // Stage 3 (lines 20-27): r이 eps로, s가 v로(둘 다 복사본에서) 옮기고, n을 비운 뒤(v'가 계속
    // clearable한지 점검) 성공. 중첩은 stage 2와 같다: 바깥 실패는 다음 후보, 안쪽 실패는 이탈.
    for (const n of cands) {
        const before = [...sim.A];
        const piLen = sim.Pi.length;
        const seg3: Move[] = [];
        let ok = false;
        try {
            move(sim, seg3, r, eps);  // line 22 (r은 v 위에 있다 — eps는 인접)
            move(sim, seg3, s, v);
            ok = clearVertex(sim, seg3, n, new Set([ck(v), ck(eps)]));
            if (ok && !clearVertex(sim, seg3, vp, new Set([ck(v), ck(eps), ck(n)]))) {
                sim.A = before;
                sim.Pi.length = piLen;
                break;  // line 27(line 19와 같은 중첩)
            }
            if (ok) return true;
        } catch {
            ok = false;  // 구성상 불가능 — 바깥 clear 실패로 취급한다
        }
        sim.A = before;
        sim.Pi.length = piLen;
    }

    // Stage 4 (lines 28-36): eps 뒤의 공간을 만들어 n의 점유자가 v를 지나 걷게 하고 r과 s가
    // 비켜선 뒤, eps에서 영구히 밀어낸다. stage 4의 clear가 후보 셀을 비우면 그 점유자는 빈
    // 셀이 된다 — Python의 잡힌 AssertionError처럼 여기서 정직하게 시도가 실패한다.
    try {
        if (!clearVertex(sim, segment, vp, new Set([ck(v)]))) return false;
        move(sim, segment, r, vp);
        if (!clearVertex(sim, segment, eps, new Set([ck(v), ck(vp), ck(sim.A[s])]))) return false;
        const n2 = nbrs(sim, v).filter((n) => !cellEq(n, vp) && !eSet.some((e) => cellEq(e, n)));
        if (n2.length === 0) return false;
        const n = n2[0];  // 고정: 고정 순서 첫 번째("아무 vertex")
        const t = occupant(sim, n);
        if (t === null) return false;  // 잡힌 assertion의 경우 — 시도가 정직하게 실패한다
        move(sim, segment, t, v);      // line 34(v를 지나 ...)
        move(sim, segment, t, eps);    // ... eps까지 — 한 칸 이동 두 번
        move(sim, segment, r, v);
        move(sim, segment, s, vp);
        return clearVertex(sim, segment, eps, new Set([ck(v), ck(vp), ck(n)]));
    } catch {
        return false;
    }
};

// 알고리즘 13(exchange): junction vertex v에서의 물리적 맞교환. v 위에 선 r이 첫 빈 이웃 v1로
// 비키고, s가 v를 지나 v2로 건너가고, r이 v를 다시 지나 s의 옛 셀로 건너오고, s가 v2에서 v로
// 돌아온다. 알짜 효과: 짝이 위치를 맞바꾼다.
const exchange = (sim: Sim, rp: number, sp: number, v: Cell): void => {
    const r = cellEq(sim.A[rp], v) ? rp : sp;  // line 1(position으로 재도출)
    const s = r === rp ? sp : rp;
    const vs = sim.A[s];
    const freeNbrs = nbrs(sim, v).filter((n) => occupant(sim, n) === null);
    if (freeNbrs.length < 2) throw new Error("push_and_rotate: no exchange site");  // 호출자의 후보가 실패한 경우
    const v1 = freeNbrs[0], v2 = freeNbrs[1];
    move(sim, null, r, v1);  // line 5: r이 v를 떠나 첫 빈 이웃으로
    move(sim, null, s, v);   // line 6 단계 1(v를 지나)
    move(sim, null, s, v2);  // line 6 단계 2
    move(sim, null, r, v);   // line 7 단계 1(v를 지나)
    move(sim, null, r, vs);  // line 7 단계 2 — 맞교환이 착지한다
    move(sim, null, s, v);   // line 8: s가 r의 옛 셀로 돌아온다
};

// 알고리즘 5(swap): 인접한 agent 둘을 r의 subgraph에 속한 degree ≥ 3 vertex에서 맞바꾼다
// (subgraph가 없으면 — 배정되지 않은 agent는 결코 swap될 수 없다 — 트리 맵이 정직하게
// 풀리지 않는 이유가 바로 이것이다). 후보는 A[r0]에서 BFS dequeue 순서, 즉 가장 가까운 것부터.
// 투기적 후보는 rollback되고(배정도 trace도), 성공한 후보의 exchange 이동들과 r/s 역할을 바꾼
// 역순 replay만 남는다 — 그 replay가 밀려난 구경꾼들까지 집으로 되돌린다.
const swap = (sim: Sim, r0: number, s0: number, subCells: Set<string> | null): boolean => {
    if (subCells === null) return false;
    const par = bfsParent(sim, sim.A[r0], new Set());  // 고정 후보 순서: A[r0]에서 BFS dequeue
    const cands = par.order.filter((x) => subCells.has(ck(x)) && nbrs(sim, x).length >= 3);
    for (const v of cands) {
        const before = [...sim.A];
        const piLen = sim.Pi.length;
        const segment: Move[] = [];
        let ok: boolean;
        try {
            ok = multipush(sim, segment, r0, s0, v) && clearPair(sim, segment, r0, s0, v);
        } catch {
            ok = false;
        }
        if (!ok) {
            sim.A = before;
            sim.Pi.length = piLen;  // 투기적 이동들을 trace에서도 되돌린다
            continue;
        }
        // 커밋은 암시적이다(A는 라이브로 갱신됐고 Pi엔 이미 붙었다). exchange는 실제 상태에서
        // 실행되고, 그 뒤 기록된 segment 이동들이 r/s 역할을 바꿔 역순 replay된다.
        exchange(sim, r0, s0, v);
        for (let i = segment.length - 1; i >= 0; i--) {
            const [agent, frm, to] = segment[i];
            const who = agent === r0 ? s0 : (agent === s0 ? r0 : agent);
            if (!cellEq(sim.A[who], to)) throw new Error("push_and_rotate: swap replay does not land");
            move(sim, null, who, frm);
        }
        return true;
    }
    return false;
};

// rotate의 캐스케이드: cycle c(순서 = c의 리스트 순서, wrap edge가 닫는다)의 모든 rider가 한 칸
// 앞으로. 빈 slot e를 그 predecessor 위의 rider가 채우고, 그 빈 자리는 또 그 predecessor가 채우는
// 식으로 역전파된다 — 정확히 n-1 방문(c 자체를 빼고 전부), 그래서 모든 rider는 정확히 한 번만
// 움직인다. c[j]=e 자체를 방문하면 처음 움직인 rider가 두 번 움직이므로 루프는 n-1에서 선다.
const cascade = (sim: Sim, c: Cell[], e: Cell): void => {
    const n = c.length;
    let j = -1;
    for (let i = 0; i < n; i++) if (cellEq(c[i], e)) { j = i; break; }
    if (j < 0) throw new Error("push_and_rotate: cascade on a cell outside the cycle");
    const mod = (x: number): number => ((x % n) + n) % n;
    for (let k = 1; k < n; k++) {
        const w = c[mod(j - k)];
        const a = occupant(sim, w);
        if (a !== null) move(sim, null, a, c[mod(j - k + 1)]);
    }
};

// 알고리즘 6(rotate): cycle c 위의 모든 agent를 한 칸 앞으로. phase 1(lines 1-4): 빈 셀이 있으면
// 거기서 캐스케이드. phase 2(lines 5-16, 가득 참): 첫 clearable cycle vertex의 rider를 cycle 밖으로
// 밀어내고, 그 predecessor가 비워진 slot으로 올라오고, 짝을 SWAP하고(rotate는 swap의 junction
// 기계를 빌린다), 새 빈 셀에서 캐스케이드한 뒤 밀어냄을 역순 replay(role 교환) — 밀려난 rider를
// 비워진 slot 위로 되돌리며 rotation을 완성한다.
const rotate = (sim: Sim, c: Cell[], f: (number | null)[], sList: Set<string>[]): boolean => {
    for (const v of c) {  // phase 1: c 순서상 첫 빈 셀
        if (occupant(sim, v) === null) {
            cascade(sim, c, v);
            return true;
        }
    }
    const n = c.length;
    for (let idx = 0; idx < n; idx++) {  // phase 2: c 순서상 첫 clearable vertex
        const v = c[idx];
        const rOpt = occupant(sim, v);
        if (rOpt === null) throw new Error("push_and_rotate: phase 2 on an unoccupied cycle");
        const r = rOpt;
        const seg: Move[] = [];
        const blocked = new Set(c.map(ck));
        blocked.delete(ck(v));
        let cleared = false;
        try {
            cleared = clearVertex(sim, seg, v, blocked);
        } catch {
            cleared = false;
        }
        if (!cleared) continue;
        const vp = c[((idx - 1) % n + n) % n];  // cycle 순서상 predecessor(line 10)
        const rpOpt = occupant(sim, vp);
        if (rpOpt === null) throw new Error("push_and_rotate: predecessor vacated on a full cycle");
        const rp_ = rpOpt;
        move(sim, null, rp_, v);  // line 12: predecessor가 비워진 셀로 올라간다
        const cells = f[r] !== null ? sList[f[r] as number] : null;
        let swapped = false;
        try {
            swapped = swap(sim, r, rp_, cells);
        } catch {
            swapped = false;
        }
        if (!swapped) return false;  // 정직한 실패가 solve까지 전파된다
        cascade(sim, c, vp);  // line 14: 새 빈 셀에서 전부 앞으로
        for (let i = seg.length - 1; i >= 0; i--) {  // line 15: role을 바꿔 역순 replay
            const [agent, frm, to] = seg[i];
            const who = agent === r ? rp_ : (agent === rp_ ? r : agent);
            if (!cellEq(sim.A[who], to)) throw new Error("push_and_rotate: rotate replay does not land");
            move(sim, null, who, frm);
        }
        return true;
    }
    return false;
};

// 알고리즘 1(find_subgraphs): nontrivial biconnected components(Hopcroft-Tarjan DFS, root와
// 이웃은 고정 순서), 뒤덮이지 않은 degree ≥ 3 셀은 singleton subgraph, 그다음 index 순서로
// 쌍을 훑어 cell-set 거리 ≤ m-2인 첫 쌍을 병합(경로는 Si 셀들을 row-major로 심은 multi-source
// BFS가 고정한다).
const findSubgraphs = (sim: Sim, m: number): Set<string>[] => {
    const comps: Set<string>[] = [];
    const disc = new Map<string, number>();
    const low = new Map<string, number>();
    const par = new Map<string, Cell | null>();
    let t = 0;
    for (const root of freeCellsRowMajor(sim)) {  // row-major DFS roots
        if (disc.has(ck(root))) continue;
        t += 1;
        disc.set(ck(root), t);
        low.set(ck(root), t);
        par.set(ck(root), null);
        const edgeStack: Array<[Cell, Cell]> = [];
        interface Frame { u: Cell; nb: Cell[]; idx: number }
        const stack: Frame[] = [{u: root, nb: nbrs(sim, root), idx: 0}];
        while (stack.length > 0) {
            const top = stack[stack.length - 1];
            const u = top.u;
            if (top.idx >= top.nb.length) {
                stack.pop();
                if (stack.length === 0) break;  // 이 DFS tree 완성
                const p = par.get(ck(u))!;
                if (low.get(ck(u))! >= disc.get(ck(p))!) {
                    const comp = new Set<string>();
                    for (;;) {
                        const [ea, eb] = edgeStack.pop()!;
                        comp.add(ck(ea));
                        comp.add(ck(eb));
                        if ((cellEq(ea, p) && cellEq(eb, u)) || (cellEq(eb, p) && cellEq(ea, u))) break;
                    }
                    if (comp.size >= 3) comps.push(comp);  // nontrivial = 사이클 하나 분량의 vertex
                } else {
                    low.set(ck(p), Math.min(low.get(ck(p))!, low.get(ck(u))!));
                }
                continue;
            }
            const w = top.nb[top.idx];
            top.idx += 1;
            if (!disc.has(ck(w))) {
                par.set(ck(w), u);
                t += 1;
                disc.set(ck(w), t);
                low.set(ck(w), t);
                edgeStack.push([u, w]);
                stack.push({u: w, nb: nbrs(sim, w), idx: 0});
            } else {
                // Python: elif w != par[u] and disc[w] < disc[u] — u는 이미 발견됐으니 par[u]는 항상 존재한다
                const pu = par.get(ck(u))!;
                if (!(pu !== null && cellEq(w, pu)) && disc.get(ck(w))! < disc.get(ck(u))!) {
                    edgeStack.push([u, w]);
                    low.set(ck(u), Math.min(low.get(ck(u))!, disc.get(ck(w))!));
                }
            }
        }
    }
    // covered는 nontrivial comps만으로 한 번 계산된다(singleton은 covered를 늘리지 않는다).
    const covered = new Set<string>();
    for (const comp of comps) for (const key of comp) covered.add(key);
    for (const v of freeCellsRowMajor(sim)) {  // row-major: 뒤덮이지 않은 join vertex의 singleton
        if (nbrs(sim, v).length >= 3 && !covered.has(ck(v))) comps.push(new Set([ck(v)]));
    }
    let changed = true;
    while (changed) {  // 병합 루프(Alg 1 lines 3-5), 고정된 스캔 순서 + 재시작
        changed = false;
        for (let i = 0; i < comps.length; i++) {
            let found: [number, Cell[]] | null = null;
            for (let j = i + 1; j < comps.length; j++) {
                const hit = mergePath(sim, comps[i], comps[j], m);
                if (hit !== null) { found = [j, hit]; break; }
            }
            if (found !== null) {
                const [j, pathCells] = found;
                const merged = new Set(comps[i]);
                for (const key of comps[j]) merged.add(key);
                for (const c of pathCells) merged.add(ck(c));
                const next = comps.slice(0, i);
                next.push(merged);
                next.push(...comps.slice(i + 1, j), ...comps.slice(j + 1));
                comps.length = 0;
                comps.push(...next);
                changed = true;
                break;  // 스캔을 맨 위에서 재시작(고정)
            }
        }
    }
    return comps;
};

// 알고리즘 1의 병합 판정: Si의 셀들을 row-major로 심은 multi-source BFS(이웃 고정 순서).
// dequeue된 첫 Sj 셀이 쌍과 그 경로를 고정하고, 그 거리가 ≤ m-2면 경로 셀을 돌려준다.
const mergePath = (sim: Sim, si: Set<string>, sj: Set<string>, m: number): Cell[] | null => {
    const par = new Map<string, Cell | null>();
    const depth = new Map<string, number>();
    const q: Cell[] = [];
    let head = 0;
    for (const seed of sortedCells(si)) {  // row-major seeds(서로 다르니 멤버십 검사는 항참 참)
        par.set(ck(seed), null);
        depth.set(ck(seed), 0);
        q.push(seed);
    }
    while (head < q.length) {
        const c = q[head++];
        const d = depth.get(ck(c))!;
        if (sj.has(ck(c))) {  // BFS 순서상 첫 Sj 셀 — 고정된 쌍, 여기서 판정한다
            if (d <= m - 2) {
                const path: Cell[] = [];
                let cur: Cell | null = c;
                while (cur !== null) {  // Python: while cur is not None: append(cur); cur = par[cur]
                    path.push(cur);
                    const p: Cell | null = par.get(ck(cur)) ?? null;  // enqueue된 셀은 전부 parent를 가진다
                    if (p === null) break;  // seed에 닿았다(Python에서 par[seed]는 None)
                    cur = p;
                }
                return path;
            }
            return null;
        }
        for (const n of nbrs(sim, c)) {
            if (!par.has(ck(n))) {
                par.set(ck(n), c);
                depth.set(ck(n), d + 1);
                q.push(n);
            }
        }
    }
    return null;
};

// 알고리즘 2(assign_agents): 배정 X(A 또는 T) 아래 각 agent를 가두는 subgraph. interior vertex는
// 직접 배정하고, junction은 논문이 m'/m''이라 부르는 수로 plank를 따라 배정한다. 나중에 온
// 배정이 앞선 배정을 덮어쓴다(고정).
const assignAgents = (sim: Sim, x: Cell[], sList: Set<string>[], m: number): (number | null)[] => {
    const f: (number | null)[] = new Array(x.length).fill(null);
    const xInv = (c: Cell): number | null => {
        for (let i = 0; i < x.length; i++) if (cellEq(x[i], c)) return i;
        return null;
    };
    for (let si = 0; si < sList.length; si++) {
        const viSet = sList[si];
        for (const v of sortedCells(viSet)) {  // Vi 위를 row-major로
            const us = nbrs(sim, v).filter((u) => !viSet.has(ck(u)));
            if (us.length === 0) {  // interior vertex(lines 11-12)
                const ai = xInv(v);
                if (ai !== null) f[ai] = si;
                continue;
            }
            // m''(line 5): G[V\{v}]에서 Vi\{v}에서 도달 가능한 점유되지 않은 셀들의 수.
            const reachCells: Cell[] = [];
            const reachKeys = new Set<string>();
            const dq: Cell[] = [];
            let head = 0;
            for (const seed of sortedCells(viSet)) {
                if (!cellEq(seed, v) && !reachKeys.has(ck(seed))) {
                    reachKeys.add(ck(seed));
                    reachCells.push(seed);
                    dq.push(seed);
                }
            }
            while (head < dq.length) {
                const c = dq[head++];
                for (const n of nbrs(sim, c)) {
                    if (!cellEq(n, v) && !reachKeys.has(ck(n))) {  // vertex v는 그래프에서 제거
                        reachKeys.add(ck(n));
                        reachCells.push(n);
                        dq.push(n);
                    }
                }
            }
            let mDprime = 0;
            for (const c of reachCells) if (xInv(c) === null) mDprime++;
            for (const u of us) {  // 고정 이웃 순서(line 6)
                // m'(line 7): edge (u,v)를 제거하고 v에서 도달 가능한 점유되지 않은 셀들의 수.
                const reach2Cells: Cell[] = [v];
                const reach2Keys = new Set<string>([ck(v)]);
                const dq2: Cell[] = [v];
                let head2 = 0;
                while (head2 < dq2.length) {
                    const c = dq2[head2++];
                    for (const n of nbrs(sim, c)) {
                        if ((cellEq(c, v) && cellEq(n, u)) || (cellEq(n, v) && cellEq(c, u))) continue;  // 지운 edge
                        if (!reach2Keys.has(ck(n))) {
                            reach2Keys.add(ck(n));
                            reach2Cells.push(n);
                            dq2.push(n);
                        }
                    }
                }
                let mPrime = 0;
                for (const c of reach2Cells) if (xInv(c) === null) mPrime++;
                const ai = xInv(v);
                // lines 8-9: 게이트는 junction 점유자만 gating한다(Fig 11(a)로 검증 — line 10은
                // gate 없이 실행돼야 한다).
                if (((mPrime >= 1 && mPrime < m) || mDprime >= 1) && ai !== null) f[ai] = si;
                // Line 10 — u 루프 안에서 무조건: plank를 u에서 v로부터 걸어(어떤 Vj에도 속하지
                // 않는 셀들을 지나, 멈추는 셀을 포함해) 처음 max(0, m'-1)명의 agent를 만난 순서대로 배정한다.
                let count = 0;
                for (const w of plankWalk(sim, v, u, sList)) {
                    if (count >= Math.max(0, mPrime - 1)) break;
                    const ai2 = xInv(w);
                    if (ai2 !== null) {
                        f[ai2] = si;
                        count++;
                    }
                }
            }
        }
    }
    return f;
};

// v에서 u로(u에서 v 쪽으로가 아니다 — 첫 수에서 v를 명시적으로 제외), 어떤 subgraph에도 속하지
// 않는 셀들을 지나 가는 유일한 극대 경로. 멈추는 셀(먼 subgraph의 join vertex)을 포함하고, dead
// end에서 끝난다.
const plankWalk = (sim: Sim, v: Cell, u: Cell, sList: Set<string>[]): Cell[] => {
    const covered = new Set<string>();
    for (const s of sList) for (const key of s) covered.add(key);
    const walk: Cell[] = [u];
    let cur = u;
    while (!covered.has(ck(cur))) {
        const nxts = nbrs(sim, cur).filter((n) => !walk.some((w) => cellEq(w, n)) && !cellEq(n, v));
        if (nxts.length === 0) return walk;  // dead-end plank
        walk.push(nxts[0]);  // 유일한 연속(non-covered 셀은 degree ≤ 2)
        cur = nxts[0];
    }
    return walk;
};

// u에서 v로 걸어가 멈추는 셀이 Sj에 속하면 (sj index, 멈추는 셀을 포함한 셀들), dead end면 null.
const plankToSubgraph = (sim: Sim, v: Cell, u: Cell, sList: Set<string>[]): [number | null, Cell[]] => {
    const walk = plankWalk(sim, v, u, sList);
    const last = walk[walk.length - 1];
    for (let i = 0; i < sList.length; i++) {
        if (sList[i].has(ck(last))) return [i, walk];
    }
    return [null, []];
};

// 알고리즘 3 lines 3-10 — (Si, v) 하나에 대해: v에서 나가는 모든 plank를 걷고, goal을 가진 셀
// 중 배정된 agent가 다른 subgraph Sj에 속하면 그 배정이 관계가 된다. 미배정 goal은 걷기를
// 계속하고(line 10), goal이 없거나 Si에 배정된 goal은 그 walk을 끝낸다. dead-end plank은 아무것도
// 내지 않는다(line 4).
const plankRelation = (sim: Sim, sList: Set<string>[], t: Cell[], f: (number | null)[],
                       si: number, viSet: Set<string>, v: Cell): number | null => {
    for (const u of nbrs(sim, v).filter((u) => !viSet.has(ck(u)))) {
        const [sj, plank] = plankToSubgraph(sim, v, u, sList);
        if (sj === null) continue;  // dead-end plank(line 4) — 이 u에선 관계가 나올 수 없다
        const walk = [v, ...plank];
        let hit: number | null = null;
        for (const vp of walk) {
            const r = t.findIndex((x) => cellEq(x, vp));
            if (r < 0) break;  // 아무 goal도 없다 — while 조건(line 6)이 끝난다
            const fr = f[r];
            if (fr !== null && fr === si) break;  // "우리" agent — 관계 없음, 다음 u로
            if (fr === null) continue;             // 미배정 goal: 걷기를 계속(line 10)
            hit = fr;
            break;
        }
        if (hit !== null) return hit;
    }
    return null;
};

// 알고리즘 3(subgraph_priority): Si < Sj는 Si가 먼저 계획된다는 뜻이다. 모든 subgraph vertex를
// row-major로 훑어, Si의 plank 중 goal 배정된 agent에 닿는 첫 plank가 (si, f(r))을 더하고
// 다음 v로 넘어간다(line 9).
const subgraphPriority = (sim: Sim, sList: Set<string>[], t: Cell[], f: (number | null)[]): Array<[number, number]> => {
    const rels: Array<[number, number]> = [];
    for (let si = 0; si < sList.length; si++) {
        for (const v of sortedCells(sList[si])) {  // row-major
            const hit = plankRelation(sim, sList, t, f, si, sList[si], v);
            if (hit !== null) rels.push([si, hit]);
        }
    }
    return rels;
};

// 우선순위 관계의 추이 닫힘. 자기 자신과의 관계는 풀 수 없다는 뜻이다(Proposition 2).
const closure = (rels: Array<[number, number]>): Set<string> => {
    const cl = new Set<string>(rels.map(([a, b]) => `${a},${b}`));
    let changed = true;
    while (changed) {
        changed = false;
        const snap = [...cl].map((key) => key.split(",").map(Number) as [number, number]);
        for (const [a, b] of snap) {
            for (const [c2, d] of snap) {
                if (b === c2 && !cl.has(`${a},${d}`)) {
                    cl.add(`${a},${d}`);
                    changed = true;
                }
            }
        }
    }
    return cl;
};

// 고정된 next-agent 규칙(논문은 이걸 비워뒀다): 배정된 agent가 먼저 — 후보는 unfinished이고
// subgraph에 ACTIVE predecessor(활성 = unfinished 구성원이 있는 subgraph)가 없는 agent들, 그중
// index 최소. 배정된 unfinished가 없으면 index 최소의 미배정 agent.
const nextAgent = (unfinished: number[], f: (number | null)[], closureSet: Set<string>): number => {
    const assignedUnfinished = unfinished.filter((i) => f[i] !== null);
    if (assignedUnfinished.length > 0) {
        const active = new Set<number>();
        for (const i of unfinished) if (f[i] !== null) active.add(f[i] as number);
        const cands: number[] = [];
        for (const i of assignedUnfinished) {
            const si = f[i] as number;
            let hasPred = false;
            for (const key of closureSet) {
                const [a, b] = key.split(",").map(Number);
                if (a !== b && b === si && active.has(a)) hasPred = true;
            }
            if (!hasPred) cands.push(i);
        }
        return Math.min(...cands);  // 고정: 가장 낮은 index
    }
    for (const i of unfinished) if (f[i] === null) return i;
    throw new Error("push_and_rotate: no unfinished agent left");  // 도달 불가
};

// Set<string>(키)를 row-major 수치 순서의 Cell 배열로 — Python의 sorted(set) 스캔 대응.
const sortedCells = (set: Set<string>): Cell[] => {
    const cells = [...set].map(parseKey);
    cells.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    return cells;
};

// 알고리즘 8(solve): agent를 정적 BFS 경로를 따라 하나씩 걷히고, walker의 다음 발이 resolving
// agent의 살아 있는 trail q에 닿으면 push/swap 대신 rotate가 발동한다. is_polygon은 끝난 agent들의
// 셀이 path BFS를 막는지 여부다(line 5).
const solve = (sim: Sim, sList: Set<string>[], f: (number | null)[], closureSet: Set<string>): boolean => {
    let isPolygon = true;
    outer: for (const c of freeCellsRowMajor(sim)) {
        if (nbrs(sim, c).length !== 2) { isPolygon = false; break outer; }
    }
    const nAgents = sim.A.length;
    const F = new Set<number>();
    const q: Cell[] = [];
    let rOpt: number | null = null;
    while (F.size !== nAgents) {  // line 6
        if (rOpt === null) {     // line 7
            const unfinished: number[] = [];
            for (let i = 0; i < nAgents; i++) if (!F.has(i)) unfinished.push(i);
            rOpt = nextAgent(unfinished, f, closureSet);  // line 8
        }
        const r = rOpt;
        const blocked = new Set<string>();  // lines 9-12: polygon일 때만 끝난 agent의 셀이 막는다
        if (isPolygon) for (const i of F) blocked.add(ck(sim.A[i]));
        const p = pathTo(sim, sim.A[r], sim.T[r], blocked);
        if (p === null) return false;
        q.push(sim.A[r]);  // line 13
        try {
            while (!cellEq(sim.A[r], sim.T[r])) {  // line 14
                const idx = p.findIndex((c) => cellEq(c, sim.A[r]));  // A(r)은 p 위에 있고 끝이 아니다
                if (idx < 0 || idx + 1 >= p.length) throw new Error("push_and_rotate: walker off its path");
                const v = p[idx + 1];  // line 15
                const qi = q.findIndex((c) => cellEq(c, v));
                if (qi >= 0) {  // line 16 — resolving agent들의 cycle 감지
                    const c = q.slice(qi);  // get_cycle 고정: 첫 등장부터 끝까지
                    q.length = qi;
                    if (!rotate(sim, c, f, sList)) return false;
                } else {
                    const uSet = new Set<string>();
                    for (const i of F) uSet.add(ck(sim.A[i]));
                    if (!push(sim, r, v, uSet)) {  // line 21
                        const occ = occupant(sim, v);
                        if (occ === null) throw new Error("push_and_rotate: push failed on an empty cell");
                        const cells = f[r] !== null ? sList[f[r] as number] : null;
                        if (!swap(sim, r, occ, cells)) return false;  // line 22
                    }
                }
                q.push(v);  // line 23(무조건 — 고정)
            }
        } catch {
            return false;
        }
        F.add(r);  // line 24
        let handoff: number | null = null;  // line 25: r = None(handoff가 없으면)
        while (q.length > 0) {  // line 26 — q를 줄이며 resolving agent들을 집으로 돌려보낸다
            const v = q[q.length - 1];
            const sOpt = occupant(sim, v);
            if (sOpt !== null && F.has(sOpt) && !cellEq(v, sim.T[sOpt])) {
                const rn = occupant(sim, sim.T[sOpt]);  // line 30
                try {
                    if (rn === null) {
                        move(sim, null, sOpt, sim.T[sOpt]);  // line 32(한 칸 — 인접성은 성립한다)
                    } else {
                        handoff = rn;  // line 34: inner 루프를 빠져나가고 q는 유지된다
                        break;
                    }
                } catch {
                    return false;
                }
            }
            q.pop();  // line 35(handoff가 아닌 모든 분기에서 실행된다)
        }
        rOpt = handoff;
    }
    return true;
};

export function runPushAndRotate(
    map: GridMap,
    tasks: Array<[Cell, Cell]>,
    params: Record<string, unknown>,
): TraceEvent[] {
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({event: "planning_started", algorithm: "push_and_rotate", params});

    const W = map.width, H = map.height;
    let freeCount = 0;
    for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) if (!map.occupied[r * W + c]) freeCount++;
    const free = (c: Cell): boolean =>
        c[0] >= 0 && c[0] < H && c[1] >= 0 && c[1] < W && !map.occupied[c[0] * W + c[1]];

    const starts = tasks.map(([s]) => [s[0], s[1]] as Cell);
    const goals = tasks.map(([, g]) => [g[0], g[1]] as Cell);
    // 배정은 정의상 단사이고 모든 관련 셀은 통과 가능해야 한다. 위반 입력은 이 문제의 인스턴스가
    // 아니다 — 계획 없음으로 정직하게 보고한다.
    if (new Set(starts.map(ck)).size < starts.length
        || new Set(goals.map(ck)).size < goals.length
        || [...starts, ...goals].some((c) => !free(c))) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: 0, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    const sim: Sim = {free, w: W, h: H, A: [...starts], T: goals, Pi: [[...starts]], expandedNodes: 0};
    const m = freeCount - tasks.length;  // Alg 7 line 1: 빈 vertex의 수
    const sList = findSubgraphs(sim, m);
    const fA = assignAgents(sim, sim.A, sList, m);
    const fT = assignAgents(sim, sim.T, sList, m);
    if (fA.length !== fT.length || fA.some((x, i) => x !== fT[i])) {  // Alg 7 line 5 — 전혀 풀 수 없다
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: sim.expandedNodes, makespan: 0, sum_of_costs: 0}});
        return events;
    }
    const rels = subgraphPriority(sim, sList, sim.T, fA);
    const cl = closure(rels);
    for (const key of cl) {
        const [a, b] = key.split(",").map(Number);
        if (a === b) {  // 순환 우선순위 — 풀 수 없다(Proposition 2)
            emit({event: "planning_finished", success: false,
                metrics: {expanded_nodes: sim.expandedNodes, makespan: 0, sum_of_costs: 0}});
            return events;
        }
    }
    if (!solve(sim, sList, fA, cl)) {
        emit({event: "planning_finished", success: false,
            metrics: {expanded_nodes: sim.expandedNodes, makespan: 0, sum_of_costs: 0}});
        return events;
    }

    // full-horizon 시공간 경로: paths[k][t]는 시각 t에 agent k가 차지하는 셀. agent들은 다른
    // agent들의 swap/rotate 기동에 밀려나고 replay된 segment를 타고 집으로 돌아가므로, 자른
    // 경로는 실체를 왜곡한다.
    const paths: Cell[][] = [];
    let cost = 0;
    for (let k = 0; k < tasks.length; k++) {
        const p = sim.Pi.map((a) => a[k]);
        for (let t = 1; t < p.length; t++) if (!cellEq(p[t], p[t - 1])) cost += 1;
        paths.push(p);
    }
    for (let k = 0; k < paths.length; k++) emit({event: "path_found", path: paths[k], agent: k});
    emit({event: "planning_finished", success: true,
        metrics: {expanded_nodes: sim.expandedNodes, makespan: sim.Pi.length - 1, sum_of_costs: cost}});
    return events;
}
