#include "mrmp/search/push_and_rotate.hpp"

#include <algorithm>
#include <cassert>
#include <deque>
#include <set>
#include <vector>

namespace mrmp::search {

using core::Cell;
using core::DiscreteSpace;

// 4-connected neighbor order — the SAME fixed convention as the map layer's move
// set. The wait self-loop there is not an action of this algorithm: every recorded
// assignment change is a real move. (Mirrors Python _MOVES_4 exactly.)
namespace {
constexpr int kMoves4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
}  // namespace

std::vector<Cell> PushAndRotate::nbrs(const std::set<Cell>& free_cells, const Cell& c) {
  std::vector<Cell> out;
  for (const auto& d : kMoves4) {
    Cell n{c.row + d[0], c.col + d[1]};
    if (free_cells.count(n)) out.push_back(n);
  }
  return out;
}

std::optional<int> PushAndRotate::occupant(const Sim& sim, const Cell& c) {
  for (size_t i = 0; i < sim.A.size(); ++i) {
    if (sim.A[i] == c) return static_cast<int>(i);
  }
  return std::nullopt;
}

void PushAndRotate::apply_move(Sim& sim, std::vector<Move>* segment, int agent, Cell to) {
  const Cell frm = sim.A[static_cast<size_t>(agent)];
  // Invariants every call site constructs (asserted in debug builds, never
  // recovered from — the reachable failure points live at the callers, which turn
  // them into honest attempt failures exactly like Python's caught AssertionError).
  assert(!(frm == to) && sim.free_cells.count(to) && !occupant(sim, to).has_value());
  sim.A[static_cast<size_t>(agent)] = to;
  sim.pi.push_back(sim.A);
  if (segment != nullptr) segment->push_back(Move{agent, frm, to});
}

PushAndRotate::BfsResult PushAndRotate::bfs_parent(Sim& sim, const Cell& start,
                                                   const std::set<Cell>& blocked) const {
  BfsResult res;
  res.start = start;
  if (blocked.count(start)) return res;  // empty BFS — zero expansions counted
  res.order.push_back(start);
  std::deque<Cell> queue{start};
  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop_front();
    for (const Cell& n : nbrs(sim.free_cells, c)) {
      if (n == start || res.parent.count(n) || blocked.count(n)) continue;
      // Python's parent dict gains the key at discovery, so its insertion order IS
      // the enqueue order; here that needs an explicit append to `order` as well.
      res.parent[n] = c;
      res.order.push_back(n);
      queue.push_back(n);
    }
  }
  // Every enqueued cell is dequeued exactly once, so order.size() IS the number of
  // node expansions this call contributes to the metric.
  sim.expanded_nodes += static_cast<int>(res.order.size());
  return res;
}

std::optional<std::vector<Cell>> PushAndRotate::path_to(Sim& sim, const Cell& start,
                                                       const Cell& goal,
                                                       const std::set<Cell>& blocked) const {
  BfsResult res = bfs_parent(sim, start, blocked);
  if (!res.has(goal)) return std::nullopt;
  std::vector<Cell> out;
  Cell cur = goal;
  for (;;) {
    out.push_back(cur);
    if (cur == start) break;
    cur = res.parent.at(cur);
  }
  std::reverse(out.begin(), out.end());
  return out;
}

// --- Algorithm 10: clear_vertex ---------------------------------------------------

bool PushAndRotate::clear_vertex(Sim& sim, std::vector<Move>* segment, const Cell& v,
                                 const std::set<Cell>& blocked) const {
  // Scan unoccupied cells row-major (free_cells is ordered — that IS the sort);
  // the FIRST one whose BFS reaches v wins; occupants on that parent chain shift
  // one step toward u, farthest-from-v occupant first.
  for (const Cell& u : sim.free_cells) {
    if (occupant(sim, u).has_value()) continue;  // only unoccupied vertices are scanned
    BfsResult res = bfs_parent(sim, u, blocked);
    if (!res.has(v)) continue;
    std::vector<Cell> chain;
    Cell cur = v;
    for (;;) {
      chain.push_back(cur);
      if (cur == res.start) break;
      cur = res.parent.at(cur);
    }
    std::reverse(chain.begin(), chain.end());  // [u = x0, ..., xk = v]
    bool have_xp = false;
    Cell xp{0, 0};
    for (const Cell& x : chain) {
      if (have_xp) {
        auto a = occupant(sim, x);
        if (a.has_value()) apply_move(sim, segment, *a, xp);
      }
      xp = x;
      have_xp = true;
    }
    return true;
  }
  return false;
}

// --- Algorithm 4: push --------------------------------------------------------------

bool PushAndRotate::push(Sim& sim, int r, const Cell& v, const std::set<Cell>& u_set) const {
  if (occupant(sim, v).has_value()) {
    std::set<Cell> blocked = u_set;
    blocked.insert(sim.A[static_cast<size_t>(r)]);
    if (!clear_vertex(sim, nullptr, v, blocked)) return false;
  }
  apply_move(sim, nullptr, r, v);
  return true;
}

// --- Algorithm 11: multipush ----------------------------------------------------------

bool PushAndRotate::multipush(Sim& sim, std::vector<Move>* segment, int r0, int s0,
                              const Cell& v) const {
  auto pr = path_to(sim, sim.A[static_cast<size_t>(r0)], v, {});
  auto ps = path_to(sim, sim.A[static_cast<size_t>(s0)], v, {});
  // closer wins; unreachable == infinite; tie -> first argument (r0).
  int r, s;
  std::vector<Cell> p;
  if (pr.has_value() && (!ps.has_value() || pr->size() <= ps->size())) {
    r = r0; s = s0; p = *pr;
  } else if (ps.has_value()) {
    r = s0; s = r0; p = *ps;
  } else {
    return false;
  }
  for (size_t i = 1; i < p.size(); ++i) {  // pinned: skip the start cell
    const Cell vr = sim.A[static_cast<size_t>(r)];
    const Cell vs = sim.A[static_cast<size_t>(s)];
    const Cell x = p[i];
    if (occupant(sim, x).has_value()) {
      const std::set<Cell> blocked{vr, vs};
      if (!clear_vertex(sim, segment, x, blocked)) return false;
    }
    apply_move(sim, segment, r, x);
    apply_move(sim, segment, s, vr);
  }
  if (!(sim.A[static_cast<size_t>(r)] == v)) return false;
  for (const Cell& n : nbrs(sim.free_cells, sim.A[static_cast<size_t>(r)])) {
    if (n == sim.A[static_cast<size_t>(s)]) return true;
  }
  return false;
}

// --- Algorithm 12: clear (the four stages around an exchange site) ---------------------

bool PushAndRotate::clear_pair(Sim& sim, std::vector<Move>* segment, int rp, int sp,
                               const Cell& v) const {
  const int r = (sim.A[static_cast<size_t>(rp)] == v) ? rp : sp;  // line 1 (by position)
  const int s = (r == rp) ? sp : rp;
  const Cell vp = sim.A[static_cast<size_t>(s)];  // v' — s's cell, adjacent to v by construction

  std::vector<Cell> e_set;  // line 2 (fixed order): the free neighbors of v
  for (const Cell& n : nbrs(sim.free_cells, v)) {
    if (!occupant(sim, n).has_value()) e_set.push_back(n);
  }
  const auto in_e = [&e_set](const Cell& c) {
    for (const Cell& e : e_set) {
      if (e == c) return true;
    }
    return false;
  };
  if (e_set.size() >= 2) return true;  // line 3 — two free neighbors already: nothing to clear

  // Stage 1 (lines 5-9): push each occupied neighbor's occupant away; the first
  // success seeds E, a second success finishes. Candidates fixed BEFORE clearing.
  std::vector<Cell> stage1;
  for (const Cell& n : nbrs(sim.free_cells, v)) {
    if (!in_e(n) && !(n == vp)) stage1.push_back(n);
  }
  for (const Cell& n : stage1) {
    std::set<Cell> blocked(e_set.begin(), e_set.end());
    blocked.insert(v);
    blocked.insert(vp);
    if (clear_vertex(sim, segment, n, blocked)) {
      if (!e_set.empty()) return true;
      e_set.push_back(n);
    }
  }
  if (e_set.empty()) return false;  // lines 10-11
  const Cell eps = e_set[0];        // line 12: the single empty neighbor

  // Stage 2 (lines 13-19): per candidate n, try on a copy (snapshot/rollback). The
  // OUTER clear_vertex failing means the loop simply continues to the NEXT
  // candidate; only when n cleared but eps did not does line 19's break leave
  // stage 2 for good.
  std::vector<Cell> cands;
  for (const Cell& n : nbrs(sim.free_cells, v)) {
    if (!(n == vp) && !in_e(n)) cands.push_back(n);
  }
  for (const Cell& n : cands) {
    const std::vector<Cell> before = sim.A;
    const size_t pi_len = sim.pi.size();
    std::vector<Move> seg2;
    const bool ok = clear_vertex(sim, &seg2, n, std::set<Cell>{v, vp});
    if (ok && !clear_vertex(sim, &seg2, eps, std::set<Cell>{v, vp, n})) {
      sim.A = before;  // line 19: rollback (the copy was never committed)
      sim.pi.resize(pi_len);
      break;          // inner failure stops stage 2 — fall through to stage 3
    }
    if (ok) return true;  // lines 17-18: commit is implicit (live state mutated)
    sim.A = before;
    sim.pi.resize(pi_len);
  }

  // Stage 3 (lines 20-27): r steps into eps, s steps onto v (both on the copy);
  // then n clears (its occupant leaves on a path avoiding {v, eps}) and v' stays
  // clearable. Same nesting as stage 2.
  for (const Cell& n : cands) {
    const std::vector<Cell> before = sim.A;
    const size_t pi_len = sim.pi.size();
    std::vector<Move> seg3;
    apply_move(sim, &seg3, r, eps);  // line 22 (r is on v — eps adjacent)
    apply_move(sim, &seg3, s, v);
    const bool ok = clear_vertex(sim, &seg3, n, std::set<Cell>{v, eps});
    if (ok && !clear_vertex(sim, &seg3, vp, std::set<Cell>{v, eps, n})) {
      sim.A = before;
      sim.pi.resize(pi_len);
      break;  // line 27 (same nesting as line 19)
    }
    if (ok) return true;
    sim.A = before;
    sim.pi.resize(pi_len);
  }

  // Stage 4 (lines 28-36): create space behind eps by walking n's occupant through
  // v while r and s step aside, then push it away from eps for good. A stage-4
  // clear that vacates a candidate cell leaves its occupant unoccupied — Python's
  // caught AssertionError turns that into an honest attempt failure here too.
  if (!clear_vertex(sim, segment, vp, std::set<Cell>{v})) return false;
  apply_move(sim, segment, r, vp);
  if (!clear_vertex(sim, segment, eps,
                    std::set<Cell>{v, vp, sim.A[static_cast<size_t>(s)]})) {
    return false;
  }
  std::vector<Cell> n2;
  for (const Cell& n : nbrs(sim.free_cells, v)) {
    if (!(n == vp) && !in_e(n)) n2.push_back(n);
  }
  if (n2.empty()) return false;
  const Cell n = n2[0];  // pinned: fixed-order first ("any vertex")
  auto t = occupant(sim, n);
  if (!t.has_value()) return false;  // the caught-assertion case — attempt fails honestly
  apply_move(sim, segment, *t, v);    // line 34 (through v ...)
  apply_move(sim, segment, *t, eps);  // ... to eps — two single steps
  apply_move(sim, segment, r, v);
  apply_move(sim, segment, s, vp);
  return clear_vertex(sim, segment, eps, std::set<Cell>{v, vp, n});
}

// --- Algorithm 13: exchange -------------------------------------------------------------

void PushAndRotate::exchange(Sim& sim, int rp, int sp, const Cell& v) const {
  // The physical exchange at the junction vertex v: r (the one ON v) steps aside to
  // v1, s crosses through v onto v2, r crosses back through v onto s's old cell, s
  // comes back from v2 onto v. Net effect: the pair exchanged positions.
  const int r = (sim.A[static_cast<size_t>(rp)] == v) ? rp : sp;  // line 1 (by position)
  const int s = (r == rp) ? sp : rp;
  const Cell vs = sim.A[static_cast<size_t>(s)];
  std::vector<Cell> free_nbrs;
  for (const Cell& n : nbrs(sim.free_cells, v)) {
    if (!occupant(sim, n).has_value()) free_nbrs.push_back(n);
  }
  assert(free_nbrs.size() >= 2);  // no exchange site — the caller's candidate fails
  const Cell v1 = free_nbrs[0];
  const Cell v2 = free_nbrs[1];
  apply_move(sim, nullptr, r, v1);  // line 5: r leaves v to the first free neighbor
  apply_move(sim, nullptr, s, v);   // line 6 step 1 (through v)
  apply_move(sim, nullptr, s, v2);  // line 6 step 2
  apply_move(sim, nullptr, r, v);   // line 7 step 1 (through v)
  apply_move(sim, nullptr, r, vs);  // line 7 step 2 — the exchange lands
  apply_move(sim, nullptr, s, v);   // line 8: s back onto r's old cell
}

// --- Algorithm 5: swap ---------------------------------------------------------------------

bool PushAndRotate::swap(Sim& sim, int r0, int s0,
                         const std::optional<std::set<Cell>>& sub_cells) const {
  // Exchange two adjacent agents at a degree >= 3 vertex of r's subgraph (no
  // subgraph — an unassigned agent can never swap, which is what makes tree maps
  // honestly unsolvable). Candidates nearest-first (BFS dequeue from A[r0]); the
  // exchange itself is Algorithm 12 + Algorithm 13. Speculative candidates roll back
  // (assignment AND trace history); only the successful candidate's exchange moves
  // survive, plus the reversed replay with r/s roles exchanged.
  if (!sub_cells.has_value()) return false;
  BfsResult par = bfs_parent(sim, sim.A[static_cast<size_t>(r0)], {});
  std::vector<Cell> cands;
  for (const Cell& x : par.order) {
    if (sub_cells->count(x) > 0 && nbrs(sim.free_cells, x).size() >= 3) cands.push_back(x);
  }
  for (const Cell& v : cands) {
    const std::vector<Cell> before = sim.A;
    const size_t pi_len = sim.pi.size();
    std::vector<Move> segment;
    bool ok = multipush(sim, &segment, r0, s0, v);
    if (ok) ok = clear_pair(sim, &segment, r0, s0, v);
    if (!ok) {
      sim.A = before;
      sim.pi.resize(pi_len);
      continue;
    }
    // Commit is implicit (A mutated live, pi already appended). The exchange runs on
    // the real state; then every recorded segment move replays reversed with r/s
    // roles exchanged — that replay carries displaced bystanders home.
    exchange(sim, r0, s0, v);
    for (auto it = segment.rbegin(); it != segment.rend(); ++it) {
      const int who = it->agent == r0 ? s0 : (it->agent == s0 ? r0 : it->agent);
      assert(sim.A[static_cast<size_t>(who)] == it->to);  // lands exactly by construction
      apply_move(sim, nullptr, who, it->from);
    }
    return true;
  }
  return false;
}

// --- Algorithm 6: rotate -------------------------------------------------------------------

void PushAndRotate::cascade(Sim& sim, const std::vector<Cell>& c, const Cell& e) const {
  // Move EVERY rider on cycle c one step forward (cycle order = c's list order, the
  // wrap edge closing it). The empty slot e is filled by the rider on its
  // predecessor, whose old cell is filled in turn, propagating backwards — exactly
  // n-1 visits (every cycle cell except e itself), so every rider moves EXACTLY
  // once; visiting c[j]=e itself would move the first moved rider a second time.
  const int n = static_cast<int>(c.size());
  int j = -1;
  for (int i = 0; i < n; ++i) {
    if (c[static_cast<size_t>(i)] == e) {
      j = i;
      break;
    }
  }
  assert(j >= 0);
  for (int k = 1; k < n; ++k) {
    const Cell w = c[static_cast<size_t>(((j - k) % n + n) % n)];
    auto a = occupant(sim, w);
    if (a.has_value()) {
      apply_move(sim, nullptr, *a, c[static_cast<size_t>(((j - k + 1) % n + n) % n)]);
    }
  }
}

bool PushAndRotate::rotate(Sim& sim, const std::vector<Cell>& c,
                           const std::vector<std::optional<int>>& f,
                           const std::vector<std::set<Cell>>& s_list) const {
  // Phase 1 (lines 1-4): an empty cell exists — cascade from it.
  for (const Cell& v : c) {
    if (!occupant(sim, v).has_value()) {
      cascade(sim, c, v);
      return true;
    }
  }
  // Phase 2 (lines 5-16, fully occupied): push the rider at the first clearable
  // cycle vertex off the cycle, step its predecessor up onto the vacated slot, SWAP
  // the pair (rotate borrows swap's junction machinery), cascade from the new empty
  // cell, then replay the push-off reversed with roles exchanged.
  const int n = static_cast<int>(c.size());
  for (int idx = 0; idx < n; ++idx) {
    const Cell v = c[static_cast<size_t>(idx)];
    auto r_opt = occupant(sim, v);
    assert(r_opt.has_value());  // phase 2 only runs on a fully occupied cycle
    const int r = *r_opt;
    std::vector<Move> seg;
    std::set<Cell> blocked(c.begin(), c.end());
    blocked.erase(v);
    const bool cleared = clear_vertex(sim, &seg, v, blocked);
    if (!cleared) continue;
    const Cell vp = c[static_cast<size_t>(((idx - 1) % n + n) % n)];  // predecessor (line 10)
    auto rp_opt = occupant(sim, vp);
    assert(rp_opt.has_value());  // fully occupied; the push-off only vacated v itself
    const int rp_ = *rp_opt;
    apply_move(sim, nullptr, rp_, v);  // line 12: predecessor steps into the vacated cell
    std::optional<std::set<Cell>> cells;
    if (f[static_cast<size_t>(r)].has_value()) {
      cells = s_list[static_cast<size_t>(*f[static_cast<size_t>(r)])];
    }
    const bool swapped = swap(sim, r, rp_, cells);
    if (!swapped) return false;  // honest failure propagates out of solve
    cascade(sim, c, vp);         // line 14: everyone forward starting at the new empty cell
    for (auto it = seg.rbegin(); it != seg.rend(); ++it) {  // line 15: reversed, roles exchanged
      const int who = it->agent == r ? rp_ : (it->agent == rp_ ? r : it->agent);
      assert(sim.A[static_cast<size_t>(who)] == it->to);  // lands exactly by construction
      apply_move(sim, nullptr, who, it->from);
    }
    return true;
  }
  return false;
}

// --- Algorithms 1/2/3: decomposition, assignment, priority ---------------------------------

std::vector<std::set<Cell>> PushAndRotate::find_subgraphs(const std::set<Cell>& free_cells,
                                                          int m) const {
  // Hopcroft-Tarjan biconnected components (DFS roots row-major — free_cells order —
  // and neighbors in fixed order); nontrivial = a whole cycle's worth of vertices.
  std::vector<std::set<Cell>> comps;
  std::map<Cell, int> disc;
  std::map<Cell, int> low;
  std::map<Cell, std::optional<Cell>> par;
  int t = 0;
  for (const Cell& root : free_cells) {  // row-major DFS roots
    if (disc.count(root)) continue;
    t += 1;
    disc[root] = t;
    low[root] = t;
    par[root] = std::nullopt;
    std::vector<std::pair<Cell, Cell>> edge_stack;
    struct Frame {
      Cell u;
      std::vector<Cell> nb;
      size_t idx = 0;
    };
    std::vector<Frame> stack{Frame{root, nbrs(free_cells, root), 0}};
    while (!stack.empty()) {
      Frame& top = stack.back();
      const Cell u = top.u;
      if (top.idx >= top.nb.size()) {
        stack.pop_back();
        if (stack.empty()) break;  // this DFS tree is complete
        const Cell p = *par[u];
        if (low[u] >= disc[p]) {
          std::set<Cell> comp;
          for (;;) {
            const auto [ea, eb] = edge_stack.back();
            edge_stack.pop_back();
            comp.insert(ea);
            comp.insert(eb);
            if ((ea == p && eb == u) || (eb == p && ea == u)) break;
          }
          if (comp.size() >= 3) comps.push_back(comp);  // nontrivial = a cycle's worth
        } else {
          low[p] = std::min(low[p], low[u]);
        }
        continue;
      }
      const Cell w = top.nb[top.idx];
      top.idx += 1;
      if (!disc.count(w)) {
        par[w] = u;
        t += 1;
        disc[w] = t;
        low[w] = t;
        edge_stack.push_back({u, w});
        stack.push_back(Frame{w, nbrs(free_cells, w), 0});
      } else if (!(w == par[u].value_or(w)) && disc[w] < disc[u]) {
        edge_stack.push_back({u, w});
        low[u] = std::min(low[u], disc[w]);
      }
    }
  }
  // covered is computed ONCE from the nontrivial comps (singletons never extend it).
  std::set<Cell> covered;
  for (const auto& comp : comps) {
    covered.insert(comp.begin(), comp.end());
  }
  for (const Cell& v : free_cells) {  // row-major singletons for uncovered join vertices
    if (nbrs(free_cells, v).size() >= 3 && !covered.count(v)) comps.push_back(std::set<Cell>{v});
  }
  bool changed = true;
  while (changed) {  // merge loop (Alg 1 lines 3-5), pinned scan order + restart
    changed = false;
    for (size_t i = 0; i < comps.size(); ++i) {
      std::optional<std::pair<size_t, std::vector<Cell>>> merged_found;
      for (size_t j = i + 1; j < comps.size(); ++j) {
        auto hit = merge_path(free_cells, comps[i], comps[j], m);
        if (hit.has_value()) {
          merged_found = std::make_pair(j, *hit);
          break;
        }
      }
      if (merged_found.has_value()) {
        const size_t j = merged_found->first;
        std::set<Cell> merged = comps[i];
        merged.insert(comps[j].begin(), comps[j].end());
        for (const Cell& c : merged_found->second) merged.insert(c);
        std::vector<std::set<Cell>> next_comps(comps.begin(), comps.begin() + static_cast<long>(i));
        next_comps.push_back(merged);
        next_comps.insert(next_comps.end(), comps.begin() + static_cast<long>(i) + 1,
                          comps.begin() + static_cast<long>(j));
        next_comps.insert(next_comps.end(), comps.begin() + static_cast<long>(j) + 1, comps.end());
        comps = next_comps;
        changed = true;
        break;  // restart the scan from the top (pinned)
      }
    }
  }
  return comps;
}

std::optional<std::vector<Cell>> PushAndRotate::merge_path(const std::set<Cell>& free_cells,
                                                           const std::set<Cell>& si,
                                                           const std::set<Cell>& sj, int m) const {
  // Multi-source BFS seeded from Si's cells in row-major order (fixed neighbor
  // order); the FIRST Sj cell dequeued pins the pair and their path. Returns the
  // path cells when that distance <= m-2, else nullopt.
  std::map<Cell, Cell> par;
  std::map<Cell, int> depth;
  std::deque<Cell> q;
  for (const Cell& seed : si) {  // row-major seeds (set order); distinct by construction
    par[seed] = Cell{0, 0};      // placeholder parent — the root check uses `is_root` below
    depth[seed] = 0;
    q.push_back(seed);
  }
  // Python's dict maps every seed to None (its "no parent"); C++ needs a sentinel.
  // Seeds are exactly the cells with depth 0, so the walk stops there instead.
  while (!q.empty()) {
    const Cell c = q.front();
    q.pop_front();
    const int d = depth.at(c);
    if (sj.count(c)) {  // first Sj cell in BFS order — the pinned pair, decide now
      if (d <= m - 2) {
        std::vector<Cell> path;
        Cell cur = c;
        for (;;) {
          path.push_back(cur);
          if (depth.at(cur) == 0) break;  // reached a seed (Python's par[cur] is None there)
          cur = par.at(cur);
        }
        return path;
      }
      return std::nullopt;
    }
    for (const Cell& n : nbrs(free_cells, c)) {
      if (!par.count(n)) {
        par[n] = c;
        depth[n] = d + 1;
        q.push_back(n);
      }
    }
  }
  return std::nullopt;
}

std::vector<std::optional<int>> PushAndRotate::assign_agents(
    const Sim& sim, const std::vector<Cell>& x, const std::vector<std::set<Cell>>& s_list,
    int m) const {
  // Alg 2: which subgraph confines each agent under assignment X (A or T). Later
  // assignments overwrite earlier ones (pinned).
  std::vector<std::optional<int>> f(x.size(), std::nullopt);
  auto x_inv = [&x](const Cell& c) -> std::optional<int> {
    for (size_t i = 0; i < x.size(); ++i) {
      if (x[i] == c) return static_cast<int>(i);
    }
    return std::nullopt;
  };
  for (size_t si = 0; si < s_list.size(); ++si) {
    const std::set<Cell>& vi_set = s_list[si];
    for (const Cell& v : vi_set) {  // row-major over Vi
      std::vector<Cell> us;
      for (const Cell& u : nbrs(sim.free_cells, v)) {
        if (!vi_set.count(u)) us.push_back(u);
      }
      if (us.empty()) {  // interior vertex (lines 11-12)
        auto ai = x_inv(v);
        if (ai.has_value()) f[static_cast<size_t>(*ai)] = static_cast<int>(si);
        continue;
      }
      // m'' (line 5): unoccupied cells reachable from Vi\{v} in G[V\{v}] — a count.
      std::set<Cell> reach;
      std::deque<Cell> dq;
      for (const Cell& seed : vi_set) {
        if (!(seed == v)) {
          reach.insert(seed);
          dq.push_back(seed);
        }
      }
      while (!dq.empty()) {
        const Cell c = dq.front();
        dq.pop_front();
        for (const Cell& n : nbrs(sim.free_cells, c)) {
          if (!(n == v) && !reach.count(n)) {  // vertex v removed from the graph
            reach.insert(n);
            dq.push_back(n);
          }
        }
      }
      int m_dprime = 0;
      for (const Cell& c : reach) {
        if (!x_inv(c).has_value()) ++m_dprime;
      }
      for (const Cell& u : us) {  // fixed neighbor order (line 6)
        // m' (line 7): unoccupied cells reachable from v with the EDGE (u,v) removed.
        std::set<Cell> reach2{v};
        std::deque<Cell> dq2{v};
        while (!dq2.empty()) {
          const Cell c = dq2.front();
          dq2.pop_front();
          for (const Cell& n : nbrs(sim.free_cells, c)) {
            if ((c == v && n == u) || (n == v && c == u)) continue;  // the removed edge
            if (!reach2.count(n)) {
              reach2.insert(n);
              dq2.push_back(n);
            }
          }
        }
        int m_prime = 0;
        for (const Cell& c : reach2) {
          if (!x_inv(c).has_value()) ++m_prime;
        }
        auto ai = x_inv(v);
        // Lines 8-9: the gate gates ONLY the junction occupant. (Verified against
        // Fig 11(a): a4 IS assigned "in line 10 since m'=3" while the junction cell
        // was unoccupied — line 10 must run ungated.)
        if (((m_prime >= 1 && m_prime < m) || m_dprime >= 1) && ai.has_value()) {
          f[static_cast<size_t>(*ai)] = static_cast<int>(si);
        }
        // Line 10 — UNCONDITIONAL inside the u-loop: walk the plank from u away from
        // v (through cells in no Vj, INCLUDING the stopping cell); assign the first
        // max(0, m'-1) agents encountered, in encounter order.
        int count = 0;
        for (const Cell& w : plank_walk(sim.free_cells, v, u, s_list)) {
          if (count >= std::max(0, m_prime - 1)) break;
          auto ai2 = x_inv(w);
          if (ai2.has_value()) {
            f[static_cast<size_t>(*ai2)] = static_cast<int>(si);
            ++count;
          }
        }
      }
    }
  }
  return f;
}

std::vector<Cell> PushAndRotate::plank_walk(const std::set<Cell>& free_cells, const Cell& v,
                                            const Cell& u,
                                            const std::vector<std::set<Cell>>& s_list) const {
  // The unique maximal path from u AWAY from v (v is explicitly excluded at the
  // first step; other cells belonging to no subgraph have degree <= 2, so the
  // continuation is unique). The walk INCLUDES the stopping cell — the join vertex
  // of the far subgraph — and ends there, or at a dead end.
  std::set<Cell> covered;
  for (const auto& s : s_list) covered.insert(s.begin(), s.end());
  std::vector<Cell> walk{u};
  Cell cur = u;
  bool in_covered = covered.count(cur) > 0;
  while (!in_covered) {
    std::vector<Cell> nxts;
    for (const Cell& n : nbrs(free_cells, cur)) {
      bool in_walk = false;
      for (const Cell& w : walk) {
        if (w == n) in_walk = true;
      }
      if (!in_walk && !(n == v)) nxts.push_back(n);
    }
    if (nxts.empty()) return walk;  // dead-end plank
    walk.push_back(nxts[0]);        // unique continuation (non-covered cells have degree <= 2)
    cur = nxts[0];
    in_covered = covered.count(cur) > 0;
  }
  return walk;
}

std::pair<std::optional<int>, std::vector<Cell>> PushAndRotate::plank_to_subgraph(
    const std::set<Cell>& free_cells, const Cell& v, const Cell& u,
    const std::vector<std::set<Cell>>& s_list) const {
  // Walk from u away from v; return (sj_index, cells INCLUDING the stopping cell
  // that belongs to Sj), or (nullopt, {}) at a dead end.
  std::vector<Cell> walk = plank_walk(free_cells, v, u, s_list);
  const Cell last = walk.back();
  for (size_t i = 0; i < s_list.size(); ++i) {
    if (s_list[i].count(last)) return {static_cast<int>(i), walk};
  }
  return {std::nullopt, {}};
}

std::optional<int> PushAndRotate::plank_relation(const std::set<Cell>& free_cells,
                                                 const std::vector<std::set<Cell>>& s_list,
                                                 const std::vector<Cell>& t,
                                                 const std::vector<std::optional<int>>& f, int si,
                                                 const std::set<Cell>& vi_set,
                                                 const Cell& v) const {
  // Alg 3 lines 3-10: walk every plank leading away from v; the first goal-bearing
  // cell on a walk whose occupant-goal agent IS assigned to some subgraph Sj yields
  // that assignment (the relation); an unassigned-goal cell keeps the walk going
  // (line 10), a no-goal or Si-assigned goal ends that walk. Dead-end planks yield
  // nothing (line 4).
  for (const Cell& u : nbrs(free_cells, v)) {
    if (vi_set.count(u)) continue;
    auto [sj, plank] = plank_to_subgraph(free_cells, v, u, s_list);
    if (!sj.has_value()) continue;  // dead-end plank (line 4) — no relation from this u
    std::vector<Cell> walk{v};
    walk.insert(walk.end(), plank.begin(), plank.end());
    std::optional<int> hit;
    for (const Cell& vp : walk) {
      int r = -1;
      for (size_t i = 0; i < t.size(); ++i) {
        if (t[i] == vp) {
          r = static_cast<int>(i);
          break;
        }
      }
      if (r < 0) break;  // nobody's goal here — the while condition (line 6) ends
      const std::optional<int>& fr = f[static_cast<size_t>(r)];
      if (fr.has_value() && *fr == si) break;  // one of "our" agents — no relation, next u
      if (!fr.has_value()) continue;           // unassigned-goal agent: keep walking (line 10)
      hit = fr;
      break;
    }
    if (hit.has_value()) return hit;
  }
  return std::nullopt;
}

std::vector<std::pair<int, int>> PushAndRotate::subgraph_priority(
    const std::set<Cell>& free_cells, const std::vector<std::set<Cell>>& s_list,
    const std::vector<Cell>& t, const std::vector<std::optional<int>>& f) const {
  // Alg 3: Si < Sj means Si is planned FIRST (its agents get higher priority). For
  // every subgraph vertex v in row-major order: the first plank of Si whose walk
  // hits a goal assigned to another subgraph adds (si, f(r)) — and line 9 then moves
  // on to the NEXT v (remaining u's of this v are skipped).
  std::vector<std::pair<int, int>> rels;
  for (size_t si = 0; si < s_list.size(); ++si) {
    for (const Cell& v : s_list[si]) {  // row-major
      auto hit = plank_relation(free_cells, s_list, t, f, static_cast<int>(si), s_list[si], v);
      if (hit.has_value()) rels.emplace_back(static_cast<int>(si), *hit);
    }
  }
  return rels;
}

std::set<std::pair<int, int>> PushAndRotate::closure(std::vector<std::pair<int, int>> rels) {
  // Transitive closure of the priority relation; a self-relation means unsolvable.
  std::set<std::pair<int, int>> cl(rels.begin(), rels.end());
  bool changed = true;
  while (changed) {
    changed = false;
    const std::vector<std::pair<int, int>> snap(cl.begin(), cl.end());
    for (const auto& [a, b] : snap) {
      for (const auto& [c2, d] : snap) {
        if (b == c2 && !cl.count({a, d})) {
          cl.insert({a, d});
          changed = true;
        }
      }
    }
  }
  return cl;
}

int PushAndRotate::next_agent(const std::set<int>& unfinished,
                              const std::vector<std::optional<int>>& f,
                              const std::set<std::pair<int, int>>& closure) {
  // Pinned next-agent rule (the paper leaves it open): assigned agents go first —
  // candidates are unfinished agents whose subgraph has no ACTIVE predecessor
  // (active = subgraphs with unfinished members; transitive closure of <); lowest
  // index among candidates wins. With no assigned unfinished agent left, the
  // lowest-index unassigned one goes.
  std::vector<int> assigned_unfinished;
  for (int i : unfinished) {  // std::set iterates ascending
    if (f[static_cast<size_t>(i)].has_value()) assigned_unfinished.push_back(i);
  }
  if (!assigned_unfinished.empty()) {
    std::set<int> active;
    for (int i : unfinished) {
      if (f[static_cast<size_t>(i)].has_value()) active.insert(*f[static_cast<size_t>(i)]);
    }
    std::vector<int> cands;
    for (int i : assigned_unfinished) {
      const int si = *f[static_cast<size_t>(i)];
      bool has_pred = false;
      for (const auto& [a, b] : closure) {
        if (a != b && b == si && active.count(a)) has_pred = true;
      }
      if (!has_pred) cands.push_back(i);
    }
    return *std::min_element(cands.begin(), cands.end());  // pinned lowest index
  }
  for (int i : unfinished) {
    if (!f[static_cast<size_t>(i)].has_value()) return i;
  }
  assert(false);  // unreachable: the outer loop only runs while agents are unfinished
  return -1;
}

// --- Algorithm 8: solve -----------------------------------------------------------------

bool PushAndRotate::solve(Sim& sim, const std::vector<std::set<Cell>>& s_list,
                          const std::vector<std::optional<int>>& f,
                          const std::set<std::pair<int, int>>& closure) const {
  // Alg 8 verbatim. F = finished agents; q = the trail of resolving cells; r = the
  // agent being walked (absent between walks). is_polygon gates whether finished
  // agents' cells block r's path BFS.
  const size_t n_agents = sim.A.size();
  bool is_polygon = true;
  for (const Cell& c : sim.free_cells) {
    if (nbrs(sim.free_cells, c).size() != 2) {
      is_polygon = false;
      break;
    }
  }
  std::set<int> F;
  std::vector<Cell> q;
  std::optional<int> r_opt;  // the walked agent; nullopt between walks (Python: r = None)
  while (F.size() != n_agents) {  // line 6
    if (!r_opt.has_value()) {    // line 7
      std::set<int> unfinished;
      for (size_t i = 0; i < n_agents; ++i) {
        if (!F.count(static_cast<int>(i))) unfinished.insert(static_cast<int>(i));
      }
      r_opt = next_agent(unfinished, f, closure);  // line 8
    }
    const int r = *r_opt;
    std::set<Cell> blocked;  // lines 9-12: finished agents' cells block the path only on polygons
    if (is_polygon) {
      for (int i : F) blocked.insert(sim.A[static_cast<size_t>(i)]);
    }
    auto p = path_to(sim, sim.A[static_cast<size_t>(r)], sim.T[static_cast<size_t>(r)], blocked);
    if (!p.has_value()) return false;
    q.push_back(sim.A[static_cast<size_t>(r)]);  // line 13
    while (!(sim.A[static_cast<size_t>(r)] == sim.T[static_cast<size_t>(r)])) {  // line 14
      // A(r) is on p, never at its end here (a miss would be a bug).
      const auto on_p = std::find(p->begin(), p->end(), sim.A[static_cast<size_t>(r)]);
      assert(on_p != p->end() && on_p + 1 < p->end());
      const Cell v = *(on_p + 1);  // line 15
      auto in_q = std::find(q.begin(), q.end(), v);
      if (in_q != q.end()) {         // line 16 — cycle of resolving agents detected
        std::vector<Cell> c(in_q, q.end());  // get_cycle pinned: first occurrence to the end
        q.resize(static_cast<size_t>(std::distance(q.begin(), in_q)));
        if (!rotate(sim, c, f, s_list)) return false;
      } else {
        std::set<Cell> u_set;
        for (int i : F) u_set.insert(sim.A[static_cast<size_t>(i)]);
        if (!push(sim, r, v, u_set)) {  // line 21
          auto occ = occupant(sim, v);
          if (!occ.has_value()) return false;  // unreachable by construction (Python asserts)
          std::optional<std::set<Cell>> cells;
          if (f[static_cast<size_t>(r)].has_value()) {
            cells = s_list[static_cast<size_t>(*f[static_cast<size_t>(r)])];
          }
          if (!swap(sim, r, *occ, cells)) return false;  // line 22
        }
      }
      q.push_back(v);  // line 23 (UNCONDITIONAL — pinned)
    }
    F.insert(r);       // line 24
    std::optional<int> handoff;  // line 25: r = None unless the unwind hands off below
    while (!q.empty()) {  // line 26 — shrink q, returning resolving agents home
      const Cell v = q.back();
      auto s_opt = occupant(sim, v);
      if (s_opt.has_value() && F.count(*s_opt) && !(v == sim.T[static_cast<size_t>(*s_opt)])) {
        auto rn = occupant(sim, sim.T[static_cast<size_t>(*s_opt)]);  // line 30
        if (!rn.has_value()) {
          apply_move(sim, nullptr, *s_opt, sim.T[static_cast<size_t>(*s_opt)]);  // line 32
        } else {
          handoff = *rn;  // line 34: break inner loop, continue outer WITH q kept
          break;
        }
      }
      q.pop_back();  // line 35 (runs on every non-handoff branch)
    }
    r_opt = handoff;
  }
  return true;
}

core::MultiPlanResult PushAndRotate::fail(core::TraceRecorder* recorder, int expanded) const {
  // Honest failure (the prioritized branch's convention): no paths, 0 cost. BFS
  // expansions counted up to the failure are still reported.
  if (recorder != nullptr) {
    recorder->planning_finished(false, {{"expanded_nodes", static_cast<double>(expanded)},
                                        {"makespan", 0.0},
                                        {"sum_of_costs", 0.0}});
  }
  core::MultiPlanResult result;
  result.success = false;
  result.stats.expanded_nodes = expanded;
  return result;
}

core::MultiPlanResult PushAndRotate::plan(const DiscreteSpace& space,
                                          const std::vector<core::AgentTask>& tasks,
                                          core::TraceRecorder* recorder) {
  Sim sim;
  for (const Cell& c : space.cells()) sim.free_cells.insert(c);
  std::vector<Cell> starts, goals;
  for (const auto& t : tasks) {
    starts.push_back(t.start);
    goals.push_back(t.goal);
  }
  // The paper's assignment is injective by definition (unique starts AND unique
  // targets) and every involved cell must be passable. A violating input is not an
  // instance of this problem — report it honestly, no plan.
  {
    std::set<Cell> s_starts(starts.begin(), starts.end());
    std::set<Cell> s_goals(goals.begin(), goals.end());
    bool invalid = s_starts.size() < starts.size() || s_goals.size() < goals.size();
    for (const Cell& c : starts) invalid = invalid || !sim.free_cells.count(c);
    for (const Cell& c : goals) invalid = invalid || !sim.free_cells.count(c);
    if (invalid) return fail(recorder, 0);
  }
  sim.A = starts;
  sim.T = goals;
  sim.pi.push_back(starts);

  const int m = static_cast<int>(sim.free_cells.size()) - static_cast<int>(tasks.size());
  const std::vector<std::set<Cell>> s_list = find_subgraphs(sim.free_cells, m);
  const std::vector<std::optional<int>> f_a = assign_agents(sim, sim.A, s_list, m);
  const std::vector<std::optional<int>> f_t = assign_agents(sim, sim.T, s_list, m);
  if (f_a != f_t) return fail(recorder, sim.expanded_nodes);  // Alg 7 line 5: not solvable at all
  const auto rels = subgraph_priority(sim.free_cells, s_list, sim.T, f_a);
  const auto cl = closure(rels);
  for (const auto& [a, b] : cl) {
    if (a == b) return fail(recorder, sim.expanded_nodes);  // cyclic priority — unsolvable
  }
  if (!solve(sim, s_list, f_a, cl)) return fail(recorder, sim.expanded_nodes);

  // Full-horizon space-time paths: paths[k][t] is the cell agent k occupies at
  // global step t. Agents get displaced by other agents' swap/rotate maneuvers and
  // ride the replayed segment back home — a trimmed path would misrepresent that.
  std::vector<std::vector<Cell>> paths(tasks.size());
  double cost = 0.0;
  for (size_t k = 0; k < tasks.size(); ++k) {
    for (size_t t = 0; t < sim.pi.size(); ++t) {
      if (t > 0 && !(sim.pi[t][k] == sim.pi[t - 1][k])) cost += 1.0;
      paths[k].push_back(sim.pi[t][k]);
    }
  }
  if (recorder != nullptr) {
    for (size_t k = 0; k < paths.size(); ++k) {
      recorder->path_found(paths[k], static_cast<int>(k));
    }
    recorder->planning_finished(true, {
      {"expanded_nodes", static_cast<double>(sim.expanded_nodes)},
      {"makespan", static_cast<double>(sim.pi.size() - 1)},
      {"sum_of_costs", cost},
    });
  }
  core::MultiPlanResult result;
  result.success = true;
  result.paths = std::move(paths);
  result.cost = cost;
  result.stats.expanded_nodes = sim.expanded_nodes;
  return result;
}

}  // namespace mrmp::search
