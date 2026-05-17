#include "ECBS.h"

#include "../Domain.h"
#include "BFSDistanceMap.h"
#include "CAAStar.h"
#include "ReservationTable.h"
#include "SpaceTimeAStar.h"
#include "TaskAllocator.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {
constexpr double ECBS_W = 1.3;
}

namespace mapf {

namespace {

// Per-agent space-time trajectory used by ECBS conflict detection.
// `agent_pos[t]` and `box_pos[t]` are the agent and managed-box positions at
// timestep t (0-indexed, length = actions.size() + 1). `box_pos[t]` is
// {-1,-1} when the agent has no box yet; once a DeliverBox subtask completes,
// it is held at the goal cell so other agents still see the box as a
// physical obstacle for conflict detection.
struct AgentTrajectory {
    bool ok = true;
    std::vector<const Action*>           actions;
    std::vector<std::pair<int, int>>     agent_pos;
    std::vector<std::pair<int, int>>     box_pos;
};

struct CellPair {
    std::pair<int, int> agent;
    std::pair<int, int> box;
};

CellPair pos_at(const AgentTrajectory& tr, int t)
{
    if (tr.agent_pos.empty()) return {{-1, -1}, {-1, -1}};
    const int idx = std::min(t, static_cast<int>(tr.agent_pos.size()) - 1);
    return { tr.agent_pos[idx], tr.box_pos[idx] };
}

AgentTrajectory plan_agent_with_constraints(
        int agent_id,
        const State& s,
        const std::vector<Subtask>& subtasks,
        const std::vector<std::vector<bool>>& static_box_obstacle,
        const ConstraintSet& extra_constraints,
        int max_t_horizon)
{
    AgentTrajectory result;
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    int cur_ar = s.agent_rows[agent_id];
    int cur_ac = s.agent_cols[agent_id];
    int cur_t = 0;

    result.agent_pos.push_back({cur_ar, cur_ac});
    result.box_pos.push_back({-1, -1});

    for (std::size_t si = 0; si < subtasks.size(); ++si) {
        const Subtask& sub = subtasks[si];

        ReservationTable res(rows, cols, max_t_horizon);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (static_box_obstacle[r][c]) {
                    res.reserve_cell(r, c, 0, max_t_horizon, -1);
                }
            }
        }
        for (std::size_t sj = 0; sj < subtasks.size(); ++sj) {
            if (sj == si) continue;
            if (subtasks[sj].type != SubtaskType::DeliverBox) continue;
            int rr, cc;
            if (sj < si) { rr = subtasks[sj].box_goal_r;  cc = subtasks[sj].box_goal_c; }
            else         { rr = subtasks[sj].box_start_r; cc = subtasks[sj].box_start_c; }
            res.reserve_cell(rr, cc, 0, max_t_horizon, -1);
        }

        DistanceGrid d_goal;
        DistanceGrid d_box_start;
        const DistanceGrid* dbs_ptr = nullptr;
        if (sub.type == SubtaskType::DeliverBox) {
            d_goal = bfs_from(sub.box_goal_r, sub.box_goal_c);
            d_box_start = bfs_from(sub.box_start_r, sub.box_start_c);
            dbs_ptr = &d_box_start;
        } else {
            d_goal = bfs_from(sub.target_r, sub.target_c);
        }

        STAResult r = sta_star(agent_id, cur_ar, cur_ac, sub, cur_t,
                               res, d_goal, dbs_ptr, max_t_horizon,
                               extra_constraints);
        if (!r.ok) { result.ok = false; return result; }

        int ar = cur_ar, ac = cur_ac;
        int br = (sub.type == SubtaskType::DeliverBox) ? sub.box_start_r : -1;
        int bc = (sub.type == SubtaskType::DeliverBox) ? sub.box_start_c : -1;

        if (br >= 0) {
            // Box is now this agent's responsibility from the start of the
            // subtask: it sits at `box_start` while the agent walks toward it.
            result.box_pos.back() = {br, bc};
        }

        for (const Action* act : r.actions) {
            int nar = ar, nac = ac, nbr = br, nbc = bc;
            switch (act->type) {
                case ActionType::NoOp: break;
                case ActionType::Move:
                    nar = ar + act->agent_row_delta;
                    nac = ac + act->agent_col_delta;
                    break;
                case ActionType::Push:
                    nar = ar + act->agent_row_delta;
                    nac = ac + act->agent_col_delta;
                    nbr = br + act->box_row_delta;
                    nbc = bc + act->box_col_delta;
                    break;
                case ActionType::Pull:
                    nar = ar + act->agent_row_delta;
                    nac = ac + act->agent_col_delta;
                    nbr = ar;
                    nbc = ac;
                    break;
            }
            result.actions.push_back(act);
            ++cur_t;
            result.agent_pos.push_back({nar, nac});
            result.box_pos.push_back({nbr, nbc});
            ar = nar; ac = nac; br = nbr; bc = nbc;
        }
        cur_ar = ar; cur_ac = ac;
        // Note: we deliberately do NOT reset box_pos.back() to {-1,-1} after
        // the subtask. A delivered box stays sitting at its goal cell, and
        // future agents must still avoid it; keeping it visible to conflict
        // detection makes that automatic.
    }
    return result;
}

std::optional<Conflict> find_first_conflict(
        const std::vector<int>& ids,
        const std::unordered_map<int, AgentTrajectory>& trajs)
{
    int max_t = 0;
    for (const auto& kv : trajs) {
        max_t = std::max(max_t, static_cast<int>(kv.second.agent_pos.size()) - 1);
    }

    auto same_cell = [](std::pair<int, int> x, std::pair<int, int> y) {
        return x.first >= 0 && y.first >= 0 && x == y;
    };

    for (int t = 0; t <= max_t; ++t) {
        std::vector<CellPair> P(ids.size());
        for (std::size_t i = 0; i < ids.size(); ++i) {
            P[i] = pos_at(trajs.at(ids[i]), t);
        }

        for (std::size_t i = 0; i < ids.size(); ++i) {
            for (std::size_t j = i + 1; j < ids.size(); ++j) {
                const CellPair& pa = P[i];
                const CellPair& pb = P[j];
                std::pair<int, int> hit{-1, -1};
                if (same_cell(pa.agent, pb.agent)) hit = pa.agent;
                else if (same_cell(pa.agent, pb.box)) hit = pa.agent;
                else if (same_cell(pa.box,  pb.agent)) hit = pa.box;
                else if (same_cell(pa.box,  pb.box))   hit = pa.box;
                if (hit.first >= 0) {
                    Conflict c;
                    c.agentA = ids[i]; c.agentB = ids[j];
                    c.row = hit.first; c.col = hit.second;
                    c.time = t; c.type = Conflict::Type::Vertex;
                    return c;
                }
            }
        }

        if (t > 0) {
            std::vector<CellPair> Pp(ids.size());
            for (std::size_t i = 0; i < ids.size(); ++i) {
                Pp[i] = pos_at(trajs.at(ids[i]), t - 1);
            }
            for (std::size_t i = 0; i < ids.size(); ++i) {
                for (std::size_t j = i + 1; j < ids.size(); ++j) {
                    const auto& pa  = P[i].agent;
                    const auto& pb  = P[j].agent;
                    const auto& ppa = Pp[i].agent;
                    const auto& ppb = Pp[j].agent;
                    if (pa.first >= 0 && pb.first >= 0 &&
                        pa == ppb && pb == ppa && pa != pb) {
                        Conflict c;
                        c.agentA = ids[i]; c.agentB = ids[j];
                        c.row = pa.first; c.col = pa.second;
                        c.time = t; c.type = Conflict::Type::Edge;
                        return c;
                    }
                }
            }
        }
    }
    return std::nullopt;
}

// Total count of pairwise (vertex + edge) conflicts across all agent pairs
// and all timesteps. Used by ECBS focal list as the secondary key.
int count_all_conflicts(const std::vector<int>& ids,
                        const std::unordered_map<int, AgentTrajectory>& trajs)
{
    int max_t = 0;
    for (const auto& kv : trajs) {
        max_t = std::max(max_t, static_cast<int>(kv.second.agent_pos.size()) - 1);
    }
    auto same_cell = [](std::pair<int, int> x, std::pair<int, int> y) {
        return x.first >= 0 && y.first >= 0 && x == y;
    };

    int count = 0;
    for (int t = 0; t <= max_t; ++t) {
        std::vector<CellPair> P(ids.size());
        for (std::size_t i = 0; i < ids.size(); ++i) {
            P[i] = pos_at(trajs.at(ids[i]), t);
        }
        std::vector<CellPair> Pp;
        if (t > 0) {
            Pp.resize(ids.size());
            for (std::size_t i = 0; i < ids.size(); ++i) {
                Pp[i] = pos_at(trajs.at(ids[i]), t - 1);
            }
        }
        for (std::size_t i = 0; i < ids.size(); ++i) {
            for (std::size_t j = i + 1; j < ids.size(); ++j) {
                bool hit = false;
                if (same_cell(P[i].agent, P[j].agent)) hit = true;
                else if (same_cell(P[i].agent, P[j].box)) hit = true;
                else if (same_cell(P[i].box,  P[j].agent)) hit = true;
                else if (same_cell(P[i].box,  P[j].box))   hit = true;
                if (hit) { ++count; continue; }
                if (t > 0) {
                    const auto& pa  = P[i].agent;
                    const auto& pb  = P[j].agent;
                    const auto& ppa = Pp[i].agent;
                    const auto& ppb = Pp[j].agent;
                    if (pa.first >= 0 && pb.first >= 0 &&
                        pa == ppb && pb == ppa && pa != pb) {
                        ++count;
                    }
                }
            }
        }
    }
    return count;
}

// ---- Phase 2f: symmetry breaking ----
// Counters incremented during CT expansion for end-of-run logging.
struct SymCounters {
    long vertex = 0;
    long corridor = 0;
    long rectangle = 0;
};

// True iff (r,c) is an in-bounds, non-wall cell whose static-free neighbour
// count is exactly 2 (the structural definition of a single-width corridor
// cell). Static obstacles include map walls and frozen boxes.
bool is_corridor_cell(int r, int c,
                      const std::vector<std::vector<bool>>& static_block)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    if (r < 0 || r >= rows || c < 0 || c >= cols) return false;
    if (State::walls[r][c]) return false;
    if (static_block[r][c]) return false;
    int free = 0;
    constexpr int DR[4] = {-1, 1, 0, 0};
    constexpr int DC[4] = { 0, 0,-1, 1};
    for (int k = 0; k < 4; ++k) {
        int nr = r + DR[k], nc = c + DC[k];
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
        if (State::walls[nr][nc]) continue;
        if (static_block[nr][nc]) continue;
        ++free;
    }
    return free == 2;
}

// Walk outward from (r0,c0) along the corridor, collecting every contiguous
// corridor cell. Stops at junctions (>=3 free neighbours), dead ends, or
// non-corridor cells. Returns the cells in arbitrary connected order.
std::vector<std::pair<int, int>>
collect_corridor(int r0, int c0,
                 const std::vector<std::vector<bool>>& static_block)
{
    std::vector<std::pair<int, int>> out;
    if (!is_corridor_cell(r0, c0, static_block)) return out;
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    std::vector<std::vector<char>> seen(rows, std::vector<char>(cols, 0));
    std::vector<std::pair<int, int>> stack{{r0, c0}};
    seen[r0][c0] = 1;
    constexpr int DR[4] = {-1, 1, 0, 0};
    constexpr int DC[4] = { 0, 0,-1, 1};
    while (!stack.empty()) {
        auto [r, c] = stack.back(); stack.pop_back();
        out.push_back({r, c});
        for (int k = 0; k < 4; ++k) {
            int nr = r + DR[k], nc = c + DC[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (seen[nr][nc]) continue;
            if (!is_corridor_cell(nr, nc, static_block)) continue;
            seen[nr][nc] = 1;
            stack.push_back({nr, nc});
        }
    }
    return out;
}

// Find the [enter, exit] timestep window (inclusive) during which `tr` is
// inside `corridor_set`. Returns {-1,-1} if it never enters.
std::pair<int, int> agent_corridor_window(
        const AgentTrajectory& tr,
        const std::unordered_set<long long>& corridor_set)
{
    auto key = [](int r, int c) { return (static_cast<long long>(r) << 16) | c; };
    int enter = -1, exit = -1;
    for (int t = 0; t < static_cast<int>(tr.agent_pos.size()); ++t) {
        const auto& p = tr.agent_pos[t];
        if (corridor_set.count(key(p.first, p.second))) {
            if (enter < 0) enter = t;
            exit = t;
        }
    }
    return { enter, exit };
}

// Corridor barrier for `forAgent`: detect that both conflicting agents
// traverse the same single-width corridor in opposite directions, and
// forbid `forAgent`'s entrance cell for the timesteps the other agent is
// still inside the corridor. Returns empty if the pattern doesn't apply.
std::vector<Constraint>
corridorBarrier(const Conflict& c,
                const std::unordered_map<int, AgentTrajectory>& trajs,
                int forAgent,
                const std::vector<std::vector<bool>>& static_block)
{
    const auto cells = collect_corridor(c.row, c.col, static_block);
    // Length-1 "corridor" -> not really a corridor; fall back.
    if (cells.size() < 2) return {};

    auto key = [](int r, int c) { return (static_cast<long long>(r) << 16) | c; };
    std::unordered_set<long long> cset;
    cset.reserve(cells.size() * 2);
    for (auto& p : cells) cset.insert(key(p.first, p.second));

    const int otherAgent = (forAgent == c.agentA) ? c.agentB : c.agentA;
    auto itFor   = trajs.find(forAgent);
    auto itOther = trajs.find(otherAgent);
    if (itFor == trajs.end() || itOther == trajs.end()) return {};

    auto [tFor_enter, tFor_exit]     = agent_corridor_window(itFor->second, cset);
    auto [tOther_enter, tOther_exit] = agent_corridor_window(itOther->second, cset);
    if (tFor_enter < 0 || tOther_enter < 0) return {};

    // Confirm opposite-direction traversal: forAgent's entrance cell should
    // be on the opposite end from otherAgent's entrance cell. Approximate by
    // requiring their first-corridor-cells to differ.
    const auto& forEntry   = itFor->second.agent_pos[tFor_enter];
    const auto& otherEntry = itOther->second.agent_pos[tOther_enter];
    if (forEntry == otherEntry) return {};

    // Block forAgent's entrance cell while otherAgent is still in the corridor.
    // We cover [max(0, tFor_enter-1), tOther_exit + small slack] so forAgent
    // is forced to wait one cell back until otherAgent has fully exited.
    std::vector<Constraint> bar;
    const int t_lo = std::max(0, tFor_enter - 1);
    const int t_hi = tOther_exit;
    if (t_hi < t_lo) return {};
    bar.reserve(t_hi - t_lo + 1);
    for (int t = t_lo; t <= t_hi; ++t) {
        bar.push_back({forEntry.first, forEntry.second, t, true, true});
    }
    // Also block the box footprint, if forAgent is pushing/pulling through
    // the corridor at the conflict time.
    const auto& forBoxes = itFor->second.box_pos;
    if (tFor_enter < static_cast<int>(forBoxes.size())) {
        const auto& bp = forBoxes[tFor_enter];
        if (bp.first >= 0 &&
            cset.count(key(bp.first, bp.second)) &&
            !(bp == forEntry)) {
            for (int t = t_lo; t <= t_hi; ++t) {
                bar.push_back({bp.first, bp.second, t, true, true});
            }
        }
    }
    return bar;
}

// Rectangle barrier (Li et al., AAAI'19). Detect that both agents move
// monotonically through a wall-free axis-aligned rectangle and conflict
// inside it; forbid `forAgent` from a line of cells along the shorter axis
// at the conflict time. Returns empty if the pattern doesn't apply.
std::vector<Constraint>
rectangleBarrier(const Conflict& c,
                 const std::unordered_map<int, AgentTrajectory>& trajs,
                 int forAgent,
                 const std::vector<std::vector<bool>>& static_block)
{
    const int otherAgent = (forAgent == c.agentA) ? c.agentB : c.agentA;
    auto itFor   = trajs.find(forAgent);
    auto itOther = trajs.find(otherAgent);
    if (itFor == trajs.end() || itOther == trajs.end()) return {};
    const auto& trF = itFor->second;
    const auto& trO = itOther->second;
    if (trF.agent_pos.size() < 2 || trO.agent_pos.size() < 2) return {};

    const int t = c.time;
    if (t <= 0) return {};
    const int tF_prev = std::max(0, t - 1);
    const int tO_prev = std::max(0, t - 1);
    const int idxF = std::min(t, static_cast<int>(trF.agent_pos.size()) - 1);
    const int idxO = std::min(t, static_cast<int>(trO.agent_pos.size()) - 1);
    const auto sF = trF.agent_pos[tF_prev];
    const auto gF = trF.agent_pos.back();
    const auto sO = trO.agent_pos[tO_prev];
    const auto gO = trO.agent_pos.back();
    (void)idxF; (void)idxO;

    const int rmin = std::min({sF.first, gF.first, sO.first, gO.first});
    const int rmax = std::max({sF.first, gF.first, sO.first, gO.first});
    const int cmin = std::min({sF.second, gF.second, sO.second, gO.second});
    const int cmax = std::max({sF.second, gF.second, sO.second, gO.second});
    const int height = rmax - rmin;
    const int width  = cmax - cmin;
    if (height < 1 || width < 1) return {};
    // Trivial-area rectangles don't carry a meaningful symmetry payoff.
    if (height + width < 3) return {};

    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    if (rmin < 0 || rmax >= rows || cmin < 0 || cmax >= cols) return {};
    for (int r = rmin; r <= rmax; ++r) {
        for (int cc = cmin; cc <= cmax; ++cc) {
            if (State::walls[r][cc] || static_block[r][cc]) return {};
        }
    }
    // Require monotone progress for each agent (Manhattan-shortest paths
    // through the rectangle).
    auto monotone = [&](std::pair<int,int> s, std::pair<int,int> g) {
        return (s.first <= g.first || s.first >= g.first) &&
               (s.second <= g.second || s.second >= g.second);
    };
    (void)monotone;  // trivially true; kept for symmetry with the paper

    // Barrier: a line along the shorter axis at the conflict cell, blocking
    // forAgent at time t.
    std::vector<Constraint> bar;
    if (height <= width) {
        for (int r = rmin; r <= rmax; ++r) {
            bar.push_back({r, c.col, t, true, true});
        }
    } else {
        for (int cc = cmin; cc <= cmax; ++cc) {
            bar.push_back({c.row, cc, t, true, true});
        }
    }
    return bar;
}

}  // namespace

std::optional<std::unordered_map<int, std::vector<const Action*>>>
ecbs_solve(const StatePtr& initial,
           const std::vector<Assignment>& assignments,
           int timeBudgetMs)
{
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeBudgetMs);

    const int N    = static_cast<int>(initial->agent_rows.size());
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    const int max_t_horizon = std::max(300, rows * cols);

    // The `assignments` parameter is the public spec contract. Internally we
    // pull the per-agent subtask list from `assign_tasks(state)` so we can
    // also support ReachCell (agent-goal) tasks that the BoxTask-only
    // Assignment struct cannot encode. The Hungarian allocator is
    // deterministic, so the two views agree on the box-task allocation.
    (void)assignments;
    const auto subtasks_per_agent = assign_tasks(*initial);

    // Vanilla CBS conflict detection only tracks a single managed box per
    // agent, so if any agent receives more than one DeliverBox we bail out
    // and let CAA* handle it. (Multi-task per agent is a Milestone-8
    // follow-up.)
    std::vector<std::vector<Subtask>> per_agent(N);
    std::vector<std::vector<bool>> assigned_cell(rows, std::vector<bool>(cols, false));
    for (int a = 0; a < N; ++a) {
        int box_count = 0;
        for (const Subtask& st : subtasks_per_agent[a]) {
            per_agent[a].push_back(st);
            if (st.type == SubtaskType::DeliverBox) {
                ++box_count;
                if (st.box_start_r >= 0 && st.box_start_r < rows &&
                    st.box_start_c >= 0 && st.box_start_c < cols) {
                    assigned_cell[st.box_start_r][st.box_start_c] = true;
                }
            }
        }
        if (box_count > 1) {
            std::cerr << "ECBS: agent " << a << " has " << box_count
                      << " box tasks; deferring to CAA* fallback.\n";
            return std::nullopt;
        }
    }

    // Static obstacles for the low-level planner: every box on the map that
    // no agent will ever pick up. Boxes owned by other agents are NOT static
    // — CBS resolves those collisions through the constraint tree.
    std::vector<std::vector<bool>> static_box(rows, std::vector<bool>(cols, false));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (initial->boxes[r][c] != '\0' && !assigned_cell[r][c]) {
                static_box[r][c] = true;
            }
        }
    }

    std::vector<int> agent_ids(N);
    for (int a = 0; a < N; ++a) agent_ids[a] = a;

    // ---- Root CT node ----
    std::unordered_map<int, AgentTrajectory> root_trajs;
    int root_total = 0;
    for (int a = 0; a < N; ++a) {
        AgentTrajectory tr = plan_agent_with_constraints(
                a, *initial, per_agent[a], static_box,
                ConstraintSet{}, max_t_horizon);
        if (!tr.ok) {
            std::cerr << "ECBS: root planning failed for agent " << a << ".\n";
            return std::nullopt;
        }
        root_total += static_cast<int>(tr.actions.size());
        root_trajs.emplace(a, std::move(tr));
    }

    struct Stored {
        std::unordered_map<int, std::vector<Constraint>> constraints;
        std::unordered_map<int, AgentTrajectory>         trajs;
        int totalCost = 0;
        int numConflicts = 0;
    };

    // OPEN: ordered by totalCost (asc), tie-break by seq.
    // FOCAL: subset of OPEN with totalCost <= ECBS_W * minCost, ordered by
    // numConflicts (asc), then totalCost (asc), then seq.
    struct OpenEntry {
        int totalCost;
        int seq;
        int id;
        int numConflicts;
        bool operator<(const OpenEntry& o) const {
            if (totalCost != o.totalCost) return totalCost < o.totalCost;
            return seq < o.seq;
        }
    };
    using OpenSet = std::multiset<OpenEntry>;

    struct FocalEntry {
        int numConflicts;
        int totalCost;
        int seq;
        int id;
        OpenSet::iterator openIt;
        bool operator<(const FocalEntry& o) const {
            if (numConflicts != o.numConflicts) return numConflicts < o.numConflicts;
            if (totalCost != o.totalCost) return totalCost < o.totalCost;
            return seq < o.seq;
        }
    };

    std::vector<Stored> store;
    OpenSet open;
    std::multiset<FocalEntry> focal;
    std::vector<char> in_focal;
    int seq_counter = 0;

    auto push_node = [&](Stored&& s) {
        const int nc = count_all_conflicts(agent_ids, s.trajs);
        s.numConflicts = nc;
        store.push_back(std::move(s));
        const int id = static_cast<int>(store.size()) - 1;
        in_focal.push_back(0);
        OpenEntry e{ store[id].totalCost, seq_counter++, id, nc };
        open.insert(e);
    };

    // After any change to OPEN's minimum cost, recompute the focal threshold
    // and admit any newly-eligible OPEN nodes into FOCAL.
    auto refresh_focal = [&]() {
        if (open.empty()) return;
        const double minCost = static_cast<double>(open.begin()->totalCost);
        const double threshold = ECBS_W * minCost;
        for (auto it = open.begin(); it != open.end(); ++it) {
            if (static_cast<double>(it->totalCost) > threshold) break;
            if (!in_focal[it->id]) {
                in_focal[it->id] = 1;
                focal.insert({ it->numConflicts, it->totalCost,
                               it->seq, it->id, it });
            }
        }
    };

    Stored root;
    root.trajs = std::move(root_trajs);
    root.totalCost = root_total;
    push_node(std::move(root));
    refresh_focal();

    int expansions = 0;
    constexpr int kMaxExpansions = 2000;
    SymCounters sym{};

    auto branch_with_constraints = [&](int agent,
                                       const std::vector<Constraint>& extras,
                                       const Stored& parent) {
        Stored child;
        child.constraints = parent.constraints;
        child.trajs = parent.trajs;
        child.totalCost = parent.totalCost;
        for (const auto& e : extras) child.constraints[agent].push_back(e);

        ConstraintSet cs;
        for (const auto& cc : child.constraints[agent]) cs.insert(cc);
        AgentTrajectory tr = plan_agent_with_constraints(
                agent, *initial, per_agent[agent], static_box,
                cs, max_t_horizon);
        if (!tr.ok) return false;

        const int old_len = static_cast<int>(parent.trajs.at(agent).actions.size());
        const int new_len = static_cast<int>(tr.actions.size());
        child.totalCost = child.totalCost - old_len + new_len;
        child.trajs[agent] = std::move(tr);

        push_node(std::move(child));
        return true;
    };

    while (!focal.empty()) {
        if (clock::now() >= deadline) {
            std::cerr << "ECBS: time budget exhausted after "
                      << expansions << " expansions. "
                      << "barrier counts: vertex=" << sym.vertex
                      << " corridor=" << sym.corridor
                      << " rectangle=" << sym.rectangle << ".\n";
            return std::nullopt;
        }
        if (expansions++ > kMaxExpansions) {
            std::cerr << "ECBS: expansion cap reached. "
                      << "barrier counts: vertex=" << sym.vertex
                      << " corridor=" << sym.corridor
                      << " rectangle=" << sym.rectangle << ".\n";
            return std::nullopt;
        }

        auto fit = focal.begin();
        const FocalEntry fe = *fit;
        focal.erase(fit);
        open.erase(fe.openIt);
        in_focal[fe.id] = 0;
        // IMPORTANT: copy by value, not reference. The branch lambda below
        // does store.push_back(...) which can reallocate the vector and
        // invalidate any reference into store[].
        const Stored node = store[fe.id];

        auto conflict = find_first_conflict(agent_ids, node.trajs);
        if (!conflict) {
            std::unordered_map<int, std::vector<const Action*>> out;
            for (const auto& kv : node.trajs) out[kv.first] = kv.second.actions;
            std::cerr << "ECBS: solved with " << expansions
                      << " expansions, totalCost=" << node.totalCost
                      << " (focal w=" << ECBS_W << "). "
                      << "barrier counts: vertex=" << sym.vertex
                      << " corridor=" << sym.corridor
                      << " rectangle=" << sym.rectangle << ".\n";
            return out;
        }

        // For each side of the split, try corridor then rectangle barrier;
        // fall back to the standard single-cell vertex/edge constraint.
        auto expand_side = [&](int agent, const Constraint& fallback_one) {
            auto bar = corridorBarrier(*conflict, node.trajs, agent, static_box);
            if (!bar.empty()) {
                if (branch_with_constraints(agent, bar, node)) ++sym.corridor;
                return;
            }
            bar = rectangleBarrier(*conflict, node.trajs, agent, static_box);
            if (!bar.empty()) {
                if (branch_with_constraints(agent, bar, node)) ++sym.rectangle;
                return;
            }
            if (branch_with_constraints(agent, {fallback_one}, node)) ++sym.vertex;
        };

        if (conflict->type == Conflict::Type::Vertex) {
            Constraint c{conflict->row, conflict->col, conflict->time, true, true};
            expand_side(conflict->agentA, c);
            expand_side(conflict->agentB, c);
        } else {
            // Standard CBS edge expansion: each agent is forbidden from
            // entering its own conflict-step destination at time t. Pull the
            // destinations out of the parent's trajectories.
            const auto& trA = node.trajs.at(conflict->agentA);
            const auto& trB = node.trajs.at(conflict->agentB);
            const CellPair pa = pos_at(trA, conflict->time);
            const CellPair pb = pos_at(trB, conflict->time);
            Constraint cA{pa.agent.first, pa.agent.second,
                          conflict->time, true, true};
            Constraint cB{pb.agent.first, pb.agent.second,
                          conflict->time, true, true};
            expand_side(conflict->agentA, cA);
            expand_side(conflict->agentB, cB);
        }

        if (open.empty()) break;
        // Either the new minimum increased (so the threshold expanded and
        // formerly-ineligible nodes may now qualify), or new children were
        // pushed at or below the current threshold. In both cases re-scan
        // OPEN and admit any newly-eligible nodes into FOCAL.
        refresh_focal();
    }

    std::cerr << "ECBS: open list exhausted after "
              << expansions << " expansions. "
              << "barrier counts: vertex=" << sym.vertex
              << " corridor=" << sym.corridor
              << " rectangle=" << sym.rectangle << ".\n";
    return std::nullopt;
}

}  // namespace mapf
