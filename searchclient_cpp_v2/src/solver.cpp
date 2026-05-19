#include "aimas/solver.hpp"
#include "aimas/compact.hpp"
#include "aimas/pibt.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <set>
#include <tuple>

namespace aimas {

namespace {

int manhattan(int r1, int c1, int r2, int c2)
{
    return std::abs(r1 - r2) + std::abs(c1 - c2);
}

}  // namespace

Solver::Solver(const Level& level)
    : level_(level),
      initial_state_(State::initial(level)),
      state_(initial_state_),
      topology_(level),
      planner_(state_, topology_)
{
}

std::vector<int> Solver::noop_joint() const
{
    return std::vector<int>(state_.agent_rows.size(), kNoOpIndex);
}

void Solver::append_joint(const std::vector<int>& joint)
{
    plan_.push_back(joint);
}

std::vector<Solver::Task> Solver::build_tasks_matched(int goal_order_mode)
{
    // For each goal cell with a letter, pick a same-letter box that isn't
    // already on a matching goal cell, and a same-color agent. The matching
    // step uses topology distances (BFS over walls) rather than Manhattan, so
    // pairings reflect actual corridor lengths.
    std::vector<Task> tasks;
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        const int color = level_.box_color[letter - 'A'];
        if (color < 0) continue;
        bool has_agent = false;
        for (int agent_color : level_.agent_color) {
            if (agent_color == color) { has_agent = true; break; }
        }
        if (!has_agent) continue;

        std::vector<std::pair<int, int>> goals;
        std::vector<std::pair<int, int>> boxes;
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (level_.goals[row][col] == letter
                    && state_.boxes[row][col] != letter) {
                    goals.emplace_back(row, col);
                }
                if (state_.boxes[row][col] == letter
                    && level_.goals[row][col] != letter) {
                    boxes.emplace_back(row, col);
                }
            }
        }
        if (goals.empty() || boxes.empty()) continue;

        // Score goals by nearest available box (via walls-only BFS).
        auto nearest_box_d = [&](int gr, int gc) -> int {
            const auto& dist = topology_.dist_walls_only(gr, gc);
            int best = kInf;
            for (const auto& b : boxes) {
                if (dist[b.first][b.second] < best) best = dist[b.first][b.second];
            }
            return best;
        };

        std::stable_sort(goals.begin(), goals.end(),
            [&](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
                switch (goal_order_mode) {
                    case 1:  // bottom-right first (reverse row,col)
                        if (lhs.first != rhs.first) return lhs.first > rhs.first;
                        return lhs.second > rhs.second;
                    case 2: {  // hardest first (largest nearest-box distance)
                        const int la = nearest_box_d(lhs.first, lhs.second);
                        const int lb = nearest_box_d(rhs.first, rhs.second);
                        if (la != lb) return la > lb;
                        break;
                    }
                    case 3: {  // easiest first (smallest nearest-box distance)
                        const int la = nearest_box_d(lhs.first, lhs.second);
                        const int lb = nearest_box_d(rhs.first, rhs.second);
                        if (la != lb) return la < lb;
                        break;
                    }
                    default: break;
                }
                if (lhs.first != rhs.first) return lhs.first < rhs.first;
                return lhs.second < rhs.second;
            });

        std::set<int> used;
        for (const auto& g : goals) {
            const auto& dist = topology_.dist_walls_only(g.first, g.second);
            int best_i = -1, best_d = kInf;
            for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                if (used.count(i)) continue;
                const int d = dist[boxes[i].first][boxes[i].second];
                if (d < best_d) { best_d = d; best_i = i; }
            }
            if (best_i < 0 || best_d == kInf) continue;
            used.insert(best_i);

            int agent_pick = -1, ad = kInf;
            for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
                if (level_.agent_color[a] != color) continue;
                const int d = manhattan(state_.agent_rows[a], state_.agent_cols[a],
                                        boxes[best_i].first, boxes[best_i].second);
                if (d < ad) { ad = d; agent_pick = a; }
            }
            if (agent_pick < 0) continue;

            Task t;
            t.letter = letter;
            t.box_row = boxes[best_i].first;
            t.box_col = boxes[best_i].second;
            t.goal_row = g.first;
            t.goal_col = g.second;
            t.agent = agent_pick;
            tasks.push_back(t);
        }
    }
    return tasks;
}

std::vector<Solver::Task> Solver::build_tasks_matched_dp()
{
    // DP-optimal box→goal assignment per letter: bitmask DP minimising the
    // sum of walls-only distances from each chosen box to its assigned goal.
    // Falls back to per-letter greedy when boxes.size() > kMaxBoxesDp to
    // keep state count manageable (2^N). Mirrors cpp_enhanced's
    // build_matched_tasks.
    constexpr int kMaxBoxesDp = 12;  // 4096 mask states is comfortable.
    std::vector<Task> tasks;
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        const int color = level_.box_color[letter - 'A'];
        if (color < 0) continue;
        bool has_agent = false;
        for (int agent_color : level_.agent_color) {
            if (agent_color == color) { has_agent = true; break; }
        }
        if (!has_agent) continue;

        std::vector<std::pair<int, int>> goals;
        std::vector<std::pair<int, int>> boxes;
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (level_.goals[r][c] == letter && state_.boxes[r][c] != letter)
                    goals.emplace_back(r, c);
                if (state_.boxes[r][c] == letter && level_.goals[r][c] != letter)
                    boxes.emplace_back(r, c);
            }
        }
        if (goals.empty() || boxes.empty()) continue;

        // If too many boxes or more goals than boxes, fall back to greedy
        // matching for this letter so we still emit something.
        if (static_cast<int>(boxes.size()) > kMaxBoxesDp
            || goals.size() > boxes.size()) {
            std::set<int> used;
            for (const auto& g : goals) {
                const auto& dist = topology_.dist_walls_only(g.first, g.second);
                int best_i = -1, best_d = kInf;
                for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                    if (used.count(i)) continue;
                    const int d = dist[boxes[i].first][boxes[i].second];
                    if (d < best_d) { best_d = d; best_i = i; }
                }
                if (best_i < 0 || best_d == kInf) continue;
                used.insert(best_i);
                int agent_pick = -1, ad = kInf;
                for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
                    if (level_.agent_color[a] != color) continue;
                    const int d = manhattan(state_.agent_rows[a],
                                            state_.agent_cols[a],
                                            boxes[best_i].first, boxes[best_i].second);
                    if (d < ad) { ad = d; agent_pick = a; }
                }
                if (agent_pick < 0) continue;
                Task t;
                t.letter = letter;
                t.box_row = boxes[best_i].first;
                t.box_col = boxes[best_i].second;
                t.goal_row = g.first; t.goal_col = g.second;
                t.agent = agent_pick;
                tasks.push_back(t);
            }
            continue;
        }

        const int B = static_cast<int>(boxes.size());
        const int G = static_cast<int>(goals.size());
        const int mask_count = 1 << B;
        std::vector<int> dp(mask_count, kInf);
        std::vector<std::vector<int>> parent_mask(
            G, std::vector<int>(mask_count, -1));
        std::vector<std::vector<int>> parent_box(
            G, std::vector<int>(mask_count, -1));
        dp[0] = 0;

        auto popcount = [](int m) {
            int c = 0; while (m) { c += m & 1; m >>= 1; } return c;
        };

        for (int gi = 0; gi < G; ++gi) {
            std::vector<int> next(mask_count, kInf);
            const auto& dist = topology_.dist_walls_only(goals[gi].first,
                                                         goals[gi].second);
            for (int mask = 0; mask < mask_count; ++mask) {
                if (dp[mask] == kInf || popcount(mask) != gi) continue;
                for (int bi = 0; bi < B; ++bi) {
                    if (mask & (1 << bi)) continue;
                    const int d = dist[boxes[bi].first][boxes[bi].second];
                    if (d == kInf) continue;
                    const int nm = mask | (1 << bi);
                    const int cost = dp[mask] + d;
                    if (cost < next[nm]) {
                        next[nm] = cost;
                        parent_mask[gi][nm] = mask;
                        parent_box[gi][nm] = bi;
                    }
                }
            }
            dp.swap(next);
        }

        int best_mask = -1, best_cost = kInf;
        for (int mask = 0; mask < mask_count; ++mask) {
            if (popcount(mask) == G && dp[mask] < best_cost) {
                best_cost = dp[mask]; best_mask = mask;
            }
        }
        if (best_mask < 0) continue;

        std::vector<int> assignment(G, -1);
        for (int gi = G - 1; gi >= 0 && best_mask >= 0; --gi) {
            const int bi = parent_box[gi][best_mask];
            assignment[gi] = bi;
            best_mask = parent_mask[gi][best_mask];
        }
        for (int gi = 0; gi < G; ++gi) {
            const int bi = assignment[gi];
            if (bi < 0) continue;
            int agent_pick = -1, ad = kInf;
            for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
                if (level_.agent_color[a] != color) continue;
                const int d = manhattan(state_.agent_rows[a],
                                        state_.agent_cols[a],
                                        boxes[bi].first, boxes[bi].second);
                if (d < ad) { ad = d; agent_pick = a; }
            }
            if (agent_pick < 0) continue;
            Task t;
            t.letter = letter;
            t.box_row = boxes[bi].first; t.box_col = boxes[bi].second;
            t.goal_row = goals[gi].first; t.goal_col = goals[gi].second;
            t.agent = agent_pick;
            tasks.push_back(t);
        }
    }
    return tasks;
}

void Solver::sort_tasks(std::vector<Task>& tasks, int final_order_mode)
{
    auto dist_of = [&](const Task& t) {
        const auto& d = topology_.dist_walls_only(t.goal_row, t.goal_col);
        const int v = d[t.box_row][t.box_col];
        return v == kInf ? manhattan(t.box_row, t.box_col, t.goal_row, t.goal_col) : v;
    };
    std::stable_sort(tasks.begin(), tasks.end(),
        [&](const Task& lhs, const Task& rhs) {
            const int ld = dist_of(lhs);
            const int rd = dist_of(rhs);
            switch (final_order_mode) {
                case 0:  // farthest first (default in cpp_enhanced)
                    if (ld != rd) return ld > rd;
                    break;
                case 1:  // nearest first
                    if (ld != rd) return ld < rd;
                    break;
                case 2:  // top-left goals first
                    if (lhs.goal_row != rhs.goal_row) return lhs.goal_row < rhs.goal_row;
                    if (lhs.goal_col != rhs.goal_col) return lhs.goal_col < rhs.goal_col;
                    break;
                case 3:  // bottom-right goals first
                    if (lhs.goal_row != rhs.goal_row) return lhs.goal_row > rhs.goal_row;
                    if (lhs.goal_col != rhs.goal_col) return lhs.goal_col > rhs.goal_col;
                    break;
                case 4:  // letter-grouped
                    if (lhs.letter != rhs.letter) return lhs.letter < rhs.letter;
                    if (ld != rd) return ld < rd;
                    break;
                default: break;
            }
            if (lhs.goal_row != rhs.goal_row) return lhs.goal_row < rhs.goal_row;
            return lhs.goal_col < rhs.goal_col;
        });
}

std::vector<std::vector<Solver::Task>> Solver::build_task_variants()
{
    // Restore initial state so build_tasks_matched sees pristine box positions.
    state_ = initial_state_;

    std::vector<std::vector<Task>> variants;
    std::set<std::string> seen;
    auto signature = [](const std::vector<Task>& v) {
        std::string s;
        s.reserve(v.size() * 16);
        for (const auto& t : v) {
            s += t.letter;
            s += ':';
            s += std::to_string(t.box_row);
            s += ',';
            s += std::to_string(t.box_col);
            s += "->";
            s += std::to_string(t.goal_row);
            s += ',';
            s += std::to_string(t.goal_col);
            s += ';';
        }
        return s;
    };
    auto add = [&](std::vector<Task> v) {
        if (v.empty()) return;
        const std::string s = signature(v);
        if (seen.insert(s).second) variants.push_back(std::move(v));
    };

    // 5 final-order modes × 4 goal-order modes; cap at 25 distinct signatures
    // (4×5 greedy + 5 DP-matched). Most final orderings collapse to the
    // same task list after sorting, so practical variant count is well
    // under the cap. We still run DP-matched first because it tends to
    // find globally cheaper assignments — preserve greedy variants by
    // letting the cap accommodate both.
    const int kMaxVariants = 25;
    // DP-optimal matching first (cheap when boxes-per-letter <= 12). Often
    // produces a different and better assignment than greedy when boxes are
    // close to each other (DECrunchy-class), unlocking deliveries that the
    // greedy-by-distance match makes infeasible.
    {
        auto dp_base = build_tasks_matched_dp();
        if (!dp_base.empty()) {
            for (int f = 0; f < 5; ++f) {
                auto v = dp_base;
                sort_tasks(v, f);
                add(std::move(v));
                if (variants.size() >= static_cast<std::size_t>(kMaxVariants)) {
                    return variants;
                }
            }
        }
    }
    for (int g = 0; g < 4; ++g) {
        auto base = build_tasks_matched(g);
        if (base.empty()) continue;
        for (int f = 0; f < 5; ++f) {
            auto v = base;
            sort_tasks(v, f);
            add(std::move(v));
            if (variants.size() >= static_cast<std::size_t>(kMaxVariants)) {
                return variants;
            }
        }
    }
    return variants;
}

bool Solver::deliver_task(Task& task)
{
    // Earlier task processing (relocation, scatter, prior deliveries) may have
    // moved the box of this letter to a different cell than the one originally
    // recorded. Re-find the box's current position by letter, preferring the
    // box closest to the original recorded position.
    {
        int best_r = -1, best_c = -1;
        int best_d = kInf;
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (state_.boxes[r][c] != task.letter) continue;
                // Skip boxes already on a same-letter goal — they're "done".
                if (level_.goals[r][c] == task.letter
                    && !(r == task.goal_row && c == task.goal_col)) continue;
                const int d = manhattan(r, c, task.box_row, task.box_col);
                if (d < best_d) {
                    best_d = d;
                    best_r = r;
                    best_c = c;
                }
            }
        }
        if (best_r < 0) {
            // No box of this letter available (perhaps already delivered on
            // some other goal). Treat as a success (no-op delivery) only if
            // the task's goal already has the correct letter.
            if (state_.boxes[task.goal_row][task.goal_col] == task.letter) {
                return true;
            }
            return false;
        }
        task.box_row = best_r;
        task.box_col = best_c;
    }

    // Try the originally-assigned agent first, then fall back to any other
    // color-compatible agent ranked by Manhattan-to-box. This is the v2
    // analogue of the cpp_enhanced "alt-agent retry" layer.
    const int box_color = level_.box_color[task.letter - 'A'];

    std::vector<int> agent_order;
    agent_order.push_back(task.agent);
    {
        std::vector<std::pair<int, int>> alts;  // (dist, agent)
        for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
            if (a == task.agent) continue;
            if (level_.agent_color[a] != box_color) continue;
            const int d = manhattan(state_.agent_rows[a], state_.agent_cols[a],
                                    task.box_row, task.box_col);
            alts.emplace_back(d, a);
        }
        std::sort(alts.begin(), alts.end());
        for (const auto& p : alts) agent_order.push_back(p.second);
    }

    for (int candidate : agent_order) {
        auto seq = planner_.plan(candidate, task.letter,
                                 task.box_row, task.box_col,
                                 task.goal_row, task.goal_col);
        if (seq.empty()) continue;

        // Embed each per-agent action into a joint action (other agents NoOp)
        // and commit via apply_joint. Roll back the whole task on any failure.
        const std::size_t snapshot_plan_len = plan_.size();
        const State snapshot_state = state_;
        bool ok = true;
        for (int ai : seq) {
            std::vector<int> joint = noop_joint();
            joint[candidate] = ai;
            if (!state_.apply_joint(joint)) {
                state_ = snapshot_state;
                plan_.resize(snapshot_plan_len);
                ok = false;
                break;
            }
            append_joint(joint);
        }
        if (ok) {
            task.agent = candidate;
            return true;
        }
    }
    return false;
}

std::vector<std::pair<int, int>> Solver::agent_goal_targets() const
{
    const int num_agents = static_cast<int>(state_.agent_rows.size());
    std::vector<std::pair<int, int>> targets(num_agents, {-1, -1});
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            const char g = level_.goals[r][c];
            if (g >= '0' && g <= '9') {
                const int a = g - '0';
                if (a >= 0 && a < num_agents) targets[a] = {r, c};
            }
        }
    }
    return targets;
}

std::set<std::pair<int, int>> Solver::agent_goal_cells() const
{
    std::set<std::pair<int, int>> cells;
    auto targets = agent_goal_targets();
    for (const auto& t : targets) {
        if (t.first >= 0) cells.insert(t);
    }
    return cells;
}

bool Solver::variant_time_up() const
{
    return std::chrono::steady_clock::now() >= variant_deadline_;
}

bool Solver::complete_agent_goals_pibt()
{
    Pibt pibt(state_, level_);
    auto targets = agent_goal_targets();
    auto joint_plan = pibt.plan(targets);
    if (joint_plan.empty()) {
        // PIBT returns empty either when already-at-goal or on failure.
        // Treat already-at-goal as success.
        bool any_off = false;
        for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
            if (targets[a].first < 0) continue;
            if (state_.agent_rows[a] != targets[a].first
                || state_.agent_cols[a] != targets[a].second) {
                any_off = true; break;
            }
        }
        return !any_off;
    }

    const std::size_t snap_len = plan_.size();
    const State snap_state = state_;
    for (const auto& joint : joint_plan) {
        if (!state_.apply_joint(joint)) {
            state_ = snap_state;
            plan_.resize(snap_len);
            return false;
        }
        append_joint(joint);
    }
    return true;
}

bool Solver::complete_agent_goals_reserved()
{
    // Cooperative time-extended planner. Each agent plans a path in
    // (row, col, time) space, reserving the cells it occupies and the
    // edges it traverses at each time-step. Later-planned agents avoid
    // those reservations. NoOp ("wait") is allowed at any step, so agents
    // can pause to let others pass.
    //
    // Agents are ordered farthest-first so the hardest path commits first
    // (it's much easier to thread short paths around a long one than
    // vice-versa). Static-agent positions (those with no numeric goal, or
    // already at goal) are reserved at every time-step so others avoid them.
    const int R = level_.rows, C = level_.cols;
    const int num_agents = static_cast<int>(state_.agent_rows.size());
    if (num_agents == 0) return true;

    auto targets = agent_goal_targets();

    auto cell_key = [&](int r, int c) -> long long {
        return static_cast<long long>(r) * C + c;
    };
    auto edge_key = [&](int fr, int fc, int tr, int tc) -> long long {
        const long long n = static_cast<long long>(R) * C;
        return cell_key(fr, fc) * n + cell_key(tr, tc);
    };

    // Compute target distance for each agent that needs to move. Sorted
    // farthest-first. Static agents (no target or already-at-target) are
    // permanently reserved at their current cell.
    struct Mover { int agent; int dist; };
    std::vector<Mover> movers;
    int total_dist = 0;
    for (int a = 0; a < num_agents; ++a) {
        const auto [tr, tc] = targets[a];
        if (tr < 0) continue;
        const int ar = state_.agent_rows[a], ac = state_.agent_cols[a];
        if (ar == tr && ac == tc) continue;
        const auto& d = topology_.dist_walls_only(tr, tc);
        const int distance = d[ar][ac];
        if (distance == kInf) return false;
        movers.push_back({a, distance});
        total_dist += distance;
    }
    if (movers.empty()) return true;
    std::stable_sort(movers.begin(), movers.end(),
        [](const Mover& l, const Mover& r) { return l.dist > r.dist; });

    const int max_time = std::min(800, std::max(80, total_dist * 3 + num_agents * 10));

    // Reservations indexed by time-step. vertex[t] = set of cells occupied
    // at time t. edge[t] = set of directed edges traversed AT time t (i.e.,
    // from cell at t-1 to cell at t).
    std::vector<std::set<long long>> rv(max_time + 1);
    std::vector<std::set<long long>> re(max_time + 1);

    // Pre-reserve cells of static agents (no target or already there) at
    // every time-step so movers don't collide with them.
    std::set<int> moving_set;
    for (const auto& m : movers) moving_set.insert(m.agent);
    for (int a = 0; a < num_agents; ++a) {
        if (moving_set.count(a)) continue;
        const long long k = cell_key(state_.agent_rows[a], state_.agent_cols[a]);
        for (int t = 0; t <= max_time; ++t) rv[t].insert(k);
    }

    // Plan each mover with reservations from prior plans. Time-extended BFS.
    auto plan_with_reservations = [&](int agent, int tr, int tc) -> std::vector<int> {
        struct Node { int r, c, t, parent, action; };
        std::vector<Node> nodes;
        nodes.reserve(8192);
        nodes.push_back({state_.agent_rows[agent], state_.agent_cols[agent],
                         0, -1, kNoOpIndex});
        std::deque<int> q;
        q.push_back(0);
        std::set<std::tuple<int, int, int>> seen;
        seen.insert({nodes[0].r, nodes[0].c, 0});

        const auto& dist_to_target = topology_.dist_walls_only(tr, tc);
        const auto& tbl = actions();
        std::array<int, 5> all_acts = {kNoOpIndex, 1, 2, 3, 4};
        const int kExpansionCap = 200000;
        int expansions = 0;

        while (!q.empty() && expansions < kExpansionCap) {
            ++expansions;
            if ((expansions & 4095) == 0 && variant_time_up()) return {};
            const int idx = q.front();
            q.pop_front();
            const Node n = nodes[idx];
            if (n.r == tr && n.c == tc) {
                bool safe = true;
                const long long k = cell_key(tr, tc);
                for (int t = n.t; t <= max_time; ++t) {
                    if (rv[t].count(k)) { safe = false; break; }
                }
                if (safe) {
                    std::vector<int> path;
                    int cur = idx;
                    while (nodes[cur].parent != -1) {
                        path.push_back(nodes[cur].action);
                        cur = nodes[cur].parent;
                    }
                    std::reverse(path.begin(), path.end());
                    return path;
                }
            }
            if (n.t >= max_time) continue;

            std::array<int, 5> acts = all_acts;
            std::stable_sort(acts.begin(), acts.end(), [&](int la, int rb) {
                const int lr = n.r + tbl[la].agent_dr;
                const int lc = n.c + tbl[la].agent_dc;
                const int rr = n.r + tbl[rb].agent_dr;
                const int rc = n.c + tbl[rb].agent_dc;
                auto d = [&](int r, int c) {
                    if (r < 0 || r >= R || c < 0 || c >= C) return kInf;
                    return dist_to_target[r][c];
                };
                return d(lr, lc) < d(rr, rc);
            });

            for (int ai : acts) {
                const Action& act = tbl[ai];
                const int nr = n.r + act.agent_dr;
                const int nc = n.c + act.agent_dc;
                const int nt = n.t + 1;
                if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                if (level_.walls[nr][nc]) continue;
                if (state_.boxes[nr][nc] != '\0') continue;
                const long long kv = cell_key(nr, nc);
                if (rv[nt].count(kv)) continue;
                if (ai != kNoOpIndex) {
                    const long long rev = edge_key(nr, nc, n.r, n.c);
                    if (re[nt].count(rev)) continue;
                }
                const auto key = std::make_tuple(nr, nc, nt);
                if (seen.count(key)) continue;
                seen.insert(key);
                nodes.push_back({nr, nc, nt, idx, ai});
                q.push_back(static_cast<int>(nodes.size()) - 1);
            }
        }
        return {};
    };

    std::vector<std::vector<int>> paths(num_agents);
    for (const auto& m : movers) {
        if (variant_time_up()) return false;
        const auto [tr, tc] = targets[m.agent];
        auto path = plan_with_reservations(m.agent, tr, tc);
        if (path.empty()) return false;
        paths[m.agent] = path;

        // Register this agent's reservations for subsequent movers.
        int r = state_.agent_rows[m.agent];
        int c = state_.agent_cols[m.agent];
        rv[0].insert(cell_key(r, c));
        for (int t = 1; t <= max_time; ++t) {
            const int ai = t <= static_cast<int>(path.size())
                ? path[t - 1] : kNoOpIndex;
            const int nr = r + actions()[ai].agent_dr;
            const int nc = c + actions()[ai].agent_dc;
            rv[t].insert(cell_key(nr, nc));
            re[t].insert(edge_key(r, c, nr, nc));
            r = nr; c = nc;
        }
    }

    // Replay paths step-by-step as joint actions.
    std::size_t makespan = 0;
    for (const auto& p : paths) makespan = std::max(makespan, p.size());
    for (std::size_t step = 0; step < makespan; ++step) {
        std::vector<int> joint(num_agents, kNoOpIndex);
        for (int a = 0; a < num_agents; ++a) {
            if (step < paths[a].size()) joint[a] = paths[a][step];
        }
        if (!state_.apply_joint(joint)) return false;
        append_joint(joint);
    }
    return true;
}

bool Solver::evacuate_final_goal_agent_blockers()
{
    // Build a forbidden zone that's the union of (a) every agent's walls-only
    // path from its current position to its numeric goal cell, plus (b) all
    // numeric goal cells themselves. Then, any agent currently sitting INSIDE
    // that zone — and not already on its own goal — needs to step aside so
    // that the serial / PIBT walk can succeed. Picks an evacuation cell from
    // `evacuation_targets`, restricted to those still walls-only-reachable
    // from the agent's own target so the agent can re-approach it later.
    const auto targets = agent_goal_targets();
    const int num_agents = static_cast<int>(state_.agent_rows.size());

    std::set<std::pair<int, int>> forbidden;
    for (int a = 0; a < num_agents; ++a) {
        if (a >= static_cast<int>(targets.size())) continue;
        const auto [tr, tc] = targets[a];
        if (tr < 0) continue;
        forbidden.insert({tr, tc});
        const int ar = state_.agent_rows[a];
        const int ac = state_.agent_cols[a];
        if (ar == tr && ac == tc) continue;
        const auto path = path_box_to_goal_ignore_boxes(ar, ac, tr, tc);
        for (const auto& cell : path) forbidden.insert(cell);
    }
    if (forbidden.empty()) return false;

    // Order: agents WITHOUT targets first (they can park anywhere safely),
    // then by ascending index. Stable so deterministic.
    std::vector<int> ordered;
    for (int a = 0; a < num_agents; ++a) {
        const int ar = state_.agent_rows[a];
        const int ac = state_.agent_cols[a];
        if (!forbidden.count({ar, ac})) continue;
        const bool has_target = a < static_cast<int>(targets.size())
            && targets[a].first >= 0;
        const bool at_own = has_target
            && ar == targets[a].first && ac == targets[a].second;
        if (at_own) continue;  // already correctly placed; leave alone
        ordered.push_back(a);
    }
    std::stable_sort(ordered.begin(), ordered.end(), [&](int lhs, int rhs) {
        const bool lh = lhs < static_cast<int>(targets.size())
            && targets[lhs].first >= 0;
        const bool rh = rhs < static_cast<int>(targets.size())
            && targets[rhs].first >= 0;
        if (lh != rh) return !lh;
        return lhs < rhs;
    });

    bool moved_any = false;
    for (int a : ordered) {
        if (variant_time_up()) return moved_any;
        const int ar = state_.agent_rows[a];
        const int ac = state_.agent_cols[a];
        if (!forbidden.count({ar, ac})) continue;  // re-check after prior moves

        const bool has_target = a < static_cast<int>(targets.size())
            && targets[a].first >= 0;

        bool moved = false;
        for (const auto& [tr, tc] : evacuation_targets(a, forbidden)) {
            // If agent has its own target, restrict candidates to cells
            // walls-only-reachable from that target. Otherwise the
            // subsequent serial walk would have no way back.
            if (has_target) {
                const auto& d = topology_.dist_walls_only(
                    targets[a].first, targets[a].second);
                if (d[tr][tc] == kInf) continue;
            }
            if (scatter_agent_to(a, tr, tc)) {
                forbidden.insert({tr, tc});
                moved = true;
                moved_any = true;
                break;
            }
        }
        if (!moved) {
            // Couldn't evict this agent; bail so we don't corrupt state.
            return moved_any;
        }
    }
    return moved_any;
}

bool Solver::complete_agent_goals()
{
    // Sequence: PIBT → serial → evac-other-agents + PIBT → serial → clear-
    // box-blockers + PIBT → serial. Each attempt is transactional: snapshot
    // and roll back on failure so we don't commit half-finished moves that
    // corrupt later attempts. Two layers of "fix the world then retry":
    //   layer 1 — evict OTHER AGENTS occupying any agent's goal cell or
    //             walls-only path to its goal (cpp_enhanced's
    //             evacuate_final_goal_agent_blockers). Critical for dense
    //             multi-agent levels (CphAirprt-class) where agents finish
    //             box deliveries near goal cells of OTHER agents.
    //   layer 2 — relocate BOX BLOCKERS off any agent's path.
    auto try_attempt = [&](auto&& fn) -> bool {
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        if (fn()) return true;
        state_ = snap_state;
        plan_.resize(snap_len);
        return false;
    };

    if (try_attempt([&]{ return complete_agent_goals_pibt(); })) return true;
    if (try_attempt([&]{ return complete_agent_goals_reserved(); })) return true;
    if (try_attempt([&]{ return complete_agent_goals_serial(); })) return true;
    for (int round = 0; round < 4; ++round) {
        if (variant_time_up()) break;
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        const bool moved = evacuate_final_goal_agent_blockers();
        if (!moved) {
            state_ = snap_state;
            plan_.resize(snap_len);
            break;
        }
        if (try_attempt([&]{ return complete_agent_goals_pibt(); })) return true;
        if (try_attempt([&]{ return complete_agent_goals_reserved(); })) return true;
        if (try_attempt([&]{ return complete_agent_goals_serial(); })) return true;
    }

    // Layer 2: snapshot before clearing box blockers so we can roll back if
    // both follow-ups fail.
    const std::size_t pre_clear_len = plan_.size();
    const State pre_clear_state = state_;
    clear_paths_to_agent_goals();

    if (try_attempt([&]{ return complete_agent_goals_pibt(); })) return true;
    if (try_attempt([&]{ return complete_agent_goals_reserved(); })) return true;
    if (try_attempt([&]{ return complete_agent_goals_serial(); })) return true;

    // One more agent-eviction pass after box clearing (box clearing may have
    // shifted agent positions onto goal cells).
    for (int round = 0; round < 2; ++round) {
        if (variant_time_up()) break;
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        const bool moved = evacuate_final_goal_agent_blockers();
        if (!moved) {
            state_ = snap_state;
            plan_.resize(snap_len);
            break;
        }
        if (try_attempt([&]{ return complete_agent_goals_pibt(); })) return true;
        if (try_attempt([&]{ return complete_agent_goals_reserved(); })) return true;
        if (try_attempt([&]{ return complete_agent_goals_serial(); })) return true;
    }

    state_ = pre_clear_state;
    plan_.resize(pre_clear_len);
    return false;
}

void Solver::clear_paths_to_agent_goals()
{
    // Goal: when an agent can't reach its numeric goal under current box
    // positions, identify boxes on its walls-only path and relocate them.
    //
    // Conservative rules (per rubber-duck critique):
    //   - Never move a box that already sits on its own letter goal — that
    //     would silently invalidate a previously delivered letter goal, and
    //     no redelivery pass runs after this method.
    //   - Avoid parking boxes ON agent-goal cells (would block another
    //     agent's final positioning).
    //   - Anti-thrash with a `tried` set keyed on (letter, from, to).
    //   - Bounded round budget; bail early if variant deadline is up.
    const int num_agents = static_cast<int>(state_.agent_rows.size());
    auto targets = agent_goal_targets();

    const auto agent_goals = agent_goal_cells();
    std::set<std::tuple<char, int, int, int, int>> tried;
    const int max_rounds = 4;

    for (int round = 0; round < max_rounds; ++round) {
        if (variant_time_up()) return;

        bool any_action = false;

        for (int a = 0; a < num_agents; ++a) {
            if (variant_time_up()) return;
            if (targets[a].first < 0) continue;

            const int ar = state_.agent_rows[a];
            const int ac = state_.agent_cols[a];
            const int tr = targets[a].first;
            const int tc = targets[a].second;
            if (ar == tr && ac == tc) continue;

            // Box-aware reachability: if agent can already reach its goal
            // under current box positions, no clearing is needed for this
            // agent.
            const auto box_dist = topology_.dist_box_aware(state_, tr, tc);
            if (box_dist[ar][ac] != kInf) continue;

            // Walls-only path from agent to its goal. Reconstructed from the
            // existing helper which BFSes from the goal cell and walks back.
            const auto path = path_box_to_goal_ignore_boxes(ar, ac, tr, tc);
            if (path.empty()) continue;

            // Forbidden parking cells for blockers cleared on this agent's
            // path: the path itself, the goal cell, and ALL OTHER agents'
            // numeric goal cells.
            std::set<std::pair<int, int>> forbidden(path.begin(), path.end());
            forbidden.insert({tr, tc});
            for (const auto& g : agent_goals) {
                if (g.first == tr && g.second == tc) continue;
                forbidden.insert(g);
            }

            for (const auto& cell : path) {
                const int r = cell.first;
                const int c = cell.second;
                const char box_letter = state_.boxes[r][c];
                if (box_letter == '\0') continue;
                if (r == ar && c == ac) continue;  // agent's own cell

                // Skip satisfied letter-goal boxes (don't break prior work).
                if (level_.goals[r][c] == box_letter) continue;

                const int color = level_.box_color[box_letter - 'A'];
                if (color < 0) continue;
                // Need at least one colour-compatible mover agent for this
                // box. pick_mover_for_box does the actual selection inside
                // try_relocate_box_recursive.
                bool has_mover = false;
                for (int m = 0; m < num_agents; ++m) {
                    if (level_.agent_color[m] == color) { has_mover = true; break; }
                }
                if (!has_mover) continue;

                const auto parks = find_parking_cells(box_letter, r, c, forbidden, 6);
                if (parks.empty()) continue;

                bool moved = false;
                for (const auto& park : parks) {
                    if (variant_time_up()) return;
                    const int pr = park.first;
                    const int pc = park.second;
                    const auto key = std::make_tuple(box_letter, r, c, pr, pc);
                    if (tried.count(key)) continue;
                    tried.insert(key);
                    const int mover_after = pick_mover_for_box(box_letter, r, c);
                    if (try_relocate_box_recursive(
                            box_letter, r, c, pr, pc, forbidden, 2)) {
                        if (mover_after >= 0)
                            (void)evict_agent_from_forbidden(mover_after, forbidden);
                        moved = true;
                        any_action = true;
                        break;
                    }
                }
                if (moved) break;  // re-scan blockers next round under new state
            }
        }

        if (!any_action) break;
    }
}

bool Solver::complete_agent_goals_serial()
{
    // Incremental serial agent walks: each round, try every agent that still
    // needs to reach its goal. Commit walks for those that succeed (they
    // open up new cells for stuck agents next round); defer the rest. Repeat
    // until either everybody is at goal or no progress was made (stuck).
    //
    // Other agents are treated as static obstacles by `plan_agent_to`, so a
    // successful walk for one agent may unblock another agent in a later
    // round (because the newly-arrived agent now sits at a different cell
    // that the stuck agent's path didn't need). The serial planner cannot
    // handle simultaneous swaps — those need PIBT or the higher-level
    // box-relocation passes wrapping this call.
    const int num_agents = static_cast<int>(state_.agent_rows.size());

    std::vector<std::pair<int, int>> goals(num_agents, {-1, -1});
    for (int agent = 0; agent < num_agents; ++agent) {
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (level_.goals[r][c] == static_cast<char>('0' + agent)) {
                    goals[agent] = {r, c};
                }
            }
        }
    }

    auto all_done = [&]() {
        for (int a = 0; a < num_agents; ++a) {
            if (goals[a].first < 0) continue;
            if (state_.agent_rows[a] != goals[a].first
                || state_.agent_cols[a] != goals[a].second) return false;
        }
        return true;
    };
    if (all_done()) return true;

    const int max_rounds = num_agents + 2;  // worst case each agent unblocks one
    for (int round = 0; round < max_rounds; ++round) {
        if (variant_time_up()) return all_done();
        bool any_progress = false;
        for (int agent = 0; agent < num_agents; ++agent) {
            const int gr = goals[agent].first;
            const int gc = goals[agent].second;
            if (gr < 0) continue;
            if (state_.agent_rows[agent] == gr
                && state_.agent_cols[agent] == gc) continue;
            auto seq = planner_.plan_agent_to(agent, gr, gc);
            if (seq.empty()) continue;
            const std::size_t snap_len = plan_.size();
            const State snap_state = state_;
            bool ok = true;
            for (int ai : seq) {
                std::vector<int> joint = noop_joint();
                joint[agent] = ai;
                if (!state_.apply_joint(joint)) { ok = false; break; }
                append_joint(joint);
            }
            if (!ok) {
                state_ = snap_state;
                plan_.resize(snap_len);
                continue;
            }
            any_progress = true;
        }
        if (all_done()) return true;
        if (!any_progress) break;
    }
    return all_done();
}

std::vector<std::pair<int, int>> Solver::path_box_to_goal_ignore_boxes(
    int br, int bc, int gr, int gc) const
{
    // BFS from goal cell over walls only (ignore boxes and agents entirely),
    // then reconstruct path from box to goal. This gives us the *intended*
    // route for the box if there were no obstacles; intersecting cells with
    // currently-occupied box cells gives us blocker candidates.
    const int R = level_.rows, C = level_.cols;
    std::vector<std::vector<int>> dist(R, std::vector<int>(C, kInf));
    std::vector<std::vector<int>> par_r(R, std::vector<int>(C, -1));
    std::vector<std::vector<int>> par_c(R, std::vector<int>(C, -1));
    if (gr < 0 || gr >= R || gc < 0 || gc >= C || level_.walls[gr][gc])
        return {};
    dist[gr][gc] = 0;
    std::deque<std::pair<int, int>> q;
    q.emplace_back(gr, gc);
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop_front();
        for (int k = 0; k < 4; ++k) {
            int nr = cr + dr[k], nc = cc + dc[k];
            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
            if (level_.walls[nr][nc]) continue;
            if (dist[nr][nc] != kInf) continue;
            dist[nr][nc] = dist[cr][cc] + 1;
            par_r[nr][nc] = cr; par_c[nr][nc] = cc;
            q.emplace_back(nr, nc);
        }
    }
    if (br < 0 || br >= R || bc < 0 || bc >= C || dist[br][bc] == kInf)
        return {};
    std::vector<std::pair<int, int>> path;
    int cr = br, cc = bc;
    while (!(cr == gr && cc == gc)) {
        path.emplace_back(cr, cc);
        int pr = par_r[cr][cc], pc = par_c[cr][cc];
        if (pr < 0) return {};
        cr = pr; cc = pc;
    }
    path.emplace_back(gr, gc);
    return path;
}

bool Solver::find_parking_cell(int blocker_r, int blocker_c,
                               const std::set<std::pair<int, int>>& forbidden,
                               int& out_r, int& out_c) const
{
    // BFS outward from blocker; first reachable empty cell that:
    //  - is not on the active task's box-to-goal path (forbidden)
    //  - is not occupied by any box or agent
    //  - is not the blocker's own cell
    //
    // Letter-goal cells ARE valid parking spots (they'll either get delivered
    // properly later, or — by happy accident — fulfil their own goal). This
    // is one of the cpp_enhanced relaxations that unlocks dense levels.
    const int R = level_.rows, C = level_.cols;
    std::vector<std::vector<int>> dist(R, std::vector<int>(C, kInf));
    if (blocker_r < 0 || blocker_r >= R || blocker_c < 0 || blocker_c >= C)
        return false;
    dist[blocker_r][blocker_c] = 0;
    std::deque<std::pair<int, int>> q;
    q.emplace_back(blocker_r, blocker_c);
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, -1, 1};
    auto cell_is_occupied = [&](int r, int c) {
        if (state_.boxes[r][c] != '\0' && !(r == blocker_r && c == blocker_c))
            return true;
        for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a)
            if (state_.agent_rows[a] == r && state_.agent_cols[a] == c)
                return true;
        return false;
    };
    while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop_front();
        if (!(cr == blocker_r && cc == blocker_c)
            && !forbidden.count({cr, cc})
            && !cell_is_occupied(cr, cc)) {
            out_r = cr; out_c = cc;
            return true;
        }
        for (int k = 0; k < 4; ++k) {
            int nr = cr + dr[k], nc = cc + dc[k];
            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
            if (level_.walls[nr][nc]) continue;
            if (dist[nr][nc] != kInf) continue;
            dist[nr][nc] = dist[cr][cc] + 1;
            q.emplace_back(nr, nc);
        }
    }
    return false;
}

std::vector<std::pair<int, int>> Solver::find_parking_cells(
    char box, int blocker_r, int blocker_c,
    const std::set<std::pair<int, int>>& forbidden,
    int limit)
{
    // Smarter multi-candidate parking. Mirrors cpp_enhanced::parking_cells:
    //   - BFS distances from the blocker (walls only)
    //   - reject walls / any goal cell / any box / any agent / forbidden
    //   - reject pull-aware dead cells for this specific box letter
    //   - score by distance × 4 - degree × 3 (closer + more open neighbours
    //     beats farther + tight passages)
    //   - return up to `limit` candidates sorted ascending by score
    const int R = level_.rows, C = level_.cols;
    std::vector<std::pair<int, int>> out;
    if (blocker_r < 0 || blocker_r >= R || blocker_c < 0 || blocker_c >= C) return out;

    const auto& dist = topology_.dist_walls_only(blocker_r, blocker_c);

    auto cell_occupied = [&](int r, int c) {
        if (state_.boxes[r][c] != '\0' && !(r == blocker_r && c == blocker_c))
            return true;
        for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a)
            if (state_.agent_rows[a] == r && state_.agent_cols[a] == c)
                return true;
        return false;
    };

    auto degree_of = [&](int r, int c) {
        int d = 0;
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        for (int k = 0; k < 4; ++k) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
            if (!level_.walls[nr][nc]) ++d;
        }
        return d;
    };

    struct Cand { int score; int r; int c; };
    std::vector<Cand> cands;
    cands.reserve(64);

    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            if (level_.walls[r][c]) continue;
            if (r == blocker_r && c == blocker_c) continue;
            if (dist[r][c] == kInf) continue;
            // Skip any goal cell (letter or digit) — parking here disrupts
            // either letter-goal satisfaction or final agent positioning.
            if (level_.goals[r][c] != '\0') continue;
            if (cell_occupied(r, c)) continue;
            if (forbidden.count({r, c})) continue;
            if (topology_.pull_aware_dead_cell(box, r, c)) continue;

            const int score = dist[r][c] * 4 - degree_of(r, c) * 3;
            cands.push_back({score, r, c});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
        if (a.score != b.score) return a.score < b.score;
        if (a.r != b.r) return a.r < b.r;
        return a.c < b.c;
    });
    for (const auto& c : cands) {
        out.emplace_back(c.r, c.c);
        if (static_cast<int>(out.size()) >= limit) break;
    }
    return out;
}

bool Solver::relocate_blocker(int blocker_agent, char blocker_letter,
                              int br, int bc, int pr, int pc)
{
    auto seq = planner_.plan(blocker_agent, blocker_letter, br, bc, pr, pc);
    if (seq.empty()) return false;
    const std::size_t snap_len = plan_.size();
    const State snap_state = state_;
    for (int ai : seq) {
        std::vector<int> joint = noop_joint();
        joint[blocker_agent] = ai;
        if (!state_.apply_joint(joint)) {
            state_ = snap_state;
            plan_.resize(snap_len);
            return false;
        }
        append_joint(joint);
    }
    return true;
}

int Solver::pick_mover_for_box(char box, int br, int bc) const
{
    if (box < 'A' || box > 'Z') return -1;
    const int color = level_.box_color[box - 'A'];
    if (color < 0) return -1;
    int best_agent = -1, best_d = kInf;
    for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
        if (level_.agent_color[a] != color) continue;
        const int d = manhattan(state_.agent_rows[a], state_.agent_cols[a], br, bc);
        if (d < best_d) { best_d = d; best_agent = a; }
    }
    return best_agent;
}

bool Solver::try_relocate_box_recursive(
    char box, int from_r, int from_c, int target_r, int target_c,
    const std::set<std::pair<int, int>>& forbidden, int depth)
{
    // Already at target.
    if (state_.boxes[target_r][target_c] == box) return true;
    // Box must actually be at its claimed start cell.
    if (state_.boxes[from_r][from_c] != box) return false;
    if (variant_time_up()) return false;

    const State saved_state = state_;
    const std::size_t saved_plan = plan_.size();

    auto attempt_direct = [&]() -> bool {
        const int agent = pick_mover_for_box(box, from_r, from_c);
        if (agent < 0) return false;
        return relocate_blocker(agent, box, from_r, from_c, target_r, target_c);
    };

    if (attempt_direct()) return true;
    if (depth <= 0) {
        state_ = saved_state;
        plan_.resize(saved_plan);
        return false;
    }

    // Build the walls-only paths we care about:
    //   path_box: the box's own route from (from_r,from_c) to (target).
    //   path_agent: the mover agent's route to reach the box.
    // Boxes lying on EITHER path are sub-blockers — chain-clear them.
    // (Without path_agent the recursive call misses cases like BStar's
    // D-blocks-agent2-from-reaching-E.)
    const auto path_box = path_box_to_goal_ignore_boxes(
        from_r, from_c, target_r, target_c);
    if (path_box.empty()) {
        state_ = saved_state;
        plan_.resize(saved_plan);
        return false;
    }

    const int mover = pick_mover_for_box(box, from_r, from_c);
    std::vector<std::pair<int, int>> path_agent;
    if (mover >= 0) {
        path_agent = path_box_to_goal_ignore_boxes(
            state_.agent_rows[mover], state_.agent_cols[mover], from_r, from_c);
    }

    std::set<std::pair<int, int>> combined = forbidden;
    combined.insert(path_box.begin(), path_box.end());
    combined.insert(path_agent.begin(), path_agent.end());
    combined.insert({target_r, target_c});
    combined.insert({from_r, from_c});

    auto collect_blockers = [&](const std::vector<std::pair<int, int>>& path,
                                std::vector<std::tuple<int, int, int, char>>& out)
    {
        for (std::size_t i = 0; i < path.size(); ++i) {
            const int r = path[i].first, c = path[i].second;
            const char ch = state_.boxes[r][c];
            if (ch == '\0') continue;
            if (r == from_r && c == from_c) continue;        // ourselves
            if (level_.goals[r][c] == ch) continue;          // skip on-goal
            // Skip same-letter that lives in path — recursive infinite loop risk.
            if (ch == box) continue;
            out.emplace_back(static_cast<int>(i), r, c, ch);
        }
    };

    std::vector<std::tuple<int, int, int, char>> sub_blockers;
    // Prefer agent-path blockers FIRST (these unblock the mover; without them
    // direct retry is hopeless even after clearing box-path blockers).
    collect_blockers(path_agent, sub_blockers);
    collect_blockers(path_box, sub_blockers);

    // Dedup by (r, c) while preserving first-seen order.
    {
        std::set<std::pair<int, int>> seen;
        std::vector<std::tuple<int, int, int, char>> uniq;
        for (const auto& sb : sub_blockers) {
            const auto key = std::make_pair(std::get<1>(sb), std::get<2>(sb));
            if (seen.insert(key).second) uniq.push_back(sb);
        }
        sub_blockers.swap(uniq);
    }

    const std::size_t sub_limit = std::min<std::size_t>(4, sub_blockers.size());
    for (std::size_t bi = 0; bi < sub_limit; ++bi) {
        if (variant_time_up()) break;
        const auto [_, sbr, sbc, sletter] = sub_blockers[bi];
        (void)_;
        const auto sub_parks = find_parking_cells(sletter, sbr, sbc, combined, 4);
        for (const auto& sp : sub_parks) {
            if (variant_time_up()) break;
            const State branch = state_;
            const std::size_t branch_plan = plan_.size();
            if (try_relocate_box_recursive(
                    sletter, sbr, sbc, sp.first, sp.second,
                    combined, depth - 1)
                && attempt_direct()) {
                return true;
            }
            state_ = branch;
            plan_.resize(branch_plan);
        }
    }

    state_ = saved_state;
    plan_.resize(saved_plan);
    return false;
}

bool Solver::scatter_agent_to(int agent, int target_r, int target_c)
{
    auto seq = planner_.plan_agent_to(agent, target_r, target_c);
    if (seq.empty()) return false;
    const std::size_t snap_len = plan_.size();
    const State snap_state = state_;
    for (int ai : seq) {
        std::vector<int> joint = noop_joint();
        joint[agent] = ai;
        if (!state_.apply_joint(joint)) {
            state_ = snap_state;
            plan_.resize(snap_len);
            return false;
        }
        append_joint(joint);
    }
    return true;
}

bool Solver::evict_agent_from_forbidden(
    int agent, const std::set<std::pair<int, int>>& forbidden)
{
    // If the given agent is sitting on a cell in `forbidden` (typically the
    // task's box-to-goal path or goal cell), BFS outward from the agent for
    // the closest unoccupied cell NOT in `forbidden` and walk the agent there.
    // No-op (returns true) if the agent is already off the forbidden set.
    const int sr = state_.agent_rows[agent];
    const int sc = state_.agent_cols[agent];
    if (!forbidden.count({sr, sc})) return true;

    const int R = level_.rows, C = level_.cols;
    std::vector<std::vector<int>> dist(R, std::vector<int>(C, kInf));
    dist[sr][sc] = 0;
    std::deque<std::pair<int, int>> q;
    q.emplace_back(sr, sc);
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, -1, 1};

    auto cell_occupied_by_other = [&](int r, int c) {
        if (state_.boxes[r][c] != '\0') return true;
        for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
            if (a == agent) continue;
            if (state_.agent_rows[a] == r && state_.agent_cols[a] == c) return true;
        }
        return false;
    };

    int target_r = -1, target_c = -1;
    while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop_front();
        if (!(cr == sr && cc == sc)
            && !forbidden.count({cr, cc})
            && !cell_occupied_by_other(cr, cc)) {
            target_r = cr;
            target_c = cc;
            break;
        }
        for (int k = 0; k < 4; ++k) {
            int nr = cr + dr[k], nc = cc + dc[k];
            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
            if (level_.walls[nr][nc]) continue;
            if (dist[nr][nc] != kInf) continue;
            dist[nr][nc] = dist[cr][cc] + 1;
            q.emplace_back(nr, nc);
        }
    }
    if (target_r < 0) return false;
    return scatter_agent_to(agent, target_r, target_c);
}

void Solver::add_path_radius(std::set<std::pair<int, int>>& cells,
                             const std::vector<std::pair<int, int>>& path,
                             int radius) const
{
    // Expand each path cell to a Manhattan-radius ball, clipped to bounds and
    // walls. Used by `evacuate_corridor_agents` so the corridor we clear is
    // wide enough that a single-box A* push past an off-path agent still has
    // room to swap.
    for (const auto& [cr, cc] : path) {
        for (int r = cr - radius; r <= cr + radius; ++r) {
            for (int c = cc - radius; c <= cc + radius; ++c) {
                if (r < 0 || r >= level_.rows || c < 0 || c >= level_.cols) continue;
                if (level_.walls[r][c]) continue;
                if (std::abs(r - cr) + std::abs(c - cc) > radius) continue;
                cells.insert({r, c});
            }
        }
    }
}

std::vector<std::pair<int, int>> Solver::evacuation_targets(
    int agent, const std::set<std::pair<int, int>>& forbidden)
{
    // Rank candidate parking cells for `agent`: walls-only-reachable, free,
    // non-forbidden. Lower score is better: distance×3 − degree×2 +
    // goal_penalty. The distance term favors short walks; the degree term
    // favors hub cells (so we don't dead-end agents); the goal_penalty
    // prevents accidentally parking on a level goal cell. Returns up to 12.
    const int R = level_.rows, C = level_.cols;
    const auto& dist = topology_.dist_walls_only(
        state_.agent_rows[agent], state_.agent_cols[agent]);
    struct Candidate { int score; int r; int c; };
    std::vector<Candidate> candidates;
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, 1, -1};
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            if (forbidden.count({r, c})) continue;
            if (dist[r][c] == kInf) continue;
            if (level_.walls[r][c]) continue;
            if (state_.boxes[r][c] != '\0') continue;
            bool occ_agent = false;
            for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
                if (a == agent) continue;
                if (state_.agent_rows[a] == r && state_.agent_cols[a] == c) {
                    occ_agent = true; break;
                }
            }
            if (occ_agent) continue;
            int degree = 0;
            for (int k = 0; k < 4; ++k) {
                const int nr = r + dr[k], nc = c + dc[k];
                if (0 <= nr && nr < R && 0 <= nc && nc < C && !level_.walls[nr][nc])
                    ++degree;
            }
            const int goal_penalty = level_.goals[r][c] == '\0' ? 0 : 25;
            candidates.push_back({dist[r][c] * 3 - degree * 2 + goal_penalty, r, c});
        }
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            if (a.score != b.score) return a.score < b.score;
            if (a.r != b.r) return a.r < b.r;
            return a.c < b.c;
        });
    std::vector<std::pair<int, int>> targets;
    for (const Candidate& cand : candidates) {
        targets.push_back({cand.r, cand.c});
        if (targets.size() >= 12) break;
    }
    return targets;
}

bool Solver::evacuate_corridor_agents(int active_agent,
                                      int box_r, int box_c,
                                      int goal_r, int goal_c)
{
    // Build a wide forbidden zone: radius-1 around the box-to-goal path AND
    // radius-1 around the active agent's path to reach the box. Any OTHER
    // agent currently in this zone must be evacuated; otherwise single-box
    // A* will get stuck mid-push when an adjacent agent blocks a needed
    // swap-step. Up to 2 passes lets agent B evacuate to a cell that agent
    // C just vacated. All-or-nothing: rolls back on any failure.
    const auto box_path = path_box_to_goal_ignore_boxes(
        box_r, box_c, goal_r, goal_c);
    if (box_path.empty()) return false;
    const auto agent_path = path_box_to_goal_ignore_boxes(
        state_.agent_rows[active_agent],
        state_.agent_cols[active_agent],
        box_r, box_c);

    std::set<std::pair<int, int>> forbidden;
    add_path_radius(forbidden, box_path, 1);
    if (!agent_path.empty()) add_path_radius(forbidden, agent_path, 1);
    forbidden.insert({box_r, box_c});
    forbidden.insert({goal_r, goal_c});

    const State snap_state = state_;
    const std::size_t snap_plan = plan_.size();

    bool moved_any = false;
    const int num_agents = static_cast<int>(state_.agent_rows.size());
    for (int pass = 0; pass < 2; ++pass) {
        bool moved_this_pass = false;
        for (int a = 0; a < num_agents; ++a) {
            if (a == active_agent) continue;
            const int ar = state_.agent_rows[a];
            const int ac = state_.agent_cols[a];
            if (!forbidden.count({ar, ac})) continue;
            bool moved = false;
            for (const auto& [tr, tc] : evacuation_targets(a, forbidden)) {
                if (scatter_agent_to(a, tr, tc)) {
                    moved = true;
                    moved_any = true;
                    moved_this_pass = true;
                    forbidden.insert({tr, tc});
                    break;
                }
            }
            if (!moved) {
                state_ = snap_state;
                plan_.resize(snap_plan);
                return false;
            }
        }
        if (!moved_this_pass) break;
    }
    return moved_any;
}

bool Solver::deliver_task_with_scatter(Task& task)
{
    // Compute the walls-only path from box to goal. Any OTHER agent currently
    // sitting on that path blocks single-box A* (which treats other agents as
    // immovable obstacles). Try to scatter each such agent to a free cell off
    // the path, then retry plain delivery. Roll back ALL scatter moves on
    // delivery failure so we don't leave the world in a worse state.
    const auto path = path_box_to_goal_ignore_boxes(
        task.box_row, task.box_col, task.goal_row, task.goal_col);
    if (path.empty()) return false;

    std::set<std::pair<int, int>> forbidden(path.begin(), path.end());
    forbidden.insert({task.goal_row, task.goal_col});

    // Collect blocking agents on the path.
    std::vector<int> blockers;
    for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
        if (a == task.agent) continue;
        const int r = state_.agent_rows[a], c = state_.agent_cols[a];
        if (forbidden.count({r, c})) blockers.push_back(a);
    }
    if (blockers.empty()) return false;

    const std::size_t outer_snap_len = plan_.size();
    const State outer_snap_state = state_;

    for (int agent : blockers) {
        int sr = state_.agent_rows[agent], sc = state_.agent_cols[agent];
        // BFS outward from the blocker for a non-forbidden floor cell.
        const int R = level_.rows, C = level_.cols;
        std::vector<std::vector<int>> dist(R, std::vector<int>(C, kInf));
        dist[sr][sc] = 0;
        std::deque<std::pair<int, int>> q;
        q.emplace_back(sr, sc);
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        int tr = -1, tc = -1;
        while (!q.empty()) {
            auto [cr, cc] = q.front();
            q.pop_front();
            const bool occupied_box = state_.boxes[cr][cc] != '\0';
            bool occupied_agent = false;
            for (int oa = 0; oa < static_cast<int>(state_.agent_rows.size()); ++oa) {
                if (oa == agent) continue;
                if (state_.agent_rows[oa] == cr && state_.agent_cols[oa] == cc) {
                    occupied_agent = true; break;
                }
            }
            if (!(cr == sr && cc == sc)
                && !forbidden.count({cr, cc})
                && !occupied_box
                && !occupied_agent) {
                tr = cr; tc = cc;
                break;
            }
            for (int k = 0; k < 4; ++k) {
                int nr = cr + dr[k], nc = cc + dc[k];
                if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                if (level_.walls[nr][nc]) continue;
                if (dist[nr][nc] != kInf) continue;
                dist[nr][nc] = dist[cr][cc] + 1;
                q.emplace_back(nr, nc);
            }
        }
        if (tr < 0) {
            state_ = outer_snap_state;
            plan_.resize(outer_snap_len);
            return false;
        }
        if (!scatter_agent_to(agent, tr, tc)) {
            state_ = outer_snap_state;
            plan_.resize(outer_snap_len);
            return false;
        }
        // Reserve so subsequent blockers don't pick the same cell.
        forbidden.insert({tr, tc});
    }

    if (!deliver_task(task)) {
        state_ = outer_snap_state;
        plan_.resize(outer_snap_len);
        return false;
    }
    return true;
}

bool Solver::deliver_task_with_relocation(Task& task, bool allow_on_goal_blockers)
{
    // Compute the rough path (walls-only BFS) from the active box to its goal.
    // Boxes lying on this path that are NOT goal-occupying same-letter
    // matches are blocker candidates. Try to relocate the closest blocker
    // (to the active box) to a free parking cell, then retry delivery.
    //
    // If `allow_on_goal_blockers` is true, also relocate boxes that are
    // currently sitting on their own goal cell (last-resort: caller is
    // responsible for re-delivering them afterwards via the redelivery scan
    // in `solve_once`).
    const int snapshot_attempts = 6;
    std::set<std::tuple<char, int, int, int, int>> tried;
    const int num_agents = static_cast<int>(state_.agent_rows.size());

    for (int round = 0; round < snapshot_attempts; ++round) {
        const auto path = path_box_to_goal_ignore_boxes(
            task.box_row, task.box_col, task.goal_row, task.goal_col);
        if (path.empty()) return false;
        std::set<std::pair<int, int>> forbidden(path.begin(), path.end());
        forbidden.insert({task.goal_row, task.goal_col});

        // Pre-step: any AGENT sitting on the path is a wall to single-box A*.
        // Evict each such agent to the closest non-forbidden free cell. If
        // eviction succeeds for at least one, retry deliver_task before
        // touching boxes — many failed deliveries are agent-on-path, not
        // box-on-path, especially in dense multi-agent levels (Planarchy,
        // TriSplit, escAIpe). Skip the task's own agent (it WILL want to be
        // on the path).
        bool any_evicted = false;
        for (int a = 0; a < num_agents; ++a) {
            if (a == task.agent) continue;
            const int ar = state_.agent_rows[a];
            const int ac = state_.agent_cols[a];
            if (!forbidden.count({ar, ac})) continue;
            if (evict_agent_from_forbidden(a, forbidden)) any_evicted = true;
        }
        if (any_evicted && deliver_task(task)) return true;

        // Find blockers along the path: cells occupied by other boxes.
        std::vector<std::tuple<int, int, int, char>> blockers;  // (dist, r, c, letter)
        for (std::size_t i = 0; i < path.size(); ++i) {
            const auto [r, c] = path[i];
            const char ch = state_.boxes[r][c];
            if (ch == '\0') continue;
            if (r == task.box_row && c == task.box_col) continue;  // active box
            // By default, don't move boxes already on their own goal cell.
            if (!allow_on_goal_blockers && level_.goals[r][c] == ch) continue;
            blockers.emplace_back(static_cast<int>(i), r, c, ch);
        }
        if (blockers.empty()) {
            // No box blockers and either eviction didn't help or no agents
            // were on the path. Nothing more we can do this round.
            return false;
        }
        std::sort(blockers.begin(), blockers.end());

        bool progressed = false;
        const std::size_t blocker_limit = std::min<std::size_t>(6, blockers.size());
        for (std::size_t bi = 0; bi < blocker_limit && !progressed; ++bi) {
            const auto [_, br, bc, letter] = blockers[bi];
            (void)_;

            // Multi-candidate parking + recursive chain-clearing. Mirrors
            // cpp_enhanced::relocate_blocker_and_deliver's inner loop.
            const auto parks = find_parking_cells(letter, br, bc, forbidden, 6);
            for (const auto& park : parks) {
                const int pr = park.first;
                const int pc = park.second;
                const auto key = std::make_tuple(letter, br, bc, pr, pc);
                if (tried.count(key)) continue;
                tried.insert(key);

                if (!try_relocate_box_recursive(
                        letter, br, bc, pr, pc, forbidden, 1)) continue;

                // Evict the mover off any forbidden cell so single-box A*
                // can use the cleared path during the next deliver_task.
                const int mover_after = pick_mover_for_box(letter, pr, pc);
                if (mover_after >= 0)
                    (void)evict_agent_from_forbidden(mover_after, forbidden);
                progressed = true;
                break;
            }
        }
        if (!progressed) return false;
        if (deliver_task(task)) return true;
    }
    return false;
}

bool Solver::solve_once(std::vector<Task> tasks)
{
    // Per-variant wall-clock budget. Generous enough to absorb relocation +
    // redelivery work, tight enough that one slow variant can't starve the
    // others. Tuned around the empirical worst-case of MAze (2.4s) +
    // Tittling-class outliers.
    constexpr double kVariantBudgetSeconds = 12.0;
    variant_deadline_ = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(
            static_cast<int>(kVariantBudgetSeconds * 1000));

    if (tasks.empty()) {
        // Levels with zero box tasks still need the final agent-goal phase.
        if (!complete_agent_goals()) return false;
        return state_.goal_state();
    }

    std::deque<Task> queue(tasks.begin(), tasks.end());
    const int max_passes = 3;
    int consecutive_failures = 0;
    int total_attempts = 0;
    const int attempt_cap =
        static_cast<int>(tasks.size()) * (max_passes + 1) + 16;
    const int redelivery_rounds_cap = 3;
    int redelivery_rounds = 0;

    auto run_queue = [&]() -> bool {
        while (!queue.empty() && total_attempts < attempt_cap) {
            if (variant_time_up()) return false;
            Task t = queue.front();
            queue.pop_front();
            ++total_attempts;

            if (deliver_task(t)) {
                consecutive_failures = 0;
                continue;
            }
            // Eager scatter: cheap to attempt and rolls back fully on failure.
            // Many fast-failing tasks just need a single agent off the corridor.
            if (deliver_task_with_scatter(t)) {
                consecutive_failures = 0;
                continue;
            }
            // Corridor evacuation: wider radius-1 buffer around BOTH the
            // box path AND the agent-to-box path. Evicts off-path agents
            // that adjoin the corridor and would otherwise deadlock the
            // single-box push (e.g., CphAirprt-class long corridors).
            {
                const State snap = state_;
                const std::size_t snap_len = plan_.size();
                if (evacuate_corridor_agents(t.agent, t.box_row, t.box_col,
                                             t.goal_row, t.goal_col)
                    && deliver_task(t)) {
                    consecutive_failures = 0;
                    continue;
                }
                state_ = snap;
                plan_.resize(snap_len);
            }

            ++consecutive_failures;
            if (consecutive_failures > static_cast<int>(queue.size()) + 1) {
                // defer-and-retry exhausted: try blocker relocation (move other
                // boxes off the path) before giving up.
                if (deliver_task_with_relocation(t, /*allow_on_goal_blockers=*/false)) {
                    consecutive_failures = 0;
                    continue;
                }
                // Last resort: allow displacing on-goal blockers (they will
                // be re-queued by the redelivery scan after this loop).
                if (deliver_task_with_relocation(t, /*allow_on_goal_blockers=*/true)) {
                    consecutive_failures = 0;
                    continue;
                }
                return false;
            }
            queue.push_back(t);
        }
        return queue.empty();
    };

    if (!run_queue()) return false;

    // Redelivery scan: aggressive relocation may have displaced previously
    // delivered (or initially-on-goal) boxes. Scan letter-goal cells; for any
    // goal cell whose letter is missing, find a same-letter box elsewhere and
    // queue a redelivery task. Loop until stable or budget exhausted.
    while (redelivery_rounds < redelivery_rounds_cap) {
        if (variant_time_up()) return false;
        std::vector<Task> redo;
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                const char gch = level_.goals[r][c];
                if (gch < 'A' || gch > 'Z') continue;
                if (state_.boxes[r][c] == gch) continue;  // already satisfied
                // Find the closest same-letter box not on its own goal.
                int best_r = -1, best_c = -1, best_d = kInf;
                for (int rr = 0; rr < level_.rows; ++rr) {
                    for (int cc = 0; cc < level_.cols; ++cc) {
                        if (state_.boxes[rr][cc] != gch) continue;
                        if (level_.goals[rr][cc] == gch) continue;
                        const int d = manhattan(rr, cc, r, c);
                        if (d < best_d) {
                            best_d = d;
                            best_r = rr;
                            best_c = cc;
                        }
                    }
                }
                if (best_r < 0) continue;
                const int color = level_.box_color[gch - 'A'];
                int agent = -1, ad = kInf;
                for (int a = 0; a < static_cast<int>(state_.agent_rows.size()); ++a) {
                    if (level_.agent_color[a] != color) continue;
                    const int d = manhattan(state_.agent_rows[a],
                                            state_.agent_cols[a], best_r, best_c);
                    if (d < ad) { ad = d; agent = a; }
                }
                if (agent < 0) continue;
                Task t;
                t.goal_row = r; t.goal_col = c;
                t.letter   = gch;
                t.box_row  = best_r; t.box_col = best_c;
                t.agent    = agent;
                redo.push_back(t);
            }
        }
        if (redo.empty()) break;
        for (auto& t : redo) queue.push_back(t);
        consecutive_failures = 0;
        ++redelivery_rounds;
        if (!run_queue()) return false;
    }

    if (!complete_agent_goals()) return false;
    return state_.goal_state();
}

std::vector<std::vector<int>> Solver::solve()
{
    auto finalize = [&](std::vector<std::vector<int>> plan)
        -> std::vector<std::vector<int>> {
        if (plan.empty()) return plan;
        // Post-processing: greedy interleaving of independent per-agent
        // actions so the server (and the GUI) executes them in parallel.
        // Safe by construction — falls back to `plan` if verification fails.
        return compact_plan(plan, initial_state_);
    };

    auto variants = build_task_variants();
    if (variants.empty()) {
        // Either no box tasks at all (pure agent-positioning level), or no
        // feasible matching. Try one empty pass for the agent-only case.
        state_ = initial_state_;
        plan_.clear();
        if (solve_once({})) return finalize(plan_);
        return {};
    }

    for (std::size_t i = 0; i < variants.size(); ++i) {
        state_ = initial_state_;
        plan_.clear();
        if (solve_once(variants[i])) return finalize(plan_);
    }

    return {};
}

}  // namespace aimas
