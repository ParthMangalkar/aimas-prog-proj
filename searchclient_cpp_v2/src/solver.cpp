#include "aimas/solver.hpp"
#include "aimas/compact.hpp"
#include "aimas/pibt.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>

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

std::vector<Solver::Task> Solver::build_tasks_matched_dp_minmax()
{
    // Like build_tasks_matched_dp() but minimises the MAXIMUM walls-only
    // distance instead of the SUM. Helps when the sum-min assignment picks
    // a layout where one box is trapped behind another (each individually
    // short, but together infeasible). The max-min variant prefers
    // balanced assignments that often unlock alternative delivery orders.
    constexpr int kMaxBoxesDp = 12;
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
        // Bail to greedy for too-many boxes or under-supplied (less common).
        if (static_cast<int>(boxes.size()) > kMaxBoxesDp
            || goals.size() > boxes.size()) continue;

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
                    const int cost = std::max(dp[mask], d);  // min-max objective
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

std::vector<std::vector<Solver::Task>> Solver::build_extra_variants(
    const std::vector<std::vector<Task>>& already_tried)
{
    // Fallback variants generated only after the primary list has failed.
    // Two sources:
    //   1. min-max DP assignment (per-letter), tried under all 5 sort modes.
    //   2. Deterministic random permutations of letter-groups inside the
    //      DP-min-sum base, generated with a fixed seed for reproducibility.
    // Deduped against `already_tried` using the same task-signature as
    // build_task_variants() so we never repeat work.
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
    for (const auto& v : already_tried) seen.insert(signature(v));
    auto add = [&](std::vector<Task> v) {
        if (v.empty()) return;
        const std::string s = signature(v);
        if (seen.insert(s).second) variants.push_back(std::move(v));
    };

    // Pass B: min-max DP × 5 sort modes.
    {
        auto base = build_tasks_matched_dp_minmax();
        if (!base.empty()) {
            for (int f = 0; f < 5; ++f) {
                auto v = base;
                sort_tasks(v, f);
                add(std::move(v));
            }
        }
    }

    // Pass A: deterministic random shuffles of LETTER-GROUPS inside the DP
    // min-sum base. Preserves per-letter grouping so multi-box-per-letter
    // assignments stay coherent. K=8 attempts keeps wall time bounded.
    {
        auto base = build_tasks_matched_dp();
        if (base.empty()) base = build_tasks_matched(0);
        if (!base.empty()) {
            std::map<char, std::vector<Task>> by_letter;
            std::vector<char> letters;
            for (const auto& t : base) {
                if (!by_letter.count(t.letter)) letters.push_back(t.letter);
                by_letter[t.letter].push_back(t);
            }
            std::mt19937 rng(0xC0FFEE);
            for (int k = 0; k < 8; ++k) {
                std::vector<char> order = letters;
                std::shuffle(order.begin(), order.end(), rng);
                std::vector<Task> v;
                v.reserve(base.size());
                for (char L : order) {
                    for (auto& t : by_letter[L]) v.push_back(t);
                }
                add(std::move(v));
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

bool Solver::overall_time_up() const
{
    return std::chrono::steady_clock::now() >= overall_deadline_;
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
    if (env_flag_enabled("V2_VERBOSE")) {
        std::cerr << "[v2:cag] ENTRY plan_len=" << plan_.size() << " agents:";
        for (int a = 0; a < (int)state_.agent_rows.size(); ++a)
            std::cerr << " a" << a << "=(" << state_.agent_rows[a] << "," << state_.agent_cols[a] << ")";
        std::cerr << " boxes:";
        for (int r = 0; r < level_.rows; ++r)
            for (int c = 0; c < level_.cols; ++c)
                if (state_.boxes[r][c] != '\0')
                    std::cerr << " " << state_.boxes[r][c] << "(" << r << "," << c << ")";
        std::cerr << "\n";
    }
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
    const bool _cag_v = env_flag_enabled("V2_VERBOSE");
    auto try_attempt = [&](const char* nm, auto&& fn) -> bool {
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        bool ok = fn();
        if (_cag_v) std::cerr << "[v2:cag]   try " << nm << " => " << (ok ? "OK" : "fail") << " plan_len=" << plan_.size() << "\n";
        if (ok) return true;
        state_ = snap_state;
        plan_.resize(snap_len);
        return false;
    };

    if (try_attempt("pibt", [&]{ return complete_agent_goals_pibt(); })) return true;
    if (try_attempt("reserved", [&]{ return complete_agent_goals_reserved(); })) return true;
    if (try_attempt("serial", [&]{ return complete_agent_goals_serial(); })) return true;
    for (int round = 0; round < 4; ++round) {
        if (variant_time_up()) break;
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        const bool moved = evacuate_final_goal_agent_blockers();
        if (_cag_v) std::cerr << "[v2:cag]   evac r" << round << " moved=" << moved << "\n";
        if (!moved) {
            state_ = snap_state;
            plan_.resize(snap_len);
            break;
        }
        if (try_attempt("pibt", [&]{ return complete_agent_goals_pibt(); })) return true;
        if (try_attempt("reserved", [&]{ return complete_agent_goals_reserved(); })) return true;
        if (try_attempt("serial", [&]{ return complete_agent_goals_serial(); })) return true;
    }

    // Layer 2: snapshot before clearing box blockers so we can roll back if
    // both follow-ups fail.
    const std::size_t pre_clear_len = plan_.size();
    const State pre_clear_state = state_;
    clear_paths_to_agent_goals();
    if (_cag_v) std::cerr << "[v2:cag]   after clear_paths plan_len=" << plan_.size() << "\n";

    if (try_attempt("pibt(post-clear)", [&]{ return complete_agent_goals_pibt(); })) return true;
    if (try_attempt("reserved(post-clear)", [&]{ return complete_agent_goals_reserved(); })) return true;
    if (try_attempt("serial(post-clear)", [&]{ return complete_agent_goals_serial(); })) return true;

    // One more agent-eviction pass after box clearing (box clearing may have
    // shifted agent positions onto goal cells).
    for (int round = 0; round < 2; ++round) {
        if (variant_time_up()) break;
        const std::size_t snap_len = plan_.size();
        const State snap_state = state_;
        const bool moved = evacuate_final_goal_agent_blockers();
        if (_cag_v) std::cerr << "[v2:cag]   post-clear evac r" << round << " moved=" << moved << "\n";
        if (!moved) {
            state_ = snap_state;
            plan_.resize(snap_len);
            break;
        }
        if (try_attempt("pibt(post-evac)", [&]{ return complete_agent_goals_pibt(); })) return true;
        if (try_attempt("reserved(post-evac)", [&]{ return complete_agent_goals_reserved(); })) return true;
        if (try_attempt("serial(post-evac)", [&]{ return complete_agent_goals_serial(); })) return true;
    }

    // Last resort: joint A* over agent positions only. Only fires for ≤6
    // agents (branching factor 5^N) and is guarded by a 200k expansion
    // cap + the variant deadline. Targets tight rotation/swap puzzles
    // where PIBT, cooperative A* and serial all give up. On larger
    // levels (N > 6) the method returns false immediately.
    if (try_attempt("cagj", [&]{ return complete_agent_goals_joint(); })) return true;

    // Active-agent-reduced variant: same algorithm but only agents whose
    // numeric goal is unsatisfied actually search; the rest emit NoOp. This
    // lifts the N ≤ 6 cap to "N_active ≤ 6" and rescues CphAirprt-class
    // situations where there are many agents on the board but only a
    // handful still need to reach their goal cell. Static inactive agents
    // sit in place — sound, but they may block the active set; that is
    // acceptable as a fallback (no regression on N ≤ 6 because the legacy
    // path runs first).
    if (static_cast<int>(state_.agent_rows.size()) > 6) {
        if (try_attempt("cagj-reduced", [&]{ return complete_agent_goals_joint(/*active=*/true); })) return true;
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

bool Solver::complete_agent_goals_joint(bool active_agent_reduction)
{
    // Joint A* over (agent positions only) treating walls + current boxes as
    // static obstacles. Targets tight rotation/swap puzzles that PIBT,
    // cooperative A*, and serial BFS all give up on (Apdo-style, where the
    // agents-on-each-others'-goal-cell case requires real cooperative motion
    // and the box layout is fixed). Branching factor is 5^N (4 Moves +
    // NoOp per agent); capped to N ≤ 6 in the legacy path so 5^6 = 15625
    // successors per node remains tractable for short residuals. With
    // active_agent_reduction=true, inactive agents (those already on their
    // numeric goal, or with no goal) only emit NoOp, so effective branching
    // becomes 5^N_active — allowing larger boards (up to 16 agents) as long
    // as only a few still need to move.
    // Other safeties: 200k expansion cap, per-variant deadline check, full
    // state/plan rollback on any failure.
    const int N = static_cast<int>(state_.agent_rows.size());
    constexpr int kAbsMaxN = 16;
    const bool jv = env_flag_enabled("V2_VERBOSE");
    if (N < 1 || N > kAbsMaxN) return false;
    if (!active_agent_reduction && N > 6) {
        if (jv) std::cerr << "[v2:cagj] reject N=" << N << " (no active reduction)\n";
        return false;
    }

    const auto targets = agent_goal_targets();

    auto all_done = [&]() {
        for (int a = 0; a < N; ++a) {
            if (targets[a].first < 0) continue;
            if (state_.agent_rows[a] != targets[a].first
                || state_.agent_cols[a] != targets[a].second) return false;
        }
        return true;
    };
    if (all_done()) return true;

    // Active mask: agent is active iff it has a numeric goal AND is not yet
    // sitting on it. Inactive agents will only emit NoOp; this is sound
    // because the goal predicate (is_goal below) treats inactive agents as
    // "already done" via the same targets[] check.
    std::vector<bool> active(N, true);
    int n_active = N;
    if (active_agent_reduction) {
        n_active = 0;
        for (int a = 0; a < N; ++a) {
            const auto& t = targets[a];
            const bool has_goal = (t.first >= 0);
            const bool at_goal  = has_goal
                && state_.agent_rows[a] == t.first
                && state_.agent_cols[a] == t.second;
            active[a] = has_goal && !at_goal;
            if (active[a]) ++n_active;
        }
        // Blocker promotion: in tight corridors (CphAirprt-class), inactive
        // agents may sit on the only shortest path between an active agent
        // and its goal. Without promotion the joint search treats them as
        // immovable walls and falsely concludes infeasibility. We BFS each
        // active agent's shortest path over walls+boxes only (ignoring
        // agents) and mark any inactive agent currently on such a path as
        // active. This rescues rotation/swap puzzles by including the
        // necessary blockers in the joint search.
        if (n_active >= 1 && n_active <= 5) {
            const int R = level_.rows;
            const int C = level_.cols;
            std::vector<std::vector<bool>> static_obstacle(R, std::vector<bool>(C, false));
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    if (level_.walls[r][c]) static_obstacle[r][c] = true;
                    else if (state_.boxes[r][c] != '\0') static_obstacle[r][c] = true;
                }
            }
            std::vector<std::pair<int, int>> agent_cells(N);
            for (int a = 0; a < N; ++a) {
                agent_cells[a] = {state_.agent_rows[a], state_.agent_cols[a]};
            }
            for (int aa = 0; aa < N && n_active <= 6; ++aa) {
                if (!active[aa]) continue;
                const int sr = state_.agent_rows[aa];
                const int sc = state_.agent_cols[aa];
                const int gr = targets[aa].first;
                const int gc = targets[aa].second;
                if (gr < 0) continue;
                if (sr == gr && sc == gc) continue;
                // BFS from start, distances. Then backtrack from goal.
                std::vector<std::vector<int>> dist(R, std::vector<int>(C, -1));
                std::deque<std::pair<int, int>> q;
                dist[sr][sc] = 0;
                q.emplace_back(sr, sc);
                static const int ddr[4] = {-1, 1, 0, 0};
                static const int ddc[4] = {0, 0, -1, 1};
                while (!q.empty() && dist[gr][gc] < 0) {
                    auto [cr, cc] = q.front();
                    q.pop_front();
                    for (int k = 0; k < 4; ++k) {
                        const int nr = cr + ddr[k];
                        const int nc = cc + ddc[k];
                        if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                        if (static_obstacle[nr][nc]) continue;
                        if (dist[nr][nc] >= 0) continue;
                        dist[nr][nc] = dist[cr][cc] + 1;
                        q.emplace_back(nr, nc);
                    }
                }
                if (dist[gr][gc] < 0) continue;
                // Backtrack one shortest path from goal -> start.
                int cr = gr, cc = gc;
                while (!(cr == sr && cc == sc)) {
                    for (int b = 0; b < N && n_active <= 6; ++b) {
                        if (!active[b] && agent_cells[b].first == cr
                            && agent_cells[b].second == cc) {
                            active[b] = true;
                            ++n_active;
                        }
                    }
                    bool stepped = false;
                    for (int k = 0; k < 4; ++k) {
                        const int nr = cr + ddr[k];
                        const int nc = cc + ddc[k];
                        if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                        if (dist[nr][nc] == dist[cr][cc] - 1) {
                            cr = nr; cc = nc;
                            stepped = true;
                            break;
                        }
                    }
                    if (!stepped) break;  // disconnected — bail
                }
            }
        }
        if (jv) std::cerr << "[v2:cagj] REDUCED N=" << N
                          << " N_active=" << n_active << "\n";
        // Effective branching guard: cap N_active so 5^N_active stays
        // tractable. 5^6 = 15625 successors per node is the practical edge.
        if (n_active < 1 || n_active > 6) {
            if (jv) std::cerr << "[v2:cagj] reject N_active=" << n_active << "\n";
            return false;
        }
    }

    const int R = level_.rows;
    const int C = level_.cols;

    // Static obstacle = walls ∪ current box cells. Agents are tracked in
    // the search state and excluded from the static map.
    std::vector<std::vector<bool>> obstacle(R, std::vector<bool>(C, false));
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            if (level_.walls[r][c]) obstacle[r][c] = true;
            else if (state_.boxes[r][c] != '\0') obstacle[r][c] = true;
        }
    }

    // Admissible heuristic per agent: BFS distance to its target over the
    // (walls + boxes)-restricted grid. Sum of per-agent distances is still
    // a lower bound on the joint solution length (each agent must traverse
    // at least its individual distance).
    std::vector<std::vector<std::vector<int>>> dist_to_target(N);
    for (int a = 0; a < N; ++a) {
        dist_to_target[a].assign(R, std::vector<int>(C, kInf));
        if (targets[a].first < 0) continue;
        const int gr = targets[a].first;
        const int gc = targets[a].second;
        if (gr < 0 || gr >= R || gc < 0 || gc >= C || obstacle[gr][gc]) {
            // In reduced mode, inactive agents are already at goal so their
            // target is not actually blocked. Only fail for active agents.
            if (active_agent_reduction && !active[a]) continue;
            if (jv) std::cerr << "[v2:cagj] reject: agent " << a
                              << " target blocked or oob (" << gr << "," << gc << ")\n";
            return false;  // agent's own goal is unreachable
        }
        dist_to_target[a][gr][gc] = 0;
        std::deque<std::pair<int, int>> q;
        q.emplace_back(gr, gc);
        static const int ddr[4] = {-1, 1, 0, 0};
        static const int ddc[4] = {0, 0, -1, 1};
        while (!q.empty()) {
            auto [cr, cc] = q.front();
            q.pop_front();
            for (int k = 0; k < 4; ++k) {
                const int nr = cr + ddr[k];
                const int nc = cc + ddc[k];
                if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                if (obstacle[nr][nc]) continue;
                if (dist_to_target[a][nr][nc] != kInf) continue;
                dist_to_target[a][nr][nc] = dist_to_target[a][cr][cc] + 1;
                q.emplace_back(nr, nc);
            }
        }
        const int sr = state_.agent_rows[a];
        const int sc = state_.agent_cols[a];
        if (dist_to_target[a][sr][sc] == kInf) {
            if (active_agent_reduction && !active[a]) continue;
            if (jv) std::cerr << "[v2:cagj] reject: agent " << a
                              << " unreachable to (" << gr << "," << gc
                              << ") from (" << sr << "," << sc << ")\n";
            return false;
        }
    }

    auto pos_of = [&](int r, int c) { return r * C + c; };

    struct JNode {
        std::vector<int> pos;     // agent cell ids
        std::vector<int> joint;   // joint action that produced this state
        int parent = -1;
        int g = 0;
        int f = 0;
    };
    std::vector<JNode> nodes;
    nodes.reserve(8192);

    auto encode = [&](const std::vector<int>& pos) {
        std::string s;
        s.reserve(pos.size() * sizeof(int));
        for (int p : pos) {
            s.push_back(static_cast<char>((p >> 24) & 0xff));
            s.push_back(static_cast<char>((p >> 16) & 0xff));
            s.push_back(static_cast<char>((p >> 8) & 0xff));
            s.push_back(static_cast<char>(p & 0xff));
        }
        return s;
    };

    auto h_of = [&](const std::vector<int>& pos) -> int {
        int h = 0;
        for (int a = 0; a < N; ++a) {
            if (targets[a].first < 0) continue;
            if (active_agent_reduction && !active[a]) continue;
            const int r = pos[a] / C, c = pos[a] % C;
            const int d = dist_to_target[a][r][c];
            if (d == kInf) return kInf;
            h += d;
        }
        return h;
    };

    auto is_goal = [&](const std::vector<int>& pos) {
        for (int a = 0; a < N; ++a) {
            if (targets[a].first < 0) continue;
            // Inactive agent already at goal (verified in mask computation);
            // still cheap to assert.
            if (pos[a] != pos_of(targets[a].first, targets[a].second)) return false;
        }
        return true;
    };

    std::vector<int> start_pos(N);
    for (int a = 0; a < N; ++a) {
        start_pos[a] = pos_of(state_.agent_rows[a], state_.agent_cols[a]);
    }
    const int h0 = h_of(start_pos);
    if (h0 == kInf) {
        if (jv) std::cerr << "[v2:cagj] reject: h0=inf\n";
        return false;
    }
    if (jv) std::cerr << "[v2:cagj] start search h0=" << h0 << "\n";
    const int w_h0 = active_agent_reduction ? (5 * h0) : h0;
    nodes.push_back({start_pos, std::vector<int>(N, 0), -1, 0, w_h0});

    auto cmp = [&nodes](int lhs, int rhs) {
        if (nodes[lhs].f != nodes[rhs].f) return nodes[lhs].f > nodes[rhs].f;
        return nodes[lhs].g < nodes[rhs].g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
    open.push(0);
    std::unordered_map<std::string, int> best_g;
    best_g[encode(start_pos)] = 0;

    // Per-agent action options: NoOp + 4 Moves.
    // Indices into actions(): 0 = NoOp, 1..4 = Move(N/S/E/W).
    static const int kAct[5] = {0, 1, 2, 3, 4};
    static const int kDr[5]  = {0, -1, 1, 0, 0};
    static const int kDc[5]  = {0, 0, 0, 1, -1};

    constexpr int kExpansionCap = 200000;
    // Reduced mode: higher cap because branching is bounded by 5^N_active.
    const int expansion_cap = active_agent_reduction ? 600000 : kExpansionCap;
    int expansions = 0;

    while (!open.empty() && expansions < expansion_cap) {
        if ((expansions & 1023) == 0 && variant_time_up()) {
            if (jv) std::cerr << "[v2:cagj] time up at expansions=" << expansions << "\n";
            return false;
        }
        const int idx = open.top();
        open.pop();
        // Copy: nodes may reallocate inside the successor loop.
        const JNode cur = nodes[idx];
        {
            const auto bg = best_g.find(encode(cur.pos));
            if (bg != best_g.end() && bg->second < cur.g) continue;
        }
        if (is_goal(cur.pos)) {
            std::vector<std::vector<int>> joints;
            int cursor = idx;
            while (nodes[cursor].parent != -1) {
                joints.push_back(nodes[cursor].joint);
                cursor = nodes[cursor].parent;
            }
            std::reverse(joints.begin(), joints.end());
            const State snap = state_;
            const std::size_t snap_len = plan_.size();
            for (const auto& j : joints) {
                if (!state_.apply_joint(j)) {
                    state_ = snap;
                    plan_.resize(snap_len);
                    return false;
                }
                append_joint(j);
            }
            return true;
        }
        ++expansions;

        // Enumerate all joint actions (5^N_active in reduced mode, 5^N
        // otherwise). For each agent, the per-cell delta is precomputed;
        // combinations with vertex/edge conflicts or out-of-bounds
        // destinations are skipped early. Pure-NoOp transition is excluded
        // (would loop forever).
        std::vector<int> jact(N, 0);
        std::function<void(int)> rec = [&](int ai) {
            if (ai == N) {
                std::vector<int> new_pos(N);
                std::vector<int> joint_full(N, 0);
                bool any_move = false;
                for (int a = 0; a < N; ++a) {
                    const int r = cur.pos[a] / C;
                    const int c = cur.pos[a] % C;
                    const int nr = r + kDr[jact[a]];
                    const int nc = c + kDc[jact[a]];
                    if (nr < 0 || nr >= R || nc < 0 || nc >= C) return;
                    if (obstacle[nr][nc]) return;
                    new_pos[a] = pos_of(nr, nc);
                    joint_full[a] = kAct[jact[a]];
                    if (jact[a] != 0) any_move = true;
                }
                if (!any_move) return;
                // Vertex conflict: no two agents on the same destination cell.
                for (int a = 0; a < N; ++a) {
                    for (int b = a + 1; b < N; ++b) {
                        if (new_pos[a] == new_pos[b]) return;
                    }
                }
                // Edge (swap) conflict: a→b's-old AND b→a's-old.
                for (int a = 0; a < N; ++a) {
                    for (int b = a + 1; b < N; ++b) {
                        if (new_pos[a] == cur.pos[b]
                            && new_pos[b] == cur.pos[a]) return;
                    }
                }
                const int ng = cur.g + 1;
                const std::string nkey = encode(new_pos);
                const auto it = best_g.find(nkey);
                if (it != best_g.end() && it->second <= ng) return;
                const int h = h_of(new_pos);
                if (h == kInf) return;
                best_g[nkey] = ng;
                // Weighted A* in reduced mode: heuristic is loose (BFS over
                // walls+boxes only, ignores agent positions), so admissible
                // A* expands too widely with N_active=5..6. W=5 trades
                // optimality for solvability — we only need any feasible
                // joint final-positioning plan, not the shortest one.
                const int w_h = active_agent_reduction ? (5 * h) : h;
                nodes.push_back({new_pos, joint_full, idx, ng, ng + w_h});
                open.push(static_cast<int>(nodes.size()) - 1);
                return;
            }
            // Inactive agents only emit NoOp in reduced mode → branching
            // collapses to 5^N_active.
            if (active_agent_reduction && !active[ai]) {
                jact[ai] = 0;
                rec(ai + 1);
                return;
            }
            for (int a = 0; a < 5; ++a) {
                jact[ai] = a;
                rec(ai + 1);
            }
        };
        rec(0);
    }
    if (jv) std::cerr << "[v2:cagj] search exhausted expansions=" << expansions
                      << " open.empty=" << (open.empty() ? 1 : 0) << "\n";
    return false;
}

// =============================================================================
// solve_joint_full() — last-resort full joint A* (agents + boxes)
// =============================================================================
//
// Bounded full joint-state A* used only when every primary task variant AND
// every extra task variant has failed. Targets tight small puzzles where the
// pipeline cannot find a serial decomposition (e.g. boxes mutually block in
// a tiny room). Uses the existing State::applicable / conflicting /
// apply_joint semantics so the resulting plan is server-valid by construction.
//
// Eligibility (any failure → false, no work done):
//   - ≥1 agent and ≤3 agents
//   - ≤6 movable boxes (letters with at least one goal)
//   - ≤120 non-wall reachable cells
//
// Heuristic (admissible under joint-step cost):
//   h(s) = max( for each letter-goal cell g: min over same-letter boxes of
//                walls-only distance(box, g),
//               for each agent-goal cell g_a: walls-only distance(agent, g_a) )
// This is a valid lower bound on the remaining makespan because each goal
// must be reached and a single joint step advances any one goal by at most 1.
//
// Caps:
//   - 80 000 node expansions
//   - 5 s wall-clock budget (also bounded by overall_deadline_)
//
// On success: plan_ is populated with the joint-action sequence; state_ is
// the goal state. On failure both are reset to initial values by the caller.
// =============================================================================
// solve_joint_full_from_current() — bounded full joint A* from current state
// =============================================================================
//
// Bounded joint-state A* used as a last-resort fallback. Searches forward
// from `state_` (the current state, which may be the initial state or a
// partial-progress snapshot restored by the caller). On success, APPENDS
// the joint-action sequence to `plan_`. On failure restores state_ and
// plan_ to their values at entry. Uses State::applicable / conflicting /
// apply_joint semantics so the resulting plan is server-valid by
// construction.
//
// Eligibility (any failure → false, no work done):
//   - 1 ≤ N ≤ 4 agents
//   - Mismatched-letter-box count and reachable-cell count gated by N:
//       N==1..3: mismatched ≤ 10, cells ≤ 200, total ≤ 18
//       N==4:    mismatched ≤  8, cells ≤ 260, total ≤ 14
//   - At least 2 seconds of overall-budget remaining.
//
// Heuristic (admissible under joint-step cost):
//   h(s) = max( for each letter-goal cell g: min over same-letter boxes of
//                walls-only distance(box, g),
//               for each agent-goal cell g_a: walls-only distance(agent, g_a) )
// This is a valid lower bound on the remaining makespan because each goal
// must be reached and a single joint step advances any one goal by at most 1.
//
// Caps:
//   - 80 000 node expansions
//   - `budget_seconds` wall-clock budget (also bounded by overall_deadline_)
bool Solver::solve_joint_full_from_current(int  budget_seconds,
                                           bool prune_satisfied_boxes,
                                           int  heuristic_weight,
                                           bool active_agent_reduction,
                                           bool force_divisor_bound)
{
    const bool jv = env_flag_enabled("V2_VERBOSE");
    const int N = static_cast<int>(state_.agent_rows.size());
    // Hard upper bound on total N: even with active reduction we still encode
    // every agent in the state and iterate per-agent for action lists; very
    // large N (e.g., 20) bloats both memory and per-node work.
    constexpr int kAbsMaxN = 16;
    if (N <= 0 || N > kAbsMaxN) {
        if (jv) std::cerr << "[v2:jaf] reject N=" << N
                          << " (abs cap " << kAbsMaxN << ")\n";
        return false;
    }
    if (!active_agent_reduction && N > 6) {
        if (jv) std::cerr << "[v2:jaf] reject N=" << N
                          << " (no active reduction)\n";
        return false;
    }

    // Active-agent mask (size N). With reduction OFF, every agent is active
    // and behaviour is identical to the legacy path.
    std::vector<bool> active(N, true);
    int n_active = N;
    if (active_agent_reduction) {
        active = compute_active_agent_mask(state_);
        n_active = 0;
        for (bool a : active) if (a) ++n_active;
        if (n_active == 0) {
            // Nothing for active set to do — check if we are at goal already.
            if (state_.goal_state()) {
                if (jv) std::cerr << "[v2:jaf] reduced: already at goal\n";
                return true;
            }
            // Unsatisfied-but-no-active-agent means a goal asks for a colour
            // no agent on the board can move. Reject.
            if (jv) std::cerr << "[v2:jaf] reduced: n_active=0 but not goal\n";
            return false;
        }
        // Trim active set to at most 6 agents when the loose color-mask
        // over-activates. Strategy: keep agents with own numeric goal
        // first (they MUST move); fill remaining slots with closest agents
        // (walls-only BFS distance) to any unsatisfied letter goal or
        // same-letter unmatched box of matching color. Eliminates the
        // "same-color sea" failure mode (Apdo-class) where every agent on
        // the board gets activated even though only a few are needed.
        constexpr int kActiveCap = 6;
        if (n_active > kActiveCap) {
            std::vector<bool> must_move(N, false);
            for (int r = 0; r < level_.rows; ++r) {
                for (int c = 0; c < level_.cols; ++c) {
                    const char g = level_.goals[r][c];
                    if (g < '0' || g > '9') continue;
                    const int ai = g - '0';
                    if (ai >= N) continue;
                    if (state_.agent_rows[ai] != r || state_.agent_cols[ai] != c) {
                        must_move[ai] = true;
                    }
                }
            }
            // Multi-source BFS over walls-only from every unsatisfied
            // letter goal + same-letter unmatched box; computes min
            // distance from each cell to nearest residual seed.
            const int R = level_.rows, C = level_.cols;
            std::vector<std::vector<int>> dist(R, std::vector<int>(C, -1));
            std::deque<std::pair<int,int>> bq;
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    if (level_.walls[r][c]) continue;
                    const char g = level_.goals[r][c];
                    const char b = state_.boxes[r][c];
                    bool seed = false;
                    if (g >= 'A' && g <= 'Z' && state_.boxes[r][c] != g) seed = true;
                    if (b >= 'A' && b <= 'Z' && level_.goals[r][c] != b) seed = true;
                    if (seed && dist[r][c] < 0) {
                        dist[r][c] = 0;
                        bq.emplace_back(r, c);
                    }
                }
            }
            static const int ddr[4] = {-1, 1, 0, 0};
            static const int ddc[4] = {0, 0, -1, 1};
            while (!bq.empty()) {
                auto [cr, cc] = bq.front();
                bq.pop_front();
                for (int k = 0; k < 4; ++k) {
                    const int nr = cr + ddr[k];
                    const int nc = cc + ddc[k];
                    if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                    if (level_.walls[nr][nc]) continue;
                    if (dist[nr][nc] >= 0) continue;
                    dist[nr][nc] = dist[cr][cc] + 1;
                    bq.emplace_back(nr, nc);
                }
            }
            // Rank candidate active agents by (must_move desc, dist asc).
            struct Cand { int a; int prio; int d; };
            std::vector<Cand> cands;
            cands.reserve(n_active);
            for (int a = 0; a < N; ++a) {
                if (!active[a]) continue;
                const int d = dist[state_.agent_rows[a]][state_.agent_cols[a]];
                cands.push_back({a, must_move[a] ? 1 : 0,
                                 d >= 0 ? d : INT_MAX});
            }
            std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) {
                if (x.prio != y.prio) return x.prio > y.prio;
                return x.d < y.d;
            });
            std::fill(active.begin(), active.end(), false);
            const int keep = std::min(static_cast<int>(cands.size()), kActiveCap);
            for (int i = 0; i < keep; ++i) active[cands[i].a] = true;
            n_active = keep;
            if (jv) std::cerr << "[v2:jaf] trimmed N_active to " << n_active << "\n";
        }
        // Blocker promotion: if N_active is too small (1 or 2), the active
        // agent(s) may be physically blocked by inactive agents sitting on
        // the only path to any seed cell (letter goal or unmatched same-letter
        // box). Search will incorrectly conclude infeasibility because
        // inactive agents are NoOp-only. Promote blockers by BFS-ing each
        // active agent toward each seed cell (walls+boxes only) and marking
        // any inactive agent on the resulting shortest path. Capped at
        // kActiveCap. Rescues GroupWon/TriSplit-class small-residual snapshots.
        if (n_active >= 1 && n_active <= 2) {
            const int R = level_.rows, C = level_.cols;
            // Collect seed cells: unsatisfied letter goals + unmatched
            // same-letter boxes of any color that has an active agent.
            std::vector<int> active_colors;
            for (int a = 0; a < N; ++a) {
                if (!active[a]) continue;
                const int col = level_.agent_color[a];
                if (col >= 0
                    && std::find(active_colors.begin(), active_colors.end(),
                                 col) == active_colors.end()) {
                    active_colors.push_back(col);
                }
            }
            std::vector<std::pair<int,int>> seeds;
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    if (level_.walls[r][c]) continue;
                    const char g = level_.goals[r][c];
                    if (g >= 'A' && g <= 'Z' && state_.boxes[r][c] != g) {
                        const int bc = level_.box_color[g - 'A'];
                        if (std::find(active_colors.begin(), active_colors.end(),
                                      bc) != active_colors.end()) {
                            seeds.emplace_back(r, c);
                        }
                    }
                    const char b = state_.boxes[r][c];
                    if (b >= 'A' && b <= 'Z' && level_.goals[r][c] != b) {
                        const int bc = level_.box_color[b - 'A'];
                        if (std::find(active_colors.begin(), active_colors.end(),
                                      bc) != active_colors.end()) {
                            seeds.emplace_back(r, c);
                        }
                    }
                }
            }
            // Static obstacle = walls ∪ boxes (agents are treated as
            // potential blockers we want to discover).
            std::vector<std::vector<bool>> static_obstacle(R,
                std::vector<bool>(C, false));
            for (int r = 0; r < R; ++r) {
                for (int c = 0; c < C; ++c) {
                    if (level_.walls[r][c]) static_obstacle[r][c] = true;
                    else if (state_.boxes[r][c] != '\0') {
                        static_obstacle[r][c] = true;
                    }
                }
            }
            std::vector<std::pair<int,int>> agent_cells(N);
            for (int a = 0; a < N; ++a) {
                agent_cells[a] = {state_.agent_rows[a], state_.agent_cols[a]};
            }
            static const int ddr[4] = {-1, 1, 0, 0};
            static const int ddc[4] = {0, 0, -1, 1};
            for (int aa = 0; aa < N && n_active < kActiveCap; ++aa) {
                if (!active[aa]) continue;
                const int sr = state_.agent_rows[aa];
                const int sc = state_.agent_cols[aa];
                // BFS distances from this active agent.
                std::vector<std::vector<int>> dist(R, std::vector<int>(C, -1));
                std::deque<std::pair<int,int>> q;
                dist[sr][sc] = 0;
                q.emplace_back(sr, sc);
                while (!q.empty()) {
                    auto [cr, cc] = q.front();
                    q.pop_front();
                    for (int k = 0; k < 4; ++k) {
                        const int nr = cr + ddr[k];
                        const int nc = cc + ddc[k];
                        if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                        // Allow stepping through agent cells (they're not
                        // walls/boxes — they'd be the blockers we want).
                        if (level_.walls[nr][nc]) continue;
                        if (state_.boxes[nr][nc] != '\0') continue;
                        if (dist[nr][nc] >= 0) continue;
                        dist[nr][nc] = dist[cr][cc] + 1;
                        q.emplace_back(nr, nc);
                    }
                }
                // For each seed, find nearest reachable cell; if reachable,
                // backtrack one shortest path and mark blockers.
                for (const auto& seed : seeds) {
                    if (n_active >= kActiveCap) break;
                    int best_r = -1, best_c = -1, best_d = INT_MAX;
                    // The seed itself may be a wall-adjacent or box cell;
                    // accept ANY adjacent reachable cell.
                    for (int k = 0; k < 4; ++k) {
                        const int nr = seed.first + ddr[k];
                        const int nc = seed.second + ddc[k];
                        if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                        if (dist[nr][nc] >= 0 && dist[nr][nc] < best_d) {
                            best_d = dist[nr][nc];
                            best_r = nr;
                            best_c = nc;
                        }
                    }
                    if (best_r < 0) continue;
                    int cr = best_r, cc = best_c;
                    while (!(cr == sr && cc == sc)) {
                        for (int b = 0; b < N && n_active < kActiveCap; ++b) {
                            if (!active[b]
                                && agent_cells[b].first == cr
                                && agent_cells[b].second == cc) {
                                active[b] = true;
                                ++n_active;
                            }
                        }
                        bool stepped = false;
                        for (int k = 0; k < 4; ++k) {
                            const int nr = cr + ddr[k];
                            const int nc = cc + ddc[k];
                            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
                            if (dist[nr][nc] == dist[cr][cc] - 1) {
                                cr = nr; cc = nc;
                                stepped = true;
                                break;
                            }
                        }
                        if (!stepped) break;
                    }
                }
            }
            if (jv) std::cerr << "[v2:jaf] blocker-promoted to N_active="
                              << n_active << "\n";
        }
        if (jv) std::cerr << "[v2:jaf] reduced mode: N=" << N
                          << " N_active=" << n_active << "\n";
    }

    // Count mismatched letter boxes (those NOT sitting on a matching letter
    // goal) AND total movable boxes from the current state. The mismatched
    // count drives "is this worth attempting"; the total count guards
    // against pathological branching when most boxes are satisfied but
    // dozens of others are still color-compatible and pushable.
    // Boxes whose color is not shared by any agent on the board are
    // immovable in practice (decoration / wall-fill). They should not
    // count toward total_movable or mismatched: they cannot be the
    // subject of push/pull and a goal cell asking for them is impossible
    // to satisfy through agent action.
    std::array<bool, 26> color_has_agent{};
    for (int a = 0; a < N; ++a) {
        const int c = level_.agent_color[a];
        if (c >= 0) {
            for (int b = 0; b < 26; ++b) {
                if (level_.box_color[b] == c) color_has_agent[b] = true;
            }
        }
    }
    int mismatched = 0;
    int total_movable = 0;
    int cell_count = 0;
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            if (level_.walls[r][c]) continue;
            ++cell_count;
            const char b = state_.boxes[r][c];
            if (b < 'A' || b > 'Z') continue;
            if (level_.box_color[b - 'A'] < 0) continue;
            if (!color_has_agent[b - 'A']) continue;
            ++total_movable;
            if (level_.goals[r][c] != b) ++mismatched;
        }
    }
    // For reduced mode, the more meaningful sizing metric is the cell count
    // walls-only-reachable from the active sub-problem (component-local).
    const int residual_cells = active_agent_reduction
        ? residual_reachable_cells(state_, active)
        : cell_count;

    if (active_agent_reduction) {
        // Per-N_active eligibility. Looser caps than the full-mode path
        // because effective branching is 9^N_active (inactive agents are
        // NoOp-only). residual_cells (component-local) used for size.
        // total_movable is NOT capped here: many movable boxes in a large
        // level are irrelevant to the residual sub-problem and the
        // applicable-action filter already bounds per-node successor
        // generation regardless of total box count.
        bool ok = false;
        if (n_active <= 3) {
            ok = (mismatched <= 20 && residual_cells <= 800);
        } else if (n_active == 4) {
            ok = (mismatched <= 16 && residual_cells <= 600);
        } else if (n_active == 5) {
            ok = (mismatched <= 14 && residual_cells <= 500);
        } else if (n_active == 6) {
            ok = (mismatched <= 12 && residual_cells <= 400);
        }
        // Positioning-only relax: when all letter goals are satisfied
        // (mismatched==0), action set collapses to Move/NoOp (Push/Pull
        // would unsatisfy a box) so branching is bounded by 5^N_active
        // rather than 9^N_active. Allow larger residual maps in that
        // regime — rescues MAze-class final-positioning failures with
        // ~500-cell active sub-region.
        if (!ok && mismatched == 0) {
            if      (n_active <= 3) ok = (residual_cells <= 1500);
            else if (n_active == 4) ok = (residual_cells <= 1000);
            else if (n_active == 5) ok = (residual_cells <= 800);
            else if (n_active == 6) ok = (residual_cells <= 600);
        }
        // Near-done relax: when the residual has very few mismatched boxes
        // (≤ 6), most actions reduce to positioning; allow large residual
        // maps. Rescues WardRush/ClauDOom-class snapshots that get close
        // to delivery completion but have a large open arena.
        if (!ok && mismatched <= 6) {
            if      (n_active <= 3) ok = (residual_cells <= 1200);
            else if (n_active == 4) ok = (residual_cells <= 900);
            else if (n_active == 5) ok = (residual_cells <= 750);
            else if (n_active == 6) ok = (residual_cells <= 650);
        }
        if (!ok) {
            if (jv) std::cerr << "[v2:jaf] reject elig REDUCED N_active=" << n_active
                              << " mis=" << mismatched
                              << " residual=" << residual_cells
                              << " total=" << total_movable << "\n";
            return false;
        }
        if (jv) std::cerr << "[v2:jaf] eligible REDUCED N=" << N
                          << " N_active=" << n_active
                          << " mis=" << mismatched
                          << " residual=" << residual_cells
                          << " total=" << total_movable
                          << " prune_sat=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
    } else if (N <= 3) {
        // Three escalating tiers:
        //   (a) baseline: tight caps on all axes
        //   (b) "almost done" relax: many boxes but ALL satisfied
        //   (c) prune-on relax: prune_satisfied_boxes=true means branching
        //       is bounded by the mismatched count (push/pull on satisfied
        //       boxes is skipped) — total_movable doesn't drive cost, so
        //       we admit large box counts when the mismatched set is small.
        //   (d) low-mis relax: regardless of prune flag, when mismatched
        //       is small the search frontier is bounded by mismatched-many
        //       box-transports + N-many agent positionings. Admits large
        //       total_movable (ZOOM-class redelivery puzzles).
        bool ok_a = (mismatched <= 12 && cell_count <= 320 && total_movable <= 28);
        bool ok_b = (mismatched == 0 && cell_count <= 700 && total_movable <= 50);
        bool ok_c = (prune_satisfied_boxes
                     && mismatched <= 18
                     && cell_count <= 600);
        bool ok_d = (mismatched <= 14 && cell_count <= 500
                     && total_movable <= 110);
        if (!(ok_a || ok_b || ok_c || ok_d)) {
            if (jv) std::cerr << "[v2:jaf] reject elig N<=3 mis=" << mismatched
                              << " cells=" << cell_count << " total=" << total_movable
                              << " prune=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
            return false;
        }
    } else if (N == 4) {
        bool ok_a = (mismatched <= 10 && cell_count <= 300 && total_movable <= 20);
        bool ok_b = (mismatched == 0 && cell_count <= 700 && total_movable <= 35);
        bool ok_c = (prune_satisfied_boxes
                     && mismatched <= 14
                     && cell_count <= 500);
        bool ok_d = (mismatched <= 10 && cell_count <= 400
                     && total_movable <= 60);
        if (!(ok_a || ok_b || ok_c || ok_d)) {
            if (jv) std::cerr << "[v2:jaf] reject elig N==4 mis=" << mismatched
                              << " cells=" << cell_count << " total=" << total_movable
                              << " prune=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
            return false;
        }
    } else if (N == 5) {
        // Branching factor up to 9^5=59k per node in worst case; in practice
        // per-agent applicable filter keeps it much lower in tight mazes.
        // Tight eligibility: keep state space + total work small.
        bool ok_a = (mismatched <= 10 && cell_count <= 250 && total_movable <= 14);
        // Positioning-only: when mis==0, branching is bounded by 5^N=3125
        // (Move/NoOp only) so we can admit larger maps + more boxes.
        bool ok_b = (mismatched == 0 && cell_count <= 900 && total_movable <= 30);
        bool ok_c = (prune_satisfied_boxes
                     && mismatched <= 8
                     && cell_count <= 400);
        if (!(ok_a || ok_b || ok_c)) {
            if (jv) std::cerr << "[v2:jaf] reject elig N==5 mis=" << mismatched
                              << " cells=" << cell_count << " total=" << total_movable
                              << " prune=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
            return false;
        }
    } else {  // N == 6
        // Even tighter: 9^6=531k per node worst-case branching.
        bool ok_a = (mismatched <= 8 && cell_count <= 200 && total_movable <= 10);
        bool ok_b = (mismatched == 0 && cell_count <= 700 && total_movable <= 25);
        bool ok_c = (prune_satisfied_boxes
                     && mismatched <= 6
                     && cell_count <= 350);
        if (!(ok_a || ok_b || ok_c)) {
            if (jv) std::cerr << "[v2:jaf] reject elig N==6 mis=" << mismatched
                              << " cells=" << cell_count << " total=" << total_movable
                              << " prune=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
            return false;
        }
    }
    if (!active_agent_reduction && jv) {
        std::cerr << "[v2:jaf] eligible N=" << N << " mis=" << mismatched
                  << " cells=" << cell_count << " total=" << total_movable
                  << " prune_sat=" << (prune_satisfied_boxes ? 1 : 0) << "\n";
    }

    auto now = std::chrono::steady_clock::now();
    if (now >= overall_deadline_) {
        if (jv) std::cerr << "[v2:jaf] reject overall_deadline\n";
        return false;
    }
    const auto remaining = overall_deadline_ - now;
    if (remaining < std::chrono::seconds(2)) {
        if (jv) std::cerr << "[v2:jaf] reject remaining<2s\n";
        return false;
    }
    const auto local_deadline = std::min(
        now + std::chrono::seconds(std::max(1, budget_seconds)),
        overall_deadline_);

    // Snapshot for full rollback on failure.
    const State snap_state = state_;
    const std::size_t snap_plan_len = plan_.size();

    // Collect goal cells.
    struct LetterGoal { int r, c; char letter; };
    struct AgentGoal  { int r, c; int agent; };
    std::vector<LetterGoal> letter_goals;
    std::vector<AgentGoal>  agent_goals;
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            const char g = level_.goals[r][c];
            if (g >= 'A' && g <= 'Z') {
                letter_goals.push_back({r, c, g});
            } else if (g >= '0' && g <= '9') {
                const int ai = g - '0';
                if (ai < N) agent_goals.push_back({r, c, ai});
            }
        }
    }

    // Walls-only BFS from each goal cell. dist[i][r*cols+c] = distance,
    // -1 if unreachable. Index i covers letter goals first then agent goals.
    const int cells = level_.rows * level_.cols;
    const int n_lg = static_cast<int>(letter_goals.size());
    const int n_ag = static_cast<int>(agent_goals.size());
    const int n_g  = n_lg + n_ag;
    std::vector<std::vector<int>> dist(n_g, std::vector<int>(cells, -1));

    auto idx = [&](int r, int c) { return r * level_.cols + c; };

    auto bfs_from = [&](int gi, int sr, int sc) {
        auto& d = dist[gi];
        std::deque<std::pair<int,int>> q;
        d[idx(sr, sc)] = 0;
        q.push_back({sr, sc});
        while (!q.empty()) {
            auto [r, c] = q.front();
            q.pop_front();
            const int dv = d[idx(r, c)];
            static const int DR[4] = {-1, 1, 0, 0};
            static const int DC[4] = { 0, 0,-1, 1};
            for (int k = 0; k < 4; ++k) {
                const int nr = r + DR[k], nc = c + DC[k];
                if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols) continue;
                if (level_.walls[nr][nc]) continue;
                if (d[idx(nr, nc)] >= 0) continue;
                d[idx(nr, nc)] = dv + 1;
                q.push_back({nr, nc});
            }
        }
    };
    for (int i = 0; i < n_lg; ++i) bfs_from(i, letter_goals[i].r, letter_goals[i].c);
    for (int i = 0; i < n_ag; ++i) bfs_from(n_lg + i, agent_goals[i].r, agent_goals[i].c);

    // Heuristic — admissible under joint-step cost (see header comment).
    // For single-agent levels (N==1) we use a tighter aggregation:
    //   h(g) = min over same-letter boxes b of
    //          (max(0, manhattan(agent, b) - 1) + walls_dist(b, g))
    //   h    = sum over unsatisfied letter goals of h(g)
    //          + max over unsatisfied position goals (sum is fine too)
    // The agent-to-box term is admissible because the agent must reach a
    // cell adjacent to box b before any push/pull of b; manhattan is a
    // walls-respecting lower bound. The sum-of-goals is admissible in
    // single-agent because all box transports are necessarily serial.
    const bool single_agent_h = (N == 1);
    // Effective agent count for the divisor in the multi-agent heuristic.
    const int N_eff_h = active_agent_reduction ? n_active : N;
    auto heuristic = [&](const State& s) -> int {
        int h_max = 0;
        int h_sum = 0;
        for (int i = 0; i < n_lg; ++i) {
            const auto& lg = letter_goals[i];
            if (s.boxes[lg.r][lg.c] == lg.letter) continue;
            int best = std::numeric_limits<int>::max();
            for (int r = 0; r < level_.rows; ++r) {
                for (int c = 0; c < level_.cols; ++c) {
                    if (s.boxes[r][c] != lg.letter) continue;
                    const int dv = dist[i][idx(r, c)];
                    if (dv < 0) continue;
                    int score = dv;
                    if (single_agent_h) {
                        const int dm = std::abs(s.agent_rows[0] - r)
                                     + std::abs(s.agent_cols[0] - c);
                        score += std::max(0, dm - 1);
                    }
                    if (score < best) best = score;
                }
            }
            if (best == std::numeric_limits<int>::max()) return std::numeric_limits<int>::max();
            if (best > h_max) h_max = best;
            h_sum += best;
        }
        for (int i = 0; i < n_ag; ++i) {
            const auto& ag = agent_goals[i];
            if (s.agent_rows[ag.agent] == ag.r && s.agent_cols[ag.agent] == ag.c) continue;
            const int dv = dist[n_lg + i][idx(s.agent_rows[ag.agent], s.agent_cols[ag.agent])];
            if (dv < 0) return std::numeric_limits<int>::max();
            if (dv > h_max) h_max = dv;
            h_sum += dv;
        }
        if (single_agent_h) return h_sum;
        // Multi-agent: gate the admissible divisor bound h_sum/N_eff so it
        // only fires when h_max is not already a meaningful binding signal.
        // The rationale: with weighted A* (w=3), tighter admissible h can
        // reorder the frontier in subtly bad ways when h_max is already
        // strong (TeamAgent-class, h_max ~= 12). But when h_max is small
        // relative to N_eff (LoopBots/ZOOM-class, h_max=1 with N>=4 boxes),
        // h_max gives no guidance and the divisor bound is essential.
        // Heuristic: trigger when h_max < N_eff_h, OR when force_divisor_bound
        // is set by the caller (used as a separate fallback pass for levels
        // where the gated heuristic fails to find a solution).
        if (force_divisor_bound || h_max < N_eff_h) {
            const int n_div = std::max(1, N_eff_h);
            const int h_div = (h_sum + n_div - 1) / n_div;
            if (h_div > h_max) return h_div;
        }
        return h_max;
    };

    auto goals_satisfied = [&](const State& s) -> int {
        int c = 0;
        for (const auto& lg : letter_goals) {
            if (s.boxes[lg.r][lg.c] == lg.letter) ++c;
        }
        for (const auto& ag : agent_goals) {
            if (s.agent_rows[ag.agent] == ag.r && s.agent_cols[ag.agent] == ag.c) ++c;
        }
        return c;
    };

    struct JNode {
        State state;
        int g = 0;
        int h = 0;
        int parent = -1;
        std::vector<int> joint_in;  // joint action that produced this state
    };

    std::vector<JNode> nodes;
    nodes.reserve(8192);

    // Open: (f, h, -goals_satisfied, idx) min-heap via greater<>.
    using Key = std::tuple<int,int,int,int>;
    std::priority_queue<Key, std::vector<Key>, std::greater<Key>> open;

    std::unordered_map<State, int, StateHash, StateEq> closed_g;

    // Weighted A*: f = g + W*h. Find feasible (not necessarily optimal)
    // plans much faster — this is a fallback layer with a tight wall-time
    // budget, and we'd rather get *any* working plan than an optimal one
    // that times out. Caller can override the default weight: higher
    // values are more greedy-toward-goal (faster but more sub-optimal)
    // and rescue longer-horizon residuals.
    const int kHWeight = std::max(1, heuristic_weight);
    // Expansion cap scales down with N because per-expansion successor
    // generation is exponentially more costly at higher N. With active
    // reduction we use N_active (effective branching base) rather than N.
    const int N_eff = active_agent_reduction ? n_active : N;
    // Single-agent levels (especially Sokoban-like SAD2/SASolo/SAboXboXboX)
    // have small per-node successor count but enormous state spaces — the
    // weak max-of-goal-distances heuristic gives little guidance, so we
    // need a much larger expansion budget to make progress.
    const int kNodeCap = (N_eff == 1) ? 1500000
                       : (N_eff <= 4) ? 250000
                       : (N_eff == 5) ? 80000
                                      : 40000;
    // Hard memory guard: bound generated state count too (per rubber-duck
    // critique). One pop can produce thousands of successors at N=5/6, so
    // expansion cap alone doesn't bound memory. Generous for N=6 because
    // tight box-rearrangement levels (Nej-class) need ≥150k states to
    // reach the goal — the 80k legacy cap was too small.
    const std::size_t kNodesMemCap = (N_eff == 1) ? 2000000u
                                   : (N_eff <= 4) ? 600000u
                                   : (N_eff == 5) ? 400000u
                                                  : 350000u;
    closed_g.reserve(kNodesMemCap);

    // Seed from current state_ (not initial_state_).
    {
        JNode root;
        root.state = state_;
        root.g = 0;
        root.h = heuristic(root.state);
        if (root.h == std::numeric_limits<int>::max()) {
            if (jv) std::cerr << "[v2:jaf] reject root h=inf (goal unreachable)\n";
            return false;
        }
        if (jv) std::cerr << "[v2:jaf] root h=" << root.h
                          << " sat=" << goals_satisfied(root.state) << "\n";
        nodes.push_back(std::move(root));
        closed_g.emplace(nodes.back().state, 0);
        open.emplace(nodes.back().g + kHWeight * nodes.back().h, nodes.back().h,
                     -goals_satisfied(nodes.back().state), 0);
    }

    const auto& acts = actions();
    const int A = static_cast<int>(acts.size());

    // When the snapshot is at a "delivery COMPLETE" state (no mismatched
    // boxes) we KEEP push/pull allowed but rely on prune_satisfied_boxes
    // to skip the wasteful disturb-and-recover joint actions. Hard
    // disabling push/pull caused pacMAn-class to exhaust the search
    // space because some satisfied boxes legitimately block the only
    // path to an agent goal and must be temporarily displaced.
    const bool agent_only_mode = false;

    std::vector<std::vector<int>> per_agent(N);

    int expansions = 0;
    int success_idx = -1;
    int budget_check = 0;

    while (!open.empty()) {
        auto [f, hv, neg_sat, cur_idx] = open.top();
        open.pop();
        (void)f; (void)hv; (void)neg_sat;

        if (++budget_check == 256) {
            budget_check = 0;
            if (std::chrono::steady_clock::now() >= local_deadline) break;
        }
        if (expansions >= kNodeCap) break;
        if (nodes.size() >= kNodesMemCap) {
            if (jv) std::cerr << "[v2:jaf] mem cap hit nodes=" << nodes.size() << "\n";
            break;
        }

        const JNode cur = nodes[cur_idx];
        auto it = closed_g.find(cur.state);
        if (it == closed_g.end() || it->second < cur.g) continue;

        if (cur.state.goal_state()) { success_idx = cur_idx; break; }

        ++expansions;

        for (int a = 0; a < N; ++a) {
            per_agent[a].clear();
            // Inactive agent in reduced mode: only NoOp is allowed. Branching
            // factor for that agent collapses to 1, so the effective branching
            // base shrinks from 9^N to 9^N_active.
            if (active_agent_reduction && !active[a]) {
                per_agent[a].push_back(0);
                continue;
            }
            const int ar = cur.state.agent_rows[a];
            const int ac = cur.state.agent_cols[a];
            for (int k = 0; k < A; ++k) {
                const auto& act = acts[k];
                if (agent_only_mode) {
                    if (act.type != ActionType::NoOp && act.type != ActionType::Move) continue;
                }
                // Sound prune (when prune_satisfied_boxes=true): skip
                // Push/Pull actions whose target box sits on a matching
                // letter goal — disturbing such a box forces a re-delivery.
                // The caller controls this prune so that a second pass
                // without pruning can rescue levels where the satisfied
                // box must move (e.g. final-positioning blocker chains).
                if (prune_satisfied_boxes && act.type == ActionType::Push) {
                    const int br = ar + act.agent_dr;
                    const int bc = ac + act.agent_dc;
                    if (br >= 0 && br < level_.rows && bc >= 0 && bc < level_.cols) {
                        const char b = cur.state.boxes[br][bc];
                        if (b >= 'A' && b <= 'Z' && level_.goals[br][bc] == b) continue;
                    }
                } else if (prune_satisfied_boxes && act.type == ActionType::Pull) {
                    const int br = ar - act.box_dr;
                    const int bc = ac - act.box_dc;
                    if (br >= 0 && br < level_.rows && bc >= 0 && bc < level_.cols) {
                        const char b = cur.state.boxes[br][bc];
                        if (b >= 'A' && b <= 'Z' && level_.goals[br][bc] == b) continue;
                    }
                }
                if (cur.state.applicable(a, act)) per_agent[a].push_back(k);
            }
            if (per_agent[a].empty()) per_agent[a].push_back(0);
        }

        std::vector<int> joint(N, 0);
        std::function<void(int)> rec = [&](int ai) {
            if (success_idx >= 0) return;
            // Memory guard: stop generating successors if we've already
            // accumulated too many states. Cuts off remaining branches of
            // this expansion (sound — we just stop exploring further).
            if (nodes.size() >= kNodesMemCap) return;
            if (ai == N) {
                bool any = false;
                for (int v : joint) if (v != 0) { any = true; break; }
                if (!any) return;
                if (cur.state.conflicting(joint)) return;
                State next = cur.state;
                if (!next.apply_joint(joint)) return;
                const int ng = cur.g + 1;
                auto ins = closed_g.emplace(next, ng);
                if (!ins.second) {
                    if (ins.first->second <= ng) return;
                    ins.first->second = ng;
                }
                int nh = heuristic(next);
                if (nh == std::numeric_limits<int>::max()) return;
                JNode nn;
                nn.state = std::move(next);
                nn.g = ng;
                nn.h = nh;
                nn.parent = cur_idx;
                nn.joint_in = joint;
                const int nf = ng + kHWeight * nh;
                const int sat = goals_satisfied(nn.state);
                nodes.push_back(std::move(nn));
                open.emplace(nf, nh, -sat, static_cast<int>(nodes.size()) - 1);
                if (nodes.back().state.goal_state()) {
                    success_idx = static_cast<int>(nodes.size()) - 1;
                }
                return;
            }
            for (int k : per_agent[ai]) {
                joint[ai] = k;
                rec(ai + 1);
                if (success_idx >= 0) return;
            }
            joint[ai] = 0;
        };
        rec(0);
        if (success_idx >= 0) break;
    }

    if (success_idx < 0) {
        if (jv) std::cerr << "[v2:jaf] no solution: expansions=" << expansions
                          << " nodes=" << nodes.size()
                          << " open=" << open.size() << "\n";
        // Rollback (we never mutated state_ or plan_, but explicit anyway).
        state_ = snap_state;
        plan_.resize(snap_plan_len);
        return false;
    }
    if (jv) std::cerr << "[v2:jaf] found solution: expansions=" << expansions
                      << " nodes=" << nodes.size() << "\n";

    // Reconstruct joint-action sequence by walking parent pointers.
    std::vector<std::vector<int>> joints;
    for (int i = success_idx; i != -1; i = nodes[i].parent) {
        if (!nodes[i].joint_in.empty()) joints.push_back(nodes[i].joint_in);
    }
    std::reverse(joints.begin(), joints.end());

    // Apply joints to state_ starting from the snapshot (which is what
    // state_ already equals — we never mutated it). On any apply_joint
    // failure (paranoia), restore.
    for (const auto& j : joints) {
        if (!state_.apply_joint(j)) {
            state_ = snap_state;
            plan_.resize(snap_plan_len);
            return false;
        }
        plan_.push_back(j);
    }
    if (!state_.goal_state()) {
        state_ = snap_state;
        plan_.resize(snap_plan_len);
        return false;
    }
    return true;
}

bool Solver::solve_joint_full()
{
    // Backwards-compatible "from scratch" wrapper. Resets to initial then
    // delegates to the from-current implementation.
    state_ = initial_state_;
    plan_.clear();
    return solve_joint_full_from_current(/*budget_seconds=*/5);
}

// =============================================================================
// compute_active_agent_mask — active-agent reduction support
// =============================================================================
//
// An agent is "active" in the current state if it could plausibly need to move
// to satisfy any remaining goal:
//   - It has an unsatisfied numeric goal, OR
//   - Its color matches the color of at least one unsatisfied letter goal,
//     AND there is at least one same-color unmatched box (or same-letter
//     unsatisfied goal) in the SAME walls-only connected component as the
//     agent. The component filter prevents waking distant same-color agents
//     that cannot possibly reach the residual sub-problem.
//
// Used by the reduced joint A* mode to mark unrelated agents as static
// (NoOp-only). Branching factor drops from 9^N to 9^N_active.
// =============================================================================
std::vector<bool> Solver::compute_active_agent_mask(const State& s) const
{
    const int N = static_cast<int>(s.agent_rows.size());
    std::vector<bool> active(N, false);

    // Index agents by component (walls-only).
    const auto& comp_ids = topology_.component_ids();
    auto comp_of = [&](int r, int c) -> int {
        if (r < 0 || r >= level_.rows || c < 0 || c >= level_.cols) return -1;
        return comp_ids[r][c];
    };

    // (1) Agent has its own unsatisfied numeric goal.
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            const char g = level_.goals[r][c];
            if (g < '0' || g > '9') continue;
            const int ai = g - '0';
            if (ai >= N) continue;
            if (s.agent_rows[ai] != r || s.agent_cols[ai] != c) {
                active[ai] = true;
            }
        }
    }

    // (2) Per-letter: find unsatisfied letter goals and same-color unmatched
    // boxes. Wake the same-color agents that share a component with at least
    // one of them.
    for (char letter = 'A'; letter <= 'Z'; ++letter) {
        const int color = level_.box_color[letter - 'A'];
        if (color < 0) continue;

        std::set<int> relevant_comps;
        bool any_unsatisfied = false;
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (level_.goals[r][c] == letter && s.boxes[r][c] != letter) {
                    const int ci = comp_of(r, c);
                    if (ci >= 0) relevant_comps.insert(ci);
                    any_unsatisfied = true;
                }
                if (s.boxes[r][c] == letter && level_.goals[r][c] != letter) {
                    const int ci = comp_of(r, c);
                    if (ci >= 0) relevant_comps.insert(ci);
                    // An unmatched same-letter box doesn't by itself need
                    // delivery (if there's no goal for it), but it might
                    // serve as a source for some unsatisfied goal — only
                    // wake agents when any_unsatisfied is also true.
                }
            }
        }
        if (!any_unsatisfied) continue;

        for (int a = 0; a < N; ++a) {
            if (active[a]) continue;
            if (level_.agent_color[a] != color) continue;
            const int aci = comp_of(s.agent_rows[a], s.agent_cols[a]);
            if (relevant_comps.count(aci) > 0) active[a] = true;
        }
    }

    return active;
}

// =============================================================================
// residual_reachable_cells — eligibility-helper for reduced joint A*
// =============================================================================
//
// Cells walls-only-reachable from any "interesting" seed in the current state.
// Seeds:
//   - each active agent's cell,
//   - each unsatisfied letter-goal cell,
//   - each same-color unmatched box cell (color of any active agent).
// Component-decomposable levels (AMC, KUTitans) typically have a much smaller
// residual reachable count than the global cell count, which lets the reduced
// joint A* fire on residuals that look too big globally.
// =============================================================================
int Solver::residual_reachable_cells(const State& s,
                                     const std::vector<bool>& active) const
{
    const int R = level_.rows, C = level_.cols;
    std::vector<std::vector<char>> seen(R, std::vector<char>(C, 0));
    std::deque<std::pair<int,int>> q;

    auto push_seed = [&](int r, int c) {
        if (r < 0 || r >= R || c < 0 || c >= C) return;
        if (level_.walls[r][c]) return;
        if (seen[r][c]) return;
        seen[r][c] = 1;
        q.emplace_back(r, c);
    };

    // Active agent positions.
    for (int a = 0; a < static_cast<int>(s.agent_rows.size()); ++a) {
        if (a < static_cast<int>(active.size()) && active[a]) {
            push_seed(s.agent_rows[a], s.agent_cols[a]);
        }
    }

    // Unsatisfied letter goals and same-color unmatched boxes.
    std::array<bool, 26> color_active{};
    for (int a = 0; a < static_cast<int>(s.agent_rows.size()); ++a) {
        if (a < static_cast<int>(active.size()) && active[a]) {
            const int col = level_.agent_color[a];
            if (col >= 0) {
                for (int b = 0; b < 26; ++b) {
                    if (level_.box_color[b] == col) color_active[b] = true;
                }
            }
        }
    }
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            const char g = level_.goals[r][c];
            if (g >= 'A' && g <= 'Z' && s.boxes[r][c] != g) {
                if (color_active[g - 'A']) push_seed(r, c);
            }
            const char b = s.boxes[r][c];
            if (b >= 'A' && b <= 'Z' && level_.goals[r][c] != b) {
                if (color_active[b - 'A']) push_seed(r, c);
            }
        }
    }

    static const int DR[4] = {-1, 1, 0, 0};
    static const int DC[4] = { 0, 0,-1, 1};
    int count = 0;
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop_front();
        ++count;
        for (int k = 0; k < 4; ++k) {
            const int nr = r + DR[k], nc = c + DC[k];
            if (nr < 0 || nr >= R || nc < 0 || nc >= C) continue;
            if (level_.walls[nr][nc]) continue;
            if (seen[nr][nc]) continue;
            seen[nr][nc] = 1;
            q.emplace_back(nr, nc);
        }
    }
    return count;
}

int Solver::letter_goals_satisfied(const State& s) const
{
    int n = 0;
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            const char g = level_.goals[r][c];
            if (g >= 'A' && g <= 'Z' && s.boxes[r][c] == g) ++n;
        }
    }
    return n;
}

int Solver::agent_goals_satisfied(const State& s) const
{
    int n = 0;
    const int N = static_cast<int>(s.agent_rows.size());
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            const char g = level_.goals[r][c];
            if (g < '0' || g > '9') continue;
            const int ai = g - '0';
            if (ai >= N) continue;
            if (s.agent_rows[ai] == r && s.agent_cols[ai] == c) ++n;
        }
    }
    return n;
}

void Solver::update_best_snapshot()
{
    BestSnapshot cand;
    cand.state = state_;
    cand.plan = plan_;
    cand.letter_goals_satisfied = letter_goals_satisfied(state_);
    cand.agent_goals_satisfied  = agent_goals_satisfied(state_);
    cand.valid = true;
    if (!best_snapshot_.valid
        || cand.score() > best_snapshot_.score()
        || (cand.score() == best_snapshot_.score()
            && cand.plan.size() < best_snapshot_.plan.size())) {
        best_snapshot_ = std::move(cand);
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
    // others. Capped by `overall_deadline_` so the post-variant fallback
    // pass also respects the solver-wide soft budget — variants near the
    // end of the budget get progressively less time, naturally degrading
    // gracefully instead of busting the server timeout.
    constexpr double kVariantBudgetSeconds = 12.0;
    const auto now = std::chrono::steady_clock::now();
    const auto soft_cap = now
        + std::chrono::milliseconds(
            static_cast<int>(kVariantBudgetSeconds * 1000));
    variant_deadline_ = (soft_cap < overall_deadline_) ? soft_cap
                                                       : overall_deadline_;

    if (tasks.empty()) {
        // Levels with zero box tasks still need the final agent-goal phase.
        if (!complete_agent_goals()) return false;
        return state_.goal_state();
    }
    const bool verbose = env_flag_enabled("V2_VERBOSE");

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
                update_best_snapshot();
                continue;
            }
            // Eager scatter: cheap to attempt and rolls back fully on failure.
            // Many fast-failing tasks just need a single agent off the corridor.
            if (deliver_task_with_scatter(t)) {
                consecutive_failures = 0;
                update_best_snapshot();
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
                    update_best_snapshot();
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
                    update_best_snapshot();
                    continue;
                }
                // Last resort: allow displacing on-goal blockers (they will
                // be re-queued by the redelivery scan after this loop).
                if (deliver_task_with_relocation(t, /*allow_on_goal_blockers=*/true)) {
                    consecutive_failures = 0;
                    update_best_snapshot();
                    continue;
                }
                return false;
            }
            queue.push_back(t);
        }
        return queue.empty();
    };

    if (!run_queue()) {
        if (verbose) std::cerr << "[v2]   first run_queue failed, plan_len=" << plan_.size() << "\n";
        return false;
    }
    if (verbose) std::cerr << "[v2]   first run_queue OK, plan_len=" << plan_.size() << "\n";

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
        if (!run_queue()) {
            if (verbose) std::cerr << "[v2]   redelivery run_queue failed, plan_len=" << plan_.size() << "\n";
            return false;
        }
    }

    if (verbose) std::cerr << "[v2]   delivery COMPLETE, plan_len=" << plan_.size() << ", now agent-positioning\n";
    // Final pre-positioning snapshot — delivery is fully done, this is the
    // cleanest state for the last-resort joint A* fallback to launch from.
    update_best_snapshot();
    if (!complete_agent_goals()) {
        if (verbose) std::cerr << "[v2]   complete_agent_goals FAILED, plan_len=" << plan_.size() << "\n";
        return false;
    }
    if (verbose) std::cerr << "[v2]   complete_agent_goals OK, plan_len=" << plan_.size() << "\n";
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
    const bool verbose = env_flag_enabled("V2_VERBOSE");

    // Overall solver-wide soft deadline. Leaves a 3s safety margin under the
    // typical 30s server timeout, but big enough that the existing primary
    // pipeline (which usually finishes in <10s on solvable levels) is never
    // truncated by it. Only the post-variant fallback loop respects this
    // deadline; primary variants still get their per-variant 12s budget.
    constexpr int kOverallBudgetSeconds = 27;
    overall_deadline_ = std::chrono::steady_clock::now()
        + std::chrono::seconds(kOverallBudgetSeconds);

    // Reset best-progress snapshot at the start of each solve() invocation
    // (the Solver may, in principle, be reused for multiple solves).
    best_snapshot_ = BestSnapshot{};

    auto variants = build_task_variants();
    if (verbose) {
        std::cerr << "[v2] variants generated: " << variants.size() << "\n";
        for (std::size_t i = 0; i < variants.size(); ++i) {
            std::cerr << "[v2] variant " << i << " tasks=" << variants[i].size() << " :";
            for (const auto& t : variants[i]) {
                std::cerr << " " << t.letter << "(" << t.box_row << "," << t.box_col
                          << ")->(" << t.goal_row << "," << t.goal_col << ")@a" << t.agent;
            }
            std::cerr << "\n";
        }
    }
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
        if (verbose) std::cerr << "[v2] trying variant " << i << "\n";
        if (solve_once(variants[i])) {
            if (verbose) std::cerr << "[v2] variant " << i << " SOLVED, plan_len=" << plan_.size() << "\n";
            return finalize(plan_);
        }
        if (verbose) std::cerr << "[v2] variant " << i << " failed, plan_len=" << plan_.size() << "\n";
    }

    // Post-variant fallback pass: every primary variant failed but we still
    // have budget. Generate min-max-DP and randomised letter-group variants
    // and try them under a tighter per-variant budget (so we get more shots
    // in the remaining time). Strictly additive — if no fallback variant
    // works, the function returns {} exactly as it would have before.
    if (!overall_time_up()) {
        auto extras = build_extra_variants(variants);
        if (verbose) std::cerr << "[v2] extras: " << extras.size() << "\n";
        for (std::size_t i = 0; i < extras.size(); ++i) {
            if (overall_time_up()) break;
            state_ = initial_state_;
            plan_.clear();
            if (verbose) std::cerr << "[v2] trying extra " << i << "\n";
            if (solve_once(extras[i])) {
                if (verbose) std::cerr << "[v2] extra " << i << " SOLVED\n";
                return finalize(plan_);
            }
        }
    }

    // ---------------------------------------------------------------------
    // Last-resort fallback layer A: full joint A* (agents + boxes) from
    // the INITIAL state, for very small problems. Often succeeds for
    // small N levels where the residual is similar to the full level
    // (e.g. TeamAgent-class) — in those cases solving from-initial is
    // faster than restoring a partial-progress snapshot first.
    //
    // Eligibility (mismatched/cells/total caps) decides whether the
    // attempt actually runs, so this is strictly additive: it returns
    // false quickly when the level is too big.
    // ---------------------------------------------------------------------
    if (!overall_time_up()) {
        state_ = initial_state_;
        plan_.clear();
        if (verbose) std::cerr << "[v2] trying solve_joint_full (from initial)\n";
        if (solve_joint_full()) {
            if (verbose) std::cerr << "[v2] solve_joint_full SOLVED\n";
            return finalize(plan_);
        }
        if (verbose) std::cerr << "[v2] solve_joint_full failed (likely ineligible or out of budget)\n";
        // For levels where NO variant made progress (letters_satisfied == 0),
        // best_snapshot_ remains invalid and the snapshot fallbacks below
        // won't fire — rendering only the W=1 attempt above. Add W=3 and
        // W=5 weighted passes from initial state to rescue tight
        // box-stacking puzzles (Nej-class) where the heuristic
        // underestimates the joint cost and admissible A* exhausts memory
        // long before the goal.
        const bool no_progress = !best_snapshot_.valid
            || best_snapshot_.letter_goals_satisfied == 0;
        if (no_progress && !overall_time_up()) {
            state_ = initial_state_;
            plan_.clear();
            if (verbose) std::cerr << "[v2] trying solve_joint_full from initial (W=3)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/3)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] solve_joint_full W=3 SOLVED\n";
                return finalize(plan_);
            }
        }
        if (no_progress && !overall_time_up()) {
            state_ = initial_state_;
            plan_.clear();
            if (verbose) std::cerr << "[v2] trying solve_joint_full from initial (W=5)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/5)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] solve_joint_full W=5 SOLVED\n";
                return finalize(plan_);
            }
        }
    }

    // ---------------------------------------------------------------------
    // Last-resort fallback layer B: launch full joint A* from the BEST
    // partial-progress snapshot captured across all variant attempts.
    //
    // Many failing levels (pacMAn-class, EpicfAIl-class) reach a state in
    // which most letter goals are satisfied but the final residual is too
    // hard for the serial/PIBT/serial-joint agent-positioning chain. By
    // restoring the highest-progress snapshot and pointing the bounded
    // joint A* at the *residual* sub-problem (which is much smaller than
    // the original level), we can finish what the variants started.
    //
    // Two passes: first with satisfied-box pruning for speed/soundness
    // on clean residuals; second without pruning to rescue levels where
    // the only path through a corridor must temporarily displace a
    // satisfied box (pacMAn-class).
    // ---------------------------------------------------------------------
    if (!overall_time_up() && best_snapshot_.valid
        && best_snapshot_.letter_goals_satisfied > 0) {
        const State snap_state = best_snapshot_.state;
        const auto  snap_plan  = best_snapshot_.plan;
        state_ = snap_state;
        plan_  = snap_plan;
        if (verbose) {
            std::cerr << "[v2] trying joint A* from snapshot pass1 (score="
                      << best_snapshot_.score()
                      << " letters=" << best_snapshot_.letter_goals_satisfied
                      << " agents=" << best_snapshot_.agent_goals_satisfied
                      << " plan_len=" << plan_.size() << ")\n";
        }
        if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                          /*prune_satisfied_boxes=*/true)
            && state_.goal_state()) {
            if (verbose) std::cerr << "[v2] joint A* pass1 SOLVED\n";
            return finalize(plan_);
        }
        if (!overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass2 (no satisfied-box prune)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/5,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/3)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass2 SOLVED\n";
                return finalize(plan_);
            }
        }
        // Pass 3: high-weight greedy A* (W=8) for long-horizon residuals
        // where the heuristic underestimates the true joint cost badly
        // (h=20+ with branching ~8^4 makes the search hit the node cap
        // long before reaching the goal at default weight). Higher W
        // accepts more sub-optimality in exchange for finding *some*
        // feasible plan.
        if (!overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass3 (W=8)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/8)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass3 SOLVED\n";
                return finalize(plan_);
            }
        }
        // Pass 3b: super-greedy weighted A* (W=15) — last shot for tight
        // redelivery puzzles (DECrunchy-class) where W=8 still fails. Very
        // sub-optimal but rescues long-horizon residuals.
        if (!overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass3b (W=15)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/15)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass3b SOLVED\n";
                return finalize(plan_);
            }
        }
        // Pass 3c: divisor-bound heuristic (force max(h_max, h_sum/N) always).
        // Targets multi-box low-h_max residuals (LoopBots-class) where the
        // gated heuristic returns just h_max=1 (no guidance) and the search
        // explodes. The divisor bound provides per-agent average distance
        // signal that orders the frontier toward the goal more aggressively.
        // Runs as a separate pass so TeamAgent-class levels (which prefer
        // plain h_max) keep solving via pass1/2.
        if (!overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass3c (divisor-bound, W=3)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/5,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/3,
                                              /*active_agent_reduction=*/false,
                                              /*force_divisor_bound=*/true)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass3c SOLVED\n";
                return finalize(plan_);
            }
        }
        // Pass 3d: divisor-bound heuristic with higher weight (W=8) for
        // residuals where W=3 still exhausts the budget.
        if (!overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass3d (divisor-bound, W=8)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/4,
                                              /*prune_satisfied_boxes=*/false,
                                              /*heuristic_weight=*/8,
                                              /*active_agent_reduction=*/false,
                                              /*force_divisor_bound=*/true)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass3d SOLVED\n";
                return finalize(plan_);
            }
        }

        // Pass 4 & 5: active-agent-reduced joint A*. Lifts the N ≤ 6 cap
        // so levels with many agents but only a few "active" ones
        // (component-local + color-relevant) can be solved through the
        // joint-A* fallback. Only fires when reduction would actually
        // help: legacy N ≤ 6 path runs first via the prior 3 passes, so
        // these passes are strictly additive.
        const int N_total = static_cast<int>(snap_state.agent_rows.size());
        if (N_total > 6 && !overall_time_up()) {
            state_ = snap_state;
            plan_  = snap_plan;
            if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass4 (REDUCED, prune, W=3)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/6,
                                              /*prune_satisfied_boxes=*/true,
                                              /*heuristic_weight=*/3,
                                              /*active_agent_reduction=*/true)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] joint A* pass4 (REDUCED) SOLVED\n";
                return finalize(plan_);
            }
            if (!overall_time_up()) {
                state_ = snap_state;
                plan_  = snap_plan;
                if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass5 (REDUCED, no prune, W=5)\n";
                if (solve_joint_full_from_current(/*budget_seconds=*/6,
                                                  /*prune_satisfied_boxes=*/false,
                                                  /*heuristic_weight=*/5,
                                                  /*active_agent_reduction=*/true)
                    && state_.goal_state()) {
                    if (verbose) std::cerr << "[v2] joint A* pass5 (REDUCED) SOLVED\n";
                    return finalize(plan_);
                }
            }
            // Pass 5b: super-greedy reduced (W=12) for tight residuals
            // where lower weights still run out of time.
            if (!overall_time_up()) {
                state_ = snap_state;
                plan_  = snap_plan;
                if (verbose) std::cerr << "[v2] trying joint A* from snapshot pass5b (REDUCED, no prune, W=12)\n";
                if (solve_joint_full_from_current(/*budget_seconds=*/5,
                                                  /*prune_satisfied_boxes=*/false,
                                                  /*heuristic_weight=*/12,
                                                  /*active_agent_reduction=*/true)
                    && state_.goal_state()) {
                    if (verbose) std::cerr << "[v2] joint A* pass5b (REDUCED) SOLVED\n";
                    return finalize(plan_);
                }
            }
        }
        if (verbose) std::cerr << "[v2] joint A* from snapshot failed (all passes)\n";
    }

    // Final fallback: reduced joint A* from the INITIAL state for levels
    // where no variant produced a useful snapshot but a small "active"
    // sub-problem exists (e.g. some N > 6 levels where most agents have
    // no goals at all). Only fires for N > 6 (legacy `solve_joint_full`
    // from-initial already ran for N ≤ 6).
    if (!overall_time_up()
        && static_cast<int>(initial_state_.agent_rows.size()) > 6) {
        state_ = initial_state_;
        plan_.clear();
        if (verbose) std::cerr << "[v2] trying solve_joint_full_from_current (REDUCED, from initial)\n";
        if (solve_joint_full_from_current(/*budget_seconds=*/10,
                                          /*prune_satisfied_boxes=*/true,
                                          /*heuristic_weight=*/3,
                                          /*active_agent_reduction=*/true)
            && state_.goal_state()) {
            if (verbose) std::cerr << "[v2] reduced joint A* (from initial) SOLVED\n";
            return finalize(plan_);
        }
        // Second pass with higher weight for tight residuals where W=3
        // still expands too slowly.
        if (!overall_time_up()) {
            state_ = initial_state_;
            plan_.clear();
            if (verbose) std::cerr << "[v2] trying solve_joint_full_from_current (REDUCED, from initial, W=8)\n";
            if (solve_joint_full_from_current(/*budget_seconds=*/8,
                                              /*prune_satisfied_boxes=*/true,
                                              /*heuristic_weight=*/8,
                                              /*active_agent_reduction=*/true)
                && state_.goal_state()) {
                if (verbose) std::cerr << "[v2] reduced joint A* (from initial, W=8) SOLVED\n";
                return finalize(plan_);
            }
        }
    }

    return {};
}

}  // namespace aimas
