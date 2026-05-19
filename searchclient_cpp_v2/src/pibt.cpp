#include "aimas/pibt.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <functional>
#include <numeric>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace aimas {

namespace {

// Per-action delta tables. Indices 0..4 = NoOp, N, S, E, W. Matches the
// canonical action table in core.cpp.
constexpr std::array<int, 5> kAdr{{0, -1, 1, 0, 0}};
constexpr std::array<int, 5> kAdc{{0, 0, 0, 1, -1}};
constexpr std::array<int, 5> kActionIndices{{0, 1, 2, 3, 4}};

std::string agent_state_key(const State& s)
{
    std::string key;
    key.reserve(s.agent_rows.size() * 8);
    for (std::size_t a = 0; a < s.agent_rows.size(); ++a) {
        key.append(std::to_string(s.agent_rows[a]));
        key.push_back(',');
        key.append(std::to_string(s.agent_cols[a]));
        key.push_back(';');
    }
    return key;
}

// BFS from goal cell over walls and boxes (boxes treated as walls).
// The goal cell itself is reachable even if box-occupied so a stranded
// agent reads distance 0. Returns kInf where unreachable.
std::vector<std::vector<int>> box_aware_bfs(
    const Level& level, const State& s, int goal_row, int goal_col)
{
    std::vector<std::vector<int>> d(level.rows, std::vector<int>(level.cols, kInf));
    if (goal_row < 0 || goal_row >= level.rows
        || goal_col < 0 || goal_col >= level.cols
        || level.walls[goal_row][goal_col]) {
        return d;
    }
    d[goal_row][goal_col] = 0;
    std::deque<std::pair<int, int>> q;
    q.emplace_back(goal_row, goal_col);
    static constexpr int dr[4] = {-1, 1, 0, 0};
    static constexpr int dc[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop_front();
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr < 0 || nr >= level.rows || nc < 0 || nc >= level.cols) continue;
            if (level.walls[nr][nc]) continue;
            if (s.boxes[nr][nc] != '\0') continue;
            if (d[nr][nc] != kInf) continue;
            d[nr][nc] = d[r][c] + 1;
            q.emplace_back(nr, nc);
        }
    }
    return d;
}

}  // namespace

Pibt::Pibt(const State& start_state, const Level& level)
    : level_(level), start_(start_state)
{
}

std::vector<std::vector<int>> Pibt::plan(
    const std::vector<std::pair<int, int>>& targets,
    int max_timesteps)
{
    const int num_agents = static_cast<int>(start_.agent_rows.size());
    if (num_agents == 0) return {};
    if (static_cast<int>(targets.size()) != num_agents) return {};

    // Which agents have a goal?
    std::vector<char> has_goal(num_agents, 0);
    bool any_off_goal = false;
    for (int a = 0; a < num_agents; ++a) {
        if (targets[a].first < 0) continue;
        has_goal[a] = 1;
        if (start_.agent_rows[a] != targets[a].first
            || start_.agent_cols[a] != targets[a].second) {
            any_off_goal = true;
        }
    }
    if (!any_off_goal) return {};  // trivially solved; nothing to do

    // Per-agent box-aware distance fields from each goal cell.
    std::vector<std::vector<std::vector<int>>> dist_to_goal(num_agents);
    for (int a = 0; a < num_agents; ++a) {
        if (!has_goal[a]) continue;
        dist_to_goal[a] = box_aware_bfs(level_, start_,
                                        targets[a].first, targets[a].second);
        const int cur = dist_to_goal[a][start_.agent_rows[a]][start_.agent_cols[a]];
        if (cur == kInf) {
            // Goal unreachable under current box positions; let the caller
            // fall through to relocation.
            return {};
        }
    }

    State local = start_;
    std::vector<std::vector<int>> joint_plan;

    std::vector<double> priority(num_agents, 0.0);
    for (int a = 0; a < num_agents; ++a) {
        if (!has_goal[a]) continue;
        const int d = dist_to_goal[a][local.agent_rows[a]][local.agent_cols[a]];
        priority[a] = static_cast<double>(d == kInf ? 0 : d);
    }

    auto all_at_goal = [&](const State& s) {
        for (int a = 0; a < num_agents; ++a) {
            if (!has_goal[a]) continue;
            if (s.agent_rows[a] != targets[a].first
                || s.agent_cols[a] != targets[a].second) return false;
        }
        return true;
    };

    std::unordered_set<std::string> visited_joint;
    visited_joint.insert(agent_state_key(local));
    int repeat_streak = 0;

    for (int step = 0; step < max_timesteps; ++step) {
        if (all_at_goal(local)) break;

        std::vector<int> order(num_agents);
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(),
                         [&](int a, int b) { return priority[a] > priority[b]; });

        std::vector<int> chosen_action(num_agents, -1);
        std::vector<std::pair<int, int>> next_cell(num_agents, {-1, -1});
        std::set<std::pair<int, int>> reserved;

        std::function<bool(int, int)> pibt_step = [&](int agent, int parent) -> bool {
            struct Cand { int action; int dist; int nr; int nc; };
            std::vector<Cand> cands;
            const int cur_r = local.agent_rows[agent];
            const int cur_c = local.agent_cols[agent];
            const bool has_target = has_goal[agent] != 0;
            int target_r = -1, target_c = -1;
            if (has_target) {
                target_r = targets[agent].first;
                target_c = targets[agent].second;
            }
            for (int ai : kActionIndices) {
                const int nr = cur_r + kAdr[ai];
                const int nc = cur_c + kAdc[ai];
                if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols) continue;
                if (level_.walls[nr][nc]) continue;
                if (ai != kNoOpIndex && local.boxes[nr][nc] != '\0') continue;
                int dist;
                if (has_target) {
                    dist = dist_to_goal[agent][nr][nc];
                } else {
                    // Agents with no goal prefer NoOp, else move arbitrarily
                    // to clear corridors for goal-bearing peers.
                    dist = (ai == kNoOpIndex) ? 0 : 1;
                }
                if (dist == kInf) continue;
                cands.push_back({ai, dist, nr, nc});
            }
            std::stable_sort(cands.begin(), cands.end(),
                             [&](const Cand& a, const Cand& b) {
                if (a.dist != b.dist) return a.dist < b.dist;
                // Tie-break: if the agent is sitting on its goal, prefer
                // NoOp to anchor; otherwise prefer move.
                const bool a_at_goal_noop = has_target && a.action == kNoOpIndex
                    && cur_r == target_r && cur_c == target_c;
                const bool b_at_goal_noop = has_target && b.action == kNoOpIndex
                    && cur_r == target_r && cur_c == target_c;
                if (a_at_goal_noop != b_at_goal_noop) return a_at_goal_noop;
                if ((a.action == kNoOpIndex) != (b.action == kNoOpIndex))
                    return a.action != kNoOpIndex;
                return a.action < b.action;
            });

            for (const Cand& cand : cands) {
                const std::pair<int, int> dest{cand.nr, cand.nc};
                if (reserved.count(dest)) continue;
                // Swap conflict: never move INTO the parent's cell.
                if (parent >= 0
                    && cand.nr == local.agent_rows[parent]
                    && cand.nc == local.agent_cols[parent]) continue;
                // Occupied by another unscheduled agent?
                int occupant = -1;
                for (int other = 0; other < num_agents; ++other) {
                    if (other == agent) continue;
                    if (chosen_action[other] != -1) continue;
                    if (local.agent_rows[other] == cand.nr
                        && local.agent_cols[other] == cand.nc) {
                        occupant = other; break;
                    }
                }
                if (occupant != -1) {
                    reserved.insert(dest);
                    chosen_action[agent] = cand.action;
                    next_cell[agent] = dest;
                    if (pibt_step(occupant, agent)) return true;
                    reserved.erase(dest);
                    chosen_action[agent] = -1;
                    next_cell[agent] = {-1, -1};
                    continue;
                }
                reserved.insert(dest);
                chosen_action[agent] = cand.action;
                next_cell[agent] = dest;
                return true;
            }
            // Forced NoOp on our own cell.
            const std::pair<int, int> own{cur_r, cur_c};
            if (reserved.count(own)) return false;
            reserved.insert(own);
            chosen_action[agent] = kNoOpIndex;
            next_cell[agent] = own;
            return true;
        };

        for (int agent : order) {
            if (chosen_action[agent] != -1) continue;
            if (!pibt_step(agent, -1)) {
                chosen_action[agent] = kNoOpIndex;
                const std::pair<int, int> own{
                    local.agent_rows[agent], local.agent_cols[agent]};
                reserved.insert(own);
                next_cell[agent] = own;
            }
        }

        std::vector<int> joint(num_agents, kNoOpIndex);
        for (int a = 0; a < num_agents; ++a) {
            joint[a] = chosen_action[a] == -1 ? kNoOpIndex : chosen_action[a];
        }

        bool any_move = false;
        for (int a : joint) if (a != kNoOpIndex) { any_move = true; break; }
        if (!any_move) {
            bool any_off = false;
            for (int a = 0; a < num_agents; ++a) {
                if (!has_goal[a]) continue;
                if (local.agent_rows[a] != targets[a].first
                    || local.agent_cols[a] != targets[a].second) {
                    priority[a] += 1.0;
                    any_off = true;
                }
            }
            if (!any_off) break;
            if (++repeat_streak > 4) return {};
            continue;
        }

        State next = local;
        if (!next.apply_joint(joint)) {
            return {};
        }

        const std::string key = agent_state_key(next);
        if (!visited_joint.insert(key).second) {
            if (++repeat_streak > 6) return {};
            for (int a = 0; a < num_agents; ++a) {
                if (!has_goal[a]) continue;
                if (local.agent_rows[a] != targets[a].first
                    || local.agent_cols[a] != targets[a].second) {
                    priority[a] += 1.5;
                }
            }
        } else {
            repeat_streak = 0;
        }

        local = std::move(next);
        joint_plan.push_back(std::move(joint));

        for (int a = 0; a < num_agents; ++a) {
            if (!has_goal[a]) continue;
            if (local.agent_rows[a] == targets[a].first
                && local.agent_cols[a] == targets[a].second) {
                priority[a] = 0.0;
            } else {
                priority[a] += 1.0;
            }
        }
    }

    if (!all_at_goal(local)) return {};
    return joint_plan;
}

}  // namespace aimas
