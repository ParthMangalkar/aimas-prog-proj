#include "TaskAllocator.h"

#include "../Domain.h"
#include "BFSDistanceMap.h"

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <set>
#include <utility>

namespace mapf {

namespace {

constexpr long long kDummyCost = 1'000'000'000LL;
constexpr long long kForbiddenCost = 4'000'000'000LL;

struct BoxInstance { char letter; int r, c; };
struct GoalInstance { char ch; int r, c, goal_id; };

using DistanceCache = std::map<std::pair<int, int>, DistanceGrid>;

bool is_box_char(char ch)
{
    return ch >= 'A' && ch <= 'Z';
}

const DistanceGrid& cached_bfs(DistanceCache& cache, int r, int c)
{
    const auto key = std::make_pair(r, c);
    auto it = cache.find(key);
    if (it == cache.end()) {
        it = cache.emplace(key, bfs_from(r, c)).first;
    }
    return it->second;
}

long long safe_distance(int distance)
{
    return distance == kInf ? kForbiddenCost : static_cast<long long>(distance);
}

std::vector<int> hungarian_minimize(const std::vector<std::vector<long long>>& cost)
{
    const int n = static_cast<int>(cost.size());
    std::vector<long long> u(n + 1), v(n + 1);
    std::vector<int> p(n + 1), way(n + 1);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<long long> minv(n + 1, kForbiddenCost * 4);
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            long long delta = kForbiddenCost * 4;
            int j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const long long cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> row_to_col(n, -1);
    for (int j = 1; j <= n; ++j) {
        if (p[j] != 0) row_to_col[p[j] - 1] = j - 1;
    }
    return row_to_col;
}

std::vector<std::pair<int, int>>
partial_match(int real_rows,
              int real_cols,
              const std::function<long long(int, int)>& real_cost)
{
    if (real_rows <= 0 || real_cols <= 0) return {};

    const int n = real_rows + real_cols;
    std::vector<std::vector<long long>> cost(
        n, std::vector<long long>(n, 0));

    for (int r = 0; r < real_rows; ++r) {
        for (int c = 0; c < real_cols; ++c) {
            cost[r][c] = std::min(real_cost(r, c), kForbiddenCost);
        }
        for (int c = real_cols; c < n; ++c) {
            cost[r][c] = kDummyCost;
        }
    }
    for (int r = real_rows; r < n; ++r) {
        for (int c = 0; c < real_cols; ++c) {
            cost[r][c] = kDummyCost;
        }
    }

    const std::vector<int> row_to_col = hungarian_minimize(cost);
    std::vector<std::pair<int, int>> pairs;
    for (int r = 0; r < real_rows; ++r) {
        const int c = row_to_col[r];
        if (0 <= c && c < real_cols && cost[r][c] < kDummyCost) {
            pairs.push_back({r, c});
        }
    }
    return pairs;
}

Subtask to_subtask(const BoxTask& task)
{
    Subtask st;
    st.type = SubtaskType::DeliverBox;
    st.box_letter  = task.boxChar;
    st.box_start_r = task.boxRow;
    st.box_start_c = task.boxCol;
    st.box_goal_r  = task.goalRow;
    st.box_goal_c  = task.goalCol;
    return st;
}

std::vector<Assignment>
reallocate_from_positions(const std::vector<int>& agent_indices,
                          const std::vector<std::pair<int, int>>& agent_positions,
                          const std::vector<BoxTask>& tasks,
                          DistanceCache& cache)
{
    const auto real_cost = [&](int agent_pos_idx, int task_idx) -> long long {
        const int agent = agent_indices[agent_pos_idx];
        const BoxTask& task = tasks[task_idx];
        if (agent < 0 || agent >= static_cast<int>(State::agent_colors.size())) {
            return kForbiddenCost;
        }
        if (!is_box_char(task.boxChar)) return kForbiddenCost;
        const Color needed = State::box_colors[task.boxChar - 'A'];
        if (State::agent_colors[agent] != needed) return kForbiddenCost;

        const auto& d_box = cached_bfs(cache, task.boxRow, task.boxCol);
        const auto& d_goal = cached_bfs(cache, task.goalRow, task.goalCol);
        const auto [ar, ac] = agent_positions[agent_pos_idx];
        const long long agent_to_box = safe_distance(d_box[ar][ac]);
        const long long box_to_goal = safe_distance(d_goal[task.boxRow][task.boxCol]);
        if (agent_to_box >= kForbiddenCost || box_to_goal >= kForbiddenCost) {
            return kForbiddenCost;
        }
        return std::min(agent_to_box + box_to_goal, kForbiddenCost);
    };

    std::vector<Assignment> assignments;
    for (const auto& [agent_pos_idx, task_idx] :
         partial_match(static_cast<int>(agent_indices.size()),
                       static_cast<int>(tasks.size()),
                       real_cost)) {
        Assignment assignment;
        assignment.agentIdx = agent_indices[agent_pos_idx];
        assignment.task = tasks[task_idx];
        assignments.push_back(assignment);
    }
    return assignments;
}

}  // namespace

std::vector<BoxTask> build_box_tasks(const State& s)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    DistanceCache cache;

    std::vector<BoxInstance> boxes;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const char ch = s.boxes[r][c];
            if (is_box_char(ch) && State::goals[r][c] != ch) {
                boxes.push_back({ch, r, c});
            }
        }
    }

    std::vector<GoalInstance> goals;
    int goal_id = 0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const char g = State::goals[r][c];
            if (is_box_char(g) && s.boxes[r][c] != g) {
                goals.push_back({g, r, c, goal_id++});
            }
        }
    }

    std::array<std::vector<int>, 26> goals_by_letter;
    std::array<std::vector<int>, 26> boxes_by_letter;
    for (int i = 0; i < static_cast<int>(goals.size()); ++i) {
        goals_by_letter[goals[i].ch - 'A'].push_back(i);
    }
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        boxes_by_letter[boxes[i].letter - 'A'].push_back(i);
    }

    std::vector<BoxTask> box_tasks;
    for (int letter = 0; letter < 26; ++letter) {
        const auto& letter_goals = goals_by_letter[letter];
        const auto& letter_boxes = boxes_by_letter[letter];
        const auto real_cost = [&](int goal_pos, int box_pos) -> long long {
            const auto& goal = goals[letter_goals[goal_pos]];
            const auto& box = boxes[letter_boxes[box_pos]];
            const auto& d_goal = cached_bfs(cache, goal.r, goal.c);
            return safe_distance(d_goal[box.r][box.c]);
        };

        for (const auto& [goal_pos, box_pos] :
             partial_match(static_cast<int>(letter_goals.size()),
                           static_cast<int>(letter_boxes.size()),
                           real_cost)) {
            const auto& goal = goals[letter_goals[goal_pos]];
            const auto& box = boxes[letter_boxes[box_pos]];
            box_tasks.push_back({box.r, box.c, goal.r, goal.c, box.letter, goal.goal_id});
        }
    }
    std::sort(box_tasks.begin(), box_tasks.end(), [](const BoxTask& lhs, const BoxTask& rhs) {
        return lhs.goalId < rhs.goalId;
    });
    return box_tasks;
}

std::vector<Assignment> reallocate(const State& s,
                                   const std::vector<int>& agent_indices,
                                   const std::vector<BoxTask>& tasks)
{
    std::vector<std::pair<int, int>> positions;
    positions.reserve(agent_indices.size());
    for (const int agent : agent_indices) {
        if (agent < 0 || agent >= static_cast<int>(s.agent_rows.size())) {
            positions.push_back({-1, -1});
        } else {
            positions.push_back({s.agent_rows[agent], s.agent_cols[agent]});
        }
    }
    DistanceCache cache;
    return reallocate_from_positions(agent_indices, positions, tasks, cache);
}

std::vector<std::vector<Subtask>> assign_tasks(const State& s)
{
    const int N = static_cast<int>(s.agent_rows.size());
    std::vector<std::vector<Subtask>> tasks(N);
    std::vector<BoxTask> remaining = build_box_tasks(s);
    DistanceCache cache;

    std::vector<int> active_agents(N);
    for (int a = 0; a < N; ++a) active_agents[a] = a;

    std::vector<std::pair<int, int>> virtual_positions;
    virtual_positions.reserve(N);
    for (int a = 0; a < N; ++a) {
        virtual_positions.push_back({s.agent_rows[a], s.agent_cols[a]});
    }

    while (!remaining.empty()) {
        const std::vector<Assignment> round =
            reallocate_from_positions(active_agents, virtual_positions, remaining, cache);
        if (round.empty()) break;

        std::set<int> assigned_goal_ids;
        for (const Assignment& assignment : round) {
            const int a = assignment.agentIdx;
            if (a < 0 || a >= N) continue;
            tasks[a].push_back(to_subtask(assignment.task));
            virtual_positions[a] = {assignment.task.goalRow, assignment.task.goalCol};
            assigned_goal_ids.insert(assignment.task.goalId);
        }
        if (assigned_goal_ids.empty()) break;

        remaining.erase(
            std::remove_if(remaining.begin(), remaining.end(),
                           [&](const BoxTask& task) {
                               return assigned_goal_ids.count(task.goalId) > 0;
                           }),
            remaining.end());
    }

    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const char g = State::goals[r][c];
            if (g < '0' || g > '9') continue;
            const int a = g - '0';
            if (a < 0 || a >= N) continue;
            Subtask st;
            st.type = SubtaskType::ReachCell;
            st.target_r = r;
            st.target_c = c;
            tasks[a].push_back(st);
        }
    }

    return tasks;
}

}  // namespace mapf
