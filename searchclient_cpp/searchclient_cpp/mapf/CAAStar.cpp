#include "CAAStar.h"

#include "../Domain.h"
#include "BFSDistanceMap.h"
#include "MapfTypes.h"
#include "ReservationTable.h"
#include "SpaceTimeAStar.h"
#include "TaskAllocator.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <tuple>
#include <utility>

namespace mapf {

namespace {

struct AgentPlanResult {
    bool ok = false;
    std::vector<const Action*> actions;
    std::vector<std::pair<int, int>> agent_pos;            // size = actions.size() + 1
    std::vector<std::tuple<int, int, int>> box_segments;   // (t, r, c) sequence per active box
};

AgentPlanResult plan_agent(int agent_id,
                           const State& s,
                           const std::vector<Subtask>& subtasks,
                           const ReservationTable& base_res,
                           int max_t_horizon)
{
    AgentPlanResult result;
    int cur_ar = s.agent_rows[agent_id];
    int cur_ac = s.agent_cols[agent_id];
    int cur_t = 0;
    result.agent_pos.push_back({cur_ar, cur_ac});

    for (std::size_t si = 0; si < subtasks.size(); ++si) {
        const Subtask& sub = subtasks[si];

        // Augment reservations with this agent's other-subtask boxes:
        // - subtasks before si: already-delivered, sit at their goal
        // - subtasks after  si: not yet picked up, sit at their start
        ReservationTable res = base_res;
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
                               res, d_goal, dbs_ptr, max_t_horizon);
        if (!r.ok) return result;

        // Replay actions to build trajectory.
        int ar = cur_ar, ac = cur_ac;
        int br = (sub.type == SubtaskType::DeliverBox) ? sub.box_start_r : -1;
        int bc = (sub.type == SubtaskType::DeliverBox) ? sub.box_start_c : -1;
        if (sub.type == SubtaskType::DeliverBox) {
            result.box_segments.push_back({cur_t, br, bc});
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
            cur_t++;
            result.agent_pos.push_back({nar, nac});
            if (br >= 0 && (nbr != br || nbc != bc)) {
                result.box_segments.push_back({cur_t, nbr, nbc});
            }
            ar = nar; ac = nac; br = nbr; bc = nbc;
        }
        cur_ar = ar; cur_ac = ac;
    }
    result.ok = true;
    return result;
}

void commit_agent(ReservationTable& res, int agent_id,
                  const AgentPlanResult& r,
                  const std::vector<Subtask>& subtasks)
{
    const int T = static_cast<int>(r.agent_pos.size()) - 1;
    for (int t = 0; t <= T; ++t) {
        res.reserve_cell(r.agent_pos[t].first, r.agent_pos[t].second, t, t, agent_id);
        // Vacation reservation: server forbids another agent entering a cell
        // we are leaving in the same joint action.
        if (t < T && r.agent_pos[t] != r.agent_pos[t + 1]) {
            res.reserve_cell(r.agent_pos[t].first, r.agent_pos[t].second,
                             t + 1, t + 1, agent_id);
        }
    }
    res.reserve_cell(r.agent_pos[T].first, r.agent_pos[T].second, T, res.max_t(), agent_id);
    for (int t = 1; t <= T; ++t) {
        if (r.agent_pos[t] != r.agent_pos[t - 1]) {
            res.reserve_edge(r.agent_pos[t - 1].first, r.agent_pos[t - 1].second,
                             r.agent_pos[t].first,     r.agent_pos[t].second,
                             t, agent_id);
        }
    }
    for (std::size_t i = 0; i < r.box_segments.size(); ++i) {
        const int t  = std::get<0>(r.box_segments[i]);
        const int rr = std::get<1>(r.box_segments[i]);
        const int cc = std::get<2>(r.box_segments[i]);
        const int t_end = (i + 1 < r.box_segments.size())
                          ? std::get<0>(r.box_segments[i + 1])
                          : res.max_t();
        res.reserve_cell(rr, cc, t, t_end, agent_id);
    }
    for (std::size_t i = 1; i < r.box_segments.size(); ++i) {
        const int t  = std::get<0>(r.box_segments[i]);
        const int pr = std::get<1>(r.box_segments[i - 1]);
        const int pc = std::get<2>(r.box_segments[i - 1]);
        const int rr = std::get<1>(r.box_segments[i]);
        const int cc = std::get<2>(r.box_segments[i]);
        if (pr != rr || pc != cc) {
            res.reserve_edge(pr, pc, rr, cc, t, agent_id);
        }
    }
    (void)subtasks;
}

}  // namespace

std::optional<std::vector<std::vector<const Action*>>>
caastar_solve(const StatePtr& initial)
{
    const int N = static_cast<int>(initial->agent_rows.size());
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());

    auto tasks = assign_tasks(*initial);
    std::cerr << "CAA*: assigned tasks per agent:";
    for (int a = 0; a < N; ++a) std::cerr << " a" << a << "=" << tasks[a].size();
    std::cerr << '\n';

    constexpr int kMaxAttempts = 5;
    const int max_t_horizon = std::max(300, rows * cols);

    std::vector<std::vector<bool>> assigned(rows, std::vector<bool>(cols, false));
    for (int a = 0; a < N; ++a) {
        for (const auto& st : tasks[a]) {
            if (st.type == SubtaskType::DeliverBox)
                assigned[st.box_start_r][st.box_start_c] = true;
        }
    }

    std::mt19937 rng(0xC0FFEEu);
    std::vector<std::vector<const Action*>> best_paths;
    bool found = false;
    int best_failures = N + 1;

    for (int attempt = 0; attempt < kMaxAttempts && !found; ++attempt) {
        std::vector<int> order(N);
        std::iota(order.begin(), order.end(), 0);
        if (attempt > 0) std::shuffle(order.begin(), order.end(), rng);

        std::vector<AgentPlanResult> committed(N);
        std::vector<bool> planned(N, false);
        std::vector<std::vector<const Action*>> paths(N);
        bool success = true;
        int failures = 0;

        for (int idx = 0; idx < N; ++idx) {
            const int agent_id = order[idx];
            ReservationTable res(rows, cols, max_t_horizon);

            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    if (initial->boxes[r][c] != '\0' && !assigned[r][c])
                        res.reserve_cell(r, c, 0, max_t_horizon, -1);
                }
            }
            for (int a = 0; a < N; ++a) {
                if (a == agent_id || planned[a]) continue;
                for (const auto& st : tasks[a]) {
                    if (st.type == SubtaskType::DeliverBox) {
                        res.reserve_cell(st.box_start_r, st.box_start_c,
                                         0, max_t_horizon, -1);
                    }
                }
            }
            for (int a = 0; a < N; ++a) {
                if (planned[a]) commit_agent(res, a, committed[a], tasks[a]);
            }
            for (int a = 0; a < N; ++a) {
                if (a == agent_id || planned[a]) continue;
                res.reserve_cell(initial->agent_rows[a], initial->agent_cols[a],
                                 0, 0, a);
            }

            AgentPlanResult r = plan_agent(agent_id, *initial, tasks[agent_id],
                                           res, max_t_horizon);
            if (!r.ok) {
                std::cerr << "CAA* attempt " << (attempt + 1)
                          << ": agent " << agent_id << " failed.\n";
                success = false;
                failures++;
                r.ok = true;
                r.actions.clear();
                r.agent_pos.clear();
                r.agent_pos.push_back({initial->agent_rows[agent_id],
                                       initial->agent_cols[agent_id]});
                r.box_segments.clear();
            }
            committed[agent_id] = std::move(r);
            paths[agent_id] = committed[agent_id].actions;
            planned[agent_id] = true;
        }

        if (success) {
            best_paths = std::move(paths);
            found = true;
            break;
        } else if (failures < best_failures) {
            best_failures = failures;
            best_paths = std::move(paths);
        }
    }

    if (best_paths.empty() && !found) return std::nullopt;
    if (!found) {
        std::cerr << "CAA*: no fully-successful attempt; submitting best partial.\n";
    }

    int T = 0;
    for (const auto& p : best_paths) T = std::max(T, static_cast<int>(p.size()));
    std::vector<std::vector<const Action*>> joint(
        T, std::vector<const Action*>(N, &ACTIONS[0]));
    for (int a = 0; a < N; ++a) {
        for (int t = 0; t < static_cast<int>(best_paths[a].size()); ++t) {
            joint[t][a] = best_paths[a][t];
        }
    }
    return joint;
}

}  // namespace mapf
