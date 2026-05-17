#include "ALNSRepair.h"

#include "../Domain.h"
#include "BFSDistanceMap.h"
#include "SpaceTimeAStar.h"

#include <algorithm>

namespace mapf {

namespace {

// Reserve a freshly-replanned path in the local reservation table, mirroring
// the cell + edge bookkeeping used by the rest of the MAPF stack. Box cells
// are reserved whenever the agent owns one (br >= 0).
void reserve_path(int agent_id,
                  int sr, int sc, int sbr, int sbc,
                  const std::vector<const Action*>& actions,
                  ReservationTable& tbl)
{
    int ar = sr, ac = sc, br = sbr, bc = sbc;
    int t = 0;
    tbl.reserve_cell(ar, ac, t, t, agent_id);
    if (br >= 0) tbl.reserve_cell(br, bc, t, t, agent_id);

    for (const Action* act : actions) {
        const int prev_ar = ar, prev_ac = ac;
        const int prev_br = br, prev_bc = bc;
        ++t;
        switch (act->type) {
            case ActionType::NoOp: break;
            case ActionType::Move:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                break;
            case ActionType::Push:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                br += act->box_row_delta;
                bc += act->box_col_delta;
                break;
            case ActionType::Pull:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                br = prev_ar;
                bc = prev_ac;
                break;
        }
        tbl.reserve_cell(ar, ac, t, t, agent_id);
        if (br >= 0) tbl.reserve_cell(br, bc, t, t, agent_id);
        if (prev_ar != ar || prev_ac != ac) {
            tbl.reserve_edge(prev_ar, prev_ac, ar, ac, t, agent_id);
        }
        if (prev_br >= 0 && br >= 0 && (prev_br != br || prev_bc != bc)) {
            tbl.reserve_edge(prev_br, prev_bc, br, bc, t, agent_id);
        }
    }
}

const Assignment* find_assignment(const std::vector<Assignment>& assignments, int aid)
{
    for (const auto& a : assignments) {
        if (a.agentIdx == aid) return &a;
    }
    return nullptr;
}

Subtask subtask_from_assignment(const Assignment& asg)
{
    Subtask sub;
    sub.type        = SubtaskType::DeliverBox;
    sub.box_letter  = asg.task.boxChar;
    sub.box_start_r = asg.task.boxRow;
    sub.box_start_c = asg.task.boxCol;
    sub.box_goal_r  = asg.task.goalRow;
    sub.box_goal_c  = asg.task.goalCol;
    return sub;
}

}  // namespace

std::optional<ALNSSolution>
alnsRepair(const std::vector<int>& selectedAgentIds,
           const ALNSSolution& current,
           const std::vector<Assignment>& assignments,
           const ReservationTable& table,
           const State& /*state*/)
{
    // Local copy — caller's table is never mutated. Returning nullopt below
    // therefore leaves the caller's reservation state untouched.
    ReservationTable local = table;
    const int max_t = local.max_t() > 0
                    ? local.max_t()
                    : 300;

    PathMap  newPaths       = current.paths;
    StartMap newAgentStarts = current.agentStarts;
    StartMap newBoxStarts   = current.boxStarts;

    for (int aid : selectedAgentIds) {
        const Assignment* asg = find_assignment(assignments, aid);
        if (!asg) return std::nullopt;

        auto itStart = current.agentStarts.find(aid);
        if (itStart == current.agentStarts.end()) return std::nullopt;
        const int sr = itStart->second.first;
        const int sc = itStart->second.second;

        const Subtask sub = subtask_from_assignment(*asg);

        DistanceGrid d_goal      = bfs_from(sub.box_goal_r,  sub.box_goal_c);
        DistanceGrid d_box_start = bfs_from(sub.box_start_r, sub.box_start_c);

        // Per the Phase 3b spec, repair queries the local table for any
        // per-agent constraints. ReservationTable doesn't carry a separate
        // CT-style ConstraintSet (those live on the ECBS CT node), so we
        // pass an empty extra-constraint set here — every other agent's
        // restrictions are already encoded as reservations in `local`.
        ConstraintSet cs;
        STAResult r = sta_star(aid, sr, sc, sub, /*start_t=*/0,
                               local, d_goal, &d_box_start, max_t, cs);
        if (!r.ok) return std::nullopt;

        reserve_path(aid, sr, sc,
                     sub.box_start_r, sub.box_start_c,
                     r.actions, local);

        newPaths[aid]     = r.actions;
        newBoxStarts[aid] = { sub.box_start_r, sub.box_start_c };
        // agentStarts is invariant across repair: it always points at the
        // agent's initial map position, not its current path-head.
    }

    int totalCost = 0;
    for (const auto& kv : newPaths) {
        totalCost += static_cast<int>(kv.second.size());
    }

    return ALNSSolution{
        std::move(newPaths),
        std::move(newAgentStarts),
        std::move(newBoxStarts),
        totalCost
    };
}

bool alnsAccept(const ALNSSolution& candidate, const ALNSSolution& current)
{
    return candidate.totalCost < current.totalCost;
}

}  // namespace mapf
