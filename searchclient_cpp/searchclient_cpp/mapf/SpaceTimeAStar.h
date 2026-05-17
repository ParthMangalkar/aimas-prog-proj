#pragma once

#include "MapfTypes.h"
#include "ReservationTable.h"

#include <vector>

struct Action;  // from Domain.h

namespace mapf {

struct STAResult {
    bool ok = false;
    std::vector<const Action*> actions;
    int final_ar = 0, final_ac = 0;
    int final_br = -1, final_bc = -1;
};

// Single-agent space-time A* respecting a global reservation table.
// dist_to_box_start may be nullptr for ReachCell subtasks (no carried box).
STAResult sta_star(int agent_id,
                   int start_ar, int start_ac,
                   const Subtask& sub,
                   int start_t,
                   const ReservationTable& res,
                   const DistanceGrid& dist_to_goal,
                   const DistanceGrid* dist_to_box_start,
                   int max_t_horizon,
                   const ConstraintSet& extra_constraints = ConstraintSet{});

}  // namespace mapf
