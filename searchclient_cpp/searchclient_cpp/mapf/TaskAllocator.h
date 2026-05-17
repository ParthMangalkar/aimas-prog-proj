#pragma once

#include "MapfTypes.h"

#include <vector>

struct State;  // from Domain.h

namespace mapf {

struct BoxTask {
    int  boxRow  = -1, boxCol  = -1;
    int  goalRow = -1, goalCol = -1;
    char boxChar = '\0';
    int  goalId  = -1;
};

struct Assignment {
    int agentIdx = -1;
    BoxTask task;
};

std::vector<BoxTask> build_box_tasks(const State& s);
std::vector<Assignment> reallocate(const State& s,
                                   const std::vector<int>& agent_indices,
                                   const std::vector<BoxTask>& tasks);

// Hungarian task allocator. Returns one ordered subtask list per agent while
// keeping the public CAA*/ECBS/ALNS call-site contract stable.
std::vector<std::vector<Subtask>> assign_tasks(const State& s);

}  // namespace mapf
