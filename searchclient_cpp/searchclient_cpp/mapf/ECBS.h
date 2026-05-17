#pragma once

#include "MapfTypes.h"
#include "TaskAllocator.h"

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

struct Action;  // from Domain.h
struct State;
using StatePtr = std::shared_ptr<State>;

namespace mapf {

// First-collision pair detected when replaying every agent's path.
// `(row, col, time)` is the conflict cell. For Edge conflicts, `(row, col)`
// is the destination cell that A moved into at time `time` (from time-1);
// the implicit swap with B is recovered from the agents' paths.
struct Conflict {
    int agentA = -1;
    int agentB = -1;
    int row = -1;
    int col = -1;
    int time = -1;
    enum class Type { Vertex, Edge } type = Type::Vertex;
};

// Constraint-tree node for vanilla CBS.
struct CTNode {
    std::unordered_map<int, std::vector<Constraint>>     constraints;
    std::unordered_map<int, std::vector<const Action*>>  paths;
    int totalCost = 0;
    int numConflicts = 0;  // cached for ECBS focal list (Phase 2e)
};

// Vanilla CBS / ECBS-without-focal high-level solver. Plans each agent's
// assigned subtask sequence with `sta_star` (using accumulated extra
// constraints) and resolves conflicts by branching on the earliest one.
//
// Returns a per-agent action list on success, or std::nullopt if the time
// budget is exhausted, the open list empties, or any agent could not be
// planned even at the root. Caller is expected to fall back to CAA*.
std::optional<std::unordered_map<int, std::vector<const Action*>>>
ecbs_solve(const StatePtr& state,
           const std::vector<Assignment>& assignments,
           int timeBudgetMs);

}  // namespace mapf
