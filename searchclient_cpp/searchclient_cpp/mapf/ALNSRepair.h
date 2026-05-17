#pragma once

// Phase 3b — ALNS repair + acceptance.
//
// Given a destroyed neighbourhood (the agent IDs returned by one of the
// destroy operators) and the post-release reservation table, replan each
// selected agent against a *local copy* of the table and merge the new
// paths into a fresh ALNSSolution. The caller's table is never mutated;
// the caller commits only on acceptance.

#include "ALNSDestroy.h"
#include "MapfTypes.h"
#include "ReservationTable.h"
#include "TaskAllocator.h"

#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

struct State;  // from Domain.h

namespace mapf {

struct ALNSSolution {
    PathMap   paths;
    StartMap  agentStarts;   // agentIdx → (r, c)
    StartMap  boxStarts;     // agentIdx → (r, c) for the box that agent owns
    int       totalCost = 0; // sum of path lengths
};

// Replan every agent in `selectedAgentIds` (in the given order) against a
// local copy of `table`. On success returns a new ALNSSolution merging the
// re-planned paths back into `current`. On any per-agent planning failure,
// returns std::nullopt and leaves the caller's table untouched.
std::optional<ALNSSolution>
alnsRepair(const std::vector<int>& selectedAgentIds,
           const ALNSSolution& current,
           const std::vector<Assignment>& assignments,
           const ReservationTable& table,
           const State& state);

// Strict hill-climbing acceptance: candidate is accepted iff it strictly
// improves on the current solution's totalCost.
bool alnsAccept(const ALNSSolution& candidate, const ALNSSolution& current);

}  // namespace mapf
