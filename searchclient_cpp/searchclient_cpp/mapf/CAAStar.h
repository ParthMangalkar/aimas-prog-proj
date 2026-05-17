#pragma once

#include <memory>
#include <optional>
#include <vector>

struct Action;  // from Domain.h
struct State;
using StatePtr = std::shared_ptr<State>;

namespace mapf {

// Cooperative A* with random priority restarts. Returns the joint plan (per
// timestep, one Action* per agent) on success, or std::nullopt if no agent
// could be planned even partially.
//
// Phase 2d will introduce ECBS as the primary high-level solver and demote
// caastar_solve to the fallback path.
std::optional<std::vector<std::vector<const Action*>>>
caastar_solve(const StatePtr& initial);

}  // namespace mapf
