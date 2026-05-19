#pragma once

#include "aimas/core.hpp"

#include <utility>
#include <vector>

namespace aimas {

// PIBT-style joint planner for the final agent-goal phase.
//
// Inspired by Okumura et al. 2019 ("Priority Inheritance with Backtracking",
// IJCAI). Boxes are treated as static obstacles; this matches the situation
// at the end of the delivery pipeline where all letter-goal boxes already
// sit on their goals. Agents with no numeric goal still participate but
// only move when needed to clear corridors.
//
// Properties:
//   * Transactional: builds the joint plan locally; caller commits via
//     apply_joint per step (which validates conflicts).
//   * Box-aware: per-agent BFS from the agent's goal cell treats other
//     boxes as walls. If any agent's goal is unreachable under static
//     boxes, plan() returns an empty plan so the caller can fall back to
//     box-aware relocation paths.
//   * Bounded: capped timesteps and joint-state visited set guard against
//     livelock; aging priority gives stalled agents a chance to lead.
//
// Returns a vector of joint actions (one per agent per step). Empty if
// PIBT could not bring every goal-bearing agent to its goal within the
// budget.
class Pibt {
public:
    Pibt(const State& start_state, const Level& level);

    // Plan agents to the provided per-agent goal cells.
    // targets[i] == (-1, -1) means agent i has no goal (free to move/clear).
    std::vector<std::vector<int>> plan(
        const std::vector<std::pair<int, int>>& targets,
        int  max_timesteps = 400);

private:
    const Level& level_;
    State        start_;
};

}  // namespace aimas
