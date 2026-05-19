#pragma once

#include "aimas/core.hpp"
#include "aimas/topology.hpp"

#include <chrono>
#include <vector>

namespace aimas {

// Single-box A* planner: given a State, plans a sequence of single-agent
// actions for one agent and one box, from current positions to a target box
// cell. All other agents and boxes are treated as static obstacles.
//
// Result is a vector of action indices (into actions()) describing the agent's
// moves; an empty vector means no plan was found within the limits. The caller
// is responsible for embedding these into a joint plan (NoOp for other agents)
// and validating via State::apply_joint.
class SingleBoxAStar {
public:
    SingleBoxAStar(const State& state, Topology& topology);

    std::vector<int> plan(
        int agent,
        char box,
        int box_row, int box_col,
        int goal_row, int goal_col,
        int expansion_cap = 400000,
        std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::time_point::max());

    // Plan an agent's move-only path to a target cell (no boxes touched).
    // Used by relocation/final-agent search.
    std::vector<int> plan_agent_to(
        int agent, int target_row, int target_col, int expansion_cap = 100000);

private:
    const State& state_;
    [[maybe_unused]] Topology& topology_;  // reserved for cached BFS heuristics
    const Level& level_;
};

}  // namespace aimas
