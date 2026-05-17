#pragma once

// Phase 3a — ALNS destroy operators.
//
// Each operator selects a neighbourhood of `neighbourhoodSize` agents from a
// per-agent action plan, releases those agents' cell + edge reservations from
// the shared ReservationTable so the repair phase can replan them, and
// returns the IDs of the selected agents.

#include "ReservationTable.h"

#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

struct Action;  // from Domain.h

namespace mapf {

using PathMap  = std::unordered_map<int, std::vector<const Action*>>;
using StartMap = std::unordered_map<int, std::pair<int, int>>;

// Top-K agents by overlapping (r,c,t) cells with any other agent's path
// (including the managed-box footprint replayed from boxStarts).
std::vector<int>
collisionBasedDestroy(const PathMap& paths,
                      const StartMap& agentStarts,
                      const StartMap& boxStarts,
                      ReservationTable& table,
                      int neighbourhoodSize,
                      std::mt19937& rng);

// Top-K agents by `path.size()` (longest paths first).
std::vector<int>
highCostDestroy(const PathMap& paths,
                const StartMap& agentStarts,
                const StartMap& boxStarts,
                ReservationTable& table,
                int neighbourhoodSize,
                std::mt19937& rng);

// Uniform random sample of K agents (without replacement).
std::vector<int>
randomDestroy(const PathMap& paths,
              const StartMap& agentStarts,
              const StartMap& boxStarts,
              ReservationTable& table,
              int neighbourhoodSize,
              std::mt19937& rng);

}  // namespace mapf
