#pragma once

// Phase 3c — ALNS adaptive main loop.
//
// Wraps Phase 3a (destroy) and Phase 3b (repair + accept) in the standard
// ALNS adaptive-weight loop with strict hill-climbing acceptance. Runs
// against a wall-clock budget and returns the best (lowest-cost) solution
// observed.

#include "ALNSRepair.h"
#include "TaskAllocator.h"

#include <random>
#include <vector>

struct State;

namespace mapf {

ALNSSolution alnsRun(const ALNSSolution& initial,
                     const std::vector<Assignment>& assignments,
                     const State& state,
                     int timeLimitMs,
                     std::mt19937& rng);

}  // namespace mapf
