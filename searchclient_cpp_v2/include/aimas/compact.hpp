// Post-processing plan compaction.
//
// The v2 solver's box-delivery layers (deliver_task, scatter, eviction,
// relocation) are all single-agent A* planners that emit one mover per
// server step and pad the rest with NoOp. That keeps each push verified
// in isolation, but visually the GUI shows only one agent moving at a
// time over long stretches.
//
// compact_plan() takes the produced joint-action plan and greedily
// re-interleaves it so multiple agents move in the same server step
// whenever their actions don't conflict. Per-agent action order is
// preserved (causality), every step is re-validated against the
// canonical State::applicable + State::conflicting rules, and the result
// is rejected (falling back to the original plan) if the final state
// doesn't exactly match the original's final state.
//
// Properties (when a compacted plan is returned):
//   * Replay from initial_state succeeds (every apply_joint returns true).
//   * Final state equals the original plan's final state under StateEq.
//   * Compacted plan size <= original plan size.
//   * Per-agent subsequences of non-NoOp actions are preserved in order.
//
// Solve count is preserved by construction — any failure of the above
// returns the original plan unchanged.

#pragma once

#include "aimas/core.hpp"

#include <vector>

namespace aimas {

std::vector<std::vector<int>> compact_plan(
    const std::vector<std::vector<int>>& original,
    const State& initial_state);

}  // namespace aimas
