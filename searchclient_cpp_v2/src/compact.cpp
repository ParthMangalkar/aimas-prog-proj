#include "aimas/compact.hpp"

#include <cstdlib>
#include <iostream>
#include <utility>

namespace aimas {

namespace {

bool verbose_logging()
{
    static const bool v = []() {
        const char* env = std::getenv("V2_VERBOSE");
        return env != nullptr && env[0] != '\0' && env[0] != '0';
    }();
    return v;
}

// Replay `plan` from `s` (mutated in place). Returns true on full success.
bool replay_plan(State& s, const std::vector<std::vector<int>>& plan)
{
    for (const auto& joint : plan) {
        if (!s.apply_joint(joint)) return false;
    }
    return true;
}

// Predicate: can we add (agent_a, act_idx) to `current_joint` in `state`
// without violating applicability, box_from duplication, or
// State::conflicting()?
bool can_merge(
    const State& state,
    const std::vector<int>& current_joint,
    int agent_a,
    int act_idx)
{
    if (current_joint[agent_a] != kNoOpIndex) return false;

    const auto& tbl = actions();
    const Action& act = tbl[act_idx];
    if (!state.applicable(agent_a, act)) return false;

    const State::Delta d = state.delta_for(agent_a, act);
    if (d.moves_box) {
        for (std::size_t o = 0; o < current_joint.size(); ++o) {
            if (current_joint[o] == kNoOpIndex) continue;
            const State::Delta o_d = state.delta_for(static_cast<int>(o),
                                                     tbl[current_joint[o]]);
            if (o_d.moves_box
                && o_d.box_from_r == d.box_from_r
                && o_d.box_from_c == d.box_from_c) {
                return false;
            }
        }
    }

    std::vector<int> trial = current_joint;
    trial[agent_a] = act_idx;
    if (state.conflicting(trial)) return false;

    return true;
}

// Strategy A: aggressive lex-priority greedy. At each compacted step,
// try to advance every agent whose next pending action fits. Can reach
// much higher parallelism than the sliding window for independent
// agents, but may deadlock when one agent races ahead into a position
// that blocks another's future move. Returns `original` on any failure.
std::vector<std::vector<int>> compact_greedy(
    const std::vector<std::vector<int>>& original,
    const State& initial_state,
    const State& reference)
{
    const std::size_t N = initial_state.agent_rows.size();

    std::vector<std::vector<int>> per_agent(N);
    for (const auto& joint : original) {
        for (std::size_t a = 0; a < N; ++a) {
            if (joint[a] != kNoOpIndex) per_agent[a].push_back(joint[a]);
        }
    }

    std::vector<std::size_t> cursor(N, 0);
    State state = initial_state;
    std::vector<std::vector<int>> out;
    out.reserve(original.size());

    auto any_remaining = [&]() {
        for (std::size_t a = 0; a < N; ++a) {
            if (cursor[a] < per_agent[a].size()) return true;
        }
        return false;
    };

    while (any_remaining()) {
        if (out.size() >= original.size()) return original;

        std::vector<int> joint(N, kNoOpIndex);
        bool advanced = false;
        for (std::size_t a = 0; a < N; ++a) {
            if (cursor[a] >= per_agent[a].size()) continue;
            const int act_idx = per_agent[a][cursor[a]];
            if (can_merge(state, joint, static_cast<int>(a), act_idx)) {
                joint[a] = act_idx;
                ++cursor[a];
                advanced = true;
            }
        }

        if (!advanced) return original;
        if (!state.apply_joint(joint)) return original;
        out.push_back(std::move(joint));
    }

    StateEq eq;
    if (!eq(state, reference)) return original;
    return out;
}

// Strategy B: conservative sliding window that strictly preserves
// original temporal order. Walks the original plan in time order,
// accumulating actions into the in-flight joint; commits and starts a
// fresh joint when an action doesn't fit. Per-agent actions never
// advance earlier than their original step, which makes this strategy
// robust against the deadlocks greedy can hit but limits parallelism
// to consecutive cross-agent steps in the original plan.
std::vector<std::vector<int>> compact_sliding_window(
    const std::vector<std::vector<int>>& original,
    const State& initial_state,
    const State& reference)
{
    const std::size_t N = initial_state.agent_rows.size();

    State state = initial_state;
    std::vector<std::vector<int>> out;
    out.reserve(original.size());
    std::vector<int> current_joint(N, kNoOpIndex);
    bool have_pending = false;

    for (const auto& joint_k : original) {
        for (std::size_t a = 0; a < N; ++a) {
            const int act_idx = joint_k[a];
            if (act_idx == kNoOpIndex) continue;

            if (!can_merge(state, current_joint, static_cast<int>(a), act_idx)) {
                if (have_pending) {
                    if (!state.apply_joint(current_joint)) return original;
                    out.push_back(current_joint);
                    if (out.size() > original.size()) return original;
                    std::fill(current_joint.begin(), current_joint.end(),
                              kNoOpIndex);
                    have_pending = false;
                }
                if (!can_merge(state, current_joint, static_cast<int>(a),
                               act_idx)) {
                    return original;
                }
            }
            current_joint[a] = act_idx;
            have_pending = true;
        }
    }

    if (have_pending) {
        if (!state.apply_joint(current_joint)) return original;
        out.push_back(current_joint);
        if (out.size() > original.size()) return original;
    }

    StateEq eq;
    if (!eq(state, reference)) return original;
    return out;
}

}  // namespace

std::vector<std::vector<int>> compact_plan(
    const std::vector<std::vector<int>>& original,
    const State& initial_state)
{
    if (original.empty()) return original;
    const std::size_t N = initial_state.agent_rows.size();
    if (N == 0) return original;
    for (const auto& joint : original) {
        if (joint.size() != N) return original;
    }

    // Reference final state (used by both strategies for verification).
    State reference = initial_state;
    if (!replay_plan(reference, original)) return original;

    // Run both strategies; take the shortest verified result. Each
    // strategy returns `original` when it bails, so both calls are safe.
    auto via_greedy = compact_greedy(original, initial_state, reference);
    auto via_window = compact_sliding_window(original, initial_state,
                                             reference);

    const std::size_t orig_n = original.size();
    const std::size_t g_n    = via_greedy.size();
    const std::size_t w_n    = via_window.size();

    auto best = (g_n <= w_n) ? std::move(via_greedy) : std::move(via_window);
    const std::size_t best_n = best.size();

    if (verbose_logging()) {
        if (best_n < orig_n) {
            std::cerr << "[v2] Compacted plan: " << orig_n
                      << " -> " << best_n << " joint actions ("
                      << (100.0 * (orig_n - best_n)) / orig_n
                      << "% reduction; greedy=" << g_n
                      << ", window=" << w_n << ").\n";
        } else {
            std::cerr << "[v2] Compaction found no parallelism (kept "
                      << orig_n << " joint actions).\n";
        }
    }

    return best;
}

}  // namespace aimas
