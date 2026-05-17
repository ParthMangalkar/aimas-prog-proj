#include "ALNS.h"

#include "../Domain.h"
#include "ALNSDestroy.h"
#include "ReservationTable.h"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace mapf {

namespace {

constexpr double ALNS_REWARD     = 1.0;
constexpr double ALNS_DECAY      = 0.95;
constexpr double ALNS_MIN_WEIGHT = 0.1;
constexpr int    ALNS_NMIN       = 3;
constexpr int    ALNS_NMAX       = 5;

// Replay an action list against the reservation table, mirroring the
// bookkeeping used by ALNSRepair / ECBS.
void reserve_path_into(int agent_id,
                       int sr, int sc, int sbr, int sbc,
                       const std::vector<const Action*>& actions,
                       ReservationTable& tbl)
{
    int ar = sr, ac = sc, br = sbr, bc = sbc;
    int t = 0;
    tbl.reserve_cell(ar, ac, t, t, agent_id);
    if (br >= 0) tbl.reserve_cell(br, bc, t, t, agent_id);

    for (const Action* act : actions) {
        const int prev_ar = ar, prev_ac = ac;
        const int prev_br = br, prev_bc = bc;
        ++t;
        switch (act->type) {
            case ActionType::NoOp: break;
            case ActionType::Move:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                break;
            case ActionType::Push:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                br += act->box_row_delta;
                bc += act->box_col_delta;
                break;
            case ActionType::Pull:
                ar += act->agent_row_delta;
                ac += act->agent_col_delta;
                br = prev_ar;
                bc = prev_ac;
                break;
        }
        tbl.reserve_cell(ar, ac, t, t, agent_id);
        if (br >= 0) tbl.reserve_cell(br, bc, t, t, agent_id);
        if (prev_ar != ar || prev_ac != ac) {
            tbl.reserve_edge(prev_ar, prev_ac, ar, ac, t, agent_id);
        }
        if (prev_br >= 0 && br >= 0 && (prev_br != br || prev_bc != bc)) {
            tbl.reserve_edge(prev_br, prev_bc, br, bc, t, agent_id);
        }
    }
}

ReservationTable buildTableFrom(const ALNSSolution& sol)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows ? static_cast<int>(State::walls[0].size()) : 0;
    int max_t = 0;
    for (const auto& kv : sol.paths) {
        max_t = std::max(max_t, static_cast<int>(kv.second.size()));
    }
    max_t = std::max(max_t + 32, 64);

    ReservationTable tbl(rows, cols, max_t);
    for (const auto& kv : sol.paths) {
        const int aid = kv.first;
        auto its = sol.agentStarts.find(aid);
        if (its == sol.agentStarts.end()) continue;
        int br = -1, bc = -1;
        auto itb = sol.boxStarts.find(aid);
        if (itb != sol.boxStarts.end()) { br = itb->second.first; bc = itb->second.second; }
        reserve_path_into(aid,
                          its->second.first, its->second.second,
                          br, bc, kv.second, tbl);
    }
    return tbl;
}

std::vector<int> callDestroy(int op,
                             const ALNSSolution& sol,
                             ReservationTable& tbl,
                             int k,
                             std::mt19937& rng)
{
    switch (op) {
        case 0: return collisionBasedDestroy(sol.paths, sol.agentStarts, sol.boxStarts, tbl, k, rng);
        case 1: return highCostDestroy        (sol.paths, sol.agentStarts, sol.boxStarts, tbl, k, rng);
        case 2: return randomDestroy          (sol.paths, sol.agentStarts, sol.boxStarts, tbl, k, rng);
    }
    return {};
}

}  // namespace

ALNSSolution alnsRun(const ALNSSolution& initial,
                     const std::vector<Assignment>& assignments,
                     const State& state,
                     int timeLimitMs,
                     std::mt19937& rng)
{
    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    auto elapsed_ms = [&]() {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - start).count());
    };

    ALNSSolution best = initial;
    std::vector<double> weights{1.0, 1.0, 1.0};

    long long iters = 0, accepted = 0, repair_failed = 0;

    std::uniform_int_distribution<int> kdist(ALNS_NMIN, ALNS_NMAX);

    while (elapsed_ms() < timeLimitMs) {
        ++iters;

        const int k = std::min<int>(kdist(rng), static_cast<int>(best.paths.size()));
        if (k <= 0) break;

        std::discrete_distribution<int> opdist(weights.begin(), weights.end());
        const int op = opdist(rng);

        ReservationTable workTable = buildTableFrom(best);

        std::vector<int> released = callDestroy(op, best, workTable, k, rng);
        if (released.empty()) {
            weights[op] *= ALNS_DECAY;
        } else {
            auto candidate = alnsRepair(released, best, assignments, workTable, state);
            if (!candidate) {
                ++repair_failed;
                weights[op] *= ALNS_DECAY;
            } else if (alnsAccept(*candidate, best)) {
                best = std::move(*candidate);
                weights[op] += ALNS_REWARD;
                ++accepted;
            } else {
                weights[op] *= ALNS_DECAY;
            }
        }

        // Clamp + renormalise so the discrete distribution stays well-formed.
        double sum = 0.0;
        for (double& w : weights) {
            w = std::max(w, ALNS_MIN_WEIGHT);
            sum += w;
        }
        if (sum > 0.0) {
            for (double& w : weights) w /= sum;
            // After normalising, re-clamp because a large operator can crush
            // the others below the floor; re-clamp + re-normalise once more.
            sum = 0.0;
            for (double& w : weights) {
                w = std::max(w, ALNS_MIN_WEIGHT);
                sum += w;
            }
            for (double& w : weights) w /= sum;
        }
    }

    std::cerr << "[ALNS] iters=" << iters
              << " accepted=" << accepted
              << " repair_failed=" << repair_failed
              << " final_cost=" << best.totalCost
              << " initial_cost=" << initial.totalCost
              << " weights=[" << weights[0]
              << "," << weights[1]
              << "," << weights[2] << "]\n";

    return best;
}

}  // namespace mapf
