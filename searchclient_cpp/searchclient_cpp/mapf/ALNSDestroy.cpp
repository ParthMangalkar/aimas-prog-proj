#include "ALNSDestroy.h"

#include "../Domain.h"

#include <algorithm>
#include <iterator>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mapf {

namespace {

// Pack (r, c, t) into a single 64-bit key for fast hash-set lookup.
inline long long pack(int r, int c, int t)
{
    return (static_cast<long long>(r) & 0xFFFF) << 48
         | (static_cast<long long>(c) & 0xFFFF) << 32
         | (static_cast<long long>(t) & 0xFFFFFFFF);
}

// Replay one agent's action list from its start positions and emit every
// (agent_r, agent_c, t) and (box_r, box_c, t) tuple it occupies. Mirrors the
// semantics used by ECBS / SpaceTimeAStar: Push moves agent into the box's
// cell; Pull moves the box into the agent's previous cell. boxStart of
// {-1,-1} means the agent has no managed box (and never picks one up here —
// caller is responsible for any subtask-aware replays).
std::vector<long long>
replay_path(int agentId,
            const std::vector<const Action*>& actions,
            const StartMap& agentStarts,
            const StartMap& boxStarts)
{
    std::vector<long long> out;
    auto itA = agentStarts.find(agentId);
    if (itA == agentStarts.end()) return out;
    int ar = itA->second.first;
    int ac = itA->second.second;
    int br = -1, bc = -1;
    auto itB = boxStarts.find(agentId);
    if (itB != boxStarts.end()) {
        br = itB->second.first;
        bc = itB->second.second;
    }

    out.reserve(actions.size() + 1);
    int t = 0;
    out.push_back(pack(ar, ac, t));
    if (br >= 0) out.push_back(pack(br, bc, t));

    for (const Action* act : actions) {
        ++t;
        int nar = ar, nac = ac, nbr = br, nbc = bc;
        switch (act->type) {
            case ActionType::NoOp: break;
            case ActionType::Move:
                nar = ar + act->agent_row_delta;
                nac = ac + act->agent_col_delta;
                break;
            case ActionType::Push:
                nar = ar + act->agent_row_delta;
                nac = ac + act->agent_col_delta;
                nbr = br + act->box_row_delta;
                nbc = bc + act->box_col_delta;
                break;
            case ActionType::Pull:
                nar = ar + act->agent_row_delta;
                nac = ac + act->agent_col_delta;
                nbr = ar;
                nbc = ac;
                break;
        }
        ar = nar; ac = nac; br = nbr; bc = nbc;
        out.push_back(pack(ar, ac, t));
        if (br >= 0) out.push_back(pack(br, bc, t));
    }
    return out;
}

void release_all(const std::vector<int>& ids, ReservationTable& table)
{
    for (int a : ids) table.release(a);
}

std::vector<int> sorted_agent_ids(const PathMap& paths)
{
    std::vector<int> ids;
    ids.reserve(paths.size());
    for (const auto& kv : paths) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    return ids;
}

int clamp_K(int requested, int available)
{
    if (requested < 0) return 0;
    if (requested > available) return available;
    return requested;
}

}  // namespace

std::vector<int>
collisionBasedDestroy(const PathMap& paths,
                      const StartMap& agentStarts,
                      const StartMap& boxStarts,
                      ReservationTable& table,
                      int neighbourhoodSize,
                      std::mt19937& rng)
{
    const auto ids = sorted_agent_ids(paths);
    const int K = clamp_K(neighbourhoodSize, static_cast<int>(ids.size()));
    if (K == 0) return {};

    // Replay every agent's path into a (r,c,t) set, then count how many of
    // each agent's tuples appear in any *other* agent's set.
    std::unordered_map<int, std::unordered_set<long long>> footprint;
    footprint.reserve(ids.size() * 2);
    for (int a : ids) {
        auto it = paths.find(a);
        if (it == paths.end()) continue;
        auto v = replay_path(a, it->second, agentStarts, boxStarts);
        footprint[a].insert(v.begin(), v.end());
    }

    std::vector<std::pair<int, int>> overlap_counts;  // (overlap, agentId)
    overlap_counts.reserve(ids.size());
    for (int a : ids) {
        int overlap = 0;
        const auto& mine = footprint[a];
        for (int b : ids) {
            if (b == a) continue;
            const auto& theirs = footprint[b];
            for (long long key : mine) {
                if (theirs.count(key)) ++overlap;
            }
        }
        overlap_counts.emplace_back(overlap, a);
    }

    // Sort by overlap desc; tie-break by random shuffle within equal counts
    // so ALNS doesn't always pick the same agents.
    std::shuffle(overlap_counts.begin(), overlap_counts.end(), rng);
    std::stable_sort(overlap_counts.begin(), overlap_counts.end(),
                     [](const std::pair<int, int>& x, const std::pair<int, int>& y) {
                         return x.first > y.first;
                     });

    std::vector<int> selected;
    selected.reserve(K);
    for (int i = 0; i < K; ++i) selected.push_back(overlap_counts[i].second);

    release_all(selected, table);
    return selected;
}

std::vector<int>
highCostDestroy(const PathMap& paths,
                const StartMap& /*agentStarts*/,
                const StartMap& /*boxStarts*/,
                ReservationTable& table,
                int neighbourhoodSize,
                std::mt19937& rng)
{
    const auto ids = sorted_agent_ids(paths);
    const int K = clamp_K(neighbourhoodSize, static_cast<int>(ids.size()));
    if (K == 0) return {};

    std::vector<std::pair<int, int>> by_cost;  // (path.size(), agentId)
    by_cost.reserve(ids.size());
    for (int a : ids) {
        auto it = paths.find(a);
        const int cost = (it == paths.end())
                       ? 0
                       : static_cast<int>(it->second.size());
        by_cost.emplace_back(cost, a);
    }

    std::shuffle(by_cost.begin(), by_cost.end(), rng);
    std::stable_sort(by_cost.begin(), by_cost.end(),
                     [](const std::pair<int, int>& x, const std::pair<int, int>& y) {
                         return x.first > y.first;
                     });

    std::vector<int> selected;
    selected.reserve(K);
    for (int i = 0; i < K; ++i) selected.push_back(by_cost[i].second);

    release_all(selected, table);
    return selected;
}

std::vector<int>
randomDestroy(const PathMap& paths,
              const StartMap& /*agentStarts*/,
              const StartMap& /*boxStarts*/,
              ReservationTable& table,
              int neighbourhoodSize,
              std::mt19937& rng)
{
    const auto ids = sorted_agent_ids(paths);
    const int K = clamp_K(neighbourhoodSize, static_cast<int>(ids.size()));
    if (K == 0) return {};

    std::vector<int> selected;
    selected.reserve(K);
    std::sample(ids.begin(), ids.end(),
                std::back_inserter(selected),
                static_cast<std::size_t>(K),
                rng);

    release_all(selected, table);
    return selected;
}

}  // namespace mapf
