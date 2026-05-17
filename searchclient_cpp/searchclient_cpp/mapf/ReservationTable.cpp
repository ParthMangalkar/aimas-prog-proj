#include "ReservationTable.h"

#include <algorithm>

namespace mapf {

ReservationTable::ReservationTable(int rows, int cols, int max_t)
    : rows_(rows), cols_(cols), max_t_(max_t),
      cells_(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols))
{
}

void ReservationTable::reserve_cell(int r, int c, int t_start, int t_end, int agent)
{
    cells_[idx(r, c)].push_back({t_start, t_end, agent});
}

void ReservationTable::reserve_edge(int r1, int c1, int r2, int c2, int t, int agent)
{
    edges_[edge_key(r1, c1, r2, c2, t)].push_back(agent);
}

bool ReservationTable::is_cell_blocked(int r, int c, int t, int except_agent) const
{
    if (r < 0 || r >= rows_ || c < 0 || c >= cols_) return true;
    for (const auto& res : cells_[idx(r, c)]) {
        if (res.agent == except_agent) continue;
        if (res.t_start <= t && t <= res.t_end) return true;
    }
    return false;
}

void ReservationTable::release(int agent)
{
    for (auto& bucket : cells_) {
        bucket.erase(
            std::remove_if(bucket.begin(), bucket.end(),
                           [agent](const CellReservation& r) { return r.agent == agent; }),
            bucket.end());
    }
    for (auto it = edges_.begin(); it != edges_.end(); ) {
        auto& v = it->second;
        v.erase(std::remove(v.begin(), v.end(), agent), v.end());
        if (v.empty()) it = edges_.erase(it); else ++it;
    }
}

bool ReservationTable::is_cell_occupied(int r, int c, int t) const
{
    if (r < 0 || r >= rows_ || c < 0 || c >= cols_) return false;
    for (const auto& res : cells_[idx(r, c)]) {
        if (res.t_start <= t && t <= res.t_end) return true;
    }
    return false;
}

bool ReservationTable::is_edge_blocked(int r1, int c1, int r2, int c2, int t, int except_agent) const
{
    // We are entering (r2,c2) at time t coming from (r1,c1).
    // Conflict: another agent traverses (r2,c2)->(r1,c1) at the same time.
    auto it = edges_.find(edge_key(r2, c2, r1, c1, t));
    if (it == edges_.end()) return false;
    for (int a : it->second) if (a != except_agent) return true;
    return false;
}

std::size_t ReservationTable::idx(int r, int c) const
{
    return static_cast<std::size_t>(r) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(c);
}

long long ReservationTable::edge_key(int r1, int c1, int r2, int c2, int t) const
{
    long long k = static_cast<long long>(r1) * cols_ + c1;
    k = k * (static_cast<long long>(rows_) * cols_) + (static_cast<long long>(r2) * cols_ + c2);
    k = k * (static_cast<long long>(max_t_) + 2) + t;
    return k;
}

}  // namespace mapf
