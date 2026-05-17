#pragma once

#include <unordered_map>
#include <vector>

namespace mapf {

struct CellReservation {
    int t_start;
    int t_end;   // inclusive; INT_MAX = forever
    int agent;   // -1 for static obstacles (e.g. immovable boxes)
};

// Space-time reservation table used by STA* (and later by ECBS / ALNS).
// Stores per-cell timed reservations plus an edge map for swap conflicts.
class ReservationTable {
public:
    ReservationTable() = default;
    ReservationTable(int rows, int cols, int max_t);

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int max_t() const { return max_t_; }

    void reserve_cell(int r, int c, int t_start, int t_end, int agent);
    void reserve_edge(int r1, int c1, int r2, int c2, int t, int agent);

    // Remove every cell/edge reservation owned by `agent`. Used by ALNS to
    // free a destroyed neighbourhood before repair re-plans it.
    void release(int agent);

    bool is_cell_blocked(int r, int c, int t, int except_agent) const;
    bool is_edge_blocked(int r1, int c1, int r2, int c2, int t, int except_agent) const;
    // True iff any agent's reservation covers (r,c,t). Convenience for tests
    // and ALNS sanity checks.
    bool is_cell_occupied(int r, int c, int t) const;

private:
    int rows_ = 0, cols_ = 0, max_t_ = 0;
    std::vector<std::vector<CellReservation>> cells_;
    std::unordered_map<long long, std::vector<int>> edges_;

    std::size_t idx(int r, int c) const;
    long long edge_key(int r1, int c1, int r2, int c2, int t) const;
};

}  // namespace mapf
