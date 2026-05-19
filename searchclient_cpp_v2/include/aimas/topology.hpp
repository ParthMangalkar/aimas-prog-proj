#pragma once

#include "aimas/core.hpp"

#include <array>
#include <map>
#include <utility>
#include <vector>

namespace aimas {

// Topology utilities: BFS distance fields and connected-component analysis on
// the static walls grid plus runtime variants that treat boxes as obstacles.
//
// All distance fields return kInf for unreachable cells. Calls that omit a
// State use only walls; calls that take a State additionally treat current box
// positions and (optionally) other-agent positions as obstacles.
class Topology {
public:
    explicit Topology(const Level& level);

    // BFS over walls only; cached.
    const std::vector<std::vector<int>>& dist_walls_only(int row, int col);

    // BFS treating all boxes in `state` as walls, plus the per-agent fix that
    // the goal cell itself is reachable even if it holds the active box.
    // Not cached (state-dependent). Used by single-box A* and PIBT for
    // box-aware agent heuristics.
    std::vector<std::vector<int>> dist_box_aware(
        const State& state, int goal_row, int goal_col) const;

    // True iff (row, col) is a cell from which the given box can NEVER be
    // pushed/pulled to ANY of its letter-goals (using walls-only BFS as
    // a relaxation). Used to filter relocation parking cells so we don't
    // park a box in a dead-end pocket from which it can never be retrieved.
    // Goal cells for the box are never dead.
    //
    // Note: this is a conservative WALLS-ONLY relaxation — it doesn't account
    // for which directions the box can actually be entered/exited; cells with
    // fewer than 2 non-wall orthogonal neighbours are also flagged.
    bool pull_aware_dead_cell(char box, int row, int col);

    // Connected components over walls only. comp_id[r][c] is -1 for walls and
    // otherwise the component index. There are num_components() distinct ids.
    const std::vector<std::vector<int>>& component_ids() const { return components_; }
    int num_components() const { return num_components_; }

    const Level& level() const { return level_; }

private:
    const Level&                                   level_;
    std::map<std::pair<int, int>, std::vector<std::vector<int>>> bfs_cache_;
    std::vector<std::vector<int>>                  components_;
    int                                            num_components_ = 0;

    // Cached per-letter "any goal reachable from cell" map. Filled lazily.
    std::array<std::vector<std::vector<char>>, 26> dead_cell_cache_;
    std::array<bool, 26>                           dead_cell_built_{};

    void build_components();
    void build_dead_cells_for(char box);
};

}  // namespace aimas
