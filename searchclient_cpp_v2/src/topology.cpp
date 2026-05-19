#include "aimas/topology.hpp"

#include <queue>

namespace aimas {

Topology::Topology(const Level& level) : level_(level)
{
    build_components();
}

void Topology::build_components()
{
    components_.assign(level_.rows, std::vector<int>(level_.cols, -1));
    int next_id = 0;
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            if (level_.walls[r][c] || components_[r][c] != -1) continue;
            std::queue<std::pair<int, int>> q;
            q.emplace(r, c);
            components_[r][c] = next_id;
            while (!q.empty()) {
                auto [cr, cc] = q.front();
                q.pop();
                static const int dr[4] = {-1, 1, 0, 0};
                static const int dc[4] = {0, 0, -1, 1};
                for (int k = 0; k < 4; ++k) {
                    const int nr = cr + dr[k];
                    const int nc = cc + dc[k];
                    if (nr < 0 || nr >= level_.rows
                        || nc < 0 || nc >= level_.cols) continue;
                    if (level_.walls[nr][nc] || components_[nr][nc] != -1) continue;
                    components_[nr][nc] = next_id;
                    q.emplace(nr, nc);
                }
            }
            ++next_id;
        }
    }
    num_components_ = next_id;
}

const std::vector<std::vector<int>>& Topology::dist_walls_only(int row, int col)
{
    auto key = std::make_pair(row, col);
    auto it = bfs_cache_.find(key);
    if (it != bfs_cache_.end()) return it->second;

    std::vector<std::vector<int>> dist(
        level_.rows, std::vector<int>(level_.cols, kInf));
    if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols
        || level_.walls[row][col]) {
        return bfs_cache_.emplace(key, std::move(dist)).first->second;
    }
    dist[row][col] = 0;
    std::queue<std::pair<int, int>> q;
    q.emplace(row, col);
    while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop();
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        for (int k = 0; k < 4; ++k) {
            const int nr = cr + dr[k];
            const int nc = cc + dc[k];
            if (nr < 0 || nr >= level_.rows
                || nc < 0 || nc >= level_.cols) continue;
            if (level_.walls[nr][nc] || dist[nr][nc] != kInf) continue;
            dist[nr][nc] = dist[cr][cc] + 1;
            q.emplace(nr, nc);
        }
    }
    return bfs_cache_.emplace(key, std::move(dist)).first->second;
}

std::vector<std::vector<int>> Topology::dist_box_aware(
    const State& state, int goal_row, int goal_col) const
{
    std::vector<std::vector<int>> dist(
        level_.rows, std::vector<int>(level_.cols, kInf));
    if (goal_row < 0 || goal_row >= level_.rows
        || goal_col < 0 || goal_col >= level_.cols
        || level_.walls[goal_row][goal_col]) {
        return dist;
    }
    dist[goal_row][goal_col] = 0;
    std::queue<std::pair<int, int>> q;
    q.emplace(goal_row, goal_col);
    while (!q.empty()) {
        auto [cr, cc] = q.front();
        q.pop();
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        for (int k = 0; k < 4; ++k) {
            const int nr = cr + dr[k];
            const int nc = cc + dc[k];
            if (nr < 0 || nr >= level_.rows
                || nc < 0 || nc >= level_.cols) continue;
            if (level_.walls[nr][nc] || dist[nr][nc] != kInf) continue;
            // Boxes (other than the goal cell, already seeded at 0) block.
            if (state.boxes[nr][nc] != '\0') continue;
            dist[nr][nc] = dist[cr][cc] + 1;
            q.emplace(nr, nc);
        }
    }
    return dist;
}

void Topology::build_dead_cells_for(char box)
{
    // For each open cell, mark it "dead" for `box` if there is NO walls-only
    // reachable letter goal for that box. Also mark cells with fewer than 2
    // non-wall orthogonal neighbours (dead-end pockets a box can never be
    // pushed/pulled into) as dead. The level's actual goal cells for `box`
    // are NEVER dead.
    const int idx = box - 'A';
    auto& dead = dead_cell_cache_[idx];
    dead.assign(level_.rows, std::vector<char>(level_.cols, 1));
    dead_cell_built_[idx] = true;

    // Collect all letter goals for this box.
    std::vector<std::pair<int, int>> goals;
    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            if (level_.goals[r][c] == box) goals.emplace_back(r, c);
        }
    }
    if (goals.empty()) {
        // No goals for this box → no cell is "useful" but parking is moot.
        // Mark all open cells alive so the filter is a no-op for goalless
        // letters (callers will skip such cases anyway).
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (!level_.walls[r][c]) dead[r][c] = 0;
            }
        }
        return;
    }

    // BFS from each goal over walls only; union of reachable cells = alive set.
    std::vector<std::vector<char>> reachable_any(
        level_.rows, std::vector<char>(level_.cols, 0));
    for (const auto& g : goals) {
        const auto& dist = dist_walls_only(g.first, g.second);
        for (int r = 0; r < level_.rows; ++r) {
            for (int c = 0; c < level_.cols; ++c) {
                if (dist[r][c] != kInf) reachable_any[r][c] = 1;
            }
        }
    }

    // Degree filter — cells with degree < 2 are dead-end pockets that a box
    // can't be pushed/pulled into (no agent on the back side to push it in).
    // Goal cells themselves are exempt: the level designer placed a goal
    // there, so by construction the box CAN reach it (via some sequence we
    // don't bother to fully verify here).
    auto degree = [&](int r, int c) {
        int d = 0;
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        for (int k = 0; k < 4; ++k) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols) continue;
            if (!level_.walls[nr][nc]) ++d;
        }
        return d;
    };

    for (int r = 0; r < level_.rows; ++r) {
        for (int c = 0; c < level_.cols; ++c) {
            if (level_.walls[r][c]) { dead[r][c] = 1; continue; }
            const bool is_goal = (level_.goals[r][c] == box);
            if (is_goal) { dead[r][c] = 0; continue; }
            if (!reachable_any[r][c]) { dead[r][c] = 1; continue; }
            if (degree(r, c) < 2) { dead[r][c] = 1; continue; }
            dead[r][c] = 0;
        }
    }
}

bool Topology::pull_aware_dead_cell(char box, int row, int col)
{
    if (box < 'A' || box > 'Z') return true;
    if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols) return true;
    if (level_.walls[row][col]) return true;
    const int idx = box - 'A';
    if (!dead_cell_built_[idx]) build_dead_cells_for(box);
    return dead_cell_cache_[idx][row][col] != 0;
}

}  // namespace aimas
