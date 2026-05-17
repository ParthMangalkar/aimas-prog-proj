#include "BFSDistanceMap.h"

#include "../Domain.h"

#include <deque>
#include <utility>

namespace mapf {

DistanceGrid bfs_from(int sr, int sc)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows > 0 ? static_cast<int>(State::walls[0].size()) : 0;
    DistanceGrid d(rows, std::vector<int>(cols, kInf));
    if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) return d;
    if (State::walls[sr][sc]) return d;

    std::deque<std::pair<int, int>> q;
    d[sr][sc] = 0;
    q.emplace_back(sr, sc);
    static constexpr int DR[4] = {-1, 1, 0, 0};
    static constexpr int DC[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        const auto [r, c] = q.front();
        q.pop_front();
        const int nd = d[r][c] + 1;
        for (int i = 0; i < 4; ++i) {
            const int nr = r + DR[i];
            const int nc = c + DC[i];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (State::walls[nr][nc]) continue;
            if (d[nr][nc] != kInf) continue;
            d[nr][nc] = nd;
            q.emplace_back(nr, nc);
        }
    }
    return d;
}

}  // namespace mapf
