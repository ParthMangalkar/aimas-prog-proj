#pragma once

#include "MapfTypes.h"

namespace mapf {

// Reverse BFS through the static wall grid (State::walls) starting at (sr, sc).
// Returns a DistanceGrid where d[r][c] == kInf for unreachable cells.
// Caller is responsible for caching results across phases.
DistanceGrid bfs_from(int sr, int sc);

}  // namespace mapf
