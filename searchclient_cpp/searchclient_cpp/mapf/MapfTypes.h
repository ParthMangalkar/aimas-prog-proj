#pragma once

// Shared types for every MAPF component (BFS, reservation, STA*, allocator,
// CAA*, and future ECBS/ALNS phases). Keep this header dependency-light;
// component-specific types stay in their own headers.

#include <limits>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace mapf {

constexpr int kInf = std::numeric_limits<int>::max();

using DistanceGrid = std::vector<std::vector<int>>;

enum class SubtaskType { DeliverBox, ReachCell };

struct Subtask {
    SubtaskType type = SubtaskType::ReachCell;
    char box_letter = '\0';
    int box_start_r = -1, box_start_c = -1;
    int box_goal_r  = -1, box_goal_c  = -1;
    int target_r    = -1, target_c    = -1;
};

struct Constraint {
    int row = -1;
    int col = -1;
    int time = -1;
    bool blocksAgent = false;
    bool blocksBox = false;
};

inline bool operator==(const Constraint& lhs, const Constraint& rhs) noexcept
{
    return lhs.row == rhs.row &&
           lhs.col == rhs.col &&
           lhs.time == rhs.time &&
           lhs.blocksAgent == rhs.blocksAgent &&
           lhs.blocksBox == rhs.blocksBox;
}

struct ConstraintHash {
    std::size_t operator()(const Constraint& c) const noexcept
    {
        std::size_t h = 1469598103934665603ULL;
        auto mix = [&](int value) {
            h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(value));
            h *= 1099511628211ULL;
        };
        mix(c.row);
        mix(c.col);
        mix(c.time);
        mix(c.blocksAgent ? 1 : 0);
        mix(c.blocksBox ? 1 : 0);
        return h;
    }
};

using ConstraintSet = std::unordered_set<Constraint, ConstraintHash>;

}  // namespace mapf
