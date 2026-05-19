// Core domain types for the AIMAS Hospital MAPF-with-boxes domain.
//
// Modular re-implementation of the foundational types from
// searchclient_cpp_enhanced. Action semantics, applicability rules,
// conflict detection, and the joint-action application contract are
// preserved exactly so plans validate identically.
//
// Public surface:
//   enum class ActionType   { NoOp, Move, Push, Pull }
//   struct Action           — one row in the action table
//   const std::vector<Action>& actions()
//   struct Level            — immutable level metadata (walls, colors, goals)
//   struct State            — mutable agent positions + box grid
//   struct StateHash, StateEq — for unordered containers
//
// Conventions:
//   - row indexing increases southward, column indexing increases eastward.
//   - boxes[row][col] is '\0' when empty, otherwise 'A'..'Z'.
//   - goals[row][col] is '\0' when no goal at that cell.
//   - agent_color[i] is the color id for agent i; box_color['X'-'A'] for boxes.
//   - Color id -1 means "unspecified" (treated as no agent allowed to move
//     that letter); the parser falls back to the first known agent color when
//     a box letter is missing from #colors, matching the legacy behavior.

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace aimas {

inline constexpr int kInf = std::numeric_limits<int>::max() / 4;
inline constexpr int kNoOpIndex = 0;

inline bool is_box_char(char ch)    { return 'A' <= ch && ch <= 'Z'; }
inline bool is_agent_char(char ch)  { return '0' <= ch && ch <= '9'; }

enum class ActionType : std::uint8_t { NoOp, Move, Push, Pull };

struct Action {
    std::string name;
    ActionType  type     = ActionType::NoOp;
    int         agent_dr = 0;
    int         agent_dc = 0;
    int         box_dr   = 0;
    int         box_dc   = 0;
};

// Canonical action table (29 entries): 1 NoOp + 4 Move + 12 Push + 12 Pull.
// Order is significant; index 0 is always NoOp.
const std::vector<Action>& actions();

struct Level {
    std::string                    name;
    int                            rows = 0;
    int                            cols = 0;
    std::vector<std::vector<bool>> walls;          // walls[r][c]
    std::vector<std::string>       boxes;          // initial box grid
    std::vector<std::string>       goals;          // goal grid (letters & digits)
    std::vector<int>               agent_rows;     // initial agent rows
    std::vector<int>               agent_cols;     // initial agent cols
    std::vector<int>               agent_color;    // color id per agent
    std::array<int, 26>            box_color{};    // color id per box letter
};

struct State {
    const Level*             level = nullptr;
    std::vector<int>         agent_rows;
    std::vector<int>         agent_cols;
    std::vector<std::string> boxes;

    // Construct an initial state from a parsed level.
    static State initial(const Level& level);

    // Resolve which agent (if any) is at (row, col); returns '\0' when none.
    char agent_at(int row, int col) const;

    bool in_bounds(int row, int col) const;
    bool cell_free(int row, int col) const;
    bool can_move_box(int agent, char box) const;

    // Whether an individual agent's action is applicable in isolation.
    bool applicable(int agent, const Action& action) const;

    struct Delta {
        int  agent_from_r = -1;
        int  agent_from_c = -1;
        int  agent_to_r   = -1;
        int  agent_to_c   = -1;
        int  box_from_r   = -1;
        int  box_from_c   = -1;
        int  box_to_r     = -1;
        int  box_to_c     = -1;
        bool moves_box    = false;
    };

    Delta delta_for(int agent, const Action& action) const;

    // Whether a joint action contains a conflict (collision or swap).
    bool conflicting(const std::vector<int>& joint_action) const;

    // Apply a joint action in place; returns false (and leaves state unchanged
    // when validation fails). joint_action[i] indexes into actions().
    bool apply_joint(const std::vector<int>& joint_action);

    // Whether all letter-goals are covered by matching boxes and all
    // agent-goals are covered by matching agents.
    bool goal_state() const;
};

// Helpers for hashing/equality in unordered containers keyed on State.
struct StateHash {
    std::size_t operator()(const State& s) const noexcept;
};

struct StateEq {
    bool operator()(const State& a, const State& b) const noexcept;
};

// ------- small string utilities reused by parser & solver -------
std::string                 trim(const std::string& s);
std::vector<std::string>    split(const std::string& s, char delim);
std::string                 to_lower(std::string s);
bool                        env_flag_enabled(const char* name);

}  // namespace aimas
