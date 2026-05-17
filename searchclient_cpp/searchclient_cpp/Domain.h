#pragma once

// Shared problem-domain types used by both the classical joint-state search
// (Heuristic / Frontier / GraphSearch in main.cpp) and the MAPF pipeline
// (everything under namespace mapf in mapf/*.h).
//
// State, Color, Action, ACTIONS and StatePtr live here so they can be
// included from any translation unit. main.cpp keeps everything else
// (parse_level, Heuristic, Frontier, GraphSearch, main()).

#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

inline std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

class JavaRandom {
public:
    explicit JavaRandom(long long seed)
        : seed_((seed ^ kMultiplier) & kMask)
    {
    }

    int next_int(int bound)
    {
        if (bound <= 0) {
            throw std::invalid_argument("bound must be positive");
        }
        if ((bound & -bound) == bound) {
            return static_cast<int>((bound * static_cast<long long>(next(31))) >> 31);
        }
        int bits = 0;
        int value = 0;
        do {
            bits = next(31);
            value = bits % bound;
        } while (bits - value + (bound - 1) < 0);
        return value;
    }

private:
    static constexpr long long kMultiplier = 0x5DEECE66DLL;
    static constexpr long long kAddend = 0xBLL;
    static constexpr long long kMask = (1LL << 48) - 1;

    int next(int bits)
    {
        seed_ = (seed_ * kMultiplier + kAddend) & kMask;
        return static_cast<int>(seed_ >> (48 - bits));
    }

    long long seed_;
};

template <typename T>
void java_shuffle(std::vector<T>& values, JavaRandom& rng)
{
    for (int i = static_cast<int>(values.size()); i > 1; --i) {
        std::swap(values[static_cast<std::size_t>(i - 1)],
                  values[static_cast<std::size_t>(rng.next_int(i))]);
    }
}

enum class Color { Blue, Red, Cyan, Purple, Green, Orange, Pink, Grey, Lightblue, Brown, Unknown };

inline Color color_from_string(const std::string& value)
{
    const std::string lowered = to_lower(value);
    if (lowered == "blue") return Color::Blue;
    if (lowered == "red") return Color::Red;
    if (lowered == "cyan") return Color::Cyan;
    if (lowered == "purple") return Color::Purple;
    if (lowered == "green") return Color::Green;
    if (lowered == "orange") return Color::Orange;
    if (lowered == "pink") return Color::Pink;
    if (lowered == "grey") return Color::Grey;
    if (lowered == "lightblue") return Color::Lightblue;
    if (lowered == "brown") return Color::Brown;
    return Color::Unknown;
}

enum class ActionType { NoOp, Move, Push, Pull };

struct Action {
    const char* name;
    ActionType type;
    int agent_row_delta;
    int agent_col_delta;
    int box_row_delta;
    int box_col_delta;
};

inline constexpr std::array<Action, 29> ACTIONS {{
    {"NoOp", ActionType::NoOp, 0, 0, 0, 0},
    {"Move(N)", ActionType::Move, -1, 0, 0, 0},
    {"Move(S)", ActionType::Move, 1, 0, 0, 0},
    {"Move(E)", ActionType::Move, 0, 1, 0, 0},
    {"Move(W)", ActionType::Move, 0, -1, 0, 0},
    {"Push(N,N)", ActionType::Push, -1, 0, -1, 0},
    {"Push(N,E)", ActionType::Push, -1, 0, 0, 1},
    {"Push(N,W)", ActionType::Push, -1, 0, 0, -1},
    {"Push(S,S)", ActionType::Push, 1, 0, 1, 0},
    {"Push(S,E)", ActionType::Push, 1, 0, 0, 1},
    {"Push(S,W)", ActionType::Push, 1, 0, 0, -1},
    {"Push(E,E)", ActionType::Push, 0, 1, 0, 1},
    {"Push(E,N)", ActionType::Push, 0, 1, -1, 0},
    {"Push(E,S)", ActionType::Push, 0, 1, 1, 0},
    {"Push(W,W)", ActionType::Push, 0, -1, 0, -1},
    {"Push(W,N)", ActionType::Push, 0, -1, -1, 0},
    {"Push(W,S)", ActionType::Push, 0, -1, 1, 0},
    {"Pull(N,N)", ActionType::Pull, -1, 0, -1, 0},
    {"Pull(N,E)", ActionType::Pull, -1, 0, 0, 1},
    {"Pull(N,W)", ActionType::Pull, -1, 0, 0, -1},
    {"Pull(S,S)", ActionType::Pull, 1, 0, 1, 0},
    {"Pull(S,E)", ActionType::Pull, 1, 0, 0, 1},
    {"Pull(S,W)", ActionType::Pull, 1, 0, 0, -1},
    {"Pull(E,N)", ActionType::Pull, 0, 1, -1, 0},
    {"Pull(E,S)", ActionType::Pull, 0, 1, 1, 0},
    {"Pull(E,E)", ActionType::Pull, 0, 1, 0, 1},
    {"Pull(W,N)", ActionType::Pull, 0, -1, -1, 0},
    {"Pull(W,S)", ActionType::Pull, 0, -1, 1, 0},
    {"Pull(W,W)", ActionType::Pull, 0, -1, 0, -1},
}};

struct State;
using StatePtr = std::shared_ptr<State>;

struct State : std::enable_shared_from_this<State> {
    static inline std::vector<Color> agent_colors;
    static inline std::vector<Color> box_colors;
    static inline std::vector<std::vector<bool>> walls;
    static inline std::vector<std::string> goals;
    static inline JavaRandom rng{1};

    std::vector<int> agent_rows;
    std::vector<int> agent_cols;
    std::vector<std::string> boxes;
    StatePtr parent;
    std::vector<const Action*> joint_action;
    int path_cost = 0;

    State(std::vector<int> rows,
          std::vector<int> cols,
          std::vector<Color> in_agent_colors,
          std::vector<std::vector<bool>> in_walls,
          std::vector<std::string> in_boxes,
          std::vector<Color> in_box_colors,
          std::vector<std::string> in_goals)
        : agent_rows(std::move(rows)),
          agent_cols(std::move(cols)),
          boxes(std::move(in_boxes))
    {
        agent_colors = std::move(in_agent_colors);
        walls = std::move(in_walls);
        box_colors = std::move(in_box_colors);
        goals = std::move(in_goals);
    }

    State(StatePtr parent_state, std::vector<const Action*> action)
        : agent_rows(parent_state->agent_rows),
          agent_cols(parent_state->agent_cols),
          boxes(parent_state->boxes),
          parent(std::move(parent_state)),
          joint_action(std::move(action)),
          path_cost(parent->path_cost + 1)
    {
        for (std::size_t agent = 0; agent < agent_rows.size(); ++agent) {
            const Action& current = *joint_action[agent];
            switch (current.type) {
                case ActionType::NoOp:
                    break;
                case ActionType::Move:
                    agent_rows[agent] += current.agent_row_delta;
                    agent_cols[agent] += current.agent_col_delta;
                    break;
                case ActionType::Push: {
                    const int box_row = agent_rows[agent] + current.agent_row_delta;
                    const int box_col = agent_cols[agent] + current.agent_col_delta;
                    const int new_box_row = box_row + current.box_row_delta;
                    const int new_box_col = box_col + current.box_col_delta;
                    boxes[new_box_row][new_box_col] = boxes[box_row][box_col];
                    boxes[box_row][box_col] = '\0';
                    agent_rows[agent] += current.agent_row_delta;
                    agent_cols[agent] += current.agent_col_delta;
                    break;
                }
                case ActionType::Pull: {
                    const int box_row = agent_rows[agent] - current.box_row_delta;
                    const int box_col = agent_cols[agent] - current.box_col_delta;
                    boxes[agent_rows[agent]][agent_cols[agent]] = boxes[box_row][box_col];
                    boxes[box_row][box_col] = '\0';
                    agent_rows[agent] += current.agent_row_delta;
                    agent_cols[agent] += current.agent_col_delta;
                    break;
                }
            }
        }
    }

    int g() const { return path_cost; }

    char agent_at(int row, int col) const
    {
        for (std::size_t i = 0; i < agent_rows.size(); ++i) {
            if (agent_rows[i] == row && agent_cols[i] == col) {
                return static_cast<char>('0' + static_cast<int>(i));
            }
        }
        return '\0';
    }

    bool cell_is_free(int row, int col) const
    {
        return !walls[row][col] && boxes[row][col] == '\0' && agent_at(row, col) == '\0';
    }

    bool is_goal_state() const
    {
        for (std::size_t row = 1; row + 1 < goals.size(); ++row) {
            for (std::size_t col = 1; col + 1 < goals[row].size(); ++col) {
                const char goal = goals[row][col];
                if ('A' <= goal && goal <= 'Z' && boxes[row][col] != goal) {
                    return false;
                }
                if ('0' <= goal && goal <= '9') {
                    const int agent = goal - '0';
                    if (agent >= static_cast<int>(agent_rows.size()) ||
                        agent_rows[agent] != static_cast<int>(row) ||
                        agent_cols[agent] != static_cast<int>(col)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    bool is_applicable(int agent, const Action& action) const
    {
        const int row = agent_rows[agent];
        const int col = agent_cols[agent];
        switch (action.type) {
            case ActionType::NoOp:
                return true;
            case ActionType::Move:
                return cell_is_free(row + action.agent_row_delta, col + action.agent_col_delta);
            case ActionType::Push: {
                const int box_row = row + action.agent_row_delta;
                const int box_col = col + action.agent_col_delta;
                const char box = boxes[box_row][box_col];
                if (box == '\0' || agent_colors[agent] != box_colors[box - 'A']) {
                    return false;
                }
                return cell_is_free(box_row + action.box_row_delta, box_col + action.box_col_delta);
            }
            case ActionType::Pull: {
                if (!cell_is_free(row + action.agent_row_delta, col + action.agent_col_delta)) {
                    return false;
                }
                const int box_row = row - action.box_row_delta;
                const int box_col = col - action.box_col_delta;
                const char box = boxes[box_row][box_col];
                return box != '\0' && agent_colors[agent] == box_colors[box - 'A'];
            }
        }
        return false;
    }

    bool is_conflicting(const std::vector<const Action*>& proposed) const
    {
        const int num_agents = static_cast<int>(agent_rows.size());
        std::vector<int> dest_rows(num_agents, -1);
        std::vector<int> dest_cols(num_agents, -1);
        std::vector<int> box_rows(num_agents, -1);
        std::vector<int> box_cols(num_agents, -1);

        for (int agent = 0; agent < num_agents; ++agent) {
            const Action& action = *proposed[agent];
            const int row = agent_rows[agent];
            const int col = agent_cols[agent];
            switch (action.type) {
                case ActionType::NoOp:
                    break;
                case ActionType::Move:
                    dest_rows[agent] = row + action.agent_row_delta;
                    dest_cols[agent] = col + action.agent_col_delta;
                    box_rows[agent] = row;
                    box_cols[agent] = col;
                    break;
                case ActionType::Push:
                    dest_rows[agent] = row + action.agent_row_delta;
                    dest_cols[agent] = col + action.agent_col_delta;
                    box_rows[agent] = dest_rows[agent] + action.box_row_delta;
                    box_cols[agent] = dest_cols[agent] + action.box_col_delta;
                    break;
                case ActionType::Pull:
                    dest_rows[agent] = row + action.agent_row_delta;
                    dest_cols[agent] = col + action.agent_col_delta;
                    box_rows[agent] = row;
                    box_cols[agent] = col;
                    break;
            }
        }

        for (int a1 = 0; a1 < num_agents; ++a1) {
            if (proposed[a1]->type == ActionType::NoOp) continue;
            for (int a2 = a1 + 1; a2 < num_agents; ++a2) {
                if (proposed[a2]->type == ActionType::NoOp) continue;
                if ((dest_rows[a1] == dest_rows[a2] && dest_cols[a1] == dest_cols[a2]) ||
                    (box_rows[a1] == box_rows[a2] && box_cols[a1] == box_cols[a2]) ||
                    (dest_rows[a1] == box_rows[a2] && dest_cols[a1] == box_cols[a2]) ||
                    (dest_rows[a2] == box_rows[a1] && dest_cols[a2] == box_cols[a1])) {
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<StatePtr> get_expanded_states()
    {
        const int num_agents = static_cast<int>(agent_rows.size());
        std::vector<std::vector<const Action*>> applicable(num_agents);
        for (int agent = 0; agent < num_agents; ++agent) {
            for (const auto& action : ACTIONS) {
                if (is_applicable(agent, action)) {
                    applicable[agent].push_back(&action);
                }
            }
        }

        std::vector<const Action*> joint(num_agents, &ACTIONS[0]);
        std::vector<int> permutation(num_agents, 0);
        std::vector<StatePtr> expanded;
        const StatePtr self = shared_from_this();

        while (true) {
            for (int agent = 0; agent < num_agents; ++agent) {
                joint[agent] = applicable[agent][permutation[agent]];
            }
            if (!is_conflicting(joint)) {
                expanded.push_back(std::make_shared<State>(self, joint));
            }

            bool done = false;
            for (int agent = 0; agent < num_agents; ++agent) {
                if (permutation[agent] < static_cast<int>(applicable[agent].size()) - 1) {
                    ++permutation[agent];
                    break;
                }
                permutation[agent] = 0;
                if (agent == num_agents - 1) {
                    done = true;
                }
            }
            if (done) break;
        }

        java_shuffle(expanded, rng);
        return expanded;
    }

    std::vector<std::vector<const Action*>> extract_plan() const
    {
        std::vector<std::vector<const Action*>> plan(static_cast<std::size_t>(path_cost));
        const State* current = this;
        while (!current->joint_action.empty()) {
            plan[static_cast<std::size_t>(current->path_cost - 1)] = current->joint_action;
            current = current->parent.get();
        }
        return plan;
    }

    bool operator==(const State& other) const
    {
        return agent_rows == other.agent_rows &&
               agent_cols == other.agent_cols &&
               boxes == other.boxes;
    }
};
