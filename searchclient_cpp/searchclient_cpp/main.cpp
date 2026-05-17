#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace {

std::string trim(const std::string& input)
{
    std::size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start]))) {
        ++start;
    }
    std::size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(start, end - start);
}

std::vector<std::string> split(const std::string& input, char delimiter)
{
    std::vector<std::string> parts;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        parts.push_back(item);
    }
    return parts;
}

std::string to_lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool profiling_enabled()
{
    static const bool enabled = [] {
        const char* value = std::getenv("AIMAS_PROFILE");
        return value != nullptr && std::string(value) != "0";
    }();
    return enabled;
}

double elapsed_seconds_since(const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

class ScopedProfile {
public:
    explicit ScopedProfile(std::string phase)
        : phase_(std::move(phase)),
          start_(std::chrono::steady_clock::now())
    {
    }

    ~ScopedProfile()
    {
        if (profiling_enabled()) {
            std::cerr << "[profile] phase=" << phase_
                      << " seconds=" << std::fixed << std::setprecision(6)
                      << elapsed_seconds_since(start_) << '\n';
        }
    }

private:
    std::string phase_;
    std::chrono::steady_clock::time_point start_;
};

void profile_event(const std::string& message)
{
    if (profiling_enabled()) {
        std::cerr << "[profile] " << message << '\n';
    }
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
        std::swap(values[static_cast<std::size_t>(i - 1)], values[static_cast<std::size_t>(rng.next_int(i))]);
    }
}

enum class Color { Blue, Red, Cyan, Purple, Green, Orange, Pink, Grey, Lightblue, Brown, Unknown };

Color color_from_string(const std::string& value)
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

constexpr std::array<Action, 29> ACTIONS {{
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

    int g() const
    {
        return path_cost;
    }

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
            if (proposed[a1]->type == ActionType::NoOp) {
                continue;
            }
            for (int a2 = a1 + 1; a2 < num_agents; ++a2) {
                if (proposed[a2]->type == ActionType::NoOp) {
                    continue;
                }
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
        std::size_t max_joint_actions = 1;
        for (const auto& actions : applicable) {
            max_joint_actions *= actions.size();
            if (max_joint_actions > 4096) {
                max_joint_actions = 4096;
                break;
            }
        }
        expanded.reserve(max_joint_actions);
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
            if (done) {
                break;
            }
        }

        java_shuffle(expanded, rng);
        return expanded;
    }

    std::size_t joint_action_upper_bound(std::size_t cap) const
    {
        std::size_t product = 1;
        for (int agent = 0; agent < static_cast<int>(agent_rows.size()); ++agent) {
            std::size_t applicable_count = 0;
            for (const auto& action : ACTIONS) {
                if (is_applicable(agent, action)) {
                    ++applicable_count;
                }
            }
            product *= applicable_count;
            if (product > cap) {
                return product;
            }
        }
        return product;
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

struct StatePtrHasher {
    std::size_t operator()(const StatePtr& state) const
    {
        std::size_t seed = 0;
        auto combine = [&seed](std::size_t value) {
            seed ^= value + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        };
        for (int value : state->agent_rows) combine(std::hash<int>{}(value));
        for (int value : state->agent_cols) combine(std::hash<int>{}(value));
        for (const auto& row : state->boxes) combine(std::hash<std::string>{}(row));
        return seed;
    }
};

struct StatePtrEqual {
    bool operator()(const StatePtr& lhs, const StatePtr& rhs) const
    {
        return *lhs == *rhs;
    }
};

class Frontier {
public:
    virtual ~Frontier() = default;
    virtual void add(const StatePtr& state) = 0;
    virtual StatePtr pop() = 0;
    virtual bool is_empty() const = 0;
    virtual int size() const = 0;
    virtual bool contains(const StatePtr& state) const = 0;
    virtual std::string name() const = 0;
};

class Heuristic {
public:
    static constexpr int kInf = std::numeric_limits<int>::max();
    static constexpr int kPenalty = 100000;

    explicit Heuristic(const State&)
    {
        rows_ = static_cast<int>(State::walls.size());
        cols_ = static_cast<int>(State::walls.front().size());
        for (int row = 0; row < rows_; ++row) {
            for (int col = 0; col < cols_; ++col) {
                const char goal = State::goals[row][col];
                if (('A' <= goal && goal <= 'Z') || ('0' <= goal && goal <= '9')) {
                    goal_rows_.push_back(row);
                    goal_cols_.push_back(col);
                    goal_types_.push_back(goal);
                }
            }
        }
        distances_.assign(goal_types_.size(),
                          std::vector<std::vector<int>>(rows_, std::vector<int>(cols_, kInf)));
        for (std::size_t i = 0; i < goal_types_.size(); ++i) {
            bfs_from(static_cast<int>(i), goal_rows_[i], goal_cols_[i]);
        }
    }

    virtual ~Heuristic() = default;
    virtual int f(const State& state) const = 0;
    virtual std::string description() const = 0;

    int h(const State& state) const
    {
        int total = 0;
        for (char letter = 'A'; letter <= 'Z'; ++letter) {
            total += matching_box_goal_distance(state, letter);
        }

        for (std::size_t i = 0; i < goal_types_.size(); ++i) {
            const char goal = goal_types_[i];
            if ('0' <= goal && goal <= '9') {
                const int agent = goal - '0';
                if (agent < static_cast<int>(state.agent_rows.size())) {
                    const int gr = goal_rows_[i];
                    const int gc = goal_cols_[i];
                    if (state.agent_rows[agent] != gr || state.agent_cols[agent] != gc) {
                        const int distance = distances_[i][state.agent_rows[agent]][state.agent_cols[agent]];
                        if (distance < kInf) {
                            total += distance;
                        }
                    }
                }
            }
        }

        for (std::size_t agent = 0; agent < state.agent_rows.size(); ++agent) {
            int min_box_distance = kInf;
            bool found = false;
            for (int row = 0; row < rows_; ++row) {
                for (int col = 0; col < cols_; ++col) {
                    const char box = state.boxes[row][col];
                    if (box == '\0') {
                        continue;
                    }
                    if (State::box_colors[box - 'A'] != State::agent_colors[agent]) {
                        continue;
                    }
                    if (State::goals[row][col] == box) {
                        continue;
                    }
                    found = true;
                    min_box_distance = std::min(min_box_distance,
                                                std::abs(state.agent_rows[agent] - row) +
                                                std::abs(state.agent_cols[agent] - col));
                }
            }
            if (found && min_box_distance < kInf) {
                total += min_box_distance;
            }
        }
        return total;
    }

    int h_smart(const State& state) const
    {
        int total = h(state);
        total += 7 * h_goal_count(state);
        total += 2 * agent_to_useful_box_distance(state);
        total += deadlock_penalty(state);
        return total;
    }

    int h_goal_count(const State& state) const
    {
        int count = 0;
        for (std::size_t i = 0; i < goal_types_.size(); ++i) {
            const char goal = goal_types_[i];
            const int gr = goal_rows_[i];
            const int gc = goal_cols_[i];
            if ('A' <= goal && goal <= 'Z') {
                if (state.boxes[gr][gc] != goal) {
                    ++count;
                }
            } else if ('0' <= goal && goal <= '9') {
                const int agent = goal - '0';
                if (agent < static_cast<int>(state.agent_rows.size()) &&
                    (state.agent_rows[agent] != gr || state.agent_cols[agent] != gc)) {
                    ++count;
                }
            }
        }
        return count;
    }

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<int> goal_rows_;
    std::vector<int> goal_cols_;
    std::vector<char> goal_types_;
    std::vector<std::vector<std::vector<int>>> distances_;

    int matching_box_goal_distance(const State& state, char letter) const
    {
        std::vector<int> goals;
        std::vector<std::pair<int, int>> boxes;
        for (std::size_t i = 0; i < goal_types_.size(); ++i) {
            if (goal_types_[i] == letter && state.boxes[goal_rows_[i]][goal_cols_[i]] != letter) {
                goals.push_back(static_cast<int>(i));
            }
        }
        if (goals.empty()) {
            return 0;
        }
        for (int row = 0; row < rows_; ++row) {
            for (int col = 0; col < cols_; ++col) {
                if (state.boxes[row][col] == letter && State::goals[row][col] != letter) {
                    boxes.push_back({row, col});
                }
            }
        }
        if (boxes.size() < goals.size()) {
            return kPenalty;
        }

        const int goal_count = static_cast<int>(goals.size());
        const int box_count = static_cast<int>(boxes.size());
        if (goal_count <= 12 && box_count <= 12) {
            std::vector<int> dp(static_cast<std::size_t>(1 << box_count), kInf);
            dp[0] = 0;
            for (int mask = 0; mask < (1 << box_count); ++mask) {
                if (dp[static_cast<std::size_t>(mask)] == kInf) {
                    continue;
                }
                const int assigned = popcount(mask);
                if (assigned >= goal_count) {
                    continue;
                }
                const int goal_index = goals[assigned];
                for (int box = 0; box < box_count; ++box) {
                    if ((mask & (1 << box)) != 0) {
                        continue;
                    }
                    const auto [row, col] = boxes[box];
                    const int distance = distances_[goal_index][row][col];
                    const int cost = distance == kInf ? kPenalty : distance;
                    int& next = dp[static_cast<std::size_t>(mask | (1 << box))];
                    next = std::min(next, dp[static_cast<std::size_t>(mask)] + cost);
                }
            }
            int best = kInf;
            for (int mask = 0; mask < (1 << box_count); ++mask) {
                if (popcount(mask) == goal_count) {
                    best = std::min(best, dp[static_cast<std::size_t>(mask)]);
                }
            }
            return best == kInf ? kPenalty : best;
        }

        int greedy_total = 0;
        std::vector<bool> used(boxes.size(), false);
        for (int goal_index : goals) {
            int best_box = -1;
            int best_distance = kInf;
            for (int box = 0; box < box_count; ++box) {
                if (used[static_cast<std::size_t>(box)]) {
                    continue;
                }
                const auto [row, col] = boxes[box];
                const int distance = distances_[goal_index][row][col];
                if (distance < best_distance) {
                    best_distance = distance;
                    best_box = box;
                }
            }
            if (best_box >= 0) {
                used[static_cast<std::size_t>(best_box)] = true;
                greedy_total += best_distance == kInf ? kPenalty : best_distance;
            }
        }
        return greedy_total;
    }

    int agent_to_useful_box_distance(const State& state) const
    {
        int total = 0;
        for (std::size_t agent = 0; agent < state.agent_rows.size(); ++agent) {
            int best = kInf;
            for (int row = 0; row < rows_; ++row) {
                for (int col = 0; col < cols_; ++col) {
                    const char box = state.boxes[row][col];
                    if (box == '\0' || State::goals[row][col] == box) {
                        continue;
                    }
                    if (State::box_colors[box - 'A'] != State::agent_colors[agent]) {
                        continue;
                    }
                    best = std::min(best,
                                    std::abs(state.agent_rows[agent] - row) +
                                    std::abs(state.agent_cols[agent] - col));
                }
            }
            if (best < kInf) {
                total += best;
            }
        }
        return total;
    }

    int deadlock_penalty(const State& state) const
    {
        int penalty = 0;
        for (int row = 0; row < rows_; ++row) {
            for (int col = 0; col < cols_; ++col) {
                const char box = state.boxes[row][col];
                if (box == '\0' || State::goals[row][col] == box) {
                    continue;
                }
                if (is_static_corner(row, col) && !cell_is_goal_for_letter(row, col, box)) {
                    penalty += kPenalty;
                }
            }
        }
        return penalty;
    }

    bool is_static_corner(int row, int col) const
    {
        const bool north = is_wall(row - 1, col);
        const bool south = is_wall(row + 1, col);
        const bool east = is_wall(row, col + 1);
        const bool west = is_wall(row, col - 1);
        return (north || south) && (east || west);
    }

    bool is_wall(int row, int col) const
    {
        return row < 0 || row >= rows_ || col < 0 || col >= cols_ || State::walls[row][col];
    }

    bool cell_is_goal_for_letter(int row, int col, char letter) const
    {
        return State::goals[row][col] == letter;
    }

    static int popcount(int value)
    {
        int count = 0;
        while (value != 0) {
            value &= value - 1;
            ++count;
        }
        return count;
    }

    void bfs_from(int goal_index, int start_row, int start_col)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::deque<std::pair<int, int>> queue;
        distances_[goal_index][start_row][start_col] = 0;
        queue.emplace_back(start_row, start_col);

        while (!queue.empty()) {
            const auto [row, col] = queue.front();
            queue.pop_front();
            const int next = distances_[goal_index][row][col] + 1;
            for (int i = 0; i < 4; ++i) {
                const int nr = row + dr[i];
                const int nc = col + dc[i];
                if (0 <= nr && nr < rows_ && 0 <= nc && nc < cols_ &&
                    !State::walls[nr][nc] && distances_[goal_index][nr][nc] == kInf) {
                    distances_[goal_index][nr][nc] = next;
                    queue.emplace_back(nr, nc);
                }
            }
        }
    }
};

class HeuristicAStar final : public Heuristic {
public:
    using Heuristic::Heuristic;
    int f(const State& state) const override { return state.g() + h(state); }
    std::string description() const override { return "A* evaluation"; }
};

class HeuristicWeightedAStar final : public Heuristic {
public:
    HeuristicWeightedAStar(const State& initial, int weight) : Heuristic(initial), weight_(weight) {}
    int f(const State& state) const override { return state.g() + weight_ * h(state); }
    std::string description() const override { return "WA*(" + std::to_string(weight_) + ") evaluation"; }

private:
    int weight_;
};

class HeuristicGreedy final : public Heuristic {
public:
    using Heuristic::Heuristic;
    int f(const State& state) const override { return h(state); }
    std::string description() const override { return "greedy evaluation"; }
};

class HeuristicGoalCountGreedy final : public Heuristic {
public:
    using Heuristic::Heuristic;
    int f(const State& state) const override { return h_goal_count(state); }
    std::string description() const override { return "greedy goal count evaluation"; }
};

class HeuristicGoalCountAStar final : public Heuristic {
public:
    using Heuristic::Heuristic;
    int f(const State& state) const override { return state.g() + h_goal_count(state); }
    std::string description() const override { return "A* goal count evaluation"; }
};

class HeuristicSmartGreedy final : public Heuristic {
public:
    using Heuristic::Heuristic;
    int f(const State& state) const override { return h_smart(state); }
    std::string description() const override { return "smart greedy matching/deadlock evaluation"; }
};

class HeuristicSmartWeightedAStar final : public Heuristic {
public:
    HeuristicSmartWeightedAStar(const State& initial, int weight) : Heuristic(initial), weight_(weight) {}
    int f(const State& state) const override { return state.g() + weight_ * h_smart(state); }
    std::string description() const override { return "smart WA*(" + std::to_string(weight_) + ") matching/deadlock evaluation"; }

private:
    int weight_;
};

class FrontierBFS final : public Frontier {
public:
    void add(const StatePtr& state) override { queue_.push_back(state); set_.insert(state); }
    StatePtr pop() override
    {
        StatePtr state = queue_.front();
        queue_.pop_front();
        set_.erase(state);
        return state;
    }
    bool is_empty() const override { return queue_.empty(); }
    int size() const override { return static_cast<int>(queue_.size()); }
    bool contains(const StatePtr& state) const override { return set_.find(state) != set_.end(); }
    std::string name() const override { return "breadth-first search"; }

private:
    std::deque<StatePtr> queue_;
    std::unordered_set<StatePtr, StatePtrHasher, StatePtrEqual> set_;
};

class FrontierDFS final : public Frontier {
public:
    void add(const StatePtr& state) override { stack_.push_back(state); set_.insert(state); }
    StatePtr pop() override
    {
        StatePtr state = stack_.back();
        stack_.pop_back();
        set_.erase(state);
        return state;
    }
    bool is_empty() const override { return stack_.empty(); }
    int size() const override { return static_cast<int>(stack_.size()); }
    bool contains(const StatePtr& state) const override { return set_.find(state) != set_.end(); }
    std::string name() const override { return "depth-first search"; }

private:
    std::vector<StatePtr> stack_;
    std::unordered_set<StatePtr, StatePtrHasher, StatePtrEqual> set_;
};

class FrontierBestFirst final : public Frontier {
public:
    explicit FrontierBestFirst(std::unique_ptr<Heuristic> heuristic)
        : heuristic_(std::move(heuristic))
    {
    }

    void add(const StatePtr& state) override
    {
        heap_.push_back(state);
        sift_up(static_cast<int>(heap_.size() - 1));
        set_.insert(state);
    }
    StatePtr pop() override
    {
        StatePtr state = heap_.front();
        set_.erase(state);
        if (heap_.size() == 1) {
            heap_.pop_back();
            return state;
        }

        heap_.front() = heap_.back();
        heap_.pop_back();
        sift_down(0);
        return state;
    }
    bool is_empty() const override { return heap_.empty(); }
    int size() const override { return static_cast<int>(heap_.size()); }
    bool contains(const StatePtr& state) const override { return set_.find(state) != set_.end(); }
    std::string name() const override { return "best-first search using " + heuristic_->description(); }

private:
    int compare(const StatePtr& lhs, const StatePtr& rhs) const
    {
        const int lhs_f = heuristic_->f(*lhs);
        const int rhs_f = heuristic_->f(*rhs);
        if (lhs_f < rhs_f) return -1;
        if (lhs_f > rhs_f) return 1;
        const int lhs_h = heuristic_->h_smart(*lhs);
        const int rhs_h = heuristic_->h_smart(*rhs);
        if (lhs_h < rhs_h) return -1;
        if (lhs_h > rhs_h) return 1;
        const int lhs_goals = heuristic_->h_goal_count(*lhs);
        const int rhs_goals = heuristic_->h_goal_count(*rhs);
        if (lhs_goals < rhs_goals) return -1;
        if (lhs_goals > rhs_goals) return 1;
        if (lhs->g() > rhs->g()) return -1;
        if (lhs->g() < rhs->g()) return 1;
        return 0;
    }

    void sift_up(int index)
    {
        StatePtr value = heap_[static_cast<std::size_t>(index)];
        while (index > 0) {
            const int parent = (index - 1) >> 1;
            if (compare(value, heap_[static_cast<std::size_t>(parent)]) >= 0) {
                break;
            }
            heap_[static_cast<std::size_t>(index)] = heap_[static_cast<std::size_t>(parent)];
            index = parent;
        }
        heap_[static_cast<std::size_t>(index)] = std::move(value);
    }

    void sift_down(int index)
    {
        StatePtr value = heap_[static_cast<std::size_t>(index)];
        const int size = static_cast<int>(heap_.size());
        const int half = size >> 1;
        while (index < half) {
            int child = (index << 1) + 1;
            int right = child + 1;
            if (right < size && compare(heap_[static_cast<std::size_t>(child)],
                                        heap_[static_cast<std::size_t>(right)]) > 0) {
                child = right;
            }
            if (compare(value, heap_[static_cast<std::size_t>(child)]) <= 0) {
                break;
            }
            heap_[static_cast<std::size_t>(index)] = heap_[static_cast<std::size_t>(child)];
            index = child;
        }
        heap_[static_cast<std::size_t>(index)] = std::move(value);
    }

    std::unique_ptr<Heuristic> heuristic_;
    std::vector<StatePtr> heap_;
    std::unordered_set<StatePtr, StatePtrHasher, StatePtrEqual> set_;
};

class Memory {
public:
    static double used_mb()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS_EX counters{};
        GetProcessMemoryInfo(GetCurrentProcess(),
                             reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters));
        return static_cast<double>(counters.PrivateUsage) / kMb;
#else
        struct rusage usage {};
        getrusage(RUSAGE_SELF, &usage);
        return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
    }

    static std::string string_rep()
    {
        std::ostringstream out;
        out << "[Used: " << std::fixed << std::setprecision(2) << used_mb() << " MB]";
        return out.str();
    }

private:
#ifdef _WIN32
    static constexpr double kMb = 1024.0 * 1024.0;
#endif
};

class GraphSearch {
public:
    static std::optional<std::vector<std::vector<const Action*>>> search(const StatePtr& initial_state,
                                                                         Frontier& frontier,
                                                                         double max_seconds = 0.0,
                                                                         std::size_t max_expanded = 0)
    {
        const auto search_start = std::chrono::steady_clock::now();
        frontier.add(initial_state);
        std::unordered_set<StatePtr, StatePtrHasher, StatePtrEqual> expanded;
        int iterations = 0;

        while (!frontier.is_empty()) {
            ++iterations;
            if (max_seconds > 0.0) {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed_seconds =
                    std::chrono::duration_cast<std::chrono::duration<double>>(now - search_start).count();
                if (elapsed_seconds >= max_seconds) {
                    std::cerr << "Search budget exhausted after "
                              << std::fixed << std::setprecision(3)
                              << elapsed_seconds << " s.\n";
                    profile_event("phase=graph_search result=time_budget expanded=" +
                                  std::to_string(expanded.size()) +
                                  " frontier=" + std::to_string(frontier.size()) +
                                  " seconds=" + std::to_string(elapsed_seconds));
                    return std::nullopt;
                }
            }
            if (max_expanded > 0 && expanded.size() >= max_expanded) {
                std::cerr << "Search expansion budget exhausted after "
                          << expanded.size() << " expanded states.\n";
                profile_event("phase=graph_search result=expansion_budget expanded=" +
                              std::to_string(expanded.size()) +
                              " frontier=" + std::to_string(frontier.size()) +
                              " seconds=" + std::to_string(elapsed_seconds_since(search_start)));
                return std::nullopt;
            }
            if (iterations % 1000 == 0) {
                print_status(expanded.size(), frontier);
            }

            StatePtr state = frontier.pop();
            if (state->is_goal_state()) {
                print_status(expanded.size(), frontier);
                profile_event("phase=graph_search result=success expanded=" +
                              std::to_string(expanded.size()) +
                              " frontier=" + std::to_string(frontier.size()) +
                              " seconds=" + std::to_string(elapsed_seconds_since(search_start)));
                return state->extract_plan();
            }
            if (max_seconds > 0.0 && max_expanded > 0) {
                static constexpr std::size_t kBudgetedBranchingCap = 1000000;
                const std::size_t branching = state->joint_action_upper_bound(kBudgetedBranchingCap);
                if (branching > kBudgetedBranchingCap) {
                    std::cerr << "Search branching budget exhausted before expanding state with at least "
                              << branching << " joint-action combinations.\n";
                    profile_event("phase=graph_search result=branching_budget expanded=" +
                                  std::to_string(expanded.size()) +
                                  " frontier=" + std::to_string(frontier.size()) +
                                  " branching=" + std::to_string(branching) +
                                  " seconds=" + std::to_string(elapsed_seconds_since(search_start)));
                    return std::nullopt;
                }
            }

            expanded.insert(state);
            for (const auto& next_state : state->get_expanded_states()) {
                if (expanded.find(next_state) == expanded.end() && !frontier.contains(next_state)) {
                    frontier.add(next_state);
                }
            }
        }

        profile_event("phase=graph_search result=frontier_empty expanded=" +
                      std::to_string(expanded.size()) +
                      " seconds=" + std::to_string(elapsed_seconds_since(search_start)));
        return std::nullopt;
    }

private:
    static inline const auto start_time_ = std::chrono::steady_clock::now();

    static void print_status(std::size_t expanded_size, const Frontier& frontier)
    {
        const auto now = std::chrono::steady_clock::now();
        const double seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
        std::cerr << "#Expanded: " << expanded_size
                  << ", #Frontier: " << frontier.size()
                  << ", #Generated: " << (expanded_size + static_cast<std::size_t>(frontier.size()))
                  << ", Time: " << std::fixed << std::setprecision(3) << seconds << " s\n"
                  << Memory::string_rep() << '\n';
    }
};

namespace prioritized {

struct Subtask {
    enum class Type { DeliverBox, ReachCell };
    Type type = Type::ReachCell;
    char box = '\0';
    int box_start_row = -1;
    int box_start_col = -1;
    int box_goal_row = -1;
    int box_goal_col = -1;
    int goal_depth = 0;
    int target_row = -1;
    int target_col = -1;
};

struct BoxTask {
    char box = '\0';
    int box_row = -1;
    int box_col = -1;
    int goal_row = -1;
    int goal_col = -1;
    int goal_depth = 0;
};

struct ReservationTable {
    std::unordered_set<long long> cells;
    std::unordered_set<long long> edges;

    static long long cell_key(int row, int col, int time)
    {
        return (static_cast<long long>(time) << 32) ^
               (static_cast<long long>(row) << 16) ^
               static_cast<unsigned int>(col);
    }

    static long long edge_key(int row1, int col1, int row2, int col2, int time)
    {
        long long key = static_cast<long long>(time);
        key = (key << 12) ^ row1;
        key = (key << 12) ^ col1;
        key = (key << 12) ^ row2;
        key = (key << 12) ^ col2;
        return key;
    }

    void reserve_cell(int row, int col, int from_time, int to_time)
    {
        if (row < 0 || col < 0) {
            return;
        }
        for (int time = std::max(0, from_time); time <= to_time; ++time) {
            cells.insert(cell_key(row, col, time));
        }
    }

    void reserve_edge(int from_row, int from_col, int to_row, int to_col, int time)
    {
        edges.insert(edge_key(from_row, from_col, to_row, to_col, time));
        edges.insert(edge_key(to_row, to_col, from_row, from_col, time));
    }

    bool blocked_cell(int row, int col, int time) const
    {
        return cells.find(cell_key(row, col, time)) != cells.end();
    }

    bool blocked_cell_between(int row, int col, int from_time, int to_time) const
    {
        for (int time = std::max(0, from_time); time <= to_time; ++time) {
            if (blocked_cell(row, col, time)) {
                return true;
            }
        }
        return false;
    }

    int last_blocked_cell_time(int row, int col, int to_time) const
    {
        for (int time = to_time; time >= 0; --time) {
            if (blocked_cell(row, col, time)) {
                return time;
            }
        }
        return -1;
    }

    bool blocked_edge(int from_row, int from_col, int to_row, int to_col, int time) const
    {
        return edges.find(edge_key(from_row, from_col, to_row, to_col, time)) != edges.end();
    }
};

struct AgentPlan {
    bool ok = false;
    std::vector<const Action*> actions;
    std::vector<std::pair<int, int>> agent_positions;
    std::vector<std::vector<std::tuple<int, int, int>>> box_paths;
    std::vector<char> box_path_boxes;
};

struct CommittedWorld {
    std::vector<std::pair<int, int>> agent_positions;
    std::vector<std::string> boxes;

    explicit CommittedWorld(const State& state)
        : boxes(state.boxes)
    {
        for (std::size_t agent = 0; agent < state.agent_rows.size(); ++agent) {
            agent_positions.push_back({state.agent_rows[agent], state.agent_cols[agent]});
        }
    }

    void apply_plan(int agent, const AgentPlan& plan)
    {
        if (!plan.agent_positions.empty() &&
            0 <= agent && agent < static_cast<int>(agent_positions.size())) {
            agent_positions[static_cast<std::size_t>(agent)] = plan.agent_positions.back();
        }
        for (std::size_t i = 0; i < plan.box_paths.size(); ++i) {
            const auto& path = plan.box_paths[i];
            if (path.empty()) {
                continue;
            }
            const char box = i < plan.box_path_boxes.size()
                                 ? plan.box_path_boxes[i]
                                 : '\0';
            if (box == '\0') {
                continue;
            }
            const int start_row = std::get<1>(path.front());
            const int start_col = std::get<2>(path.front());
            const int final_row = std::get<1>(path.back());
            const int final_col = std::get<2>(path.back());
            if (0 <= start_row && start_row < static_cast<int>(boxes.size()) &&
                0 <= start_col && start_col < static_cast<int>(boxes[start_row].size()) &&
                boxes[start_row][start_col] == box) {
                boxes[start_row][start_col] = '\0';
            }
            if (0 <= final_row && final_row < static_cast<int>(boxes.size()) &&
                0 <= final_col && final_col < static_cast<int>(boxes[final_row].size())) {
                boxes[final_row][final_col] = box;
            }
        }
    }
};

enum class ReservationPolicy {
    Conservative,
    Relaxed
};

using DistanceGrid = std::vector<std::vector<int>>;

DistanceGrid compute_bfs_from(int start_row, int start_col)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    DistanceGrid distances(rows, std::vector<int>(cols, Heuristic::kInf));
    if (start_row < 0 || start_row >= rows || start_col < 0 || start_col >= cols ||
        State::walls[start_row][start_col]) {
        return distances;
    }

    std::deque<std::pair<int, int>> queue;
    distances[start_row][start_col] = 0;
    queue.push_back({start_row, start_col});
    static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
    static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
    while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();
        const int next = distances[row][col] + 1;
        for (int i = 0; i < 4; ++i) {
            const int nr = row + dr[i];
            const int nc = col + dc[i];
            if (0 <= nr && nr < rows && 0 <= nc && nc < cols &&
                !State::walls[nr][nc] && distances[nr][nc] == Heuristic::kInf) {
                distances[nr][nc] = next;
                queue.push_back({nr, nc});
            }
        }
    }
    return distances;
}

const DistanceGrid& cached_bfs_from(int start_row, int start_col)
{
    const std::uint64_t key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(start_row)) << 32) |
        static_cast<std::uint32_t>(start_col);
    static std::unordered_map<std::uint64_t, DistanceGrid> cache;
    const auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }
    auto [inserted, _] = cache.emplace(key, compute_bfs_from(start_row, start_col));
    return inserted->second;
}

bool is_box(char value)
{
    return 'A' <= value && value <= 'Z';
}

bool basic_goal_feasibility(const State& state)
{
    std::array<int, 26> goal_counts{};
    std::array<int, 26> unsatisfied_goal_counts{};
    std::array<int, 26> box_counts{};
    for (int row = 0; row < static_cast<int>(State::goals.size()); ++row) {
        for (int col = 0; col < static_cast<int>(State::goals[row].size()); ++col) {
            const char goal = State::goals[row][col];
            if (is_box(goal)) {
                const int index = goal - 'A';
                ++goal_counts[static_cast<std::size_t>(index)];
                if (state.boxes[row][col] != goal) {
                    ++unsatisfied_goal_counts[static_cast<std::size_t>(index)];
                }
            } else if ('0' <= goal && goal <= '9') {
                const int agent = goal - '0';
                if (agent < 0 || agent >= static_cast<int>(state.agent_rows.size())) {
                    std::cerr << "Infeasible level: missing agent " << agent
                              << " for agent goal at (" << row << "," << col << ").\n";
                    return false;
                }
            }
        }
    }
    for (const auto& row : state.boxes) {
        for (char box : row) {
            if (is_box(box)) {
                ++box_counts[static_cast<std::size_t>(box - 'A')];
            }
        }
    }
    for (int letter = 0; letter < 26; ++letter) {
        const char box = static_cast<char>('A' + letter);
        if (goal_counts[static_cast<std::size_t>(letter)] >
            box_counts[static_cast<std::size_t>(letter)]) {
            std::cerr << "Infeasible level: " << goal_counts[static_cast<std::size_t>(letter)]
                      << " goal cells require box " << box << ", but only "
                      << box_counts[static_cast<std::size_t>(letter)] << " exist.\n";
            return false;
        }
        if (unsatisfied_goal_counts[static_cast<std::size_t>(letter)] == 0) {
            continue;
        }
        const Color box_color = State::box_colors[static_cast<std::size_t>(letter)];
        bool has_compatible_agent = false;
        for (Color agent_color : State::agent_colors) {
            if (agent_color == box_color) {
                has_compatible_agent = true;
                break;
            }
        }
        if (!has_compatible_agent) {
            std::cerr << "Infeasible level: unsatisfied box goal " << box
                      << " has no compatible-color agent.\n";
            return false;
        }
    }
    return true;
}

int passable_neighbor_count(int row, int col)
{
    static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
    static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    int count = 0;
    for (int i = 0; i < 4; ++i) {
        const int nr = row + dr[i];
        const int nc = col + dc[i];
        if (0 <= nr && nr < rows && 0 <= nc && nc < cols && !State::walls[nr][nc]) {
            ++count;
        }
    }
    return count;
}

int goal_corridor_depth(int start_row, int start_col)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    if (start_row < 0 || start_row >= rows || start_col < 0 || start_col >= cols ||
        State::walls[start_row][start_col]) {
        return 0;
    }
    if (passable_neighbor_count(start_row, start_col) >= 3) {
        return 0;
    }

    std::vector<std::vector<int>> distance(rows, std::vector<int>(cols, Heuristic::kInf));
    std::deque<std::pair<int, int>> queue;
    distance[start_row][start_col] = 0;
    queue.push_back({start_row, start_col});
    int farthest_corridor = 0;
    static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
    static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};

    while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();
        farthest_corridor = std::max(farthest_corridor, distance[row][col]);
        const int degree = passable_neighbor_count(row, col);
        if ((row != start_row || col != start_col) && degree >= 3) {
            return distance[row][col];
        }
        if (degree >= 3) {
            continue;
        }
        for (int i = 0; i < 4; ++i) {
            const int nr = row + dr[i];
            const int nc = col + dc[i];
            if (0 <= nr && nr < rows && 0 <= nc && nc < cols &&
                !State::walls[nr][nc] && distance[nr][nc] == Heuristic::kInf) {
                distance[nr][nc] = distance[row][col] + 1;
                queue.push_back({nr, nc});
            }
        }
    }
    return farthest_corridor;
}

struct FeasKey {
    int ar, ac, br, bc;
    bool operator==(const FeasKey& other) const
    {
        return ar == other.ar && ac == other.ac && br == other.br && bc == other.bc;
    }
};

struct FeasKeyHash {
    std::size_t operator()(const FeasKey& key) const
    {
        std::size_t seed = 1469598103934665603ULL;
        auto mix = [&seed](int value) {
            seed ^= static_cast<std::size_t>(static_cast<unsigned int>(value));
            seed *= 1099511628211ULL;
        };
        mix(key.ar);
        mix(key.ac);
        mix(key.br);
        mix(key.bc);
        return seed;
    }
};

int feasible_delivery_cost(int agent_row,
                           int agent_col,
                           int box_row,
                           int box_col,
                           int goal_row,
                           int goal_col,
                           const State& state,
                           const std::set<std::pair<int, int>>& clear_cells)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    const auto& dist_to_goal = cached_bfs_from(goal_row, goal_col);
    if (dist_to_goal[box_row][box_col] == Heuristic::kInf) {
        return Heuristic::kInf;
    }
    auto occupied = [&](int row, int col, int active_box_row, int active_box_col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols || State::walls[row][col]) {
            return true;
        }
        if (row == active_box_row && col == active_box_col) {
            return true;
        }
        if (state.boxes[row][col] != '\0' &&
            clear_cells.find({row, col}) == clear_cells.end() &&
            !(row == box_row && col == box_col)) {
            return true;
        }
        return false;
    };

    struct Node {
        int ar, ac, br, bc, g, f;
    };
    std::vector<Node> nodes;
    auto cmp = [&nodes](int lhs, int rhs) {
        if (nodes[lhs].f != nodes[rhs].f) {
            return nodes[lhs].f > nodes[rhs].f;
        }
        return nodes[lhs].g < nodes[rhs].g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
    std::unordered_map<FeasKey, int, FeasKeyHash> best;
    Node root{agent_row,
              agent_col,
              box_row,
              box_col,
              0,
              dist_to_goal[box_row][box_col] + std::abs(agent_row - box_row) + std::abs(agent_col - box_col)};
    nodes.push_back(root);
    open.push(0);
    best[{agent_row, agent_col, box_row, box_col}] = 0;

    int expansions = 0;
    const int expansion_cap = std::max(10000, rows * cols * 200);
    while (!open.empty() && expansions++ < expansion_cap) {
        const int index = open.top();
        open.pop();
        const Node current = nodes[static_cast<std::size_t>(index)];
        if (current.br == goal_row && current.bc == goal_col) {
            return current.g;
        }
        const auto best_it = best.find({current.ar, current.ac, current.br, current.bc});
        if (best_it != best.end() && best_it->second < current.g) {
            continue;
        }
        for (const Action& action : ACTIONS) {
            int nar = current.ar;
            int nac = current.ac;
            int nbr = current.br;
            int nbc = current.bc;
            bool ok = true;
            switch (action.type) {
                case ActionType::NoOp:
                    continue;
                case ActionType::Move:
                    nar += action.agent_row_delta;
                    nac += action.agent_col_delta;
                    if (occupied(nar, nac, current.br, current.bc)) {
                        ok = false;
                    }
                    break;
                case ActionType::Push: {
                    const int touched_row = current.ar + action.agent_row_delta;
                    const int touched_col = current.ac + action.agent_col_delta;
                    if (touched_row != current.br || touched_col != current.bc) {
                        ok = false;
                        break;
                    }
                    nbr = current.br + action.box_row_delta;
                    nbc = current.bc + action.box_col_delta;
                    nar = touched_row;
                    nac = touched_col;
                    if (occupied(nbr, nbc, current.br, current.bc)) {
                        ok = false;
                    }
                    break;
                }
                case ActionType::Pull: {
                    const int old_box_row = current.ar - action.box_row_delta;
                    const int old_box_col = current.ac - action.box_col_delta;
                    if (old_box_row != current.br || old_box_col != current.bc) {
                        ok = false;
                        break;
                    }
                    nar = current.ar + action.agent_row_delta;
                    nac = current.ac + action.agent_col_delta;
                    nbr = current.ar;
                    nbc = current.ac;
                    if (occupied(nar, nac, current.br, current.bc)) {
                        ok = false;
                    }
                    break;
                }
            }
            if (!ok || dist_to_goal[nbr][nbc] == Heuristic::kInf) {
                continue;
            }
            const int new_g = current.g + 1;
            const FeasKey key{nar, nac, nbr, nbc};
            const auto it = best.find(key);
            if (it != best.end() && it->second <= new_g) {
                continue;
            }
            best[key] = new_g;
            nodes.push_back({nar, nac, nbr, nbc, new_g, new_g + dist_to_goal[nbr][nbc]});
            open.push(static_cast<int>(nodes.size()) - 1);
        }
    }
    return Heuristic::kInf;
}

std::vector<BoxTask> build_box_tasks(const State& state)
{
    ScopedProfile profile("build_box_tasks");
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    std::vector<BoxTask> tasks;
    std::vector<bool> used_boxes;
    std::vector<std::tuple<char, int, int>> boxes;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (is_box(state.boxes[row][col]) && State::goals[row][col] != state.boxes[row][col]) {
                boxes.push_back({state.boxes[row][col], row, col});
                used_boxes.push_back(false);
            }
        }
    }

    std::set<std::pair<int, int>> movable_candidate_starts;
    for (const auto& [box, row, col] : boxes) {
        bool has_open_goal = false;
        for (int goal_row = 0; goal_row < rows && !has_open_goal; ++goal_row) {
            for (int goal_col = 0; goal_col < cols; ++goal_col) {
                if (State::goals[goal_row][goal_col] == box &&
                    state.boxes[goal_row][goal_col] != box) {
                    has_open_goal = true;
                    break;
                }
            }
        }
        if (has_open_goal) {
            movable_candidate_starts.insert({row, col});
        }
    }

    for (int goal_row = 0; goal_row < rows; ++goal_row) {
        for (int goal_col = 0; goal_col < cols; ++goal_col) {
            const char goal = State::goals[goal_row][goal_col];
            if (!is_box(goal) || state.boxes[goal_row][goal_col] == goal) {
                continue;
            }
            const auto& distances = cached_bfs_from(goal_row, goal_col);
            int best_box = -1;
            int best_cost = Heuristic::kInf;
            int best_distance = Heuristic::kInf;
            for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                if (used_boxes[static_cast<std::size_t>(i)] || std::get<0>(boxes[i]) != goal) {
                    continue;
                }
                const int row = std::get<1>(boxes[i]);
                const int col = std::get<2>(boxes[i]);
                if (distances[row][col] == Heuristic::kInf) {
                    continue;
                }

                int candidate_cost = Heuristic::kInf;
                for (int agent = 0; agent < static_cast<int>(state.agent_rows.size()); ++agent) {
                    if (State::agent_colors[agent] != State::box_colors[goal - 'A']) {
                        continue;
                    }
                    const int cost = feasible_delivery_cost(state.agent_rows[agent],
                                                            state.agent_cols[agent],
                                                            row,
                                                            col,
                                                            goal_row,
                                                            goal_col,
                                                            state,
                                                            movable_candidate_starts);
                    candidate_cost = std::min(candidate_cost, cost);
                }

                if (candidate_cost == Heuristic::kInf) {
                    candidate_cost = distances[row][col] + 10000;
                }

                if (candidate_cost < best_cost ||
                    (candidate_cost == best_cost && distances[row][col] < best_distance)) {
                    best_cost = candidate_cost;
                    best_distance = distances[row][col];
                    best_box = i;
                }
            }
            if (best_box >= 0) {
                used_boxes[static_cast<std::size_t>(best_box)] = true;
                tasks.push_back({goal,
                                 std::get<1>(boxes[best_box]),
                                 std::get<2>(boxes[best_box]),
                                 goal_row,
                                 goal_col,
                                 goal_corridor_depth(goal_row, goal_col)});
            }
        }
    }
    return tasks;
}

std::vector<std::vector<Subtask>> assign_tasks(const State& state)
{
    ScopedProfile profile("assign_tasks");
    const int num_agents = static_cast<int>(state.agent_rows.size());
    std::vector<std::vector<Subtask>> per_agent(static_cast<std::size_t>(num_agents));
    std::vector<std::pair<int, int>> virtual_positions;
    for (int agent = 0; agent < num_agents; ++agent) {
        virtual_positions.push_back({state.agent_rows[agent], state.agent_cols[agent]});
    }

    std::vector<BoxTask> remaining = build_box_tasks(state);
    std::set<std::pair<int, int>> chosen_box_starts;
    constexpr int kGoalDepthBias = 16;
    while (!remaining.empty()) {
        int best_task = -1;
        int best_agent = -1;
        int best_cost = Heuristic::kInf;
        for (int task_index = 0; task_index < static_cast<int>(remaining.size()); ++task_index) {
            const BoxTask& task = remaining[static_cast<std::size_t>(task_index)];
            for (int agent = 0; agent < num_agents; ++agent) {
                if (State::agent_colors[agent] != State::box_colors[task.box - 'A']) {
                    continue;
                }
                const auto [ar, ac] = virtual_positions[static_cast<std::size_t>(agent)];
                std::set<std::pair<int, int>> clear_cells = chosen_box_starts;
                clear_cells.insert({task.box_row, task.box_col});
                const int delivery_cost = feasible_delivery_cost(ar,
                                                                  ac,
                                                                  task.box_row,
                                                                  task.box_col,
                                                                  task.goal_row,
                                                                  task.goal_col,
                                                                  state,
                                                                  clear_cells);
                if (delivery_cost == Heuristic::kInf) {
                    continue;
                }
                const int load_penalty = static_cast<int>(per_agent[static_cast<std::size_t>(agent)].size()) * 8;
                const int cost = delivery_cost + load_penalty - task.goal_depth * kGoalDepthBias;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_agent = agent;
                    best_task = task_index;
                }
            }
        }
        if (best_agent < 0 || best_task < 0) {
            int fallback_cost = Heuristic::kInf;
            for (int task_index = 0; task_index < static_cast<int>(remaining.size()); ++task_index) {
                const BoxTask& task = remaining[static_cast<std::size_t>(task_index)];
                const auto& dist_to_box = cached_bfs_from(task.box_row, task.box_col);
                const auto& dist_to_goal = cached_bfs_from(task.goal_row, task.goal_col);
                for (int agent = 0; agent < num_agents; ++agent) {
                    if (State::agent_colors[agent] != State::box_colors[task.box - 'A']) {
                        continue;
                    }
                    const auto [ar, ac] = virtual_positions[static_cast<std::size_t>(agent)];
                    if (dist_to_box[ar][ac] == Heuristic::kInf ||
                        dist_to_goal[task.box_row][task.box_col] == Heuristic::kInf) {
                        continue;
                    }
                    const int cost = dist_to_box[ar][ac] +
                                     dist_to_goal[task.box_row][task.box_col] -
                                     task.goal_depth * kGoalDepthBias;
                    if (cost < fallback_cost) {
                        fallback_cost = cost;
                        best_agent = agent;
                        best_task = task_index;
                    }
                }
            }
        }

        if (best_agent >= 0 && best_task >= 0) {
            const BoxTask task = remaining[static_cast<std::size_t>(best_task)];
            Subtask subtask;
            subtask.type = Subtask::Type::DeliverBox;
            subtask.box = task.box;
            subtask.box_start_row = task.box_row;
            subtask.box_start_col = task.box_col;
            subtask.box_goal_row = task.goal_row;
            subtask.box_goal_col = task.goal_col;
            subtask.goal_depth = task.goal_depth;
            per_agent[static_cast<std::size_t>(best_agent)].push_back(subtask);
            virtual_positions[static_cast<std::size_t>(best_agent)] = {task.goal_row, task.goal_col};
            chosen_box_starts.insert({task.box_row, task.box_col});
            remaining.erase(remaining.begin() + best_task);
        } else {
            break;
        }
    }

    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const char goal = State::goals[row][col];
            if ('0' <= goal && goal <= '9') {
                const int agent = goal - '0';
                if (agent >= 0 && agent < num_agents) {
                    Subtask subtask;
                    subtask.type = Subtask::Type::ReachCell;
                    subtask.target_row = row;
                    subtask.target_col = col;
                    per_agent[static_cast<std::size_t>(agent)].push_back(subtask);
                }
            }
        }
    }
    return per_agent;
}

struct STAKey {
    int ar, ac, br, bc, time;
    bool operator==(const STAKey& other) const
    {
        return ar == other.ar && ac == other.ac &&
               br == other.br && bc == other.bc &&
               time == other.time;
    }
};

struct STAKeyHash {
    std::size_t operator()(const STAKey& key) const
    {
        std::size_t seed = 1469598103934665603ULL;
        auto mix = [&seed](int value) {
            seed ^= static_cast<std::size_t>(static_cast<unsigned int>(value));
            seed *= 1099511628211ULL;
        };
        mix(key.ar);
        mix(key.ac);
        mix(key.br);
        mix(key.bc);
        mix(key.time);
        return seed;
    }
};

struct STANode {
    int ar = -1, ac = -1, br = -1, bc = -1;
    int time = 0;
    int g = 0;
    int f = 0;
    int parent = -1;
    const Action* action = nullptr;
};

bool blocked_static(const std::vector<std::vector<bool>>& static_block, int row, int col)
{
    return row < 0 || row >= static_cast<int>(static_block.size()) ||
           col < 0 || col >= static_cast<int>(static_block[row].size()) ||
           static_block[row][col];
}

std::vector<const Action*> plan_subtask(int start_row,
                                        int start_col,
                                        int start_time,
                                        const Subtask& subtask,
                                        const std::vector<std::vector<bool>>& static_block,
                                        const ReservationTable& reservations,
                                        const std::set<std::pair<int, int>>& static_agent_positions,
                                        bool final_subtask,
                                        int horizon,
                                        double time_budget_seconds = 5.0,
                                        int expansion_budget = 500000)
{
    const auto start_time_chrono = std::chrono::steady_clock::now();
    auto log_subtask = [&](const std::string& result, int expansions, int action_count) {
        if (!profiling_enabled()) {
            return;
        }
        std::ostringstream out;
        out << "phase=plan_subtask"
            << " result=" << result
            << " type=" << (subtask.type == Subtask::Type::DeliverBox ? "deliver_box" : "reach_cell")
            << " start_time=" << start_time
            << " expansions=" << expansions
            << " actions=" << action_count
            << " seconds=" << std::fixed << std::setprecision(6)
            << elapsed_seconds_since(start_time_chrono);
        if (subtask.type == Subtask::Type::DeliverBox) {
            out << " box=" << subtask.box
                << " box_start=" << subtask.box_start_row << "," << subtask.box_start_col
                << " box_goal=" << subtask.box_goal_row << "," << subtask.box_goal_col;
        } else {
            out << " target=" << subtask.target_row << "," << subtask.target_col;
        }
        profile_event(out.str());
    };
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    const DistanceGrid& dist_to_goal =
        subtask.type == Subtask::Type::DeliverBox
            ? cached_bfs_from(subtask.box_goal_row, subtask.box_goal_col)
            : cached_bfs_from(subtask.target_row, subtask.target_col);
    const DistanceGrid* dist_to_box = nullptr;
    int earliest_final_time = 0;
    if (subtask.type == Subtask::Type::DeliverBox) {
        dist_to_box = &cached_bfs_from(subtask.box_start_row, subtask.box_start_col);
        earliest_final_time =
            reservations.last_blocked_cell_time(subtask.box_goal_row, subtask.box_goal_col, horizon) + 1;
    } else {
        if (final_subtask) {
            earliest_final_time =
                reservations.last_blocked_cell_time(subtask.target_row, subtask.target_col, horizon) + 1;
        }
    }

    auto heuristic = [&](int ar, int ac, int br, int bc) {
        if (subtask.type == Subtask::Type::DeliverBox) {
            if (br < 0 || dist_to_goal[br][bc] == Heuristic::kInf) {
                return Heuristic::kInf;
            }
            int value = dist_to_goal[br][bc];
            if (br == subtask.box_start_row && bc == subtask.box_start_col) {
                const int agent_distance = (*dist_to_box)[ar][ac];
                if (agent_distance == Heuristic::kInf) {
                    return Heuristic::kInf;
                }
                value += std::max(0, agent_distance - 1);
            }
            return value;
        }
        return dist_to_goal[ar][ac];
    };

    auto reached = [&](const STANode& node) {
        if (subtask.type == Subtask::Type::DeliverBox) {
            return node.br == subtask.box_goal_row &&
                   node.bc == subtask.box_goal_col &&
                   node.time >= earliest_final_time &&
                   !reservations.blocked_cell_between(node.br, node.bc, node.time, horizon) &&
                   (!final_subtask ||
                    !reservations.blocked_cell_between(node.ar, node.ac, node.time, horizon));
        }
        return node.ar == subtask.target_row &&
               node.ac == subtask.target_col &&
               node.time >= earliest_final_time &&
               (!final_subtask ||
                !reservations.blocked_cell_between(node.ar, node.ac, node.time, horizon));
    };

    auto occupied_by_unplanned_agent = [&](int row, int col) {
        return static_agent_positions.find({row, col}) != static_agent_positions.end();
    };

    std::vector<STANode> nodes;
    nodes.reserve(4096);
    std::unordered_map<STAKey, int, STAKeyHash> best_g;
    auto cmp = [&nodes](int lhs, int rhs) {
        if (nodes[lhs].f != nodes[rhs].f) {
            return nodes[lhs].f > nodes[rhs].f;
        }
        return nodes[lhs].g < nodes[rhs].g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    STANode root;
    root.ar = start_row;
    root.ac = start_col;
    if (subtask.type == Subtask::Type::DeliverBox) {
        root.br = subtask.box_start_row;
        root.bc = subtask.box_start_col;
    }
    root.time = start_time;
    const int root_h = heuristic(root.ar, root.ac, root.br, root.bc);
    if (root_h == Heuristic::kInf ||
        reservations.blocked_cell(root.ar, root.ac, root.time) ||
        (root.br >= 0 && reservations.blocked_cell(root.br, root.bc, root.time))) {
        log_subtask("root_blocked", 0, 0);
        return {};
    }
    root.f = root_h;
    nodes.push_back(root);
    open.push(0);
    best_g[{root.ar, root.ac, root.br, root.bc, root.time}] = 0;

    int expansions = 0;
    while (!open.empty()) {
        if ((expansions & 1023) == 0) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_seconds = std::chrono::duration<double>(now - start_time_chrono).count();
            if (elapsed_seconds > time_budget_seconds) {
                log_subtask("time_budget", expansions, 0);
                return {};
            }
        }

        const int index = open.top();
        open.pop();
        const STANode current = nodes[static_cast<std::size_t>(index)];
        const STAKey current_key{current.ar, current.ac, current.br, current.bc, current.time};
        const auto best_it = best_g.find(current_key);
        if (best_it != best_g.end() && best_it->second < current.g) {
            continue;
        }
        if (reached(current)) {
            std::vector<const Action*> actions;
            int cursor = index;
            while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                actions.push_back(nodes[static_cast<std::size_t>(cursor)].action);
                cursor = nodes[static_cast<std::size_t>(cursor)].parent;
            }
            std::reverse(actions.begin(), actions.end());
            log_subtask("success", expansions, static_cast<int>(actions.size()));
            return actions;
        }
        if (current.time >= horizon || ++expansions > expansion_budget) {
            continue;
        }

        for (const auto& action : ACTIONS) {
            int nar = current.ar;
            int nac = current.ac;
            int nbr = current.br;
            int nbc = current.bc;
            bool ok = true;

            switch (action.type) {
                case ActionType::NoOp:
                    break;
                case ActionType::Move:
                    nar += action.agent_row_delta;
                    nac += action.agent_col_delta;
                    if (nar < 0 || nar >= rows || nac < 0 || nac >= cols ||
                        State::walls[nar][nac] || blocked_static(static_block, nar, nac) ||
                        (nbr == nar && nbc == nac)) {
                        ok = false;
                    }
                    break;
                case ActionType::Push: {
                    if (nbr < 0) {
                        ok = false;
                        break;
                    }
                    const int touched_row = current.ar + action.agent_row_delta;
                    const int touched_col = current.ac + action.agent_col_delta;
                    if (touched_row != nbr || touched_col != nbc) {
                        ok = false;
                        break;
                    }
                    const int new_box_row = nbr + action.box_row_delta;
                    const int new_box_col = nbc + action.box_col_delta;
                    if (new_box_row < 0 || new_box_row >= rows || new_box_col < 0 || new_box_col >= cols ||
                        State::walls[new_box_row][new_box_col] ||
                        blocked_static(static_block, new_box_row, new_box_col)) {
                        ok = false;
                        break;
                    }
                    nar = touched_row;
                    nac = touched_col;
                    nbr = new_box_row;
                    nbc = new_box_col;
                    break;
                }
                case ActionType::Pull: {
                    if (nbr < 0) {
                        ok = false;
                        break;
                    }
                    const int old_box_row = current.ar - action.box_row_delta;
                    const int old_box_col = current.ac - action.box_col_delta;
                    if (old_box_row != nbr || old_box_col != nbc) {
                        ok = false;
                        break;
                    }
                    nar = current.ar + action.agent_row_delta;
                    nac = current.ac + action.agent_col_delta;
                    if (nar < 0 || nar >= rows || nac < 0 || nac >= cols ||
                        State::walls[nar][nac] || blocked_static(static_block, nar, nac)) {
                        ok = false;
                        break;
                    }
                    nbr = current.ar;
                    nbc = current.ac;
                    break;
                }
            }
            if (!ok) {
                continue;
            }

            const int next_time = current.time + 1;
            if (subtask.type == Subtask::Type::DeliverBox &&
                nbr == subtask.box_goal_row &&
                nbc == subtask.box_goal_col &&
                next_time < earliest_final_time) {
                continue;
            }
            if (subtask.type == Subtask::Type::ReachCell &&
                final_subtask &&
                nar == subtask.target_row &&
                nac == subtask.target_col &&
                next_time < earliest_final_time) {
                continue;
            }
            if (occupied_by_unplanned_agent(nar, nac) ||
                (action.type == ActionType::Push &&
                 occupied_by_unplanned_agent(nbr, nbc))) {
                continue;
            }
            if (reservations.blocked_cell(nar, nac, next_time) ||
                reservations.blocked_edge(current.ar, current.ac, nar, nac, next_time)) {
                continue;
            }
            if (nbr >= 0 &&
                (reservations.blocked_cell(nbr, nbc, next_time) ||
                 reservations.blocked_edge(current.br, current.bc, nbr, nbc, next_time))) {
                continue;
            }

            const int h = heuristic(nar, nac, nbr, nbc);
            if (h == Heuristic::kInf) {
                continue;
            }
            const int new_g = current.g + 1;
            const STAKey next_key{nar, nac, nbr, nbc, next_time};
            const auto it = best_g.find(next_key);
            if (it != best_g.end() && it->second <= new_g) {
                continue;
            }
            best_g[next_key] = new_g;
            STANode child;
            child.ar = nar;
            child.ac = nac;
            child.br = nbr;
            child.bc = nbc;
            child.time = next_time;
            child.g = new_g;
            child.f = new_g + h;
            child.parent = index;
            child.action = &action;
            nodes.push_back(child);
            open.push(static_cast<int>(nodes.size()) - 1);
        }
    }
    log_subtask("frontier_empty", expansions, 0);
    return {};
}

std::vector<std::vector<bool>> static_boxes_for_subtask(const std::vector<std::string>& base_boxes,
                                                        const std::vector<Subtask>& subtasks,
                                                        std::size_t current_index,
                                                        const std::set<std::pair<int, int>>& unblocked_box_starts)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    std::vector<std::vector<bool>> blocked(rows, std::vector<bool>(cols, false));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            blocked[row][col] = base_boxes[row][col] != '\0';
        }
    }
    for (const auto& cell : unblocked_box_starts) {
        blocked[cell.first][cell.second] = false;
    }
    for (std::size_t i = 0; i < subtasks.size(); ++i) {
        const Subtask& subtask = subtasks[i];
        if (subtask.type != Subtask::Type::DeliverBox) {
            continue;
        }
        blocked[subtask.box_start_row][subtask.box_start_col] = false;
        if (i < current_index) {
            blocked[subtask.box_goal_row][subtask.box_goal_col] = true;
        } else if (i > current_index) {
            blocked[subtask.box_start_row][subtask.box_start_col] = true;
        }
    }
    return blocked;
}

AgentPlan plan_agent(int agent,
                     const State& state,
                     const std::vector<Subtask>& subtasks,
                     const std::vector<std::string>& base_boxes,
                     const std::set<std::pair<int, int>>& unblocked_box_starts,
                     const ReservationTable& reservations,
                     const std::set<std::pair<int, int>>& static_agent_positions,
                     int horizon,
                     double subtask_time_budget_seconds)
{
    AgentPlan plan;
    int row = state.agent_rows[agent];
    int col = state.agent_cols[agent];
    int time = 0;
    plan.agent_positions.push_back({row, col});

    for (std::size_t i = 0; i < subtasks.size(); ++i) {
        const Subtask& subtask = subtasks[i];
        const auto static_block =
            static_boxes_for_subtask(base_boxes, subtasks, i, unblocked_box_starts);
        const std::vector<const Action*> actions =
            plan_subtask(row,
                         col,
                         time,
                         subtask,
                         static_block,
                         reservations,
                         static_agent_positions,
                         i + 1 == subtasks.size(),
                         horizon,
                         subtask_time_budget_seconds);
        if (!actions.empty() || (subtask.type == Subtask::Type::ReachCell &&
            row == subtask.target_row && col == subtask.target_col)) {
            int box_row = subtask.type == Subtask::Type::DeliverBox ? subtask.box_start_row : -1;
            int box_col = subtask.type == Subtask::Type::DeliverBox ? subtask.box_start_col : -1;
            std::vector<std::tuple<int, int, int>> box_path;
            if (box_row >= 0) {
                box_path.push_back({time, box_row, box_col});
            }
            for (const Action* action : actions) {
                const int old_row = row;
                const int old_col = col;
                switch (action->type) {
                    case ActionType::NoOp:
                        break;
                    case ActionType::Move:
                        row += action->agent_row_delta;
                        col += action->agent_col_delta;
                        break;
                    case ActionType::Push:
                        row += action->agent_row_delta;
                        col += action->agent_col_delta;
                        box_row += action->box_row_delta;
                        box_col += action->box_col_delta;
                        break;
                    case ActionType::Pull:
                        row += action->agent_row_delta;
                        col += action->agent_col_delta;
                        box_row = old_row;
                        box_col = old_col;
                        break;
                }
                ++time;
                plan.actions.push_back(action);
                plan.agent_positions.push_back({row, col});
                if (box_row >= 0) {
                    const auto& last = box_path.back();
                    if (std::get<1>(last) != box_row || std::get<2>(last) != box_col) {
                        box_path.push_back({time, box_row, box_col});
                    }
                }
            }
            if (!box_path.empty()) {
                plan.box_path_boxes.push_back(subtask.box);
                plan.box_paths.push_back(std::move(box_path));
            }
        } else {
            if (subtask.type == Subtask::Type::DeliverBox) {
                std::cerr << "Agent " << agent << " failed DeliverBox "
                          << subtask.box << " from ("
                          << subtask.box_start_row << "," << subtask.box_start_col
                          << ") to (" << subtask.box_goal_row << ","
                          << subtask.box_goal_col << ") at t=" << time << ".\n";
            } else {
                std::cerr << "Agent " << agent << " failed ReachCell ("
                          << subtask.target_row << "," << subtask.target_col
                          << ") at t=" << time << ".\n";
            }
            return plan;
        }
    }

    plan.ok = true;
    return plan;
}

void commit_plan(ReservationTable& reservations,
                 const AgentPlan& plan,
                 int horizon,
                 ReservationPolicy policy = ReservationPolicy::Conservative)
{
    ScopedProfile profile("commit_plan");
    const bool conservative = policy == ReservationPolicy::Conservative;
    const int last_time = static_cast<int>(plan.agent_positions.size()) - 1;
    for (int time = 0; time <= last_time; ++time) {
        reservations.reserve_cell(plan.agent_positions[time].first,
                                  plan.agent_positions[time].second,
                                  time,
                                  time);
        if (time > 0) {
            if (conservative && plan.agent_positions[time - 1] != plan.agent_positions[time]) {
                reservations.reserve_cell(plan.agent_positions[time].first,
                                          plan.agent_positions[time].second,
                                          time - 1,
                                          time - 1);
                reservations.reserve_cell(plan.agent_positions[time - 1].first,
                                          plan.agent_positions[time - 1].second,
                                          time,
                                          time);
            }
            reservations.reserve_edge(plan.agent_positions[time - 1].first,
                                      plan.agent_positions[time - 1].second,
                                      plan.agent_positions[time].first,
                                      plan.agent_positions[time].second,
                                      time);
        }
    }
    if (!plan.agent_positions.empty()) {
        reservations.reserve_cell(plan.agent_positions.back().first,
                                  plan.agent_positions.back().second,
                                  last_time,
                                  horizon);
    }
    for (const auto& box_path : plan.box_paths) {
        if (box_path.empty()) {
            continue;
        }
        reservations.reserve_cell(std::get<1>(box_path.front()),
                                  std::get<2>(box_path.front()),
                                  0,
                                  std::get<0>(box_path.front()));
        for (std::size_t i = 0; i < box_path.size(); ++i) {
            const int time = std::get<0>(box_path[i]);
            const int row = std::get<1>(box_path[i]);
            const int col = std::get<2>(box_path[i]);
            const int end_time = i + 1 < box_path.size()
                                     ? std::get<0>(box_path[i + 1])
                                     : horizon;
            reservations.reserve_cell(row, col, time, end_time);
            if (i > 0) {
                if (conservative) {
                    reservations.reserve_cell(row, col, time - 1, time - 1);
                    if (std::get<1>(box_path[i - 1]) != row ||
                        std::get<2>(box_path[i - 1]) != col) {
                        reservations.reserve_cell(std::get<1>(box_path[i - 1]),
                                                  std::get<2>(box_path[i - 1]),
                                                  time,
                                                  time);
                    }
                }
                reservations.reserve_edge(std::get<1>(box_path[i - 1]),
                                          std::get<2>(box_path[i - 1]),
                                          row,
                                          col,
                                          time);
            }
        }
    }
}

bool plan_is_server_valid(const StatePtr& initial,
                          const std::vector<std::vector<const Action*>>& joint_plan,
                          const std::string& attempt_label)
{
    ScopedProfile profile("plan_is_server_valid");
    StatePtr current = initial;
    auto describe_cell = [](const State& state, int row, int col) {
        std::ostringstream out;
        out << '(' << row << ',' << col << ')';
        if (row < 0 || row >= static_cast<int>(State::walls.size()) ||
            col < 0 || col >= static_cast<int>(State::walls[row].size())) {
            out << " out-of-bounds";
            return out.str();
        }
        if (State::walls[row][col]) {
            out << " wall";
        }
        const char box = state.boxes[row][col];
        if (box != '\0') {
            out << " box " << box;
        }
        const char agent = state.agent_at(row, col);
        if (agent != '\0') {
            out << " agent " << agent;
        }
        if (!State::walls[row][col] && box == '\0' && agent == '\0') {
            out << " free";
        }
        return out.str();
    };
    auto invalid_detail = [&](int agent, const Action& action, const State& state) {
        const int row = state.agent_rows[agent];
        const int col = state.agent_cols[agent];
        std::ostringstream out;
        out << "agent at " << describe_cell(state, row, col);
        switch (action.type) {
            case ActionType::NoOp:
                break;
            case ActionType::Move:
                out << ", move target "
                    << describe_cell(state, row + action.agent_row_delta, col + action.agent_col_delta);
                break;
            case ActionType::Push: {
                const int box_row = row + action.agent_row_delta;
                const int box_col = col + action.agent_col_delta;
                out << ", pushed cell " << describe_cell(state, box_row, box_col)
                    << ", box target "
                    << describe_cell(state, box_row + action.box_row_delta, box_col + action.box_col_delta);
                break;
            }
            case ActionType::Pull: {
                const int target_row = row + action.agent_row_delta;
                const int target_col = col + action.agent_col_delta;
                const int box_row = row - action.box_row_delta;
                const int box_col = col - action.box_col_delta;
                out << ", move target " << describe_cell(state, target_row, target_col)
                    << ", pulled cell " << describe_cell(state, box_row, box_col);
                break;
            }
        }
        return out.str();
    };
    for (std::size_t step = 0; step < joint_plan.size(); ++step) {
        const auto& joint_action = joint_plan[step];
        for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
            if (!current->is_applicable(agent, *joint_action[static_cast<std::size_t>(agent)])) {
                std::cerr << "Rejected prioritized " << attempt_label
                          << ": invalid action for agent " << agent
                          << " at step " << (step + 1) << " ("
                          << joint_action[static_cast<std::size_t>(agent)]->name << "); "
                          << invalid_detail(agent,
                                            *joint_action[static_cast<std::size_t>(agent)],
                                            *current)
                          << ".\n";
                return false;
            }
        }
        if (current->is_conflicting(joint_action)) {
            std::cerr << "Rejected prioritized " << attempt_label
                      << ": conflicting joint action at step " << (step + 1) << ".\n";
            return false;
        }
        current = std::make_shared<State>(current, joint_action);
    }
    if (!current->is_goal_state()) {
        std::cerr << "Rejected prioritized " << attempt_label
                  << ": executable plan did not reach all goals.\n";
        return false;
    }
    return true;
}

const Action* move_action_for_delta(int row_delta, int col_delta)
{
    for (const Action& action : ACTIONS) {
        if (action.type == ActionType::Move &&
            action.agent_row_delta == row_delta &&
            action.agent_col_delta == col_delta) {
            return &action;
        }
    }
    return nullptr;
}

std::vector<std::vector<bool>> static_boxes_except(const State& state, int active_box_row, int active_box_col)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    std::vector<std::vector<bool>> blocked(rows, std::vector<bool>(cols, false));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            blocked[row][col] = state.boxes[row][col] != '\0';
        }
    }
    if (active_box_row >= 0 && active_box_col >= 0) {
        blocked[active_box_row][active_box_col] = false;
    }
    return blocked;
}

std::pair<int, int> find_current_box_for_task(const State& state, const BoxTask& task)
{
    if (task.box_row >= 0 && task.box_col >= 0 &&
        task.box_row < static_cast<int>(state.boxes.size()) &&
        task.box_col < static_cast<int>(state.boxes[task.box_row].size()) &&
        state.boxes[task.box_row][task.box_col] == task.box) {
        return {task.box_row, task.box_col};
    }

    const auto& dist_to_goal = cached_bfs_from(task.goal_row, task.goal_col);
    int best_row = -1;
    int best_col = -1;
    int best_distance = Heuristic::kInf;
    for (int row = 0; row < static_cast<int>(state.boxes.size()); ++row) {
        for (int col = 0; col < static_cast<int>(state.boxes[row].size()); ++col) {
            if (state.boxes[row][col] != task.box || State::goals[row][col] == task.box) {
                continue;
            }
            const int distance = dist_to_goal[row][col];
            if (distance < best_distance) {
                best_distance = distance;
                best_row = row;
                best_col = col;
            }
        }
    }
    return {best_row, best_col};
}

int choose_serial_agent(const State& state, const BoxTask& task, int box_row, int box_col)
{
    int best_agent = -1;
    int best_cost = Heuristic::kInf;
    std::set<std::pair<int, int>> clear_cells;
    clear_cells.insert({box_row, box_col});
    const auto& dist_to_box = cached_bfs_from(box_row, box_col);
    const auto& dist_to_goal = cached_bfs_from(task.goal_row, task.goal_col);

    for (int agent = 0; agent < static_cast<int>(state.agent_rows.size()); ++agent) {
        if (State::agent_colors[agent] != State::box_colors[task.box - 'A']) {
            continue;
        }
        int cost = feasible_delivery_cost(state.agent_rows[agent],
                                          state.agent_cols[agent],
                                          box_row,
                                          box_col,
                                          task.goal_row,
                                          task.goal_col,
                                          state,
                                          clear_cells);
        if (cost == Heuristic::kInf) {
            const int agent_distance = dist_to_box[state.agent_rows[agent]][state.agent_cols[agent]];
            const int box_distance = dist_to_goal[box_row][box_col];
            if (agent_distance != Heuristic::kInf && box_distance != Heuristic::kInf) {
                cost = agent_distance + box_distance + 10000;
            }
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_agent = agent;
        }
    }
    return best_agent;
}

std::set<std::pair<int, int>> trace_forbidden_cells(const State& state,
                                                    int agent,
                                                    const Subtask& subtask,
                                                    const std::vector<const Action*>& actions)
{
    std::set<std::pair<int, int>> forbidden;
    int row = state.agent_rows[agent];
    int col = state.agent_cols[agent];
    int box_row = subtask.type == Subtask::Type::DeliverBox ? subtask.box_start_row : -1;
    int box_col = subtask.type == Subtask::Type::DeliverBox ? subtask.box_start_col : -1;
    forbidden.insert({row, col});
    if (box_row >= 0) {
        forbidden.insert({box_row, box_col});
    }

    for (const Action* action : actions) {
        const int old_row = row;
        const int old_col = col;
        switch (action->type) {
            case ActionType::NoOp:
                break;
            case ActionType::Move:
                row += action->agent_row_delta;
                col += action->agent_col_delta;
                break;
            case ActionType::Push:
                row += action->agent_row_delta;
                col += action->agent_col_delta;
                box_row += action->box_row_delta;
                box_col += action->box_col_delta;
                break;
            case ActionType::Pull:
                row += action->agent_row_delta;
                col += action->agent_col_delta;
                box_row = old_row;
                box_col = old_col;
                break;
        }
        forbidden.insert({row, col});
        if (box_row >= 0) {
            forbidden.insert({box_row, box_col});
        }
    }
    return forbidden;
}

int blocking_agent_for_action(const State& state, int active_agent, const Action& action)
{
    const int row = state.agent_rows[active_agent];
    const int col = state.agent_cols[active_agent];
    auto agent_index_at = [&](int query_row, int query_col) {
        const char value = state.agent_at(query_row, query_col);
        if ('0' <= value && value <= '9') {
            const int agent = value - '0';
            if (agent != active_agent) {
                return agent;
            }
        }
        return -1;
    };

    switch (action.type) {
        case ActionType::NoOp:
            return -1;
        case ActionType::Move:
            return agent_index_at(row + action.agent_row_delta, col + action.agent_col_delta);
        case ActionType::Push: {
            const int box_row = row + action.agent_row_delta;
            const int box_col = col + action.agent_col_delta;
            return agent_index_at(box_row + action.box_row_delta, box_col + action.box_col_delta);
        }
        case ActionType::Pull:
            return agent_index_at(row + action.agent_row_delta, col + action.agent_col_delta);
    }
    return -1;
}

std::vector<const Action*> plan_evicting_moves(const State& state,
                                               int agent,
                                               const std::set<std::pair<int, int>>& forbidden)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    const int start_row = state.agent_rows[agent];
    const int start_col = state.agent_cols[agent];
    static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
    static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};

    std::vector<std::vector<int>> parent_dir(rows, std::vector<int>(cols, -1));
    std::deque<std::pair<int, int>> queue;
    parent_dir[start_row][start_col] = 4;
    queue.push_back({start_row, start_col});

    auto blocked = [&](int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols || State::walls[row][col]) {
            return true;
        }
        if (state.boxes[row][col] != '\0') {
            return true;
        }
        const char other_agent = state.agent_at(row, col);
        if (other_agent != '\0' && other_agent != static_cast<char>('0' + agent)) {
            return true;
        }
        return false;
    };
    auto acceptable_target = [&](int row, int col, bool allow_goal_cell) {
        if (row == start_row && col == start_col) {
            return false;
        }
        if (forbidden.find({row, col}) != forbidden.end()) {
            return false;
        }
        return allow_goal_cell || State::goals[row][col] == '\0';
    };

    std::pair<int, int> fallback_target{-1, -1};
    while (!queue.empty()) {
        const auto [row, col] = queue.front();
        queue.pop_front();

        if (acceptable_target(row, col, false)) {
            fallback_target = {row, col};
            break;
        }
        if (fallback_target.first < 0 && acceptable_target(row, col, true)) {
            fallback_target = {row, col};
        }

        for (int dir = 0; dir < 4; ++dir) {
            const int nr = row + dr[dir];
            const int nc = col + dc[dir];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols ||
                parent_dir[nr][nc] != -1 || blocked(nr, nc)) {
                continue;
            }
            parent_dir[nr][nc] = dir;
            queue.push_back({nr, nc});
        }
    }

    if (fallback_target.first < 0) {
        return {};
    }

    std::vector<const Action*> reversed;
    int row = fallback_target.first;
    int col = fallback_target.second;
    while (row != start_row || col != start_col) {
        const int dir = parent_dir[row][col];
        const Action* action = move_action_for_delta(dr[dir], dc[dir]);
        if (action == nullptr) {
            return {};
        }
        reversed.push_back(action);
        row -= dr[dir];
        col -= dc[dir];
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

bool append_single_agent_action(StatePtr& current,
                                std::vector<std::vector<const Action*>>& joint_plan,
                                int agent,
                                const Action* action)
{
    if (!current->is_applicable(agent, *action)) {
        return false;
    }
    std::vector<const Action*> joint_action(current->agent_rows.size(), &ACTIONS[0]);
    joint_action[static_cast<std::size_t>(agent)] = action;
    if (current->is_conflicting(joint_action)) {
        return false;
    }
    current = std::make_shared<State>(current, joint_action);
    joint_plan.push_back(std::move(joint_action));
    return true;
}

bool evict_agent(StatePtr& current,
                 std::vector<std::vector<const Action*>>& joint_plan,
                 int blocker,
                 const std::set<std::pair<int, int>>& forbidden)
{
    ScopedProfile profile("evict_agent");
    const std::vector<const Action*> moves = plan_evicting_moves(*current, blocker, forbidden);
    if (moves.empty()) {
        return false;
    }
    for (const Action* move : moves) {
        if (!append_single_agent_action(current, joint_plan, blocker, move)) {
            return false;
        }
    }
    return true;
}

bool append_serial_actions(StatePtr& current,
                           std::vector<std::vector<const Action*>>& joint_plan,
                           int agent,
                           const Subtask& subtask,
                           const std::vector<const Action*>& actions)
{
    const std::set<std::pair<int, int>> forbidden =
        trace_forbidden_cells(*current, agent, subtask, actions);

    for (const Action* action : actions) {
        int evictions = 0;
        while (!current->is_applicable(agent, *action)) {
            const int blocker = blocking_agent_for_action(*current, agent, *action);
            if (blocker < 0 || evictions++ > static_cast<int>(current->agent_rows.size())) {
                return false;
            }
            if (!evict_agent(current, joint_plan, blocker, forbidden)) {
                return false;
            }
        }
        if (!append_single_agent_action(current, joint_plan, agent, action)) {
            return false;
        }
    }
    return true;
}

std::vector<std::vector<BoxTask>> serial_task_orders(const State& initial, const std::vector<BoxTask>& tasks)
{
    std::vector<std::vector<BoxTask>> orders;
    std::set<std::string> seen;
    auto signature = [](const std::vector<BoxTask>& order) {
        std::ostringstream out;
        for (const BoxTask& task : order) {
            out << task.box << ':' << task.box_row << ',' << task.box_col
                << "->" << task.goal_row << ',' << task.goal_col << ';';
        }
        return out.str();
    };
    auto add_order = [&](std::vector<BoxTask> order) {
        const std::string key = signature(order);
        if (seen.insert(key).second) {
            orders.push_back(std::move(order));
        }
    };

    add_order(tasks);

    auto reversed = tasks;
    std::reverse(reversed.begin(), reversed.end());
    add_order(std::move(reversed));

    auto deep_first = tasks;
    std::stable_sort(deep_first.begin(), deep_first.end(), [](const BoxTask& lhs, const BoxTask& rhs) {
        return lhs.goal_depth > rhs.goal_depth;
    });
    add_order(std::move(deep_first));

    auto shallow_first = tasks;
    std::stable_sort(shallow_first.begin(), shallow_first.end(), [](const BoxTask& lhs, const BoxTask& rhs) {
        return lhs.goal_depth < rhs.goal_depth;
    });
    add_order(std::move(shallow_first));

    auto nearest_first = tasks;
    std::stable_sort(nearest_first.begin(), nearest_first.end(), [&](const BoxTask& lhs, const BoxTask& rhs) {
        const int lhs_agent = choose_serial_agent(initial, lhs, lhs.box_row, lhs.box_col);
        const int rhs_agent = choose_serial_agent(initial, rhs, rhs.box_row, rhs.box_col);
        const int lhs_distance = lhs_agent < 0
                                     ? Heuristic::kInf
                                     : std::abs(initial.agent_rows[lhs_agent] - lhs.box_row) +
                                           std::abs(initial.agent_cols[lhs_agent] - lhs.box_col);
        const int rhs_distance = rhs_agent < 0
                                     ? Heuristic::kInf
                                     : std::abs(initial.agent_rows[rhs_agent] - rhs.box_row) +
                                           std::abs(initial.agent_cols[rhs_agent] - rhs.box_col);
        if (lhs_distance != rhs_distance) {
            return lhs_distance < rhs_distance;
        }
        if (lhs.goal_depth != rhs.goal_depth) {
            return lhs.goal_depth > rhs.goal_depth;
        }
        return lhs.goal_row < rhs.goal_row ||
               (lhs.goal_row == rhs.goal_row && lhs.goal_col < rhs.goal_col);
    });
    add_order(std::move(nearest_first));

    auto goal_top_left = tasks;
    std::stable_sort(goal_top_left.begin(), goal_top_left.end(), [](const BoxTask& lhs, const BoxTask& rhs) {
        return lhs.goal_row < rhs.goal_row ||
               (lhs.goal_row == rhs.goal_row && lhs.goal_col < rhs.goal_col);
    });
    add_order(std::move(goal_top_left));

    auto goal_bottom_right = tasks;
    std::stable_sort(goal_bottom_right.begin(), goal_bottom_right.end(), [](const BoxTask& lhs, const BoxTask& rhs) {
        return lhs.goal_row > rhs.goal_row ||
               (lhs.goal_row == rhs.goal_row && lhs.goal_col > rhs.goal_col);
    });
    add_order(std::move(goal_bottom_right));

    return orders;
}

StatePtr detached_state(const State& state)
{
    return std::make_shared<State>(state.agent_rows,
                                   state.agent_cols,
                                   State::agent_colors,
                                   State::walls,
                                   state.boxes,
                                   State::box_colors,
                                   State::goals);
}

int single_agent_heuristic(const State& state,
                           int agent,
                           const Subtask& subtask,
                           const std::vector<std::vector<int>>& dist_to_goal)
{
    if (subtask.type == Subtask::Type::ReachCell) {
        return dist_to_goal[state.agent_rows[agent]][state.agent_cols[agent]];
    }

    if (state.boxes[subtask.box_goal_row][subtask.box_goal_col] == subtask.box) {
        return 0;
    }

    int best = Heuristic::kInf;
    for (int row = 0; row < static_cast<int>(state.boxes.size()); ++row) {
        for (int col = 0; col < static_cast<int>(state.boxes[row].size()); ++col) {
            if (state.boxes[row][col] != subtask.box) {
                continue;
            }
            const int box_distance = dist_to_goal[row][col];
            if (box_distance == Heuristic::kInf) {
                continue;
            }
            const int agent_distance =
                std::abs(state.agent_rows[agent] - row) +
                std::abs(state.agent_cols[agent] - col);
            best = std::min(best, box_distance + agent_distance);
        }
    }
    return best;
}

std::vector<const Action*> plan_single_agent_state_search(const StatePtr& current,
                                                          int agent,
                                                          const Subtask& subtask,
                                                          double time_budget_seconds,
                                                          std::size_t expansion_budget)
{
    const auto started = std::chrono::steady_clock::now();
    StatePtr root = detached_state(*current);
    const auto& dist_to_goal =
        subtask.type == Subtask::Type::DeliverBox
            ? cached_bfs_from(subtask.box_goal_row, subtask.box_goal_col)
            : cached_bfs_from(subtask.target_row, subtask.target_col);

    auto reached = [&](const State& state) {
        if (subtask.type == Subtask::Type::DeliverBox) {
            return state.boxes[subtask.box_goal_row][subtask.box_goal_col] == subtask.box;
        }
        return state.agent_rows[agent] == subtask.target_row &&
               state.agent_cols[agent] == subtask.target_col;
    };

    struct Item {
        int f = 0;
        int h = 0;
        int g = 0;
        std::size_t order = 0;
        StatePtr state;
    };
    auto cmp = [](const Item& lhs, const Item& rhs) {
        if (lhs.f != rhs.f) {
            return lhs.f > rhs.f;
        }
        if (lhs.h != rhs.h) {
            return lhs.h > rhs.h;
        }
        if (lhs.g != rhs.g) {
            return lhs.g < rhs.g;
        }
        return lhs.order > rhs.order;
    };

    std::priority_queue<Item, std::vector<Item>, decltype(cmp)> open(cmp);
    std::unordered_set<StatePtr, StatePtrHasher, StatePtrEqual> expanded;
    std::size_t sequence = 0;
    const int root_h = single_agent_heuristic(*root, agent, subtask, dist_to_goal);
    if (root_h == Heuristic::kInf) {
        return {};
    }
    open.push({root_h, root_h, 0, sequence++, root});

    while (!open.empty()) {
        if ((expanded.size() & 511U) == 0U) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            if (elapsed >= time_budget_seconds || expanded.size() >= expansion_budget) {
                return {};
            }
        }

        Item item = open.top();
        open.pop();
        StatePtr state = item.state;
        if (expanded.find(state) != expanded.end()) {
            continue;
        }
        if (reached(*state)) {
            std::vector<const Action*> actions;
            for (const auto& joint_action : state->extract_plan()) {
                actions.push_back(joint_action[static_cast<std::size_t>(agent)]);
            }
            return actions;
        }
        expanded.insert(state);

        for (const Action& action : ACTIONS) {
            if (!state->is_applicable(agent, action)) {
                continue;
            }
            std::vector<const Action*> joint_action(state->agent_rows.size(), &ACTIONS[0]);
            joint_action[static_cast<std::size_t>(agent)] = &action;
            if (state->is_conflicting(joint_action)) {
                continue;
            }
            StatePtr child = std::make_shared<State>(state, joint_action);
            if (expanded.find(child) != expanded.end()) {
                continue;
            }
            const int h = single_agent_heuristic(*child, agent, subtask, dist_to_goal);
            if (h == Heuristic::kInf) {
                continue;
            }
            open.push({child->g() + h, h, child->g(), sequence++, child});
        }
    }

    return {};
}

std::set<std::pair<int, int>> rough_box_path_cells(int box_row, int box_col, int goal_row, int goal_col)
{
    std::set<std::pair<int, int>> path;
    const auto& distances = cached_bfs_from(goal_row, goal_col);
    if (box_row < 0 || box_col < 0 ||
        distances[box_row][box_col] == Heuristic::kInf) {
        return path;
    }
    static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
    static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
    int row = box_row;
    int col = box_col;
    path.insert({row, col});
    while (row != goal_row || col != goal_col) {
        int best_dir = -1;
        int best_distance = distances[row][col];
        for (int dir = 0; dir < 4; ++dir) {
            const int nr = row + dr[dir];
            const int nc = col + dc[dir];
            if (0 <= nr && nr < static_cast<int>(distances.size()) &&
                0 <= nc && nc < static_cast<int>(distances[nr].size()) &&
                distances[nr][nc] < best_distance) {
                best_distance = distances[nr][nc];
                best_dir = dir;
            }
        }
        if (best_dir < 0) {
            break;
        }
        row += dr[best_dir];
        col += dc[best_dir];
        path.insert({row, col});
    }
    return path;
}

std::vector<std::pair<int, int>> parking_cells_for_box(const State& state,
                                                       int box_row,
                                                       int box_col,
                                                       const std::set<std::pair<int, int>>& forbidden,
                                                       int limit)
{
    struct Candidate {
        int distance = 0;
        int degree = 0;
        int row = 0;
        int col = 0;
    };
    const auto& distances = cached_bfs_from(box_row, box_col);
    std::vector<Candidate> candidates;
    for (int row = 0; row < static_cast<int>(state.boxes.size()); ++row) {
        for (int col = 0; col < static_cast<int>(state.boxes[row].size()); ++col) {
            if (distances[row][col] == Heuristic::kInf ||
                state.boxes[row][col] != '\0' ||
                state.agent_at(row, col) != '\0' ||
                State::goals[row][col] != '\0' ||
                forbidden.find({row, col}) != forbidden.end()) {
                continue;
            }
            const int degree = passable_neighbor_count(row, col);
            if (degree < 2) {
                continue;
            }
            candidates.push_back({distances[row][col], degree, row, col});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.distance != rhs.distance) {
            return lhs.distance < rhs.distance;
        }
        if (lhs.degree != rhs.degree) {
            return lhs.degree > rhs.degree;
        }
        if (lhs.row != rhs.row) {
            return lhs.row < rhs.row;
        }
        return lhs.col < rhs.col;
    });

    std::vector<std::pair<int, int>> cells;
    for (const Candidate& candidate : candidates) {
        cells.push_back({candidate.row, candidate.col});
        if (static_cast<int>(cells.size()) >= limit) {
            break;
        }
    }
    return cells;
}

bool try_relocate_and_complete_task(StatePtr& current,
                                    std::vector<std::vector<const Action*>>& joint_plan,
                                    const BoxTask& active_task,
                                    int horizon,
                                    double relocation_budget_seconds,
                                    double active_budget_seconds)
{
    ScopedProfile profile("try_relocate_and_complete_task");
    const auto started = std::chrono::steady_clock::now();
    const double total_budget_seconds = relocation_budget_seconds + active_budget_seconds;
    auto remaining_total_budget = [&]() {
        return total_budget_seconds -
               std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    };
    std::set<std::pair<int, int>> forbidden =
        rough_box_path_cells(active_task.box_row,
                             active_task.box_col,
                             active_task.goal_row,
                             active_task.goal_col);
    forbidden.insert({active_task.goal_row, active_task.goal_col});

    struct CandidateBox {
        int distance = 0;
        char box = '\0';
        int row = 0;
        int col = 0;
    };
    std::vector<CandidateBox> candidates;
    for (int row = 0; row < static_cast<int>(current->boxes.size()); ++row) {
        for (int col = 0; col < static_cast<int>(current->boxes[row].size()); ++col) {
            const char box = current->boxes[row][col];
            if (!is_box(box) ||
                (row == active_task.box_row && col == active_task.box_col) ||
                State::goals[row][col] == box) {
                continue;
            }
            int distance = std::abs(row - active_task.box_row) +
                           std::abs(col - active_task.box_col);
            if (forbidden.find({row, col}) != forbidden.end()) {
                distance -= 1000;
            }
            candidates.push_back({distance, box, row, col});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const CandidateBox& lhs, const CandidateBox& rhs) {
        if (lhs.distance != rhs.distance) {
            return lhs.distance < rhs.distance;
        }
        if (lhs.row != rhs.row) {
            return lhs.row < rhs.row;
        }
        return lhs.col < rhs.col;
    });

    const int candidate_limit = std::min<int>(8, static_cast<int>(candidates.size()));
    for (int candidate_index = 0; candidate_index < candidate_limit; ++candidate_index) {
        const CandidateBox& candidate = candidates[static_cast<std::size_t>(candidate_index)];
        const std::vector<std::pair<int, int>> parking =
            parking_cells_for_box(*current, candidate.row, candidate.col, forbidden, 10);
        for (const auto& [park_row, park_col] : parking) {
            double remaining_total = remaining_total_budget();
            if (remaining_total <= 0.25) {
                return false;
            }
            StatePtr trial_state = current;
            auto trial_plan = joint_plan;

            BoxTask move_task;
            move_task.box = candidate.box;
            move_task.box_row = candidate.row;
            move_task.box_col = candidate.col;
            move_task.goal_row = park_row;
            move_task.goal_col = park_col;
            const int relocate_agent =
                choose_serial_agent(*trial_state, move_task, candidate.row, candidate.col);
            if (relocate_agent < 0) {
                continue;
            }

            Subtask relocate;
            relocate.type = Subtask::Type::DeliverBox;
            relocate.box = candidate.box;
            relocate.box_start_row = candidate.row;
            relocate.box_start_col = candidate.col;
            relocate.box_goal_row = park_row;
            relocate.box_goal_col = park_col;

            ReservationTable relocate_reservations;
            const auto relocate_static = static_boxes_except(*trial_state, candidate.row, candidate.col);
            const std::set<std::pair<int, int>> static_agent_positions;
            std::vector<const Action*> relocate_actions =
                plan_subtask(trial_state->agent_rows[relocate_agent],
                             trial_state->agent_cols[relocate_agent],
                             0,
                             relocate,
                             relocate_static,
                             relocate_reservations,
                             static_agent_positions,
                             true,
                             horizon,
                             std::min(relocation_budget_seconds, remaining_total),
                             200000);
            if (relocate_actions.empty()) {
                remaining_total = remaining_total_budget();
                if (remaining_total <= 0.25) {
                    return false;
                }
                relocate_actions = plan_single_agent_state_search(trial_state,
                                                                  relocate_agent,
                                                                  relocate,
                                                                  std::min(relocation_budget_seconds,
                                                                           remaining_total),
                                                                  75000);
            }
            if (relocate_actions.empty() ||
                !append_serial_actions(trial_state, trial_plan, relocate_agent, relocate, relocate_actions)) {
                continue;
            }

            if (trial_state->boxes[active_task.goal_row][active_task.goal_col] == active_task.box) {
                current = std::move(trial_state);
                joint_plan = std::move(trial_plan);
                return true;
            }

            const auto [active_box_row, active_box_col] =
                find_current_box_for_task(*trial_state, active_task);
            if (active_box_row < 0 || active_box_col < 0) {
                continue;
            }
            const int active_agent =
                choose_serial_agent(*trial_state, active_task, active_box_row, active_box_col);
            if (active_agent < 0) {
                continue;
            }
            remaining_total = remaining_total_budget();
            if (remaining_total <= 0.25) {
                return false;
            }

            Subtask active;
            active.type = Subtask::Type::DeliverBox;
            active.box = active_task.box;
            active.box_start_row = active_box_row;
            active.box_start_col = active_box_col;
            active.box_goal_row = active_task.goal_row;
            active.box_goal_col = active_task.goal_col;
            active.goal_depth = active_task.goal_depth;

            ReservationTable active_reservations;
            const auto active_static = static_boxes_except(*trial_state, active_box_row, active_box_col);
            std::vector<const Action*> active_actions =
                plan_subtask(trial_state->agent_rows[active_agent],
                             trial_state->agent_cols[active_agent],
                             0,
                             active,
                             active_static,
                             active_reservations,
                             static_agent_positions,
                             true,
                             horizon,
                             std::min(active_budget_seconds, remaining_total),
                             500000);
            if (active_actions.empty()) {
                remaining_total = remaining_total_budget();
                if (remaining_total <= 0.25) {
                    return false;
                }
                active_actions = plan_single_agent_state_search(trial_state,
                                                                active_agent,
                                                                active,
                                                                std::min(active_budget_seconds,
                                                                         remaining_total),
                                                                150000);
            }
            if (active_actions.empty() ||
                !append_serial_actions(trial_state, trial_plan, active_agent, active, active_actions) ||
                trial_state->boxes[active_task.goal_row][active_task.goal_col] != active_task.box) {
                continue;
            }

            std::cerr << "Serial fallback parked box " << candidate.box
                      << " from (" << candidate.row << ',' << candidate.col
                      << ") to (" << park_row << ',' << park_col
                      << ") and completed " << active_task.box << ".\n";
            current = std::move(trial_state);
            joint_plan = std::move(trial_plan);
            return true;
        }
    }

    return false;
}

bool append_agent_goal_moves(StatePtr& current,
                             std::vector<std::vector<const Action*>>& joint_plan,
                             double per_subtask_budget_seconds,
                             int horizon)
{
    const int num_agents = static_cast<int>(current->agent_rows.size());
    for (int pass = 0; pass < num_agents; ++pass) {
        bool changed = false;
        for (int row = 0; row < static_cast<int>(State::goals.size()); ++row) {
            for (int col = 0; col < static_cast<int>(State::goals[row].size()); ++col) {
                const char goal = State::goals[row][col];
                if (goal < '0' || goal > '9') {
                    continue;
                }
                const int agent = goal - '0';
                if (agent >= num_agents ||
                    (current->agent_rows[agent] == row && current->agent_cols[agent] == col)) {
                    continue;
                }
                Subtask reach;
                reach.type = Subtask::Type::ReachCell;
                reach.target_row = row;
                reach.target_col = col;
                ReservationTable reservations;
                const auto static_block = static_boxes_except(*current, -1, -1);
                const std::set<std::pair<int, int>> static_agent_positions;
                const std::vector<const Action*> actions =
                    plan_subtask(current->agent_rows[agent],
                                 current->agent_cols[agent],
                                 0,
                                 reach,
                                 static_block,
                                 reservations,
                                 static_agent_positions,
                                 true,
                                 horizon,
                                 per_subtask_budget_seconds,
                                 250000);
                if (actions.empty()) {
                    return false;
                }
                if (!append_serial_actions(current, joint_plan, agent, reach, actions)) {
                    return false;
                }
                changed = true;
            }
        }
        if (!changed || current->is_goal_state()) {
            return current->is_goal_state();
        }
    }
    return current->is_goal_state();
}

std::optional<std::vector<std::vector<const Action*>>> solve_serial(const StatePtr& initial,
                                                                    double total_budget_seconds = 15.0)
{
    ScopedProfile profile("solve_serial");
    const auto started = std::chrono::steady_clock::now();
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    const int horizon = std::min(20000, std::max(500, rows * cols * 12));
    const std::vector<BoxTask> tasks = build_box_tasks(*initial);
    int total_boxes = 0;
    for (const auto& row : initial->boxes) {
        for (char cell : row) {
            if (is_box(cell)) {
                ++total_boxes;
            }
        }
    }
    if (total_boxes > 16) {
        std::cerr << "Serial fallback skipped: " << total_boxes << " boxes.\n";
        return std::nullopt;
    }
    if (tasks.size() > 80) {
        std::cerr << "Serial fallback skipped: " << tasks.size() << " box tasks.\n";
        return std::nullopt;
    }

    const std::vector<std::vector<BoxTask>> orders = serial_task_orders(*initial, tasks);
    for (std::size_t order_index = 0; order_index < orders.size(); ++order_index) {
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        if (elapsed >= total_budget_seconds) {
            std::cerr << "Serial fallback budget exhausted.\n";
            break;
        }

        StatePtr current = initial;
        std::vector<std::vector<const Action*>> joint_plan;
        bool failed = false;
        const double per_subtask_budget_seconds = tasks.size() <= 12 ? 8.0 : 3.0;
        std::cerr << "Serial fallback order " << (order_index + 1)
                  << " with " << orders[order_index].size() << " box tasks.\n";

        for (const BoxTask& base_task : orders[order_index]) {
            const double elapsed_before_task =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            if (elapsed_before_task >= total_budget_seconds - 0.25) {
                failed = true;
                break;
            }
            if (current->is_goal_state()) {
                break;
            }
            if (current->boxes[base_task.goal_row][base_task.goal_col] == base_task.box) {
                continue;
            }
            const auto [box_row, box_col] = find_current_box_for_task(*current, base_task);
            if (box_row < 0 || box_col < 0) {
                failed = true;
                break;
            }
            if (box_row == base_task.goal_row && box_col == base_task.goal_col) {
                continue;
            }

            const int agent = choose_serial_agent(*current, base_task, box_row, box_col);
            if (agent < 0) {
                failed = true;
                break;
            }

            Subtask subtask;
            subtask.type = Subtask::Type::DeliverBox;
            subtask.box = base_task.box;
            subtask.box_start_row = box_row;
            subtask.box_start_col = box_col;
            subtask.box_goal_row = base_task.goal_row;
            subtask.box_goal_col = base_task.goal_col;
            subtask.goal_depth = base_task.goal_depth;
            int executing_agent = agent;

            ReservationTable reservations;
            const auto static_block = static_boxes_except(*current, box_row, box_col);
            const std::set<std::pair<int, int>> static_agent_positions;
            const double order_elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            const double remaining_budget = total_budget_seconds - order_elapsed;
            if (remaining_budget <= 0.25) {
                failed = true;
                break;
            }
            std::vector<const Action*> actions =
                plan_subtask(current->agent_rows[agent],
                             current->agent_cols[agent],
                             0,
                             subtask,
                             static_block,
                             reservations,
                             static_agent_positions,
                             true,
                             horizon,
                             std::min(per_subtask_budget_seconds, remaining_budget),
                             500000);
            if (actions.empty()) {
                actions = plan_single_agent_state_search(current,
                                                         agent,
                                                         subtask,
                                                         std::min(per_subtask_budget_seconds, remaining_budget),
                                                         tasks.size() <= 12 ? 200000 : 75000);
            }
            bool completed_by_relocation = false;
            if (actions.empty() && tasks.size() <= 20) {
                BoxTask current_task = base_task;
                current_task.box_row = box_row;
                current_task.box_col = box_col;
                const double relocate_budget = std::min(2.5, remaining_budget);
                completed_by_relocation =
                    try_relocate_and_complete_task(current,
                                                   joint_plan,
                                                   current_task,
                                                   horizon,
                                                   relocate_budget,
                                                   std::min(per_subtask_budget_seconds, remaining_budget));
            }
            if (completed_by_relocation) {
                continue;
            }
            if (actions.empty()) {
                std::cerr << "Serial fallback failed to plan " << base_task.box
                          << " from (" << box_row << ',' << box_col
                          << ") to (" << base_task.goal_row << ','
                          << base_task.goal_col << ") with agent " << executing_agent << ".\n";
                failed = true;
                break;
            }
            if (!append_serial_actions(current, joint_plan, executing_agent, subtask, actions)) {
                std::cerr << "Serial fallback failed to execute " << base_task.box
                          << " from (" << box_row << ',' << box_col
                          << ") to (" << base_task.goal_row << ','
                          << base_task.goal_col << ") with agent " << executing_agent << ".\n";
                failed = true;
                break;
            }
            if (static_cast<int>(joint_plan.size()) > horizon) {
                failed = true;
                break;
            }
        }

        if (!failed && !current->is_goal_state()) {
            const double elapsed_after_boxes =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            const double remaining = total_budget_seconds - elapsed_after_boxes;
            failed = remaining <= 0.25 ||
                     !append_agent_goal_moves(current,
                                              joint_plan,
                                              std::min(3.0, remaining),
                                              horizon);
        }

        const std::string attempt_label =
            "serial fallback order " + std::to_string(order_index + 1);
        if (!failed && plan_is_server_valid(initial, joint_plan, attempt_label)) {
            return joint_plan;
        }
    }

    return std::nullopt;
}

std::optional<std::vector<std::vector<const Action*>>> solve(const StatePtr& initial)
{
    ScopedProfile profile("prioritized_solve");
    const int num_agents = static_cast<int>(initial->agent_rows.size());
    if (num_agents == 0) {
        return std::vector<std::vector<const Action*>>{};
    }
    const int rows = static_cast<int>(State::walls.size());
    const int cols = rows == 0 ? 0 : static_cast<int>(State::walls[0].size());
    const int horizon = std::min(20000, std::max(300, rows * cols * 8));
    std::vector<std::pair<int, int>> initial_agent_positions;
    for (int agent = 0; agent < num_agents; ++agent) {
        initial_agent_positions.push_back({initial->agent_rows[agent], initial->agent_cols[agent]});
    }
    std::vector<std::vector<std::vector<Subtask>>> subtask_variants;
    auto append_task_variants = [&](const std::vector<std::vector<Subtask>>& seed_subtasks) {
        subtask_variants.push_back(seed_subtasks);

        auto deep_goal_subtasks = seed_subtasks;
        for (int agent = 0; agent < num_agents; ++agent) {
            auto& tasks = deep_goal_subtasks[static_cast<std::size_t>(agent)];
            std::stable_sort(tasks.begin(), tasks.end(), [&](const Subtask& lhs, const Subtask& rhs) {
                if (lhs.type != rhs.type) {
                    return lhs.type == Subtask::Type::DeliverBox;
                }
                if (lhs.type != Subtask::Type::DeliverBox) {
                    return false;
                }
                if (lhs.goal_depth != rhs.goal_depth) {
                    return lhs.goal_depth > rhs.goal_depth;
                }
                const int lhs_distance =
                    std::abs(lhs.box_goal_row - initial->agent_rows[agent]) +
                    std::abs(lhs.box_goal_col - initial->agent_cols[agent]);
                const int rhs_distance =
                    std::abs(rhs.box_goal_row - initial->agent_rows[agent]) +
                    std::abs(rhs.box_goal_col - initial->agent_cols[agent]);
                return lhs_distance > rhs_distance;
            });
        }
        subtask_variants.push_back(std::move(deep_goal_subtasks));

        auto far_first_subtasks = seed_subtasks;
        for (int agent = 0; agent < num_agents; ++agent) {
            auto& tasks = far_first_subtasks[static_cast<std::size_t>(agent)];
            std::stable_sort(tasks.begin(), tasks.end(), [&](const Subtask& lhs, const Subtask& rhs) {
                if (lhs.type != rhs.type) {
                    return lhs.type == Subtask::Type::DeliverBox;
                }
                if (lhs.type != Subtask::Type::DeliverBox) {
                    return false;
                }
                const int lhs_distance =
                    std::abs(lhs.box_goal_row - initial->agent_rows[agent]) +
                    std::abs(lhs.box_goal_col - initial->agent_cols[agent]);
                const int rhs_distance =
                    std::abs(rhs.box_goal_row - initial->agent_rows[agent]) +
                    std::abs(rhs.box_goal_col - initial->agent_cols[agent]);
                if (lhs_distance != rhs_distance) {
                    return lhs_distance > rhs_distance;
                }
                if (lhs.box_goal_row != rhs.box_goal_row) {
                    return lhs.box_goal_row < rhs.box_goal_row;
                }
                return lhs.box_goal_col > rhs.box_goal_col;
            });
        }
        subtask_variants.push_back(std::move(far_first_subtasks));

        auto goal_top_left_subtasks = seed_subtasks;
        for (int agent = 0; agent < num_agents; ++agent) {
            auto& tasks = goal_top_left_subtasks[static_cast<std::size_t>(agent)];
            std::stable_sort(tasks.begin(), tasks.end(), [](const Subtask& lhs, const Subtask& rhs) {
                if (lhs.type != rhs.type) {
                    return lhs.type == Subtask::Type::DeliverBox;
                }
                if (lhs.type != Subtask::Type::DeliverBox) {
                    return false;
                }
                if (lhs.box_goal_row != rhs.box_goal_row) {
                    return lhs.box_goal_row < rhs.box_goal_row;
                }
                if (lhs.box_goal_col != rhs.box_goal_col) {
                    return lhs.box_goal_col < rhs.box_goal_col;
                }
                return lhs.goal_depth > rhs.goal_depth;
            });
        }
        subtask_variants.push_back(std::move(goal_top_left_subtasks));

        auto goal_bottom_right_subtasks = seed_subtasks;
        for (int agent = 0; agent < num_agents; ++agent) {
            auto& tasks = goal_bottom_right_subtasks[static_cast<std::size_t>(agent)];
            std::stable_sort(tasks.begin(), tasks.end(), [](const Subtask& lhs, const Subtask& rhs) {
                if (lhs.type != rhs.type) {
                    return lhs.type == Subtask::Type::DeliverBox;
                }
                if (lhs.type != Subtask::Type::DeliverBox) {
                    return false;
                }
                if (lhs.box_goal_row != rhs.box_goal_row) {
                    return lhs.box_goal_row > rhs.box_goal_row;
                }
                if (lhs.box_goal_col != rhs.box_goal_col) {
                    return lhs.box_goal_col > rhs.box_goal_col;
                }
                return lhs.goal_depth > rhs.goal_depth;
            });
        }
        subtask_variants.push_back(std::move(goal_bottom_right_subtasks));

    };

    append_task_variants(assign_tasks(*initial));

    for (std::size_t variant_index = 0; variant_index < subtask_variants.size(); ++variant_index) {
    const auto& subtasks = subtask_variants[variant_index];
    std::cerr << "Prioritized task counts:";
    for (int agent = 0; agent < num_agents; ++agent) {
        std::cerr << " a" << agent << "=" << subtasks[static_cast<std::size_t>(agent)].size();
    }
    std::cerr << '\n';
    for (int agent = 0; agent < num_agents; ++agent) {
        if (subtasks[static_cast<std::size_t>(agent)].empty()) {
            continue;
        }
        std::cerr << "Agent " << agent << " tasks:";
        for (const Subtask& subtask : subtasks[static_cast<std::size_t>(agent)]) {
            if (subtask.type == Subtask::Type::DeliverBox) {
                std::cerr << ' ' << subtask.box << '('
                          << subtask.box_start_row << ',' << subtask.box_start_col
                          << "->" << subtask.box_goal_row << ',' << subtask.box_goal_col
                          << ')';
            } else {
                std::cerr << " reach(" << subtask.target_row << ',' << subtask.target_col << ')';
            }
        }
        std::cerr << '\n';
    }
    std::vector<std::vector<int>> orders;
    std::vector<int> base_order(num_agents);
    for (int agent = 0; agent < num_agents; ++agent) {
        base_order[agent] = agent;
    }
    std::vector<int> heavy_first = base_order;
    std::sort(heavy_first.begin(), heavy_first.end(), [&](int lhs, int rhs) {
        return subtasks[lhs].size() > subtasks[rhs].size();
    });
    std::vector<int> light_first = base_order;
    std::sort(light_first.begin(), light_first.end(), [&](int lhs, int rhs) {
        return subtasks[lhs].size() < subtasks[rhs].size();
    });
    orders.push_back(heavy_first);
    orders.push_back(base_order);
    orders.push_back(light_first);
    for (int preferred = 0; preferred < num_agents; ++preferred) {
        if (subtasks[preferred].empty()) {
            continue;
        }
        std::vector<int> order = heavy_first;
        order.erase(std::remove(order.begin(), order.end(), preferred), order.end());
        order.insert(order.begin(), preferred);
        orders.push_back(std::move(order));
    }

    for (std::size_t attempt = 0; attempt < orders.size(); ++attempt) {
        for (int block_later_agents_mode = 0; block_later_agents_mode < 2; ++block_later_agents_mode) {
            const bool block_later_agents = block_later_agents_mode == 0;
            std::cerr << "Prioritized attempt " << (attempt + 1)
                      << (block_later_agents ? " conservative" : " movable-agents")
                      << " order:";
            for (int agent : orders[attempt]) {
                std::cerr << ' ' << agent;
            }
            std::cerr << '\n';

            struct CandidateResult {
                bool completed = false;
                std::optional<std::vector<std::vector<const Action*>>> plan;
            };

            auto run_candidate = [&](bool use_committed_box_grid,
                                     ReservationPolicy reservation_policy,
                                     const std::string& mode_label) -> CandidateResult {
                ReservationTable reservations;
                CommittedWorld committed(*initial);
                std::set<std::pair<int, int>> planned_box_starts;
                std::vector<AgentPlan> plans(static_cast<std::size_t>(num_agents));
                const double subtask_time_budget_seconds = block_later_agents ? 5.0 : 2.0;
                bool failed = false;
                for (std::size_t order_index = 0; order_index < orders[attempt].size(); ++order_index) {
                    const int agent = orders[attempt][order_index];
                    std::set<std::pair<int, int>> static_agent_positions;
                    if (block_later_agents) {
                        for (std::size_t later = order_index + 1; later < orders[attempt].size(); ++later) {
                            const int other = orders[attempt][later];
                            static_agent_positions.insert(initial_agent_positions[static_cast<std::size_t>(other)]);
                        }
                    }
                    const std::set<std::pair<int, int>> empty_unblocked;
                    const auto& base_boxes = use_committed_box_grid ? committed.boxes : initial->boxes;
                    const auto& unblocked_starts = use_committed_box_grid ? empty_unblocked : planned_box_starts;
                    AgentPlan plan = plan_agent(agent,
                                                *initial,
                                                subtasks[static_cast<std::size_t>(agent)],
                                                base_boxes,
                                                unblocked_starts,
                                                reservations,
                                                static_agent_positions,
                                                horizon,
                                                subtask_time_budget_seconds);
                    if (!plan.ok) {
                        std::cerr << "Prioritized planner failed for agent " << agent << ".\n";
                        failed = true;
                        break;
                    }
                    commit_plan(reservations, plan, horizon, reservation_policy);
                    if (use_committed_box_grid) {
                        committed.apply_plan(agent, plan);
                    } else {
                        for (const auto& box_path : plan.box_paths) {
                            if (!box_path.empty()) {
                                planned_box_starts.insert({std::get<1>(box_path.front()),
                                                           std::get<2>(box_path.front())});
                            }
                        }
                        for (const Subtask& subtask : subtasks[static_cast<std::size_t>(agent)]) {
                            if (subtask.type == Subtask::Type::DeliverBox) {
                                planned_box_starts.insert({subtask.box_start_row, subtask.box_start_col});
                            }
                        }
                    }
                    plans[static_cast<std::size_t>(agent)] = std::move(plan);
                }
                if (failed) {
                    return {};
                }

                int total_time = 0;
                for (const AgentPlan& plan : plans) {
                    total_time = std::max(total_time, static_cast<int>(plan.actions.size()));
                }
                std::vector<std::vector<const Action*>> joint_plan(
                    static_cast<std::size_t>(total_time),
                    std::vector<const Action*>(static_cast<std::size_t>(num_agents), &ACTIONS[0]));
                for (int agent = 0; agent < num_agents; ++agent) {
                    const auto& actions = plans[static_cast<std::size_t>(agent)].actions;
                    for (int time = 0; time < static_cast<int>(actions.size()); ++time) {
                        joint_plan[static_cast<std::size_t>(time)][static_cast<std::size_t>(agent)] = actions[time];
                    }
                }
                const std::string attempt_label =
                    "variant " + std::to_string(variant_index + 1) +
                    ", attempt " + std::to_string(attempt + 1) +
                    (block_later_agents ? ", conservative" : ", movable-agents") +
                    mode_label;
                if (plan_is_server_valid(initial, joint_plan, attempt_label)) {
                    return {true, joint_plan};
                }
                return {true, std::nullopt};
            };

            CandidateResult candidate =
                run_candidate(false, ReservationPolicy::Conservative, "");
            if (candidate.plan.has_value()) {
                return candidate.plan;
            }
            if (candidate.completed) {
                CandidateResult committed_candidate =
                    run_candidate(true, ReservationPolicy::Conservative, ", committed-world");
                if (committed_candidate.plan.has_value()) {
                    return committed_candidate.plan;
                }
                CandidateResult relaxed_candidate =
                    run_candidate(false, ReservationPolicy::Relaxed, ", relaxed-reservations");
                if (relaxed_candidate.plan.has_value()) {
                    return relaxed_candidate.plan;
                }
                if (relaxed_candidate.completed) {
                    CandidateResult committed_relaxed_candidate =
                        run_candidate(true,
                                      ReservationPolicy::Relaxed,
                                      ", committed-world, relaxed-reservations");
                    if (committed_relaxed_candidate.plan.has_value()) {
                        return committed_relaxed_candidate.plan;
                    }
                }
            }
        }
    }
    }
    return std::nullopt;
}

} // namespace prioritized

StatePtr parse_level(std::istream& input)
{
    std::string line;
    std::getline(input, line);
    std::getline(input, line);
    std::getline(input, line);
    std::getline(input, line);

    std::getline(input, line);
    std::vector<Color> agent_colors(10, Color::Unknown);
    std::vector<Color> box_colors(26, Color::Unknown);
    std::getline(input, line);
    while (!line.empty() && line[0] != '#') {
        const auto parts = split(line, ':');
        if (parts.size() == 2) {
            const Color color = color_from_string(trim(parts[0]));
            for (const auto& entity : split(parts[1], ',')) {
                const std::string cleaned = trim(entity);
                if (cleaned.empty()) {
                    continue;
                }
                const char c = cleaned[0];
                if ('0' <= c && c <= '9') {
                    agent_colors[c - '0'] = color;
                } else if ('A' <= c && c <= 'Z') {
                    box_colors[c - 'A'] = color;
                }
            }
        }
        std::getline(input, line);
    }

    int num_rows = 0;
    int num_cols = 0;
    std::vector<std::string> level_lines;
    std::getline(input, line);
    while (!line.empty() && line[0] != '#') {
        level_lines.push_back(line);
        num_cols = std::max(num_cols, static_cast<int>(line.size()));
        ++num_rows;
        std::getline(input, line);
    }

    int num_agents = 0;
    std::vector<int> agent_rows(10, 0);
    std::vector<int> agent_cols(10, 0);
    std::vector<std::vector<bool>> walls(num_rows, std::vector<bool>(num_cols, false));
    std::vector<std::string> boxes(num_rows, std::string(static_cast<std::size_t>(num_cols), '\0'));

    for (int row = 0; row < num_rows; ++row) {
        for (int col = 0; col < static_cast<int>(level_lines[row].size()); ++col) {
            const char c = level_lines[row][col];
            if ('0' <= c && c <= '9') {
                agent_rows[c - '0'] = row;
                agent_cols[c - '0'] = col;
                num_agents = std::max(num_agents, c - '0' + 1);
            } else if ('A' <= c && c <= 'Z') {
                boxes[row][col] = c;
            } else if (c == '+') {
                walls[row][col] = true;
            }
        }
    }
    agent_rows.resize(static_cast<std::size_t>(num_agents));
    agent_cols.resize(static_cast<std::size_t>(num_agents));

    std::vector<std::string> goals(num_rows, std::string(static_cast<std::size_t>(num_cols), '\0'));
    std::getline(input, line);
    int goal_row = 0;
    while (!line.empty() && line[0] != '#') {
        for (int col = 0; col < static_cast<int>(line.size()); ++col) {
            const char c = line[col];
            if (('0' <= c && c <= '9') || ('A' <= c && c <= 'Z')) {
                goals[goal_row][col] = c;
            }
        }
        ++goal_row;
        std::getline(input, line);
    }

    return std::make_shared<State>(std::move(agent_rows),
                                   std::move(agent_cols),
                                   std::move(agent_colors),
                                   std::move(walls),
                                   std::move(boxes),
                                   std::move(box_colors),
                                   std::move(goals));
}

std::unique_ptr<Frontier> make_frontier(const std::vector<std::string>& args, const State& initial_state)
{
    if (!args.empty()) {
        const std::string strategy = to_lower(args[0]);
        if (strategy == "-prioritized" || strategy == "-pp" ||
            strategy == "-prioritized-fallback" || strategy == "-pp-fallback") {
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicSmartWeightedAStar>(initial_state, 3));
        }
        if (strategy == "-bfs") return std::make_unique<FrontierBFS>();
        if (strategy == "-dfs") return std::make_unique<FrontierDFS>();
        if (strategy == "-astar") {
            return std::make_unique<FrontierBestFirst>(std::make_unique<HeuristicAStar>(initial_state));
        }
        if (strategy == "-wastar") {
            int weight = 5;
            if (args.size() > 1) {
                try {
                    weight = std::stoi(args[1]);
                } catch (const std::exception&) {
                    std::cerr << "Couldn't parse weight argument to -wastar as integer, using default.\n";
                }
            }
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicWeightedAStar>(initial_state, weight));
        }
        if (strategy == "-smart" || strategy == "-smart-greedy") {
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicSmartGreedy>(initial_state));
        }
        if (strategy == "-smart-wastar") {
            int weight = 3;
            if (args.size() > 1) {
                try {
                    weight = std::stoi(args[1]);
                } catch (const std::exception&) {
                    std::cerr << "Couldn't parse weight argument to -smart-wastar as integer, using default.\n";
                }
            }
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicSmartWeightedAStar>(initial_state, weight));
        }
        if (strategy == "-greedy") {
            return std::make_unique<FrontierBestFirst>(std::make_unique<HeuristicGreedy>(initial_state));
        }
        if (strategy == "-greedy-goalcount") {
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicGoalCountGreedy>(initial_state));
        }
        if (strategy == "-astar-goalcount") {
            return std::make_unique<FrontierBestFirst>(
                std::make_unique<HeuristicGoalCountAStar>(initial_state));
        }
    }

    std::cerr << "Defaulting to smart weighted A*. Use arguments -prioritized, -bfs, -dfs, -astar, -wastar, -greedy, -smart, or -smart-wastar to set the search strategy.\n";
    return std::make_unique<FrontierBestFirst>(
        std::make_unique<HeuristicSmartWeightedAStar>(initial_state, 3));
}

int count_boxes(const State& state)
{
    int count = 0;
    for (const auto& row : state.boxes) {
        for (char cell : row) {
            if ('A' <= cell && cell <= 'Z') {
                ++count;
            }
        }
    }
    return count;
}

} // namespace

int main(int argc, char* argv[])
{
    std::cerr << "SearchClient initializing. I am sending this using the error output stream.\n";
    std::cout << "SearchClient\n";
    std::cout << "#This is a comment.\n";

    StatePtr initial_state;
    {
        ScopedProfile profile("parse_level");
        initial_state = parse_level(std::cin);
    }
    if (!prioritized::basic_goal_feasibility(*initial_state)) {
        std::cerr << "Unable to solve level: basic goal feasibility check failed.\n";
        return 0;
    }

    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    const std::string strategy = args.empty() ? "" : to_lower(args[0]);
    const bool use_prioritized =
        args.empty() ||
        strategy == "-prioritized" ||
        strategy == "-pp" ||
        strategy == "-prioritized-fallback" ||
        strategy == "-pp-fallback";
    const bool allow_prioritized_fallback =
        args.empty() ||
        strategy == "-prioritized-fallback" ||
        strategy == "-pp-fallback";

    std::unique_ptr<Frontier> frontier = make_frontier(args, *initial_state);
    std::cerr << "Starting " << frontier->name() << ".\n";

    std::optional<std::vector<std::vector<const Action*>>> plan;
    if (use_prioritized) {
        std::cerr << "Trying prioritized planning with reservation tables.\n";
        {
            ScopedProfile profile("main_prioritized_stage");
            plan = prioritized::solve(initial_state);
        }
        if (!plan.has_value()) {
            std::cerr << "Prioritized planning failed; trying serial task fallback.\n";
            ScopedProfile profile("main_serial_stage");
            plan = prioritized::solve_serial(initial_state);
        }
        if (!plan.has_value() && (strategy == "-prioritized" || strategy == "-pp" || args.empty())) {
            const int total_boxes = count_boxes(*initial_state);
            const int total_agents = static_cast<int>(initial_state->agent_rows.size());
            const bool small_joint_repair = total_agents <= 5 && total_boxes <= 40;
            const double repair_seconds = small_joint_repair ? 25.0 : 10.0;
            const std::size_t repair_expansions = small_joint_repair ? 250000U : 50000U;
            std::cerr << "Prioritized planning failed; trying bounded weighted A*(5) fallback"
                      << (small_joint_repair ? " with small-level repair budget" : "")
                      << ".\n";
            auto fallback_frontier = FrontierBestFirst(
                std::make_unique<HeuristicWeightedAStar>(*initial_state, 5));
            ScopedProfile profile("main_bounded_wastar_stage");
            plan = GraphSearch::search(initial_state,
                                       fallback_frontier,
                                       repair_seconds,
                                       repair_expansions);
        }
        if (!plan.has_value() && allow_prioritized_fallback) {
            std::cerr << "Prioritized planning failed; falling back to " << frontier->name() << ".\n";
        } else if (!plan.has_value()) {
            std::cerr << "Prioritized planning failed; not falling back in prioritized-only mode.\n";
            return 0;
        }
    }
    try {
        if (plan.has_value()) {
            // Prioritized planner already produced a plan.
        } else {
            ScopedProfile profile("main_graph_search_stage");
            plan = GraphSearch::search(initial_state, *frontier);
        }
    } catch (const std::bad_alloc&) {
        std::cerr << "Maximum memory usage exceeded.\n";
    }

    if (!plan.has_value()) {
        std::cerr << "Unable to solve level.\n";
        return 0;
    }

    std::cerr << "Found solution of length " << plan->size() << ".\n";
    for (const auto& joint_action : *plan) {
        for (std::size_t i = 0; i < joint_action.size(); ++i) {
            if (i > 0) {
                std::cout << '|';
            }
            std::cout << joint_action[i]->name;
        }
        std::cout << '\n';

        std::string response;
        if (!std::getline(std::cin, response)) {
            break;
        }
    }

    return 0;
}
