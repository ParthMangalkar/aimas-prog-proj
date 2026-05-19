#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
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

namespace enhanced {

constexpr int kInf = std::numeric_limits<int>::max() / 4;
constexpr int kNoOpIndex = 0;

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
    std::stringstream stream(input);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        parts.push_back(trim(item));
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

bool env_flag_enabled(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return false;
    }
    const std::string normalized = to_lower(trim(value));
    return normalized != "0" && normalized != "false" &&
           normalized != "no" && normalized != "off";
}

double enhanced_budget_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_BUDGET_S");
    if (value == nullptr) {
        return 25.0;
    }
    return std::max(1.0, std::atof(value));
}

double enhanced_local_repair_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_LOCAL_REPAIR_S");
    if (value == nullptr) {
        return 2.0;
    }
    return std::max(0.1, std::atof(value));
}

int enhanced_relocation_depth()
{
    const char* value = std::getenv("AIMAS_ENHANCED_RELOCATION_DEPTH");
    if (value == nullptr) {
        return 0;
    }
    return std::max(0, std::min(3, std::atoi(value)));
}

double enhanced_relocation_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_RELOCATION_S");
    if (value == nullptr) {
        return 2.0;
    }
    return std::max(0.1, std::atof(value));
}

int enhanced_single_box_expansions()
{
    const char* value = std::getenv("AIMAS_ENHANCED_SINGLE_BOX_EXPANSIONS");
    if (value == nullptr) {
        return 180000;
    }
    return std::max(10000, std::atoi(value));
}

double enhanced_neighborhood_repair_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_NEIGHBORHOOD_REPAIR_S");
    if (value == nullptr) {
        return 0.15;
    }
    return std::max(0.05, std::atof(value));
}

bool enhanced_neighborhood_repair_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_NEIGHBORHOOD_REPAIR");
}

bool enhanced_density_ordering_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_DENSITY_ORDERING");
}

bool enhanced_component_ordering_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_COMPONENT_ORDERING");
}

bool enhanced_two_agent_repair_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_TWO_AGENT_REPAIR");
}

bool enhanced_cbs_box_repair_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_CBS_BOX_REPAIR");
}

bool enhanced_pibt_disabled()
{
    // PIBT-based final-agent coordination is default-on. Set
    // AIMAS_ENHANCED_PIBT=0 to disable as an escape hatch.
    const char* value = std::getenv("AIMAS_ENHANCED_PIBT");
    if (value == nullptr) {
        return false;
    }
    const std::string normalized = to_lower(trim(value));
    return normalized == "0" || normalized == "false" ||
           normalized == "no" || normalized == "off";
}

int enhanced_pibt_max_timesteps()
{
    const char* value = std::getenv("AIMAS_ENHANCED_PIBT_T");
    if (value == nullptr) {
        return 400;
    }
    return std::max(8, std::min(5000, std::atoi(value)));
}

double enhanced_two_agent_repair_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_TWO_AGENT_REPAIR_S");
    if (value == nullptr) {
        return 0.5;
    }
    return std::max(0.05, std::atof(value));
}

double enhanced_cbs_box_repair_seconds()
{
    const char* value = std::getenv("AIMAS_ENHANCED_CBS_BOX_REPAIR_S");
    if (value == nullptr) {
        return 0.75;
    }
    return std::max(0.05, std::atof(value));
}

int enhanced_cbs_box_repair_max_agents()
{
    const char* value = std::getenv("AIMAS_ENHANCED_CBS_BOX_REPAIR_AGENTS");
    if (value == nullptr) {
        return 2;
    }
    return std::max(2, std::min(4, std::atoi(value)));
}

bool enhanced_relocation_graph_enabled()
{
    return env_flag_enabled("AIMAS_ENHANCED_RELOCATION_GRAPH") ||
           enhanced_relocation_depth() > 0;
}

bool is_box(char ch)
{
    return 'A' <= ch && ch <= 'Z';
}

bool is_agent(char ch)
{
    return '0' <= ch && ch <= '9';
}

enum class ActionType { NoOp, Move, Push, Pull };

struct Action {
    std::string name;
    ActionType type = ActionType::NoOp;
    int agent_dr = 0;
    int agent_dc = 0;
    int box_dr = 0;
    int box_dc = 0;
};

const std::vector<Action>& actions()
{
    static const std::vector<Action> all = {
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
    };
    return all;
}

struct Level {
    std::string name;
    int rows = 0;
    int cols = 0;
    std::vector<std::vector<bool>> walls;
    std::vector<std::string> boxes;
    std::vector<std::string> goals;
    std::vector<int> agent_rows;
    std::vector<int> agent_cols;
    std::vector<int> agent_color;
    std::array<int, 26> box_color{};
};

struct State {
    const Level* level = nullptr;
    std::vector<int> agent_rows;
    std::vector<int> agent_cols;
    std::vector<std::string> boxes;

    char agent_at(int row, int col) const
    {
        for (std::size_t i = 0; i < agent_rows.size(); ++i) {
            if (agent_rows[i] == row && agent_cols[i] == col) {
                return static_cast<char>('0' + static_cast<int>(i));
            }
        }
        return '\0';
    }

    bool in_bounds(int row, int col) const
    {
        return 0 <= row && row < level->rows && 0 <= col && col < level->cols;
    }

    bool cell_free(int row, int col) const
    {
        return in_bounds(row, col) &&
               !level->walls[row][col] &&
               boxes[row][col] == '\0' &&
               agent_at(row, col) == '\0';
    }

    bool can_move_box(int agent, char box) const
    {
        if (!is_box(box) || agent < 0 || agent >= static_cast<int>(level->agent_color.size())) {
            return false;
        }
        return level->agent_color[agent] == level->box_color[box - 'A'];
    }

    bool applicable(int agent, const Action& action) const
    {
        const int ar = agent_rows[agent];
        const int ac = agent_cols[agent];
        switch (action.type) {
            case ActionType::NoOp:
                return true;
            case ActionType::Move:
                return cell_free(ar + action.agent_dr, ac + action.agent_dc);
            case ActionType::Push: {
                const int br = ar + action.agent_dr;
                const int bc = ac + action.agent_dc;
                const int nbr = br + action.box_dr;
                const int nbc = bc + action.box_dc;
                return in_bounds(br, bc) &&
                       in_bounds(nbr, nbc) &&
                       is_box(boxes[br][bc]) &&
                       can_move_box(agent, boxes[br][bc]) &&
                       cell_free(nbr, nbc);
            }
            case ActionType::Pull: {
                const int br = ar - action.box_dr;
                const int bc = ac - action.box_dc;
                const int nar = ar + action.agent_dr;
                const int nac = ac + action.agent_dc;
                return in_bounds(br, bc) &&
                       in_bounds(nar, nac) &&
                       is_box(boxes[br][bc]) &&
                       can_move_box(agent, boxes[br][bc]) &&
                       cell_free(nar, nac);
            }
        }
        return false;
    }

    struct Delta {
        int agent_from_r = -1;
        int agent_from_c = -1;
        int agent_to_r = -1;
        int agent_to_c = -1;
        int box_from_r = -1;
        int box_from_c = -1;
        int box_to_r = -1;
        int box_to_c = -1;
        bool moves_box = false;
    };

    Delta delta_for(int agent, const Action& action) const
    {
        Delta delta;
        delta.agent_from_r = agent_rows[agent];
        delta.agent_from_c = agent_cols[agent];
        delta.agent_to_r = delta.agent_from_r;
        delta.agent_to_c = delta.agent_from_c;
        switch (action.type) {
            case ActionType::NoOp:
                break;
            case ActionType::Move:
                delta.agent_to_r += action.agent_dr;
                delta.agent_to_c += action.agent_dc;
                break;
            case ActionType::Push:
                delta.box_from_r = delta.agent_from_r + action.agent_dr;
                delta.box_from_c = delta.agent_from_c + action.agent_dc;
                delta.box_to_r = delta.box_from_r + action.box_dr;
                delta.box_to_c = delta.box_from_c + action.box_dc;
                delta.agent_to_r = delta.box_from_r;
                delta.agent_to_c = delta.box_from_c;
                delta.moves_box = true;
                break;
            case ActionType::Pull:
                delta.box_from_r = delta.agent_from_r - action.box_dr;
                delta.box_from_c = delta.agent_from_c - action.box_dc;
                delta.box_to_r = delta.agent_from_r;
                delta.box_to_c = delta.agent_from_c;
                delta.agent_to_r += action.agent_dr;
                delta.agent_to_c += action.agent_dc;
                delta.moves_box = true;
                break;
        }
        return delta;
    }

    bool conflicting(const std::vector<int>& joint_action) const
    {
        std::set<std::pair<int, int>> agent_destinations;
        std::set<std::pair<int, int>> box_destinations;
        std::set<std::pair<int, int>> occupied_destinations;
        std::vector<Delta> deltas;
        for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
            const Delta delta = delta_for(agent, actions()[joint_action[agent]]);
            deltas.push_back(delta);
            const auto agent_to = std::make_pair(delta.agent_to_r, delta.agent_to_c);
            if (!agent_destinations.insert(agent_to).second ||
                !occupied_destinations.insert(agent_to).second) {
                return true;
            }
            if (delta.moves_box) {
                const auto box_to = std::make_pair(delta.box_to_r, delta.box_to_c);
                if (!box_destinations.insert(box_to).second ||
                    !occupied_destinations.insert(box_to).second) {
                    return true;
                }
            }
        }

        for (std::size_t i = 0; i < deltas.size(); ++i) {
            for (std::size_t j = i + 1; j < deltas.size(); ++j) {
                const Delta& a = deltas[i];
                const Delta& b = deltas[j];
                if (std::make_pair(a.agent_from_r, a.agent_from_c) ==
                        std::make_pair(b.agent_to_r, b.agent_to_c) &&
                    std::make_pair(a.agent_to_r, a.agent_to_c) ==
                        std::make_pair(b.agent_from_r, b.agent_from_c)) {
                    return true;
                }
                if (a.moves_box && b.moves_box &&
                    std::make_pair(a.box_from_r, a.box_from_c) ==
                        std::make_pair(b.box_to_r, b.box_to_c) &&
                    std::make_pair(a.box_to_r, a.box_to_c) ==
                        std::make_pair(b.box_from_r, b.box_from_c)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool apply_joint(const std::vector<int>& joint_action)
    {
        if (joint_action.size() != agent_rows.size()) {
            return false;
        }
        for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
            if (!applicable(agent, actions()[joint_action[agent]])) {
                return false;
            }
        }
        if (conflicting(joint_action)) {
            return false;
        }

        std::vector<Delta> deltas;
        for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
            deltas.push_back(delta_for(agent, actions()[joint_action[agent]]));
        }
        for (const Delta& delta : deltas) {
            if (delta.moves_box) {
                boxes[delta.box_to_r][delta.box_to_c] = boxes[delta.box_from_r][delta.box_from_c];
                boxes[delta.box_from_r][delta.box_from_c] = '\0';
            }
        }
        for (int agent = 0; agent < static_cast<int>(deltas.size()); ++agent) {
            agent_rows[agent] = deltas[agent].agent_to_r;
            agent_cols[agent] = deltas[agent].agent_to_c;
        }
        return true;
    }

    bool goal_state() const
    {
        for (int row = 0; row < level->rows; ++row) {
            for (int col = 0; col < level->cols; ++col) {
                const char goal = level->goals[row][col];
                if (is_box(goal) && boxes[row][col] != goal) {
                    return false;
                }
                if (is_agent(goal)) {
                    const int agent = goal - '0';
                    if (agent >= static_cast<int>(agent_rows.size()) ||
                        agent_rows[agent] != row ||
                        agent_cols[agent] != col) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
};

Level parse_level(std::istream& input)
{
    Level level;
    level.box_color.fill(-1);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        lines.push_back(line);
        if (trim(line) == "#end") {
            break;
        }
    }

    auto find_section = [&](const std::string& section) {
        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (trim(lines[i]) == section) {
                return i;
            }
        }
        return lines.size();
    };

    const std::size_t level_name = find_section("#levelname");
    if (level_name + 1 < lines.size()) {
        level.name = trim(lines[level_name + 1]);
    }

    std::map<std::string, int> color_ids;
    auto color_id = [&](const std::string& name) {
        const std::string key = to_lower(trim(name));
        auto it = color_ids.find(key);
        if (it != color_ids.end()) {
            return it->second;
        }
        const int id = static_cast<int>(color_ids.size());
        color_ids[key] = id;
        return id;
    };

    std::array<int, 10> agent_color_tmp{};
    agent_color_tmp.fill(-1);
    const std::size_t colors = find_section("#colors");
    for (std::size_t i = colors + 1; i < lines.size() && !trim(lines[i]).empty() && trim(lines[i])[0] != '#'; ++i) {
        const std::string stripped = trim(lines[i]);
        const std::size_t colon = stripped.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        const int id = color_id(stripped.substr(0, colon));
        for (const std::string& token : split(stripped.substr(colon + 1), ',')) {
            if (token.empty()) {
                continue;
            }
            const char entity = token[0];
            if (is_agent(entity)) {
                agent_color_tmp[entity - '0'] = id;
            } else if (is_box(entity)) {
                level.box_color[entity - 'A'] = id;
            }
        }
    }

    std::vector<std::string> initial_lines;
    const std::size_t initial = find_section("#initial");
    for (std::size_t i = initial + 1; i < lines.size() && (trim(lines[i]).empty() || trim(lines[i])[0] != '#'); ++i) {
        if (!trim(lines[i]).empty()) {
            initial_lines.push_back(lines[i]);
            level.cols = std::max(level.cols, static_cast<int>(lines[i].size()));
        }
    }
    level.rows = static_cast<int>(initial_lines.size());
    level.walls.assign(level.rows, std::vector<bool>(level.cols, false));
    level.boxes.assign(level.rows, std::string(static_cast<std::size_t>(level.cols), '\0'));

    int max_agent = -1;
    std::array<int, 10> agent_rows_tmp{};
    std::array<int, 10> agent_cols_tmp{};
    agent_rows_tmp.fill(0);
    agent_cols_tmp.fill(0);
    for (int row = 0; row < level.rows; ++row) {
        for (int col = 0; col < static_cast<int>(initial_lines[row].size()); ++col) {
            const char ch = initial_lines[row][col];
            if (ch == '+') {
                level.walls[row][col] = true;
            } else if (is_box(ch)) {
                level.boxes[row][col] = ch;
            } else if (is_agent(ch)) {
                const int agent = ch - '0';
                max_agent = std::max(max_agent, agent);
                agent_rows_tmp[agent] = row;
                agent_cols_tmp[agent] = col;
            }
        }
    }

    level.agent_rows.resize(static_cast<std::size_t>(max_agent + 1));
    level.agent_cols.resize(static_cast<std::size_t>(max_agent + 1));
    level.agent_color.resize(static_cast<std::size_t>(max_agent + 1), -1);
    for (int agent = 0; agent <= max_agent; ++agent) {
        level.agent_rows[agent] = agent_rows_tmp[agent];
        level.agent_cols[agent] = agent_cols_tmp[agent];
        level.agent_color[agent] = agent_color_tmp[agent];
    }

    level.goals.assign(level.rows, std::string(static_cast<std::size_t>(level.cols), '\0'));
    const std::size_t goals = find_section("#goal");
    int goal_row = 0;
    for (std::size_t i = goals + 1; i < lines.size() && goal_row < level.rows &&
                                     (trim(lines[i]).empty() || trim(lines[i])[0] != '#'); ++i) {
        if (trim(lines[i]).empty()) {
            continue;
        }
        for (int col = 0; col < static_cast<int>(lines[i].size()) && col < level.cols; ++col) {
            const char ch = lines[i][col];
            if (is_box(ch) || is_agent(ch)) {
                level.goals[goal_row][col] = ch;
            }
        }
        ++goal_row;
    }

    for (char box = 'A'; box <= 'Z'; ++box) {
        const int idx = box - 'A';
        if (level.box_color[idx] < 0) {
            for (int agent_color : level.agent_color) {
                if (agent_color >= 0) {
                    level.box_color[idx] = agent_color;
                    break;
                }
            }
        }
    }
    return level;
}

struct Topology {
    const Level& level;
    std::map<std::pair<int, int>, std::vector<std::vector<int>>> bfs_cache;

    explicit Topology(const Level& input) : level(input) {}

    std::vector<std::vector<int>> bfs_from(int start_row, int start_col) const
    {
        std::vector<std::vector<int>> distances(level.rows, std::vector<int>(level.cols, kInf));
        if (start_row < 0 || start_row >= level.rows || start_col < 0 || start_col >= level.cols ||
            level.walls[start_row][start_col]) {
            return distances;
        }
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::deque<std::pair<int, int>> queue;
        distances[start_row][start_col] = 0;
        queue.push_back({start_row, start_col});
        while (!queue.empty()) {
            const auto [row, col] = queue.front();
            queue.pop_front();
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (0 <= nr && nr < level.rows && 0 <= nc && nc < level.cols &&
                    !level.walls[nr][nc] && distances[nr][nc] == kInf) {
                    distances[nr][nc] = distances[row][col] + 1;
                    queue.push_back({nr, nc});
                }
            }
        }
        return distances;
    }

    const std::vector<std::vector<int>>& distances_from(int row, int col)
    {
        const auto key = std::make_pair(row, col);
        auto it = bfs_cache.find(key);
        if (it == bfs_cache.end()) {
            it = bfs_cache.emplace(key, bfs_from(row, col)).first;
        }
        return it->second;
    }

    bool pull_aware_dead_cell(char box, int row, int col)
    {
        if (!is_box(box) || row < 0 || row >= level.rows || col < 0 || col >= level.cols ||
            level.walls[row][col]) {
            return true;
        }
        if (level.goals[row][col] == box) {
            return false;
        }
        for (int gr = 0; gr < level.rows; ++gr) {
            for (int gc = 0; gc < level.cols; ++gc) {
                if (level.goals[gr][gc] != box) {
                    continue;
                }
                if (distances_from(gr, gc)[row][col] != kInf) {
                    return false;
                }
            }
        }
        return true;
    }
};

struct BoxTask {
    char box = '\0';
    int start_row = -1;
    int start_col = -1;
    int goal_row = -1;
    int goal_col = -1;
};

struct TaskAllocator {
    const Level& level;
    Topology& topology;

    std::vector<BoxTask> build_tasks(const State& state)
    {
        std::vector<std::vector<BoxTask>> variants = build_task_variants(state);
        return variants.empty() ? std::vector<BoxTask>{} : variants.front();
    }

    std::vector<std::vector<BoxTask>> build_task_variants(const State& state)
    {
        std::vector<std::vector<BoxTask>> variants;
        std::set<std::string> seen;
        int final_order_modes = enhanced_density_ordering_enabled() ? 6 : 5;
        if (enhanced_component_ordering_enabled()) {
            final_order_modes = std::max(final_order_modes, 7);
        }
        auto add_variant = [&](std::vector<BoxTask> tasks) {
            const std::string key = signature(tasks);
            if (!tasks.empty() && seen.insert(key).second) {
                variants.push_back(std::move(tasks));
            }
        };

        for (int final_order_mode = 0; final_order_mode < final_order_modes; ++final_order_mode) {
            add_variant(build_matched_tasks(state, final_order_mode));
            if (variants.size() >= 16) {
                return variants;
            }
        }

        for (int goal_order_mode = 0; goal_order_mode < 4; ++goal_order_mode) {
            for (int final_order_mode = 0; final_order_mode < final_order_modes; ++final_order_mode) {
                add_variant(build_tasks_for_modes(state, goal_order_mode, final_order_mode));
                if (variants.size() >= 16) {
                    return variants;
                }
            }
        }
        return variants;
    }

    std::vector<BoxTask> build_tasks_for_modes(const State& state,
                                               int goal_order_mode,
                                               int final_order_mode)
    {
        std::vector<BoxTask> tasks;
        for (char letter = 'A'; letter <= 'Z'; ++letter) {
            const int color = level.box_color[letter - 'A'];
            bool has_agent = false;
            for (int agent_color : level.agent_color) {
                has_agent = has_agent || agent_color == color;
            }
            if (!has_agent) {
                continue;
            }

            std::vector<std::pair<int, int>> goals;
            std::vector<std::pair<int, int>> boxes;
            for (int row = 0; row < level.rows; ++row) {
                for (int col = 0; col < level.cols; ++col) {
                    if (level.goals[row][col] == letter && state.boxes[row][col] != letter) {
                        goals.push_back({row, col});
                    }
                    if (state.boxes[row][col] == letter && level.goals[row][col] != letter) {
                        boxes.push_back({row, col});
                    }
                }
            }
            std::set<int> used_boxes;
            std::stable_sort(goals.begin(), goals.end(), [&](const auto& lhs, const auto& rhs) {
                const int lhs_nearest = nearest_box_distance(lhs, boxes);
                const int rhs_nearest = nearest_box_distance(rhs, boxes);
                switch (goal_order_mode) {
                    case 1:
                        if (lhs.first != rhs.first) {
                            return lhs.first > rhs.first;
                        }
                        return lhs.second > rhs.second;
                    case 2:
                        if (lhs_nearest != rhs_nearest) {
                            return lhs_nearest > rhs_nearest;
                        }
                        break;
                    case 3:
                        if (lhs_nearest != rhs_nearest) {
                            return lhs_nearest < rhs_nearest;
                        }
                        break;
                    default:
                        break;
                }
                if (lhs.first != rhs.first) {
                    return lhs.first < rhs.first;
                }
                return lhs.second < rhs.second;
            });
            for (const auto& goal : goals) {
                int best_box = -1;
                int best_distance = kInf;
                const auto& distances = topology.distances_from(goal.first, goal.second);
                for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
                    if (used_boxes.count(i) != 0) {
                        continue;
                    }
                    const int distance = distances[boxes[i].first][boxes[i].second];
                    if (distance < best_distance) {
                        best_distance = distance;
                        best_box = i;
                    }
                }
                if (best_box >= 0 && best_distance != kInf) {
                    used_boxes.insert(best_box);
                    tasks.push_back({letter, boxes[best_box].first, boxes[best_box].second,
                                     goal.first, goal.second});
                }
            }
        }
        order_tasks(tasks, final_order_mode);
        return tasks;
    }

    std::vector<BoxTask> build_matched_tasks(const State& state, int final_order_mode)
    {
        std::vector<BoxTask> tasks;
        for (char letter = 'A'; letter <= 'Z'; ++letter) {
            const int color = level.box_color[letter - 'A'];
            bool has_agent = false;
            for (int agent_color : level.agent_color) {
                has_agent = has_agent || agent_color == color;
            }
            if (!has_agent) {
                continue;
            }

            std::vector<std::pair<int, int>> goals;
            std::vector<std::pair<int, int>> boxes;
            for (int row = 0; row < level.rows; ++row) {
                for (int col = 0; col < level.cols; ++col) {
                    if (level.goals[row][col] == letter && state.boxes[row][col] != letter) {
                        goals.push_back({row, col});
                    }
                    if (state.boxes[row][col] == letter && level.goals[row][col] != letter) {
                        boxes.push_back({row, col});
                    }
                }
            }
            if (goals.empty() || boxes.empty()) {
                continue;
            }
            if (goals.size() > boxes.size() || boxes.size() > 14) {
                return build_tasks_for_modes(state, 0, final_order_mode);
            }

            const int box_count = static_cast<int>(boxes.size());
            const int mask_count = 1 << box_count;
            std::vector<int> dp(static_cast<std::size_t>(mask_count), kInf);
            std::vector<std::vector<int>> parent_mask(
                goals.size(), std::vector<int>(static_cast<std::size_t>(mask_count), -1));
            std::vector<std::vector<int>> parent_box(
                goals.size(), std::vector<int>(static_cast<std::size_t>(mask_count), -1));
            dp[0] = 0;
            for (std::size_t goal_index = 0; goal_index < goals.size(); ++goal_index) {
                std::vector<int> next(static_cast<std::size_t>(mask_count), kInf);
                const auto& distances = topology.distances_from(goals[goal_index].first,
                                                                goals[goal_index].second);
                for (int mask = 0; mask < mask_count; ++mask) {
                    if (dp[static_cast<std::size_t>(mask)] == kInf ||
                        popcount(mask) != static_cast<int>(goal_index)) {
                        continue;
                    }
                    for (int box_index = 0; box_index < box_count; ++box_index) {
                        if ((mask & (1 << box_index)) != 0) {
                            continue;
                        }
                        const int distance = distances[boxes[box_index].first][boxes[box_index].second];
                        if (distance == kInf) {
                            continue;
                        }
                        const int next_mask = mask | (1 << box_index);
                        const int cost = dp[static_cast<std::size_t>(mask)] + distance;
                        if (cost < next[static_cast<std::size_t>(next_mask)]) {
                            next[static_cast<std::size_t>(next_mask)] = cost;
                            parent_mask[goal_index][static_cast<std::size_t>(next_mask)] = mask;
                            parent_box[goal_index][static_cast<std::size_t>(next_mask)] = box_index;
                        }
                    }
                }
                dp.swap(next);
            }

            int best_mask = -1;
            int best_cost = kInf;
            for (int mask = 0; mask < mask_count; ++mask) {
                if (popcount(mask) == static_cast<int>(goals.size()) &&
                    dp[static_cast<std::size_t>(mask)] < best_cost) {
                    best_cost = dp[static_cast<std::size_t>(mask)];
                    best_mask = mask;
                }
            }
            if (best_mask < 0) {
                continue;
            }

            std::vector<int> assignment(goals.size(), -1);
            for (int goal_index = static_cast<int>(goals.size()) - 1;
                 goal_index >= 0 && best_mask >= 0;
                 --goal_index) {
                const int box_index =
                    parent_box[static_cast<std::size_t>(goal_index)][static_cast<std::size_t>(best_mask)];
                assignment[static_cast<std::size_t>(goal_index)] = box_index;
                best_mask =
                    parent_mask[static_cast<std::size_t>(goal_index)][static_cast<std::size_t>(best_mask)];
            }
            for (std::size_t goal_index = 0; goal_index < goals.size(); ++goal_index) {
                const int box_index = assignment[goal_index];
                if (box_index < 0) {
                    continue;
                }
                tasks.push_back({letter,
                                 boxes[static_cast<std::size_t>(box_index)].first,
                                 boxes[static_cast<std::size_t>(box_index)].second,
                                 goals[goal_index].first,
                                 goals[goal_index].second});
            }
        }
        order_tasks(tasks, final_order_mode);
        return tasks;
    }

    int popcount(int mask) const
    {
        int count = 0;
        while (mask != 0) {
            count += mask & 1;
            mask >>= 1;
        }
        return count;
    }

    int task_distance(const BoxTask& task)
    {
        return topology.distances_from(task.goal_row, task.goal_col)[task.start_row][task.start_col];
    }

    int task_density(const BoxTask& task)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        const auto& distances = topology.distances_from(task.goal_row, task.goal_col);
        if (distances[task.start_row][task.start_col] == kInf) {
            return kInf;
        }
        int row = task.start_row;
        int col = task.start_col;
        int score = 0;
        for (int steps = 0; steps < level.rows * level.cols &&
                            (row != task.goal_row || col != task.goal_col);
             ++steps) {
            int open_neighbors = 0;
            int best_dir = -1;
            int best_distance = distances[row][col];
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (0 <= nr && nr < level.rows && 0 <= nc && nc < level.cols &&
                    !level.walls[nr][nc]) {
                    ++open_neighbors;
                    if (distances[nr][nc] < best_distance) {
                        best_distance = distances[nr][nc];
                        best_dir = dir;
                    }
                }
            }
            score += 4 - open_neighbors;
            if (level.goals[row][col] != '\0' && level.goals[row][col] != task.box) {
                score += 2;
            }
            if (best_dir < 0) {
                break;
            }
            row += dr[best_dir];
            col += dc[best_dir];
        }
        return score;
    }

    int component_id_for(int row, int col)
    {
        if (row < 0 || row >= level.rows || col < 0 || col >= level.cols ||
            level.walls[row][col]) {
            return kInf;
        }
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::vector<std::vector<bool>> seen(static_cast<std::size_t>(level.rows),
                                            std::vector<bool>(static_cast<std::size_t>(level.cols), false));
        std::deque<std::pair<int, int>> queue;
        queue.push_back({row, col});
        seen[row][col] = true;
        int best = row * level.cols + col;
        while (!queue.empty()) {
            const auto [current_row, current_col] = queue.front();
            queue.pop_front();
            best = std::min(best, current_row * level.cols + current_col);
            for (int dir = 0; dir < 4; ++dir) {
                const int next_row = current_row + dr[dir];
                const int next_col = current_col + dc[dir];
                if (0 <= next_row && next_row < level.rows &&
                    0 <= next_col && next_col < level.cols &&
                    !level.walls[next_row][next_col] &&
                    !seen[next_row][next_col]) {
                    seen[next_row][next_col] = true;
                    queue.push_back({next_row, next_col});
                }
            }
        }
        return best;
    }

    int nearest_box_distance(const std::pair<int, int>& goal,
                             const std::vector<std::pair<int, int>>& boxes)
    {
        const auto& distances = topology.distances_from(goal.first, goal.second);
        int best = kInf;
        for (const auto& box : boxes) {
            best = std::min(best, distances[box.first][box.second]);
        }
        return best;
    }

    void order_tasks(std::vector<BoxTask>& tasks, int final_order_mode)
    {
        std::stable_sort(tasks.begin(), tasks.end(), [&](const BoxTask& lhs, const BoxTask& rhs) {
            const int lhs_distance = topology.distances_from(lhs.goal_row, lhs.goal_col)[lhs.start_row][lhs.start_col];
            const int rhs_distance = topology.distances_from(rhs.goal_row, rhs.goal_col)[rhs.start_row][rhs.start_col];
            switch (final_order_mode) {
                case 1:
                    if (lhs_distance != rhs_distance) {
                        return lhs_distance < rhs_distance;
                    }
                    break;
                case 2:
                    if (lhs.goal_row != rhs.goal_row) {
                        return lhs.goal_row < rhs.goal_row;
                    }
                    if (lhs.goal_col != rhs.goal_col) {
                        return lhs.goal_col < rhs.goal_col;
                    }
                    break;
                case 3:
                    if (lhs.goal_row != rhs.goal_row) {
                        return lhs.goal_row > rhs.goal_row;
                    }
                    if (lhs.goal_col != rhs.goal_col) {
                        return lhs.goal_col > rhs.goal_col;
                    }
                    break;
                case 4:
                    if (lhs.box != rhs.box) {
                        return lhs.box < rhs.box;
                    }
                    break;
                case 5: {
                    const int lhs_density = task_density(lhs);
                    const int rhs_density = task_density(rhs);
                    if (lhs_density != rhs_density) {
                        return lhs_density < rhs_density;
                    }
                    if (lhs_distance != rhs_distance) {
                        return lhs_distance < rhs_distance;
                    }
                    break;
                }
                case 6: {
                    const int lhs_component = component_id_for(lhs.goal_row, lhs.goal_col);
                    const int rhs_component = component_id_for(rhs.goal_row, rhs.goal_col);
                    if (lhs_component != rhs_component) {
                        return lhs_component < rhs_component;
                    }
                    const int lhs_density = task_density(lhs);
                    const int rhs_density = task_density(rhs);
                    if (lhs_density != rhs_density) {
                        return lhs_density > rhs_density;
                    }
                    if (lhs_distance != rhs_distance) {
                        return lhs_distance < rhs_distance;
                    }
                    break;
                }
                default:
                    if (lhs_distance != rhs_distance) {
                        return lhs_distance > rhs_distance;
                    }
                    break;
            }
            if (lhs.goal_row != rhs.goal_row) {
                return lhs.goal_row < rhs.goal_row;
            }
            return lhs.goal_col < rhs.goal_col;
        });
    }

    std::string signature(const std::vector<BoxTask>& tasks)
    {
        std::ostringstream out;
        for (const BoxTask& task : tasks) {
            out << task.box << ':' << task.start_row << ',' << task.start_col
                << "->" << task.goal_row << ',' << task.goal_col << ';';
        }
        return out.str();
    }
};

struct PlannerNode {
    int ar = -1;
    int ac = -1;
    int br = -1;
    int bc = -1;
    int g = 0;
    int f = 0;
    int parent = -1;
    int action = 0;
};

class SingleBoxPlanner {
public:
    SingleBoxPlanner(const State& state, Topology& topology)
        : state_(state), topology_(topology), level_(*state.level)
    {
    }

    std::vector<int> plan_box(int agent,
                              char box,
                              int box_row,
                              int box_col,
                              int goal_row,
                              int goal_col,
                              int expansion_cap = 400000,
                              std::chrono::steady_clock::time_point deadline =
                                  std::chrono::steady_clock::time_point::max())
    {
        const auto& dist_to_goal = topology_.distances_from(goal_row, goal_col);
        if (dist_to_goal[box_row][box_col] == kInf) {
            return {};
        }

        auto blocked = [&](int row, int col, int active_box_row, int active_box_col) {
            if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols || level_.walls[row][col]) {
                return true;
            }
            if (row == active_box_row && col == active_box_col) {
                return true;
            }
            if (state_.boxes[row][col] != '\0' && !(row == box_row && col == box_col)) {
                return true;
            }
            for (int other = 0; other < static_cast<int>(state_.agent_rows.size()); ++other) {
                if (other != agent && state_.agent_rows[other] == row && state_.agent_cols[other] == col) {
                    return true;
                }
            }
            return false;
        };

        auto heuristic = [&](int ar, int ac, int br, int bc) {
            const int box_distance = dist_to_goal[br][bc];
            if (box_distance == kInf || topology_.pull_aware_dead_cell(box, br, bc)) {
                return kInf;
            }
            return box_distance + std::max(0, std::abs(ar - br) + std::abs(ac - bc) - 1);
        };

        std::vector<PlannerNode> nodes;
        nodes.reserve(4096);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<long long, int> best_g;

        const int root_h = heuristic(state_.agent_rows[agent], state_.agent_cols[agent], box_row, box_col);
        if (root_h == kInf) {
            return {};
        }
        nodes.push_back({state_.agent_rows[agent], state_.agent_cols[agent],
                         box_row, box_col, 0, root_h, -1, 0});
        open.push(0);
        best_g[encode(state_.agent_rows[agent], state_.agent_cols[agent], box_row, box_col)] = 0;

        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 2047) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                return {};
            }
            const int index = open.top();
            open.pop();
            const PlannerNode current = nodes[index];
            const auto best_it = best_g.find(encode(current.ar, current.ac, current.br, current.bc));
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (current.br == goal_row && current.bc == goal_col) {
                std::vector<int> plan;
                int cursor = index;
                while (nodes[cursor].parent != -1) {
                    plan.push_back(nodes[cursor].action);
                    cursor = nodes[cursor].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;

            for (int action_index = 1; action_index < static_cast<int>(actions().size()); ++action_index) {
                const Action& action = actions()[action_index];
                int nar = current.ar;
                int nac = current.ac;
                int nbr = current.br;
                int nbc = current.bc;
                bool ok = true;
                switch (action.type) {
                    case ActionType::NoOp:
                        ok = false;
                        break;
                    case ActionType::Move:
                        nar += action.agent_dr;
                        nac += action.agent_dc;
                        if (blocked(nar, nac, current.br, current.bc)) {
                            ok = false;
                        }
                        break;
                    case ActionType::Push: {
                        const int touched_row = current.ar + action.agent_dr;
                        const int touched_col = current.ac + action.agent_dc;
                        if (touched_row != current.br || touched_col != current.bc) {
                            ok = false;
                            break;
                        }
                        nbr = current.br + action.box_dr;
                        nbc = current.bc + action.box_dc;
                        nar = touched_row;
                        nac = touched_col;
                        if (blocked(nbr, nbc, current.br, current.bc)) {
                            ok = false;
                        }
                        break;
                    }
                    case ActionType::Pull: {
                        const int old_box_row = current.ar - action.box_dr;
                        const int old_box_col = current.ac - action.box_dc;
                        if (old_box_row != current.br || old_box_col != current.bc) {
                            ok = false;
                            break;
                        }
                        nar += action.agent_dr;
                        nac += action.agent_dc;
                        nbr = current.ar;
                        nbc = current.ac;
                        if (blocked(nar, nac, current.br, current.bc)) {
                            ok = false;
                        }
                        break;
                    }
                }
                if (!ok) {
                    continue;
                }
                const int h = heuristic(nar, nac, nbr, nbc);
                if (h == kInf) {
                    continue;
                }
                const int next_g = current.g + 1;
                const long long key = encode(nar, nac, nbr, nbc);
                const auto it = best_g.find(key);
                if (it != best_g.end() && it->second <= next_g) {
                    continue;
                }
                best_g[key] = next_g;
                nodes.push_back({nar, nac, nbr, nbc, next_g, next_g + h, index, action_index});
                open.push(static_cast<int>(nodes.size()) - 1);
            }
        }
        return {};
    }

    std::vector<int> plan_agent_to(int agent, int target_row, int target_col, int expansion_cap = 100000)
    {
        const auto& distances = topology_.distances_from(target_row, target_col);
        struct Node {
            int row = -1;
            int col = -1;
            int parent = -1;
            int action = 0;
        };
        std::deque<int> queue;
        std::vector<Node> nodes;
        std::unordered_map<int, int> seen;
        nodes.push_back({state_.agent_rows[agent], state_.agent_cols[agent], -1, 0});
        queue.push_back(0);
        seen[state_.agent_rows[agent] * level_.cols + state_.agent_cols[agent]] = 0;
        int expansions = 0;

        auto blocked = [&](int row, int col) {
            if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols ||
                level_.walls[row][col] || state_.boxes[row][col] != '\0') {
                return true;
            }
            for (int other = 0; other < static_cast<int>(state_.agent_rows.size()); ++other) {
                if (other != agent && state_.agent_rows[other] == row && state_.agent_cols[other] == col) {
                    return true;
                }
            }
            return false;
        };

        while (!queue.empty() && expansions < expansion_cap) {
            const int index = queue.front();
            queue.pop_front();
            const Node node = nodes[index];
            if (node.row == target_row && node.col == target_col) {
                std::vector<int> plan;
                int cursor = index;
                while (nodes[cursor].parent != -1) {
                    plan.push_back(nodes[cursor].action);
                    cursor = nodes[cursor].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;
            std::vector<int> move_indices = {1, 2, 3, 4};
            auto distance_at = [&](int row, int col) {
                if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols) {
                    return kInf;
                }
                return distances[row][col];
            };
            std::stable_sort(move_indices.begin(), move_indices.end(), [&](int lhs, int rhs) {
                const int lr = node.row + actions()[lhs].agent_dr;
                const int lc = node.col + actions()[lhs].agent_dc;
                const int rr = node.row + actions()[rhs].agent_dr;
                const int rc = node.col + actions()[rhs].agent_dc;
                return distance_at(lr, lc) < distance_at(rr, rc);
            });
            for (int action_index : move_indices) {
                const int nr = node.row + actions()[action_index].agent_dr;
                const int nc = node.col + actions()[action_index].agent_dc;
                if (blocked(nr, nc)) {
                    continue;
                }
                const int key = nr * level_.cols + nc;
                if (seen.count(key) != 0) {
                    continue;
                }
                seen[key] = static_cast<int>(nodes.size());
                nodes.push_back({nr, nc, index, action_index});
                queue.push_back(static_cast<int>(nodes.size()) - 1);
            }
        }
        return {};
    }

private:
    long long encode(int ar, int ac, int br, int bc) const
    {
        long long agent_cell = static_cast<long long>(ar) * level_.cols + ac;
        long long box_cell = static_cast<long long>(br) * level_.cols + bc;
        return agent_cell * (static_cast<long long>(level_.rows) * level_.cols) + box_cell;
    }

    const State& state_;
    Topology& topology_;
    const Level& level_;
};

class LocalMultiBoxPlanner {
    struct Window {
        std::vector<std::vector<bool>> inside;
        std::vector<std::pair<int, int>> cells;
        std::vector<std::pair<int, int>> corridor;
    };

public:
    LocalMultiBoxPlanner(const State& state, Topology& topology)
        : start_(state), topology_(topology), level_(*state.level)
    {
    }

    std::vector<int> plan(int agent,
                          const BoxTask& task,
                          int expansion_cap,
                          std::chrono::steady_clock::time_point deadline)
    {
        struct Node {
            State state;
            int g = 0;
            int f = 0;
            int parent = -1;
            int action = kNoOpIndex;
        };

        auto heuristic = [&](const State& state) {
            if (state.boxes[task.goal_row][task.goal_col] == task.box) {
                return 0;
            }
            const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
            int best = kInf;
            for (int row = 0; row < level_.rows; ++row) {
                for (int col = 0; col < level_.cols; ++col) {
                    if (state.boxes[row][col] != task.box ||
                        level_.goals[row][col] == task.box ||
                        distances[row][col] == kInf ||
                        topology_.pull_aware_dead_cell(task.box, row, col)) {
                        continue;
                    }
                    const int agent_distance =
                        std::max(0,
                                 std::abs(state.agent_rows[agent] - row) +
                                 std::abs(state.agent_cols[agent] - col) - 1);
                    best = std::min(best, distances[row][col] + agent_distance);
                }
            }
            return best;
        };

        std::vector<Node> nodes;
        nodes.reserve(2048);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<std::string, int> best_g;

        const int root_h = heuristic(start_);
        if (root_h == kInf) {
            return {};
        }
        nodes.push_back({start_, 0, root_h, -1, kNoOpIndex});
        open.push(0);
        best_g[state_key(start_, agent)] = 0;

        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 1023) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                return {};
            }
            const int index = open.top();
            open.pop();
            const Node current = nodes[static_cast<std::size_t>(index)];
            const std::string current_key = state_key(current.state, agent);
            const auto best_it = best_g.find(current_key);
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (current.state.boxes[task.goal_row][task.goal_col] == task.box) {
                std::vector<int> plan;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    plan.push_back(nodes[static_cast<std::size_t>(cursor)].action);
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;

            for (int action_index = 1; action_index < static_cast<int>(actions().size()); ++action_index) {
                const Action& action = actions()[static_cast<std::size_t>(action_index)];
                if (!current.state.applicable(agent, action) ||
                    moves_solved_box(current.state, agent, action)) {
                    continue;
                }
                State child = current.state;
                std::vector<int> joint(child.agent_rows.size(), kNoOpIndex);
                joint[static_cast<std::size_t>(agent)] = action_index;
                if (!child.apply_joint(joint)) {
                    continue;
                }
                const int h = heuristic(child);
                if (h == kInf) {
                    continue;
                }
                const int next_g = current.g + 1;
                const std::string key = state_key(child, agent);
                const auto it = best_g.find(key);
                if (it != best_g.end() && it->second <= next_g) {
                    continue;
                }
                best_g[key] = next_g;
                nodes.push_back({std::move(child), next_g, next_g + 2 * h, index, action_index});
                open.push(static_cast<int>(nodes.size()) - 1);
            }
        }
        return {};
    }

    std::vector<int> plan_neighborhood(int agent,
                                       const BoxTask& task,
                                       int expansion_cap,
                                       std::chrono::steady_clock::time_point deadline)
    {
        const Window window = build_window(agent, task);
        if (window.cells.empty() || window.cells.size() > 520) {
            return {};
        }

        struct Node {
            State state;
            int g = 0;
            int f = 0;
            int parent = -1;
            int action = kNoOpIndex;
        };

        auto heuristic = [&](const State& state) {
            if (state.boxes[task.goal_row][task.goal_col] == task.box) {
                return 0;
            }
            const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
            int best = kInf;
            for (const auto& [row, col] : window.cells) {
                if (state.boxes[row][col] != task.box ||
                    level_.goals[row][col] == task.box ||
                    distances[row][col] == kInf ||
                    topology_.pull_aware_dead_cell(task.box, row, col)) {
                    continue;
                }
                const int agent_distance =
                    std::max(0,
                             std::abs(state.agent_rows[agent] - row) +
                             std::abs(state.agent_cols[agent] - col) - 1);
                best = std::min(best, distances[row][col] + agent_distance);
            }
            if (best == kInf) {
                return kInf;
            }
            int corridor_blockers = 0;
            for (const auto& [row, col] : window.corridor) {
                const char box = state.boxes[row][col];
                if (is_box(box) && box != task.box && level_.goals[row][col] != box) {
                    ++corridor_blockers;
                }
            }
            return best + corridor_blockers;
        };

        const int root_h = heuristic(start_);
        if (root_h == kInf) {
            return {};
        }
        std::vector<Node> nodes;
        nodes.reserve(2048);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<std::string, int> best_g;
        nodes.push_back({start_, 0, root_h, -1, kNoOpIndex});
        open.push(0);
        best_g[state_key_window(start_, agent, window)] = 0;

        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 511) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                return {};
            }
            const int index = open.top();
            open.pop();
            const Node current = nodes[static_cast<std::size_t>(index)];
            const std::string current_key = state_key_window(current.state, agent, window);
            const auto best_it = best_g.find(current_key);
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (current.state.boxes[task.goal_row][task.goal_col] == task.box) {
                std::vector<int> plan;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    plan.push_back(nodes[static_cast<std::size_t>(cursor)].action);
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;

            for (int action_index = 1; action_index < static_cast<int>(actions().size()); ++action_index) {
                const Action& action = actions()[static_cast<std::size_t>(action_index)];
                if (!current.state.applicable(agent, action)) {
                    continue;
                }
                const auto delta = current.state.delta_for(agent, action);
                if (!cell_in_window(window, delta.agent_to_r, delta.agent_to_c)) {
                    continue;
                }
                if (delta.moves_box) {
                    if (!cell_in_window(window, delta.box_from_r, delta.box_from_c) ||
                        !cell_in_window(window, delta.box_to_r, delta.box_to_c) ||
                        moves_solved_box(current.state, agent, action)) {
                        continue;
                    }
                }
                State child = current.state;
                std::vector<int> joint(child.agent_rows.size(), kNoOpIndex);
                joint[static_cast<std::size_t>(agent)] = action_index;
                if (!child.apply_joint(joint)) {
                    continue;
                }
                const int h = heuristic(child);
                if (h == kInf) {
                    continue;
                }
                const int next_g = current.g + 1;
                const std::string key = state_key_window(child, agent, window);
                const auto it = best_g.find(key);
                if (it != best_g.end() && it->second <= next_g) {
                    continue;
                }
                best_g[key] = next_g;
                nodes.push_back({std::move(child), next_g, next_g + 2 * h, index, action_index});
                open.push(static_cast<int>(nodes.size()) - 1);
            }
        }
        return {};
    }

    std::vector<std::vector<int>> plan_two_agent(int active_agent,
                                                 int helper_agent,
                                                 const BoxTask& task,
                                                 int expansion_cap,
                                                 std::chrono::steady_clock::time_point deadline)
    {
        if (active_agent == helper_agent) {
            return {};
        }
        Window window = build_window(active_agent, task);
        auto [source_row, source_col] = source_box_for(task);
        if (window.cells.empty() || source_row < 0 || source_col < 0) {
            return {};
        }
        for (const auto& [row, col] :
             greedy_path(start_.agent_rows[helper_agent], start_.agent_cols[helper_agent], source_row, source_col)) {
            add_radius(window, row, col, 2);
        }
        refresh_window_cells(window);
        if (window.cells.size() > 620) {
            return {};
        }

        struct Node {
            State state;
            int g = 0;
            int f = 0;
            int parent = -1;
            int active_action = kNoOpIndex;
            int helper_action = kNoOpIndex;
        };

        auto heuristic = [&](const State& state) {
            if (state.boxes[task.goal_row][task.goal_col] == task.box) {
                return 0;
            }
            const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
            int best = kInf;
            for (const auto& [row, col] : window.cells) {
                if (state.boxes[row][col] != task.box ||
                    level_.goals[row][col] == task.box ||
                    distances[row][col] == kInf ||
                    topology_.pull_aware_dead_cell(task.box, row, col)) {
                    continue;
                }
                const int active_distance =
                    std::max(0,
                             std::abs(state.agent_rows[active_agent] - row) +
                             std::abs(state.agent_cols[active_agent] - col) - 1);
                const int helper_distance =
                    std::max(0,
                             std::abs(state.agent_rows[helper_agent] - row) +
                             std::abs(state.agent_cols[helper_agent] - col) - 1);
                best = std::min(best, distances[row][col] + std::min(active_distance, helper_distance));
            }
            return best;
        };

        const int root_h = heuristic(start_);
        if (root_h == kInf) {
            return {};
        }
        std::vector<Node> nodes;
        nodes.reserve(2048);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<std::string, int> best_g;
        nodes.push_back({start_, 0, root_h, -1, kNoOpIndex, kNoOpIndex});
        open.push(0);
        best_g[state_key_window_two(start_, active_agent, helper_agent, window)] = 0;

        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 255) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                return {};
            }
            const int index = open.top();
            open.pop();
            const Node current = nodes[static_cast<std::size_t>(index)];
            const std::string current_key =
                state_key_window_two(current.state, active_agent, helper_agent, window);
            const auto best_it = best_g.find(current_key);
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (current.state.boxes[task.goal_row][task.goal_col] == task.box) {
                std::vector<std::vector<int>> plan;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    std::vector<int> joint(start_.agent_rows.size(), kNoOpIndex);
                    joint[active_agent] = nodes[static_cast<std::size_t>(cursor)].active_action;
                    joint[helper_agent] = nodes[static_cast<std::size_t>(cursor)].helper_action;
                    plan.push_back(std::move(joint));
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;

            const std::vector<int> active_actions = local_action_candidates(current.state,
                                                                            active_agent,
                                                                            window);
            const std::vector<int> helper_actions = local_action_candidates(current.state,
                                                                            helper_agent,
                                                                            window);
            for (int active_action : active_actions) {
                for (int helper_action : helper_actions) {
                    if (active_action == kNoOpIndex && helper_action == kNoOpIndex) {
                        continue;
                    }
                    State child = current.state;
                    std::vector<int> joint(child.agent_rows.size(), kNoOpIndex);
                    joint[active_agent] = active_action;
                    joint[helper_agent] = helper_action;
                    if (!child.apply_joint(joint)) {
                        continue;
                    }
                    const int h = heuristic(child);
                    if (h == kInf) {
                        continue;
                    }
                    const int next_g = current.g + 1;
                    const std::string key =
                        state_key_window_two(child, active_agent, helper_agent, window);
                    const auto it = best_g.find(key);
                    if (it != best_g.end() && it->second <= next_g) {
                        continue;
                    }
                    best_g[key] = next_g;
                    nodes.push_back({std::move(child),
                                     next_g,
                                     next_g + 2 * h,
                                     index,
                                     active_action,
                                     helper_action});
                    open.push(static_cast<int>(nodes.size()) - 1);
                }
            }
        }
        return {};
    }

    std::vector<std::vector<int>> plan_cbs_box_repair(
        int active_agent,
        const std::vector<int>& helper_agents,
        const BoxTask& task,
        int expansion_cap,
        std::chrono::steady_clock::time_point deadline)
    {
        std::vector<int> selected_agents;
        selected_agents.push_back(active_agent);
        for (int helper : helper_agents) {
            if (helper == active_agent ||
                std::find(selected_agents.begin(), selected_agents.end(), helper) != selected_agents.end()) {
                continue;
            }
            selected_agents.push_back(helper);
            if (selected_agents.size() >= static_cast<std::size_t>(enhanced_cbs_box_repair_max_agents())) {
                break;
            }
        }
        if (selected_agents.size() < 2) {
            return {};
        }

        Window window = build_window_for_agents(selected_agents, task);
        auto [source_row, source_col] = source_box_for(task);
        if (window.cells.empty() || source_row < 0 || source_col < 0) {
            return {};
        }
        const std::size_t max_window = selected_agents.size() <= 2 ? 680 : 760;
        if (window.cells.size() > max_window) {
            return {};
        }

        struct Node {
            State state;
            int g = 0;
            int f = 0;
            int parent = -1;
            std::vector<int> selected_actions;
        };

        auto selected_can_move_box = [&](const State& state, char box) {
            for (int agent : selected_agents) {
                if (state.can_move_box(agent, box)) {
                    return true;
                }
            }
            return false;
        };

        auto heuristic = [&](const State& state) {
            if (state.boxes[task.goal_row][task.goal_col] == task.box) {
                return 0;
            }
            const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
            int best = kInf;
            for (const auto& [row, col] : window.cells) {
                if (state.boxes[row][col] != task.box ||
                    level_.goals[row][col] == task.box ||
                    distances[row][col] == kInf ||
                    topology_.pull_aware_dead_cell(task.box, row, col)) {
                    continue;
                }
                int best_agent_distance = kInf;
                for (int agent : selected_agents) {
                    if (!state.can_move_box(agent, task.box)) {
                        continue;
                    }
                    best_agent_distance = std::min(
                        best_agent_distance,
                        std::max(0,
                                 std::abs(state.agent_rows[agent] - row) +
                                     std::abs(state.agent_cols[agent] - col) - 1));
                }
                if (best_agent_distance != kInf) {
                    best = std::min(best, distances[row][col] + best_agent_distance);
                }
            }
            if (best == kInf) {
                return kInf;
            }
            int blocker_penalty = 0;
            for (const auto& [row, col] : window.corridor) {
                const char box = state.boxes[row][col];
                if (is_box(box) && box != task.box && level_.goals[row][col] != box) {
                    blocker_penalty += selected_can_move_box(state, box) ? 2 : 8;
                }
                const char agent = state.agent_at(row, col);
                if (is_agent(agent)) {
                    const int agent_id = agent - '0';
                    if (std::find(selected_agents.begin(), selected_agents.end(), agent_id) !=
                        selected_agents.end()) {
                        ++blocker_penalty;
                    }
                }
            }
            return best + blocker_penalty;
        };

        const int root_h = heuristic(start_);
        if (root_h == kInf) {
            return {};
        }

        std::vector<Node> nodes;
        nodes.reserve(2048);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<std::string, int> best_g;
        nodes.push_back({start_, 0, root_h, -1, std::vector<int>(selected_agents.size(), kNoOpIndex)});
        open.push(0);
        best_g[state_key_window_multi(start_, selected_agents, window)] = 0;

        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 127) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                return {};
            }
            const int index = open.top();
            open.pop();
            const Node current = nodes[static_cast<std::size_t>(index)];
            const std::string current_key = state_key_window_multi(current.state, selected_agents, window);
            const auto best_it = best_g.find(current_key);
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (current.state.boxes[task.goal_row][task.goal_col] == task.box) {
                std::vector<std::vector<int>> plan;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    std::vector<int> joint(start_.agent_rows.size(), kNoOpIndex);
                    const auto& selected_actions = nodes[static_cast<std::size_t>(cursor)].selected_actions;
                    for (std::size_t i = 0; i < selected_agents.size(); ++i) {
                        joint[static_cast<std::size_t>(selected_agents[i])] = selected_actions[i];
                    }
                    plan.push_back(std::move(joint));
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(plan.begin(), plan.end());
                return plan;
            }
            ++expansions;

            std::vector<std::vector<int>> candidates_by_agent;
            candidates_by_agent.reserve(selected_agents.size());
            for (std::size_t i = 0; i < selected_agents.size(); ++i) {
                std::vector<int> candidates =
                    local_action_candidates(current.state, selected_agents[i], window);
                const std::size_t limit = selected_agents.size() <= 2 ? 10 : (i == 0 ? 8 : 5);
                if (candidates.size() > limit) {
                    candidates.resize(limit);
                }
                candidates_by_agent.push_back(std::move(candidates));
            }

            std::vector<int> selected_actions(selected_agents.size(), kNoOpIndex);
            int generated_for_node = 0;
            const int max_generated_for_node = selected_agents.size() <= 2 ? 100 : 160;
            std::function<void(std::size_t, bool)> enumerate = [&](std::size_t depth, bool any_non_noop) {
                if (generated_for_node >= max_generated_for_node) {
                    return;
                }
                if (depth == selected_agents.size()) {
                    if (!any_non_noop) {
                        return;
                    }
                    State child = current.state;
                    std::vector<int> joint(child.agent_rows.size(), kNoOpIndex);
                    for (std::size_t i = 0; i < selected_agents.size(); ++i) {
                        joint[static_cast<std::size_t>(selected_agents[i])] = selected_actions[i];
                    }
                    if (!child.apply_joint(joint)) {
                        return;
                    }
                    const int h = heuristic(child);
                    if (h == kInf) {
                        return;
                    }
                    const int next_g = current.g + 1;
                    const std::string key = state_key_window_multi(child, selected_agents, window);
                    const auto it = best_g.find(key);
                    if (it != best_g.end() && it->second <= next_g) {
                        return;
                    }
                    best_g[key] = next_g;
                    nodes.push_back({std::move(child),
                                     next_g,
                                     next_g + 2 * h,
                                     index,
                                     selected_actions});
                    open.push(static_cast<int>(nodes.size()) - 1);
                    ++generated_for_node;
                    return;
                }
                for (int action_index : candidates_by_agent[depth]) {
                    selected_actions[depth] = action_index;
                    enumerate(depth + 1, any_non_noop || action_index != kNoOpIndex);
                    if (generated_for_node >= max_generated_for_node) {
                        break;
                    }
                }
            };
            enumerate(0, false);
        }
        return {};
    }

private:
    bool cell_in_window(const Window& window, int row, int col) const
    {
        return 0 <= row && row < level_.rows &&
               0 <= col && col < level_.cols &&
               window.inside[row][col];
    }

    std::pair<int, int> source_box_for(const BoxTask& task) const
    {
        if (task.start_row >= 0 && task.start_row < level_.rows &&
            task.start_col >= 0 && task.start_col < level_.cols &&
            start_.boxes[task.start_row][task.start_col] == task.box) {
            return {task.start_row, task.start_col};
        }
        int best_row = -1;
        int best_col = -1;
        int best_distance = kInf;
        const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (start_.boxes[row][col] != task.box ||
                    level_.goals[row][col] == task.box ||
                    distances[row][col] == kInf) {
                    continue;
                }
                if (distances[row][col] < best_distance) {
                    best_distance = distances[row][col];
                    best_row = row;
                    best_col = col;
                }
            }
        }
        return {best_row, best_col};
    }

    std::vector<std::pair<int, int>> greedy_path(int start_row,
                                                 int start_col,
                                                 int goal_row,
                                                 int goal_col)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::vector<std::pair<int, int>> path;
        const auto& distances = topology_.distances_from(goal_row, goal_col);
        if (start_row < 0 || start_row >= level_.rows ||
            start_col < 0 || start_col >= level_.cols ||
            distances[start_row][start_col] == kInf) {
            return path;
        }
        int row = start_row;
        int col = start_col;
        path.push_back({row, col});
        for (int steps = 0; steps < level_.rows * level_.cols &&
                            (row != goal_row || col != goal_col);
             ++steps) {
            int best_dir = -1;
            int best_distance = distances[row][col];
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (0 <= nr && nr < level_.rows && 0 <= nc && nc < level_.cols &&
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
            path.push_back({row, col});
        }
        return path;
    }

    void add_radius(Window& window, int center_row, int center_col, int radius) const
    {
        for (int row = center_row - radius; row <= center_row + radius; ++row) {
            for (int col = center_col - radius; col <= center_col + radius; ++col) {
                if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols ||
                    level_.walls[row][col] ||
                    std::abs(row - center_row) + std::abs(col - center_col) > radius) {
                    continue;
                }
                window.inside[row][col] = true;
            }
        }
    }

    void refresh_window_cells(Window& window) const
    {
        window.cells.clear();
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (window.inside[row][col]) {
                    window.cells.push_back({row, col});
                }
            }
        }
    }

    std::vector<int> local_action_candidates(const State& state,
                                             int agent,
                                             const Window& window) const
    {
        struct Candidate {
            int priority = 0;
            int action = kNoOpIndex;
        };
        std::vector<Candidate> candidates;
        candidates.push_back({20, kNoOpIndex});
        for (int action_index = 1; action_index < static_cast<int>(actions().size()); ++action_index) {
            const Action& action = actions()[static_cast<std::size_t>(action_index)];
            if (!state.applicable(agent, action)) {
                continue;
            }
            const auto delta = state.delta_for(agent, action);
            if (!cell_in_window(window, delta.agent_to_r, delta.agent_to_c)) {
                continue;
            }
            int priority = action.type == ActionType::Move ? 10 : 0;
            if (delta.moves_box) {
                if (!cell_in_window(window, delta.box_from_r, delta.box_from_c) ||
                    !cell_in_window(window, delta.box_to_r, delta.box_to_c) ||
                    moves_solved_box(state, agent, action)) {
                    continue;
                }
                priority -= 5;
            }
            candidates.push_back({priority, action_index});
        }
        std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& lhs,
                                                                  const Candidate& rhs) {
            if (lhs.priority != rhs.priority) {
                return lhs.priority < rhs.priority;
            }
            return lhs.action < rhs.action;
        });
        std::vector<int> actions_out;
        for (const Candidate& candidate : candidates) {
            actions_out.push_back(candidate.action);
            if (actions_out.size() >= 10) {
                break;
            }
        }
        return actions_out;
    }

    int distance_to_path(int row, int col, const std::vector<std::pair<int, int>>& path) const
    {
        int best = kInf;
        for (const auto& [path_row, path_col] : path) {
            best = std::min(best, std::abs(row - path_row) + std::abs(col - path_col));
        }
        return best;
    }

    Window build_window(int agent, const BoxTask& task)
    {
        Window window;
        window.inside.assign(static_cast<std::size_t>(level_.rows),
                             std::vector<bool>(static_cast<std::size_t>(level_.cols), false));
        auto [source_row, source_col] = source_box_for(task);
        if (source_row < 0 || source_col < 0) {
            return window;
        }
        std::vector<std::pair<int, int>> box_path =
            greedy_path(source_row, source_col, task.goal_row, task.goal_col);
        if (box_path.empty()) {
            return window;
        }
        window.corridor = box_path;
        for (const auto& [row, col] : box_path) {
            add_radius(window, row, col, 3);
        }
        for (const auto& [row, col] :
             greedy_path(start_.agent_rows[agent], start_.agent_cols[agent], source_row, source_col)) {
            add_radius(window, row, col, 2);
        }
        add_radius(window, source_row, source_col, 4);
        add_radius(window, task.goal_row, task.goal_col, 4);
        add_radius(window, start_.agent_rows[agent], start_.agent_cols[agent], 3);

        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char box = start_.boxes[row][col];
                if (!is_box(box) || level_.goals[row][col] == box) {
                    continue;
                }
                const int distance = distance_to_path(row, col, box_path);
                if (distance == 0 && box != task.box && !start_.can_move_box(agent, box)) {
                    return Window{};
                }
                if (distance <= 2) {
                    add_radius(window, row, col, 3);
                }
            }
        }

        refresh_window_cells(window);
        return window;
    }

    bool any_agent_can_move_box(const std::vector<int>& agents, char box) const
    {
        for (int agent : agents) {
            if (start_.can_move_box(agent, box)) {
                return true;
            }
        }
        return false;
    }

    Window build_window_for_agents(const std::vector<int>& agents, const BoxTask& task)
    {
        Window window;
        window.inside.assign(static_cast<std::size_t>(level_.rows),
                             std::vector<bool>(static_cast<std::size_t>(level_.cols), false));
        auto [source_row, source_col] = source_box_for(task);
        if (source_row < 0 || source_col < 0 || agents.empty()) {
            return window;
        }
        std::vector<std::pair<int, int>> box_path =
            greedy_path(source_row, source_col, task.goal_row, task.goal_col);
        if (box_path.empty()) {
            return window;
        }
        window.corridor = box_path;
        for (const auto& [row, col] : box_path) {
            add_radius(window, row, col, 3);
        }
        for (int agent : agents) {
            for (const auto& [row, col] :
                 greedy_path(start_.agent_rows[agent], start_.agent_cols[agent], source_row, source_col)) {
                add_radius(window, row, col, 2);
            }
            add_radius(window, start_.agent_rows[agent], start_.agent_cols[agent], 3);
        }
        add_radius(window, source_row, source_col, 4);
        add_radius(window, task.goal_row, task.goal_col, 4);

        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char box = start_.boxes[row][col];
                if (!is_box(box) || level_.goals[row][col] == box) {
                    continue;
                }
                const int distance = distance_to_path(row, col, box_path);
                if (distance == 0 && box != task.box && !any_agent_can_move_box(agents, box)) {
                    return Window{};
                }
                if (distance <= 2) {
                    add_radius(window, row, col, 3);
                }
            }
        }

        refresh_window_cells(window);
        return window;
    }

    std::string state_key_window(const State& state, int agent, const Window& window) const
    {
        std::string key;
        key.reserve(window.cells.size() + 16);
        key.append(std::to_string(state.agent_rows[agent]));
        key.push_back(',');
        key.append(std::to_string(state.agent_cols[agent]));
        key.push_back('|');
        for (const auto& [row, col] : window.cells) {
            key.push_back(state.boxes[row][col]);
        }
        return key;
    }

    std::string state_key_window_two(const State& state,
                                     int active_agent,
                                     int helper_agent,
                                     const Window& window) const
    {
        std::string key;
        key.reserve(window.cells.size() + 32);
        key.append(std::to_string(state.agent_rows[active_agent]));
        key.push_back(',');
        key.append(std::to_string(state.agent_cols[active_agent]));
        key.push_back(';');
        key.append(std::to_string(state.agent_rows[helper_agent]));
        key.push_back(',');
        key.append(std::to_string(state.agent_cols[helper_agent]));
        key.push_back('|');
        for (const auto& [row, col] : window.cells) {
            key.push_back(state.boxes[row][col]);
        }
        return key;
    }

    std::string state_key_window_multi(const State& state,
                                       const std::vector<int>& agents,
                                       const Window& window) const
    {
        std::string key;
        key.reserve(window.cells.size() + agents.size() * 8 + 1);
        for (int agent : agents) {
            key.append(std::to_string(state.agent_rows[agent]));
            key.push_back(',');
            key.append(std::to_string(state.agent_cols[agent]));
            key.push_back(';');
        }
        key.push_back('|');
        for (const auto& [row, col] : window.cells) {
            key.push_back(state.boxes[row][col]);
        }
        return key;
    }

    std::string state_key(const State& state, int agent) const
    {
        std::string key;
        key.reserve(static_cast<std::size_t>(level_.rows * level_.cols + 16));
        key.append(std::to_string(state.agent_rows[agent]));
        key.push_back(',');
        key.append(std::to_string(state.agent_cols[agent]));
        key.push_back('|');
        for (const std::string& row : state.boxes) {
            key.append(row.data(), row.size());
            key.push_back('\n');
        }
        return key;
    }

    bool moves_solved_box(const State& state, int agent, const Action& action) const
    {
        const auto delta = state.delta_for(agent, action);
        if (!delta.moves_box) {
            return false;
        }
        const char box = state.boxes[delta.box_from_r][delta.box_from_c];
        if (!is_box(box)) {
            return true;
        }
        if (level_.goals[delta.box_from_r][delta.box_from_c] == box &&
            !(delta.box_from_r == delta.box_to_r && delta.box_from_c == delta.box_to_c)) {
            return true;
        }
        return topology_.pull_aware_dead_cell(box, delta.box_to_r, delta.box_to_c);
    }

    const State& start_;
    Topology& topology_;
    const Level& level_;
};

class HierarchicalSolver {
public:
    explicit HierarchicalSolver(const Level& level)
        : level_(level), topology_(level), allocator_{level, topology_}
    {
        state_.level = &level_;
        state_.agent_rows = level_.agent_rows;
        state_.agent_cols = level_.agent_cols;
        state_.boxes = level_.boxes;
        initial_state_ = state_;
    }

    std::optional<std::vector<std::vector<int>>> solve()
    {
        solve_started_ = std::chrono::steady_clock::now();
        max_solve_seconds_ = enhanced_budget_seconds();
        if (!metadata_feasible()) {
            std::cerr << "[enhanced] Metadata feasibility check failed.\n";
            return std::nullopt;
        }
        std::vector<std::vector<BoxTask>> variants = allocator_.build_task_variants(initial_state_);
        if (variants.empty()) {
            variants.push_back({});
        }
        for (std::size_t variant_index = 0; variant_index < variants.size(); ++variant_index) {
            if (time_exhausted()) {
                std::cerr << "[enhanced] Internal budget exhausted before variant "
                          << (variant_index + 1) << ".\n";
                break;
            }
            reset_attempt();
            if (try_task_sequence(variants[variant_index])) {
                if (variant_index > 0) {
                    std::cerr << "[enhanced] Solved with task variant "
                              << (variant_index + 1) << ".\n";
                }
                return plan_;
            }
        }
        return std::nullopt;
    }

private:
    struct BlockerCandidate {
        int score = 0;
        int row = -1;
        int col = -1;
        char box = '\0';
    };

    struct GoalBlocker {
        int score = 0;
        int row = -1;
        int col = -1;
        char box = '\0';
    };

    struct GoalDiagnostic {
        std::string reason;
        char blocker = '\0';
        int row = -1;
        int col = -1;
    };

    bool time_exhausted() const
    {
        if (max_solve_seconds_ <= 0.0) {
            return false;
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - solve_started_).count();
        return elapsed >= max_solve_seconds_;
    }

    void reset_attempt()
    {
        state_ = initial_state_;
        plan_.clear();
        agent_work_.fill(0);
        current_sequence_ = nullptr;
        current_task_index_ = 0;
    }

    bool try_task_sequence(const std::vector<BoxTask>& tasks)
    {
        current_sequence_ = &tasks;
        for (std::size_t task_index = 0; task_index < tasks.size(); ++task_index) {
            current_task_index_ = task_index;
            const BoxTask& task = tasks[task_index];
            if (time_exhausted()) {
                return false;
            }
            if (state_.goal_state()) {
                break;
            }
            if (state_.boxes[task.goal_row][task.goal_col] == task.box) {
                continue;
            }

            // Snapshot taken BEFORE any delivery attempts so each
            // alternative-agent candidate (and the original) starts from
            // the exact same clean baseline. Without this, a failed
            // `relocate_blocker_and_deliver` that left orphan relocation
            // moves could poison subsequent retries.
            const State pre_delivery_state = state_;
            const std::size_t pre_delivery_plan_size = plan_.size();
            const auto pre_delivery_work = agent_work_;

            bool completed = deliver_task(task);
            if (!completed) {
                state_ = pre_delivery_state;
                plan_.resize(pre_delivery_plan_size);
                agent_work_ = pre_delivery_work;
                completed = relocate_blocker_and_deliver(task);
            }
            if (!completed) {
                // Roll back any partial mutations from the failed
                // relocate_blocker_and_deliver before trying alternates.
                state_ = pre_delivery_state;
                plan_.resize(pre_delivery_plan_size);
                agent_work_ = pre_delivery_work;

                auto [bx_row, bx_col] = current_box_for(task);
                if (bx_row >= 0 && bx_col >= 0) {
                    const std::vector<int> ranked =
                        rank_agents_for_box(task.box, bx_row, bx_col,
                                            task.goal_row, task.goal_col);
                    const int original_choice = choose_agent(
                        task.box, bx_row, bx_col,
                        task.goal_row, task.goal_col);
                    const std::set<int> save_forbidden = forbidden_agents_;
                    for (int candidate : ranked) {
                        if (time_exhausted()) {
                            break;
                        }
                        if (candidate == original_choice) {
                            continue;
                        }
                        state_ = pre_delivery_state;
                        plan_.resize(pre_delivery_plan_size);
                        agent_work_ = pre_delivery_work;
                        forbidden_agents_ = save_forbidden;
                        for (int other : ranked) {
                            if (other != candidate) {
                                forbidden_agents_.insert(other);
                            }
                        }
                        bool ok = deliver_task(task);
                        if (!ok) {
                            state_ = pre_delivery_state;
                            plan_.resize(pre_delivery_plan_size);
                            agent_work_ = pre_delivery_work;
                            forbidden_agents_ = save_forbidden;
                            for (int other : ranked) {
                                if (other != candidate) {
                                    forbidden_agents_.insert(other);
                                }
                            }
                            ok = relocate_blocker_and_deliver(task);
                        }
                        forbidden_agents_ = save_forbidden;
                        if (ok) {
                            std::cerr << "[enhanced] Alt-agent retry: agent "
                                      << candidate << " delivered " << task.box
                                      << ".\n";
                            completed = true;
                            break;
                        }
                    }
                    if (!completed) {
                        state_ = pre_delivery_state;
                        plan_.resize(pre_delivery_plan_size);
                        agent_work_ = pre_delivery_work;
                    }
                }
            }
            if (!completed) {
                std::cerr << "[enhanced] Failed task " << task.box
                          << " (" << task.start_row << "," << task.start_col << " -> "
                          << task.goal_row << "," << task.goal_col << ").\n";
                return false;
            }
        }
        current_sequence_ = nullptr;
        if (!complete_agent_goals()) {
            std::cerr << "[enhanced] Failed to complete agent goals.\n";
            return false;
        }
        if (!state_.goal_state()) {
            std::cerr << "[enhanced] Exhausted tasks before reaching goal state.\n";
            return false;
        }
        return true;
    }

    bool metadata_feasible() const
    {
        std::array<int, 26> goals{};
        std::array<int, 26> boxes{};
        std::array<int, 26> unsatisfied_goals{};
        goals.fill(0);
        boxes.fill(0);
        unsatisfied_goals.fill(0);
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char goal_cell = level_.goals[row][col];
                const char box_cell = state_.boxes[row][col];
                if (is_box(goal_cell)) {
                    ++goals[goal_cell - 'A'];
                    if (box_cell != goal_cell) {
                        ++unsatisfied_goals[goal_cell - 'A'];
                    }
                }
                if (is_box(box_cell)) {
                    ++boxes[box_cell - 'A'];
                }
            }
        }
        for (int letter = 0; letter < 26; ++letter) {
            if (goals[letter] > boxes[letter]) {
                return false;
            }
            if (unsatisfied_goals[letter] == 0) {
                // Either no goals for this letter, or every goal cell is
                // already covered by a matching box in the initial state.
                // Such letters require no agent action.
                continue;
            }
            bool has_agent = false;
            for (int color : level_.agent_color) {
                if (color == level_.box_color[letter]) {
                    has_agent = true;
                    break;
                }
            }
            if (!has_agent) {
                return false;
            }
        }
        return true;
    }

    std::pair<int, int> current_box_for(const BoxTask& task)
    {
        if (task.start_row >= 0 && task.start_row < level_.rows &&
            task.start_col >= 0 && task.start_col < level_.cols &&
            state_.boxes[task.start_row][task.start_col] == task.box) {
            return {task.start_row, task.start_col};
        }
        int best_row = -1;
        int best_col = -1;
        int best_distance = kInf;
        const auto& distances = topology_.distances_from(task.goal_row, task.goal_col);
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (state_.boxes[row][col] != task.box || level_.goals[row][col] == task.box) {
                    continue;
                }
                if (distances[row][col] < best_distance) {
                    best_distance = distances[row][col];
                    best_row = row;
                    best_col = col;
                }
            }
        }
        return {best_row, best_col};
    }

    int choose_agent(char box, int box_row, int box_col, int goal_row, int goal_col)
    {
        int best_agent = -1;
        int best_score = kInf;
        const auto& to_box = topology_.distances_from(box_row, box_col);
        const auto& to_goal = topology_.distances_from(goal_row, goal_col);
        for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
            if (!state_.can_move_box(agent, box)) {
                continue;
            }
            if (forbidden_agents_.count(agent)) {
                continue;
            }
            const int agent_distance = to_box[state_.agent_rows[agent]][state_.agent_cols[agent]];
            const int box_distance = to_goal[box_row][box_col];
            if (agent_distance == kInf || box_distance == kInf) {
                continue;
            }
            const int load_penalty = agent_work_[agent] * 4;
            const int score = agent_distance + box_distance + load_penalty;
            if (score < best_score) {
                best_score = score;
                best_agent = agent;
            }
        }
        return best_agent;
    }

    // Ranked candidate agents (best-first) capable of moving `box` from
    // (box_row, box_col) to (goal_row, goal_col). Used for alternative-
    // agent retry on full task failure.
    std::vector<int> rank_agents_for_box(char box, int box_row, int box_col,
                                         int goal_row, int goal_col)
    {
        struct Cand {
            int agent;
            int score;
        };
        std::vector<Cand> cands;
        const auto& to_box = topology_.distances_from(box_row, box_col);
        const auto& to_goal = topology_.distances_from(goal_row, goal_col);
        for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
            if (!state_.can_move_box(agent, box)) {
                continue;
            }
            const int agent_distance = to_box[state_.agent_rows[agent]][state_.agent_cols[agent]];
            const int box_distance = to_goal[box_row][box_col];
            if (agent_distance == kInf || box_distance == kInf) {
                continue;
            }
            cands.push_back({agent, agent_distance + box_distance + agent_work_[agent] * 4});
        }
        std::stable_sort(cands.begin(), cands.end(),
                         [](const Cand& a, const Cand& b) { return a.score < b.score; });
        std::vector<int> out;
        out.reserve(cands.size());
        for (const auto& c : cands) {
            out.push_back(c.agent);
        }
        return out;
    }

    int choose_helper_agent(int active_agent, char box, int box_row, int box_col)
    {
        int best_agent = -1;
        int best_score = kInf;
        const auto& to_box = topology_.distances_from(box_row, box_col);
        for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
            if (agent == active_agent || !state_.can_move_box(agent, box)) {
                continue;
            }
            const int distance = to_box[state_.agent_rows[agent]][state_.agent_cols[agent]];
            if (distance == kInf) {
                continue;
            }
            const int score = distance + agent_work_[agent] * 4;
            if (score < best_score) {
                best_score = score;
                best_agent = agent;
            }
        }
        return best_agent;
    }

    bool deliver_task(BoxTask task)
    {
        auto [box_row, box_col] = current_box_for(task);
        if (box_row < 0 || box_col < 0) {
            return false;
        }
        if (box_row == task.goal_row && box_col == task.goal_col) {
            return true;
        }
        const int agent = choose_agent(task.box, box_row, box_col, task.goal_row, task.goal_col);
        if (agent < 0) {
            return false;
        }
        SingleBoxPlanner planner(state_, topology_);
        std::vector<int> actions = planner.plan_box(agent,
                                                    task.box,
                                                    box_row,
                                                    box_col,
                                                    task.goal_row,
                                                    task.goal_col,
                                                    enhanced_single_box_expansions());
        if (actions.empty()) {
            if (retry_delivery_after_agent_evacuation(agent, task)) {
                return true;
            }
            if (cbs_box_repair(agent, task)) {
                return true;
            }
            if (two_agent_local_repair(agent, task)) {
                return true;
            }
            return local_multibox_repair(agent, task);
        }
        if (!append_serial(agent, actions)) {
            return false;
        }
        ++agent_work_[agent];
        return state_.boxes[task.goal_row][task.goal_col] == task.box;
    }

    std::chrono::steady_clock::time_point solve_deadline() const
    {
        return solve_started_ +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(max_solve_seconds_));
    }

    void add_path_radius(std::set<std::pair<int, int>>& cells,
                         const std::vector<std::pair<int, int>>& path,
                         int radius) const
    {
        for (const auto& [center_row, center_col] : path) {
            for (int row = center_row - radius; row <= center_row + radius; ++row) {
                for (int col = center_col - radius; col <= center_col + radius; ++col) {
                    if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols ||
                        level_.walls[row][col] ||
                        std::abs(row - center_row) + std::abs(col - center_col) > radius) {
                        continue;
                    }
                    cells.insert({row, col});
                }
            }
        }
    }

    std::vector<std::pair<int, int>> evacuation_targets(
        int agent,
        const std::set<std::pair<int, int>>& forbidden)
    {
        struct Candidate {
            int score = 0;
            int row = 0;
            int col = 0;
        };
        std::vector<Candidate> candidates;
        const auto& distances = topology_.distances_from(state_.agent_rows[agent], state_.agent_cols[agent]);
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (forbidden.count({row, col}) != 0 ||
                    distances[row][col] == kInf ||
                    !state_.cell_free(row, col)) {
                    continue;
                }
                int degree = 0;
                for (int dir = 0; dir < 4; ++dir) {
                    const int nr = row + dr[dir];
                    const int nc = col + dc[dir];
                    if (0 <= nr && nr < level_.rows && 0 <= nc && nc < level_.cols &&
                        !level_.walls[nr][nc]) {
                        ++degree;
                    }
                }
                const int goal_penalty = level_.goals[row][col] == '\0' ? 0 : 25;
                candidates.push_back({distances[row][col] * 3 -
                                      degree * 2 +
                                      goal_penalty +
                                      future_path_penalty(row, col),
                                      row,
                                      col});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
            }
            if (lhs.row != rhs.row) {
                return lhs.row < rhs.row;
            }
            return lhs.col < rhs.col;
        });
        std::vector<std::pair<int, int>> targets;
        for (const Candidate& candidate : candidates) {
            targets.push_back({candidate.row, candidate.col});
            if (targets.size() >= 12) {
                break;
            }
        }
        return targets;
    }

    bool evacuate_corridor_agents(int active_agent,
                                  int box_row,
                                  int box_col,
                                  int goal_row,
                                  int goal_col)
    {
        std::set<std::pair<int, int>> forbidden;
        const auto box_path = rough_path(box_row, box_col, goal_row, goal_col);
        const auto agent_path = rough_path(state_.agent_rows[active_agent],
                                           state_.agent_cols[active_agent],
                                           box_row,
                                           box_col);
        if (box_path.empty()) {
            return false;
        }
        add_path_radius(forbidden, box_path, 1);
        add_path_radius(forbidden, agent_path, 1);
        forbidden.insert({box_row, box_col});
        forbidden.insert({goal_row, goal_col});

        bool moved_any = false;
        for (int pass = 0; pass < 2; ++pass) {
            bool moved_this_pass = false;
            for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
                if (agent == active_agent ||
                    forbidden.count({state_.agent_rows[agent], state_.agent_cols[agent]}) == 0) {
                    continue;
                }
                bool moved = false;
                for (const auto& [target_row, target_col] : evacuation_targets(agent, forbidden)) {
                    SingleBoxPlanner planner(state_, topology_);
                    std::vector<int> moves = planner.plan_agent_to(agent, target_row, target_col, 20000);
                    if (moves.empty()) {
                        continue;
                    }
                    if (append_serial(agent, moves)) {
                        moved = true;
                        moved_any = true;
                        moved_this_pass = true;
                        break;
                    }
                }
                if (!moved) {
                    return false;
                }
            }
            if (!moved_this_pass) {
                break;
            }
        }
        return moved_any;
    }

    bool retry_delivery_after_agent_evacuation(int active_agent, const BoxTask& task)
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;
        auto [box_row, box_col] = current_box_for(task);
        if (box_row < 0 || box_col < 0 ||
            !evacuate_corridor_agents(active_agent, box_row, box_col, task.goal_row, task.goal_col)) {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
            return false;
        }

        std::tie(box_row, box_col) = current_box_for(task);
        SingleBoxPlanner planner(state_, topology_);
        std::vector<int> actions = planner.plan_box(active_agent,
                                                    task.box,
                                                    box_row,
                                                    box_col,
                                                    task.goal_row,
                                                    task.goal_col,
                                                    enhanced_single_box_expansions());
        if (actions.empty() || !append_serial(active_agent, actions) ||
            state_.boxes[task.goal_row][task.goal_col] != task.box) {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
            return false;
        }
        ++agent_work_[active_agent];
        std::cerr << "[enhanced] Evacuated corridor agents for " << task.box
                  << " to (" << task.goal_row << "," << task.goal_col << ").\n";
        return true;
    }

    int distance_to_path(int row, int col, const std::vector<std::pair<int, int>>& path) const
    {
        int best = kInf;
        for (const auto& [path_row, path_col] : path) {
            best = std::min(best, std::abs(row - path_row) + std::abs(col - path_col));
        }
        return best;
    }

    std::vector<int> choose_cbs_helper_agents(int active_agent,
                                              const BoxTask& task,
                                              int box_row,
                                              int box_col)
    {
        struct Candidate {
            int score = 0;
            int agent = -1;
        };
        std::vector<std::pair<int, int>> path = rough_path(box_row, box_col, task.goal_row, task.goal_col);
        if (path.empty()) {
            return {};
        }
        const auto& to_box = topology_.distances_from(box_row, box_col);
        std::vector<Candidate> candidates;
        for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
            if (agent == active_agent) {
                continue;
            }
            const int path_distance = distance_to_path(state_.agent_rows[agent],
                                                       state_.agent_cols[agent],
                                                       path);
            const int agent_to_box = to_box[state_.agent_rows[agent]][state_.agent_cols[agent]];
            bool can_move_path_blocker = false;
            for (int row = 0; row < level_.rows && !can_move_path_blocker; ++row) {
                for (int col = 0; col < level_.cols && !can_move_path_blocker; ++col) {
                    const char box = state_.boxes[row][col];
                    if (!is_box(box) ||
                        level_.goals[row][col] == box ||
                        distance_to_path(row, col, path) > 1) {
                        continue;
                    }
                    can_move_path_blocker = state_.can_move_box(agent, box);
                }
            }
            const bool can_move_task_box = state_.can_move_box(agent, task.box);
            if (path_distance > 4 &&
                agent_to_box == kInf &&
                !can_move_path_blocker &&
                !can_move_task_box) {
                continue;
            }
            const int reach_score = agent_to_box == kInf ? path_distance * 20 : std::min(agent_to_box, path_distance * 4);
            candidates.push_back({path_distance * 12 +
                                      reach_score * 2 +
                                      agent_work_[agent] * 4 -
                                      (path_distance == 0 ? 50 : 0) -
                                      (can_move_path_blocker ? 35 : 0) -
                                      (can_move_task_box ? 15 : 0),
                                  agent});
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
            }
            return lhs.agent < rhs.agent;
        });

        std::vector<int> helpers;
        const int max_helpers = enhanced_cbs_box_repair_max_agents() - 1;
        for (const Candidate& candidate : candidates) {
            helpers.push_back(candidate.agent);
            if (static_cast<int>(helpers.size()) >= max_helpers) {
                break;
            }
        }
        if (helpers.empty()) {
            const int same_color_helper = choose_helper_agent(active_agent, task.box, box_row, box_col);
            if (same_color_helper >= 0) {
                helpers.push_back(same_color_helper);
            }
        }
        return helpers;
    }

    bool cbs_box_repair(int active_agent, const BoxTask& task)
    {
        if (!enhanced_cbs_box_repair_enabled() || time_exhausted()) {
            return false;
        }
        auto [box_row, box_col] = current_box_for(task);
        if (box_row < 0 || box_col < 0) {
            return false;
        }
        const std::vector<int> helper_agents =
            choose_cbs_helper_agents(active_agent, task, box_row, box_col);
        if (helper_agents.empty()) {
            return false;
        }

        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;
        auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(enhanced_cbs_box_repair_seconds()));
        if (solve_deadline() < deadline) {
            deadline = solve_deadline();
        }

        LocalMultiBoxPlanner planner(state_, topology_);
        const int expansion_cap = enhanced_cbs_box_repair_max_agents() <= 2 ? 35000 : 22000;
        std::vector<std::vector<int>> joint_plan =
            planner.plan_cbs_box_repair(active_agent, helper_agents, task, expansion_cap, deadline);
        if (joint_plan.empty() || !append_joint_sequence(joint_plan) ||
            state_.boxes[task.goal_row][task.goal_col] != task.box) {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
            return false;
        }
        ++agent_work_[active_agent];
        for (int helper : helper_agents) {
            ++agent_work_[helper];
        }
        std::cerr << "[enhanced] CBS-style box repair delivered " << task.box
                  << " to (" << task.goal_row << "," << task.goal_col << ") with "
                  << (helper_agents.size() + 1) << " agents.\n";
        return true;
    }

    bool two_agent_local_repair(int active_agent, const BoxTask& task)
    {
        if (!enhanced_two_agent_repair_enabled() || time_exhausted()) {
            return false;
        }
        auto [box_row, box_col] = current_box_for(task);
        if (box_row < 0 || box_col < 0) {
            return false;
        }
        const int helper_agent = choose_helper_agent(active_agent, task.box, box_row, box_col);
        if (helper_agent < 0) {
            return false;
        }

        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;
        auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(enhanced_two_agent_repair_seconds()));
        if (solve_deadline() < deadline) {
            deadline = solve_deadline();
        }

        LocalMultiBoxPlanner planner(state_, topology_);
        std::vector<std::vector<int>> joint_plan =
            planner.plan_two_agent(active_agent, helper_agent, task, 25000, deadline);
        if (joint_plan.empty() || !append_joint_sequence(joint_plan) ||
            state_.boxes[task.goal_row][task.goal_col] != task.box) {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
            return false;
        }
        ++agent_work_[active_agent];
        ++agent_work_[helper_agent];
        std::cerr << "[enhanced] Two-agent repair delivered " << task.box
                  << " to (" << task.goal_row << "," << task.goal_col << ").\n";
        return true;
    }

    bool local_multibox_repair(int agent, const BoxTask& task)
    {
        if (time_exhausted()) {
            return false;
        }
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;
        LocalMultiBoxPlanner planner(state_, topology_);
        auto local_deadline =
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(enhanced_local_repair_seconds()));
        if (solve_deadline() < local_deadline) {
            local_deadline = solve_deadline();
        }
        std::vector<int> actions;
        if (enhanced_neighborhood_repair_enabled()) {
            auto neighborhood_deadline =
                std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(enhanced_neighborhood_repair_seconds()));
            if (local_deadline < neighborhood_deadline) {
                neighborhood_deadline = local_deadline;
            }
            actions = planner.plan_neighborhood(agent, task, 30000, neighborhood_deadline);
            if (!actions.empty() && append_serial(agent, actions) &&
                state_.boxes[task.goal_row][task.goal_col] == task.box) {
                ++agent_work_[agent];
                std::cerr << "[enhanced] Neighborhood repair delivered " << task.box
                          << " to (" << task.goal_row << "," << task.goal_col << ").\n";
                return true;
            }
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
        }

        actions = planner.plan(agent, task, 250000, local_deadline);
        if (actions.empty() || !append_serial(agent, actions) ||
            state_.boxes[task.goal_row][task.goal_col] != task.box) {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
            return false;
        }
        ++agent_work_[agent];
        std::cerr << "[enhanced] Local multi-box repair delivered " << task.box
                  << " to (" << task.goal_row << "," << task.goal_col << ").\n";
        return true;
    }

    std::vector<std::pair<int, int>> rough_path(int start_row, int start_col, int goal_row, int goal_col)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::vector<std::pair<int, int>> path;
        const auto& distances = topology_.distances_from(goal_row, goal_col);
        if (distances[start_row][start_col] == kInf) {
            return path;
        }
        int row = start_row;
        int col = start_col;
        path.push_back({row, col});
        while (row != goal_row || col != goal_col) {
            int best_dir = -1;
            int best_distance = distances[row][col];
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (0 <= nr && nr < level_.rows && 0 <= nc && nc < level_.cols &&
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
            path.push_back({row, col});
        }
        return path;
    }

    int future_path_penalty(int row, int col)
    {
        if (current_sequence_ == nullptr) {
            return 0;
        }
        int penalty = 0;
        const std::size_t start = std::min(current_task_index_ + 1, current_sequence_->size());
        for (std::size_t i = start; i < current_sequence_->size(); ++i) {
            const BoxTask& pending = (*current_sequence_)[i];
            if (state_.boxes[pending.goal_row][pending.goal_col] == pending.box) {
                continue;
            }
            const auto [box_row, box_col] = current_box_for(pending);
            if (box_row < 0 || box_col < 0) {
                continue;
            }
            for (const auto& [path_row, path_col] :
                 rough_path(box_row, box_col, pending.goal_row, pending.goal_col)) {
                const int distance = std::abs(row - path_row) + std::abs(col - path_col);
                if (distance == 0) {
                    penalty += 40;
                } else if (distance == 1) {
                    penalty += 8;
                }
            }
        }
        return penalty;
    }

    int agent_goal_route_penalty(int row, int col)
    {
        int penalty = 0;
        const auto targets = agent_goal_targets();
        for (int agent = 0; agent < static_cast<int>(targets.size()); ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            if (target_row < 0 ||
                (state_.agent_rows[agent] == target_row && state_.agent_cols[agent] == target_col)) {
                continue;
            }
            for (const auto& [path_row, path_col] :
                 rough_path(state_.agent_rows[agent], state_.agent_cols[agent], target_row, target_col)) {
                const int distance = std::abs(row - path_row) + std::abs(col - path_col);
                if (distance == 0) {
                    penalty += 35;
                } else if (distance == 1) {
                    penalty += 6;
                }
            }
        }
        return penalty;
    }

    std::vector<std::pair<int, int>> parking_cells(char box,
                                                   int from_row,
                                                   int from_col,
                                                   const std::set<std::pair<int, int>>& forbidden,
                                                   int limit)
    {
        const auto& distances = topology_.distances_from(from_row, from_col);
        struct Candidate {
            int score = 0;
            int row = 0;
            int col = 0;
        };
        std::vector<Candidate> candidates;
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                if (distances[row][col] == kInf ||
                    level_.walls[row][col] ||
                    level_.goals[row][col] != '\0' ||
                    state_.boxes[row][col] != '\0' ||
                    state_.agent_at(row, col) != '\0' ||
                    forbidden.count({row, col}) != 0 ||
                    topology_.pull_aware_dead_cell(box, row, col)) {
                    continue;
                }
                int degree = 0;
                static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
                static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
                for (int dir = 0; dir < 4; ++dir) {
                    const int nr = row + dr[dir];
                    const int nc = col + dc[dir];
                    if (0 <= nr && nr < level_.rows && 0 <= nc && nc < level_.cols &&
                        !level_.walls[nr][nc]) {
                        ++degree;
                    }
                }
                int score = distances[row][col] * 4 -
                            degree * 3 +
                            future_path_penalty(row, col);
                if (enhanced_relocation_graph_enabled()) {
                    score += agent_goal_route_penalty(row, col);
                    if (degree <= 1) {
                        score += 20;
                    }
                }
                candidates.push_back({score, row, col});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
            }
            if (lhs.row != rhs.row) {
                return lhs.row < rhs.row;
            }
            return lhs.col < rhs.col;
        });
        std::vector<std::pair<int, int>> result;
        for (const Candidate& candidate : candidates) {
            result.push_back({candidate.row, candidate.col});
            if (static_cast<int>(result.size()) >= limit) {
                break;
            }
        }
        return result;
    }

    std::vector<BlockerCandidate> blocker_candidates_for_path(
        const std::vector<std::pair<int, int>>& path,
        const std::set<std::pair<int, int>>& ignored_cells,
        int max_distance)
    {
        std::vector<BlockerCandidate> blockers;
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char blocker = state_.boxes[row][col];
                if (!is_box(blocker) ||
                    ignored_cells.count({row, col}) != 0 ||
                    level_.goals[row][col] == blocker) {
                    continue;
                }
                int best_path_distance = kInf;
                int path_index = 0;
                int best_path_index = 0;
                for (const auto& [path_row, path_col] : path) {
                    const int distance = std::abs(row - path_row) + std::abs(col - path_col);
                    if (distance < best_path_distance) {
                        best_path_distance = distance;
                        best_path_index = path_index;
                    }
                    ++path_index;
                }
                if (best_path_distance > max_distance) {
                    continue;
                }
                blockers.push_back({best_path_distance * 100 + best_path_index,
                                    row,
                                    col,
                                    blocker});
            }
        }
        std::sort(blockers.begin(), blockers.end(), [](const BlockerCandidate& lhs,
                                                       const BlockerCandidate& rhs) {
            if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
            }
            if (lhs.row != rhs.row) {
                return lhs.row < rhs.row;
            }
            return lhs.col < rhs.col;
        });
        return blockers;
    }

    bool try_relocate_box(char box,
                          int from_row,
                          int from_col,
                          int target_row,
                          int target_col,
                          const std::set<std::pair<int, int>>& forbidden,
                          std::chrono::steady_clock::time_point deadline,
                          int depth)
    {
        if (time_exhausted() || std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        if (state_.boxes[target_row][target_col] == box) {
            return true;
        }
        if (state_.boxes[from_row][from_col] != box) {
            return false;
        }
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;

        auto restore_saved = [&]() {
            state_ = saved_state;
            plan_.resize(saved_plan_size);
            agent_work_ = saved_agent_work;
        };

        auto attempt_direct = [&]() {
            const int agent = choose_agent(box, from_row, from_col, target_row, target_col);
            if (agent < 0) {
                return false;
            }
            SingleBoxPlanner planner(state_, topology_);
            std::vector<int> relocate =
                planner.plan_box(agent, box, from_row, from_col, target_row, target_col, 90000, deadline);
            return !relocate.empty() &&
                   append_serial(agent, relocate) &&
                   state_.boxes[target_row][target_col] == box;
        };

        if (attempt_direct()) {
            return true;
        }
        if (depth <= 0) {
            restore_saved();
            return false;
        }

        std::vector<std::pair<int, int>> path =
            rough_path(from_row, from_col, target_row, target_col);
        if (path.empty()) {
            restore_saved();
            return false;
        }
        std::set<std::pair<int, int>> combined_forbidden = forbidden;
        combined_forbidden.insert(path.begin(), path.end());
        combined_forbidden.insert({target_row, target_col});
        combined_forbidden.insert({from_row, from_col});

        const std::vector<BlockerCandidate> blockers =
            blocker_candidates_for_path(path, {{from_row, from_col}}, 1);
        const int blocker_limit = std::min<int>(4, static_cast<int>(blockers.size()));
        for (int blocker_index = 0; blocker_index < blocker_limit; ++blocker_index) {
            if (time_exhausted() || std::chrono::steady_clock::now() >= deadline) {
                restore_saved();
                return false;
            }
            const BlockerCandidate& blocker = blockers[blocker_index];
            const std::vector<std::pair<int, int>> parking_options =
                parking_cells(blocker.box, blocker.row, blocker.col, combined_forbidden, 4);
            for (const auto& parking : parking_options) {
                if (time_exhausted() || std::chrono::steady_clock::now() >= deadline) {
                    restore_saved();
                    return false;
                }
                const State branch_state = state_;
                const std::size_t branch_plan_size = plan_.size();
                const auto branch_agent_work = agent_work_;
                if (try_relocate_box(blocker.box,
                                     blocker.row,
                                     blocker.col,
                                     parking.first,
                                     parking.second,
                                     combined_forbidden,
                                     deadline,
                                     depth - 1) &&
                    attempt_direct()) {
                    return true;
                }
                state_ = branch_state;
                plan_.resize(branch_plan_size);
                agent_work_ = branch_agent_work;
            }
        }

        restore_saved();
        return false;
    }

    bool relocate_blocker_and_deliver(BoxTask task)
    {
        // Anti-thrash: blacklist (blocker letter, from_row, from_col,
        // parking_row, parking_col) pairs that have already been tried
        // for THIS task without unlocking delivery. Also reject immediate
        // undo moves where the just-relocated box would be sent back to
        // exactly where it came from in the previous round.
        std::set<std::tuple<char, int, int, int, int>> tried_pairs;
        std::tuple<char, int, int, int, int> last_move{
            '\0', -1, -1, -1, -1};
        for (int repair_round = 0; repair_round < 6; ++repair_round) {
            if (time_exhausted()) {
                return false;
            }
            auto [box_row, box_col] = current_box_for(task);
            if (box_row < 0 || box_col < 0) {
                return false;
            }
            std::vector<std::pair<int, int>> path = rough_path(box_row, box_col, task.goal_row, task.goal_col);
            std::set<std::pair<int, int>> forbidden(path.begin(), path.end());
            forbidden.insert({task.goal_row, task.goal_col});

            const std::vector<BlockerCandidate> blockers =
                blocker_candidates_for_path(path, {{box_row, box_col}}, 1);

            bool relocated = false;
            const int blocker_limit = std::min<int>(8, static_cast<int>(blockers.size()));
            for (int blocker_index = 0; blocker_index < blocker_limit && !relocated; ++blocker_index) {
                if (time_exhausted()) {
                    return false;
                }
                const BlockerCandidate& blocker = blockers[blocker_index];
                const std::vector<std::pair<int, int>> parking_options =
                    parking_cells(blocker.box, blocker.row, blocker.col, forbidden, 6);
                for (const auto& parking : parking_options) {
                    if (time_exhausted()) {
                        return false;
                    }
                    const std::tuple<char, int, int, int, int> pair_key{
                        blocker.box, blocker.row, blocker.col, parking.first, parking.second};
                    if (tried_pairs.count(pair_key)) {
                        continue;
                    }
                    // Reject the exact reverse of the last successful
                    // relocation (would re-create the previous blocker
                    // configuration).
                    if (std::get<0>(last_move) == blocker.box &&
                        std::get<1>(last_move) == parking.first &&
                        std::get<2>(last_move) == parking.second &&
                        std::get<3>(last_move) == blocker.row &&
                        std::get<4>(last_move) == blocker.col) {
                        continue;
                    }
                    auto relocation_deadline =
                        std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(enhanced_relocation_seconds()));
                    if (solve_deadline() < relocation_deadline) {
                        relocation_deadline = solve_deadline();
                    }
                    if (!try_relocate_box(blocker.box,
                                          blocker.row,
                                          blocker.col,
                                          parking.first,
                                          parking.second,
                                          forbidden,
                                          relocation_deadline,
                                          enhanced_relocation_depth())) {
                        continue;
                    }
                    std::cerr << "[enhanced] Relocated blocker " << blocker.box << " from ("
                              << blocker.row << "," << blocker.col << ") to (" << parking.first << ","
                              << parking.second << ").\n";
                    relocated = true;
                    tried_pairs.insert(pair_key);
                    last_move = pair_key;
                    break;
                }
            }
            if (!relocated) {
                return false;
            }
            if (deliver_task(task)) {
                return true;
            }
        }
        return false;
    }

    bool complete_agent_goals()
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;
        if (complete_agent_goals_concurrent()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        if (complete_agent_goals_serial()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        if (complete_agent_goals_pibt()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        if (complete_agent_goals_joint_search()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        if (complete_agent_goals_with_relocation()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        log_final_goal_diagnostics(agent_goal_targets());
        return false;
    }

    std::vector<std::pair<int, int>> agent_goal_targets() const
    {
        const int num_agents = static_cast<int>(state_.agent_rows.size());
        std::vector<std::pair<int, int>> targets(static_cast<std::size_t>(num_agents), {-1, -1});
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char goal = level_.goals[row][col];
                if (!is_agent(goal)) {
                    continue;
                }
                const int agent = goal - '0';
                if (0 <= agent && agent < num_agents) {
                    targets[static_cast<std::size_t>(agent)] = {row, col};
                }
            }
        }
        return targets;
    }

    bool try_complete_agent_goals_once()
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        if (complete_agent_goals_concurrent()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        if (complete_agent_goals_serial()) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);
        return false;
    }

    bool complete_agent_goals_with_relocation()
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        const auto saved_agent_work = agent_work_;

        for (int round = 0; round < 6; ++round) {
            if (time_exhausted()) {
                break;
            }
            const State round_state = state_;
            const std::size_t round_plan_size = plan_.size();
            const auto round_agent_work = agent_work_;
            if (try_complete_agent_goals_once()) {
                return true;
            }
            state_ = round_state;
            plan_.resize(round_plan_size);
            agent_work_ = round_agent_work;

            const auto targets = agent_goal_targets();
            if (evacuate_final_goal_agent_blockers(targets)) {
                continue;
            }
            std::set<std::pair<int, int>> forbidden(targets.begin(), targets.end());
            std::vector<GoalBlocker> blockers;
            std::set<std::pair<int, int>> seen_blockers;

            for (int agent = 0; agent < static_cast<int>(targets.size()); ++agent) {
                const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
                if (target_row < 0) {
                    continue;
                }
                const auto path = rough_path(state_.agent_rows[agent],
                                             state_.agent_cols[agent],
                                             target_row,
                                             target_col);
                forbidden.insert(path.begin(), path.end());
                int path_index = 0;
                for (const auto& [row, col] : path) {
                    const char box = state_.boxes[row][col];
                    if (is_box(box) &&
                        level_.goals[row][col] != box &&
                        seen_blockers.insert({row, col}).second) {
                        const int target_bonus = (row == target_row && col == target_col) ? -100 : 0;
                        blockers.push_back({path_index + target_bonus, row, col, box});
                    }
                    ++path_index;
                }
                const char target_box = state_.boxes[target_row][target_col];
                if (is_box(target_box) &&
                    level_.goals[target_row][target_col] != target_box &&
                    seen_blockers.insert({target_row, target_col}).second) {
                        blockers.push_back({-200, target_row, target_col, target_box});
                }
                for (const GoalBlocker& blocker :
                     agent_goal_route_blockers(agent, target_row, target_col)) {
                    if (seen_blockers.insert({blocker.row, blocker.col}).second) {
                        blockers.push_back(blocker);
                    }
                }
            }

            std::sort(blockers.begin(), blockers.end(), [](const GoalBlocker& lhs,
                                                           const GoalBlocker& rhs) {
                if (lhs.score != rhs.score) {
                    return lhs.score < rhs.score;
                }
                if (lhs.row != rhs.row) {
                    return lhs.row < rhs.row;
                }
                return lhs.col < rhs.col;
            });

            bool relocated = false;
            const int blocker_limit = std::min<int>(6, static_cast<int>(blockers.size()));
            for (int blocker_index = 0; blocker_index < blocker_limit && !relocated; ++blocker_index) {
                const GoalBlocker& blocker = blockers[blocker_index];
                const auto parking_options =
                    parking_cells(blocker.box, blocker.row, blocker.col, forbidden, 8);
                for (const auto& parking : parking_options) {
                    auto deadline =
                        std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(enhanced_relocation_seconds()));
                    if (solve_deadline() < deadline) {
                        deadline = solve_deadline();
                    }
                    const State branch_state = state_;
                    const std::size_t branch_plan_size = plan_.size();
                    const auto branch_agent_work = agent_work_;
                    if (try_relocate_box(blocker.box,
                                         blocker.row,
                                         blocker.col,
                                         parking.first,
                                         parking.second,
                                         forbidden,
                                         deadline,
                                         enhanced_relocation_depth())) {
                        std::cerr << "[enhanced] Relocated final-goal blocker "
                                  << blocker.box << " from (" << blocker.row << ","
                                  << blocker.col << ") to (" << parking.first
                                  << "," << parking.second << ").\n";
                        relocated = true;
                        break;
                    }
                    state_ = branch_state;
                    plan_.resize(branch_plan_size);
                    agent_work_ = branch_agent_work;
                }
            }
            if (!relocated) {
                break;
            }
        }

        state_ = saved_state;
        plan_.resize(saved_plan_size);
        agent_work_ = saved_agent_work;
        return false;
    }

    bool evacuate_final_goal_agent_blockers(const std::vector<std::pair<int, int>>& targets)
    {
        std::set<std::pair<int, int>> forbidden;
        std::set<int> agents_to_move;
        for (int agent = 0; agent < static_cast<int>(targets.size()); ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            if (target_row < 0 ||
                (state_.agent_rows[agent] == target_row && state_.agent_cols[agent] == target_col)) {
                if (target_row >= 0) {
                    forbidden.insert({target_row, target_col});
                }
            } else {
                const auto path = rough_path(state_.agent_rows[agent],
                                             state_.agent_cols[agent],
                                             target_row,
                                             target_col);
                for (const auto& cell : path) {
                    forbidden.insert(cell);
                    const char occupant = state_.agent_at(cell.first, cell.second);
                    if (is_agent(occupant) && occupant - '0' != agent) {
                        agents_to_move.insert(occupant - '0');
                    }
                }
                forbidden.insert({target_row, target_col});
            }

            if (target_row >= 0) {
                const char occupant = state_.agent_at(target_row, target_col);
                if (is_agent(occupant) && occupant - '0' != agent) {
                    agents_to_move.insert(occupant - '0');
                }
            }
        }
        for (int agent = 0; agent < static_cast<int>(state_.agent_rows.size()); ++agent) {
            if (forbidden.count({state_.agent_rows[agent], state_.agent_cols[agent]}) == 0) {
                continue;
            }
            const bool has_target =
                agent < static_cast<int>(targets.size()) &&
                targets[static_cast<std::size_t>(agent)].first >= 0;
            const auto [own_row, own_col] =
                has_target ? targets[static_cast<std::size_t>(agent)] : std::make_pair(-1, -1);
            const bool at_own_target =
                has_target && state_.agent_rows[agent] == own_row && state_.agent_cols[agent] == own_col;
            if (!has_target || !at_own_target ||
                agents_to_move.count(agent) != 0) {
                agents_to_move.insert(agent);
            }
        }
        if (forbidden.empty() || agents_to_move.empty()) {
            return false;
        }

        std::vector<int> ordered_agents(agents_to_move.begin(), agents_to_move.end());
        std::stable_sort(ordered_agents.begin(), ordered_agents.end(), [&](int lhs, int rhs) {
            const bool lhs_has_target =
                lhs < static_cast<int>(targets.size()) &&
                targets[static_cast<std::size_t>(lhs)].first >= 0;
            const bool rhs_has_target =
                rhs < static_cast<int>(targets.size()) &&
                targets[static_cast<std::size_t>(rhs)].first >= 0;
            if (lhs_has_target != rhs_has_target) {
                return !lhs_has_target;
            }
            return lhs < rhs;
        });

        bool moved_any = false;
        for (int agent : ordered_agents) {
            const bool has_target =
                agent < static_cast<int>(targets.size()) &&
                targets[static_cast<std::size_t>(agent)].first >= 0;
            if (forbidden.count({state_.agent_rows[agent], state_.agent_cols[agent]}) == 0) {
                continue;
            }
            bool moved = false;
            for (const auto& [target_row, target_col] : evacuation_targets(agent, forbidden)) {
                if (has_target) {
                    const auto [final_row, final_col] = targets[static_cast<std::size_t>(agent)];
                    if (topology_.distances_from(final_row, final_col)[target_row][target_col] == kInf) {
                        continue;
                    }
                }
                SingleBoxPlanner planner(state_, topology_);
                std::vector<int> moves = planner.plan_agent_to(agent, target_row, target_col, 20000);
                if (moves.empty()) {
                    continue;
                }
                if (append_serial(agent, moves)) {
                    std::cerr << "[enhanced] Evacuated final-goal blocker agent "
                              << agent << " to (" << target_row << "," << target_col << ").\n";
                    moved = true;
                    moved_any = true;
                    break;
                }
            }
            if (!moved) {
                std::cerr << "[enhanced] Final-goal blocker agent " << agent
                          << " could not reach a temporary evacuation cell.\n";
                return false;
            }
        }
        return moved_any;
    }

    std::vector<GoalBlocker> agent_goal_route_blockers(int agent,
                                                       int target_row,
                                                       int target_col)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::vector<GoalBlocker> blockers;
        std::deque<std::pair<int, int>> queue;
        std::vector<std::vector<int>> seen(static_cast<std::size_t>(level_.rows),
                                           std::vector<int>(static_cast<std::size_t>(level_.cols), -1));
        queue.push_back({state_.agent_rows[agent], state_.agent_cols[agent]});
        seen[state_.agent_rows[agent]][state_.agent_cols[agent]] = 0;
        const auto& to_target = topology_.distances_from(target_row, target_col);
        std::set<std::pair<int, int>> added;

        while (!queue.empty()) {
            const auto [row, col] = queue.front();
            queue.pop_front();
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols ||
                    level_.walls[nr][nc]) {
                    continue;
                }
                const char box = state_.boxes[nr][nc];
                if (is_box(box)) {
                    if (level_.goals[nr][nc] != box && added.insert({nr, nc}).second) {
                        const int target_distance = to_target[nr][nc] == kInf ? 10000 : to_target[nr][nc];
                        const int movable_penalty = state_.can_move_box(agent, box) ? 0 : 1000;
                        blockers.push_back({seen[row][col] * 4 + target_distance + movable_penalty,
                                            nr,
                                            nc,
                                            box});
                    }
                    continue;
                }
                if (state_.agent_at(nr, nc) != '\0' ||
                    seen[nr][nc] >= 0) {
                    continue;
                }
                seen[nr][nc] = seen[row][col] + 1;
                queue.push_back({nr, nc});
            }
        }

        std::sort(blockers.begin(), blockers.end(), [](const GoalBlocker& lhs,
                                                       const GoalBlocker& rhs) {
            if (lhs.score != rhs.score) {
                return lhs.score < rhs.score;
            }
            if (lhs.row != rhs.row) {
                return lhs.row < rhs.row;
            }
            return lhs.col < rhs.col;
        });
        return blockers;
    }

    bool agent_path_exists_with_blocks(int agent,
                                       int target_row,
                                       int target_col,
                                       bool block_boxes,
                                       bool block_agents,
                                       GoalDiagnostic& diagnostic)
    {
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        std::deque<std::pair<int, int>> queue;
        std::vector<std::vector<bool>> seen(static_cast<std::size_t>(level_.rows),
                                            std::vector<bool>(static_cast<std::size_t>(level_.cols), false));
        queue.push_back({state_.agent_rows[agent], state_.agent_cols[agent]});
        seen[state_.agent_rows[agent]][state_.agent_cols[agent]] = true;
        while (!queue.empty()) {
            const auto [row, col] = queue.front();
            queue.pop_front();
            if (row == target_row && col == target_col) {
                return true;
            }
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols ||
                    level_.walls[nr][nc] || seen[nr][nc]) {
                    continue;
                }
                if (block_boxes && is_box(state_.boxes[nr][nc])) {
                    if (diagnostic.blocker == '\0') {
                        diagnostic.blocker = state_.boxes[nr][nc];
                        diagnostic.row = nr;
                        diagnostic.col = nc;
                    }
                    continue;
                }
                const char occupying_agent = state_.agent_at(nr, nc);
                if (block_agents && is_agent(occupying_agent) && occupying_agent - '0' != agent) {
                    if (diagnostic.blocker == '\0') {
                        diagnostic.blocker = occupying_agent;
                        diagnostic.row = nr;
                        diagnostic.col = nc;
                    }
                    continue;
                }
                seen[nr][nc] = true;
                queue.push_back({nr, nc});
            }
        }
        return false;
    }

    GoalDiagnostic final_goal_diagnostic(int agent, int target_row, int target_col)
    {
        GoalDiagnostic diagnostic;
        const auto& walls_only = topology_.distances_from(target_row, target_col);
        if (walls_only[state_.agent_rows[agent]][state_.agent_cols[agent]] == kInf) {
            diagnostic.reason = "topology_unreachable";
            return diagnostic;
        }

        GoalDiagnostic box_diagnostic;
        if (!agent_path_exists_with_blocks(agent, target_row, target_col, true, false, box_diagnostic)) {
            box_diagnostic.reason = "blocked_by_box";
            return box_diagnostic;
        }

        GoalDiagnostic agent_diagnostic;
        if (!agent_path_exists_with_blocks(agent, target_row, target_col, true, true, agent_diagnostic)) {
            agent_diagnostic.reason = "blocked_by_agent";
            return agent_diagnostic;
        }

        diagnostic.reason = "reservation_or_timing_conflict";
        return diagnostic;
    }

    void log_final_goal_diagnostics(const std::vector<std::pair<int, int>>& targets)
    {
        for (int agent = 0; agent < static_cast<int>(targets.size()); ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            if (target_row < 0 ||
                (state_.agent_rows[agent] == target_row && state_.agent_cols[agent] == target_col)) {
                continue;
            }
            const GoalDiagnostic diagnostic = final_goal_diagnostic(agent, target_row, target_col);
            std::cerr << "[enhanced] Final-goal diagnostic: agent " << agent
                      << " target (" << target_row << "," << target_col << ") reason="
                      << diagnostic.reason;
            if (diagnostic.blocker != '\0') {
                std::cerr << " blocker=" << diagnostic.blocker
                          << " at (" << diagnostic.row << "," << diagnostic.col << ")";
            }
            std::cerr << ".\n";
        }
    }

    std::string final_agent_state_key(const State& state) const
    {
        std::string key;
        key.reserve(state.agent_rows.size() * 8);
        for (std::size_t agent = 0; agent < state.agent_rows.size(); ++agent) {
            key.append(std::to_string(state.agent_rows[agent]));
            key.push_back(',');
            key.append(std::to_string(state.agent_cols[agent]));
            key.push_back(';');
        }
        return key;
    }

    // BFS from goal cell with current boxes treated as walls.
    // Returns distance grid; kInf where unreachable.
    std::vector<std::vector<int>> box_aware_bfs(const State& state, int goal_row, int goal_col) const
    {
        std::vector<std::vector<int>> distances(level_.rows, std::vector<int>(level_.cols, kInf));
        if (goal_row < 0 || goal_row >= level_.rows ||
            goal_col < 0 || goal_col >= level_.cols ||
            level_.walls[goal_row][goal_col]) {
            return distances;
        }
        // The goal cell itself may be box-occupied; allow it as the destination
        // anyway so an agent currently standing on the goal has distance 0.
        std::deque<std::pair<int, int>> queue;
        distances[goal_row][goal_col] = 0;
        queue.push_back({goal_row, goal_col});
        static constexpr std::array<int, 4> dr{{-1, 1, 0, 0}};
        static constexpr std::array<int, 4> dc{{0, 0, 1, -1}};
        while (!queue.empty()) {
            const auto [row, col] = queue.front();
            queue.pop_front();
            for (int dir = 0; dir < 4; ++dir) {
                const int nr = row + dr[dir];
                const int nc = col + dc[dir];
                if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols) {
                    continue;
                }
                if (level_.walls[nr][nc]) {
                    continue;
                }
                if (state.boxes[nr][nc] != '\0') {
                    continue;
                }
                if (distances[nr][nc] != kInf) {
                    continue;
                }
                distances[nr][nc] = distances[row][col] + 1;
                queue.push_back({nr, nc});
            }
        }
        return distances;
    }

    // PIBT-based agent-only coordinator for the final-agent-goal phase.
    // Inspired by Okumura et al. 2019 ("Priority Inheritance with
    // Backtracking"). Boxes are treated as static obstacles (the typical
    // case at this phase: all box goals are already satisfied).
    //
    // Properties:
    //   * Transactional: builds the full joint plan in a local State copy
    //     and only commits via append_joint_sequence() when every agent
    //     with an assigned goal has reached it.
    //   * Box-aware: per-agent BFS distance maps treat current boxes as
    //     walls; if an agent's goal is unreachable under static boxes this
    //     routine returns false and the caller falls through to relocation.
    //   * Bounded: capped timesteps and joint-state visited-set to avoid
    //     livelock; priority aging gives stalled agents a chance to lead.
    bool complete_agent_goals_pibt()
    {
        if (enhanced_pibt_disabled()) {
            return false;
        }
        const int num_agents = static_cast<int>(state_.agent_rows.size());
        if (num_agents == 0 || time_exhausted()) {
            return false;
        }

        std::vector<std::pair<int, int>> targets = agent_goal_targets();
        std::vector<char> has_goal(static_cast<std::size_t>(num_agents), 0);
        bool any_off_goal = false;
        for (int agent = 0; agent < num_agents; ++agent) {
            const auto& t = targets[static_cast<std::size_t>(agent)];
            if (t.first < 0) {
                continue;
            }
            has_goal[static_cast<std::size_t>(agent)] = 1;
            if (state_.agent_rows[agent] != t.first ||
                state_.agent_cols[agent] != t.second) {
                any_off_goal = true;
            }
        }
        if (!any_off_goal) {
            return true;
        }

        // Per-agent box-aware distance maps from the agent's goal cell.
        std::vector<std::vector<std::vector<int>>> dist_to_goal(static_cast<std::size_t>(num_agents));
        for (int agent = 0; agent < num_agents; ++agent) {
            if (!has_goal[static_cast<std::size_t>(agent)]) {
                continue;
            }
            const auto& t = targets[static_cast<std::size_t>(agent)];
            dist_to_goal[static_cast<std::size_t>(agent)] = box_aware_bfs(state_, t.first, t.second);
            const int cur_dist =
                dist_to_goal[static_cast<std::size_t>(agent)][state_.agent_rows[agent]][state_.agent_cols[agent]];
            if (cur_dist == kInf) {
                // Goal unreachable with current boxes as walls; let the
                // box-aware relocation path handle this case.
                return false;
            }
        }

        // Local working state and accumulating joint plan.
        State local = state_;
        std::vector<std::vector<int>> joint_plan;

        // Priority: distance-to-goal plus aging. Higher priority moves first.
        std::vector<double> priority(static_cast<std::size_t>(num_agents), 0.0);
        for (int agent = 0; agent < num_agents; ++agent) {
            if (!has_goal[static_cast<std::size_t>(agent)]) {
                continue;
            }
            const int d = dist_to_goal[static_cast<std::size_t>(agent)][local.agent_rows[agent]][local.agent_cols[agent]];
            priority[static_cast<std::size_t>(agent)] = static_cast<double>(d == kInf ? 0 : d);
        }

        auto all_at_goal = [&](const State& s) {
            for (int agent = 0; agent < num_agents; ++agent) {
                if (!has_goal[static_cast<std::size_t>(agent)]) {
                    continue;
                }
                const auto& t = targets[static_cast<std::size_t>(agent)];
                if (s.agent_rows[agent] != t.first || s.agent_cols[agent] != t.second) {
                    return false;
                }
            }
            return true;
        };

        // Movement deltas for action indices 1..4 = N, S, E, W (NoOp = 0).
        static constexpr std::array<int, 5> kActionIndices{{0, 1, 2, 3, 4}};
        static constexpr std::array<int, 5> kAdr{{0, -1, 1, 0, 0}};
        static constexpr std::array<int, 5> kAdc{{0, 0, 0, 1, -1}};

        const int max_t = enhanced_pibt_max_timesteps();
        std::unordered_set<std::string> visited_joint;
        visited_joint.insert(final_agent_state_key(local));
        int repeat_streak = 0;

        for (int step = 0; step < max_t; ++step) {
            if ((step & 31) == 0 && time_exhausted()) {
                return false;
            }
            if (all_at_goal(local)) {
                break;
            }

            // Priority order this timestep: descending by priority value.
            std::vector<int> order(static_cast<std::size_t>(num_agents));
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
                return priority[static_cast<std::size_t>(a)] > priority[static_cast<std::size_t>(b)];
            });

            // PIBT bookkeeping for this timestep.
            std::vector<int> chosen_action(static_cast<std::size_t>(num_agents), -1);
            std::vector<std::pair<int, int>> next_cell(
                static_cast<std::size_t>(num_agents), {-1, -1});
            std::set<std::pair<int, int>> reserved;

            std::function<bool(int, int)> pibt_step = [&](int agent, int parent) -> bool {
                // Build candidate action list ranked by distance-to-goal of
                // the resulting cell. Includes NoOp.
                struct Cand {
                    int action;
                    int dist;
                    int next_r;
                    int next_c;
                };
                std::vector<Cand> cands;
                const int cur_r = local.agent_rows[agent];
                const int cur_c = local.agent_cols[agent];
                const bool has_target = has_goal[static_cast<std::size_t>(agent)] != 0;
                int target_r = -1;
                int target_c = -1;
                if (has_target) {
                    target_r = targets[static_cast<std::size_t>(agent)].first;
                    target_c = targets[static_cast<std::size_t>(agent)].second;
                }
                for (int ai : kActionIndices) {
                    const int nr = cur_r + kAdr[ai];
                    const int nc = cur_c + kAdc[ai];
                    if (nr < 0 || nr >= level_.rows || nc < 0 || nc >= level_.cols) {
                        continue;
                    }
                    if (level_.walls[nr][nc]) {
                        continue;
                    }
                    if (ai != kNoOpIndex && local.boxes[nr][nc] != '\0') {
                        continue;
                    }
                    int dist;
                    if (has_target) {
                        dist = dist_to_goal[static_cast<std::size_t>(agent)][nr][nc];
                    } else {
                        // Agents with no goal prefer NoOp, then any move
                        // away to clear corridors for goal-carrying peers.
                        dist = (ai == kNoOpIndex) ? 0 : 1;
                    }
                    if (dist == kInf) {
                        continue;
                    }
                    cands.push_back({ai, dist, nr, nc});
                }

                std::stable_sort(cands.begin(), cands.end(), [&](const Cand& a, const Cand& b) {
                    // Prefer cells with lower distance to goal. Among equals,
                    // prefer Move over NoOp so agents make progress rather
                    // than sitting on their goal blocking peers (unless they
                    // are already at goal and have no work to do).
                    if (a.dist != b.dist) {
                        return a.dist < b.dist;
                    }
                    const bool a_at_goal_noop =
                        has_target && a.action == kNoOpIndex &&
                        cur_r == target_r && cur_c == target_c;
                    const bool b_at_goal_noop =
                        has_target && b.action == kNoOpIndex &&
                        cur_r == target_r && cur_c == target_c;
                    if (a_at_goal_noop != b_at_goal_noop) {
                        return a_at_goal_noop;
                    }
                    // Otherwise prefer non-NoOp.
                    if ((a.action == kNoOpIndex) != (b.action == kNoOpIndex)) {
                        return a.action != kNoOpIndex;
                    }
                    return a.action < b.action;
                });

                for (const Cand& cand : cands) {
                    const std::pair<int, int> dest = {cand.next_r, cand.next_c};
                    // Vertex conflict with an agent that already chose this cell.
                    if (reserved.count(dest)) {
                        continue;
                    }
                    // Swap conflict: dest is the current cell of the parent
                    // (the agent that called us). Skip to avoid A<->B swap.
                    if (parent >= 0 &&
                        cand.next_r == local.agent_rows[parent] &&
                        cand.next_c == local.agent_cols[parent]) {
                        continue;
                    }
                    // Is dest currently occupied by another agent that has
                    // not yet chosen this timestep?
                    int occupant = -1;
                    for (int other = 0; other < num_agents; ++other) {
                        if (other == agent) continue;
                        if (chosen_action[static_cast<std::size_t>(other)] != -1) continue;
                        if (local.agent_rows[other] == cand.next_r &&
                            local.agent_cols[other] == cand.next_c) {
                            occupant = other;
                            break;
                        }
                    }
                    if (occupant != -1) {
                        // Tentatively reserve our move so the recursive call
                        // sees this cell as taken and won't try to enter it.
                        reserved.insert(dest);
                        chosen_action[static_cast<std::size_t>(agent)] = cand.action;
                        next_cell[static_cast<std::size_t>(agent)] = dest;
                        const bool ok = pibt_step(occupant, agent);
                        if (ok) {
                            return true;
                        }
                        // Roll back our tentative choice and try next candidate.
                        reserved.erase(dest);
                        chosen_action[static_cast<std::size_t>(agent)] = -1;
                        next_cell[static_cast<std::size_t>(agent)] = {-1, -1};
                        continue;
                    }
                    // Free destination.
                    reserved.insert(dest);
                    chosen_action[static_cast<std::size_t>(agent)] = cand.action;
                    next_cell[static_cast<std::size_t>(agent)] = dest;
                    return true;
                }
                // No valid candidate at all (we are surrounded with no
                // legal move and no NoOp possible - but NoOp on our own
                // cell is always legal). Force NoOp on our current cell.
                const std::pair<int, int> own = {cur_r, cur_c};
                if (reserved.count(own)) {
                    return false;
                }
                reserved.insert(own);
                chosen_action[static_cast<std::size_t>(agent)] = kNoOpIndex;
                next_cell[static_cast<std::size_t>(agent)] = own;
                return true;
            };

            for (int agent : order) {
                if (chosen_action[static_cast<std::size_t>(agent)] != -1) {
                    continue;
                }
                if (!pibt_step(agent, -1)) {
                    // Failed to schedule this agent; force NoOp.
                    chosen_action[static_cast<std::size_t>(agent)] = kNoOpIndex;
                    const std::pair<int, int> own = {
                        local.agent_rows[agent], local.agent_cols[agent]};
                    reserved.insert(own);
                    next_cell[static_cast<std::size_t>(agent)] = own;
                }
            }

            // Assemble joint action; validate via apply_joint to catch any
            // subtle swap/edge corner cases the simplified rules missed.
            std::vector<int> joint(static_cast<std::size_t>(num_agents), kNoOpIndex);
            for (int agent = 0; agent < num_agents; ++agent) {
                joint[static_cast<std::size_t>(agent)] =
                    chosen_action[static_cast<std::size_t>(agent)] == -1
                        ? kNoOpIndex
                        : chosen_action[static_cast<std::size_t>(agent)];
            }
            // Skip all-NoOp steps (no progress this tick).
            bool any_move = false;
            for (int a : joint) {
                if (a != kNoOpIndex) {
                    any_move = true;
                    break;
                }
            }
            if (!any_move) {
                // Bump priority of all off-goal agents to break ties next
                // round; if we still can't make progress, abort.
                bool any_off = false;
                for (int agent = 0; agent < num_agents; ++agent) {
                    if (!has_goal[static_cast<std::size_t>(agent)]) {
                        continue;
                    }
                    const auto& t = targets[static_cast<std::size_t>(agent)];
                    if (local.agent_rows[agent] != t.first ||
                        local.agent_cols[agent] != t.second) {
                        priority[static_cast<std::size_t>(agent)] += 1.0;
                        any_off = true;
                    }
                }
                if (!any_off) {
                    break;
                }
                ++repeat_streak;
                if (repeat_streak > 4) {
                    return false;
                }
                continue;
            }

            State next = local;
            if (!next.apply_joint(joint)) {
                return false;
            }

            const std::string key = final_agent_state_key(next);
            if (!visited_joint.insert(key).second) {
                ++repeat_streak;
                if (repeat_streak > 6) {
                    return false;
                }
                // Bump priority on stuck agents.
                for (int agent = 0; agent < num_agents; ++agent) {
                    if (!has_goal[static_cast<std::size_t>(agent)]) continue;
                    const auto& t = targets[static_cast<std::size_t>(agent)];
                    if (local.agent_rows[agent] != t.first ||
                        local.agent_cols[agent] != t.second) {
                        priority[static_cast<std::size_t>(agent)] += 1.5;
                    }
                }
            } else {
                repeat_streak = 0;
            }

            local = std::move(next);
            joint_plan.push_back(std::move(joint));

            // Update priorities: increment for off-goal, reset to 0 at goal.
            for (int agent = 0; agent < num_agents; ++agent) {
                if (!has_goal[static_cast<std::size_t>(agent)]) {
                    continue;
                }
                const auto& t = targets[static_cast<std::size_t>(agent)];
                if (local.agent_rows[agent] == t.first &&
                    local.agent_cols[agent] == t.second) {
                    priority[static_cast<std::size_t>(agent)] = 0.0;
                } else {
                    priority[static_cast<std::size_t>(agent)] += 1.0;
                }
            }
        }

        if (!all_at_goal(local)) {
            return false;
        }
        if (joint_plan.empty()) {
            return true;
        }
        if (!append_joint_sequence(joint_plan)) {
            return false;
        }
        std::cerr << "[enhanced] PIBT final-agent coordinator solved "
                  << num_agents << " agents in " << joint_plan.size() << " steps.\n";
        return true;
    }

    bool complete_agent_goals_joint_search()
    {
        const int num_agents = static_cast<int>(state_.agent_rows.size());
        if (num_agents == 0 || num_agents > 5 || time_exhausted()) {
            return false;
        }
        std::vector<std::pair<int, int>> targets = agent_goal_targets();
        for (int agent = 0; agent < num_agents; ++agent) {
            if (targets[static_cast<std::size_t>(agent)].first < 0) {
                targets[static_cast<std::size_t>(agent)] = {state_.agent_rows[agent], state_.agent_cols[agent]};
            }
        }

        int total_distance = 0;
        for (int agent = 0; agent < num_agents; ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            const int distance =
                topology_.distances_from(target_row, target_col)[state_.agent_rows[agent]][state_.agent_cols[agent]];
            if (distance == kInf) {
                return false;
            }
            total_distance += distance;
        }

        auto heuristic = [&](const State& state) {
            int sum = 0;
            for (int agent = 0; agent < num_agents; ++agent) {
                const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
                const int distance =
                    topology_.distances_from(target_row, target_col)[state.agent_rows[agent]][state.agent_cols[agent]];
                if (distance == kInf) {
                    return kInf;
                }
                sum += distance;
            }
            return sum;
        };

        const int root_h = heuristic(state_);
        if (root_h == 0 || root_h == kInf) {
            return root_h == 0;
        }

        struct Node {
            State state;
            int g = 0;
            int f = 0;
            int parent = -1;
            std::vector<int> joint;
        };

        std::vector<Node> nodes;
        nodes.reserve(2048);
        auto cmp = [&nodes](int lhs, int rhs) {
            if (nodes[lhs].f != nodes[rhs].f) {
                return nodes[lhs].f > nodes[rhs].f;
            }
            return nodes[lhs].g < nodes[rhs].g;
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
        std::unordered_map<std::string, int> best_g;

        nodes.push_back({state_, 0, root_h, -1, std::vector<int>(static_cast<std::size_t>(num_agents), kNoOpIndex)});
        open.push(0);
        best_g[final_agent_state_key(state_)] = 0;

        const int max_time = std::min(180, std::max(40, total_distance * 4 + num_agents * 12));
        const int expansion_cap = 30000;
        int expansions = 0;
        while (!open.empty() && expansions < expansion_cap) {
            if ((expansions & 255) == 0 && time_exhausted()) {
                return false;
            }
            const int index = open.top();
            open.pop();
            const Node current = nodes[static_cast<std::size_t>(index)];
            const std::string current_key = final_agent_state_key(current.state);
            const auto best_it = best_g.find(current_key);
            if (best_it != best_g.end() && best_it->second < current.g) {
                continue;
            }
            if (heuristic(current.state) == 0) {
                std::vector<std::vector<int>> joint_plan;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    joint_plan.push_back(nodes[static_cast<std::size_t>(cursor)].joint);
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(joint_plan.begin(), joint_plan.end());
                if (append_joint_sequence(joint_plan)) {
                    std::cerr << "[enhanced] Joint final-agent recovery solved "
                              << num_agents << " agents.\n";
                    return true;
                }
                return false;
            }
            if (current.g >= max_time) {
                continue;
            }
            ++expansions;

            std::vector<std::vector<int>> candidates_by_agent(static_cast<std::size_t>(num_agents));
            for (int agent = 0; agent < num_agents; ++agent) {
                std::array<int, 5> move_indices{{kNoOpIndex, 1, 2, 3, 4}};
                const auto target = targets[static_cast<std::size_t>(agent)];
                const int target_row = target.first;
                const int target_col = target.second;
                const auto& distances = topology_.distances_from(target_row, target_col);
                std::stable_sort(move_indices.begin(), move_indices.end(), [&](int lhs, int rhs) {
                    const int lhs_row = current.state.agent_rows[agent] + actions()[lhs].agent_dr;
                    const int lhs_col = current.state.agent_cols[agent] + actions()[lhs].agent_dc;
                    const int rhs_row = current.state.agent_rows[agent] + actions()[rhs].agent_dr;
                    const int rhs_col = current.state.agent_cols[agent] + actions()[rhs].agent_dc;
                    auto distance_at = [&](int row, int col) {
                        if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols) {
                            return kInf;
                        }
                        return distances[row][col];
                    };
                    if (lhs == kNoOpIndex &&
                        current.state.agent_rows[agent] == target_row &&
                        current.state.agent_cols[agent] == target_col) {
                        return true;
                    }
                    if (rhs == kNoOpIndex &&
                        current.state.agent_rows[agent] == target_row &&
                        current.state.agent_cols[agent] == target_col) {
                        return false;
                    }
                    return distance_at(lhs_row, lhs_col) < distance_at(rhs_row, rhs_col);
                });
                for (int action_index : move_indices) {
                    if (!current.state.applicable(agent, actions()[static_cast<std::size_t>(action_index)])) {
                        continue;
                    }
                    candidates_by_agent[static_cast<std::size_t>(agent)].push_back(action_index);
                    const std::size_t limit =
                        current.state.agent_rows[agent] == target_row &&
                                current.state.agent_cols[agent] == target_col
                            ? 3
                            : 4;
                    if (candidates_by_agent[static_cast<std::size_t>(agent)].size() >= limit) {
                        break;
                    }
                }
                if (candidates_by_agent[static_cast<std::size_t>(agent)].empty()) {
                    candidates_by_agent[static_cast<std::size_t>(agent)].push_back(kNoOpIndex);
                }
            }

            std::vector<int> joint(static_cast<std::size_t>(num_agents), kNoOpIndex);
            int generated_for_node = 0;
            const int max_generated_for_node = 320;
            std::function<void(int, bool)> enumerate = [&](int agent, bool any_non_noop) {
                if (generated_for_node >= max_generated_for_node) {
                    return;
                }
                if (agent == num_agents) {
                    if (!any_non_noop) {
                        return;
                    }
                    State child = current.state;
                    if (!child.apply_joint(joint)) {
                        return;
                    }
                    const int h = heuristic(child);
                    if (h == kInf) {
                        return;
                    }
                    const int next_g = current.g + 1;
                    const std::string key = final_agent_state_key(child);
                    const auto it = best_g.find(key);
                    if (it != best_g.end() && it->second <= next_g) {
                        return;
                    }
                    best_g[key] = next_g;
                    nodes.push_back({std::move(child), next_g, next_g + 2 * h, index, joint});
                    open.push(static_cast<int>(nodes.size()) - 1);
                    ++generated_for_node;
                    return;
                }
                for (int action_index : candidates_by_agent[static_cast<std::size_t>(agent)]) {
                    joint[static_cast<std::size_t>(agent)] = action_index;
                    enumerate(agent + 1, any_non_noop || action_index != kNoOpIndex);
                    if (generated_for_node >= max_generated_for_node) {
                        break;
                    }
                }
                joint[static_cast<std::size_t>(agent)] = kNoOpIndex;
            };
            enumerate(0, false);
        }
        return false;
    }

    bool complete_agent_goals_concurrent()
    {
        const int num_agents = static_cast<int>(state_.agent_rows.size());
        std::vector<std::pair<int, int>> targets(static_cast<std::size_t>(num_agents), {-1, -1});
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char goal = level_.goals[row][col];
                if (is_agent(goal)) {
                    const int agent = goal - '0';
                    if (agent < 0 || agent >= num_agents) {
                        return false;
                    }
                    targets[static_cast<std::size_t>(agent)] = {row, col};
                }
            }
        }

        std::vector<std::vector<int>> paths(static_cast<std::size_t>(num_agents));
        bool needs_move = false;
        for (int agent = 0; agent < num_agents; ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            if (target_row < 0) {
                continue;
            }
            if (state_.agent_rows[agent] == target_row && state_.agent_cols[agent] == target_col) {
                continue;
            }
            needs_move = true;
            SingleBoxPlanner planner(state_, topology_);
            paths[static_cast<std::size_t>(agent)] =
                planner.plan_agent_to(agent, target_row, target_col);
            if (paths[static_cast<std::size_t>(agent)].empty()) {
                return false;
            }
        }
        if (!needs_move) {
            return true;
        }

        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        if (complete_agent_goals_reserved(targets)) {
            return true;
        }
        state_ = saved_state;
        plan_.resize(saved_plan_size);

        std::vector<std::size_t> cursor(static_cast<std::size_t>(num_agents), 0);
        int total_remaining = 0;
        for (const auto& path : paths) {
            total_remaining += static_cast<int>(path.size());
        }
        const int max_steps = std::max(20, total_remaining * 4 + 20);
        for (int step = 0; step < max_steps; ++step) {
            bool all_done = true;
            for (int agent = 0; agent < num_agents; ++agent) {
                if (cursor[static_cast<std::size_t>(agent)] <
                    paths[static_cast<std::size_t>(agent)].size()) {
                    all_done = false;
                    break;
                }
            }
            if (all_done) {
                return true;
            }

            std::vector<int> joint(static_cast<std::size_t>(num_agents), kNoOpIndex);
            bool advanced = false;
            for (int agent = 0; agent < num_agents; ++agent) {
                auto& agent_cursor = cursor[static_cast<std::size_t>(agent)];
                const auto& path = paths[static_cast<std::size_t>(agent)];
                if (agent_cursor >= path.size()) {
                    continue;
                }
                const int action_index = path[agent_cursor];
                if (!state_.applicable(agent, actions()[action_index])) {
                    continue;
                }
                joint[static_cast<std::size_t>(agent)] = action_index;
                if (state_.conflicting(joint)) {
                    joint[static_cast<std::size_t>(agent)] = kNoOpIndex;
                    continue;
                }
                ++agent_cursor;
                advanced = true;
            }
            if (!advanced) {
                return false;
            }
            if (static_cast<int>(plan_.size()) >= max_joint_actions_ ||
                !state_.apply_joint(joint)) {
                return false;
            }
            plan_.push_back(std::move(joint));
        }
        return false;
    }

    long long cell_key(int row, int col) const
    {
        return static_cast<long long>(row) * level_.cols + col;
    }

    long long edge_key(int from_row, int from_col, int to_row, int to_col) const
    {
        const long long cells = static_cast<long long>(level_.rows) * level_.cols;
        return cell_key(from_row, from_col) * cells + cell_key(to_row, to_col);
    }

    bool reserved_target_safe(const std::vector<std::set<long long>>& reserved_vertices,
                              int row,
                              int col,
                              int time,
                              int max_time) const
    {
        const long long key = cell_key(row, col);
        for (int t = time; t <= max_time; ++t) {
            if (reserved_vertices[static_cast<std::size_t>(t)].count(key) != 0) {
                return false;
            }
        }
        return true;
    }

    std::optional<std::vector<int>> plan_agent_with_reservations(
        int agent,
        int target_row,
        int target_col,
        const std::vector<std::set<long long>>& reserved_vertices,
        const std::vector<std::set<long long>>& reserved_edges,
        int max_time)
    {
        struct Node {
            int row = -1;
            int col = -1;
            int time = 0;
            int parent = -1;
            int action = kNoOpIndex;
        };
        std::deque<int> queue;
        std::vector<Node> nodes;
        std::set<std::tuple<int, int, int>> seen;
        nodes.push_back({state_.agent_rows[agent], state_.agent_cols[agent], 0, -1, kNoOpIndex});
        queue.push_back(0);
        seen.insert({state_.agent_rows[agent], state_.agent_cols[agent], 0});

        while (!queue.empty()) {
            const int index = queue.front();
            queue.pop_front();
            const Node node = nodes[static_cast<std::size_t>(index)];
            if (node.row == target_row && node.col == target_col &&
                reserved_target_safe(reserved_vertices, node.row, node.col, node.time, max_time)) {
                std::vector<int> path;
                int cursor = index;
                while (nodes[static_cast<std::size_t>(cursor)].parent != -1) {
                    path.push_back(nodes[static_cast<std::size_t>(cursor)].action);
                    cursor = nodes[static_cast<std::size_t>(cursor)].parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            if (node.time >= max_time) {
                continue;
            }

            std::array<int, 5> move_indices{{kNoOpIndex, 1, 2, 3, 4}};
            const auto& distances = topology_.distances_from(target_row, target_col);
            std::stable_sort(move_indices.begin(), move_indices.end(), [&](int lhs, int rhs) {
                const int lhs_row = node.row + actions()[lhs].agent_dr;
                const int lhs_col = node.col + actions()[lhs].agent_dc;
                const int rhs_row = node.row + actions()[rhs].agent_dr;
                const int rhs_col = node.col + actions()[rhs].agent_dc;
                auto distance_at = [&](int row, int col) {
                    if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols) {
                        return kInf;
                    }
                    return distances[row][col];
                };
                return distance_at(lhs_row, lhs_col) < distance_at(rhs_row, rhs_col);
            });

            for (int action_index : move_indices) {
                const Action& action = actions()[static_cast<std::size_t>(action_index)];
                const int next_row = node.row + action.agent_dr;
                const int next_col = node.col + action.agent_dc;
                const int next_time = node.time + 1;
                if (next_row < 0 || next_row >= level_.rows ||
                    next_col < 0 || next_col >= level_.cols ||
                    level_.walls[next_row][next_col] ||
                    state_.boxes[next_row][next_col] != '\0') {
                    continue;
                }
                const long long next_cell = cell_key(next_row, next_col);
                if (reserved_vertices[static_cast<std::size_t>(next_time)].count(next_cell) != 0) {
                    continue;
                }
                const long long reverse_edge = edge_key(next_row, next_col, node.row, node.col);
                if (reserved_edges[static_cast<std::size_t>(next_time)].count(reverse_edge) != 0) {
                    continue;
                }
                const auto key = std::make_tuple(next_row, next_col, next_time);
                if (seen.count(key) != 0) {
                    continue;
                }
                seen.insert(key);
                nodes.push_back({next_row, next_col, next_time, index, action_index});
                queue.push_back(static_cast<int>(nodes.size()) - 1);
            }
        }
        return std::nullopt;
    }

    bool complete_agent_goals_reserved(const std::vector<std::pair<int, int>>& targets)
    {
        const int num_agents = static_cast<int>(state_.agent_rows.size());
        std::vector<int> order;
        int total_distance = 0;
        for (int agent = 0; agent < num_agents; ++agent) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            if (target_row < 0 ||
                (state_.agent_rows[agent] == target_row && state_.agent_cols[agent] == target_col)) {
                continue;
            }
            const int distance =
                topology_.distances_from(target_row, target_col)[state_.agent_rows[agent]][state_.agent_cols[agent]];
            if (distance == kInf) {
                return false;
            }
            total_distance += distance;
            order.push_back(agent);
        }
        if (order.empty()) {
            return true;
        }
        std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) {
            const auto [lhs_row, lhs_col] = targets[static_cast<std::size_t>(lhs)];
            const auto [rhs_row, rhs_col] = targets[static_cast<std::size_t>(rhs)];
            const int lhs_distance =
                topology_.distances_from(lhs_row, lhs_col)[state_.agent_rows[lhs]][state_.agent_cols[lhs]];
            const int rhs_distance =
                topology_.distances_from(rhs_row, rhs_col)[state_.agent_rows[rhs]][state_.agent_cols[rhs]];
            return lhs_distance > rhs_distance;
        });

        const int max_time = std::min(800, std::max(80, total_distance * 3 + num_agents * 10));
        std::vector<std::set<long long>> reserved_vertices(static_cast<std::size_t>(max_time + 1));
        std::vector<std::set<long long>> reserved_edges(static_cast<std::size_t>(max_time + 1));
        std::vector<std::vector<int>> paths(static_cast<std::size_t>(num_agents));
        std::set<int> moving_agents(order.begin(), order.end());
        for (int agent = 0; agent < num_agents; ++agent) {
            if (moving_agents.count(agent) != 0) {
                continue;
            }
            const long long key = cell_key(state_.agent_rows[agent], state_.agent_cols[agent]);
            for (int time = 0; time <= max_time; ++time) {
                reserved_vertices[static_cast<std::size_t>(time)].insert(key);
            }
        }

        for (int agent : order) {
            const auto [target_row, target_col] = targets[static_cast<std::size_t>(agent)];
            auto path = plan_agent_with_reservations(agent,
                                                     target_row,
                                                     target_col,
                                                     reserved_vertices,
                                                     reserved_edges,
                                                     max_time);
            if (!path.has_value()) {
                return false;
            }
            paths[static_cast<std::size_t>(agent)] = *path;

            int row = state_.agent_rows[agent];
            int col = state_.agent_cols[agent];
            reserved_vertices[0].insert(cell_key(row, col));
            for (int time = 1; time <= max_time; ++time) {
                const int action_index =
                    time <= static_cast<int>(path->size()) ? (*path)[static_cast<std::size_t>(time - 1)] : kNoOpIndex;
                const int next_row = row + actions()[static_cast<std::size_t>(action_index)].agent_dr;
                const int next_col = col + actions()[static_cast<std::size_t>(action_index)].agent_dc;
                reserved_vertices[static_cast<std::size_t>(time)].insert(cell_key(next_row, next_col));
                reserved_edges[static_cast<std::size_t>(time)].insert(edge_key(row, col, next_row, next_col));
                row = next_row;
                col = next_col;
            }
        }

        std::size_t makespan = 0;
        for (const auto& path : paths) {
            makespan = std::max(makespan, path.size());
        }
        for (std::size_t step = 0; step < makespan; ++step) {
            if (static_cast<int>(plan_.size()) >= max_joint_actions_) {
                return false;
            }
            std::vector<int> joint(static_cast<std::size_t>(num_agents), kNoOpIndex);
            for (int agent = 0; agent < num_agents; ++agent) {
                const auto& path = paths[static_cast<std::size_t>(agent)];
                if (step < path.size()) {
                    joint[static_cast<std::size_t>(agent)] = path[step];
                }
            }
            if (!state_.apply_joint(joint)) {
                return false;
            }
            plan_.push_back(std::move(joint));
        }
        return true;
    }

    bool complete_agent_goals_serial()
    {
        for (int row = 0; row < level_.rows; ++row) {
            for (int col = 0; col < level_.cols; ++col) {
                const char goal = level_.goals[row][col];
                if (!is_agent(goal)) {
                    continue;
                }
                const int agent = goal - '0';
                if (agent >= static_cast<int>(state_.agent_rows.size())) {
                    return false;
                }
                if (state_.agent_rows[agent] == row && state_.agent_cols[agent] == col) {
                    continue;
                }
                SingleBoxPlanner planner(state_, topology_);
                std::vector<int> moves = planner.plan_agent_to(agent, row, col);
                if (moves.empty()) {
                    return false;
                }
                if (!append_serial(agent, moves)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool append_serial(int active_agent, const std::vector<int>& active_actions)
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        for (int action_index : active_actions) {
            if (static_cast<int>(plan_.size()) >= max_joint_actions_) {
                state_ = saved_state;
                plan_.resize(saved_plan_size);
                return false;
            }
            std::vector<int> joint(state_.agent_rows.size(), kNoOpIndex);
            joint[active_agent] = action_index;
            if (!state_.apply_joint(joint)) {
                state_ = saved_state;
                plan_.resize(saved_plan_size);
                return false;
            }
            plan_.push_back(std::move(joint));
        }
        return true;
    }

    bool append_joint_sequence(const std::vector<std::vector<int>>& joint_actions)
    {
        const State saved_state = state_;
        const std::size_t saved_plan_size = plan_.size();
        for (const auto& joint : joint_actions) {
            if (joint.size() != state_.agent_rows.size() ||
                static_cast<int>(plan_.size()) >= max_joint_actions_ ||
                !state_.apply_joint(joint)) {
                state_ = saved_state;
                plan_.resize(saved_plan_size);
                return false;
            }
            plan_.push_back(joint);
        }
        return true;
    }

    const Level& level_;
    Topology topology_;
    TaskAllocator allocator_;
    State state_;
    State initial_state_;
    std::vector<std::vector<int>> plan_;
    std::array<int, 10> agent_work_{};
    std::set<int> forbidden_agents_{};
    int max_joint_actions_ = 20000;
    std::chrono::steady_clock::time_point solve_started_{};
    double max_solve_seconds_ = 25.0;
    const std::vector<BoxTask>* current_sequence_ = nullptr;
    std::size_t current_task_index_ = 0;
};

bool validate_plan(const Level& level, const std::vector<std::vector<int>>& plan)
{
    State state;
    state.level = &level;
    state.agent_rows = level.agent_rows;
    state.agent_cols = level.agent_cols;
    state.boxes = level.boxes;
    for (const auto& joint : plan) {
        if (!state.apply_joint(joint)) {
            return false;
        }
    }
    return state.goal_state();
}

}  // namespace enhanced

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    std::cerr << "Enhanced SearchClient initializing.\n";
    std::cout << "SearchClient\n";
    std::cout << "#This is a comment.\n";
    std::cout.flush();

    enhanced::Level level = enhanced::parse_level(std::cin);
    enhanced::HierarchicalSolver solver(level);
    auto plan = solver.solve();
    if (!plan.has_value() || !enhanced::validate_plan(level, *plan)) {
        std::cerr << "[enhanced] Unable to solve level.\n";
        return 0;
    }

    std::cerr << "[enhanced] Found solution of length " << plan->size() << ".\n";
    for (const auto& joint : *plan) {
        for (std::size_t i = 0; i < joint.size(); ++i) {
            if (i > 0) {
                std::cout << '|';
            }
            std::cout << enhanced::actions()[joint[i]].name;
        }
        std::cout << '\n' << std::flush;
        std::string response;
        if (!std::getline(std::cin, response)) {
            break;
        }
    }
    return 0;
}
