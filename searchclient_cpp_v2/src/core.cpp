#include "aimas/core.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <set>
#include <sstream>
#include <utility>

namespace aimas {

const std::vector<Action>& actions()
{
    static const std::vector<Action> kAll = {
        {"NoOp",        ActionType::NoOp,  0,  0,  0,  0},

        {"Move(N)",     ActionType::Move, -1,  0,  0,  0},
        {"Move(S)",     ActionType::Move,  1,  0,  0,  0},
        {"Move(E)",     ActionType::Move,  0,  1,  0,  0},
        {"Move(W)",     ActionType::Move,  0, -1,  0,  0},

        {"Push(N,N)",   ActionType::Push, -1,  0, -1,  0},
        {"Push(N,E)",   ActionType::Push, -1,  0,  0,  1},
        {"Push(N,W)",   ActionType::Push, -1,  0,  0, -1},
        {"Push(S,S)",   ActionType::Push,  1,  0,  1,  0},
        {"Push(S,E)",   ActionType::Push,  1,  0,  0,  1},
        {"Push(S,W)",   ActionType::Push,  1,  0,  0, -1},
        {"Push(E,E)",   ActionType::Push,  0,  1,  0,  1},
        {"Push(E,N)",   ActionType::Push,  0,  1, -1,  0},
        {"Push(E,S)",   ActionType::Push,  0,  1,  1,  0},
        {"Push(W,W)",   ActionType::Push,  0, -1,  0, -1},
        {"Push(W,N)",   ActionType::Push,  0, -1, -1,  0},
        {"Push(W,S)",   ActionType::Push,  0, -1,  1,  0},

        {"Pull(N,N)",   ActionType::Pull, -1,  0, -1,  0},
        {"Pull(N,E)",   ActionType::Pull, -1,  0,  0,  1},
        {"Pull(N,W)",   ActionType::Pull, -1,  0,  0, -1},
        {"Pull(S,S)",   ActionType::Pull,  1,  0,  1,  0},
        {"Pull(S,E)",   ActionType::Pull,  1,  0,  0,  1},
        {"Pull(S,W)",   ActionType::Pull,  1,  0,  0, -1},
        {"Pull(E,N)",   ActionType::Pull,  0,  1, -1,  0},
        {"Pull(E,S)",   ActionType::Pull,  0,  1,  1,  0},
        {"Pull(E,E)",   ActionType::Pull,  0,  1,  0,  1},
        {"Pull(W,N)",   ActionType::Pull,  0, -1, -1,  0},
        {"Pull(W,S)",   ActionType::Pull,  0, -1,  1,  0},
        {"Pull(W,W)",   ActionType::Pull,  0, -1,  0, -1},
    };
    return kAll;
}

State State::initial(const Level& level)
{
    State s;
    s.level       = &level;
    s.agent_rows  = level.agent_rows;
    s.agent_cols  = level.agent_cols;
    s.boxes       = level.boxes;
    return s;
}

char State::agent_at(int row, int col) const
{
    for (std::size_t i = 0; i < agent_rows.size(); ++i) {
        if (agent_rows[i] == row && agent_cols[i] == col) {
            return static_cast<char>('0' + static_cast<int>(i));
        }
    }
    return '\0';
}

bool State::in_bounds(int row, int col) const
{
    return level && 0 <= row && row < level->rows && 0 <= col && col < level->cols;
}

bool State::cell_free(int row, int col) const
{
    return in_bounds(row, col)
        && !level->walls[row][col]
        && boxes[row][col] == '\0'
        && agent_at(row, col) == '\0';
}

bool State::can_move_box(int agent, char box) const
{
    if (!is_box_char(box) || agent < 0
        || agent >= static_cast<int>(level->agent_color.size())) {
        return false;
    }
    return level->agent_color[agent] == level->box_color[box - 'A'];
}

bool State::applicable(int agent, const Action& action) const
{
    const int ar = agent_rows[agent];
    const int ac = agent_cols[agent];
    switch (action.type) {
        case ActionType::NoOp:
            return true;
        case ActionType::Move:
            return cell_free(ar + action.agent_dr, ac + action.agent_dc);
        case ActionType::Push: {
            const int br  = ar + action.agent_dr;
            const int bc  = ac + action.agent_dc;
            const int nbr = br + action.box_dr;
            const int nbc = bc + action.box_dc;
            return in_bounds(br, bc) && in_bounds(nbr, nbc)
                && is_box_char(boxes[br][bc])
                && can_move_box(agent, boxes[br][bc])
                && cell_free(nbr, nbc);
        }
        case ActionType::Pull: {
            const int br  = ar - action.box_dr;
            const int bc  = ac - action.box_dc;
            const int nar = ar + action.agent_dr;
            const int nac = ac + action.agent_dc;
            return in_bounds(br, bc) && in_bounds(nar, nac)
                && is_box_char(boxes[br][bc])
                && can_move_box(agent, boxes[br][bc])
                && cell_free(nar, nac);
        }
    }
    return false;
}

State::Delta State::delta_for(int agent, const Action& action) const
{
    Delta d;
    d.agent_from_r = agent_rows[agent];
    d.agent_from_c = agent_cols[agent];
    d.agent_to_r   = d.agent_from_r;
    d.agent_to_c   = d.agent_from_c;
    switch (action.type) {
        case ActionType::NoOp:
            break;
        case ActionType::Move:
            d.agent_to_r += action.agent_dr;
            d.agent_to_c += action.agent_dc;
            break;
        case ActionType::Push:
            d.box_from_r = d.agent_from_r + action.agent_dr;
            d.box_from_c = d.agent_from_c + action.agent_dc;
            d.box_to_r   = d.box_from_r + action.box_dr;
            d.box_to_c   = d.box_from_c + action.box_dc;
            d.agent_to_r = d.box_from_r;
            d.agent_to_c = d.box_from_c;
            d.moves_box  = true;
            break;
        case ActionType::Pull:
            d.box_from_r = d.agent_from_r - action.box_dr;
            d.box_from_c = d.agent_from_c - action.box_dc;
            d.box_to_r   = d.agent_from_r;
            d.box_to_c   = d.agent_from_c;
            d.agent_to_r += action.agent_dr;
            d.agent_to_c += action.agent_dc;
            d.moves_box   = true;
            break;
    }
    return d;
}

bool State::conflicting(const std::vector<int>& joint_action) const
{
    std::set<std::pair<int, int>> agent_destinations;
    std::set<std::pair<int, int>> box_destinations;
    std::set<std::pair<int, int>> occupied_destinations;
    std::vector<Delta> deltas;
    deltas.reserve(joint_action.size());

    const auto& tbl = actions();
    for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
        const Delta d = delta_for(agent, tbl[joint_action[agent]]);
        deltas.push_back(d);
        const auto agent_to = std::make_pair(d.agent_to_r, d.agent_to_c);
        if (!agent_destinations.insert(agent_to).second
            || !occupied_destinations.insert(agent_to).second) {
            return true;
        }
        if (d.moves_box) {
            const auto box_to = std::make_pair(d.box_to_r, d.box_to_c);
            if (!box_destinations.insert(box_to).second
                || !occupied_destinations.insert(box_to).second) {
                return true;
            }
        }
    }

    // Swap conflicts: agents/agents and boxes/boxes swapping positions.
    for (std::size_t i = 0; i < deltas.size(); ++i) {
        for (std::size_t j = i + 1; j < deltas.size(); ++j) {
            const Delta& a = deltas[i];
            const Delta& b = deltas[j];
            if (std::make_pair(a.agent_from_r, a.agent_from_c)
                    == std::make_pair(b.agent_to_r, b.agent_to_c)
                && std::make_pair(a.agent_to_r, a.agent_to_c)
                    == std::make_pair(b.agent_from_r, b.agent_from_c)) {
                return true;
            }
            if (a.moves_box && b.moves_box
                && std::make_pair(a.box_from_r, a.box_from_c)
                    == std::make_pair(b.box_to_r, b.box_to_c)
                && std::make_pair(a.box_to_r, a.box_to_c)
                    == std::make_pair(b.box_from_r, b.box_from_c)) {
                return true;
            }
        }
    }
    return false;
}

bool State::apply_joint(const std::vector<int>& joint_action)
{
    if (joint_action.size() != agent_rows.size()) {
        return false;
    }
    const auto& tbl = actions();
    for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
        if (!applicable(agent, tbl[joint_action[agent]])) {
            return false;
        }
    }
    if (conflicting(joint_action)) {
        return false;
    }
    std::vector<Delta> deltas;
    deltas.reserve(joint_action.size());
    for (int agent = 0; agent < static_cast<int>(joint_action.size()); ++agent) {
        deltas.push_back(delta_for(agent, tbl[joint_action[agent]]));
    }
    for (const Delta& d : deltas) {
        if (d.moves_box) {
            boxes[d.box_to_r][d.box_to_c]     = boxes[d.box_from_r][d.box_from_c];
            boxes[d.box_from_r][d.box_from_c] = '\0';
        }
    }
    for (int agent = 0; agent < static_cast<int>(deltas.size()); ++agent) {
        agent_rows[agent] = deltas[agent].agent_to_r;
        agent_cols[agent] = deltas[agent].agent_to_c;
    }
    return true;
}

bool State::goal_state() const
{
    for (int row = 0; row < level->rows; ++row) {
        for (int col = 0; col < level->cols; ++col) {
            const char goal = level->goals[row][col];
            if (is_box_char(goal) && boxes[row][col] != goal) {
                return false;
            }
            if (is_agent_char(goal)) {
                const int agent = goal - '0';
                if (agent >= static_cast<int>(agent_rows.size())
                    || agent_rows[agent] != row
                    || agent_cols[agent] != col) {
                    return false;
                }
            }
        }
    }
    return true;
}

std::size_t StateHash::operator()(const State& s) const noexcept
{
    std::size_t h = 1469598103934665603ull;  // FNV offset
    auto mix = [&](std::size_t x) {
        h ^= x;
        h *= 1099511628211ull;
    };
    for (std::size_t i = 0; i < s.agent_rows.size(); ++i) {
        mix(static_cast<std::size_t>(s.agent_rows[i]) * 131 + s.agent_cols[i]);
    }
    for (std::size_t r = 0; r < s.boxes.size(); ++r) {
        for (std::size_t c = 0; c < s.boxes[r].size(); ++c) {
            if (s.boxes[r][c] != '\0') {
                mix((r * 131 + c) * 27 + s.boxes[r][c]);
            }
        }
    }
    return h;
}

bool StateEq::operator()(const State& a, const State& b) const noexcept
{
    return a.agent_rows == b.agent_rows
        && a.agent_cols == b.agent_cols
        && a.boxes      == b.boxes;
}

std::string trim(const std::string& input)
{
    std::size_t start = 0;
    while (start < input.size()
           && std::isspace(static_cast<unsigned char>(input[start]))) {
        ++start;
    }
    std::size_t end = input.size();
    while (end > start
           && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(start, end - start);
}

std::vector<std::string> split(const std::string& input, char delim)
{
    std::vector<std::string> out;
    std::stringstream s(input);
    std::string item;
    while (std::getline(s, item, delim)) {
        out.push_back(trim(item));
    }
    return out;
}

std::string to_lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool env_flag_enabled(const char* name)
{
    const char* v = std::getenv(name);
    if (!v) return false;
    const std::string n = to_lower(trim(v));
    return n != "0" && n != "false" && n != "no" && n != "off";
}

}  // namespace aimas
