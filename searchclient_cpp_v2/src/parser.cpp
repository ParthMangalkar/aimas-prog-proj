#include "aimas/parser.hpp"

#include <istream>
#include <map>
#include <stdexcept>
#include <string>

namespace aimas {

Level parse_level(std::istream& input)
{
    Level level;
    level.box_color.fill(-1);

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
        if (trim(line) == "#end") break;
    }

    auto find_section = [&](const std::string& section) -> std::size_t {
        for (std::size_t i = 0; i < lines.size(); ++i) {
            if (trim(lines[i]) == section) return i;
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
        if (it != color_ids.end()) return it->second;
        const int id = static_cast<int>(color_ids.size());
        color_ids[key] = id;
        return id;
    };

    std::array<int, 10> agent_color_tmp{};
    agent_color_tmp.fill(-1);
    const std::size_t colors = find_section("#colors");
    for (std::size_t i = colors + 1;
         i < lines.size() && !trim(lines[i]).empty() && trim(lines[i])[0] != '#';
         ++i) {
        const std::string s = trim(lines[i]);
        const std::size_t colon = s.find(':');
        if (colon == std::string::npos) continue;
        const int id = color_id(s.substr(0, colon));
        for (const std::string& tok : split(s.substr(colon + 1), ',')) {
            if (tok.empty()) continue;
            const char e = tok[0];
            if (is_agent_char(e))       agent_color_tmp[e - '0']      = id;
            else if (is_box_char(e))    level.box_color[e - 'A']      = id;
        }
    }

    std::vector<std::string> initial_lines;
    const std::size_t initial = find_section("#initial");
    for (std::size_t i = initial + 1;
         i < lines.size() && (trim(lines[i]).empty() || trim(lines[i])[0] != '#');
         ++i) {
        if (!trim(lines[i]).empty()) {
            initial_lines.push_back(lines[i]);
            level.cols = std::max(level.cols, static_cast<int>(lines[i].size()));
        }
    }
    level.rows = static_cast<int>(initial_lines.size());
    if (level.rows == 0 || level.cols == 0) {
        throw std::runtime_error("parse_level: empty initial grid");
    }
    level.walls.assign(level.rows, std::vector<bool>(level.cols, false));
    level.boxes.assign(level.rows,
                       std::string(static_cast<std::size_t>(level.cols), '\0'));

    int max_agent = -1;
    std::array<int, 10> agent_rows_tmp{};
    std::array<int, 10> agent_cols_tmp{};
    agent_rows_tmp.fill(0);
    agent_cols_tmp.fill(0);
    for (int row = 0; row < level.rows; ++row) {
        const std::string& src = initial_lines[row];
        for (int col = 0;
             col < static_cast<int>(src.size()) && col < level.cols; ++col) {
            const char ch = src[col];
            if (ch == '+') {
                level.walls[row][col] = true;
            } else if (is_box_char(ch)) {
                level.boxes[row][col] = ch;
            } else if (is_agent_char(ch)) {
                const int agent = ch - '0';
                if (agent > max_agent) max_agent = agent;
                agent_rows_tmp[agent] = row;
                agent_cols_tmp[agent] = col;
            }
        }
    }

    level.agent_rows.resize(static_cast<std::size_t>(max_agent + 1));
    level.agent_cols.resize(static_cast<std::size_t>(max_agent + 1));
    level.agent_color.resize(static_cast<std::size_t>(max_agent + 1), -1);
    for (int agent = 0; agent <= max_agent; ++agent) {
        level.agent_rows[agent]  = agent_rows_tmp[agent];
        level.agent_cols[agent]  = agent_cols_tmp[agent];
        level.agent_color[agent] = agent_color_tmp[agent];
    }

    level.goals.assign(level.rows,
                       std::string(static_cast<std::size_t>(level.cols), '\0'));
    const std::size_t goals = find_section("#goal");
    int goal_row = 0;
    for (std::size_t i = goals + 1;
         i < lines.size() && goal_row < level.rows
             && (trim(lines[i]).empty() || trim(lines[i])[0] != '#');
         ++i) {
        if (trim(lines[i]).empty()) continue;
        for (int col = 0;
             col < static_cast<int>(lines[i].size()) && col < level.cols; ++col) {
            const char ch = lines[i][col];
            if (is_box_char(ch) || is_agent_char(ch)) {
                level.goals[goal_row][col] = ch;
            }
        }
        ++goal_row;
    }

    // Fall back to first known agent color for any box letter whose color was
    // not declared explicitly. Matches legacy parser; the solver later treats
    // such boxes as not deliverable if no matching agent exists.
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

}  // namespace aimas
