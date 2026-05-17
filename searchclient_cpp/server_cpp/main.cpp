#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

namespace fs = std::filesystem;

std::string trim(const std::string& input)
{
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
        ++begin;
    }
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(begin, end - begin);
}

std::vector<std::string> split(const std::string& input, char delimiter)
{
    std::vector<std::string> parts;
    std::stringstream stream(input);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

std::vector<std::string> split_whitespace(const std::string& input)
{
    std::vector<std::string> tokens;
    std::stringstream stream(input);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

<<<<<<< feature
bool is_header_line(const std::string& line)
{
    return !line.empty() && line[0] == '#';
}

void strip_trailing_carriage_return(std::string& line)
{
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
}

std::vector<std::string> trim_empty_boundary_lines(std::vector<std::string> lines)
{
    std::size_t begin = 0;
    while (begin < lines.size() && lines[begin].empty()) {
        ++begin;
    }
    std::size_t end = lines.size();
    while (end > begin && lines[end - 1].empty()) {
        --end;
    }
    std::vector<std::string> trimmed_lines;
    for (std::size_t i = begin; i < end; ++i) {
        trimmed_lines.push_back(std::move(lines[i]));
    }
    return trimmed_lines;
}

std::vector<std::string> section_lines(const std::vector<std::string>& lines,
                                       const std::string& header)
{
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (!is_header_line(lines[i]) || trim(lines[i]) != header) {
            continue;
        }

        std::vector<std::string> section;
        for (++i; i < lines.size() && !is_header_line(lines[i]); ++i) {
            section.push_back(lines[i]);
        }
        return trim_empty_boundary_lines(std::move(section));
    }
    return {};
}

=======
>>>>>>> main
enum class Color {
    Blue,
    Red,
    Cyan,
    Purple,
    Green,
    Orange,
    Pink,
    Grey,
    Lightblue,
    Brown,
    Unknown
};

Color parse_color(const std::string& name)
{
    const std::string lower = [&name]() {
        std::string value = name;
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return value;
    }();

    if (lower == "blue") return Color::Blue;
    if (lower == "red") return Color::Red;
    if (lower == "cyan") return Color::Cyan;
    if (lower == "purple") return Color::Purple;
    if (lower == "green") return Color::Green;
    if (lower == "orange") return Color::Orange;
    if (lower == "pink") return Color::Pink;
    if (lower == "grey") return Color::Grey;
    if (lower == "lightblue") return Color::Lightblue;
    if (lower == "brown") return Color::Brown;
    return Color::Unknown;
}

struct Action {
    std::string name;
    int agent_row_delta = 0;
    int agent_col_delta = 0;
    int box_row_delta = 0;
    int box_col_delta = 0;
    enum Kind { NoOp, Move, Push, Pull } kind = NoOp;
};

const std::unordered_map<std::string, Action>& action_table()
{
    static const std::unordered_map<std::string, Action> table = {
        {"NoOp", {"NoOp", 0, 0, 0, 0, Action::NoOp}},
        {"Move(N)", {"Move(N)", -1, 0, 0, 0, Action::Move}},
        {"Move(S)", {"Move(S)", 1, 0, 0, 0, Action::Move}},
        {"Move(E)", {"Move(E)", 0, 1, 0, 0, Action::Move}},
        {"Move(W)", {"Move(W)", 0, -1, 0, 0, Action::Move}},
        {"Push(N,N)", {"Push(N,N)", -1, 0, -1, 0, Action::Push}},
        {"Push(N,E)", {"Push(N,E)", -1, 0, 0, 1, Action::Push}},
        {"Push(N,W)", {"Push(N,W)", -1, 0, 0, -1, Action::Push}},
        {"Push(S,S)", {"Push(S,S)", 1, 0, 1, 0, Action::Push}},
        {"Push(S,E)", {"Push(S,E)", 1, 0, 0, 1, Action::Push}},
        {"Push(S,W)", {"Push(S,W)", 1, 0, 0, -1, Action::Push}},
        {"Push(E,E)", {"Push(E,E)", 0, 1, 0, 1, Action::Push}},
        {"Push(E,N)", {"Push(E,N)", 0, 1, -1, 0, Action::Push}},
        {"Push(E,S)", {"Push(E,S)", 0, 1, 1, 0, Action::Push}},
        {"Push(W,W)", {"Push(W,W)", 0, -1, 0, -1, Action::Push}},
        {"Push(W,N)", {"Push(W,N)", 0, -1, -1, 0, Action::Push}},
        {"Push(W,S)", {"Push(W,S)", 0, -1, 1, 0, Action::Push}},
        {"Pull(N,N)", {"Pull(N,N)", -1, 0, -1, 0, Action::Pull}},
        {"Pull(N,E)", {"Pull(N,E)", -1, 0, 0, 1, Action::Pull}},
        {"Pull(N,W)", {"Pull(N,W)", -1, 0, 0, -1, Action::Pull}},
        {"Pull(S,S)", {"Pull(S,S)", 1, 0, 1, 0, Action::Pull}},
        {"Pull(S,E)", {"Pull(S,E)", 1, 0, 0, 1, Action::Pull}},
        {"Pull(S,W)", {"Pull(S,W)", 1, 0, 0, -1, Action::Pull}},
        {"Pull(E,N)", {"Pull(E,N)", 0, 1, -1, 0, Action::Pull}},
        {"Pull(E,S)", {"Pull(E,S)", 0, 1, 1, 0, Action::Pull}},
        {"Pull(E,E)", {"Pull(E,E)", 0, 1, 0, 1, Action::Pull}},
        {"Pull(W,N)", {"Pull(W,N)", 0, -1, -1, 0, Action::Pull}},
        {"Pull(W,S)", {"Pull(W,S)", 0, -1, 1, 0, Action::Pull}},
        {"Pull(W,W)", {"Pull(W,W)", 0, -1, 0, -1, Action::Pull}},
    };
    return table;
}

struct State {
    std::vector<int> agent_rows;
    std::vector<int> agent_cols;
    std::vector<Color> agent_colors;
    std::vector<std::string> boxes;
    std::vector<Color> box_colors;
    std::vector<std::vector<bool>> walls;
    std::vector<std::string> goals;
};

struct ParsedLevel {
    std::string level_name;
    std::vector<std::string> raw_lines;
    State initial_state;
};

struct Playback {
    ParsedLevel level;
    std::vector<State> states;
};

struct JointActionResult {
    bool overall_success = false;
    std::vector<bool> per_agent_success;
    std::string reason;
};

ParsedLevel parse_level_file(const fs::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Could not open level file: " + path.string());
    }

    ParsedLevel parsed;
    std::string line;
    while (std::getline(input, line)) {
<<<<<<< feature
        strip_trailing_carriage_return(line);
        parsed.raw_lines.push_back(line);
    }

    const std::vector<std::string> domain_lines = section_lines(parsed.raw_lines, "#domain");
    if (domain_lines.empty() || trim(domain_lines.front()) != "hospital") {
        throw std::runtime_error("Unsupported domain in " + path.string());
    }

    const std::vector<std::string> level_name_lines = section_lines(parsed.raw_lines, "#levelname");
    if (level_name_lines.empty()) {
        throw std::runtime_error("Missing level name in " + path.string());
    }
    parsed.level_name = level_name_lines.front();

    const bool has_end = std::any_of(parsed.raw_lines.begin(), parsed.raw_lines.end(),
                                     [](const std::string& value) {
                                         return is_header_line(value) && trim(value) == "#end";
                                     });
    if (!has_end) {
        throw std::runtime_error("Missing #end in " + path.string());
    }

    std::vector<Color> agent_colors(10, Color::Unknown);
    std::vector<Color> box_colors(26, Color::Unknown);
    for (const std::string& color_line : section_lines(parsed.raw_lines, "#colors")) {
        if (color_line.empty()) {
            continue;
        }
        const auto parts = split(color_line, ':');
=======
        parsed.raw_lines.push_back(line);
    }

    std::size_t i = 0;
    auto expect_header = [&](const std::string& header) {
        if (i >= parsed.raw_lines.size() || parsed.raw_lines[i] != header) {
            throw std::runtime_error("Expected header " + header + " in " + path.string());
        }
        ++i;
    };

    expect_header("#domain");
    if (i >= parsed.raw_lines.size() || trim(parsed.raw_lines[i]) != "hospital") {
        throw std::runtime_error("Unsupported domain in " + path.string());
    }
    ++i;

    expect_header("#levelname");
    if (i >= parsed.raw_lines.size()) {
        throw std::runtime_error("Missing level name in " + path.string());
    }
    parsed.level_name = parsed.raw_lines[i++];

    expect_header("#colors");
    std::vector<Color> agent_colors(10, Color::Unknown);
    std::vector<Color> box_colors(26, Color::Unknown);
    while (i < parsed.raw_lines.size() && !parsed.raw_lines[i].empty() && parsed.raw_lines[i][0] != '#') {
        const auto parts = split(parsed.raw_lines[i], ':');
>>>>>>> main
        if (parts.size() == 2) {
            const Color color = parse_color(trim(parts[0]));
            for (const auto& entry : split(parts[1], ',')) {
                const std::string token = trim(entry);
                if (token.empty()) {
                    continue;
                }
                const char c = token[0];
                if ('0' <= c && c <= '9') {
                    agent_colors[c - '0'] = color;
                } else if ('A' <= c && c <= 'Z') {
                    box_colors[c - 'A'] = color;
                }
            }
        }
<<<<<<< feature
    }

    const std::vector<std::string> initial_lines = section_lines(parsed.raw_lines, "#initial");
    const std::vector<std::string> goal_lines = section_lines(parsed.raw_lines, "#goal");
    if (initial_lines.empty()) {
        throw std::runtime_error("No #initial grid rows found in " + path.string());
    }

    std::size_t max_cols = 0;
    for (const std::string& row : initial_lines) {
        max_cols = std::max(max_cols, row.size());
    }
    for (const std::string& row : goal_lines) {
        max_cols = std::max(max_cols, row.size());
    }
    if (max_cols == 0) {
        throw std::runtime_error("No grid columns found in " + path.string());
    }

    const int rows = static_cast<int>(std::max(initial_lines.size(), goal_lines.size()));
=======
        ++i;
    }

    expect_header("#initial");
    std::vector<std::string> initial_lines;
    std::size_t max_cols = 0;
    while (i < parsed.raw_lines.size() && !parsed.raw_lines[i].empty() && parsed.raw_lines[i][0] != '#') {
        initial_lines.push_back(parsed.raw_lines[i]);
        max_cols = std::max(max_cols, parsed.raw_lines[i].size());
        ++i;
    }

    const int rows = static_cast<int>(initial_lines.size());
>>>>>>> main
    const int cols = static_cast<int>(max_cols);
    std::vector<int> agent_rows(10, 0);
    std::vector<int> agent_cols(10, 0);
    int num_agents = 0;
<<<<<<< feature
    std::vector<std::vector<bool>> walls(rows, std::vector<bool>(cols, true));
    std::vector<std::string> boxes(rows, std::string(max_cols, '\0'));

    for (int row = 0; row < static_cast<int>(initial_lines.size()); ++row) {
        for (int col = 0; col < static_cast<int>(initial_lines[row].size()); ++col) {
            const char c = initial_lines[row][col];
            walls[row][col] = (c == '+');
=======
    std::vector<std::vector<bool>> walls(rows, std::vector<bool>(cols, false));
    std::vector<std::string> boxes(rows, std::string(max_cols, '\0'));

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < static_cast<int>(initial_lines[row].size()); ++col) {
            const char c = initial_lines[row][col];
>>>>>>> main
            if ('0' <= c && c <= '9') {
                agent_rows[c - '0'] = row;
                agent_cols[c - '0'] = col;
                num_agents = std::max(num_agents, c - '0' + 1);
            } else if ('A' <= c && c <= 'Z') {
                boxes[row][col] = c;
<<<<<<< feature
            }
        }
    }

    std::vector<std::string> goals(rows, std::string(max_cols, '\0'));
    for (int goal_row = 0; goal_row < static_cast<int>(goal_lines.size()); ++goal_row) {
        const std::string& goal_line = goal_lines[goal_row];
        for (int col = 0; col < static_cast<int>(goal_line.size()); ++col) {
            const char c = goal_line[col];
            if (('0' <= c && c <= '9') || ('A' <= c && c <= 'Z')) {
                if (walls[goal_row][col]) {
                    std::cerr << "[server][warning] Ignoring goal on wall/outside grid at row "
                              << goal_row << ", col " << col << " in " << path.string() << ".\n";
                    continue;
                }
                goals[goal_row][col] = c;
            }
        }
    }

=======
            } else if (c == '+') {
                walls[row][col] = true;
            }
        }
    }

    expect_header("#goal");
    std::vector<std::string> goals(rows, std::string(max_cols, '\0'));
    int goal_row = 0;
    while (i < parsed.raw_lines.size() && !parsed.raw_lines[i].empty() && parsed.raw_lines[i][0] != '#') {
        const std::string& goal_line = parsed.raw_lines[i];
        for (int col = 0; col < static_cast<int>(goal_line.size()); ++col) {
            const char c = goal_line[col];
            if (('0' <= c && c <= '9') || ('A' <= c && c <= 'Z')) {
                goals[goal_row][col] = c;
            }
        }
        ++goal_row;
        ++i;
    }

    expect_header("#end");

>>>>>>> main
    parsed.initial_state.agent_rows.assign(agent_rows.begin(), agent_rows.begin() + num_agents);
    parsed.initial_state.agent_cols.assign(agent_cols.begin(), agent_cols.begin() + num_agents);
    parsed.initial_state.agent_colors.assign(agent_colors.begin(), agent_colors.begin() + num_agents);
    parsed.initial_state.boxes = std::move(boxes);
    parsed.initial_state.box_colors = std::move(box_colors);
    parsed.initial_state.walls = std::move(walls);
    parsed.initial_state.goals = std::move(goals);
    return parsed;
}

bool cell_in_bounds(const State& state, int row, int col)
{
    return 0 <= row &&
           row < static_cast<int>(state.walls.size()) &&
           0 <= col &&
           col < static_cast<int>(state.walls[row].size());
}

char agent_at(const State& state, int row, int col)
{
    for (std::size_t i = 0; i < state.agent_rows.size(); ++i) {
        if (state.agent_rows[i] == row && state.agent_cols[i] == col) {
            return static_cast<char>('0' + static_cast<int>(i));
        }
    }
    return '\0';
}

bool cell_is_free(const State& state, int row, int col)
{
    return cell_in_bounds(state, row, col) &&
           !state.walls[row][col] &&
           state.boxes[row][col] == '\0' &&
           agent_at(state, row, col) == '\0';
}

bool is_goal_state(const State& state)
{
    for (std::size_t row = 0; row < state.goals.size(); ++row) {
        for (std::size_t col = 0; col < state.goals[row].size(); ++col) {
            const char goal = state.goals[row][col];
            if ('A' <= goal && goal <= 'Z' && state.boxes[row][col] != goal) {
                return false;
            }
            if ('0' <= goal && goal <= '9') {
                const int agent = goal - '0';
                if (agent >= static_cast<int>(state.agent_rows.size()) ||
                    state.agent_rows[agent] != static_cast<int>(row) ||
                    state.agent_cols[agent] != static_cast<int>(col)) {
                    return false;
                }
            }
        }
    }
    return true;
}

std::optional<Action> parse_action_token(const std::string& token)
{
    const auto& table = action_table();
    const auto it = table.find(trim(token));
    if (it == table.end()) {
        return std::nullopt;
    }
    return it->second;
}

JointActionResult validate_joint_action(const State& state, const std::vector<Action>& actions)
{
    JointActionResult result;
    result.per_agent_success.assign(actions.size(), false);

    std::vector<int> dest_rows(actions.size(), -1);
    std::vector<int> dest_cols(actions.size(), -1);
    std::vector<int> moved_box_rows(actions.size(), -1);
    std::vector<int> moved_box_cols(actions.size(), -1);

    for (std::size_t agent = 0; agent < actions.size(); ++agent) {
        const Action& action = actions[agent];
        const int row = state.agent_rows[agent];
        const int col = state.agent_cols[agent];

        if (action.kind == Action::NoOp) {
            result.per_agent_success[agent] = true;
            continue;
        }

        if (action.kind == Action::Move) {
            const int nr = row + action.agent_row_delta;
            const int nc = col + action.agent_col_delta;
            result.per_agent_success[agent] = cell_is_free(state, nr, nc);
            dest_rows[agent] = nr;
            dest_cols[agent] = nc;
            continue;
        }

        if (action.kind == Action::Push) {
            const int box_row = row + action.agent_row_delta;
            const int box_col = col + action.agent_col_delta;
            const int new_box_row = box_row + action.box_row_delta;
            const int new_box_col = box_col + action.box_col_delta;
            const char box = cell_in_bounds(state, box_row, box_col) ? state.boxes[box_row][box_col] : '\0';
            result.per_agent_success[agent] =
                box != '\0' &&
                state.agent_colors[agent] == state.box_colors[box - 'A'] &&
                cell_is_free(state, new_box_row, new_box_col);
            dest_rows[agent] = box_row;
            dest_cols[agent] = box_col;
            moved_box_rows[agent] = new_box_row;
            moved_box_cols[agent] = new_box_col;
            continue;
        }

        if (action.kind == Action::Pull) {
            const int nr = row + action.agent_row_delta;
            const int nc = col + action.agent_col_delta;
            const int box_row = row - action.box_row_delta;
            const int box_col = col - action.box_col_delta;
            const char box = cell_in_bounds(state, box_row, box_col) ? state.boxes[box_row][box_col] : '\0';
            result.per_agent_success[agent] =
                cell_is_free(state, nr, nc) &&
                box != '\0' &&
                state.agent_colors[agent] == state.box_colors[box - 'A'];
            dest_rows[agent] = nr;
            dest_cols[agent] = nc;
            moved_box_rows[agent] = row;
            moved_box_cols[agent] = col;
        }
    }

    for (std::size_t a1 = 0; a1 < actions.size(); ++a1) {
        if (!result.per_agent_success[a1] || actions[a1].kind == Action::NoOp) {
            continue;
        }
        for (std::size_t a2 = a1 + 1; a2 < actions.size(); ++a2) {
            if (!result.per_agent_success[a2] || actions[a2].kind == Action::NoOp) {
                continue;
            }

            const bool same_destination =
                dest_rows[a1] == dest_rows[a2] && dest_cols[a1] == dest_cols[a2];
            const bool same_box_destination =
                moved_box_rows[a1] != -1 &&
                moved_box_rows[a1] == moved_box_rows[a2] &&
                moved_box_cols[a1] == moved_box_cols[a2];
            const bool agent_into_box =
                moved_box_rows[a2] != -1 &&
                dest_rows[a1] == moved_box_rows[a2] &&
                dest_cols[a1] == moved_box_cols[a2];
            const bool box_into_agent =
                moved_box_rows[a1] != -1 &&
                dest_rows[a2] == moved_box_rows[a1] &&
                dest_cols[a2] == moved_box_cols[a1];

            if (same_destination || same_box_destination || agent_into_box || box_into_agent) {
                result.per_agent_success[a1] = false;
                result.per_agent_success[a2] = false;
            }
        }
    }

    result.overall_success =
        std::all_of(result.per_agent_success.begin(), result.per_agent_success.end(), [](bool ok) { return ok; });
    if (!result.overall_success) {
        result.reason = "One or more actions were invalid or conflicting.";
    }
    return result;
}

void apply_joint_action(State& state, const std::vector<Action>& actions)
{
    for (std::size_t agent = 0; agent < actions.size(); ++agent) {
        const Action& action = actions[agent];
        switch (action.kind) {
            case Action::NoOp:
                break;
            case Action::Move:
                state.agent_rows[agent] += action.agent_row_delta;
                state.agent_cols[agent] += action.agent_col_delta;
                break;
            case Action::Push: {
                const int box_row = state.agent_rows[agent] + action.agent_row_delta;
                const int box_col = state.agent_cols[agent] + action.agent_col_delta;
                const int new_box_row = box_row + action.box_row_delta;
                const int new_box_col = box_col + action.box_col_delta;
                state.boxes[new_box_row][new_box_col] = state.boxes[box_row][box_col];
                state.boxes[box_row][box_col] = '\0';
                state.agent_rows[agent] += action.agent_row_delta;
                state.agent_cols[agent] += action.agent_col_delta;
                break;
            }
            case Action::Pull: {
                const int box_row = state.agent_rows[agent] - action.box_row_delta;
                const int box_col = state.agent_cols[agent] - action.box_col_delta;
                state.boxes[state.agent_rows[agent]][state.agent_cols[agent]] = state.boxes[box_row][box_col];
                state.boxes[box_row][box_col] = '\0';
                state.agent_rows[agent] += action.agent_row_delta;
                state.agent_cols[agent] += action.agent_col_delta;
                break;
            }
        }
    }
}

class ChildProcess {
public:
    explicit ChildProcess(const std::string& command)
    {
        const std::vector<std::string> tokens = split_whitespace(command);
        if (tokens.empty()) {
            throw std::runtime_error("Empty client command.");
        }

#ifdef _WIN32
        SECURITY_ATTRIBUTES sa{};
        sa.nLength = sizeof(sa);
        sa.bInheritHandle = TRUE;

        HANDLE child_stdout_read = nullptr;
        HANDLE child_stdout_write = nullptr;
        HANDLE child_stdin_read = nullptr;
        HANDLE child_stdin_write = nullptr;

        if (!CreatePipe(&child_stdout_read, &child_stdout_write, &sa, 0) ||
            !CreatePipe(&child_stdin_read, &child_stdin_write, &sa, 0)) {
            throw std::runtime_error("Failed to create pipes for client process.");
        }

        SetHandleInformation(child_stdout_read, HANDLE_FLAG_INHERIT, 0);
        SetHandleInformation(child_stdin_write, HANDLE_FLAG_INHERIT, 0);

        STARTUPINFOA si{};
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = child_stdin_read;
        si.hStdOutput = child_stdout_write;
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);

        PROCESS_INFORMATION pi{};
        std::string command_line = command;
        if (!CreateProcessA(nullptr,
                            command_line.data(),
                            nullptr,
                            nullptr,
                            TRUE,
                            0,
                            nullptr,
                            nullptr,
                            &si,
                            &pi)) {
            CloseHandle(child_stdout_read);
            CloseHandle(child_stdout_write);
            CloseHandle(child_stdin_read);
            CloseHandle(child_stdin_write);
            throw std::runtime_error("Failed to launch client process.");
        }

        CloseHandle(child_stdout_write);
        CloseHandle(child_stdin_read);
        process_ = pi.hProcess;
        thread_ = pi.hThread;
        read_handle_ = child_stdout_read;
        write_handle_ = child_stdin_write;
#else
        int stdin_pipe[2];
        int stdout_pipe[2];
        if (pipe(stdin_pipe) != 0 || pipe(stdout_pipe) != 0) {
            throw std::runtime_error("Failed to create pipes for client process.");
        }

        pid_ = fork();
        if (pid_ < 0) {
            throw std::runtime_error("Failed to fork client process.");
        }

        if (pid_ == 0) {
            dup2(stdin_pipe[0], STDIN_FILENO);
            dup2(stdout_pipe[1], STDOUT_FILENO);
            dup2(STDERR_FILENO, STDERR_FILENO);
            close(stdin_pipe[0]);
            close(stdin_pipe[1]);
            close(stdout_pipe[0]);
            close(stdout_pipe[1]);

            std::vector<char*> argv;
            argv.reserve(tokens.size() + 1);
            for (const auto& token : tokens) {
                argv.push_back(const_cast<char*>(token.c_str()));
            }
            argv.push_back(nullptr);
            execvp(argv[0], argv.data());
            std::_Exit(127);
        }

        close(stdin_pipe[0]);
        close(stdout_pipe[1]);
        write_fd_ = stdin_pipe[1];
        read_fd_ = stdout_pipe[0];
#endif
    }

    ~ChildProcess()
    {
        terminate();
    }

    void write_line(const std::string& line)
    {
        write_raw(line + "\n");
    }

    void write_raw(const std::string& data)
    {
#ifdef _WIN32
        DWORD written = 0;
        if (!WriteFile(write_handle_, data.data(), static_cast<DWORD>(data.size()), &written, nullptr) ||
            written != data.size()) {
            throw std::runtime_error("Failed writing to client.");
        }
#else
        std::size_t offset = 0;
        while (offset < data.size()) {
            const ssize_t written = ::write(write_fd_, data.data() + offset, data.size() - offset);
            if (written <= 0) {
                throw std::runtime_error("Failed writing to client.");
            }
            offset += static_cast<std::size_t>(written);
        }
#endif
    }

    std::optional<std::string> read_line()
    {
        std::string line;
        char ch = '\0';
        while (true) {
#ifdef _WIN32
            DWORD read = 0;
            if (!ReadFile(read_handle_, &ch, 1, &read, nullptr) || read == 0) {
                return line.empty() ? std::nullopt : std::optional<std::string>(line);
            }
#else
            const ssize_t read = ::read(read_fd_, &ch, 1);
            if (read <= 0) {
                return line.empty() ? std::nullopt : std::optional<std::string>(line);
            }
#endif
            if (ch == '\n') {
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                return line;
            }
            line.push_back(ch);
        }
    }

    bool wait_for_exit(int timeout_seconds)
    {
#ifdef _WIN32
        const DWORD timeout_ms =
            timeout_seconds < 0 ? INFINITE : static_cast<DWORD>(timeout_seconds) * 1000U;
        return WaitForSingleObject(process_, timeout_ms) == WAIT_OBJECT_0;
#else
        if (timeout_seconds < 0) {
            int status = 0;
            return waitpid(pid_, &status, 0) == pid_;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);
        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            const pid_t result = waitpid(pid_, &status, WNOHANG);
            if (result == pid_) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
#endif
    }

    void terminate()
    {
#ifdef _WIN32
        if (write_handle_ != nullptr) {
            CloseHandle(write_handle_);
            write_handle_ = nullptr;
        }
        if (read_handle_ != nullptr) {
            CloseHandle(read_handle_);
            read_handle_ = nullptr;
        }
        if (process_ != nullptr) {
            DWORD exit_code = STILL_ACTIVE;
            GetExitCodeProcess(process_, &exit_code);
            if (exit_code == STILL_ACTIVE) {
                TerminateProcess(process_, 1);
            }
            CloseHandle(process_);
            process_ = nullptr;
        }
        if (thread_ != nullptr) {
            CloseHandle(thread_);
            thread_ = nullptr;
        }
#else
        if (write_fd_ != -1) {
            close(write_fd_);
            write_fd_ = -1;
        }
        if (read_fd_ != -1) {
            close(read_fd_);
            read_fd_ = -1;
        }
        if (pid_ > 0) {
            int status = 0;
            const pid_t waited = waitpid(pid_, &status, WNOHANG);
            if (waited == 0) {
                kill(pid_, SIGTERM);
                waitpid(pid_, nullptr, 0);
            }
            pid_ = -1;
        }
#endif
    }

private:
#ifdef _WIN32
    HANDLE process_ = nullptr;
    HANDLE thread_ = nullptr;
    HANDLE read_handle_ = nullptr;
    HANDLE write_handle_ = nullptr;
#else
    pid_t pid_ = -1;
    int read_fd_ = -1;
    int write_fd_ = -1;
#endif
};

struct Options {
    std::string client_command;
    fs::path level_path;
    int timeout_seconds = -1;
    bool gui = false;
    int speed_ms = 250;
};

void print_help()
{
    std::cout
        << "Headless C++ server for the hospital domain\n"
        << "Usage:\n"
        << "  server_cpp -c <client-cmd> -l <level-file-or-dir> [-t <seconds>] [-g] [-s <ms-per-action>]\n\n"
        << "Implemented in this version:\n"
        << "  -c  Launch client command\n"
        << "  -l  Single .lvl file or directory of .lvl files\n"
        << "  -t  Optional timeout in seconds\n"
        << "  -g  Open a GUI playback window after the run (Windows only)\n"
        << "  -s  GUI playback speed in milliseconds per action\n";
}

Options parse_args(int argc, char* argv[])
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_help();
            std::exit(0);
        }
        if (arg == "-c" && i + 1 < argc) {
            options.client_command = argv[++i];
            continue;
        }
        if (arg == "-l" && i + 1 < argc) {
            options.level_path = argv[++i];
            continue;
        }
        if (arg == "-t" && i + 1 < argc) {
            options.timeout_seconds = std::stoi(argv[++i]);
            continue;
        }
        if (arg == "-g") {
            options.gui = true;
            continue;
        }
        if (arg == "-s" && i + 1 < argc) {
            options.speed_ms = std::stoi(argv[++i]);
            continue;
        }
        throw std::runtime_error("Unsupported or malformed argument: " + arg);
    }

    if (options.client_command.empty()) {
        throw std::runtime_error("Missing required -c <client-cmd> argument.");
    }
    if (options.level_path.empty()) {
        throw std::runtime_error("Missing required -l <level-file-or-dir> argument.");
    }
    if (options.speed_ms <= 0) {
        throw std::runtime_error("Playback speed must be a positive integer.");
    }
    return options;
}

std::string make_server_response(const std::vector<bool>& per_agent_success)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < per_agent_success.size(); ++i) {
        if (i > 0) {
            out << '|';
        }
        out << (per_agent_success[i] ? "true" : "false");
    }
    return out.str();
}

std::vector<Action> parse_joint_action_line(const std::string& line, int expected_agents)
{
    std::vector<Action> actions;
    const auto parts = split(line, '|');
    if (static_cast<int>(parts.size()) != expected_agents) {
        throw std::runtime_error("Expected " + std::to_string(expected_agents) +
                                 " joint actions but got " + std::to_string(parts.size()) + ".");
    }

    for (const auto& part : parts) {
        auto action = parse_action_token(trim(part));
        if (!action.has_value()) {
            throw std::runtime_error("Unknown action token: " + trim(part));
        }
        actions.push_back(*action);
    }
    return actions;
}

void send_level_to_client(ChildProcess& client, const ParsedLevel& level)
{
    for (const auto& line : level.raw_lines) {
        client.write_line(line);
    }
}

Playback run_single_level(const Options& options, const fs::path& level_path)
{
    ParsedLevel level = parse_level_file(level_path);
    State state = level.initial_state;
    Playback playback{level, {state}};

    std::cout << "[server][info] Running level: " << level_path.string() << '\n';
    ChildProcess client(options.client_command);

    const auto start = std::chrono::steady_clock::now();
    auto expired = [&]() {
        if (options.timeout_seconds < 0) {
            return false;
        }
        return std::chrono::steady_clock::now() - start > std::chrono::seconds(options.timeout_seconds);
    };

    const std::optional<std::string> client_name = client.read_line();
    if (!client_name.has_value()) {
        throw std::runtime_error("Client closed output before sending its name.");
    }
    std::cout << "[server][info] Client name: " << *client_name << '\n';

    send_level_to_client(client, level);

    std::size_t steps = 0;
    while (!expired()) {
        const std::optional<std::string> line = client.read_line();
        if (!line.has_value()) {
            break;
        }

        const std::string trimmed = trim(*line);
        if (trimmed.empty()) {
            continue;
        }
        if (!trimmed.empty() && trimmed[0] == '#') {
            continue;
        }

        ++steps;
        std::vector<bool> success(state.agent_rows.size(), false);
        try {
            const std::vector<Action> joint_action =
                parse_joint_action_line(trimmed, static_cast<int>(state.agent_rows.size()));
            const JointActionResult validation = validate_joint_action(state, joint_action);
            success = validation.per_agent_success;
            if (validation.overall_success) {
                apply_joint_action(state, joint_action);
                playback.states.push_back(state);
            }
        } catch (const std::exception& error) {
            std::cerr << "[server][warning] " << error.what() << '\n';
        }

        client.write_line(make_server_response(success));

        if (is_goal_state(state)) {
            std::cout << "[server][info] Solved in " << steps << " steps.\n";
            return playback;
        }
    }

    if (expired()) {
        std::cerr << "[server][error] Client timed out.\n";
        client.terminate();
        return playback;
    }

    if (is_goal_state(state)) {
        std::cout << "[server][info] Solved in " << steps << " steps.\n";
    } else {
        std::cout << "[server][info] Client terminated without solving the level.\n";
    }
    return playback;
}

std::vector<fs::path> collect_levels(const fs::path& level_path)
{
    std::vector<fs::path> levels;
    if (fs::is_regular_file(level_path)) {
        levels.push_back(level_path);
        return levels;
    }
    if (!fs::is_directory(level_path)) {
        throw std::runtime_error("Level path is neither a file nor a directory: " + level_path.string());
    }

    for (const auto& entry : fs::directory_iterator(level_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".lvl") {
            levels.push_back(entry.path());
        }
    }
    std::sort(levels.begin(), levels.end());
    return levels;
}

<<<<<<< feature
#ifdef _WIN32
=======
>>>>>>> main
COLORREF color_to_rgb(Color color)
{
    switch (color) {
        case Color::Blue: return RGB(59, 130, 246);
        case Color::Red: return RGB(239, 68, 68);
        case Color::Cyan: return RGB(6, 182, 212);
        case Color::Purple: return RGB(147, 51, 234);
        case Color::Green: return RGB(34, 197, 94);
        case Color::Orange: return RGB(249, 115, 22);
        case Color::Pink: return RGB(236, 72, 153);
        case Color::Grey: return RGB(107, 114, 128);
        case Color::Lightblue: return RGB(125, 211, 252);
        case Color::Brown: return RGB(120, 73, 40);
        case Color::Unknown: return RGB(156, 163, 175);
    }
    return RGB(156, 163, 175);
}

<<<<<<< feature
=======
#ifdef _WIN32
>>>>>>> main
struct GuiPlaybackContext {
    Playback playback;
    int speed_ms = 250;
    std::size_t frame_index = 0;
    bool playing = true;
};

LRESULT CALLBACK gui_window_proc(HWND hwnd, UINT message, WPARAM w_param, LPARAM l_param)
{
    auto* context = reinterpret_cast<GuiPlaybackContext*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));

    switch (message) {
        case WM_CREATE: {
            auto* create_struct = reinterpret_cast<CREATESTRUCT*>(l_param);
            SetWindowLongPtr(hwnd,
                             GWLP_USERDATA,
                             reinterpret_cast<LONG_PTR>(create_struct->lpCreateParams));
            return 0;
        }
        case WM_TIMER:
            if (context != nullptr && context->playing && context->frame_index + 1 < context->playback.states.size()) {
                ++context->frame_index;
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            return 0;
        case WM_KEYDOWN:
            if (context == nullptr) {
                return 0;
            }
            if (w_param == VK_SPACE) {
                context->playing = !context->playing;
                return 0;
            }
            if (w_param == VK_RIGHT && context->frame_index + 1 < context->playback.states.size()) {
                ++context->frame_index;
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            if (w_param == VK_LEFT && context->frame_index > 0) {
                --context->frame_index;
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            if (w_param == VK_HOME) {
                context->frame_index = 0;
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            if (w_param == VK_END && !context->playback.states.empty()) {
                context->frame_index = context->playback.states.size() - 1;
                InvalidateRect(hwnd, nullptr, FALSE);
                return 0;
            }
            return 0;
        case WM_PAINT: {
            if (context == nullptr || context->playback.states.empty()) {
                return 0;
            }

            PAINTSTRUCT ps{};
            HDC hdc = BeginPaint(hwnd, &ps);

            RECT client_rect{};
            GetClientRect(hwnd, &client_rect);
            FillRect(hdc, &client_rect, reinterpret_cast<HBRUSH>(GetStockObject(WHITE_BRUSH)));

            const State& state = context->playback.states[context->frame_index];
            const int rows = static_cast<int>(state.walls.size());
            const int cols = rows > 0 ? static_cast<int>(state.walls[0].size()) : 0;
            const int header_height = 48;
            const int available_width = std::max(1L, client_rect.right - client_rect.left - 20L);
            const int available_height = std::max(1L, client_rect.bottom - client_rect.top - header_height - 20L);
            const int cell_size = std::max(16, static_cast<int>(std::min(available_width / std::max(cols, 1),
                                                                         available_height / std::max(rows, 1))));
            const int offset_x = 10;
            const int offset_y = header_height;

            SetBkMode(hdc, TRANSPARENT);
            std::ostringstream title;
            title << context->playback.level.level_name
                  << "  frame " << context->frame_index << "/" << (context->playback.states.size() - 1)
                  << "  [" << (context->playing ? "playing" : "paused") << "]";
            const std::string title_text = title.str();
            TextOutA(hdc, 10, 10, title_text.c_str(), static_cast<int>(title_text.size()));

            const std::string help = "Space: play/pause   Left/Right: step   Home/End: jump";
            TextOutA(hdc, 10, 28, help.c_str(), static_cast<int>(help.size()));

            for (int row = 0; row < rows; ++row) {
                for (int col = 0; col < cols; ++col) {
                    RECT cell{
                        offset_x + col * cell_size,
                        offset_y + row * cell_size,
                        offset_x + (col + 1) * cell_size,
                        offset_y + (row + 1) * cell_size
                    };

                    HBRUSH base_brush = CreateSolidBrush(RGB(243, 244, 246));
                    FillRect(hdc, &cell, base_brush);
                    DeleteObject(base_brush);

                    if (state.walls[row][col]) {
                        HBRUSH wall_brush = CreateSolidBrush(RGB(31, 41, 55));
                        FillRect(hdc, &cell, wall_brush);
                        DeleteObject(wall_brush);
                    } else {
                        const char goal = state.goals[row][col];
                        if (goal != '\0') {
                            HBRUSH goal_brush = CreateSolidBrush(RGB(254, 243, 199));
                            FillRect(hdc, &cell, goal_brush);
                            DeleteObject(goal_brush);
                        }
                    }

                    FrameRect(hdc, &cell, reinterpret_cast<HBRUSH>(GetStockObject(GRAY_BRUSH)));

                    const char box = state.boxes[row][col];
                    if (box != '\0') {
                        RECT inner{
                            cell.left + 4,
                            cell.top + 4,
                            cell.right - 4,
                            cell.bottom - 4
                        };
                        HBRUSH box_brush = CreateSolidBrush(color_to_rgb(state.box_colors[box - 'A']));
                        FillRect(hdc, &inner, box_brush);
                        DeleteObject(box_brush);

                        const std::string label(1, box);
                        SetTextColor(hdc, RGB(255, 255, 255));
                        DrawTextA(hdc, label.c_str(), 1, &inner, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
                    }

                    const char agent = agent_at(state, row, col);
                    if (agent != '\0') {
                        HBRUSH agent_brush = CreateSolidBrush(color_to_rgb(state.agent_colors[agent - '0']));
                        HBRUSH old_brush = reinterpret_cast<HBRUSH>(SelectObject(hdc, agent_brush));
                        HPEN pen = CreatePen(PS_SOLID, 1, RGB(17, 24, 39));
                        HPEN old_pen = reinterpret_cast<HPEN>(SelectObject(hdc, pen));
                        Ellipse(hdc, cell.left + 6, cell.top + 6, cell.right - 6, cell.bottom - 6);
                        SelectObject(hdc, old_pen);
                        DeleteObject(pen);
                        SelectObject(hdc, old_brush);
                        DeleteObject(agent_brush);

                        RECT inner{
                            cell.left + 6,
                            cell.top + 6,
                            cell.right - 6,
                            cell.bottom - 6
                        };
                        const std::string label(1, agent);
                        SetTextColor(hdc, RGB(255, 255, 255));
                        DrawTextA(hdc, label.c_str(), 1, &inner, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
                    } else if (state.goals[row][col] != '\0') {
                        const std::string label(1, state.goals[row][col]);
                        SetTextColor(hdc, RGB(146, 64, 14));
                        DrawTextA(hdc, label.c_str(), 1, &cell, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
                    }
                }
            }

            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProc(hwnd, message, w_param, l_param);
    }
}

void show_gui_playback(const Playback& playback, int speed_ms)
{
    GuiPlaybackContext context{playback, speed_ms, 0, true};

    const char* class_name = "SearchClientCppPlaybackWindow";
    WNDCLASSA window_class{};
    window_class.lpfnWndProc = gui_window_proc;
    window_class.hInstance = GetModuleHandle(nullptr);
    window_class.lpszClassName = class_name;
    window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
    window_class.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    RegisterClassA(&window_class);

    const int rows = static_cast<int>(playback.level.initial_state.walls.size());
    const int cols = rows > 0 ? static_cast<int>(playback.level.initial_state.walls[0].size()) : 0;
    const int cell_size = 40;
    const int width = std::max(640, cols * cell_size + 40);
    const int height = std::max(480, rows * cell_size + 100);

    HWND hwnd = CreateWindowExA(0,
                                class_name,
                                playback.level.level_name.c_str(),
                                WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                                CW_USEDEFAULT,
                                CW_USEDEFAULT,
                                width,
                                height,
                                nullptr,
                                nullptr,
                                GetModuleHandle(nullptr),
                                &context);

    if (hwnd == nullptr) {
        throw std::runtime_error("Failed to create GUI window.");
    }

    SetTimer(hwnd, 1, static_cast<UINT>(speed_ms), nullptr);
    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);

    MSG msg{};
    while (GetMessage(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}
#else
void show_gui_playback(const Playback&, int)
{
    throw std::runtime_error("GUI playback is currently implemented only on Windows.");
}
#endif

} // namespace

int main(int argc, char* argv[])
{
    try {
        const Options options = parse_args(argc, argv);
        const std::vector<fs::path> levels = collect_levels(options.level_path);
        if (levels.empty()) {
            throw std::runtime_error("No .lvl files found at " + options.level_path.string());
        }

        for (const auto& level : levels) {
            Playback playback = run_single_level(options, level);
            if (options.gui) {
                show_gui_playback(playback, options.speed_ms);
            }
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[server][error] " << error.what() << '\n';
        return 1;
    }
}
