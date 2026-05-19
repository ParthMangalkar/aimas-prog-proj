#include "aimas/single_box.hpp"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <queue>
#include <unordered_map>

namespace aimas {

namespace {

struct Node {
    int ar = 0, ac = 0;
    int br = 0, bc = 0;
    int g = 0;
    int f = 0;
    int parent = -1;
    int action = 0;
};

// Pack (ar, ac, br, bc) into one 64-bit key. Assumes coords < 2^16.
inline long long encode(int ar, int ac, int br, int bc)
{
    return (static_cast<long long>(ar) << 48)
         | (static_cast<long long>(ac) << 32)
         | (static_cast<long long>(br) << 16)
         | static_cast<long long>(bc);
}

}  // namespace

SingleBoxAStar::SingleBoxAStar(const State& state, Topology& topology)
    : state_(state), topology_(topology), level_(*state.level)
{
}

std::vector<int> SingleBoxAStar::plan(
    int agent, char box,
    int box_row, int box_col,
    int goal_row, int goal_col,
    int expansion_cap,
    std::chrono::steady_clock::time_point deadline)
{
    (void)box;

    // Heuristic uses box-aware BFS from the goal cell. Boxes other than the
    // active one (at box_row, box_col) are treated as walls when computing the
    // distance field for the box itself; this matches the active-box-only A*
    // model where pushing requires the destination to be empty.
    std::vector<std::vector<int>> dist_to_goal(
        level_.rows, std::vector<int>(level_.cols, kInf));
    {
        // BFS from goal cell, blocking on walls and boxes except the active
        // box's current cell (so the box can move out of its current cell).
        if (goal_row < 0 || goal_row >= level_.rows
            || goal_col < 0 || goal_col >= level_.cols
            || level_.walls[goal_row][goal_col]) {
            return {};
        }
        dist_to_goal[goal_row][goal_col] = 0;
        std::queue<std::pair<int, int>> q;
        q.emplace(goal_row, goal_col);
        while (!q.empty()) {
            auto [cr, cc] = q.front();
            q.pop();
            static const int dr[4] = {-1, 1, 0, 0};
            static const int dc[4] = {0, 0, -1, 1};
            for (int k = 0; k < 4; ++k) {
                const int nr = cr + dr[k];
                const int nc = cc + dc[k];
                if (nr < 0 || nr >= level_.rows
                    || nc < 0 || nc >= level_.cols) continue;
                if (level_.walls[nr][nc] || dist_to_goal[nr][nc] != kInf) continue;
                if (state_.boxes[nr][nc] != '\0'
                    && !(nr == box_row && nc == box_col)) continue;
                dist_to_goal[nr][nc] = dist_to_goal[cr][cc] + 1;
                q.emplace(nr, nc);
            }
        }
    }
    if (dist_to_goal[box_row][box_col] == kInf) return {};

    auto blocked = [&](int row, int col, int active_box_row, int active_box_col) {
        if (row < 0 || row >= level_.rows || col < 0 || col >= level_.cols
            || level_.walls[row][col]) return true;
        if (row == active_box_row && col == active_box_col) return true;
        if (state_.boxes[row][col] != '\0'
            && !(row == box_row && col == box_col)) return true;
        for (int other = 0;
             other < static_cast<int>(state_.agent_rows.size()); ++other) {
            if (other != agent
                && state_.agent_rows[other] == row
                && state_.agent_cols[other] == col) return true;
        }
        return false;
    };

    auto heuristic = [&](int ar, int ac, int br, int bc) -> int {
        const int box_d = dist_to_goal[br][bc];
        if (box_d == kInf) return kInf;
        return box_d + std::max(0, std::abs(ar - br) + std::abs(ac - bc) - 1);
    };

    std::vector<Node> nodes;
    nodes.reserve(4096);
    auto cmp = [&nodes](int lhs, int rhs) {
        if (nodes[lhs].f != nodes[rhs].f) return nodes[lhs].f > nodes[rhs].f;
        return nodes[lhs].g < nodes[rhs].g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);
    std::unordered_map<long long, int> best_g;

    const int ar0 = state_.agent_rows[agent];
    const int ac0 = state_.agent_cols[agent];
    const int h0  = heuristic(ar0, ac0, box_row, box_col);
    if (h0 == kInf) return {};
    nodes.push_back({ar0, ac0, box_row, box_col, 0, h0, -1, 0});
    open.push(0);
    best_g[encode(ar0, ac0, box_row, box_col)] = 0;

    const auto& tbl = actions();
    int expansions = 0;
    while (!open.empty() && expansions < expansion_cap) {
        if ((expansions & 2047) == 0
            && std::chrono::steady_clock::now() >= deadline) {
            return {};
        }
        const int index = open.top();
        open.pop();
        const Node cur = nodes[index];
        const auto bg = best_g.find(encode(cur.ar, cur.ac, cur.br, cur.bc));
        if (bg != best_g.end() && bg->second < cur.g) continue;
        if (cur.br == goal_row && cur.bc == goal_col) {
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

        for (int ai = 1; ai < static_cast<int>(tbl.size()); ++ai) {
            const Action& a = tbl[ai];
            int nar = cur.ar, nac = cur.ac;
            int nbr = cur.br, nbc = cur.bc;
            bool ok = true;
            switch (a.type) {
                case ActionType::NoOp:
                    ok = false;
                    break;
                case ActionType::Move:
                    nar += a.agent_dr;
                    nac += a.agent_dc;
                    if (blocked(nar, nac, cur.br, cur.bc)) ok = false;
                    break;
                case ActionType::Push: {
                    const int tr = cur.ar + a.agent_dr;
                    const int tc = cur.ac + a.agent_dc;
                    if (tr != cur.br || tc != cur.bc) { ok = false; break; }
                    nbr = cur.br + a.box_dr;
                    nbc = cur.bc + a.box_dc;
                    nar = tr;
                    nac = tc;
                    if (blocked(nbr, nbc, cur.br, cur.bc)) ok = false;
                    break;
                }
                case ActionType::Pull: {
                    const int obr = cur.ar - a.box_dr;
                    const int obc = cur.ac - a.box_dc;
                    if (obr != cur.br || obc != cur.bc) { ok = false; break; }
                    nar += a.agent_dr;
                    nac += a.agent_dc;
                    nbr = cur.ar;
                    nbc = cur.ac;
                    if (blocked(nar, nac, cur.br, cur.bc)) ok = false;
                    break;
                }
            }
            if (!ok) continue;
            const int h = heuristic(nar, nac, nbr, nbc);
            if (h == kInf) continue;
            const int ng = cur.g + 1;
            const long long key = encode(nar, nac, nbr, nbc);
            const auto it = best_g.find(key);
            if (it != best_g.end() && it->second <= ng) continue;
            best_g[key] = ng;
            nodes.push_back({nar, nac, nbr, nbc, ng, ng + h, index, ai});
            open.push(static_cast<int>(nodes.size()) - 1);
        }
    }
    return {};
}

std::vector<int> SingleBoxAStar::plan_agent_to(
    int agent, int target_row, int target_col, int expansion_cap)
{
    struct PNode {
        int row = 0, col = 0;
        int parent = -1;
        int action = 0;
    };
    std::deque<int> queue;
    std::vector<PNode> nodes;
    std::unordered_map<int, int> seen;

    auto encode_cell = [&](int r, int c) {
        return r * level_.cols + c;
    };

    auto cell_passable = [&](int r, int c) {
        if (r < 0 || r >= level_.rows || c < 0 || c >= level_.cols) return false;
        if (level_.walls[r][c]) return false;
        if (state_.boxes[r][c] != '\0') return false;
        for (int other = 0;
             other < static_cast<int>(state_.agent_rows.size()); ++other) {
            if (other != agent
                && state_.agent_rows[other] == r
                && state_.agent_cols[other] == c) return false;
        }
        return true;
    };

    const int sr = state_.agent_rows[agent];
    const int sc = state_.agent_cols[agent];
    nodes.push_back({sr, sc, -1, 0});
    queue.push_back(0);
    seen[encode_cell(sr, sc)] = 0;

    const auto& tbl = actions();
    int expansions = 0;
    while (!queue.empty() && expansions < expansion_cap) {
        const int idx = queue.front();
        queue.pop_front();
        const PNode cur = nodes[idx];
        if (cur.row == target_row && cur.col == target_col) {
            std::vector<int> plan;
            int cursor = idx;
            while (nodes[cursor].parent != -1) {
                plan.push_back(nodes[cursor].action);
                cursor = nodes[cursor].parent;
            }
            std::reverse(plan.begin(), plan.end());
            return plan;
        }
        ++expansions;
        for (int ai = 1; ai <= 4; ++ai) {
            const Action& a = tbl[ai];
            const int nr = cur.row + a.agent_dr;
            const int nc = cur.col + a.agent_dc;
            if (!cell_passable(nr, nc)) continue;
            const int key = encode_cell(nr, nc);
            if (seen.count(key)) continue;
            seen[key] = static_cast<int>(nodes.size());
            nodes.push_back({nr, nc, idx, ai});
            queue.push_back(static_cast<int>(nodes.size()) - 1);
        }
    }
    return {};
}

}  // namespace aimas
