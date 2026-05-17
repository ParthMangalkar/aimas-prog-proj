#include "SpaceTimeAStar.h"

#include "../Domain.h"

#include <algorithm>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace mapf {

namespace {

struct STAKey {
    int ar, ac, br, bc, t;
    bool operator==(const STAKey& o) const {
        return ar == o.ar && ac == o.ac && br == o.br && bc == o.bc && t == o.t;
    }
};
struct STAKeyHash {
    std::size_t operator()(const STAKey& k) const {
        std::size_t h = 1469598103934665603ULL;
        auto mix = [&](int v) {
            h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(v));
            h *= 1099511628211ULL;
        };
        mix(k.ar); mix(k.ac); mix(k.br); mix(k.bc); mix(k.t);
        return h;
    }
};
struct STANode {
    int ar, ac;
    int br, bc;   // -1 if no active box
    int t;
    int g;
    int f;
    int parent;
    const Action* action;
};

bool is_extra_constrained(const ConstraintSet& constraints,
                          int row, int col, int time,
                          bool agent)
{
    if (row < 0 || col < 0) return false;
    const Constraint exact{row, col, time, agent, !agent};
    if (constraints.find(exact) != constraints.end()) return true;
    const Constraint both{row, col, time, true, true};
    return constraints.find(both) != constraints.end();
}

}  // namespace

STAResult sta_star(int agent_id,
                   int start_ar, int start_ac,
                   const Subtask& sub,
                   int start_t,
                   const ReservationTable& res,
                   const DistanceGrid& dist_to_goal,
                   const DistanceGrid* dist_to_box_start,
                   int max_t_horizon,
                   const ConstraintSet& extra_constraints)
{
    const int rows = static_cast<int>(State::walls.size());
    const int cols = static_cast<int>(State::walls[0].size());

    auto h_value = [&](int ar, int ac, int br, int bc) -> int {
        if (sub.type == SubtaskType::DeliverBox) {
            if (br < 0) return kInf;
            const int hbox = dist_to_goal[br][bc];
            if (hbox >= kInf) return kInf;
            int hagent = 0;
            if (br == sub.box_start_r && bc == sub.box_start_c && dist_to_box_start) {
                const int dab = (*dist_to_box_start)[ar][ac];
                if (dab >= kInf) return kInf;
                hagent = std::max(dab - 1, 0);
            }
            return hbox + hagent;
        }
        const int d = dist_to_goal[ar][ac];
        return d >= kInf ? kInf : d;
    };
    auto goal_reached = [&](const STANode& n) -> bool {
        if (sub.type == SubtaskType::DeliverBox) {
            return n.br == sub.box_goal_r && n.bc == sub.box_goal_c;
        }
        return n.ar == sub.target_r && n.ac == sub.target_c;
    };

    std::vector<STANode> nodes;
    nodes.reserve(2048);
    std::unordered_map<STAKey, int, STAKeyHash> best_g;

    auto cmp = [&nodes](int a, int b) {
        if (nodes[a].f != nodes[b].f) return nodes[a].f > nodes[b].f;
        return nodes[a].g < nodes[b].g;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    STANode root;
    root.ar = start_ar; root.ac = start_ac;
    if (sub.type == SubtaskType::DeliverBox) {
        root.br = sub.box_start_r; root.bc = sub.box_start_c;
    } else {
        root.br = -1; root.bc = -1;
    }
    root.t = start_t; root.g = 0;
    const int rh = h_value(root.ar, root.ac, root.br, root.bc);
    if (rh >= kInf) return {};
    root.f = rh;
    root.parent = -1; root.action = nullptr;

    if (res.is_cell_blocked(root.ar, root.ac, root.t, agent_id)) return {};
    if (root.br >= 0 && res.is_cell_blocked(root.br, root.bc, root.t, agent_id)) return {};
    if (is_extra_constrained(extra_constraints, root.ar, root.ac, root.t, true)) return {};
    if (root.br >= 0 &&
        is_extra_constrained(extra_constraints, root.br, root.bc, root.t, false)) return {};

    nodes.push_back(root);
    open.push(0);
    best_g[{root.ar, root.ac, root.br, root.bc, root.t}] = 0;

    long long expansions = 0;
    constexpr long long kExpansionCap = 2'000'000;

    while (!open.empty()) {
        const int idx = open.top(); open.pop();
        const STANode cur = nodes[idx];
        const STAKey ck{cur.ar, cur.ac, cur.br, cur.bc, cur.t};
        auto bit = best_g.find(ck);
        if (bit != best_g.end() && bit->second < cur.g) continue;

        if (goal_reached(cur)) {
            STAResult out;
            out.ok = true;
            out.final_ar = cur.ar; out.final_ac = cur.ac;
            out.final_br = cur.br; out.final_bc = cur.bc;
            int x = idx;
            while (nodes[x].parent != -1) {
                out.actions.push_back(nodes[x].action);
                x = nodes[x].parent;
            }
            std::reverse(out.actions.begin(), out.actions.end());
            return out;
        }

        if (cur.t >= max_t_horizon) continue;
        if (++expansions > kExpansionCap) break;

        for (const auto& act : ACTIONS) {
            int nar = cur.ar, nac = cur.ac;
            int nbr = cur.br, nbc = cur.bc;
            const int nt = cur.t + 1;
            bool ok = true;

            switch (act.type) {
                case ActionType::NoOp:
                    break;
                case ActionType::Move: {
                    nar = cur.ar + act.agent_row_delta;
                    nac = cur.ac + act.agent_col_delta;
                    if (nar < 0 || nar >= rows || nac < 0 || nac >= cols) { ok = false; break; }
                    if (State::walls[nar][nac]) { ok = false; break; }
                    if (cur.br >= 0 && nar == cur.br && nac == cur.bc) { ok = false; break; }
                    break;
                }
                case ActionType::Push: {
                    if (cur.br < 0) { ok = false; break; }
                    const int wbr = cur.ar + act.agent_row_delta;
                    const int wbc = cur.ac + act.agent_col_delta;
                    if (cur.br != wbr || cur.bc != wbc) { ok = false; break; }
                    const int new_br = cur.br + act.box_row_delta;
                    const int new_bc = cur.bc + act.box_col_delta;
                    if (new_br < 0 || new_br >= rows || new_bc < 0 || new_bc >= cols) { ok = false; break; }
                    if (State::walls[new_br][new_bc]) { ok = false; break; }
                    nar = wbr; nac = wbc;
                    nbr = new_br; nbc = new_bc;
                    break;
                }
                case ActionType::Pull: {
                    if (cur.br < 0) { ok = false; break; }
                    const int prev_br = cur.ar - act.box_row_delta;
                    const int prev_bc = cur.ac - act.box_col_delta;
                    if (cur.br != prev_br || cur.bc != prev_bc) { ok = false; break; }
                    const int newar = cur.ar + act.agent_row_delta;
                    const int newac = cur.ac + act.agent_col_delta;
                    if (newar < 0 || newar >= rows || newac < 0 || newac >= cols) { ok = false; break; }
                    if (State::walls[newar][newac]) { ok = false; break; }
                    nar = newar; nac = newac;
                    nbr = cur.ar; nbc = cur.ac;
                    break;
                }
            }
            if (!ok) continue;

            if (res.is_cell_blocked(nar, nac, nt, agent_id)) continue;
            if (nbr >= 0 && res.is_cell_blocked(nbr, nbc, nt, agent_id)) continue;
            if ((nar != cur.ar || nac != cur.ac) &&
                res.is_cell_blocked(cur.ar, cur.ac, nt, agent_id)) continue;
            if (nbr >= 0 && (nbr != cur.br || nbc != cur.bc) &&
                res.is_cell_blocked(cur.br, cur.bc, nt, agent_id)) continue;
            if ((nar != cur.ar || nac != cur.ac) &&
                res.is_edge_blocked(cur.ar, cur.ac, nar, nac, nt, agent_id)) continue;
            if (nbr >= 0 && (nbr != cur.br || nbc != cur.bc) &&
                res.is_edge_blocked(cur.br, cur.bc, nbr, nbc, nt, agent_id)) continue;
            if (is_extra_constrained(extra_constraints, nar, nac, nt, true)) continue;
            if (nbr >= 0 &&
                is_extra_constrained(extra_constraints, nbr, nbc, nt, false)) continue;

            const int hh = h_value(nar, nac, nbr, nbc);
            if (hh >= kInf) continue;

            const STAKey nk{nar, nac, nbr, nbc, nt};
            const int new_g = cur.g + 1;
            auto it2 = best_g.find(nk);
            if (it2 != best_g.end() && it2->second <= new_g) continue;
            best_g[nk] = new_g;

            STANode child;
            child.ar = nar; child.ac = nac;
            child.br = nbr; child.bc = nbc;
            child.t = nt;
            child.g = new_g;
            child.f = new_g + hh;
            child.parent = idx;
            child.action = &act;
            nodes.push_back(child);
            open.push(static_cast<int>(nodes.size()) - 1);
        }
    }
    return {};
}

#ifdef MAPF_SELFTEST
bool sta_constraints_selftest()
{
    std::vector<std::vector<bool>> walls(3, std::vector<bool>(3, false));
    std::vector<std::string> boxes(3, std::string(3, '\0'));
    std::vector<std::string> goals(3, std::string(3, '\0'));
    std::vector<Color> box_colors(26, Color::Unknown);
    State state({1}, {1}, {Color::Blue}, walls, boxes, box_colors, goals);

    Subtask sub;
    sub.type = SubtaskType::ReachCell;
    sub.target_r = 1;
    sub.target_c = 1;
    DistanceGrid dist_to_goal(3, std::vector<int>(3, kInf));
    dist_to_goal[1][1] = 0;
    ReservationTable res(3, 3, 5);
    ConstraintSet constraints{{Constraint{1, 1, 0, true, false}}};

    return !sta_star(0, 1, 1, sub, 0, res, dist_to_goal, nullptr, 5, constraints).ok;
}
#endif

}  // namespace mapf
