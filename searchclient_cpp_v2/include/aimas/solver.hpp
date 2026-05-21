#pragma once

#include "aimas/core.hpp"
#include "aimas/single_box.hpp"
#include "aimas/topology.hpp"

#include <chrono>
#include <set>
#include <utility>
#include <vector>

namespace aimas {

// Orchestrator. Owns the State, Topology, and SingleBoxAStar. Produces a
// committed joint-action plan (vector of joint-action index vectors) that the
// caller emits to the server.
//
// Pipeline (v2.1):
//   1. Build several task-variant orderings (matched-greedy with multiple
//      goal/final orderings). cpp_enhanced uses up to ~16 variants; v2 ships
//      6 of the most differentiated ones to keep the search budget tight.
//   2. For each variant in turn: reset state, run delivery + final agent
//      phase, return on first full success. Roll back on per-variant failure.
//   3. Per-variant pipeline:
//        a. Defer-on-failure delivery: serial single-box A* for each task,
//           push failures to the back of the queue, retry with blocker
//           relocation as a last resort.
//        b. Final agent MAPF: PIBT → serial fallback for agent-goal cells.
//
// Each layer is a plain code path (no flags) — see README.md.
class Solver {
public:
    Solver(const Level& level);

    // Returns the joint-action plan (each entry is a joint action: indices into
    // actions(), one per agent). Empty result means no plan was found.
    std::vector<std::vector<int>> solve();

private:
    struct Task {
        int  goal_row    = 0;
        int  goal_col    = 0;
        char letter      = 0;
        int  box_row     = 0;
        int  box_col     = 0;
        int  agent       = -1;
    };

    // Multi-variant orchestration.
    std::vector<std::vector<Task>> build_task_variants();
    std::vector<Task> build_tasks_matched(int goal_order_mode);
    // DP-optimal assignment (bitmask). Per letter: assigns boxes to goals
    // minimizing total walls-only distance. Used for small letter sets
    // (boxes.size() <= 14). Adds a distinct task variant when assignment
    // differs from greedy. Mirrors cpp_enhanced's build_matched_tasks.
    std::vector<Task> build_tasks_matched_dp();
    void sort_tasks(std::vector<Task>& tasks, int final_order_mode);
    bool solve_once(std::vector<Task> tasks);

    bool deliver_task(Task& task);  // may mutate task.agent on alt-agent retry
    bool deliver_task_with_relocation(Task& task, bool allow_on_goal_blockers = false);
    bool deliver_task_with_scatter(Task& task);
    bool relocate_blocker(int blocker_agent, char blocker_letter,
                          int br, int bc, int pr, int pc);
    // Recursive variant: if direct relocation fails AND depth>0, identify
    // boxes on the relocation rough-path and recursively relocate them out
    // of the way, then retry. Mirrors cpp_enhanced's try_relocate_box.
    // Picks the mover agent itself based on box colour. Transactional:
    // on full failure rolls back state and plan to the pre-call snapshot.
    bool try_relocate_box_recursive(
        char box, int from_r, int from_c, int target_r, int target_c,
        const std::set<std::pair<int, int>>& forbidden, int depth);
    int  pick_mover_for_box(char box, int br, int bc) const;
    bool scatter_agent_to(int agent, int target_r, int target_c);
    bool evict_agent_from_forbidden(int agent,
                                    const std::set<std::pair<int, int>>& forbidden);
    // cpp_enhanced-style corridor evacuation: forbid a radius-1 buffer around
    // both the active box's intended path AND the active agent's path to
    // reach that box, then move every OTHER agent currently in that buffer
    // to a safe parking cell. Picks parking cells with multiple candidates,
    // ranked by distance/degree/goal-avoidance. Up to 2 passes for cascade
    // evictions. All-or-nothing: rolls back state and plan if any agent in
    // the buffer can't be evacuated.
    bool evacuate_corridor_agents(int active_agent,
                                  int box_r, int box_c,
                                  int goal_r, int goal_c);
    // Candidate evacuation cells for an agent: reachable (walls-only),
    // currently free, non-forbidden, ranked by (distance×3 − degree×2 +
    // goal_penalty). Returns up to 12 candidates.
    std::vector<std::pair<int, int>> evacuation_targets(
        int agent, const std::set<std::pair<int, int>>& forbidden);
    void add_path_radius(std::set<std::pair<int, int>>& cells,
                         const std::vector<std::pair<int, int>>& path,
                         int radius) const;
    // Evict agents sitting on the union of all agent-to-target paths.
    // Specifically targets the case where agent X is sitting on agent Y's
    // goal cell, or blocking Y's path to its goal. Picks an evacuation cell
    // away from the forbidden zone but still walls-only-reachable from the
    // agent's own target (so subsequent serial walks can find it again).
    // Returns true if any agent was moved.
    bool evacuate_final_goal_agent_blockers();
    // Cooperative time-extended planner: each agent plans its path through
    // (cell, time) space, reserving the cells/edges it occupies at each
    // time-step. Subsequent agents plan around the reservations and can
    // wait (NoOp) at intermediate cells. Longer-path agents go first.
    //
    // Handles cases where serial fails because the only walkable corridor
    // is shared by multiple agents at different times — exactly what
    // CphAirprt-class final-positioning needs. Treats boxes (placed at
    // their goals already) as permanent obstacles.
    bool complete_agent_goals_reserved();
    std::vector<std::pair<int, int>> path_box_to_goal_ignore_boxes(
        int br, int bc, int gr, int gc) const;
    bool find_parking_cell(int blocker_r, int blocker_c,
                           const std::set<std::pair<int, int>>& forbidden,
                           int& out_r, int& out_c) const;
    // Multi-candidate variant: returns up to `limit` parking cells, sorted by
    // a quality score combining distance, neighbour-degree, and pull-aware
    // dead-cell filtering for the given box letter. Skips walls, occupied
    // cells, level goal cells, forbidden cells, and dead cells for the box.
    std::vector<std::pair<int, int>> find_parking_cells(
        char box, int blocker_r, int blocker_c,
        const std::set<std::pair<int, int>>& forbidden,
        int limit);
    bool complete_agent_goals();          // dispatcher: pibt → serial → clear+retry
    bool complete_agent_goals_pibt();     // joint coordinator (PIBT)
    bool complete_agent_goals_serial();   // serial BFS fallback
    // Joint A* over (agent positions) treating walls + current boxes + agents
    // of other variants as static obstacles. Last-resort agent-positioning
    // planner for tight rotation puzzles (≤4 agents) that PIBT, cooperative
    // A* and serial BFS all give up on. Branching factor is 5^N (Move×4 +
    // NoOp per agent); a node cap and per-call deadline keep this from
    // exploding on bigger levels. Returns false (and rolls back state/plan)
    // on failure. Emits real multi-agent joint actions.
    //
    // `active_agent_reduction`: when true, treat agents whose own goal is
    // already satisfied (or who have no goal) as "static" (NoOp-only). This
    // unlocks N > 6 cases where only a handful of agents actually need to
    // reposition at the end (CphAirprt-class). The total N is still capped
    // (≤ 16) but eligibility is gauged on N_active.
    bool complete_agent_goals_joint(bool active_agent_reduction = false);
    void clear_paths_to_agent_goals();    // relocate boxes blocking agent goal paths
    std::vector<std::pair<int, int>> agent_goal_targets() const;
    std::set<std::pair<int, int>> agent_goal_cells() const;
    void append_joint(const std::vector<int>& joint);
    std::vector<int> noop_joint() const;

    // Per-variant wall-clock deadline. Set at the top of solve_once(); checked
    // from delivery / clearing loops to abort hopelessly slow variants.
    bool variant_time_up() const;

    // Overall (solver-wide) soft deadline; set in solve(). Used by the
    // post-variant fallback loop to avoid pushing past the server timeout.
    bool overall_time_up() const;

    // Min-max DP variant of build_tasks_matched_dp(): assigns boxes to goals
    // minimising the MAXIMUM walls-only distance, not the sum. Helps when
    // sum-min picks an assignment that traps one box behind another.
    std::vector<Task> build_tasks_matched_dp_minmax();

    // Extra fallback variants tried only after the primary variant list has
    // failed: min-max DP × 5 sort modes + deterministic random shuffles of
    // the existing DP base. Returns a deduped, signature-filtered list.
    std::vector<std::vector<Task>> build_extra_variants(
        const std::vector<std::vector<Task>>& already_tried);

    // Bounded full joint A* (agents + boxes) used as a last-resort fallback.
    // Searches forward from `state_` (the *current* state, which may be the
    // initial state or a partial-progress snapshot restored by the caller)
    // and, on success, APPENDS the joint-action sequence to `plan_`.
    // Eligibility is gauged on the START state's mismatched-box count, total
    // movable boxes, and reachable cell count — only viable for small
    // residual sub-problems. Returns true on success, leaving state_ at the
    // goal state and plan_ extended; on failure restores state_ and plan_
    // to their values at call entry. Strictly additive — never regresses a
    // level that the existing pipeline can already solve.
    //
    // When `active_agent_reduction` is true:
    //   - Computes an "active" mask: agent active iff
    //       (own goal unsatisfied) OR
    //       (color matches color of any unsatisfied letter goal AND in
    //        same walls-only component as that goal or a same-letter
    //        unmatched box).
    //   - Inactive agents only emit NoOp in the per-agent action list.
    //   - Eligibility uses N_active and residual reachable-cell count
    //     (cells reachable walls-only from active agents + relevant
    //     boxes/goals), with looser caps because branching = 9^N_active.
    //   - This unlocks levels where N_total > 6 but N_active ≤ 4.
    bool solve_joint_full_from_current(int  budget_seconds            = 5,
                                       bool prune_satisfied_boxes     = true,
                                       int  heuristic_weight          = 3,
                                       bool active_agent_reduction    = false,
                                       bool force_divisor_bound       = false);

    // Convenience wrapper: reset state_ to initial, clear plan_, then run
    // solve_joint_full_from_current. Kept for backwards-compatibility of
    // the original "from scratch" fallback path; the new partial-state
    // fallback in solve() uses solve_joint_full_from_current directly.
    bool solve_joint_full();

    // Compute the active-agent mask for the current `state_`. Active iff:
    //   - agent has unsatisfied own numeric goal, OR
    //   - agent's color matches the color of any unsatisfied letter goal,
    //     AND the agent shares a walls-only component with at least one
    //     same-color unmatched box OR same-letter unsatisfied goal.
    // Output `active.size() == state_.agent_rows.size()`.
    std::vector<bool> compute_active_agent_mask(const State& s) const;

    // Walls-only reachable cell count from any active agent (or any
    // unsatisfied goal / unmatched same-color box). Used as a tighter
    // eligibility metric than the global `cell_count` for reduced
    // joint A*.
    int residual_reachable_cells(const State& s,
                                 const std::vector<bool>& active) const;

    // Best-progress snapshot captured during variant exploration.
    // Updated at safe transactional boundaries inside solve_once() and used
    // by solve() as a launchpad for the last-resort joint A* fallback.
    struct BestSnapshot {
        State                            state;
        std::vector<std::vector<int>>    plan;
        int                              letter_goals_satisfied = 0;
        int                              agent_goals_satisfied  = 0;
        bool                             valid                  = false;
        // Total goals progress (letter-goals weighted higher because each
        // unblocks delivery of subsequent tasks). Tie-break by shorter plan.
        int score() const {
            return letter_goals_satisfied * 1000 + agent_goals_satisfied * 10;
        }
    };
    void update_best_snapshot();
    int  letter_goals_satisfied(const State& s) const;
    int  agent_goals_satisfied(const State& s) const;

    const Level&                     level_;
    State                            initial_state_;     // snapshot for resets
    State                            state_;             // mutable
    Topology                         topology_;
    SingleBoxAStar                   planner_;
    std::vector<std::vector<int>>    plan_;
    BestSnapshot                     best_snapshot_;
    std::chrono::steady_clock::time_point variant_deadline_ =
        std::chrono::steady_clock::time_point::max();
    std::chrono::steady_clock::time_point overall_deadline_ =
        std::chrono::steady_clock::time_point::max();
};

}  // namespace aimas
