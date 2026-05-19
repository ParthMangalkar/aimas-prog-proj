# Current C++ SearchClient Architecture

This document describes the active implementation in `searchclient_cpp/searchclient_cpp/main.cpp` and the algorithms currently available through the compiled C++ client.

## High-level summary

The current client is a hybrid solver for the AIMAS hospital/Sokoban domain. It keeps the original global joint-state search algorithms, but the default path is now:

```text
parse level
  -> run a basic goal-feasibility gate
  -> try prioritized multi-agent task planner
     -> validate every complete candidate with local server-style replay
     -> retry replay-invalid candidates with committed-world and/or relaxed reservations
  -> if that fails, try serial task fallback
  -> if AIMAS_ENABLE_CBS_REPLAN=1, try an experimental fixed-assignment CBS fallback
  -> if that fails, try bounded weighted A* fallback in default and prioritized-only modes
  -> if AIMAS_ENABLE_WINDOWED_REPLAN=1, try an experimental windowed full-joint fallback
  -> if allowed, fall back to smart best-first graph search
  -> emit one joint action per timestep to the server
```

The implementation is "hybrid" because it combines:

1. Whole-state graph search over complete joint actions.
2. Greedy task assignment for box goals.
3. Single-agent Space-Time A* with reservation tables.
4. An opt-in CBS constraint-tree coordinator over fixed per-agent assignments.
5. Serial single-agent fallback planning with blocker eviction and box relocation.
6. Smart heuristic best-first fallback search.

## Active source layout

| File or folder | Role |
|---|---|
| `searchclient_cpp/searchclient_cpp/main.cpp` | Active client implementation. It contains domain model, parser, global search, heuristics, prioritized planner, serial fallback, and protocol output. |
| `searchclient_cpp/searchclient_cpp/CMakeLists.txt` | Builds the active `searchclient_cpp` executable and a compile-only `mapf_experiments` static target. |
| `searchclient_cpp/searchclient_cpp/mapf/` | Separate MAPF experiment code. It is compile-checked as `mapf_experiments`, but not linked into the active client target. |
| `misc/server.jar` | Current validation server used by the benchmark workflow. |
| `searchclient_cpp/server_cpp/` | Legacy C++ server source. It currently has pre-existing conflict markers, so it is not the validation path. |
| `benchmarks/run_all_levels.py` | Benchmark runner that executes the Java server/client workflow over level sets and records CSV/Markdown/log output. |
| `benchmarks/check_solved_regressions.py` | Regression checker for solved-level benchmark CSVs. |
| `benchmarks/triage_logs.py` | Helper for classifying failed benchmark logs. |
| `other_misc/AGENT_HANDOFF.md` | Operational handoff and current improvement priorities. |
| `other_misc/IMPLEMENTATION_CHANGES.md` | Detailed record of implementation changes. |

Important: stale MAPF entry-point flags such as `-mapf`, `-ecbs`, `-alns`, and `-full` are not parsed by the active `main.cpp`. The MAPF sources are intentionally isolated in the `mapf_experiments` build target and do not affect the executable behavior.

## Domain model

The client models the server domain with these core types:

| Type | Purpose |
|---|---|
| `Action` | One primitive action: `NoOp`, `Move`, `Push`, or `Pull`, with agent and box deltas. |
| `State` | Full world state: agent positions, box grid, parent pointer, joint action, and path cost. Static level data such as colors, walls, and goals are stored as static members. |
| `StatePtrHasher` / `StatePtrEqual` | Hashing/equality for duplicate detection in graph search. |
| `Frontier` | Abstract search frontier interface used by BFS, DFS, and best-first search. |
| `Heuristic` | Base heuristic clakss that precomputes BFS distance maps from every goal cell. |

`State` is responsible for:

- checking action applicability for each agent;
- checking joint-action conflicts;
- applying a joint action to produce the next state;
- detecting goal states;
- expanding all applicable non-conflicting joint actions;
- extracting the final plan by walking parent pointers.

## Input/output protocol

At startup, the client writes the expected handshake:

```text
SearchClient
#This is a comment.
```

Then it reads the full level from standard input, computes a plan, and sends one joint action per timestep. Individual agent actions in a joint action are separated by `|`.

Example:

```text
Move(E)|NoOp|Push(N,N)
```

After each emitted joint action, the client reads one response line from the server before sending the next action.

## Profiling and validation guards

`AIMAS_PROFILE=1` enables profiling events on stderr. The implementation uses `ScopedProfile` and `profile_event` around parsing, prioritized planning, serial fallback, bounded weighted A*, graph search, and selected inner planner stages.

### Default-on safety / heuristic features

Three default-on features can be disabled for ablation by setting the matching env var to `0`:

| Env var (default ON) | Feature | Effect when ON |
|---|---|---|
| `AIMAS_DEADLOCK_PRUNING` | Static-deadlock pruning (push+pull reverse reachability per box letter) | Prunes pushes/pulls that would move a box into a cell from which it cannot reach any goal of its letter. Sound because Pull is a legal action in this domain. |
| `AIMAS_GOAL_DAG` | Goal-dependency DAG ordering with chronological backtracking | Adds an ordering variant where each agent's `DeliverBox` subtasks are topologically sorted so that placing one box's goal does not strand another box. Falls back to input order on cycle. |
| `AIMAS_PUSH_DISTANCE_H` | Tighter admissible heuristic combining passable-BFS with push-distance BFS | For `DeliverBox` heuristic values, takes `max(passable_dist, push_dist)` from the box's current cell to its goal. Push-distance BFS counts box moves where each move requires push or pull agent feasibility. Both grids are admissible, so `max` keeps admissibility. |

All three are validated regression-clean against the full 116-level known-solved set and are safe to ship default-on.

### Experimental opt-in repair

`AIMAS_LNS_REPAIR=1` enables experimental prioritized-side repair attempts as a last-ditch pass after normal prioritized attempts fail. The current implementation is intentionally opt-in only: it can retry same-agent movable-box state search, bounded local joint repair, blocker relocation for final agent goals, future-owner/self relocation slices, and serial-completion splices using the existing serial machinery. The tuned repair is regression-clean on the 116-level solved set and converts `complevels/MArtians.lvl`, raising the opt-in competition count from 26/47 to 27/47, but it is still not default-on because the remaining unsolved cases mostly time out and need stronger coordination/windowed planning.

Additional opt-in tuning knobs include `AIMAS_LNS_STATE_BUDGET_S`, `AIMAS_LNS_LOCAL_AREA`, `AIMAS_LNS_JOINT_BUDGET_S`, `AIMAS_LNS_JOINT_EXPANSIONS`, `AIMAS_LNS_LOCAL_AGENTS`, `AIMAS_LNS_LOCAL_RADIUS`, and `AIMAS_LNS_LOCAL_HORIZON`.

### Experimental windowed fallback

`AIMAS_ENABLE_WINDOWED_REPLAN=1` enables a bounded full-joint windowed fallback
after the prioritized, serial, and bounded weighted-A* stages fail. It searches
from the current state for a horizon-limited partial plan, commits a short
prefix only if the heuristic/goal count improves, and replays every committed
joint action before accepting it. `AIMAS_WINDOWED_CLASSIC_BUDGET_S` can reserve
time for this stage on levels where the classic pipeline would otherwise consume
the whole server timeout. Other knobs are `AIMAS_WINDOWED_HORIZON`,
`AIMAS_WINDOWED_COMMIT`, `AIMAS_WINDOWED_MAX_WINDOWS`,
`AIMAS_WINDOWED_BUDGET_S`, `AIMAS_WINDOWED_STEP_BUDGET_S`,
`AIMAS_WINDOWED_EXPANSIONS`, and `AIMAS_WINDOWED_WEIGHT`.

This is not true WHCA*/RHCR yet because each window still expands full joint
states rather than planning per agent against a reservation window. Targeted
testing on `BigSplit`, `Lily`, `MArtians`, `Minchia`, and `ZOOM` produced no
new solves. The implementation is kept opt-in as a safe experimental tool, but
the next likely improvement should be a decomposed MAPF/CBS or local
neighborhood repair adapter.

### Experimental CBS fallback

`AIMAS_ENABLE_CBS_REPLAN=1` enables a bounded fixed-assignment CBS fallback
after prioritized and serial planning fail. The high-level CBS node stores
per-agent constraint lists, per-agent low-level plans, a sum-of-costs score, and
a conflict count. The open list is best-first on fewer conflicts, then lower
cost, then fewer constraints. Duplicate high-level nodes are rejected with a
canonical key over the sorted constraints.

The low-level planner is the existing `plan_agent` Space-Time A* over the
assigned subtasks. For CBS only, starts of boxes assigned to any agent are
treated as dynamic rather than permanent static blockers, and the CBS conflict
detector tracks their planned positions from time zero until they move. A
conflict creates two CBS children: one constraining each involved owner. The
implemented constraint types are:

| Constraint | Representation | Low-level effect |
|---|---|---|
| Vertex | `(agent, row, col, time)` or `[time, horizon]` for parked entities | Blocks the constrained agent or its active box from occupying that cell at the constrained time/range. |
| Edge/swap | `(agent, from, to, time)` | Blocks the constrained agent or active box from traversing that edge at that time. |

The final accept gate is still `plan_is_server_valid`, so an invalid CBS plan is
never emitted. CBS-specific knobs are `AIMAS_CBS_CLASSIC_BUDGET_S` (reserve time
from the older pipeline), `AIMAS_CBS_BUDGET_S`, `AIMAS_CBS_LOW_LEVEL_BUDGET_S`,
`AIMAS_CBS_MAX_NODES`, `AIMAS_CBS_VARIANTS`, `AIMAS_CBS_MAX_AGENTS`, and
`AIMAS_CBS_MAX_BOXES`.

This is a real CBS mechanism, but it is not yet a full box-MAPF solver. It keeps
the current fixed task assignment and task order; it can coordinate timing
between independently planned agents and boxes, but it cannot yet reassign a
box, change a goal order across agents, or force an inactive future box to be
moved earlier. Targeted tests on `BigSplit`, `MArtians`, and `ZOOM` produced no
new solves because their current failures still occur inside the low-level box
delivery planner before high-level CBS branching can help.

Before any selected strategy runs, `prioritized::basic_goal_feasibility` rejects simple impossible metadata cases:

- numbered agent goals whose agent is missing;
- box-goal letters with too few boxes;
- unsatisfied box goals with no compatible-color agent.

When this gate fails, the client exits without emitting an invalid plan.

## Global graph search algorithms

The original global search path is still available. These algorithms search in the full joint state space, meaning a node contains every agent and every box, and each expansion enumerates valid joint actions.

| Flag | Algorithm | Evaluation |
|---|---|---|
| `-bfs` | Breadth-first search | FIFO queue. |
| `-dfs` | Depth-first search | LIFO stack. |
| `-astar` | A* | `f(n) = g(n) + h(n)`. |
| `-wastar [weight]` | Weighted A* | `f(n) = g(n) + weight * h(n)`, default weight 5. |
| `-greedy` | Greedy best-first | `f(n) = h(n)`. |
| `-greedy-goalcount` | Greedy using unsatisfied-goal count | `f(n) = h_goal_count(n)`. |
| `-astar-goalcount` | A* using unsatisfied-goal count | `f(n) = g(n) + h_goal_count(n)`. |
| `-smart`, `-smart-greedy` | Smart greedy | `f(n) = h_smart(n)`. |
| `-smart-wastar [weight]` | Smart weighted A* | `f(n) = g(n) + weight * h_smart(n)`, default weight 3. |

`GraphSearch::search` also supports optional runtime and expansion budgets. Those budgets are used by internal fallbacks, not by every command-line strategy.

## Heuristics

The base heuristic precomputes grid distances from each goal using BFS over walls. It then combines several estimates.

### Box-to-goal matching

The old implementation estimated each unsatisfied goal independently by finding the nearest matching box. That can reuse the same box for multiple goals.

The current implementation uses one-to-one matching per box letter:

- For each letter `A` to `Z`, collect unsatisfied goals with that letter.
- Collect boxes of the same letter that are not already correctly placed.
- If there are at most 12 goals/boxes, solve the matching with dynamic programming over bitmasks.
- For larger cases, use a greedy nearest-box approximation.
- If not enough boxes exist or a target is unreachable, add a large penalty.

### Agent and goal pressure

The heuristic also includes:

- distance for numbered agent goals;
- distance from each agent to a useful compatible-color box;
- count of unsatisfied goals;
- static corner deadlock penalties for boxes stuck in non-goal corners.

### Smart heuristic

`h_smart` is:

```text
h_smart = h
        + 7 * h_goal_count
        + 2 * agent_to_useful_box_distance
        + deadlock_penalty
```

Best-first tie-breaking also uses smarter ordering:

1. lower `f`;
2. lower `h_smart`;
3. fewer unsatisfied goals;
4. deeper `g`, which prefers progress among equal-scoring nodes.

## Hybrid default and strategy flow

The current `main` chooses whether to run the prioritized planner based on the first strategy flag.

| Invocation | Flow |
|---|---|
| no flag | prioritized planner -> serial fallback -> optional `AIMAS_ENABLE_CBS_REPLAN` fallback -> bounded WA*(5) -> optional `AIMAS_ENABLE_WINDOWED_REPLAN` fallback -> final smart WA*(3) graph search |
| `-prioritized`, `-pp` | prioritized planner -> serial fallback -> optional `AIMAS_ENABLE_CBS_REPLAN` fallback -> bounded WA*(5) -> optional `AIMAS_ENABLE_WINDOWED_REPLAN` fallback -> stop if still unsolved |
| `-prioritized-fallback`, `-pp-fallback` | prioritized planner -> serial fallback -> final smart WA*(3) graph search |
| any graph-search flag | run the selected graph search directly |

The basic feasibility gate runs before this strategy dispatch. `-prioritized-fallback` / `-pp-fallback` skips the bounded weighted-A*(5) repair and goes directly to the selected best-first frontier after prioritized and serial stages fail.

## Prioritized planner

The prioritized planner lives in `namespace prioritized` inside `main.cpp`. It tries to decompose the level into per-agent tasks, plan each agent with Space-Time A*, reserve its trajectory, and then combine all per-agent paths into a joint plan. Complete candidates are always replayed through `plan_is_server_valid` before they can be emitted.

### Task representation

The planner uses two task types:

| Type | Meaning |
|---|---|
| `DeliverBox` | Move a specific box letter from a start cell to a matching goal cell. |
| `ReachCell` | Move a numbered agent to its own goal cell. |

`BoxTask` represents an unsatisfied box goal before it is assigned to an agent.

### Building box tasks

`build_box_tasks` scans the level for unsatisfied box goals, then pairs each goal with a box of the same letter.

The selection uses:

- BFS distance from goal to candidate box;
- compatible-color agent availability;
- `feasible_delivery_cost`, an A*-like reachability probe over `(agent position, box position)`;
- corridor depth bias, so boxes needed deep in corridors are often planned earlier.

The result is a list of box delivery tasks with chosen box start cells and goal cells.

Repeated static reachability probes reuse cached BFS distance grids through `cached_bfs_from`.

### Feasibility probe

`feasible_delivery_cost` is not the final planner. It is a fast static check used for assignment quality.

It searches a reduced state:

```text
agent row, agent col, box row, box col
```

It ignores time and reservations, but respects walls and most current boxes. It estimates whether a compatible agent can actually push or pull a candidate box to a candidate goal.

### Task assignment

`assign_tasks` assigns `BoxTask`s to compatible-color agents. It maintains each agent's virtual position and repeatedly chooses the cheapest task-agent pair.

The assignment score includes:

- feasible delivery cost;
- per-agent load penalty, to avoid overloading one agent;
- goal corridor depth bias, to prefer deep goals earlier;
- a fallback BFS distance score if the full feasibility probe fails.

After box tasks are assigned, numbered agent-goal tasks are appended to the relevant agent.

## Space-Time A* subtask planner

`plan_subtask` is the low-level planner used by the prioritized planner. It searches:

```text
agent row, agent col, box row, box col, time
```

For a `DeliverBox` subtask, the box position is active. For a `ReachCell` subtask, only the agent position matters.

It respects:

- walls;
- static boxes not involved in the current subtask;
- already reserved cells;
- already reserved edges;
- swap conflicts;
- unplanned agents that should temporarily stay fixed;
- time horizon;
- per-subtask runtime and expansion budgets.

It uses an A*-style priority queue with a distance-to-goal heuristic. For box delivery, the heuristic estimates the box distance to the goal plus the agent distance needed to reach/push the box.

## Reservation table

The prioritized planner's `ReservationTable` stores:

- reserved cells: `(row, col, time)`;
- reserved edges: `(from row, from col, to row, to col, time)`.

`commit_plan` supports two reservation policies:

| Policy | Use |
|---|---|
| `ReservationPolicy::Conservative` | Default. Keeps extra movement padding around old/new cells to reduce vacated-cell conflicts. |
| `ReservationPolicy::Relaxed` | Retry mode. Removes extra padding while keeping real occupancy, edge, tail, and delivered-box reservations. |

After an agent plan is accepted, `commit_plan` reserves:

- every agent cell at each timestep;
- the agent's final cell through the horizon;
- every moved box cell over time;
- the final box cell through the horizon;
- edges for moving agents and boxes;
- in conservative mode, extra old/new cell reservations around movement to reduce vacated-cell conflicts and match server behavior more closely.

This is how later agents avoid colliding with earlier planned agents and boxes.

`CommittedWorld` is used only as a retry path after a complete candidate fails replay validation. It applies already committed agent plans to a stricter box grid before planning later agents, which avoids treating moved boxes as if they were still at their original starts.

## Prioritized solve attempts

`prioritized::solve` tries several variants instead of relying on one fixed order.

### Task-order variants

For each agent, it tries variants such as:

- original assignment order;
- deeper corridor goals first;
- farther goals first;
- top-left goal ordering;
- bottom-right goal ordering.

### Agent-priority orders

For each task variant, it tries agent priority orders:

- agents with the most tasks first;
- natural agent order;
- agents with the fewest tasks first;
- each tasked agent boosted to the front.

### Static-agent modes

Each priority order is tried in two modes:

- conservative: later agents are treated as static obstacles while planning earlier agents;
- movable-agents: later agents are not blocked as aggressively.

Each complete candidate joint plan is replayed through `plan_is_server_valid`. A plan is accepted only if every action is applicable, no joint action conflicts, and the final state satisfies all goals.

For each variant/order/mode, the retry order is:

1. legacy conservative planning;
2. if a complete candidate fails replay validation, retry with committed-world state;
3. retry with relaxed reservations;
4. if that also completes but fails replay validation, retry with both committed-world state and relaxed reservations.

These behavior-changing modes are conditional fallbacks, not the default path.

## Serial fallback planner

If prioritized planning fails, `solve_serial` tries a more sequential strategy. It is intended for smaller levels:

- skipped when there are more than 16 boxes;
- skipped when there are more than 80 box tasks;
- default total budget is 15 seconds.

The serial fallback:

1. Builds the same box-task list.
2. Tries multiple global task orders: original, reversed, deep goals first, shallow goals first, nearest first, top-left goals first, and bottom-right goals first.
3. For each task, chooses the best compatible agent.
4. Plans the active task with `plan_subtask`.
5. If that fails, tries `plan_single_agent_state_search`, a bounded single-agent best-first state search.
6. If still blocked, may relocate an interfering box to a parking cell and then complete the active task.
7. Appends any required numbered agent-goal moves.
8. Validates the final plan with the same server-style replay check.

### Blocker eviction

Because serial execution gives one active agent real work while others mostly `NoOp`, an action can be blocked by another agent. The serial fallback detects this and uses `evict_agent`:

- finds the blocking agent;
- plans simple BFS moves to a nearby safe cell outside the active path;
- appends those moves before retrying the active action.

### Box relocation

If a task cannot be planned because another box blocks the rough path, `try_relocate_and_complete_task` can:

- identify candidate blocking boxes near the active box path;
- choose parking cells that are not goals and not on the rough active path;
- move the blocking box aside;
- retry the original active box delivery.

This is not a general solver, but it helps on levels where one misplaced box blocks a corridor.

## Bounded weighted-A* fallback

For default mode and `-prioritized` / `-pp`, if both prioritized planning and serial fallback fail, the code tries:

```text
Weighted A* with weight 5
normal budget: 10 seconds, 50,000 expanded states
small joint repair budget: 25 seconds, 250,000 expanded states
```

The small joint repair budget is used when the instance has at most 5 agents and at most 40 boxes.

This is a last quick attempt to solve cases where decomposition failed but global search is still small enough.

## Final smart graph-search fallback

For default mode and `-prioritized-fallback` / `-pp-fallback`, if no plan has been found, the client falls back to the selected frontier. In practice this is smart weighted A* with weight 3 for default/prioritized-fallback modes. Plain `-prioritized` / `-pp` stops after the bounded weighted-A* repair and exits without printing an invalid plan.

This fallback can solve small and medium levels, but it can still blow up on larger multi-agent levels because it searches the full joint state space.

## Dormant MAPF components

The `searchclient_cpp/searchclient_cpp/mapf/` folder contains a separate MAPF architecture, but it is not currently wired into the active executable.

| Component | Intended role |
|---|---|
| `BFSDistanceMap` | Reusable BFS distance grids. |
| `TaskAllocator` | Box-task construction and Hungarian-style task assignment interface. |
| `ReservationTable` | Space-time reservations for MAPF components. |
| `SpaceTimeAStar` | Single-agent Space-Time A* respecting reservations and constraints. |
| `CAAStar` | Cooperative A* with random priority restarts. |
| `ECBS` | CBS/ECBS-style high-level conflict resolution over per-agent paths. |
| `ALNSDestroy`, `ALNSRepair`, `ALNS` | Adaptive Large Neighborhood Search optimizer around an initial solution. |

Those files describe algorithms that could become a cleaner modular MAPF pipeline. They are compile-checked by the isolated `mapf_experiments` target, but making them active would still require an adapter and explicit `main.cpp` flag dispatch.

## Benchmarking and observed behavior

The current repository includes benchmark support for running many levels through `misc/server.jar` under a timeout:

```bash
python3 benchmarks/run_all_levels.py \
  --level-root levels \
  --level-root new_comp_levels \
  --level-root complevels \
  --algorithm=-prioritized \
  --timeout 180 \
  --output-name solved-all-current \
  --normalize
```

The benchmark runner:

- normalizes level files;
- runs the Java server with the chosen client command;
- enforces timeout;
- records solution length, wall time, timeout status, and logs;
- writes CSV and Markdown summaries.

`searchclient_cpp/solved_levels.md` tracks verified solved levels for the prioritized strategy. The current verified default baseline is 90 / 104 in `levels`, 0 / 6 in `new_comp_levels`, and 26 / 47 in `complevels`; the `complevels` increase comes from rechecking the previous solved set plus the parser-fixed `rooMbA.lvl`. With `AIMAS_LNS_REPAIR=1`, the tuned experimental repair also solves `complevels/MArtians.lvl` without solved-set regressions, for an opt-in competition count of 27 / 47.

## Practical interpretation

The current implementation is best understood as a competition-oriented heuristic planner:

- The prioritized planner is fast when tasks can be decomposed cleanly by agent and box.
- Committed-world and relaxed-reservation retries are safety valves for complete candidates that fail replay validation.
- The serial fallback handles smaller levels and some corridor/blocker cases.
- Smart weighted A* is a safety net for cases where global search is still feasible.
- The separate `mapf/` algorithms are promising but currently inactive from the executable's point of view.
