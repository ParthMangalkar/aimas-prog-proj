# Handoff

This document summarizes the current solver work so another developer can pick
up from here without reconstructing the session history.

## Repository state and main artifacts

| Artifact | Purpose |
|---|---|
| `searchclient_cpp_enhanced/` | Standalone C++17 enhanced solver implementation. |
| `searchclient_cpp_enhanced/src/main.cpp` | Main enhanced solver source. Contains protocol parsing, state engine, planners, allocation, relocation, final-goal handling, and feature flags. |
| `searchclient_cpp_enhanced/CMakeLists.txt` | Independent CMake build for the enhanced solver binary. |
| `solved_levels.md` | Current full benchmark table for `complevels` and `complevels_2026` using the enhanced solver. |
| `ENHANCEMENT_PLAN.md` | Concrete next-step plan for high-return improvements. |

There is also earlier work in the original `searchclient_cpp` solver and
`README.md`, including a default-on medium serial fallback with
`AIMAS_ENABLE_MEDIUM_SERIAL=0` as an opt-out. The current handoff focus is the
standalone enhanced solver.

## Current benchmark result

The latest generated table is in `solved_levels.md`.

| Level Folder | Solved | Total |
|---|---:|---:|
| `complevels` | 27 | 47 |
| `complevels_2026` | 35 | 69 |
| **Total** | **62** | **116** |

Method used for the table: `searchclient_cpp_enhanced` with the benchmark label
`-enhanced`, timeout 60 seconds, joint-action cap 20000.

The previous baseline was 56/116. The most recent improvements unlocked
6 additional levels (`PFarthing`, `doggy`, `TheDevil`, `MAmaMASS`,
`MArachnid`, `moveMAcat`) with no regressions. See "Research-driven
improvements (latest)" below for details.

## Build commands

Use an out-of-tree build. The session used a build directory under the Copilot
session folder, but any external build directory works.

```bash
cmake -S searchclient_cpp_enhanced -B searchclient_cpp_enhanced/build
cmake --build searchclient_cpp_enhanced/build -- -j2
```

The enhanced solver speaks the expected server protocol:

```text
SearchClient
#This is a comment.
```

## Benchmark commands

Run one folder:

```bash
python3 benchmarks/run_all_levels.py \
  --client searchclient_cpp_enhanced/build/searchclient_cpp_enhanced \
  --level-root complevels_2026 \
  --algorithm=-enhanced \
  --timeout 60 \
  --max-joint-actions 20000 \
  --output-name enhanced-complevels-2026-full \
  --normalize \
  --profile
```

Run both benchmark folders and regenerate `solved_levels.md` using the CSVs:

```bash
python3 benchmarks/run_all_levels.py \
  --client searchclient_cpp_enhanced/build/searchclient_cpp_enhanced \
  --level-root complevels \
  --algorithm=-enhanced \
  --timeout 60 \
  --max-joint-actions 20000 \
  --output-name enhanced-complevels-full \
  --normalize \
  --profile

python3 benchmarks/run_all_levels.py \
  --client searchclient_cpp_enhanced/build/searchclient_cpp_enhanced \
  --level-root complevels_2026 \
  --algorithm=-enhanced \
  --timeout 60 \
  --max-joint-actions 20000 \
  --output-name enhanced-complevels-2026-full \
  --normalize \
  --profile
```

Or use the enhanced workflow wrapper:

```bash
python3 benchmarks/run_enhanced_workflows.py --workflow smoke
python3 benchmarks/run_enhanced_workflows.py --workflow targets \
  --env AIMAS_ENHANCED_CBS_BOX_REPAIR=1
python3 benchmarks/run_enhanced_workflows.py --workflow full --update-solved-levels
```

Generated benchmark files under `benchmarks/results/` are usually temporary
unless explicitly preserving a result table.

## Enhanced solver architecture

### Parser and state engine

`parse_level` reads the hospital-domain level format, colors, initial grid, and
goals. `State` owns agent positions and box grid and provides:

- `applicable`
- `delta_for`
- `conflicting`
- `apply_joint`
- `goal_state`

All planner outputs are replayed through this state engine before being
committed.

### Topology

`Topology` caches grid distances and provides dead-cell style checks such as
`pull_aware_dead_cell`. These checks are used by single-box planning, local
repair, relocation, and parking selection.

### Task allocation

`TaskAllocator` builds box-goal tasks by letter/color. It includes:

- greedy matching variants,
- DP matching for same-letter boxes/goals,
- multiple task ordering modes,
- gated density-aware ordering via `AIMAS_ENHANCED_DENSITY_ORDERING=1`.

### Single-box planner

`SingleBoxPlanner` plans one agent and one active box with moves, pushes, and
pulls. It treats all other boxes and agents as obstacles. This is the default
delivery primitive.

### Local multi-box repair

`LocalMultiBoxPlanner` provides fallback repair when single-box planning fails.
It can search over a full local state with real `State::apply_joint` semantics.
Implemented repair variants include:

- full local multi-box repair,
- gated bounded neighborhood repair,
- gated two-agent local repair.

The two-agent repair is opt-in because it currently adds infrastructure and can
progress some targets, but it does not improve the default full benchmark yet.

### Hierarchical solver

`HierarchicalSolver` attempts task variants serially. For each box task it:

1. selects an agent by color/distance/load,
2. tries single-box delivery,
3. tries rollback-safe corridor-agent evacuation,
4. optionally tries two-agent local repair,
5. tries local multi-box repair,
6. tries blocker relocation and delivery retry.

After box goals, it completes agent goals with:

- reservation-table final MAPF,
- greedy concurrent final movement,
- serial fallback,
- final-goal recovery for idle-agent blockers and route-frontier blockers.

## Feature flags

| Environment variable | Default | Purpose |
|---|---:|---|
| `AIMAS_ENHANCED_BUDGET_S` | `25.0` | Internal per-level solver budget in seconds. |
| `AIMAS_ENHANCED_LOCAL_REPAIR_S` | `2.0` | Local multi-box repair per-call budget. |
| `AIMAS_ENHANCED_RELOCATION_DEPTH` | `0` | Recursive relocation depth. Default off because depth > 0 slowed hard failures. |
| `AIMAS_ENHANCED_RELOCATION_S` | `2.0` | Per-relocation budget. |
| `AIMAS_ENHANCED_SINGLE_BOX_EXPANSIONS` | `180000` | A* expansion cap for regular single-box delivery. |
| `AIMAS_ENHANCED_NEIGHBORHOOD_REPAIR` | off | Enables bounded neighborhood repair. |
| `AIMAS_ENHANCED_NEIGHBORHOOD_REPAIR_S` | `0.15` | Neighborhood repair budget. |
| `AIMAS_ENHANCED_DENSITY_ORDERING` | off | Enables density-aware task ordering variants. |
| `AIMAS_ENHANCED_COMPONENT_ORDERING` | off | Enables component-grouped task ordering variants. |
| `AIMAS_ENHANCED_TWO_AGENT_REPAIR` | off | Enables bounded two-agent local repair. |
| `AIMAS_ENHANCED_TWO_AGENT_REPAIR_S` | `0.5` | Two-agent repair budget. |
| `AIMAS_ENHANCED_CBS_BOX_REPAIR` | off | Enables bounded CBS-style local box-delivery repair. |
| `AIMAS_ENHANCED_CBS_BOX_REPAIR_S` | `0.75` | CBS-style repair per-call budget. |
| `AIMAS_ENHANCED_CBS_BOX_REPAIR_AGENTS` | `2` | Maximum agents in CBS-style repair, clamped to 2-4. |
| `AIMAS_ENHANCED_RELOCATION_GRAPH` | off | Enables extra relocation parking penalties for agent-goal routes and low-degree cells. |
| `AIMAS_ENHANCED_PIBT` | `1` (on) | PIBT-based final-agent coordinator (Okumura et al. 2019). Set to `0` to disable as escape hatch. |
| `AIMAS_ENHANCED_PIBT_T` | `400` | Maximum PIBT timesteps per call. |

## Research-driven improvements (latest)

Added in the most recent iteration, informed by the papers in
`research_papers/` (LMAPF/PIBT) and on-disk failure-mode diagnostics.
These changes are default-on:

1. **Per-cell metadata feasibility check.** `metadata_feasible()` now
   rejects a level only when some goal cell is currently unsatisfied AND
   no agent of the matching color exists. Previously a goal letter whose
   color had no agent caused an immediate forfeit even when every
   matching box was already on its goal. Unlocks `TheDevil` and similar.

2. **PIBT-based final-agent coordinator.** New
   `complete_agent_goals_pibt()` implements Priority Inheritance with
   Backtracking (Okumura et al. 2019) for the final-agent-only phase.
   Boxes are treated as static obstacles (the typical case here: all box
   goals already satisfied). Includes a box-aware BFS heuristic per
   agent, swap-via-parent prevention, priority aging, visited
   joint-state detection, bounded timesteps, and full transactional
   commit through `append_joint_sequence`. Slotted between
   `complete_agent_goals_serial` and the bounded joint-A* fallback.
   Unlocks `PFarthing` and other symmetric `blocked_by_agent` failures.

3. **Anti-thrash blacklist in blocker relocation.**
   `relocate_blocker_and_deliver` now tracks
   `(blocker letter, from, to, parking_row, parking_col)` pairs per task
   and rejects exact reverse moves to prevent infinite back-and-forth
   relocations observed on `MAceship`/`BStar`.

4. **Alternative-agent retry on task failure.** New
   `rank_agents_for_box()` and refactored `try_task_sequence` retry
   failing tasks with each next-best capable agent, with full
   per-candidate state/plan/work rollback. Helps levels with multiple
   same-color agents where the greedy choice cannot reach (e.g.,
   delivers `E` and `I` on `Apdo`).

## Known target failures and observations

| Level | Current observation |
|---|---|
| `complevels/AMC.lvl` | Now solved by the enhanced solver via bounded joint final-agent recovery after local box delivery. |
| `complevels_2026/BoxBender.lvl` | Early dense box-delivery tasks fail, especially around blocker relocation and short-range box movement. |
| `complevels_2026/brAIn.lvl` | Large level; failures burn internal budget across task variants. Needs stronger decomposition or CBS rather than more global A*. |
| `complevels_2026/CudBSlvd.lvl`, `MASaos.lvl` | Long-running failures; useful for budget/regression checks. |

## What has been implemented so far

1. Created a standalone enhanced C++17 solver in `searchclient_cpp_enhanced`.
2. Implemented server-compatible protocol output and level parsing.
3. Implemented a full state/action engine with replay validation.
4. Added topology distance caching and dead-cell style pruning.
5. Added task allocation with greedy and DP matching variants.
6. Added single-box A* delivery with move/push/pull support.
7. Added serial hierarchical execution and rollback-safe plan append.
8. Added blocker relocation and future-path parking penalties.
9. Added local multi-box repair with real joint-action validation.
10. Added final agent-goal reservation MAPF.
11. Added solver and repair budget guards.
12. Added opt-in recursive relocation controls.
13. Added opt-in bounded neighborhood repair.
14. Added opt-in density-aware ordering.
15. Added rollback-safe corridor-agent evacuation.
16. Added opt-in bounded two-agent local repair.
17. Added final-goal recovery for idle-agent and route-frontier blockers.
18. Created `solved_levels.md` with current full benchmark results.

## Recommended next step

Run the full enhanced benchmark with `benchmarks/run_enhanced_workflows.py
--workflow full --update-solved-levels` before enabling any gated repair by
default. Use `BoxBender` and `brAIn` as regression guardrails for CBS repair,
component ordering, and relocation-graph scoring.
