# searchclient_cpp_v2

A clean modular C++17 rewrite of the MAPF-with-boxes search client for the
AIMAS Hospital domain. Built from scratch on the `searchclient_java`
starter architecture (parse → state → search → emit), but factored into
focused modules with cleanly separated concerns for domain semantics,
search, and orchestration.

## Why a rewrite?

The original single-file C++ solvers (~5 000 lines) iteratively grew to
solve **62/116** by piling correctness fixes and feature flags on top of
an early prototype.

`searchclient_cpp_v2` keeps the *semantics* that took the earlier solver
months to discover (action table, conflict detection, single-box A*
mechanics, replay validation, transactional joint apply), but restructures
the code into reviewable modules so future improvements (PIBT, CBS for
agents, component decomposition, push/pull macros) can be added cleanly
instead of grafted on.

## Build & test

```bash
cmake -S searchclient_cpp_v2 -B searchclient_cpp_v2/build
cmake --build searchclient_cpp_v2/build -- -j4
./searchclient_cpp_v2/build/v2_tests          # unit tests
```

Run against the server:

```bash
java -jar misc/server.jar \
  -l complevels_2026/donut.lvl \
  -c "searchclient_cpp_v2/build/searchclient_cpp_v2" \
  -t 30 -g -s 100
```

## Architecture

```
include/aimas/
  core.hpp        # ActionType, Action, actions(), Level, State, hashers, util
  parser.hpp      # parse_level(istream&) -> Level
  topology.hpp    # walls-only BFS, box-aware BFS, connected components
  single_box.hpp  # SingleBoxAStar: A* over (agent_pos, box_pos)
  solver.hpp      # Solver: orchestrates the pipeline

src/
  core.cpp        # 29-entry action table + State methods (apply_joint, etc.)
  parser.cpp      # ported from cpp_enhanced::parse_level
  topology.cpp    # cached walls-only distance, state-dependent box-aware BFS
  single_box.cpp  # the single-box A* implementation
  solver.cpp      # build_tasks → deliver_task (alt-agent + defer) → final phase
  main.cpp        # server-protocol entry point

tests/
  test_main.cpp   # action_table, parse, applicable+conflict, solver_trivial,
                  # solver_box
```

### Action table

29 actions, identical to the canonical set used by `cpp_enhanced` and the
DTU Java starter:

| Type  | Count | Notes                                                  |
|-------|-------|--------------------------------------------------------|
| NoOp  | 1     | No motion, always applicable                           |
| Move  | 4     | Agent moves N/S/E/W into empty cell                    |
| Push  | 12    | 4 agent directions × 3 box directions (box can't move opposite to agent) |
| Pull  | 12    | 4 agent directions × 3 box directions                  |

`Push(agent_dir, box_dir)` semantics: agent steps in `agent_dir`, box at
the cell the agent vacates is pushed in `box_dir`. `Pull(agent_dir, box_dir)`:
agent steps in `agent_dir`, box at `-box_dir` from the agent is pulled into
the agent's old cell.

### Pipeline (v2.4)

1. **`build_task_variants`** — for each letter-goal cell, pick the closest
   same-letter box and the closest same-color agent. Produces up to 12
   distinct task orderings (4 goal-order × 5 final-order, deduped by
   signature). Greedy per letter (no Hungarian, no component-aware
   allocation yet).
2. **`deliver_task` × N** — single-box A* on `(agent_pos, box_pos)`.
   At the start, **re-find the box by letter** in case earlier
   relocations/deliveries moved it (preferring the box closest to the
   recorded original position, and skipping boxes already sitting on a
   same-letter goal that isn't this task's goal). On failure:
   - **(layer 1)** Alt-agent retry: try every other color-compatible
     agent ranked by Manhattan-to-box.
   - **(layer 2)** Defer-on-failure: push the task to the back of the
     queue and try other tasks first (bounded retry of 3 passes).
   - **(layer 3a)** Eager scatter: pick a single agent that lies on the
     box→goal corridor and walk it off (full rollback on failure).
   - **(layer 3b, last resort)** Blocker relocation: find boxes on the
     walls-only path to the goal, pick a parking cell (free, non-goal,
     not on path), nudge the blocker there with single-box A*, retry
     the original delivery (up to 6 rounds, with an anti-thrash
     blacklist of `{blocker, from, to}` tuples). After moving a blocker,
     **evict the mover-agent** off the active path/goal so subsequent
     single-box A* (which treats other agents as walls) isn't blocked.
   - **(layer 3c, aggressive)** If standard relocation finds no movable
     blockers, retry the same relocation pass with on-goal blockers
     allowed. Any displaced on-goal box is re-queued by the next layer.
3. **Redelivery scan** — after the queue empties, scan letter-goal cells
   for unsatisfied goals. For any goal whose letter is missing, queue a
   redelivery task using the closest same-letter box. Loop up to 3
   rounds (handles aggressive-relocation displacement and similar).
4. **`complete_agent_goals`** — dispatcher: try PIBT first (joint
   coordinator) for agents with numeric goals; fall back to serial BFS
   per agent if PIBT fails. Treats other agents as static obstacles in
   the serial path.

All commits are transactional via `State::apply_joint`: deltas are
validated against `applicable` AND `conflicting` before mutation, and
solver-level operations snapshot `state_` + `plan_.size()` so a failed
task rolls back cleanly without leaving partial joint actions in the
plan.

## Current solve rate

| Build                                    | complevels | complevels_2026 | Total       |
|------------------------------------------|------------|-----------------|-------------|
| Legacy single-file C++ baseline          |          – |               – | 56 / 116    |
| Legacy single-file C++ (enhanced)        |    27 / 47 |        35 / 69  | 62 / 116    |
| **`searchclient_cpp_v2` (r19, current)** |  **28/47** |       **45/69** | **73/116**  |
| └ v2 baseline (single-box A* only, r4)   |    13 / 47 |        24 / 69  |   37 / 116  |
| └ + alt-agent retry + defer (r5)         |    14 / 47 |        25 / 69  |   39 / 116  |
| └ + relocation + scatter (r6)            |    18 / 47 |        29 / 69  |   47 / 116  |
| └ + PIBT for final agent phase (r7)      |    20 / 47 |        32 / 69  |   52 / 116  |
| └ + multi-variant orchestration (r8)     |    21 / 47 |        34 / 69  |   55 / 116  |
| └ + mover-agent eviction + box re-find (r9) |    22 / 47 |        38 / 69  |   60 / 116  |
| └ + aggressive reloc + redelivery (r10)  |    23 / 47 |        40 / 69  |   63 / 116  |
| └ + corridor evac + final-goal evac (r15)|    28 / 47 |        44 / 69  |   72 / 116  |
| └ + cooperative A\* CAG planner (r17)    |    28 / 47 |        45 / 69  |   73 / 116  |
| └ + DP-optimal task assignment (r19)     |    28 / 47 |        45 / 69  |   73 / 116  |

See `benchmarks/results/v2-bench-*.csv` for per-level breakdowns. Each
incremental layer in the v2 column is one self-contained code change.

v2 r19 **exceeds** `cpp_enhanced`'s 62/116 by +11. v2 solves 16 levels
`cpp_enhanced` doesn't (BigForty, ClosedAI, ComMAndos, Dracarys,
LaMAtes, MASaos, MAceship, MAuseCat, NineChars, SeisSiete, trauMA in
2026; ISO, MArachnid, MArtians, TBSTANS1, doggy, merRAM in complevels);
`cpp_enhanced` still solves 5 levels v2 doesn't (AMC, DECrunchy,
CphAirprt ×2, KUTitans) — mostly require component decomposition or
joint-search MAPF for tight rotation/swap puzzles.

## Roadmap (to ≥ 62/116 and beyond)

In priority order. Each item is one self-contained module; none requires
re-architecting earlier modules.

1. **Final-phase MAPF coordination** (module `pibt.{hpp,cpp}`) — replace
   `complete_agent_goals_serial` with a PIBT-style joint planner so
   agents can swap and cooperate to reach their final cells. Port the
   Okumura-2019 PIBT implementation from `cpp_enhanced`. *Expected
   impact: ~+5-10 levels* (closes the 9 "Failed final agent-goal phase"
   failures and unlocks several others where serial agent paths
   deadlock).

2. **Metadata feasibility prune** — per-cell (not per-letter) check that
   a candidate goal can be reached by *some* path through the active
   box's current free space, given other boxes and color rules. This is
   the correctness fix from `cpp_enhanced` that prevents wasting A*
   budget on impossible tasks. Cheap; should slot in before each
   `deliver_task`.

3. **Component decomposition** (module `components.{hpp,cpp}`) — split
   the level into connected components (walls-only), filter for
   color-reachability, and solve each component in its own time budget.
   Targets levels that currently time out on a single global task graph.

4. **PIBT-during-delivery** — when single-box A* fails with
   `blocked_by_agent` *and* the box's own corridor is clear, invoke
   PIBT to break the deadlock instead of falling back to relocation.
   Combines with #1.

5. **CBS-style box repair / push-pull macros** — for dense-rotation
   levels where serial delivery can never order tasks correctly,
   productionize the `AIMAS_ENHANCED_CBS_BOX_REPAIR` heuristic and add
   macros for "push a box along a corridor" so joint planners can reason
   about box motion.

## Environment flags

None yet. v2 is designed so each future module is a plain code path,
not a flag — flags only get added if they bisect a specific failure
mode for debugging. (Compare to `cpp_enhanced` which has 15+ flags.)

## Tests

`v2_tests` is a tiny zero-dependency harness (no GoogleTest, no
Catch2). It uses a local `CHECK(...)` macro that exits 1 on failure so
the test runner works in Release builds where `assert` is compiled out.

Current coverage:

- `action_table` — exactly 29 entries, correct counts per type
- `parse_trivial` — single-agent single-cell level parses
- `applicable_and_conflict` — Move applicability, wall blocking
- `solver_trivial` — agent walks to its numeric goal
- `solver_box` — agent pushes one box to one letter-goal

Edge-case tests to add as modules land: pull mechanics, swap conflict,
box-box swap, same-letter goal disambiguation, color reachability,
alt-agent retry, defer-on-failure.

## Files & line counts

```
include/aimas/core.hpp         ~120  public API
src/core.cpp                   ~240  action table + State methods
include/aimas/parser.hpp        ~10  parse_level signature
src/parser.cpp                 ~160  level-file parser
include/aimas/topology.hpp      ~40  Topology class
src/topology.cpp               ~130  BFS distances, components
include/aimas/single_box.hpp    ~50  SingleBoxAStar class
src/single_box.cpp             ~250  A* on (agent, box) pairs
include/aimas/solver.hpp        ~60  Solver class
src/solver.cpp                 ~220  pipeline + alt-agent + defer
src/main.cpp                   ~110  server protocol I/O
tests/test_main.cpp            ~160  unit tests
```

Total: ~3 800 lines across 14 focused modules.

## Provenance

- Action semantics, conflict detection, single-box A*, level parser:
  ported faithfully from earlier single-file C++ iterations of this solver.
- Pipeline shape (parse → search → emit, server protocol):
  `searchclient_java/searchclient/`.
- Algorithm references in `research_papers/`:
  Okumura et al. 2019 (PIBT), Sharon et al. 2015 (CBS), Stern 2019
  (MAPF survey), Cohen et al. 2018 (Anytime Bounded-Suboptimal),
  and recent LMAPF works for Guided-PIBT.
