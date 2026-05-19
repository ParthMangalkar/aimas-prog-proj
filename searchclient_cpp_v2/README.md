# searchclient_cpp_v2

A modular C++17 search client for the **AIMAS Hospital (Multi-Agent
Path Finding with Boxes)** domain used in DTU's Artificial Intelligence
& Multi-Agent Systems course.

Current solve rate (r23): **74 / 116** on the combined level set
(29 / 47 `complevels` + 45 / 69 `complevels_2026`).

---

## Table of contents

- [Highlights](#highlights)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Project layout](#project-layout)
- [Usage](#usage)
- [Algorithms in one page](#algorithms-in-one-page)
- [Benchmarks](#benchmarks)
- [Configuration & limits](#configuration--limits)
- [Tests](#tests)
- [Troubleshooting](#troubleshooting)
- [Roadmap](#roadmap)
- [License & references](#license--references)

See [`ARCHITECTURE.md`](../ARCHITECTURE.md) at the repo root for the
full architectural deep-dive (module dependency graph, transactional
invariants, planner internals, joint-action accounting).

---

## Highlights

- **One executable, no flags.** Every feature is on by default; no env
  vars to remember, no flag bisection when debugging.
- **Three coordinated planners** for the final agent phase:
  - PIBT (Okumura et al. 2019)
  - Cooperative A\* with time-extended reservations (Silver 2005)
  - Serial BFS as a deterministic fallback
- **DP-optimal task assignment** (bitmask DP) layered alongside greedy
  matching so neither one dominates — they run as separate variants
  and the first to fully succeed wins.
- **Post-variant fallback layers** (r23): when every primary variant
  fails, the solver tries min-max-distance DP assignment, deterministic
  letter-group shuffles, and — for small problems (≤4 agents, ≤10
  boxes, ≤200 cells) — a bounded **full joint A\*** over agents and
  boxes with an admissible makespan heuristic.
- **Recursive blocker relocation** with parking-cell scoring and an
  anti-thrash blacklist.
- **Corridor evacuation** (radius-1 buffer around both the box path
  and the agent-to-box path) to clear deadlocks that single-box A\*
  alone cannot.
- **Transactional commits.** Every layer snapshots `(state_,
  plan_.size())` and rolls both back atomically on failure — no
  partial joint actions ever leak into the final plan.
- **Post-processing plan compaction.** A safety-net pass merges the
  serially-emitted single-agent steps into real joint actions so
  independent agents move in parallel in the GUI. Runs both a greedy
  and a sliding-window scheduler, verifies the replay against the
  original final state, and keeps the shorter. Same solve count,
  ~5 % fewer joint actions on average (up to 65 % on independent-agent
  levels like Agentix and TourDeDTU); falls back to the original plan
  on any failure.
- **Zero-dependency unit tests** (`v2_tests`), identical in Debug and
  Release.

---

## Requirements

| Tool        | Version           | Used for                          |
|-------------|-------------------|-----------------------------------|
| C++ compiler | C++17            | Building the client               |
| CMake       | ≥ 3.10            | Build configuration               |
| Java        | ≥ 11              | Running the AIMAS validation server |
| Python      | ≥ 3.9 (optional)  | `benchmarks/run_all_levels.py`    |

Tested on macOS (Apple Clang 14+) and Linux (GCC 9+). No external C++
libraries; only the standard library.

---

## Quick start

From the repo root:

```bash
# 1. Build (Release recommended)
cmake -S searchclient_cpp_v2 -B searchclient_cpp_v2/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build searchclient_cpp_v2/build -- -j4

# 2. Run unit tests
./searchclient_cpp_v2/build/v2_tests

# 3. Solve one level with the GUI
java -jar misc/server.jar \
     -l complevels_2026/donut.lvl \
     -c "searchclient_cpp_v2/build/searchclient_cpp_v2" \
     -t 30 -g -s 100
```

Expected output (excerpt):

```
[v2] Parsed level 'donut' 11x17 with 3 agents.
[v2] Plan length 42 joint actions, found in 0.084s.
```

---

## Project layout

```
searchclient_cpp_v2/
├── CMakeLists.txt
├── README.md                ← (this file)
├── include/aimas/
│   ├── core.hpp             # ActionType, Action, actions(), Level, State
│   ├── parser.hpp           # parse_level(istream&) -> Level
│   ├── topology.hpp         # walls-only / box-aware BFS, components
│   ├── single_box.hpp       # SingleBoxAStar
│   ├── pibt.hpp             # PIBT joint planner
│   ├── compact.hpp          # post-processing plan compaction
│   └── solver.hpp           # Solver orchestrator
├── src/
│   ├── core.cpp             # 29-entry action table + State methods
│   ├── parser.cpp
│   ├── topology.cpp
│   ├── single_box.cpp
│   ├── pibt.cpp
│   ├── compact.cpp          # greedy + sliding-window plan compaction
│   ├── solver.cpp           # pipeline, delivery, relocation, final phase
│   └── main.cpp             # server protocol I/O
└── tests/
    └── test_main.cpp
```

Total: ~3 800 lines of source across 14 files.

---

## Usage

### Building

```bash
cmake -S searchclient_cpp_v2 -B searchclient_cpp_v2/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build searchclient_cpp_v2/build -- -j4
```

Targets:

| Target                       | Output                                              |
|------------------------------|-----------------------------------------------------|
| `searchclient_cpp_v2`        | The client binary                                   |
| `v2_tests`                   | Self-contained unit-test runner                     |

### Running

The client speaks the standard DTU search-server protocol over stdin
/ stdout. You launch it through the server, not directly:

```bash
java -jar misc/server.jar \
     -l <level-file>            \
     -c "<path-to-binary>"      \
     -t 30                      \   # solver wall-clock budget (s)
     -g                         \   # show GUI
     -s 100                     \   # ms between GUI frames
     -o solution.txt                # write the plan
```

Run **without** `-g` for headless benchmarking.

### Batch benchmarks

```bash
python3 benchmarks/run_all_levels.py \
        --client searchclient_cpp_v2/build/searchclient_cpp_v2 \
        --server misc/server.jar \
        --level-root complevels_2026 \
        --algorithm -prioritized \
        --timeout 30 \
        --max-joint-actions 20000 \
        --normalize \
        --output benchmarks/results/v2-bench-complevels-2026.csv
```

The runner emits a CSV with `solved / timeout / over_cap / category /
server_len / wall_seconds` per level plus a Markdown summary and per-
level log files.

---

## Algorithms in one page

```
                ┌────────────────────────────────────────────────┐
                │            Solver::solve()                     │
                │                                                │
                │  build_task_variants():                        │
                │    • DP-optimal per-letter assignment (×5)     │
                │    • greedy matched × 4 goal-orders × 5 final  │
                │    • dedupe, cap at 25 variants                │
                │                                                │
                │  for variant in variants:                      │
                │      state_ = initial_state_; plan_.clear()    │
                │      if solve_once(variant): return plan_      │
                │                                                │
                │  # Post-variant fallbacks (r23):               │
                │  for v in build_extra_variants():              │
                │    • min-max-DP × 5 sort modes                 │
                │    • 8 deterministic letter-group shuffles     │
                │    if solve_once(v): return plan_              │
                │                                                │
                │  # Last-resort full joint A* (small problems): │
                │  if solve_joint_full(): return plan_           │
                │                                                │
                │  plan_ = compact_plan(plan_, initial_state_)   │
                └────────────────────────────────────────────────┘
                                  │
                                  ▼
                ┌────────────────────────────────────────────────┐
                │            solve_once(tasks)                   │
                │  ┌──────────────────────────────────────────┐  │
                │  │      run_queue() — task delivery         │  │
                │  │   deliver_task                           │  │
                │  │   └→ scatter                             │  │
                │  │      └→ corridor evacuate                │  │
                │  │         └→ defer-and-retry               │  │
                │  │            └→ blocker relocation         │  │
                │  │               (recursive, anti-thrash)   │  │
                │  └──────────────────────────────────────────┘  │
                │  ┌──────────────────────────────────────────┐  │
                │  │  redelivery scan (≤3 rounds)             │  │
                │  └──────────────────────────────────────────┘  │
                │  ┌──────────────────────────────────────────┐  │
                │  │  complete_agent_goals()                  │  │
                │  │   • final-goal agent eviction passes     │  │
                │  │   • PIBT → cooperative A* → serial       │  │
                │  └──────────────────────────────────────────┘  │
                └────────────────────────────────────────────────┘
```

| Stage | Primary planner | Fallback chain |
|---|---|---|
| Task assignment | DP-optimal (bitmask) | Greedy matched per letter |
| Per-task delivery | Single-box A\* | scatter → corridor-evac → defer → relocation |
| Blocker relocation | Single-box A\* (recursive) | Larger parking-cell pool, allow-on-goal pass |
| Final agent phase | PIBT | Cooperative A\* → serial BFS → agent-only joint A\* (≤4 agents) |
| Last-resort fallback (r23) | Min-max-DP + letter-group shuffle extras | Bounded **full joint A\*** over agents + boxes (≤4 agents / ≤10 boxes / ≤200 cells) |
| Post-processing | Plan compaction (greedy + sliding window) | Original plan if either verification fails |

Joint-action plans are built incrementally by `Solver::append_joint`.
Single-agent layers wrap their step in a joint action with `NoOp` for
all other agents; PIBT and cooperative A\* emit real multi-agent joint
actions. Before returning, `compact_plan` re-packs the per-agent
sub-sequences into the shortest equivalent joint-action plan it can
find, so independent agents move in parallel in the GUI. The
`server_len` column in benchmark CSVs is the **post-compaction**
`plan_.size()`. See `ARCHITECTURE.md` §4 and §5.7 for the per-layer
breakdown.

---

## Benchmarks

| Build                                       | complevels | complevels_2026 | Total       |
|---------------------------------------------|-----------:|----------------:|-------------|
| Legacy single-file C++ baseline             |          – |               – | 56 / 116    |
| Legacy single-file C++ (enhanced)           |    27 / 47 |        35 / 69  | 62 / 116    |
| **`searchclient_cpp_v2` (r23, current)**    |  **29/47** |       **45/69** | **74/116**  |
| └ v2 baseline (single-box A\* only, r4)     |    13 / 47 |        24 / 69  |   37 / 116  |
| └ + alt-agent retry + defer (r5)            |    14 / 47 |        25 / 69  |   39 / 116  |
| └ + relocation + scatter (r6)               |    18 / 47 |        29 / 69  |   47 / 116  |
| └ + PIBT for final agent phase (r7)         |    20 / 47 |        32 / 69  |   52 / 116  |
| └ + multi-variant orchestration (r8)        |    21 / 47 |        34 / 69  |   55 / 116  |
| └ + mover-agent eviction + box re-find (r9) |    22 / 47 |        38 / 69  |   60 / 116  |
| └ + aggressive reloc + redelivery (r10)     |    23 / 47 |        40 / 69  |   63 / 116  |
| └ + corridor evac + final-goal evac (r15)   |    28 / 47 |        44 / 69  |   72 / 116  |
| └ + cooperative A\* CAG planner (r17)       |    28 / 47 |        45 / 69  |   73 / 116  |
| └ + DP-optimal task assignment (r19)        |    28 / 47 |        45 / 69  |   73 / 116  |
| └ + post-processing plan compaction (r20)   |    28 / 47 |        45 / 69  |   73 / 116  |
| └ + post-variant fallbacks + full joint A\* (r23) | **29 / 47** |  **45 / 69** | **74 / 116** |

- Per-level CSVs and Markdown tables: `benchmarks/results/v2-bench-*.csv`.
- Per-level logs: `benchmarks/results/v2-bench-*-logs/`.
- Source-of-truth solved table (sorted, with timings and joint-action
  counts): `solved_levels.md` at the repo root.

### Levels v2 solves that the legacy enhanced doesn't (+16)

`complevels_2026`: BigForty, ClosedAI, ComMAndos, Dracarys, LaMAtes,
MASaos, MAceship, MAuseCat, NineChars, SeisSiete, trauMA

`complevels`: ISO, MArachnid, MArtians, TBSTANS1, doggy, merRAM,
TriWards

### Levels the legacy enhanced solves that v2 doesn't (–5)

AMC, DECrunchy, CphAirprt (×2), KUTitans — all require either
component decomposition or a joint-search MAPF planner for tight
rotation/swap configurations. Tracked on the [roadmap](#roadmap).

### Universally hard

Planarchy, TriSplit, Nej, escAIpe, GroupWon, LoopBots, amogus, Apdo,
DayBreak, WardRush — none of our solvers (legacy or v2) handles these
dense-rotation puzzles; they need joint-search MAPF over ≤5 agents.

---

## Configuration & limits

All limits are **compile-time constants** at the top of their owning
function in `src/solver.cpp`. There are no runtime knobs.

| Constant | Value | Where | Purpose |
|---|---:|---|---|
| `kMaxVariants` | 25 | `build_task_variants` | Cap on distinct task orderings tried |
| `kMaxBoxesDp` | 12 | `build_tasks_matched_dp` | Bitmask-DP boxes/letter (else greedy) |
| `kVariantBudgetSeconds` | 12.0 | `solve_once` | Per-variant wall-clock budget |
| Expansion cap (single-box A\*) | 200 000 | `single_box.cpp` | Per A\* call |
| Expansion cap (cooperative A\*) | 200 000 | `complete_agent_goals_reserved` | Per agent BFS |
| Relocation snapshot attempts | 6 | `deliver_task_with_relocation` | Outer rounds per task |
| Redelivery rounds | 3 | `solve_once` | Rescue passes after queue drains |

The server's `--timeout` (30 s in our benchmarks) is the **total**
wall-clock budget across all variants.

---

## Tests

Run with:

```bash
./searchclient_cpp_v2/build/v2_tests
```

Current coverage:

- `applicable_and_conflict` — Move applicability, wall blocking,
  vertex/edge conflict detection.
- `solver_trivial` — an agent walks to its numeric goal.
- `solver_box` — an agent pushes one box to one letter-goal.
- `pibt_swap` — two agents must swap positions; PIBT resolves it.
- `compact_preserves_final_state_box_level` — compaction of a real
  box-delivery plan yields the same final state.
- `compact_shrinks_independent_agents` — two agents whose sub-plans
  don't conflict get re-packed into one joint action per step.
- `compact_rejects_same_box_joint` — two agents that would touch the
  same physical box in one step are never merged into one joint
  action.

`v2_tests` is intentionally tiny — a 30-line `CHECK(...)` macro that
exits 1 on failure, zero external dependencies, identical behaviour in
Debug and Release.

---

## Troubleshooting

### Build fails on macOS with "unknown type name 'std::byte'"

Force C++17:

```bash
cmake -S searchclient_cpp_v2 -B searchclient_cpp_v2/build \
      -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release
```

### Client prints output but the server times out

Add `-t 30` (or larger) to the `java -jar misc/server.jar` line —
without it the server's default timeout is short.

### Plan is much longer than expected

Probably the level fell back to **serial** delivery / final phase
(each step is one joint action with all-but-one `NoOp`). Look at the
per-level log under `benchmarks/results/.../logs/` for which planner
won the level. PIBT and cooperative A\* compress plans dramatically.

### `Unable to solve level (in 30.0s)` on a benchmarked level

Check `benchmarks/results/v2-bench-...-logs/<level>.log` for the last
stderr line. Common reasons:

- `Failed delivery for box X` — single-box A\* and all fallbacks
  failed; usually a corridor blocked by a sibling box that the
  current relocation depth (1) can't free.
- `Failed final agent-goal phase` — PIBT, cooperative A\*, and
  serial BFS all failed for the agent positioning. Often a swap that
  needs joint-search MAPF.
- `variant timeout` — one variant ate its 12 s budget; the next
  variant is being tried.

---

## Roadmap

In priority order; each item is a self-contained module that doesn't
require re-architecting earlier modules.

1. **Joint-search MAPF for ≤5 agents** (`joint_search.{hpp,cpp}`,
   ~150 LOC) — joint A\* over the agent product state. Targets the
   universally-hard rotation puzzles (Planarchy, TriSplit, Apdo,
   escAIpe).
2. **Component decomposition** (`components.{hpp,cpp}`, ~250 LOC) —
   split the level into walls-only connected components and solve each
   in its own time budget. Targets AMC, DECrunchy, KUTitans.
3. **PIBT during delivery** (~250 LOC) — when single-box A\* fails
   with `blocked_by_agent` and the box's own corridor is clear,
   invoke PIBT to break the deadlock instead of relocation.
4. **Push/pull macros** for joint planners — abstract "push box along
   corridor" into a single action so joint A\* can reason about box
   motion without paying for every push step.
5. **Metadata feasibility prune** — per-cell reachability check that
   prunes impossible tasks before A\* runs.

---

## License & references

This client is course-project code for DTU 02285 Artificial
Intelligence & Multi-Agent Systems. It builds on the official
`searchclient_java` starter; reuses the server (`misc/server.jar`) and
level files (`complevels/`, `complevels_2026/`) provided with the
course.

Algorithm references (PDFs in `research_papers/`):

- Okumura, M., Machida, M., Défago, X., Tamura, Y. (2019).
  *Priority Inheritance with Backtracking for Iterative Multi-agent
  Path Finding.* IJCAI.
- Silver, D. (2005). *Cooperative Pathfinding.* AIIDE.
- Sharon, G., Stern, R., Felner, A., Sturtevant, N. (2015).
  *Conflict-based search for optimal multi-agent pathfinding.*
  Artificial Intelligence 219.
- Stern, R. et al. (2019). *Multi-agent Pathfinding: Definitions,
  Variants, and Benchmarks.* SoCS.
- Cohen, L. et al. (2018). *Anytime Bounded-Suboptimal Search for
  Multi-Agent Path Finding.* SoCS.
