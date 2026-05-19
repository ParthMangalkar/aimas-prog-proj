# Architecture — `searchclient_cpp_v2`

This document describes the current architecture of the production solver
in this repository: `searchclient_cpp_v2`. It is the only active C++
client; the previous monolithic implementations have been removed.

> **Status (r23):** 74 / 116 levels solved (29 / 47 on `complevels`,
> 45 / 69 on `complevels_2026`). Beats the legacy single-file solver
> (62 / 116) by **+12** while shrinking from ~5 000 LOC in one file to
> ~4 100 LOC across 14 focused modules with unit tests.

---

## 1. High-level pipeline

```text
stdin (level file, DTU/AIMAS server format)
       │
       ▼
┌───────────────────┐
│  parse_level()    │  parser.cpp
└─────────┬─────────┘
          │ Level (immutable: walls, colors, goals, initial agent rows/cols, initial boxes)
          ▼
┌───────────────────┐
│  Solver::solve()  │  solver.cpp
│ ┌───────────────┐ │
│ │ build_task_   │ │  DP-optimal assignment per letter (bitmask DP, ≤12 boxes)
│ │   variants    │ │  + 4 goal-order × 5 final-order greedy variants
│ └───────┬───────┘ │  → up to 25 distinct task orderings (deduped)
│         ▼         │
│ ┌───────────────┐ │
│ │ for v in      │ │
│ │   variants:   │ │  reset state, run solve_once(v), return first success
│ │   solve_once  │ │
│ └───────┬───────┘ │
│         ▼         │
│ ┌───────────────┐ │
│ │ run_queue()   │ │   deliver_task → scatter → corridor-evac → defer
│ │  + redelivery │ │   → relocation (no-on-goal then with-on-goal) → fail
│ └───────┬───────┘ │
│         ▼         │
│ ┌───────────────┐ │
│ │ complete_     │ │   PIBT → cooperative A* → serial
│ │  agent_goals  │ │   → agent-only joint A* (≤4 agents)
│ │               │ │   + final-goal agent eviction passes
│ └───────┬───────┘ │
│         ▼         │
│ ┌───────────────┐ │
│ │ extras +      │ │   r23 fallbacks (only if every primary variant failed):
│ │ full joint A* │ │   • min-max-DP × 5 sort modes + 8 letter-group shuffles
│ │               │ │   • bounded joint A* over agents+boxes (≤4 agents,
│ │               │ │     ≤10 boxes, ≤200 cells, 80k node / 5s caps)
│ └───────────────┘ │
└─────────┬─────────┘
          │ std::vector<std::vector<int>> plan_
          ▼
┌───────────────────┐
│  main.cpp emits   │  one joint action per stdout line; reads server ack
└───────────────────┘
```

A single per-variant wall-clock deadline (`kVariantBudgetSeconds = 12s`)
bounds how long any one task ordering can run; an overall solver-wide
soft deadline (`kOverallBudgetSeconds = 27s`) caps how long the
post-variant fallback loop can run; the overall benchmark timeout is 30 s.

---

## 2. Module map

```
searchclient_cpp_v2/
├── CMakeLists.txt            # builds `searchclient_cpp_v2` + `v2_tests`
├── include/aimas/
│   ├── core.hpp              # ActionType, Action, actions(), Level, State
│   ├── parser.hpp            # parse_level(istream&) -> Level
│   ├── topology.hpp          # walls-only / box-aware BFS, components
│   ├── single_box.hpp        # SingleBoxAStar: A* on (agent_pos, box_pos)
│   ├── pibt.hpp              # PIBT joint planner for final phase
│   ├── compact.hpp           # post-processing plan compaction
│   └── solver.hpp            # Solver: orchestrates the whole pipeline
├── src/
│   ├── core.cpp              # 29-entry action table + State methods
│   ├── parser.cpp            # level-file parser
│   ├── topology.cpp          # cached BFS distances, connected components
│   ├── single_box.cpp        # A* over (agent, box) pairs
│   ├── pibt.cpp              # PIBT joint coordinator
│   ├── compact.cpp           # greedy + sliding-window plan compaction
│   ├── solver.cpp            # ~2 680 LOC — task variants, delivery,
│   │                         #   relocation, evacuation, final phase,
│   │                         #   r23 fallbacks (extras + full joint A*)
│   └── main.cpp              # server protocol I/O
└── tests/
    └── test_main.cpp         # action_table, parse, applicable+conflict,
                              # solver_trivial, solver_box, pibt_swap,
                              # compact_*
```

| Module | LOC | Depends on |
|---|---:|---|
| `core` | 333 | (stdlib only) |
| `parser` | 162 | `core` |
| `topology` | 256 | `core` |
| `single_box` | 308 | `core`, `topology` |
| `pibt` | 347 | `core`, `topology` |
| `compact` | 222 | `core` |
| `solver` | 2 680 | all of the above |
| `main` | 60 | `core`, `parser`, `solver` |
| `tests` | 280 | all |

Dependency graph is strictly acyclic; lower-level modules never reference
the orchestrator.

---

## 3. Domain model (`core.hpp`)

### Action table

29 actions, identical to the canonical DTU set:

| Type   | Count | Notes                                                          |
|--------|------:|----------------------------------------------------------------|
| `NoOp` | 1     | No motion, always applicable                                   |
| `Move` | 4     | Agent moves N/S/E/W into an empty cell                         |
| `Push` | 12    | 4 agent dirs × 3 box dirs (box can't reverse the agent)        |
| `Pull` | 12    | 4 agent dirs × 3 box dirs                                      |

Semantics:

- `Push(adir, bdir)`: agent steps in `adir`; box at the cell the agent
  vacates is pushed in `bdir`.
- `Pull(adir, bdir)`: agent steps in `adir`; box at `-bdir` from the
  agent is pulled into the agent's old cell.

### Level (immutable per game)

```cpp
struct Level {
    std::string name;
    int rows, cols;
    std::vector<std::vector<bool>> walls;     // rows × cols
    std::vector<std::vector<char>> goals;     // letter (A–Z) or digit (0–9)
    std::vector<int>  agent_color;            // index → color id
    std::vector<int>  agent_rows, agent_cols; // initial positions
    std::array<int,26> box_color;             // letter → color (or -1)
    std::vector<std::vector<char>> initial_boxes;
};
```

### State (mutable per search node)

```cpp
struct State {
    std::vector<int> agent_rows, agent_cols;
    std::vector<std::vector<char>> boxes;  // rows × cols, '\0' if empty

    static State initial(const Level&);
    bool applicable(int agent, const Action&) const;
    bool conflicting(const std::vector<int>& joint_action) const;
    bool apply_joint(const std::vector<int>& joint_action);   // transactional
    bool goal_state() const;                                  // boxes + numeric agents
};
```

`apply_joint` is the **only** mutator. It checks `applicable` for every
agent and runs `conflicting` (vertex + edge/swap conflicts on the
intended deltas) before any mutation. On failure it leaves the state
exactly as before.

---

## 4. Joint-action plan: what `plan_.size()` actually counts

The solver accumulates a plan as `std::vector<std::vector<int>> plan_`.
Every outer entry is **one server time-step**; every inner entry is the
action index (0..28) the corresponding agent executes that step. All
layers funnel through a single mutator:

```cpp
void Solver::append_joint(const std::vector<int>& joint) { plan_.push_back(joint); }
```

Different layers emit very different "shapes" of joint action:

| Layer                                    | Shape per step                              | Plan-length characteristic               |
|------------------------------------------|---------------------------------------------|------------------------------------------|
| `deliver_task` (single-box A*)           | Active agent moves, others `NoOp`           | Sum of per-box step counts (serial)      |
| `complete_agent_goals_pibt` (PIBT)       | Real joint move; many agents per step       | ≈ longest agent path                     |
| `complete_agent_goals_reserved` (CA*)    | Real joint move from time-extended replay   | ≈ max plan_time across agents            |
| `complete_agent_goals_serial`            | One agent moves, others `NoOp`              | Sum of per-agent BFS path lengths        |
| `complete_agent_goals_joint`             | Real joint move (Move/NoOp, ≤4 agents)      | ≈ longest agent path                     |
| `solve_joint_full` (r23 fallback)        | Real joint move (Move/Push/Pull/NoOp)       | True joint makespan from A\* search      |
| `evacuate_corridor_agents`               | One evacuee moves, others `NoOp`            | Adds path length of each evicted agent   |
| `relocate_blocker`, `scatter_agent_to`   | One mover, others `NoOp`                    | One step per nudge                       |

`main.cpp` emits each row to stdout and reads the server's per-row ack
once. The server's count (= `plan_.size()` when solved) is what gets
recorded in the benchmark CSVs as `server_len`. **Note**: before
`solve()` returns, `compact_plan` (§5.8) re-packs the plan, so
`server_len` is the *post-compaction* length, not the length each
layer accumulated during search.

**Practical consequence**: a level solved entirely via PIBT will have a
plan dozens of times shorter than the same level solved serially. There
is no plan-length penalty for `NoOp`-padded joint actions other than
making the count larger — the server happily accepts them.

---

## 5. Search algorithms

### 5.1 Topology (`topology.{hpp,cpp}`)

- **Walls-only BFS distance map**: shortest path treating only walls as
  obstacles (no boxes, no agents). Cached per source cell. Used as the
  admissible heuristic for `SingleBoxAStar` and for ranking candidates
  in greedy/DP task assignment and parking-cell scoring.
- **Box-aware BFS** (state-dependent): used by `single_box` to verify
  whether an agent can still reach a particular cell given the current
  box layout.
- **Connected components**: walls-only flood fill. Currently used for
  reachability checks; reserved for future component decomposition.

### 5.2 Single-box A\* (`single_box.{hpp,cpp}`)

A\* over the joint state of one agent and one box. Other agents are
treated as moving obstacles (frozen at their current positions). The
result is a single-agent action sequence; the caller is responsible for
embedding each action into a joint action (other agents NoOp) and
calling `State::apply_joint` to commit.

- **Heuristic**: walls-only Manhattan from box to its goal +
  walls-only Manhattan from agent to the closest box-adjacent cell.
  Admissible and consistent.
- **Tie-breaking**: f-score, then g-score, then deterministic action
  ordering.
- **Expansion cap**: 200 000 nodes per call; aborts cleanly on cap.
- **Dead-cell filter**: marks cells from which the box can never reach
  any same-letter goal (push-only mode) and prunes pushes into them.

### 5.3 Task assignment

Two complementary strategies, used **additively** so each level gets
multiple distinct orderings to try:

1. **DP-optimal matching** (`build_tasks_matched_dp`, bitmask DP):
   per letter, assigns boxes to goals to **minimise total walls-only
   distance**. Bitmask DP over `2^B` states; falls back to greedy when
   `B > 12` (4 096 states is the cap). Mirrors the matching from
   the legacy enhanced solver.
2. **Greedy matched** (`build_tasks_matched`): per letter, picks each
   goal's closest unused same-letter box. Cheap, generally close to
   optimal, sometimes better than DP under tie-breaking.

Each base matching is then permuted into **5 final-orders** (closest-
agent-first, FIFO, LIFO, far-goal-first, color-grouped) by
`sort_tasks`, with the 4-mode goal-order applied to the greedy version,
yielding up to **4 × 5 + 5 = 25 distinct task variants** (deduplicated
by signature, capped at `kMaxVariants = 25`).

### 5.4 Delivery (`solve_once::run_queue`)

For each task in FIFO order:

```
deliver_task                              (single-box A*)
  └─ if fails →
deliver_task_with_scatter                 (one evictable agent off corridor)
  └─ if fails →
evacuate_corridor_agents + deliver_task   (radius-1 buffer; cascade evict)
  └─ if fails →
push task to back of queue, try others    (defer-on-failure)
  └─ if all defer fails →
deliver_task_with_relocation              (move boxes off path, recursive)
  └─ if fails →
deliver_task_with_relocation(allow_on_goal_blockers=true)
  └─ if fails →
return false → next variant
```

After the queue empties, a **redelivery scan** re-queues any goal cell
that should contain a letter but doesn't (in case aggressive relocation
displaced a previously-delivered box). Up to 3 redelivery rounds.

All non-trivial layers are transactional: each takes a `state_` and
`plan_.size()` snapshot, and on failure restores both exactly — no
partial joint actions ever leak into the final plan.

### 5.5 Relocation (`deliver_task_with_relocation`,
`try_relocate_box_recursive`)

When the box-to-goal walls-only path is occupied by another box:

1. Find blocker boxes on the path, sorted by proximity to the active
   box.
2. For each blocker, generate up to 6 candidate parking cells ranked by
   distance × 3 − degree × 2 + goal-penalty.
3. Try `try_relocate_box_recursive` on `(blocker → park)`:
   - Direct path attempt with `SingleBoxAStar`.
   - If blocked and `depth > 0`, **recursively** relocate boxes blocking
     the relocation path, then retry. Depth 1 by default.
4. Evict the mover-agent off the forbidden zone before retrying delivery
   (so single-box A\* — which treats other agents as walls — isn't
   blocked by the agent we just used).
5. Up to 6 outer rounds per task with an anti-thrash blacklist of
   `{letter, from_r, from_c, to_r, to_c}` tuples.

### 5.6 Final agent phase (`complete_agent_goals`)

Numeric agent goals (`0`–`9`) are handled last because the boxes are
already on their letter goals and acting as walls. Three planners,
tried in order, with up to 4 evict-and-retry rounds layered around
them:

1. **PIBT** (`complete_agent_goals_pibt`, `pibt.cpp`)
   Priority Inheritance with Backtracking (Okumura et al. 2019). Each
   agent is assigned a static priority by remaining distance; the
   highest-priority agent picks first, lower-priority agents pick
   non-conflicting cells with backtracking when blocked. Wrapped in
   our `State::apply_joint` so swap and vertex conflicts are still
   validated at commit time.
2. **Cooperative A\*** (`complete_agent_goals_reserved`)
   Time-extended single-agent A\* with reservation tables. Movers are
   ordered farthest-first; static agents pre-reserve their cells at
   every time-step. Each mover plans through `(cell, time)` space,
   forbidden from entering reserved cells or traversing reserved edges
   (in the **reverse** direction, to prevent swap). Target cell is
   reserved for all subsequent time steps. `max_time = min(800,
   max(80, total_dist * 3 + num_agents * 10))`; 200 000 expansion cap
   per agent.
3. **Serial BFS** (`complete_agent_goals_serial`)
   Per-agent BFS treating every other agent as a static wall. Now
   **incremental** (commits per agent, multi-round) so partial
   progress is preserved when only some agents need to move.
4. **Agent-only joint A\*** (`complete_agent_goals_joint`)
   Last attempt before rolling back the whole agent phase: a bounded
   joint A\* over agent positions only (≤4 agents), treating walls
   and currently-placed boxes as obstacles. Branching is 5^N (Move ×
   4 + NoOp per agent); a 200k-node cap and per-call deadline keep
   it bounded. Unlocks tight rotation puzzles where PIBT, CA\* and
   serial BFS all deadlock around each other.

Between rounds, `evacuate_final_goal_agent_blockers` evicts any agent
sitting on another agent's final goal cell or path, and
`clear_paths_to_agent_goals` relocates boxes that would prevent serial
BFS from completing.

### 5.7 Post-variant fallbacks and bounded full joint A\* (r23)

When every primary task variant fails, `Solver::solve` doesn't give up
immediately. While there's still budget under the 27-second overall
deadline, two additional layers fire:

1. **Extra variants** (`build_extra_variants`)
   - **Min-max DP assignment** (`build_tasks_matched_dp_minmax`):
     same bitmask DP as the primary, but the cost combinator is
     `max(dp[mask], d)` instead of `dp[mask] + d`. Often unlocks
     instances where the sum-optimal assignment traps one box
     behind another.
   - Five sort modes of the min-max DP base (default / agent-locality
     / goal-row / goal-col / reverse) are emitted as separate variants.
   - **Eight deterministic letter-group shuffles** of the DP min-sum
     base (RNG seeded with `0xC0FFEE` for reproducibility, per-letter
     grouping preserved). These re-order which agent picks first
     within each letter, sometimes side-stepping a delivery deadlock.
   - All extras are signature-deduped against the primary variants so
     identical orderings aren't re-tried.

2. **Full joint A\*** (`solve_joint_full`)
   Last-resort joint A\* over the **full state** (all agent positions
   *and* all box positions). Eligibility is tight to keep state-space
   tractable: **N ≤ 3 → ≤ 10 boxes, ≤ 200 cells; N = 4 → ≤ 8 boxes,
   ≤ 260 cells**. Heuristic is an admissible makespan lower bound —
   for each letter-goal cell take the min walls-only distance to any
   same-letter box; for each agent-goal cell take the walls-only
   distance from that agent; return the max. Reuses `State::applicable
   / conflicting / apply_joint` for successor generation, so any plan
   returned is server-valid by construction. Caps: **80 000 node
   expansions, 5-second wall-clock budget**. On success, the joint-
   action sequence is replayed from `initial_state_` and verified
   against `State::goal_state` before commit.

Real-world impact (r23 vs. r20): **+1 level (`TeamAgent`)**, 0
regressions, ~38 s additional wall-time across both suites for
levels that hit the joint search and exhaust the cap. The fallbacks
are strictly additive — currently-solved levels never reach this
code path (they all finish via the primary pipeline in <10 s).

### 5.8 Post-processing plan compaction (`compact.{hpp,cpp}`)

The serial delivery layers (single-box A\*, scatter, relocation,
corridor evacuation, serial CAG) all emit one joint action per
mover step, with every other agent forced to `NoOp`. That's correct
but visually serial — in the GUI only one agent moves at a time even
though many of those steps were independent. `Solver::solve` therefore
ends with one final pass:

```cpp
plan_ = compact_plan(plan_, initial_state_);
```

`compact_plan` runs **two** schedulers and keeps the shorter result:

1. **Greedy.** Extract each agent's non-`NoOp` sub-sequence (its
   private action queue). At each compacted step, walk agents in
   index order and tentatively pop one action from each queue whose
   head is applicable on the running state and doesn't conflict with
   already-chosen actions. Commit the joint action and advance. Bails
   if no agent can move (deadlock from a too-aggressive lex-order
   choice).
2. **Sliding window.** Walk the *original* plan in temporal order and
   accumulate consecutive non-conflicting single-agent steps into one
   joint action; commit on the first conflict, then continue. Strictly
   preserves the original per-agent ordering, so it never deadlocks
   but only saves at agent-transition boundaries.

Both schedulers go through `can_merge(state, joint, agent, action)`,
which checks:

- the candidate action is applicable on `state`;
- adding it doesn't create a duplicate `box_from` with any agent
  already in the in-flight joint (guard against a latent
  `core::conflicting()` blind spot — see §6);
- `State::conflicting(joint)` still passes on the augmented joint.

After building, both candidates are **replayed** from `initial_state_`
through `State::apply_joint`; if the resulting state doesn't
`StateEq`-match the reference, that candidate is discarded and the
*original* plan is returned. Real-world impact across the 74 solved
levels: ~71 plans shrink, 5 % fewer joint actions on average, up to
65 % on independent-agent levels (Agentix, TourDeDTU). Solve count is
unchanged by construction — compaction is a safe rewrite of an
already-valid plan, never a search step.

---

## 6. Transactional invariants

| Operation                          | Snapshots `state_`? | Snapshots `plan_.size()`? | On failure                  |
|------------------------------------|:-:|:-:|---|
| `deliver_task`                     | ✓ | ✓ | Restore both                |
| `deliver_task_with_scatter`        | ✓ | ✓ | Restore both                |
| `deliver_task_with_relocation`     | ✓ | ✓ | Restore both                |
| `try_relocate_box_recursive`       | ✓ | ✓ | Restore both                |
| `evacuate_corridor_agents`         | ✓ | ✓ | Restore both                |
| `solve_once` (per variant)         | – | – | Caller (`solve`) restarts   |
| `Solver::solve` (top level)        | – | – | Returns `{}` if all fail    |

`solve` itself snapshots nothing — it just resets `state_ =
initial_state_` and `plan_.clear()` between variants. This is safe
because `initial_state_` is captured once in the constructor.

---

## 7. Build, run, test

```bash
# Build
cmake -S searchclient_cpp_v2 -B searchclient_cpp_v2/build -DCMAKE_BUILD_TYPE=Release
cmake --build searchclient_cpp_v2/build -- -j4

# Unit tests
./searchclient_cpp_v2/build/v2_tests

# Run against the server
java -jar misc/server.jar \
    -l complevels_2026/donut.lvl \
    -c "searchclient_cpp_v2/build/searchclient_cpp_v2" \
    -t 30 -g -s 100

# Benchmark a whole level set
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

Unit tests use a tiny zero-dependency `CHECK(...)` harness (no
GoogleTest, no Catch2) so they run identically in Debug and Release.

---

## 8. Provenance & references

| Component | Source |
|---|---|
| Domain semantics (action table, conflict rules) | DTU `searchclient_java` starter |
| Single-box A\* with walls-only heuristic | Standard textbook A\*, pattern from earlier in-house solvers |
| Multi-variant task assignment + relocation pipeline | Carried over from earlier monolithic C++ iterations |
| **PIBT** | Okumura et al. (2019) "Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding" |
| **Cooperative A\*** | Silver (2005) "Cooperative Pathfinding" (with the standard reverse-edge swap check) |
| **DP-optimal letter matching** | Bitmask DP over assignment; folklore |
| **Bounded full joint A\* (r23)** | Classical multi-agent A\* with admissible makespan heuristic; e.g. Standley (2010) "Finding Optimal Solutions to Cooperative Pathfinding Problems" for the joint-state framing |
| **Plan compaction (compact.cpp)** | Custom; conceptual sibling of post-hoc MAPF plan smoothing |
| References in `research_papers/` | A\*+ (`AStar_Plus/`), LMAPF (`LMAPF/`) |

---

## 9. What's deliberately **not** here

These were considered and rejected (or postponed) for v2:

- **CBS / ECBS** as the primary planner. PIBT + cooperative A\* +
  bounded full joint A\* (r23 fallback) give most of the benefit at a
  fraction of the implementation cost; CBS remains an option for the
  dense-rotation puzzles (Planarchy / TriSplit / Apdo class) we still
  can't solve.
- **Environment flags** for behavioural toggles. v2 is one path: every
  feature is either always-on or it doesn't exist. The legacy solver's
  15+ env flags made debugging and reproducibility painful. (One
  exception: `V2_VERBOSE=1` re-enables compaction-pass stderr logging,
  off by default.)
- **GoogleTest / Catch2**. The custom `CHECK(...)` macro is 30 lines
  and zero dependencies; tests stay portable.
- **Component decomposition** and **PIBT-during-delivery** are designed
  for but not yet implemented; see the roadmap in
  `searchclient_cpp_v2/README.md`. **Joint A\* for ≤4 agents** is now
  implemented (§5.6 and §5.7).
