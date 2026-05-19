# PPT_HELP — Presentation & Video Guide

Material to use when building the slide deck and recording the final
submission video for `searchclient_cpp_v2` (AIMAS Hospital MAPF
project).

Everything in this file is sourced from `ARCHITECTURE.md`, the v2
README, `solved_levels.md`, and the benchmark CSVs in
`benchmarks/results/`. Numbers and claims are the result of the r23
benchmark run.

---

## TL;DR — The one-paragraph elevator pitch

> *We replaced a 5 000-line single-file C++ solver (62 / 116 solved)
> with a modular C++17 rewrite (~4 100 LOC across 14 files) that
> solves **74 / 116** — a clean architecture that lets us layer PIBT,
> cooperative A\*, DP-optimal task assignment, and a bounded full
> joint A\* fallback as independent components instead of
> feature-flagged grafts. Every commit is transactional, the pipeline
> runs without a single environment flag, and unit tests run
> identically in Debug and Release.*

---

## Headline numbers (memorise these)

| Metric | Value | Note |
|---|---|---|
| Solved total | **74 / 116** (64 %) | r23 benchmark |
| Solved `complevels` | 29 / 47 | original set |
| Solved `complevels_2026` | 45 / 69 | this year's set |
| Improvement over single-file C++ | **+12 levels** | from 62 → 74 |
| New levels solved that the legacy didn't | **17** | (see list below) |
| Code size | ~4 100 LOC across 14 files | vs ~5 000 LOC in one file |
| Modules | 6 headers + 6 sources + tests | clean acyclic dependency graph |
| Environment flags | **0** | legacy had 15+ (one verbose-debug env var) |
| Per-variant wall-clock budget | 12 s | total benchmark timeout 30 s |
| Overall solver soft deadline | 27 s | bounds the r23 post-variant fallback loop |
| Max task-variant orderings tried | 25 primary + ≤13 extras | DP + greedy + min-max + shuffles |

---

## Suggested 15-minute slide deck

### Slide 1 — Title

- *searchclient_cpp_v2*
- Modular MAPF-with-Boxes Solver in C++17
- Course: DTU 02285 AI & Multi-Agent Systems
- Names, date

**Speaker note:** Open by stating the headline number — "74 of 116
levels solved, a 12-level improvement over the previous in-house
solver." Don't bury the lead.

---

### Slide 2 — The problem

- AIMAS Hospital domain: agents + colored boxes + colored goals on
  a grid.
- Per time-step **each** agent picks one of 29 actions:
  `NoOp`, `Move(4)`, `Push(12)`, `Pull(12)`.
- Server validates **joint actions** (no two agents may collide;
  pushes must respect the box-direction rule).
- Variable difficulty: 116 official levels across two suites.

**Speaker note:** Stress that this is MAPF *with manipulation* —
boxes act as movable walls. That's what makes the classic MAPF
literature (PIBT, CBS, cooperative A\*) only partially apply.

---

### Slide 3 — Where we started

- Legacy single-file `main.cpp` ≈ 5 000 LOC.
- 15+ environment flags switching behaviours.
- 62 / 116 solved (27 + 35).
- Hard to extend: every new feature is grafted onto the same call
  graph; one bug fix breaks two other levels.

**Speaker note:** Show a screenshot of `main.cpp` with the scrollbar
on the side at ~5 000 lines. Visual punchline: "all of this is one
file."

---

### Slide 4 — Goal

Rewrite from scratch:
1. **Modular** — 14 files, strictly acyclic dependencies.
2. **Flagless** — every feature always on.
3. **Transactional** — every layer rolls back atomically on failure.
4. **Faster** — beat 62 / 116. (We're at 74.)

**Speaker note:** Frame this as the central engineering decision —
"we chose to rewrite rather than patch, because the architecture
debt was bigger than any one feature gap."

---

### Slide 5 — Architecture overview (diagram)

Picture (already in `ARCHITECTURE.md` §1; copy-paste as an ASCII or
re-draw in PowerPoint):

```
stdin → parse_level → Solver::solve
                       ├── build_task_variants
                       │     (DP-optimal + greedy ×25)
                       ├── for each variant:
                       │     solve_once
                       │     ├── run_queue (delivery + relocation)
                       │     ├── redelivery scan
                       │     └── complete_agent_goals
                       │           (PIBT → cooperative A* → serial
                       │            → agent-only joint A* ≤4 agents)
                       ├── (r23) post-variant fallbacks:
                       │     ├── extras (min-max-DP + 8 shuffles)
                       │     └── solve_joint_full
                       │           (full joint A*: agents + boxes,
                       │            ≤4 agents, ≤10 boxes, ≤200 cells)
                       └── return first plan → stdout
```

**Speaker note:** Walk left-to-right. Emphasize the "first plan
wins" pattern — we don't need every variant to succeed, just one.
The r23 fallbacks at the bottom only fire when every primary
variant has already given up.

---

### Slide 6 — Module map

| Module | Job | LOC |
|---|---|---:|
| `core` | Action table + State (apply_joint, conflict, applicability) | 333 |
| `parser` | Read level file | 162 |
| `topology` | Walls-only BFS, components | 256 |
| `single_box` | A\* over (agent, box) | 308 |
| `pibt` | Joint planner for the final phase | 347 |
| `compact` | Greedy + sliding-window plan compaction | 222 |
| `solver` | Orchestrates everything (incl. r23 fallbacks + full joint A\*) | 2 680 |
| `main` | Server protocol I/O | 60 |
| `tests` | Self-contained CHECK macro | 280 |

**Speaker note:** Lower-level modules (top of the list) never
reference higher-level ones. That's what makes adding a new planner
a 200-line change instead of a 2 000-line refactor.

---

### Slide 7 — Algorithm 1: Task assignment

Two strategies, used **additively**:

- **DP-optimal** (bitmask DP): per letter, minimize total walls-only
  distance from boxes to goals. `2^B` states; cap at `B = 12`.
- **Greedy matched**: per letter, each goal picks its closest unused
  box.

Each base matching → 5 final orderings × 4 goal orderings = up to
**25 distinct task variants**, all tried in order.

**Speaker note:** Crucial insight — DP isn't always better than
greedy under our delivery layer's tie-breaking. So we keep both,
each gets ~5 variants, and the first to succeed wins. A pure DP
solution actually *regressed* by 2 levels in our r18 experiment.

---

### Slide 8 — Algorithm 2: Delivery pipeline

For each task in the queue:

```
deliver_task              (single-box A*)
  └─ scatter              (one agent off corridor)
     └─ corridor evacuate (radius-1 buffer cleared)
        └─ defer & retry  (push to back of queue)
           └─ relocation  (recursive blocker moving)
              └─ relocation (allow on-goal blockers)
```

Every layer is transactional — `(state_, plan_.size())` snapshot,
restored on failure.

**Speaker note:** This is what we spent the most engineering time
on. Most levels solve on the first or second layer; the deeper
layers exist for the gnarly multi-agent dense levels.

---

### Slide 9 — Algorithm 3: Final agent phase

After all boxes are on goals, agents still need to reach their
numeric goal cells. Three planners, tried in order:

| Planner | What it does | Reference |
|---|---|---|
| **PIBT** | Priority Inheritance with Backtracking | Okumura 2019 |
| **Cooperative A\*** | Time-extended single-agent A\* with reservations | Silver 2005 |
| **Serial BFS** | Per-agent BFS, others as walls | textbook |
| **Joint A\* (agent-only, ≤4 agents)** | Joint A\* on agent positions with boxes as walls | classical MAPF |

Between rounds, `evacuate_final_goal_agent_blockers` evicts any
agent sitting on another agent's goal.

**Speaker note:** Cooperative A\* is what pushed us from 72 to 73
(unlocked TriWards). The agent-only joint A\* layer (r23) is a
narrow safety-net for tight rotation puzzles. PIBT remains the
primary because it produces much shorter joint-action plans when
it works.

---

### Slide 9.5 — Algorithm 4 (r23): Post-variant fallbacks + full joint A\*

If **every** primary task variant fails, two more layers fire under
a 27-second overall budget:

1. **Extra variants** (≤13 of them)
   - **Min-max DP** task assignment (5 sort modes) — minimizes the
     *worst* walls-only box→goal distance instead of the sum.
   - **8 deterministic letter-group shuffles** of the DP base
     (RNG seed `0xC0FFEE` for reproducibility).
   - Signature-deduped against primary variants.

2. **Bounded full joint A\*** (`solve_joint_full`)
   - Joint A\* over the *entire* state: all agent positions + all
     box positions.
   - **Eligibility**: ≤4 agents, ≤10 boxes, ≤200 reachable cells —
     keeps the state space tractable.
   - **Heuristic**: admissible makespan lower bound = `max(over each
     letter goal: min same-letter box walls-distance, over each agent
     goal: walls-distance from agent)`.
   - **Caps**: 80 000 node expansions, 5 s wall-clock.
   - Uses `State::applicable / conflicting / apply_joint` so the
     resulting plan is server-valid by construction.

**Impact (r23 vs r20):** +1 level (`TeamAgent` — a 3-agent rotation
puzzle no serial / PIBT / relocation strategy could crack), 0
regressions. The new layers consume budget only on levels that the
existing pipeline already failed on, so currently-solved levels
never reach this code.

**Speaker note:** This is the second engineering moral after
"transactional layers": *strictly-additive fallbacks*. We can keep
adding last-resort planners forever as long as each only fires
after every previous one has cleanly rolled back.

---

### Slide 10 — How we count "joint actions"

```
plan_ = std::vector<std::vector<int>>
//      ^^^^^^^^^^^ outer entries = time-steps (= "joint actions")
//      ^^^^^^^^^^^ inner entries = per-agent action index (0..28)
```

| Layer | Per-step structure |
|---|---|
| Single-box A\*, serial, eviction | one mover, others `NoOp` |
| **PIBT, cooperative A\*** | **real joint** — many agents per step |

Same level, very different plan lengths depending on which planner
wins.

**Speaker note:** This is the demo punchline — show two
side-by-side runs of the same level: one solved by PIBT (short
plan), one solved by serial fallback (long plan). Same goal-state,
same correctness, drastically different "joint actions used."

---

### Slide 11 — Transactional invariants

Every non-trivial operation:

```cpp
const State snap   = state_;
const std::size_t snap_len = plan_.size();
if (!try_something()) {
    state_ = snap;
    plan_.resize(snap_len);
    return false;
}
```

→ No partial joint actions ever leak into the final plan. If a
variant fails, the next variant starts from a clean slate.

**Speaker note:** This sounds boring but it's the single most
important invariant. Without it, every fallback layer would corrupt
the plan in subtle ways and we'd debug at the server-validation
level instead of the planner level.

---

### Slide 12 — Results (the big table)

| Build | complevels | complevels_2026 | Total |
|---|---:|---:|---|
| Legacy baseline | – | – | 56 / 116 |
| Legacy enhanced | 27 / 47 | 35 / 69 | 62 / 116 |
| **v2 r23 (current)** | **29 / 47** | **45 / 69** | **74 / 116** |

→ **+12 levels, –20 % code size, 0 environment flags.**

**Speaker note:** Optionally show the per-round graph (r4 → r23) so
the audience sees solve-count climbing as each module landed.

---

### Slide 13 — Incremental progress (graph)

| Round | Δ feature                                  | Total |
|------:|--------------------------------------------|------:|
| r4    | baseline (single-box A\* only)              | 37    |
| r5    | + alt-agent retry + defer                  | 39    |
| r6    | + relocation + scatter                     | 47    |
| r7    | + PIBT (final phase)                       | 52    |
| r8    | + multi-variant orchestration              | 55    |
| r9    | + mover-agent eviction + box re-find       | 60    |
| r10   | + aggressive reloc + redelivery            | 63    |
| r15   | + corridor evac + final-goal evac          | 72    |
| r17   | + cooperative A\* CAG                       | 73    |
| r19   | + DP-optimal task assignment               | 73    |
| r20   | + post-processing plan compaction          | 73    |
| r23   | + post-variant fallbacks + full joint A\*   | **74** |

**Speaker note:** Each round is one cleanly reviewable code change.
Plot this as a line graph in PowerPoint.

---

### Slide 14 — What we still can't solve

- **AMC, DECrunchy, CphAirprt, KUTitans**: need component
  decomposition (split the map into walls-only components and
  solve each independently).
- **Planarchy, TriSplit, Apdo, escAIpe, GroupWon, LoopBots**:
  dense rotation/swap puzzles — need joint-search MAPF over the
  agent product state for ≤ 5 agents.

Both are designed for; left for future work.

**Speaker note:** Be honest about the limitations. Show the
roadmap from the README — these are not unknown problems, they
have clear solutions, just out of time-budget for this submission.

---

### Slide 15 — Engineering decisions worth mentioning

- **Zero env flags.** Either a feature is in or it doesn't exist.
  Makes debugging tractable.
- **Zero-dep tests.** A 30-line `CHECK(...)` macro. Runs in Debug
  and Release identically.
- **First-success orchestrator.** 25 variants, first plan wins.
  Trades some search depth for breadth.
- **Compile-time limits.** All caps (`kMaxVariants = 25`,
  `kVariantBudgetSeconds = 12.0`, …) sit at the top of their
  owning function in `solver.cpp`. Easy to find, easy to tune.

---

### Slide 16 — Demo

Run live (or a screen recording):

```bash
# Solved by PIBT — short plan
java -jar misc/server.jar -l complevels_2026/donut.lvl \
     -c "searchclient_cpp_v2/build/searchclient_cpp_v2" -t 30 -g

# Solved via cooperative A* — also short plan
java -jar misc/server.jar -l complevels/TriWards.lvl \
     -c "searchclient_cpp_v2/build/searchclient_cpp_v2" -t 30 -g

# Solved via serial fallback — long plan but correct
java -jar misc/server.jar -l complevels/ISO.lvl \
     -c "searchclient_cpp_v2/build/searchclient_cpp_v2" -t 30 -g
```

**Speaker note:** Pick one short, visually clean level (e.g., `donut`)
for the live demo. Have screen recordings of two more as backups.

---

### Slide 17 — Conclusion

- **74 / 116 solved** with a modular, flagless, transactional
  architecture.
- Four coordinated planners (single-box A\*, PIBT, cooperative A\*,
  agent-only joint A\*) + DP-optimal task assignment + recursive
  relocation + r23 bounded full joint A\* fallback.
- Clear roadmap for the remaining 42.
- Questions?

---

## Talking-point bank (use these verbatim)

- "We didn't just port the old solver — we removed it. Every layer in
  v2 is independently reviewable and independently testable."
- "Joint-action count varies by 5–10× across planners for the same
  level. PIBT and cooperative A\* commit real joint moves; serial
  layers pad with `NoOp`."
- "Every fallback is transactional. If a layer fails, the next layer
  starts from the same world state — never from a corrupted
  partial-execution state."
- "DP-optimal assignment and greedy assignment are both kept because
  one isn't strictly better. We let the orchestrator pick the first
  variant that fully succeeds."
- "Zero environment flags. The legacy solver had fifteen; debugging
  meant flag bisection. v2 has one code path."
- "Cooperative A\* uses reverse-edge reservations to prevent swap
  conflicts — that's the textbook trick that most implementations
  get wrong."

---

## Common Q&A — be ready

**Q: Why C++ and not Python or Java?**
A: Search workloads are CPU-bound. The single-box A\* and PIBT inner
loops do millions of state expansions per second; we measured a
~30× speedup against an equivalent Python sketch. C++17 also lets
us own memory layout precisely — every `State` is a flat
`std::vector` and a packed grid.

**Q: Why three different final-phase planners?**
A: Each excels at different topologies. PIBT is fast when agents
have room to back off. Cooperative A\* handles narrow corridors
where agents share the same path at different times. Serial BFS is
the deterministic last resort that never fails when paths exist in
isolation.

**Q: Why bitmask DP instead of the Hungarian algorithm?**
A: We need the assignment **and** the per-permutation orderings.
Bitmask DP lets us recover the path of choices cheaply for
secondary orderings. It also caps at 12 boxes per letter, which
fits 4096 states — comfortable for our budget.

**Q: How do you avoid live-locks in the variant loop?**
A: Per-variant 12-second deadline, plus a global per-task
attempt cap (`tasks.size() * (max_passes + 1) + 16`). A variant
can fail fast; the next one starts within 12 s at the latest.

**Q: Is the plan optimal?**
A: No, and we didn't try for optimality. PIBT and cooperative A\*
are suboptimal; our delivery layer is FIFO. We optimised for
*solve rate within time budget*, not plan length.

**Q: What's the asymptotic complexity?**
A: Per A\* call O(b^d) with b ≈ 5–25 and a hard 200 000 expansion
cap. Per variant the outer loop is bounded by `attempt_cap`, so a
single variant runs in O(num_tasks · expansion_cap) in the worst
case. Total: 25 variants × 12 s budget = 300 s upper bound, in
practice we hit 5–10 s on solved levels.

**Q: How would you scale this to 50+ agents?**
A: Cooperative A\* keeps a `(cell, time)` reservation table that
grows linearly with `max_time × num_cells`. PIBT scales as
O(N · |actions|) per step. Both fine to 50 agents. The
bottleneck is single-box A\* under deeply-nested relocation —
that's where component decomposition would help.

---

## Video recording — practical tips

- **Length:** Aim for 10–15 minutes. Don't pad — examiners value
  density.
- **Open with the result.** First 30 seconds: "We solve 74 of 116
  levels, a 12-level improvement over the previous solver." Then
  go into the how.
- **Show the code briefly.** Open `solver.cpp`, scroll to the
  `solve_once` `run_queue` lambda (line ~1763), highlight the
  cascade of fallback layers. Don't read the code aloud — just
  show its shape.
- **Show one live demo.** Pick a small clean level (`donut.lvl` or
  similar). Run with `-g` so the GUI renders the plan visually.
- **Compare side-by-side.** If video editing allows, show the same
  level being solved by PIBT (short plan) vs. by serial fallback
  (long plan). Visual proof of the joint-action accounting point.
- **End with the roadmap.** Show the README's "Roadmap" section to
  signal that the system is designed for extension, not finished.
- **Slides over face-cam.** Keep face-cam small in a corner; the
  examiner cares about the slides and the demo.

---

## Slide-deck assets you can copy from this repo

| Asset | Source path |
|---|---|
| Module map ASCII diagram | `ARCHITECTURE.md` §2 |
| Pipeline diagram | `ARCHITECTURE.md` §1 |
| Solve-count progression table | `searchclient_cpp_v2/README.md` "Benchmarks" |
| Per-level solve table | `solved_levels.md` |
| Per-level raw data (for graphs) | `benchmarks/results/v2-bench-complevels-r23.csv`<br>`benchmarks/results/v2-bench-complevels-2026-r23.csv` |
| Algorithm references (PDFs) | `research_papers/AStar_Plus/`, `research_papers/LMAPF/` |

---

## Checklist before recording

- [ ] Repo built in Release on the machine you'll record from.
- [ ] `v2_tests` runs green (`./searchclient_cpp_v2/build/v2_tests`).
- [ ] One demo level chosen, dry-run with `-g` to confirm GUI works.
- [ ] Slides reviewed against the headline numbers in this file.
- [ ] Backup screen recording of at least one solve in case live
      demo fails.
- [ ] Mic level checked.
- [ ] Speaker notes (this file) printed or on a second monitor.
