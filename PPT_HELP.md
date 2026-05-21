# PPT_HELP — Presentation & Video Guide

Material to use when building the slide deck and recording the final
submission video for `searchclient_cpp_v2` (AIMAS Hospital MAPF
project).

Everything in this file is sourced from `ARCHITECTURE.md`, the v2
README, `solved_levels.md`, and the benchmark CSVs in
`benchmarks/results/`. Numbers and claims are the result of the **r41**
benchmark run.

---

## TL;DR — The one-paragraph elevator pitch

> *We replaced a 5 000-line single-file C++ solver (62 / 116 solved)
> with a modular C++17 rewrite that now solves **78 / 116** —
> a clean architecture that lets us layer PIBT, cooperative A\*,
> DP-optimal task assignment, and a multi-pass **snapshot-based bounded
> joint A\* fallback chain** (with active-agent reduction and a gated
> divisor-bound heuristic) as independent components instead of
> feature-flagged grafts. Every commit is transactional, the pipeline
> runs without a single environment flag, and unit tests run
> identically in Debug and Release.*

---

## Headline numbers (memorise these)

| Metric | Value | Note |
|---|---|---|
| Solved total | **78 / 116** (67 %) | r41 benchmark |
| Solved `complevels` | 30 / 47 | original set |
| Solved `complevels_2026` | 48 / 69 | this year's set |
| Solved `levels` (starter) | 92 / 104 | course-provided set |
| Improvement over single-file C++ | **+16 levels** | from 62 → 78 |
| Improvement over our own r23 | +4 levels | pacMAn, Dolor, GroupWon, LoopBots |
| Code size | ~6 400 LOC across 14 files | most of growth in solver.cpp fallbacks |
| Modules | 6 headers + 6 sources + tests | clean acyclic dependency graph |
| Environment flags | **0** (1 verbose-only debug var) | legacy had 15+ |
| Per-variant wall-clock budget | 12 s | total benchmark timeout 30 s |
| Overall solver soft deadline | 27 s | bounds the post-variant fallback chain |
| Max task-variant orderings tried | 25 primary + ≤13 extras | DP + greedy + min-max + shuffles |
| Joint-A\* fallback passes | 13 passes (r41) | from-initial × 2, snapshot 1..5b, reduced from-initial × 2 |

---

## Suggested 15-minute slide deck

### Slide 1 — Title

- *searchclient_cpp_v2*
- Modular MAPF-with-Boxes Solver in C++17
- Course: DTU 02285 AI & Multi-Agent Systems
- Names, date

**Speaker note:** Open by stating the headline number — "78 of 116
levels solved, a 16-level improvement over the previous in-house
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
4. **Faster** — beat 62 / 116. (We're at 78.)

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
                       │            → agent-only joint A* ≤6 agents)
                       ├── post-variant fallbacks:
                       │     ├── extras (min-max-DP + 8 shuffles)
                       │     ├── from-initial joint A* (W=3, W=5)
                       │     └── snapshot-based joint A* chain:
                       │           passes 1..3b   (gated heuristic)
                       │           passes 3c, 3d  (forced divisor bound)
                       │           passes 4, 5, 5b (active-agent reduction)
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
| `parser` | Read level file | 149 |
| `topology` | Walls-only BFS, components | 192 |
| `single_box` | A\* over (agent, box) | 265 |
| `pibt` | Joint planner for the final phase | 301 |
| `compact` | Greedy + sliding-window plan compaction | 222 |
| `solver` | Orchestrates everything (incl. snapshot fallback chain + joint A\* matrix) | 3 932 |
| `main` | Server protocol I/O | 60 |
| `tests` | Self-contained CHECK macro | 384 |

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
numeric goal cells. Four planners, tried in order:

| Planner | What it does | Reference |
|---|---|---|
| **PIBT** | Priority Inheritance with Backtracking | Okumura 2019 |
| **Cooperative A\*** | Time-extended single-agent A\* with reservations | Silver 2005 |
| **Serial BFS** | Per-agent BFS, others as walls | textbook |
| **Joint A\* (agent-only, ≤6 agents)** | Joint A\* on agent positions with boxes as walls; active-agent reduction lifts effective N to ≤16 | classical MAPF |

Between rounds, `evacuate_final_goal_agent_blockers` evicts any
agent sitting on another agent's goal.

**Speaker note:** Cooperative A\* unlocked TriWards back in r17. The
agent-only joint A\* with active-agent reduction (r38) lets us solve
levels like Dolor where 7+ agents are on the board but only 2-3
actually need to reposition. PIBT remains the primary because it
produces much shorter joint-action plans when it works.

---

### Slide 9.5 — Algorithm 4 (r23 → r41): Snapshot-based fallback chain

If **every** primary task variant fails, three layers fire under
a 27-second overall budget:

1. **Extra variants** (≤13 of them, r23)
   - **Min-max DP** task assignment (5 sort modes) — minimizes the
     *worst* walls-only box→goal distance instead of the sum.
   - **8 deterministic letter-group shuffles** of the DP base
     (RNG seed `0xC0FFEE` for reproducibility).

2. **From-initial bounded joint A\*** (r23)
   Restart the joint A\* from the initial state with weights W=3
   and W=5. Catches small puzzles where the variant pipeline tripped
   on a bad task ordering early.

3. **Snapshot-based joint A\* chain** (r38 → r41) — *the centerpiece*
   - During variant exploration, `solve_once` captures the
     highest-progress snapshot ever seen (most letter goals satisfied,
     tie-broken by agent goals).
   - When all variants fail, restart joint A\* **from that snapshot**,
     so the residual subproblem is small enough to brute-force.
   - **13 passes total**, each with a different (prune × weight ×
     reduction × heuristic) parameter combination:

   | Passes | Knob varied | Targets |
   |---|---|---|
   | 1 / 2 | satisfied-box prune on / off | pacMAn-class (corridor through satisfied box) |
   | 3 / 3b | weight W ∈ {8, 15} | DECrunchy-class (long-horizon) |
   | **3c / 3d (r41)** | force divisor-bound heuristic | **LoopBots-class (multi-box low-h_max)** |
   | 4 / 5 / 5b | active-agent reduction on | N_total > 6 with N_active ≤ 6 (Dolor) |

4. **Bounded eligibility** (`solve_joint_full_from_current`)
   - Four-tier OR: `ok_a/b/c/d` for standard / agent-only /
     wide-corridor / redelivery residuals (caps scale by N).
   - Heuristic: admissible makespan = `max(h_max,
     ceil(h_sum/N_eff))` (gated by default, forced for passes 3c/3d).
   - Caps: 40k–200k expansions, 2..10 s per pass (ANDed with overall
     deadline).
   - Uses `State::applicable / conflicting / apply_joint` so the
     resulting plan is server-valid by construction.

**Impact (r41 vs r23):** +4 levels (`pacMAn` from snapshot pass 1,
`Dolor` from passes 4/5, `GroupWon` from passes 4/5, `LoopBots`
from pass 3c). **0 regressions** — every pass only fires after all
prior passes have cleanly rolled back.

**Speaker note:** This is the third engineering moral after
"transactional layers" and "first-success orchestration":
*launch-from-snapshot fallbacks*. By capturing the best partial
progress observed and restarting joint A\* from there, we get to
brute-force only the residual — which is often dozens of times
smaller than the original problem.

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
| v2 r23 | 29 / 47 | 45 / 69 | 74 / 116 |
| **v2 r41 (current)** | **30 / 47** | **48 / 69** | **78 / 116** |

→ **+16 levels over legacy enhanced, 0 environment flags, 100 % transactional.**
Also: **92 / 104** on the course-provided `levels/` set.

**Speaker note:** Optionally show the per-round graph (r4 → r41) so
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
| r23   | + post-variant fallbacks + full joint A\*   | 74    |
| r38   | + active-agent reduction (joint A\*)        | 76    |
| r40   | + best-snapshot fallback chain (1..5b)     | 77    |
| r41   | + gated divisor-bound + force-divisor pass | **78** |

**Speaker note:** Each round is one cleanly reviewable code change.
Plot this as a line graph in PowerPoint. The recent jumps (r38..r41)
all come from refining a single algorithm — bounded joint A\* —
through better start states (snapshot), better agent subset
(reduction), and better heuristic (gated divisor bound).

---

### Slide 14 — What we still can't solve

- **AMC, DECrunchy, CphAirprt, KUTitans**: need component
  decomposition (split the map into walls-only components and
  solve each independently).
- **Planarchy, TriSplit, Apdo, escAIpe, brAIn**: dense rotation/swap
  puzzles where even snapshot-based joint A\* (≤6 agents) cannot
  find a feasible plan within the budget. Need CBS-style conflict
  resolution.

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

- **78 / 116 solved** with a modular, flagless, transactional
  architecture (plus 92 / 104 on the course-provided `levels/`).
- Four coordinated planners (single-box A\*, PIBT, cooperative A\*,
  agent-only joint A\*) + DP-optimal task assignment + recursive
  relocation + a 13-pass snapshot-based bounded-joint-A\* fallback
  chain with active-agent reduction and a gated divisor-bound
  heuristic.
- Clear roadmap for the remaining 38.
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
- "The snapshot-based fallback chain is the big late-stage win. We
  capture the best partial progress observed across all variants,
  then restart joint A\* from there — the residual subproblem is
  often dozens of times smaller than the original."
- "Tighter admissible heuristics aren't always better under weighted
  A\*. We learned this the hard way when adding the divisor bound —
  it sped up some levels but hurt others. The fix was a gate plus
  a separate fallback pass that forces the bound."

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
isolation. Joint A\* (≤6 agents) is the last-resort for tight
rotation puzzles where everything else deadlocks.

**Q: Why bitmask DP instead of the Hungarian algorithm?**
A: We need the assignment **and** the per-permutation orderings.
Bitmask DP lets us recover the path of choices cheaply for
secondary orderings. It also caps at 12 boxes per letter, which
fits 4096 states — comfortable for our budget.

**Q: How do you avoid live-locks in the variant loop?**
A: Per-variant 12-second deadline, plus a global per-task
attempt cap (`tasks.size() * (max_passes + 1) + 16`). A variant
can fail fast; the next one starts within 12 s at the latest.
The post-variant fallback chain is gated by an overall 27-second
soft deadline.

**Q: Why so many joint-A\* fallback passes (13!)?**
A: Each pass varies one knob — start state (initial vs. snapshot),
satisfied-box pruning, heuristic weight (W ∈ {3, 5, 8, 12, 15}),
active-agent reduction (on/off), divisor-bound heuristic (gated/
forced). Different residual subproblems are sensitive to different
combinations. The chain is ordered cheapest-first so easy levels
exit at pass 1; the deeper passes only fire when the budget
permits and the prior ones failed cleanly.

**Q: What's the role of "active-agent reduction"?**
A: Many levels have N > 6 total agents but only 2-3 are actually
relevant to the residual subproblem (other agents have already
delivered their boxes and have no numeric goal). Active-agent
reduction treats irrelevant agents as NoOp-only, dropping the
joint A\* branching factor from `9^N` to `9^N_active`. This
unlocked Dolor and GroupWon, which have 7-8 agents total but
only 2-3 active during the residual phase.

**Q: What's the divisor-bound heuristic about?**
A: The default joint A\* heuristic is `h_max = max over goals of
walls-only distance`. We added a second admissible bound,
`h_div = ceil(h_sum / N_eff)` — the per-agent average distance.
Both are valid lower bounds; the max of two lower bounds is also
a lower bound. But tighter heuristics aren't always better under
weighted A\* (W=3) — they can reorder the frontier in ways that
backfire on levels where `h_max` already gives strong guidance.
We gate the divisor bound so it only fires when `h_max < N_eff`
(weak guidance regime); passes 3c/3d force it for multi-box
low-h_max residuals like LoopBots.

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
| Snapshot fallback chain table | `ARCHITECTURE.md` §5.7 |
| Active-agent reduction explanation | `ARCHITECTURE.md` §5.8 |
| Gated divisor-bound heuristic | `ARCHITECTURE.md` §5.9 |
| Solve-count progression table | `searchclient_cpp_v2/README.md` "Benchmarks" |
| Per-level solve table | `solved_levels.md` |
| Per-level raw data (for graphs) | `benchmarks/results/v2-bench-complevels-r41.csv`<br>`benchmarks/results/v2-bench-complevels-2026-r41.csv`<br>`benchmarks/results/v2-bench-levels-r41.csv` |
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
