# Enhancement Plan

This plan targets the remaining high-return work after the standalone
`searchclient_cpp_enhanced` implementation. Current benchmark status for the
enhanced solver is recorded in `solved_levels.md`:

| Level Folder | Solved | Total |
|---|---:|---:|
| `complevels` | 27 | 47 |
| `complevels_2026` | 35 | 69 |
| **Total** | **62** | **116** |

> Latest run includes the four research-driven improvements listed in
> the section below. Baseline before this work was 56/116; the new run
> is 62/116 with **zero regressions** (6 levels unlocked).

## Research-driven improvements (latest round)

After studying the papers in `research_papers/` (LMAPF: PIBT + Guided
Online Planning; A*+: grid compression) and running verbose-mode failure
diagnostics on the failing levels, four root causes were addressed.
All changes are default-on with environment-variable escape hatches.

**Result: 56/116 → 62/116 levels solved (+6, no regressions).**

Newly unlocked levels:
- `complevels/PFarthing` — PIBT final-agent coordinator
- `complevels/doggy`
- `complevels_2026/TheDevil` — metadata feasibility fix
- `complevels_2026/MAmaMASS`
- `complevels_2026/MArachnid`
- `complevels_2026/moveMAcat`

| # | Improvement | Source | Impact |
|---|---|---|---|
| 1 | Per-cell metadata feasibility | failure-mode analysis (7 levels forfeited at 0.0s) | `TheDevil` unlocked; pipeline now reached on `EpicfAIl`, `Lily`, `TheGate`, `ZOOM`, `QRscammer`, `amogus`. |
| 2 | PIBT-based final-agent coordinator (`complete_agent_goals_pibt`) | Okumura et al. 2019 (`research_papers/LMAPF/LMAPF.pdf`, §2.1) | `PFarthing` solved in 0.017s; targets all "blocked_by_agent" symmetry failures. |
| 3 | Anti-thrash blacklist in `relocate_blocker_and_deliver` | failure-mode analysis (`BStar`, `MAceship` oscillation) | Eliminates infinite back-and-forth, frees time budget for other attempts. |
| 4 | Ranked alternative-agent retry in `try_task_sequence` | failure-mode analysis (`ComMAndos`, `Nej`, `Apdo`) | Apdo delivers `E` (agent 5) and `I` (agent 8) where agent 0 originally failed. Unlocked `MAmaMASS`, `MArachnid`, `moveMAcat`, `doggy`. |

PIBT is implemented strictly as a rollback-safe coordinator inserted
between `complete_agent_goals_serial` and the bounded joint A* fallback,
using a box-aware BFS heuristic (boxes treated as walls except the goal
cell). It is not a planner for box-delivery, only the final agent-only
phase. Two new flags:

| Flag | Default | Purpose |
|---|---:|---|
| `AIMAS_ENHANCED_PIBT` | `1` (on) | Enable PIBT final-agent coordinator. Set to `0` to disable. |
| `AIMAS_ENHANCED_PIBT_T` | `400` | Maximum PIBT timesteps per call. |

## Guiding principles

1. Keep new risky strategies behind opt-in flags until a full benchmark proves
   they improve solved count or runtime without regressions.
2. Validate every new planner by replaying through `State::apply_joint`; never
   print a plan that the internal validator rejects.
3. Optimize for new solves on dense competition levels, not only faster solves
   on easy instances.
4. Preserve the current default baseline while adding portfolio candidates.

## High-return implementation tasks

| Priority | Task | Why it matters | Concrete implementation | Success criteria |
|---:|---|---|---|---|
| 1 | CBS-style box-delivery repair | Current failures are mostly dense box-delivery infeasibilities, not parsing or final output issues. | Add a local conflict-based search layer over 2-4 agents and the boxes in a bounded component. Use the existing `LocalMultiBoxPlanner` state model, but branch on agent/box vertex conflicts and add constraints to low-level replans. Start opt-in as `AIMAS_ENHANCED_CBS_BOX_REPAIR=1`. | Solve at least one of `complevels/AMC.lvl`, `complevels_2026/BoxBender.lvl`, `complevels_2026/brAIn.lvl`, `complevels_2026/CudBSlvd.lvl`, or `complevels_2026/MASaos.lvl` without lowering the default first-10 sample. |
| 2 | Robust final-agent-goal recovery | Opt-in two-agent repair can get `AMC` past more box goals, then final agent-goal movement fails. | Extend final-goal recovery to plan with temporary evacuation goals for all non-target agents, including agents with no final goal. Add diagnostics that report which agent target is unreachable and whether the blocker is an agent, box, or wall topology. | With `AIMAS_ENHANCED_TWO_AGENT_REPAIR=1`, either solve `AMC` or produce a precise final-goal failure reason in the log. |
| 3 | Recursive relocation graph | Current relocation is shallow and can move blockers to places that later block future tasks. | Replace greedy relocation loops with a small relocation dependency graph: blocker -> candidate parking -> blockers of that relocation. Score parking by future rough paths, goal cells, articulation/corridor degree, and agent-goal routes. Keep default depth 0 until benchmarked. | Improve at least one dense relocation-heavy level (`AMC`, `BoxBender`, `PokeNOM`, `Medibots`) or reduce failed-level wall time by 20% without solved-count loss. |
| 4 | Component-aware task decomposition | Current task ordering is global and serial; dense components need local ordering and batch repair. | Build connected components over boxes/goals/passages. Solve independent components separately; for dense components, run a portfolio of matched-task orders and local repair before moving to other regions. | Increase `complevels_2026` solved count by at least 2 or reduce average failed wall time on hard levels. |
| 5 | Default-worthy portfolio controller | Several features are implemented but gated because they help only some cases. | Add a lightweight classifier using level stats: number of agents, active box goals, box density, connected components, corridor width, and color multiplicity. Use it to choose which opt-in repairs get time budget. | Enable at least one currently opt-in feature by default for a level class with no regression in the 116-level benchmark. |
| 6 | Benchmark regression harness | Full manual benchmarks are slow and easy to lose. | Add a checked-in level-list set for smoke, dense targets, and first-10 `complevels_2026`. Add a small script or documented command group that runs the enhanced solver and updates `solved_levels.md` from CSV output. | A developer can run smoke, target, and full benchmark workflows without reconstructing commands. |

## Suggested order of work

1. Start with **CBS-style box-delivery repair** because it is the only remaining
   change likely to produce a large solved-count jump.
2. Use `AMC` as the first debugging target because current logs show meaningful
   partial progress and a final-goal bottleneck after extra repair work.
3. Use `BoxBender` and `brAIn` as guardrails: if a strategy slows them without
   progress, gate or tighten its budget.
4. After any major repair layer, run:

```bash
build_dir="$HOME/.copilot/session-state/24cf2cff-dea1-46d7-9a75-140ff297d83d/files/searchclient_cpp_enhanced-build"
cmake -S searchclient_cpp_enhanced -B "$build_dir"
cmake --build "$build_dir" -- -j2

python3 benchmarks/run_all_levels.py \
  --client "$build_dir/searchclient_cpp_enhanced" \
  --level-root complevels_2026 \
  --algorithm=-enhanced \
  --timeout 60 \
  --max-joint-actions 20000 \
  --output-name enhanced-complevels-2026-full \
  --normalize \
  --profile
```

## Current opt-in features to revisit

| Feature flag | Current status | Next step |
|---|---|---|
| `AIMAS_ENHANCED_CBS_BOX_REPAIR=1` | Implemented, gated. Adds bounded conflict-aware local box-delivery repair. | Benchmark on dense targets and first-10 `complevels_2026` before considering broader portfolio use. |
| `AIMAS_ENHANCED_NEIGHBORHOOD_REPAIR=1` | Implemented, gated. Stable but did not improve target solved count. | Reuse as low-level search inside CBS/local component repair. |
| `AIMAS_ENHANCED_DENSITY_ORDERING=1` | Implemented, gated. Added extra variants but no default win. | Feed into component-aware ordering instead of global variant enumeration. |
| `AIMAS_ENHANCED_COMPONENT_ORDERING=1` | Implemented, gated. Adds component-grouped task variants for decomposition experiments. | Benchmark on dense components before enabling by classifier. |
| `AIMAS_ENHANCED_TWO_AGENT_REPAIR=1` | Implemented, gated. Useful for dense local repairs; `AMC` is now solved by stronger final-goal recovery after local delivery. | Benchmark with CBS repair on dense targets before enabling by classifier. |
| `AIMAS_ENHANCED_RELOCATION_DEPTH` | Implemented, default 0. Depth > 0 slowed hard failures. | Replace with relocation graph and better parking scoring before enabling. |
| `AIMAS_ENHANCED_RELOCATION_GRAPH=1` | Implemented, gated. Adds parking penalties for final-agent routes and low-degree cells. | Pair with bounded relocation depth only after dense-target benchmarks. |
