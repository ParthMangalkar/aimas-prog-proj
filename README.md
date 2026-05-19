# AIMAS Hospital Solver

This repository contains the current C++ AIMAS hospital-domain solver and the
supporting benchmark tooling used to validate it against the Java server.

The active implementation is a hybrid C++ solver centered on prioritized
planning, serial fallback planning, and bounded weighted A* fallback. It is not
the older pure graph-search client, and the dormant MAPF experiment code is not
linked into the default solver.

## Current status

The current verified baseline uses `searchclient_cpp -prioritized` through
`misc/server.jar`, normalized level input, and a 180 second timeout. It combines
the previous full benchmark with a recheck of every listed solved level and the
newly parser-fixed `complevels/rooMbA.lvl`.

| Level folder | Solved | Total |
|---|---:|---:|
| `levels` | 90 | 104 |
| `new_comp_levels` | 0 | 6 |
| `complevels` | 26 | 47 |

`AIMAS_LNS_REPAIR=1` enables an experimental last-ditch repair pass. It is not
default-on, but the tuned version currently solves one additional competition
level (`complevels/MArtians.lvl`) without regressing the 116-level solved
regression set, for an opt-in `complevels` count of 27 / 47.

`AIMAS_ENABLE_WINDOWED_REPLAN=1` enables an experimental full-joint
windowed/RHCR fallback after the normal prioritized pipeline fails. It remains
default-off: targeted testing on `BigSplit`, `Lily`, `MArtians`, `Minchia`, and
`ZOOM` produced no additional solves, so the next high-ROI direction is a true
decomposed MAPF/CBS or neighborhood repair adapter rather than more full-joint
window tuning.

`AIMAS_ENABLE_CBS_REPLAN=1` enables an experimental fixed-assignment CBS
fallback. It builds a CBS constraint tree over the existing per-agent box plans,
branches on body/box vertex conflicts and swap conflicts, and validates any
candidate with the normal server-style replay. This is now the cleanest
coordination foundation in the solver, but it is still default-off: targeted
testing on `BigSplit`, `MArtians`, and `ZOOM` produced no new solves because the
remaining failures are usually low-level box-delivery infeasibilities before CBS
can branch.

The solved-level source of truth is:

```text
searchclient_cpp/solved_levels.md
```

## Repository layout

| Path | Purpose |
|---|---|
| `searchclient_cpp/searchclient_cpp/main.cpp` | Active C++ solver implementation. Contains parsing, domain model, graph search, prioritized planner, serial fallback, and output protocol. |
| `searchclient_cpp/searchclient_cpp/CMakeLists.txt` | Builds the active `searchclient_cpp` executable and the compile-only `mapf_experiments` target. |
| `searchclient_cpp/searchclient_cpp/mapf/` | Dormant MAPF experiment code. It is compile-checked as `mapf_experiments` but not linked into the active solver. |
| `levels/` | Standard assignment levels. |
| `new_comp_levels/` | New competition levels. |
| `complevels/` | Competition levels. |
| `benchmarks/` | Benchmark runners, regression checker, smoke-level list, and benchmark README. |
| `misc/` | Java server and legacy/support files. |
| `other_misc/` | Agent handoff, implementation-change notes, and work context. |
| `direct-tests/` | Direct test assets and folder-specific README. |
| `ARCHITECTURE.md` | Detailed current architecture and algorithm explanation. |
| `other_misc/IMPLEMENTATION_CHANGES.md` | Detailed record of what changed from the starting point. |
| `other_misc/AGENT_HANDOFF.md` | Handoff notes for future improvement sessions. |

Folder-specific READMEs are intentionally kept in `benchmarks/`, `misc/`, and
`direct-tests/`. Solver documentation is centralized in this root README,
`ARCHITECTURE.md`, and the project notes under `other_misc/`.

## Build

From the repository root:

```bash
cmake -S searchclient_cpp -B searchclient_cpp/build-darwin -DCMAKE_BUILD_TYPE=Release
cmake --build searchclient_cpp/build-darwin --target searchclient_cpp mapf_experiments -j2
```

The active executable is normally:

```text
searchclient_cpp/build-darwin/searchclient_cpp/searchclient_cpp
```

Use `misc/server.jar` for validation. The C++ server sources currently contain
pre-existing conflict markers and are not the current validation path.

## Run one level

Example with the current default solver path:

```bash
java -jar misc/server.jar \
  -l levels/MAExample.lvl \
  -c "./searchclient_cpp/build-darwin/searchclient_cpp/searchclient_cpp -prioritized" \
  -t 180
```

The client writes the expected server handshake, computes a plan, then emits one
joint action per timestep. Actions for multiple agents are separated with `|`.

## Supported strategy flags

The active `main.cpp` supports these strategy flags:

| Flag | Meaning |
|---|---|
| no flag | Try prioritized planning first, then serial fallback, then bounded weighted A*(5), then smart weighted A*. |
| `-prioritized`, `-pp` | Prioritized planner path. If prioritized and serial fallback fail, a bounded weighted A*(5) repair is attempted. |
| `-prioritized-fallback`, `-pp-fallback` | Try prioritized planning and serial fallback, then fall back to the selected best-first frontier if those stages fail. |
| `-bfs` | Breadth-first graph search. |
| `-dfs` | Depth-first graph search. |
| `-astar` | A* with the base heuristic. |
| `-wastar [weight]` | Weighted A* with the base heuristic. Default weight is 5. |
| `-greedy` | Greedy best-first search with the base heuristic. |
| `-smart`, `-smart-greedy` | Greedy best-first search with the smart heuristic. |
| `-smart-wastar [weight]` | Weighted A* with the smart heuristic. Default weight is 3. |
| `-greedy-goalcount` | Greedy best-first search using unsatisfied-goal count. |
| `-astar-goalcount` | A* using unsatisfied-goal count. |

Do not use stale MAPF flags such as `-mapf`, `-ecbs`, `-alns`, or `-full` as
current solver entry points. The MAPF sources compile as an isolated experiment
target, but those flags are not part of the active `main.cpp` control flow.

## Current prioritized solver flow

The active `-prioritized` / `-pp` path is:

1. Parse the level.
2. Run a basic goal-feasibility gate.
3. Try prioritized planning with reservation tables.
4. For each complete prioritized candidate, validate it with local server-style
   replay before emitting it.
5. If replay fails, retry with committed-world box state and/or relaxed
   reservations.
6. If prioritized planning fails, try serial task fallback.
7. If `AIMAS_ENABLE_CBS_REPLAN=1`, try the experimental fixed-assignment CBS
   fallback.
8. If CBS is disabled or fails, try bounded weighted A*(5), using a larger bounded
   repair budget for small joint instances.
9. If `AIMAS_ENABLE_WINDOWED_REPLAN=1`, try the experimental windowed
   full-joint fallback.
10. If no valid plan is found, exit without printing an invalid plan.

No-flag mode uses the same initial stages, but can continue to final smart
weighted A* after bounded repair fails. `-prioritized-fallback` / `-pp-fallback`
uses prioritized planning and serial fallback, then skips bounded repair and
falls back to the selected best-first frontier.

Important current implementation features:

1. `AIMAS_PROFILE=1` enables profile events on stderr.
2. BFS distance grids are cached and reused.
3. `CommittedWorld` gives replay-invalid prioritized candidates a stricter world
   state retry path.
4. `ReservationPolicy::Relaxed` is a fallback retry, not the default.
5. `basic_goal_feasibility` fails fast on simple impossible metadata cases.
6. Graph-search fallback has time, expansion, and branch-factor guards.
7. Medium-instance serial fallback is default-on for levels with many boxes but
   few active box goals; set `AIMAS_ENABLE_MEDIUM_SERIAL=0` to disable it.
8. `AIMAS_LNS_REPAIR=1` adds an experimental last-ditch repair pass after
   normal prioritized attempts fail; keep it opt-in unless a full benchmark
   proves it is a strict default improvement.
9. `AIMAS_ENABLE_WINDOWED_REPLAN=1` adds an experimental windowed full-joint
   fallback. Use `AIMAS_WINDOWED_CLASSIC_BUDGET_S` in experiments to reserve
   time for it on levels where classic prioritized planning would otherwise
   consume the full server timeout.
10. `AIMAS_ENABLE_CBS_REPLAN=1` adds an experimental fixed-assignment CBS
   fallback. Use `AIMAS_CBS_CLASSIC_BUDGET_S` to reserve time for it, and tune
   `AIMAS_CBS_BUDGET_S`, `AIMAS_CBS_LOW_LEVEL_BUDGET_S`,
   `AIMAS_CBS_MAX_NODES`, `AIMAS_CBS_VARIANTS`, `AIMAS_CBS_MAX_AGENTS`, and
   `AIMAS_CBS_MAX_BOXES` for experiments.

## Benchmarking

Fast smoke benchmark:

```bash
python3 benchmarks/run_all_levels.py \
  --level-list benchmarks/smoke-levels.txt \
  --algorithm=-prioritized \
  --timeout 90 \
  --output-name smoke-darwin \
  --normalize \
  --profile
```

Regression check against solved levels present in the CSV:

```bash
python3 benchmarks/check_solved_regressions.py benchmarks/results/smoke-darwin.csv
```

Full grouped benchmark:

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

Strict full regression check:

```bash
python3 benchmarks/check_solved_regressions.py \
  benchmarks/results/solved-all-current.csv \
  --require-all
```

Failure triage:

```bash
python3 benchmarks/triage_logs.py benchmarks/results/solved-all-current-logs
```

Benchmark outputs are written under:

```text
benchmarks/results/
```

## Regression policy

Future solver changes should not lose levels listed in:

```text
searchclient_cpp/solved_levels.md
```

Minimum validation for a solver change:

1. Build `searchclient_cpp` and `mapf_experiments`.
2. Run the affected target levels.
3. Run the smoke benchmark.
4. Run `benchmarks/check_solved_regressions.py` on the smoke CSV.

For planner-control-flow changes, also run the full `complevels` benchmark. If
the solved set changes after a full grouped benchmark, regenerate
`searchclient_cpp/solved_levels.md` from the benchmark CSV.

## More documentation

| Document | Contents |
|---|---|
| `ARCHITECTURE.md` | Detailed active solver architecture and algorithms. |
| `other_misc/IMPLEMENTATION_CHANGES.md` | What changed from the previous starting point. |
| `other_misc/AGENT_HANDOFF.md` | Operational handoff for the next improvement session. |
| `benchmarks/README.md` | Benchmark-specific commands and historical Java/C++ comparison notes. |
