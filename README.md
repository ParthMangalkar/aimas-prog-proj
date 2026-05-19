# AIMAS Hospital Solver — `searchclient_cpp_v2`

C++17 search client for the DTU 02285 **AIMAS Hospital
(Multi-Agent Path Finding with Boxes)** domain, together with the
Java validation server and the benchmark tooling used to validate it.

> **Status (r23):** **74 / 116** levels solved — 29 / 47 on
> `complevels` and 45 / 69 on `complevels_2026`. +12 over the
> previous in-house single-file C++ solver (62 / 116), in ~25 %
> less code.

---

## Table of contents

- [What's in this repository](#whats-in-this-repository)
- [Quick start](#quick-start)
- [Benchmarks](#benchmarks)
- [Repository layout](#repository-layout)
- [Documentation](#documentation)
- [Course context](#course-context)

---

## What's in this repository

| Folder / file | Purpose |
|---|---|
| `searchclient_cpp_v2/` | The **only** active solver. Modular C++17 implementation of the search client. |
| `misc/server.jar` | DTU/AIMAS validation server used to score every plan. |
| `complevels/` | Original competition level set (47 levels). |
| `complevels_2026/` | 2026 competition level set (69 levels). |
| `levels/`, `new_comp_levels/` | Additional / starter / experimental level sets. |
| `direct-tests/` | Direct test assets. |
| `benchmarks/` | Python benchmark runner + per-round CSV/Markdown/log results. |
| `research_papers/` | Reference PDFs (PIBT, LMAPF, A\*+) cited by the solver. |
| `ARCHITECTURE.md` | Full architectural deep-dive of the v2 solver. |
| `searchclient_cpp_v2/README.md` | v2-specific README (Quick Start, Algorithms, Troubleshooting, Roadmap). |
| `solved_levels.md` | Source-of-truth per-level results table. |
| `PPT_HELP.md` | Presentation/video preparation guide. |

There is no `searchclient_cpp`, `searchclient_cpp_enhanced`, or
`searchclient_java` folder anymore — they were intermediate code
paths that were fully superseded by `searchclient_cpp_v2` and have
been removed (see commits `8536c81` and `52af1b5`).

---

## Quick start

```bash
# 1. Build (Release)
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

You should see something like:

```
[v2] Parsed level 'donut' 13x15 with 4 agents.
[v2] Plan length 122 joint actions, found in 0.002s.
[server][info] Level solved: Yes.
```

### Requirements

| Tool         | Version          | Used for                            |
|--------------|------------------|-------------------------------------|
| C++ compiler | C++17            | Building the client                 |
| CMake        | ≥ 3.10           | Build configuration                 |
| Java         | ≥ 11             | Running `misc/server.jar`           |
| Python       | ≥ 3.9 (optional) | `benchmarks/run_all_levels.py`      |

Tested on macOS (Apple Clang 14+) and Linux (GCC 9+). No external
C++ libraries.

---

## Benchmarks

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

| Build                                       | complevels | complevels_2026 | Total       |
|---------------------------------------------|-----------:|----------------:|-------------|
| Legacy single-file C++ baseline             |          – |               – | 56 / 116    |
| Legacy single-file C++ (enhanced)           |    27 / 47 |        35 / 69  | 62 / 116    |
| **`searchclient_cpp_v2` (r23, current)**    |  **29/47** |       **45/69** | **74/116**  |

Per-round progression (v2 baseline r4 → current r23) and per-level
results live in `searchclient_cpp_v2/README.md` and `solved_levels.md`
respectively. Per-level logs and CSVs are under
`benchmarks/results/v2-bench-*`.

---

## Repository layout

```
aimas-prog-proj/
├── README.md                       ← (this file)
├── ARCHITECTURE.md                 # v2 architectural deep-dive
├── PPT_HELP.md                     # presentation/video guide
├── solved_levels.md                # source-of-truth solve table
│
├── searchclient_cpp_v2/            # the solver
│   ├── README.md                   # v2-specific README
│   ├── CMakeLists.txt
│   ├── include/aimas/              # core, parser, topology, single_box, pibt, solver
│   ├── src/                        # implementations of each header
│   └── tests/test_main.cpp         # v2_tests (5 unit tests)
│
├── complevels/                     # 47 levels (original competition)
├── complevels_2026/                # 69 levels (2026 competition)
├── levels/                         # starter / class levels
├── new_comp_levels/                # additional experimental levels
│
├── misc/
│   ├── server.jar                  # AIMAS validation server
│   ├── README.md                   # course-provided server notes
│   ├── prog_proj_assignment.pdf    # the assignment brief
│   └── ...                         # screenshots, heuristic explanation
│
├── benchmarks/
│   ├── run_all_levels.py           # Python benchmark runner
│   └── results/                    # CSV / Markdown / per-level logs
│
├── research_papers/                # cited algorithm references (PDFs)
│   ├── AStar_Plus/
│   └── LMAPF/
│
└── direct-tests/                   # direct test assets
```

---

## Documentation

| Document | Purpose |
|---|---|
| `searchclient_cpp_v2/README.md` | v2-specific Quick Start, Algorithms, Configuration, Troubleshooting, Roadmap, References. **Start here for using the solver.** |
| `ARCHITECTURE.md` | Architectural deep-dive: pipeline, module dependency graph, domain model, joint-action accounting, planner internals, transactional invariants. **Start here for understanding how it works.** |
| `solved_levels.md` | Per-level solve table (status, wall-time, joint-action count). Source of truth, regenerated from the latest benchmark CSVs. |
| `PPT_HELP.md` | Slide-by-slide presentation outline, headline numbers, talking points, Q&A bank, video recording tips. **Use this for the submission video/deck.** |
| `benchmarks/README.md` | Benchmark-runner-specific options and historical notes. |
| `direct-tests/README.md` | Notes on direct-test assets. |
| `misc/README.md` | Course-provided notes on the Java server. |

---

## Course context

- Course: **DTU 02285 — Artificial Intelligence and Multi-Agent Systems.**
- Domain spec & API: `misc/prog_proj_assignment.pdf`, `misc/README.md`.
- All client-server protocol details and validation are owned by
  `misc/server.jar` (provided with the course); we don't ship or
  modify a custom server.
- Algorithm references: PDFs in `research_papers/` — Okumura et al.
  2019 (PIBT), Silver 2005 (Cooperative A\*), Sharon et al. 2015
  (CBS), Stern 2019 (MAPF survey), Cohen et al. 2018
  (Anytime Bounded-Suboptimal).
