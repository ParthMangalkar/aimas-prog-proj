# Benchmarking Java vs C++

This folder contains a direct benchmark harness for comparing the C++ client against the original Java client logic.

The benchmark runs both clients directly on the same level input instead of going through `server.jar`. That keeps the comparison focused on search behavior and avoids the old Java client's multi-agent output formatting bug.

## What It Benchmarks

- `-bfs`
- `-dfs`
- `-astar`
- `-wastar 5`
- `-greedy`
- `-greedy-goalcount`
- `-astar-goalcount`

The default curated level set is stored in [default-levels.txt](C:\Users\Lenovo\Desktop\semester-2\MAS\new_cpp\aimas-warmup-assignment\benchmarks\default-levels.txt).

## Prerequisites

- a built C++ client
- Java and `javac` available on `PATH`
- a git clone of this repository

The script restores the original Java reference client from commit history into `benchmarks/.generated/reference-java/` and compiles it automatically.

## Run The Benchmark

From the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File ".\benchmarks\run-benchmark.ps1"
```

If you want to point at a specific C++ executable:

```powershell
powershell -ExecutionPolicy Bypass -File ".\benchmarks\run-benchmark.ps1" -CppClient ".\direct-tests\bin\searchclient_cpp.exe"
```

If you want a custom timeout per run:

```powershell
powershell -ExecutionPolicy Bypass -File ".\benchmarks\run-benchmark.ps1" -TimeoutSeconds 90
```

If you want a custom level list file:

```powershell
powershell -ExecutionPolicy Bypass -File ".\benchmarks\run-benchmark.ps1" -LevelList ".\benchmarks\default-levels.txt"
```

## Outputs

The script writes:

- `benchmarks/results/latest.csv`
- `benchmarks/results/latest.md`

Each row records:

- client (`java` or `cpp`)
- level
- algorithm
- solved / timeout status
- solution length
- expanded states
- frontier size
- generated states
- reported search time
- wall-clock time

The Markdown summary also includes a Java-vs-C++ comparison table for matching runs.

## Notes

- The benchmark uses the original Java client source recovered from commit `60ada58`.
- The Java reference is only used for comparison. The active deliverable client remains the C++ client used with the Java server.
- Running benchmarks directly through stdin keeps the search comparison honest even though the old Java client still has the known joint-action print bug.

## Cross-platform all-level benchmark

On macOS/Linux, use the Python runner with the Java server:

```bash
cmake -S searchclient_cpp -B searchclient_cpp/build-darwin -DCMAKE_BUILD_TYPE=Release
cmake --build searchclient_cpp/build-darwin --target searchclient_cpp
python3 benchmarks/run_all_levels.py --level-root complevels --algorithm=-prioritized --timeout 180 --normalize --profile
python3 benchmarks/triage_logs.py benchmarks/results/baseline-darwin-logs
```

`--profile` enables `AIMAS_PROFILE=1`, which adds `[profile] key=value` timing
lines to each log without changing the protocol sent to the server.

For a faster smoke run:

```bash
python3 benchmarks/run_all_levels.py --level-list benchmarks/smoke-levels.txt --algorithm=-prioritized --timeout 90 --output-name smoke-darwin --normalize --profile
python3 benchmarks/check_solved_regressions.py benchmarks/results/smoke-darwin.csv
```

For a full 47-level regression gate, add `--require-all` to make the checker
fail if any level from `searchclient_cpp/solved_levels.md` is absent or lost.
