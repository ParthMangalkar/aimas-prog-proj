#!/usr/bin/env python3
"""Check a benchmark CSV for regressions against solved_levels.md."""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def read_solved_levels(path: Path) -> set[str]:
    solved: set[str] = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.search(r"\b([A-Za-z0-9_.-]+\.lvl)\b", line)
        if match:
            solved.add(match.group(1))
    return solved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="Benchmark CSV from run_all_levels.py.")
    parser.add_argument(
        "--solved-levels",
        type=Path,
        default=repo_root() / "searchclient_cpp" / "solved_levels.md",
        help="Solved-level source of truth.",
    )
    parser.add_argument(
        "--require-all",
        action="store_true",
        help="Fail if any solved level is missing from the benchmark CSV.",
    )
    args = parser.parse_args()

    expected = read_solved_levels(args.solved_levels)
    seen: dict[str, bool] = {}
    with args.csv_path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            level_name = row["level"].replace("\\", "/").split("/")[-1]
            if level_name not in expected:
                continue
            solved = row.get("solved", "").lower() == "true"
            seen[level_name] = seen.get(level_name, False) or solved

    missing = sorted(expected - set(seen))
    regressed = sorted(level for level, solved in seen.items() if not solved)

    if args.require_all and missing:
        print("Missing solved levels from benchmark CSV:")
        for level in missing:
            print(f"  {level}")
    if regressed:
        print("Regressed solved levels:")
        for level in regressed:
            print(f"  {level}")

    if regressed or (args.require_all and missing):
        return 1

    print(f"No regressions among {len(seen)} checked solved levels.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
