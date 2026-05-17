#!/usr/bin/env python3
"""Classify AIMAS benchmark logs by failure mode."""

from __future__ import annotations

import argparse
import csv
import re
from collections import Counter
from pathlib import Path


def classify(text: str) -> str:
    if re.search(r"Level solved:\s+Yes|Solved in\s+[0-9,]+\s+steps", text):
        return "solved"
    if "Client timed out" in text or "timed out" in text.lower():
        return "timeout"
    if "Expected header" in text or "ParseException" in text or "[server][error]" in text:
        return "parse_error"
    if "Rejected prioritized variant" in text and "invalid action" in text:
        return "invalid_replay"
    if "failed DeliverBox" in text:
        return "deliver_box_failed"
    if "failed ReachCell" in text:
        return "reach_cell_failed"
    if "Search budget exhausted" in text:
        return "search_budget"
    if "Unable to solve level" in text:
        return "unable_to_solve"
    return "failed"


def iter_logs(paths: list[Path]) -> list[Path]:
    logs: list[Path] = []
    for path in paths:
        if path.is_dir():
            logs.extend(sorted(path.rglob("*.log")))
        elif path.exists():
            logs.append(path)
    return logs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path, help="Log files or directories.")
    parser.add_argument("--csv", type=Path, help="Optional per-log CSV output.")
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    counts: Counter[str] = Counter()
    for log in iter_logs(args.paths):
        text = log.read_text(encoding="utf-8", errors="replace")
        category = classify(text)
        counts[category] += 1
        rows.append({"log": str(log), "category": category})

    for category, count in counts.most_common():
        print(f"{category},{count}")

    if args.csv:
        with args.csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["log", "category"])
            writer.writeheader()
            writer.writerows(rows)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
