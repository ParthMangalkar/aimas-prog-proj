#!/usr/bin/env python3
"""Build and run benchmark workflows for searchclient_cpp_enhanced."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path
from typing import Iterable


WORKFLOWS = {
    "smoke": {
        "level_list": "benchmarks/enhanced-smoke-levels.txt",
        "output_name": "enhanced-smoke",
    },
    "targets": {
        "level_list": "benchmarks/enhanced-dense-targets.txt",
        "output_name": "enhanced-dense-targets",
    },
    "first10": {
        "level_list": "benchmarks/enhanced-complevels-2026-first10.txt",
        "output_name": "enhanced-complevels-2026-first10",
    },
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def run(command: list[str], env: dict[str, str] | None = None) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=repo_root(), env=env, check=True)


def build_client(build_dir: Path) -> Path:
    root = repo_root()
    run(["cmake", "-S", "searchclient_cpp_enhanced", "-B", str(build_dir)])
    run(["cmake", "--build", str(build_dir), "--", "-j2"])
    client = build_dir / "searchclient_cpp_enhanced"
    if not client.exists():
        raise FileNotFoundError(f"Expected enhanced client at {client}")
    return client


def workflow_commands(args: argparse.Namespace, client: Path) -> list[tuple[str, list[str]]]:
    base = [
        sys.executable,
        "benchmarks/run_all_levels.py",
        "--client",
        str(client),
        f"--algorithm={args.algorithm}",
        "--timeout",
        str(args.timeout),
        "--max-joint-actions",
        str(args.max_joint_actions),
        "--normalize",
    ]
    if args.profile:
        base.append("--profile")

    if args.workflow == "full":
        return [
            (
                "enhanced-complevels-full",
                base
                + [
                    "--level-root",
                    "complevels",
                    "--output-name",
                    "enhanced-complevels-full",
                ],
            ),
            (
                "enhanced-complevels-2026-full",
                base
                + [
                    "--level-root",
                    "complevels_2026",
                    "--output-name",
                    "enhanced-complevels-2026-full",
                ],
            ),
        ]

    workflow = WORKFLOWS[args.workflow]
    return [
        (
            workflow["output_name"],
            base
            + [
                "--level-list",
                workflow["level_list"],
                "--output-name",
                workflow["output_name"],
            ],
        )
    ]


def parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def update_solved_levels(csv_paths: Iterable[Path], output_path: Path) -> None:
    rows: list[dict[str, str]] = []
    for csv_path in csv_paths:
        with csv_path.open(encoding="utf-8", newline="") as handle:
            rows.extend(csv.DictReader(handle))
    if not rows:
        raise ValueError("No benchmark rows available to write solved_levels.md")

    folder_order = {"complevels": 0, "complevels_2026": 1}

    def row_key(row: dict[str, str]) -> tuple[int, str]:
        level = Path(row["level"].replace("\\", "/"))
        return (folder_order.get(level.parent.as_posix(), 99), level.stem)

    rows.sort(key=row_key)
    summary: dict[str, list[int]] = {}
    for row in rows:
        folder = Path(row["level"].replace("\\", "/")).parent.as_posix()
        solved = parse_bool(row.get("solved", "false"))
        counts = summary.setdefault(folder, [0, 0])
        counts[0] += 1 if solved else 0
        counts[1] += 1

    total_solved = sum(counts[0] for counts in summary.values())
    total_count = sum(counts[1] for counts in summary.values())
    lines = [
        "# Solved Levels",
        "",
        "Method: `searchclient_cpp_enhanced` with benchmark algorithm label `-enhanced`.",
        "",
        "## Summary",
        "",
        "| Level Folder | Solved | Total |",
        "|---|---:|---:|",
    ]
    for folder in sorted(summary, key=lambda item: folder_order.get(item, 99)):
        solved, total = summary[folder]
        lines.append(f"| `{folder}` | {solved} | {total} |")
    lines.extend(
        [
            f"| **Total** | **{total_solved}** | **{total_count}** |",
            "",
            "## Per-level results",
            "",
            "| # | Level Name | Level Folder | Solved (Yes/No) | Solve Time | Joint Actions Used | Method Used (Best Method Name, A*, etc.) |",
            "|---:|---|---|---|---:|---:|---|",
        ]
    )
    for index, row in enumerate(rows, start=1):
        level = Path(row["level"].replace("\\", "/"))
        solved = "Yes" if parse_bool(row.get("solved", "false")) else "No"
        wall = float(row.get("wall_seconds") or 0.0)
        actions = row.get("server_len", "")
        lines.append(
            f"| {index} | `{level.stem}` | `{level.parent.as_posix()}` | {solved} | "
            f"{wall:.3f}s | {actions} | Enhanced hierarchical solver (-enhanced) |"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {output_path}")


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workflow",
        choices=["smoke", "targets", "first10", "full"],
        default="smoke",
        help="Benchmark workflow to run.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("searchclient_cpp_enhanced/build"),
        help="Out-of-tree CMake build directory.",
    )
    parser.add_argument("--algorithm", default="-enhanced", help="Client algorithm label.")
    parser.add_argument("--timeout", type=int, default=60, help="Server timeout per level.")
    parser.add_argument("--max-joint-actions", type=int, default=20000)
    parser.add_argument("--profile", action="store_true", help="Enable AIMAS_PROFILE through run_all_levels.py.")
    parser.add_argument(
        "--update-solved-levels",
        action="store_true",
        help="After --workflow full, regenerate solved_levels.md from the full CSVs.",
    )
    parser.add_argument(
        "--env",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="Extra environment variables for the benchmark run.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    if args.update_solved_levels and args.workflow != "full":
        raise SystemExit("--update-solved-levels requires --workflow full")

    client = build_client((repo_root() / args.build_dir).resolve())
    env = os.environ.copy()
    for assignment in args.env:
        if "=" not in assignment:
            raise SystemExit(f"Invalid --env value: {assignment!r}")
        name, value = assignment.split("=", 1)
        env[name] = value

    csv_paths: list[Path] = []
    for output_name, command in workflow_commands(args, client):
        run(command, env=env)
        csv_paths.append(repo_root() / "benchmarks" / "results" / f"{output_name}.csv")

    if args.update_solved_levels:
        update_solved_levels(csv_paths, repo_root() / "solved_levels.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
