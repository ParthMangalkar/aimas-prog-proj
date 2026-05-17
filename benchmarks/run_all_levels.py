#!/usr/bin/env python3
"""Cross-platform benchmark runner for the AIMAS C++ client.

The existing all-level runner is PowerShell/Windows-oriented. This script runs
the same client through the Java server, works on macOS/Linux, writes per-level
logs, and emits CSV/Markdown summaries that are easy to compare between runs.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Iterable


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_path(root: Path, value: str | None) -> Path | None:
    if not value:
        return None
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def default_client(root: Path) -> Path:
    candidates = [
        root / "searchclient_cpp" / "build-darwin" / "searchclient_cpp" / "searchclient_cpp",
        root / "searchclient_cpp" / "build" / "searchclient_cpp" / "searchclient_cpp",
        root / "searchclient_cpp" / "build" / "searchclient_cpp" / "Release" / "searchclient_cpp.exe",
        root / "direct-tests" / "bin" / "searchclient_cpp.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError("Could not find a built searchclient_cpp binary; pass --client.")


def collect_levels(root: Path, level_roots: list[str], level_list: str | None, limit: int) -> list[Path]:
    levels: list[Path] = []
    if level_list:
        list_path = resolve_path(root, level_list)
        assert list_path is not None
        for raw in list_path.read_text(encoding="utf-8-sig").splitlines():
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            resolved = resolve_path(root, line)
            if resolved is not None:
                levels.append(resolved)
    else:
        for level_root in level_roots:
            resolved_root = resolve_path(root, level_root)
            if resolved_root is None:
                continue
            levels.extend(sorted(resolved_root.rglob("*.lvl")))

    unique = sorted({level.resolve() for level in levels})
    if limit > 0:
        unique = unique[:limit]
    return unique


def normalize_level(original: Path, temp_dir: Path) -> Path:
    text = original.read_text(encoding="utf-8-sig")
    lines = text.splitlines()
    normalized: list[str] = []
    in_grid = False
    for raw in lines:
        stripped = raw.strip()
        if stripped.startswith("#"):
            in_grid = stripped in {"#initial", "#goal"}
            normalized.append(stripped)
            continue
        if in_grid:
            # Keep leading spaces and blank rows in level grids, but drop
            # trailing whitespace that trips stricter parsers.
            normalized.append(raw.rstrip())
        elif stripped:
            normalized.append(raw.strip())

    target = temp_dir / original.name
    target.write_text("\n".join(normalized) + "\n", encoding="utf-8")
    return target


def parse_output(text: str) -> dict[str, object]:
    solved = bool(re.search(r"Level solved:\s+Yes|Solved in\s+[0-9,]+\s+steps", text))
    server_len = None
    client_len = None
    expanded = None
    frontier = None
    generated = None
    search_seconds = None

    if solved:
        server_match = re.search(r"(?:Actions used:|Solved in)\s+([0-9,]+)", text)
        if server_match:
            server_len = int(server_match.group(1).replace(",", ""))

    client_match = re.search(r"Found solution of length\s+([0-9,]+)", text)
    if client_match:
        client_len = int(client_match.group(1).replace(",", ""))

    status_matches = re.findall(
        r"#Expanded:\s*([0-9,]+),\s*#Frontier:\s*([0-9,]+),\s*#Generated:\s*([0-9,]+),\s*Time:\s*([0-9.]+)\s*s",
        text,
    )
    if status_matches:
        last = status_matches[-1]
        expanded = int(last[0].replace(",", ""))
        frontier = int(last[1].replace(",", ""))
        generated = int(last[2].replace(",", ""))
        search_seconds = float(last[3])

    return {
        "solved": solved,
        "server_len": server_len,
        "client_len": client_len,
        "expanded": expanded,
        "frontier": frontier,
        "generated": generated,
        "search_seconds": search_seconds,
    }


def classify_failure(text: str, timeout: bool) -> str:
    if timeout:
        return "timeout"
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
    if "Unable to solve level" in text:
        return "unable_to_solve"
    if "Search budget exhausted" in text:
        return "search_budget"
    return "failed"


def safe_name(level: Path, algorithm: str) -> str:
    algo = re.sub(r"[^A-Za-z0-9_.-]+", "_", algorithm.strip() or "default")
    return f"{level.parent.name}_{level.name}__{algo}.log"


def write_markdown(path: Path, rows: list[dict[str, object]], timeout_seconds: int, max_joint_actions: int) -> None:
    solved = sum(1 for row in rows if row["solved"])
    timed_out = sum(1 for row in rows if row["timeout"])
    over_cap = sum(1 for row in rows if row["over_cap"])
    failed = len(rows) - solved
    avg_wall = sum(float(row["wall_seconds"]) for row in rows) / max(1, len(rows))
    solved_lengths = [int(row["server_len"]) for row in rows if row["server_len"] not in ("", None)]
    avg_len = sum(solved_lengths) / len(solved_lengths) if solved_lengths else 0.0

    lines = [
        "# All-Level Benchmark",
        "",
        f"Generated: {dt.datetime.now().isoformat(timespec='seconds')}",
        "",
        f"- Timeout per run: {timeout_seconds} seconds",
        f"- Joint-action cap for reporting: {max_joint_actions}",
        f"- Runs: {len(rows)}",
        "",
        "## Summary",
        "",
        "| Solved | Timeout | Over cap | Failed | Avg Wall s | Avg Solution |",
        "|---:|---:|---:|---:|---:|---:|",
        f"| {solved} / {len(rows)} | {timed_out} | {over_cap} | {failed} | {avg_wall:.3f} | {avg_len:.2f} |",
        "",
        "## Unsolved Or Timed Out",
        "",
        "| Level | Algorithm | Category | Timeout | Server Len | Client Len | Wall s | Log |",
        "|---|---|---|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        if row["solved"] and not row["over_cap"]:
            continue
        lines.append(
            f"| {row['level']} | {row['algorithm']} | {row['category']} | {row['timeout']} | "
            f"{row['server_len']} | {row['client_len']} | {float(row['wall_seconds']):.3f} | {row['log']} |"
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_benchmark(args: argparse.Namespace) -> int:
    root = repo_root()
    client = resolve_path(root, args.client) if args.client else default_client(root)
    server_jar = resolve_path(root, args.server_jar) or (root / "misc" / "server.jar")
    assert server_jar is not None
    level_roots = args.level_root or ["complevels"]
    levels = collect_levels(root, level_roots, args.level_list, args.limit)
    if not levels:
        raise ValueError("No levels selected.")

    output_dir = root / "benchmarks" / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir = output_dir / f"{args.output_name}-logs"
    if log_dir.exists():
        shutil.rmtree(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, object]] = []
    env = os.environ.copy()
    if args.profile:
        env["AIMAS_PROFILE"] = "1"

    algorithms = args.algorithm or ["-prioritized"]
    with tempfile.TemporaryDirectory(prefix="aimas-levels-") as temp:
        temp_dir = Path(temp)
        for level in levels:
            run_level = normalize_level(level, temp_dir) if args.normalize else level
            for algorithm in algorithms:
                log_path = log_dir / safe_name(level, algorithm)
                client_command = shlex.quote(str(client))
                if algorithm.strip():
                    client_command = f"{client_command} {algorithm.strip()}"
                command = [
                    "java",
                    "-jar",
                    str(server_jar),
                    "-l",
                    str(run_level),
                    "-c",
                    client_command,
                    "-t",
                    str(args.timeout),
                ]
                started = time.monotonic()
                timed_out = False
                try:
                    completed = subprocess.run(
                        command,
                        cwd=root,
                        env=env,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        timeout=args.timeout + 10,
                        check=False,
                    )
                    output = completed.stdout
                    exit_code = completed.returncode
                except subprocess.TimeoutExpired as exc:
                    timed_out = True
                    output = exc.stdout or ""
                    exit_code = -1
                wall_seconds = time.monotonic() - started
                log_path.write_text(output, encoding="utf-8", errors="replace")
                timed_out = timed_out or "Client timed out" in output or "[server][error] Timeout" in output
                parsed = parse_output(output)
                server_len = parsed["server_len"]
                over_cap = bool(server_len is not None and int(server_len) > args.max_joint_actions)
                category = classify_failure(output, timed_out)
                row = {
                    "level": str(level.relative_to(root)),
                    "algorithm": algorithm,
                    "solved": parsed["solved"],
                    "timeout": timed_out,
                    "over_cap": over_cap,
                    "category": category,
                    "server_len": server_len if server_len is not None else "",
                    "client_len": parsed["client_len"] if parsed["client_len"] is not None else "",
                    "expanded": parsed["expanded"] if parsed["expanded"] is not None else "",
                    "frontier": parsed["frontier"] if parsed["frontier"] is not None else "",
                    "generated": parsed["generated"] if parsed["generated"] is not None else "",
                    "search_seconds": parsed["search_seconds"] if parsed["search_seconds"] is not None else "",
                    "wall_seconds": f"{wall_seconds:.6f}",
                    "exit_code": exit_code,
                    "log": str(log_path.relative_to(root)),
                }
                rows.append(row)
                print(
                    f"{row['level']} {algorithm}: category={category} "
                    f"solved={row['solved']} wall={float(row['wall_seconds']):.3f}s"
                )

    csv_path = output_dir / f"{args.output_name}.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    write_markdown(output_dir / f"{args.output_name}.md", rows, args.timeout, args.max_joint_actions)
    return 0


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client", help="Path to searchclient_cpp binary.")
    parser.add_argument("--server-jar", default="misc/server.jar", help="Path to Java server jar.")
    parser.add_argument("--algorithm", action="append", default=[], help="Client algorithm flag(s).")
    parser.add_argument("--level-root", action="append", default=[], help="Directory containing .lvl files.")
    parser.add_argument("--level-list", help="Text file with one level path per line.")
    parser.add_argument("--timeout", type=int, default=180, help="Server timeout per level.")
    parser.add_argument("--max-joint-actions", type=int, default=20000, help="Reporting cap for plan length.")
    parser.add_argument("--limit", type=int, default=0, help="Limit number of levels for smoke runs.")
    parser.add_argument("--output-name", default="baseline-darwin", help="Output basename under benchmarks/results.")
    parser.add_argument("--normalize", action="store_true", help="Normalize level files before running.")
    parser.add_argument("--profile", action="store_true", help="Enable AIMAS_PROFILE=1 for client profiling.")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run_benchmark(parse_args()))
