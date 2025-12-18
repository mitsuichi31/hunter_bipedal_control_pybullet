#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import subprocess
import sys


def _run(cmd: list[str], cwd: str | None = None) -> int:
    p = subprocess.run(cmd, cwd=cwd)
    return p.returncode


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run standing-pd-ext once, then compare recent logs."
    )
    ap.add_argument("--duration", type=float, default=10.0, help="Simulation duration seconds")
    ap.add_argument("--gui", action="store_true", help="Enable GUI (default: off)")
    ap.add_argument("--log-dir", default="runs", help="Directory containing run logs")
    ap.add_argument("--prefix", default="standing_pd_ext", help="Log filename prefix")
    ap.add_argument("--limit", type=int, default=10, help="How many runs to show")
    ap.add_argument(
        "--sort-by",
        default="survival",
        choices=["mtime", "survival", "score", "tilt", "energy"],
        help="How to rank runs in compare table",
    )
    args = ap.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_dir = os.path.join(repo_root, "src")

    # 1) Run one experiment
    run_cmd = [
        sys.executable,
        "main_simulation.py",
        "--mode",
        "standing-pd-ext",
        "--duration",
        str(args.duration),
    ]
    if not args.gui:
        run_cmd.append("--no-gui")

    print("\n== Running experiment ==")
    rc = _run(run_cmd, cwd=src_dir)
    if rc != 0:
        print(f"Experiment failed with exit code {rc}")
        return rc

    # 2) Compare logs
    print("\n== Comparing logs ==")
    compare_cmd = [
        sys.executable,
        os.path.join("scripts", "compare_runs.py"),
        "--log-dir",
        args.log_dir,
        "--prefix",
        args.prefix,
        "--limit",
        str(args.limit),
        "--sort-by",
        args.sort_by,
    ]
    rc2 = _run(compare_cmd, cwd=repo_root)
    return rc2


if __name__ == "__main__":
    raise SystemExit(main())
