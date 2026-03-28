#!/usr/bin/env python3
"""
Generate curated poster-quality figures into a run-local directory.

This is the sane replacement for the old generic poster graph script when you
want the polished figures preserved alongside a specific experiment run.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def run_step(args, required=False):
    print(f"\n>>> {' '.join(args)}")
    try:
        completed = subprocess.run(args, check=True, text=True, capture_output=True)
        if completed.stdout.strip():
            print(completed.stdout.strip())
        if completed.stderr.strip():
            print(completed.stderr.strip())
        return True
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or "").strip().splitlines()
        tail = detail[-1] if detail else "No additional details."
        print(f"[SKIP] Step failed with exit code {exc.returncode}: {tail}")
        if required:
            raise
        return False


def main():
    parser = argparse.ArgumentParser(description="Generate curated figures into <run_dir>/curated_figures.")
    parser.add_argument("run_dir", help="Experiment run directory")
    parser.add_argument("--output-dir", default=None,
                        help="Directory to save figures (default: <run_dir>/curated_figures)")
    parser.add_argument("--include-paper-suite", action="store_true",
                        help="Also run the curated IEEE paper suite into the same output directory")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    if not run_dir.is_dir():
        raise FileNotFoundError(f"Run directory not found: {run_dir}")

    output_dir = Path(args.output_dir) if args.output_dir else run_dir / "curated_figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    py = sys.executable

    run_step([py, "test/_fault_threshold_figures.py", str(run_dir), "--output-dir", str(output_dir)])
    run_step([py, "test/_fault_success_callout.py", str(run_dir), "--output-dir", str(output_dir)])
    run_step([py, "test/_fault_raincloud.py", str(run_dir), "--output-dir", str(output_dir)])
    run_step([py, "test/_generate_blackout_figures.py", str(run_dir), "--output-dir", str(output_dir)])

    if not run_step([
        py, "test/_coverage_slope_overlay.py", str(run_dir),
        "--fault", "bad_leader",
        "--schedule", "default",
        "--output-dir", str(output_dir),
    ]):
        print("Skipping default bad-leader slope overlay for this run.")

    if not run_step([
        py, "test/_coverage_slope_overlay.py", str(run_dir),
        "--fault", "bad_leader",
        "--schedule", "cyber_burst",
        "--output-name", "coverage_slope_overlay_cyber_burst",
        "--output-dir", str(output_dir),
    ]):
        print("Skipping cyber-burst slope overlay for this run.")

    if args.include_paper_suite:
        run_step([
            py, "test/_generate_paper_figures.py",
            "--main-dir", str(run_dir),
            "--freeze-dir", str(run_dir),
            "--ablation-dir", str(run_dir),
            "--output-dir", str(output_dir),
        ], required=True)

    print(f"\nCurated figures saved to: {output_dir}")


if __name__ == "__main__":
    main()
