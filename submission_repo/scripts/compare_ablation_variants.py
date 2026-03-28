"""Run the 4-mode ablation path using a workflow similar to
`scripts/compare_reip_vs_baseline.py`.

This script is simply a convenience wrapper around the lower-level
helpers already used by `ablation_study.py`.  It repeatedly runs each
variant, aggregates the requested metrics (final coverage, success
rate, median time-to-95, etc.), writes summary / detailed CSV files,
saves coverage histories, and regenerates the IEEE-style 3-panel figure.
"""

from __future__ import annotations

import argparse
import copy
import csv
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from typing import Dict, List

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if os.path.join(_PROJECT_ROOT, "src") not in sys.path:
    sys.path.insert(0, os.path.join(_PROJECT_ROOT, "src"))

from scripts.ablation_study import (  # noqa: E402  pylint: disable=wrong-import-position
    create_ablation_configs,
    _run_single_worker,
    plot_ablation_results,
    load_cfg,
)

VARIANT_ORDER = ["baseline", "baseline_fallback", "reip_gov", "reip_full"]


def run_variant(cfg: dict, variant_name: str, runs: int, max_workers: int) -> List[dict]:
    """Run *runs* simulations for a single variant in parallel."""
    print(f"\n=== {variant_name.upper()} ===")
    args = [(copy.deepcopy(cfg), run_id) for run_id in range(runs)]
    results: List[dict] = []
    failures = 0
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_run_single_worker, arg) for arg in args]
        for idx, future in enumerate(as_completed(futures), 1):
            try:
                res = future.result()
                if res.get("success", False):
                    results.append(res)
                else:
                    failures += 1
            except Exception as exc:  # pragma: no cover - defensive
                failures += 1
                print(f"    Worker failed: {exc}")
            if idx % max(1, runs // 10 or 1) == 0:
                print(f"    Progress: {idx}/{runs}")
    print(f"    Completed {len(results)}/{runs} runs (failures={failures})")
    return results


def summarize_variant(results: List[dict]) -> Dict[str, float]:
    if not results:
        return {}
    final_cov = np.array([r.get("final_coverage", 0.0) for r in results])
    elections = np.array([r.get("elections", 0) for r in results])
    impeachments = np.array([r.get("impeachments", 0) for r in results])
    success_flags = np.array([bool(r.get("reached_95", False)) for r in results], dtype=float)
    times = np.array(
        [r.get("time_to_95") for r in results if r.get("time_to_95") is not None],
        dtype=float,
    )
    stats = {
        "mean_coverage": float(final_cov.mean()),
        "std_coverage": float(final_cov.std()),
        "sem_coverage": float(final_cov.std() / np.sqrt(len(final_cov))),
        "median_coverage": float(np.median(final_cov)),
        "success_rate": float(success_flags.mean()),
        "success_rate_sem": float(success_flags.std() / np.sqrt(len(success_flags))),
        "median_time_to_95": float(np.median(times)) if len(times) else None,
        "mean_time_to_95": float(times.mean()) if len(times) else None,
        "std_time_to_95": float(times.std()) if len(times) else None,
        "mean_elections": float(elections.mean()),
        "std_elections": float(elections.std()),
        "mean_impeachments": float(impeachments.mean()),
        "n_runs": len(results),
        "n_reached_95": int(success_flags.sum()),
    }
    return stats


def save_outputs(stats: Dict[str, dict], all_results: Dict[str, List[dict]], output_dir: str, timestamp: str) -> None:
    summary_path = os.path.join(output_dir, f"ablation_compare_{timestamp}.csv")
    with open(summary_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "variant",
                "mean_coverage",
                "std_coverage",
                "sem_coverage",
                "median_coverage",
                "success_rate",
                "success_rate_sem",
                "median_time_to_95",
                "mean_time_to_95",
                "std_time_to_95",
                "mean_elections",
                "std_elections",
                "mean_impeachments",
                "n_runs",
                "n_reached_95",
            ],
        )
        writer.writeheader()
        for variant in VARIANT_ORDER:
            if variant in stats:
                row = {"variant": variant, **stats[variant]}
                writer.writerow(row)
    print(f"Summary saved to: {summary_path}")

    detailed_path = os.path.join(output_dir, f"ablation_detailed_{timestamp}.csv")
    with open(detailed_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "variant",
                "run_id",
                "final_coverage",
                "elections",
                "impeachments",
                "reached_95",
                "reached_90",
                "time_to_95",
            ],
        )
        writer.writeheader()
        for variant, runs in all_results.items():
            for r in runs:
                writer.writerow(
                    {
                        "variant": variant,
                        "run_id": r.get("run_id", 0),
                        "final_coverage": r.get("final_coverage", 0.0),
                        "elections": r.get("elections", 0),
                        "impeachments": r.get("impeachments", 0),
                        "reached_95": r.get("reached_95", False),
                        "reached_90": r.get("reached_90", False),
                        "time_to_95": r.get("time_to_95", "") if r.get("time_to_95") is not None else "",
                    }
                )
    print(f"Detailed per-run data saved to: {detailed_path}")

    histories_dir = os.path.join(output_dir, f"histories_{timestamp}")
    os.makedirs(histories_dir, exist_ok=True)
    for variant, runs in all_results.items():
        if not runs:
            continue
        hist_path = os.path.join(histories_dir, f"{variant}_histories.csv")
        max_len = max(len(r.get("coverage_history", [])) for r in runs)
        with open(hist_path, "w", newline="") as f:
            fieldnames = ["run_id"] + [f"t_{i}" for i in range(max_len)]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in runs:
                hist = r.get("coverage_history", [])
                if len(hist) < max_len:
                    if hist:
                        hist = hist + [hist[-1]] * (max_len - len(hist))
                    else:
                        hist = [0.0] * max_len
                row = {"run_id": r.get("run_id", 0)}
                row.update({f"t_{i}": cov for i, cov in enumerate(hist)})
                writer.writerow(row)
        print(f"Histories for {variant} saved to: {hist_path}")


def print_console_summary(stats: Dict[str, dict]) -> None:
    print("\n=== COMPARISON SUMMARY ===")
    for variant in VARIANT_ORDER:
        if variant not in stats:
            continue
        stat = stats[variant]
        cov = stat["mean_coverage"] * 100
        sr = stat["success_rate"] * 100
        t95 = stat.get("median_time_to_95")
        t95_str = f"{t95:.0f}" if t95 is not None else "--"
        print(
            f"{variant:18s} | Coverage={cov:5.1f}%  Success={sr:5.1f}%  Median T95={t95_str}  "
            f"Elections={stat['mean_elections']:.1f}  Impeachments={stat['mean_impeachments']:.1f}"
        )


def build_variant_configs(args) -> Dict[str, dict]:
    """Return dict of variant_name -> cfg based on CLI args."""
    explicit = [
        args.baseline_config,
        args.baseline_fallback_config,
        args.reip_gov_config,
        args.reip_full_config,
    ]
    if any(explicit):
        if not all(explicit):
            raise ValueError("If you provide any explicit config paths, you must provide all four.")
        return {
            "baseline": load_cfg(args.baseline_config),
            "baseline_fallback": load_cfg(args.baseline_fallback_config),
            "reip_gov": load_cfg(args.reip_gov_config),
            "reip_full": load_cfg(args.reip_full_config),
        }
    if not args.faulted_config:
        raise ValueError("Provide --faulted_config or all four explicit configs.")
    return create_ablation_configs(args.faulted_config)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare ablation variants (4-mode path)")
    parser.add_argument("--faulted_config", type=str, default="configs/benchmarks/reip_faulted.yaml",
                        help="Single faulted config to clone for all variants (optional if explicit configs provided)")
    parser.add_argument("--baseline_config", type=str, help="Config file for baseline (fixed leader, no fallback)")
    parser.add_argument("--baseline_fallback_config", type=str, help="Config file for baseline+fallback")
    parser.add_argument("--reip_gov_config", type=str, help="Config file for REIP governance-only mode")
    parser.add_argument("--reip_full_config", type=str, help="Config file for REIP full mode")
    parser.add_argument("--runs", type=int, default=2000, help="Number of runs per variant")
    parser.add_argument("--max_workers", type=int, default=6, help="Parallel workers")
    parser.add_argument("--output_dir", type=str, default="results/ablation", help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    variants = build_variant_configs(args)

    all_results: Dict[str, List[dict]] = {}
    for variant in VARIANT_ORDER:
        if variant not in variants:
            continue
        runs = run_variant(variants[variant], variant, args.runs, args.max_workers)
        all_results[variant] = runs

    stats = {
        variant: summarize_variant(all_results.get(variant, []))
        for variant in VARIANT_ORDER
        if all_results.get(variant)
    }

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_outputs(stats, all_results, args.output_dir, timestamp)
    print_console_summary(stats)

    plot_ablation_results(stats, all_results, args.output_dir, timestamp)
    print("\nComparison complete.")


if __name__ == "__main__":
    main()
