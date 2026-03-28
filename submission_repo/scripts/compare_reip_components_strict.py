"""Strict REIP ablation: remove one framework component at a time."""

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

from scripts.ablation_study import (  # noqa: E402
    load_cfg,
    _run_single_worker,
    plot_ablation_results,
)

VARIANT_ORDER = [
    "reip_full",
    "reip_no_fallback",
    "reip_no_detection",
    "reip_no_governance",
]
VARIANT_LABELS = [
    "REIP-\nFull",
    "REIP-\nNoFallback",
    "REIP-\nNoDetect",
    "REIP-\nNoGov",
]
VARIANT_LABEL_MAP = dict(zip(VARIANT_ORDER, VARIANT_LABELS))


def create_strict_reip_variants(base_cfg_path: str) -> Dict[str, dict]:
    base = load_cfg(base_cfg_path)
    variants: Dict[str, dict] = {}

    variants["reip_full"] = copy.deepcopy(base)

    cfg_no_fb = copy.deepcopy(base)
    cfg_no_fb.setdefault("reip", {})["fallback_enabled"] = False
    variants["reip_no_fallback"] = cfg_no_fb

    cfg_no_det = copy.deepcopy(base)
    reip_no_det = cfg_no_det.setdefault("reip", {})
    reip_no_det["detection_enabled"] = False
    # Also disable Bayesian detector when detection is ablated.
    reip_no_det["bayes_enabled"] = False
    variants["reip_no_detection"] = cfg_no_det

    cfg_no_gov = copy.deepcopy(base)
    reip_no_gov = cfg_no_gov.setdefault("reip", {})
    reip_no_gov["governance_enabled"] = False
    # Optional stress: when governance is disabled, also drop fallback so the corrupt leader has full control.
    reip_no_gov["fallback_enabled"] = False
    reip_no_gov["bayes_enabled"] = False
    variants["reip_no_governance"] = cfg_no_gov

    return variants


def run_variant(cfg: dict, name: str, runs: int, max_workers: int) -> List[dict]:
    print(f"\n=== {name.upper()} ===")
    args = [(copy.deepcopy(cfg), run_id) for run_id in range(runs)]
    results: List[dict] = []
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_run_single_worker, arg) for arg in args]
        for idx, future in enumerate(as_completed(futures), 1):
            try:
                res = future.result()
                if res.get("success", False):
                    results.append(res)
            except Exception as exc:
                print(f"    Worker failed: {exc}")
            if idx % max(1, runs // 10 or 1) == 0:
                print(f"    Progress: {idx}/{runs}")
    print(f"    Completed {len(results)}/{runs} successful runs")
    return results


def summarize_variant(results: List[dict]) -> Dict[str, float]:
    if not results:
        return {}
    final_cov = np.array([r.get("final_coverage", 0.0) for r in results])
    elections = np.array([r.get("elections", 0) for r in results])
    impeachments = np.array([r.get("impeachments", 0) for r in results])
    success_flags = np.array([bool(r.get("reached_95", False)) for r in results], dtype=float)
    times = np.array([r.get("time_to_95") for r in results if r.get("time_to_95") is not None], dtype=float)

    return {
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


def log_component_checks(stats: Dict[str, dict]) -> None:
    """Print quick sanity checks to ensure components are disabled as expected."""
    for variant, summary in stats.items():
        mean_elections = summary.get("mean_elections")
        mean_impeachments = summary.get("mean_impeachments")
        print(f"[CHECK] {variant}: mean_elections={mean_elections:.2f}, mean_impeachments={mean_impeachments:.2f}")

    nogov = stats.get("reip_no_governance")
    if nogov and (nogov.get("mean_elections", 0.0) > 1e-3 or nogov.get("mean_impeachments", 0.0) > 1e-3):
        print("WARNING: reip_no_governance variant reported non-zero elections/impeachments.")

    nodet = stats.get("reip_no_detection")
    if nodet and nodet.get("mean_impeachments", 0.0) > 0:
        print("NOTE: reip_no_detection still reported impeachments (expected if governance active but detector off).")


def save_histories(all_results: Dict[str, List[dict]], output_dir: str, timestamp: str) -> None:
    histories_dir = os.path.join(output_dir, f"histories_{timestamp}")
    os.makedirs(histories_dir, exist_ok=True)
    for variant, runs in all_results.items():
        if not runs:
            continue
        max_len = max(len(r.get("coverage_history", [])) for r in runs)
        csv_path = os.path.join(histories_dir, f"{variant}_histories.csv")
        fieldnames = ["run_id"] + [f"t_{i}" for i in range(max_len)]
        with open(csv_path, "w", newline="") as f:
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


def save_outputs(stats: Dict[str, dict], all_results: Dict[str, List[dict]], output_dir: str, ts: str, variant_order: List[str]) -> None:
    summary_path = os.path.join(output_dir, f"reip_strict_compare_{ts}.csv")
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
        for variant in variant_order:
            if variant in stats:
                writer.writerow({"variant": variant, **stats[variant]})

    detailed_path = os.path.join(output_dir, f"reip_strict_detailed_{ts}.csv")
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Strict REIP component ablation")
    parser.add_argument("--config", type=str, default="configs/benchmarks/reip_faulted.yaml",
                        help="Base REIP config to clone")
    parser.add_argument("--runs", type=int, default=200, help="Runs per variant")
    parser.add_argument("--max_workers", type=int, default=6, help="Parallel workers")
    parser.add_argument("--output_dir", type=str, default="results/reip_strict_ablation", help="Output dir")
    parser.add_argument("--skip_no_detection", action="store_true",
                        help="Omit the reip_no_detection variant (and bar) from this run/plot")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    variants = create_strict_reip_variants(args.config)

    active_variants = [v for v in VARIANT_ORDER if not (args.skip_no_detection and v == "reip_no_detection")]

    all_results: Dict[str, List[dict]] = {}
    for variant in active_variants:
        if variant not in variants:
            continue
        runs = run_variant(variants[variant], variant, args.runs, args.max_workers)
        all_results[variant] = runs

    stats = {variant: summarize_variant(all_results.get(variant, [])) for variant in active_variants if all_results.get(variant)}
    if stats:
        log_component_checks(stats)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_outputs(stats, all_results, args.output_dir, timestamp, active_variants)
    save_histories(all_results, args.output_dir, timestamp)

    mode_order = [v for v in active_variants if v in stats]
    mode_labels = [VARIANT_LABEL_MAP[v] for v in mode_order]
    plot_ablation_results(stats, all_results, args.output_dir, timestamp, mode_order=mode_order, mode_labels=mode_labels)

    print("\nStrict REIP component ablation complete.")


if __name__ == "__main__":
    main()

