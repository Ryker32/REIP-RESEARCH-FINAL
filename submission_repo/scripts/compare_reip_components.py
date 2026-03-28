"""REIP internal ablation study (framework components).

Variants evaluated (default):
  - reip_full: governance + fallback (baseline config)
  - reip_no_fallback: governance only, fallback disabled
  - reip_lazy_governance: fallback enabled but trust threshold / decay slowed
  - reip_no_detection: fallback enabled but hallucination detection disabled (high CUSUM threshold)

Usage mirrors compare_ablation_variants.py:
    python scripts/compare_reip_components.py --runs 200 --config configs/benchmarks/reip_faulted.yaml
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
    load_cfg,
    _run_single_worker,
    plot_ablation_results,
)

VARIANT_ORDER = [
    "reip_full",
    "reip_no_fallback",
    "reip_lazy_governance",
    "reip_no_detection",
]

VARIANT_LABELS = [
    "REIP-\nFull",
    "REIP-\nNoFallback",
    "REIP-\nLazyGov",
    "REIP-\nNoDetect",
]


def create_reip_component_configs(base_cfg_path: str) -> Dict[str, dict]:
    base = load_cfg(base_cfg_path)
    variants: Dict[str, dict] = {}

    variants["reip_full"] = copy.deepcopy(base)

    cfg_no_fb = copy.deepcopy(base)
    cfg_no_fb.setdefault("reip", {})["fallback_enabled"] = False
    variants["reip_no_fallback"] = cfg_no_fb

    cfg_lazy = copy.deepcopy(base)
    reip_lazy = cfg_lazy.setdefault("reip", {})
    reip_lazy["trust_threshold"] = 0.9
    reip_lazy["trust_decay_rate"] = 0.15
    reip_lazy["trust_recovery_rate"] = 0.01
    reip_lazy["cusum_threshold"] = reip_lazy.get("cusum_threshold", 25.0) * 2.5
    variants["reip_lazy_governance"] = cfg_lazy

    cfg_no_detect = copy.deepcopy(base)
    reip_no_det = cfg_no_detect.setdefault("reip", {})
    reip_no_det["fallback_enabled"] = True
    reip_no_det["detection_enabled"] = False
    reip_no_det["cusum_threshold"] = 1e9  # effectively disable hallucination detection
    reip_no_det["cusum_min_samples"] = 1_000_000
    variants["reip_no_detection"] = cfg_no_detect

    return variants


def run_variant(cfg: dict, variant_name: str, runs: int, max_workers: int) -> List[dict]:
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
            except Exception as exc:
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
    times = np.array([r.get("time_to_95") for r in results if r.get("time_to_95") is not None], dtype=float)
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
    summary_path = os.path.join(output_dir, f"reip_ablation_compare_{timestamp}.csv")
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
                writer.writerow({"variant": variant, **stats[variant]})
    print(f"Summary saved to: {summary_path}")

    detailed_path = os.path.join(output_dir, f"reip_ablation_detailed_{timestamp}.csv")
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Internal REIP component ablation")
    parser.add_argument("--config", type=str, default="configs/benchmarks/reip_faulted.yaml",
                        help="Base REIP config to clone for variants")
    parser.add_argument("--runs", type=int, default=200, help="Runs per variant")
    parser.add_argument("--max_workers", type=int, default=6, help="Parallel workers")
    parser.add_argument("--output_dir", type=str, default="results/reip_ablation", help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    variants = create_reip_component_configs(args.config)

    all_results: Dict[str, List[dict]] = {}
    for variant in VARIANT_ORDER:
        if variant not in variants:
            continue
        runs = run_variant(variants[variant], variant, args.runs, args.max_workers)
        all_results[variant] = runs

    stats = {variant: summarize_variant(all_results.get(variant, [])) for variant in VARIANT_ORDER if all_results.get(variant)}

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_outputs(stats, all_results, args.output_dir, timestamp)

    mode_order = [v for v in VARIANT_ORDER if v in stats]
    mode_labels = [VARIANT_LABELS[VARIANT_ORDER.index(v)] for v in mode_order]
    plot_ablation_results(stats, all_results, args.output_dir, timestamp, mode_order=mode_order, mode_labels=mode_labels)

    print("\nREIP component ablation complete.")


if __name__ == "__main__":
    main()
