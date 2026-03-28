#!/usr/bin/env python3
"""
Build a clean fair-submission export inside `submission_repo/`.

The export keeps core code, configs, docs, hardware support, and the live demo,
while excluding the working-tree clutter from experiments, logs, and ad hoc
debug artifacts.
"""

from __future__ import annotations

import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "submission_repo"

INCLUDE_DIRS = [
    "src",
    "robot",
    "pc",
    "pico",
    "coordinator",
    "configs",
    "docs",
    "demo",
    "scripts",
    "paper_docs/Ryker_Kollmyer___UPDATED_PAPER",
    "REIP_Supplemental",
]

# Individual test files needed (demo imports test.visual_sim,
# experiments use isef_experiments, etc.)
TEST_FILES = [
    "test/__init__.py",
    "test/visual_sim.py",
    "test/isef_experiments.py",
    "test/sim_harness.py",
    "test/simple_sim.py",
    "test/generate_poster_graphs.py",
    "test/_generate_paper_figures.py",
    "test/_generate_run_figures.py",
    "test/_convergence_chart.py",
    "test/_ablation_charts.py",
    "test/_fault_split_shap_style.py",
    "test/_fault_raincloud.py",
    "test/_fault_success_callout.py",
    "test/_fault_threshold_figures.py",
    "test/_fault_threshold_ladder.py",
    "test/_fault_early_window_compare.py",
    "test/_fault_metric_kde.py",
    "test/_fault_plum_pudding.py",
    "test/_coverage_slope_overlay.py",
    "test/_reip_derivative_only.py",
    "test/_generate_blackout_figures.py",
    "test/_generate_cyberattack_figures.py",
]

INCLUDE_FILES = [
    "README.md",
    "requirements.txt",
    "run_trial.py",
    "parse_trial.py",
    "parse_hardware_trials.py",
    "aggregate_hardware_results.py",
    "hardware_fidelity.py",
    "deploy.ps1",
    "HARDWARE_SETUP.md",
    "HARDWARE_RUN_GUIDE.md",
    "HARDWARE_EXPERIMENT_MATRIX.md",
    "EXPERIMENT_PROCEDURE.md",
    "SINGLE_ROBOT_TEST.md",
]


def copy_path(rel_path: str) -> None:
    src = ROOT / rel_path
    dst = DEST / rel_path
    if not src.exists():
        return
    if src.is_dir():
        shutil.copytree(src, dst, dirs_exist_ok=True)
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def write_submission_readme() -> None:
    text = """# Submission Repo

This folder is a clean export of the REIP project for fair submission.

## Included

- Core simulation source in `src/`
- Hardware/control code in `robot/`, `pc/`, and `pico/`
- Config files in `configs/`
- Presentation demo in `demo/`
- Final paper assets in `paper_docs/Ryker_Kollmyer___UPDATED_PAPER/`
- Supplemental material in `REIP_Supplemental/`

## Excluded

- Raw runtime logs
- Old archive outputs
- Experimental scratch folders
- One-off debug/check/trace clutter from the working repo

## Demo Command

```bash
python demo/live_comparison_demo.py
```
"""
    (DEST / "README_SUBMISSION.md").write_text(text, encoding="utf-8")


def main() -> None:
    if DEST.exists():
        shutil.rmtree(DEST)
    DEST.mkdir(parents=True, exist_ok=True)

    for rel_path in INCLUDE_DIRS:
        copy_path(rel_path)
    for rel_path in TEST_FILES:
        copy_path(rel_path)
    for rel_path in INCLUDE_FILES:
        copy_path(rel_path)

    write_submission_readme()
    print(f"Created clean submission export at {DEST}")


if __name__ == "__main__":
    main()
