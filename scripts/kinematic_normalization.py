#!/usr/bin/env python3
"""
Kinematic Normalization: C(τ) curves for sim-vs-hardware comparison.

τ = Σ d_i(t) / A  (swarm travel fraction)

where d_i(t) is robot i's cumulative distance and A is arena area.
Plotting coverage C as a function of τ instead of wall-clock time
collapses sim and hardware onto a universal curve if the protocol
is identical -- speed differences vanish.
"""

import json
import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict

ARENA_W = 2000  # mm
ARENA_H = 1500  # mm
ARENA_AREA = ARENA_W * ARENA_H  # 3,000,000 mm²
TOTAL_CELLS = 192
SIM_SPEED = 500  # mm/s per robot
N_ROBOTS = 5

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRIAL_DIR = os.path.join(PROJECT_ROOT, "trials")
SIM_DIR = os.path.join(
    PROJECT_ROOT,
    "experiments",
    "run_20260312_005745_multiroom_100trials_all",
)


def load_sim_coverage_tau(controller, fault_type, max_trials=100):
    """Load sim C(τ) curves by converting wall-clock time → τ analytically.

    τ(t) = N * v_sim * t / A
    """
    logs_dir = os.path.join(SIM_DIR, "logs")
    if not os.path.isdir(logs_dir):
        print(f"  [WARN] sim logs dir not found: {logs_dir}")
        return []

    fault_map = {"none": "NoFault", "bad_leader": "BadLeader", "freeze_leader": "FreezeLeader"}
    fault_label = fault_map.get(fault_type, fault_type)

    curves = []
    for trial_idx in range(1, max_trials + 1):
        dirname = f"multiroom_{controller}_{fault_label}_t{trial_idx}"
        tl_path = os.path.join(logs_dir, dirname, "coverage_timeline.json")
        if not os.path.exists(tl_path):
            continue
        data = json.load(open(tl_path))
        timeline = data.get("timeline", [])
        if not timeline:
            continue

        tau_vals = []
        cov_vals = []
        for t_sec, cov_pct in timeline:
            tau = N_ROBOTS * SIM_SPEED * t_sec / ARENA_AREA
            tau_vals.append(tau)
            cov_vals.append(cov_pct / 100.0)

        curves.append((np.array(tau_vals), np.array(cov_vals)))

    return curves


def load_hw_coverage_tau(trial_dirname):
    """Load hardware C(τ) by computing exact per-robot distances from JSONL."""
    trial_path = os.path.join(TRIAL_DIR, trial_dirname)
    if not os.path.isdir(trial_path):
        return None

    jsonl_files = sorted(f for f in os.listdir(trial_path) if f.endswith(".jsonl"))
    if not jsonl_files:
        return None

    robot_entries = {}
    for fname in jsonl_files:
        entries = []
        for line in open(os.path.join(trial_path, fname)):
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "t" in e and "x" in e:
                entries.append(e)
        if entries:
            robot_entries[fname] = entries

    if not robot_entries:
        return None

    t0 = min(e[0]["t"] for e in robot_entries.values())

    events = []
    for fname, entries in robot_entries.items():
        cum_dist = 0.0
        prev_x, prev_y = entries[0].get("x", 0), entries[0].get("y", 0)
        for e in entries:
            x, y = e.get("x", 0), e.get("y", 0)
            cum_dist += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y

            cov_count = e.get("known_visited_count", 0)
            rel_t = e["t"] - t0
            events.append((rel_t, fname, cum_dist, cov_count))

    events.sort(key=lambda x: x[0])

    robot_dist = defaultdict(float)
    robot_cov = defaultdict(int)

    tau_vals = []
    cov_vals = []
    last_tau = -1

    for rel_t, fname, cum_dist, cov_count in events:
        robot_dist[fname] = cum_dist
        robot_cov[fname] = cov_count

        total_dist = sum(robot_dist.values())
        tau = total_dist / ARENA_AREA
        team_cov = max(robot_cov.values()) / TOTAL_CELLS

        if tau - last_tau > 0.0001:
            tau_vals.append(tau)
            cov_vals.append(team_cov)
            last_tau = tau

    return np.array(tau_vals), np.array(cov_vals)


def compute_median_band(curves, tau_grid):
    """Interpolate curves onto common τ grid, compute median and IQR."""
    interped = []
    for tau, cov in curves:
        if len(tau) < 2:
            continue
        interp_cov = np.interp(tau_grid, tau, cov, left=cov[0], right=cov[-1])
        interped.append(interp_cov)

    if not interped:
        return None, None, None
    arr = np.array(interped)
    median = np.median(arr, axis=0)
    q25 = np.percentile(arr, 25, axis=0)
    q75 = np.percentile(arr, 75, axis=0)
    return median, q25, q75


def compute_dCdtau(tau_vals, cov_vals, window=5):
    """Smoothed derivative dC/dτ (exploration efficiency)."""
    if len(tau_vals) < window * 2:
        return tau_vals, np.zeros_like(cov_vals)
    dtau = np.diff(tau_vals)
    dcov = np.diff(cov_vals)
    dtau[dtau == 0] = 1e-10
    deriv = dcov / dtau
    kernel = np.ones(window) / window
    deriv_smooth = np.convolve(deriv, kernel, mode="same")
    return tau_vals[:-1], deriv_smooth


def main():
    out_dir = os.path.join(PROJECT_ROOT, "paper_docs", "Ryker_Kollmyer___UPDATED_PAPER")
    os.makedirs(out_dir, exist_ok=True)

    # --- Load data ---
    print("Loading simulation data...")
    sim_reip_clean = load_sim_coverage_tau("reip", "none")
    sim_raft_clean = load_sim_coverage_tau("raft", "none")
    sim_reip_bad = load_sim_coverage_tau("reip", "bad_leader")
    sim_raft_bad = load_sim_coverage_tau("raft", "bad_leader")
    print(f"  REIP clean: {len(sim_reip_clean)} trials")
    print(f"  Raft clean: {len(sim_raft_clean)} trials")
    print(f"  REIP bad_leader: {len(sim_reip_bad)} trials")
    print(f"  Raft bad_leader: {len(sim_raft_bad)} trials")

    print("\nLoading hardware data...")
    hw_reip_trials = [
        "reip_none_t1_20260308_151155",
        "reip_none_t1_20260308_152650",
        "reip_none_t1_20260312_201053",
        "reip_none_t1_20260312_204229",
        "reip_none_t1_20260312_205223",
    ]
    hw_raft_trials = [
        "raft_none_t1_20260312_221216",
        "raft_none_t1_20260312_224345",
    ]
    hw_reip_freeze = ["reip_freeze_leader_t1_20260312_212620"]

    hw_reip_curves = []
    for d in hw_reip_trials:
        result = load_hw_coverage_tau(d)
        if result is not None:
            hw_reip_curves.append(result)
            print(f"  HW REIP clean: {d} -> tau_max={result[0][-1]:.4f}")

    hw_raft_curves = []
    for d in hw_raft_trials:
        result = load_hw_coverage_tau(d)
        if result is not None:
            hw_raft_curves.append(result)
            print(f"  HW Raft clean: {d} -> tau_max={result[0][-1]:.4f}")

    hw_reip_freeze_curves = []
    for d in hw_reip_freeze:
        result = load_hw_coverage_tau(d)
        if result is not None:
            hw_reip_freeze_curves.append(result)
            print(f"  HW REIP freeze: {d} -> tau_max={result[0][-1]:.4f}")

    # --- Figure 1: C(τ) collapse plot ---
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)

    tau_max_hw = max(c[0][-1] for c in hw_reip_curves) if hw_reip_curves else 0.02
    tau_grid_overlap = np.linspace(0, tau_max_hw * 1.1, 500)

    # Panel (a): Clean conditions -- sim vs hw
    ax = axes[0]
    ax.set_title("(a) Clean — REIP: Sim vs. Hardware", fontsize=11)

    if sim_reip_clean:
        med, q25, q75 = compute_median_band(sim_reip_clean, tau_grid_overlap)
        if med is not None:
            ax.plot(tau_grid_overlap, med * 100, "b-", linewidth=2, label="Sim REIP (N=100)")
            ax.fill_between(tau_grid_overlap, q25 * 100, q75 * 100, alpha=0.15, color="blue")

    if sim_raft_clean:
        med, q25, q75 = compute_median_band(sim_raft_clean, tau_grid_overlap)
        if med is not None:
            ax.plot(tau_grid_overlap, med * 100, "r--", linewidth=2, label="Sim Raft (N=100)")
            ax.fill_between(tau_grid_overlap, q25 * 100, q75 * 100, alpha=0.1, color="red")

    for i, (tau, cov) in enumerate(hw_reip_curves):
        lbl = "HW REIP" if i == 0 else None
        ax.plot(tau, cov * 100, "b-", linewidth=1, alpha=0.5, marker="o",
                markersize=2, label=lbl)

    for i, (tau, cov) in enumerate(hw_raft_curves):
        lbl = "HW Raft" if i == 0 else None
        ax.plot(tau, cov * 100, "r-", linewidth=1, alpha=0.5, marker="s",
                markersize=2, label=lbl)

    ax.set_xlabel(r"$\tau$ (swarm travel fraction)", fontsize=11)
    ax.set_ylabel("Coverage (%)", fontsize=11)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_xlim(0, tau_max_hw * 1.1)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)

    # Panel (b): Full sim range showing convergence
    ax = axes[1]
    ax.set_title("(b) Full Sim Range — REIP vs. Raft", fontsize=11)

    sim_tau_max = 0.1 if sim_reip_clean else 0.1
    tau_grid_full = np.linspace(0, 0.25, 1000)

    if sim_reip_clean:
        med, q25, q75 = compute_median_band(sim_reip_clean, tau_grid_full)
        if med is not None:
            ax.plot(tau_grid_full, med * 100, "b-", linewidth=2, label="Sim REIP clean")
            ax.fill_between(tau_grid_full, q25 * 100, q75 * 100, alpha=0.15, color="blue")

    if sim_raft_clean:
        med, q25, q75 = compute_median_band(sim_raft_clean, tau_grid_full)
        if med is not None:
            ax.plot(tau_grid_full, med * 100, "r--", linewidth=2, label="Sim Raft clean")
            ax.fill_between(tau_grid_full, q25 * 100, q75 * 100, alpha=0.1, color="red")

    if sim_reip_bad:
        med, q25, q75 = compute_median_band(sim_reip_bad, tau_grid_full)
        if med is not None:
            ax.plot(tau_grid_full, med * 100, "b:", linewidth=2, label="Sim REIP bad_ldr")

    if sim_raft_bad:
        med, q25, q75 = compute_median_band(sim_raft_bad, tau_grid_full)
        if med is not None:
            ax.plot(tau_grid_full, med * 100, "r:", linewidth=2, label="Sim Raft bad_ldr")

    # Mark HW τ range
    if hw_reip_curves:
        ax.axvspan(0, tau_max_hw, alpha=0.08, color="green", label=r"HW $\tau$ range")

    ax.set_xlabel(r"$\tau$ (swarm travel fraction)", fontsize=11)
    ax.legend(fontsize=8, loc="lower right")
    ax.set_xlim(0, 0.25)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = os.path.join(out_dir, "kinematic_normalization.png")
    plt.savefig(fig_path, dpi=300, bbox_inches="tight")
    print(f"\nSaved: {fig_path}")
    plt.close()

    # --- Figure 2: dC/dτ (exploration efficiency) ---
    fig, ax = plt.subplots(1, 1, figsize=(7, 4))
    ax.set_title(r"Exploration Efficiency $dC/d\tau$", fontsize=12)

    if sim_reip_clean:
        all_derivs = []
        tau_d_grid = np.linspace(0, tau_max_hw * 1.1, 200)
        for tau, cov in sim_reip_clean[:50]:
            tau_d, deriv = compute_dCdtau(tau, cov, window=10)
            if len(tau_d) > 1:
                interp_d = np.interp(tau_d_grid, tau_d, deriv, left=0, right=0)
                all_derivs.append(interp_d)
        if all_derivs:
            arr = np.array(all_derivs)
            ax.plot(tau_d_grid, np.median(arr, axis=0), "b-", linewidth=2,
                    label="Sim REIP", alpha=0.8)
            ax.fill_between(tau_d_grid,
                            np.percentile(arr, 25, axis=0),
                            np.percentile(arr, 75, axis=0),
                            alpha=0.15, color="blue")

    if sim_raft_clean:
        all_derivs = []
        for tau, cov in sim_raft_clean[:50]:
            tau_d, deriv = compute_dCdtau(tau, cov, window=10)
            if len(tau_d) > 1:
                interp_d = np.interp(tau_d_grid, tau_d, deriv, left=0, right=0)
                all_derivs.append(interp_d)
        if all_derivs:
            arr = np.array(all_derivs)
            ax.plot(tau_d_grid, np.median(arr, axis=0), "r--", linewidth=2,
                    label="Sim Raft", alpha=0.8)

    for i, (tau, cov) in enumerate(hw_reip_curves):
        tau_d, deriv = compute_dCdtau(tau, cov, window=20)
        lbl = "HW REIP" if i == 0 else None
        ax.plot(tau_d, deriv, "b-", linewidth=1, alpha=0.4, label=lbl)

    for i, (tau, cov) in enumerate(hw_raft_curves):
        tau_d, deriv = compute_dCdtau(tau, cov, window=20)
        lbl = "HW Raft" if i == 0 else None
        ax.plot(tau_d, deriv, "r-", linewidth=1, alpha=0.4, label=lbl)

    ax.set_xlabel(r"$\tau$ (swarm travel fraction)", fontsize=11)
    ax.set_ylabel(r"$dC/d\tau$ (exploration efficiency)", fontsize=11)
    ax.legend(fontsize=9)
    ax.set_xlim(0, tau_max_hw * 1.1)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    fig_path2 = os.path.join(out_dir, "exploration_efficiency.png")
    plt.savefig(fig_path2, dpi=300, bbox_inches="tight")
    print(f"Saved: {fig_path2}")
    plt.close()

    # --- Print key τ values ---
    print("\n=== KEY tau VALUES ===")
    if hw_reip_curves:
        taus = [c[0][-1] for c in hw_reip_curves]
        print(f"HW REIP tau_max: {np.mean(taus):.4f} (mean over {len(taus)} trials)")
        covs = [c[1][-1] for c in hw_reip_curves]
        print(f"HW REIP C(tau_max): {np.mean(covs)*100:.1f}%")
    if hw_raft_curves:
        taus = [c[0][-1] for c in hw_raft_curves]
        print(f"HW Raft tau_max: {np.mean(taus):.4f}")
        covs = [c[1][-1] for c in hw_raft_curves]
        print(f"HW Raft C(tau_max): {np.mean(covs)*100:.1f}%")
    if sim_reip_clean:
        tau_at_hw_range = tau_max_hw
        covs_at_tau = []
        for tau, cov in sim_reip_clean:
            idx = np.searchsorted(tau, tau_at_hw_range)
            if idx < len(cov):
                covs_at_tau.append(cov[idx])
        if covs_at_tau:
            print(f"Sim REIP C(tau={tau_at_hw_range:.4f}): {np.mean(covs_at_tau)*100:.1f}%")
    if sim_raft_clean:
        covs_at_tau = []
        for tau, cov in sim_raft_clean:
            idx = np.searchsorted(tau, tau_at_hw_range)
            if idx < len(cov):
                covs_at_tau.append(cov[idx])
        if covs_at_tau:
            print(f"Sim Raft C(tau={tau_at_hw_range:.4f}): {np.mean(covs_at_tau)*100:.1f}%")


if __name__ == "__main__":
    main()
