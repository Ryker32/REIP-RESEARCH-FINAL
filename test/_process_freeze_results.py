#!/usr/bin/env python3
"""Process freeze_leader N=100 results: run analysis, generate figures,
compute numbers for the paper, and print LaTeX-ready table rows.

Usage:
    python test/_process_freeze_results.py <freeze_experiment_dir>
    
Example:
    python test/_process_freeze_results.py experiments/run_20260228_135632_multiroom_100trials_FreezeLeader
"""

import json, glob, os, sys, math, shutil
from collections import defaultdict

def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None

def std(vals):
    vals = [v for v in vals if v is not None]
    if len(vals) < 2:
        return 0
    m = mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))

def median_val(vals):
    vals = sorted([v for v in vals if v is not None])
    if not vals: return None
    n = len(vals)
    if n % 2 == 1:
        return vals[n // 2]
    return (vals[n // 2 - 1] + vals[n // 2]) / 2

def time_to_threshold(timeline, threshold):
    """Return first time coverage exceeds threshold, or None."""
    for t, cov in timeline:
        if cov >= threshold:
            return t
    return None

def coverage_at_time(timeline, target_t):
    """Interpolate coverage at a given time. Returns 0-1 fraction."""
    if not timeline:
        return 0.0
    prev_t, prev_c = 0, 0
    for t, cov in timeline:
        cov_frac = cov / 100.0  # normalize from percentage to fraction
        if t >= target_t:
            if t == target_t:
                return cov_frac
            if t == prev_t:
                return cov_frac
            frac = (target_t - prev_t) / (t - prev_t)
            return prev_c + frac * (cov_frac - prev_c)
        prev_t, prev_c = t, cov_frac
    return (timeline[-1][1] / 100.0) if timeline else 0.0


def main():
    if len(sys.argv) < 2:
        print("Usage: python test/_process_freeze_results.py <freeze_experiment_dir>")
        sys.exit(1)
    
    base = sys.argv[1]
    logs_dir = os.path.join(base, 'logs')
    
    if not os.path.isdir(logs_dir):
        print(f"ERROR: {logs_dir} not found")
        sys.exit(1)
    
    # Load timeline data
    data = defaultdict(list)  # (controller, fault) -> [{'timeline': [...], 'coverage': float, ...}]
    
    for exp_dir in sorted(os.listdir(logs_dir)):
        exp_path = os.path.join(logs_dir, exp_dir)
        if not os.path.isdir(exp_path):
            continue
        
        tl_file = os.path.join(exp_path, 'coverage_timeline.json')
        if not os.path.exists(tl_file):
            continue
        
        with open(tl_file) as f:
            tl_data = json.load(f)
        
        name = tl_data.get('name', exp_dir)
        timeline = tl_data.get('timeline', [])
        
        # Parse name: multiroom_<controller>_<fault>_t<N>
        parts = name.split('_')
        if len(parts) >= 3:
            controller = parts[1]
            # fault is everything between controller and t<N>
            fault_parts = parts[2:-1]
            fault = '_'.join(fault_parts) if fault_parts else 'none'
        else:
            controller = 'unknown'
            fault = 'unknown'
        
        # Coverage stored as 0-100 (percentage), normalize to 0-1
        final_cov = (timeline[-1][1] / 100.0) if timeline else 0.0
        
        # Get detection metrics from result json if available
        result_file = os.path.join(exp_path, 'result.json')
        metrics = {}
        if os.path.exists(result_file):
            with open(result_file) as f:
                metrics = json.load(f)
        
        data[(controller, fault)].append({
            'timeline': timeline,
            'coverage': final_cov,
            'name': name,
            'metrics': metrics,
        })
    
    # Print results
    print("=" * 100)
    print(f"FREEZE LEADER RESULTS  |  Source: {base}")
    print(f"  Experiments loaded: {sum(len(v) for v in data.values())}")
    print("=" * 100)
    
    TIMESTAMPS = [10, 15, 20, 30, 45, 60, 90, 120]
    
    for key in sorted(data.keys()):
        controller, fault = key
        trials = data[key]
        n = len(trials)
        coverages = [t['coverage'] for t in trials]
        
        m_cov = mean(coverages)
        s_cov = std(coverages)
        med_cov = median_val(coverages)
        n_catastrophic = sum(1 for c in coverages if c < 0.70)
        n_perfect = sum(1 for c in coverages if c >= 0.999)
        
        print(f"\n--- {controller} / {fault} (N={n}) ---")
        print(f"  Mean coverage:  {m_cov*100:.1f}% +/- {s_cov*100:.1f}%")
        print(f"  Median coverage: {med_cov*100:.1f}%")
        print(f"  Catastrophic (<70%): {n_catastrophic}/{n}")
        print(f"  Perfect (100%): {n_perfect}/{n}")
        
        # Coverage at fixed timestamps
        cov_at_t = {}
        for ts in TIMESTAMPS:
            vals = [coverage_at_time(t['timeline'], ts) for t in trials]
            cov_at_t[ts] = (mean(vals), std(vals))
        
        # Compute efficiency eta
        # Using trapezoidal integration on mean coverage at timestamps
        mean_covs = [(0, 0.0)] + [(ts, cov_at_t[ts][0]) for ts in TIMESTAMPS]
        eta = 0.0
        for i in range(1, len(mean_covs)):
            t0, c0 = mean_covs[i-1]
            t1, c1 = mean_covs[i]
            eta += (c0 + c1) / 2.0 * (t1 - t0)
        eta = eta / 120.0  # normalize by trial duration
        
        print(f"  Efficiency (eta): {eta*100:.1f}%")
        
        # Detection metrics
        first_suspicions = []
        impeachment_times = []
        det_count = 0
        false_positives = []
        leader_changes = []
        
        for t in trials:
            m = t.get('metrics', {})
            if 'first_suspicion_time' in m and m['first_suspicion_time'] is not None:
                first_suspicions.append(m['first_suspicion_time'])
            if 'first_impeachment_time' in m and m['first_impeachment_time'] is not None:
                impeachment_times.append(m['first_impeachment_time'])
                det_count += 1
            if 'false_positives' in m:
                false_positives.append(m['false_positives'])
            if 'leader_changes' in m:
                leader_changes.append(m['leader_changes'])
        
        if first_suspicions:
            print(f"  First suspicion: {mean(first_suspicions):.2f} +/- {std(first_suspicions):.2f}s  (median {median_val(first_suspicions):.2f}s)")
        if impeachment_times:
            print(f"  Impeachment: {mean(impeachment_times):.2f} +/- {std(impeachment_times):.2f}s  (median {median_val(impeachment_times):.2f}s)")
        if det_count > 0 or fault != 'none':
            print(f"  Detection rate: {det_count}/{n}")
        if false_positives:
            print(f"  False positives: {mean(false_positives):.1f}")
        if leader_changes:
            print(f"  Leader changes: {mean(leader_changes):.1f} +/- {std(leader_changes):.1f}")
        
        # Coverage at timestamps
        print(f"  Coverage @ timestamps: ", end='')
        for ts in TIMESTAMPS:
            m, s = cov_at_t[ts]
            print(f"@{ts}s:{m*100:.1f}%  ", end='')
        print()
        
        # LaTeX table row
        print(f"\n  LaTeX row: {controller:<12s} & Freeze Ldr & {m_cov*100:.1f}\\% & {med_cov*100:.1f}\\% & {n_catastrophic}/{n} & {n_perfect}/{n} \\\\")
        print(f"  Efficiency: {controller:<12s} & Freeze Ldr & {eta*100:.1f}\\% \\\\")
    
    print("\n" + "=" * 100)
    print("Done!")


if __name__ == '__main__':
    main()
