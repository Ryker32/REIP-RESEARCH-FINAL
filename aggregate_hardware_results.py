#!/usr/bin/env python3
"""
Aggregate all hardware trial results and compute paper metrics.

Usage:
    python aggregate_hardware_results.py

Outputs:
    - Summary table matching paper format
    - Individual trial details
    - Detection times for fault conditions
"""
import json
import os
import glob
import re
from collections import defaultdict
from statistics import mean, median, stdev

# Paper arena: 2000x1500mm, multiroom layout
TOTAL_EXPLORABLE_CELLS = 135  # From parse_trial.py
TRIAL_DURATION = 120  # seconds

def parse_trial_dir(trial_dir):
    """Parse a single trial directory and extract metrics."""
    meta_path = os.path.join(trial_dir, 'trial_meta.json')
    if not os.path.exists(meta_path):
        return None
    
    with open(meta_path) as f:
        meta = json.load(f)
    
    controller = meta.get('controller', 'unknown')
    fault_type = meta.get('fault_type', 'none')
    fault1_time = meta.get('fault1_actual_time')
    fault1_robot = meta.get('fault1_robot')
    
    # Parse robot logs
    log_files = sorted(glob.glob(os.path.join(trial_dir, 'r*_robot_*.jsonl')))
    if not log_files:
        return None
    
    all_entries = {}
    for logfile in log_files:
        rid = int(os.path.basename(logfile).split('_')[0][1:])
        entries = []
        with open(logfile) as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if line:
                    try:
                        entries.append(json.loads(line))
                    except json.JSONDecodeError:
                        # Skip malformed lines
                        continue
        if entries:
            all_entries[rid] = entries
    
    if not all_entries:
        return None
    
    # Find trial start time (first entry across all robots)
    t0 = min(e[0]['t'] for e in all_entries.values() if e)
    
    # Coverage: max known_visited_count across all robots at end
    final_known = max(e[-1].get('known_visited_count', 0) for e in all_entries.values() if e)
    coverage_pct = final_known / TOTAL_EXPLORABLE_CELLS * 100
    
    # Speed: compute from position changes
    speeds = []
    for rid, entries in all_entries.items():
        if len(entries) < 2:
            continue
        positions = [(e['t'], e['x'], e['y']) for e in entries if 'x' in e and 'y' in e]
        if len(positions) < 2:
            continue
        
        total_dist = 0
        for i in range(1, len(positions)):
            t1, x1, y1 = positions[i-1]
            t2, x2, y2 = positions[i]
            dt = t2 - t1
            if dt > 0:
                dx = x2 - x1
                dy = y2 - y1
                dist = (dx*dx + dy*dy) ** 0.5
                total_dist += dist
        
        if positions:
            duration = positions[-1][0] - positions[0][0]
            if duration > 0:
                avg_speed = total_dist / duration  # mm/s
                speeds.append(avg_speed)
    
    avg_speed = mean(speeds) if speeds else 0
    
    # Detection time: find first trust drop below 0.85 after fault injection
    detection_time = None
    if fault1_time and fault1_robot and controller == 'reip':
        # fault1_time is already absolute, need to convert to relative
        # Find when fault actually happened relative to trial start
        fault_t_rel = None
        for rid, entries in all_entries.items():
            for e in entries:
                # Check if this entry is close to fault injection time
                if abs(e['t'] - fault1_time) < 5.0:  # Within 5s
                    fault_t_rel = e['t'] - t0
                    break
            if fault_t_rel is not None:
                break
        
        if fault_t_rel is None:
            # Fallback: use meta fault time if available
            fault_t_rel = fault1_time - t0 if fault1_time > t0 else 20.0  # Default to 20s
        
        # Now find first trust drop after fault
        for rid, entries in all_entries.items():
            if rid == fault1_robot:
                continue  # Skip the leader itself
            for e in entries:
                t_rel = e['t'] - t0
                if t_rel >= fault_t_rel:
                    trust = e.get('trust_in_leader', 1.0)
                    if trust < 0.85:
                        detection_time = t_rel - fault_t_rel
                        break
            if detection_time is not None:
                break
    
    return {
        'controller': controller,
        'fault': fault_type,
        'trial_dir': trial_dir,
        'coverage': coverage_pct,
        'speed': avg_speed,
        'detection_time': detection_time,
        'fault1_time': fault1_time - t0 if fault1_time else None,
        'n_robots': len(all_entries),
    }


def main():
    # Find all trial directories - filter to recent ones (20260315) if specified
    all_trial_dirs = sorted(glob.glob('trials/*/'))
    
    # Filter to only trials from the run dates in the guide (20260315)
    # Or use all if you want everything
    trial_dirs = [d for d in all_trial_dirs if '20260315' in d]
    
    if not trial_dirs:
        print("No trial directories found for 20260315, using all trials")
        trial_dirs = all_trial_dirs
    
    if not trial_dirs:
        print("No trial directories found in 'trials/'")
        return
    
    print(f"Found {len(trial_dirs)} trial directories (filtered to 20260315)\n")
    
    # Parse all trials
    all_results = []
    for trial_dir in trial_dirs:
        result = parse_trial_dir(trial_dir)
        if result:
            all_results.append(result)
    
    # Group by (controller, fault)
    groups = defaultdict(list)
    for r in all_results:
        key = (r['controller'], r['fault'])
        groups[key].append(r)
    
    # Print summary table (matching paper format)
    print("=" * 80)
    print("HARDWARE RESULTS SUMMARY (Paper Table Format)")
    print("=" * 80)
    print(f"{'Ctrl.':<8} {'Fault':<15} {'N':>3}  {'Coverage':>10}  {'Speed':>8}  {'Detect':>8}")
    print(f"{'':8} {'':15} {'':3}  {'@120s':>10}  {'(mm/s)':>8}  {'(s)':>8}")
    print("-" * 80)
    
    # Order: REIP first, then Raft; within each: none, bad_leader, freeze_leader, self_injure_leader
    order = [
        ('reip', 'none'),
        ('reip', 'bad_leader'),
        ('reip', 'freeze_leader'),
        ('reip', 'self_injure_leader'),
        ('raft', 'none'),
        ('raft', 'bad_leader'),
        ('raft', 'freeze_leader'),
        ('raft', 'self_injure_leader'),
    ]
    
    summary_data = {}
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in groups:
            continue
        
        trials = groups[key]
        n = len(trials)
        coverages = [t['coverage'] for t in trials]
        speeds = [t['speed'] for t in trials]
        detections = [t['detection_time'] for t in trials if t['detection_time'] is not None]
        
        avg_coverage = mean(coverages)
        avg_speed = mean(speeds)
        avg_detect = mean(detections) if detections else None
        
        # Format fault name for paper
        fault_display = {
            'none': 'None',
            'bad_leader': 'Bad Leader',
            'freeze_leader': 'Freeze Ldr',
            'self_injure_leader': 'Self-Injure',
        }.get(fault, fault)
        
        ctrl_display = ctrl.upper()
        detect_str = f"{avg_detect:.1f}" if avg_detect is not None else "---"
        
        print(f"{ctrl_display:<8} {fault_display:<15} {n:>3}  {avg_coverage:>9.1f}%  {avg_speed:>7.0f}  {detect_str:>8}")
        
        summary_data[key] = {
            'n': n,
            'coverage': avg_coverage,
            'speed': avg_speed,
            'detection': avg_detect,
            'trials': trials,
        }
    
    print("=" * 80)
    print()
    
    # Print detailed breakdown
    print("DETAILED BREAKDOWN BY CONDITION")
    print("=" * 80)
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in summary_data:
            continue
        
        data = summary_data[key]
        print(f"\n{ctrl.upper()} + {fault}:")
        print(f"  N = {data['n']} trials")
        print(f"  Coverage: {data['coverage']:.1f}% (mean)")
        print(f"  Speed: {data['speed']:.0f} mm/s (mean)")
        if data['detection']:
            print(f"  Detection: {data['detection']:.1f}s (mean)")
        
        # Individual trial values
        print("  Individual trials:")
        for i, trial in enumerate(data['trials'], 1):
            detect_str = f", detect={trial['detection_time']:.1f}s" if trial['detection_time'] else ""
            print(f"    Trial {i}: {trial['coverage']:.1f}%, {trial['speed']:.0f} mm/s{detect_str}")
    
    # LaTeX table format
    print("\n" + "=" * 80)
    print("LATEX TABLE FORMAT (copy into paper)")
    print("=" * 80)
    print("\\begin{tabular}{@{}llcccc@{}}")
    print("\\toprule")
    print("\\textbf{Ctrl.} & \\textbf{Fault} & $N$ & \\textbf{Cov.} &")
    print("\\textbf{Speed} & \\textbf{Detect} \\\\")
    print(" & & & \\textbf{@120s} & \\textbf{(mm/s)} & \\textbf{(s)} \\\\")
    print("\\midrule")
    
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in summary_data:
            continue
        
        data = summary_data[key]
        ctrl_display = ctrl.capitalize()
        fault_display = {
            'none': 'None',
            'bad_leader': 'Bad Leader',
            'freeze_leader': 'Freeze Ldr',
            'self_injure_leader': 'Self-Injure',
        }.get(fault, fault)
        
        detect_str = f"{data['detection']:.1f}" if data['detection'] else "---"
        
        print(f"{ctrl_display:<8} & {fault_display:<15} & {data['n']:>3}  & {data['coverage']:>9.1f}\\% & "
              f"{data['speed']:>7.0f} & {detect_str:>8} \\\\")
    
    print("\\bottomrule")
    print("\\end{tabular}")


if __name__ == '__main__':
    main()
