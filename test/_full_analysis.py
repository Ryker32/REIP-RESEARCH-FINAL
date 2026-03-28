#!/usr/bin/env python3
"""Comprehensive analysis of multiroom experiment results.
Produces every metric imaginable for hand analysis."""

import json, glob, os, sys, math
from collections import defaultdict

base = sys.argv[1] if len(sys.argv) > 1 else 'experiments/run_20260226_192129_multiroom_10trials_all'

# ---- Load main results ----
results_file = glob.glob(os.path.join(base, 'results_final_*.json'))[0]
with open(results_file) as f:
    results = json.load(f)

# ---- Load timeline data from per-experiment directories ----
logs_dir = os.path.join(base, 'logs')
timelines = {}  # name -> [(t, cov), ...]
for exp_dir in os.listdir(logs_dir):
    tl_file = os.path.join(logs_dir, exp_dir, 'coverage_timeline.json')
    if os.path.exists(tl_file):
        with open(tl_file) as f:
            data = json.load(f)
        timelines[data['name']] = data['timeline']

# ---- Helper functions ----
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

def fmt(m, s, decimals=1):
    if m is None:
        return "N/A"
    return f"{m:.{decimals}f} +/- {s:.{decimals}f}"

def time_to_threshold(timeline, threshold):
    for t, cov in timeline:
        if cov >= threshold:
            return t
    return None

def cov_at_time(timeline, target_time):
    last_cov = 0
    for t, cov in timeline:
        if t > target_time:
            return last_cov
        last_cov = cov
    return last_cov

# ---- Group results by condition ----
groups = defaultdict(list)
for r in results:
    ft = r.get('fault_type') or 'none'
    key = (r['controller'], ft)
    groups[key].append(r)

CONDITIONS = [
    ('reip', 'none'), ('reip', 'bad_leader'), ('reip', 'freeze_leader'), ('reip', 'oscillate_leader'), ('reip', 'spin'),
    ('raft', 'none'), ('raft', 'bad_leader'), ('raft', 'freeze_leader'), ('raft', 'oscillate_leader'), ('raft', 'spin'),
    ('decentralized', 'none'), ('decentralized', 'bad_leader'), ('decentralized', 'freeze_leader'), ('decentralized', 'oscillate_leader'), ('decentralized', 'spin'),
]

out = []
def p(s=""):
    out.append(s)
    print(s)

p("=" * 140)
p(f"COMPREHENSIVE MULTIROOM RESULTS  |  Source: {base}")
p(f"  {len(results)} total experiments  |  {len(timelines)} timelines loaded")
p("=" * 140)

# TABLE 1
p("\n" + "=" * 120)
p("TABLE 1: FINAL COVERAGE & RELIABILITY")
p("=" * 120)
p(f"{'Controller':<15} {'Fault':<12} {'N':>3}  {'Mean Cov':>18}  {'Min':>6}  {'Max':>6}  "
  f"{'Median':>6}  {'Catastrophic':>12}  {'Perfect':>8}")
p("-" * 120)
for ctrl, fault in CONDITIONS:
    trials = groups.get((ctrl, fault), [])
    if not trials: continue
    covs = [r['final_coverage'] for r in trials]
    n = len(covs)
    m = mean(covs); s = std(covs)
    mn = min(covs); mx = max(covs); med = sorted(covs)[n // 2]
    catastrophic = sum(1 for c in covs if c < 70)
    perfect = sum(1 for c in covs if c >= 99.5)
    p(f"{ctrl:<15} {fault:<12} {n:>3}  {fmt(m, s):>18}  {mn:>5.1f}%  {mx:>5.1f}%  "
      f"{med:>5.1f}%  {catastrophic:>6}/{n:<5}  {perfect:>4}/{n}")

# TABLE 2
p("\n" + "=" * 180)
p("TABLE 2: TIME TO COVERAGE MILESTONES (seconds, mean +/- std)")
p("=" * 180)
thresholds = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
header = f"{'Controller':<15} {'Fault':<12} {'N':>3}"
for th in thresholds:
    header += f"  {'T->' + str(th) + '%':>14}"
p(header)
p("-" * 180)
for ctrl, fault in CONDITIONS:
    trials = groups.get((ctrl, fault), [])
    if not trials: continue
    row = f"{ctrl:<15} {fault:<12} {len(trials):>3}"
    for th in thresholds:
        times = []
        for r in trials:
            tl = timelines.get(r['name'], [])
            times.append(time_to_threshold(tl, th))
        valid = [t for t in times if t is not None]
        never = len(times) - len(valid)
        if valid:
            m = mean(valid); s = std(valid)
            cell = f"{m:.1f}+/-{s:.1f}"
            if never > 0: cell += f"({never}N)"
            row += f"  {cell:>14}"
        else:
            row += f"  {'NEVER':>14}"
    p(row)

# TABLE 3
p("\n" + "=" * 120)
p("TABLE 3: DETECTION & TRUST METRICS")
p("=" * 120)
p(f"{'Controller':<15} {'Fault':<12} {'N':>3}  {'1st Suspicion':>20}  {'Impeachment':>20}  "
  f"{'Det Rate':>8}  {'FP':>6}  {'Leader Chg':>15}")
p("-" * 120)
for ctrl, fault in CONDITIONS:
    trials = groups.get((ctrl, fault), [])
    if not trials: continue
    susps = [r.get('time_to_first_suspicion') for r in trials]
    dets = [r.get('time_to_detection') for r in trials]
    fps = [r.get('false_positives', 0) for r in trials]
    lcs = [r.get('num_leader_changes', 0) for r in trials]
    susp_valid = [s for s in susps if s is not None]
    det_valid = [d for d in dets if d is not None]
    susp_str = fmt(mean(susp_valid), std(susp_valid), 2) if susp_valid else "N/A"
    det_str = fmt(mean(det_valid), std(det_valid), 2) if det_valid else "N/A"
    det_rate = f"{len(det_valid)}/{len(trials)}" if fault in ('bad_leader', 'freeze_leader') else "N/A"
    fp_mean = mean(fps)
    lc_str = fmt(mean(lcs), std(lcs), 1)
    p(f"{ctrl:<15} {fault:<12} {len(trials):>3}  {susp_str:>20}  {det_str:>20}  "
      f"{det_rate:>8}  {fp_mean:>5.1f}  {lc_str:>15}")

# TABLE 4
p("\n" + "=" * 140)
p("TABLE 4: PER-TRIAL DETAIL -- BAD LEADER CONDITION")
p("=" * 140)
p(f"{'Controller':<15} {'Trial':>5} {'Seed':>6}  {'Coverage':>8}  {'T->50':>8}  {'T->60':>8}  "
  f"{'T->80':>8}  {'T->90':>8}  {'1stSusp':>8}  {'Impeach':>8}  {'FP':>3}  {'LdrChg':>6}")
p("-" * 140)
for ctrl in ['reip', 'raft', 'decentralized']:
    trials = groups.get((ctrl, 'bad_leader'), [])
    for r in sorted(trials, key=lambda x: x['trial']):
        cov = r['final_coverage']
        t50 = f"{r['time_to_50']:.1f}" if r.get('time_to_50') is not None else "NEVER"
        t60 = f"{r['time_to_60']:.1f}" if r.get('time_to_60') is not None else "NEVER"
        t80 = f"{r['time_to_80']:.1f}" if r.get('time_to_80') is not None else "NEVER"
        # compute t90 from timeline
        tl = timelines.get(r['name'], [])
        t90 = time_to_threshold(tl, 90)
        t90_str = f"{t90:.1f}" if t90 is not None else "NEVER"
        susp = f"{r['time_to_first_suspicion']:.2f}" if r.get('time_to_first_suspicion') is not None else "-"
        det = f"{r['time_to_detection']:.2f}" if r.get('time_to_detection') is not None else "-"
        fp = r.get('false_positives', 0)
        lc = r.get('num_leader_changes', 0)
        flag = " ***" if cov < 70 else ""
        p(f"{ctrl:<15} {r['trial']:>5} {r['seed']:>6}  {cov:>7.1f}%  {t50:>8}  {t60:>8}  "
          f"{t80:>8}  {t90_str:>8}  {susp:>8}  {det:>8}  {fp:>3}  {lc:>6}{flag}")
    if trials: p()

# TABLE 5
p("\n" + "=" * 150)
p("TABLE 5: COVERAGE AT FIXED TIMESTAMPS (%, mean +/- std)")
p("=" * 150)
timestamps = [10, 15, 20, 30, 45, 60, 90, 120]
header = f"{'Controller':<15} {'Fault':<12} {'N':>3}"
for ts in timestamps:
    header += f"  {'@' + str(ts) + 's':>14}"
p(header)
p("-" * 150)
for ctrl, fault in CONDITIONS:
    trials = groups.get((ctrl, fault), [])
    if not trials: continue
    row = f"{ctrl:<15} {fault:<12} {len(trials):>3}"
    for ts in timestamps:
        covs = []
        for r in trials:
            tl = timelines.get(r['name'], [])
            if tl: covs.append(cov_at_time(tl, ts))
        if covs:
            row += f"  {fmt(mean(covs), std(covs)):>14}"
        else:
            row += f"  {'N/A':>14}"
    p(row)

# TABLE 6
p("\n" + "=" * 130)
p("TABLE 6: RESILIENCE GAP (coverage penalty from fault) - Mean and Median")
p("=" * 160)
p(f"{'Controller':<15} {'':>8} {'Clean':>10}  {'BadLeader':>10}  {'BadLdr Gap':>10}  {'Freeze':>10}  {'Frz Gap':>10}  {'Oscillate':>10}  {'Osc Gap':>10}  {'Spin':>10}  {'Spin Gap':>10}")
p("-" * 200)
for ctrl in ['reip', 'raft', 'decentralized']:
    clean = groups.get((ctrl, 'none'), [])
    bad = groups.get((ctrl, 'bad_leader'), [])
    frz = groups.get((ctrl, 'freeze_leader'), [])
    osc = groups.get((ctrl, 'oscillate_leader'), [])
    spin = groups.get((ctrl, 'spin'), [])
    clean_covs = [r['final_coverage'] for r in clean]
    bad_covs = [r['final_coverage'] for r in bad]
    frz_covs = [r['final_coverage'] for r in frz] if frz else []
    osc_covs = [r['final_coverage'] for r in osc] if osc else []
    spin_covs = [r['final_coverage'] for r in spin] if spin else []
    clean_m = mean(clean_covs); bad_m = mean(bad_covs)
    frz_m = mean(frz_covs) if frz_covs else 0
    osc_m = mean(osc_covs) if osc_covs else 0; spin_m = mean(spin_covs) if spin_covs else 0
    clean_med = median_val(clean_covs); bad_med = median_val(bad_covs)
    frz_med = median_val(frz_covs) if frz_covs else 0
    osc_med = median_val(osc_covs) if osc_covs else 0; spin_med = median_val(spin_covs) if spin_covs else 0
    gap_bad_m = f"{bad_m - clean_m:+.1f}pp" if clean_m and bad_m else "N/A"
    gap_frz_m = f"{frz_m - clean_m:+.1f}pp" if clean_m and frz_m else "N/A"
    gap_osc_m = f"{osc_m - clean_m:+.1f}pp" if clean_m and osc_m else "N/A"
    gap_spin_m = f"{spin_m - clean_m:+.1f}pp" if clean_m and spin_m else "N/A"
    gap_bad_med = f"{bad_med - clean_med:+.1f}pp" if clean_med and bad_med else "N/A"
    gap_frz_med = f"{frz_med - clean_med:+.1f}pp" if clean_med and frz_med else "N/A"
    gap_osc_med = f"{osc_med - clean_med:+.1f}pp" if clean_med and osc_med else "N/A"
    gap_spin_med = f"{spin_med - clean_med:+.1f}pp" if clean_med and spin_med else "N/A"
    p(f"{ctrl:<15} {'Mean':>8} {clean_m:>9.1f}%  {bad_m:>9.1f}%  {gap_bad_m:>10}  {frz_m:>9.1f}%  {gap_frz_m:>10}  {osc_m:>9.1f}%  {gap_osc_m:>10}  {spin_m:>9.1f}%  {gap_spin_m:>10}")
    p(f"{'':15} {'Median':>8} {clean_med:>9.1f}%  {bad_med:>9.1f}%  {gap_bad_med:>10}  {frz_med:>9.1f}%  {gap_frz_med:>10}  {osc_med:>9.1f}%  {gap_osc_med:>10}  {spin_med:>9.1f}%  {gap_spin_med:>10}")

# TABLE 7
p("\n" + "=" * 100)
p("TABLE 7: HEAD-TO-HEAD COMPARISONS")
p("=" * 100)
comparisons = [
    ("reip", "raft", "none", "Clean: coordination overhead"),
    ("reip", "raft", "bad_leader", "Bad Leader: trust vs no trust"),
    ("reip", "raft", "freeze_leader", "Freeze Leader: stale assignment detection"),
    ("reip", "raft", "oscillate_leader", "Oscillate Leader: Tier 3 detection"),
    ("reip", "decentralized", "none", "Clean: leadership value"),
    ("reip", "decentralized", "bad_leader", "Bad Leader: resilience"),
    ("reip", "decentralized", "freeze_leader", "Freeze Leader: resilience"),
    ("reip", "decentralized", "oscillate_leader", "Oscillate: resilience"),
    ("raft", "decentralized", "none", "Clean: RAFT coordination"),
]
for c1, c2, fault, desc in comparisons:
    t1 = groups.get((c1, fault), [])
    t2 = groups.get((c2, fault), [])
    if not t1 or not t2: continue
    covs1 = [r['final_coverage'] for r in t1]
    covs2 = [r['final_coverage'] for r in t2]
    m1, m2 = mean(covs1), mean(covs2)
    med1, med2 = median_val(covs1), median_val(covs2)
    t50_1 = [r['time_to_50'] for r in t1 if r.get('time_to_50') is not None]
    t50_2 = [r['time_to_50'] for r in t2 if r.get('time_to_50') is not None]
    cat1 = sum(1 for c in covs1 if c < 70)
    cat2 = sum(1 for c in covs2 if c < 70)
    p(f"\n  {c1.upper()} vs {c2.upper()} | {fault} | {desc}")
    p(f"    Mean cov:     {c1} {m1:.1f}% vs {c2} {m2:.1f}% = {m1-m2:+.1f}pp")
    p(f"    Median cov:   {c1} {med1:.1f}% vs {c2} {med2:.1f}% = {med1-med2:+.1f}pp")
    if t50_1 and t50_2:
        p(f"    T->50%:       {c1} {mean(t50_1):.1f}s vs {c2} {mean(t50_2):.1f}s = {mean(t50_1)-mean(t50_2):+.1f}s")
    p(f"    Catastrophic: {c1} {cat1}/{len(covs1)} vs {c2} {cat2}/{len(covs2)}")

# TABLE 8
p("\n" + "=" * 120)
p("TABLE 8: PER-TRIAL FINAL COVERAGE -- ALL CONDITIONS (sorted)")
p("=" * 120)
for ctrl, fault in CONDITIONS:
    trials = groups.get((ctrl, fault), [])
    if not trials: continue
    covs = sorted([r['final_coverage'] for r in trials])
    p(f"  {ctrl:>15} {fault:<12}: {' '.join(f'{c:5.1f}' for c in covs)}  | mean={mean(covs):.1f} std={std(covs):.1f}")

p("\n\nDone. All tables above.")

# Save to file
outfile = os.path.join(base, 'FULL_ANALYSIS.txt')
with open(outfile, 'w') as f:
    f.write('\n'.join(out))
print(f"\nSaved to: {outfile}")
