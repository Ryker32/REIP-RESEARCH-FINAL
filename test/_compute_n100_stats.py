"""Compute full N=100 statistics for paper update."""
import json, glob, statistics, os
from collections import defaultdict

DATA_DIR = "experiments/run_20260227_203622_multiroom_100trials_all"
files = glob.glob(os.path.join(DATA_DIR, "results_final_*.json"))
results = []
for f in files:
    with open(f) as fh:
        results.extend(json.load(fh))

print(f"Total results: {len(results)}")

# Group by (controller, fault_type)
groups = defaultdict(list)
for r in results:
    ft = r.get("fault_type") or "none"
    key = (r["controller"], ft)
    groups[key].append(r)

# Coverage table
print()
hdr = f"{'Controller':15s} {'Fault':20s} {'N':>4s} {'Mean':>8s} {'Median':>8s} {'Std':>8s} {'Catas':>8s} {'Perfect':>8s}"
print(hdr)
print("-" * len(hdr))

for key in sorted(groups.keys()):
    trials = groups[key]
    covs = [t["final_coverage"] for t in trials]
    n = len(covs)
    mean_c = statistics.mean(covs)
    med_c = statistics.median(covs)
    std_c = statistics.stdev(covs) if n > 1 else 0
    catas = sum(1 for c in covs if c < 70)
    perfect = sum(1 for c in covs if c >= 99.5)
    print(f"{key[0]:15s} {key[1]:20s} {n:4d} {mean_c:7.1f}% {med_c:7.1f}% {std_c:7.1f}% {catas:4d}/{n:3d} {perfect:4d}/{n:3d}")

# REIP Detection
print()
print("=== REIP Detection Stats ===")
for key in sorted(groups.keys()):
    if key[0] != "reip" or key[1] == "none":
        continue
    trials = groups[key]
    sus_times = [t["first_suspicion_time"] for t in trials if t.get("first_suspicion_time")]
    imp_times = [t["impeachment_time"] for t in trials if t.get("impeachment_time")]
    det_rate = len(sus_times)
    print(f"  {key[1]}:")
    print(f"    Detection rate: {det_rate}/{len(trials)}")
    if sus_times:
        print(f"    First suspicion: mean={statistics.mean(sus_times):.2f}s +/- {statistics.stdev(sus_times):.2f}s, median={statistics.median(sus_times):.2f}s, range=[{min(sus_times):.2f}, {max(sus_times):.2f}]")
    if imp_times:
        print(f"    Impeachment:    mean={statistics.mean(imp_times):.2f}s +/- {statistics.stdev(imp_times):.2f}s, median={statistics.median(imp_times):.2f}s, range=[{min(imp_times):.2f}, {max(imp_times):.2f}]")

# Raft Detection
print()
print("=== Raft Detection Stats ===")
for key in sorted(groups.keys()):
    if key[0] != "raft" or key[1] == "none":
        continue
    trials = groups[key]
    sus_times = [t["first_suspicion_time"] for t in trials if t.get("first_suspicion_time")]
    imp_times = [t["impeachment_time"] for t in trials if t.get("impeachment_time")]
    print(f"  {key[1]}:")
    print(f"    Detection rate: {len(sus_times)}/{len(trials)}")
    if imp_times:
        print(f"    Impeachment: {len(imp_times)}/{len(trials)}")

# Resilience Gap
print()
print("=== Resilience Gap ===")
for ctrl in ["reip", "raft", "decentralized"]:
    clean = groups.get((ctrl, "none"), [])
    bl = groups.get((ctrl, "bad_leader"), [])
    osc = groups.get((ctrl, "oscillate_leader"), [])
    if clean and bl:
        mc = statistics.mean([t["final_coverage"] for t in clean])
        mb = statistics.mean([t["final_coverage"] for t in bl])
        medc = statistics.median([t["final_coverage"] for t in clean])
        medb = statistics.median([t["final_coverage"] for t in bl])
        print(f"  {ctrl} bad_leader:  mean gap = {mb - mc:+.1f}pp  ({mb:.1f} vs {mc:.1f}),  median gap = {medb - medc:+.1f}pp  ({medb:.1f} vs {medc:.1f})")
    if clean and osc:
        mc = statistics.mean([t["final_coverage"] for t in clean])
        mo = statistics.mean([t["final_coverage"] for t in osc])
        medc = statistics.median([t["final_coverage"] for t in clean])
        medo = statistics.median([t["final_coverage"] for t in osc])
        print(f"  {ctrl} oscillate:   mean gap = {mo - mc:+.1f}pp  ({mo:.1f} vs {mc:.1f}),  median gap = {medo - medc:+.1f}pp  ({medo:.1f} vs {medc:.1f})")

# Cumulative coverage efficiency
print()
print("=== Cumulative Coverage Efficiency (eta) ===")
for key in sorted(groups.keys()):
    trials = groups[key]
    etas = []
    for t in trials:
        ts = t.get("coverage_timeline", {})
        if not ts:
            continue
        times = sorted(float(k) for k in ts.keys())
        if len(times) < 2:
            continue
        area = 0.0
        for i in range(1, len(times)):
            dt = times[i] - times[i - 1]
            cov_prev = ts[str(int(times[i - 1]))] if str(int(times[i - 1])) in ts else ts[str(times[i - 1])]
            cov_curr = ts[str(int(times[i]))] if str(int(times[i])) in ts else ts[str(times[i])]
            area += 0.5 * (cov_prev + cov_curr) * dt / 100.0  # normalize to [0,1]
        T = times[-1] - times[0]
        if T > 0:
            etas.append(area / T * 100.0)
    if etas:
        mean_e = statistics.mean(etas)
        med_e = statistics.median(etas)
        print(f"  {key[0]:15s} {key[1]:20s}  mean eta={mean_e:.1f}%  median eta={med_e:.1f}%")
    else:
        print(f"  {key[0]:15s} {key[1]:20s}  no timeline data")

# REIP advantage headline
print()
print("=== HEADLINE: REIP vs Raft under Bad Leader ===")
reip_bl = [t["final_coverage"] for t in groups.get(("reip", "bad_leader"), [])]
raft_bl = [t["final_coverage"] for t in groups.get(("raft", "bad_leader"), [])]
if reip_bl and raft_bl:
    print(f"  REIP mean: {statistics.mean(reip_bl):.1f}%  Raft mean: {statistics.mean(raft_bl):.1f}%  Advantage: +{statistics.mean(reip_bl) - statistics.mean(raft_bl):.1f}pp")
    print(f"  REIP median: {statistics.median(reip_bl):.1f}%  Raft median: {statistics.median(raft_bl):.1f}%  Advantage: +{statistics.median(reip_bl) - statistics.median(raft_bl):.1f}pp")
