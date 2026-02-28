"""Collect results from timeline files when main process fails."""
import json, os, sys
from collections import defaultdict

try:
    import numpy as np
except ImportError:
    print("pip install numpy")
    sys.exit(1)

logs_dir = sys.argv[1] if len(sys.argv) > 1 else "experiments/run_20260226_175052_open_10trials_all/logs"

results = []
for exp_dir in sorted(os.listdir(logs_dir)):
    tl_path = os.path.join(logs_dir, exp_dir, "coverage_timeline.json")
    if os.path.exists(tl_path):
        with open(tl_path) as f:
            data = json.load(f)
        tl = data.get("timeline", [])
        final = tl[-1][1] if tl else 0
        t50 = next((t for t, c in tl if c >= 50), None)
        t60 = next((t for t, c in tl if c >= 60), None)
        t80 = next((t for t, c in tl if c >= 80), None)
        results.append({
            "name": exp_dir,
            "controller": data["controller"],
            "fault": data.get("fault_type") or "none",
            "final": final,
            "t50": t50,
            "t60": t60,
            "t80": t80,
        })

groups = defaultdict(list)
for r in results:
    groups[(r["controller"], r["fault"])].append(r)

print(f"Collected {len(results)} timelines from {logs_dir}")
print()

hdr = f"{'Controller':>15} {'Fault':<12} {'N':>3}  {'Coverage':>14}  {'T->50%':>14}  {'T->60%':>14}  {'T->80%':>14}"
print(hdr)
print("-" * len(hdr))

for (ctrl, fault), trials in sorted(groups.items()):
    covs = [t["final"] for t in trials]
    t50s = [t["t50"] for t in trials if t["t50"] is not None]
    t60s = [t["t60"] for t in trials if t["t60"] is not None]
    t80s = [t["t80"] for t in trials if t["t80"] is not None]

    cm, cs = np.mean(covs), np.std(covs)
    t5m = np.mean(t50s) if t50s else float("nan")
    t5s = np.std(t50s) if t50s else float("nan")
    t6m = np.mean(t60s) if t60s else float("nan")
    t6s = np.std(t60s) if t60s else float("nan")
    t8m = np.mean(t80s) if t80s else float("nan")
    t8s = np.std(t80s) if t80s else float("nan")

    print(f"{ctrl:>15} {fault:<12} {len(trials):>3}  "
          f"{cm:>5.1f}+/-{cs:<5.1f}  "
          f"{t5m:>5.1f}+/-{t5s:<5.1f}  "
          f"{t6m:>5.1f}+/-{t6s:<5.1f}  "
          f"{t8m:>5.1f}+/-{t8s:<5.1f}")

# Also print per-trial for conditions with high std
print()
print("=== Per-trial detail (conditions with std > 10) ===")
for (ctrl, fault), trials in sorted(groups.items()):
    covs = [t["final"] for t in trials]
    if np.std(covs) > 10:
        print(f"\n  {ctrl} / {fault}:")
        for t in sorted(trials, key=lambda x: x["final"]):
            print(f"    {t['name']:>45}  cov={t['final']:>5.1f}%  t50={t['t50']}")
