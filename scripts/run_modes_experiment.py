import os, sys, yaml, csv, math
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

from scripts.compare_reip_vs_baseline import run_single_experiment as _crv_run_single

MODES = ["baseline", "baseline_fallback", "reip_gov", "reip_full"]
ENVS = ["clean", "adversarial"]

# Map modes/env to the exact configs used in compare_reip_vs_baseline.py
MODE_ENV_CONFIG = {
    ("baseline", "clean"): "configs/benchmarks/baseline_clean.yaml",
    ("baseline", "adversarial"): "configs/benchmarks/baseline_faulted.yaml",
    ("baseline_fallback", "clean"): "configs/benchmarks/baseline_clean.yaml",
    ("baseline_fallback", "adversarial"): "configs/benchmarks/baseline_faulted.yaml",
    ("reip_gov", "clean"): "configs/benchmarks/reip_clean.yaml",
    ("reip_gov", "adversarial"): "configs/benchmarks/reip_faulted.yaml",
    ("reip_full", "clean"): "configs/benchmarks/reip_clean.yaml",
    ("reip_full", "adversarial"): "configs/benchmarks/reip_faulted.yaml",
}

def _next_available_file(path: Path) -> Path:
    if not path.exists():
        return path
    stem, suffix = path.stem, path.suffix
    idx = 0
    while True:
        candidate = path.with_name(f"{stem}_{idx}{suffix}")
        if not candidate.exists():
            return candidate
        idx += 1


def _next_run_subdir(base_dir: Path) -> Path:
    base_dir.mkdir(parents=True, exist_ok=True)
    idx = 0
    while True:
        candidate = base_dir / f"run_{idx:03d}"
        if not candidate.exists():
            candidate.mkdir(parents=True, exist_ok=False)
            return candidate
        idx += 1


def load_yaml(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def build_config(mode: str, env_profile: str):
    cfg_path = MODE_ENV_CONFIG[(mode, env_profile)]
    cfg = load_yaml(cfg_path)
    cfg['mode'] = mode
    cfg['env_profile'] = env_profile

    # Force coverage sampling every step for consistency
    cfg['coverage_sample_rate'] = 1

    reip = cfg.setdefault('reip', {})

    # Governance flags
    if mode in ("reip_gov", "reip_full"):
        reip['governance_enabled'] = True
    else:
        reip['governance_enabled'] = False

    # Fallback handling
    if mode == "baseline_fallback":
        cfg['fallback_enabled'] = True
    else:
        cfg['fallback_enabled'] = bool(cfg.get('fallback_enabled', False) and mode == "baseline")

    if mode == "reip_gov":
        reip['fallback_enabled'] = False
    elif mode == "reip_full":
        reip['fallback_enabled'] = True

    return cfg


def run_single(cfg, compute_t95=False):
    # Delegate to the canonical runner to ensure identical metrics/behavior
    res = _crv_run_single(cfg)
    out = {
        'final_coverage': float(res['final_coverage']),
        'elections': int(res.get('elections', 0)),
        'coverage_history': list(res.get('coverage_history', [])),
        'reached_95': False,
        'time_to_95': None,
    }
    if compute_t95 and out['coverage_history']:
        thresh = float(cfg.get('time_to_95_threshold', 0.95))
        for t, c in enumerate(out['coverage_history']):
            if c >= thresh:
                out['reached_95'] = True
                out['time_to_95'] = t
                break
    return out


def run_suite(base_cfg_path: str, runs: int, out_csv: str, seeds: list[int] | None = None, max_workers: int | None = None, compute_t95=False, fixed_seed=False):
    # base_cfg_path kept for compatibility; per-mode configs are loaded via MODE_ENV_CONFIG
    if seeds is None:
        seeds = list(range(runs))
    assert len(seeds) >= runs

    rows = []
    tasks = []
    configs = []

    # Prepare all jobs
    for env_profile in ENVS:
        for mode in MODES:
            for i in range(runs):
                cfg = build_config(mode, env_profile)
                base_seed = int(cfg.get('env', {}).get('seed', 42))
                cfg.setdefault('env', {})['seed'] = base_seed if fixed_seed else (base_seed + int(seeds[i]))
                configs.append((mode, env_profile, seeds[i], cfg))

    # Parallel execution
    if max_workers is None:
        max_workers = max(1, (multiprocessing.cpu_count() or 2) - 1)
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        futs = {}
        for (mode, envp, seed, cfg) in configs:
            fut = ex.submit(run_single, cfg, compute_t95)
            futs[fut] = (mode, envp, seed)
        for fut in as_completed(futs):
            mode, envp, seed = futs[fut]
            res = fut.result()
            rows.append({
                'mode': mode,
                'env': envp,
                'seed': int(seed),
                'final_coverage': float(res['final_coverage']),
                'elections_count': int(res['elections']),
                'reached_95': bool(res.get('reached_95', False)),
                'time_to_95': ('' if res.get('time_to_95', None) is None else int(res['time_to_95'])),
            })
            cov_pct = res['final_coverage'] if isinstance(res['final_coverage'], float) else 0.0
            t95_disp = res.get('time_to_95', None)
            t95_str = 'None' if t95_disp is None else str(t95_disp)
            print(f"[{envp} | {mode}] seed={seed} -> cov={cov_pct:.1%}, elec={res['elections']}, t95={t95_str}")

    # Save CSV
    Path(os.path.dirname(out_csv) or '.').mkdir(parents=True, exist_ok=True)
    with open(out_csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['mode','env','seed','final_coverage','elections_count','reached_95','time_to_95'])
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r['env'], r['mode'], r['seed'])):
            w.writerow(r)
    print(f"\nOK Wrote results to {out_csv}")

    return rows


def analyze_and_plot(rows: list[dict], out_dir: str):
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    # Group by (env, mode)
    groups = {}
    for r in rows:
        key = (r['env'], r['mode'])
        groups.setdefault(key, []).append(r)

    # Stats table
    stats = {}
    for key, lst in groups.items():
        cov = np.array([x['final_coverage'] for x in lst], dtype=float)
        n = len(cov)
        mean = float(np.mean(cov))
        std = float(np.std(cov))
        sem = float(std / math.sqrt(max(1, n)))
        mn = float(np.min(cov))
        mx = float(np.max(cov))
        stats[key] = dict(n=n, mean=mean, std=std, sem=sem, min=mn, max=mx)

    # Bar plots: clean and adversarial
    for envp in ENVS:
        modes = MODES
        means = [stats[(envp, m)]['mean'] for m in modes]
        sems = [stats[(envp, m)]['sem'] for m in modes]
        plt.figure(figsize=(8,4))
        plt.bar(modes, means, yerr=sems, capsize=5, color=['#E63946','#A8DADC','#2A9D8F','#1D7874'])
        plt.ylim([0,1])
        n_per_mode = stats[(envp, modes[0])]['n']
        plt.title(f"Final Coverage ({envp.title()})  n~{n_per_mode} per mode")
        plt.ylabel('Coverage')
        for i, m in enumerate(means):
            plt.text(i, m+0.02, f"{m*100:.1f}%", ha='center', fontsize=9)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, f'coverage_{envp}.png'), dpi=150)
        plt.close()

    # Resilience: adv - clean per mode
    drops = []
    for m in MODES:
        adv = stats[('adversarial', m)]['mean']
        cln = stats[('clean', m)]['mean']
        drops.append(adv - cln)
    plt.figure(figsize=(7,4))
    plt.bar(MODES, drops, color=['#E63946','#A8DADC','#2A9D8F','#1D7874'])
    plt.axhline(0, color='k', linewidth=1)
    for i, d in enumerate(drops):
        plt.text(i, d + (0.01 if d>=0 else -0.05), f"{d*100:+.1f} pp", ha='center')
    plt.title('Resilience (Adversarial − Clean)')
    plt.ylabel('ΔCoverage (pp)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'resilience.png'), dpi=150)
    plt.close()

    # Pairwise comparisons (adversarial)
    def report_pair(a, b):
        ma = stats[('adversarial', a)]['mean']
        mb = stats[('adversarial', b)]['mean']
        sa = stats[('adversarial', a)]['sem']
        sb = stats[('adversarial', b)]['sem']
        d = (mb - ma) * 100.0
        se = math.sqrt(sa*sa + sb*sb) * 100.0
        ci = 1.96 * se
        return d, se, ci

    pairs = [
        ('baseline', 'baseline_fallback'),
        ('baseline', 'reip_gov'),
        ('baseline', 'reip_full'),
        ('baseline_fallback', 'reip_full'),
        ('reip_gov', 'reip_full'),
    ]

    with open(os.path.join(out_dir, 'pairwise_adv.txt'), 'w', encoding='utf-8') as f:
        for a, b in pairs:
            d, se, ci = report_pair(a, b)
            f.write(f"{a} -> {b}: Δµ={d:+.2f} pp  (SE={se:.2f} pp, 95% CI +/-{ci:.2f} pp)\n")
    print(f"OK Wrote pairwise analysis to {os.path.join(out_dir, 'pairwise_adv.txt')}")

    # --------------- Time-to-95% summaries ---------------
    t95_path = os.path.join(out_dir, 'time_to_95_summary.txt')
    with open(t95_path, 'w') as f:
        f.write('Time-to-95% Coverage (threshold=0.95)\n')
        for envp in ENVS:
            f.write(f"\n[{envp.upper()}]\n")
            for m in MODES:
                subset = [r for r in rows if r['env']==envp and r['mode']==m]
                n = len(subset)
                succ = [r for r in subset if str(r['reached_95'])=='True' or r['reached_95'] is True]
                n_succ = len(succ)
                rate = (n_succ/ n) if n>0 else 0.0
                times = [int(r['time_to_95']) for r in succ if str(r['time_to_95']).isdigit()]
                if n_succ>0 and len(times)>0:
                    mean_t = float(np.mean(times))
                    std_t = float(np.std(times))
                    sem_t = float(std_t / math.sqrt(max(1, len(times))))
                    f.write(f"{m:18s}  success={rate*100:5.1f}%  time_to_95={mean_t:.1f} +/- {sem_t:.1f} steps (n={len(times)})\n")
                else:
                    f.write(f"{m:18s}  success={rate*100:5.1f}%  time_to_95=-- (n=0)\n")
    print(f"OK Wrote time-to-95% summary to {t95_path}")


def load_rows_from_csv(csv_path: str) -> list[dict]:
    rows = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                'mode': row['mode'],
                'env': row['env'],
                'seed': int(row['seed']),
                'final_coverage': float(row['final_coverage']),
                'elections_count': int(row.get('elections_count', 0)),
                'reached_95': row.get('reached_95', 'False') in ('True', 'true', '1'),
                'time_to_95': row.get('time_to_95', '') if row.get('time_to_95', '') != '' else None,
            })
    return rows


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Run 4-mode * 2-env experiment suite with shared seeds.')
    ap.add_argument('--base_cfg', type=str, default='configs/benchmarks/reip_clean.yaml', help='Base YAML to start from.')
    ap.add_argument('--runs', type=int, default=200)
    ap.add_argument('--seeds_start', type=int, default=0)
    ap.add_argument('--out_csv', type=str, default='results/modes_results.csv')
    ap.add_argument('--out_dir', type=str, default='results/modes_plots')
    ap.add_argument('--max_workers', type=int, default=None)
    ap.add_argument('--compute_t95', action='store_true', help='Compute time-to-95% coverage for each run.')
    ap.add_argument('--t95_only', action='store_true', help='Generate only time-to-95% summary.')
    ap.add_argument('--fixed_seed', action='store_true', help='Do not increment env.seed per run; keep the base seed identical across runs.')
    ap.add_argument('--analyze_only', action='store_true', help='Skip running simulations; load CSV and regenerate plots/analytics.')
    args = ap.parse_args()

    seeds = list(range(args.seeds_start, args.seeds_start + args.runs))

    if args.analyze_only:
        rows = load_rows_from_csv(args.out_csv)
        analyze_and_plot(rows, args.out_dir)
        return

    out_csv_path = _next_available_file(Path(args.out_csv))
    out_dir_path = _next_run_subdir(Path(args.out_dir))
    print(f" Writing CSV to: {out_csv_path}")
    print(f"  Saving plots to: {out_dir_path}")

    if args.t95_only:
        rows = run_suite(args.base_cfg, args.runs, str(out_csv_path), seeds=seeds,
                         max_workers=args.max_workers, compute_t95=args.compute_t95,
                         fixed_seed=args.fixed_seed)
    else:
        rows = run_suite(args.base_cfg, args.runs, str(out_csv_path), seeds=seeds,
                         max_workers=args.max_workers, compute_t95=args.compute_t95,
                         fixed_seed=args.fixed_seed)
    analyze_and_plot(rows, str(out_dir_path))


if __name__ == '__main__':
    main()
