import os
import sys
import argparse
import csv
import random
import time
import copy
import yaml

# Ensure we can import src.* when running from scripts/
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from src.main import run as run_sim

# -----------------------------
# Utility: load metrics from a run log
# -----------------------------

def read_log_metrics(log_path: str):
    rows = []
    if not os.path.exists(log_path):
        return rows
    with open(log_path, newline='') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def aggregate_score(rows: list[dict]) -> tuple[float, dict]:
    if not rows:
        return -1e9, {}
    # Convert helpers
    def get_float(r, k, default=0.0):
        try:
            v = r.get(k)
            if v is None or v == "":
                return default
            return float(v)
        except Exception:
            return default
    # Final coverage
    final = rows[-1]
    coverage = get_float(final, 'coverage', 0.0)
    # Elections and impeachments (from last row, cumulative)
    elections = int(get_float(final, 'elections', 0))
    impeach = int(get_float(final, 'impeachments', 0))
    # Averages over the run for graph telemetry
    n = max(1, len(rows))
    avg_deg = sum(get_float(r, 'graph_avg_degree', 0.0) for r in rows) / n
    avg_comp = sum(get_float(r, 'graph_component_count', 1.0) for r in rows) / n
    avg_ttl_risk = sum(get_float(r, 'graph_ttl_risk_frac', 0.0) for r in rows) / n
    avg_fallback = sum(get_float(r, 'fallback_applied', 0.0) for r in rows) / n

    # Scalarized objective (tweak weights as needed):
    # - Maximize coverage
    # - Prefer single component (avg_comp → 1)
    # - Prefer higher avg degree mildly
    # - Penalize excessive elections and fallbacks
    score = (
        100.0 * coverage
        - 50.0 * max(0.0, avg_comp - 1.0)
        + 2.5 * avg_deg
        - 5.0 * elections
        - 2.0 * impeach
        - 1.0 * avg_fallback
        - 10.0 * avg_ttl_risk
    )
    details = {
        'coverage': coverage,
        'avg_deg': avg_deg,
        'avg_comp': avg_comp,
        'avg_ttl_risk': avg_ttl_risk,
        'elections': elections,
        'impeachments': impeach,
        'avg_fallback': avg_fallback,
        'score': score,
    }
    return score, details


# -----------------------------
# Parameter sampling
# -----------------------------

def sample_params(rng: random.Random):
    """Return a dict of REIP parameters to set for a trial.

    Keep ranges conservative to avoid aggressive behavior.
    """
    return {
        # Assignment spacing / anti-bunching
        'frontier_spacing_radius': rng.randint(2, 6),
        'frontier_spacing_extra_safe': rng.randint(0, 2),
        'prox_radius': rng.randint(5, 9),
        # Regional diversity
        'region_capacity': 1,  # keep at 1 for most tests
        'region_size': rng.randint(6, 14),
        # Adaptive connectivity baseline strength
        'adapt_scale_min': round(rng.uniform(0.3, 0.6), 2),
        # Degree cohesion soft penalty
        'degree_min_target': rng.randint(1, 2),
        'degree_under_penalty': round(rng.uniform(0.05, 0.5), 2),
        # Tether and heading (light touch)
        'tether_lambda': round(rng.uniform(0.0, 0.4), 2),
        'heading_gamma': round(rng.uniform(0.0, 0.3), 2),
        'heading_risk_min_degree': rng.randint(1, 2),
    }


# -----------------------------
# Main tuner
# -----------------------------

def run_trial(base_cfg: dict, params: dict, trial_id: int, rep_id: int = 0, max_timesteps: int | None = None, seed_offset: int = 0):
    cfg = copy.deepcopy(base_cfg)
    # Apply overrides
    reip = cfg.setdefault('reip', {})
    for k, v in params.items():
        reip[k] = v
    # Optionally shorten runs for rapid tuning
    if max_timesteps is not None:
        cfg['T'] = int(max_timesteps)
    # Use a deterministic per-trial seed to reduce variance
    if 'env' not in cfg:
        cfg['env'] = {}
    base_seed = int(cfg['env'].get('seed', 42))
    cfg['env']['seed'] = base_seed + seed_offset + trial_id

    # Name runs to segregate logs
    run_tag = cfg.get('name', 'auto')
    stamp = time.strftime('%Y%m%d_%H%M%S')
    # Ensure unique path even under parallel execution
    pid = os.getpid()
    cfg['name'] = os.path.join('auto', run_tag, f"{stamp}_p{pid}_trial{trial_id}_rep{rep_id}")

    # Execute
    run_sim(cfg, visualize=False, debug=False)

    # Read metrics
    log_path = os.path.join('runs', cfg['name'], 'log.csv')
    rows = read_log_metrics(log_path)
    score, details = aggregate_score(rows)
    return score, details, cfg


def _worker(args_tuple):
    base_cfg, params, trial_id, rep_id, max_timesteps, seed_offset = args_tuple
    return trial_id, run_trial(base_cfg, params, trial_id, rep_id, max_timesteps, seed_offset)


def main():
    ap = argparse.ArgumentParser(description='Auto-tune REIP spacing/connectivity parameters via random search.')
    ap.add_argument('--config', type=str, required=True, help='Base YAML config to start from (e.g., configs/reip_adversarial_spread.yaml)')
    ap.add_argument('--trials', type=int, default=20, help='Number of random trials')
    ap.add_argument('--repeats', type=int, default=1, help='Repeat each trial with different seeds and average scores')
    ap.add_argument('--max_timesteps', type=int, default=160, help='Override T to shorten runs during tuning')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--jobs', type=int, default=None, help='Number of parallel processes (default: cpu_count-1)')
    args = ap.parse_args()

    with open(args.config, 'r') as f:
        base_cfg = yaml.safe_load(f)
    # Ensure policy is REIP
    if base_cfg.get('policy') not in ('reip_true', 'src.policy.reip_true.REIPController'):
        base_cfg['policy'] = 'reip_true'

    rng = random.Random(args.seed)
    best = None
    best_params = None
    best_details = None
    best_cfg = None

    # Parallel execution across repeats and/or trials
    jobs = args.jobs if args.jobs is not None else max(1, (os.cpu_count() or 2) - 1)
    from concurrent.futures import ProcessPoolExecutor, as_completed

    trial_counter = 0
    for trial in range(args.trials):
        params = sample_params(rng)
        # Dispatch repeats in parallel
        tasks = []
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            for r in range(args.repeats):
                trial_id = trial  # stable ID per trial for aggregation
                args_tuple = (base_cfg, params, trial_id, r, args.max_timesteps, r)
                tasks.append(ex.submit(_worker, args_tuple))

            # Collect
            scores = []
            detail_acc = None
            cfg_used = None
            for fut in as_completed(tasks):
                t_id, (score, details, cfg_ret) = fut.result()
                scores.append(score)
                cfg_used = cfg_ret
                if detail_acc is None:
                    detail_acc = details
                else:
                    for k, v in details.items():
                        if isinstance(v, (int, float)):
                            detail_acc[k] += v

        avg_score = sum(scores) / float(len(scores)) if scores else -1e9
        if detail_acc is not None and scores:
            for k in list(detail_acc.keys()):
                if isinstance(detail_acc[k], (int, float)):
                    detail_acc[k] /= float(len(scores))
        print(f"Trial {trial+1}/{args.trials} params={params} avg_score={avg_score:.3f} details={detail_acc}")

        if best is None or avg_score > best:
            best = avg_score
            best_params = params
            best_details = detail_acc
            best_cfg = copy.deepcopy(cfg_used) if cfg_used is not None else None

    # Save best params into a YAML next to the base config
    out_dir = os.path.join('runs', 'auto_best')
    os.makedirs(out_dir, exist_ok=True)
    best_out = {
        'base_config': args.config,
        'best_score': best,
        'best_params': best_params,
        'best_details': best_details,
    }
    out_path = os.path.join(out_dir, 'best_params.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(best_out, f)

    # Also emit a fully materialized config with best params merged
    if best_cfg is not None:
        best_cfg_path = os.path.join(out_dir, 'best_config.yaml')
        with open(best_cfg_path, 'w') as f:
            yaml.safe_dump(best_cfg, f)
    print(f"\nBest score={best:.3f}\nSaved: {out_path} and {best_cfg_path if best_cfg is not None else '(no cfg)'}")


if __name__ == '__main__':
    main()
