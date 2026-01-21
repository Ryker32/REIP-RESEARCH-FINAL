"""
Comparative Analysis: REIP vs Baseline Leader-Follower

Demonstrates REIP's advantages over traditional fixed-leader systems under:
1. Clean conditions (no faults)
2. Adversarial conditions (hallucinations + communication loss)

Metrics compared:
- Final coverage
- Coverage efficiency (coverage / time)
- Elections (REIP only)
- System resilience under faults
"""

import os, sys, yaml, copy, csv, math
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
import argparse

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
from src.policy.reip_true import REIPController
from src.policy.baseline_leader_follower import BaselineLeaderFollower
from src.policy.frontier import assign_frontiers_entropy_global


def load_cfg(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _run_single_worker(args):
    """Worker function for parallel execution.
    
    Must be at module level for multiprocessing to work.
    """
    cfg, run_id = args
    # Re-import here to ensure modules are available in worker process
    import sys
    import os
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))
    _PROJECT_ROOT = os.path.dirname(_THIS_DIR)
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    if os.path.join(_PROJECT_ROOT, 'src') not in sys.path:
        sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))
    
    from src.env.gridworld import GridWorld
    from src.comms.channel import share_maps
    from src.policy.reip_true import REIPController
    from src.policy.baseline_leader_follower import BaselineLeaderFollower
    from src.policy.frontier import assign_frontiers_entropy_global
    
    # Re-define run_single_experiment locally to avoid pickling issues
    def run_single_experiment_local(cfg):
        """Run a single experiment and return metrics."""
        env = GridWorld(cfg)
        
        # Load appropriate controller
        policy_name = cfg.get('policy', 'reip_true')
        policy_key = str(policy_name).lower()
        controller = None
        policy_fn = None

        if policy_key in ('reip_true', 'reip', 'src.policy.reip_true.reipcontroller'):
            controller = REIPController(cfg)
        elif policy_key in ('baseline', 'baseline_leader_follower', 'src.policy.baseline_leader_follower.baselineleaderfollower'):
            controller = BaselineLeaderFollower(cfg)
        elif policy_key in ('entropy_global', 'src.policy.frontier.assign_frontiers_entropy_global'):
            policy_fn = assign_frontiers_entropy_global
        else:
            raise ValueError(f"Unknown policy: {policy_name}")
        
        state = {'t': 0, 'prev': {}, 'leader': 0, 'N': cfg['N']}
        coverage_history = []
        
        # Performance optimization: sample coverage less frequently for history
        coverage_sample_rate = max(1, int(cfg.get('coverage_sample_rate', 1)))
        
        # Run simulation
        for t in range(cfg['T']):
            state['t'] = t
            share_maps(env, 
                       R=cfg.get('comm', {}).get('radius', 2),
                       p_loss=cfg.get('comm', {}).get('p_loss', 0.0),
                       p_corrupt=cfg.get('comm', {}).get('p_corrupt', 0.0))
            
            frontiers = env.detect_frontiers()
            if controller is not None:
                controller.step(env, frontiers, state)
            else:
                assigns, _, _ = policy_fn(env, frontiers, prev_assigns=state.get('prev'))
                env.step(assigns, t=t)
                state['prev'] = assigns
            
            # Only calculate coverage when needed
            if t % coverage_sample_rate == 0 or t == cfg['T'] - 1:
                cov = env.coverage()
                cov = min(1.0, max(0.0, float(cov)))
                coverage_history.append((t, cov))
            else:
                if coverage_history:
                    coverage_history.append((t, coverage_history[-1][1]))
                else:
                    coverage_history.append((t, 0.0))
        
        # Get final metrics
        if controller is not None:
            metrics = controller.get_metrics()
        else:
            metrics = {'election_count': 0}
        
        # Convert coverage_history
        if coverage_history and isinstance(coverage_history[0], tuple):
            coverage_values = [cov for _, cov in coverage_history]
            final_cov = min(1.0, max(0.0, float(coverage_values[-1] if coverage_values else 0.0)))
            coverage_values = [min(1.0, max(0.0, float(c))) for c in coverage_values]
        else:
            final_cov = min(1.0, max(0.0, float(coverage_history[-1] if coverage_history else 0.0)))
            coverage_values = [min(1.0, max(0.0, float(c))) for c in coverage_history]
        
        return {
            'final_coverage': final_cov,
            'coverage_history': coverage_values,
            'elections': metrics.get('election_count', 0),
            'metrics': metrics
        }
    
    result = run_single_experiment_local(cfg)
    return run_id, result

def run_experiment(cfg_path, runs=5, parallel=True, max_workers=None):
    """Run experiment multiple times with different seeds.
    
    Args:
        cfg_path: Path to config file
        runs: Number of runs to execute
        parallel: If True, use multiprocessing (default: True)
        max_workers: Number of parallel workers (default: cpu_count - 1)
    """
    cfg_template = load_cfg(cfg_path)
    env_cfg = cfg_template.setdefault('env', {})
    base_seed = env_cfg.get('seed', 42)
    
    # Prepare configs for all runs
    configs = []
    for run in range(runs):
        cfg = copy.deepcopy(cfg_template)
        cfg.setdefault('env', {})['seed'] = base_seed + run
        configs.append((cfg, run))
    
    results = []
    
    if parallel and runs > 1:
        # Parallel execution using all CPU cores
        if max_workers is None:
            max_workers = max(1, multiprocessing.cpu_count() - 1)
        
        print(f"  Running {runs} experiments in parallel using {max_workers} workers...")
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {executor.submit(_run_single_worker, args): args[1] for args in configs}
            
            completed = 0
            for future in as_completed(futures):
                run_id, result = future.result()
                results.append((run_id, result))
                print(f"  Run {run_id+1}/{runs}: Coverage={result['final_coverage']:.1%}, "
                      f"Elections={result['elections']}")
                completed += 1
        
        # Sort by run_id to maintain order
        results.sort(key=lambda x: x[0])
        results = [r[1] for r in results]
    else:
        # Sequential execution (for debugging)
        for run, cfg in enumerate([c[0] for c in configs]):
            result = run_single_experiment(cfg)
            results.append(result)
            print(f"  Run {run+1}/{runs}: Coverage={result['final_coverage']:.1%}, "
                  f"Elections={result['elections']}")
    
    # Aggregate results with validation
    coverage_values = [r['final_coverage'] for r in results]
    
    # Validate: Check for any values > 100% (should never happen)
    max_coverage = max(coverage_values) if coverage_values else 0.0
    min_coverage = min(coverage_values) if coverage_values else 0.0
    if max_coverage > 1.0:
        print(f"  ⚠️  WARNING: Found coverage value > 100%: {max_coverage:.2%}")
    
    coverage_mean = np.mean(coverage_values)
    coverage_std = np.std(coverage_values)
    
    # For bounded data [0, 1], clamp the confidence interval
    # Std can suggest values > 100%, but actual data is bounded
    coverage_min_actual = min_coverage
    coverage_max_actual = max_coverage
    
    return {
        'coverage_mean': coverage_mean,
        'coverage_std': coverage_std,
        'coverage_min': coverage_min_actual,
        'coverage_max': coverage_max_actual,
        'elections_mean': np.mean([r['elections'] for r in results]),
        'elections_std': np.std([r['elections'] for r in results]),
        'coverage_histories': [r['coverage_history'] for r in results],
        'all_results': results
    }


def run_single_experiment(cfg):
    """Run a single experiment and return metrics."""
    env = GridWorld(cfg)
    
    # Load appropriate controller
    policy_name = cfg.get('policy', 'reip_true')
    policy_key = str(policy_name).lower()
    controller = None
    policy_fn = None

    if policy_key in ('reip_true', 'reip', 'src.policy.reip_true.reipcontroller'):
        controller = REIPController(cfg)
    elif policy_key in ('baseline', 'baseline_leader_follower', 'src.policy.baseline_leader_follower.baselineleaderfollower'):
        controller = BaselineLeaderFollower(cfg)
    elif policy_key in ('entropy_global', 'src.policy.frontier.assign_frontiers_entropy_global'):
        policy_fn = assign_frontiers_entropy_global
    else:
        raise ValueError(f"Unknown policy: {policy_name}")
    
    state = {'t': 0, 'prev': {}, 'leader': 0, 'N': cfg['N']}
    coverage_history = []
    
    # Performance optimization: sample coverage less frequently for history
    # (we still need final coverage, but don't need every single timestep)
    coverage_sample_rate = max(1, int(cfg.get('coverage_sample_rate', 1)))  # Default: every step, but can be increased
    
    # Run simulation
    for t in range(cfg['T']):
        state['t'] = t
        share_maps(env, 
                   R=cfg.get('comm', {}).get('radius', 2),
                   p_loss=cfg.get('comm', {}).get('p_loss', 0.0),
                   p_corrupt=cfg.get('comm', {}).get('p_corrupt', 0.0))
        
        frontiers = env.detect_frontiers()
        if controller is not None:
            controller.step(env, frontiers, state)
        else:
            assigns, _, _ = policy_fn(env, frontiers, prev_assigns=state.get('prev'))
            env.step(assigns, t=t)
            state['prev'] = assigns
        
        # Only calculate coverage when needed (every Nth step or at the end)
        if t % coverage_sample_rate == 0 or t == cfg['T'] - 1:
            cov = env.coverage()
            # Safety: ensure coverage never exceeds 1.0 (100%)
            cov = min(1.0, max(0.0, float(cov)))
            coverage_history.append((t, cov))
        else:
            # Interpolate: use last known coverage for intermediate steps
            if coverage_history:
                coverage_history.append((t, coverage_history[-1][1]))
            else:
                coverage_history.append((t, 0.0))
    
    # Get final metrics
    if controller is not None:
        metrics = controller.get_metrics()
    else:
        metrics = {
            'election_count': 0
        }
    
    # Convert coverage_history from (t, cov) tuples to just coverage values
    # (for backward compatibility with plotting code)
    if coverage_history and isinstance(coverage_history[0], tuple):
        coverage_values = [cov for _, cov in coverage_history]
        final_cov = min(1.0, max(0.0, float(coverage_values[-1] if coverage_values else 0.0)))
        # Clamp all history values
        coverage_values = [min(1.0, max(0.0, float(c))) for c in coverage_values]
    else:
        # Legacy format: just a list of coverage values
        final_cov = min(1.0, max(0.0, float(coverage_history[-1] if coverage_history else 0.0)))
        coverage_values = [min(1.0, max(0.0, float(c))) for c in coverage_history]
    
    return {
        'final_coverage': final_cov,
        'coverage_history': coverage_values,
        'elections': metrics.get('election_count', 0),
        'metrics': metrics
    }


def _plot_histories_with_mean(ax, histories, color, label, max_traces=80):
    """Plot a subset of coverage histories plus their mean curve."""
    histories_arr = np.asarray(histories)
    n_runs, _ = histories_arr.shape
    rng = np.random.default_rng(0)  # deterministic subset for reproducibility
    if n_runs > max_traces:
        idx = rng.choice(n_runs, size=max_traces, replace=False)
        subset = histories_arr[idx]
    else:
        subset = histories_arr
    for hist in subset:
        ax.plot(hist, color=color, alpha=0.15, linewidth=0.7)
    ax.plot(
        np.mean(histories_arr, axis=0),
        color=color,
        linewidth=1.2,
        label=label,
    )


def _save_coverage_panel(baseline_data, reip_data, title, output_path):
    """Save a single coverage–time panel in an IEEE-friendly style."""
    import matplotlib as mpl

    # Local style: serif, small fonts, thin lines.
    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "axes.linewidth": 0.5,
        }
    )

    fig, ax = plt.subplots(figsize=(3.5, 2.4))

    # Plot individual trajectories (subset) and mean curves.
    _plot_histories_with_mean(
        ax, baseline_data["coverage_histories"], color="#E63946", label="Baseline"
    )
    _plot_histories_with_mean(
        ax, reip_data["coverage_histories"], color="#2A9D8F", label="REIP"
    )

    ax.set_xlabel("Timestep")
    ax.set_ylabel("Coverage")
    # IEEE: caption will carry the title; keep in-plot title minimal.
    ax.set_title(title, fontsize=9)
    ax.set_ylim(0.0, 1.0)
    ax.grid(True, alpha=0.2, axis="y")

    # Remove top/right spines for cleaner look.
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    for spine in ["left", "bottom"]:
        ax.spines[spine].set_linewidth(0.5)

    ax.legend(fontsize=7, frameon=False)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Save high-res PNG plus vector PDF for IEEE submission.
    fig.savefig(output_path, dpi=600, bbox_inches="tight")
    fig.savefig(output_path.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


def plot_comparison(baseline_clean, baseline_adv, reip_clean, reip_adv, output='comparison.png'):
    """Create comprehensive comparison plots."""
    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(2, 3, hspace=0.3, wspace=0.3)
    
    # Plot 1: Coverage over time (Clean)
    ax1 = fig.add_subplot(gs[0, 0])
    _plot_histories_with_mean(
        ax1, baseline_clean['coverage_histories'], color='#E63946', label='Baseline'
    )
    _plot_histories_with_mean(
        ax1, reip_clean['coverage_histories'], color='#2A9D8F', label='REIP'
    )
    
    ax1.set_xlabel('Timestep', fontsize=11)
    ax1.set_ylabel('Coverage', fontsize=11)
    ax1.set_title('Clean Conditions (No Faults)', fontsize=13, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([0, 1])
    
    # Plot 2: Coverage over time (Adversarial)
    ax2 = fig.add_subplot(gs[0, 1])
    _plot_histories_with_mean(
        ax2, baseline_adv['coverage_histories'], color='#E63946', label='Baseline'
    )
    _plot_histories_with_mean(
        ax2, reip_adv['coverage_histories'], color='#2A9D8F', label='REIP'
    )
    
    ax2.set_xlabel('Timestep', fontsize=11)
    ax2.set_ylabel('Coverage', fontsize=11)
    ax2.set_title('Adversarial Conditions (Faults)', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 1])
    
    # Plot 3: Final Coverage Comparison
    ax3 = fig.add_subplot(gs[0, 2])
    systems = ['Baseline\nClean', 'REIP\nClean', 'Baseline\nAdversarial', 'REIP\nAdversarial']
    means = [
        baseline_clean['coverage_mean'],
        reip_clean['coverage_mean'],
        baseline_adv['coverage_mean'],
        reip_adv['coverage_mean']
    ]
    # Use SEM (standard error of the mean) for error bars to better communicate uncertainty
    n_bc = max(1, len(baseline_clean.get('all_results', [])))
    n_rc = max(1, len(reip_clean.get('all_results', [])))
    n_ba = max(1, len(baseline_adv.get('all_results', [])))
    n_ra = max(1, len(reip_adv.get('all_results', [])))
    stds_sem = [
        baseline_clean['coverage_std'] / np.sqrt(n_bc),
        reip_clean['coverage_std'] / np.sqrt(n_rc),
        baseline_adv['coverage_std'] / np.sqrt(n_ba),
        reip_adv['coverage_std'] / np.sqrt(n_ra)
    ]
    colors = ['#E63946', '#2A9D8F', '#E63946', '#2A9D8F']
    
    bars = ax3.bar(systems, means, yerr=stds_sem, color=colors, alpha=0.7, 
                   capsize=5, edgecolor='black', linewidth=2)
    ax3.set_ylabel('Final Coverage', fontsize=11)
    ax3.set_title('Coverage Comparison (Error bars: SEM)', fontsize=13, fontweight='bold')
    ax3.set_ylim([0, 1])
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, mean in zip(bars, means):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                f'{mean:.1%}',
                ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # Plot 4: Elections (REIP only)
    ax4 = fig.add_subplot(gs[1, 0])
    election_data = [
        ('REIP\nClean', reip_clean['elections_mean'], reip_clean['elections_std']),
        ('REIP\nAdversarial', reip_adv['elections_mean'], reip_adv['elections_std'])
    ]
    labels = [d[0] for d in election_data]
    means = [d[1] for d in election_data]
    stds = [d[2] for d in election_data]
    
    bars = ax4.bar(labels, means, yerr=stds, color='#2A9D8F', alpha=0.7,
                   capsize=5, edgecolor='black', linewidth=2)
    ax4.set_ylabel('Number of Elections', fontsize=11)
    ax4.set_title('REIP Governance Activity', fontsize=13, fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    
    for bar, mean in zip(bars, means):
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                f'{int(mean)}',
                ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # Plot 5: Resilience Metrics (positive = good)
    ax5 = fig.add_subplot(gs[1, 1])
    
    # Compute resilience: coverage gain/loss under faults (adv - clean)
    baseline_gain = (baseline_adv['coverage_mean'] - baseline_clean['coverage_mean']) * 100
    reip_gain = (reip_adv['coverage_mean'] - reip_clean['coverage_mean']) * 100
    
    labels = ['Baseline', 'REIP']
    values = [baseline_gain, reip_gain]
    colors = ['#E63946', '#2A9D8F']
    
    bars = ax5.bar(labels, values, color=colors, alpha=0.7, edgecolor='black', linewidth=2)
    ax5.axhline(0, color='black', linewidth=1)
    ax5.set_ylabel('Coverage Change Under Faults (pp)', fontsize=11)
    ax5.set_title('System Resilience (Adversarial - Clean)', fontsize=13, fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='y')
    
    for bar, val in zip(bars, values):
        height = bar.get_height()
        offset = 1 if val >= 0 else -3
        ax5.text(bar.get_x() + bar.get_width()/2., height + offset,
                f'{val:+.1f} pp',
                ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    # Plot 6: Summary Table
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.axis('off')
    
    baseline_drop = (baseline_clean['coverage_mean'] - baseline_adv['coverage_mean'])
    reip_drop = (reip_clean['coverage_mean'] - reip_adv['coverage_mean'])
    summary_text = f"""
    === COMPARATIVE RESULTS ===
    
    CLEAN CONDITIONS:
    Baseline: {baseline_clean['coverage_mean']:.1%} ± {baseline_clean['coverage_std']*100/np.sqrt(n_bc):.2f} pp (SEM)
    REIP:     {reip_clean['coverage_mean']:.1%} ± {reip_clean['coverage_std']*100/np.sqrt(n_rc):.2f} pp (SEM)
    Difference: {(reip_clean['coverage_mean'] - baseline_clean['coverage_mean'])*100:+.1f} pp
    
    ADVERSARIAL CONDITIONS:
    Baseline: {baseline_adv['coverage_mean']:.1%} ± {baseline_adv['coverage_std']*100/np.sqrt(n_ba):.2f} pp (SEM)
    REIP:     {reip_adv['coverage_mean']:.1%} ± {reip_adv['coverage_std']*100/np.sqrt(n_ra):.2f} pp (SEM)
    Advantage: {(reip_adv['coverage_mean'] - baseline_adv['coverage_mean'])*100:+.1f} pp (~{((reip_adv['coverage_mean'] - baseline_adv['coverage_mean'])/max(1e-9, baseline_adv['coverage_mean'])*100):.1f}%)
    
    RESILIENCE:
    Baseline Drop: {baseline_drop*100:.1f}%
    REIP Drop:     {reip_drop*100:.1f}%
    
    Note: Error bars show SEM (std/√n). All coverage values are bounded to [0, 1].
    """
    
    ax6.text(0.1, 0.95, summary_text, transform=ax6.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    plt.suptitle('REIP vs Traditional Leader-Follower: Comparative Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.savefig(output, dpi=150, bbox_inches='tight')
    print(f"\n✓ Comparison plot saved to: {output}")

    # Additionally save IEEE-style single-panel coverage plots for the paper.
    out_path = Path(output)
    paper_dir = out_path.parent / "paper_figures"
    _save_coverage_panel(
        baseline_clean,
        reip_clean,
        "Clean Conditions (No Faults)",
        paper_dir / f"{out_path.stem}_coverage_clean.png",
    )
    _save_coverage_panel(
        baseline_adv,
        reip_adv,
        "Adversarial Conditions (Faults)",
        paper_dir / f"{out_path.stem}_coverage_adversarial.png",
    )



if __name__ == '__main__':
    import argparse

def main():
    parser = argparse.ArgumentParser(
        description="Benchmark REIP governance against baseline leader-follower controllers."
    )
    parser.add_argument('--runs', type=int, default=5, help='Number of runs per configuration seed sweep.')
    parser.add_argument('--output', default='results/reip_vs_baseline.png', help='Path to save the comparison figure.')
    parser.add_argument('--auto_index_output', action='store_true', help='Automatically append an index to the output filename if it already exists.')
    parser.add_argument('--export_csv', default=None, help='Optional CSV path to export aggregate metrics for LaTeX plots.')
    parser.add_argument('--export_histories', default=None, help='Optional directory to export per-scenario coverage histories as CSVs.')
    parser.add_argument('--baseline_clean', default='configs/benchmarks/baseline_clean.yaml', help='Baseline clean benchmark config.')
    parser.add_argument('--baseline_faulted', default='configs/benchmarks/baseline_faulted.yaml', help='Baseline faulted benchmark config.')
    parser.add_argument('--reip_clean', default='configs/benchmarks/reip_clean.yaml', help='REIP clean benchmark config.')
    parser.add_argument('--reip_faulted', default='configs/benchmarks/reip_faulted.yaml', help='REIP faulted benchmark config.')
    parser.add_argument('--decentralized_clean', default='configs/benchmarks/decentralized_clean.yaml', help='Decentralized clean benchmark config.')
    parser.add_argument('--skip_baseline', action='store_true', help='Skip running baseline scenarios.')
    parser.add_argument('--skip_reip', action='store_true', help='Skip running REIP scenarios.')
    parser.add_argument('--include_decentralized', action='store_true', help='Include decentralized controller comparison.')
    parser.add_argument('--parallel', action='store_true', default=True, help='Use multiprocessing for parallel runs (default: True).')
    parser.add_argument('--no-parallel', dest='parallel', action='store_false', help='Disable parallel execution (for debugging).')
    parser.add_argument('--max_workers', type=int, default=None, help='Number of parallel workers (default: cpu_count - 1).')
    args = parser.parse_args()

    output_path = Path(args.output)
    # Always auto-index output files to preserve all runs for bookkeeping
    stem, suffix = output_path.stem, output_path.suffix
    file_counter = 0
    candidate = output_path
    while candidate.exists():
        candidate = output_path.with_name(f"{stem}_{file_counter:03d}{suffix}")
        file_counter += 1
    output_path = candidate
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("REIP VS BASELINE LEADER-FOLLOWER COMPARISON")
    print("=" * 70)
    print(f"\nRunning {args.runs} trials per configuration...\n")

    baseline_clean = baseline_adv = reip_clean = reip_adv = decentralized_clean = None
    summary_rows = []

    def append_summary(tag: str, data: dict):
        summary_rows.append({
            'scenario': tag,
            'coverage_mean': data['coverage_mean'],
            'coverage_std': data['coverage_std'],
            'coverage_min': data.get('coverage_min', data['coverage_mean'] - data['coverage_std']),
            'coverage_max': data.get('coverage_max', data['coverage_mean'] + data['coverage_std']),
            'elections_mean': data['elections_mean'],
            'elections_std': data['elections_std']
        })

    # Print parallelization info
    if args.parallel:
        workers = args.max_workers if args.max_workers else max(1, multiprocessing.cpu_count() - 1)
        print(f"🚀 Parallel execution enabled: Using {workers} workers (CPU cores: {multiprocessing.cpu_count()})")
    else:
        print("⚠️  Parallel execution disabled (sequential mode)")

    if args.skip_baseline:
        print("Skipping baseline scenarios (per --skip_baseline).")
    else:
        print("\n1. Baseline - Clean Conditions:")
        baseline_clean = run_experiment(args.baseline_clean, runs=args.runs,
                                       parallel=args.parallel, max_workers=args.max_workers)
        append_summary('baseline_clean', baseline_clean)

        print("\n2. Baseline - Adversarial Conditions:")
        baseline_adv = run_experiment(args.baseline_faulted, runs=args.runs,
                                     parallel=args.parallel, max_workers=args.max_workers)
        append_summary('baseline_faulted', baseline_adv)

    if args.skip_reip:
        print("\nSkipping REIP scenarios (per --skip_reip).")
    else:
        print("\n3. REIP - Clean Conditions:")
        reip_clean = run_experiment(args.reip_clean, runs=args.runs,
                                   parallel=args.parallel, max_workers=args.max_workers)
        append_summary('reip_clean', reip_clean)

        print("\n4. REIP - Adversarial Conditions:")
        reip_adv = run_experiment(args.reip_faulted, runs=args.runs,
                                 parallel=args.parallel, max_workers=args.max_workers)
        append_summary('reip_faulted', reip_adv)

    if args.include_decentralized:
        print("\n5. Decentralized Controller - Clean Conditions:")
        decentralized_clean = run_experiment(args.decentralized_clean, runs=args.runs)
        append_summary('decentralized_clean', decentralized_clean)

    if not args.skip_baseline and not args.skip_reip:
        print("\nGenerating comparison plots...")
        plot_comparison(baseline_clean, baseline_adv, reip_clean, reip_adv, output=str(output_path))
    else:
        print("\nSkipping comparison plot because required baseline and/or REIP scenarios were not executed.")

    if args.export_csv:
        csv_path = Path(args.export_csv)
        # Auto-index CSV to match PNG numbering for bookkeeping (use same counter)
        csv_stem, csv_suffix = csv_path.stem, csv_path.suffix
        csv_candidate = csv_path.with_name(f"{csv_stem}_{file_counter:03d}{csv_suffix}")
        csv_path = csv_candidate
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open('w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=['scenario', 'coverage_mean', 'coverage_std', 'coverage_min', 'coverage_max', 'elections_mean', 'elections_std'])
            writer.writeheader()
            for row in summary_rows:
                writer.writerow(row)
        print(f"\n✓ Metrics exported to: {csv_path}")

    # Optionally export full coverage histories for later plotting.
    if args.export_histories:
        hist_dir = Path(args.export_histories)
        hist_dir.mkdir(parents=True, exist_ok=True)

        def write_histories(tag: str, data: dict):
            if not data or 'coverage_histories' not in data:
                return
            histories = np.asarray(data['coverage_histories'])
            n_runs, T = histories.shape
            out_path = hist_dir / f"{tag}_histories.csv"
            with out_path.open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['run_id', 'timestep', 'coverage'])
                for run_id in range(n_runs):
                    for t in range(T):
                        writer.writerow([run_id, t, histories[run_id, t]])
            print(f"✓ Histories exported for {tag} to: {out_path}")

        write_histories('baseline_clean', baseline_clean)
        write_histories('baseline_faulted', baseline_adv)
        write_histories('reip_clean', reip_clean)
        write_histories('reip_faulted', reip_adv)

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    for row in summary_rows:
        label = row['scenario'].replace('_', ' ').title()
        std_pp = row['coverage_std'] * 100
        mean_val = row['coverage_mean']
        min_val = row.get('coverage_min', mean_val - row['coverage_std'])
        max_val = row.get('coverage_max', mean_val + row['coverage_std'])
        print(f"\n{label}:")
        print(f"  Coverage: {mean_val:.1%} ± {std_pp:.2f} pp (range: {min_val:.1%} - {max_val:.1%})")
        if row['elections_mean'] > 0:
            print(f"  Elections: {row['elections_mean']:.1f} ± {row['elections_std']:.1f}")

    if baseline_adv and reip_adv:
        advantage = (reip_adv['coverage_mean'] - baseline_adv['coverage_mean']) / max(1e-9, baseline_adv['coverage_mean'])
        print(f"\n🎯 REIP Advantage under faults: {advantage * 100:+.1f}%")

    print(f"\n✓ Analysis complete!")


if __name__ == '__main__':
    main()
