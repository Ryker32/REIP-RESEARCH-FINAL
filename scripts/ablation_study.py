"""
Ablation Study: REIP Component Analysis

Focuses on the 4-mode ablation path:
1. Baseline - fixed leader, no fallback, no governance
2. Baseline+Fallback - Baseline + local fallback autonomy
3. REIP-Gov - Baseline + governance (trust, detection, impeachment, election), no fallback
4. REIP-Full - REIP-Gov + fallback

Metrics (adversarial faults only):
- Final coverage at horizon
- Success rate (fraction reaching 95%)
- Median time-to-95% (on successful runs)

Figure layout:
- Panel (a): Final coverage bar chart
- Panel (b): Success rate bar chart
- Panel (c): Median time-to-95% (optional)
"""

import os, sys, yaml, copy, csv, math
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing
import argparse
from datetime import datetime

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
from src.policy.reip_true import REIPController
from src.policy.baseline_leader_follower import BaselineLeaderFollower


def load_cfg(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _run_single_worker(args):
    """Worker function for parallel execution."""
    cfg, run_id = args
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
    
    def run_single_experiment_local(cfg):
        """Run a single experiment and return metrics."""
        env = GridWorld(cfg)
        
        # Load appropriate controller
        policy_name = cfg.get('policy', 'reip_true')
        policy_key = str(policy_name).lower()
        
        if policy_key in ('reip_true', 'reip', 'src.policy.reip_true.reipcontroller'):
            controller = REIPController(cfg)
        elif policy_key in ('baseline', 'baseline_leader_follower', 'src.policy.baseline_leader_follower.baselineleaderfollower'):
            controller = BaselineLeaderFollower(cfg)
        else:
            raise ValueError(f"Unknown policy: {policy_name}")
        
        state = {'t': 0, 'prev': {}, 'leader': 0, 'N': cfg['N']}
        coverage_history = []
        coverage_sample_rate = max(1, int(cfg.get('coverage_sample_rate', 1)))
        
        # Track time to 95%
        time_to_95 = None
        
        for t in range(cfg['T']):
            state['t'] = t
            share_maps(env, 
                       R=cfg.get('comm', {}).get('radius', 2),
                       p_loss=cfg.get('comm', {}).get('p_loss', 0.0),
                       p_corrupt=cfg.get('comm', {}).get('p_corrupt', 0.0))
            
            frontiers = env.detect_frontiers()
            controller.step(env, frontiers, state)
            
            if t % coverage_sample_rate == 0:
                cov = env.coverage()
                coverage_history.append(cov)
                # Track first time reaching 95%
                if time_to_95 is None and cov >= 0.95:
                    time_to_95 = t
        
        final_coverage = env.coverage()
        elections = controller.election_count if hasattr(controller, 'election_count') else 0
        impeachments = controller.impeachment_count if hasattr(controller, 'impeachment_count') else 0
        
        return {
            'final_coverage': final_coverage,
            'coverage_history': coverage_history,
            'elections': elections,
            'impeachments': impeachments,
            'reached_95': final_coverage >= 0.95,
            'reached_90': final_coverage >= 0.90,
            'time_to_95': time_to_95,  # None if never reached
        }
    
    try:
        # Use seed = run_id (0 to runs-1) to match benchmark trials
        unique_seed = run_id
        
        # Set random seeds BEFORE creating environment
        np.random.seed(unique_seed)
        import random
        random.seed(unique_seed)
        
        # Update config with unique seed so GridWorld uses it
        if 'env' not in cfg:
            cfg['env'] = {}
        cfg['env']['seed'] = unique_seed
        
        result = run_single_experiment_local(cfg)
        result['run_id'] = run_id
        result['success'] = True
        return result
    except Exception as e:
        return {
            'run_id': run_id,
            'success': False,
            'error': str(e),
            'final_coverage': 0.0,
            'elections': 0,
            'impeachments': 0,
            'reached_95': False,
            'time_to_95': None,
        }


def create_ablation_configs(base_faulted_path):
    """Create the 4-mode ablation configs (adversarial only).
    
    Variants:
    1. Baseline - fixed leader, no fallback, no governance
    2. Baseline+Fallback - Baseline + local fallback (need to add this to baseline)
    3. REIP-Gov - governance only, no fallback
    4. REIP-Full - governance + fallback
    """
    faulted_cfg = load_cfg(base_faulted_path)
    variants = {}
    
    # 1. Baseline (faulted) - fixed leader, no fallback
    variants['baseline'] = copy.deepcopy(faulted_cfg)
    variants['baseline']['name'] = 'ablation/baseline'
    variants['baseline']['policy'] = 'baseline_leader_follower'
    if 'reip' not in variants['baseline']:
        variants['baseline']['reip'] = {}
    variants['baseline']['reip']['fallback_enabled'] = False
    # Fixed leader baseline (no round-robin rotations)
    # Also need to copy hallucination schedule and command_loss_rate to top level for baseline
    if 'reip' in faulted_cfg and 'hallucination_schedule' in faulted_cfg['reip']:
        variants['baseline']['hallucination_schedule'] = faulted_cfg['reip']['hallucination_schedule']
        variants['baseline']['hallucination_rate'] = faulted_cfg['reip'].get('hallucination_rate', 0.0)
    if 'reip' in faulted_cfg and 'command_loss_rate' in faulted_cfg['reip']:
        variants['baseline']['command_loss_rate'] = faulted_cfg['reip']['command_loss_rate']
    
    # 2. Baseline+Fallback (faulted) - fixed leader + local fallback autonomy
    variants['baseline_fallback'] = copy.deepcopy(faulted_cfg)
    variants['baseline_fallback']['name'] = 'ablation/baseline_fallback'
    variants['baseline_fallback']['policy'] = 'baseline_leader_follower'
    variants['baseline_fallback']['fallback_enabled'] = True
    if 'reip' not in variants['baseline_fallback']:
        variants['baseline_fallback']['reip'] = {}
    variants['baseline_fallback']['reip']['fallback_enabled'] = True
    # Governance stays disabled; fallback handles autonomy
    if 'reip' in faulted_cfg and 'hallucination_schedule' in faulted_cfg['reip']:
        variants['baseline_fallback']['hallucination_schedule'] = faulted_cfg['reip']['hallucination_schedule']
        variants['baseline_fallback']['hallucination_rate'] = faulted_cfg['reip'].get('hallucination_rate', 0.0)
    if 'reip' in faulted_cfg and 'command_loss_rate' in faulted_cfg['reip']:
        variants['baseline_fallback']['command_loss_rate'] = faulted_cfg['reip']['command_loss_rate']
    
    # 3. REIP-Gov (faulted, no fallback)
    variants['reip_gov'] = copy.deepcopy(faulted_cfg)
    variants['reip_gov']['name'] = 'ablation/reip_gov'
    variants['reip_gov']['policy'] = 'reip_true'
    if 'reip' not in variants['reip_gov']:
        variants['reip_gov']['reip'] = {}
    variants['reip_gov']['reip']['fallback_enabled'] = False
    
    # 4. REIP-Full (faulted, with fallback)
    variants['reip_full'] = copy.deepcopy(faulted_cfg)
    variants['reip_full']['name'] = 'ablation/reip_full'
    variants['reip_full']['policy'] = 'reip_true'
    if 'reip' not in variants['reip_full']:
        variants['reip_full']['reip'] = {}
    variants['reip_full']['reip']['fallback_enabled'] = True
    
    return variants


def run_ablation_study(faulted_config_path, runs=2000, max_workers=6, output_dir='results/ablation'):
    """Run ablation study focusing on adversarial conditions."""
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 70)
    print("REIP ABLATION STUDY (4-Mode Path)")
    print("=" * 70)
    print(f"Faulted config: {faulted_config_path}")
    print(f"Runs per variant: {runs}")
    print(f"Max workers: {max_workers}")
    print()
    
    # Create ablation variants
    variants = create_ablation_configs(faulted_config_path)
    variant_names = ['baseline', 'baseline_fallback', 'reip_gov', 'reip_full']
    
    print(f"Testing {len(variant_names)} variants (adversarial only):")
    for name in variant_names:
        print(f"  - {name}")
    print()
    
    # Run experiments
    all_results = {}
    
    for variant_name in variant_names:
        print(f"Running {variant_name}...")
        cfg = variants[variant_name]
        
        args_list = [(cfg, run_id) for run_id in range(runs)]
        
        results = []
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {executor.submit(_run_single_worker, args): args for args in args_list}
            
            completed = 0
            for future in as_completed(futures):
                completed += 1
                if completed % 100 == 0:
                    print(f"  Progress: {completed}/{runs}")
                try:
                    result = future.result()
                    if result.get('success', False):
                        results.append(result)
                except Exception as e:
                    print(f"  Error in worker: {e}")
        
        all_results[variant_name] = results
        print(f"  Completed: {len(results)}/{runs} successful runs")
        print()
    
    # Compute statistics
    stats = {}
    for variant_name, results in all_results.items():
        if not results:
            continue
        
        final_covs = [r['final_coverage'] for r in results]
        elections = [r['elections'] for r in results]
        impeachments = [r['impeachments'] for r in results]
        reached_95 = [r.get('reached_95', False) for r in results]
        times_to_95 = [r.get('time_to_95') for r in results if r.get('time_to_95') is not None]
        
        stats[variant_name] = {
            'mean_coverage': np.mean(final_covs),
            'std_coverage': np.std(final_covs),
            'sem_coverage': np.std(final_covs) / np.sqrt(len(final_covs)),  # Standard error
            'median_coverage': np.median(final_covs),
            'mean_elections': np.mean(elections),
            'std_elections': np.std(elections),
            'mean_impeachments': np.mean(impeachments),
            'success_rate': np.mean(reached_95),
            'success_rate_sem': np.std(reached_95) / np.sqrt(len(reached_95)),
            'median_time_to_95': np.median(times_to_95) if times_to_95 else None,
            'mean_time_to_95': np.mean(times_to_95) if times_to_95 else None,
            'std_time_to_95': np.std(times_to_95) if times_to_95 else None,
            'n_runs': len(results),
            'n_reached_95': sum(reached_95),
        }
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(output_dir, f'ablation_{timestamp}.csv')
    
    with open(csv_path, 'w', newline='') as f:
        # Include all keys from stats dict so we don't drop anything useful
        writer = csv.DictWriter(
            f,
            fieldnames=[
                'variant',
                'mean_coverage',
                'std_coverage',
                'sem_coverage',
                'median_coverage',
                'success_rate',
                'success_rate_sem',
                'median_time_to_95',
                'mean_time_to_95',
                'std_time_to_95',
                'mean_elections',
                'std_elections',
                'mean_impeachments',
                'n_runs',
                'n_reached_95',
            ],
        )
        writer.writeheader()
        for variant_name, stat in stats.items():
            row = {'variant': variant_name, **stat}
            writer.writerow(row)
    
    print(f"Results saved to: {csv_path}")
    
    # Save detailed per-run data for future analysis
    detailed_csv_path = os.path.join(output_dir, f'ablation_detailed_{timestamp}.csv')
    with open(detailed_csv_path, 'w', newline='') as f:
        fieldnames = ['variant', 'run_id', 'final_coverage', 'elections', 'impeachments', 
                      'reached_95', 'reached_90', 'time_to_95']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for variant_name, results in all_results.items():
            for r in results:
                writer.writerow({
                    'variant': variant_name,
                    'run_id': r.get('run_id', 0),
                    'final_coverage': r.get('final_coverage', 0.0),
                    'elections': r.get('elections', 0),
                    'impeachments': r.get('impeachments', 0),
                    'reached_95': r.get('reached_95', False),
                    'reached_90': r.get('reached_90', False),
                    'time_to_95': r.get('time_to_95', '') if r.get('time_to_95') is not None else '',
                })
    print(f"Detailed per-run data saved to: {detailed_csv_path}")
    
    # Save coverage histories for each variant (for future plotting/analysis)
    histories_dir = os.path.join(output_dir, f'histories_{timestamp}')
    os.makedirs(histories_dir, exist_ok=True)
    
    for variant_name, results in all_results.items():
        if not results:
            continue
        
        # Save coverage histories CSV for this variant
        hist_path = os.path.join(histories_dir, f'{variant_name}_histories.csv')
        with open(hist_path, 'w', newline='') as f:
            # Find max history length
            max_len = max(len(r.get('coverage_history', [])) for r in results)
            
            # Write header: run_id, t_0, t_1, ..., t_n
            fieldnames = ['run_id'] + [f't_{i}' for i in range(max_len)]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            # Write each run's history
            for r in results:
                hist = r.get('coverage_history', [])
                # Pad with last value if shorter
                if len(hist) < max_len:
                    hist = hist + [hist[-1]] * (max_len - len(hist)) if hist else [0.0] * max_len
                row = {'run_id': r.get('run_id', 0)}
                for i, cov in enumerate(hist):
                    row[f't_{i}'] = cov
                writer.writerow(row)
        
        print(f"  Coverage histories for {variant_name} saved to: {hist_path}")
    
    print(f"\nAll data saved for future analysis:")
    print(f"  - Summary statistics: {csv_path}")
    print(f"  - Per-run details: {detailed_csv_path}")
    print(f"  - Coverage histories: {histories_dir}/")
    
    # Generate plots
    plot_ablation_results(stats, all_results, output_dir, timestamp)
    
    return stats, all_results


def plot_ablation_results(stats, all_results, output_dir, timestamp, mode_order=None, mode_labels=None):
    """Generate ablation study plots (IEEE style).
    
    Args:
        stats (dict): summary stats keyed by variant.
        all_results (dict): per-run results.
        output_dir (str): directory to write figure files.
        timestamp (str): timestamp string for filenames.
        mode_order (list[str], optional): Variant names to plot (default baseline order).
        mode_labels (list[str], optional): Labels (with line breaks) matching mode_order.
    """
    
    # Configure IEEE-friendly style: serif font, small labels, thin lines
    import matplotlib as mpl
    mpl.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
        'font.size': 8,
        'axes.labelsize': 8,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'axes.linewidth': 0.5,
        'axes.titlesize': 9,
        'legend.fontsize': 7,
    })
    
    if mode_order is None:
        mode_order = ['baseline', 'baseline_fallback', 'reip_gov', 'reip_full']
    if mode_labels is None:
        mode_labels = ['Baseline', 'Baseline+\nFallback', 'REIP-\nGov', 'REIP-\nFull']
    
    # Create direct mapping from mode name to label
    mode_to_label = dict(zip(mode_order, mode_labels))
    
    # Filter to only variants we have
    available_modes = [m for m in mode_order if m in stats]
    
    if not available_modes:
        print("No data to plot!")
        return
    
    # Create figure with subplots (IEEE double-column width ~7 inches)
    fig = plt.figure(figsize=(7, 2.5))
    
    # Panel (a): Final coverage
    ax1 = plt.subplot(1, 3, 1)
    means = [stats[m]['mean_coverage'] * 100 for m in available_modes]
    sems = [stats[m]['sem_coverage'] * 100 for m in available_modes]
    
    x_pos = np.arange(len(available_modes))
    bars = ax1.bar(x_pos, means, yerr=sems, capsize=3, alpha=0.7, 
                   color='steelblue', edgecolor='black', linewidth=0.5)
    
    # Annotate bars with values
    for i, (mean, sem) in enumerate(zip(means, sems)):
        ax1.text(i, mean + sem + 1, f'{mean:.1f}%', ha='center', va='bottom', fontsize=7)
    
    # Remove top/right spines (IEEE style)
    for spine in ['top', 'right']:
        ax1.spines[spine].set_visible(False)
    for spine in ['left', 'bottom']:
        ax1.spines[spine].set_linewidth(0.5)
    
    ax1.set_xlabel('Controller Mode', fontsize=8)
    ax1.set_ylabel('Final Coverage (%)', fontsize=8)
    ax1.set_title('(a) Final Coverage', fontsize=9, fontweight='bold')
    ax1.set_xticks(x_pos)
    # Ensure proper spacing for labels (especially "Baseline")
    labels_a = [mode_to_label[m] for m in available_modes]
    ax1.set_xticklabels(labels_a, rotation=0, ha='center', fontsize=7)
    # Add extra padding if needed to prevent cramping
    ax1.tick_params(axis='x', pad=2)
    ax1.grid(axis='y', alpha=0.3, linestyle='--', linewidth=0.5)
    ax1.set_ylim([0, max(means) * 1.15])
    
    # Panel (b): Success rate
    ax2 = plt.subplot(1, 3, 2)
    success_rates = [stats[m]['success_rate'] * 100 for m in available_modes]
    success_sems = [stats[m]['success_rate_sem'] * 100 for m in available_modes]
    
    bars2 = ax2.bar(x_pos, success_rates, yerr=success_sems, capsize=3, alpha=0.7,
                    color='coral', edgecolor='black', linewidth=0.5)
    
    # Annotate bars
    for i, (rate, sem) in enumerate(zip(success_rates, success_sems)):
        ax2.text(i, rate + sem + 2, f'{rate:.1f}%', ha='center', va='bottom', fontsize=7)
    
    # Remove top/right spines (IEEE style)
    for spine in ['top', 'right']:
        ax2.spines[spine].set_visible(False)
    for spine in ['left', 'bottom']:
        ax2.spines[spine].set_linewidth(0.5)
    
    ax2.set_xlabel('Controller Mode', fontsize=8)
    ax2.set_ylabel('Success Rate (%)', fontsize=8)
    ax2.set_title('(b) Success Rate (95% Coverage)', fontsize=9, fontweight='bold')
    ax2.set_xticks(x_pos)
    labels_b = [mode_to_label[m] for m in available_modes]
    ax2.set_xticklabels(labels_b, rotation=0, ha='center', fontsize=7)
    ax2.tick_params(axis='x', pad=2)
    ax2.grid(axis='y', alpha=0.3, linestyle='--', linewidth=0.5)
    ax2.set_ylim([0, 105])
    
    # Panel (c): Median time-to-95%
    ax3 = plt.subplot(1, 3, 3)
    times_to_95 = []
    time_sems = []
    for m in available_modes:
        if stats[m]['median_time_to_95'] is not None:
            times_to_95.append(stats[m]['median_time_to_95'])
            # Use IQR as error estimate (simpler than SEM for median)
            times = [r.get('time_to_95') for r in all_results[m] if r.get('time_to_95') is not None]
            if times:
                q75, q25 = np.percentile(times, [75, 25])
                time_sems.append((q75 - q25) / 2)
            else:
                time_sems.append(0)
        else:
            times_to_95.append(0)
            time_sems.append(0)
    
    # Only plot modes that reached 95%
    valid_indices = [i for i, t in enumerate(times_to_95) if t > 0]
    if valid_indices:
        valid_modes = [available_modes[i] for i in valid_indices]
        valid_times = [times_to_95[i] for i in valid_indices]
        valid_sems = [time_sems[i] for i in valid_indices]
        valid_x = np.arange(len(valid_indices))
        
        bars3 = ax3.bar(valid_x, valid_times, yerr=valid_sems, capsize=3, alpha=0.7,
                        color='mediumseagreen', edgecolor='black', linewidth=0.5)
        
        # Annotate bars
        for i, (t, sem) in enumerate(zip(valid_times, valid_sems)):
            ax3.text(i, t + sem + 5, f'{t:.0f}', ha='center', va='bottom', fontsize=7)
        
        ax3.set_xticks(valid_x)
        labels_c = [mode_to_label[m] for m in valid_modes]
        ax3.set_xticklabels(labels_c, rotation=0, ha='center', fontsize=7)
        ax3.tick_params(axis='x', pad=2)
    else:
        ax3.text(0.5, 0.5, 'No runs reached 95%', ha='center', va='center', transform=ax3.transAxes, fontsize=8)
        ax3.set_xticks([])
    
    # Remove top/right spines (IEEE style)
    for spine in ['top', 'right']:
        ax3.spines[spine].set_visible(False)
    for spine in ['left', 'bottom']:
        ax3.spines[spine].set_linewidth(0.5)
    
    ax3.set_xlabel('Controller Mode', fontsize=8)
    ax3.set_ylabel('Median Time-to-95%', fontsize=8)
    # Title clarifies that baseline is omitted (success rate = 0%)
    if 'baseline' in available_modes and 'baseline' not in valid_modes:
        ax3.set_title('(c) Time to 95% Coverage\n(successful runs only)', fontsize=9, fontweight='bold')
    else:
        ax3.set_title('(c) Time to 95% Coverage', fontsize=9, fontweight='bold')
    ax3.grid(axis='y', alpha=0.3, linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    # Save at 600 DPI for IEEE (high quality)
    plot_path = os.path.join(output_dir, f'ablation_figure_{timestamp}.png')
    plt.savefig(plot_path, dpi=600, bbox_inches='tight', facecolor='white')
    print(f"Ablation figure saved to: {plot_path}")
    
    # Also save as PDF for paper (vector format, preferred by IEEE)
    pdf_path = os.path.join(output_dir, f'ablation_figure_{timestamp}.pdf')
    plt.savefig(pdf_path, bbox_inches='tight', facecolor='white')
    print(f"Ablation figure (PDF) saved to: {pdf_path}")
    
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Run REIP ablation study (4-mode path)')
    parser.add_argument('--faulted_config', type=str,
                       default='configs/benchmarks/reip_faulted.yaml',
                       help='Faulted environment config file')
    parser.add_argument('--runs', type=int, default=2000,
                       help='Number of runs per variant (default: 2000 for statistical power)')
    parser.add_argument('--max_workers', type=int, default=6,
                       help='Max parallel workers')
    parser.add_argument('--output_dir', type=str, default='results/ablation',
                       help='Output directory')
    
    args = parser.parse_args()
    
    stats, results = run_ablation_study(
        args.faulted_config,
        runs=args.runs,
        max_workers=args.max_workers,
        output_dir=args.output_dir
    )
    
    print("\n" + "=" * 70)
    print("[COMPLETE] ABLATION STUDY COMPLETE")
    print("=" * 70)
    print("\nSummary Statistics (Adversarial Conditions):")
    print("-" * 70)
    mode_order = ['baseline', 'baseline_fallback', 'reip_gov', 'reip_full']
    for variant_name in mode_order:
        if variant_name in stats:
            stat = stats[variant_name]
            print(f"{variant_name:20s}: Coverage={stat['mean_coverage']*100:5.1f}% +/- {stat['sem_coverage']*100:4.1f}%, "
                  f"Success={stat['success_rate']*100:5.1f}% +/- {stat['success_rate_sem']*100:4.1f}%, "
                  f"Time-to-95={stat['median_time_to_95']:.0f}" if stat['median_time_to_95'] else "Time-to-95=N/A")
    print()


if __name__ == '__main__':
    main()
