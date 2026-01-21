import os, sys, yaml, collections
import matplotlib.pyplot as plt
import numpy as np

# ensure src on path
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
_SRC_DIR = os.path.join(_PROJECT_ROOT, 'src')
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
import importlib, inspect


def load_cfg(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def run_simulation(cfg):
    env = GridWorld(cfg)
    policy_path = cfg.get('policy', 'reip')
    if policy_path == 'reip':
        policy_path = 'src.policy.reip.REIPController'
    mod, name = policy_path.rsplit('.', 1)
    symbol = getattr(importlib.import_module(mod), name)
    controller = symbol(cfg) if inspect.isclass(symbol) else None

    state = {'t':0, 'prev': None, 'leader': 0, 'N': cfg.get('N',1)}

    # Metrics storage
    coverage_history = []
    position_history = collections.defaultdict(list)
    assignment_changes = collections.defaultdict(int)
    prev_assigns = {}

    for t in range(cfg['T']):
        state['t'] = t
        share_maps(env, R=cfg.get('comm',{}).get('radius',2), p_loss=cfg.get('comm',{}).get('p_loss',0.0))
        frontiers = env.detect_frontiers()
        
        if controller is None:
            assigns, claims, U_pred = symbol(env, frontiers, lam=cfg.get('lam',0.0), prev_assigns=state.get('prev', {}))
            env.step(assigns, t=t)
            synced = {}
            for i in assigns.keys():
                a = env.agents.get(i)
                held = getattr(a, 'hold_target', None)
                synced[i] = held if held is not None else assigns[i]
            state['prev'] = synced
        else:
            controller.step(env, frontiers, state)
            assigns = state.get('prev', {})

        # Track metrics
        coverage_history.append(env.coverage())
        for i in env.agent_positions.keys():
            position_history[i].append(env.agent_positions[i])
            if i in prev_assigns and prev_assigns[i] != assigns.get(i):
                assignment_changes[i] += 1
        prev_assigns = assigns.copy()

    return coverage_history, position_history, assignment_changes, len(frontiers)


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--config', default='configs/dual_res.yaml')
    p.add_argument('--T', type=int, default=150)
    p.add_argument('--output', default='metrics_plot.png')
    args = p.parse_args()

    cfg = load_cfg(args.config)
    cfg['T'] = args.T

    print(f"Running simulation for T={args.T}...")
    coverage_hist, pos_hist, assign_changes, final_frontiers = run_simulation(cfg)

    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Multi-Agent Exploration Metrics (T={args.T})', fontsize=16, fontweight='bold')

    # Plot 1: Coverage over time
    ax = axes[0, 0]
    ax.plot(coverage_hist, linewidth=2, color='#2E86AB')
    ax.set_xlabel('Timestep', fontsize=12)
    ax.set_ylabel('Coverage', fontsize=12)
    ax.set_title(f'Coverage Over Time (Final: {coverage_hist[-1]:.1%})', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_ylim([0, 1.05])
    
    # Check monotonicity
    decreases = sum(1 for i in range(1, len(coverage_hist)) if coverage_hist[i] < coverage_hist[i-1])
    mono_text = "✓ Monotonic" if decreases == 0 else f"✗ {decreases} decreases"
    ax.text(0.02, 0.98, mono_text, transform=ax.transAxes, 
            fontsize=11, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightgreen' if decreases == 0 else 'lightcoral', alpha=0.8))

    # Plot 2: Unique positions per agent
    ax = axes[0, 1]
    agent_ids = sorted(pos_hist.keys())
    unique_counts = [len(set(pos_hist[i])) for i in agent_ids]
    percentages = [(u / args.T) * 100 for u in unique_counts]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(agent_ids)))
    bars = ax.bar(agent_ids, percentages, color=colors, edgecolor='black', linewidth=1.5)
    ax.set_xlabel('Agent ID', fontsize=12)
    ax.set_ylabel('Unique Positions (%)', fontsize=12)
    ax.set_title('Spatial Diversity (% Unique Cells Visited)', fontsize=13, fontweight='bold')
    ax.set_ylim([0, 105])
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, val in zip(bars, percentages):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{val:.0f}%', ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    avg_diversity = np.mean(percentages)
    ax.axhline(avg_diversity, color='red', linestyle='--', linewidth=2, label=f'Avg: {avg_diversity:.1f}%')
    ax.legend(fontsize=10)

    # Plot 3: Assignment changes per agent
    ax = axes[1, 0]
    changes = [assign_changes.get(i, 0) for i in agent_ids]
    bars = ax.bar(agent_ids, changes, color=colors, edgecolor='black', linewidth=1.5)
    ax.set_xlabel('Agent ID', fontsize=12)
    ax.set_ylabel('Assignment Changes', fontsize=12)
    ax.set_title('Assignment Stability (Lower = More Stable)', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, val in zip(bars, changes):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                f'{val}', ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    avg_changes = np.mean(changes)
    ax.axhline(avg_changes, color='red', linestyle='--', linewidth=2, label=f'Avg: {avg_changes:.1f}')
    ax.legend(fontsize=10)

    # Plot 4: Movement backtrack analysis
    ax = axes[1, 1]
    backtracks = []
    for i in agent_ids:
        seq = pos_hist[i]
        bt = sum(1 for k in range(2, len(seq)) if seq[k] == seq[k-2] and seq[k] != seq[k-1])
        backtracks.append(bt)
    
    bars = ax.bar(agent_ids, backtracks, color=colors, edgecolor='black', linewidth=1.5)
    ax.set_xlabel('Agent ID', fontsize=12)
    ax.set_ylabel('Backtrack Count', fontsize=12)
    ax.set_title('Movement Efficiency (Immediate Backtracks)', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, val in zip(bars, backtracks):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.2,
                f'{val}', ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    total_bt = sum(backtracks)
    ax.text(0.98, 0.98, f'Total: {total_bt}', transform=ax.transAxes, 
            fontsize=11, verticalalignment='top', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='lightgreen' if total_bt < 10 else 'lightyellow', alpha=0.8))

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"\n✓ Metrics plot saved to: {args.output}")
    print(f"\nSummary:")
    print(f"  Final Coverage: {coverage_hist[-1]:.1%}")
    print(f"  Avg Unique Positions: {avg_diversity:.1f}%")
    print(f"  Avg Assignment Changes: {avg_changes:.1f}")
    print(f"  Total Backtracks: {total_bt}")
    print(f"  Remaining Frontiers: {final_frontiers}")
    
    plt.show()
