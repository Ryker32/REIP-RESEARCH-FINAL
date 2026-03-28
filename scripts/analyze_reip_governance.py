"""
Analyze TRUE REIP governance metrics: trust, elections, hallucinations
"""
import os, sys, yaml, collections
import matplotlib.pyplot as plt
import numpy as np

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


def run_reip_governance_test(cfg):
    """Run REIP simulation and collect governance metrics."""
    env = GridWorld(cfg)
    
    # Load TRUE REIP controller
    from src.policy.reip_true import REIPController
    controller = REIPController(cfg)
    
    state = {'t': 0, 'prev': None, 'leader': 0, 'N': cfg.get('N', 1)}
    
    # Track metrics
    coverage_history = []
    leader_history = []
    trust_history = []
    hallucination_events = []
    
    for t in range(cfg['T']):
        state['t'] = t
        share_maps(env, R=cfg.get('comm',{}).get('radius',2), p_loss=cfg.get('comm',{}).get('p_loss',0.0))
        frontiers = env.detect_frontiers()
        
        # REIP step
        controller.step(env, frontiers, state)
        
        # Collect metrics
        coverage_history.append(env.coverage())
        leader_history.append(state.get('leader', 0))
        trust_history.append(state.get('avg_trust', 1.0))
        if state.get('hallucinating', False):
            hallucination_events.append(t)
    
    # Get final governance metrics from controller
    governance_metrics = controller.get_metrics()
    
    return {
        'coverage': coverage_history,
        'leaders': leader_history,
        'trust': trust_history,
        'hallucinations': hallucination_events,
        'governance': governance_metrics
    }


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--config', default='configs/reip_true.yaml')
    p.add_argument('--T', type=int, default=None)
    p.add_argument('--output', default='reip_governance.png')
    args = p.parse_args()
    
    cfg = load_cfg(args.config)
    if args.T is not None:
        cfg['T'] = args.T
    
    print(f"Running TRUE REIP simulation for T={cfg['T']}...")
    print(f"Hallucination rate: {cfg.get('reip', {}).get('hallucination_rate', 0.0):.1%}")
    print(f"Trust threshold (impeachment): {cfg.get('reip', {}).get('trust_threshold', 0.6):.2f}")
    
    results = run_reip_governance_test(cfg)
    
    # === Generate Plots ===
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Plot 1: Coverage over time
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(results['coverage'], linewidth=2, color='#2E86AB')
    ax1.set_xlabel('Timestep', fontsize=11)
    ax1.set_ylabel('Coverage', fontsize=11)
    ax1.set_title(f'Coverage (Final: {results["coverage"][-1]:.1%})', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([0, 1.05])
    
    # Mark hallucination events
    for t in results['hallucinations']:
        ax1.axvline(t, color='red', alpha=0.15, linewidth=0.8)
    if results['hallucinations']:
        ax1.axvline(results['hallucinations'][0], color='red', alpha=0.15, linewidth=0.8, label='Hallucination')
        ax1.legend(fontsize=9)
    
    # Plot 2: Leader over time
    ax2 = fig.add_subplot(gs[0, 1])
    leaders = results['leaders']
    ax2.plot(leaders, linewidth=2, color='#A23B72', marker='o', markersize=2)
    ax2.set_xlabel('Timestep', fontsize=11)
    ax2.set_ylabel('Leader ID', fontsize=11)
    ax2.set_title(f'Leader Transitions (Elections: {results["governance"]["election_count"]})', 
                  fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_yticks(range(cfg['N']))
    
    # Mark elections
    for i in range(1, len(leaders)):
        if leaders[i] != leaders[i-1]:
            ax2.axvline(i, color='orange', linestyle='--', alpha=0.5, linewidth=1.5)
    
    # Plot 3: Average Trust over time
    ax3 = fig.add_subplot(gs[1, 0])
    trust = results['trust']
    ax3.plot(trust, linewidth=2, color='#F18F01')
    ax3.axhline(cfg.get('reip', {}).get('trust_threshold', 0.6), 
                color='red', linestyle='--', linewidth=2, label='Impeachment Threshold')
    ax3.set_xlabel('Timestep', fontsize=11)
    ax3.set_ylabel('Average Trust in Leader', fontsize=11)
    ax3.set_title('Trust Evolution', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim([0, 1.05])
    ax3.legend(fontsize=9)
    
    # Mark hallucination events
    for t in results['hallucinations']:
        ax3.axvline(t, color='red', alpha=0.15, linewidth=0.8)
    
    # Plot 4: Trust detail (prediction error)
    ax4 = fig.add_subplot(gs[1, 1])
    trust_hist = results['governance']['trust_history']
    if trust_hist:
        timesteps = [h['timestep'] for h in trust_hist]
        pred_errors = [abs(h['predicted_gain'] - h['observed_gain']) for h in trust_hist]
        ax4.plot(timesteps, pred_errors, linewidth=1.5, color='#C73E1D', alpha=0.7)
        ax4.set_xlabel('Timestep', fontsize=11)
        ax4.set_ylabel('|U_pred - U_obs|', fontsize=11)
        ax4.set_title('Leader Prediction Error', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        # Mark hallucination periods
        for t in results['hallucinations']:
            ax4.axvline(t, color='red', alpha=0.15, linewidth=0.8)
    
    # Plot 5: Impeachment History
    ax5 = fig.add_subplot(gs[2, 0])
    failures = results['governance']['leader_failures']
    agents = sorted(failures.keys())
    counts = [failures[a] for a in agents]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(agents)))
    bars = ax5.bar(agents, counts, color=colors, edgecolor='black', linewidth=1.5)
    ax5.set_xlabel('Agent ID', fontsize=11)
    ax5.set_ylabel('Impeachment Count', fontsize=11)
    ax5.set_title('Leadership Failures by Agent', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, val in zip(bars, counts):
        if val > 0:
            height = bar.get_height()
            ax5.text(bar.get_x() + bar.get_width()/2., height + 0.05,
                    f'{int(val)}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Plot 6: System Summary Stats
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.axis('off')
    
    # Summary text
    summary_text = f"""
    === REIP Governance Summary ===
    
    Total Elections: {results['governance']['election_count']}
    Final Leader: Agent {results['governance']['final_leader']}
    Final Avg Trust: {results['governance']['final_avg_trust']:.3f}
    
    Hallucination Events: {len(results['hallucinations'])}
    Hallucination Rate: {cfg.get('reip', {}).get('hallucination_rate', 0.0):.1%}
    
    Final Coverage: {results['coverage'][-1]:.1%}
    Trust Threshold: {cfg.get('reip', {}).get('trust_threshold', 0.6):.2f}
    
    Most Impeached Agent: {max(failures, key=failures.get)} ({max(failures.values())} times)
    Most Trusted Agent: {results['governance']['final_leader']}
    """
    
    ax6.text(0.1, 0.9, summary_text, transform=ax6.transAxes, fontsize=11,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.suptitle('TRUE REIP: Resilient Election & Impeachment Policy Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.savefig(args.output, dpi=150, bbox_inches='tight')
    print(f"\nOK Analysis saved to: {args.output}")
    
    # Print detailed summary
    print(f"\n{'='*60}")
    print("REIP GOVERNANCE RESULTS")
    print(f"{'='*60}")
    print(f"Elections: {results['governance']['election_count']}")
    print(f"Final Leader: Agent {results['governance']['final_leader']}")
    print(f"Final Trust: {results['governance']['final_avg_trust']:.3f}")
    print(f"Hallucinations: {len(results['hallucinations'])} events")
    print(f"Final Coverage: {results['coverage'][-1]:.1%}")
    print(f"\nImpeachment Counts:")
    for agent in sorted(failures.keys()):
        if failures[agent] > 0:
            print(f"  Agent {agent}: {failures[agent]} times")
    print(f"{'='*60}\n")
    
    plt.show()
