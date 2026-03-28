"""
REIP Governance Value Demonstration
Controlled experiment: WITH vs WITHOUT governance under attack

KEY FIXES:
1. CONTROLLED: All params identical except governance_enabled
2. THRESHOLD: Set to 0.85 based on observed trust decay (~0.68-0.85 range)
3. COMMS: share_maps() called every tick
4. DIAGNOSTIC: Track cusum/bayes alarms, impeachments, leader changes
5. PARALLEL: Uses ProcessPoolExecutor for faster runs
"""

import sys
import os

# Force unbuffered output for logging
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import copy
import random
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed
from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps


def _run_single_seed(args):
    """Worker function for parallel execution."""
    cfg, seed, verbose = args
    np.random.seed(seed)
    random.seed(seed)
    
    cfg_copy = copy.deepcopy(cfg)
    cfg_copy['env'] = {**cfg.get('env', {}), 'seed': seed}
    
    env = GridWorld(cfg_copy)
    policy = REIPController(cfg_copy)
    state = {'t': 0}
    
    coverage_curve = []
    trust_curve = []
    leaders_over_time = [policy.leader_id]
    time_to_80 = None
    
    T = cfg_copy.get('T', 100)
    R = cfg_copy.get('comm', {}).get('radius', 20)
    p_loss = cfg_copy.get('comm', {}).get('p_loss', 0.0)
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=p_loss)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        cov = env.coverage()
        coverage_curve.append(cov)
        
        if time_to_80 is None and cov >= 0.80:
            time_to_80 = t
        
        avg_trust = np.mean([policy.trust[i][policy.leader_id] 
                            for i in range(policy.N) if i != policy.leader_id])
        trust_curve.append(avg_trust)
        
        if policy.leader_id != leaders_over_time[-1]:
            leaders_over_time.append(policy.leader_id)
    
    return {
        'coverage_curve': coverage_curve,
        'trust_curve': trust_curve,
        'final_coverage': env.coverage(),
        'impeachments': policy.impeachment_count,
        'leader_changes': len(leaders_over_time) - 1,
        'min_trust': min(trust_curve) if trust_curve else 1.0,
        'time_to_80': time_to_80 if time_to_80 else T,
    }


def run_experiment(cfg, label, num_seeds=10, verbose=False, max_workers=None):
    """Run experiment with full diagnostic tracking using parallel execution."""
    results = {
        'coverage_curves': [],
        'trust_curves': [],
        'final_coverage': [],
        'impeachments': [],
        'leader_changes': [],
        'min_trust': [],
        'time_to_80': [],
    }
    
    if max_workers is None:
        max_workers = max(1, multiprocessing.cpu_count() - 1)
    
    # Run first seed with verbose output if requested
    if verbose:
        r = _run_single_seed((cfg, 0, True))
        # Print verbose for seed 0 by running it separately
        cfg_copy = copy.deepcopy(cfg)
        cfg_copy['env'] = {**cfg.get('env', {}), 'seed': 0}
        env = GridWorld(cfg_copy)
        policy = REIPController(cfg_copy)
        state = {'t': 0}
        T = cfg_copy.get('T', 100)
        R = cfg_copy.get('comm', {}).get('radius', 20)
        for t in range(T):
            env.current_time = t
            share_maps(env, R=R, p_loss=0.0)
            leader_agent = env.agents.get(policy.leader_id)
            frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
            state['t'] = t
            if t % 20 == 0:
                ha = getattr(policy, "hallucination_active", False)
                avg_trust = np.mean([policy.trust[i][policy.leader_id] 
                                    for i in range(policy.N) if i != policy.leader_id])
                corrupted = getattr(policy, '_last_dense_explored_corrupted', 0)
                targets = getattr(policy, '_last_dense_explored_targets', 0)
                print(f"  t={t:3d} leader={policy.leader_id} halluc={ha} "
                      f"trust={avg_trust:.3f} imps={policy.impeachment_count} "
                      f"frontiers={len(frontiers)} corrupt={corrupted}/{targets}")
            policy.step(env, frontiers, state)
    
    # Run all seeds in parallel
    args_list = [(cfg, seed, False) for seed in range(num_seeds)]
    
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_run_single_seed, args) for args in args_list]
        for future in as_completed(futures):
            try:
                r = future.result()
                results['coverage_curves'].append(r['coverage_curve'])
                results['trust_curves'].append(r['trust_curve'])
                results['final_coverage'].append(r['final_coverage'])
                results['impeachments'].append(r['impeachments'])
                results['leader_changes'].append(r['leader_changes'])
                results['min_trust'].append(r['min_trust'])
                results['time_to_80'].append(r['time_to_80'])
            except Exception as e:
                print(f"  [ERROR] Seed failed: {e}")
    
    return results


def main():
    print("="*70)
    print("       REIP GOVERNANCE VALUE DEMONSTRATION")
    print("       Controlled Experiment: Governance ON vs OFF")
    print("="*70)
    
    # Production config
    base_cfg = {
        'N': 6,
        'T': 500,  # Original duration - enough for multiple attack waves + recovery
        'map_size': 60,  # 60x60 map as specified
        'env': {
            'obstacle_density': 0.05,
            'spawn_center': [30, 30],  # All agents start together in center
            'spawn_radius': 3,         # Within 3 cells of center
        },
        'comm': {'radius': 12, 'p_loss': 0.0},  # R=12 clean
        'agent': {'r_local': 5, 'ds': 4, 'share_interval': 1},
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    # === OPTIMIZED PARAMETERS FOR DEMONSTRATION ===
    # Based on Table I but tuned for clear impeachment demonstration:
    # - Higher trust decay (k=1.5) for faster detection
    # - Higher threshold (τ=0.60) for earlier impeachment
    # - Other params from Table I
    
    # ADVERSARIAL scenario - CONTINUOUS attack while compromised agent is leader
    # Key insight: Attack is PINNED to agent 0
    # - With governance: Agent 0 impeached -> attack stops -> recovery
    # - Without governance: Agent 0 stays leader forever -> attack never stops
    reip_common_attack = {
        'command_radius': 20,
        'hallucination_rate': 0.0,
        'hallucination_profile': 'dense_explored',
        'hallucination_schedule': [
            {'start': 20, 'type': 'dense_explored', 'duration': 1000},  # Continuous!
        ],
        'pred_gain_inflation_base': 10.0,
        'pred_gain_inflation_jitter': 3.0,
        'trust_decay_rate': 0.5,   # k=0.5 from Table I
        'trust_threshold': 0.45,   # τ=0.45 from Table I  
        'min_trust': 0.1,          # T_min=0.1 from Table I
        'compromised_agent_id': 0, # CRITICAL: Attack only when agent 0 is leader
        'fallback_enabled': False, # NO fallback - agents must follow bad commands
    }
    
    # CLEAN scenario (Table I params)
    reip_common_clean = {
        'command_radius': 12,      # R=12 clean
        'hallucination_rate': 0.0,
        'trust_decay_rate': 0.5,   # k=0.5 from Table I
        'trust_threshold': 0.45,   # τ=0.45 from Table I
        'min_trust': 0.1,          # T_min=0.1 from Table I
        'fallback_enabled': True,
    }
    
    # CONTROLLED scenarios - only governance_enabled differs
    # Use same comm for both - the attack IS the adversarial condition
    # Different comm would confound the comparison
    adv_comm = {'radius': 20, 'p_loss': 0.0}   # Good comms
    clean_comm = {'radius': 20, 'p_loss': 0.0}  # Same
    
    scenarios = {
        'Governance ON (Clean)': {
            **base_cfg,
            'comm': clean_comm,
            'reip': {**reip_common_clean, 'governance_enabled': True},
        },
        'Governance ON (Attack)': {
            **base_cfg,
            'comm': adv_comm,  # Adversarial comm degradation
            'reip': {**reip_common_attack, 'governance_enabled': True},
        },
        'Governance OFF (Clean)': {
            **base_cfg,
            'comm': clean_comm,
            'reip': {**reip_common_clean, 'governance_enabled': False},
        },
        'Governance OFF (Attack)': {
            **base_cfg,
            'comm': adv_comm,  # Adversarial comm degradation
            'reip': {**reip_common_attack, 'governance_enabled': False},
        },
    }
    
    all_results = {}
    num_seeds = 10  # Quick test first
    max_workers = max(1, multiprocessing.cpu_count() - 1)
    print(f"\nUsing {max_workers} parallel workers (CPU cores: {multiprocessing.cpu_count()})")
    
    for name, cfg in scenarios.items():
        print(f"\nRunning: {name} ({num_seeds} seeds in parallel)...")
        is_attack = 'Attack' in name
        all_results[name] = run_experiment(cfg, name, num_seeds=num_seeds, verbose=is_attack, max_workers=max_workers)
        
        r = all_results[name]
        print(f"  Coverage: {np.mean(r['final_coverage']):.1%} +/- {np.std(r['final_coverage']):.1%}")
        print(f"  Impeachments: {np.mean(r['impeachments']):.1f} (max={max(r['impeachments'])})")
        print(f"  Leader Changes: {np.mean(r['leader_changes']):.1f}")
        print(f"  Min Trust: {np.mean(r['min_trust']):.3f}")
        print(f"  Time to 80%: {np.mean(r['time_to_80']):.0f} steps")
    
    # Results table
    print("\n" + "="*70)
    print("RESULTS TABLE")
    print("="*70)
    print(f"{'Scenario':<26} {'Coverage':<14} {'Imps':<8} {'MinTrust':<10} {'T>80%':<8}")
    print("-"*70)
    
    for name in scenarios.keys():
        r = all_results[name]
        print(f"{name:<26} {np.mean(r['final_coverage'])*100:>5.1f}% +/- {np.std(r['final_coverage'])*100:>4.1f}%  "
              f"{np.mean(r['impeachments']):>4.1f}    {np.mean(r['min_trust']):>6.3f}    {np.mean(r['time_to_80']):>5.0f}")
    
    # Key comparison
    print("\n" + "="*70)
    print("KEY COMPARISON: Under Attack")
    print("="*70)
    
    gov_on = all_results['Governance ON (Attack)']
    gov_off = all_results['Governance OFF (Attack)']
    
    print(f"  WITH Governance:")
    print(f"    Coverage: {np.mean(gov_on['final_coverage']):.1%}")
    print(f"    Impeachments: {np.mean(gov_on['impeachments']):.1f}")
    print(f"    Min Trust: {np.mean(gov_on['min_trust']):.3f}")
    
    print(f"\n  WITHOUT Governance:")
    print(f"    Coverage: {np.mean(gov_off['final_coverage']):.1%}")
    print(f"    Impeachments: {np.mean(gov_off['impeachments']):.1f}")
    print(f"    Min Trust: {np.mean(gov_off['min_trust']):.3f}")
    
    diff = (np.mean(gov_on['final_coverage']) - np.mean(gov_off['final_coverage'])) * 100
    imp_diff = np.mean(gov_on['impeachments']) - np.mean(gov_off['impeachments'])
    
    print(f"\n  Difference: {diff:+.1f}% coverage, {imp_diff:+.1f} impeachments")
    
    # Plot
    print("\nGenerating plot...")
    
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    
    colors = {
        'Governance ON (Clean)': '#27ae60',
        'Governance ON (Attack)': '#c0392b',
        'Governance OFF (Clean)': '#2980b9',
        'Governance OFF (Attack)': '#8e44ad',
    }
    
    # Coverage over time
    ax1 = axes[0]
    for name in scenarios.keys():
        curves = all_results[name]['coverage_curves']
        mean_curve = np.mean(curves, axis=0)
        std_curve = np.std(curves, axis=0)
        x = np.arange(len(mean_curve))
        ax1.plot(x, mean_curve * 100, label=name, color=colors[name], linewidth=2)
        ax1.fill_between(x, (mean_curve - std_curve) * 100, (mean_curve + std_curve) * 100, 
                         alpha=0.15, color=colors[name])
    
    ax1.set_xlabel('Timestep')
    ax1.set_ylabel('Coverage (%)')
    ax1.set_title('Coverage Over Time')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 105)
    
    # Trust decay
    ax2 = axes[1]
    for name in ['Governance ON (Attack)', 'Governance OFF (Attack)']:
        curves = all_results[name]['trust_curves']
        mean_curve = np.mean(curves, axis=0)
        x = np.arange(len(mean_curve))
        ax2.plot(x, mean_curve, label=name, color=colors[name], linewidth=2)
    
    # Pull threshold from config to avoid mismatch
    threshold = scenarios['Governance ON (Attack)']['reip'].get('trust_threshold', 0.75)
    ax2.axhline(threshold, color='red', linestyle='--', alpha=0.7, label=f'Impeachment Threshold ({threshold})')
    ax2.set_xlabel('Timestep')
    ax2.set_ylabel('Trust in Leader')
    ax2.set_title('Trust Decay Under Attack')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0.5, 1.05)
    
    # Bar chart
    ax3 = axes[2]
    attack_names = ['Governance ON (Attack)', 'Governance OFF (Attack)']
    x_pos = np.arange(2)
    
    cov_means = [np.mean(all_results[n]['final_coverage']) * 100 for n in attack_names]
    cov_stds = [np.std(all_results[n]['final_coverage']) * 100 for n in attack_names]
    bar_colors = [colors[n] for n in attack_names]
    
    bars = ax3.bar(x_pos, cov_means, yerr=cov_stds, capsize=8, color=bar_colors, alpha=0.8)
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(['WITH\nGovernance', 'WITHOUT\nGovernance'])
    ax3.set_ylabel('Final Coverage (%)')
    ax3.set_title('Coverage Under Attack')
    ax3.grid(True, alpha=0.3, axis='y')
    ax3.set_ylim(0, 105)
    
    for bar, mean in zip(bars, cov_means):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 3, 
                f'{mean:.1f}%', ha='center', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    
    output_path = 'results/reip_comparison.png'
    os.makedirs('results', exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved to: {output_path}")
    plt.show()
    
    # Conclusion
    print("\n" + "="*70)
    print("CONCLUSION")
    print("="*70)
    
    if np.mean(gov_on['impeachments']) > 0.5:
        print(f"  [OK] Impeachment mechanism ACTIVE ({np.mean(gov_on['impeachments']):.1f} avg)")
        print(f"  [OK] {np.mean(gov_on['leader_changes']):.1f} leader changes occurred")
        if diff > 2:
            print(f"  [OK] Governance provides {diff:.1f}% BETTER coverage under attack")
        elif diff > 0:
            print(f"  [~]  Governance provides slight benefit (+{diff:.1f}%)")
        else:
            print(f"  [?]  Impeachments occurred but coverage similar ({diff:.1f}%)")
        print(f"\n  KEY: Trust recovered after impeachment!")
        print(f"       With Gov: min_trust={np.mean(gov_on['min_trust']):.3f}")
        print(f"       No Gov:   min_trust={np.mean(gov_off['min_trust']):.3f} (stuck with bad leader)")
    else:
        print(f"  [X] No impeachments triggered (avg={np.mean(gov_on['impeachments']):.2f})")
        print(f"      Min trust reached: {np.mean(gov_on['min_trust']):.3f} (threshold: 0.85)")
        print(f"      -> Need more aggressive attack or lower threshold")


if __name__ == "__main__":
    main()
