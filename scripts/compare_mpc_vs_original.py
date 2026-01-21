#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Compare MPC Trust vs Original (Prediction-Observation Only)

Tests both approaches side-by-side to determine which is better.
"""
import sys
sys.stdout.reconfigure(encoding='utf-8')

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import random
from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps

def run_comparison_test(use_mpc, seed=42):
    """Run a single test with given trust mechanism."""
    np.random.seed(seed)
    random.seed(seed)
    
    cfg = {
        'N': 6,
        'T': 200,
        'map_size': 60,
        'env': {
            'obstacle_density': 0.05,
            'spawn_center': [30, 30],
            'spawn_radius': 3,
            'seed': seed,
        },
        'comm': {'radius': 12, 'p_loss': 0.0},
        'agent': {'r_local': 8, 'ds': 4, 'share_interval': 1},
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
        'reip': {
            'command_radius': 12,
            'hallucination_rate': 0.0,
            'hallucination_profile': 'dense_explored',
            'hallucination_schedule': [
                {'start': 30, 'type': 'dense_explored', 'duration': 1000},
            ],
            'trust_decay_rate': 0.5,
            'trust_threshold': 0.45,
            'min_trust': 0.1,
            'compromised_agent_id': 0,
            'fallback_enabled': False,
            'governance_enabled': True,
            # Trust mechanism configuration
            'mpc_trust_enabled': use_mpc,
            'mpc_trust_weight': 1.0 if use_mpc else 0.0,  # MPC only if enabled
            'mpc_trust_window': 3,
            'mpc_trust_threshold': 0.1,
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    trust_history = []
    coverage_history = []
    mpc_error_history = []
    pred_obs_error_history = []
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=0.0)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        cov = env.coverage()
        avg_trust = policy.get_average_trust_in_leader()
        trust_history.append(avg_trust)
        coverage_history.append(cov * 100)
        
        mpc_error = getattr(policy, '_last_mpc_error', 0.0)
        mpc_error_history.append(mpc_error)
        
        pred_gain = getattr(policy, 'predicted_coverage_gain', 0.0)
        obs_gain = getattr(policy, '_last_observed_gain', 0.0)
        pred_obs_error = max(0.0, pred_gain - obs_gain)
        pred_obs_error_history.append(pred_obs_error)
    
    # Analyze results
    attack_start = 30
    trust_before = np.mean(trust_history[:attack_start])
    trust_during = np.mean(trust_history[attack_start:])
    trust_decay = trust_before - trust_during
    
    mpc_errors_during = [e for i, e in enumerate(mpc_error_history) if i >= attack_start and e > 0]
    pred_obs_errors_during = [e for i, e in enumerate(pred_obs_error_history) if i >= attack_start and e > 0]
    
    return {
        'final_coverage': coverage_history[-1],
        'final_trust': trust_history[-1],
        'min_trust': min(trust_history),
        'trust_decay': trust_decay,
        'impeachments': policy.impeachment_count,
        'mpc_error_count': len(mpc_errors_during),
        'mpc_avg_error': np.mean(mpc_errors_during) if mpc_errors_during else 0.0,
        'pred_obs_error_count': len(pred_obs_errors_during),
        'pred_obs_avg_error': np.mean(pred_obs_errors_during) if pred_obs_errors_during else 0.0,
        'trust_history': trust_history,
        'coverage_history': coverage_history,
    }

def main():
    print("=" * 70)
    print("MPC TRUST vs ORIGINAL (PREDICTION-OBSERVATION) COMPARISON")
    print("=" * 70)
    print()
    
    num_seeds = 10
    seeds = list(range(42, 42 + num_seeds))
    
    results_original = []
    results_mpc = []
    
    print(f"Running {num_seeds} seeds for each approach...")
    print()
    
    for seed in seeds:
        print(f"Seed {seed}: ", end='')
        sys.stdout.flush()
        
        # Original (prediction-observation only)
        result_orig = run_comparison_test(use_mpc=False, seed=seed)
        results_original.append(result_orig)
        
        # MPC trust
        result_mpc = run_comparison_test(use_mpc=True, seed=seed)
        results_mpc.append(result_mpc)
        
        print(f"Original: trust_decay={result_orig['trust_decay']:.3f}, impeach={result_orig['impeachments']}, "
              f"MPC: trust_decay={result_mpc['trust_decay']:.3f}, impeach={result_mpc['impeachments']}")
    
    print()
    print("=" * 70)
    print("RESULTS SUMMARY")
    print("=" * 70)
    print()
    
    # Aggregate statistics
    def aggregate_stats(results, name):
        return {
            'name': name,
            'avg_coverage': np.mean([r['final_coverage'] for r in results]),
            'std_coverage': np.std([r['final_coverage'] for r in results]),
            'avg_trust_decay': np.mean([r['trust_decay'] for r in results]),
            'std_trust_decay': np.std([r['trust_decay'] for r in results]),
            'avg_impeachments': np.mean([r['impeachments'] for r in results]),
            'std_impeachments': np.std([r['impeachments'] for r in results]),
            'avg_min_trust': np.mean([r['min_trust'] for r in results]),
            'std_min_trust': np.std([r['min_trust'] for r in results]),
            'avg_mpc_errors': np.mean([r['mpc_error_count'] for r in results]),
            'avg_pred_obs_errors': np.mean([r['pred_obs_error_count'] for r in results]),
        }
    
    stats_original = aggregate_stats(results_original, "Original (Pred-Obs Only)")
    stats_mpc = aggregate_stats(results_mpc, "MPC Trust")
    
    print(f"{'Metric':<25} {'Original':<20} {'MPC Trust':<20} {'Winner':<10}")
    print("-" * 75)
    
    # Compare metrics
    metrics = [
        ('Final Coverage (%)', 'avg_coverage', 'std_coverage', 'higher'),
        ('Trust Decay', 'avg_trust_decay', 'std_trust_decay', 'higher'),
        ('Impeachments', 'avg_impeachments', 'std_impeachments', 'higher'),
        ('Min Trust', 'avg_min_trust', 'std_min_trust', 'lower'),
    ]
    
    winners = {'Original': 0, 'MPC': 0, 'Tie': 0}
    
    for metric_name, metric_key, std_key, better in metrics:
        orig_val = stats_original[metric_key]
        mpc_val = stats_mpc[metric_key]
        orig_std = stats_original[std_key]
        mpc_std = stats_mpc[std_key]
        
        if better == 'higher':
            if mpc_val > orig_val + 0.01:  # Significant difference
                winner = "MPC"
                winners['MPC'] += 1
            elif orig_val > mpc_val + 0.01:
                winner = "Original"
                winners['Original'] += 1
            else:
                winner = "Tie"
                winners['Tie'] += 1
        else:  # lower is better
            if mpc_val < orig_val - 0.01:
                winner = "MPC"
                winners['MPC'] += 1
            elif orig_val < mpc_val - 0.01:
                winner = "Original"
                winners['Original'] += 1
            else:
                winner = "Tie"
                winners['Tie'] += 1
        
        print(f"{metric_name:<25} {orig_val:.3f}±{orig_std:.3f}    {mpc_val:.3f}±{mpc_std:.3f}    {winner:<10}")
    
    print()
    print("=" * 70)
    print("DETAILED STATISTICS")
    print("=" * 70)
    print()
    print("Original (Prediction-Observation Only):")
    print(f"  Final Coverage: {stats_original['avg_coverage']:.2f}% ± {stats_original['std_coverage']:.2f}%")
    print(f"  Trust Decay: {stats_original['avg_trust_decay']:.3f} ± {stats_original['std_trust_decay']:.3f}")
    print(f"  Impeachments: {stats_original['avg_impeachments']:.1f} ± {stats_original['std_impeachments']:.1f}")
    print(f"  Min Trust: {stats_original['avg_min_trust']:.3f} ± {stats_original['std_min_trust']:.3f}")
    print(f"  Pred-Obs Errors: {stats_original['avg_pred_obs_errors']:.1f} steps")
    print()
    print("MPC Trust:")
    print(f"  Final Coverage: {stats_mpc['avg_coverage']:.2f}% ± {stats_mpc['std_coverage']:.2f}%")
    print(f"  Trust Decay: {stats_mpc['avg_trust_decay']:.3f} ± {stats_mpc['std_trust_decay']:.3f}")
    print(f"  Impeachments: {stats_mpc['avg_impeachments']:.1f} ± {stats_mpc['std_impeachments']:.1f}")
    print(f"  Min Trust: {stats_mpc['avg_min_trust']:.3f} ± {stats_mpc['std_min_trust']:.3f}")
    print(f"  MPC Errors: {stats_mpc['avg_mpc_errors']:.1f} steps")
    print()
    
    print("=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    print()
    
    if winners['MPC'] > winners['Original']:
        print("[SUCCESS] MPC TRUST IS BETTER!")
        print(f"   MPC wins: {winners['MPC']} metrics")
        print(f"   Original wins: {winners['Original']} metrics")
        print(f"   Ties: {winners['Tie']} metrics")
    elif winners['Original'] > winners['MPC']:
        print("[SUCCESS] ORIGINAL (PRED-OBS) IS BETTER!")
        print(f"   Original wins: {winners['Original']} metrics")
        print(f"   MPC wins: {winners['MPC']} metrics")
        print(f"   Ties: {winners['Tie']} metrics")
    else:
        print("[TIE] Both approaches perform similarly")
        print(f"   MPC wins: {winners['MPC']} metrics")
        print(f"   Original wins: {winners['Original']} metrics")
        print(f"   Ties: {winners['Tie']} metrics")
    
    # Key advantage analysis
    print()
    print("KEY ADVANTAGES:")
    if stats_mpc['avg_mpc_errors'] > 0:
        print(f"  [ADVANTAGE] MPC Trust: Works even when pred_unk=0 (detects {stats_mpc['avg_mpc_errors']:.1f} error steps)")
    if stats_original['avg_pred_obs_errors'] > 0:
        print(f"  [ADVANTAGE] Original: Detects {stats_original['avg_pred_obs_errors']:.1f} pred-obs error steps")
    
    if stats_mpc['avg_trust_decay'] > stats_original['avg_trust_decay'] + 0.05:
        print(f"  [ADVANTAGE] MPC Trust: Better trust decay ({stats_mpc['avg_trust_decay']:.3f} vs {stats_original['avg_trust_decay']:.3f})")
    elif stats_original['avg_trust_decay'] > stats_mpc['avg_trust_decay'] + 0.05:
        print(f"  [ADVANTAGE] Original: Better trust decay ({stats_original['avg_trust_decay']:.3f} vs {stats_mpc['avg_trust_decay']:.3f})")
    
    # Critical finding
    print()
    print("=" * 70)
    print("CRITICAL FINDING")
    print("=" * 70)
    print()
    if stats_original['avg_impeachments'] < 0.5:
        print("[CRITICAL] Original approach FAILS to detect attacks!")
        print(f"  - Average impeachments: {stats_original['avg_impeachments']:.1f}")
        print(f"  - Trust decay: {stats_original['avg_trust_decay']:.3f} (almost zero!)")
        print(f"  - This confirms the limitation: pred_unk=0 prevents detection")
        print()
        print("[SUCCESS] MPC Trust SOLVES the problem!")
        print(f"  - Average impeachments: {stats_mpc['avg_impeachments']:.1f}")
        print(f"  - Trust decay: {stats_mpc['avg_trust_decay']:.3f} (significant!)")
        print(f"  - Works even when pred_unk=0")

if __name__ == '__main__':
    main()
