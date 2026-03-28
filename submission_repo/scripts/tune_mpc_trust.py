#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tune MPC Trust Parameters

Tests different combinations of:
- mpc_trust_weight: [0.3, 0.5, 0.7, 1.0] (how much to weight MPC vs prediction-observation)
- mpc_trust_threshold: [0.1, 0.2, 0.3] (minimum error to trigger trust decay)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import random
from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps

def run_tuning_test(weight, threshold, seed=42):
    """Run a single test with given parameters."""
    np.random.seed(seed)
    random.seed(seed)
    
    cfg = {
        'N': 6,
        'T': 150,  # Shorter for faster tuning
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
            # MPC Trust Parameters (tuning)
            'mpc_trust_enabled': True,
            'mpc_trust_weight': weight,
            'mpc_trust_window': 3,
            'mpc_trust_threshold': threshold,
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    trust_history = []
    mpc_error_history = []
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=0.0)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        avg_trust = policy.get_average_trust_in_leader()
        trust_history.append(avg_trust)
        
        mpc_error = getattr(policy, '_last_mpc_error', 0.0)
        mpc_error_history.append(mpc_error)
    
    # Analyze results
    attack_start = 30
    trust_before = np.mean(trust_history[:attack_start])
    trust_during = np.mean(trust_history[attack_start:])
    trust_decay = trust_before - trust_during
    
    mpc_errors_during = [e for i, e in enumerate(mpc_error_history) if i >= attack_start and e > 0]
    avg_mpc_error = np.mean(mpc_errors_during) if mpc_errors_during else 0.0
    
    final_coverage = env.coverage() * 100
    impeachments = policy.impeachment_count
    min_trust = min(trust_history)
    
    return {
        'weight': weight,
        'threshold': threshold,
        'trust_decay': trust_decay,
        'avg_mpc_error': avg_mpc_error,
        'mpc_error_count': len(mpc_errors_during),
        'final_coverage': final_coverage,
        'impeachments': impeachments,
        'min_trust': min_trust,
    }

def main():
    print("=" * 70)
    print("MPC TRUST PARAMETER TUNING")
    print("=" * 70)
    print()
    
    weights = [0.3, 0.5, 0.7, 1.0]
    thresholds = [0.1, 0.2, 0.3]
    
    results = []
    
    total_tests = len(weights) * len(thresholds)
    current_test = 0
    
    for weight in weights:
        for threshold in thresholds:
            current_test += 1
            print(f"[{current_test}/{total_tests}] Testing weight={weight:.1f}, threshold={threshold:.1f}...", end=' ')
            sys.stdout.flush()
            
            result = run_tuning_test(weight, threshold)
            results.append(result)
            
            print(f"Trust decay={result['trust_decay']:.3f}, MPC errors={result['mpc_error_count']}")
    
    print()
    print("=" * 70)
    print("RESULTS SUMMARY")
    print("=" * 70)
    print()
    print(f"{'Weight':<8} {'Threshold':<10} {'Trust Decay':<12} {'MPC Errors':<12} {'Impeach':<8} {'Min Trust':<10}")
    print("-" * 70)
    
    # Sort by trust decay (descending)
    results_sorted = sorted(results, key=lambda x: x['trust_decay'], reverse=True)
    
    for r in results_sorted:
        print(f"{r['weight']:<8.1f} {r['threshold']:<10.1f} {r['trust_decay']:<12.3f} "
              f"{r['mpc_error_count']:<12} {r['impeachments']:<8} {r['min_trust']:<10.3f}")
    
    print()
    print("RECOMMENDED PARAMETERS:")
    best = results_sorted[0]
    print(f"  mpc_trust_weight: {best['weight']:.1f}")
    print(f"  mpc_trust_threshold: {best['threshold']:.1f}")
    print(f"  Expected trust decay: {best['trust_decay']:.3f}")
    print(f"  Expected impeachments: {best['impeachments']}")

if __name__ == '__main__':
    main()
