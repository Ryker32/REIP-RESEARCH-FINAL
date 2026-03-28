#!/usr/bin/env python
"""
Test 3 Fault Types
==================
Runs reip_comparison.py logic with 3 different fault types.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
import random
import copy
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed

from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps


def _run_single_seed(args):
    """Run single seed - matches reip_comparison.py exactly."""
    cfg, seed, verbose = args
    
    np.random.seed(seed)
    random.seed(seed)
    
    cfg_copy = copy.deepcopy(cfg)
    cfg_copy['env'] = {**cfg.get('env', {}), 'seed': seed}
    
    env = GridWorld(cfg_copy)
    policy = REIPController(cfg_copy)
    state = {'t': 0}
    
    T = cfg_copy.get('T', 500)
    R = cfg_copy.get('comm', {}).get('radius', 20)
    p_loss = cfg_copy.get('comm', {}).get('p_loss', 0.0)
    
    coverage_curve = []
    trust_curve = []
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=p_loss)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        coverage_curve.append(env.coverage())
        avg_trust = np.mean([policy.trust[i][policy.leader_id] 
                            for i in range(policy.N) if i != policy.leader_id])
        trust_curve.append(avg_trust)
    
    return {
        'final_coverage': env.coverage(),
        'impeachments': policy.impeachment_count,
        'min_trust': min(trust_curve) if trust_curve else 1.0,
    }


def run_comparison(fault_type, num_seeds=10):
    """Run comparison for a specific fault type."""
    
    # Base config - matches reip_comparison.py EXACTLY
    base_cfg = {
        'N': 6,
        'T': 500,
        'map_size': 60,
        'env': {
            'obstacle_density': 0.05,
            'spawn_center': [30, 30],
            'spawn_radius': 3,
        },
        'comm': {'radius': 12, 'p_loss': 0.0},  # Comms radius = 12
        'agent': {'r_local': 8, 'ds': 4, 'share_interval': 1},  # Sensing radius = 8
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    # Attack config - changes based on fault_type
    # compromised_agent_id = 0 means only Agent 0 is compromised
    # When Agent 0 is impeached, the NEW leader is CLEAN and attack stops
    reip_attack = {
        'command_radius': 12,  # Match comms radius
        'hallucination_rate': 0.0,
        'hallucination_profile': fault_type,
        'hallucination_schedule': [
            {'start': 20, 'type': fault_type, 'duration': 1000},
        ],
        'pred_gain_inflation_base': 10.0,
        'pred_gain_inflation_jitter': 3.0,
        'trust_decay_rate': 0.5,
        'trust_threshold': 0.45,
        'min_trust': 0.1,
        'compromised_agent_id': 0,  # Only Agent 0 is compromised
        'fallback_enabled': False,
    }
    
    adv_comm = {'radius': 12, 'p_loss': 0.0}  # Same as base comms
    
    # Run WITH governance
    cfg_with = copy.deepcopy(base_cfg)
    cfg_with['comm'] = adv_comm
    cfg_with['reip'] = {**reip_attack, 'governance_enabled': True}
    
    # Run WITHOUT governance
    cfg_without = copy.deepcopy(base_cfg)
    cfg_without['comm'] = adv_comm
    cfg_without['reip'] = {**reip_attack, 'governance_enabled': False}
    
    max_workers = max(1, multiprocessing.cpu_count() - 1)
    
    results = {'with': [], 'without': []}
    
    # Run WITH governance
    args_list = [(cfg_with, seed, False) for seed in range(num_seeds)]
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_run_single_seed, args) for args in args_list]
        for future in as_completed(futures):
            try:
                results['with'].append(future.result())
            except Exception as e:
                print(f"Error: {e}")
    
    # Run WITHOUT governance  
    args_list = [(cfg_without, seed, False) for seed in range(num_seeds)]
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(_run_single_seed, args) for args in args_list]
        for future in as_completed(futures):
            try:
                results['without'].append(future.result())
            except Exception as e:
                print(f"Error: {e}")
    
    return results


def main():
    print("=" * 70)
    print("       MULTI-FAULT REIP COMPARISON")
    print("       Testing: dense_explored, corner_trap, byzantine")
    print("=" * 70)
    print()
    
    fault_types = ['dense_explored', 'corner_trap', 'byzantine']
    num_seeds = 10
    
    all_results = {}
    
    for fault_type in fault_types:
        print(f"\n{'='*60}")
        print(f"Testing: {fault_type.upper()}")
        print(f"{'='*60}")
        
        results = run_comparison(fault_type, num_seeds)
        
        with_cov = np.mean([r['final_coverage'] for r in results['with']]) * 100
        without_cov = np.mean([r['final_coverage'] for r in results['without']]) * 100
        with_imp = np.mean([r['impeachments'] for r in results['with']])
        with_trust = np.mean([r['min_trust'] for r in results['with']])
        without_trust = np.mean([r['min_trust'] for r in results['without']])
        
        diff = with_cov - without_cov
        
        all_results[fault_type] = {
            'with_cov': with_cov,
            'without_cov': without_cov,
            'diff': diff,
            'imp': with_imp,
            'with_trust': with_trust,
            'without_trust': without_trust,
        }
        
        print(f"\n  WITH Governance:    {with_cov:.1f}%, {with_imp:.1f} imps, trust={with_trust:.3f}")
        print(f"  WITHOUT Governance: {without_cov:.1f}%, 0 imps, trust={without_trust:.3f}")
        print(f"  Difference: {diff:+.1f}%")
    
    # Summary
    print("\n")
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print()
    print(f"{'Fault Type':<18} {'Gov ON':>10} {'Gov OFF':>10} {'Diff':>10} {'Impeach':>10}")
    print("-" * 70)
    
    for fault_type in fault_types:
        r = all_results[fault_type]
        print(f"{fault_type:<18} {r['with_cov']:>9.1f}% {r['without_cov']:>9.1f}% {r['diff']:>+9.1f}% {r['imp']:>9.1f}")
    
    print("-" * 70)
    avg_diff = np.mean([all_results[f]['diff'] for f in fault_types])
    avg_imp = np.mean([all_results[f]['imp'] for f in fault_types])
    print(f"{'AVERAGE':<18} {'':>10} {'':>10} {avg_diff:>+9.1f}% {avg_imp:>9.1f}")
    print()
    
    # Conclusion
    positive = sum(1 for f in fault_types if all_results[f]['diff'] > 0)
    print(f"REIP provides improvement in {positive}/{len(fault_types)} fault types")
    print(f"Average coverage advantage: {avg_diff:+.1f}%")


if __name__ == '__main__':
    main()
