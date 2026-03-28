#!/usr/bin/env python
"""
Quick Multi-Fault Test
======================
Tests REIP across 3 fault types using the proven reip_comparison.py logic.
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


def run_single_seed(cfg, seed, verbose=False):
    """Run one experiment seed."""
    np.random.seed(seed)
    random.seed(seed)
    
    T = cfg.get('T', 500)
    R = cfg.get('r_comm', 15)
    p_loss = cfg.get('p_loss', 0.30)
    
    # Important: each seed needs different environment!
    cfg = copy.deepcopy(cfg)
    cfg['env'] = {**cfg.get('env', {}), 'seed': seed}
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    
    coverage_curve = []
    trust_curve = []
    leaders_over_time = [policy.leader_id]
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=p_loss)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state = {'t': t}
        policy.step(env, frontiers, state)
        
        coverage_curve.append(env.coverage())
        avg_trust = np.mean([policy.trust[i][policy.leader_id] 
                            for i in range(policy.N) if i != policy.leader_id])
        trust_curve.append(avg_trust)
        
        if policy.leader_id != leaders_over_time[-1]:
            leaders_over_time.append(policy.leader_id)
    
    return {
        'final_coverage': env.coverage(),
        'impeachments': policy.impeachment_count,
        'min_trust': min(trust_curve) if trust_curve else 1.0,
    }


def run_experiment(base_cfg, fault_type, governance_enabled, num_seeds=10):
    """Run experiment with given fault type and governance setting."""
    cfg = copy.deepcopy(base_cfg)
    cfg['reip']['governance_enabled'] = governance_enabled
    cfg['reip']['hallucination_schedule'] = [
        {'start': 20, 'type': fault_type, 'duration': 1000},
    ]
    
    max_workers = max(1, multiprocessing.cpu_count() - 1)
    results = {'coverage': [], 'impeachments': [], 'min_trust': []}
    
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(run_single_seed, copy.deepcopy(cfg), seed) 
                   for seed in range(num_seeds)]
        for future in as_completed(futures):
            try:
                r = future.result()
                results['coverage'].append(r['final_coverage'])
                results['impeachments'].append(r['impeachments'])
                results['min_trust'].append(r['min_trust'])
            except Exception as e:
                print(f"Error: {e}")
    
    return results


def main():
    print("=" * 70)
    print("       MULTI-FAULT TYPE REIP COMPARISON")
    print("       Testing: dense_explored, corner_trap, byzantine")
    print("=" * 70)
    print()
    
    # Base configuration (from working reip_comparison.py)
    base_cfg = {
        'T': 500,
        'N': 6,
        'map_size': 60,
        'r_local': 8,
        'r_comm': 15,
        'ds': 4,
        'p_obstacle': 0.10,
        'p_loss': 0.30,
        'spawn_center': [30, 30],
        'spawn_radius': 5,
        'reip': {
            'command_radius': 20,
            'hallucination_rate': 0.0,
            'pred_gain_inflation_base': 10.0,
            'pred_gain_inflation_jitter': 3.0,
            'trust_decay_rate': 0.5,
            'trust_threshold': 0.45,
            'min_trust': 0.1,
            'compromised_agent_id': 0,
            'fallback_enabled': False,
            'dense_explored_strict': True,
        },
    }
    
    fault_types = ['dense_explored', 'corner_trap', 'byzantine']
    num_seeds = 10
    
    all_results = {}
    
    for fault_type in fault_types:
        print(f"\n{'='*60}")
        print(f"Testing: {fault_type.upper()}")
        print(f"{'='*60}")
        
        print(f"  Running WITH governance ({num_seeds} seeds)...")
        with_gov = run_experiment(base_cfg, fault_type, True, num_seeds)
        
        print(f"  Running WITHOUT governance ({num_seeds} seeds)...")
        without_gov = run_experiment(base_cfg, fault_type, False, num_seeds)
        
        # Calculate stats
        with_cov = np.mean(with_gov['coverage']) * 100
        without_cov = np.mean(without_gov['coverage']) * 100
        diff = with_cov - without_cov
        
        with_imp = np.mean(with_gov['impeachments'])
        with_trust = np.mean(with_gov['min_trust'])
        without_trust = np.mean(without_gov['min_trust'])
        
        all_results[fault_type] = {
            'with_gov': with_gov,
            'without_gov': without_gov,
            'diff': diff,
        }
        
        print(f"\n  Results for {fault_type}:")
        print(f"    WITH Gov:    {with_cov:.1f}% coverage, {with_imp:.1f} impeachments, min_trust={with_trust:.3f}")
        print(f"    WITHOUT Gov: {without_cov:.1f}% coverage, 0 impeachments, min_trust={without_trust:.3f}")
        print(f"    Difference:  {diff:+.1f}%")
    
    # Summary table
    print("\n")
    print("=" * 70)
    print("SUMMARY: REIP Performance Across Fault Types")
    print("=" * 70)
    print()
    print(f"{'Fault Type':<18} {'Gov ON':>12} {'Gov OFF':>12} {'Diff':>10} {'Impeach':>10}")
    print("-" * 70)
    
    for fault_type in fault_types:
        r = all_results[fault_type]
        with_cov = np.mean(r['with_gov']['coverage']) * 100
        without_cov = np.mean(r['without_gov']['coverage']) * 100
        diff = with_cov - without_cov
        imp = np.mean(r['with_gov']['impeachments'])
        
        status = "+" if diff > 0 else ""
        print(f"{fault_type:<18} {with_cov:>11.1f}% {without_cov:>11.1f}% {status}{diff:>9.1f}% {imp:>9.1f}")
    
    print("-" * 70)
    
    # Overall average
    all_with = np.concatenate([np.array(all_results[f]['with_gov']['coverage']) for f in fault_types])
    all_without = np.concatenate([np.array(all_results[f]['without_gov']['coverage']) for f in fault_types])
    avg_diff = (all_with.mean() - all_without.mean()) * 100
    avg_imp = np.mean([np.mean(all_results[f]['with_gov']['impeachments']) for f in fault_types])
    
    print(f"{'AVERAGE':<18} {all_with.mean()*100:>11.1f}% {all_without.mean()*100:>11.1f}% {'+' if avg_diff > 0 else ''}{avg_diff:>9.1f}% {avg_imp:>9.1f}")
    print()
    
    print("=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    
    positive_results = sum(1 for f in fault_types if all_results[f]['diff'] > 0)
    if positive_results == len(fault_types):
        print("  [OK] REIP provides CONSISTENT improvement across ALL fault types!")
    elif positive_results > 0:
        print(f"  [OK] REIP provides improvement in {positive_results}/{len(fault_types)} fault types")
    else:
        print("  [!!] Results inconclusive - may need more seeds or tuning")
    
    print(f"  Average improvement: {avg_diff:+.1f}% coverage")
    print(f"  Average impeachments: {avg_imp:.1f} per run")
    print()


if __name__ == '__main__':
    main()
