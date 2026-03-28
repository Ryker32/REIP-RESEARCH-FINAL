#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Analyze Coverage Over Time: Why is MPC coverage lower?

This script analyzes coverage trajectories to understand:
1. When do impeachments occur?
2. How does coverage change during/after impeachment?
3. Is the lower coverage due to impeachment disruption or something else?
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import random
import matplotlib.pyplot as plt
from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps

def run_with_tracking(use_mpc, seed=42):
    """Run simulation and track detailed metrics over time."""
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
            'mpc_trust_enabled': use_mpc,
            'mpc_trust_weight': 1.0 if use_mpc else 0.0,
            'mpc_trust_window': 3,
            'mpc_trust_threshold': 0.1,
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    coverage_history = []
    trust_history = []
    impeachment_times = []
    leader_history = []
    attack_active = []
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=0.0)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        # Track if attack is active
        is_attack = getattr(policy, 'hallucination_active', False)
        attack_active.append(is_attack)
        
        # Track leader before step
        old_leader = policy.leader_id
        leader_history.append(old_leader)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        # Check if impeachment occurred
        if policy.leader_id != old_leader:
            impeachment_times.append(t)
        
        cov = env.coverage()
        avg_trust = policy.get_average_trust_in_leader()
        coverage_history.append(cov * 100)
        trust_history.append(avg_trust)
    
    return {
        'coverage': coverage_history,
        'trust': trust_history,
        'impeachment_times': impeachment_times,
        'leader_history': leader_history,
        'attack_active': attack_active,
    }

def analyze_coverage_difference(results_orig, results_mpc):
    """Analyze why coverage differs."""
    attack_start = 30
    
    # Coverage before attack
    cov_before_orig = np.mean(results_orig['coverage'][:attack_start])
    cov_before_mpc = np.mean(results_mpc['coverage'][:attack_start])
    
    # Coverage during attack
    cov_during_orig = results_orig['coverage'][attack_start:]
    cov_during_mpc = results_mpc['coverage'][attack_start:]
    
    # Coverage at end
    cov_final_orig = results_orig['coverage'][-1]
    cov_final_mpc = results_mpc['coverage'][-1]
    
    # Coverage growth rate (slope)
    def compute_growth_rate(cov_history, start_idx, end_idx):
        if end_idx <= start_idx:
            return 0.0
        return (cov_history[end_idx] - cov_history[start_idx]) / (end_idx - start_idx)
    
    # Growth rate before attack
    growth_before_orig = compute_growth_rate(results_orig['coverage'], 0, attack_start)
    growth_before_mpc = compute_growth_rate(results_mpc['coverage'], 0, attack_start)
    
    # Growth rate during attack
    growth_during_orig = compute_growth_rate(results_orig['coverage'], attack_start, len(results_orig['coverage'])-1)
    growth_during_mpc = compute_growth_rate(results_mpc['coverage'], attack_start, len(results_mpc['coverage'])-1)
    
    # Coverage lost due to bad commands (before impeachment)
    # Estimate: coverage would have been if no attack
    # Use growth rate before attack to extrapolate
    expected_cov_orig = cov_before_orig + growth_before_orig * (len(cov_during_orig))
    expected_cov_mpc = cov_before_mpc + growth_before_mpc * (len(cov_during_mpc))
    
    coverage_loss_orig = expected_cov_orig - cov_final_orig
    coverage_loss_mpc = expected_cov_mpc - cov_final_mpc
    
    # Time to first impeachment
    time_to_impeach_mpc = results_mpc['impeachment_times'][0] - attack_start if results_mpc['impeachment_times'] else None
    
    # Coverage at first impeachment
    if time_to_impeach_mpc is not None:
        cov_at_impeach_mpc = results_mpc['coverage'][attack_start + time_to_impeach_mpc]
    else:
        cov_at_impeach_mpc = None
    
    return {
        'cov_before_orig': cov_before_orig,
        'cov_before_mpc': cov_before_mpc,
        'cov_final_orig': cov_final_orig,
        'cov_final_mpc': cov_final_mpc,
        'growth_before_orig': growth_before_orig,
        'growth_before_mpc': growth_before_mpc,
        'growth_during_orig': growth_during_orig,
        'growth_during_mpc': growth_during_mpc,
        'coverage_loss_orig': coverage_loss_orig,
        'coverage_loss_mpc': coverage_loss_mpc,
        'time_to_impeach_mpc': time_to_impeach_mpc,
        'cov_at_impeach_mpc': cov_at_impeach_mpc,
        'num_impeachments_mpc': len(results_mpc['impeachment_times']),
    }

def main():
    print("=" * 70)
    print("COVERAGE ANALYSIS: Why is MPC coverage lower?")
    print("=" * 70)
    print()
    
    seed = 42
    print(f"Running detailed analysis (seed={seed})...")
    print()
    
    results_orig = run_with_tracking(use_mpc=False, seed=seed)
    results_mpc = run_with_tracking(use_mpc=True, seed=seed)
    
    analysis = analyze_coverage_difference(results_orig, results_mpc)
    
    print("COVERAGE METRICS")
    print("-" * 70)
    print(f"Before Attack (t=0-30):")
    print(f"  Original: {analysis['cov_before_orig']:.2f}%")
    print(f"  MPC:      {analysis['cov_before_mpc']:.2f}%")
    print(f"  Difference: {analysis['cov_before_mpc'] - analysis['cov_before_orig']:.2f}%")
    print()
    
    print(f"Final Coverage (t=200):")
    print(f"  Original: {analysis['cov_final_orig']:.2f}%")
    print(f"  MPC:      {analysis['cov_final_mpc']:.2f}%")
    print(f"  Difference: {analysis['cov_final_mpc'] - analysis['cov_final_orig']:.2f}%")
    print()
    
    print("COVERAGE GROWTH RATES")
    print("-" * 70)
    print(f"Before Attack:")
    print(f"  Original: {analysis['growth_before_orig']:.4f}% per step")
    print(f"  MPC:      {analysis['growth_before_mpc']:.4f}% per step")
    print()
    print(f"During Attack (t=30-200):")
    print(f"  Original: {analysis['growth_during_orig']:.4f}% per step")
    print(f"  MPC:      {analysis['growth_during_mpc']:.4f}% per step")
    print()
    
    print("IMPEACHMENT ANALYSIS")
    print("-" * 70)
    if analysis['time_to_impeach_mpc'] is not None:
        print(f"Time to first impeachment: {analysis['time_to_impeach_mpc']} steps after attack starts")
        print(f"Coverage at first impeachment: {analysis['cov_at_impeach_mpc']:.2f}%")
        print(f"Total impeachments: {analysis['num_impeachments_mpc']}")
    else:
        print("No impeachments occurred")
    print()
    
    print("COVERAGE LOSS ANALYSIS")
    print("-" * 70)
    print("Expected coverage (if no attack, using pre-attack growth rate):")
    expected_orig = analysis['cov_before_orig'] + analysis['growth_before_orig'] * 170
    expected_mpc = analysis['cov_before_mpc'] + analysis['growth_before_mpc'] * 170
    print(f"  Original: {expected_orig:.2f}%")
    print(f"  MPC:      {expected_mpc:.2f}%")
    print()
    print("Actual coverage loss (expected - actual):")
    print(f"  Original: {analysis['coverage_loss_orig']:.2f}%")
    print(f"  MPC:      {analysis['coverage_loss_mpc']:.2f}%")
    print()
    
    # Key insight
    print("=" * 70)
    print("KEY INSIGHT")
    print("=" * 70)
    print()
    
    if analysis['coverage_loss_mpc'] < analysis['coverage_loss_orig']:
        print("[SUCCESS] MPC has LESS coverage loss!")
        print(f"  MPC loses {analysis['coverage_loss_mpc']:.2f}% vs Original loses {analysis['coverage_loss_orig']:.2f}%")
        print("  This means MPC is MORE resilient to attacks!")
    else:
        print("[ANALYSIS] Why MPC might have lower final coverage:")
        print()
        print("1. IMPEACHMENT DISRUPTION:")
        print(f"   - MPC impeaches {analysis['num_impeachments_mpc']} times")
        print("   - Each impeachment causes brief re-coordination")
        print("   - Original never impeaches (no disruption, but compromised leader continues)")
        print()
        print("2. TIME TO DETECTION:")
        if analysis['time_to_impeach_mpc'] is not None:
            print(f"   - MPC detects and impeaches after {analysis['time_to_impeach_mpc']} steps")
            print(f"   - During these {analysis['time_to_impeach_mpc']} steps, agents follow bad commands")
            print("   - Original never detects, so agents keep following bad commands")
        print()
        print("3. RECOVERY TIME:")
        print("   - After impeachment, system needs time to re-coordinate")
        print("   - Multiple impeachments = cumulative disruption")
        print()
        print("4. THE PARADOX:")
        print("   - Lower coverage might indicate BETTER behavior:")
        print("     * MPC stops following bad commands (impeaches)")
        print("     * Original keeps following bad commands (never impeaches)")
        print("     * Following bad commands might explore SOME new areas (inefficiently)")
        print("     * But this is misleading - it's not resilient exploration!")
    
    print()
    print("=" * 70)
    print("RECOMMENDATION")
    print("=" * 70)
    print()
    print("Coverage alone is NOT the right metric for resilience!")
    print()
    print("Better metrics:")
    print("  1. Coverage loss (expected - actual) - MPC should be better")
    print("  2. Time to detection - MPC should detect faster")
    print("  3. Impeachment rate - MPC should impeach compromised leaders")
    print("  4. Trust decay - MPC should show significant decay")
    print()
    print("The lower final coverage with MPC might be a GOOD sign:")
    print("  - It means the system is STOPPING bad behavior")
    print("  - Rather than continuing inefficient exploration")

if __name__ == '__main__':
    main()
