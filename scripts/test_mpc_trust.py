#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test MPC Trust Implementation

Verifies that MPC trust mechanism:
1. Computes local optima correctly
2. Detects bad assignments
3. Triggers trust decay when leader assignment is worse than local optimum
4. Works even when pred_unk = 0 (addresses limitation)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import random
import yaml

from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps

def test_mpc_trust():
    """Test MPC trust mechanism with dense_explored attack."""
    print("=" * 60)
    print("MPC TRUST TEST")
    print("=" * 60)
    print()
    
    seed = 42
    np.random.seed(seed)
    random.seed(seed)
    
    # Configuration with MPC trust enabled
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
            'pred_gain_inflation_base': 10.0,
            'trust_decay_rate': 0.5,
            'trust_threshold': 0.45,
            'min_trust': 0.1,
            'compromised_agent_id': 0,
            'fallback_enabled': False,
            'governance_enabled': True,
            # MPC Trust Parameters
            'mpc_trust_enabled': True,
            'mpc_trust_weight': 0.5,  # Equal weighting with prediction-observation
            'mpc_trust_window': 3,  # Match pred_gain_radius
            'mpc_trust_threshold': 0.2,  # Lower threshold for sensitivity
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    # Track metrics
    trust_history = []
    mpc_error_history = []
    pred_obs_error_history = []
    coverage_history = []
    
    print("Running simulation with MPC trust enabled...")
    print()
    
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
        
        # Track MPC error (if computed)
        mpc_error = getattr(policy, '_last_mpc_error', None)
        if mpc_error is not None:
            mpc_error_history.append(mpc_error)
        else:
            mpc_error_history.append(0.0)
        
        # Track prediction-observation error
        pred_gain = getattr(policy, 'predicted_coverage_gain', 0.0)
        obs_gain = getattr(policy, '_last_observed_gain', 0.0)
        pred_obs_error = max(0.0, pred_gain - obs_gain)
        pred_obs_error_history.append(pred_obs_error)
        
        # Debug output every 20 steps
        if t % 20 == 0 or t < 10:
            attack_active = getattr(policy, 'hallucination_active', False)
            pred_unk = getattr(policy, '_last_predicted_unknown', 0)
            mpc_err = mpc_error_history[-1] if mpc_error_history else 0.0
            print(f"t={t:3d}: Coverage={cov*100:5.1f}%, Trust={avg_trust:.3f}, "
                  f"pred_unk={pred_unk}, mpc_err={mpc_err:.3f}, "
                  f"attack={'ON' if attack_active else 'OFF'}")
    
    print()
    print("=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"Final Coverage: {coverage_history[-1]:.1f}%")
    print(f"Final Trust: {trust_history[-1]:.3f}")
    print(f"Min Trust: {min(trust_history):.3f}")
    print(f"Impeachments: {policy.impeachment_count}")
    print()
    
    # Analyze trust decay
    attack_start = 30
    trust_before_attack = np.mean(trust_history[:attack_start])
    trust_during_attack = np.mean(trust_history[attack_start:])
    trust_decay = trust_before_attack - trust_during_attack
    
    print(f"Trust Before Attack: {trust_before_attack:.3f}")
    print(f"Trust During Attack: {trust_during_attack:.3f}")
    print(f"Trust Decay: {trust_decay:.3f}")
    print()
    
    # Analyze MPC error
    mpc_errors_during_attack = [e for i, e in enumerate(mpc_error_history) if i >= attack_start and e > 0]
    if mpc_errors_during_attack:
        print(f"MPC Errors During Attack: {len(mpc_errors_during_attack)} steps with error > 0")
        print(f"Average MPC Error: {np.mean(mpc_errors_during_attack):.3f}")
        print(f"Max MPC Error: {max(mpc_errors_during_attack):.3f}")
    else:
        print("WARNING: No MPC errors detected during attack!")
    print()
    
    # Check if MPC trust is working
    if trust_decay > 0.05 and mpc_errors_during_attack:
        print("[SUCCESS] MPC TRUST IS WORKING!")
        print("   - Trust decayed during attack")
        print("   - MPC errors detected bad assignments")
    elif mpc_errors_during_attack:
        print("[WARNING] MPC errors detected, but trust decay is minimal")
        print("   - May need to tune mpc_trust_weight or mpc_trust_threshold")
        print("   - Current: weight={}, threshold={}".format(
            cfg['reip'].get('mpc_trust_weight', 0.5),
            cfg['reip'].get('mpc_trust_threshold', 0.2)
        ))
    else:
        print("[ERROR] MPC TRUST NOT WORKING")
        print("   - No MPC errors detected")
        print("   - Check MPC trust computation logic")
    
    return {
        'final_coverage': coverage_history[-1],
        'final_trust': trust_history[-1],
        'min_trust': min(trust_history),
        'impeachments': policy.impeachment_count,
        'trust_decay': trust_decay,
        'mpc_errors_count': len(mpc_errors_during_attack),
        'mpc_avg_error': np.mean(mpc_errors_during_attack) if mpc_errors_during_attack else 0.0,
    }

if __name__ == '__main__':
    results = test_mpc_trust()
    print()
    print("Test complete!")
