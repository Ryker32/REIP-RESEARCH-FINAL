"""
Quick verification script to test that continuous hallucinations work
even when a schedule exists, and that baseline performs worse.
"""

import os, sys
import yaml
import numpy as np
from pathlib import Path

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'src'))

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
from src.policy.baseline_leader_follower import BaselineLeaderFollower
from src.policy.reip_true import REIPController


def load_cfg(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def run_single_test(cfg, description):
    """Run a single test and return metrics."""
    print(f"\n{'='*60}")
    print(f"Testing: {description}")
    print(f"{'='*60}")
    
    env = GridWorld(cfg)
    
    # Load controller
    policy_name = cfg.get('policy', 'baseline_leader_follower')
    if policy_name == 'baseline_leader_follower':
        controller = BaselineLeaderFollower(cfg)
    elif policy_name == 'reip_true':
        controller = REIPController(cfg)
    else:
        raise ValueError(f"Unknown policy: {policy_name}")
    
    state = {'t': 0, 'prev': {}, 'leader': 0, 'N': cfg['N']}
    coverage_history = []
    hallucination_active_steps = 0
    
    # Run simulation
    for t in range(cfg['T']):
        state['t'] = t
        share_maps(env, 
                   R=cfg.get('comm', {}).get('radius', 2),
                   p_loss=cfg.get('comm', {}).get('p_loss', 0.0),
                   p_corrupt=cfg.get('comm', {}).get('p_corrupt', 0.0))
        
        frontiers = env.detect_frontiers()
        
        # Track hallucination state
        if hasattr(controller, 'hallucination_active') and controller.hallucination_active:
            hallucination_active_steps += 1
        
        controller.step(env, frontiers, state)
        coverage_history.append(env.coverage())
    
    final_coverage = coverage_history[-1]
    hallucination_fraction = hallucination_active_steps / cfg['T']
    
    print(f"  Final Coverage: {final_coverage:.1%}")
    print(f"  Hallucination Active: {hallucination_active_steps}/{cfg['T']} steps ({hallucination_fraction:.1%})")
    
    if hasattr(controller, 'hallucination_history'):
        print(f"  Hallucination Events: {len(controller.hallucination_history)}")
        if controller.hallucination_history:
            print(f"  First event: t={controller.hallucination_history[0].get('t', 'N/A')}")
            print(f"  Last event: t={controller.hallucination_history[-1].get('t', 'N/A')}")
    
    return {
        'final_coverage': final_coverage,
        'hallucination_fraction': hallucination_fraction,
        'hallucination_steps': hallucination_active_steps,
        'coverage_history': coverage_history
    }


def main():
    print("="*70)
    print("HALLUCINATION FIX VERIFICATION")
    print("="*70)
    print("\nTesting that continuous hallucinations apply even with schedule...")
    
    # Load the faulted config
    faulted_cfg = load_cfg('configs/benchmarks/reip_faulted.yaml')
    
    # Test 1: Baseline with schedule + continuous rate (should be severe)
    print("\n" + "="*70)
    print("TEST 1: Baseline with schedule + continuous rate (NEW BEHAVIOR)")
    print("="*70)
    
    baseline_cfg = faulted_cfg.copy()
    baseline_cfg['policy'] = 'baseline_leader_follower'
    baseline_cfg['name'] = 'test/baseline_severe'
    baseline_cfg['round_robin_interval'] = 50
    baseline_cfg['hallucination_rate'] = faulted_cfg['reip']['hallucination_rate']
    baseline_cfg['hallucination_schedule'] = faulted_cfg['reip']['hallucination_schedule']
    baseline_cfg['command_loss_rate'] = faulted_cfg['reip']['command_loss_rate']
    if 'reip' not in baseline_cfg:
        baseline_cfg['reip'] = {}
    baseline_cfg['reip']['fallback_enabled'] = False
    
    result1 = run_single_test(baseline_cfg, "Baseline (schedule + 35% continuous rate)")
    
    # Test 2: Baseline with ONLY schedule (old behavior - for comparison)
    print("\n" + "="*70)
    print("TEST 2: Baseline with ONLY schedule (OLD BEHAVIOR - for comparison)")
    print("="*70)
    print("(This would be the old behavior where continuous rate was ignored)")
    print("Expected: Much fewer hallucination steps (only scheduled attacks)")
    
    # We can't easily test the old behavior without reverting the code,
    # but we can calculate what it would be:
    schedule_duration = 24 + 30 + 50  # 104 steps total
    expected_old_fraction = schedule_duration / baseline_cfg['T']
    print(f"\n  Expected hallucination steps (old): ~{schedule_duration}/{baseline_cfg['T']} ({expected_old_fraction:.1%})")
    print(f"  Actual hallucination steps (new): {result1['hallucination_steps']}/{baseline_cfg['T']} ({result1['hallucination_fraction']:.1%})")
    
    # Test 3: REIP with schedule + continuous rate
    print("\n" + "="*70)
    print("TEST 3: REIP with schedule + continuous rate")
    print("="*70)
    
    reip_cfg = faulted_cfg.copy()
    reip_cfg['policy'] = 'reip_true'
    reip_cfg['name'] = 'test/reip_severe'
    
    result3 = run_single_test(reip_cfg, "REIP (schedule + 35% continuous rate)")
    
    # Summary
    print("\n" + "="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)
    print(f"\nBaseline Coverage: {result1['final_coverage']:.1%}")
    print(f"REIP Coverage:     {result3['final_coverage']:.1%}")
    print(f"\nBaseline Hallucination Steps: {result1['hallucination_steps']}/{baseline_cfg['T']} ({result1['hallucination_fraction']:.1%})")
    print(f"REIP Hallucination Steps:     {result3['hallucination_steps']}/{reip_cfg['T']} ({result3['hallucination_fraction']:.1%})")
    
    # Check if fix is working
    print("\n" + "="*70)
    print("FIX VERIFICATION")
    print("="*70)
    
    if result1['hallucination_fraction'] > expected_old_fraction * 1.5:
        print("[PASS] Continuous hallucinations are applying (hallucination fraction is much higher than schedule-only)")
        print(f"   New behavior: {result1['hallucination_fraction']:.1%} vs expected old: {expected_old_fraction:.1%}")
    else:
        print("[FAIL] Continuous hallucinations may not be applying correctly")
        print(f"   Current: {result1['hallucination_fraction']:.1%} vs expected old: {expected_old_fraction:.1%}")
    
    if result1['final_coverage'] < 0.10:  # Baseline should be very low
        print("[PASS] Baseline is performing poorly (<10% coverage) as expected under severe conditions")
    else:
        print(f"[WARNING] Baseline coverage ({result1['final_coverage']:.1%}) is higher than expected")
        print("   This might indicate the adversarial conditions aren't severe enough")
        print("   Note: With round-robin rotation, baseline can recover somewhat")
    
    if result3['final_coverage'] > result1['final_coverage']:
        print("[PASS] REIP outperforms baseline under adversarial conditions")
        print(f"   REIP advantage: {(result3['final_coverage'] - result1['final_coverage'])*100:.1f} percentage points")
    else:
        print("[FAIL] REIP should outperform baseline")
    
    print("\n" + "="*70)
    print("Verification complete!")
    print("="*70)


if __name__ == '__main__':
    main()

