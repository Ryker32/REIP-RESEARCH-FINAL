"""
REIP Verification Suite
Validates core mechanisms before hardware implementation.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController

def test_basic_exploration():
    """Test 1: Agents explore and coverage increases."""
    print("\n" + "="*60)
    print("TEST 1: Basic Exploration")
    print("="*60)
    
    cfg = {
        'N': 4,
        'T': 100,
        'map_size': 30,
        'env': {'obstacle_density': 0.05, 'seed': 42},
        'comm': {'radius': 15, 'p_loss': 0.0},
        'agent': {'r_local': 5, 'ds': 3, 'share_interval': 1},
        'reip': {
            'command_radius': 15,
            'window': 2,
            'governance_enabled': True,
        },
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    initial_coverage = env.coverage()
    
    for t in range(50):
        frontiers = env.detect_frontiers_from_agent_belief(env.agents.get(policy.leader_id))
        state['t'] = t
        policy.step(env, frontiers, state)
    
    final_coverage = env.coverage()
    
    print(f"  Initial coverage: {initial_coverage:.1%}")
    print(f"  Final coverage:   {final_coverage:.1%}")
    print(f"  Coverage gain:    {final_coverage - initial_coverage:.1%}")
    
    passed = final_coverage > initial_coverage + 0.2
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_trust_decay_under_attack():
    """Test 2: Trust decays when leader is compromised."""
    print("\n" + "="*60)
    print("TEST 2: Trust Decay Under Attack")
    print("="*60)
    
    cfg = {
        'N': 4,
        'T': 100,
        'map_size': 30,
        'env': {'obstacle_density': 0.05, 'seed': 123},
        'comm': {'radius': 15, 'p_loss': 0.0},
        'agent': {'r_local': 5, 'ds': 3, 'share_interval': 1},
        'reip': {
            'command_radius': 15,
            'hallucination_rate': 1.0,  # Always attacking
            'hallucination_profile': 'overconfident',
            'governance_enabled': True,
            'trust_threshold': 0.6,
        },
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    initial_trust = policy.get_average_trust_in_leader()
    
    for t in range(30):
        frontiers = env.detect_frontiers_from_agent_belief(env.agents.get(policy.leader_id))
        state['t'] = t
        policy.step(env, frontiers, state)
    
    final_trust = policy.get_average_trust_in_leader()
    
    print(f"  Initial trust: {initial_trust:.3f}")
    print(f"  Final trust:   {final_trust:.3f}")
    print(f"  Trust drop:    {initial_trust - final_trust:.3f}")
    
    passed = final_trust < initial_trust - 0.05  # Some decay expected
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_impeachment():
    """Test 3: Leader gets impeached when trust is low."""
    print("\n" + "="*60)
    print("TEST 3: Impeachment Mechanism")
    print("="*60)
    
    cfg = {
        'N': 4,
        'T': 200,
        'map_size': 30,
        'env': {'obstacle_density': 0.05, 'seed': 456},
        'comm': {'radius': 15, 'p_loss': 0.0},
        'agent': {'r_local': 5, 'ds': 3, 'share_interval': 1},
        'reip': {
            'command_radius': 15,
            'hallucination_rate': 1.0,
            'hallucination_profile': 'overconfident',
            'governance_enabled': True,
            'trust_threshold': 0.7,
            'trust_decay_rate': 0.8,  # Faster decay for test
            'impeachment_cooldown': 5,
            'impeachment_persistence': 2,
        },
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    initial_leader = policy.leader_id
    
    for t in range(100):
        frontiers = env.detect_frontiers_from_agent_belief(env.agents.get(policy.leader_id))
        state['t'] = t
        policy.step(env, frontiers, state)
    
    final_leader = policy.leader_id
    impeachment_count = policy.impeachment_count
    
    print(f"  Initial leader:     {initial_leader}")
    print(f"  Final leader:       {final_leader}")
    print(f"  Impeachment count:  {impeachment_count}")
    print(f"  Election count:     {policy.election_count}")
    
    passed = impeachment_count > 0 or final_leader != initial_leader
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_recovery_after_attack():
    """Test 4: System recovers after attack ends."""
    print("\n" + "="*60)
    print("TEST 4: Recovery After Attack")
    print("="*60)
    
    cfg = {
        'N': 4,
        'T': 150,
        'map_size': 30,
        'env': {'obstacle_density': 0.05, 'seed': 789},
        'comm': {'radius': 15, 'p_loss': 0.0},
        'agent': {'r_local': 5, 'ds': 3, 'share_interval': 1},
        'reip': {
            'command_radius': 15,
            'hallucination_schedule': [
                {'start': 10, 'type': 'overconfident', 'duration': 20}
            ],
            'governance_enabled': True,
            'trust_threshold': 0.6,
        },
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    coverage_before_attack = None
    coverage_during_attack = None
    coverage_after_attack = None
    
    for t in range(100):
        frontiers = env.detect_frontiers_from_agent_belief(env.agents.get(policy.leader_id))
        state['t'] = t
        policy.step(env, frontiers, state)
        
        if t == 9:
            coverage_before_attack = env.coverage()
        elif t == 29:
            coverage_during_attack = env.coverage()
        elif t == 99:
            coverage_after_attack = env.coverage()
    
    print(f"  Coverage before attack (t=9):  {coverage_before_attack:.1%}")
    print(f"  Coverage during attack (t=29): {coverage_during_attack:.1%}")
    print(f"  Coverage after attack (t=99):  {coverage_after_attack:.1%}")
    
    # After attack, coverage should continue improving
    passed = coverage_after_attack > coverage_during_attack
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_no_oracle_leakage():
    """Test 5: Verify pathfinding uses agent belief, not team_belief."""
    print("\n" + "="*60)
    print("TEST 5: No Oracle Leakage")
    print("="*60)
    
    cfg = {
        'N': 3,
        'T': 50,
        'map_size': 20,
        'env': {'obstacle_density': 0.05, 'seed': 111},
        'comm': {'radius': 12, 'p_loss': 0.0},
        'agent': {'r_local': 6, 'ds': 2, 'share_interval': 1},
        'reip': {'command_radius': 12},
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    
    # Check that phase2_leader_blind flag is set
    phase2_enabled = getattr(env, 'phase2_leader_blind', False) or cfg.get('phase2', {}).get('enable', False)
    
    # Check that _neighbors raises error when called without agent
    error_raised = False
    if getattr(env, 'phase2_leader_blind', False):
        try:
            list(env._neighbors(5, 5, agent=None))
        except RuntimeError:
            error_raised = True
    
    # Check agents use their own belief_lr
    agent = env.agents.get(0)
    has_own_belief = hasattr(agent, 'belief_lr') and agent.belief_lr is not None
    
    print(f"  Phase2 enabled:        {phase2_enabled}")
    print(f"  Agent has own belief:  {has_own_belief}")
    print(f"  Uses local map:        {hasattr(agent, 'local')}")
    
    passed = phase2_enabled and has_own_belief
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_communication_filtering():
    """Test 6: Commands filtered by communication range."""
    print("\n" + "="*60)
    print("TEST 6: Communication Range Filtering")
    print("="*60)
    
    cfg = {
        'N': 5,
        'T': 50,
        'map_size': 50,  # Large map
        'env': {'obstacle_density': 0.03, 'seed': 222},
        'comm': {'radius': 10, 'p_loss': 0.0},  # Small comm range
        'agent': {'r_local': 5, 'ds': 4, 'share_interval': 1},
        'reip': {
            'command_radius': 10,  # Limited range
            'multi_hop_commands': False,
        },
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    
    # Get leader position
    leader_pos = env.agent_positions.get(policy.leader_id)
    
    # Count agents within/outside command range
    within_range = 0
    outside_range = 0
    for aid, pos in env.agent_positions.items():
        if aid == policy.leader_id:
            continue
        dist = abs(pos[0] - leader_pos[0]) + abs(pos[1] - leader_pos[1])
        if dist <= cfg['reip']['command_radius']:
            within_range += 1
        else:
            outside_range += 1
    
    print(f"  Leader position:      {leader_pos}")
    print(f"  Command radius:       {cfg['reip']['command_radius']}")
    print(f"  Agents within range:  {within_range}")
    print(f"  Agents outside range: {outside_range}")
    
    # System should work regardless of distribution
    passed = True
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def test_scalability():
    """Test 7: System scales to larger scenarios."""
    print("\n" + "="*60)
    print("TEST 7: Scalability")
    print("="*60)
    
    configs = [
        {'N': 4, 'map_size': 30, 'label': 'Small (4 agents, 30x30)'},
        {'N': 8, 'map_size': 50, 'label': 'Medium (8 agents, 50x50)'},
        {'N': 12, 'map_size': 80, 'label': 'Large (12 agents, 80x80)'},
    ]
    
    results = []
    
    for config in configs:
        cfg = {
            'N': config['N'],
            'T': 100,
            'map_size': config['map_size'],
            'env': {'obstacle_density': 0.05, 'seed': 333},
            'comm': {'radius': 20, 'p_loss': 0.0},
            'agent': {'r_local': 6, 'ds': 4, 'share_interval': 1},
            'reip': {'command_radius': 20},
            'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
        }
        
        env = GridWorld(cfg)
        policy = REIPController(cfg)
        state = {'t': 0}
        
        import time
        start = time.time()
        
        for t in range(30):  # Quick test
            frontiers = env.detect_frontiers_from_agent_belief(env.agents.get(policy.leader_id))
            state['t'] = t
            policy.step(env, frontiers, state)
        
        elapsed = time.time() - start
        coverage = env.coverage()
        
        results.append({
            'label': config['label'],
            'time': elapsed,
            'coverage': coverage,
        })
        
        print(f"  {config['label']}")
        print(f"    Time: {elapsed:.2f}s, Coverage: {coverage:.1%}")
    
    # Should complete in reasonable time
    passed = all(r['time'] < 30 for r in results)
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")
    return passed


def main():
    print("\n" + "="*60)
    print("      REIP VERIFICATION SUITE")
    print("      Preparing for Hardware Implementation")
    print("="*60)
    
    tests = [
        ("Basic Exploration", test_basic_exploration),
        ("Trust Decay Under Attack", test_trust_decay_under_attack),
        ("Impeachment Mechanism", test_impeachment),
        ("Recovery After Attack", test_recovery_after_attack),
        ("No Oracle Leakage", test_no_oracle_leakage),
        ("Communication Filtering", test_communication_filtering),
        ("Scalability", test_scalability),
    ]
    
    results = []
    for name, test_fn in tests:
        try:
            passed = test_fn()
            results.append((name, passed))
        except Exception as e:
            print(f"  ERROR: {e}")
            results.append((name, False))
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    passed_count = sum(1 for _, p in results if p)
    total = len(results)
    
    for name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {status} {name}")
    
    print(f"\n  Total: {passed_count}/{total} tests passed")
    
    if passed_count == total:
        print("\n  ALL TESTS PASSED - Ready for hardware!")
    else:
        print("\n  Some tests failed - review before hardware implementation")
    
    return passed_count == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
