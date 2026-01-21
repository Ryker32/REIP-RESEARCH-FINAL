#!/usr/bin/env python
"""
Debug REIP Flow - Trace exactly what happens during attack and impeachment
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

import numpy as np
import random
import copy

from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps


def main():
    print("=" * 70)
    print("       REIP DEBUG TRACE")
    print("       Watching attack -> trust drop -> impeachment")
    print("=" * 70)
    print()
    
    seed = 42
    np.random.seed(seed)
    random.seed(seed)
    
    cfg = {
        'N': 6,
        'T': 500,  # Full run
        'map_size': 60,
        'env': {
            'obstacle_density': 0.05,
            'spawn_center': [30, 30],
            'spawn_radius': 3,
            'seed': seed,
        },
        'comm': {'radius': 20, 'p_loss': 0.0},
        'agent': {'r_local': 5, 'ds': 4, 'share_interval': 1},
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
        'reip': {
            'command_radius': 20,
            'hallucination_rate': 0.0,
            'hallucination_profile': 'dense_explored',
            'hallucination_schedule': [
                {'start': 10, 'type': 'dense_explored', 'duration': 1000},
            ],
            'pred_gain_inflation_base': 10.0,
            'trust_decay_rate': 0.5,
            'trust_threshold': 0.45,
            'min_trust': 0.1,
            'compromised_agent_id': 0,
            'fallback_enabled': False,
            'governance_enabled': True,  # WITH GOVERNANCE
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    # Track positions over time
    position_history = {i: [] for i in range(cfg['N'])}
    
    print(f"Initial leader: Agent {policy.leader_id}")
    print(f"Compromised agent: {policy.compromised_agent_id}")
    print(f"Attack starts at: t=10")
    print(f"Trust threshold: {policy.trust_threshold}")
    print(f"Impeachment persistence: {policy.impeachment_persistence}")
    print(f"Impeachment cooldown: {policy.impeachment_cooldown}")
    print(f"Detection enabled: {policy.detection_enabled}")
    print(f"CUSUM threshold: {policy.hallucination_detector.threshold if policy.hallucination_detector else 'N/A'}")
    print()
    print("-" * 90)
    print(f"{'t':>4} | {'Ldr':>3} | {'Atk':>3} | {'Pred':>6} | {'Obs':>6} | {'TrstOld':>7} | {'CUSUM':>6} | {'Status'}")
    print("-" * 90)
    
    impeached = False
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=0.0)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        
        # Track state BEFORE step
        old_leader = policy.leader_id
        old_trust = policy.get_average_trust_in_leader()
        
        # Debug: Check leader's belief map and assignments
        if t in [11, 12, 13, 20, 50] or old_leader != policy.leader_id:
            leader_a = env.agents.get(policy.leader_id)
            if leader_a and hasattr(leader_a, 'belief_lr'):
                blr = leader_a.belief_lr
                unknown = np.sum(blr == -1)
                free = np.sum(blr == 0)
                obs = np.sum(blr == 2)
                total = blr.size
                # Check agent targets
                targets = []
                for aid, a in env.agents.items():
                    tgt = getattr(a, 'hold_target', None)
                    targets.append(f"{aid}:{tgt}")
                print(f"  [DEBUG t={t}] Leader {policy.leader_id} belief: unknown={unknown}/{total}, frontiers={len(frontiers)}, targets={targets[:3]}")
        
        policy.step(env, frontiers, state)
        
        # Get state AFTER step
        new_leader = policy.leader_id
        new_trust = policy.get_average_trust_in_leader()
        pred = getattr(policy, 'predicted_coverage_gain', 0)
        obs_gain = getattr(policy, '_last_observed_gain', 0)
        
        # Get individual trust values for old leader
        trust_in_old_leader = np.mean([policy.trust[i][old_leader] for i in range(policy.N) if i != old_leader])
        
        # Check attack status
        attack_active = getattr(policy, 'hallucination_active', False)
        
        # Check CUSUM
        cusum_val = 0
        cusum_detected_now = False
        if policy.hallucination_detector:
            cusum_val = policy.hallucination_detector.cusum_statistic.get(old_leader, 0)
            # Check if CUSUM just detected (from last step)
            if policy.trust_history:
                last = policy.trust_history[-1]
                cusum_detected_now = last.get('cusum_detected', False)
        
        # Check impeachment trigger reason from history
        imp_reason = ""
        if policy.trust_history and len(policy.trust_history) > 0:
            last_hist = policy.trust_history[-1]
            if last_hist.get('cusum_detected', False):
                imp_reason = "CUSUM"
            elif (last_hist.get('bayes_fault_prob') or 0) > 0.5:
                imp_reason = "BAYES"
        
        # Determine status
        status = ""
        if old_leader != new_leader:
            status = f"IMPEACHED! {old_leader} -> {new_leader}"
            if imp_reason:
                status += f" ({imp_reason})"
            impeached = True
        elif attack_active:
            status = "ATTACK ACTIVE"
        elif t < 10:
            status = "normal"
        else:
            status = "attack stopped" if impeached else "normal"
        
        # Print every step for first 20, then every 5 steps, or on events
        cusum_flag = "*" if cusum_detected_now else " "
        if t < 30 or t % 5 == 0 or old_leader != new_leader or (attack_active and t >= 10):
            print(f"{t:>4} | {new_leader:>3} | {'Y' if attack_active else 'n':>3} | {pred:>6.3f} | {obs_gain:>6.3f} | {trust_in_old_leader:>7.3f} | {cusum_val:>5.2f}{cusum_flag} | {status}")
        
        # Don't break early - run full simulation
        # if impeached and not attack_active and t > 50:
        #     print(f"\n... (skipping to end, attack stopped at impeachment)")
        #     break
    
    print("-" * 90)
    print()
    
    # Calculate agent movement stats
    print("AGENT MOVEMENT STATS:")
    for aid in range(cfg['N']):
        a = env.agents.get(aid)
        if a:
            pos = env.agent_positions.get(aid, (0,0))
            print(f"  Agent {aid}: final pos {pos}")
    
    # Show fine-resolution coverage
    total_cells = env.size * env.size
    seen_cells = sum(a.seen_world.sum() for a in env.agents.values())
    
    # Check team_belief state
    tb = env.team_belief
    tb_unknown = np.sum(tb == -1)
    tb_free = np.sum(tb == 0)
    tb_obs = np.sum(tb == 2)
    
    print(f"\nCOVERAGE BREAKDOWN:")
    print(f"  Map size: {env.size}x{env.size} = {total_cells} cells")
    print(f"  Actual obstacles in map: {np.sum(env.grid == -1)}")
    print(f"  team_belief: unknown={tb_unknown}, free={tb_free}, obstacle={tb_obs}")
    print(f"  env.coverage(): {env.coverage()*100:.1f}%")
    
    # Check if agents can pathfind
    leader_a = env.agents.get(policy.leader_id)
    frontiers = env.detect_frontiers_from_agent_belief(leader_a)
    print(f"  Final frontiers: {len(frontiers)}")
    if frontiers:
        test_frontier = frontiers[0]
        test_pos = env.agent_positions.get(0)
        print(f"  Test: Agent 0 at {test_pos}, frontier at {test_frontier}")
    
    print(f"\nFINAL STATE:")
    print(f"  Final leader: Agent {policy.leader_id}")
    print(f"  Impeachments: {policy.impeachment_count}")
    print(f"  Final coverage: {env.coverage()*100:.1f}%")
    print(f"  Final trust in leader: {policy.get_average_trust_in_leader():.3f}")


if __name__ == '__main__':
    main()
