#!/usr/bin/env python
"""
Live Debug Visualization - Watch REIP in action!
Shows: agents, frontiers, trust, coverage, and attacks
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
import random

from src.env.gridworld import GridWorld
from src.policy.reip_true import REIPController
from src.comms.channel import share_maps


def run_visualization():
    seed = 42
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
        'comm': {'radius': 12, 'p_loss': 0.0},  # Fixed: comms radius = 12
        'agent': {'r_local': 8, 'ds': 4, 'share_interval': 1},  # Fixed: sensing radius = 8
        'phase2': {'enable': True, 'leader_blind': False, 'frontiers_from_leader': True},
        'reip': {
            'command_radius': 12,  # Match comms radius
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
            # MPC Trust (optimal parameters from tuning)
            'mpc_trust_enabled': True,
            'mpc_trust_weight': 1.0,  # MPC trust exclusively
            'mpc_trust_window': 3,
            'mpc_trust_threshold': 0.1,  # Low threshold for sensitivity
        },
    }
    
    env = GridWorld(cfg)
    policy = REIPController(cfg)
    state = {'t': 0}
    
    T = cfg['T']
    R = cfg['comm']['radius']
    
    # Setup figure
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    plt.ion()  # Interactive mode
    
    coverage_history = []
    trust_history = []
    impeachment_times = []  # Track when impeachments happen
    prev_impeachment_count = 0
    
    # Colors for agents
    agent_colors = plt.cm.tab10(np.linspace(0, 1, cfg['N']))
    
    for t in range(T):
        env.current_time = t
        share_maps(env, R=R, p_loss=0.0)
        
        leader_agent = env.agents.get(policy.leader_id)
        frontiers = env.detect_frontiers_from_agent_belief(leader_agent)
        
        state['t'] = t
        policy.step(env, frontiers, state)
        
        cov = env.coverage()
        avg_trust = policy.get_average_trust_in_leader()
        coverage_history.append(cov * 100)
        trust_history.append(avg_trust)
        
        # Track impeachment events
        if policy.impeachment_count > prev_impeachment_count:
            impeachment_times.append(t)
            prev_impeachment_count = policy.impeachment_count
        
        # Debug: Print attack info
        attack_active = getattr(policy, 'hallucination_active', False)
        if attack_active and t % 10 == 0:
            corrupted = getattr(policy, '_last_dense_explored_corrupted', 0)
            targets = getattr(policy, '_last_dense_explored_targets', 0)
            candidates = getattr(policy, '_last_dense_explored_candidates', 0)
            pred = getattr(policy, 'predicted_coverage_gain', 0)
            obs = getattr(policy, '_last_observed_gain', 0)
            pred_unk = getattr(policy, '_last_predicted_unknown', 0)
            raw_pred_ratio = getattr(policy, '_raw_pred_ratio', 0)
            pred_min = getattr(policy, 'trust_event_pred_min', 0.05)
            obs_max = getattr(policy, 'trust_event_obs_max', 0.30)
            print(f"[t={t}] ATTACK: corrupt={corrupted}/{targets}, candidates={candidates}")
            print(f"  Trust: pred_unk={pred_unk}, raw_ratio={raw_pred_ratio:.3f}, pred={pred:.3f}, obs={obs:.3f}")
            print(f"  Gating: raw_ratio>={pred_min}? {raw_pred_ratio >= pred_min}, obs<={obs_max}? {obs <= obs_max}, frontiers={len(frontiers)}")
        
        # Update visualization every 5 steps
        if t % 5 == 0 or t < 10:
            # Clear axes
            axes[0].clear()
            axes[1].clear()
            
            # === Left plot: Map ===
            ax = axes[0]
            
            # Draw team belief
            tb = env.team_belief.T  # Transpose for correct orientation
            map_display = np.zeros_like(tb, dtype=float)
            map_display[tb == -1] = 0.5  # Unknown = gray
            map_display[tb == 0] = 1.0   # Free = white
            map_display[tb == 2] = 0.0   # Obstacle = black
            
            ax.imshow(map_display, cmap='gray', origin='lower', vmin=0, vmax=1)
            
            # Draw agents
            r_local = cfg['agent']['r_local']  # Sensing radius
            for aid, pos in env.agent_positions.items():
                color = agent_colors[aid]
                is_leader = (aid == policy.leader_id)
                is_compromised = (aid == policy.compromised_agent_id)
                
                marker = 's' if is_leader else 'o'
                size = 200 if is_leader else 100
                edgecolor = 'red' if is_compromised else 'black'
                linewidth = 3 if is_compromised else 1
                
                # Draw sensing radius circle (faint)
                circle = plt.Circle((pos[1], pos[0]), r_local, 
                                   color=color, fill=False, alpha=0.3, linestyle='--')
                ax.add_patch(circle)
                
                ax.scatter(pos[1], pos[0], c=[color], s=size, marker=marker,
                          edgecolors=edgecolor, linewidths=linewidth, zorder=10)
                
                # Agent label
                ax.annotate(str(aid), (pos[1]+1, pos[0]+1), fontsize=8, 
                           fontweight='bold', color=color, zorder=11)
                
                # Draw target line
                tgt = getattr(env.agents.get(aid), 'hold_target', None)
                if tgt:
                    ax.plot([pos[1], tgt[1]], [pos[0], tgt[0]], 
                           color=color, alpha=0.5, linewidth=1)
            
            # Draw frontiers
            for fx, fy in frontiers[:10]:  # Limit to 10
                ax.scatter(fy, fx, c='cyan', s=50, marker='x', zorder=5)
            
            # Attack indicator
            attack_active = getattr(policy, 'hallucination_active', False)
            status = f"t={t} | Leader={policy.leader_id}"
            if attack_active:
                status += " | ATTACK ACTIVE!"
                ax.set_facecolor('#ffcccc')
            
            ax.set_title(status)
            ax.set_xlim(-1, env.size)
            ax.set_ylim(-1, env.size)
            
            # Legend
            legend_elements = [
                mpatches.Patch(facecolor='white', edgecolor='black', label='Free'),
                mpatches.Patch(facecolor='gray', edgecolor='gray', label='Unknown'),
                mpatches.Patch(facecolor='black', edgecolor='black', label='Obstacle'),
                plt.Line2D([0], [0], marker='x', color='w', markerfacecolor='cyan', 
                          markeredgecolor='cyan', markersize=10, label='Frontier'),
            ]
            ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
            
            # === Right plot: Metrics ===
            ax2 = axes[1]
            
            time_steps = list(range(len(coverage_history)))
            ax2.plot(time_steps, coverage_history, 'b-', label='Coverage %', linewidth=2)
            trust_scaled = [tr * 100 for tr in trust_history]  # Fixed variable name
            ax2.plot(time_steps, trust_scaled, 'g-', label='Trust x100', linewidth=2)
            
            # Mark attack period
            attack_start = 30
            if t >= attack_start:
                ax2.axvline(x=attack_start, color='red', linestyle='--', alpha=0.7, label='Attack Start')
            
            # Mark all impeachments
            for imp_t in impeachment_times:
                ax2.axvline(x=imp_t, color='purple', linestyle='-', linewidth=2, alpha=0.8)
            
            ax2.set_xlim(0, T)
            ax2.set_ylim(0, 105)
            ax2.set_xlabel('Timestep')
            ax2.set_ylabel('Percentage')
            
            imp_label = f"Impeachment at t={impeachment_times[-1]}" if impeachment_times else "No impeachment"
            ax2.set_title(f'Coverage: {cov*100:.1f}% | Trust: {avg_trust:.2f} | {imp_label}')
            ax2.legend(loc='lower right')
            ax2.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.pause(0.1)  # Brief pause for animation
    
    plt.ioff()
    print(f"\nFinal: Coverage={cov*100:.1f}%, Trust={avg_trust:.2f}, Impeachments={policy.impeachment_count}")
    plt.show()


if __name__ == '__main__':
    print("=" * 60)
    print("  REIP LIVE VISUALIZATION")
    print("  Watch the attack, trust drop, and impeachment!")
    print("=" * 60)
    print()
    print("Legend:")
    print("  Square = Leader")
    print("  Red border = Compromised agent")
    print("  Cyan X = Frontier")
    print("  Red background = Attack active")
    print()
    run_visualization()
