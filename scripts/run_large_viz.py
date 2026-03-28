"""
Enhanced visualization with stats overlay.
Shows live exploration with coverage, trust, and communication metrics.
"""

import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
import time

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
from src.policy.reip_true import REIPController

# Load config
print("Loading configuration...")
with open("configs/reip_large_viz.yaml", "r") as f:
    cfg = yaml.safe_load(f)

print(f"Creating {cfg['map_size']}*{cfg['map_size']} environment with {cfg['N']} agents...")
env = GridWorld(cfg)

# Initialize REIP controller
controller = REIPController(cfg)
state = {
    't': 0,
    'leader': 0,
    'prev': {},
    'election_count': 0
}

# Setup visualization
fig, (ax_main, ax_stats) = plt.subplots(1, 2, figsize=(16, 8), 
                                         gridspec_kw={'width_ratios': [3, 1]})
plt.ion()

# Colors
cmap_bg = ListedColormap(["#222222", "#9ec3e6", "#ffffff"])
agent_colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 
                'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray',
                'tab:olive', 'tab:cyan', '#FF6B6B', '#4ECDC4']

comm_cfg = cfg.get("comm", {})
T = cfg["T"]

print(f"\nStarting {T}-step exploration with live visualization...")
print("Watch the matplotlib window for real-time updates!")
print("Press Ctrl+C in terminal to stop early\n")

coverage_history = []
trust_history = []
election_history = []
comm_success_history = []

try:
    for t in range(T):
        state['t'] = t
        
        # Communication with packet loss tracking
        use_ge = comm_cfg.get("use_gilbert_elliot", False)
        ge_params = comm_cfg.get("gilbert_elliot_params", None)
        
        success_count = share_maps(env,
                                   R=comm_cfg.get("radius", 20),
                                   p_loss=comm_cfg.get("p_loss", 0.0),
                                   use_gilbert_elliot=use_ge,
                                   ge_params=ge_params,
                                   timestep=t)
        
        # Track communication success rate
        n_agents = len(env.agents)
        max_exchanges = n_agents * (n_agents - 1)  # Bidirectional
        comm_rate = success_count / max(1, max_exchanges) if max_exchanges > 0 else 0
        comm_success_history.append(comm_rate)
        
        # Detect frontiers
        frontiers = env.detect_frontiers()
        
        if not frontiers:
            print(f"\n Exploration complete at step {t}!")
            break
        
        # REIP controller step
        controller.step(env, frontiers, state)
        
        # Get assignments from state
        assigns = state.get('prev', {})
        
        # Move agents (simplified)
        env.step(assigns, t=t)
        
        # Track metrics
        coverage = env.coverage()
        coverage_history.append(coverage)
        
        avg_trust = np.mean([np.mean(list(controller.trust[i].values())) 
                            for i in range(cfg['N'])])
        trust_history.append(avg_trust)
        
        election_history.append(controller.election_count)
        
        # Update visualization every 5 steps
        if t % 5 == 0:
            # Clear axes
            ax_main.clear()
            ax_stats.clear()
            
            # === MAIN PLOT: Environment ===
            team_belief = env._team_known_free_world()
            display = np.full((env.size, env.size), -1, dtype=int)
            display[env.grid == -1] = -2  # Obstacles
            display[team_belief] = 0  # Known free
            
            ax_main.imshow(display.T, cmap=cmap_bg, origin='lower', 
                          alpha=0.5, vmin=-2.5, vmax=0.5)
            
            # Draw frontiers
            if len(frontiers) < 1000:  # Don't plot too many
                fx = [f[0] for f in frontiers]
                fy = [f[1] for f in frontiers]
                ax_main.scatter(fx, fy, marker='s', c='yellow', 
                              s=20, alpha=0.6, edgecolors='orange', 
                              linewidth=0.5, zorder=5)
            
            # Draw agents
            leader_id = state['leader']
            for i, agent in env.agents.items():
                pos = agent.pos
                color = agent_colors[i % len(agent_colors)]
                
                # Sensing radius
                circ = Circle(pos, agent.r_local, color=color, 
                            fill=False, linestyle='--', alpha=0.3, linewidth=1)
                ax_main.add_patch(circ)
                
                # Agent marker (leader is larger with star)
                if i == leader_id:
                    ax_main.plot([pos[0]], [pos[1]], marker='*', 
                               markersize=20, color='gold', 
                               markeredgecolor='black', linewidth=2, zorder=10)
                    ax_main.plot([pos[0]], [pos[1]], marker='o', 
                               markersize=15, color=color, 
                               markeredgecolor='black', linewidth=2, zorder=9)
                else:
                    ax_main.plot([pos[0]], [pos[1]], marker='o', 
                               markersize=12, color=color, 
                               markeredgecolor='black', linewidth=1.5, zorder=10)
                
                # Agent ID label
                ax_main.text(pos[0], pos[1], str(i),
                           ha='center', va='center', fontsize=8, 
                           weight='bold', color='white', zorder=11)
            
            ax_main.set_xlim(-1, env.size)
            ax_main.set_ylim(-1, env.size)
            ax_main.set_aspect('equal')
            ax_main.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)
            ax_main.set_title(f"Step {t}/{T} | Coverage: {coverage:.1%} | "
                            f"Leader: Agent {leader_id} | "
                            f"Elections: {controller.election_count}",
                            fontsize=12, weight='bold')
            
            # === STATS PLOT: Metrics ===
            ax_stats.axis('off')
            
            # Text stats
            stats_text = f"""
EXPLORATION METRICS
{'='*30}

Coverage: {coverage:.2%}
Frontiers: {len(frontiers)}
Timestep: {t}/{T}

GOVERNANCE
{'='*30}

Current Leader: Agent {leader_id}
Elections: {controller.election_count}
Avg Trust: {avg_trust:.3f}

COMMUNICATION
{'='*30}

Success Rate: {comm_rate:.1%}
Gilbert-Elliot: {'ON' if use_ge else 'OFF'}

AGENTS
{'='*30}
"""
            
            # Add per-agent trust in leader
            for i in range(min(6, cfg['N'])):  # Show first 6
                trust_in_leader = controller.trust[i][leader_id]
                stats_text += f"Agent {i}: T={trust_in_leader:.2f}\n"
            
            if cfg['N'] > 6:
                stats_text += f"... ({cfg['N']-6} more)\n"
            
            ax_stats.text(0.1, 0.95, stats_text, transform=ax_stats.transAxes,
                         fontsize=9, verticalalignment='top', family='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', 
                                  alpha=0.8, edgecolor='black', linewidth=1))
            
            # Mini plots
            if len(coverage_history) > 10:
                # Coverage plot
                ax_cov = fig.add_axes([0.72, 0.55, 0.22, 0.15])
                ax_cov.plot(coverage_history, color='green', linewidth=2)
                ax_cov.set_ylabel('Coverage', fontsize=8)
                ax_cov.set_xlim(0, T)
                ax_cov.set_ylim(0, 1)
                ax_cov.grid(True, alpha=0.3)
                ax_cov.tick_params(labelsize=7)
                
                # Trust plot
                ax_trust = fig.add_axes([0.72, 0.35, 0.22, 0.15])
                ax_trust.plot(trust_history, color='blue', linewidth=2)
                ax_trust.set_ylabel('Avg Trust', fontsize=8)
                ax_trust.set_xlim(0, T)
                ax_trust.set_ylim(0, 1)
                ax_trust.grid(True, alpha=0.3)
                ax_trust.tick_params(labelsize=7)
                
                # Communication plot
                ax_comm = fig.add_axes([0.72, 0.15, 0.22, 0.15])
                ax_comm.plot(comm_success_history, color='orange', linewidth=2)
                ax_comm.set_ylabel('Comm Rate', fontsize=8)
                ax_comm.set_xlabel('Timestep', fontsize=8)
                ax_comm.set_xlim(0, T)
                ax_comm.set_ylim(0, 1)
                ax_comm.grid(True, alpha=0.3)
                ax_comm.tick_params(labelsize=7)
            
            plt.pause(0.01)
            
            # Print progress
            if t % 20 == 0:
                print(f"Step {t:3d}: Coverage {coverage:.1%}, "
                      f"Leader={leader_id}, Trust={avg_trust:.2f}, "
                      f"CommRate={comm_rate:.1%}")

except KeyboardInterrupt:
    print("\n\nStopped by user")

print("\n" + "="*60)
print("FINAL RESULTS")
print("="*60)
print(f"Final coverage: {env.coverage():.2%}")
print(f"Steps taken: {t}")
print(f"Leadership elections: {controller.election_count}")
print(f"Final average trust: {avg_trust:.3f}")

# Keep window open
plt.ioff()
input("\nPress Enter to close visualization...")
plt.close()
