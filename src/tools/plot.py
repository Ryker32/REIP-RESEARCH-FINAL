"""
Enhanced visualization for REIP multi-agent exploration system.

Provides comprehensive visualization of:
- Main exploration map with team belief overlay
- Individual agent worldviews (belief maps) shown side-by-side
- Trust metrics, leader status, and governance indicators
- Real-time metrics for conference demonstrations

References:
- Matplotlib documentation: Subplot grids and advanced layouts
- Visualization best practices for multi-robot systems (IEEE T-RO)

Usage example:
    from src.tools.plot import plot_env_enhanced
    for t in range(T):
        plot_env_enhanced(env, reip_controller, timestep=t, show=False)
        plt.pause(0.1)
"""

from typing import Optional, Dict, Any
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.gridspec import GridSpec


# assign some distinct colors for agents
_AGENT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", 
                 "tab:brown", "tab:pink", "tab:gray", "tab:olive", "tab:cyan"]


def plot_env(env,
             ax: Optional[plt.Axes] = None,
             show: bool = True,
             draw_local: bool = True,
             draw_frontiers: bool = True,
             frontiers: Optional[list] = None,
             timestep: Optional[int] = None,
             leader_id: Optional[int] = None,
             command_radius: Optional[int] = None,
             avg_trust: Optional[float] = None):
    """Plot the current GridWorld state.

    Args:
        env: GridWorld instance
        ax: optional matplotlib Axes to draw onto
        show: if True calls plt.show(), else leaves drawing to caller
        draw_local: whether to overlay per-agent local observations
        draw_frontiers: whether to highlight detected frontiers
    """
    size = env.size

    # Prepare base display: -2 obstacle, -1 unknown, 0 known free
    display = np.full((size, size), -1, dtype=int)
    # obstacles
    display[env.grid == -1] = -2
    # team-known free cells
    try:
        known = env._team_known_free_world()
    except Exception:
        # fallback: union of agents' global_lr (coarse)
        known = np.zeros((size, size), dtype=bool)
        for a in env.agents.values():
            if hasattr(a, "global_lr"):
                lr = a.global_lr
                ds = a.ds
                gx, gy = lr.shape
                for ix in range(gx):
                    for iy in range(gy):
                        if lr[ix, iy] == 0:  # free
                            wx0, wy0 = ix * ds, iy * ds
                            wx1, wy1 = min(size, wx0 + ds), min(size, wy0 + ds)
                            known[wx0:wx1, wy0:wy1] = True

    display[known] = 0

    # colormap mapping: indices [-2, -1, 0] -> [obst, unknown, free]
    cmap = ListedColormap(["#222222", "#9ec3e6", "#ffffff"])  # dark, light blue, white
    bounds = [-2.5, -1.5, -0.5, 0.5]

    # Optionally render per-agent belief maps to the right of the main map
    draw_agent_maps = getattr(env, 'draw_agent_maps', False)
    agent_map_axes = None
    if ax is None:
        if draw_agent_maps:
            # Create a wider figure with a right column for small agent maps
            fig = plt.figure(figsize=(10, 6))
            gs = fig.add_gridspec(1, 2, width_ratios=[3, 1])
            ax = fig.add_subplot(gs[0, 0])
            # create small axes stacked vertically for each agent
            agent_map_axes = []
            n_agents = max(1, len(env.agents))
            sub_gs = gs[0, 1].subgridspec(n_agents, 1)
            for i in range(n_agents):
                agent_map_axes.append(fig.add_subplot(sub_gs[i, 0]))
        else:
            fig, ax = plt.subplots(figsize=(6, 6))
    else:
        fig = ax.figure

    ax.clear()

    # imshow expects increasing y downwards; we want origin at top-left like array indices
    ax.imshow(display.T, cmap=cmap, origin="lower", vmin=-2.5, vmax=0.5)

    # draw frontiers
    if draw_frontiers:
        if frontiers is None:
            try:
                frontiers = env.detect_frontiers()
            except Exception:
                frontiers = []
        if frontiers:
            fx = [p[0] for p in frontiers]
            fy = [p[1] for p in frontiers]
            ax.scatter(fx, fy, marker="s", c="yellow", edgecolors="k", s=40, label="frontier")

    # draw per-agent local observations and agent markers
    for i, a in env.agents.items():
        x, y = env.agent_positions.get(i, a.pos)
        color = _AGENT_COLORS[i % len(_AGENT_COLORS)]

        # local observation overlay (high-res)
        if draw_local and hasattr(a, "local"):
            L = a.local
            R = a.r_local
            lx, ly = L.shape
            xs = []
            ys = []
            vals = []
            for xi in range(lx):
                for yi in range(ly):
                    v = L[xi, yi]
                    if v == -1:
                        continue
                    # world coords
                    wx, wy = (x - R + xi), (y - R + yi)
                    if 0 <= wx < size and 0 <= wy < size:
                        xs.append(wx)
                        ys.append(wy)
                        vals.append(v)
            if xs:
                # color free as translucent green, obs as translucent black
                cols = [(0.2, 1.0, 0.2, 0.35) if v == 0 else (0.0, 0.0, 0.0, 0.35) for v in vals]
                ax.scatter(xs, ys, marker="s", s=80, c=cols, linewidths=0)

        # draw agent position
        ax.plot([x], [y], marker='o', markersize=10, color=color, markeredgecolor='k')
        ax.text(x + 0.3, y + 0.3, str(i), color='k', fontsize=9, weight='bold', bbox=dict(facecolor='white', alpha=0.6, edgecolor='none', pad=0))

        # dashed circle for sensing radius
        circ = plt.Circle((x, y), a.r_local, color=color, fill=False, linestyle='dashed', alpha=0.5)
        ax.add_patch(circ)

        # highlight current leader and draw command radius overlay
        if leader_id is not None and i == leader_id:
            # leader ring
            ring = plt.Circle((x, y), 1.2, color='gold', fill=False, linewidth=2.5, linestyle='solid', alpha=0.9)
            ax.add_patch(ring)
            # command radius
            if command_radius is not None and command_radius > 0:
                cmd = plt.Circle((x, y), command_radius, color='gold', fill=False, linestyle='dashdot', alpha=0.4)
                ax.add_patch(cmd)

    ax.set_xlim(-0.5, size - 0.5)
    ax.set_ylim(-0.5, size - 0.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    if timestep is not None:
        if avg_trust is not None:
            ax.set_title(f"t={timestep}, N={env.N}, Avg_Trust={avg_trust:.2f}, Current_Leader={leader_id}")
        else:
            ax.set_title(f"t={timestep}, N={env.N}")
    else:
        ax.set_title(f"t=??, N={env.N}")

    if show:
        # Also draw per-agent mini-maps if requested
        if agent_map_axes is not None:
            # For each agent, render its coarse belief map (belief_lr) or local view
            for idx, (i, a) in enumerate(env.agents.items()):
                try:
                    subax = agent_map_axes[idx]
                except Exception:
                    continue
                subax.clear()
                # render belief_lr if available
                bl = getattr(a, 'belief_lr', None)
                if bl is not None:
                    # Upsample coarse belief to a small image
                    img = np.array(bl)
                    cmap_small = ListedColormap(["#ffffff", "#9ec3e6", "#222222"])  # free, unknown, obstacle
                    subax.imshow(img.T, cmap=cmap_small, origin='lower')
                else:
                    # fallback: show local view centered
                    L = getattr(a, 'local', None)
                    if L is not None:
                        subax.imshow(L.T, cmap=ListedColormap(["#222222", "#ffffff"]), origin='lower')
                subax.set_title(f"Agent {i}")
                subax.set_xticks([])
                subax.set_yticks([])
        plt.show()

    return ax


def plot_env_enhanced(env,
                     reip_controller: Optional[Any] = None,
                     ax: Optional[plt.Axes] = None,
                     show: bool = True,
                     draw_local: bool = True,
                     draw_frontiers: bool = True,
                     frontiers: Optional[list] = None,
                     timestep: Optional[int] = None,
                     leader_id: Optional[int] = None,
                     command_radius: Optional[int] = None,
                     avg_trust: Optional[float] = None,
                     show_agent_worldviews: bool = True):
    """
    Enhanced visualization with side-by-side agent worldview panels.
    
    Displays:
    - Main map: Team belief, frontiers, agents, connectivity
    - Agent worldviews: Individual belief_lr maps showing each agent's limited knowledge
    - Trust metrics: Per-agent trust scores and leader indicators
    - Governance status: Impeachment warnings, election history
    
    Designed for conference demonstrations to show:
    1. Distributed nature (each agent has different worldview)
    2. Limited communication (belief maps diverge)
    3. Trust-based governance (leader impeachment visible)
    
    Args:
        env: GridWorld environment
        reip_controller: REIPController instance (for trust/leader data)
        show_agent_worldviews: If True, display individual agent belief maps
        Other args: See plot_env()
    
    Returns:
        Main axes object
    """
    size = env.size
    
    # Prepare base display
    display = np.full((size, size), -1, dtype=int)
    display[env.grid == -1] = -2  # obstacles
    
    try:
        known = env._team_known_free_world()
    except Exception:
        known = np.zeros((size, size), dtype=bool)
        for a in env.agents.values():
            if hasattr(a, "global_lr"):
                lr = a.global_lr
                ds = a.ds
                gx, gy = lr.shape
                for ix in range(gx):
                    for iy in range(gy):
                        if lr[ix, iy] == 0:
                            wx0, wy0 = ix * ds, iy * ds
                            wx1, wy1 = min(size, wx0 + ds), min(size, wy0 + ds)
                            known[wx0:wx1, wy0:wy1] = True
    
    display[known] = 0
    
    # Colormap
    cmap = ListedColormap(["#222222", "#9ec3e6", "#ffffff"])  # dark, light blue, white
    
    # Create figure with enhanced layout
    # CRITICAL: Always create enhanced layout when show_agent_worldviews=True, 
    # even if ax is provided (reuse existing figure if it has the right layout)
    if show_agent_worldviews and len(env.agents) > 0:
        # Check if we need to create a new figure with enhanced layout
        if ax is None or not hasattr(ax.figure, '_reip_enhanced_layout'):
            # Create new enhanced figure: main map | agent worldviews (each with label to right)
            fig = plt.figure(figsize=(16, 10))
            fig._reip_enhanced_layout = True  # Mark as enhanced layout
            # Layout: [main map (left) | agent worldviews with labels (right)]
            # Each agent gets: small map + label to its right in a nested grid
            gs = GridSpec(2, 2, figure=fig, height_ratios=[3, 1], width_ratios=[3, 1.5], 
                         hspace=0.3, wspace=0.3)
            
            # Main map takes left column
            ax = fig.add_subplot(gs[0, 0])
            
            # Agent worldviews in right column (stacked) - each will have label on its right
            n_agents = len(env.agents)
            agent_axes = []
            agent_label_axes = []
            if n_agents <= 6:
                # Stack all agents vertically, each with its own label ax
                # Create subgrid once for all agents: map on left, label on right
                agent_grid = gs[0, 1].subgridspec(n_agents, 2, height_ratios=[1]*n_agents, 
                                                  width_ratios=[3, 0.8], hspace=0.1, wspace=0.1)
                for i in range(n_agents):
                    agent_axes.append(fig.add_subplot(agent_grid[i, 0]))
                    label_ax = fig.add_subplot(agent_grid[i, 1])
                    label_ax.axis('off')
                    agent_label_axes.append(label_ax)
            else:
                # Use a subgrid for many agents
                agent_gs = gs[0, 1].subgridspec(n_agents, 2, width_ratios=[3, 0.8], hspace=0.1, wspace=0.1)
                for i in range(n_agents):
                    agent_axes.append(fig.add_subplot(agent_gs[i, 0]))
                    label_ax = fig.add_subplot(agent_gs[i, 1])
                    label_ax.axis('off')
                    agent_label_axes.append(label_ax)
            
            # Store label axes for reuse
            fig._reip_agent_label_axes = agent_label_axes
            
            # Metrics panel at bottom left
            metrics_ax = fig.add_subplot(gs[1, 0])
        else:
            # Reuse existing enhanced layout - find the agent axes from figure
            fig = ax.figure
            # Try to find existing agent axes and label axes
            agent_axes = []
            agent_label_axes = []
            metrics_ax = None
            
            # Try to get stored references first
            if hasattr(fig, '_reip_agent_label_axes'):
                agent_label_axes = fig._reip_agent_label_axes
            
            # Extract agent axes and metrics ax from existing figure
            for child in fig.get_children():
                if isinstance(child, plt.Axes):
                    pos = child.get_position()
                    # Right column should be agent worldviews or labels (x > 0.65)
                    # Labels will be narrower (width < 0.1) and to the right of maps
                    # Maps will be wider
                    if pos.x0 > 0.65:
                        if pos.width < 0.1:  # Narrow = label
                            if child not in agent_label_axes:
                                agent_label_axes.append(child)
                        else:  # Wide = agent worldview map
                            agent_axes.append(child)
                    elif pos.y0 < 0.3:  # Bottom row - metrics
                        metrics_ax = child
            
            # Sort axes by vertical position (top to bottom)
            agent_axes.sort(key=lambda a: a.get_position().y0, reverse=True)
            agent_label_axes.sort(key=lambda a: a.get_position().y0, reverse=True)
            
            # If we couldn't find them, recreate them
            if len(agent_axes) == 0:
                gs = GridSpec(2, 2, figure=fig, height_ratios=[3, 1], width_ratios=[3, 1.5], 
                             hspace=0.3, wspace=0.3)
                n_agents = len(env.agents)
                agent_axes = []
                agent_label_axes = []
                if n_agents <= 6:
                    agent_grid = gs[0, 1].subgridspec(n_agents, 2, height_ratios=[1]*n_agents,
                                                      width_ratios=[3, 0.8], hspace=0.1, wspace=0.1)
                    for i in range(n_agents):
                        agent_axes.append(fig.add_subplot(agent_grid[i, 0]))
                        label_ax = fig.add_subplot(agent_grid[i, 1])
                        label_ax.axis('off')
                        agent_label_axes.append(label_ax)
                else:
                    agent_gs = gs[0, 1].subgridspec(n_agents, 2, width_ratios=[3, 0.8], 
                                                     hspace=0.1, wspace=0.1)
                    for i in range(n_agents):
                        agent_axes.append(fig.add_subplot(agent_gs[i, 0]))
                        label_ax = fig.add_subplot(agent_gs[i, 1])
                        label_ax.axis('off')
                        agent_label_axes.append(label_ax)
                metrics_ax = fig.add_subplot(gs[1, 0])
                fig._reip_agent_label_axes = agent_label_axes
    else:
        # Standard layout (no agent worldviews)
        if ax is None:
            fig, ax = plt.subplots(figsize=(8, 8))
        else:
            fig = ax.figure
        agent_axes = None
        agent_label_axes = []  # Initialize empty to avoid errors
        metrics_ax = None
    
    ax.clear()
    ax.imshow(display.T, cmap=cmap, origin="lower", vmin=-2.5, vmax=0.5)
    
    # Draw frontiers
    if draw_frontiers:
        if frontiers is None:
            try:
                frontiers = env.detect_frontiers()
            except Exception:
                frontiers = []
        if frontiers:
            fx = [p[0] for p in frontiers]
            fy = [p[1] for p in frontiers]
            ax.scatter(fx, fy, marker="s", c="yellow", edgecolors="k", s=40, 
                      label="frontier", zorder=3)
    
    # Draw agents with enhanced annotations
    for i, a in env.agents.items():
        x, y = env.agent_positions.get(i, a.pos)
        color = _AGENT_COLORS[i % len(_AGENT_COLORS)]
        
        # Local observation overlay
        if draw_local and hasattr(a, "local"):
            L = a.local
            R = a.r_local
            lx, ly = L.shape
            xs, ys, vals = [], [], []
            for xi in range(lx):
                for yi in range(ly):
                    v = L[xi, yi]
                    if v == -1:
                        continue
                    wx, wy = (x - R + xi), (y - R + yi)
                    if 0 <= wx < size and 0 <= wy < size:
                        xs.append(wx)
                        ys.append(wy)
                        vals.append(v)
            if xs:
                cols = [(0.2, 1.0, 0.2, 0.35) if v == 0 else (0.0, 0.0, 0.0, 0.35) 
                       for v in vals]
                ax.scatter(xs, ys, marker="s", s=80, c=cols, linewidths=0, zorder=1)
        
        # Agent marker
        marker_size = 12
        marker_color = color
        if leader_id is not None and i == leader_id:
            marker_size = 14
            marker_color = 'gold'
            # Leader crown indicator
            ring = plt.Circle((x, y), 1.5, color='gold', fill=False, 
                            linewidth=3, linestyle='solid', alpha=0.9, zorder=5)
            ax.add_patch(ring)
            if command_radius is not None and command_radius > 0:
                cmd = plt.Circle((x, y), command_radius, color='gold', fill=False, 
                               linestyle='dashdot', alpha=0.3, zorder=2)
                ax.add_patch(cmd)
        
        ax.plot([x], [y], marker='o', markersize=marker_size, color=marker_color, 
               markeredgecolor='k', markeredgewidth=2, zorder=4)
        ax.text(x + 0.4, y + 0.4, str(i), color='k', fontsize=10, weight='bold',
               bbox=dict(facecolor='white', alpha=0.8, edgecolor='k', pad=2), zorder=6)
        
        # Sensing radius
        circ = plt.Circle((x, y), a.r_local, color=color, fill=False, 
                         linestyle='dashed', alpha=0.4, linewidth=1.5, zorder=2)
        ax.add_patch(circ)
        
        # Trust annotation (if available)
        if reip_controller is not None and hasattr(reip_controller, 'trust'):
            try:
                trust_score = reip_controller.get_average_trust_in_leader()
                if i == leader_id:
                    trust_text = f"T={trust_score:.2f}"
                    ax.text(x, y - 2.5, trust_text, fontsize=8, ha='center',
                           bbox=dict(facecolor='yellow', alpha=0.7, pad=1), zorder=6)
            except Exception:
                pass
    
    ax.set_xlim(-0.5, size - 0.5)
    ax.set_ylim(-0.5, size - 0.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    
    # Enhanced title with governance metrics
    title_parts = [f"t={timestep}" if timestep is not None else "t=??"]
    title_parts.append(f"N={env.N}")
    if leader_id is not None:
        title_parts.append(f"Leader={leader_id}")
    if avg_trust is not None:
        title_parts.append(f"Trust={avg_trust:.2f}")
    if reip_controller is not None:
        try:
            title_parts.append(f"Elections={reip_controller.election_count}")
        except Exception:
            pass
    ax.set_title(" | ".join(title_parts), fontsize=11, weight='bold')
    
    # Draw agent worldview panels with labels directly to the right of each
    if agent_axes is not None and show_agent_worldviews:
        for idx, (agent_id, agent) in enumerate(env.agents.items()):
            if idx >= len(agent_axes):
                break
            
            agent_ax = agent_axes[idx]
            agent_ax.clear()
            
            # Render belief_lr map
            bl = getattr(agent, 'belief_lr', None)
            if bl is not None:
                img = np.array(bl)
                # Colormap: white=free, light blue=unknown, dark=obstacle
                cmap_agent = ListedColormap(["#ffffff", "#9ec3e6", "#222222"])
                agent_ax.imshow(img.T, cmap=cmap_agent, origin='lower', interpolation='nearest')
                
                # Add agent position indicator
                pos = env.agent_positions.get(agent_id)
                if pos is not None:
                    # Convert world position to belief_lr coordinates
                    ds = agent.ds
                    bl_x, bl_y = pos[0] // ds, pos[1] // ds
                    if 0 <= bl_x < bl.shape[0] and 0 <= bl_y < bl.shape[1]:
                        agent_ax.plot([bl_x], [bl_y], 'o', color='red', 
                                    markersize=6, markeredgecolor='white', markeredgewidth=1.5)
            else:
                # Fallback: show local map
                L = getattr(agent, 'local', None)
                if L is not None:
                    agent_ax.imshow(L.T, cmap=ListedColormap(["#222222", "#ffffff"]), 
                                  origin='lower', interpolation='nearest')
            
            # No title on agent axes (label is directly to the right)
            agent_ax.set_xticks([])
            agent_ax.set_yticks([])
            agent_ax.set_aspect('equal')
            
            # Draw label directly to the right of this agent's map - simple "Agent X" format
            if show_agent_worldviews and 'agent_label_axes' in locals() and idx < len(agent_label_axes):
                label_ax = agent_label_axes[idx]
                label_ax.clear()
                label_ax.axis('off')
                
                # Simple label: "Agent X"
                agent_label = f"Agent {agent_id}"
                
                # Draw centered label
                label_ax.text(0.5, 0.5, agent_label, 
                             transform=label_ax.transAxes,
                             fontsize=11, verticalalignment='center', horizontalalignment='center',
                             family='sans-serif', weight='bold', color='black')
    
    # Metrics panel (if space available)
    if metrics_ax is not None and reip_controller is not None:
        metrics_ax.clear()
        metrics_ax.axis('off')
        
        metrics_text = []
        metrics_text.append("REIP Governance Metrics")
        metrics_text.append("=" * 30)
        
        try:
            if hasattr(reip_controller, 'election_count'):
                metrics_text.append(f"Elections: {reip_controller.election_count}")
            if hasattr(reip_controller, 'impeachment_count'):
                metrics_text.append(f"Impeachments: {reip_controller.impeachment_count}")
            if avg_trust is not None:
                metrics_text.append(f"Avg Trust: {avg_trust:.3f}")
            if hasattr(reip_controller, 'hallucination_active'):
                if reip_controller.hallucination_active:
                    htype = getattr(reip_controller, 'hallucination_type', 'unknown')
                    metrics_text.append(f"WARNING Hallucination: {htype}")
        except Exception:
            pass
        
        metrics_ax.text(0.05, 0.95, "\n".join(metrics_text), 
                       transform=metrics_ax.transAxes,
                       fontsize=9, verticalalignment='top',
                       family='monospace',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    if show:
        plt.show()
    
    return ax
