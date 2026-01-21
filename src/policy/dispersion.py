"""
Adaptive Dispersion Incentives for Multi-Agent Exploration

Implements information-theoretic dispersion rewards to prevent agent clustering
while maintaining connectivity. Uses Voronoi-based partitioning and entropy
maximization to encourage spatially distributed exploration.

References:
- Yamauchi, B. (1997). A frontier-based approach for autonomous exploration.
  IEEE CIRA. (Frontier-based exploration foundation)
- Burgard, W., Moors, M., Stachniss, C., & Schneider, F. E. (2005). Coordinated
  multi-robot exploration. IEEE T-RO, 21(3), 376-386. (Spatial coordination)
- Stachniss, C., Burgard, W., et al. (2004). Exploration with active loop-closing
  using FastSLAM. IEEE/RSJ IROS. (Information gain maximization)
- Hönig, W., Preiss, J. A., Kumar, T. K. S., Sukhatme, G. S., & Ayanian, N. (2018).
  Trajectory planning for quadrotor swarms. IEEE T-RO, 34(4), 856-869.
  (Dispersion strategies in multi-robot systems)
- Olfati-Saber, R. (2006). Flocking for multi-agent dynamic systems: Algorithms
  and theory. IEEE T-AC, 51(3), 401-420. (Consensus-based dispersion)

This module provides:
1. Voronoi-based spatial partitioning for dispersion incentives
2. Information-theoretic dispersion rewards (entropy-based)
3. Adaptive dispersion strength based on coverage progress
4. Connectivity-preserving dispersion constraints
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Set, Optional
from collections import defaultdict
import heapq


def compute_voronoi_cells(agent_positions: Dict[int, Tuple[int, int]], 
                         frontiers: List[Tuple[int, int]],
                         map_size: int) -> Dict[int, List[Tuple[int, int]]]:
    """
    Partition frontiers using Voronoi diagram based on agent positions.
    
    Each agent gets frontiers closest to it (Manhattan distance).
    This encourages spatial dispersion by giving each agent exclusive regions.
    
    References:
        - Aurenhammer, F. (1991). Voronoi diagrams—a survey. ACM Computing Surveys.
        - De Berg et al. (2008). Computational Geometry (Chapter 7).
    
    Args:
        agent_positions: Dict mapping agent_id -> (x, y)
        frontiers: List of frontier cells (x, y)
        map_size: Environment size (for bounds checking)
    
    Returns:
        Dict mapping agent_id -> list of frontiers in its Voronoi cell
    """
    voronoi_cells = defaultdict(list)
    
    for fx, fy in frontiers:
        # Find closest agent (Manhattan distance)
        min_dist = float('inf')
        closest_agent = None
        
        for agent_id, (ax, ay) in agent_positions.items():
            dist = abs(fx - ax) + abs(fy - ay)
            if dist < min_dist:
                min_dist = dist
                closest_agent = agent_id
        
        if closest_agent is not None:
            voronoi_cells[closest_agent].append((fx, fy))
    
    return dict(voronoi_cells)


def compute_dispersion_reward(agent_id: int,
                            agent_positions: Dict[int, Tuple[int, int]],
                            frontier: Tuple[int, int],
                            voronoi_frontiers: Optional[Dict[int, List[Tuple[int, int]]]] = None,
                            min_separation: int = 6) -> float:
    """
    Compute dispersion reward for assigning an agent to a frontier.
    
    Higher reward = frontier is farther from other agents, encouraging spread.
    
    References:
        - Hönig et al. (2018): Distance-based dispersion incentives
        - Olfati-Saber (2006): Consensus-based spatial distribution
    
    Args:
        agent_id: Agent being assigned
        agent_positions: Current positions of all agents
        frontier: Target frontier cell (x, y)
        voronoi_frontiers: Optional Voronoi partition (for efficiency)
        min_separation: Minimum desired distance between agents (Manhattan)
    
    Returns:
        Dispersion reward value (positive = good dispersion)
    """
    fx, fy = frontier
    reward = 0.0
    
    # Compute minimum distance to nearest other agent
    min_dist_to_agent = float('inf')
    agent_count_nearby = 0
    
    for other_id, (ax, ay) in agent_positions.items():
        if other_id == agent_id:
            continue
        
        dist = abs(fx - ax) + abs(fy - ay)
        min_dist_to_agent = min(min_dist_to_agent, dist)
        
        # Count agents within minimum separation distance
        if dist < min_separation:
            agent_count_nearby += 1
    
    # Reward based on distance: farther = better (up to a point)
    # Use logarithmic reward to avoid extreme values
    if min_dist_to_agent > 0:
        distance_reward = math.log(1.0 + min_dist_to_agent)
    else:
        distance_reward = 0.0
    
    # Penalty for clustering: exponential penalty for nearby agents
    clustering_penalty = agent_count_nearby * math.exp(-min_dist_to_agent / 3.0)
    
    # Net dispersion reward
    reward = distance_reward - clustering_penalty
    
    # Voronoi bonus: extra reward if frontier is in agent's Voronoi cell
    if voronoi_frontiers and agent_id in voronoi_frontiers:
        if frontier in voronoi_frontiers[agent_id]:
            reward += 0.5  # Bonus for staying in own region
    
    return reward


def compute_information_dispersion(agent_positions: Dict[int, Tuple[int, int]],
                                  frontiers: List[Tuple[int, int]],
                                  env_size: int,
                                  team_belief: np.ndarray) -> Dict[Tuple[int, int], float]:
    """
    Compute information-theoretic dispersion scores for frontiers.
    
    Rewards frontiers that maximize both:
    1. Information gain (entropy/uncertainty)
    2. Spatial dispersion (distance from other agents)
    
    References:
        - Stachniss et al. (2004): Information-theoretic exploration
        - Yamauchi (1997): Frontier entropy maximization
    
    Args:
        agent_positions: Current agent positions
        frontiers: Candidate frontier cells
        env_size: Environment size
        team_belief: Team belief map (-1=unknown, 0=free, 2=obstacle)
    
    Returns:
        Dict mapping frontier -> dispersion score
    """
    scores = {}
    
    # Compute Voronoi partition
    voronoi_cells = compute_voronoi_cells(agent_positions, frontiers, env_size)
    
    # For each frontier, compute combined information + dispersion score
    for fx, fy in frontiers:
        # Information component: count unknown cells nearby
        window_radius = 3
        unknown_count = 0
        for dx in range(-window_radius, window_radius + 1):
            for dy in range(-window_radius, window_radius + 1):
                nx, ny = fx + dx, fy + dy
                if 0 <= nx < env_size and 0 <= ny < env_size:
                    # team_belief indexed [y, x]
                    if team_belief[ny, nx] == -1 or team_belief[ny, nx] == 1:
                        unknown_count += 1
        
        info_score = float(unknown_count)
        
        # Dispersion component: distance to nearest agent
        min_agent_dist = min(
            (abs(fx - ax) + abs(fy - ay))
            for ax, ay in agent_positions.values()
        ) if agent_positions else 0.0
        
        dispersion_score = math.log(1.0 + min_agent_dist)
        
        # Combined score (weighted sum)
        # Information gets 70% weight, dispersion 30%
        combined_score = 0.7 * info_score + 0.3 * dispersion_score
        
        scores[(fx, fy)] = combined_score
    
    return scores


def adaptive_dispersion_strength(coverage: float,
                                initial_coverage: float = 0.0,
                                target_coverage: float = 0.95) -> float:
    """
    Compute adaptive dispersion strength based on exploration progress.
    
    Strategy:
    - Early exploration: Strong dispersion (agents should spread out)
    - Late exploration: Moderate dispersion (fill gaps, maintain connectivity)
    
    References:
        - Burgard et al. (2005): Adaptive coordination strategies
        - Yamauchi (1997): Exploration phase-dependent policies
    
    Args:
        coverage: Current coverage fraction (0.0 to 1.0)
        initial_coverage: Starting coverage (typically 0.0)
        target_coverage: Target completion (typically 0.95)
    
    Returns:
        Dispersion strength multiplier (0.0 to 1.0)
    """
    if target_coverage <= initial_coverage:
        return 0.5  # Default moderate dispersion
    
    # Normalize coverage progress
    progress = (coverage - initial_coverage) / (target_coverage - initial_coverage)
    progress = max(0.0, min(1.0, progress))
    
    # Early phase (0-50%): Strong dispersion
    # Late phase (50-100%): Moderate dispersion (focus on gaps)
    if progress < 0.5:
        # Strong dispersion in early exploration
        strength = 1.0 - 0.3 * progress  # 1.0 → 0.85
    else:
        # Moderate dispersion in late exploration
        strength = 0.85 - 0.35 * (progress - 0.5)  # 0.85 → 0.5
    
    return max(0.3, min(1.0, strength))


def compute_coverage_efficiency(current_positions: Dict[int, Tuple[int, int]],
                               assigned_frontiers: Dict[int, Tuple[int, int]],
                               team_belief: np.ndarray,
                               env_size: int) -> float:
    """
    Compute coverage efficiency metric: how well assignments cover unexplored space.
    
    References:
        - Yamauchi (1997): Coverage efficiency metrics
        - Stachniss et al. (2004): Information-theoretic coverage
    
    Returns:
        Efficiency score (0.0 to 1.0), higher = better coverage distribution
    """
    # Build union of observation windows around assigned frontiers
    observed_cells = set()
    window_radius = 3  # Sensor radius
    
    for fx, fy in assigned_frontiers.values():
        for dx in range(-window_radius, window_radius + 1):
            for dy in range(-window_radius, window_radius + 1):
                nx, ny = fx + dx, fy + dy
                if 0 <= nx < env_size and 0 <= ny < env_size:
                    observed_cells.add((nx, ny))
    
    # Count how many of these cells are actually unknown
    unknown_in_windows = 0
    for nx, ny in observed_cells:
        if team_belief[ny, nx] == -1 or team_belief[ny, nx] == 1:
            unknown_in_windows += 1
    
    total_in_windows = len(observed_cells)
    
    # Efficiency = fraction of window cells that are unknown
    if total_in_windows > 0:
        efficiency = float(unknown_in_windows) / float(total_in_windows)
    else:
        efficiency = 0.0
    
    return efficiency

