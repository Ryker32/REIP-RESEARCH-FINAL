"""
Improved frontier assignment strategies that handle isolated regions better.

Key improvements over standard entropy_global:

1. **Voronoi Partitioning**: Divide map into territories, one per agent
   - Prevents agents from clustering in same region
   - Ensures spatial coverage across entire map

2. **Directional Exploration**: Agents explore systematically outward
   - Outward bias: Prefer frontiers farther from starting position
   - Advancement bias: Push toward boundary of unexplored regions  
   - Momentum: Continue in same direction to avoid oscillation

3. **Multi-objective Scoring**: Balance multiple exploration goals
   - Information gain (entropy): Prefer high-uncertainty frontiers
   - Travel efficiency: Don't waste time on distant targets
   - Congestion avoidance: Don't assign multiple agents to same frontier
   - Frontier progression: Advance the exploration boundary systematically

This prevents the "random wandering" problem where agents oscillate
back-and-forth in already-explored areas with only slight advancement.

References:
- Shannon, Claude (1948): A Mathematical Theory of Communication
  Used for information-theoretic frontier evaluation via entropy H(p) = -Σ p log(p)
- Yamauchi (1997): A Frontier-Based Approach for Autonomous Exploration
  Original frontier-based exploration concept
- Burgard et al. (2000): Collaborative Multi-Robot Exploration
  Multi-agent coordination for exploration tasks
- Aurenhammer (1991): Voronoi Diagrams - A Survey of a Fundamental Geometric Data Structure
  Theoretical foundation for spatial partitioning
"""

import numpy as np
import math
from collections import Counter, defaultdict


def cluster_frontiers_by_proximity(frontiers, radius=5):
    """
    Group frontiers into connected regions using proximity clustering.
    
    Args:
        frontiers: List of (x, y) frontier cell positions
        radius: Maximum Manhattan distance to consider cells "connected"
    
    Returns:
        List of lists, where each sublist is a cluster of nearby frontiers
    """
    if not frontiers:
        return []
    
    # Build adjacency based on proximity
    n = len(frontiers)
    adjacent = [set() for _ in range(n)]
    
    for i in range(n):
        for j in range(i+1, n):
            fx1, fy1 = frontiers[i]
            fx2, fy2 = frontiers[j]
            dist = abs(fx1 - fx2) + abs(fy1 - fy2)
            if dist <= radius:
                adjacent[i].add(j)
                adjacent[j].add(i)
    
    # Find connected components using DFS
    visited = [False] * n
    clusters = []
    
    for start in range(n):
        if visited[start]:
            continue
        
        # DFS to find all connected frontiers
        cluster = []
        stack = [start]
        while stack:
            i = stack.pop()
            if visited[i]:
                continue
            visited[i] = True
            cluster.append(frontiers[i])
            
            for j in adjacent[i]:
                if not visited[j]:
                    stack.append(j)
        
        if cluster:
            clusters.append(cluster)
    
    # Sort clusters by size (largest first for load balancing)
    clusters.sort(key=lambda c: len(c), reverse=True)
    
    return clusters


def assign_agents_to_regions_voronoi(env, clusters):
    """
    Assign agents to frontier regions using Voronoi partitioning.
    
    Each cluster is assigned to the nearest agent (Voronoi cell).
    This ensures spatial distribution and avoids clustering.
    
    Args:
        env: GridWorld environment
        clusters: List of frontier clusters from cluster_frontiers_by_proximity
    
    Returns:
        Dict {agent_id: region_index}, mapping each agent to a cluster
    """
    if not clusters:
        return {}
    
    # Compute centroid of each cluster
    cluster_centroids = []
    for cluster in clusters:
        cx = sum(f[0] for f in cluster) / len(cluster)
        cy = sum(f[1] for f in cluster) / len(cluster)
        cluster_centroids.append((cx, cy))
    
    # Voronoi assignment: each cluster goes to nearest agent
    region_assignments = defaultdict(list)  # agent_id -> [list of region indices]
    
    for region_idx, (cx, cy) in enumerate(cluster_centroids):
        # Find nearest agent to this cluster centroid
        best_agent = None
        best_dist = math.inf
        
        for agent_id, agent_pos in env.agent_positions.items():
            dist = abs(cx - agent_pos[0]) + abs(cy - agent_pos[1])
            if dist < best_dist:
                best_dist = dist
                best_agent = agent_id
        
        if best_agent is not None:
            region_assignments[best_agent].append(region_idx)
    
    # Convert to agent -> single region (pick largest cluster for each agent)
    final_assignments = {}
    for agent_id, region_list in region_assignments.items():
        # Assign agent to their largest cluster
        best_region = max(region_list, key=lambda r: len(clusters[r]))
        final_assignments[agent_id] = best_region
    
    return final_assignments


def assign_agents_to_regions(env, clusters, prev_region_assignments=None):
    """
    Assign agents to frontier regions (clusters) using load balancing.
    
    Strategy:
    1. Agents stay in their current region if it still has frontiers (persistence)
    2. Unassigned agents go to nearest large cluster
    3. Balance load: avoid too many agents in one region
    
    Args:
        env: GridWorld environment
        clusters: List of frontier clusters from cluster_frontiers_by_proximity
        prev_region_assignments: Dict {agent_id: region_index} from previous step
    
    Returns:
        Dict {agent_id: region_index}, mapping each agent to a cluster
    """
    if not clusters:
        return {}
    
    n_agents = len(env.agents)
    n_regions = len(clusters)
    
    # Track which agents are assigned to which regions
    region_assignments = {}
    region_loads = defaultdict(int)  # How many agents assigned to each region
    
    # Step 1: Persistence - agents stick to their current region if valid
    if prev_region_assignments:
        for agent_id, prev_region in prev_region_assignments.items():
            if 0 <= prev_region < n_regions:
                region_assignments[agent_id] = prev_region
                region_loads[prev_region] += 1
    
    # Step 2: Assign unassigned agents to nearest region with load balancing
    unassigned = [i for i in env.agents.keys() if i not in region_assignments]
    
    for agent_id in unassigned:
        agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
        
        # Find nearest region with consideration for load
        best_region = None
        best_score = math.inf
        
        for region_idx, cluster in enumerate(clusters):
            # Compute distance to nearest frontier in this cluster
            min_dist = min(abs(f[0] - agent_pos[0]) + abs(f[1] - agent_pos[1]) 
                          for f in cluster)
            
            # Load penalty: prefer regions with fewer agents
            current_load = region_loads[region_idx]
            ideal_load = n_agents / n_regions
            load_penalty = max(0, current_load - ideal_load) * 10  # Tunable weight
            
            score = min_dist + load_penalty
            
            if score < best_score:
                best_score = score
                best_region = region_idx
        
        if best_region is not None:
            region_assignments[agent_id] = best_region
            region_loads[best_region] += 1
    
    return region_assignments


def assign_frontiers_voronoi(env, frontiers, lam=3.0, prev_assigns=None, 
                            cluster_radius=8, prev_regions=None):
    """
    Frontier assignment using Voronoi partitioning with directional exploration.
    
    Strategy:
    1. Cluster frontiers into connected regions
    2. Assign regions to agents using Voronoi cells (nearest agent to cluster centroid)
    3. Within each agent's region, prioritize frontiers that:
       - Are FARTHEST from agent's starting position (push outward)
       - Have high information gain (entropy)
       - Are on the BOUNDARY of explored/unexplored (advance the frontier)
    
    This ensures spatial distribution AND systematic outward expansion.
    
    Args:
        env: GridWorld environment
        frontiers: List of frontier cells
        lam: Lambda parameter for congestion penalty
        prev_assigns: Previous frontier assignments
        cluster_radius: Manhattan distance threshold for clustering
        prev_regions: Dict {agent_id: region_idx} from previous step (unused for Voronoi)
    
    Returns:
        assignments: Dict {agent_id: (x, y) frontier position}
        leader_claims: Dict (for compatibility)
        U_pred: Predicted utility (number of new cells)
        region_assignments: Dict {agent_id: region_idx} (return for next step)
    """
    if not frontiers:
        return {}, {}, 0, {}
    
    # Step 1: Cluster frontiers into regions
    clusters = cluster_frontiers_by_proximity(frontiers, radius=cluster_radius)
    
    # Step 2: Voronoi partitioning - assign regions to nearest agents
    region_assignments = assign_agents_to_regions_voronoi(env, clusters)
    
    # Step 3: Within-region frontier assignment with DIRECTIONAL BIAS
    assignments = {}
    leader_claims = {}
    U_pred = 0
    
    # Get agent belief maps for entropy calculation
    size = env.size
    ds = next(iter(env.agents.values())).ds
    gx = (size + ds - 1) // ds
    gy = gx
    
    # Build team probability map (for entropy)
    team_prob = [[None for _ in range(gy)] for __ in range(gx)]
    for a in env.agents.values():
        if not hasattr(a, 'belief_lr'):
            continue
        lr = a.belief_lr
        for ix in range(lr.shape[0]):
            for iy in range(lr.shape[1]):
                val = lr[ix, iy]
                if val == -1:
                    continue
                p = 1.0 if val == 2 else 0.0
                if team_prob[ix][iy] is None:
                    team_prob[ix][iy] = p
                else:
                    team_prob[ix][iy] = 0.5 * (team_prob[ix][iy] + p)
    
    def shannon(p):
        if p is None:
            p = 0.5
        if p <= 0.0 or p >= 1.0:
            return 0.0
        return - (p * math.log(p + 1e-12) + (1 - p) * math.log(1 - p + 1e-12))
    
    entropy_map = [[shannon(team_prob[ix][iy]) if team_prob[ix][iy] is not None 
                    else shannon(0.5) for iy in range(gy)] for ix in range(gx)]
    
    # Compute explored region centroid (center of mass of explored area)
    explored_cells = []
    for x in range(size):
        for y in range(size):
            if env.team_belief[x, y] == 0:  # explored
                explored_cells.append((x, y))
    
    if explored_cells:
        explored_cx = sum(x for x, y in explored_cells) / len(explored_cells)
        explored_cy = sum(y for x, y in explored_cells) / len(explored_cells)
    else:
        explored_cx = size // 2
        explored_cy = size // 2
    
    # Assign frontiers to agents based on their Voronoi regions
    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()
    
    for agent_id in env.agents.keys():
        assigned_region = region_assignments.get(agent_id)
        
        if assigned_region is None or assigned_region >= len(clusters):
            # Agent has no assigned region, assign to nearest frontier
            if frontiers:
                agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
                nearest = min(frontiers, key=lambda f: abs(f[0] - agent_pos[0]) + abs(f[1] - agent_pos[1]))
                assignments[agent_id] = nearest
                leader_claims[agent_id] = nearest
                U_pred += 1
            continue
        
        # Get frontiers in this agent's region
        cluster = clusters[assigned_region]
        
        if not cluster:
            continue
        
        agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
        
        # Store agent's starting position for outward bias
        agent_obj = env.agents[agent_id]
        if not hasattr(agent_obj, 'starting_pos'):
            agent_obj.starting_pos = agent_pos
        start_pos = agent_obj.starting_pos
        
        # Score each frontier in this region with DIRECTIONAL BIAS
        best_frontier = None
        best_score = -math.inf  # MAXIMIZE score now (higher = better)
        
        for f in cluster:
            fx, fy = f
            cx, cy = fx // ds, fy // ds
            
            # 1. OUTWARD BIAS: Prefer frontiers FARTHER from starting position
            # This pushes agents to explore outward, not oscillate
            dist_from_start = abs(fx - start_pos[0]) + abs(fy - start_pos[1])
            outward_bonus = dist_from_start * 0.3  # Moderate push outward
            
            # 2. FRONTIER ADVANCEMENT: Prefer frontiers far from explored center
            # This pushes toward the boundary of unexplored regions
            dist_from_explored_center = abs(fx - explored_cx) + abs(fy - explored_cy)
            advancement_bonus = dist_from_explored_center * 0.2  # Gentle push to boundary
            
            # 3. Distance cost from CURRENT position (for travel efficiency)
            # This is the PRIMARY driver - don't waste time traveling
            dist_from_current = abs(fx - agent_pos[0]) + abs(fy - agent_pos[1])
            travel_penalty = -dist_from_current * 1.0  # Strong preference for nearby targets
            
            # 4. Entropy bonus (information gain)
            if 0 <= cx < gx and 0 <= cy < gy:
                entropy_bonus = entropy_map[cx][cy] * 20  # High weight on information
            else:
                entropy_bonus = 0
            
            # 5. Congestion penalty
            congestion_penalty = -lam * counts[f] * 2  # Strong avoidance of crowded frontiers
            
            # 6. MOMENTUM: Prefer continuing in same direction as previous move
            momentum_bonus = 0
            if prev_assigns and agent_id in prev_assigns:
                prev_target = prev_assigns[agent_id]
                # Direction vector from previous target to this frontier
                dx_prev = fx - prev_target[0]
                dy_prev = fy - prev_target[1]
                # Direction vector from agent to this frontier
                dx_curr = fx - agent_pos[0]
                dy_curr = fy - agent_pos[1]
                # Dot product measures alignment (positive = same direction)
                if (dx_prev != 0 or dy_prev != 0) and (dx_curr != 0 or dy_curr != 0):
                    alignment = (dx_prev * dx_curr + dy_prev * dy_curr)
                    momentum_bonus = max(0, alignment) * 0.5  # Strong momentum to prevent zig-zagging
            
            # TOTAL SCORE (higher = better)
            score = (outward_bonus + advancement_bonus + travel_penalty + 
                    entropy_bonus + congestion_penalty + momentum_bonus)
            
            if score > best_score:
                best_score = score
                best_frontier = f
        
        if best_frontier is not None:
            assignments[agent_id] = best_frontier
            leader_claims[agent_id] = best_frontier
            counts[best_frontier] += 1
            U_pred += 1
    
    # Handle any agents not assigned yet
    unassigned = [i for i in env.agents.keys() if i not in assignments]
    for agent_id in unassigned:
        if frontiers:
            agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
            nearest = min(frontiers, key=lambda f: abs(f[0] - agent_pos[0]) + abs(f[1] - agent_pos[1]))
            assignments[agent_id] = nearest
            leader_claims[agent_id] = nearest
            U_pred += 1
    
    return assignments, leader_claims, U_pred, region_assignments


def assign_frontiers_clustered(env, frontiers, lam=3.0, prev_assigns=None, 
                               cluster_radius=8, prev_regions=None):
    """
    Improved frontier assignment with region clustering.
    
    Strategy:
    1. Cluster frontiers into connected regions
    2. Assign agents to regions (load balanced)
    3. Within each region, use entropy-based scoring
    4. Persist: Agents stick to their region across timesteps
    
    Args:
        env: GridWorld environment
        frontiers: List of frontier cells
        lam: Lambda parameter for congestion penalty
        prev_assigns: Previous frontier assignments
        cluster_radius: Manhattan distance threshold for clustering
        prev_regions: Dict {agent_id: region_idx} from previous step
    
    Returns:
        assignments: Dict {agent_id: (x, y) frontier position}
        leader_claims: Dict (for compatibility)
        U_pred: Predicted utility (number of new cells)
        region_assignments: Dict {agent_id: region_idx} (return for next step)
    """
    if not frontiers:
        return {}, {}, 0, {}
    
    # Step 1: Cluster frontiers into regions
    clusters = cluster_frontiers_by_proximity(frontiers, radius=cluster_radius)
    
    # Step 2: Assign agents to regions
    region_assignments = assign_agents_to_regions(env, clusters, prev_regions)
    
    # Step 3: Within-region frontier assignment
    assignments = {}
    leader_claims = {}
    U_pred = 0
    
    # Get agent belief maps for entropy calculation
    size = env.size
    ds = next(iter(env.agents.values())).ds
    gx = (size + ds - 1) // ds
    gy = gx
    
    # Build team probability map (for entropy)
    team_prob = [[None for _ in range(gy)] for __ in range(gx)]
    for a in env.agents.values():
        if not hasattr(a, 'belief_lr'):
            continue
        lr = a.belief_lr
        for ix in range(lr.shape[0]):
            for iy in range(lr.shape[1]):
                val = lr[ix, iy]
                if val == -1:
                    continue
                p = 1.0 if val == 2 else 0.0
                if team_prob[ix][iy] is None:
                    team_prob[ix][iy] = p
                else:
                    team_prob[ix][iy] = 0.5 * (team_prob[ix][iy] + p)
    
    def shannon(p):
        if p is None:
            p = 0.5
        if p <= 0.0 or p >= 1.0:
            return 0.0
        return - (p * math.log(p + 1e-12) + (1 - p) * math.log(1 - p + 1e-12))
    
    entropy_map = [[shannon(team_prob[ix][iy]) if team_prob[ix][iy] is not None 
                    else shannon(0.5) for iy in range(gy)] for ix in range(gx)]
    
    # Assign within each region
    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()
    
    for region_idx, cluster in enumerate(clusters):
        # Get agents assigned to this region
        agents_in_region = [a_id for a_id, r_idx in region_assignments.items() 
                           if r_idx == region_idx]
        
        if not agents_in_region:
            continue
        
        # Greedy assignment within this region
        available_frontiers = cluster.copy()
        
        for agent_id in agents_in_region:
            if not available_frontiers:
                # Fallback: assign to any remaining frontier
                if frontiers:
                    available_frontiers = frontiers.copy()
                else:
                    break
            
            agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
            
            # Score each available frontier
            best_frontier = None
            best_score = math.inf
            
            for f in available_frontiers:
                fx, fy = f
                cx, cy = fx // ds, fy // ds
                
                # Distance cost
                dist = abs(fx - agent_pos[0]) + abs(fy - agent_pos[1])
                
                # Entropy bonus (higher entropy = more information gain)
                if 0 <= cx < gx and 0 <= cy < gy:
                    entropy_bonus = -entropy_map[cx][cy] * 5  # Negative = prefer high entropy
                else:
                    entropy_bonus = 0
                
                # Congestion penalty
                congestion = lam * counts[f]
                
                score = dist + entropy_bonus + congestion
                
                if score < best_score:
                    best_score = score
                    best_frontier = f
            
            if best_frontier is not None:
                assignments[agent_id] = best_frontier
                leader_claims[agent_id] = best_frontier
                counts[best_frontier] += 1
                available_frontiers.remove(best_frontier)
                U_pred += 1
    
    # Handle any agents not assigned yet (shouldn't happen, but safety)
    unassigned = [i for i in env.agents.keys() if i not in assignments]
    for agent_id in unassigned:
        if frontiers:
            agent_pos = env.agent_positions.get(agent_id, env.agents[agent_id].pos)
            # Just pick nearest frontier
            nearest = min(frontiers, key=lambda f: abs(f[0] - agent_pos[0]) + abs(f[1] - agent_pos[1]))
            assignments[agent_id] = nearest
            leader_claims[agent_id] = nearest
            U_pred += 1
    
    return assignments, leader_claims, U_pred, region_assignments


# Alias for compatibility with existing code
assign_greedy = assign_frontiers_clustered
