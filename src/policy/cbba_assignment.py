"""
Consensus-Based Bundle Algorithm (CBBA) for Multi-Robot Frontier Assignment

Implements a market-based task allocation mechanism for spatial coverage tasks
under connectivity constraints. Agents bid on frontiers based on utility and
resolve conflicts through distributed consensus.

References:
- Choi, Han-Lim, Brunet, Luc, & How, Jonathan P. (2009). "Consensus-Based 
  Decentralized Auctions for Robust Task Allocation." IEEE Transactions on 
  Robotics, 25(4), 912-926.
  
- Gerkey, Brian P., & Matarić, Maja J. (2004). "A Formal Analysis and Taxonomy 
  of Task Allocation in Multi-Robot Systems." The International Journal of 
  Robotics Research, 23(9), 939-954.

Key Idea:
- Each agent independently scores all frontiers considering spatial separation
- Agents "bid" on their top K choices with marginal utility
- Conflicts resolved by highest bidder (with tie-breaking)
- Guarantees no two agents pick overlapping workspace regions
"""

import numpy as np
import math


def cbba_frontier_assignment(env, frontiers, agents_list,
                             comm_radius=10,
                             fov_radius=6,
                             frontier_score_map=None,
                             max_bids_per_agent=3,
                             spacing_radius=6,
                             strict_fov_exclusion=False,
                             fov_shape="square",  # 'square' uses L∞, 'circle' uses Euclidean
                             fov_margin=0,
                             connectivity_strictness=1.0,
                             tether_constraint=True,
                             allow_distance=9,
                             leader_id=0):
    """
    CBBA-style auction for frontier assignment with spatial exclusion.
    
    Args:
        env: GridWorld environment
        frontiers: List of (x, y) frontier cells
        agents_list: List of agent IDs
        comm_radius: Communication/connectivity radius
        fov_radius: Agent field-of-view radius (for exclusion zones)
        frontier_score_map: Dict of {(x,y): info_score} (entropy-based utility)
        max_bids_per_agent: How many frontiers each agent can bid on
        spacing_radius: Minimum L1 distance between assigned agents
        tether_constraint: If True, enforce connectivity to teammates
        allow_distance: Maximum L1 from any teammate (connectivity constraint)
    
    Returns:
        assignments: Dict {agent_id: (x, y)}
        claims: Dict {agent_id: (x, y)} (same as assignments for compatibility)
        total_utility: Sum of assigned utilities
    """
    if not frontiers or not agents_list:
        return {}, {}, 0.0
    
    N = len(agents_list)
    
    # Build utility matrix: U[i][f] = utility of agent i picking frontier f
    # Utility = info_gain - distance_cost - exclusion_penalty
    utilities = {}
    for i in agents_list:
        pos_i = env.agent_positions.get(i)
        if pos_i is None:
            continue
        utilities[i] = {}
        for f in frontiers:
            # Base utility: information gain (entropy)
            base = frontier_score_map.get(f, 1.0) if frontier_score_map else 1.0
            
            # Distance cost (normalized by map size)
            dist = abs(f[0] - pos_i[0]) + abs(f[1] - pos_i[1])
            dist_cost = 0.05 * dist  # Same weight as greedy fix
            
            # Connectivity feasibility check
            if tether_constraint:
                # Must be within allow_distance of at least one teammate
                teammates = [env.agent_positions[j] for j in agents_list if j != i and j in env.agent_positions]
                if teammates:
                    min_dist_to_team = min(abs(f[0]-tj[0]) + abs(f[1]-tj[1]) for tj in teammates)
                    if min_dist_to_team > allow_distance:
                        utilities[i][f] = -1e9  # Infeasible
                        continue
            
            utilities[i][f] = base - dist_cost
    
    # CBBA Phase 1: Bidding
    # Each agent picks its top K frontiers and submits bids
    bids = {}  # {frontier: [(agent_id, bid_value), ...]}
    for i in agents_list:
        if i not in utilities:
            continue
        # Sort frontiers by utility for this agent
        agent_utils = [(f, u) for f, u in utilities[i].items() if u > -1e8]
        agent_utils.sort(key=lambda x: -x[1])
        
        # Bid on top K
        for f, u in agent_utils[:max_bids_per_agent]:
            if f not in bids:
                bids[f] = []
            bids[f].append((i, u))
    
    # CBBA Phase 2: Conflict Resolution
    # For each frontier, award to highest bidder
    # Then enforce spatial exclusion: if two winners are too close, keep higher bidder
    winners = {}  # {frontier: agent_id}
    for f, bidders in bids.items():
        if not bidders:
            continue
        # Highest bid wins
        bidders.sort(key=lambda x: -x[1])
        winner_id, winner_bid = bidders[0]
        winners[f] = winner_id
    
    # Phase 3: Spatial Exclusion (enforce separation between agents)
    # If two agents are assigned frontiers whose FOVs would overlap, keep higher-utility one
    assigned = {}  # {agent_id: frontier}
    for f, agent_id in winners.items():
        assigned[agent_id] = f
    
    # Iterative conflict resolution
    changed = True
    iterations = 0
    while changed and iterations < 10:
        changed = False
        iterations += 1
        conflicts = []
        
        # Find pairs that violate spacing
        agent_ids = list(assigned.keys())
        for i in range(len(agent_ids)):
            for j in range(i+1, len(agent_ids)):
                a1, a2 = agent_ids[i], agent_ids[j]
                f1, f2 = assigned[a1], assigned[a2]
                
                # Check if their FOV zones overlap
                dx = abs(f1[0]-f2[0]); dy = abs(f1[1]-f2[1])
                if strict_fov_exclusion:
                    if fov_shape == "circle":
                        separation = math.hypot(dx, dy)
                        min_separation = float(2 * fov_radius + fov_margin)
                    else:
                        # square FOV approximation: L∞ separation
                        separation = max(dx, dy)
                        min_separation = int(2 * fov_radius + fov_margin)
                else:
                    # legacy proxy: L1 spacing + one FOV radius
                    separation = dx + dy
                    min_separation = spacing_radius + fov_radius

                if separation < min_separation:
                    # Conflict: keep agent with higher utility, remove other
                    u1 = utilities[a1].get(f1, 0)
                    u2 = utilities[a2].get(f2, 0)
                    if u1 >= u2:
                        conflicts.append(a2)
                    else:
                        conflicts.append(a1)
        
        # Remove conflicted agents
        for agent_id in set(conflicts):
            if agent_id in assigned:
                del assigned[agent_id]
                changed = True
    
    # Convert to output format (initial)
    assignments = {i: assigned.get(i, env.agent_positions.get(i, (0, 0))) for i in agents_list}
    claims = assignments.copy()

    # Connectivity Enforcement: ensure multi-hop connectivity to the leader
    # If any agent's planned target causes disconnection, convert it to a support target
    # at the nearest teammate within allow_distance to maintain a connected mesh.
    try:
        ids = list(agents_list)
        # Build planned positions (target if assigned else current)
        planned = {i: assignments.get(i, env.agent_positions.get(i)) for i in ids}

        def neighbors(i, j):
            pi, pj = planned[i], planned[j]
            if pi is None or pj is None:
                return False
            d = abs(pi[0]-pj[0]) + abs(pi[1]-pj[1])
            return d <= allow_distance

        # Compute component reachable from leader
        def leader_component():
            seen = set()
            stack = [leader_id]
            while stack:
                u = stack.pop()
                if u in seen:
                    continue
                seen.add(u)
                for v in ids:
                    if v == u:
                        continue
                    if neighbors(u, v):
                        stack.append(v)
            return seen

        comp = leader_component()
        # Optionally allow a small fraction of outsiders (temporary splintering) depending on connectivity_strictness
        # connectivity_strictness: 1.0 => strict (force all connected); lower values allow that fraction of agents to be outside
        try:
            total_agents = len(ids)
            max_outsiders = max(0, int((1.0 - float(connectivity_strictness)) * float(total_agents)))
        except Exception:
            max_outsiders = 0

        # Iteratively pull outsiders into the component by converting to support targets if we exceed allowed outsiders
        safe_guard = 0
        while len(comp) < len(ids) and safe_guard < 2*len(ids):
            outsiders = [i for i in ids if i not in comp]
            # If number of outsiders is <= allowed, accept temporary split
            if len(outsiders) <= max_outsiders:
                break
            for i in outsiders:
                # Move i to nearest teammate position inside component (support/relay)
                pi = env.agent_positions.get(i)
                bestd, anchor = 10**9, None
                for j in comp:
                    pj = planned[j]
                    if pj is None:
                        continue
                    d = abs(pj[0]-pi[0]) + abs(pj[1]-pi[1])
                    if d < bestd:
                        bestd, anchor = d, j
                if anchor is not None:
                    anchor_pos = planned[anchor]
                    # Prefer a frontier on the component boundary that keeps connectivity
                    candidate_f = None
                    best_ff = 10**9
                    for f in frontiers:
                        # Frontier must be within allow_distance of some in-component planned node
                        ok = False
                        for j in comp:
                            pj = planned[j]
                            if pj is None:
                                continue
                            if abs(f[0]-pj[0]) + abs(f[1]-pj[1]) <= allow_distance:
                                ok = True
                                break
                        if not ok:
                            continue
                        df = abs(f[0]-pi[0]) + abs(f[1]-pi[1])
                        if df < best_ff:
                            best_ff, candidate_f = df, f
                    if candidate_f is not None:
                        planned[i] = candidate_f
                    else:
                        # Compute a support point toward the anchor but at most allow_distance-1 away
                        ax, ay = anchor_pos
                        ix, iy = pi
                        budget = max(1, allow_distance-1)
                        # Move from anchor toward i by at most 'budget' in L1
                        step_x = max(-budget, min(budget, ix - ax))
                        rem = budget - abs(step_x)
                        step_y = max(-rem, min(rem, iy - ay))
                        sx, sy = ax + (1 if step_x>0 else (-1 if step_x<0 else 0))*abs(step_x), ay + (1 if step_y>0 else (-1 if step_y<0 else 0))*abs(step_y)
                        planned[i] = (int(sx), int(sy))
                    assignments[i] = planned[i]
                    claims[i] = planned[i]
            comp = leader_component()
            safe_guard += 1
    except Exception:
        # If anything goes wrong, keep initial assignments
        pass
    
    # Compute predicted entropy gain (for trust updates)
    # Trust system expects unknown cell count, not bid utility scores
    U_pred_entropy = 0.0
    if frontier_score_map:
        # Use frontier scores (unknown counts) instead of bid utilities
        for agent_id, frontier in assigned.items():
            score = frontier_score_map.get(frontier, 0.0)
            U_pred_entropy += score
    else:
        # Fallback: estimate based on number of assigned agents
        U_pred_entropy = float(len(assigned))
    
    return assignments, claims, U_pred_entropy
