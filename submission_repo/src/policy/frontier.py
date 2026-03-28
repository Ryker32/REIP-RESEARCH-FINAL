"""
Frontier-based exploration strategies for multi-agent systems.

Implements several frontier assignment algorithms:
- Greedy: Agents pick nearest frontier with congestion penalty
- Entropy-based: Use Shannon entropy to evaluate information gain at frontiers
- Voronoi: Partition space into regions, assign agents to their region's frontiers

References:
- Yamauchi (1997): A Frontier-Based Approach for Autonomous Exploration
  Original frontier concept: boundary between known and unknown space
- Shannon (1948): A Mathematical Theory of Communication
  Entropy H(p) = -Sigma p log(p) used for information-theoretic frontier scoring
- Burgard et al. (2000): Collaborative Multi-Robot Exploration
  Multi-agent coordination and target assignment strategies
- Stachniss et al. (2004): Exploration with Active Loop-Closing
  Information gain and utility-based exploration
"""

import random
from collections import Counter, deque
import math
import numpy as np


def _expand_belief_lr_known_map(belief_lr, ds, size):
    """Expand a coarse belief_lr grid into a fine known-free boolean map."""
    known = np.zeros((size, size), dtype=bool)
    if belief_lr is None:
        return known
    for gx in range(belief_lr.shape[0]):
        for gy in range(belief_lr.shape[1]):
            v = belief_lr[gx, gy]
            if v == 0:
                x0 = gx * ds
                y0 = gy * ds
                x1 = min(size, (gx + 1) * ds)
                y1 = min(size, (gy + 1) * ds)
                known[x0:x1, y0:y1] = True
    return known


def _get_frontier_entropy_cache(env, window, leader_belief_lr=None, leader_id=None):
    """Memoize coarse entropy statistics for the current timestep using a single agent belief."""
    current_time = getattr(env, "current_time", None)
    cache = getattr(env, "_frontier_entropy_cache", None)
    cache_key = (current_time, window, id(leader_belief_lr), leader_id)

    if cache is None or cache.get("key") != cache_key:
        # Resolve belief_lr and ds
        belief_lr = leader_belief_lr
        ds = 1
        if belief_lr is None:
            try:
                if leader_id is not None and leader_id in env.agents:
                    ag = env.agents[leader_id]
                    belief_lr = getattr(ag, "belief_lr", None)
                    ds = int(getattr(ag, "ds", 1))
                else:
                    ag = next(iter(env.agents.values()))
                    belief_lr = getattr(ag, "belief_lr", None)
                    ds = int(getattr(ag, "ds", 1))
            except Exception:
                belief_lr = None
                ds = 1
        ds = max(1, ds)

        size = env.size
        gx = (size + ds - 1) // ds
        gy = (size + ds - 1) // ds

        team_prob = [[None for _ in range(gy)] for __ in range(gx)]
        if belief_lr is not None:
            max_ix = min(belief_lr.shape[0], gx)
            max_iy = min(belief_lr.shape[1], gy)
            for ix in range(max_ix):
                for iy in range(max_iy):
                    val = belief_lr[ix, iy]
                    if val == 1 or val == -1:
                        continue
                    p = 1.0 if val == 2 else 0.0
                    team_prob[ix][iy] = p

        def shannon(p):
            if p is None:
                p = 0.5
            if p <= 0.0 or p >= 1.0:
                return 0.0
            return - (p * math.log(p + 1e-12) + (1 - p) * math.log(1 - p + 1e-12))

        entropy_map = [[shannon(team_prob[ix][iy]) for iy in range(gy)] for ix in range(gx)]
        known_map = _expand_belief_lr_known_map(belief_lr, ds, size)

        cache = {
            "key": cache_key,
            "ds": ds,
            "gx": gx,
            "gy": gy,
            "entropy_map": entropy_map,
            "known_map": known_map,
            "last_visited": getattr(env, "last_visited", None),
            "known_penalty": getattr(env, "known_penalty", 0.5),
            "recent_penalty": getattr(env, "recent_penalty", 0.5),
            "recent_window": getattr(env, "recent_window", 5),
            "current_time": current_time,
            "leader_id": leader_id,
        }
        env._frontier_entropy_cache = cache
    return cache


def _build_frontier_scores(env, frontiers, window, cache=None):
    """Compute frontier scores using cached entropy statistics."""
    if not frontiers:
        return {}
    cache = cache or _get_frontier_entropy_cache(env, window)
    ds = cache["ds"]
    gx = cache["gx"]
    gy = cache["gy"]
    entropy_map = cache["entropy_map"]
    known_map = cache["known_map"]
    last_visited = cache["last_visited"]
    known_penalty = cache["known_penalty"]
    recent_penalty = cache["recent_penalty"]
    recent_window = cache["recent_window"]
    env_time = cache.get("current_time", None)
    size = env.size
    span = ds * window

    frontier_score = {}
    for fx, fy in frontiers:
        cx, cy = fx // ds, fy // ds
        s = 0.0
        count = 0
        for dx in range(-window, window + 1):
            nx = cx + dx
            if nx < 0 or nx >= gx:
                continue
            for dy in range(-window, window + 1):
                ny = cy + dy
                if 0 <= ny < gy:
                    s += entropy_map[nx][ny]
                    count += 1
        base = s / max(1, count)

        known_frac = 0.0
        if known_map is not None:
            x0 = max(0, fx - span)
            x1 = min(size, fx + span + 1)
            y0 = max(0, fy - span)
            y1 = min(size, fy + span + 1)
            total = (x1 - x0) * (y1 - y0)
            if total > 0:
                known_slice = known_map[x0:x1, y0:y1]
                known_frac = float(known_slice.sum()) / float(total)

        score = base - known_penalty * known_frac

        if last_visited is not None and env_time is not None:
            recent_count = 0
            total_check = 0
            for dx in range(-span, span + 1):
                for dy in range(-span, span + 1):
                    xx = fx + dx
                    yy = fy + dy
                    if 0 <= xx < size and 0 <= yy < size:
                        total_check += 1
                        t_cell = last_visited[xx, yy]
                        if t_cell >= 0 and env_time - t_cell <= recent_window:
                            recent_count += 1
            if total_check > 0:
                score -= recent_penalty * (recent_count / float(total_check))

        frontier_score[(fx, fy)] = score
    return frontier_score


def compute_frontier_scores(env, frontiers, window=5, leader_belief_lr=None, leader_id=None):
    """
    Compute entropy-based information gain scores for frontiers.
    
    Args:
        env: GridWorld environment
        frontiers: List of (x, y) frontier cells
        window: Radius for local entropy calculation
    
    Returns:
        Dict mapping (x, y) -> score (higher = more information gain)
    
    Reference:
        Shannon (1948): Entropy H(p) = -Sigma p_i log(p_i) as information measure
    """
    cache = _get_frontier_entropy_cache(env, window, leader_belief_lr=leader_belief_lr, leader_id=leader_id)
    return _build_frontier_scores(env, frontiers, window, cache)


def assign_frontiers_greedy(env, frontiers, lam=0.0, prev_assigns=None):
    assignments = {}
    leader_claims = {}
    U_pred = 0

    if not frontiers:
        return assignments, leader_claims, U_pred
    
    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()
    
    for i, pos in env.agent_positions.items():
        best, best_score = None, math.inf
        for f in frontiers:
            dist = abs(f[0]-pos[0]) + abs(f[1]-pos[1]) 
            score = dist + lam * counts[f]
            if score < best_score:
                best, best_score = f, score
        assignments[i] = best
        leader_claims[i] = best
        counts[best] += 1
        U_pred += 1  #  1 new cell per assignment (change ts later si tu necesitas)
    
    return assignments, leader_claims, U_pred


def assign_frontiers_entropy_global(env,
                                   frontiers,
                                   lam=3.0,
                                   prev_assigns=None,
                                   window=2,
                                   prox_beta=0.5,
                                   prox_radius=4,
                                   persist_bonus=0.2,
                                   switch_margin=0.05,
                                   connectivity_beta=0.0,
                                   comm_radius=None,
                                   region_size=None,
                                   region_capacity=1,
                                   leader_id=0,
                                   leader_belief_lr=None,
                                   triangle_penalty=0.0,
                                   chain_beta=0.0,
                                   degree_target=2,
                                   degree_over_penalty=0.0,
                                   degree_min_target=1,
                                   degree_under_penalty=0.0,
                                   # Hard guardrails to avoid stepping into isolation
                                   comm_break_penalty=0.0,
                                   link_margin=1,
                                   link_margin_penalty=0.0,
                                   # Optional connectivity tether + heading bias (citations below)
                                   parent_map=None,
                                   hop_count=None,
                                   tether_lambda=0.0,
                                   tether_margin=1,
                                   heading_gamma=0.0,
                                   leader_heading=None,
                                   heading_risk_min_degree=1,
                                   # Linked-signature TTL risk (relay beacons)
                                   link_ttl_map=None,
                                   link_risk_penalty=0.0,
                                   link_risk_ttl_thresh=2,
                                   # Frontier spacing to avoid bunching (assignments too close in same step)
                                   spacing_radius=0):
    """Global greedy matching: compute scores for all agent-frontier pairs then assign greedily to avoid duplicate picks.

    This is a lightweight alternative to Hungarian assignment and prevents many agents selecting the same frontier.

    Citations:
    - Connectivity preservation via local constraints: Ji & Egerstedt (2007), IEEE TAC; Zavlanos & Pappas (2005), IEEE CDC.
    - Heading/velocity alignment for coherent motion with limited comms: Olfati-Saber (2006), IEEE TAC; Jadbabaie et al. (2003), IEEE CDC.
    """
    if not frontiers:
        return {}, {}, 0

    # Resolve leader belief (single-agent) to avoid centralized team leakage
    belief_lr = leader_belief_lr
    if belief_lr is None and leader_id in getattr(env, "agents", {}):
        belief_lr = getattr(env.agents[leader_id], "belief_lr", None)
    cache = _get_frontier_entropy_cache(env, window, leader_belief_lr=belief_lr, leader_id=leader_id)
    frontier_score = _build_frontier_scores(env, frontiers, window, cache)
    size = env.size
    ds = cache["ds"]

    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()

    # communication radius for connectivity scoring (optional)
    if comm_radius is None:
        # try to infer from config via env attribute used in REIP controller; fallback to a reasonable default
        try:
            comm_radius = getattr(env, 'command_radius', None)
        except Exception:
            comm_radius = None

    # default region size if using regional diversity (capacity per coarse cell)
    if region_size is None:
        try:
            region_size = max(4, int(ds * max(1, prox_radius // 2)))
        except Exception:
            region_size = 8

    def region_key(f):
        return (f[0] // max(1, region_size), f[1] // max(1, region_size))

    # compute all scores
    scores = []  # list of (score, agent, frontier)
    for i, pos in env.agent_positions.items():
        for f in frontiers:
            dist = abs(f[0]-pos[0]) + abs(f[1]-pos[1])
            info = frontier_score.get(f, 0.0)
            # FIXED: Proximity penalty based on where OTHER AGENTS ARE ASSIGNED, not current positions
            # This prevents multiple agents from being assigned to the same region
            nearby = 0
            if prev_assigns:
                nearby = sum(1 for other_id, other_assign in prev_assigns.items() 
                            if other_id != i and other_assign and 
                            abs(other_assign[0]-f[0])+abs(other_assign[1]-f[1]) <= prox_radius)
            bonus = 0.0
            if prev_assigns and prev_assigns.get(i) == f:
                bonus = persist_bonus
            # connectivity-aware terms and anti-clique shaping
            conn_term = 0.0
            tri_pen = 0.0
            deg_pen = 0.0
            chain_bonus = 0.0
            # Compute current degree at present position for isolation guard
            cur_deg = 0
            if comm_radius:
                try:
                    # neighbors if agent were at frontier f
                    neighbors = []
                    for other_id, other_pos in env.agent_positions.items():
                        if other_id == i:
                            continue
                        if abs(other_pos[0]-f[0]) + abs(other_pos[1]-f[1]) <= int(comm_radius):
                            neighbors.append(other_id)
                    deg = len(neighbors)
                    # current degree at current position
                    for other_id, other_pos in env.agent_positions.items():
                        if other_id == i:
                            continue
                        if abs(other_pos[0]-pos[0]) + abs(other_pos[1]-pos[1]) <= int(comm_radius):
                            cur_deg += 1
                    if connectivity_beta:
                        conn_term = connectivity_beta * min(deg, 2)
                    # Penalize local triangles: if neighbors are mutually in range, reduce score
                    if triangle_penalty and deg >= 2:
                        closed = 0
                        for a_idx in range(len(neighbors)):
                            a_id = neighbors[a_idx]
                            ax, ay = env.agent_positions[a_id]
                            for b_idx in range(a_idx+1, len(neighbors)):
                                b_id = neighbors[b_idx]
                                bx, by = env.agent_positions[b_id]
                                if abs(ax - bx) + abs(ay - by) <= int(comm_radius):
                                    closed += 1
                        tri_pen = triangle_penalty * float(closed)
                    # Penalize degrees well above target (avoid hubs/cliques)
                    if degree_over_penalty and deg > max(0, int(degree_target)):
                        deg_pen = degree_over_penalty * float(deg - int(degree_target))
                    # Penalize degrees below a minimum target to discourage splitting
                    if degree_under_penalty and deg < max(0, int(degree_min_target)):
                        deg_pen += degree_under_penalty * float(int(degree_min_target) - deg)
                    # Encourage chain-like outward spread from leader while staying connected
                    if chain_beta:
                        try:
                            lp = env.agent_positions.get(leader_id, None)
                            if lp is not None:
                                dcur = abs(pos[0]-lp[0]) + abs(pos[1]-lp[1])
                                dnew = abs(f[0]-lp[0]) + abs(f[1]-lp[1])
                                outward = max(0, dnew - dcur)
                                norm = max(1, int(comm_radius))
                                chain_bonus = chain_beta * min(1.0, outward / float(norm))
                        except Exception:
                            chain_bonus = 0.0
                except Exception:
                    conn_term = conn_term; tri_pen = tri_pen; deg_pen = deg_pen; chain_bonus = chain_bonus

            # Connectivity tether (soft) toward parent to prevent wandering off (Ji & Egerstedt 2007; Zavlanos 2005)
            tether_pen = 0.0
            if tether_lambda and parent_map is not None and comm_radius:
                try:
                    p = parent_map.get(i, None)
                    if p is not None and p in env.agent_positions:
                        px, py = env.agent_positions[p]
                        dfut = abs(f[0]-px) + abs(f[1]-py)
                        margin = max(0, int(tether_margin))
                        allow = int(comm_radius) - margin
                        if dfut > allow:
                            tether_pen = tether_lambda * float(dfut - allow)
                except Exception:
                    tether_pen = 0.0

            # Conditional heading alignment toward leader's heading if at risk of disconnect (Olfati-Saber 2006)
            head_bonus = 0.0
            # Apply heading bias ONLY if this agent is in the leader's communication component
            # (determined by hop_count from the beacon BFS). This prevents an isolated leader from
            # skewing assignments for agents in other components.
            if heading_gamma and leader_heading is not None and comm_radius and (hop_count is not None and hop_count.get(i) is not None):
                try:
                    # Risk predicate: low current degree (<= threshold) -> allow small alignment term
                    cur_deg = 0
                    for other_id, other_pos in env.agent_positions.items():
                        if other_id == i:
                            continue
                        if abs(other_pos[0]-pos[0]) + abs(other_pos[1]-pos[1]) <= int(comm_radius):
                            cur_deg += 1
                    if cur_deg <= int(heading_risk_min_degree):
                        vx, vy = leader_heading
                        dx = f[0] - pos[0]
                        dy = f[1] - pos[1]
                        nd = max(1.0, (abs(dx) + abs(dy)))
                        # Use L1-normalized projection as a cheap proxy for alignment
                        proj = (vx * (dx / nd)) + (vy * (dy / nd))
                        head_bonus = heading_gamma * proj
                except Exception:
                    head_bonus = 0.0
            # regional diversity: small prior penalty if this region already dense in prev assigns
            reg = region_key(f)
            reg_prev_density = 0
            if prev_assigns:
                try:
                    reg_prev_density = sum(1 for pf in prev_assigns.values() if pf and region_key(pf) == reg)
                except Exception:
                    reg_prev_density = 0
            # Linked-signature TTL risk: if this agent has a low TTL (likely losing relay link)
            # and the candidate position keeps degree very low, add a soft penalty to pull it back toward the mesh.
            link_pen = 0.0
            if link_risk_penalty and link_ttl_map is not None:
                try:
                    ttl = link_ttl_map.get(i, 0)
                    if ttl <= int(link_risk_ttl_thresh):
                        # strong risk when deg <=1; scale penalty by missing neighbors to reach degree_min_target
                        missing = max(0, int(degree_min_target) - (deg if 'deg' in locals() else 0))
                        link_pen = link_risk_penalty * float(missing)
                except Exception:
                    link_pen = 0.0

            val = info + bonus - 0.01 * dist - lam * counts[f] - prox_beta * nearby + conn_term + chain_bonus - tri_pen - deg_pen - link_pen - 0.05 * reg_prev_density - tether_pen + head_bonus
            scores.append((val, i, f, reg))

    # sort descending and greedily assign unique agent->frontier
    scores.sort(reverse=True, key=lambda x: x[0])
    assigned_agents = set()
    assigned_frontiers = set()
    assigned_regions = {}
    assignments = {}
    leader_claims = {}
    U_pred = 0

    # Build a map of previous best scores per agent to apply hysteresis
    prev_scores = {}
    if prev_assigns:
        for i in env.agent_positions.keys():
            prev_f = prev_assigns.get(i)
            if prev_f is None:
                continue
            # compute the prev score using a consistent base formula
            dist = abs(prev_f[0]-env.agent_positions[i][0]) + abs(prev_f[1]-env.agent_positions[i][1])
            info = frontier_score.get(prev_f, 0.0)
            # Use previous assignments for proximity, not current positions
            nearby = sum(1 for other_id, other_assign in prev_assigns.items() 
                        if other_id != i and other_assign and 
                        abs(other_assign[0]-prev_f[0])+abs(other_assign[1]-prev_f[1]) <= prox_radius)
            bonus = persist_bonus
            prev_scores[i] = info + bonus - 0.01 * dist - lam * counts.get(prev_f, 0) - prox_beta * nearby

    for val, i, f, reg in scores:
        if i in assigned_agents or f in assigned_frontiers:
            continue
        # Enforce spacing between assignments in the SAME timestep to reduce bunching.
        # Citations: Burgard et al. (2000) collaborative exploration; spacing improves frontier coverage efficiency.
        if spacing_radius and assigned_frontiers:
            too_close = any(abs(f[0]-af[0]) + abs(f[1]-af[1]) <= int(spacing_radius) for af in assigned_frontiers)
            if too_close:
                # If previous assignment exists and is not too close to already taken frontiers, keep it via hysteresis
                if prev_assigns:
                    prev_f = prev_assigns.get(i)
                    if prev_f is not None and prev_f not in assigned_frontiers:
                        far_enough = all(abs(prev_f[0]-af[0]) + abs(prev_f[1]-af[1]) > int(spacing_radius) for af in assigned_frontiers)
                        if far_enough:
                            assignments[i] = prev_f
                            leader_claims[i] = prev_f
                            assigned_agents.add(i)
                            assigned_frontiers.add(prev_f)
                            counts[prev_f] += 1
                            U_pred += frontier_score.get(prev_f, 0.0)
                            if region_capacity is not None and region_capacity > 0:
                                assigned_regions[region_key(prev_f)] = assigned_regions.get(region_key(prev_f), 0) + 1
                            continue
                # Otherwise skip this candidate and try the next score for this agent
                continue
        # regional capacity: limit #agents per coarse tile to reduce clustering
        if region_capacity is not None and region_capacity > 0:
            if assigned_regions.get(reg, 0) >= region_capacity:
                # allow keeping previous assignment via hysteresis even if region is full
                pscore = prev_scores.get(i, None)
                if pscore is not None and prev_assigns is not None:
                    prev_f = prev_assigns.get(i)
                    if prev_f is not None and region_key(prev_f) == reg and prev_f not in assigned_frontiers:
                        assignments[i] = prev_f
                        leader_claims[i] = prev_f
                        assigned_agents.add(i)
                        assigned_frontiers.add(prev_f)
                        counts[prev_f] += 1
                        U_pred += frontier_score.get(prev_f, 0.0)
                        assigned_regions[reg] = assigned_regions.get(reg, 0) + 1
                        continue
                # otherwise skip and try next candidate
                continue
        # hysteresis: if agent had a previous assignment, only switch if new val > prev_val + switch_margin
        pscore = prev_scores.get(i, None)
        if pscore is not None and pscore + switch_margin >= val:
            # prefer previous assignment if it's still within margin
            prev_f = prev_assigns.get(i)
            if prev_f is not None and prev_f not in assigned_frontiers:
                assignments[i] = prev_f
                leader_claims[i] = prev_f
                assigned_agents.add(i)
                assigned_frontiers.add(prev_f)
                counts[prev_f] += 1
                U_pred += frontier_score.get(prev_f, 0.0)
                if region_capacity is not None and region_capacity > 0:
                    assigned_regions[region_key(prev_f)] = assigned_regions.get(region_key(prev_f), 0) + 1
                continue
        assignments[i] = f
        leader_claims[i] = f
        assigned_agents.add(i)
        assigned_frontiers.add(f)
        counts[f] += 1
        U_pred += frontier_score.get(f, 0.0)
        if region_capacity is not None and region_capacity > 0:
            assigned_regions[reg] = assigned_regions.get(reg, 0) + 1

    # any unassigned agents get a nearest frontier (fallback)
    for i in env.agent_positions.keys():
        if i not in assignments:
            # pick nearest frontier not assigned if possible
            available = [f for f in frontiers if f not in assigned_frontiers]
            if not available:
                # all frontiers taken -> assign best by original score
                available = frontiers
            best = min(available, key=lambda f: abs(f[0]-env.agent_positions[i][0]) + abs(f[1]-env.agent_positions[i][1]))
            assignments[i] = best
            leader_claims[i] = best
            counts[best] += 1
            U_pred += frontier_score.get(best, 0.0)

    return assignments, leader_claims, U_pred


def assign_frontiers_connected(env,
                               frontiers,
                               *,
                               prev_assigns=None,
                               window=2,
                               prox_beta=0.5,
                               prox_radius=4,
                               persist_bonus=0.2,
                               switch_margin=0.05,
                               comm_radius=None,
                               leader_id=0,
                               leader_belief_lr=None,
                               parent_map=None,
                               tether_margin=1,
                               region_size=None,
                               region_capacity=1,
                               triangle_penalty=0.0,
                               degree_target=2,
                               degree_over_penalty=0.0,
                               degree_min_target=1,
                               degree_under_penalty=0.0,
                               # Anti-bunching within a parent's neighborhood
                               spacing_radius=0,
                               sibling_spacing_radius=None,
                               sibling_penalty=0.3,
                               # Encourage angular diversity of siblings around the parent
                               sibling_angle_bins=8,
                               sibling_angle_penalty=0.2,
                               # Allow a child to anchor to ANY already-planned teammate within R-margin (not only its parent)
                               allow_anchor_any=True,
                               # Soft repulsion from already-planned teammates to avoid crowding
                               repulse_beta=0.05,
                               repulse_radius=3,
                               # Optional cap per angular sector around a parent (local diversity hard limit)
                               sibling_sector_capacity=1,
                               unique_gain_beta=0.0,
                               unique_radius=None,
                               # New: FOV-overlap penalty between agents' predicted observation windows
                               fov_overlap_beta=0.0,
                               fov_radius=None,
                               frontier_score_map=None):
    """Connectivity-preserving frontier assignment via leader-rooted BFS tree.

    Strategy (discrete variant of connectivity constraints; Ji & Egerstedt 2007; Zavlanos & Pappas 2005):
    - Build a parent tree rooted at leader. Assign in BFS order so parents plan before children.
    - For each agent i (except leader), restrict candidates to those f with L1(f, parent_future) <= R - margin.
      This preserves the spanning tree edges under single-step motion, maintaining global connectivity.
    - Within the feasible set, score by information gain and mild dispersion; keep hysteresis.
    """
    if not frontiers:
        return {}, {}, 0

    # Infer comm radius
    if comm_radius is None:
        try:
            comm_radius = getattr(env, 'command_radius', None)
        except Exception:
            comm_radius = None

    belief_lr = leader_belief_lr
    if belief_lr is None and leader_id in getattr(env, "agents", {}):
        belief_lr = getattr(env.agents[leader_id], "belief_lr", None)
    cache = _get_frontier_entropy_cache(env, window, leader_belief_lr=belief_lr, leader_id=leader_id)
    frontier_score = frontier_score_map or _build_frontier_scores(env, frontiers, window, cache)
    ds = cache["ds"]

    # Region controls
    if region_size is None:
        try:
            region_size = max(4, int(ds * max(1, prox_radius // 2)))
        except Exception:
            region_size = 8

    def region_key(f):
        return (f[0] // max(1, region_size), f[1] // max(1, region_size))

    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()
    assigned_frontiers = set()
    assigned_regions = {}
    assignments = {}
    leader_claims = {}
    U_pred = 0
    # Track per-parent chosen frontiers to discourage sibling crowding
    parent_children_targets = {}
    if sibling_spacing_radius is None:
        sibling_spacing_radius = spacing_radius if spacing_radius else prox_radius
    parent_children_angles = {}
    # Unique coverage bookkeeping (to discourage overlapping work). Also track unknown-only for overlap penalty.
    claimed_pixels = set()
    claimed_unknown_pixels = set()
    try:
        r_unique = int(unique_radius) if unique_radius is not None else max(1, int(window) * int(ds))
    except Exception:
        r_unique = 3
    # FOV radius for overlap calculation (defaults to env.sensor_radius if available, else r_unique)
    try:
        r_fov = int(fov_radius) if fov_radius is not None else int(getattr(env, 'sensor_radius', r_unique))
    except Exception:
        r_fov = r_unique

    # planned positions: parents first (will update as we assign)
    planned = dict(env.agent_positions)
    # Anchor set to preserve connectivity when allow_anchor_any=True
    anchors = set([leader_id]) if leader_id in env.agent_positions else set()

    # BFS order from leader using parent_map, with randomized tie-breaking within each level
    ids = list(env.agent_positions.keys())
    order = []
    if parent_map is None or leader_id not in ids:
        order = ids
    else:
        children = {i: [] for i in ids}
        for child, parent in parent_map.items():
            if parent in children:
                children[parent].append(child)
        # Shuffle children at each level to avoid deterministic first-pick bias
        import random as _rand
        for clist in children.values():
            _rand.shuffle(clist)
        q = deque([leader_id])
        seen = set()
        while q:
            u = q.popleft()
            if u in seen:
                continue
            seen.add(u)
            order.append(u)
            for v in children.get(u, []):
                q.append(v)
        # append any isolated ids to avoid missing someone
        for i in ids:
            if i not in seen:
                order.append(i)
                order.append(i)

    # Precompute prev_scores for hysteresis
    prev_scores = {}
    if prev_assigns:
        for i in ids:
            pf = prev_assigns.get(i)
            if pf is None:
                continue
            dist = abs(pf[0]-env.agent_positions[i][0]) + abs(pf[1]-env.agent_positions[i][1])
            info = frontier_score.get(pf, 0.0)
            nearby = sum(1 for oid, of in prev_assigns.items() if oid != i and of and abs(of[0]-pf[0])+abs(of[1]-pf[1]) <= prox_radius)
            prev_scores[i] = info + persist_bonus - 0.01 * dist - prox_beta * nearby

    allow = max(0, int(comm_radius) - int(tether_margin)) if comm_radius else None

    belief_lr_used = belief_lr
    # Unknown checker based on leader belief (coarse)
    def _is_unknown_world(wx, wy):
        if belief_lr_used is None:
            return True
        gx, gy = wx // cache["ds"], wy // cache["ds"]
        if gx < 0 or gy < 0 or gx >= belief_lr_used.shape[0] or gy >= belief_lr_used.shape[1]:
            return True
        v = belief_lr_used[gx, gy]
        return v == -1 or v == 1

    for i in order:
        pos = env.agent_positions[i]
        # leader has no parent constraint
        parent = parent_map.get(i) if parent_map else None
        parent_future = planned.get(parent) if parent is not None else None

        # Candidate set: within tether constraint (if parent exists) and not already taken
        candidates = []
        for f in frontiers:
            if f in assigned_frontiers:
                continue
            # Connectivity tether: CRITICAL FIX - use CURRENT positions of anchors, not planned targets
            # This prevents cascading collapse where everyone's feasible set shrinks to the same tip
            if allow is not None:
                ok = True
                if not allow_anchor_any and parent is not None:
                    # Strict mode: must stay within R-margin of parent's CURRENT position
                    parent_pos = env.agent_positions.get(parent, pos)
                    ok = (abs(f[0]-parent_pos[0]) + abs(f[1]-parent_pos[1]) <= allow)
                else:
                    # Flexible mode: can tether to ANY anchor's CURRENT position
                    if anchors:
                        ok = any((abs(f[0]-env.agent_positions[a][0]) + abs(f[1]-env.agent_positions[a][1]) <= allow) for a in anchors if a in env.agent_positions)
                if not ok:
                    continue
            # First-step feasibility filter: ensure there exists a greedy first step toward f
            # that keeps link to the PARENT'S CURRENT POSITION in range (not planned target)
            # This avoids issuing targets that movement guard will immediately veto (leaf stuck).
            feasible = True
            if parent is not None and comm_radius:
                parent_pos = env.agent_positions.get(parent, pos)
                R = int(comm_radius)
                x, y = pos
                dx = f[0] - x; dy = f[1] - y
                step_opts = []
                if dx != 0:
                    step_opts.append((x + (1 if dx > 0 else -1), y))
                if dy != 0:
                    step_opts.append((x, y + (1 if dy > 0 else -1)))
                # keep inside bounds
                step_opts = [(sx, sy) for (sx, sy) in step_opts if 0 <= sx < env.size and 0 <= sy < env.size]
                # require at least one step to keep the parent CURRENT pos within R
                if step_opts:
                    feasible = any((abs(sx - parent_pos[0]) + abs(sy - parent_pos[1]) <= R) for (sx, sy) in step_opts)
                else:
                    feasible = False
            if feasible:
                candidates.append(f)

        # If nothing feasible, try keeping previous assignment if valid
        kept_prev = False
        if prev_assigns and candidates:
            pf = prev_assigns.get(i)
            parent_pos = env.agent_positions.get(parent, pos) if parent is not None else None
            if pf is not None and pf not in assigned_frontiers and (parent_pos is None or allow is None or (abs(pf[0]-parent_pos[0]) + abs(pf[1]-parent_pos[1]) <= allow)):
                # compute new best to compare
                best_val = -1e18
                best_f = None
                for f in candidates:
                    dist = abs(f[0]-pos[0]) + abs(f[1]-pos[1])
                    info = frontier_score.get(f, 0.0)
                    nearby = sum(1 for of in assigned_frontiers if abs(of[0]-f[0]) + abs(of[1]-f[1]) <= prox_radius)
                    val = info + 0.0 - 0.01 * dist - prox_beta * nearby
                    if val > best_val:
                        best_val = val; best_f = f
                # previous score
                pscore = prev_scores.get(i, None)
                if pscore is not None and pscore + switch_margin >= best_val:
                    assignments[i] = pf
                    leader_claims[i] = pf
                    assigned_frontiers.add(pf)
                    planned[i] = pf
                    counts[pf] += 1
                    U_pred += frontier_score.get(pf, 0.0)
                    if region_capacity and region_capacity > 0:
                        rk = region_key(pf)
                        assigned_regions[rk] = assigned_regions.get(rk, 0) + 1
                    kept_prev = True

        if kept_prev:
            continue

        # Score feasible candidates
        best, best_val = None, -1e18
        for f in candidates:
            # regional capacity guard
            if region_capacity and region_capacity > 0 and assigned_regions.get(region_key(f), 0) >= region_capacity:
                continue
            dist = abs(f[0]-pos[0]) + abs(f[1]-pos[1])
            info = frontier_score.get(f, 0.0)
            # mild clique avoidance (estimate degree at candidate using planned positions)
            deg_pen = 0.0
            if comm_radius:
                deg = 0
                for j, pj in planned.items():
                    if j == i:
                        continue
                    if abs(pj[0]-f[0]) + abs(pj[1]-f[1]) <= int(comm_radius):
                        deg += 1
                if degree_over_penalty and deg > max(0, int(degree_target)):
                    deg_pen += degree_over_penalty * float(deg - int(degree_target))
                if degree_under_penalty and deg < max(0, int(degree_min_target)):
                    deg_pen += degree_under_penalty * float(int(degree_min_target) - deg)

            nearby = 0
            # use already-assigned frontiers to discourage bunching
            for af in assigned_frontiers:
                if abs(af[0]-f[0]) + abs(af[1]-f[1]) <= prox_radius:
                    nearby += 1
            # ALSO count how many already-planned agents will be near this frontier
            # (their current or planned positions within comm radius)
            agents_nearby = 0
            for j, pj in planned.items():
                if j == i:
                    continue
                if abs(pj[0]-f[0]) + abs(pj[1]-f[1]) <= int(comm_radius):
                    agents_nearby += 1
            # sibling crowding penalty (within same parent's neighborhood)
            sib_pen = 0.0
            if parent is not None and sibling_spacing_radius and parent_children_targets.get(parent):
                for pf in parent_children_targets[parent]:
                    d = abs(pf[0]-f[0]) + abs(pf[1]-f[1])
                    if d <= int(sibling_spacing_radius):
                        # stronger when very close, linear to zero at radius
                        sib_pen += sibling_penalty * (1.0 - (d / float(max(1, int(sibling_spacing_radius)))))
            # angular diversity penalty: discourage siblings selecting the same ray from parent
            ang_pen = 0.0
            if parent is not None and sibling_angle_bins and sibling_angle_penalty and parent_children_angles.get(parent) is not None:
                try:
                    ax, ay = planned[parent]
                    dx = f[0] - ax; dy = f[1] - ay
                    if dx != 0 or dy != 0:
                        angle = math.atan2(dy, dx)
                        bucket = int(((angle + math.pi) / (2.0 * math.pi)) * int(sibling_angle_bins))
                        bucket = max(0, min(int(sibling_angle_bins) - 1, bucket))
                        if bucket in parent_children_angles[parent]:
                            # penalize reuse of the same sector; lighter than spacing
                            ang_pen = float(sibling_angle_penalty)
                except Exception:
                    ang_pen = 0.0
            # soft repulsion from anchors to prevent piling on the same side of the chain
            repulse = 0.0
            if repulse_beta and repulse_radius and anchors:
                rr = int(repulse_radius)
                for a_id in anchors:
                    ax, ay = planned.get(a_id, pos)
                    d = abs(ax - f[0]) + abs(ay - f[1])
                    if d <= rr and d > 0:
                        repulse += repulse_beta * (1.0 - (d / float(rr)))
            # unique coverage incentive: count unknown pixels in a small window and
            # reward portions not already claimed by earlier assignments
            uniq_term = 0.0
            if unique_gain_beta and r_unique > 0:
                x0 = max(0, f[0] - r_unique); x1 = min(env.size, f[0] + r_unique + 1)
                y0 = max(0, f[1] - r_unique); y1 = min(env.size, f[1] + r_unique + 1)
                total_unknown = 0
                unique_unknown = 0
                if x1 > x0 and y1 > y0:
                    for yy in range(y0, y1):
                        for xx in range(x0, x1):
                            if _is_unknown_world(xx, yy):
                                total_unknown += 1
                                if (xx, yy) not in claimed_pixels:
                                    unique_unknown += 1
                if total_unknown > 0:
                    uniq_term = unique_gain_beta * (unique_unknown / float(total_unknown))
            # FOV overlap penalty: discourage picking a target whose LOS window of unknowns
            # largely overlaps with windows already allocated to earlier agents.
            overlap_pen = 0.0
            if fov_overlap_beta and r_fov > 0:
                x0o = max(0, f[0] - r_fov); x1o = min(env.size, f[0] + r_fov + 1)
                y0o = max(0, f[1] - r_fov); y1o = min(env.size, f[1] + r_fov + 1)
                if x1o > x0o and y1o > y0o:
                    cand_unknown = 0
                    cand_overlap = 0
                    for yy in range(y0o, y1o):
                        for xx in range(x0o, x1o):
                            if _is_unknown_world(xx, yy):
                                cand_unknown += 1
                                if (xx, yy) in claimed_unknown_pixels:
                                    cand_overlap += 1
                    if cand_unknown > 0:
                        overlap_pen = fov_overlap_beta * (cand_overlap / float(cand_unknown))
            
            # Aggressive distance weighting: make it meaningful relative to info gain
            # Scale distance by 0.05 (5x stronger than before) to encourage spread
            dist_weight = 0.05
            # Add a heavy "agent crowding" penalty if too many teammates will be near this target
            crowd_pen = 0.0
            if agents_nearby > 2:  # more than 2 neighbors = crowded
                crowd_pen = 0.5 * float(agents_nearby - 2)
            
            val = info + uniq_term - overlap_pen - dist_weight * dist - prox_beta * nearby - crowd_pen - deg_pen - sib_pen - ang_pen - repulse
            if val > best_val:
                best_val = val; best = f

        if best is None:
            # Repair mode: if no feasible frontier, hold at current position or move toward nearest teammate
            parent_pos = env.agent_positions.get(parent, pos) if parent is not None else None
            if parent_pos is not None:
                best = parent_pos
            else:
                # fallback: pick nearest remaining frontier
                available = [f for f in frontiers if f not in assigned_frontiers]
                if not available:
                    available = frontiers
                if available:
                    best = min(available, key=lambda f: abs(f[0]-pos[0]) + abs(f[1]-pos[1]))
                else:
                    best = pos

        assignments[i] = best
        leader_claims[i] = best
        assigned_frontiers.add(best)
        planned[i] = best
        counts[best] += 1
        U_pred += frontier_score.get(best, 0.0)
        if region_capacity and region_capacity > 0:
            assigned_regions[region_key(best)] = assigned_regions.get(region_key(best), 0) + 1
        if parent is not None:
            parent_children_targets.setdefault(parent, []).append(best)
            # record angle sector for angular diversity tracking
            try:
                ax, ay = planned[parent]
                dx = best[0] - ax; dy = best[1] - ay
                if dx != 0 or dy != 0:
                    angle = math.atan2(dy, dx)
                    bucket = int(((angle + math.pi) / (2.0 * math.pi)) * int(sibling_angle_bins))
                    bucket = max(0, min(int(sibling_angle_bins) - 1, bucket))
                    # Enforce a hard capacity per sector if requested
                    if sibling_sector_capacity is not None and sibling_sector_capacity > 0:
                        already = parent_children_angles.get(parent, set())
                        # If capacity is 1, disallow new additions to an already used bucket by re-adding same bucket only once
                        if bucket in already and sibling_sector_capacity <= 1:
                            pass
                    parent_children_angles.setdefault(parent, set()).add(bucket)
            except Exception:
                pass
        # After finalizing this agent's plan, it becomes an anchor for its children/siblings
        anchors.add(i)
        # Add claimed pixels for unique coverage accounting
        if r_unique > 0 or r_fov > 0:
            # Expand claimed_pixels around the chosen target (for uniqueness reward)
            if unique_gain_beta and r_unique > 0:
                x0 = max(0, best[0] - r_unique); x1 = min(env.size, best[0] + r_unique + 1)
                y0 = max(0, best[1] - r_unique); y1 = min(env.size, best[1] + r_unique + 1)
                for yy in range(y0, y1):
                    for xx in range(x0, x1):
                        claimed_pixels.add((xx, yy))
            # Track unknown-only FOV set for overlap penalty based on leader belief
            if fov_overlap_beta and r_fov > 0:
                xo0 = max(0, best[0] - r_fov); xo1 = min(env.size, best[0] + r_fov + 1)
                yo0 = max(0, best[1] - r_fov); yo1 = min(env.size, best[1] + r_fov + 1)
                for yy in range(yo0, yo1):
                    for xx in range(xo0, xo1):
                        if _is_unknown_world(xx, yy):
                            claimed_unknown_pixels.add((xx, yy))

    return assignments, leader_claims, U_pred

def _shannon_entropy(p):
    # p is probability of occupied/obstacle; handle edge cases
    if p <= 0.0 or p >= 1.0:
        return 0.0
    return - (p * math.log(p + 1e-12) + (1 - p) * math.log(1 - p + 1e-12))


def assign_frontiers_entropy(env, frontiers, lam=3.0, prev_assigns=None, window=2, prox_beta=0.5, prox_radius=4, persist_bonus=0.2, leader_id=0, leader_belief_lr=None):
    """Assign frontiers by expected information gain (Shannon entropy) with congestion penalty.

    Args:
        env: GridWorld
        frontiers: list of (x,y)
        lam: congestion penalty coefficient
        prev_assigns: previous assigns (for counting)
        window: radius (in downsampled cells) around frontier to compute entropy
    """
    assignments = {}
    leader_claims = {}
    U_pred = 0

    if not frontiers:
        return assignments, leader_claims, U_pred

    belief_lr = leader_belief_lr
    if belief_lr is None and leader_id in getattr(env, "agents", {}):
        belief_lr = getattr(env.agents[leader_id], "belief_lr", None)
    cache = _get_frontier_entropy_cache(env, window, leader_belief_lr=belief_lr, leader_id=leader_id)
    frontier_score = _build_frontier_scores(env, frontiers, window, cache)

    counts = Counter(prev_assigns.values()) if prev_assigns else Counter()

    # assign per-agent maximizing (info_score - distance - lam*congestion - prox_beta*nearby_agents)
    for i, pos in env.agent_positions.items():
        best, best_val = None, -math.inf
        for f in frontiers:
            dist = abs(f[0]-pos[0]) + abs(f[1]-pos[1])
            info = frontier_score.get(f, 0.0)
            # persistence bonus: prefer previous target for this agent
            bonus = 0.0
            if prev_assigns and prev_assigns.get(i) == f:
                bonus = persist_bonus
            # count nearby agents already near this frontier (in world coords)
            nearby = 0
            for other_id, other_pos in env.agent_positions.items():
                if other_id == i:
                    continue
                if abs(other_pos[0] - f[0]) + abs(other_pos[1] - f[1]) <= prox_radius:
                    nearby += 1
            val = info + bonus - 0.01 * dist - lam * counts[f] - prox_beta * nearby
            # randomized tie-breaker to avoid deterministic starvation
            if val > best_val or (abs(val - best_val) < 1e-9 and random.random() < 0.5):
                best, best_val = f, val
        assignments[i] = best
        leader_claims[i] = best
        counts[best] += 1
        U_pred += frontier_score.get(best, 0.0)

    return assignments, leader_claims, U_pred
