"""
GridWorld Environment for Multi-Agent Exploration

Implements a 2D grid environment with obstacles, agent movement, and visibility.
Uses A* pathfinding for navigation and breadth-first search for connectivity.

References:
- Hart et al. (1968): A Formal Basis for the Heuristic Determination of Minimum Cost Paths (A*)
- LaValle (2006): Planning Algorithms (Chapter 2: Discrete Planning)
"""

import numpy as np
import random
import heapq
from collections import deque
from src.agents.agent import Agent



class GridWorld:
    def __init__(self, cfg):
        self.size = cfg["map_size"]
        self.N = cfg["N"]
        # assignment hold parameter (timesteps to keep an assignment)
        self.min_hold = cfg.get("min_hold", 3)
        # soft multi-agent deconfliction parameters
        self.neighbor_proximity_radius = cfg.get("neighbor_proximity_radius", 2)
        self.neighbor_repulsion = cfg.get("neighbor_repulsion", 0.6)
        self.reservation_penalty = cfg.get("reservation_penalty", 3.0)
        # timestamps of when a cell was last visited by any agent (None if never)
        self.last_visited = np.full((self.size, self.size), -1, dtype=int)
        # tuning parameters for recency/known penalties (available to policies)
        self.known_penalty = cfg.get("known_penalty", 0.5)
        self.recent_penalty = cfg.get("recent_penalty", 0.5)
        self.recent_window = cfg.get("recent_window", 5)
        # current timestep (updated each env.step call)
        self.current_time = 0

        # Optional debug/logging toggles
        env_cfg = cfg.get("env", {})
        self.debug_logging = bool(env_cfg.get("debug_logging", cfg.get("debug_logging", False)))
        self.debug_frontiers = bool(env_cfg.get("debug_frontiers", cfg.get("debug_frontiers", False)))
        self.debug_frontiers_every = max(1, int(env_cfg.get("debug_frontiers_every", 10)))
        # Expose helper methods for guarded prints
        self._log = lambda msg: print(msg) if self.debug_logging else None
        self._log_frontier = lambda msg: print(msg) if self.debug_frontiers else None

        # Optional communication-aware movement guardrails
        comm_cfg = cfg.get("comm", {}) if isinstance(cfg.get("comm", {}), dict) else {}
        reip_cfg = cfg.get("reip", {}) if isinstance(cfg.get("reip", {}), dict) else {}
        try:
            # Prefer policy command radius if provided to keep assignment and motion consistent
            r_comm = int(comm_cfg.get("radius", 0))
            r_reip = int(reip_cfg.get("command_radius", 0))
            self.comm_radius = max(r_comm, r_reip)
        except Exception:
            try:
                self.comm_radius = int(comm_cfg.get("radius", 0))
            except Exception:
                self.comm_radius = 0
        # Enable a soft guard: avoid moves that drop an agent's neighbor degree below a minimum
        guard_cfg = comm_cfg if isinstance(comm_cfg, dict) else {}
        self.comm_guard_enable = bool(guard_cfg.get("guard_enable", True))
        self.comm_guard_min_degree = int(guard_cfg.get("guard_min_degree", 1))
        self.comm_guard_margin = int(guard_cfg.get("guard_margin", 0))
        # Optional replanning throttle: compute full path every k steps (per agent)
        # 1 = replan every step (default behavior)
        self.replan_interval = int(cfg.get("replan_interval", env_cfg.get("replan_interval", 1)) or 1)

        # Get obstacle parameters
        p_obs = env_cfg.get("obstacle_density", 0.0)
        seed = env_cfg.get("seed", None)
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        # Initialize grid: 0 = free, -1 = obstacle
        # Check if custom map is specified
        custom_map = env_cfg.get("custom_map", None)
        map_file = env_cfg.get("map_file", None)
        
        if custom_map or map_file:
            # Load custom map
            from src.env.custom_maps import load_office_layout, load_map_from_file
            
            if map_file:
                if self.debug_logging:
                    self._log(f"[GridWorld] Loading custom map from file: {map_file}")
                self.grid = load_map_from_file(map_file, self.size)
            elif custom_map:
                if self.debug_logging:
                    self._log(f"[GridWorld] Loading predefined map: {custom_map}")
                self.grid = load_office_layout(custom_map)
                # Resize if needed
                if self.grid.shape[0] != self.size:
                    resized = np.zeros((self.size, self.size), dtype=np.int8)
                    min_size = min(self.size, self.grid.shape[0])
                    resized[:min_size, :min_size] = self.grid[:min_size, :min_size]
                    self.grid = resized
        else:
            # Random obstacle generation (original behavior)
            self.grid = np.zeros((self.size, self.size), dtype=np.int8)
            if p_obs > 0:
                mask = np.random.rand(self.size, self.size) < p_obs
                self.grid[mask] = -1

        # Team belief map: accumulates all observations from all agents (monotonic).
        # NOTE: Decision logic must not depend on this in Phase 2.
        # -1 = unknown, 0 = free, 2 = obstacle
        self.team_belief = np.full((self.size, self.size), -1, dtype=np.int8)

        # Phase 2 role gating / leader blindness
        phase2_cfg = cfg.get("phase2", {}) if isinstance(cfg.get("phase2", {}), dict) else {}
        self.phase2_leader_blind = bool(phase2_cfg.get("leader_blind", False))
        # Controller/main should set env.leader_id each step; default safe value
        self.leader_id = 0

        # Random agent positions in free cells only (optionally constrained to a spawn region)
        free = list(zip(*np.where(self.grid == 0)))
        assert len(free) >= self.N, "Insufficient free cells for agents"

        # Optional spawn constraints to keep agents in the same area
        # Configure via env.spawn_box: [x0, y0, x1, y1]
        # or env.spawn_center: [x, y] and env.spawn_radius: int
        spawn_candidates = free
        try:
            x0 = y0 = x1 = y1 = None
            if 'spawn_box' in env_cfg:
                box = env_cfg.get('spawn_box') or []
                if isinstance(box, (list, tuple)) and len(box) == 4:
                    x0, y0, x1, y1 = map(int, box)
                    x0, y0 = max(0, x0), max(0, y0)
                    x1, y1 = min(self.size - 1, x1), min(self.size - 1, y1)
                    # Preserve (x, y) ordering when filtering spawn candidates
                    spawn_candidates = [(x, y) for (x, y) in spawn_candidates if x0 <= x <= x1 and y0 <= y <= y1]
            elif 'spawn_center' in env_cfg:
                cx, cy = env_cfg.get('spawn_center') or [self.size // 2, self.size // 2]
                r = int(env_cfg.get('spawn_radius', 3))
                cx, cy = int(cx), int(cy)
                # Use a simple bounding box around center (Chebyshev radius)
                x0, y0 = max(0, cx - r), max(0, cy - r)
                x1, y1 = min(self.size - 1, cx + r), min(self.size - 1, cy + r)
                # Preserve (x, y) ordering consistently when filtering spawn candidates
                spawn_candidates = [(x, y) for (x, y) in spawn_candidates if x0 <= x <= x1 and y0 <= y <= y1]

            # Ensure we have enough candidates; if not, fall back to full free set
            if len(spawn_candidates) < self.N:
                # print a soft warning but continue
                # print(f"[Spawn] Not enough cells in constrained region ({len(spawn_candidates)} < {self.N}); using full free set.")
                spawn_candidates = free
        except Exception:
            # On any parsing error, just use full free set
            spawn_candidates = free

        # Spawn mode controls (deterministic vs random) and dedicated RNG
        spawn_mode = str(env_cfg.get("spawn_mode", "random")).lower()
        self.spawn_mode = spawn_mode if spawn_mode in ("random", "deterministic") else "random"
        spawn_seed = env_cfg.get("spawn_seed", None)
        try:
            spawn_seed = int(spawn_seed) if spawn_seed is not None else None
        except Exception:
            spawn_seed = None
        self.spawn_seed = spawn_seed
        spawn_rng = random.Random(spawn_seed) if spawn_seed is not None else random
        if self.spawn_mode == "deterministic":
            spawn_candidates = sorted(spawn_candidates)

        # Coverage bookkeeping
        self.coverage_history = []
        self.coverage_history_maxlen = int(cfg.get("coverage_history_maxlen", env_cfg.get("coverage_history_maxlen", 0)) or 0)

        r_local = cfg.get("agent", {}).get("r_local", 8)
        ds = cfg.get("agent", {}).get("downsample", 4)

        self.agents = {}
        self.agent_positions = {}

        for i in range(self.N):
            if not spawn_candidates:
                spawn_candidates = list(free)
                if self.spawn_mode == "deterministic":
                    spawn_candidates = sorted(spawn_candidates)
            if self.spawn_mode == "deterministic":
                pos = spawn_candidates.pop(0)
                if pos in free:
                    free.remove(pos)
            else:
                pos = spawn_rng.choice(spawn_candidates)
                if pos in spawn_candidates:
                    spawn_candidates.remove(pos)
                if pos in free:
                    free.remove(pos)
            self.agent_positions[i] = pos
            a = Agent(self.size, r_local=r_local, ds=ds)
            a.pos = pos
            a.observe(self, r=1)
            a.writeback_belief_lr()  # Updated to use belief map instead of oracle global_lr
            self.agents[i] = a
            
            # Update team_belief with initial observations
            cx, cy = a.pos
            R = a.r_local
            for lx in range(a.local.shape[0]):
                for ly in range(a.local.shape[1]):
                    v = a.local[lx, ly]
                    if v != -1:  # observed (free or obstacle)
                        wx_cell = cx - R + lx
                        wy_cell = cy - R + ly
                        if 0 <= wx_cell < self.size and 0 <= wy_cell < self.size:
                            if self.team_belief[wx_cell, wy_cell] == -1:
                                self.team_belief[wx_cell, wy_cell] = v

        # Seed coverage bookkeeping with the initial state
        self._record_coverage(0)


    def has_path(env, start, goal, agent=None):
        """
        Check if there's a path from start to goal using belief-based navigation.
        
        Uses team_belief (observed map) rather than ground truth. Agents optimistically
        assume unknown cells are passable. This is realistic behavior for SLAM-based 
        navigation where the map is incomplete.
        
        Returns True if a path exists through known-free or unknown cells.
        """
        if start ==goal:
            return True 
        q = deque([start]); seen = {start}
        while q:
            x, y = q.popleft()
            for nx, ny in env._neighbors(x, y, agent=agent):
                if (nx, ny) in seen:
                    continue
                if (nx, ny) == goal:
                    return True
                seen.add((nx, ny)); q.append((nx, ny))
        return False
    
    def _neighbors(self, x, y, agent=None):
        """
        Return valid neighboring cells for pathfinding.
        
        If an agent is provided, use that agent's local map (inside r_local)
        and coarse belief_lr elsewhere. Unknown cells are assumed passable.
        This removes centralized team_belief leakage from navigation.
        
        KEY: For cells within local window, use ONLY local map.
             For cells outside local window, use coarse belief_lr.
             Never double-check local cells with belief_lr.
        """
        S = self.size
        if getattr(self, "phase2_leader_blind", False) and agent is None:
            raise RuntimeError("Phase2: _neighbors called without agent (would leak team belief).")
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < S and 0 <= ny < S:
                is_obstacle = False
                checked_local = False  # Track whether we used local map
                
                if agent is not None:
                    # Check if cell is within local sensing radius
                    if hasattr(agent, "r_local") and hasattr(agent, "local") and agent.pos is not None:
                        cx, cy = agent.pos
                        if abs(nx - cx) <= agent.r_local and abs(ny - cy) <= agent.r_local:
                            lx, ly = agent._world_to_local(nx, ny)
                            if 0 <= lx < agent.local.shape[0] and 0 <= ly < agent.local.shape[1]:
                                v = agent.local[lx, ly]
                                # Only trust local if it has DEFINITIVE info (0=free, 2=obstacle)
                                # If local is unknown (-1), fall through to belief_lr
                                if v == 2:
                                    is_obstacle = True
                                    checked_local = True
                                elif v == 0:
                                    is_obstacle = False
                                    checked_local = True
                                # else: v == -1 (unknown), checked_local stays False -> check belief_lr
                    
                    # ONLY use coarse belief for cells OUTSIDE local window
                    if not checked_local and hasattr(agent, "belief_lr"):
                        ds = max(1, int(getattr(agent, "ds", 1)))
                        gx, gy = nx // ds, ny // ds
                        if 0 <= gx < agent.belief_lr.shape[0] and 0 <= gy < agent.belief_lr.shape[1]:
                            is_obstacle = (agent.belief_lr[gx, gy] == 2)
                else:
                    # Legacy fallback for callers not providing agent
                    is_obstacle = (self.team_belief[nx, ny] == 2)

                if not is_obstacle:
                    yield nx, ny

    def _bfs_next_step(self, start, goal, agent=None):
        if start == goal:
            return start
        q = deque([start])
        prev = {start: None}
        found = False
        while q:
            cur=q.popleft()
            if cur == goal:
                found = True
                break
            for nb in self._neighbors(*cur, agent=agent):
                if nb not in prev:
                    prev[nb] = cur
                    q.append(nb)
        if not found:
            return start  # nuh uh
        # backtrack
        cur = goal
        while prev[cur] is not None and prev[cur] != start:
            cur = prev[cur]
        return cur

    def _weighted_path(self, start, goal, agent_penalty=2.0, agent=None, avoid_cells=None):
        """
        Weighted A*/Dijkstra full path: return list of steps from start to goal (excluding start).
        Uses the same cost model as _weighted_next_step, but returns the full path for caching.
        """
        if start == goal:
            return []

        S = self.size
        agents_set = set(self.agent_positions.values())
        avoid_cells = avoid_cells or set()
        last_visited = self.last_visited
        recent_window = self.recent_window
        recent_penalty = self.recent_penalty
        reservation_penalty = self.reservation_penalty
        neighbor_repulsion = self.neighbor_repulsion
        neighbor_radius = self.neighbor_proximity_radius
        current_time = self.current_time

        # Precompute agent proximity repulsion costs once per call
        repulsion_cost = {}
        if neighbor_repulsion > 0 and neighbor_radius > 0:
            for other in self.agent_positions.values():
                if other == start:
                    continue
                ox, oy = other
                for dx in range(-neighbor_radius, neighbor_radius + 1):
                    for dy in range(-neighbor_radius, neighbor_radius + 1):
                        d = abs(dx) + abs(dy)
                        if d == 0 or d > neighbor_radius:
                            continue
                        nx, ny = ox + dx, oy + dy
                        if 0 <= nx < S and 0 <= ny < S:
                            repulsion_cost[(nx, ny)] = repulsion_cost.get((nx, ny), 0.0) + neighbor_repulsion * (1.0 / d)

        # Cache agent move history for quick membership checks
        move_history_set = None
        if agent is not None and hasattr(agent, 'move_history'):
            try:
                move_history_set = set(agent.move_history)
            except Exception:
                move_history_set = None

        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        # Early exit if goal is an immediate neighbor
        first_neighbors = list(self._neighbors(*start, agent=agent))
        if goal in first_neighbors:
            return [goal]

        # Run A* with enhanced cost model
        open_heap = []
        heapq.heappush(open_heap, (heuristic(start, goal), 0, start))
        came_from = {start: None}
        gscore = {start: 0}

        found = False
        while open_heap:
            f, g, cur = heapq.heappop(open_heap)
            if cur == goal:
                found = True
                break
            for nb in self._neighbors(*cur, agent=agent):
                # base movement cost
                move_cost = 1.0
                # occupancy penalty
                if nb in agents_set and nb != start:
                    move_cost += agent_penalty
                # recency penalty: avoid cells visited very recently by any agent
                lv = last_visited[nb[0], nb[1]]
                if lv >= 0:
                    age = current_time - lv
                    if age < recent_window:
                        # stronger penalty when very recent
                        move_cost += recent_penalty * (1.0 - (age / float(recent_window)))
                # personal history penalty: avoid repeating own short path
                if move_history_set is not None and nb in move_history_set:
                    move_cost += 0.75
                # proximity repulsion: discourage being near other agents (soft)
                if repulsion_cost:
                    move_cost += repulsion_cost.get(nb, 0.0)
                # reservation penalty: avoid cells already reserved by higher-priority agents this tick
                if nb in avoid_cells:
                    move_cost += max(0.0, float(reservation_penalty))

                ng = g + move_cost
                if nb not in gscore or ng < gscore[nb]:
                    gscore[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (ng + heuristic(nb, goal), ng, nb))

        if not found:
            return []

        # Reconstruct full path
        path = []
        cur = goal
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path[1:]

    def _weighted_next_step(self, start, goal, agent_penalty=2.0, agent=None, avoid_cells=None):
        """
        Weighted A*/Dijkstra next-step: prefer paths through less crowded cells.
        
        Uses belief-based navigation (team_belief) rather than ground truth.
        Agents optimistically assume unknown cells are passable, which is realistic
        for SLAM-based navigation. Only cells observed as obstacles are avoided.

        Cost to move into a neighbor = 1 + agent_penalty * occupied + recency_penalty + history_penalty
        + proximity_repulsion + reservation_penalty.
        - agent_penalty applies if another agent occupies the cell
        - recency_penalty: avoid cells visited recently by ANY agent (using self.last_visited)
        - history_penalty: avoid cells in this agent's move_history (personal short-term memory)
        - backtrack avoidance: prefer not to return to agent.prev_pos if alternatives exist
        - proximity_repulsion: discourage moving near other agents within a radius
        - reservation_penalty: discourage moving into cells reserved by earlier agents this tick
        """
        if start == goal:
            return start

        S = self.size
        agents_set = set(self.agent_positions.values())
        avoid_cells = avoid_cells or set()
        last_visited = self.last_visited
        recent_window = self.recent_window
        recent_penalty = self.recent_penalty
        reservation_penalty = self.reservation_penalty
        neighbor_repulsion = self.neighbor_repulsion
        neighbor_radius = self.neighbor_proximity_radius
        current_time = self.current_time

        # Precompute agent proximity repulsion costs once per call
        repulsion_cost = {}
        if neighbor_repulsion > 0 and neighbor_radius > 0:
            for other in self.agent_positions.values():
                if other == start:
                    continue
                ox, oy = other
                for dx in range(-neighbor_radius, neighbor_radius + 1):
                    for dy in range(-neighbor_radius, neighbor_radius + 1):
                        d = abs(dx) + abs(dy)
                        if d == 0 or d > neighbor_radius:
                            continue
                        nx, ny = ox + dx, oy + dy
                        if 0 <= nx < S and 0 <= ny < S:
                            repulsion_cost[(nx, ny)] = repulsion_cost.get((nx, ny), 0.0) + neighbor_repulsion * (1.0 / d)

        # Cache agent move history for quick membership checks
        move_history_set = None
        if agent is not None and hasattr(agent, 'move_history'):
            try:
                move_history_set = set(agent.move_history)
            except Exception:
                move_history_set = None

        def heuristic(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        # Collect candidate neighbors from start, with backtrack avoidance
        first_neighbors = list(self._neighbors(*start, agent=agent))
        if agent is not None and hasattr(agent, 'prev_pos') and agent.prev_pos is not None:
            # Try to avoid immediate backtrack if alternatives exist
            non_back = [nb for nb in first_neighbors if nb != agent.prev_pos]
            if non_back:
                first_neighbors = non_back

        # If only one choice or goal is immediate neighbor, shortcut
        if goal in first_neighbors:
            return goal
        if len(first_neighbors) == 1:
            return first_neighbors[0]

        # Otherwise run A* with enhanced cost model
        open_heap = []
        heapq.heappush(open_heap, (heuristic(start, goal), 0, start))
        came_from = {start: None}
        gscore = {start: 0}

        found = False
        while open_heap:
            f, g, cur = heapq.heappop(open_heap)
            if cur == goal:
                found = True
                break
            for nb in self._neighbors(*cur, agent=agent):
                # base movement cost
                move_cost = 1.0
                # occupancy penalty
                if nb in agents_set and nb != start:
                    move_cost += agent_penalty
                # recency penalty: avoid cells visited very recently by any agent
                lv = last_visited[nb[0], nb[1]]
                if lv >= 0:
                    age = current_time - lv
                    if age < recent_window:
                        # stronger penalty when very recent
                        move_cost += recent_penalty * (1.0 - (age / float(recent_window)))
                # personal history penalty: avoid repeating own short path
                if move_history_set is not None and nb in move_history_set:
                    move_cost += 0.75
                # proximity repulsion: discourage being near other agents (soft)
                if repulsion_cost:
                    move_cost += repulsion_cost.get(nb, 0.0)
                # reservation penalty: avoid cells already reserved by higher-priority agents this tick
                if nb in avoid_cells:
                    move_cost += max(0.0, float(reservation_penalty))

                ng = g + move_cost
                if nb not in gscore or ng < gscore[nb]:
                    gscore[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (ng + heuristic(nb, goal), ng, nb))

        if not found:
            # No path under current belief. Use a reactive fallback: pick a neighbor
            # that reduces the L1 distance to goal and avoids known obstacles/reservations.
            # Citations: Khatib (1986) potential fields; simple local detour when global plan fails.
            best = None; best_h = 10**9
            for nb in self._neighbors(*start, agent=agent):
                # avoid reserved targets if provided
                if avoid_cells and nb in avoid_cells:
                    continue
                # avoid stepping back into a cell with high recent traffic
                lv = last_visited[nb[0], nb[1]]
                if lv >= 0 and (current_time - lv) <= max(1, recent_window // 2):
                    continue
                h = heuristic(nb, goal)
                if h < best_h:
                    best_h = h; best = nb
            return best if best is not None else start

        # backtrack to the first step
        cur = goal
        while came_from[cur] is not None and came_from[cur] != start:
            cur = came_from[cur]
        return cur

        

    def _team_known_free_world(self):
        """Return boolean map of cells known to be free (using accumulated team belief)."""
        return (self.team_belief == 0)


    def _known_free_from_belief_lr(self, belief_lr, ds, size=None):
        """Expand a coarse belief_lr to a fine known-free boolean map."""
        size = self.size if size is None else size
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

    def detect_frontiers_from_agent_belief(self, agent):
        """
        Compute frontiers using the given agent's coarse belief (belief_lr), not team_belief.
        Frontiers are coarse unknown cells adjacent to coarse known-free cells; returned as cell centers.
        """
        if agent is None or not hasattr(agent, "belief_lr"):
            return []
        ds = max(1, int(getattr(agent, "ds", 1)))
        belief_lr = agent.belief_lr
        F = []
        for gx in range(belief_lr.shape[0]):
            for gy in range(belief_lr.shape[1]):
                v = belief_lr[gx, gy]
                if v != -1 and v != 1:
                    continue
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < belief_lr.shape[0] and 0 <= ny < belief_lr.shape[1]:
                        if belief_lr[nx, ny] == 0:
                            cx = gx * ds + ds // 2
                            cy = gy * ds + ds // 2
                            F.append((cx, cy))
                            break

        if self.debug_frontiers and hasattr(self, 'current_time'):
            if self.current_time % self.debug_frontiers_every == 0:
                self._log_frontier(f"[Frontier] t={self.current_time}: frontiers={len(F)} (agent belief)")
        return F

    def detect_frontiers(self):
        """
        Legacy frontier computation using team belief (kept for backward compatibility).
        """
        known = self._team_known_free_world()
        F = []
        for x in range(self.size):
            for y in range(self.size):
                if self.team_belief[x, y] != -1:
                    continue
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < self.size and 0 <= ny < self.size and known[nx, ny]:
                        F.append((x,y))
                        break
        return F


    def step(self, assignments, t=0):
        """Move each agent one BFS step toward its assigned target and mark explored."""
        # update current time
        try:
            self.current_time = int(t)
        except Exception:
            self.current_time = 0
        # hold length (in timesteps) to keep an assignment before allowing reassignment
        MIN_HOLD = getattr(self, 'min_hold', 3)
        # First compute each agent's desired next cell (tentative) with soft reservations
        desired = {}
        reserved = set()
        replan_interval = max(1, int(getattr(self, 'replan_interval', 1)))
        for i in sorted(assignments.keys()):
            target = assignments[i]
            a = self.agents[i]
            # enforce hold: if agent has a hold_timer, respect its hold_target
            if getattr(a, 'hold_timer', 0) > 0 and a.hold_target is not None:
                target = a.hold_target
                a.hold_timer -= 1
            else:
                # new target: set hold
                a.hold_target = target
                a.hold_timer = MIN_HOLD
            x, y = self.agent_positions[i]
            # prefer weighted planner that penalizes occupied cells
            nxt = None

            # Optional replanning throttle: follow cached path for a few steps
            if replan_interval > 1:
                # Reset cache if target changed
                if getattr(a, 'path_cache_goal', None) != target:
                    a.path_cache = []
                    a.path_cache_goal = target
                # Use cached next step if still valid
                cache_ok = bool(getattr(a, 'path_cache', [])) and (self.current_time - getattr(a, 'path_cache_step', -9999) < replan_interval)
                if cache_ok:
                    cached_next = a.path_cache[0]
                    if cached_next in reserved:
                        cache_ok = False
                    else:
                        # Validate cached next step is still a valid neighbor
                        if cached_next in self._neighbors(x, y, agent=a):
                            nxt = a.path_cache.pop(0)
                        else:
                            cache_ok = False
                if not cache_ok:
                    # Recompute full path and cache it
                    path = self._weighted_path((x, y), target, agent_penalty=2.0, agent=a, avoid_cells=reserved)
                    if path:
                        a.path_cache = path
                        a.path_cache_goal = target
                        a.path_cache_step = self.current_time
                        nxt = a.path_cache.pop(0)
                    else:
                        a.path_cache = []
                        a.path_cache_goal = target
                        a.path_cache_step = self.current_time

            if nxt is None:
                nxt = self._weighted_next_step((x, y), target, agent_penalty=2.0, agent=a, avoid_cells=reserved)
            # Debug: if agent doesn't move, show why
            if nxt == (x, y) and target != (x, y) and self.debug_logging:
                neighbors = list(self._neighbors(x, y, agent=a))
                print(f"[PATH DBG] Agent {i} at {(x,y)} wants {target}, pathfinder returned {nxt}, neighbors={neighbors}")
            desired[i] = nxt
            # reserve the chosen next cell to deconflict followers in this tick
            reserved.add(nxt)

        # Resolve conflicts: multiple agents wanting same cell -> lowest id wins
        # Also prevent direct swapping: if two agents want each other's cell, lower id wins
        # Start by mapping desired cell -> agents
        cell_to_agents = {}
        for i, cell in desired.items():
            cell_to_agents.setdefault(cell, []).append(i)

        # Final next positions default to current positions
        final_next = {i: self.agent_positions[i] for i in self.agent_positions}

        # Handle collisions where multiple agents want same destination
        for cell, agents_wanting in cell_to_agents.items():
            if len(agents_wanting) == 1:
                agent = agents_wanting[0]
                # allow move only if destination isn't currently occupied by a stationary agent
                # find if someone currently occupies that cell
                occupant = None
                for aid, pos in self.agent_positions.items():
                    if pos == cell:
                        occupant = aid
                        break
                if occupant is not None:
                    # if occupant intends to stay (its desired == its current), block the move
                    if desired.get(occupant) == self.agent_positions[occupant]:
                        final_next[agent] = self.agent_positions[agent]
                    else:
                        final_next[agent] = cell
                else:
                    final_next[agent] = cell
            else:
                # tie-breaker: agent with smallest id moves, others stay
                winner = min(agents_wanting)
                final_next[winner] = cell
                for a in agents_wanting:
                    if a != winner:
                        final_next[a] = self.agent_positions[a]

        # Prevent swapping: if agent A's final is B's current and B's final is A's current,
        # then keep the higher-id agent in place.
        for a, next_a in list(final_next.items()):
            for b, next_b in list(final_next.items()):
                if a >= b:
                    continue
                if next_a == self.agent_positions[b] and next_b == self.agent_positions[a]:
                    # a < b, allow a to move, keep b in place
                    final_next[b] = self.agent_positions[b]

        # Apply moves and update agent observations
        # detect stuck agents and attempt simple detour if needed
        occupied_targets = set(final_next.values())
        for i in list(self.agent_positions.keys()):
            cur = self.agent_positions[i]
            proposed = final_next.get(i, cur)
            a = self.agents[i]
            if proposed == cur and desired.get(i) != cur:
                # agent wanted to move but was blocked
                a.stuck_count = getattr(a, 'stuck_count', 0) + 1
            else:
                a.stuck_count = 0

            if a.stuck_count >= 2 and proposed == cur:
                # try simple detour: pick a free neighbor not already targeted
                detour = None
                # prefer neighbors not recently visited by this agent to break cycles
                history = getattr(a, 'move_history', None)
                # first pass: pick neighbor not in history
                for nb in self._neighbors(*cur, agent=a):
                    if nb == cur:
                        continue
                    if nb in occupied_targets:
                        continue
                    if history is not None and nb in history:
                        continue
                    detour = nb
                    break
                # fallback: accept any free neighbor
                if detour is None:
                    for nb in self._neighbors(*cur, agent=a):
                        if nb == cur:
                            continue
                        if nb in occupied_targets:
                            continue
                        detour = nb
                        break
                if detour is not None:
                    final_next[i] = detour
                    occupied_targets.add(detour)
                    a.stuck_count = 0

        # Communication guardrails: prevent agents from stepping into isolation when possible.
        # Evaluate degrees using proposed next positions; if an agent would lose all neighbors
        # while currently connected, keep it in place for this tick.
        if self.comm_guard_enable and int(self.comm_radius) > 0 and self.comm_guard_min_degree > 0:
            # Precompute proposed adjacency using L1 distance and an effective radius minus margin
            eff_R = max(0, int(self.comm_radius) - max(0, int(self.comm_guard_margin)))
            # Build reverse map: proposed position per agent
            next_pos = {i: final_next.get(i, self.agent_positions[i]) for i in self.agent_positions}
            for i in list(self.agent_positions.keys()):
                cur = self.agent_positions[i]
                nxt = next_pos.get(i, cur)
                # current degree
                deg_now = 0
                for j, pj in self.agent_positions.items():
                    if j == i:
                        continue
                    if abs(pj[0] - cur[0]) + abs(pj[1] - cur[1]) <= eff_R:
                        deg_now += 1
                # proposed degree
                deg_next = 0
                for j, pj in next_pos.items():
                    if j == i:
                        continue
                    if abs(pj[0] - nxt[0]) + abs(pj[1] - nxt[1]) <= eff_R:
                        deg_next += 1
                # Only veto truly isolating steps: dropping from a single-link (deg_now==1) to zero
                if deg_now == 1 and deg_next < 1:
                    # keep current position to preserve at least minimal connectivity
                    final_next[i] = cur
                    # update next_pos so subsequent checks see the corrected choice
                    next_pos[i] = cur

        for i in self.agent_positions.keys():
            old_pos = self.agent_positions[i]
            self.agent_positions[i] = final_next.get(i, self.agent_positions[i])
            a = self.agents[i]
            a.pos = self.agent_positions[i]
            # track previous position for backtrack avoidance
            a.prev_pos = old_pos
            # update agent move history to prevent local cycles
            if hasattr(a, 'move_history'):
                try:
                    a.move_history.append(a.pos)
                except Exception:
                    pass
            # record last visited timestamp for the tile the agent moved into
            wx, wy = a.pos
            self.last_visited[wx, wy] = int(t)

            # Leader-blind gating: skip sensing/writeback for current leader when enabled
            is_leader = (i == getattr(self, "leader_id", None))
            if not (self.phase2_leader_blind and is_leader):
                a.observe(self, r=a.r_local if hasattr(a, "r_local") else 2)
                # Accumulate observations into team belief (monotonic union)
                cx, cy = a.pos
                R = a.r_local
                for lx in range(a.local.shape[0]):
                    for ly in range(a.local.shape[1]):
                        v = a.local[lx, ly]
                        if v != -1:  # observed (free or obstacle)
                            wx_cell = cx - R + lx
                            wy_cell = cy - R + ly
                            if 0 <= wx_cell < self.size and 0 <= wy_cell < self.size:
                                if self.team_belief[wx_cell, wy_cell] == -1:
                                    self.team_belief[wx_cell, wy_cell] = v
                a.writeback_belief_lr()  # Updated to use belief map instead of oracle global_lr

        self._record_coverage(t)


    def coverage(self):
        # Use the fine-grained team-known free map to compute coverage.
        # This avoids non-monotonic coverage when coarse downsampled cells
        # flip between free/obstacle labels as agents observe different microcells.
        known = self._team_known_free_world()
        seen_cells = int(known.sum())
        total = int((self.grid != -1).sum())
        return 1.0 if total == 0 else min(1.0, seen_cells / total)

    def _record_coverage(self, t=None):
        """Append coverage samples for offline analysis."""
        try:
            ts = int(self.current_time if t is None else t)
        except Exception:
            ts = 0
        cov = float(self.coverage())
        self.coverage_history.append((ts, cov))
        if self.coverage_history_maxlen and len(self.coverage_history) > self.coverage_history_maxlen:
            self.coverage_history = self.coverage_history[-self.coverage_history_maxlen:]
        return cov

