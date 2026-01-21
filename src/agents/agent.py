"""
Agent Module: Individual Robot Perception and Belief State

Implements dual-resolution mapping and distributed belief system where each agent
maintains its own map built from local observations and peer communications.

References:
- Yamauchi (1997): A frontier-based approach for autonomous exploration
- Burgard et al. (2000): Collaborative multi-robot exploration
- Fox et al. (2006): Distributed multirobot exploration and mapping
- Elfes (1989): Using occupancy grids for mobile robot perception
"""

import numpy as np
class Agent:
    def __init__(self, size, r_local=8, ds=4):
        self.pos = None
        self.map_size = size  # store for conservative writeback
        self.r_local = r_local # local hi resolution map radius
        self.ds = ds #downsample factor for the global map
        
        # LOCAL MAP: What I've personally observed (high-resolution)
        self.local = np.full((2*r_local+1, 2*r_local+1), -1, np.int8) #-1 is unknown, 0 is free, 2 is obst
        
        # BELIEF MAP: My estimate of the world (low-resolution)
        # Built from: (1) My local observations + (2) Maps received from neighbors
        # This replaces the "oracle" global_lr view
        self.belief_lr = np.full((size//ds, size//ds), -1, np.int8)
        
        # Fine-resolution masks to track what has ever been seen (for conservative coarse updates)
        self.seen_world = np.zeros((size, size), dtype=bool)
        self.obstacle_world = np.zeros((size, size), dtype=bool)
        
        # Track when I last communicated with each neighbor
        self.last_comm = {}  # {agent_id: timestep}
        self.last_merge_step = -1
        
        # helper to detect if agent is repeatedly blocked
        self.stuck_count = 0
        # hold assigned target for a few steps to reduce oscillation
        self.hold_target = None
        self.hold_timer = 0
        # recent move history to avoid local cycles (store last k positions)
        from collections import deque
        self.move_history = deque(maxlen=12)  # increased from 8 to remember longer loops
        # previous position to prevent immediate backtracking
        self.prev_pos = None
        # Precomputed line-of-sight rays for current sensing radius
        self._los_rays = None
        self._los_radius = None
        # Cached path to reduce replanning overhead
        self.path_cache = []
        self.path_cache_goal = None
        self.path_cache_step = -1
    
    def _world_to_local(self, wx, wy):
        cx, cy = self.pos
        #c(x,y) is denoting the center of the world RELATIVE to the agent, so basically the local coords
        return wx-(cx-self.r_local), wy-(cy-self.r_local)  
        #takes the local coords, subtractsthe observation radius to get the world coords
#this is meant to convert the world coords to local map coords
#wx is the World X- coord and "wy" is the world Y coord.. (both int)
#(lx, ly) is LOCAL x and LOCAL y indices.
#


    def _build_los_rays(self, r):
        """Precompute Bresenham rays for all (dx, dy) within radius r."""
        rays = {}
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    rays[(0, 0)] = []
                    continue
                x0, y0 = 0, 0
                x1, y1 = dx, dy
                sx = 1 if x0 < x1 else -1
                sy = 1 if y0 < y1 else -1
                ax = abs(x1 - x0)
                ay = abs(y1 - y0)
                err = ax - ay
                x, y = x0, y0
                points = []
                while True:
                    if x == x1 and y == y1:
                        break
                    e2 = 2 * err
                    if e2 > -ay:
                        err -= ay
                        x += sx
                    if e2 < ax:
                        err += ax
                        y += sy
                    # Store only intermediate points (exclude endpoint)
                    if not (x == x1 and y == y1):
                        points.append((x, y))
                rays[(dx, dy)] = points
        return rays

    def _get_los_rays(self, r):
        """Return cached LOS rays for radius r, rebuilding if needed."""
        if self._los_radius != r or self._los_rays is None:
            self._los_rays = self._build_los_rays(r)
            self._los_radius = r
        return self._los_rays

    #r = observation radius - how far the agent can see
    #env is the environment object with env.grid being the map and env.size the size
    def observe(self, env, r=1):
        """
        Observe cells within radius r using LINE-OF-SIGHT.
        
        CRITICAL FIX: Agents cannot see through walls!
        Uses Bresenham's line algorithm to check if line-of-sight is blocked.
        """
        cx, cy = self.pos
        size = env.size
        grid = env.grid
        rays = self._get_los_rays(r)
        
        #check cells w/in observation radius
        for x in range(cx-r, cx+r+1):
            for y in range(cy-r, cy+r+1):
                #is the cell within bounds?
                if 0 <= x < size and 0 <= y < size:
                    # ✅ NEW: Check line-of-sight before observing
                    dx, dy = x - cx, y - cy
                    blocked = False
                    for ox, oy in rays.get((dx, dy), ()):
                        gx, gy = cx + ox, cy + oy
                        if 0 <= gx < size and 0 <= gy < size:
                            if grid[gx, gy] == -1:  # Obstacle blocks vision
                                blocked = True
                                break
                    if blocked:
                        continue  # Can't see through walls!
                    
                    #convert to local coords
                    lx, ly = self._world_to_local(x, y)
                    # is the local cell valid w/in the local map?
                    if 0 <= lx < self.local.shape[0] and 0 <= ly < self.local.shape[1]:
                        #update grid slot on local map as either obstacle(2) or free(0)
                        is_obs = (grid[x, y] == -1)
                        self.local[lx, ly] = 2 if is_obs else 0
                        # Mark fine-resolution seen masks for conservative coarse updates
                        self.seen_world[x, y] = True
                        if is_obs:
                            self.obstacle_world[x, y] = True
    
    # Purpose: Transfer knowledge from high-res local map to low-res belief map
    def writeback_belief_lr(self, seen_threshold: float = None):
        """
        Update my belief map from my local observations conservatively.
        
        Only processes coarse cells that overlap with my current local sensing window.
        This prevents overwriting belief data received from neighbors via comms.
        
        Args:
            seen_threshold: Fraction of fine cells that must be seen to mark coarse cell free.
                           If None, uses adaptive threshold based on ds:
                           - ds=1-2: 0.9 (strict)
                           - ds=3-4: 0.7 (moderate)
                           - ds>4: 0.5 (lenient, large blocks need less coverage)
        """
        if self.pos is None:
            return
        
        cx, cy = self.pos
        ds = self.ds
        size = self.map_size
        R = self.r_local
        
        # Adaptive threshold: larger ds = lower threshold
        if seen_threshold is None:
            if ds <= 2:
                seen_threshold = 0.9
            elif ds <= 4:
                seen_threshold = 0.7
            else:
                seen_threshold = 0.5
        
        # Only process coarse cells that overlap with my current local window
        gx_min = max(0, (cx - R) // ds)
        gx_max = min(self.belief_lr.shape[0] - 1, (cx + R) // ds)
        gy_min = max(0, (cy - R) // ds)
        gy_max = min(self.belief_lr.shape[1] - 1, (cy + R) // ds)
        
        for gx in range(gx_min, gx_max + 1):
            for gy in range(gy_min, gy_max + 1):
                x0, x1 = gx * ds, min(size, (gx + 1) * ds)
                y0, y1 = gy * ds, min(size, (gy + 1) * ds)
                
                # If any obstacle has ever been seen in this coarse block, mark obstacle
                if self.obstacle_world[x0:x1, y0:y1].any():
                    self.belief_lr[gx, gy] = 2
                    continue
                
                # Mark free if enough fine cells have been seen (threshold-based)
                block = self.seen_world[x0:x1, y0:y1]
                seen_fraction = block.sum() / max(1, block.size)
                if seen_fraction >= seen_threshold:
                    if self.belief_lr[gx, gy] != 2:
                        self.belief_lr[gx, gy] = 0
                # Don't downgrade to unknown - preserve comms data
    
    def receive_map_from_neighbor(self, neighbor_belief_lr, neighbor_id, current_step):
        """
        Merge a neighbor's belief map into my belief map.
        This is the core communication protocol - agents share what they know.
        
        Args:
            neighbor_belief_lr: The neighbor's belief map (numpy array)
            neighbor_id: ID of the neighbor sending the map
            current_step: Current simulation timestep
        """
        self.last_comm[neighbor_id] = current_step
        
        for gx in range(self.belief_lr.shape[0]):
            for gy in range(self.belief_lr.shape[1]):
                neighbor_val = neighbor_belief_lr[gx, gy]
                if neighbor_val == -1:
                    continue  # Neighbor doesn't know this cell
                
                my_val = self.belief_lr[gx, gy]
                if my_val == -1:
                    # I didn't know, now I do
                    self.belief_lr[gx, gy] = neighbor_val
                elif my_val == 2 or neighbor_val == 2:
                    # Obstacle evidence wins (conservative)
                    self.belief_lr[gx, gy] = 2
                # else: Both agree it's free, keep it
    
    # # DEPRECATED: Old oracle-view function (kept for backward compatibility)
    # def writeback_global_lr(self):
    #     """DEPRECATED: Use writeback_belief_lr() instead."""
    #     cx, cy = self.pos
    #     for lx in range(self.local.shape[0]):
    #         for ly in range(self.local.shape[1]):
    #             wx, wy = (cx - self.r_local + lx), (cy - self.r_local + ly)
    #             if wx < 0 or wy < 0: continue
    #             gx, gy = wx // self.ds, wy // self.ds
    #             if gx < 0 or gy < 0: continue
    #             # Check both belief_lr and global_lr for backward compatibility
    #             if hasattr(self, 'global_lr') and gx < self.global_lr.shape[0] and gy < self.global_lr.shape[1]:
    #                 v = self.local[lx, ly]
    #                 if v == -1: continue
    #                 if v == 2 or self.global_lr[gx, gy] == -1:
    #                     self.global_lr[gx, gy] = v
    #             elif v == 0 and self.global_lr[gx, gy] != 2:
    #                 self.global_lr[gx, gy] = 0
        
                                 
                                 #Use an agent coordiation system right here, like work on extra



