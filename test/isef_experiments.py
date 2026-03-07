#!/usr/bin/env python3
"""
ISEF Experimental Data Collection — Rigorous Comparison Suite

Runs systematic experiments comparing REIP vs RAFT vs Decentralized baselines
under various fault conditions with statistical rigor:
  - N repeated trials per condition (randomized starting positions)
  - Real-time fault detection timing (tracks leader changes in broadcasts)
  - Two arena layouts (open + multiroom) for environmental diversity
  - Statistical summary: mean +/- std for all metrics

Usage:
  python test/isef_experiments.py                    # Full suite (all layouts, N=10)
  python test/isef_experiments.py --layout open      # Single layout
  python test/isef_experiments.py --trials 5         # Fewer trials (faster)
  python test/isef_experiments.py --quick            # Quick check: N=3, open only
"""

import subprocess
import time
import json
import socket
import os
import sys
import math
import random
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple
from collections import defaultdict
import heapq

# ==================== Configuration ====================
REIP_SCRIPT = "robot/reip_node.py"
RAFT_SCRIPT = "robot/baselines/raft_node.py"
NUM_ROBOTS = 5
EXPERIMENT_DURATION = 120   # seconds per trial
FAULT_INJECT_TIME = 10      # first fault injection (earlier = more impact)
FAULT_INJECT_TIME_2 = 30    # second bad_leader injection on current leader
TRIALS_PER_CONDITION = 10   # repeated trials for statistics

# After initial leader election settles (~5s), leader changes before
# fault injection count as false positives.
SETTLE_TIME = 5

UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300

ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500
CELL_SIZE = 125

# Each layout: { "walls": [...], "width": W, "height": H, "start_zone": (x0,x1,y0,y1) }
# width/height default to ARENA_WIDTH/ARENA_HEIGHT if omitted.
# start_zone defines where robots spawn (defaults to full arena).
ARENA_LAYOUTS = {
    "open": [],
    "multiroom": [
        (1000, 0, 1000, 1200),  # Vertical wall, passage at top
    ],
    # ---- Scaled complexity layouts (sim-only, post hardware validation) ----
    # Same arena size as hardware, but L-shaped wall creates 3 zones
    # Tests sustained coordination demand in matched dimensions
    "complex_L": [
        (1000, 0, 1000, 1200),     # Vertical wall (same as multiroom)
        (1000, 600, 1600, 600),     # Horizontal extension creating alcove
    ],
    # Larger arena: 4x area, 4 rooms with connecting passages
    # Tests REIP scalability beyond hardware testbed
    # Note: uses custom dimensions (4000x3000), passed to robot nodes
    "large_office": [
        # Central vertical wall (passage at y=1200-1500 and y=2700-3000)
        (2000, 0, 2000, 1200),      # Lower vertical segment
        (2000, 1500, 2000, 2700),   # Upper vertical segment
        # Left horizontal wall (passage at x=800-1200)
        (0, 1500, 800, 1500),       # Left segment
        (1200, 1500, 2000, 1500),   # Right segment up to center
        # Right horizontal wall (passage at x=2800-3200)
        (2000, 1500, 2800, 1500),   # Left segment from center
        (3200, 1500, 4000, 1500),   # Right segment
    ],
}

# Per-layout arena dimensions (overrides for scaled layouts)
ARENA_DIMENSIONS = {
    "large_office": (4000, 3000),
}

# Per-layout starting zones: (x_min, x_max, y_min, y_max)
ARENA_START_ZONES = {
    "multiroom": (150, 850, 150, 1350),       # Room A (left of wall)
    "complex_L": (150, 850, 150, 1350),        # Room A (left of wall)
    "large_office": (150, 1800, 150, 1350),    # Lower-left quadrant (Room 1)
}

# ==================== Data Classes ====================
@dataclass
class ExperimentResult:
    name: str
    controller: str           # 'reip', 'raft', 'decentralized'
    fault_type: Optional[str]
    fault_robot: Optional[int]
    layout: str
    trial: int
    seed: int
    final_coverage: float
    time_to_50: Optional[float]            # seconds to reach 50% coverage
    time_to_60: Optional[float]            # seconds to reach 60% coverage
    time_to_80: Optional[float]            # seconds to reach 80% coverage
    time_to_detection: Optional[float]     # seconds from fault 1 to first impeachment
    time_to_first_suspicion: Optional[float]  # seconds from fault 1 to first trust decay
    time_to_detection_2: Optional[float]   # seconds from fault 2 to second impeachment
    num_leader_changes: int
    false_positives: int                   # leader changes between settle and fault
    coverage_at_90s: Optional[float]       # coverage at 90s mark
    coverage_timeline: List[Tuple[float, float]] = field(default_factory=list)
    ablation: Optional[str] = None        # 'no_trust', 'no_causality', 'no_direction', or None


# ==================== Experiment Runner ====================
class ExperimentRunner:
    def __init__(self, output_dir: str = "experiments", port_offset: int = 0):
        self._fixed_positions_for_next_trial = None
        self.output_dir = output_dir
        self.layout = "open"
        self.walls: List[Tuple[int, int, int, int]] = []
        self.port_offset = port_offset  # For parallel execution
        os.makedirs(output_dir, exist_ok=True)

        # Port bases (offset for parallel workers)
        self._pos_port = UDP_POSITION_PORT + port_offset
        self._peer_port = UDP_PEER_PORT + port_offset
        self._fault_port = UDP_FAULT_PORT + port_offset

        # Sockets
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Sim traffic is explicitly sent to 127.0.0.1. Bind to loopback
        # directly so localhost peer-state packets are captured reliably.
        self.state_socket.bind(('127.0.0.1', self._peer_port))
        self.state_socket.setblocking(False)
        self.state_socket.settimeout(0.1)

        self.robot_processes: List[subprocess.Popen] = []
        self.sim_positions: Dict[int, Tuple[float, float, float]] = {}
        self.stuck_counters: Dict[int, int] = {}
        self.prev_positions: Dict[int, Tuple[float, float]] = {}
        self.stuck_overrides: Dict[int, Optional[list]] = {}  # locked override targets
        self.motor_faults: Dict[int, Optional[str]] = {}  # rid -> 'spin', 'stop', etc.
        self.path_waypoints: Dict[int, list] = {}       # rid -> [(x,y), ...] waypoints
        self.path_target_key: Dict[int, tuple] = {}     # rid -> (tx,ty) target that generated the path

        # Per-experiment state (reset each trial)
        self.visited_cells: set = set()
        self.coverage_history: List[Tuple[float, float]] = []
        self.tracked_leaders: Dict[int, Optional[int]] = {}  # rid -> leader they report
        self.last_targets: Dict[int, Optional[list]] = {}  # rid -> last known nav target
        self.last_states_by_robot: Dict[int, dict] = {}

    def set_layout(self, layout: str):
        """Switch arena layout between experiments."""
        self.layout = layout
        self.walls = list(ARENA_LAYOUTS.get(layout, []))
        # Per-layout arena dimensions (for scaled layouts)
        dims = ARENA_DIMENSIONS.get(layout)
        if dims:
            self.arena_width, self.arena_height = dims
        else:
            self.arena_width, self.arena_height = ARENA_WIDTH, ARENA_HEIGHT

    # -------------------- Process Management --------------------
    def start_robots(self, script: str, extra_args: List[str] = None,
                     seed: int = 42, experiment_name: str = "",
                     fixed_positions: Dict[int, Tuple[float, float, float]] = None):
        """Start robot processes with seed-based random starting positions."""
        # Use per-experiment log directory to avoid overwriting
        if experiment_name:
            log_dir = os.path.join(self.output_dir, "logs", experiment_name)
        else:
            log_dir = os.path.join(self.output_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)
        self._current_log_dir = log_dir  # Save for log parsing

        rng = random.Random(seed)

        for i in range(1, NUM_ROBOTS + 1):
            log_file = open(os.path.join(log_dir, f"robot_{i}.log"), 'w')
            cmd = [sys.executable, "-u", script, str(i), "--sim"]
            if self.port_offset:
                cmd.extend(["--port-base", str(self.port_offset)])
            if extra_args:
                cmd.extend(extra_args)
            proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            )
            self.robot_processes.append(proc)

            # Starting position: use fixed (hardware) positions if provided,
            # otherwise randomize (seed-controlled for reproducibility).
            if fixed_positions and i in fixed_positions:
                sx, sy, theta = fixed_positions[i]
            else:
                start_zone = ARENA_START_ZONES.get(self.layout)
                if start_zone:
                    sx = rng.uniform(start_zone[0], start_zone[1])
                    sy = rng.uniform(start_zone[2], start_zone[3])
                else:
                    sx = rng.uniform(200, self.arena_width - 200)
                    sy = rng.uniform(200, self.arena_height - 200)
                theta = rng.uniform(0, 2 * math.pi)

                # Reject positions inside walls
                while self._collides_with_wall(sx, sy):
                    if start_zone:
                        sx = rng.uniform(start_zone[0], start_zone[1])
                        sy = rng.uniform(start_zone[2], start_zone[3])
                    else:
                        sx = rng.uniform(200, self.arena_width - 200)
                        sy = rng.uniform(200, self.arena_height - 200)

            self.sim_positions[i] = (sx, sy, theta)
            self.stuck_counters[i] = 0
            self.prev_positions[i] = (sx, sy)

        time.sleep(2)  # Let processes initialize

    def stop_robots(self):
        """Stop all robot processes"""
        for proc in self.robot_processes:
            proc.terminate()
        for proc in self.robot_processes:
            try:
                proc.wait(timeout=2)
            except Exception:
                proc.kill()
        self.robot_processes = []

    # -------------------- Communication --------------------
    def send_positions(self):
        """Send mock positions to robots"""
        for rid, (x, y, theta) in self.sim_positions.items():
            msg = {
                'type': 'position',
                'robot_id': rid,
                'x': x, 'y': y, 'theta': theta,
                'timestamp': time.time()
            }
            self.pos_socket.sendto(
                json.dumps(msg).encode(),
                ('127.0.0.1', self._pos_port + rid)
            )

    def inject_fault(self, robot_id: int, fault_type: str):
        """Inject fault into robot and track motor faults locally for physics."""
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        # Track motor faults so update_positions can simulate them
        if fault_type in ('spin', 'stop', 'erratic'):
            self.motor_faults[robot_id] = fault_type
        elif fault_type in ('none', 'clear'):
            self.motor_faults[robot_id] = None
        # bad_leader, oscillate_leader, freeze_leader are NOT motor faults — robot still moves normally
        self.fault_socket.sendto(
            json.dumps(msg).encode(),
            ('127.0.0.1', self._fault_port + robot_id)
        )

    def receive_states(self) -> Dict[int, dict]:
        """Receive state broadcasts from robots.

        In sim mode each robot sends directly to every other robot's
        unique peer port.  They also send to the base port (5200) so
        the harness can observe.
        """
        states = {}
        try:
            while True:
                data, _ = self.state_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        states[rid] = msg
                        self.last_states_by_robot[rid] = msg
                        for cell in msg.get('visited_cells', []):
                            self.visited_cells.add(tuple(cell))
        except (socket.timeout, BlockingIOError, ConnectionResetError, OSError):
            pass
        return states

    # -------------------- Physics / Wall Collision --------------------
    def _line_circle_collision(self, x1, y1, x2, y2, cx, cy, r):
        """Check if circle (cx,cy,r) overlaps line segment (x1,y1)-(x2,y2).
        Tangent contact (distance == radius) is NOT collision."""
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy
        a = dx * dx + dy * dy
        if a < 1e-6:
            return math.hypot(cx - x1, cy - y1) < r
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4 * a * c
        if disc <= 0:
            return False
        disc = math.sqrt(disc)
        t1 = (-b - disc) / (2 * a)
        t2 = (-b + disc) / (2 * a)
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)

    def _collides_with_wall(self, x, y, radius=75):
        for wx1, wy1, wx2, wy2 in self.walls:
            if self._line_circle_collision(wx1, wy1, wx2, wy2, x, y, radius):
                return True
        return False

    @staticmethod
    def _ccw(ax, ay, bx, by, cx, cy):
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

    def _path_crosses_wall(self, x1, y1, x2, y2):
        """Check if direct path crosses any wall segment."""
        for wx1, wy1, wx2, wy2 in self.walls:
            d1 = self._ccw(x1, y1, x2, y2, wx1, wy1)
            d2 = self._ccw(x1, y1, x2, y2, wx2, wy2)
            d3 = self._ccw(wx1, wy1, wx2, wy2, x1, y1)
            d4 = self._ccw(wx1, wy1, wx2, wy2, x2, y2)
            if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
               ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
                return True
        return False

    def _find_reachable_unexplored(self, x, y):
        """Find nearest unexplored cell reachable via BFS (path distance, not Euclidean)."""
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(x / CELL_SIZE))),
              max(0, min(rows - 1, int(y / CELL_SIZE))))

        # BFS from robot cell — returns cells in order of path distance
        from collections import deque
        queue = deque([sc])
        visited_bfs = {sc}
        while queue:
            cx, cy = queue.popleft()
            if (cx, cy) not in self.visited_cells:
                tx = (cx + 0.5) * CELL_SIZE
                ty = (cy + 0.5) * CELL_SIZE
                return (tx, ty)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and (nx, ny) not in visited_bfs
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    visited_bfs.add((nx, ny))
                    queue.append((nx, ny))

        # Absolute fallback — pick any unexplored cell
        for cx in range(cols):
            for cy in range(rows):
                if (cx, cy) not in self.visited_cells:
                    return ((cx + 0.5) * CELL_SIZE, (cy + 0.5) * CELL_SIZE)
        return None

    # -------------------- A* Pathfinding on Cell Grid --------------------
    def _wall_blocks_move(self, cx, cy, nx, ny):
        """Check if moving between adjacent cells crosses a wall.
        Walls are zero-thickness line segments; cells are always passable
        but movement BETWEEN cells may be blocked."""
        ax = (cx + 0.5) * CELL_SIZE
        ay = (cy + 0.5) * CELL_SIZE
        bx = (nx + 0.5) * CELL_SIZE
        by = (ny + 0.5) * CELL_SIZE
        return self._path_crosses_wall(ax, ay, bx, by)

    def _build_nav_grid(self):
        """Return grid dimensions.  All cells are passable (walls are
        zero-thickness and block movement between cells, not cells
        themselves).  Use _wall_blocks_move() for adjacency checks."""
        cols = self.arena_width // CELL_SIZE
        rows = self.arena_height // CELL_SIZE
        return cols, rows

    def _astar_cell(self, start_cell, goal_cell, cols, rows):
        """A* on the cell grid.  Returns list of (cx, cy) from start to goal
        (inclusive), or None if no path exists.  Uses _wall_blocks_move()
        for adjacency instead of a passability grid."""
        if start_cell == goal_cell:
            return [start_cell]
        sx, sy = start_cell
        gx, gy = goal_cell

        if not (0 <= sx < cols and 0 <= sy < rows):
            return None
        if not (0 <= gx < cols and 0 <= gy < rows):
            return None

        open_set = [(abs(gx - sx) + abs(gy - sy), 0, sx, sy)]
        came_from = {}
        g_score = {start_cell: 0}

        while open_set:
            _, g, cx, cy = heapq.heappop(open_set)
            if (cx, cy) == goal_cell:
                # Reconstruct
                path = []
                cur = goal_cell
                while cur is not None:
                    path.append(cur)
                    cur = came_from.get(cur)
                path.reverse()
                return path
            if g > g_score.get((cx, cy), float('inf')):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < cols and 0 <= ny < rows
                        and not self._wall_blocks_move(cx, cy, nx, ny)):
                    ng = g + 1
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        h = abs(gx - nx) + abs(gy - ny)
                        heapq.heappush(open_set, (ng + h, ng, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)
        return None  # no path

    def _get_path_waypoints(self, x, y, target_x, target_y):
        """Return a list of world-coordinate waypoints from (x,y) to
        (target_x, target_y), routed around walls via A*."""
        cols, rows = self._build_nav_grid()
        sc = (max(0, min(cols - 1, int(x / CELL_SIZE))),
              max(0, min(rows - 1, int(y / CELL_SIZE))))
        gc = (max(0, min(cols - 1, int(target_x / CELL_SIZE))),
              max(0, min(rows - 1, int(target_y / CELL_SIZE))))

        path = self._astar_cell(sc, gc, cols, rows)
        if path is None or len(path) <= 1:
            # Fallback: drive direct (existing behaviour)
            return [(target_x, target_y)]

        # Convert cell path to world-coordinate waypoints (cell centres),
        # but skip the first cell (robot is already there) and use the
        # actual target position for the last waypoint.
        raw = []
        for i, (cx, cy) in enumerate(path[1:], 1):
            if i == len(path) - 1:
                raw.append((target_x, target_y))
            else:
                raw.append(((cx + 0.5) * CELL_SIZE,
                            (cy + 0.5) * CELL_SIZE))

        # ---- Line-of-sight smoothing: skip intermediate waypoints ----
        # If the robot can reach waypoint i+2 without crossing a wall,
        # drop waypoint i+1.  This makes paths much shorter.
        if len(raw) <= 2:
            return raw
        smoothed = [raw[0]]
        i = 0
        while i < len(raw) - 1:
            # Try to skip ahead as far as possible
            farthest = i + 1
            for j in range(i + 2, len(raw)):
                if not self._path_crosses_wall(smoothed[-1][0], smoothed[-1][1],
                                                raw[j][0], raw[j][1]):
                    farthest = j
            smoothed.append(raw[farthest])
            i = farthest
        return smoothed

    # -------------------- Movement Simulation --------------------
    def update_positions(self, states: Dict[int, dict], dt: float):
        """Update simulated positions based on each robot's decided target.

        Uses navigation_target from each robot's broadcast — this is the
        position the robot actually chose to go to (leader assignment if
        trusting, local fallback if not).  Includes wall collision,
        wall-following, and stuck detection with locked override.

        Movement is TIME-BASED: robots move at SPEED_PER_SEC * dt pixels per
        call, ensuring consistent behavior regardless of tick rate / CPU load.
        """
        SPEED_PER_SEC = 500       # pixels/sec (~robot body length per second)
        STUCK_TIME = 1.5          # seconds stuck before override fires
        STUCK_MOVE_EPS = 15       # pixels — less than this = stuck
        OVERRIDE_ARRIVE_DIST = 60 # close enough to release override lock
        WALL_RADIUS = 75          # collision radius used in _collides_with_wall
        MAX_STEP = WALL_RADIUS * 0.8  # Max step per sub-tick to prevent tunneling

        total_move = SPEED_PER_SEC * dt  # total pixels to move this tick

        for rid in range(1, NUM_ROBOTS + 1):
            x, y, theta = self.sim_positions[rid]

            # ---- Motor fault simulation (matches visual_sim.py) ----
            motor_fault = self.motor_faults.get(rid)
            if motor_fault == 'stop':
                # Robot is frozen — don't move, don't count coverage
                continue
            elif motor_fault == 'spin':
                # Robot spins in place — only theta changes, no position change
                theta += 0.2 * (dt / 0.05)  # scale by dt so rotation is time-consistent
                self.sim_positions[rid] = (x, y, theta)
                # Still count the cell it's sitting on
                cell = (int(x / CELL_SIZE), int(y / CELL_SIZE))
                self.visited_cells.add(cell)
                continue
            elif motor_fault == 'erratic':
                # Random jitter
                x += random.uniform(-20, 20) * (dt / 0.05)
                y += random.uniform(-20, 20) * (dt / 0.05)
                theta += random.uniform(-0.3, 0.3) * (dt / 0.05)
                x = max(75, min(self.arena_width - 75, x))
                y = max(75, min(self.arena_height - 75, y))
                self.sim_positions[rid] = (x, y, theta)
                cell = (int(x / CELL_SIZE), int(y / CELL_SIZE))
                self.visited_cells.add(cell)
                continue

            # ---- Stuck detection (time-based) ----
            px, py = self.prev_positions.get(rid, (x, y))
            move_since = math.sqrt((x - px)**2 + (y - py)**2)
            if move_since < STUCK_MOVE_EPS:
                self.stuck_counters[rid] = self.stuck_counters.get(rid, 0) + dt
            else:
                self.stuck_counters[rid] = 0
                self.prev_positions[rid] = (x, y)

            # ---- If there's an active stuck override, check if robot arrived ----
            override = self.stuck_overrides.get(rid)
            if override:
                od = math.sqrt((x - override[0])**2 + (y - override[1])**2)
                if od < OVERRIDE_ARRIVE_DIST:
                    self.stuck_overrides[rid] = None
                    self.stuck_counters[rid] = 0

            # ---- Accept robot's navigation target (unless override is active) ----
            if not self.stuck_overrides.get(rid):
                current_state = states.get(rid) or self.last_states_by_robot.get(rid)
                if current_state:
                    nav = current_state.get('navigation_target')
                    if nav:
                        self.last_targets[rid] = nav

                # Fallback: use leader assignments if robot hasn't reported a target yet
                if rid not in self.last_targets or self.last_targets[rid] is None:
                    state_pool = list(states.values()) or list(self.last_states_by_robot.values())
                    for state in state_pool:
                        if state.get('state') == 'leader':
                            assignments = state.get('assignments', {})
                            if str(rid) in assignments:
                                self.last_targets[rid] = assignments[str(rid)]
                                break

            target = self.last_targets.get(rid)

            # ---- Override target if stuck for too long ----
            # IMPORTANT: Only fire when robot is FAR from its target but can't
            # move (wall wedge).  If the robot arrived at its target and is
            # sitting there, that's not "stuck" — it successfully followed its
            # command.  On real hardware a robot told to go to an explored cell
            # would just stay there; only REIP's trust model can recover.
            # Without this guard, stuck detection acts as an oracle that rescues
            # Raft followers from bad leader assignments.
            target_dist = (math.sqrt((target[0] - x)**2 + (target[1] - y)**2)
                           if target else 0)
            if (self.stuck_counters.get(rid, 0) >= STUCK_TIME
                    and target_dist > OVERRIDE_ARRIVE_DIST):
                alt = self._find_reachable_unexplored(x, y)
                if alt:
                    target = alt
                    self.last_targets[rid] = alt
                    self.stuck_overrides[rid] = alt
                self.stuck_counters[rid] = 0
                self.prev_positions[rid] = (x, y)

            if target:
                # ---- A* waypoint navigation ----
                # If the final target changed, recompute the path
                tkey = (round(target[0], 1), round(target[1], 1))
                if tkey != self.path_target_key.get(rid):
                    wps = self._get_path_waypoints(x, y, target[0], target[1])
                    self.path_waypoints[rid] = wps
                    self.path_target_key[rid] = tkey

                # Current waypoint to chase (first in list)
                wps = self.path_waypoints.get(rid) or [(target[0], target[1])]

                # Sub-step to prevent wall tunneling: break large moves
                # into steps no bigger than MAX_STEP (< collision radius)
                remaining = total_move
                while remaining > 0.5:
                    step = min(remaining, MAX_STEP)
                    remaining -= step

                    # Advance through waypoints as we reach them
                    while len(wps) > 1:
                        wd = math.sqrt((wps[0][0] - x)**2 + (wps[0][1] - y)**2)
                        if wd < CELL_SIZE * 0.6:  # close enough to waypoint
                            wps.pop(0)
                        else:
                            break

                    wp = wps[0]
                    dx = wp[0] - x
                    dy = wp[1] - y
                    dist = math.sqrt(dx*dx + dy*dy)

                    if dist <= 20 and len(wps) <= 1:
                        # Arrived — clear stale target so we pick up the
                        # robot's next broadcast target instead of looping
                        # on the same arrived position forever.
                        self.last_targets[rid] = None
                        self.path_target_key[rid] = None
                        break  # arrived at final target

                    if dist <= 1:
                        # On top of waypoint but more remain
                        if len(wps) > 1:
                            wps.pop(0)
                        continue

                    theta = math.atan2(dy, dx)
                    new_x = x + step * math.cos(theta)
                    new_y = y + step * math.sin(theta)

                    if self._collides_with_wall(new_x, new_y):
                        moved = False
                        if not self._collides_with_wall(new_x, y):
                            x = new_x
                            moved = True
                        if not self._collides_with_wall(x, new_y) and abs(new_y - y) > 0.5:
                            y = new_y
                            moved = True
                        if not moved:
                            perp1 = theta + math.pi / 2
                            perp2 = theta - math.pi / 2
                            options = []
                            for perp in [perp1, perp2]:
                                tx = x + step * math.cos(perp)
                                ty = y + step * math.sin(perp)
                                if (not self._collides_with_wall(tx, ty) and
                                        75 <= tx <= self.arena_width - 75 and
                                        75 <= ty <= self.arena_height - 75):
                                    d = math.sqrt((tx - wp[0])**2 +
                                                  (ty - wp[1])**2)
                                    options.append((d, tx, ty))
                            if options:
                                options.sort()
                                _, x, y = options[0]
                            else:
                                break  # completely blocked, stop sub-stepping
                    else:
                        x = new_x
                        y = new_y

                # Update stored waypoints
                self.path_waypoints[rid] = wps

            # Clamp
            x = max(75, min(self.arena_width - 75, x))
            y = max(75, min(self.arena_height - 75, y))

            self.sim_positions[rid] = (x, y, theta)

            # Track coverage using the same center-cell semantics as the
            # robot nodes. The node broadcasts its own visited_cells too, so
            # this is just a local mirror to keep milestone timing aligned.
            cell = (int(x / CELL_SIZE), int(y / CELL_SIZE))
            self.visited_cells.add(cell)

    def get_coverage(self) -> float:
        """Calculate current coverage percentage"""
        total_cells = (self.arena_width // CELL_SIZE) * (self.arena_height // CELL_SIZE)
        return len(self.visited_cells) / total_cells * 100

    # -------------------- Current Leader Detection --------------------
    def _get_current_leader(self) -> int:
        """Find the most commonly reported leader from tracked states."""
        from collections import Counter
        if not self.tracked_leaders:
            return 1
        counts = Counter(v for v in self.tracked_leaders.values() if v is not None)
        return counts.most_common(1)[0][0] if counts else 1

    # -------------------- Run Single Experiment --------------------
    def run_experiment(
        self,
        name: str,
        controller: str,
        fault_type: Optional[str] = None,
        fault_robot: int = 1,
        trial_num: int = 1,
        seed: int = 42,
        save_snapshots: bool = False,
        ablation: Optional[str] = None,
    ) -> ExperimentResult:
        """Run a single experiment with real-time leader-change detection."""
        print(f"\n{'='*60}")
        print(f"Experiment: {name}  (trial {trial_num}, seed {seed})")
        print(f"  Controller: {controller}, Layout: {self.layout}")
        print(f"  Fault: {fault_type or 'none'} on Robot {fault_robot}")
        if ablation:
            print(f"  Ablation: {ablation}")
        print(f"{'='*60}")

        # ---- Reset per-trial state ----
        self.visited_cells = set()
        self.coverage_history = []
        self.tracked_leaders = {}
        self.stuck_counters = {}
        self.prev_positions = {}
        self.last_targets = {}
        self.last_states_by_robot = {}
        self.stuck_overrides = {}
        self.motor_faults = {}  # Reset motor faults each trial
        snapshot_frames = []  # Position snapshots for gridworld viz

        # Flush buffered socket data
        try:
            self.state_socket.setblocking(False)
            while True:
                self.state_socket.recvfrom(8192)
        except (BlockingIOError, ConnectionResetError):
            pass
        self.state_socket.settimeout(0.1)

        # ---- Choose script ----
        extra_args = []
        if controller == 'decentralized':
            script = REIP_SCRIPT
            extra_args = ["--decentralized"]
        elif controller == 'raft':
            script = RAFT_SCRIPT
        else:
            script = REIP_SCRIPT

        # Ablation flags (only applies to REIP controller)
        if ablation and controller == 'reip':
            extra_args.extend(["--ablation", ablation])

        # Pass arena dimensions for scaled layouts
        if self.layout in ARENA_DIMENSIONS:
            extra_args.extend(["--arena-width", str(self.arena_width),
                               "--arena-height", str(self.arena_height)])

        # Use fixed positions if provided (for hardware validation)
        fixed_pos = getattr(self, '_fixed_positions_for_next_trial', None)
        self.start_robots(script, extra_args, seed=seed, experiment_name=name, fixed_positions=fixed_pos)
        self._fixed_positions_for_next_trial = None  # Clear after use

        # Give nodes a moment to initialize, then send "start" so
        # trial_started gates open and motors/targets become active.
        time.sleep(0.5)
        for rid in range(1, NUM_ROBOTS + 1):
            start_msg = json.dumps({
                'type': 'fault_inject', 'robot_id': 0,
                'fault': 'start', 'timestamp': time.time()
            }).encode()
            self.fault_socket.sendto(
                start_msg, ('127.0.0.1', self._fault_port + rid))
        time.sleep(0.2)
        # Send twice to compensate for any startup packet loss
        for rid in range(1, NUM_ROBOTS + 1):
            start_msg = json.dumps({
                'type': 'fault_inject', 'robot_id': 0,
                'fault': 'start', 'timestamp': time.time()
            }).encode()
            self.fault_socket.sendto(
                start_msg, ('127.0.0.1', self._fault_port + rid))

        # ---- Tracking variables ----
        start_time = time.time()
        fault_injected = False
        fault_inject_actual_time = None
        fault2_injected = False           # Second bad_leader injection
        fault2_inject_time = None
        fault2_robot = None               # Who got the second fault
        time_to_detection = None          # Fault -> leader change (impeachment)
        time_to_first_suspicion = None    # Fault -> first trust decay
        time_to_detection_2 = None        # Second fault -> second impeachment
        time_to_50 = None
        time_to_60 = None
        time_to_80 = None
        coverage_at_90s = None
        num_leader_changes = 0
        false_positives = 0
        last_print_sec = -1
        # Track pre-fault trust levels to detect first decay
        pre_fault_trust: Dict[int, float] = {}  # rid -> trust at fault time
        last_states: Dict[int, dict] = {}  # most recent state per robot

        self._last_tick_time = time.time()
        try:
            while time.time() - start_time < EXPERIMENT_DURATION:
                elapsed = time.time() - start_time

                # ---- Inject fault at scheduled time ----
                if fault_type and not fault_injected and elapsed >= FAULT_INJECT_TIME:
                    print(f"  [t={elapsed:.1f}s] Injecting {fault_type} on Robot {fault_robot}")
                    self.inject_fault(fault_robot, fault_type)
                    fault_injected = True
                    fault_inject_actual_time = time.time()
                    # Snapshot trust levels at fault injection for decay detection
                    for rid, st in last_states.items():
                        pre_fault_trust[rid] = st.get('trust_in_leader', 1.0)

                # ---- Second leadership fault injection at FAULT_INJECT_TIME_2 ----
                # Targets whoever is currently leader (tests repeated attack survival)
                if (fault_type in ('bad_leader', 'freeze_leader') and fault_injected
                        and not fault2_injected and elapsed >= FAULT_INJECT_TIME_2):
                    current_leader = self._get_current_leader()
                    fault2_robot = current_leader
                    if current_leader != fault_robot or controller == 'raft':
                        # REIP: new leader after impeachment; RAFT: same robot
                        print(f"  [t={elapsed:.1f}s] SECOND FAULT: {fault_type} on "
                              f"Robot {current_leader} (current leader)")
                        self.inject_fault(current_leader, fault_type)
                    else:
                        print(f"  [t={elapsed:.1f}s] SECOND FAULT: Robot {fault_robot} "
                              f"still leader (re-injecting {fault_type})")
                        self.inject_fault(fault_robot, fault_type)
                        fault2_robot = fault_robot
                    fault2_injected = True
                    fault2_inject_time = time.time()

                # ---- Simulation step (time-based movement) ----
                now = time.time()
                dt = now - self._last_tick_time if hasattr(self, '_last_tick_time') else 0.05
                dt = min(dt, 0.15)  # Cap dt to prevent huge jumps after stalls
                self._last_tick_time = now

                self.send_positions()
                states = self.receive_states()
                self.update_positions(states, dt)

                # ---- Track leader changes in real time ----
                for rid, st in states.items():
                    reported_leader = st.get('leader_id')
                    prev_leader = self.tracked_leaders.get(rid)

                    if prev_leader is not None and reported_leader != prev_leader:
                        num_leader_changes += 1

                        # Was the first fault robot deposed?
                        if fault_injected and prev_leader == fault_robot and \
                                time_to_detection is None:
                            time_to_detection = time.time() - fault_inject_actual_time
                            print(f"  [t={elapsed:.1f}s] IMPEACHED: Robot {rid} saw "
                                  f"leader change {prev_leader}->{reported_leader} "
                                  f"({time_to_detection:.2f}s after fault 1)")

                        # Was the second fault robot deposed?
                        if (fault2_injected and fault2_robot is not None
                                and prev_leader == fault2_robot
                                and time_to_detection_2 is None
                                and fault2_inject_time is not None):
                            time_to_detection_2 = time.time() - fault2_inject_time
                            print(f"  [t={elapsed:.1f}s] IMPEACHED (2nd): Robot {rid} saw "
                                  f"leader change {prev_leader}->{reported_leader} "
                                  f"({time_to_detection_2:.2f}s after fault 2)")

                        # Leader change before fault = potential false positive
                        # (only count after initial election settles)
                        if not fault_injected and elapsed > SETTLE_TIME:
                            false_positives += 1

                    self.tracked_leaders[rid] = reported_leader
                    last_states[rid] = st  # Keep latest state for trust tracking

                    # ---- Detect first trust decay (suspicion) ----
                    if fault_injected and time_to_first_suspicion is None:
                        trust = st.get('trust_in_leader', 1.0)
                        suspicion = st.get('suspicion', 0.0)
                        baseline = pre_fault_trust.get(rid, 1.0)
                        # Trust has dropped noticeably, or suspicion is accumulating
                        if trust < baseline - 0.05 or suspicion > 0.5:
                            time_to_first_suspicion = time.time() - fault_inject_actual_time
                            print(f"  [t={elapsed:.1f}s] FIRST SUSPICION: Robot {rid} "
                                  f"trust={trust:.2f} sus={suspicion:.2f} "
                                  f"({time_to_first_suspicion:.2f}s after fault)")

                # ---- Coverage milestones ----
                coverage = self.get_coverage()
                self.coverage_history.append((elapsed, coverage))

                if time_to_50 is None and coverage >= 50.0:
                    time_to_50 = elapsed
                if time_to_60 is None and coverage >= 60.0:
                    time_to_60 = elapsed
                if time_to_80 is None and coverage >= 80.0:
                    time_to_80 = elapsed

                if coverage_at_90s is None and elapsed >= 90.0:
                    coverage_at_90s = coverage

                # ---- Progress print (every 15s) ----
                sec = int(elapsed)
                if sec % 15 == 0 and sec != last_print_sec and sec > 0:
                    det_str = f", det={time_to_detection:.1f}s" if time_to_detection else ""
                    sus_str = f", sus={time_to_first_suspicion:.1f}s" \
                        if time_to_first_suspicion else ""
                    print(f"  [t={sec}s] Coverage: {coverage:.1f}%{sus_str}{det_str}")
                    last_print_sec = sec

                # ---- Snapshot logging (for gridworld viz) ----
                if save_snapshots and int(elapsed * 4) != getattr(self, '_last_snap', -1):
                    # 4 Hz snapshot rate (every 0.25s)
                    self._last_snap = int(elapsed * 4)
                    robots_snap = {}
                    for rid, (rx, ry, rtheta) in self.sim_positions.items():
                        st = last_states.get(rid, {})
                        # Next A* waypoint = actual direction (around walls)
                        wps = self.path_waypoints.get(rid)
                        next_wp = list(wps[0]) if wps else None
                        robots_snap[str(rid)] = {
                            'x': round(rx, 1), 'y': round(ry, 1),
                            'theta': round(rtheta, 3),
                            'state': st.get('state', 'unknown'),
                            'leader_id': st.get('leader_id'),
                            'target': st.get('navigation_target'),
                            'predicted_target': st.get('predicted_target'),
                            'commanded_target': st.get('commanded_target'),
                            'next_waypoint': next_wp,
                            'trust': round(st.get('trust_in_leader', 1.0), 3),
                            'suspicion': round(st.get('suspicion', 0.0), 3),
                            'omega': round(st.get('omega', 0.0), 4),
                            'classified_fault': st.get('classified_fault'),
                            'motor_fault': self.motor_faults.get(rid),
                        }
                    snapshot_frames.append({
                        't': round(elapsed, 2),
                        'coverage': round(coverage, 2),
                        'visited_cells': [list(c) for c in self.visited_cells],
                        'robots': robots_snap,
                    })

                time.sleep(0.05)  # 20 Hz

        finally:
            self.stop_robots()

        # ---- Also parse robot logs for precise first_decay_time ----
        # Robot logs contain [DETECT] messages with exact timestamps
        if fault_type and fault_inject_actual_time:
            log_suspicion, log_impeachment = self._parse_robot_logs(
                fault_inject_actual_time)
            # Prefer log-based timing if available (more precise)
            if log_suspicion is not None and (time_to_first_suspicion is None
                    or log_suspicion < time_to_first_suspicion):
                time_to_first_suspicion = log_suspicion
            if log_impeachment is not None and (time_to_detection is None
                    or log_impeachment < time_to_detection):
                time_to_detection = log_impeachment

        # ---- Final metrics ----
        final_coverage = self.get_coverage()
        if coverage_at_90s is None:
            coverage_at_90s = final_coverage  # experiment was shorter than 90s

        result = ExperimentResult(
            name=name,
            controller=controller,
            fault_type=fault_type,
            fault_robot=fault_robot,
            layout=self.layout,
            trial=trial_num,
            seed=seed,
            final_coverage=final_coverage,
            time_to_50=time_to_50,
            time_to_60=time_to_60,
            time_to_80=time_to_80,
            time_to_detection=time_to_detection,
            time_to_first_suspicion=time_to_first_suspicion,
            time_to_detection_2=time_to_detection_2,
            num_leader_changes=num_leader_changes,
            false_positives=false_positives,
            coverage_at_90s=coverage_at_90s,
            coverage_timeline=self.coverage_history,
            ablation=ablation,
        )

        # Save coverage timeline to per-experiment log dir for convergence plots
        if hasattr(self, '_current_log_dir') and self._current_log_dir:
            tl_path = os.path.join(self._current_log_dir, "coverage_timeline.json")
            try:
                with open(tl_path, 'w') as f:
                    json.dump({
                        'name': name, 'controller': controller,
                        'fault_type': fault_type, 'layout': self.layout,
                        'trial': trial_num, 'seed': seed,
                        'fault_time_1': FAULT_INJECT_TIME,
                        'fault_time_2': FAULT_INJECT_TIME_2 if fault_type in ('bad_leader', 'freeze_leader') else None,
                        'timeline': self.coverage_history
                    }, f)
            except Exception:
                pass

            # Save position snapshots for gridworld visualization
            if save_snapshots and snapshot_frames:
                snap_path = os.path.join(self._current_log_dir, "snapshots.json")
                try:
                    with open(snap_path, 'w') as f:
                        json.dump({
                            'name': name, 'controller': controller,
                            'fault_type': fault_type, 'layout': self.layout,
                            'trial': trial_num, 'seed': seed,
                            'arena_width': self.arena_width,
                            'arena_height': self.arena_height,
                            'cell_size': CELL_SIZE,
                            'walls': self.walls,
                            'fault_time_1': FAULT_INJECT_TIME,
                            'fault_time_2': FAULT_INJECT_TIME_2 if fault_type in ('bad_leader', 'freeze_leader') else None,
                            'frames': snapshot_frames,
                        }, f)
                    print(f"  Saved {len(snapshot_frames)} snapshots to {snap_path}")
                except Exception as e:
                    print(f"  Warning: failed to save snapshots: {e}")

        t50_str = f"{time_to_50:.1f}s" if time_to_50 else "N/A"
        t60_str = f"{time_to_60:.1f}s" if time_to_60 else "N/A"
        t80_str = f"{time_to_80:.1f}s" if time_to_80 else "N/A"
        sus_str = f"{time_to_first_suspicion:.2f}s" if time_to_first_suspicion else "N/A"
        det_str = f"{time_to_detection:.2f}s" if time_to_detection else "N/A"
        det2_str = f"{time_to_detection_2:.2f}s" if time_to_detection_2 else "N/A"
        print(f"\n  Results: cov={final_coverage:.1f}%, t50={t50_str}, t60={t60_str}, "
              f"t80={t80_str}, suspicion={sus_str}, impeach1={det_str}, "
              f"impeach2={det2_str}, FP={false_positives}")

        return result

    def _parse_robot_logs(self, fault_time: float) -> Tuple[Optional[float], Optional[float]]:
        """Parse robot log files for precise detection timing.

        Looks for [DETECT] messages that contain first_decay_time and
        impeachment_time timestamps, and computes offsets from fault_time.

        Returns (first_suspicion_offset, impeachment_offset) in seconds.
        """
        log_dir = getattr(self, '_current_log_dir',
                          os.path.join(self.output_dir, "logs"))
        first_sus = None
        first_imp = None

        for i in range(1, NUM_ROBOTS + 1):
            log_path = os.path.join(log_dir, f"robot_{i}.log")
            if not os.path.exists(log_path):
                continue
            try:
                with open(log_path, 'r', errors='replace') as f:
                    for line in f:
                        # [DETECT] First trust decay after N bad commands (Xs)
                        if '[DETECT] First trust decay' in line:
                            # Extract the time in parentheses e.g. "(1.23s)"
                            import re
                            m = re.search(r'\((\d+\.?\d*)s\)', line)
                            if m:
                                dt = float(m.group(1))
                                if first_sus is None or dt < first_sus:
                                    first_sus = dt
                        # [DETECT] IMPEACHMENT after N bad commands, Xs from first
                        if '[DETECT] IMPEACHMENT' in line:
                            import re
                            m = re.search(r'(\d+\.?\d*)s from first', line)
                            if m:
                                dt = float(m.group(1))
                                if first_imp is None or dt < first_imp:
                                    first_imp = dt
            except Exception:
                continue

        return first_sus, first_imp

    # -------------------- Persistence --------------------
    def save_results(self, results: List[ExperimentResult], tag: str = ""):
        """Save results to JSON"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"results_{tag}_{timestamp}.json" if tag else f"results_{timestamp}.json"
        path = os.path.join(self.output_dir, fname)

        data = []
        for r in results:
            d = asdict(r)
            data.append(d)

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"\nResults saved to: {path}")
        return path


# ==================== Statistical Analysis ====================
def compute_statistics(results: List[ExperimentResult]):
    """Group results by condition (controller x fault x layout x ablation) and compute stats."""
    groups: Dict[str, List[ExperimentResult]] = defaultdict(list)
    for r in results:
        abl = getattr(r, 'ablation', None) or 'none'
        key = f"{r.layout}|{r.controller}|{r.fault_type or 'none'}|{abl}"
        groups[key].append(r)

    stats = {}
    for key, trials in sorted(groups.items()):
        parts = key.split('|')
        layout, ctrl, fault = parts[0], parts[1], parts[2]
        abl = parts[3] if len(parts) > 3 else 'none'
        n = len(trials)

        coverages = [t.final_coverage for t in trials]
        cov_mean = sum(coverages) / n
        cov_std = math.sqrt(sum((c - cov_mean)**2 for c in coverages) / max(n - 1, 1))

        t50s = [t.time_to_50 for t in trials if t.time_to_50 is not None]
        t50_mean = sum(t50s) / len(t50s) if t50s else None
        t50_std = math.sqrt(sum((t - t50_mean)**2 for t in t50s) / max(len(t50s) - 1, 1)) \
            if t50s and len(t50s) > 1 else 0.0

        t60s = [t.time_to_60 for t in trials if t.time_to_60 is not None]
        t60_mean = sum(t60s) / len(t60s) if t60s else None
        t60_std = math.sqrt(sum((t - t60_mean)**2 for t in t60s) / max(len(t60s) - 1, 1)) \
            if t60s and len(t60s) > 1 else 0.0

        t80s = [t.time_to_80 for t in trials if t.time_to_80 is not None]
        t80_mean = sum(t80s) / len(t80s) if t80s else None
        t80_std = math.sqrt(sum((t - t80_mean)**2 for t in t80s) / max(len(t80s) - 1, 1)) \
            if t80s and len(t80s) > 1 else 0.0

        detections = [t.time_to_detection for t in trials if t.time_to_detection is not None]
        det_mean = sum(detections) / len(detections) if detections else None
        det_std = math.sqrt(sum((d - det_mean)**2 for d in detections) / max(len(detections) - 1, 1)) \
            if detections and len(detections) > 1 else 0.0
        det_rate = len(detections) / n  # fraction of trials where fault was detected

        # First suspicion timing (fault -> first trust decay)
        suspicions = [t.time_to_first_suspicion for t in trials
                      if t.time_to_first_suspicion is not None]
        sus_mean = sum(suspicions) / len(suspicions) if suspicions else None
        sus_std = math.sqrt(sum((s - sus_mean)**2 for s in suspicions) /
                            max(len(suspicions) - 1, 1)) \
            if suspicions and len(suspicions) > 1 else 0.0

        fps = [t.false_positives for t in trials]
        fp_mean = sum(fps) / n

        cov90s = [t.coverage_at_90s for t in trials if t.coverage_at_90s is not None]
        cov90_mean = sum(cov90s) / len(cov90s) if cov90s else None

        stats[key] = {
            'layout': layout,
            'controller': ctrl,
            'fault': fault,
            'ablation': abl,
            'n': n,
            'coverage_mean': cov_mean,
            'coverage_std': cov_std,
            'time_to_50_mean': t50_mean,
            'time_to_50_std': t50_std,
            'time_to_60_mean': t60_mean,
            'time_to_60_std': t60_std,
            'time_to_80_mean': t80_mean,
            'time_to_80_std': t80_std,
            'suspicion_mean': sus_mean,
            'suspicion_std': sus_std,
            'detection_mean': det_mean,
            'detection_std': det_std,
            'detection_rate': det_rate,
            'false_positive_mean': fp_mean,
            'coverage_at_90s_mean': cov90_mean,
        }

    return stats


def print_statistics(results: List[ExperimentResult]):
    """Print a publication-ready summary table."""
    stats = compute_statistics(results)

    print("\n" + "=" * 100)
    print("STATISTICAL SUMMARY  (mean +/- std)")
    print("=" * 100)

    header = (f"{'Layout':<10} {'Controller':<14} {'Fault':<12} {'Ablation':<15} {'N':>3}  "
              f"{'Coverage':>14}  {'T->50%':>12}  {'T->60%':>12}  {'T->80%':>12}  "
              f"{'1st Suspicion':>14}  {'Impeachment':>14}  {'Det%':>5}  {'FP':>4}  {'Cov@90s':>8}")
    print(header)
    print("-" * 170)

    for key in sorted(stats):
        s = stats[key]
        abl_label = s.get('ablation', 'none')
        cov = f"{s['coverage_mean']:.1f}+/-{s['coverage_std']:.1f}"
        t50 = f"{s['time_to_50_mean']:.1f}+/-{s['time_to_50_std']:.1f}" \
            if s['time_to_50_mean'] is not None else "N/A"
        t60 = f"{s['time_to_60_mean']:.1f}+/-{s['time_to_60_std']:.1f}" \
            if s['time_to_60_mean'] is not None else "N/A"
        t80 = f"{s['time_to_80_mean']:.1f}+/-{s['time_to_80_std']:.1f}" \
            if s['time_to_80_mean'] is not None else "N/A"
        sus = f"{s['suspicion_mean']:.2f}+/-{s['suspicion_std']:.2f}" \
            if s['suspicion_mean'] is not None else "N/A"
        det = f"{s['detection_mean']:.2f}+/-{s['detection_std']:.2f}" \
            if s['detection_mean'] is not None else "N/A"
        det_pct = f"{s['detection_rate']*100:.0f}%"
        fp = f"{s['false_positive_mean']:.1f}"
        c90 = f"{s['coverage_at_90s_mean']:.1f}" if s['coverage_at_90s_mean'] else "N/A"

        print(f"{s['layout']:<10} {s['controller']:<14} {s['fault']:<12} {abl_label:<15} "
              f"{s['n']:>3}  {cov:>14}  {t50:>12}  {t60:>12}  {t80:>12}  "
              f"{sus:>14}  {det:>14}  {det_pct:>5}  {fp:>4}  {c90:>8}")

    print("=" * 120)

    # ---- Key comparisons ----
    print("\nKEY COMPARISONS:")
    for layout in ["open", "multiroom"]:
        reip = stats.get(f"{layout}|reip|bad_leader|none")
        raft = stats.get(f"{layout}|raft|bad_leader|none")
        dec = stats.get(f"{layout}|decentralized|bad_leader|none")
        reip_clean = stats.get(f"{layout}|reip|none|none")

        if reip and raft:
            delta = reip['coverage_mean'] - raft['coverage_mean']
            print(f"\n  [{layout}] Bad Leader — REIP vs RAFT:")
            print(f"    Coverage: REIP {reip['coverage_mean']:.1f}% vs RAFT {raft['coverage_mean']:.1f}% "
                  f"(+{delta:.1f}pp advantage)")
            if reip['suspicion_mean'] is not None:
                print(f"    REIP: First suspicion in {reip['suspicion_mean']:.2f}s, "
                      f"full impeachment in {reip['detection_mean']:.2f}s"
                      if reip['detection_mean'] else
                      f"    REIP: First suspicion in {reip['suspicion_mean']:.2f}s, no impeachment")
            else:
                print(f"    REIP detection: {reip['detection_rate']*100:.0f}% rate "
                      f"at {reip['detection_mean']:.2f}s avg"
                      if reip['detection_mean'] else
                      f"    REIP detection: 0% rate")
            print(f"    RAFT detection: {raft['detection_rate']*100:.0f}% rate"
                  + (f" at {raft['detection_mean']:.2f}s avg"
                     if raft['detection_mean'] else " (never detects)"))

        if reip and dec:
            delta = reip['coverage_mean'] - dec['coverage_mean']
            direction = "+" if delta >= 0 else ""
            print(f"    REIP vs Decentralized: {direction}{delta:.1f}pp coverage "
                  f"(REIP {reip['coverage_mean']:.1f}% vs Dec {dec['coverage_mean']:.1f}%)")

        if reip and reip_clean:
            delta = reip_clean['coverage_mean'] - reip['coverage_mean']
            print(f"    REIP resilience: clean={reip_clean['coverage_mean']:.1f}%, "
                  f"fault={reip['coverage_mean']:.1f}% "
                  f"(only {delta:.1f}pp penalty from fault)")

        # ---- Speed comparison (time to milestones) ----
        all_conds = {}
        for tag in ['reip', 'raft', 'decentralized']:
            for fault_tag in ['none', 'bad_leader', 'freeze_leader']:
                k = f"{layout}|{tag}|{fault_tag}"
                if k in stats:
                    label = f"{tag}{'(fault)' if fault_tag != 'none' else ''}"
                    all_conds[label] = stats[k]

        if len(all_conds) >= 2:
            print(f"\n  [{layout}] Time-to-Coverage Milestones:")
            for label, s in sorted(all_conds.items()):
                t50 = f"{s['time_to_50_mean']:.1f}s" if s.get('time_to_50_mean') else "---"
                t60 = f"{s['time_to_60_mean']:.1f}s" if s.get('time_to_60_mean') else "---"
                t80 = f"{s['time_to_80_mean']:.1f}s" if s.get('time_to_80_mean') else "---"
                print(f"    {label:<22}  50%: {t50:>7}  60%: {t60:>7}  80%: {t80:>7}")


# ==================== Experiment Conditions ====================
# (condition_tag, controller, fault_type, fault_robot, ablation)
EXPERIMENT_CONDITIONS = [

    # --- Clean performance ---
    ("NoFault",    "reip",          None,          1, None),
    ("NoFault",    "raft",          None,          1, None),
    ("NoFault",    "decentralized", None,          1, None),
    # --- Bad leader (REIP's key differentiator) ---
    ("BadLeader",  "reip",          "bad_leader",  1, None),
    ("BadLeader",  "raft",          "bad_leader",  1, None),
    ("BadLeader",  "decentralized", "bad_leader",  1, None),
    # --- Freeze leader: leader stops updating assignments ---
    ("FreezeLeader", "reip",          "freeze_leader",  1, None),
    ("FreezeLeader", "raft",          "freeze_leader",  1, None),
    ("FreezeLeader", "decentralized", "freeze_leader",  1, None),
]

# Ablation conditions: each removes ONE REIP component under bad_leader
# to prove necessity. Also test no_causality under clean to show
# false positive prevention.
ABLATION_CONDITIONS = [
    # No trust assessment: REIP with elections but no detection/impeachment
    # Expected: plateaus like RAFT under bad_leader (proves trust IS the contribution)
    ("Ablation_NoTrust_BL",    "reip", "bad_leader", 1, "no_trust"),
    # No causality grace: CAUSALITY_GRACE_PERIOD=0
    # Under clean: expect false positives (unnecessary impeachments)
    ("Ablation_NoCausality_Clean", "reip", None,         1, "no_causality"),
    # No causality grace under bad_leader: faster detection but more false positives
    ("Ablation_NoCausality_BL",    "reip", "bad_leader", 1, "no_causality"),
    # No direction consistency check: only three-tier cell check
    # Expected: still detects bad_leader via Tier 1/3, but possibly slower
    ("Ablation_NoDirection_BL",    "reip", "bad_leader", 1, "no_direction"),
]


# ==================== Parallel Worker (module-level for pickling) ===========
def _run_worker(args_tuple):
    """Run a sequence of experiments on one worker with one port offset.

    args_tuple: (worker_id, job_list, output_dir)
      job_list: [(name, layout, controller, fault_type, fault_robot, trial, seed, ablation), ...]
    """
    worker_id, job_list, output_dir = args_tuple
    port_offset = worker_id * 1000
    results = []
    runner = ExperimentRunner(output_dir, port_offset=port_offset)

    for job in job_list:
        if len(job) == 9:
            name, layout, controller, fault_type, fault_robot, trial, seed, ablation, fixed_positions = job
        else:
            name, layout, controller, fault_type, fault_robot, trial, seed, ablation = job[:8]
            fixed_positions = None
        
        runner.set_layout(layout)
        try:
            # Pass fixed_positions to start_robots via run_experiment
            runner._fixed_positions_for_next_trial = fixed_positions
            r = runner.run_experiment(
                name=name, controller=controller,
                fault_type=fault_type, fault_robot=fault_robot,
                trial_num=trial, seed=seed, ablation=ablation)
            results.append(asdict(r))
        except Exception as e:
            print(f"  [W{worker_id}][ERROR] {name}: {e}")
            import traceback
            traceback.print_exc()
        time.sleep(1)

    try:
        runner.state_socket.close()
    except Exception:
        pass
    return worker_id, results


# ==================== Validation: Sim vs Hardware ====================
def validate_sim_hardware(sim_results: List[ExperimentResult], hardware_json: str):
    """Compare simulation results to hardware results and report validation status.
    
    Returns: (passed: bool, report: str)
    """
    # Load hardware results
    with open(hardware_json) as f:
        hardware_data = json.load(f)
    
    # Convert hardware data to ExperimentResult format if needed
    hardware_results = []
    if isinstance(hardware_data, list):
        for h in hardware_data:
            # Try to create ExperimentResult from hardware data
            try:
                r = ExperimentResult(**{k: v for k, v in h.items() 
                                      if k in ExperimentResult.__dataclass_fields__})
                hardware_results.append(r)
            except Exception:
                pass
    
    if not hardware_results:
        return False, f"ERROR: Could not parse hardware results from {hardware_json}"
    
    # Group by condition
    sim_groups = defaultdict(list)
    hw_groups = defaultdict(list)
    
    for r in sim_results:
        key = f"{r.controller}|{r.fault_type or 'none'}"
        sim_groups[key].append(r)
    
    for r in hardware_results:
        key = f"{r.controller}|{r.fault_type or 'none'}"
        hw_groups[key].append(r)
    
    # Compare each condition
    report_lines = []
    report_lines.append("=" * 80)
    report_lines.append("SIMULATION vs HARDWARE VALIDATION")
    report_lines.append("=" * 80)
    report_lines.append("")
    
    all_passed = True
    tolerance = {
        'coverage': 10.0,      # 10% absolute difference
        'detection_time': 0.5,  # 0.5s difference
        'suspicion_time': 0.3,  # 0.3s difference
    }
    
    for key in sorted(set(list(sim_groups.keys()) + list(hw_groups.keys()))):
        sim_trials = sim_groups.get(key, [])
        hw_trials = hw_groups.get(key, [])
        
        if not sim_trials:
            report_lines.append(f"  {key}: Hardware only (no sim data)")
            continue
        if not hw_trials:
            report_lines.append(f"  {key}: Sim only (no hardware data)")
            continue
        
        # Compute means
        sim_cov = sum(t.final_coverage for t in sim_trials) / len(sim_trials)
        hw_cov = sum(t.final_coverage for t in hw_trials) / len(hw_trials)
        cov_diff = abs(sim_cov - hw_cov)
        
        sim_det = [t.time_to_detection for t in sim_trials if t.time_to_detection is not None]
        hw_det = [t.time_to_detection for t in hw_trials if t.time_to_detection is not None]
        sim_det_mean = sum(sim_det) / len(sim_det) if sim_det else None
        hw_det_mean = sum(hw_det) / len(hw_det) if hw_det else None
        
        sim_sus = [t.time_to_first_suspicion for t in sim_trials if t.time_to_first_suspicion is not None]
        hw_sus = [t.time_to_first_suspicion for t in hw_trials if t.time_to_first_suspicion is not None]
        sim_sus_mean = sum(sim_sus) / len(sim_sus) if sim_sus else None
        hw_sus_mean = sum(hw_sus) / len(hw_sus) if hw_sus else None
        
        # Check pass/fail
        passed = True
        if cov_diff > tolerance['coverage']:
            passed = False
            all_passed = False
        
        det_diff = abs(sim_det_mean - hw_det_mean) if (sim_det_mean and hw_det_mean) else None
        if det_diff and det_diff > tolerance['detection_time']:
            passed = False
            all_passed = False
        
        sus_diff = abs(sim_sus_mean - hw_sus_mean) if (sim_sus_mean and hw_sus_mean) else None
        if sus_diff and sus_diff > tolerance['suspicion_time']:
            passed = False
            all_passed = False
        
        status = "✓ PASS" if passed else "✗ FAIL"
        report_lines.append(f"  {status} {key}:")
        report_lines.append(f"    Coverage: Sim {sim_cov:.1f}% vs HW {hw_cov:.1f}% (diff: {cov_diff:.1f}%)")
        if sim_det_mean and hw_det_mean:
            report_lines.append(f"    Detection: Sim {sim_det_mean:.2f}s vs HW {hw_det_mean:.2f}s (diff: {det_diff:.2f}s)")
        if sim_sus_mean and hw_sus_mean:
            report_lines.append(f"    Suspicion: Sim {sim_sus_mean:.2f}s vs HW {hw_sus_mean:.2f}s (diff: {sus_diff:.2f}s)")
        report_lines.append("")
    
    report_lines.append("=" * 80)
    if all_passed:
        report_lines.append("VALIDATION PASSED: Simulation matches hardware within tolerances")
        report_lines.append("  → Safe to use 100-trial sim results for paper")
    else:
        report_lines.append("VALIDATION FAILED: Significant differences detected")
        report_lines.append("  → Fix simulation physics before using results")
    report_lines.append("=" * 80)
    
    return all_passed, "\n".join(report_lines)


# ==================== Main ====================
def main():
    import argparse
    from concurrent.futures import ProcessPoolExecutor, as_completed

    parser = argparse.ArgumentParser(description="ISEF Experiment Suite")
    parser.add_argument("--layout", default="all",
                        help="'open', 'multiroom', or 'all' (default: all)")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_CONDITION,
                        help=f"Trials per condition (default: {TRIALS_PER_CONDITION})")
    parser.add_argument("--quick", action="store_true",
                        help="Quick check: N=3, open layout only")
    parser.add_argument("--condition", default=None,
                        help="Run only conditions matching this substring (e.g. 'BadLeader')")
    parser.add_argument("--workers", type=int, default=0,
                        help="Parallel workers (0=auto based on CPU cores, 1=sequential)")
    parser.add_argument("--snapshots", action="store_true",
                        help="Save per-timestep position snapshots for gridworld viz")
    parser.add_argument("--ablation", action="store_true",
                        help="Run ablation study conditions instead of standard conditions")
    parser.add_argument("--duration", type=float, default=EXPERIMENT_DURATION,
                        help=f"Per-trial duration in seconds (default: {EXPERIMENT_DURATION})")
    parser.add_argument("--validate", type=str, metavar="HARDWARE_JSON",
                        help="Validation mode: compare sim results to hardware results from JSON file")
    parser.add_argument("--hardware-positions", type=str, metavar="POSITIONS_JSON",
                        help="Use fixed starting positions from JSON (e.g., from ArUco detection)")
    args = parser.parse_args()
    globals()["EXPERIMENT_DURATION"] = args.duration

    # Handle validation mode
    if args.validate:
        # Load hardware results and compare
        print(f"\nVALIDATION MODE: Comparing to hardware results from {args.validate}")
        # Run a quick sim run first (or load existing sim results)
        # For now, assume user provides both sim and hardware JSON files
        # TODO: Auto-run sim if not provided
        print("ERROR: Validation mode requires both sim and hardware results")
        print("  Usage: Run sim first, then: --validate hardware_results.json")
        return 1
    
    # Load hardware positions if provided
    fixed_positions = None
    if args.hardware_positions:
        with open(args.hardware_positions) as f:
            data = json.load(f)
        if 'sim_fixed_positions' in data:
            fixed_positions = {int(k): tuple(v) for k, v in data['sim_fixed_positions'].items()}
            print(f"Loaded {len(fixed_positions)} hardware starting positions")
        elif 'robot_positions' in data:
            import math
            fixed_positions = {}
            for rid, pos in data['robot_positions'].items():
                x, y, heading_deg = pos
                fixed_positions[int(rid)] = (float(x), float(y), math.radians(heading_deg))
            print(f"Loaded {len(fixed_positions)} hardware starting positions (converted)")
        else:
            print(f"WARNING: No position data found in {args.hardware_positions}")
    
    if args.quick:
        n_trials = 3
        layouts = ["open"]
    else:
        n_trials = args.trials
        layouts = list(ARENA_LAYOUTS.keys()) if args.layout == "all" else [args.layout]

    # Filter conditions if requested
    if args.ablation:
        conditions = ABLATION_CONDITIONS
    else:
        conditions = EXPERIMENT_CONDITIONS
    if args.condition:
        conditions = [c for c in conditions if args.condition.lower() in c[0].lower()
                      or args.condition.lower() in c[1].lower()]

    # Build the full job list
    # Each job: (name, layout, controller, fault_type, fault_robot, trial, seed, ablation)
    jobs = []
    for layout in layouts:
        for trial in range(1, n_trials + 1):
            seed = trial * 1000
            for cond_tag, controller, fault_type, fault_robot, ablation in conditions:
                name = f"{layout}_{controller}_{cond_tag}_t{trial}"
                # Use fixed positions if provided (only for first trial for validation)
                use_fixed = fixed_positions if (fixed_positions and trial == 1) else None
                jobs.append((name, layout, controller, fault_type, fault_robot, trial, seed, ablation, use_fixed))
                jobs.append((name, layout, controller, fault_type,
                             fault_robot, trial, seed, ablation))

    total = len(jobs)
    if total == 0:
        print("No experiments matched the given filters. Check --condition and --layout.")
        sys.exit(1)

    # Decide number of parallel workers
    # Each experiment uses ~6 processes (5 robots + harness) but they're mostly
    # sleeping (20Hz loop = 95% idle), so we can run many workers per core.
    # Physics is time-based, so results are consistent regardless of load.
    cpu_count = os.cpu_count() or 4
    if args.workers > 0:
        n_workers = args.workers
    else:
        # Default: ~2 workers per physical core (each experiment is I/O-bound)
        n_workers = max(1, min(cpu_count // 2, total))
    n_workers = min(n_workers, total)  # Never more workers than jobs

    seq_hours = total * (EXPERIMENT_DURATION + 8) / 3600
    par_hours = (total / n_workers) * (EXPERIMENT_DURATION + 8) / 3600

    print(f"\nISEF Experiment Suite")
    print(f"  Layouts: {layouts}")
    print(f"  Conditions: {len(conditions)}")
    print(f"  Trials per condition: {n_trials}")
    print(f"  Total experiments: {total}")
    print(f"  Workers: {n_workers} parallel")
    print(f"  Est. time: {par_hours:.1f}h  (sequential would be {seq_hours:.1f}h)")
    print()

    # ---- Create a named run directory ----
    # Format: experiments/run_YYYYMMDD_HHMMSS_<layout>_<N>trials/
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    layout_tag = "+".join(layouts)
    cond_tag = args.condition or "all"
    run_name = f"run_{timestamp}_{layout_tag}_{n_trials}trials_{cond_tag}"
    output_dir = os.path.join("experiments", run_name)
    os.makedirs(output_dir, exist_ok=True)

    print(f"  Output: {output_dir}/")
    print()

    if n_workers <= 1:
        # ---- Sequential mode (simple, for debugging) ----
        runner = ExperimentRunner(output_dir)
        all_results: List[ExperimentResult] = []
        for i, (name, layout, controller, fault_type, fault_robot, trial, seed, ablation) in enumerate(jobs):
            print(f"\n>>> Experiment {i+1}/{total}: {name}")
            runner.set_layout(layout)
            result = runner.run_experiment(
                name=name, controller=controller,
                fault_type=fault_type, fault_robot=fault_robot,
                trial_num=trial, seed=seed,
                save_snapshots=args.snapshots,
                ablation=ablation,
            )
            all_results.append(result)
            time.sleep(2)

        final_path = runner.save_results(all_results, tag="final")
        print_statistics(all_results)
    else:
        # ---- Parallel mode ----
        # Partition jobs into N worker queues; each queue runs sequentially
        # within its process, so port offsets never collide.
        worker_queues: List[List[tuple]] = [[] for _ in range(n_workers)]
        for i, job in enumerate(jobs):
            worker_queues[i % n_workers].append(job)

        all_results: List[ExperimentResult] = []
        completed = 0
        start_time = time.time()

        # Build worker args: (worker_id, job_list, output_dir)
        worker_args = [(wid, wq, output_dir)
                       for wid, wq in enumerate(worker_queues)]

        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            futures = {executor.submit(_run_worker, wa): wa[0]
                       for wa in worker_args}

            for future in as_completed(futures):
                wid = futures[future]
                try:
                    returned_wid, result_dicts = future.result()
                except Exception as e:
                    import traceback
                    print(f"  [Worker {wid} CRASHED]: {e}")
                    traceback.print_exc()
                    continue
                for rd in result_dicts:
                    completed += 1
                    rd.pop('coverage_timeline', None)
                    r = ExperimentResult(**{
                        k: v for k, v in rd.items()
                        if k in ExperimentResult.__dataclass_fields__
                    })
                    all_results.append(r)
                elapsed = time.time() - start_time
                print(f"  Worker {wid} done: {len(result_dicts)} experiments "
                      f"({elapsed/60:.1f}m elapsed)")

        # Save results directly (no ExperimentRunner needed)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        final_path = os.path.join(output_dir, f"results_final_{timestamp}.json")
        with open(final_path, 'w') as f:
            json.dump([asdict(r) for r in all_results], f, indent=2)
        print(f"\nResults saved to: {final_path}")
        print(f"  ({len(all_results)} experiments collected from {n_workers} workers)")
        print_statistics(all_results)

    # ---- Write a human-readable summary alongside the JSON ----
    summary_path = os.path.join(output_dir, "SUMMARY.txt")
    with open(summary_path, 'w') as f:
        f.write(f"REIP Experiment Run: {run_name}\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Layouts: {layouts}\n")
        f.write(f"Conditions: {len(conditions)}\n")
        f.write(f"Trials per condition: {n_trials}\n")
        f.write(f"Total experiments: {total}\n")
        f.write(f"Workers: {n_workers}\n\n")
        # Write the stats table
        import io
        old_stdout = sys.stdout
        sys.stdout = buf = io.StringIO()
        print_statistics(all_results)
        sys.stdout = old_stdout
        f.write(buf.getvalue())

    print(f"\nDone! {len(all_results)} experiments completed.")
    print(f"Results dir: {output_dir}/")
    print(f"  results_final_*.json  — raw data")
    print(f"  SUMMARY.txt           — human-readable stats")
    print(f"  logs/                 — per-experiment robot logs")
    print(f"\nGenerate graphs:")
    print(f"  python test/generate_poster_graphs.py {output_dir}/results_final_*.json")


if __name__ == "__main__":
    main()
