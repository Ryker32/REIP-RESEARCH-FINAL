#!/usr/bin/env python3
"""
RAFT Heartbeat Baseline - Judge-Proof Comparison

Implements a simplified version of the RAFT consensus algorithm
(Ongaro & Ousterhout 2014, "In Search of an Understandable Consensus
Algorithm") adapted for multi-robot frontier exploration.

This is industry-standard failover:
- Leader sends heartbeats
- If heartbeats stop, elect new leader
- NO trust. NO performance reasoning.

Key difference from REIP:
- RAFT catches CRASH faults (leader dies, heartbeats stop)
- REIP catches BYZANTINE faults (leader alive but misbehaving)
  Lamport et al. 1982, "The Byzantine Generals Problem"

A spinning/erratic leader still sends heartbeats -> RAFT never catches it.
REIP's trust model does.

Frontier exploration uses greedy nearest-frontier assignment
(Burgard et al. 2000, "Collaborative multi-robot exploration").

Usage: python3 raft_node.py <robot_id> [--sim]
"""

import socket
import json
import time
import math
import threading
import random
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Set, Tuple
from enum import Enum

# Hardware imports
try:
    import serial
    import smbus
    import busio
    import board
    import adafruit_vl53l0x
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARN] Hardware modules not available - simulation mode")

# ============== Configuration ==============
# MUST MATCH robot/reip_node.py
UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
BROADCAST_IP = "255.255.255.255"
NUM_ROBOTS = 5

ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500
CELL_SIZE = 125

ROBOT_RADIUS = 110   # mm, bounding diagonal from ArUco to rear wheel corner
BODY_RADIUS = 77     # mm, max body extent behind ArUco (rear edge)
BODY_HALF_WIDTH = 64 # mm, half width (128mm / 2)
BODY_FRONT = 70      # mm, body extent ahead of ArUco (front bumper with ToF)
SWEPT_RADIUS = 100   # mm, sqrt(77² + 64²) — rear corner during rotation
TOF_SENSOR_OFFSET = 70  # mm, ToF sensors are this far ahead of ArUco center
AVOID_DISTANCE = 200
CRITICAL_DISTANCE = 100

# Interior wall: 35.7mm thick foam board, left face at x=1000
INTERIOR_WALL_X_LEFT = 1000
INTERIOR_WALL_X_RIGHT = 1036
INTERIOR_WALL_X = 1018        # center (for legacy/tip)
INTERIOR_WALL_Y_END = 1200

# Wall margins - must match reip_node.py for fair comparison
OUTER_WALL_MARGIN = SWEPT_RADIUS + 15  # 115mm, clears worst-case rear corner sweep + 15mm gap
DIVIDER_MARGIN = 125                  # 1 full cell around the divider
WALL_MARGIN = OUTER_WALL_MARGIN       # default used by most code
REPULSION_ZONE = 300                  # mm, must match reip_node.py

# RAFT parameters - NO TRUST, only heartbeats
HEARTBEAT_INTERVAL = 0.5   # Leader sends heartbeat this often
HEARTBEAT_TIMEOUT = 2.0    # Assume leader dead after this silence
ELECTION_TIMEOUT_MIN = 1.0
ELECTION_TIMEOUT_MAX = 2.0

CONTROL_RATE = 10
BROADCAST_RATE = 5
BASE_SPEED = 80              # must match reip_node.py for fair comparison

TOF_CHANNELS = {0: "right", 1: "front_right", 2: "front", 3: "front_left", 4: "left"}

# ============== Enums ==============
class RaftState(Enum):
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"

# ============== Hardware (same as reip_node) ==============
class Hardware:
    def __init__(self):
        self._uart_lock = threading.Lock()
        if not HARDWARE_AVAILABLE:
            print("  Hardware: SIMULATED")
            return
        self.uart = serial.Serial('/dev/serial0', 115200, timeout=0.1)
        time.sleep(0.5)
        self.uart.reset_input_buffer()
        self.bus = smbus.SMBus(1)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.MUX_ADDR = 0x70
        self.tof_sensors = {}
        self._init_tof_sensors()
        self._ping_pico()
    
    def _ping_pico(self):
        if not HARDWARE_AVAILABLE: return True
        for _ in range(3):
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'PING\n')
                time.sleep(0.1)
                if self.uart.readline().decode().strip() == "PONG":
                    print("  Pico: OK")
                    return True
            except: pass
            time.sleep(0.3)
        print("  Pico: FAILED")
        return False
    
    def _init_tof_sensors(self):
        if not HARDWARE_AVAILABLE: return
        for ch, name in TOF_CHANNELS.items():
            try:
                self.bus.write_byte(self.MUX_ADDR, 1 << ch)
                time.sleep(0.05)
                self.tof_sensors[name] = adafruit_vl53l0x.VL53L0X(self.i2c)
            except Exception as e:
                print(f"  ToF {name}: FAILED ({e})")
    
    def _select_mux(self, ch):
        if not HARDWARE_AVAILABLE: return
        try:
            self.bus.write_byte(self.MUX_ADDR, 1 << ch)
            time.sleep(0.001)  # 1ms, tca9548a switches in <1us
        except: pass
    
    def read_tof_all(self) -> Dict[str, int]:
        if not HARDWARE_AVAILABLE:
            return {name: 500 for name in TOF_CHANNELS.values()}
        distances = {}
        for ch, name in TOF_CHANNELS.items():
            try:
                self._select_mux(ch)
                if name in self.tof_sensors:
                    distances[name] = self.tof_sensors[name].range
                else:
                    distances[name] = -1
            except:
                distances[name] = -1
        return distances
    
    def read_encoders(self) -> tuple:
        if not HARDWARE_AVAILABLE: return (0, 0)
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'ENC\n')
                time.sleep(0.02)
                resp = self.uart.readline().decode().strip()
                if ',' in resp:
                    l, r = resp.split(',')
                    return (int(l), int(r))
            except: pass
        return (0, 0)
    
    def set_motors(self, left: float, right: float):
        if not HARDWARE_AVAILABLE: return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(f"MOT,{left:.1f},{right:.1f}\n".encode())
            except: pass
    
    def stop(self):
        if not HARDWARE_AVAILABLE: return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'STOP\n')
            except: pass

# ============== RAFT Node ==============
class RAFTNode:
    """
    RAFT-style leader election with HEARTBEAT ONLY.
    
    NO trust scores. NO performance checks.
    Leader is replaced ONLY when heartbeats stop.
    
    This is the judge-proof baseline: industry-standard failover.
    REIP should beat this under adversarial faults.
    """
    
    def __init__(self, robot_id: int, sim_mode: bool = False):
        self.robot_id = robot_id
        self.sim_mode = sim_mode
        
        mode_str = "SIM" if sim_mode else "HARDWARE"
        print(f"=== RAFT Baseline - Robot {robot_id} [{mode_str}] ===")
        print(f"NO TRUST. Heartbeat timeout only.\n")
        
        # Position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.position_timestamp = 0.0
        
        # Sensors
        self.tof = {}
        self.encoders = (0, 0)
        
        # RAFT state
        self.state = RaftState.FOLLOWER
        self.current_term = 0
        self.voted_for: Optional[int] = None
        self.current_leader: Optional[int] = None
        self.last_heartbeat = time.time()
        self.election_timeout = self._random_election_timeout()
        
        # Peers
        self.peers: Dict[int, dict] = {}
        self.peer_timeout = 3.0
        
        # Coverage
        self.coverage_width = int(ARENA_WIDTH / CELL_SIZE)
        self.coverage_height = int(ARENA_HEIGHT / CELL_SIZE)
        self.my_visited: Set[tuple] = set()
        self.known_visited: Set[tuple] = set()
        
        # Task from leader
        self.assigned_target: Optional[Tuple[float, float]] = None
        self._prev_assignments: Dict[str, Tuple[float, float]] = {}  # Assignment persistence
        self.current_navigation_target: Optional[Tuple[float, float]] = None
        
        # Logging
        self.log_file = None
        self.init_logging()
        
        # Hardware
        print("Initializing hardware...")
        self.hw = Hardware()
        
        # Sockets
        self._init_sockets()
        
        # Fault injection
        self.injected_fault = None
        self.bad_leader_mode = False
        self.oscillate_leader_mode = False
        self._oscillate_phase = 0
        self.freeze_leader_mode = False
        self._frozen_assignments: Dict[str, Tuple[float, float]] = {}
        
        self.running = False
        print("Ready!\n")
    
    def _random_election_timeout(self) -> float:
        return random.uniform(ELECTION_TIMEOUT_MIN, ELECTION_TIMEOUT_MAX)
    
    def _init_sockets(self):
        if self.sim_mode:
            pos_port = UDP_POSITION_PORT + self.robot_id
            peer_port = UDP_PEER_PORT + self.robot_id
            fault_port = UDP_FAULT_PORT + self.robot_id
        else:
            pos_port = UDP_POSITION_PORT
            peer_port = UDP_PEER_PORT
            fault_port = UDP_FAULT_PORT
        
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pos_socket.bind(('', pos_port))
        self.pos_socket.setblocking(False)
        print(f"[INFO] Position socket listening on port {pos_port}")
        
        self.peer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.peer_socket.bind(('', peer_port))
        self.peer_socket.setblocking(False)
        print(f"[INFO] Peer socket listening on port {peer_port}")
        
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.fault_socket.bind(('', fault_port))
        self.fault_socket.setblocking(False)
        print(f"[INFO] Fault socket listening on port {fault_port}")
    
    def init_logging(self):
        os.makedirs("logs", exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = f"logs/raft_{self.robot_id}_{timestamp}.jsonl"
        self.log_file = open(self.log_path, 'w')
        print(f"  Logging to: {self.log_path}")
    
    def log_state(self):
        if not self.log_file: return
        record = {
            't': time.time(),
            'robot_id': self.robot_id,
            'x': self.x, 'y': self.y, 'theta': self.theta,
            'state': self.state.value,
            'leader_id': self.current_leader,
            'term': self.current_term,
            'my_visited_count': len(self.my_visited),
            'known_visited_count': len(self.known_visited),
            'fault': self.injected_fault
        }
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()
    
    # ==================== POSITION ====================
    def receive_position(self):
        try:
            while True:
                data, _ = self.pos_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') == 'position' and msg.get('robot_id') == self.robot_id:
                    self.x = msg['x']
                    self.y = msg['y']
                    self.theta = msg['theta']
                    self.position_timestamp = msg.get('timestamp', time.time())

                    # Mark cells the robot body overlaps as visited
                    min_cx = max(0, int((self.x - BODY_RADIUS) / CELL_SIZE))
                    max_cx = min(self.coverage_width - 1,
                                 int((self.x + BODY_RADIUS) / CELL_SIZE))
                    min_cy = max(0, int((self.y - BODY_RADIUS) / CELL_SIZE))
                    max_cy = min(self.coverage_height - 1,
                                 int((self.y + BODY_RADIUS) / CELL_SIZE))
                    robot_left = self.x < INTERIOR_WALL_X_LEFT
                    for _cx in range(min_cx, max_cx + 1):
                        for _cy in range(min_cy, max_cy + 1):
                            if self.y < INTERIOR_WALL_Y_END:
                                cell_center_x = (_cx + 0.5) * CELL_SIZE
                                if robot_left and cell_center_x > INTERIOR_WALL_X_LEFT:
                                    continue
                                if not robot_left and cell_center_x < INTERIOR_WALL_X_RIGHT:
                                    continue
                            _cell = (_cx, _cy)
                            if _cell not in self.my_visited:
                                self.my_visited.add(_cell)
                                self.known_visited.add(_cell)
                elif msg.get('type') == 'peer_state':
                    peer_id = msg.get('robot_id')
                    if peer_id and peer_id != self.robot_id:
                        self.handle_peer_state(msg)
        except (BlockingIOError, ConnectionResetError):
            pass
        except OSError:
            pass
    
    def get_cell(self, x: float, y: float) -> Optional[tuple]:
        cx = int(x / CELL_SIZE)
        cy = int(y / CELL_SIZE)
        if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
            return (cx, cy)
        return None
    
    def cell_to_pos(self, cell: tuple) -> Tuple[float, float]:
        return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)
    
    def pos_to_cell(self, pos: Tuple[float, float]) -> tuple:
        return (int(pos[0] / CELL_SIZE), int(pos[1] / CELL_SIZE))
    
    def _is_wall_cell(self, cx: int, cy: int) -> bool:
        """True if the cell center is physically unreachable (inside the robot body
        when pressed against a wall).  Keep this tight — the repulsion system
        handles safe navigation; this only excludes truly impossible cells."""
        x, y = self.cell_to_pos((cx, cy))
        if x < OUTER_WALL_MARGIN or x > ARENA_WIDTH - OUTER_WALL_MARGIN:
            return True
        if y < OUTER_WALL_MARGIN or y > ARENA_HEIGHT - OUTER_WALL_MARGIN:
            return True
        if y < INTERIOR_WALL_Y_END and (
            INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN < x < INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN):
            return True
        if INTERIOR_WALL_X_LEFT - BODY_RADIUS < x < INTERIOR_WALL_X_RIGHT + BODY_RADIUS:
            return True
        return False

    def distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def path_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Distance accounting for the interior wall."""
        different_sides = (p1[0] < INTERIOR_WALL_X_LEFT) != (p2[0] < INTERIOR_WALL_X_LEFT)
        if different_sides and (p1[1] < INTERIOR_WALL_Y_END or p2[1] < INTERIOR_WALL_Y_END):
            passage_y = INTERIOR_WALL_Y_END + 70
            d1_to_passage = math.sqrt((p1[0] - INTERIOR_WALL_X)**2 + (p1[1] - passage_y)**2)
            d2_to_passage = math.sqrt((p2[0] - INTERIOR_WALL_X)**2 + (p2[1] - passage_y)**2)
            return d1_to_passage + d2_to_passage
        return self.distance(p1, p2)
    
    # ==================== RAFT PROTOCOL ====================
    def broadcast_message(self, msg: dict):
        data = json.dumps(msg).encode()
        try:
            if self.sim_mode:
                # Sim mode: send to each peer's unique port on localhost
                for peer_id in range(1, NUM_ROBOTS + 1):
                    if peer_id != self.robot_id:
                        self.peer_socket.sendto(
                            data, ('127.0.0.1', UDP_PEER_PORT + peer_id))
                # Send to base port for simulator/experiment harness
                self.peer_socket.sendto(data, ('127.0.0.1', UDP_PEER_PORT))
            else:
                # Hardware mode: single broadcast, all robots on same port
                self.peer_socket.sendto(data, (BROADCAST_IP, UDP_PEER_PORT))
        except Exception:
            pass
    
    def send_heartbeat(self):
        """Leader sends heartbeats with task assignments"""
        if self.state != RaftState.LEADER:
            return
        
        assignments = self.compute_task_assignments()
        
        msg = {
            'type': 'heartbeat',
            'leader_id': self.robot_id,
            'term': self.current_term,
            'assignments': assignments,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def send_state(self):
        """Broadcast state to peers"""
        # If we're leader, include assignments in peer_state for experiment harness
        assignments = {}
        if self.state == RaftState.LEADER:
            assignments = self.compute_task_assignments()
        
        msg = {
            'type': 'peer_state',
            'robot_id': self.robot_id,
            'x': self.x, 'y': self.y, 'theta': self.theta,
            'state': self.state.value,
            'term': self.current_term,
            'leader_id': self.current_leader,
            'coverage_count': len(self.my_visited),
            'visited_cells': list(self.my_visited)[-100:],
            'navigation_target': list(self.current_navigation_target) if self.current_navigation_target else None,
            'commanded_target': list(self.assigned_target) if self.assigned_target else None,
            'assignments': assignments,  # Include for experiment harness
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def request_votes(self):
        """Candidate requests votes"""
        msg = {
            'type': 'vote_request',
            'candidate_id': self.robot_id,
            'term': self.current_term,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def send_vote(self, candidate_id: int, term: int):
        """Send vote to candidate"""
        msg = {
            'type': 'vote_response',
            'voter_id': self.robot_id,
            'candidate_id': candidate_id,
            'term': term,
            'granted': True,
            'timestamp': time.time()
        }
        self.broadcast_message(msg)
    
    def receive_messages(self):
        """Process incoming messages"""
        try:
            while True:
                data, _ = self.peer_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                self.handle_message(msg)
        except (BlockingIOError, ConnectionResetError):
            pass
    
    def handle_message(self, msg: dict):
        msg_type = msg.get('type')
        
        if msg_type == 'heartbeat':
            self.handle_heartbeat(msg)
        elif msg_type == 'vote_request':
            self.handle_vote_request(msg)
        elif msg_type == 'vote_response':
            self.handle_vote_response(msg)
        elif msg_type == 'peer_state':
            self.handle_peer_state(msg)
    
    def handle_heartbeat(self, msg: dict):
        """Receive heartbeat from leader - reset election timeout"""
        leader_id = msg.get('leader_id')
        term = msg.get('term', 0)
        
        if term >= self.current_term:
            self.current_term = term
            self.current_leader = leader_id
            self.state = RaftState.FOLLOWER
            self.last_heartbeat = time.time()
            self.election_timeout = self._random_election_timeout()
            
            # Get my assignment (NO TRUST CHECK - just follow)
            assignments = msg.get('assignments', {})
            if str(self.robot_id) in assignments:
                self.assigned_target = tuple(assignments[str(self.robot_id)])
    
    def handle_vote_request(self, msg: dict):
        """Handle vote request from candidate"""
        candidate_id = msg.get('candidate_id')
        term = msg.get('term', 0)
        
        if term > self.current_term:
            self.current_term = term
            self.voted_for = None
            self.state = RaftState.FOLLOWER
        
        if term >= self.current_term and self.voted_for in (None, candidate_id):
            self.voted_for = candidate_id
            self.send_vote(candidate_id, term)
    
    def handle_vote_response(self, msg: dict):
        """Handle vote response"""
        if self.state != RaftState.CANDIDATE:
            return
        
        if msg.get('granted') and msg.get('term') == self.current_term:
            # Count votes
            voter = msg.get('voter_id')
            if voter not in self._votes_received:
                self._votes_received.add(voter)
            
            # Check majority
            total_nodes = len(self.peers) + 1
            if len(self._votes_received) > total_nodes / 2:
                self.become_leader()
    
    def handle_peer_state(self, msg: dict):
        """Update peer info"""
        peer_id = msg.get('robot_id')
        if peer_id and peer_id != self.robot_id:
            self.peers[peer_id] = {
                'x': msg.get('x', 0),
                'y': msg.get('y', 0),
                'theta': msg.get('theta', 0),
                'last_seen': time.time()
            }
            # Merge coverage
            for cell in msg.get('visited_cells', []):
                self.known_visited.add(tuple(cell))
    
    def check_election_timeout(self):
        """Check if we should start election (leader heartbeat timeout)"""
        if self.state == RaftState.LEADER:
            return
        
        if time.time() - self.last_heartbeat > self.election_timeout:
            self.start_election()
    
    def start_election(self):
        """Start leader election"""
        self.state = RaftState.CANDIDATE
        self.current_term += 1
        self.voted_for = self.robot_id
        self._votes_received = {self.robot_id}
        self.election_timeout = self._random_election_timeout()
        self.last_heartbeat = time.time()
        
        print(f"[RAFT] Starting election, term {self.current_term}")
        self.request_votes()
    
    def become_leader(self):
        """Transition to leader state"""
        self.state = RaftState.LEADER
        self.current_leader = self.robot_id
        self._prev_assignments.clear()  # Fresh assignments for new leader
        print(f"[RAFT] Became leader, term {self.current_term}")
    
    # ==================== TASK ASSIGNMENT ====================
    def compute_task_assignments(self) -> Dict[str, Tuple[float, float]]:
        """Leader assigns frontiers to robots.
        If bad_leader_mode is active, deliberately sends robots to
        already-explored cells.  Raft has NO trust model so followers
        obey blindly - this is the whole point of the baseline."""
        if self.state != RaftState.LEADER:
            return {}
        
        # --- Bad-leader fault: send robots to explored cells ---
        if self.bad_leader_mode:
            return self._compute_bad_assignments()
        
        # --- Oscillate-leader fault: flip-flop targets between corners ---
        if self.oscillate_leader_mode:
            return self._compute_oscillate_assignments()
        
        # --- Freeze-leader fault: stop updating assignments ---
        if self.freeze_leader_mode:
            return self._compute_freeze_assignments()
        
        frontiers = []
        frontier_set = set()
        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                if (cx, cy) not in self.known_visited and not self._is_wall_cell(cx, cy):
                    frontiers.append((cx, cy))
                    frontier_set.add((cx, cy))
        
        if not frontiers:
            self._prev_assignments.clear()
            return {}
        
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
        
        # Assignment persistence: keep previous assignments if target still unexplored
        still_valid = {}
        assigned = set()
        for rid_str, prev_target in self._prev_assignments.items():
            rid = int(rid_str)
            if rid in robots:
                cell = self.pos_to_cell(prev_target)
                if cell in frontier_set:
                    still_valid[rid_str] = prev_target
                    assigned.add(cell)
        
        # Greedy assignment for robots without valid targets
        need_assignment = [rid for rid in robots if str(rid) not in still_valid]
        available_frontiers = [f for f in frontiers if f not in assigned]
        
        if need_assignment and available_frontiers:
            def nearest_frontier_dist(rid):
                pos = robots[rid]
                return min(self.path_distance(pos, self.cell_to_pos(f))
                           for f in available_frontiers)
            sorted_robots = sorted(need_assignment, key=nearest_frontier_dist)

            for rid in sorted_robots:
                pos = robots[rid]
                best = None
                best_dist = float('inf')
                for f in available_frontiers:
                    if f in assigned:
                        continue
                    fp = self.cell_to_pos(f)
                    d = self.path_distance(pos, fp)
                    if d < best_dist:
                        best_dist = d
                        best = f
                if best:
                    assigned.add(best)
                    still_valid[str(rid)] = self.cell_to_pos(best)
        
        self._prev_assignments = dict(still_valid)
        return still_valid
    
    def _compute_bad_assignments(self) -> Dict[str, Tuple[float, float]]:
        """BAD LEADER: deliberately assign already-explored cells.
        Simulates a compromised/hallucinating leader that fixates on
        territory it believes needs coverage.  assignments are persisted -
        each robot gets the SAME bad target every broadcast until cleared.
        raft followers have no way to detect this - they just obey."""
        explored = list(self.known_visited)
        if not explored:
            return {}
        
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
        
        # Persist assignments: pick bad targets ONCE, reuse until fault cleared
        if not hasattr(self, '_bad_assignments_cache'):
            self._bad_assignments_cache = {}
        
        assignments = {}
        for rid in robots:
            if str(rid) not in self._bad_assignments_cache:
                bad_cell = random.choice(explored)
                self._bad_assignments_cache[str(rid)] = self.cell_to_pos(bad_cell)
            
            target = self._bad_assignments_cache[str(rid)]
            assignments[str(rid)] = target
            # Leader stores its own bad assignment so compute_motor_command uses it
            if rid == self.robot_id:
                self.assigned_target = target
        
        print(f"[BAD_LEADER] Raft leader sending {len(assignments)} robots "
              f"to EXPLORED cells (persistent, no detection possible)")
        return assignments
    
    def _compute_oscillate_assignments(self) -> Dict[str, Tuple[float, float]]:
        """OSCILLATE LEADER: flip-flop all followers between two far corners.
        Every broadcast cycle, targets alternate between corner A and B.
        raft has no trust model - followers flip-flop forever, coverage stalls."""
        margin = ROBOT_RADIUS + CELL_SIZE
        corner_a = (margin, margin)
        corner_b = (ARENA_WIDTH - margin, ARENA_HEIGHT - margin)
        
        self._oscillate_phase = 1 - self._oscillate_phase
        target = corner_a if self._oscillate_phase == 0 else corner_b
        
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                robots[pid] = (peer['x'], peer['y'])
        
        assignments = {}
        for rid in robots:
            assignments[str(rid)] = target
            if rid == self.robot_id:
                self.assigned_target = target
        
        print(f"[OSCILLATE] Raft phase {self._oscillate_phase}: all {len(assignments)} "
              f"robots -> ({target[0]:.0f}, {target[1]:.0f}) -- NO DETECTION POSSIBLE")
        return assignments
    
    def _compute_freeze_assignments(self) -> Dict[str, Tuple[float, float]]:
        """FREEZE LEADER: stop updating assignments entirely.
        Snapshots current assignments on first call, then returns the same stale
        targets forever.  raft has no trust model - followers obey the stale
        assignments indefinitely, coverage stalls once targets are explored."""
        if not self._frozen_assignments:
            # First call: snapshot current assignments or use leader position
            if self._prev_assignments:
                self._frozen_assignments = dict(self._prev_assignments)
            else:
                # no previous assignments - assign everyone to leader position
                robots = {self.robot_id: (self.x, self.y)}
                for pid, peer in self.peers.items():
                    if time.time() - peer.get('last_seen', 0) < self.peer_timeout:
                        robots[pid] = (peer['x'], peer['y'])
                for rid in robots:
                    self._frozen_assignments[str(rid)] = (self.x, self.y)
            
            print(f"[FREEZE_LEADER] Raft frozen {len(self._frozen_assignments)} assignments "
                  f"(will never update -- NO DETECTION POSSIBLE)")
        
        # Always return the same stale assignments
        my_key = str(self.robot_id)
        if my_key in self._frozen_assignments:
            self.assigned_target = self._frozen_assignments[my_key]
        
        return dict(self._frozen_assignments)
    
    def get_my_frontier(self) -> Optional[Tuple[float, float]]:
        """Find nearest unexplored cell using path-aware distance.
        Crossing the interior wall adds the actual detour through the
        passage so robots explore their own side first."""
        my_cell = self.get_cell(self.x, self.y)
        if not my_cell:
            return None

        wall_cell_x_left = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)
        wall_cell_y_end = int(INTERIOR_WALL_Y_END / CELL_SIZE)
        robot_on_left = my_cell[0] < wall_cell_x_left

        best_dist = float('inf')
        best_cell = None

        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                cell = (cx, cy)
                if cell in self.known_visited or self._is_wall_cell(cx, cy):
                    continue

                dist = abs(cx - my_cell[0]) + abs(cy - my_cell[1])

                cell_on_left = cx < wall_cell_x_left
                if robot_on_left != cell_on_left and cy < wall_cell_y_end:
                    detour_up = max(0, wall_cell_y_end + 2 - my_cell[1])
                    detour_down = max(0, wall_cell_y_end + 2 - cy)
                    dist += detour_up + detour_down + 3

                if dist < best_dist:
                    best_dist = dist
                    best_cell = cell

        if best_cell:
            return self.cell_to_pos(best_cell)
        return None
    
    # ==================== NAVIGATION ====================
    def _init_stuck_detection(self):
        self._nav_history = []
        self._escape_until = 0
        self._escape_start = 0

    def _check_stuck(self):
        """Detect if robot is physically stuck (wall-grinding, spinning wheels).
        25mm threshold catches lateral drift along walls."""
        now = time.time()

        if now - self.position_timestamp > 1.0:
            self._nav_history.clear()
            return False

        self._nav_history.append((now, self.x, self.y))
        self._nav_history = [(t, x, y) for t, x, y in self._nav_history
                             if now - t < 2.0]
        if len(self._nav_history) < 2:
            return False
        oldest_t, oldest_x, oldest_y = self._nav_history[0]
        elapsed = now - oldest_t
        moved = math.sqrt((self.x - oldest_x)**2 + (self.y - oldest_y)**2)
        return elapsed > 0.5 and moved < 25

    def _route_around_wall(self, target):
        """If target is across the interior wall, route through the passage.

        Three-stage routing:
          Stage 0 → go up on your side to the passage entrance
          Stage 1 → cross above the wall to the other side
          Stage 2 → done, proceed to real target
        """
        tx, ty = target
        on_left = self.x < INTERIOR_WALL_X_LEFT
        target_on_left = tx < INTERIOR_WALL_X_LEFT

        if not hasattr(self, '_wall_route_stage'):
            self._wall_route_stage = 0
            self._wall_route_side = None

        different_sides = on_left != target_on_left

        if not different_sides:
            self._wall_route_stage = 0
            self._wall_route_side = None
            return target

        if self._wall_route_stage == 0 and self.y > INTERIOR_WALL_Y_END + 80:
            return target

        passage_y = INTERIOR_WALL_Y_END + 70
        clearance = DIVIDER_MARGIN + 70
        rid_offset = (self.robot_id % 5) * 10 - 20

        if self._wall_route_stage == 0:
            self._wall_route_side = on_left
            wp_x = (INTERIOR_WALL_X_LEFT - clearance) if on_left else (INTERIOR_WALL_X_RIGHT + clearance)
            wp = [wp_x, passage_y + rid_offset]
            if math.sqrt((self.x - wp_x)**2 + (self.y - passage_y - rid_offset)**2) < 130:
                self._wall_route_stage = 1
            return wp

        if self._wall_route_stage == 1:
            other_side = not self._wall_route_side
            wp_x = (INTERIOR_WALL_X_LEFT - clearance) if other_side else (INTERIOR_WALL_X_RIGHT + clearance)
            wp = [wp_x, passage_y + rid_offset]
            if math.sqrt((self.x - wp_x)**2 + (self.y - passage_y - rid_offset)**2) < 130:
                self._wall_route_stage = 2
            return wp

        self._wall_route_stage = 0
        self._wall_route_side = None
        return target

    def _wall_slide_heading(self, desired_angle):
        """Modify heading to avoid walls using separate outer/divider margins.

        Three layers:
          1. Soft quadratic repulsion starting at REPULSION_ZONE
          2. Hard clamp at WALL_MARGIN / DIVIDER_MARGIN
          3. Deadlock escape toward center if both components zeroed
        """
        hx = math.cos(desired_angle)
        hy = math.sin(desired_angle)

        def _repel(dist_to_wall, zone):
            if dist_to_wall < zone:
                n = (zone - dist_to_wall) / zone
                return n * n
            return 0.0

        repel_x, repel_y = 0.0, 0.0
        repel_x += _repel(self.x, REPULSION_ZONE)
        repel_x -= _repel(ARENA_WIDTH - self.x, REPULSION_ZONE)
        repel_y += _repel(self.y, REPULSION_ZONE)
        repel_y -= _repel(ARENA_HEIGHT - self.y, REPULSION_ZONE)

        if self.y < INTERIOR_WALL_Y_END:
            d_right = self.x - INTERIOR_WALL_X_RIGHT
            d_left = INTERIOR_WALL_X_LEFT - self.x
            div_zone = REPULSION_ZONE
            if 0 < d_left < div_zone:
                repel_x -= _repel(d_left, div_zone)
            if 0 < d_right < div_zone:
                repel_x += _repel(d_right, div_zone)

        for tip_x in [INTERIOR_WALL_X_LEFT, INTERIOR_WALL_X_RIGHT]:
            tip_dx = self.x - tip_x
            tip_dy = self.y - INTERIOR_WALL_Y_END
            tip_dist = math.sqrt(tip_dx * tip_dx + tip_dy * tip_dy)
            TIP_ZONE = DIVIDER_MARGIN + 30
            if tip_dist < TIP_ZONE and tip_dist > 5 and self.y < INTERIOR_WALL_Y_END + 50:
                tip_strength = _repel(tip_dist, TIP_ZONE)
                repel_x += (tip_dx / tip_dist) * tip_strength
                repel_y += (tip_dy / tip_dist) * tip_strength

        REPEL_GAIN = 12.0  # must match reip_node.py
        hx += repel_x * REPEL_GAIN
        hy += repel_y * REPEL_GAIN

        if self.x < WALL_MARGIN and hx < 0:
            hx = 0
        if self.x > ARENA_WIDTH - WALL_MARGIN and hx > 0:
            hx = 0
        if self.y < WALL_MARGIN and hy < 0:
            hy = 0
        if self.y > ARENA_HEIGHT - WALL_MARGIN and hy > 0:
            hy = 0
        if self.y < INTERIOR_WALL_Y_END:
            d_from_right = self.x - INTERIOR_WALL_X_RIGHT
            d_from_left = INTERIOR_WALL_X_LEFT - self.x
            if 0 < d_from_right < DIVIDER_MARGIN and hx < 0:
                hx = 0
            elif 0 < d_from_left < DIVIDER_MARGIN and hx > 0:
                hx = 0

        if abs(hx) < 0.01 and abs(hy) < 0.01:
            cx, cy = ARENA_WIDTH / 2, ARENA_HEIGHT / 2
            return math.atan2(cy - self.y, cx - self.x)

        return math.atan2(hy, hx)

    def _peer_avoidance_heading(self, desired_angle) -> float:
        """Steer away from nearby peers using their broadcast positions."""
        PEER_AVOID_DIST = 400
        PEER_REPEL_GAIN = 4.0

        hx = math.cos(desired_angle)
        hy = math.sin(desired_angle)

        repel_x, repel_y = 0.0, 0.0
        closest_dist = 9999.0
        n_close = 0
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                continue
            dx = self.x - peer['x']
            dy = self.y - peer['y']
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < closest_dist:
                closest_dist = dist
            if dist < PEER_AVOID_DIST and dist > 5:
                n_close += 1
                strength = (PEER_AVOID_DIST - dist) / PEER_AVOID_DIST
                if dist < ROBOT_RADIUS:
                    strength *= 6.0
                elif dist < 2 * ROBOT_RADIUS:
                    strength *= 3.0
                repel_x += (dx / dist) * strength
                repel_y += (dy / dist) * strength

        if n_close >= 2 and (abs(repel_x) + abs(repel_y)) < 0.5:
            offset_angle = (self.robot_id * 1.2566)
            repel_x += math.cos(offset_angle) * 0.8
            repel_y += math.sin(offset_angle) * 0.8

        if abs(repel_x) < 0.01 and abs(repel_y) < 0.01:
            return desired_angle

        if closest_dist < ROBOT_RADIUS:
            return math.atan2(repel_y, repel_x)

        hx += repel_x * PEER_REPEL_GAIN
        hy += repel_y * PEER_REPEL_GAIN
        return math.atan2(hy, hx)

    # ==================== MOTOR CONTROL ====================
    def compute_motor_command(self) -> Tuple[float, float]:
        if not hasattr(self, '_nav_history'):
            self._init_stuck_detection()

        # Fault injection
        if self.injected_fault == 'spin':
            return (BASE_SPEED, -BASE_SPEED)
        elif self.injected_fault == 'stop':
            return (0, 0)
        elif self.injected_fault == 'erratic':
            return (random.uniform(-BASE_SPEED, BASE_SPEED),
                   random.uniform(-BASE_SPEED, BASE_SPEED))

        # Escape mode: reverse briefly, then turn and drive away from obstacle.
        if time.time() < self._escape_until:
            phase_elapsed = time.time() - self._escape_start
            escape_angle = getattr(self, '_escape_angle', math.atan2(
                ARENA_HEIGHT / 2 - self.y, ARENA_WIDTH / 2 - self.x))

            if phase_elapsed < 0.6:
                return (-BASE_SPEED * 0.6, -BASE_SPEED * 0.6)
            elif phase_elapsed < 1.2:
                diff = escape_angle - self.theta
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                if diff > 0:
                    return (BASE_SPEED, -BASE_SPEED)
                else:
                    return (-BASE_SPEED, BASE_SPEED)
            else:
                safe_angle = self._wall_slide_heading(escape_angle)
                diff = safe_angle - self.theta
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                turn = max(-1, min(1, diff / (math.pi / 4)))
                left = BASE_SPEED * (1 - turn * 0.5)
                right = BASE_SPEED * (1 + turn * 0.5)
                self._stuck_count = max(0, self._stuck_count - 1)
                return (left, right)

        # ToF emergency: ANY sensor < 80mm triggers reverse.
        # All 5 sensors checked — catches side collisions (divider, peers).
        TOF_EMERGENCY_DIST = 120
        if not hasattr(self, '_tof_emergency_count'):
            self._tof_emergency_count = 0

        front_d = self.tof.get('front', 999)
        fl_d = self.tof.get('front_left', 999)
        fr_d = self.tof.get('front_right', 999)
        left_d = self.tof.get('left', 999)
        right_d = self.tof.get('right', 999)
        all_tof = [front_d, fl_d, fr_d, left_d, right_d]
        min_any = min(all_tof)

        if min_any > 20 and min_any < TOF_EMERGENCY_DIST:
            self._tof_emergency_count += 1
            if self._tof_emergency_count >= 3:
                self._tof_emergency_count = 0
                if not hasattr(self, '_stuck_count'):
                    self._stuck_count = 0
                self._stuck_count += 1
                escape_time = min(2.0 + self._stuck_count * 0.5, 4.0)
                self._escape_until = time.time() + escape_time
                self._escape_start = time.time()
                self._nav_history.clear()
                flee_x, flee_y = 0.0, 0.0
                if self.x < 300: flee_x += 1.0
                if ARENA_WIDTH - self.x < 300: flee_x -= 1.0
                if self.y < 300: flee_y += 1.0
                if ARENA_HEIGHT - self.y < 300: flee_y -= 1.0
                if self.y < INTERIOR_WALL_Y_END:
                    if 0 < INTERIOR_WALL_X_LEFT - self.x < 300:
                        flee_x -= 1.0
                    if 0 < self.x - INTERIOR_WALL_X_RIGHT < 300:
                        flee_x += 1.0
                if abs(flee_x) < 0.01 and abs(flee_y) < 0.01:
                    flee_x = math.cos(self.robot_id * 1.2566)
                    flee_y = math.sin(self.robot_id * 1.2566)
                self._escape_angle = math.atan2(flee_y, flee_x)
                return (-BASE_SPEED * 0.7, -BASE_SPEED * 0.7)
            rev = BASE_SPEED * 0.6
            if left_d < right_d and left_d < front_d:
                return (-rev * 0.3, -rev)
            elif right_d < left_d and right_d < front_d:
                return (-rev, -rev * 0.3)
            elif fr_d < fl_d:
                return (-rev, -rev * 0.4)
            else:
                return (-rev * 0.4, -rev)
        else:
            self._tof_emergency_count = 0

        # NO TRUST CHECK - just follow leader assignment blindly
        target = None
        if self.state == RaftState.LEADER:
            if self.bad_leader_mode or self.oscillate_leader_mode or self.freeze_leader_mode:
                target = self.assigned_target if self.assigned_target else self.get_my_frontier()
            else:
                target = self.get_my_frontier()
        elif self.assigned_target:
            target = self.assigned_target  # Follow blindly!
        else:
            target = self.get_my_frontier()

        if not target:
            self.current_navigation_target = None
            return (0, 0)

        # Clamp target outside physical body margin (outer walls + divider)
        tx = max(OUTER_WALL_MARGIN, min(ARENA_WIDTH - OUTER_WALL_MARGIN, target[0]))
        ty = max(OUTER_WALL_MARGIN, min(ARENA_HEIGHT - OUTER_WALL_MARGIN, target[1]))
        if ty < INTERIOR_WALL_Y_END and (
                INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN < tx < INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN):
            if tx < INTERIOR_WALL_X:
                tx = INTERIOR_WALL_X_LEFT - DIVIDER_MARGIN
            else:
                tx = INTERIOR_WALL_X_RIGHT + DIVIDER_MARGIN
        target = (tx, ty)
        self.current_navigation_target = target

        # Route around interior wall
        target = self._route_around_wall(target)

        # Stuck detection → escape: reverse, then drive to open space
        if self._check_stuck():
            if not hasattr(self, '_stuck_count'):
                self._stuck_count = 0
            self._stuck_count += 1
            escape_time = min(2.0 + self._stuck_count * 0.5, 4.0)
            self._escape_until = time.time() + escape_time
            self._escape_start = time.time()
            self._nav_history.clear()

            flee_x, flee_y = 0.0, 0.0
            for pid, peer in self.peers.items():
                if time.time() - peer.last_seen > PEER_TIMEOUT:
                    continue
                d = math.sqrt((self.x - peer.x)**2 + (self.y - peer.y)**2)
                if d < 400 and d > 5:
                    flee_x += (self.x - peer.x) / d
                    flee_y += (self.y - peer.y) / d
            flee_range = 300.0
            if self.x < flee_range:
                flee_x += (flee_range - self.x) / flee_range
            if ARENA_WIDTH - self.x < flee_range:
                flee_x -= (flee_range - (ARENA_WIDTH - self.x)) / flee_range
            if self.y < flee_range:
                flee_y += (flee_range - self.y) / flee_range
            if ARENA_HEIGHT - self.y < flee_range:
                flee_y -= (flee_range - (ARENA_HEIGHT - self.y)) / flee_range
            if self.y < INTERIOR_WALL_Y_END:
                d_to_left = abs(self.x - INTERIOR_WALL_X_LEFT)
                d_to_right = abs(self.x - INTERIOR_WALL_X_RIGHT)
                if d_to_left < flee_range:
                    flee_x += -(flee_range - d_to_left) / flee_range if self.x > INTERIOR_WALL_X_LEFT else (flee_range - d_to_left) / flee_range
                if d_to_right < flee_range:
                    flee_x += (flee_range - d_to_right) / flee_range if self.x > INTERIOR_WALL_X_RIGHT else -(flee_range - d_to_right) / flee_range

            if abs(flee_x) < 0.01 and abs(flee_y) < 0.01:
                flee_x = math.cos(self.robot_id * 1.2566)
                flee_y = math.sin(self.robot_id * 1.2566)

            self._escape_angle = math.atan2(flee_y, flee_x)
            return (-BASE_SPEED * 0.6, -BASE_SPEED * 0.6)

        # Navigate to target
        dx = target[0] - self.x
        dy = target[1] - self.y
        target_angle = math.atan2(dy, dx)

        # Wall-sliding and peer avoidance
        target_angle = self._wall_slide_heading(target_angle)
        target_angle = self._peer_avoidance_heading(target_angle)

        diff = target_angle - self.theta
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        turn = max(-1, min(1, diff / (math.pi / 3)))

        # ToF-based reactive turn bias (includes front sensor)
        TOF_BIAS_DIST = 300
        fl = self.tof.get('front_left', 999)
        fr = self.tof.get('front_right', 999)
        front_tof = self.tof.get('front', 999)
        left_tof = self.tof.get('left', 999)
        right_tof = self.tof.get('right', 999)
        tof_bias = 0.0
        if fl < TOF_BIAS_DIST and fl > 20:
            tof_bias += 0.3 * (TOF_BIAS_DIST - fl) / TOF_BIAS_DIST
        if left_tof < TOF_BIAS_DIST and left_tof > 20:
            tof_bias += 0.15 * (TOF_BIAS_DIST - left_tof) / TOF_BIAS_DIST
        if fr < TOF_BIAS_DIST and fr > 20:
            tof_bias -= 0.3 * (TOF_BIAS_DIST - fr) / TOF_BIAS_DIST
        if right_tof < TOF_BIAS_DIST and right_tof > 20:
            tof_bias -= 0.15 * (TOF_BIAS_DIST - right_tof) / TOF_BIAS_DIST
        if front_tof < TOF_BIAS_DIST and front_tof > 20:
            steer_sign = 1.0 if (fl > fr or left_tof > right_tof) else -1.0
            tof_bias += steer_sign * 0.4 * (TOF_BIAS_DIST - front_tof) / TOF_BIAS_DIST
        turn = max(-1, min(1, turn + tof_bias))

        # Speed scales with heading alignment + ToF proximity.
        alignment = max(0.0, math.cos(diff))
        effective_speed = BASE_SPEED * (0.4 + 0.6 * alignment)
        min_front_tof = min(front_d, fl_d, fr_d)
        if 20 < min_front_tof < 250:
            tof_speed_factor = max(0.7, min_front_tof / 250)
            effective_speed *= tof_speed_factor

        min_wall_dist = min(
            self.x, ARENA_WIDTH - self.x,
            self.y, ARENA_HEIGHT - self.y)
        if self.y < INTERIOR_WALL_Y_END:
            if self.x < INTERIOR_WALL_X_LEFT:
                min_wall_dist = min(min_wall_dist, INTERIOR_WALL_X_LEFT - self.x)
            elif self.x > INTERIOR_WALL_X_RIGHT:
                min_wall_dist = min(min_wall_dist, self.x - INTERIOR_WALL_X_RIGHT)
        min_peer_dist = 9999.0
        for pid, peer in self.peers.items():
            if time.time() - peer.get('last_seen', 0) > self.peer_timeout:
                continue
            d = math.sqrt((self.x - peer['x'])**2 + (self.y - peer['y'])**2)
            min_peer_dist = min(min_peer_dist, d)

        min_obstacle_dist = min(min_wall_dist, min_peer_dist)
        TURN_BOOST_DIST = 300
        if min_obstacle_dist < TURN_BOOST_DIST:
            proximity = 1.0 - (min_obstacle_dist / TURN_BOOST_DIST)
            if min_obstacle_dist < 150:
                turn_mix = 0.95
            else:
                turn_mix = 0.5 + 0.4 * proximity
        else:
            turn_mix = 0.5

        MIN_MOTOR_PWM = 25
        left_speed = effective_speed * (1 - turn * turn_mix)
        right_speed = effective_speed * (1 + turn * turn_mix)
        if abs(turn) > 0.1:
            if abs(left_speed) > 0 and abs(left_speed) < MIN_MOTOR_PWM:
                left_speed = math.copysign(MIN_MOTOR_PWM, left_speed)
            if abs(right_speed) > 0 and abs(right_speed) < MIN_MOTOR_PWM:
                right_speed = math.copysign(MIN_MOTOR_PWM, right_speed)

        self._last_turn = turn
        return (left_speed, right_speed)
    
    # ==================== FAULT INJECTION ====================
    def receive_fault_injection(self):
        try:
            while True:
                data, _ = self.fault_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                target = msg.get('robot_id')
                if target == self.robot_id or target == 'all':
                    fault = msg.get('fault', msg.get('cmd', 'none'))
                    if fault in ('none', 'clear'):
                        self.injected_fault = None
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self._frozen_assignments.clear()
                        if hasattr(self, '_bad_assignments_cache'):
                            self._bad_assignments_cache.clear()
                        print(f"[FAULT] Cleared")
                    elif fault == 'bad_leader':
                        # Leadership fault: send robots to explored cells.
                        # Raft has NO trust model, so it will follow blindly.
                        self.bad_leader_mode = True
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        print(f"[FAULT] BAD_LEADER mode - will send bad assignments (Raft CANNOT detect this)")
                    elif fault == 'oscillate_leader':
                        # Oscillation fault: flip-flop targets.
                        # Raft has NO trust model, so followers oscillate forever.
                        self.oscillate_leader_mode = True
                        self.bad_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        self._oscillate_phase = 0
                        print(f"[FAULT] OSCILLATE_LEADER mode - will flip-flop targets (Raft CANNOT detect this)")
                    elif fault == 'freeze_leader':
                        # Freeze fault: leader stops updating assignments.
                        # Raft has NO trust model, so followers stall indefinitely.
                        self.freeze_leader_mode = True
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.injected_fault = None
                        self._frozen_assignments.clear()  # Will be populated on first call
                        print(f"[FAULT] FREEZE_LEADER mode - will stop updating (Raft CANNOT detect this)")
                    else:
                        self.injected_fault = fault
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        print(f"[FAULT] Injected: {fault}")
        except (BlockingIOError, ConnectionResetError):
            pass
    
    # ==================== MAIN LOOPS ====================
    def sensor_loop(self):
        while self.running:
            self.tof = self.hw.read_tof_all()
            # encoder reads removed - positions come from camera,
            # and UART contention with motor commands trips Pico watchdog.
            time.sleep(0.02)
    
    def network_loop(self):
        last_broadcast = 0
        last_heartbeat = 0
        
        while self.running:
            self.receive_position()
            self.receive_messages()
            self.receive_fault_injection()
            
            now = time.time()
            
            # State broadcast
            if now - last_broadcast > 1.0 / BROADCAST_RATE:
                self.send_state()
                last_broadcast = now
            
            # Leader heartbeat
            if self.state == RaftState.LEADER and now - last_heartbeat > HEARTBEAT_INTERVAL:
                self.send_heartbeat()
                last_heartbeat = now
            
            # Check election timeout
            self.check_election_timeout()
            
            # Prune peers
            dead = [p for p, d in self.peers.items() if now - d.get('last_seen', 0) > self.peer_timeout]
            for p in dead:
                del self.peers[p]
            
            time.sleep(0.01)
    
    def control_loop(self):
        interval = 1.0 / CONTROL_RATE
        last_print = 0
        last_log = 0
        
        # Initialize votes set
        self._votes_received = set()
        
        while self.running:
            start = time.time()
            
            left, right = 0.0, 0.0
            pos_age = time.time() - self.position_timestamp
            if self.position_timestamp > 0 and pos_age < 1.5:
                left, right = self.compute_motor_command()
                self.hw.set_motors(left, right)
            else:
                self.hw.stop()
            
            if time.time() - last_log > 0.2:
                self.log_state()
                last_log = time.time()
            
            if time.time() - last_print > 2.0:
                cov = len(self.my_visited) / (self.coverage_width * self.coverage_height) * 100
                total = len(self.known_visited) / (self.coverage_width * self.coverage_height) * 100
                print(f"[R{self.robot_id}] state={self.state.value} "
                      f"leader={self.current_leader} term={self.current_term} "
                      f"cov={cov:.1f}%/{total:.1f}%")
                last_print = time.time()
            
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def run(self):
        self.running = True
        
        sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        network_thread = threading.Thread(target=self.network_loop, daemon=True)
        
        sensor_thread.start()
        network_thread.start()
        
        print("Running RAFT baseline... Press Ctrl+C to stop\n")
        
        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            self.hw.stop()
            if self.log_file:
                self.log_file.close()
            print("Stopped.")

# ============== Entry ==============
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 raft_node.py <robot_id> [--sim] [--port-base N]")
        sys.exit(1)
    
    robot_id = int(sys.argv[1])
    sim_mode = "--sim" in sys.argv
    
    # Parse --port-base for parallel experiment support
    if "--port-base" in sys.argv:
        idx = sys.argv.index("--port-base")
        port_offset = int(sys.argv[idx + 1])
        UDP_POSITION_PORT += port_offset
        UDP_PEER_PORT += port_offset
        UDP_FAULT_PORT += port_offset
    
    # Parse --arena-width / --arena-height for scaled layouts
    if "--arena-width" in sys.argv:
        idx = sys.argv.index("--arena-width")
        ARENA_WIDTH = int(sys.argv[idx + 1])
    if "--arena-height" in sys.argv:
        idx = sys.argv.index("--arena-height")
        ARENA_HEIGHT = int(sys.argv[idx + 1])
    
    node = RAFTNode(robot_id, sim_mode=sim_mode)
    node.run()
