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

ROBOT_RADIUS = 75
AVOID_DISTANCE = 200
CRITICAL_DISTANCE = 100

# RAFT parameters - NO TRUST, only heartbeats
HEARTBEAT_INTERVAL = 0.5   # Leader sends heartbeat this often
HEARTBEAT_TIMEOUT = 2.0    # Assume leader dead after this silence
ELECTION_TIMEOUT_MIN = 1.0
ELECTION_TIMEOUT_MAX = 2.0

CONTROL_RATE = 10
BROADCAST_RATE = 5
BASE_SPEED = 50

TOF_CHANNELS = {0: "right", 1: "front_right", 2: "front", 3: "front_left", 4: "left"}

# ============== Enums ==============
class RaftState(Enum):
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"

# ============== Hardware (same as reip_node) ==============
class Hardware:
    def __init__(self):
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
            time.sleep(0.005)
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
        try:
            self.uart.write(f"MOT,{left:.1f},{right:.1f}\n".encode())
            self.uart.readline()
        except: pass
    
    def stop(self):
        if not HARDWARE_AVAILABLE: return
        try:
            self.uart.write(b'STOP\n')
            self.uart.readline()
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
                data, _ = self.pos_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                if msg.get('type') == 'position' and msg.get('robot_id') == self.robot_id:
                    self.x = msg['x']
                    self.y = msg['y']
                    self.theta = msg['theta']
                    self.position_timestamp = msg.get('timestamp', time.time())
                    cell = self.get_cell(self.x, self.y)
                    if cell:
                        self.my_visited.add(cell)
                        self.known_visited.add(cell)
        except (BlockingIOError, ConnectionResetError):
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
        obey blindly — this is the whole point of the baseline."""
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
                if (cx, cy) not in self.known_visited:
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
            for rid in sorted(need_assignment):
                pos = robots[rid]
                best = None
                best_dist = float('inf')
                for f in available_frontiers:
                    if f in assigned:
                        continue
                    fp = self.cell_to_pos(f)
                    d = math.sqrt((pos[0] - fp[0])**2 + (pos[1] - fp[1])**2)
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
        territory it believes needs coverage.  Assignments are persisted —
        each robot gets the SAME bad target every broadcast until cleared.
        Raft followers have no way to detect this — they just obey."""
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
        Raft has NO trust model — followers flip-flop forever, coverage stalls."""
        margin = 75 + 125  # ROBOT_RADIUS + CELL_SIZE
        corner_a = (margin, margin)
        corner_b = (self.arena_width - margin, self.arena_height - margin)
        
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
        targets forever.  Raft has NO trust model — followers obey the stale
        assignments indefinitely, coverage stalls once targets are explored."""
        if not self._frozen_assignments:
            # First call: snapshot current assignments or use leader position
            if self._prev_assignments:
                self._frozen_assignments = dict(self._prev_assignments)
            else:
                # No previous assignments — assign everyone to leader position
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
        """Find nearest unexplored cell"""
        my_cell = self.get_cell(self.x, self.y)
        if not my_cell:
            return None
        
        for radius in range(1, max(self.coverage_width, self.coverage_height)):
            best_cell = None
            best_dist = float('inf')
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    cx, cy = my_cell[0] + dx, my_cell[1] + dy
                    if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
                        if (cx, cy) not in self.known_visited:
                            d = abs(dx) + abs(dy)
                            if d < best_dist:
                                best_dist = d
                                best_cell = (cx, cy)
            if best_cell:
                return self.cell_to_pos(best_cell)
        return None
    
    # ==================== MOTOR CONTROL ====================
    def compute_motor_command(self) -> Tuple[float, float]:
        # Fault injection
        if self.injected_fault == 'spin':
            return (BASE_SPEED, -BASE_SPEED)
        elif self.injected_fault == 'stop':
            return (0, 0)
        elif self.injected_fault == 'erratic':
            return (random.uniform(-BASE_SPEED, BASE_SPEED),
                   random.uniform(-BASE_SPEED, BASE_SPEED))
        
        front = self.tof.get('front', 9999)
        front_left = self.tof.get('front_left', 9999)
        front_right = self.tof.get('front_right', 9999)
        
        if front < 0: front = 9999
        if front_left < 0: front_left = 9999
        if front_right < 0: front_right = 9999
        
        if front < CRITICAL_DISTANCE:
            return (-BASE_SPEED * 0.5, -BASE_SPEED * 0.3)
        
        if front < AVOID_DISTANCE or front_left < AVOID_DISTANCE or front_right < AVOID_DISTANCE:
            if front_left < front_right:
                return (BASE_SPEED * 0.6, BASE_SPEED * 0.2)
            else:
                return (BASE_SPEED * 0.2, BASE_SPEED * 0.6)
        
        # NO TRUST CHECK - just follow leader assignment blindly
        target = None
        if self.state == RaftState.LEADER:
            if self.bad_leader_mode or self.oscillate_leader_mode or self.freeze_leader_mode:
                # Compromised leader follows its own bad/oscillating/frozen assignment
                target = self.assigned_target if self.assigned_target else self.get_my_frontier()
            else:
                target = self.get_my_frontier()
        elif self.assigned_target:
            target = self.assigned_target  # Follow blindly!
        else:
            target = self.get_my_frontier()
        
        self.current_navigation_target = target
        
        if not target:
            return (0, 0)
        
        dx = target[0] - self.x
        dy = target[1] - self.y
        target_angle = math.atan2(dy, dx)
        
        diff = target_angle - self.theta
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        
        turn = max(-1, min(1, diff / (math.pi / 3)))
        left_speed = BASE_SPEED * (1 - turn * 0.5)
        right_speed = BASE_SPEED * (1 + turn * 0.5)
        
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
            self.encoders = self.hw.read_encoders()
            time.sleep(0.05)
    
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
            
            if self.position_timestamp > 0:
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
