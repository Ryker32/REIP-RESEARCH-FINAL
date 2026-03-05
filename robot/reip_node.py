#!/usr/bin/env python3
"""
REIP Distributed Node - Faithful Implementation
Runs on each Pi Zero - this is the robot's brain.

Key Features:
- Three-tier confidence-weighted trust (personal/ToF/peer)
- Leader task assignment with proactive verification
- Suspicion accumulation with carry-over (no reset)
- Logging for vector overlay visualization

Academic References (see paper for full citations):
- Frontier-based exploration: Yamauchi 1997, "A frontier-based approach
  for autonomous exploration"
- Coverage coordination / greedy assignment: Burgard et al. 2000,
  "Collaborative multi-robot exploration"
- Trust formalization / computational trust: Marsh 1994, "Formalising
  Trust as a Computational Concept"
- Byzantine fault tolerance (the failure class REIP targets): Lamport
  et al. 1982, "The Byzantine Generals Problem"
- RAFT consensus (baseline comparison): Ongaro & Ousterhout 2014,
  "In Search of an Understandable Consensus Algorithm"
- Information-theoretic exploration / entropy-based utility: Bourgault
  et al. 2002, "Information based adaptive robotic exploration"
- Anomaly detection for peer faults: Chandola et al. 2009, "Anomaly
  detection: A survey"

Usage: python3 reip_node.py <robot_id> [--sim] [--decentralized]
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
from collections import deque

# Hardware imports (will fail gracefully on PC for testing)
try:
    import serial
    import smbus
    import busio
    import board
    import adafruit_vl53l0x
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARN] Hardware modules not available - running in simulation mode")

# ============== Configuration ==============
# Network
# Hardware mode (default): all robots share the same port per service.
#   Each Pi runs one process, so no port collision. Broadcast reaches everyone.
# Sim mode (--sim): each robot uses base+robot_id to avoid localhost collisions.
UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
BROADCAST_IP = "255.255.255.255"
NUM_ROBOTS = 5             # Number of robots in swarm

# Arena
ARENA_WIDTH = 2000   # mm
ARENA_HEIGHT = 1500  # mm
CELL_SIZE = 125      # mm - gives 16×12 = 192 cells, ~38 per robot with 5 bots

# Robot physical geometry (from CAD: 147mm x 128mm body)
#   ArUco center: 70mm from front bumper, 77mm from rear, centered in width
#   Wheels: mid-body at ArUco center, 43mm diameter, differential drive
#   ToF sensors: on front bumper arc, 70mm ahead of ArUco center
#     0° (front), ±37.5° (front-left/right), ±75° (left/right)
#     Sensor height: 13mm above ground (walls are ~150mm tall)
#   ArUco tag: 80mm above ground
ROBOT_RADIUS = 110   # mm, bounding diagonal from ArUco to rear wheel corner
BODY_RADIUS = 77     # mm, max body extent behind ArUco (rear edge)
BODY_HALF_WIDTH = 64 # mm, half width (128mm / 2)
BODY_FRONT = 70      # mm, body extent ahead of ArUco (front bumper with ToF)
SWEPT_RADIUS = 100   # mm, sqrt(77² + 64²) — rear corner during in-place rotation
TOF_SENSOR_OFFSET = 70  # mm, ToF sensors are this far ahead of ArUco center
TOF_RANGE = 200      # mm - high-res verification zone
AVOID_DISTANCE = 200 # mm
CRITICAL_DISTANCE = 100  # mm

# Interior wall: 35.7mm thick foam board, left face at x=1000
# Runs from y=0 to y=1200, passage at y=1200-1500
INTERIOR_WALL_X_LEFT = 1000   # left face of the divider
INTERIOR_WALL_X_RIGHT = 1036  # right face (1000 + 35.7 ≈ 1036)
INTERIOR_WALL_X = 1018        # center of wall (for legacy/tip calculations)
INTERIOR_WALL_Y_END = 1200    # wall runs y=0 to this value

# Outer wall margin: at least ROBOT_RADIUS so frontier assignment never
# sends robots to cells they can't physically reach without hitting the wall.
OUTER_WALL_MARGIN = ROBOT_RADIUS      # ~110mm — excludes perimeter cells
DIVIDER_MARGIN = BODY_HALF_WIDTH       # 64mm — robot body extends this far from center
WALL_MARGIN = OUTER_WALL_MARGIN       # default used by most code
REPULSION_ZONE = 220                  # mm — soft repulsion, wide enough to steer clear

# ============== REIP Trust Parameters ==============
# Three-tier confidence weights
# Inspired by Marsh 1994's trust formalization: evidence reliability
# varies by source.  Personal experience is ground truth (weight 1.0),
# direct sensor observation is high confidence (1.0), and peer reports
# are discounted for potential staleness (0.3).
WEIGHT_PERSONAL = 1.0    # Cells I personally visited - ground truth
WEIGHT_TOF = 1.0         # Obstacles within ToF range - I can see it
WEIGHT_PEER = 0.3        # Peer-reported only - might be stale

# Suspicion and trust
# Causality-aware checking handles sync delays correctly, so we can use
# tight thresholds that detect bad leaders quickly.
# MPC severe triggers independently (0.5-0.8), so threshold of 1.5 allows
# ~2 severe MPC triggers OR ~2 personal_visited to decay trust.
SUSPICION_THRESHOLD = 1.5    # Faster detection
RECOVERY_RATE = 0.1          # Slower recovery to catch persistent bad behavior
TRUST_DECAY_RATE = 0.2       # Faster decay per threshold crossing
MIN_TRUST = 0.1              # Floor
TRUST_THRESHOLD = 0.5        # Below = untrusted for voting
IMPEACHMENT_THRESHOLD = 0.3  # Below = vote to impeach

# ============== Provable Worst-Case Detection Bounds ==============
# These bounds are a novel contribution of REIP.  Under persistent fault,
# suspicion accumulates at rate (w - r) per bad command, where w is the
# evidence weight and r is RECOVERY_RATE.  This is a fixed-increment
# threshold test applied to trust.
# Detection (first trust decay) occurs when suspicion >= SUSPICION_THRESHOLD.
#
# Tier 1 (personal_visited): ceil(1.5 / (1.0 - 0.1)) = ceil(1.67) = 2 commands
# Tier 3 (peer_reported):    ceil(1.5 / (0.3 - 0.1)) = ceil(7.5)  = 8 commands
#
# Full impeachment requires trust to drop from 1.0 to IMPEACHMENT_THRESHOLD (0.3).
# Each threshold crossing decays trust by TRUST_DECAY_RATE (0.2).
# Need ceil((1.0 - 0.3) / 0.2) = 4 threshold crossings.
# Worst case (all Tier 3): 4 * 8 = 32 commands to impeachment.
# Best case (all Tier 1):  4 * 2 = 8 commands to impeachment.
WORST_CASE_DETECT_T1 = math.ceil(SUSPICION_THRESHOLD / (WEIGHT_PERSONAL - RECOVERY_RATE))
WORST_CASE_DETECT_T3 = math.ceil(SUSPICION_THRESHOLD / (WEIGHT_PEER - RECOVERY_RATE))
THRESHOLD_CROSSINGS_TO_IMPEACH = math.ceil((1.0 - IMPEACHMENT_THRESHOLD) / TRUST_DECAY_RATE)
WORST_CASE_IMPEACH_T1 = THRESHOLD_CROSSINGS_TO_IMPEACH * WORST_CASE_DETECT_T1
WORST_CASE_IMPEACH_T3 = THRESHOLD_CROSSINGS_TO_IMPEACH * WORST_CASE_DETECT_T3

# Causality grace period: accounts for broadcast delay in distributed system.
# A cell visited less than this many seconds before the assignment was sent
# should NOT trigger suspicion, because the leader's map was stale.
# At BROADCAST_RATE=5Hz, worst-case broadcast delay is ~0.2s per hop.
CAUSALITY_GRACE_PERIOD = 0.5  # seconds

# Timing
ELECTION_INTERVAL = 2.0      # seconds (steady-state)
ELECTION_FAST_INTERVAL = 0.5 # seconds (startup: first 3 elections)
ELECTION_FAST_COUNT = 3      # number of fast elections before switching to normal
PEER_TIMEOUT = 3.0           # seconds before peer considered dead
CONTROL_RATE = 10            # Hz
BROADCAST_RATE = 5           # Hz
BASE_SPEED = 80              # pwm %, n20 100:1 motors need high duty for 217g robot

# ToF channels (hardware wiring)
TOF_CHANNELS = {
    0: "right",
    1: "front_right",
    2: "front",
    3: "front_left",
    4: "left",
}

# ============== Enums ==============
class RobotState(Enum):
    IDLE = "idle"
    EXPLORING = "exploring"
    LEADER = "leader"
    FOLLOWER = "follower"
    IMPEACHED = "impeached"

# ============== Data Classes ==============
@dataclass
class PeerInfo:
    robot_id: int
    x: float = 0
    y: float = 0
    theta: float = 0
    state: str = "idle"
    trust_score: float = 1.0      # Their reported trust in leader
    my_trust_for_them: float = 1.0  # MY trust assessment
    my_suspicion: float = 0.0     # Accumulated suspicion
    vote: Optional[int] = None
    coverage_count: int = 0
    visited_cells: Set[tuple] = field(default_factory=set)
    assigned_target: Optional[Tuple[float, float]] = None  # Leader's assignment for them
    last_seen: float = 0

# ============== Hardware Interface ==============
class Hardware:
    def __init__(self):
        self._uart_lock = threading.Lock()
        self.hw_ok = False
        if not HARDWARE_AVAILABLE:
            print("  Hardware: SIMULATED")
            return

        for attempt in range(2):
            try:
                self.uart = serial.Serial('/dev/serial0', 115200, timeout=0.1)
                time.sleep(0.5)
                self.uart.reset_input_buffer()
                self.bus = smbus.SMBus(1)
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.MUX_ADDR = 0x70
                self.tof_sensors = {}
                self._init_tof_sensors()
                self._ping_pico()
                self.hw_ok = True
                break
            except Exception as e:
                print(f"  Hardware init attempt {attempt+1} failed: {e}")
                if attempt == 0:
                    time.sleep(1)
        if not self.hw_ok:
            print("  WARNING: Hardware init failed, running with degraded sensors")
    
    def _ping_pico(self):
        if not HARDWARE_AVAILABLE:
            return True
        for _ in range(3):
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'PING\n')
                time.sleep(0.1)
                if self.uart.readline().decode().strip() == "PONG":
                    print("  Pico: OK")
                    return True
            except:
                pass
            time.sleep(0.3)
        print("  Pico: FAILED")
        return False
    
    def _init_tof_sensors(self):
        """Initialize ToF sensors once"""
        if not HARDWARE_AVAILABLE:
            return
        for ch, name in TOF_CHANNELS.items():
            try:
                self.bus.write_byte(self.MUX_ADDR, 1 << ch)
                time.sleep(0.05)
                self.tof_sensors[name] = adafruit_vl53l0x.VL53L0X(self.i2c)
            except Exception as e:
                print(f"  ToF {name}: FAILED ({e})")
    
    def _select_mux(self, ch):
        if not HARDWARE_AVAILABLE:
            return
        try:
            self.bus.write_byte(self.MUX_ADDR, 1 << ch)
            time.sleep(0.001)  # 1ms, tca9548a switches in <1us
        except:
            pass
    
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
        if not HARDWARE_AVAILABLE:
            return (0, 0)
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'ENC\n')
                time.sleep(0.02)
                resp = self.uart.readline().decode().strip()
                if ',' in resp:
                    l, r = resp.split(',')
                    return (int(l), int(r))
            except:
                pass
        return (0, 0)
    
    def set_motors(self, left: float, right: float):
        if not HARDWARE_AVAILABLE:
            return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(f"MOT,{left:.1f},{right:.1f}\n".encode())
            except:
                pass
    
    def stop(self):
        if not HARDWARE_AVAILABLE:
            return
        with self._uart_lock:
            try:
                self.uart.reset_input_buffer()
                self.uart.write(b'STOP\n')
            except:
                pass

# ============== REIP Node ==============
class REIPNode:
    def __init__(self, robot_id: int, sim_mode: bool = False):
        self.robot_id = robot_id
        self.sim_mode = sim_mode
        
        mode_str = "SIM (unique ports)" if sim_mode else "HARDWARE (shared ports)"
        print(f"=== REIP Node - Robot {robot_id} [{mode_str}] ===")
        print(f"Trust Model: Three-tier confidence-weighted")
        print(f"  Personal: {WEIGHT_PERSONAL}, ToF: {WEIGHT_TOF}, Peer: {WEIGHT_PEER}")
        print(f"  Suspicion threshold: {SUSPICION_THRESHOLD}, Recovery: {RECOVERY_RATE}\n")
        
        # My state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.state = RobotState.IDLE
        self.position_timestamp = 0.0
        self._pos_initialized = False
        
        # Sensor data
        self.tof = {}
        self.tof_obstacles = set()  # Cells with obstacles detected by ToF
        self.encoders = (0, 0)
        
        # Peer tracking
        self.peers: Dict[int, PeerInfo] = {}
        
        # Leadership
        self.current_leader: Optional[int] = None
        self.my_vote: Optional[int] = None
        self.trust_in_leader = 1.0
        self.leader_failures: Dict[int, int] = {}  # Track impeachment history
        self._election_settled = False  # Becomes True after initial convergence
        self.suspicion_of_leader = 0.0
        self.last_election = 0
        
        # Detection timing metrics (for convergence bound validation)
        self.bad_commands_received = 0       # Count of bad commands since fault started
        self.first_bad_command_time = None   # Timestamp of first bad command
        self.first_decay_time = None         # Timestamp of first trust decay
        self.impeachment_time = None         # Timestamp of impeachment
        
        # Decentralized mode (--decentralized flag disables leader coordination)
        self.decentralized_mode = False
        
        # Task assignment
        self.leader_assigned_target: Optional[Tuple[float, float]] = None
        self.leader_assignment_time: float = 0.0  # When assignment was received
        self.my_assigned_target: Optional[Tuple[float, float]] = None  # Leader's own target
        self._prev_assignments: Dict[str, Tuple[float, float]] = {}   # Assignment persistence cache
        self.my_predicted_target: Optional[Tuple[float, float]] = None
        self.current_navigation_target: Optional[Tuple[float, float]] = None  # What I'm ACTUALLY navigating to
        
        # Assignment staleness
        self.ASSIGNMENT_STALE_TIME = 5.0  # seconds — must survive WiFi drops (leader broadcasts at 5Hz)
        
        # Coverage - with timestamps for causality-aware trust
        self.coverage_width = int(ARENA_WIDTH / CELL_SIZE)
        self.coverage_height = int(ARENA_HEIGHT / CELL_SIZE)
        self.my_visited: Dict[tuple, float] = {}  # cell -> timestamp when I visited
        self.known_visited: Set[tuple] = set()    # All known visited (mine + peers)
        self.known_visited_time: Dict[tuple, float] = {}  # cell -> when we first learned about it
        
        # Logging for visualization
        self.log_file = None
        self.init_logging()
        
        # Hardware
        print("Initializing hardware...")
        self.hw = Hardware()
        
        # Network sockets
        self._init_sockets()
        
        # Trial gating — motors stay off until explicit "start" command
        self.trial_started = False

        # Fault injection state
        self.injected_fault = None  # Motor faults: 'spin', 'stop', 'erratic'
        self.bad_leader_mode = False  # Leadership fault: send bad assignments
        self.oscillate_leader_mode = False  # Oscillation fault: flip-flop targets
        self._oscillate_phase = 0  # Alternates 0/1 each broadcast cycle
        self.freeze_leader_mode = False  # Freeze fault: leader stops assigning new targets
        self._frozen_assignments: Dict[str, Tuple[float, float]] = {}  # Cached at freeze time
        
        # ===== Fault Classification: Command Stability Metric (Ω) =====
        # Tracks the angular velocity of the commanded direction to distinguish:
        #   - Byzantine Assignment (bad_leader): stable wrong direction → low Ω
        #   - Byzantine Oscillation (oscillate_leader): flip-flopping → high Ω
        # Ω(t) = (1/W) * Σ |Δθ_cmd(i)|  over sliding window W
        self._cmd_theta_history: deque = deque(maxlen=10)  # recent cmd angles
        self._omega_window: deque = deque(maxlen=8)         # |Δθ| values
        self.classified_fault_type: Optional[str] = None    # Result of classification
        self._leader_tenure_start: float = 0.0              # Time current leader was elected
        # Grace period: suppress Ω detection for this many seconds after election.
        # During startup/re-election, the leader rapidly assigns new targets as it
        # discovers the environment, producing transient high Ω that is NOT a fault.
        # With threshold at 3π/4 (2.356 rad), normal startup Ω (max ~2.2) stays
        # below threshold, so 2.0s is sufficient.
        self._OMEGA_GRACE_PERIOD: float = 2.0               # seconds
        
        # Threading
        self.running = False
        
        print("Ready!\n")
    
    def _init_sockets(self):
        """Initialize network sockets.
        
        Hardware mode (default): all robots share the same ports.
          - Position: all on 5100, filter by robot_id in message
          - Peer:     all on 5200, filter by robot_id != self
          - Fault:    all on 5300, filter by target robot_id
          Broadcast reaches everyone. No port collision (separate Pis).
        
        Sim mode (--sim): unique ports per robot for localhost.
          - Position: 5100 + robot_id
          - Peer:     5200 + robot_id
          - Fault:    5300 + robot_id
          Simulator relays messages to each port individually.
        """
        if self.sim_mode:
            pos_port = UDP_POSITION_PORT + self.robot_id
            peer_port = UDP_PEER_PORT + self.robot_id
            fault_port = UDP_FAULT_PORT + self.robot_id
        else:
            pos_port = UDP_POSITION_PORT
            peer_port = UDP_PEER_PORT
            fault_port = UDP_FAULT_PORT
        
        # Position from camera
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pos_socket.bind(('', pos_port))
        self.pos_socket.setblocking(False)
        print(f"[INFO] Position socket listening on port {pos_port}")
        
        # Peer communication
        self.peer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.peer_socket.bind(('', peer_port))
        self.peer_socket.setblocking(False)
        print(f"[INFO] Peer socket listening on port {peer_port}")
        
        # Fault injection
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.fault_socket.bind(('', fault_port))
        self.fault_socket.setblocking(False)
        print(f"[INFO] Fault socket listening on port {fault_port}")
    
    def init_logging(self):
        """Initialize logging for vector overlay visualization"""
        os.makedirs("logs", exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = f"logs/robot_{self.robot_id}_{timestamp}.jsonl"
        self.log_file = open(self.log_path, 'w')
        print(f"  Logging to: {self.log_path}")
    
    def log_state(self):
        """Log state for post-processing visualization"""
        if not self.log_file:
            return
        
        # Convert tuples to lists for JSON compatibility
        def to_list(t):
            return list(t) if t else None
        
        record = {
            't': time.time(),
            'robot_id': self.robot_id,
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'state': self.state.value,
            'leader_id': self.current_leader,
            'trust_in_leader': self.trust_in_leader,
            'suspicion': self.suspicion_of_leader,
            'predicted_target': to_list(self.my_predicted_target),
            'commanded_target': to_list(self.leader_assigned_target),
            'leader_self_target': to_list(self.my_assigned_target) if self.state == RobotState.LEADER else None,
            'my_visited_count': len(self.my_visited),
            'known_visited_count': len(self.known_visited),
            'fault': self.injected_fault,
            'omega': sum(self._omega_window) / len(self._omega_window) if self._omega_window else 0.0,
            'classified_fault': self.classified_fault_type,
            'bad_commands_received': self.bad_commands_received,
            'first_bad_command_time': self.first_bad_command_time,
            'first_decay_time': self.first_decay_time,
            'impeachment_time': self.impeachment_time,
            'decentralized': self.decentralized_mode
        }
        self.log_file.write(json.dumps(record) + '\n')
        self.log_file.flush()
    
    # ==================== POSITION ====================
    def receive_position(self):
        """Receive our position from camera server AND relayed peer states"""
        msg_count = 0
        try:
            while True:
                data, addr = self.pos_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                msg_count += 1
                
                # Handle position updates for us
                if msg.get('type') == 'position':
                    msg_rid = msg.get('robot_id')
                    if msg_rid == self.robot_id:
                        POS_EMA = 0.5
                        raw_x, raw_y = msg['x'], msg['y']
                        if not self._pos_initialized:
                            self.x = raw_x
                            self.y = raw_y
                            self._pos_initialized = True
                        else:
                            self.x += POS_EMA * (raw_x - self.x)
                            self.y += POS_EMA * (raw_y - self.y)
                        self.theta = msg['theta']
                        self.position_timestamp = msg.get('timestamp', time.time())
                        
                        # Mark cells the robot body covers as visited.
                        # Robot body is 128mm wide, cell is 125mm — the robot
                        # physically overlaps the center cell plus its direct
                        # neighbors.  Only mark non-wall cells.
                        now = time.time()
                        _cell = self.get_cell(self.x, self.y)
                        if _cell is not None:
                            cx, cy = _cell
                            for dx, dy in ((0,0),(1,0),(-1,0),(0,1),(0,-1)):
                                nc = (cx+dx, cy+dy)
                                if (0 <= nc[0] < self.coverage_width and
                                    0 <= nc[1] < self.coverage_height and
                                    not self._is_wall_cell(nc[0], nc[1]) and
                                    nc not in self.my_visited):
                                    self.my_visited[nc] = now
                                    if nc not in self.known_visited:
                                        self.known_visited.add(nc)
                                        self.known_visited_time[nc] = now
                
                # Handle relayed peer states (from visual_sim acting as relay)
                elif msg.get('type') == 'peer_state':
                    peer_id = msg.get('robot_id')
                    if peer_id and peer_id != self.robot_id:
                        self.update_peer(peer_id, msg)
        except (BlockingIOError, ConnectionResetError):
            pass
        except ConnectionResetError:
            pass
        except OSError:
            pass
    
    def get_cell(self, x: float, y: float) -> Optional[tuple]:
        """Convert position to cell index"""
        cx = int(x / CELL_SIZE)
        cy = int(y / CELL_SIZE)
        if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
            return (cx, cy)
        return None
    
    def cell_to_pos(self, cell: tuple) -> Tuple[float, float]:
        """Convert cell to center position in mm"""
        return ((cell[0] + 0.5) * CELL_SIZE, (cell[1] + 0.5) * CELL_SIZE)
    
    def pos_to_cell(self, pos: Tuple[float, float]) -> tuple:
        """Convert position in mm to grid cell coordinates"""
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
        return False

    def distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Euclidean distance between two points"""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def path_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Distance accounting for the interior wall. If p1 and p2 are on
        opposite sides and either is below the wall end, the path must
        detour through the passage above the wall."""
        p1_left = p1[0] < INTERIOR_WALL_X_LEFT
        p2_left = p2[0] > INTERIOR_WALL_X_RIGHT
        different_sides = (p1[0] < INTERIOR_WALL_X_LEFT) != (p2[0] < INTERIOR_WALL_X_LEFT)
        if different_sides and (p1[1] < INTERIOR_WALL_Y_END or p2[1] < INTERIOR_WALL_Y_END):
            passage_y = INTERIOR_WALL_Y_END + 70
            d1_to_passage = math.sqrt((p1[0] - INTERIOR_WALL_X)**2 + (p1[1] - passage_y)**2)
            d2_to_passage = math.sqrt((p2[0] - INTERIOR_WALL_X)**2 + (p2[1] - passage_y)**2)
            return d1_to_passage + d2_to_passage
        return self.distance(p1, p2)
    
    # ==================== PEER COMMUNICATION ====================
    def broadcast_state(self):
        """Broadcast our state to all peers"""
        # If I'm leader, include task assignments
        assignments = {}
        if self.state == RobotState.LEADER:
            assignments = self.compute_task_assignments()
        
        msg = {
            'type': 'peer_state',
            'robot_id': self.robot_id,
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'state': self.state.value,
            'trust_in_leader': self.trust_in_leader,
            'vote': self.my_vote,
            'coverage_count': len(self.my_visited),
            'visited_cells': list(self.my_visited.keys())[-100:],  # Last 100 cells
            'leader_id': self.current_leader,
            'suspicion': self.suspicion_of_leader,
            'navigation_target': list(self.current_navigation_target) if self.current_navigation_target else None,
            'predicted_target': list(self.my_predicted_target) if self.my_predicted_target else None,
            'commanded_target': list(self.leader_assigned_target) if self.leader_assigned_target else None,
            'assignments': assignments,  # Leader broadcasts assignments
            'timestamp': time.time()
        }
        data = json.dumps(msg).encode()
        try:
            if self.sim_mode:
                # Sim mode: send to each peer's unique port on localhost
                for peer_id in range(1, NUM_ROBOTS + 1):
                    if peer_id != self.robot_id:
                        self.peer_socket.sendto(
                            data, ('127.0.0.1', UDP_PEER_PORT + peer_id))
                # Send to base port for simulator/logger listener
                self.peer_socket.sendto(data, ('127.0.0.1', UDP_PEER_PORT))
            else:
                # Hardware mode: single broadcast, all robots on same port
                self.peer_socket.sendto(data, (BROADCAST_IP, UDP_PEER_PORT))
        except Exception:
            pass
    
    def receive_peer_states(self):
        """Receive state broadcasts from peers"""
        try:
            while True:
                data, _ = self.peer_socket.recvfrom(8192)
                msg = json.loads(data.decode())
                
                if msg.get('type') == 'peer_state':
                    peer_id = msg.get('robot_id')
                    if peer_id and peer_id != self.robot_id:
                        self.update_peer(peer_id, msg)
        except (BlockingIOError, ConnectionResetError):
            pass
        except ConnectionResetError:
            # Windows UDP issue - ignore
            pass
        except OSError:
            # Other socket issues - ignore
            pass
    
    def update_peer(self, peer_id: int, msg: dict):
        """Update peer info"""
        if peer_id not in self.peers:
            self.peers[peer_id] = PeerInfo(robot_id=peer_id)
        
        peer = self.peers[peer_id]
        peer.x = msg.get('x', 0)
        peer.y = msg.get('y', 0)
        peer.theta = msg.get('theta', 0)
        peer.state = msg.get('state', 'idle')
        peer.trust_score = msg.get('trust_in_leader', 1.0)
        peer.vote = msg.get('vote')
        peer.coverage_count = msg.get('coverage_count', 0)
        peer.last_seen = time.time()
        
        # Merge their visited cells (with timestamps for causality)
        now = time.time()
        for cell in msg.get('visited_cells', []):
            cell_tuple = tuple(cell)
            peer.visited_cells.add(cell_tuple)
            if cell_tuple not in self.known_visited:
                self.known_visited.add(cell_tuple)
                self.known_visited_time[cell_tuple] = now  # When WE learned about it
        
        # If this peer is the leader, get my assignment
        if msg.get('leader_id') == peer_id and peer_id == self.current_leader:
            assignments = msg.get('assignments', {})
            if str(self.robot_id) in assignments:
                target = assignments[str(self.robot_id)]
                new_target = tuple(target) if target else None
                
                # Always update assignment and timestamp from each broadcast.
                # Each broadcast is a fresh leadership decision that must be
                # independently verified, even if the target hasn't changed.
                # Without this, a bad leader that persists with the same bad
                # assignment only triggers ONE assessment, which may not be
                # enough to cross the suspicion threshold.
                self.leader_assigned_target = new_target
                # Use leader's SEND timestamp, not receive time
                # This is critical for causality-aware trust checking
                self.leader_assignment_time = msg.get('timestamp', time.time())
                # Verify each assignment broadcast using three-tier trust
                self.assess_leader_command()
        
        # Assess peer trust (anomaly detection for non-leaders)
        self.assess_peer_behavior(peer_id)
    
    # ==================== PEER BEHAVIOR ASSESSMENT ====================
    def assess_peer_behavior(self, peer_id: int):
        """
        Anomaly detection for ALL peers (not just leader).
        Catches spinning/stuck behavior so bad peers don't start with full trust
        if they later become leader.
        
        IMPORTANT: When we already suspect the leader (trust_in_leader < TRUST_THRESHOLD),
        we SKIP coverage-stall checks for non-leader peers.  A bad leader's commands cause
        all followers to stall - blaming fellow followers for the leader's commands would
        prevent the swarm from electing a replacement (everyone excludes everyone).
        Only motor anomalies (stuck/spinning) are checked when leader trust is low.
        """
        peer = self.peers[peer_id]
        
        # Track position history for this peer
        history_key = f'_pos_history_{peer_id}'
        if not hasattr(self, history_key):
            setattr(self, history_key, deque(maxlen=20))
        history = getattr(self, history_key)
        
        history.append({
            'x': peer.x, 'y': peer.y, 'theta': peer.theta,
            't': time.time(), 'coverage': peer.coverage_count
        })
        
        if len(history) < 5:
            return
        
        # Check for stuck (no movement)
        first, last = history[0], history[-1]
        dx = last['x'] - first['x']
        dy = last['y'] - first['y']
        distance_moved = math.sqrt(dx*dx + dy*dy)
        time_elapsed = last['t'] - first['t']
        
        # Check for spinning (lots of theta change, little position change)
        theta_changes = 0
        for i in range(1, len(history)):
            dtheta = abs(history[i]['theta'] - history[i-1]['theta'])
            if dtheta > math.pi:
                dtheta = 2 * math.pi - dtheta
            theta_changes += dtheta
        
        anomaly_detected = False
        
        # stuck: moved less than 50mm in 2+ seconds (motor anomaly, always check)
        if time_elapsed > 2.0 and distance_moved < 50:
            anomaly_detected = True
        
        # spinning: high rotation, low translation (motor anomaly, always check)
        if time_elapsed > 1.0 and theta_changes > math.pi and distance_moved < 100:
            anomaly_detected = True
        
        # No coverage progress for extended period (COMMAND-dependent check)
        # Only blame peers for coverage stall when we TRUST the leader.
        # If leader is suspected, coverage stall is the LEADER's fault, not this peer's.
        if self.trust_in_leader >= TRUST_THRESHOLD:
            coverage_progress = last['coverage'] - first['coverage']
            if time_elapsed > 5.0 and coverage_progress <= 0:
                anomaly_detected = True
        
        # Update trust for this peer
        if anomaly_detected:
            peer.my_trust_for_them = max(MIN_TRUST, peer.my_trust_for_them - 0.02)
        else:
            # Slow recovery
            peer.my_trust_for_them = min(1.0, peer.my_trust_for_them + 0.005)
    
    # ==================== HYBRID TRUST MODEL (Three-tier + MPC) ====================
    def assess_leader_command(self):
        """
        REIP Proactive Trust Assessment (novel contribution).
        
        Verifies leader commands BEFORE execution, unlike reactive
        approaches that only detect faults after coverage drops
        (Chandola et al. 2009).
        
        1. Three-tier confidence check (catches "send to explored cell"):
           Evidence weighted by source reliability (Marsh 1994):
           - tier 1: cells i personally visited (weight 1.0) - ground truth
           - tier 2: obstacles within tof range (weight 1.0) - direct sensor
           - tier 3: cells in known_visited from peers (weight 0.3) - may be stale
           All checks are causality-aware: evidence is only used if it
           predates the leader's assignment timestamp, preventing false
           positives from network delay.
        
        2. MPC direction check (catches "send wrong direction"):
           Compares leader's commanded direction against locally computed
           optimal direction (nearest unexplored cell, per Yamauchi 1997).
           Only fires when three-tier ALSO found evidence (reinforcing).
        """
        # ===== ABLATION: no_trust - skip all trust assessment =====
        if getattr(self, '_ablation_no_trust', False):
            return
        
        if self.leader_assigned_target is None:
            return
        
        if self.current_leader is None or self.current_leader == self.robot_id:
            return  # I am leader or no leader
        
        # Only skip if we haven't received ANY position data yet
        if self.x == 0 and self.y == 0 and self.position_timestamp == 0:
            return  # No position data at all
        
        target = self.leader_assigned_target
        cell = self.get_cell(target[0], target[1])
        
        if cell is None:
            return  # Target outside arena
        
        suspicion_added = 0.0
        reasons = []
        
        # ===== THREE-TIER CHECK WITH CAUSALITY =====
        # Only penalize if the cell was known to be explored BEFORE the
        # assignment was made, MINUS a grace period that accounts for
        # broadcast latency in the distributed system.  Without this grace,
        # a cell visited 0.1s before the assignment triggers a false positive
        # because the leader's map hadn't received the update yet.
        
        assignment_time = self.leader_assignment_time
        causality_cutoff = assignment_time - CAUSALITY_GRACE_PERIOD
        dist_to_target = self.distance((self.x, self.y), target)
        
        # Tier 1: Cell I personally visited (absolute ground truth)
        if cell in self.my_visited:
            visited_time = self.my_visited[cell]
            # Only penalize if I visited well BEFORE the assignment
            # (giving time for the info to reach the leader via broadcast)
            if visited_time < causality_cutoff:
                suspicion_added += WEIGHT_PERSONAL
                reasons.append("personal_visited")
            # else: visited too recently - leader's map was stale, not malicious
        
        # Tier 2: Obstacle within ToF range (I can see it NOW)
        elif dist_to_target <= TOF_RANGE and cell in self.tof_obstacles:
            suspicion_added += WEIGHT_TOF
            reasons.append("tof_obstacle")
        
        # Tier 3: Cell in known_visited (peer-reported, with causality)
        elif cell in self.known_visited and cell not in self.my_visited:
            known_time = self.known_visited_time.get(cell, time.time())
            # Only penalize if we knew about it before the assignment
            if known_time < causality_cutoff:
                suspicion_added += WEIGHT_PEER
                reasons.append("peer_reported")
        
        # ===== MPC DIRECTION CHECK =====
        # Two modes:
        # 1. SEVERE mismatch (>135°): Can trigger independently (obvious wrong direction)
        # 2. MODERATE mismatch (90-135°): Only reinforces three-tier evidence
        # (Ablation: no_direction skips this entire check)
        if not getattr(self, '_ablation_no_direction', False):
            mpc_suspicion, mpc_severe = self._compute_mpc_direction_error()
            
            if mpc_severe:
                # Severe direction mismatch - can trigger on its own
                suspicion_added += mpc_suspicion
                reasons.append(f"mpc_severe({mpc_suspicion:.2f})")
            elif suspicion_added > 0 and mpc_suspicion > 0:
                # Moderate mismatch - only reinforces existing evidence
                suspicion_added += mpc_suspicion
                reasons.append(f"mpc_direction({mpc_suspicion:.2f})")
        
        # ===== command stability metric (omega) - fault classification =====
        # Track angular velocity of commanded direction over a sliding window.
        # This is the discrete derivative of θ_cmd(t): Ω = mean(|Δθ|)
        #   Low Ω + high S → Byzantine Assignment (bad_leader)
        #   High Ω + high S → Byzantine Oscillation (oscillate_leader)
        cmd_theta = math.atan2(target[1] - self.y, target[0] - self.x)
        if self._cmd_theta_history:
            prev_theta = self._cmd_theta_history[-1]
            delta_theta = abs(cmd_theta - prev_theta)
            if delta_theta > math.pi:
                delta_theta = 2 * math.pi - delta_theta  # wrap to [0, π]
            self._omega_window.append(delta_theta)
        self._cmd_theta_history.append(cmd_theta)
        
        # Current Ω value (windowed mean angular velocity)
        omega = sum(self._omega_window) / len(self._omega_window) if self._omega_window else 0.0
        
        # ===== COMMAND INSTABILITY DETECTION =====
        # High Ω indicates rapid command flipping (oscillation attack).
        # This can occur even when individual commands pass the three-tier check
        # (e.g., oscillating between two valid unexplored corners).
        # Ω acts as a FOURTH detection signal, generating suspicion independently.
        #
        # Threshold selection (empirical + theoretical):
        #   Normal exploration Ω ≈ 1.6-2.2 (target reassignments produce moderate Δθ)
        #   Oscillation attack Ω ≈ 2.5-3.0 (180° flips every cycle → Δθ ≈ π)
        #   Threshold 3π/4 ≈ 2.356 (135°) sits between these distributions.
        #   Suspicion scales linearly from 0 at threshold to OMEGA_WEIGHT at Ω=π.
        OMEGA_DETECT_THRESHOLD = 3 * math.pi / 4  # ~2.356 rad (135°)
        OMEGA_WEIGHT = 0.3  # Max suspicion per command at Ω=π
        leader_tenure = time.time() - self._leader_tenure_start
        if (len(self._omega_window) >= self._omega_window.maxlen
                and omega >= OMEGA_DETECT_THRESHOLD
                and leader_tenure >= self._OMEGA_GRACE_PERIOD):
            # Scale suspicion by excess above threshold: 0 at threshold, OMEGA_WEIGHT at π
            omega_excess = (omega - OMEGA_DETECT_THRESHOLD) / (math.pi - OMEGA_DETECT_THRESHOLD)
            omega_suspicion = OMEGA_WEIGHT * min(omega_excess, 1.0)
            suspicion_added += omega_suspicion
            reasons.append(f"cmd_instability(omega={omega:.2f})")
        
        # ===== UPDATE SUSPICION =====
        if suspicion_added > 0:
            self.suspicion_of_leader += suspicion_added
            self.bad_commands_received += 1
            if self.first_bad_command_time is None:
                self.first_bad_command_time = time.time()
            print(f"[TRUST] Bad command #{self.bad_commands_received} ({', '.join(reasons)}): "
                  f"suspicion +{suspicion_added:.2f} = {self.suspicion_of_leader:.2f} "
                  f"omega={omega:.3f}")
        else:
            # Good command - recover slowly
            self.suspicion_of_leader = max(0, self.suspicion_of_leader - RECOVERY_RATE)
        
        # Check threshold (with carry-over, not reset)
        if self.suspicion_of_leader >= SUSPICION_THRESHOLD:
            self.trust_in_leader = max(MIN_TRUST, self.trust_in_leader - TRUST_DECAY_RATE)
            self.suspicion_of_leader -= SUSPICION_THRESHOLD  # Carry over excess
            
            # ===== FAULT CLASSIFICATION at first trust decay =====
            # Ω threshold: oscillation produces Δθ ≈ π each cycle → Ω ≈ π
            # Bad leader produces Δθ ≈ 0 (same target) → Ω ≈ 0
            # Threshold π/4 (45°) cleanly separates the two distributions.
            OMEGA_CLASSIFY_THRESHOLD = math.pi / 4  # ~0.785 rad
            
            if self.first_decay_time is None:
                self.first_decay_time = time.time()
                
                # Classify the fault type
                if omega >= OMEGA_CLASSIFY_THRESHOLD:
                    self.classified_fault_type = "byzantine_oscillation"
                else:
                    self.classified_fault_type = "byzantine_assignment"
                
                if self.first_bad_command_time:
                    dt = self.first_decay_time - self.first_bad_command_time
                    print(f"[DETECT] First trust decay after {self.bad_commands_received} bad commands "
                          f"({dt:.2f}s) [Theoretical bound: T1={WORST_CASE_DETECT_T1}, T3={WORST_CASE_DETECT_T3}]")
                print(f"[CLASSIFY] Fault type: {self.classified_fault_type} "
                      f"(omega={omega:.3f}, threshold={OMEGA_CLASSIFY_THRESHOLD:.3f})")
            
            print(f"[TRUST] Trust decayed: trust={self.trust_in_leader:.2f}, "
                  f"remaining suspicion={self.suspicion_of_leader:.2f}, "
                  f"omega={omega:.3f}, class={self.classified_fault_type}")
    
    def _compute_mpc_direction_error(self) -> tuple:
        """
        MPC direction check: Is leader's command at least toward SOME unexplored area?
        
        Key insight: A good leader might send you to a frontier that isn't the centroid.
        But a bad leader sends you AWAY from ALL unexplored areas.
        
        So we check: Is the commanded direction within 90° of ANY unexplored cell?
        If not within 90° of ANY frontier → severe mismatch.
        
        Returns: (suspicion_weight, is_severe)
        """
        if not self.leader_assigned_target:
            return 0.0, False
        
        # Direction from me to leader's command
        cmd_dir = math.atan2(self.leader_assigned_target[1] - self.y,
                            self.leader_assigned_target[0] - self.x)
        
        # Find unexplored cells
        unexplored = []
        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                if (cx, cy) not in self.known_visited:
                    unexplored.append(self.cell_to_pos((cx, cy)))
        
        if not unexplored:
            return 0.0, False  # Everything explored, can't check
        
        # Check if command direction is within 90° of ANY unexplored cell
        # A good leader might send you to any of these, so we check the BEST alignment
        best_alignment = math.pi  # Start with worst case (180°)
        
        for frontier_pos in unexplored:
            frontier_dir = math.atan2(frontier_pos[1] - self.y, frontier_pos[0] - self.x)
            angle_diff = abs(frontier_dir - cmd_dir)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff < best_alignment:
                best_alignment = angle_diff
            
            # Early exit if we find good alignment
            if best_alignment < math.pi / 4:  # Within 45° of some frontier
                return 0.0, False
        
        # Now check if even the BEST alignment is bad
        SEVERE_THRESHOLD = math.pi * 3 / 4  # 135° from nearest frontier
        MODERATE_THRESHOLD = math.pi / 2     # 90° from nearest frontier
        
        if best_alignment > SEVERE_THRESHOLD:
            # Not even close to ANY frontier - severe
            error = 0.5 + (best_alignment - SEVERE_THRESHOLD) / (math.pi - SEVERE_THRESHOLD) * 0.3
            print(f"[MPC SEVERE] No frontier within {math.degrees(SEVERE_THRESHOLD):.0f}° of cmd! "
                  f"Best alignment: {math.degrees(best_alignment):.0f}°, error={error:.2f}")
            return error, True
        elif best_alignment > MODERATE_THRESHOLD:
            # Somewhat misaligned with nearest frontier - moderate
            error = (best_alignment - MODERATE_THRESHOLD) / (SEVERE_THRESHOLD - MODERATE_THRESHOLD) * 0.3
            # Only print if significant
            if error > 0.1:
                print(f"[MPC] Moderate mismatch: best alignment {math.degrees(best_alignment):.0f}°, error={error:.2f}")
            return error, False
        
        return 0.0, False
    
    def update_tof_obstacles(self):
        """
        Update obstacle map from ToF readings.
        INSTANTANEOUS - clears and rebuilds each cycle to avoid stale data
        (e.g., another robot temporarily blocking a cell).
        """
        # Clear previous - obstacles are instantaneous, not accumulated
        self.tof_obstacles.clear()
        
        if not self.tof:
            return
        
        # ToF sensor angles relative to robot heading
        sensor_angles = {
            'right': -math.pi/2.4,      # -75 deg
            'front_right': -math.pi/4.8, # -37.5 deg
            'front': 0,
            'front_left': math.pi/4.8,   # 37.5 deg
            'left': math.pi/2.4          # 75 deg
        }
        
        for name, reading in self.tof.items():
            if reading <= 30 or reading > TOF_RANGE:
                continue
            
            angle = self.theta + sensor_angles.get(name, 0)
            obs_x = self.x + reading * math.cos(angle)
            obs_y = self.y + reading * math.sin(angle)
            
            cell = self.get_cell(obs_x, obs_y)
            if cell:
                self.tof_obstacles.add(cell)
    
    # ==================== LEADER TASK ASSIGNMENT ====================
    def compute_task_assignments(self) -> Dict[str, Tuple[float, float]]:
        """
        Greedy nearest-frontier assignment (Burgard et al. 2000).
        Leader computes frontier assignments for all robots including self.
        Sorts robots by distance to nearest frontier (not by ID) to avoid
        systematic bias toward low-ID robots.
        
        If bad_leader_mode is active, deliberately assigns already-explored cells
        to trigger three-tier trust detection in followers.
        """
        if self.state != RobotState.LEADER:
            return {}
        
        # BAD LEADER MODE: assign explored cells instead of frontiers
        # This triggers the three-tier trust model (the main REIP feature)
        if self.bad_leader_mode:
            return self._compute_bad_assignments()
        
        # OSCILLATE LEADER MODE: rapidly flip-flop targets between far corners
        # Tests Tier 3 direction consistency (high Ω in fault classifier)
        if self.oscillate_leader_mode:
            return self._compute_oscillate_assignments()
        
        # FREEZE LEADER MODE: stop updating assignments entirely
        # Followers keep stale targets → as those cells get explored, coverage stalls
        if self.freeze_leader_mode:
            return self._compute_freeze_assignments()
        
        # Find all unexplored cells (frontiers), excluding wall-adjacent cells
        frontiers = []
        frontier_set = set()
        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                cell = (cx, cy)
                if cell not in self.known_visited and not self._is_wall_cell(cx, cy):
                    frontiers.append(cell)
                    frontier_set.add(cell)
        
        if not frontiers:
            self.my_assigned_target = None
            self._prev_assignments.clear()
            return {}
        
        # Collect all robot positions (me + peers)
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen < PEER_TIMEOUT:
                robots[pid] = (peer.x, peer.y)

        # Passage yielding: the passage (y>1200, x near divider) is ~190mm
        # after margins — one robot fits, two jam.  Count how many robots
        # are currently in the passage zone so the assignment logic can
        # avoid sending more than one through at a time.
        PASSAGE_Y_START = INTERIOR_WALL_Y_END          # 1200mm
        PASSAGE_X_MIN = INTERIOR_WALL_X_LEFT - ROBOT_RADIUS   # ~890mm
        PASSAGE_X_MAX = INTERIOR_WALL_X_RIGHT + ROBOT_RADIUS  # ~1146mm
        robots_in_passage = set()
        for rid, pos in robots.items():
            if pos[1] > PASSAGE_Y_START and PASSAGE_X_MIN < pos[0] < PASSAGE_X_MAX:
                robots_in_passage.add(rid)

        # === ASSIGNMENT PERSISTENCE (Hysteresis) ===
        # Keep previous assignments if their target frontier is still unexplored
        # AND the robot hasn't already reached it.  The proximity check closes
        # the loop when WiFi drops prevent visited_cells from arriving: the
        # leader can see the robot is at its target and will reassign.
        still_valid = {}
        assigned_frontiers = set()
        for rid_str, prev_target in self._prev_assignments.items():
            rid = int(rid_str)
            if rid in robots:
                cell = self.pos_to_cell(prev_target)
                if cell in frontier_set:
                    pos = robots[rid]
                    dist = math.sqrt((pos[0] - prev_target[0])**2 +
                                     (pos[1] - prev_target[1])**2)
                    if dist < CELL_SIZE:
                        # Robot reached target — treat as explored, reassign
                        self.known_visited.add(cell)
                        self.known_visited_time[cell] = time.time()
                        continue
                    still_valid[rid_str] = prev_target
                    assigned_frontiers.add(cell)
        
        # Robots that need new assignments
        need_assignment = [rid for rid in robots if str(rid) not in still_valid]
        
        # Greedy assignment for robots without valid targets
        # Sort by distance to nearest frontier (closest first)
        available_frontiers = [f for f in frontiers if f not in assigned_frontiers]
        
        if need_assignment and available_frontiers:
            def nearest_frontier_dist(rid):
                pos = robots[rid]
                return min(self.path_distance(pos, self.cell_to_pos(f))
                           for f in available_frontiers)
            sorted_robots = sorted(need_assignment, key=nearest_frontier_dist)
            
            MIN_TARGET_SPACING = CELL_SIZE * 3
            wall_cx = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)

            for rid in sorted_robots:
                pos = robots[rid]
                robot_on_left = pos[0] < INTERIOR_WALL_X_LEFT
                best_frontier = None
                best_dist = float('inf')

                # Passage yielding: if someone else is already in the
                # passage, don't send this robot cross-room (it would
                # jam the passage).  The robot already IN the passage
                # keeps its assignment.
                passage_blocked = (len(robots_in_passage) > 0 and
                                   rid not in robots_in_passage)

                for frontier in available_frontiers:
                    if frontier in assigned_frontiers:
                        continue
                    frontier_pos = self.cell_to_pos(frontier)

                    # Skip cross-room frontiers if passage is blocked
                    frontier_on_left = frontier[0] < wall_cx
                    if passage_blocked and frontier_on_left != robot_on_left:
                        continue

                    too_close = False
                    for _, existing_tgt in still_valid.items():
                        if self.distance(frontier_pos, existing_tgt) < MIN_TARGET_SPACING:
                            too_close = True
                            break
                    if too_close:
                        continue

                    dist = self.path_distance(pos, frontier_pos)
                    if dist < best_dist:
                        best_dist = dist
                        best_frontier = frontier

                if not best_frontier:
                    for frontier in available_frontiers:
                        if frontier in assigned_frontiers:
                            continue
                        frontier_pos = self.cell_to_pos(frontier)
                        dist = self.path_distance(pos, frontier_pos)
                        if dist < best_dist:
                            best_dist = dist
                            best_frontier = frontier

                if best_frontier:
                    assigned_frontiers.add(best_frontier)
                    target = self.cell_to_pos(best_frontier)
                    still_valid[str(rid)] = target
        
        # Spatial diversity: ensure at least one robot is assigned to
        # the other room if it has unexplored cells.  Without this, the
        # greedy algorithm keeps all robots in whichever room they started.
        wall_cx = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)
        room_a_assigned = 0
        room_b_assigned = 0
        for _, tgt in still_valid.items():
            tcx = int(tgt[0] / CELL_SIZE)
            if tcx < wall_cx:
                room_a_assigned += 1
            else:
                room_b_assigned += 1

        room_b_frontiers = [f for f in available_frontiers
                            if f not in assigned_frontiers and f[0] >= wall_cx]
        room_a_frontiers = [f for f in available_frontiers
                            if f not in assigned_frontiers and f[0] < wall_cx]

        # Spatial diversity: ensure at least one robot targets each room —
        # BUT only if the passage is clear.  Forcing a cross-room assignment
        # while someone is already in the passage causes the pile-up.
        if len(robots_in_passage) == 0:
            if room_b_assigned == 0 and room_b_frontiers and len(still_valid) >= 2:
                worst_rid = None
                worst_dist = -1
                for rid_str, tgt in still_valid.items():
                    rid = int(rid_str)
                    if rid == self.robot_id:
                        continue
                    if rid in robots:
                        d = self.path_distance(robots[rid], tgt)
                        if d > worst_dist:
                            worst_dist = d
                            worst_rid = rid_str
                if worst_rid is not None:
                    rid = int(worst_rid)
                    pos = robots[rid]
                    best_b = min(room_b_frontiers,
                                 key=lambda f: self.path_distance(pos, self.cell_to_pos(f)))
                    still_valid[worst_rid] = self.cell_to_pos(best_b)

            if room_a_assigned == 0 and room_a_frontiers and len(still_valid) >= 2:
                worst_rid = None
                worst_dist = -1
                for rid_str, tgt in still_valid.items():
                    rid = int(rid_str)
                    if rid == self.robot_id:
                        continue
                    if rid in robots:
                        d = self.path_distance(robots[rid], tgt)
                        if d > worst_dist:
                            worst_dist = d
                            worst_rid = rid_str
                if worst_rid is not None:
                    rid = int(worst_rid)
                    pos = robots[rid]
                    best_a = min(room_a_frontiers,
                                 key=lambda f: self.path_distance(pos, self.cell_to_pos(f)))
                    still_valid[worst_rid] = self.cell_to_pos(best_a)

        # Build final assignments
        assignments = dict(still_valid)
        
        # Update leader's own target and cache
        if str(self.robot_id) in assignments:
            self.my_assigned_target = assignments[str(self.robot_id)]
        self._prev_assignments = dict(assignments)
        
        return assignments
    
    def _compute_bad_assignments(self) -> Dict[str, Tuple[float, float]]:
        """
        BAD LEADER: deliberately assign already-explored cells.
        This simulates a compromised/hallucinating leader that fixates on
        territory it believes needs coverage.  assignments are persisted -
        each robot gets the SAME bad target every broadcast until a new
        election occurs.  This prevents followers from accidentally covering
        the arena via a high-speed random walk.
        """
        # Find explored cells to send robots to (the bad assignments)
        explored = list(self.known_visited)
        if not explored:
            # No explored cells yet, can't be bad
            return {}
        
        # Collect all robot positions
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen < PEER_TIMEOUT:
                robots[pid] = (peer.x, peer.y)
        
        # Persist assignments: pick bad targets ONCE, reuse until election
        if not hasattr(self, '_bad_assignments_cache'):
            self._bad_assignments_cache = {}
        
        assignments = {}
        for rid in robots:
            if str(rid) not in self._bad_assignments_cache:
                # first time seeing this robot - pick a fixed bad target
                bad_cell = random.choice(explored)
                self._bad_assignments_cache[str(rid)] = self.cell_to_pos(bad_cell)
            
            target = self._bad_assignments_cache[str(rid)]
            assignments[str(rid)] = target
            
            if rid == self.robot_id:
                self.my_assigned_target = target
        
        if assignments:
            print(f"[BAD_LEADER] Sending {len(assignments)} robots to EXPLORED cells (persistent): {list(assignments.values())[:3]}...")
        return assignments
    
    def _compute_oscillate_assignments(self) -> Dict[str, Tuple[float, float]]:
        """
        OSCILLATE LEADER: rapidly alternate targets between two far corners.
        
        Every broadcast cycle, the leader flips all followers between
        corner A (top-left) and corner B (bottom-right).  Followers constantly
        reverse direction, effectively oscillating in place.  Coverage stalls.
        
        Detection signature:
          - Tier 1 (personal_visited): targets are unexplored corners → may NOT trigger
          - Tier 3 (direction consistency): commanded vector flips 180° each cycle
            while predicted vector stays stable → high angular velocity Ω → triggers
          - Fault classifier: high S(t) + high Ω(t) → "Byzantine Oscillation"
        
        This tests a fundamentally different mechanism than bad_leader:
          bad_leader = constant wrong direction (low Ω, caught by Tier 1)
          oscillate  = flip-flopping direction (high Ω, caught by Tier 3)
        """
        # Two corners of the arena, offset by robot radius to be reachable
        margin = ROBOT_RADIUS + CELL_SIZE
        corner_a = (margin, margin)
        corner_b = (ARENA_WIDTH - margin, ARENA_HEIGHT - margin)
        
        # Alternate phase each call (called every broadcast at BROADCAST_RATE)
        self._oscillate_phase = 1 - self._oscillate_phase
        target = corner_a if self._oscillate_phase == 0 else corner_b
        
        # Collect all robot positions
        robots = {self.robot_id: (self.x, self.y)}
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen < PEER_TIMEOUT:
                robots[pid] = (peer.x, peer.y)
        
        # ALL robots get the SAME target, which flips every cycle
        assignments = {}
        for rid in robots:
            assignments[str(rid)] = target
            if rid == self.robot_id:
                self.my_assigned_target = target
        
        print(f"[OSCILLATE] Phase {self._oscillate_phase}: all {len(assignments)} "
              f"robots -> {('A' if self._oscillate_phase == 0 else 'B')} "
              f"({target[0]:.0f}, {target[1]:.0f})")
        return assignments
    
    def _compute_freeze_assignments(self) -> Dict[str, Tuple[float, float]]:
        """
        FREEZE LEADER: stop updating assignments entirely.
        
        On first call, snapshots whatever the current assignments are (or assigns
        the leader's own position if none exist).  From then on, the leader
        returns the exact same stale targets forever.
        
        Detection signature:
          - Tier 1 (personal_visited): once a frozen target is explored, followers
            are commanded to already-visited cells -> trust decay
          - Tier 3 (direction consistency): stale target = stable direction,
            but predicted (local optimal) keeps changing -> moderate mismatch
          - The fault is PASSIVE: the leader just stopped doing its job.
            REIP should detect it via coverage stagnation + Tier 1 checks.
            Raft has NO detection mechanism and will stall indefinitely.
        """
        if not self._frozen_assignments:
            # First call: snapshot current assignments (or generate fallback)
            if self._prev_assignments:
                self._frozen_assignments = dict(self._prev_assignments)
            else:
                # no previous assignments - assign everyone to the leader's position
                robots = {self.robot_id: (self.x, self.y)}
                for pid, peer in self.peers.items():
                    if time.time() - peer.last_seen < PEER_TIMEOUT:
                        robots[pid] = (peer.x, peer.y)
                for rid in robots:
                    self._frozen_assignments[str(rid)] = (self.x, self.y)
            
            print(f"[FREEZE_LEADER] Frozen {len(self._frozen_assignments)} assignments "
                  f"(will never update)")
        
        # Always return the same stale assignments
        # Set own target from frozen cache
        my_key = str(self.robot_id)
        if my_key in self._frozen_assignments:
            self.my_assigned_target = self._frozen_assignments[my_key]
        
        return dict(self._frozen_assignments)
    
    def get_my_frontier(self) -> Optional[Tuple[float, float]]:
        """Compute nearest unexplored cell (Yamauchi 1997 frontier-based).
        Uses path-aware distance: crossing the interior wall adds the
        actual detour through the passage so robots explore their own
        side first instead of all rushing the passage at once."""
        my_cell = self.get_cell(self.x, self.y)
        if not my_cell:
            return None

        wall_cell_x_left = int(INTERIOR_WALL_X_LEFT / CELL_SIZE)
        wall_cell_x_right = int(INTERIOR_WALL_X_RIGHT / CELL_SIZE) + 1
        wall_cell_y_end = int(INTERIOR_WALL_Y_END / CELL_SIZE)
        robot_on_left = my_cell[0] < wall_cell_x_left

        # Check if any peer is currently in the passage
        PASSAGE_Y_START = INTERIOR_WALL_Y_END
        PASSAGE_X_MIN = INTERIOR_WALL_X_LEFT - ROBOT_RADIUS
        PASSAGE_X_MAX = INTERIOR_WALL_X_RIGHT + ROBOT_RADIUS
        peer_in_passage = False
        me_in_passage = (self.y > PASSAGE_Y_START and
                         PASSAGE_X_MIN < self.x < PASSAGE_X_MAX)
        if not me_in_passage:
            for p in self.peers.values():
                if (time.time() - p.last_seen < PEER_TIMEOUT and
                    p.y > PASSAGE_Y_START and
                    PASSAGE_X_MIN < p.x < PASSAGE_X_MAX):
                    peer_in_passage = True
                    break

        best_dist = float('inf')
        best_cell = None

        for cx in range(self.coverage_width):
            for cy in range(self.coverage_height):
                cell = (cx, cy)
                if cell in self.known_visited or self._is_wall_cell(cx, cy):
                    continue

                cell_on_left = cx < wall_cell_x_left
                # Hard-skip cross-room if passage is occupied by a peer
                if peer_in_passage and cell_on_left != robot_on_left:
                    continue

                dist = abs(cx - my_cell[0]) + abs(cy - my_cell[1])

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
    
    # ==================== ELECTION & IMPEACHMENT ====================
    def run_election(self):
        """Trust-weighted leader election (novel REIP contribution).
        Unlike RAFT (Ongaro 2014) which uses term numbers and log
        completeness, REIP elections filter candidates by trust score
        and tie-break by leadership failure history."""
        # FIRST: Update peer trust for current leader (before building candidates)
        if self.current_leader and self.current_leader != self.robot_id:
            if self.current_leader in self.peers:
                self.peers[self.current_leader].my_trust_for_them = self.trust_in_leader
        
        # Build candidates: myself + all peers with sufficient trust
        candidates = [(self.robot_id, 1.0, self.leader_failures.get(self.robot_id, 0))]
        
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen < PEER_TIMEOUT:
                # Use UPDATED trust values (including leader's decayed trust)
                if peer.my_trust_for_them > TRUST_THRESHOLD:
                    failures = self.leader_failures.get(pid, 0)
                    candidates.append((pid, peer.my_trust_for_them, failures))
        
        # Sort by: highest trust, then fewest failures, then lowest ID
        # (trust desc, failures asc, id asc)
        candidates.sort(key=lambda x: (-x[1], x[2], x[0]))
        
        if candidates:
            self.my_vote = candidates[0][0]
            
            # count votes from peers, but only for eligible candidates.
            # A robot excluded from OUR candidate list (e.g. a bad leader with
            # trust below threshold) should not win just because it votes for
            # itself.  Each node enforces its own trust assessment.
            eligible_ids = set(c[0] for c in candidates)
            votes = {self.my_vote: 1}  # my_vote is always eligible
            for pid, peer in self.peers.items():
                if peer.vote is not None and time.time() - peer.last_seen < PEER_TIMEOUT:
                    if peer.vote in eligible_ids:
                        votes[peer.vote] = votes.get(peer.vote, 0) + 1
            
            # Winner by: most votes -> fewest failures -> lowest ID
            old_leader = self.current_leader
            # Sort by (-votes, failures, id) so first item wins
            sorted_candidates = sorted(votes.items(), 
                key=lambda x: (-x[1], self.leader_failures.get(x[0], 0), x[0]))
            
            self.current_leader = sorted_candidates[0][0]
            
            if self.current_leader != old_leader:
                # Track failure for old leader (they got impeached)
                # don't count the initial startup convergence - when robots first
                # discover peers, their self-election gets overridden.  This is normal
                # convergence, not a real impeachment, and should not bias tie-breaking.
                if old_leader is not None and self._election_settled:
                    self.leader_failures[old_leader] = self.leader_failures.get(old_leader, 0) + 1
                    print(f"[ELECTION] Leader {old_leader} impeached (failures={self.leader_failures[old_leader]})")
                if not self._election_settled and len(self.peers) >= 2:
                    # first election with peers present - startup is done
                    self._election_settled = True
                
                print(f"[ELECTION] New leader: {self.current_leader}")
                # Reset trust for new leader
                self.trust_in_leader = 1.0
                self.suspicion_of_leader = 0.0
                # Reset detection timing counters for new leader
                self.bad_commands_received = 0
                self.first_bad_command_time = None
                self.first_decay_time = None
                self.impeachment_time = None
                # Reset fault classifier state for new leader
                self._cmd_theta_history.clear()
                self._omega_window.clear()
                self.classified_fault_type = None
                self._leader_tenure_start = time.time()
                # Clear assignment cache (stale for new leader)
                self._prev_assignments.clear()
                # Clear fault modes if we were the bad leader
                if old_leader == self.robot_id:
                    self.bad_leader_mode = False
                    self.oscillate_leader_mode = False
                    self.freeze_leader_mode = False
                    self._oscillate_phase = 0
                    self._frozen_assignments.clear()
                    if hasattr(self, '_bad_assignments_cache'):
                        self._bad_assignments_cache.clear()
            
            # Split-brain resolution: if I think I'm leader but a trusted
            # peer with a lower ID also claims leadership, yield to them.
            # This converges within one election cycle even under packet loss.
            if self.current_leader == self.robot_id:
                for pid, peer in self.peers.items():
                    if (peer.state == 'leader' and pid < self.robot_id and
                        time.time() - peer.last_seen < PEER_TIMEOUT and
                        peer.my_trust_for_them > TRUST_THRESHOLD):
                        self.current_leader = pid
                        self.trust_in_leader = 1.0
                        self.suspicion_of_leader = 0.0
                        self._prev_assignments.clear()
                        print(f"[ELECTION] Yielding to lower-ID leader R{pid}")
                        break

            if self.current_leader == self.robot_id:
                self.state = RobotState.LEADER
            else:
                self.state = RobotState.FOLLOWER
    
    def check_impeachment(self):
        """Check if I should vote to impeach the leader (called BEFORE election)"""
        if self.current_leader is None or self.current_leader == self.robot_id:
            return
        
        if self.trust_in_leader < IMPEACHMENT_THRESHOLD:
            if self.impeachment_time is None and self.first_bad_command_time:
                self.impeachment_time = time.time()
                dt = self.impeachment_time - self.first_bad_command_time
                print(f"[DETECT] IMPEACHMENT after {self.bad_commands_received} bad commands, "
                      f"{dt:.2f}s from first bad command "
                      f"[Bound: T1={WORST_CASE_IMPEACH_T1} cmds, T3={WORST_CASE_IMPEACH_T3} cmds]")
            print(f"[IMPEACH] Trust too low ({self.trust_in_leader:.2f}), excluding leader {self.current_leader} from candidates")
    
    # ==================== NAVIGATION ====================
    # Hardware-specific additions below replicate what visual_sim.py provides
    # via its physics engine:  wall collision + sliding, A* routing, stuck
    # override.  The REIP algorithm (target selection, trust, elections) is
    # untouched - only the motor-command layer differs.

    def _init_stuck_detection(self):
        self._nav_history = []
        self._escape_until = 0
        self._escape_start = 0
        self._stuck_count = 0
        self._tof_emergency_count = 0

    def _init_heading_pd(self):
        self._smooth_theta = None       # EMA-filtered heading
        self._prev_heading_err = None   # previous error for D term
        self._prev_heading_t = 0.0      # timestamp of previous error
        self._in_pivot = False          # hysteresis flag for pivot turns

    def _check_stuck(self):
        """Detect if robot is physically stuck (wall-grinding, spinning wheels).
        Skips during pivot turns — the robot intentionally stays in place."""
        if self._in_pivot:
            self._nav_history.clear()
            return False

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

    def _nearest_navigable_cell(self, cx, cy):
        """BFS from a wall cell to find the closest non-wall cell."""
        from collections import deque
        W, H = self.coverage_width, self.coverage_height
        q = deque([(cx, cy)])
        seen = {(cx, cy)}
        while q:
            x, y = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in seen:
                    seen.add((nx, ny))
                    if not self._is_wall_cell(nx, ny):
                        return (nx, ny)
                    q.append((nx, ny))
        return None

    def _route_around_wall(self, target):
        """A* pathfinding on the cell grid — mirrors sim's GridWorld pathfinder.
        Returns the next waypoint (center of the second cell on the path)
        so the reactive heading controller drives toward it."""
        import heapq

        my_cell = self.get_cell(self.x, self.y)
        tgt_cell = self.get_cell(target[0], target[1])
        if my_cell is None or tgt_cell is None or my_cell == tgt_cell:
            return target

        W = self.coverage_width
        H = self.coverage_height
        tof_blocked = self.tof_obstacles

        # If starting in a wall cell (corner/edge), find the nearest
        # navigable cell and head there first.  A* can't route out of
        # wall cells because all 4-connected neighbors are also walls.
        if self._is_wall_cell(*my_cell):
            escape = self._nearest_navigable_cell(*my_cell)
            if escape:
                return self.cell_to_pos(escape)
            return target

        # If target is in a wall cell, snap to nearest navigable cell
        if self._is_wall_cell(*tgt_cell):
            snap = self._nearest_navigable_cell(*tgt_cell)
            if snap:
                tgt_cell = snap
                target = self.cell_to_pos(snap)

        def neighbors(cx, cy):
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < W and 0 <= ny < H:
                    if self._is_wall_cell(nx, ny):
                        continue
                    if (nx, ny) in tof_blocked and (nx, ny) != tgt_cell:
                        continue
                    yield (nx, ny)

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        start = my_cell
        goal = tgt_cell
        open_set = [(heuristic(start, goal), 0, start)]
        came_from = {start: None}
        g_score = {start: 0}

        while open_set:
            _, g, cur = heapq.heappop(open_set)
            if cur == goal:
                path_cell = goal
                while came_from.get(path_cell) is not None and came_from[path_cell] != start:
                    path_cell = came_from[path_cell]
                return self.cell_to_pos(path_cell)
            if g > g_score.get(cur, float('inf')):
                continue
            for nb in neighbors(*cur):
                ng = g + 1
                if ng < g_score.get(nb, float('inf')):
                    g_score[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + heuristic(nb, goal), ng, nb))

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

        # Repulsion around both wall tip corners.
        for tip_x in [INTERIOR_WALL_X_LEFT, INTERIOR_WALL_X_RIGHT]:
            tip_dx = self.x - tip_x
            tip_dy = self.y - INTERIOR_WALL_Y_END
            tip_dist = math.sqrt(tip_dx * tip_dx + tip_dy * tip_dy)
            TIP_ZONE = DIVIDER_MARGIN + 30
            if tip_dist < TIP_ZONE and tip_dist > 5 and self.y < INTERIOR_WALL_Y_END + 50:
                tip_strength = _repel(tip_dist, TIP_ZONE)
                repel_x += (tip_dx / tip_dist) * tip_strength
                repel_y += (tip_dy / tip_dist) * tip_strength

        REPEL_GAIN = 8.0
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
        """Steer away from nearby peers using their broadcast positions.
        At very close range, overrides heading to flee. Adds a per-robot
        random offset to break symmetry when surrounded.
        NOTE: peer positions are updated at BROADCAST_RATE (5 Hz), so they
        can be up to 200ms stale. Use a generous avoidance distance."""
        PEER_AVOID_DIST = 400   # was 350, wider to compensate for stale positions
        PEER_REPEL_GAIN = 4.0   # was 3.0, stronger push when close

        hx = math.cos(desired_angle)
        hy = math.sin(desired_angle)

        repel_x, repel_y = 0.0, 0.0
        closest_dist = 9999.0
        n_close = 0
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen > PEER_TIMEOUT:
                continue
            dx = self.x - peer.x
            dy = self.y - peer.y
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

        # Symmetry breaker: when surrounded by multiple peers, repulsion
        # vectors can cancel out. Add a per-robot deterministic offset
        # so each robot picks a different escape direction.
        if n_close >= 2 and (abs(repel_x) + abs(repel_y)) < 0.5:
            offset_angle = (self.robot_id * 1.2566)  # 72 degrees apart for 5 robots
            repel_x += math.cos(offset_angle) * 0.8
            repel_y += math.sin(offset_angle) * 0.8

        if abs(repel_x) < 0.01 and abs(repel_y) < 0.01:
            return desired_angle

        if closest_dist < ROBOT_RADIUS:
            return math.atan2(repel_y, repel_x)

        hx += repel_x * PEER_REPEL_GAIN
        hy += repel_y * PEER_REPEL_GAIN
        return math.atan2(hy, hx)

    def compute_motor_command(self) -> Tuple[float, float]:
        """Compute motor speeds.

        REIP logic (faults, target selection) is identical to the sim.
        Hardware additions mirror what the sim's physics engine provides:
          - _wall_slide_heading  →  sim's wall collision + component motion
          - _route_around_wall   →  sim's A* pathfinding
          - _check_stuck + escape →  sim's stuck_counter + override
        """
        if not hasattr(self, '_nav_history'):
            self._init_stuck_detection()
        if not hasattr(self, '_smooth_theta'):
            self._init_heading_pd()

        # Handle injected faults (identical to sim)
        if self.injected_fault == 'spin':
            return (BASE_SPEED, -BASE_SPEED)
        elif self.injected_fault == 'stop':
            return (0, 0)
        elif self.injected_fault == 'erratic':
            return (random.uniform(-BASE_SPEED, BASE_SPEED),
                   random.uniform(-BASE_SPEED, BASE_SPEED))

        # Escape mode: pivot toward escape angle, then drive away.
        # When in a wall zone, skip blind reverse — reversing pushes
        # the robot deeper into the wall it's already stuck against.
        if time.time() < self._escape_until:
            phase_elapsed = time.time() - self._escape_start
            escape_angle = getattr(self, '_escape_angle', math.atan2(
                ARENA_HEIGHT / 2 - self.y, ARENA_WIDTH / 2 - self.x))

            in_wall_zone = (
                self.x < OUTER_WALL_MARGIN + 50 or
                self.x > ARENA_WIDTH - OUTER_WALL_MARGIN - 50 or
                self.y < OUTER_WALL_MARGIN + 50 or
                self.y > ARENA_HEIGHT - OUTER_WALL_MARGIN - 50 or
                (self.y < INTERIOR_WALL_Y_END and
                 abs(self.x - INTERIOR_WALL_X) < DIVIDER_MARGIN + 80)
            )

            reverse_dur = 0.0 if in_wall_zone else 0.4
            pivot_end = reverse_dur + 0.6

            if phase_elapsed < reverse_dur:
                return (-BASE_SPEED * 0.6, -BASE_SPEED * 0.6)
            elif phase_elapsed < pivot_end:
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

        # ToF emergency: FRONT-FACING sensors only (front, front_left, front_right).
        # Side sensors read close when driving parallel to walls — that's normal
        # and must NOT trigger emergency reversal.
        TOF_EMERGENCY_DIST = 120

        front_d = self.tof.get('front', 9999)
        fl_d = self.tof.get('front_left', 9999)
        fr_d = self.tof.get('front_right', 9999)
        left_d = self.tof.get('left', 9999)
        right_d = self.tof.get('right', 9999)
        if front_d <= 0: front_d = 9999
        if fl_d <= 0: fl_d = 9999
        if fr_d <= 0: fr_d = 9999
        if left_d <= 0: left_d = 9999
        if right_d <= 0: right_d = 9999
        min_front = min(front_d, fl_d, fr_d)

        if min_front > 20 and min_front < TOF_EMERGENCY_DIST:
            self._tof_emergency_count += 1
            if self._tof_emergency_count >= 3:
                self._tof_emergency_count = 0
                self._stuck_count += 1
                escape_time = min(2.0 + self._stuck_count * 0.5, 4.0)
                self._escape_until = time.time() + escape_time
                self._escape_start = time.time()
                self._nav_history.clear()
                # Flee from nearest wall/divider, not blindly toward center
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
            # Reverse away from whichever side is closest
            rev = BASE_SPEED * 0.6
            if left_d < right_d and left_d < front_d:
                return (-rev * 0.3, -rev)       # reverse turning right
            elif right_d < left_d and right_d < front_d:
                return (-rev, -rev * 0.3)       # reverse turning left
            elif fr_d < fl_d:
                return (-rev, -rev * 0.4)
            else:
                return (-rev * 0.4, -rev)
        else:
            self._tof_emergency_count = 0

        # Peer collision: if touching another robot, pivot away immediately.
        # Don't drive forward at all — forward motion just pushes both robots
        # together harder. Pure rotation to face the flee direction, then the
        # normal nav code takes over once there's clearance.
        _closest_peer_dist = 9999.0
        _flee_px, _flee_py = 0.0, 0.0
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen > PEER_TIMEOUT:
                continue
            pdx = self.x - peer.x
            pdy = self.y - peer.y
            pdist = math.sqrt(pdx * pdx + pdy * pdy)
            if pdist < _closest_peer_dist:
                _closest_peer_dist = pdist
            if pdist < ROBOT_RADIUS * 1.5 and pdist > 5:
                _flee_px += pdx / pdist
                _flee_py += pdy / pdist
        if _closest_peer_dist < ROBOT_RADIUS * 1.5 and (abs(_flee_px) > 0.01 or abs(_flee_py) > 0.01):
            flee_angle = math.atan2(_flee_py, _flee_px)
            diff = flee_angle - self.theta
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            if abs(diff) > math.pi * 0.3:
                pivot_spd = BASE_SPEED * 0.7
                if diff > 0:
                    return (pivot_spd, -pivot_spd)
                else:
                    return (-pivot_spd, pivot_spd)
            else:
                return (BASE_SPEED * 0.5, BASE_SPEED * 0.5)

        # --- Target selection (identical to sim) ---
        target = None

        self.my_predicted_target = self.get_my_frontier()

        if self.decentralized_mode:
            target = self.my_predicted_target
        elif self.state == RobotState.LEADER:
            target = self.my_assigned_target if self.my_assigned_target else self.my_predicted_target
        else:
            assignment_age = time.time() - self.leader_assignment_time
            assignment_valid = (
                self.leader_assigned_target is not None and
                assignment_age < self.ASSIGNMENT_STALE_TIME
            )

            if self.trust_in_leader > TRUST_THRESHOLD and assignment_valid:
                target = self.leader_assigned_target
            else:
                target = self.my_predicted_target
                if not assignment_valid and self.leader_assigned_target:
                    self.leader_assigned_target = None

        if not target:
            return (0.0, 0.0)

        # Arrival detection: mark the target cell as visited.
        # DO NOT clear the leader assignment here.  The leader will see
        # the cell in known_visited (via our broadcast) or notice we're
        # at the target (proximity check) and reassign on its next cycle.
        # Clearing it here causes the robot to immediately fall back to
        # local frontier, which defeats the whole point of having a leader.
        ARRIVAL_RADIUS = CELL_SIZE
        dist_to_target = math.sqrt((self.x - target[0])**2 + (self.y - target[1])**2)
        if dist_to_target < ARRIVAL_RADIUS:
            cell = self.get_cell(target[0], target[1])
            now = time.time()
            if cell not in self.my_visited:
                self.my_visited[cell] = now
            if cell not in self.known_visited:
                self.known_visited.add(cell)
                self.known_visited_time[cell] = now
            self._stuck_count = 0

        # Clamp target outside physical body margin (outer walls + divider).
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

        # Route around interior wall (equivalent to sim's A* pathfinding)
        target = self._route_around_wall(target)

        # If we've moved into open space, reset the stuck escalation counter
        # so the robot doesn't carry over long escape durations from a corner.
        if (self.x > OUTER_WALL_MARGIN + 50 and
            self.x < ARENA_WIDTH - OUTER_WALL_MARGIN - 50 and
            self.y > OUTER_WALL_MARGIN + 50 and
            self.y < ARENA_HEIGHT - OUTER_WALL_MARGIN - 50):
            if self._stuck_count > 0:
                self._stuck_count = max(0, self._stuck_count - 1)

        # Stuck detection → escape: pivot away, then drive to open space
        if self._check_stuck():
            self._stuck_count += 1
            escape_time = min(1.5 + self._stuck_count * 0.3, 3.0)
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
            # Outer walls
            flee_range = 300.0
            if self.x < flee_range:
                flee_x += (flee_range - self.x) / flee_range
            if ARENA_WIDTH - self.x < flee_range:
                flee_x -= (flee_range - (ARENA_WIDTH - self.x)) / flee_range
            if self.y < flee_range:
                flee_y += (flee_range - self.y) / flee_range
            if ARENA_HEIGHT - self.y < flee_range:
                flee_y -= (flee_range - (ARENA_HEIGHT - self.y)) / flee_range
            # Interior divider — flee away from whichever face is closer
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

        # --- Navigate to target ---
        dx = target[0] - self.x
        dy = target[1] - self.y
        target_angle = math.atan2(dy, dx)

        # Wall-sliding: modify heading to avoid driving into walls
        # (equivalent to sim's wall collision + component-wise motion)
        target_angle = self._wall_slide_heading(target_angle)
        target_angle = self._peer_avoidance_heading(target_angle)

        # EMA-filtered heading to suppress ArUco jitter (~5-10° per frame)
        EMA_ALPHA = 0.35
        if self._smooth_theta is None:
            self._smooth_theta = self.theta
        else:
            delta = self.theta - self._smooth_theta
            while delta > math.pi: delta -= 2 * math.pi
            while delta < -math.pi: delta += 2 * math.pi
            self._smooth_theta += EMA_ALPHA * delta
            while self._smooth_theta > math.pi: self._smooth_theta -= 2 * math.pi
            while self._smooth_theta < -math.pi: self._smooth_theta += 2 * math.pi

        diff = target_angle - self._smooth_theta
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi

        KP = 1.0 / (math.pi / 2)
        KD = 0.2
        HEADING_DEADBAND = 0.10  # ~5.7° — wider to ignore ArUco noise
        if abs(diff) < HEADING_DEADBAND:
            diff = 0.0

        now = time.time()
        d_term = 0.0
        if self._prev_heading_err is not None and self._prev_heading_t > 0:
            dt = now - self._prev_heading_t
            if 0.01 < dt < 0.5:
                d_err = diff - self._prev_heading_err
                while d_err > math.pi: d_err -= 2 * math.pi
                while d_err < -math.pi: d_err += 2 * math.pi
                d_term = KD * (d_err / dt) / (math.pi / 2)
        self._prev_heading_err = diff
        self._prev_heading_t = now

        turn = max(-1, min(1, KP * diff + d_term))

        # ToF-based reactive turn bias: sensors see an obstacle within 200mm,
        # bias turn away. Includes front sensor so the robot steers away from
        # head-on obstacles early (before the 70mm emergency threshold).
        TOF_BIAS_DIST = 300
        fl = self.tof.get('front_left', 9999)
        fr = self.tof.get('front_right', 9999)
        front_tof = self.tof.get('front', 9999)
        left_tof = self.tof.get('left', 9999)
        right_tof = self.tof.get('right', 9999)
        if fl <= 0: fl = 9999
        if fr <= 0: fr = 9999
        if front_tof <= 0: front_tof = 9999
        if left_tof <= 0: left_tof = 9999
        if right_tof <= 0: right_tof = 9999
        tof_bias = 0.0
        if fl < TOF_BIAS_DIST and fl > 20:
            tof_bias += 0.15 * (TOF_BIAS_DIST - fl) / TOF_BIAS_DIST
        if left_tof < TOF_BIAS_DIST and left_tof > 20:
            tof_bias += 0.08 * (TOF_BIAS_DIST - left_tof) / TOF_BIAS_DIST
        if fr < TOF_BIAS_DIST and fr > 20:
            tof_bias -= 0.15 * (TOF_BIAS_DIST - fr) / TOF_BIAS_DIST
        if right_tof < TOF_BIAS_DIST and right_tof > 20:
            tof_bias -= 0.08 * (TOF_BIAS_DIST - right_tof) / TOF_BIAS_DIST
        if front_tof < TOF_BIAS_DIST and front_tof > 20:
            steer_sign = 1.0 if (fl > fr or left_tof > right_tof) else -1.0
            tof_bias += steer_sign * 0.2 * (TOF_BIAS_DIST - front_tof) / TOF_BIAS_DIST
        turn = max(-1, min(1, turn + tof_bias))

        # --- Motor command ---
        # Speed: pick the MOST RESTRICTIVE single factor (min, not product).
        # Product of multiple <1.0 factors compounds to near-zero, stalling
        # the motors below their dead zone.
        alignment = max(0.0, math.cos(diff))
        sf_align = 0.7 + 0.3 * alignment

        SPEED_TAPER_ZONE = 250
        wall_dists = [self.x, ARENA_WIDTH - self.x,
                      self.y, ARENA_HEIGHT - self.y]
        if self.y < INTERIOR_WALL_Y_END:
            if self.x < INTERIOR_WALL_X_LEFT:
                wall_dists.append(INTERIOR_WALL_X_LEFT - self.x)
            elif self.x > INTERIOR_WALL_X_RIGHT:
                wall_dists.append(self.x - INTERIOR_WALL_X_RIGHT)
        nearest_wall = min(wall_dists)
        sf_wall = 1.0
        if nearest_wall < SPEED_TAPER_ZONE:
            sf_wall = 0.45 + 0.55 * (nearest_wall / SPEED_TAPER_ZONE)

        min_front_tof = min(front_d, fl_d, fr_d)
        sf_tof = 1.0
        if 20 < min_front_tof < 250:
            sf_tof = max(0.5, min_front_tof / 250)

        min_peer_dist = 9999.0
        for pid, peer in self.peers.items():
            if time.time() - peer.last_seen > PEER_TIMEOUT:
                continue
            d = math.sqrt((self.x - peer.x)**2 + (self.y - peer.y)**2)
            min_peer_dist = min(min_peer_dist, d)

        sf_peer = 1.0
        if min_peer_dist < ROBOT_RADIUS * 1.5:
            sf_peer = 0.10
        elif min_peer_dist < 400:
            sf_peer = 0.3 + 0.7 * (min_peer_dist / 400)

        MIN_MOTOR_PWM = 25  # Below this, N20 100:1 can't move 217g
        effective_speed = BASE_SPEED * min(sf_align, sf_wall, sf_tof, sf_peer)
        effective_speed = max(MIN_MOTOR_PWM + 5, effective_speed)

        turn_mix = 0.55

        # Pivot turn with hysteresis.  Only pivot for near-reversals;
        # the differential drive handles everything up to ~110° as an arc.
        PIVOT_ENTER = math.pi * 0.61   # ~110° — only near-U-turns
        PIVOT_EXIT  = math.pi * 0.19   # ~35°  — finish pivot, resume arcing
        if abs(diff) > PIVOT_ENTER:
            self._in_pivot = True
        if abs(diff) < PIVOT_EXIT:
            self._in_pivot = False
        if self._in_pivot:
            pivot_speed = BASE_SPEED * 0.55
            if diff > 0:
                left_speed, right_speed = -pivot_speed, pivot_speed
            else:
                left_speed, right_speed = pivot_speed, -pivot_speed
            self._last_turn = turn
            return (left_speed, right_speed)

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
        """
        Check for fault injection commands.
        
        Motor faults: spin, stop, erratic
          - Caught by assess_peer_behavior (anomaly detection)
        
        Leadership fault: bad_leader
          - Leader sends robots to explored cells
          - Caught by three-tier trust model (the main REIP feature)
        """
        try:
            while True:
                data, _ = self.fault_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                
                target = msg.get('robot_id')
                if target == self.robot_id or target == 'all' or target == 0:
                    fault = msg.get('fault', msg.get('cmd', 'none'))

                    if fault == 'start':
                        self.trial_started = True
                        self.my_visited.clear()
                        self.known_visited.clear()
                        self.known_visited_time.clear()
                        for p in self.peers.values():
                            p.visited_cells.clear()
                        print(f"[START] Trial started — coverage reset, motors engaged")
                        continue
                    elif fault in ('none', 'clear'):
                        self.injected_fault = None
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self._frozen_assignments.clear()
                        if hasattr(self, '_bad_assignments_cache'):
                            self._bad_assignments_cache.clear()
                        print(f"[FAULT] All faults cleared")
                    elif fault == 'bad_leader':
                        # Leadership fault - triggers three-tier trust
                        self.bad_leader_mode = True
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None  # Don't also do motor fault
                        print(f"[FAULT] BAD_LEADER mode activated - will send bad assignments")
                    elif fault == 'oscillate_leader':
                        # Oscillation fault - triggers Tier 3 direction consistency
                        self.oscillate_leader_mode = True
                        self.bad_leader_mode = False
                        self.freeze_leader_mode = False
                        self.injected_fault = None
                        self._oscillate_phase = 0
                        print(f"[FAULT] OSCILLATE_LEADER mode activated - will flip-flop targets")
                    elif fault == 'freeze_leader':
                        # Freeze fault - leader stops updating assignments
                        self.freeze_leader_mode = True
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.injected_fault = None
                        self._frozen_assignments.clear()  # Will be populated on first call
                        print(f"[FAULT] FREEZE_LEADER mode activated - will stop updating assignments")
                    else:
                        # Motor fault - triggers anomaly detection
                        self.injected_fault = fault
                        self.bad_leader_mode = False
                        self.oscillate_leader_mode = False
                        self.freeze_leader_mode = False
                        print(f"[FAULT] Motor fault injected: {fault}")
        except (BlockingIOError, ConnectionResetError):
            pass
        except ConnectionResetError:
            pass
        except OSError:
            pass
    
    # ==================== MAIN LOOPS ====================
    def sensor_loop(self):
        """read tof sensors continuously. encoder reads disabled - positions
        come from camera, and uart contention with motor commands trips the
        pico's 500ms watchdog.
        
        Target: ~8-10 Hz ToF updates (5 sensors × ~20ms each ≈ 100ms cycle).
        Old 500ms sleep made ToF data too stale for real-time avoidance."""
        while self.running:
            self.tof = self.hw.read_tof_all()
            self.update_tof_obstacles()
            time.sleep(0.02)  # ~50 Hz poll, but read_tof_all takes ~100ms
                              # so effective rate is ~8-10 Hz
    
    def network_loop(self):
        """Handle network communication"""
        last_broadcast = 0
        
        while self.running:
            self.receive_position()
            self.receive_peer_states()
            self.receive_fault_injection()
            
            if time.time() - last_broadcast > 1.0 / BROADCAST_RATE:
                self.broadcast_state()
                last_broadcast = time.time()
            
            # Prune dead peers
            now = time.time()
            dead = [pid for pid, p in self.peers.items() if now - p.last_seen > PEER_TIMEOUT]
            for pid in dead:
                del self.peers[pid]
            
            time.sleep(0.01)
    
    def control_loop(self):
        """Main control loop"""
        interval = 1.0 / CONTROL_RATE
        last_election = 0
        election_count = 0          # Fast-start: first N elections at higher rate
        last_print = 0
        last_log = 0
        printed_waiting = False
        
        while self.running:
            start = time.time()

            if not self.trial_started and not printed_waiting:
                print(f"[R{self.robot_id}] Waiting for 'start' command...")
                printed_waiting = True

            # Elections run even before trial starts so the leader is
            # already elected by the time motors engage.
            elect_interval = (ELECTION_FAST_INTERVAL
                              if election_count < ELECTION_FAST_COUNT
                              else ELECTION_INTERVAL)
            if not self.decentralized_mode and time.time() - last_election > elect_interval:
                self.check_impeachment()
                self.run_election()
                last_election = time.time()
                election_count += 1
            
            # Motors only engage when: trial started + fresh position data.
            left, right = 0.0, 0.0
            pos_age = time.time() - self.position_timestamp
            if self.trial_started and self.position_timestamp > 0 and pos_age < 1.5:
                left, right = self.compute_motor_command()
                self.hw.set_motors(left, right)
            else:
                self.hw.stop()
            
            # Logging (for visualization)
            if time.time() - last_log > 0.2:  # 5 Hz logging
                self.log_state()
                last_log = time.time()
            
            # Status print with motor debug
            if time.time() - last_print > 2.0:
                my_cov = len(self.my_visited) / (self.coverage_width * self.coverage_height) * 100
                total_cov = len(self.known_visited) / (self.coverage_width * self.coverage_height) * 100
                
                tgt = self.current_navigation_target
                tgt_str = f"({tgt[0]:.0f},{tgt[1]:.0f})" if tgt else "None"
                print(f"[R{self.robot_id}] pos=({self.x:.0f},{self.y:.0f}) "
                      f"state={self.state.value} leader={self.current_leader} "
                      f"trust={self.trust_in_leader:.2f} susp={self.suspicion_of_leader:.2f} "
                      f"cov={my_cov:.1f}%/{total_cov:.1f}% "
                      f"mot=({left:.0f},{right:.0f}) tgt={tgt_str}")
                last_print = time.time()
            
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def run(self):
        """Start the node"""
        self.running = True
        
        sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        network_thread = threading.Thread(target=self.network_loop, daemon=True)
        
        sensor_thread.start()
        network_thread.start()
        
        print("Running... Press Ctrl+C to stop\n")
        
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
        print("Usage: python3 reip_node.py <robot_id> [--sim] [--decentralized] [--port-base N] [--ablation MODE]")
        print("  --sim           Use unique ports for localhost testing")
        print("  --decentralized Disable leader coordination")
        print("  --port-base N   Offset all port numbers by N (for parallel experiments)")
        print("  --ablation MODE Ablation study mode:")
        print("       no_trust      - Disable all trust assessment (no detection/impeachment)")
        print("       no_causality  - Set causality grace period to 0 (expect false positives)")
        print("       no_direction  - Disable MPC direction consistency check")
        sys.exit(1)
    
    robot_id = int(sys.argv[1])
    sim_mode = "--sim" in sys.argv
    
    # Parse --port-base for parallel experiment support
    port_offset = 0
    if "--port-base" in sys.argv:
        idx = sys.argv.index("--port-base")
        port_offset = int(sys.argv[idx + 1])
        UDP_POSITION_PORT += port_offset
        UDP_PEER_PORT += port_offset
        UDP_FAULT_PORT += port_offset
    
    # Parse --ablation for ablation study
    ablation_mode = None
    if "--ablation" in sys.argv:
        idx = sys.argv.index("--ablation")
        ablation_mode = sys.argv[idx + 1]
        valid_modes = ("no_trust", "no_causality", "no_direction")
        if ablation_mode not in valid_modes:
            print(f"ERROR: Unknown ablation mode '{ablation_mode}'. Valid: {valid_modes}")
            sys.exit(1)
    
    # Apply ablation overrides BEFORE constructing the node
    if ablation_mode == "no_causality":
        CAUSALITY_GRACE_PERIOD = 0.0
        print(f"=== ABLATION: no_causality -- CAUSALITY_GRACE_PERIOD = 0 ===")
    
    # Parse --arena-width / --arena-height for scaled layouts
    if "--arena-width" in sys.argv:
        idx = sys.argv.index("--arena-width")
        ARENA_WIDTH = int(sys.argv[idx + 1])
        print(f"=== Arena width override: {ARENA_WIDTH} ===")
    if "--arena-height" in sys.argv:
        idx = sys.argv.index("--arena-height")
        ARENA_HEIGHT = int(sys.argv[idx + 1])
        print(f"=== Arena height override: {ARENA_HEIGHT} ===")
    
    node = REIPNode(robot_id, sim_mode=sim_mode)
    
    # Apply ablation flags to the node instance
    if ablation_mode == "no_trust":
        node._ablation_no_trust = True
        print(f"=== ABLATION: no_trust -- assess_leader_command() DISABLED ===")
    else:
        node._ablation_no_trust = False
    
    if ablation_mode == "no_direction":
        node._ablation_no_direction = True
        print(f"=== ABLATION: no_direction -- MPC direction check DISABLED ===")
    else:
        node._ablation_no_direction = False
    
    if "--decentralized" in sys.argv:
        node.decentralized_mode = True
        node.state = RobotState.EXPLORING
        print(f"=== DECENTRALIZED MODE - No leader coordination ===")
    
    # Print detection bounds at startup
    print(f"=== Detection Bounds: T1={WORST_CASE_DETECT_T1} cmds, "
          f"T3={WORST_CASE_DETECT_T3} cmds | "
          f"Impeachment: T1={WORST_CASE_IMPEACH_T1} cmds, "
          f"T3={WORST_CASE_IMPEACH_T3} cmds ===")
    if ablation_mode:
        print(f"=== ABLATION MODE: {ablation_mode} ===")
    
    node.run()
