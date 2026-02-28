#!/usr/bin/env python3
"""
REIP Distributed Node
Runs on each Pi Zero - this is the robot's brain.

Each robot:
- Receives its own position from camera server
- Reads local sensors (ToF, encoders)
- Broadcasts state to peers
- Receives peer states
- Computes trust scores locally
- Participates in elections
- Makes its own movement decisions

Usage: python3 reip_node.py <robot_id>
"""

import socket
import json
import time
import math
import threading
import serial
import smbus
import busio
import board
import adafruit_vl53l0x
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Set
from enum import Enum
from collections import deque

# ============== Configuration ==============
# Network
UDP_POSITION_PORT = 5003   # Receive position from camera
UDP_PEER_PORT = 5004       # Peer-to-peer communication
BROADCAST_IP = "255.255.255.255"

# Arena
ARENA_WIDTH = 2000   # mm
ARENA_HEIGHT = 1000  # mm
CELL_SIZE = 150      # mm - robot-sized grid cells

# Robot physical
ROBOT_RADIUS = 75    # mm
AVOID_DISTANCE = 200 # mm
CRITICAL_DISTANCE = 100  # mm

# REIP parameters
TRUST_DECAY_RATE = 0.05      # Per anomaly detection
TRUST_BOOST_RATE = 0.02      # Per normal behavior
TRUST_THRESHOLD = 0.5        # Below = untrusted
IMPEACHMENT_THRESHOLD = 0.3  # Below = impeach vote
ELECTION_INTERVAL = 2.0      # seconds
PEER_TIMEOUT = 3.0           # seconds before peer considered dead

# Control
CONTROL_RATE = 10    # Hz
BASE_SPEED = 50      # PWM %
BROADCAST_RATE = 5   # Hz - state broadcast to peers

# ToF channels (your wiring)
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
    state: RobotState = RobotState.IDLE
    trust_score: float = 1.0  # Their reported trust
    my_trust_for_them: float = 1.0  # MY trust assessment
    vote: Optional[int] = None
    coverage_count: int = 0
    last_seen: float = 0
    position_history: deque = field(default_factory=lambda: deque(maxlen=20))

@dataclass
class CoverageCell:
    x: int  # cell index
    y: int
    visited_by: Set[int] = field(default_factory=set)

# ============== Hardware Interface ==============
class Hardware:
    def __init__(self):
        # UART to Pico
        self.uart = serial.Serial('/dev/serial0', 115200, timeout=0.1)
        time.sleep(0.5)
        self.uart.reset_input_buffer()
        
        # I2C for ToF
        self.bus = smbus.SMBus(1)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.MUX_ADDR = 0x70
        
        self._ping_pico()
    
    def _ping_pico(self):
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
    
    def _select_mux(self, ch):
        try:
            self.bus.write_byte(self.MUX_ADDR, 1 << ch)
            time.sleep(0.005)
        except:
            pass
    
    def read_tof_all(self) -> Dict[str, int]:
        distances = {}
        for ch, name in TOF_CHANNELS.items():
            try:
                self._select_mux(ch)
                time.sleep(0.01)
                tof = adafruit_vl53l0x.VL53L0X(self.i2c)
                distances[name] = tof.range
            except:
                distances[name] = -1
        return distances
    
    def read_encoders(self) -> tuple:
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
        try:
            self.uart.write(f"MOT,{left:.1f},{right:.1f}\n".encode())
            self.uart.readline()
        except:
            pass
    
    def stop(self):
        try:
            self.uart.write(b'STOP\n')
            self.uart.readline()
        except:
            pass

# ============== REIP Node ==============
class REIPNode:
    def __init__(self, robot_id: int):
        self.robot_id = robot_id
        
        print(f"=== REIP Node - Robot {robot_id} ===\n")
        
        # My state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.state = RobotState.IDLE
        self.my_trust = 1.0
        self.position_timestamp = 0.0
        
        # Sensor data
        self.tof = {}
        self.encoders = (0, 0)
        
        # Peer tracking
        self.peers: Dict[int, PeerInfo] = {}
        
        # Election state
        self.current_leader: Optional[int] = None
        self.my_vote: Optional[int] = None
        self.last_election = 0
        
        # Coverage map (local)
        self.coverage_width = int(ARENA_WIDTH / CELL_SIZE)
        self.coverage_height = int(ARENA_HEIGHT / CELL_SIZE)
        self.my_visited: Set[tuple] = set()
        self.known_visited: Set[tuple] = set()  # Union of all peers' coverage
        
        # Hardware
        print("Initializing hardware...")
        self.hw = Hardware()
        
        # Network - position from camera
        self.pos_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pos_socket.bind(('', UDP_POSITION_PORT))
        self.pos_socket.setblocking(False)
        
        # Network - peer communication
        self.peer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.peer_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.peer_socket.bind(('', UDP_PEER_PORT))
        self.peer_socket.setblocking(False)
        
        # Network - fault injection (for testing)
        UDP_FAULT_PORT = 5005
        self.fault_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fault_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.fault_socket.bind(('', UDP_FAULT_PORT))
        self.fault_socket.setblocking(False)
        
        # Fault state
        self.injected_fault = None  # None, 'spin', 'stop', 'erratic'
        
        # Threading
        self.running = False
        
        print("Ready!\n")
    
    # ==================== POSITION ====================
    def receive_position(self):
        """Receive our position from camera server"""
        try:
            while True:
                data, _ = self.pos_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                if msg.get('type') == 'position' and msg.get('robot_id') == self.robot_id:
                    self.x = msg['x']
                    self.y = msg['y']
                    self.theta = msg['theta']
                    self.position_timestamp = msg['timestamp']
                    
                    # Update coverage
                    cell = self.get_cell(self.x, self.y)
                    if cell:
                        self.my_visited.add(cell)
                        self.known_visited.add(cell)
        except BlockingIOError:
            pass
    
    def get_cell(self, x: float, y: float) -> Optional[tuple]:
        """Convert position to cell index"""
        cx = int(x / CELL_SIZE)
        cy = int(y / CELL_SIZE)
        if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
            return (cx, cy)
        return None
    
    # ==================== PEER COMMUNICATION ====================
    def broadcast_state(self):
        """Broadcast our state to all peers"""
        msg = {
            'type': 'peer_state',
            'robot_id': self.robot_id,
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'state': self.state.value,
            'trust': self.my_trust,
            'vote': self.my_vote,
            'coverage_count': len(self.my_visited),
            'visited_cells': list(self.my_visited),  # Full coverage - no radius limit for validation
            'timestamp': time.time()
        }
        data = json.dumps(msg).encode()
        try:
            self.peer_socket.sendto(data, (BROADCAST_IP, UDP_PEER_PORT))
        except:
            pass
    
    def receive_peer_states(self):
        """Receive state broadcasts from peers"""
        try:
            while True:
                data, addr = self.peer_socket.recvfrom(4096)
                msg = json.loads(data.decode())
                
                if msg.get('type') == 'peer_state':
                    peer_id = msg.get('robot_id')
                    if peer_id and peer_id != self.robot_id:
                        self.update_peer(peer_id, msg)
        except BlockingIOError:
            pass
    
    def update_peer(self, peer_id: int, msg: dict):
        """Update peer info and assess trust"""
        if peer_id not in self.peers:
            self.peers[peer_id] = PeerInfo(robot_id=peer_id)
        
        peer = self.peers[peer_id]
        
        # Store position history for anomaly detection
        peer.position_history.append({
            'x': msg['x'], 'y': msg['y'], 
            'theta': msg['theta'], 'time': msg['timestamp']
        })
        
        # Update info
        peer.x = msg['x']
        peer.y = msg['y']
        peer.theta = msg['theta']
        peer.state = RobotState(msg.get('state', 'idle'))
        peer.trust_score = msg.get('trust', 1.0)
        peer.vote = msg.get('vote')
        peer.coverage_count = msg.get('coverage_count', 0)
        peer.last_seen = time.time()
        
        # Merge their coverage into known
        for cell in msg.get('visited_cells', []):
            self.known_visited.add(tuple(cell))
        
        # Assess trust based on behavior
        self.assess_peer_trust(peer_id)
    
    def assess_peer_trust(self, peer_id: int):
        """Compute MY trust assessment for a peer based on observed behavior"""
        peer = self.peers[peer_id]
        
        if len(peer.position_history) < 5:
            return
        
        history = list(peer.position_history)[-10:]
        
        # Check for anomalies
        anomaly = False
        
        # Spinning? (lots of rotation, little translation)
        total_dist = 0
        total_rot = 0
        for i in range(1, len(history)):
            dx = history[i]['x'] - history[i-1]['x']
            dy = history[i]['y'] - history[i-1]['y']
            total_dist += math.sqrt(dx*dx + dy*dy)
            total_rot += abs(history[i]['theta'] - history[i-1]['theta'])
        
        if total_dist < 20 and total_rot > math.pi:
            anomaly = True  # Spinning in place
        
        # Stuck? (no movement at all)
        if total_dist < 10 and len(history) >= 10:
            anomaly = True
        
        # Update trust
        if anomaly:
            peer.my_trust_for_them = max(0, peer.my_trust_for_them - TRUST_DECAY_RATE)
        else:
            peer.my_trust_for_them = min(1, peer.my_trust_for_them + TRUST_BOOST_RATE)
    
    def prune_peers(self):
        """Remove peers not seen recently"""
        now = time.time()
        dead = [pid for pid, p in self.peers.items() if now - p.last_seen > PEER_TIMEOUT]
        for pid in dead:
            del self.peers[pid]
    
    def receive_fault_injection(self):
        """Check for fault injection commands (testing only)"""
        try:
            while True:
                data, _ = self.fault_socket.recvfrom(1024)
                msg = json.loads(data.decode())
                
                if msg.get('type') == 'fault_inject' and msg.get('robot_id') == self.robot_id:
                    fault = msg.get('fault', 'none')
                    if fault == 'none':
                        self.injected_fault = None
                        print(f"[FAULT] Cleared")
                    else:
                        self.injected_fault = fault
                        print(f"[FAULT] Injected: {fault}")
        except BlockingIOError:
            pass
    
    # ==================== REIP ELECTION ====================
    def run_election(self):
        """Run leader election based on trust"""
        candidates = [(self.robot_id, self.my_trust)]
        
        for pid, peer in self.peers.items():
            if peer.my_trust_for_them > TRUST_THRESHOLD:
                candidates.append((pid, peer.my_trust_for_them))
        
        # Highest trust wins, lowest ID breaks tie
        candidates.sort(key=lambda x: (-x[1], x[0]))
        
        if candidates:
            self.my_vote = candidates[0][0]
            
            # Count votes (including from peers)
            votes = {self.my_vote: 1}
            for pid, peer in self.peers.items():
                if peer.vote is not None:
                    votes[peer.vote] = votes.get(peer.vote, 0) + 1
            
            # Winner by majority
            self.current_leader = max(votes.items(), key=lambda x: x[1])[0]
            
            # Update state
            if self.current_leader == self.robot_id:
                self.state = RobotState.LEADER
            else:
                self.state = RobotState.FOLLOWER
    
    def check_impeachment(self):
        """Check if I should vote to impeach the leader"""
        if self.current_leader is None or self.current_leader == self.robot_id:
            return
        
        if self.current_leader in self.peers:
            trust = self.peers[self.current_leader].my_trust_for_them
            if trust < IMPEACHMENT_THRESHOLD:
                # I vote to impeach - my vote changes
                self.my_vote = None  # Abstain from voting for current leader
                print(f"Voting to impeach leader {self.current_leader} (trust={trust:.2f})")
    
    # ==================== NAVIGATION ====================
    def get_frontier(self) -> Optional[tuple]:
        """Find nearest unvisited cell"""
        my_cell = self.get_cell(self.x, self.y)
        if not my_cell:
            return None
        
        # Spiral search
        for radius in range(1, max(self.coverage_width, self.coverage_height)):
            best_dist = float('inf')
            best_cell = None
            
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    
                    cx = my_cell[0] + dx
                    cy = my_cell[1] + dy
                    
                    if 0 <= cx < self.coverage_width and 0 <= cy < self.coverage_height:
                        cell = (cx, cy)
                        if cell not in self.known_visited:
                            dist = abs(dx) + abs(dy)
                            if dist < best_dist:
                                best_dist = dist
                                best_cell = cell
            
            if best_cell:
                # Return center of cell in mm
                return ((best_cell[0] + 0.5) * CELL_SIZE,
                       (best_cell[1] + 0.5) * CELL_SIZE)
        
        return None
    
    def compute_motor_command(self) -> tuple:
        """Compute motor speeds based on state and sensors"""
        
        # Handle injected faults (for testing)
        import random
        if self.injected_fault == 'spin':
            return (BASE_SPEED, -BASE_SPEED)  # Spin in place
        elif self.injected_fault == 'stop':
            return (0, 0)  # Freeze
        elif self.injected_fault == 'erratic':
            return (random.uniform(-BASE_SPEED, BASE_SPEED),
                   random.uniform(-BASE_SPEED, BASE_SPEED))
        
        front = self.tof.get('front', 9999)
        front_left = self.tof.get('front_left', 9999)
        front_right = self.tof.get('front_right', 9999)
        left = self.tof.get('left', 9999)
        right = self.tof.get('right', 9999)
        
        # Handle -1 (error) values
        if front < 0: front = 9999
        if front_left < 0: front_left = 9999
        if front_right < 0: front_right = 9999
        
        # Critical stop
        if front < CRITICAL_DISTANCE:
            # Back up and turn
            return (-BASE_SPEED * 0.5, -BASE_SPEED * 0.3)
        
        # Obstacle avoidance takes priority
        if front < AVOID_DISTANCE or front_left < AVOID_DISTANCE or front_right < AVOID_DISTANCE:
            if front_left < front_right:
                return (BASE_SPEED * 0.6, BASE_SPEED * 0.2)  # Turn right
            else:
                return (BASE_SPEED * 0.2, BASE_SPEED * 0.6)  # Turn left
        
        # Exploration - head to frontier
        frontier = self.get_frontier()
        if frontier:
            fx, fy = frontier
            dx = fx - self.x
            dy = fy - self.y
            target_angle = math.atan2(dy, dx)
            
            # Angle difference
            diff = target_angle - self.theta
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            
            # Proportional steering
            turn = max(-1, min(1, diff / (math.pi / 3)))
            left_speed = BASE_SPEED * (1 - turn * 0.5)
            right_speed = BASE_SPEED * (1 + turn * 0.5)
            
            return (left_speed, right_speed)
        
        # No frontier - all explored or no position yet
        return (0, 0)
    
    # ==================== MAIN LOOPS ====================
    def sensor_loop(self):
        """Read sensors continuously"""
        while self.running:
            self.tof = self.hw.read_tof_all()
            self.encoders = self.hw.read_encoders()
            time.sleep(0.05)
    
    def network_loop(self):
        """Handle network communication"""
        last_broadcast = 0
        
        while self.running:
            # Receive position from camera
            self.receive_position()
            
            # Receive peer states
            self.receive_peer_states()
            
            # Check for fault injection (testing)
            self.receive_fault_injection()
            
            # Broadcast our state
            if time.time() - last_broadcast > 1.0 / BROADCAST_RATE:
                self.broadcast_state()
                last_broadcast = time.time()
            
            # Prune dead peers
            self.prune_peers()
            
            time.sleep(0.01)
    
    def control_loop(self):
        """Main control loop"""
        interval = 1.0 / CONTROL_RATE
        last_election = 0
        last_print = 0
        
        while self.running:
            start = time.time()
            
            # Run election periodically
            if time.time() - last_election > ELECTION_INTERVAL:
                self.run_election()
                self.check_impeachment()
                last_election = time.time()
            
            # Compute and execute motor command
            if self.position_timestamp > 0:  # Only move if we have position
                left, right = self.compute_motor_command()
                self.hw.set_motors(left, right)
            else:
                self.hw.stop()
            
            # Status print
            if time.time() - last_print > 2.0:
                coverage = len(self.my_visited) / (self.coverage_width * self.coverage_height) * 100
                total_known = len(self.known_visited) / (self.coverage_width * self.coverage_height) * 100
                print(f"[R{self.robot_id}] pos=({self.x:.0f},{self.y:.0f}) "
                      f"state={self.state.value} leader={self.current_leader} "
                      f"peers={len(self.peers)} my_cov={coverage:.1f}% total={total_known:.1f}%")
                last_print = time.time()
            
            # Rate limit
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def run(self):
        """Start the node"""
        self.running = True
        
        # Start threads
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
            print("Stopped.")

# ============== Entry ==============
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 reip_node.py <robot_id>")
        sys.exit(1)
    
    robot_id = int(sys.argv[1])
    node = REIPNode(robot_id)
    node.run()
