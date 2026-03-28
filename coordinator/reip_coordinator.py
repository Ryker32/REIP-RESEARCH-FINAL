#!/usr/bin/env python3
"""
REIP Coordinator - Central controller for robot fleet
Receives positions from tracker, runs REIP algorithm, sends motor commands.

Usage: python reip_coordinator.py <robot_ids> [log_file]
Example: python reip_coordinator.py 1,2,3,4,5 experiment.jsonl

Interactive commands (type while running):
    f <id> spin     - Inject spin fault on robot
    f <id> stop     - Inject stop fault (robot freezes)
    f <id> reverse  - Inject reverse fault
    c <id>          - Clear fault on robot
    q               - Quit
"""

import sys
import os
import time
import json
import math
import socket
import threading
import select
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ============================================================================
# CONFIGURATION
# ============================================================================

POSITION_PORT = 5003    # Receive positions from tracker
TELEMETRY_PORT = 5002   # Receive sensor data from robots
CMD_PORT = 5001         # Send motor commands to robots

CONTROL_RATE = 10       # Hz

# Arena bounds (mm) - adjust to your pool table
ARENA_WIDTH = 2000
ARENA_HEIGHT = 1000

# REIP parameters
TRUST_THRESHOLD = 0.6
TRUST_DECAY_RATE = 0.1
TRUST_RECOVERY_RATE = 0.02
MIN_TRUST = 0.1

# Navigation
MAX_SPEED = 50
OBSTACLE_THRESHOLD = 200  # mm
GOAL_REACHED_DIST = 100   # mm


# ============================================================================
# REIP GOVERNANCE
# ============================================================================

class REIPGovernance:
    """Trust-based leader election and impeachment."""
    
    def __init__(self, robot_ids: List[int]):
        self.robot_ids = robot_ids
        self.n = len(robot_ids)
        
        # Trust matrix: trust[i][j] = robot i's trust in robot j
        self.trust = {i: {j: 1.0 for j in robot_ids} for i in robot_ids}
        
        # Leadership
        self.leader_id = robot_ids[0]
        self.election_count = 0
        self.impeachment_count = 0
        
        # Tracking
        self.predictions = {}  # robot_id -> predicted coverage gain
        self.observations = {}  # robot_id -> observed coverage gain
        
    def update_trust(self, robot_id: int, predicted: float, observed: float):
        """Update trust based on predicted vs observed performance."""
        error = max(0.0, predicted - observed)
        decay = math.exp(-TRUST_DECAY_RATE * error)
        
        # All robots lower trust in this robot
        for i in self.robot_ids:
            if i != robot_id:
                self.trust[i][robot_id] = max(MIN_TRUST, 
                    self.trust[i][robot_id] * decay)
                
                # Recover trust in others
                for j in self.robot_ids:
                    if j != robot_id and self.trust[i][j] < 1.0:
                        self.trust[i][j] = min(1.0, 
                            self.trust[i][j] + TRUST_RECOVERY_RATE)
    
    def get_avg_trust_in_leader(self) -> float:
        """Get average trust in current leader."""
        leader = self.leader_id
        voters = [i for i in self.robot_ids if i != leader]
        if not voters:
            return 1.0
        return sum(self.trust[i][leader] for i in voters) / len(voters)
    
    def should_impeach(self) -> bool:
        """Check if impeachment threshold reached."""
        return self.get_avg_trust_in_leader() < TRUST_THRESHOLD
    
    def elect_new_leader(self) -> int:
        """Elect leader with highest trust."""
        scores = {}
        for candidate in self.robot_ids:
            voters = [i for i in self.robot_ids if i != candidate]
            scores[candidate] = sum(self.trust[i][candidate] for i in voters)
        
        new_leader = max(scores.keys(), key=lambda k: scores[k])
        
        if new_leader != self.leader_id:
            print(f"\n[REIP] ELECTION: {self.leader_id} -> {new_leader}")
            self.leader_id = new_leader
            self.election_count += 1
        
        return self.leader_id
    
    def check_impeachment(self) -> bool:
        """Check and execute impeachment if needed."""
        if self.should_impeach():
            print(f"\n[REIP] IMPEACHMENT! Trust in leader {self.leader_id}: "
                  f"{self.get_avg_trust_in_leader():.2f}")
            self.impeachment_count += 1
            self.elect_new_leader()
            return True
        return False


# ============================================================================
# COORDINATOR
# ============================================================================

class REIPCoordinator:
    def __init__(self, robot_ids: List[int], log_file: str = None):
        self.robot_ids = robot_ids
        self.running = True
        
        # State
        self.positions = {}   # id -> (x, y, theta, timestamp)
        self.telemetry = {}   # id -> {tof, enc, timestamp}
        self.goals = {}       # id -> (x, y)
        self.faults = {}      # id -> fault_type or None
        
        # REIP
        self.reip = REIPGovernance(robot_ids)
        
        # Coverage tracking
        self.visited_cells = set()
        self.cell_size = 100  # mm
        self.coverage_history = deque(maxlen=100)
        
        # Logging
        self.log_file = None
        if log_file:
            self.log_file = open(log_file, 'w')
            print(f"[Coordinator] Logging to {log_file}")
        
        # UDP sockets
        self._setup_sockets()
        
        # Input thread for commands
        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()
        
        print(f"[Coordinator] Managing robots: {robot_ids}")
        print("[Coordinator] Commands: f <id> spin|stop|reverse, c <id>, q")
    
    def _setup_sockets(self):
        """Initialize UDP sockets."""
        # Position receiver (from tracker)
        self.pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.pos_sock.bind(('', POSITION_PORT))
        self.pos_sock.setblocking(False)
        
        # Telemetry receiver (from robots)
        self.tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tel_sock.bind(('', TELEMETRY_PORT))
        self.tel_sock.setblocking(False)
        
        # Command sender
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    def _input_loop(self):
        """Handle keyboard input for fault injection."""
        while self.running:
            try:
                line = input()
                self._process_command(line.strip())
            except EOFError:
                break
            except Exception as e:
                print(f"[Input Error] {e}")
    
    def _process_command(self, cmd: str):
        """Process interactive command."""
        parts = cmd.split()
        if not parts:
            return
        
        if parts[0] == 'q':
            print("[Coordinator] Quitting...")
            self.running = False
        
        elif parts[0] == 'f' and len(parts) >= 3:
            # Fault injection: f <id> <type>
            try:
                robot_id = int(parts[1])
                fault_type = parts[2]
                if robot_id in self.robot_ids:
                    self.faults[robot_id] = fault_type
                    print(f"[FAULT] Injected '{fault_type}' on robot {robot_id}")
            except ValueError:
                print("[Error] Usage: f <id> spin|stop|reverse")
        
        elif parts[0] == 'c' and len(parts) >= 2:
            # Clear fault: c <id>
            try:
                robot_id = int(parts[1])
                if robot_id in self.faults:
                    del self.faults[robot_id]
                    print(f"[FAULT] Cleared fault on robot {robot_id}")
            except ValueError:
                print("[Error] Usage: c <id>")
    
    def receive_positions(self):
        """Receive position updates from tracker."""
        try:
            data, addr = self.pos_sock.recvfrom(4096)
            msg = json.loads(data.decode())
            
            for robot_id_str, pos in msg.get('robots', {}).items():
                robot_id = int(robot_id_str)
                if robot_id in self.robot_ids:
                    self.positions[robot_id] = (
                        pos['x'], pos['y'], pos['theta'], time.time()
                    )
                    # Update coverage
                    cx = int(pos['x'] / self.cell_size)
                    cy = int(pos['y'] / self.cell_size)
                    self.visited_cells.add((cx, cy))
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[Pos Error] {e}")
    
    def receive_telemetry(self):
        """Receive sensor data from robots."""
        try:
            data, addr = self.tel_sock.recvfrom(4096)
            msg = json.loads(data.decode())
            
            robot_id = msg.get('id')
            if robot_id in self.robot_ids:
                self.telemetry[robot_id] = {
                    'tof': msg.get('tof', {}),
                    'enc': msg.get('enc', [0, 0]),
                    'ts': time.time(),
                }
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[Tel Error] {e}")
    
    def compute_goals(self):
        """Assign exploration goals to robots (simple frontier-based)."""
        # For now: random goals within arena if no goal or goal reached
        import random
        
        for robot_id in self.robot_ids:
            pos = self.positions.get(robot_id)
            goal = self.goals.get(robot_id)
            
            # Check if need new goal
            need_new = False
            if goal is None:
                need_new = True
            elif pos is not None:
                dist = math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
                if dist < GOAL_REACHED_DIST:
                    need_new = True
            
            if need_new:
                # Simple: random goal within arena bounds
                gx = random.uniform(100, ARENA_WIDTH - 100)
                gy = random.uniform(100, ARENA_HEIGHT - 100)
                self.goals[robot_id] = (gx, gy)
    
    def compute_motor_commands(self, robot_id: int) -> Tuple[float, float]:
        """Compute motor commands for a robot."""
        # Check for injected fault
        fault = self.faults.get(robot_id)
        if fault == 'spin':
            return (40, -40)
        elif fault == 'stop':
            return (0, 0)
        elif fault == 'reverse':
            return (-40, -40)
        
        # Get position and goal
        pos = self.positions.get(robot_id)
        goal = self.goals.get(robot_id)
        tof = self.telemetry.get(robot_id, {}).get('tof', {})
        
        if pos is None or goal is None:
            return (0, 0)
        
        x, y, theta, _ = pos
        gx, gy = goal
        
        # Check obstacles from ToF
        front = tof.get('front', 2000)
        if front < OBSTACLE_THRESHOLD:
            # Turn away from obstacle
            left = tof.get('left', 2000)
            right = tof.get('right', 2000)
            if left > right:
                return (-30, 30)  # Turn left
            else:
                return (30, -30)  # Turn right
        
        # Navigate toward goal
        dx = gx - x
        dy = gy - y
        target_angle = math.atan2(dy, dx)
        
        # Angle difference
        angle_diff = target_angle - theta
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Proportional control
        if abs(angle_diff) > 0.3:  # ~17 degrees
            # Turn toward goal
            turn = MAX_SPEED * 0.6 * (1 if angle_diff > 0 else -1)
            return (MAX_SPEED * 0.3 - turn, MAX_SPEED * 0.3 + turn)
        else:
            # Drive toward goal
            return (MAX_SPEED, MAX_SPEED)
    
    def send_commands(self):
        """Send motor commands to all robots."""
        for robot_id in self.robot_ids:
            left, right = self.compute_motor_commands(robot_id)
            
            msg = {
                'id': robot_id,
                'left': left,
                'right': right,
                'ts': time.time(),
            }
            
            data = json.dumps(msg).encode()
            self.cmd_sock.sendto(data, ('<broadcast>', CMD_PORT))
    
    def update_trust(self):
        """Update REIP trust scores based on robot behavior."""
        # Compare predicted vs actual coverage gains
        # For simplicity: check if robots are making progress
        for robot_id in self.robot_ids:
            pos = self.positions.get(robot_id)
            tel = self.telemetry.get(robot_id)
            
            if pos is None or tel is None:
                continue
            
            # Predict: robot should be moving toward goal
            goal = self.goals.get(robot_id)
            if goal is None:
                continue
            
            # Simple heuristic: if robot has fault, it won't reach goal properly
            fault = self.faults.get(robot_id)
            if fault:
                # Faulty behavior detected
                self.reip.update_trust(robot_id, predicted=0.5, observed=0.0)
            else:
                # Normal behavior
                self.reip.update_trust(robot_id, predicted=0.5, observed=0.45)
        
        # Check for impeachment
        self.reip.check_impeachment()
    
    def log_state(self):
        """Log current state to file."""
        if self.log_file is None:
            return
        
        # Calculate coverage
        total_cells = (ARENA_WIDTH // self.cell_size) * (ARENA_HEIGHT // self.cell_size)
        coverage = len(self.visited_cells) / total_cells if total_cells > 0 else 0
        
        entry = {
            'ts': time.time(),
            'coverage': coverage,
            'leader': self.reip.leader_id,
            'elections': self.reip.election_count,
            'impeachments': self.reip.impeachment_count,
            'avg_trust': self.reip.get_avg_trust_in_leader(),
            'positions': {k: v[:3] for k, v in self.positions.items()},
            'faults': dict(self.faults),
            'trust_matrix': {str(i): {str(j): self.reip.trust[i][j] 
                            for j in self.robot_ids} for i in self.robot_ids},
        }
        
        self.log_file.write(json.dumps(entry) + '\n')
        self.log_file.flush()
    
    def run(self):
        """Main control loop."""
        print("[Coordinator] Running... Type 'q' to quit")
        
        interval = 1.0 / CONTROL_RATE
        last_log = 0
        
        try:
            while self.running:
                start = time.time()
                
                # Receive updates
                self.receive_positions()
                self.receive_telemetry()
                
                # Compute and send commands
                self.compute_goals()
                self.send_commands()
                
                # REIP governance
                self.update_trust()
                
                # Log state
                now = time.time()
                if now - last_log >= 0.5:  # Log every 500ms
                    self.log_state()
                    last_log = now
                
                # Status display
                coverage = len(self.visited_cells) / max(1, 
                    (ARENA_WIDTH // self.cell_size) * (ARENA_HEIGHT // self.cell_size))
                trust = self.reip.get_avg_trust_in_leader()
                print(f"\r[Coord] Robots:{len(self.positions)} "
                      f"Cov:{coverage*100:.1f}% "
                      f"Leader:{self.reip.leader_id} "
                      f"Trust:{trust:.2f} "
                      f"Faults:{list(self.faults.keys())}", end="")
                
                # Sleep for remaining time
                elapsed = time.time() - start
                if elapsed < interval:
                    time.sleep(interval - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[Coordinator] Interrupted")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        
        # Stop all robots
        for robot_id in self.robot_ids:
            msg = {'id': robot_id, 'left': 0, 'right': 0}
            data = json.dumps(msg).encode()
            self.cmd_sock.sendto(data, ('<broadcast>', CMD_PORT))
        
        # Close sockets
        self.pos_sock.close()
        self.tel_sock.close()
        self.cmd_sock.close()
        
        # Close log
        if self.log_file:
            self.log_file.close()
        
        print("[Coordinator] Shutdown complete")


# ============================================================================
# MAIN
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python reip_coordinator.py <robot_ids> [log_file]")
        print("Example: python reip_coordinator.py 1,2,3,4,5 experiment.jsonl")
        sys.exit(1)
    
    # Parse robot IDs
    robot_ids = [int(x) for x in sys.argv[1].split(',')]
    
    # Log file (optional)
    log_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    coordinator = REIPCoordinator(robot_ids, log_file)
    coordinator.run()


if __name__ == "__main__":
    main()
