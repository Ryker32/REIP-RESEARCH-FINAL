#!/usr/bin/env python3
"""
REIP Simulation Harness

Tests the actual reip_node.py code in simulation:
- Runs multiple robot instances in threads
- Provides mock position data (simulated movement)
- Visualizes state in terminal
- Allows fault injection

Usage: python test/sim_harness.py [num_robots]
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import json
import time
import math
import threading
import random
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ============== Configuration ==============
NUM_ROBOTS = 5
UDP_POSITION_PORT = 5100
UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
BROADCAST_IP = "127.0.0.1"

# Arena (must match reip_node.py)
ARENA_WIDTH = 2000
ARENA_HEIGHT = 1500
CELL_SIZE = 125
ARENA_LAYOUTS = {
    "open": [],
    "multiroom": [
        (1000, 0, 1000, 1200),
    ],
}
START_PRESETS = {
    "hardware_clone_20260308": {
        1: (185.9, 382.1, 0.188),
        2: (792.9, 709.2, 2.255),
        3: (318.8, 147.4, 0.736),
        4: (768.7, 219.2, 1.577),
        5: (182.4, 709.9, 0.410),
    },
}

# Simulation
SIM_RATE = 10  # Hz
ROBOT_SPEED = 50  # mm per tick

# ============== Simulated Robot State ==============
@dataclass
class SimRobot:
    robot_id: int
    x: float
    y: float
    theta: float
    target_x: float = 0
    target_y: float = 0
    fault: Optional[str] = None

# ============== Mock Position Server ==============
class MockPositionServer:
    """Sends simulated position data to robots"""
    
    def __init__(self, robots: Dict[int, SimRobot]):
        self.robots = robots
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.running = False
    
    def send_positions(self):
        """Send position update for all robots"""
        for rid, robot in self.robots.items():
            msg = {
                'type': 'position',
                'robot_id': rid,
                'x': robot.x,
                'y': robot.y,
                'theta': robot.theta,
                'timestamp': time.time()
            }
            data = json.dumps(msg).encode()
            self.socket.sendto(data, (BROADCAST_IP, UDP_POSITION_PORT))
    
    def update_positions(self):
        """Move robots toward their targets (simple simulation)"""
        for rid, robot in self.robots.items():
            if robot.fault == 'stop':
                continue
            elif robot.fault == 'spin':
                robot.theta += 0.3
                continue
            elif robot.fault == 'erratic':
                robot.x += random.uniform(-30, 30)
                robot.y += random.uniform(-30, 30)
                robot.theta += random.uniform(-0.5, 0.5)
                robot.x = max(50, min(ARENA_WIDTH - 50, robot.x))
                robot.y = max(50, min(ARENA_HEIGHT - 50, robot.y))
                continue
            
            # Normal movement toward target
            dx = robot.target_x - robot.x
            dy = robot.target_y - robot.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 10:
                # Move toward target
                robot.theta = math.atan2(dy, dx)
                robot.x += ROBOT_SPEED * math.cos(robot.theta)
                robot.y += ROBOT_SPEED * math.sin(robot.theta)
                
                # Clamp to arena
                robot.x = max(50, min(ARENA_WIDTH - 50, robot.x))
                robot.y = max(50, min(ARENA_HEIGHT - 50, robot.y))
            else:
                # Pick new random target (simulates frontier selection)
                robot.target_x = random.uniform(100, ARENA_WIDTH - 100)
                robot.target_y = random.uniform(100, ARENA_HEIGHT - 100)
    
    def run(self):
        self.running = True
        interval = 1.0 / SIM_RATE
        
        while self.running:
            start = time.time()
            
            self.update_positions()
            self.send_positions()
            
            elapsed = time.time() - start
            if elapsed < interval:
                time.sleep(interval - elapsed)

# ============== State Monitor ==============
class StateMonitor:
    """Listens to peer broadcasts and displays state"""
    
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', UDP_PEER_PORT))
        self.socket.setblocking(False)
        
        self.states: Dict[int, dict] = {}
        self.running = False
    
    def receive(self):
        """Receive state broadcasts"""
        try:
            while True:
                data, _ = self.socket.recvfrom(8192)
                msg = json.loads(data.decode())
                if msg.get('type') == 'peer_state':
                    rid = msg.get('robot_id')
                    if rid:
                        self.states[rid] = msg
        except BlockingIOError:
            pass
    
    def display(self):
        """Print current state"""
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("=" * 70)
        print("REIP SIMULATION HARNESS")
        print("=" * 70)
        print()
        
        # Find leader
        leader_id = None
        for rid, state in self.states.items():
            if state.get('state') == 'leader':
                leader_id = rid
                break
        
        print(f"Leader: Robot {leader_id}")
        print()
        
        # Robot states
        print(f"{'Robot':<8} {'State':<12} {'Pos':<16} {'Trust':<8} {'Coverage':<10}")
        print("-" * 60)
        
        for rid in sorted(self.states.keys()):
            state = self.states[rid]
            pos = f"({state.get('x', 0):.0f}, {state.get('y', 0):.0f})"
            trust = state.get('trust_in_leader', 1.0)
            cov = state.get('coverage_count', 0)
            st = state.get('state', 'unknown')
            
            # Color trust
            trust_str = f"{trust:.2f}"
            if trust < 0.5:
                trust_str += " !"
            
            print(f"R{rid:<7} {st:<12} {pos:<16} {trust_str:<8} {cov:<10}")
        
        # Coverage
        total_cells = (ARENA_WIDTH // CELL_SIZE) * (ARENA_HEIGHT // CELL_SIZE)
        max_cov = max((s.get('coverage_count', 0) for s in self.states.values()), default=0)
        print()
        print(f"Max coverage: {max_cov}/{total_cells} cells ({100*max_cov/total_cells:.1f}%)")
        print()
        print("Commands: fault <id> <type> | clear <id> | quit")
        print("Fault types: spin, stop, erratic, bad_leader")
    
    def run(self):
        self.running = True
        while self.running:
            self.receive()
            self.display()
            time.sleep(0.5)

# ============== Fault Injector ==============
class FaultInjector:
    def __init__(self, robots: Dict[int, SimRobot]):
        self.robots = robots
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    def inject(self, robot_id: int, fault_type: str):
        """Inject fault into robot"""
        # Update local sim state
        if robot_id in self.robots:
            if fault_type in ('none', 'clear'):
                self.robots[robot_id].fault = None
            else:
                self.robots[robot_id].fault = fault_type
        
        # Send to reip_node (for bad_leader etc)
        msg = {
            'type': 'fault_inject',
            'robot_id': robot_id,
            'fault': fault_type,
            'timestamp': time.time()
        }
        data = json.dumps(msg).encode()
        self.socket.sendto(data, (BROADCAST_IP, UDP_FAULT_PORT))

# ============== Main ==============
def main():
    num_robots = int(sys.argv[1]) if len(sys.argv) > 1 else NUM_ROBOTS
    layout = sys.argv[2] if len(sys.argv) > 2 else "open"
    start_preset = sys.argv[3] if len(sys.argv) > 3 else None
    
    print(f"Starting REIP simulation with {num_robots} robots...")
    print("This tests the actual reip_node.py code with mocked positions.\n")
    if layout not in ARENA_LAYOUTS:
        print(f"Unknown layout '{layout}'. Available: {', '.join(ARENA_LAYOUTS.keys())}")
        sys.exit(1)
    if start_preset is not None and start_preset not in START_PRESETS:
        print(f"Unknown start preset '{start_preset}'. Available: {', '.join(START_PRESETS.keys())}")
        sys.exit(1)
    
    # Create simulated robots with random starting positions
    robots = {}
    preset_starts = START_PRESETS.get(start_preset, {})
    for i in range(1, num_robots + 1):
        if i in preset_starts:
            start_x, start_y, start_theta = preset_starts[i]
            target_x, target_y = start_x, start_y
        elif layout == "multiroom":
            col = (i - 1) % 2
            row = (i - 1) // 2
            start_x = 200 + col * 250
            start_y = 200 + row * 300
            start_theta = 0.0
            target_x, target_y = start_x, start_y
        else:
            start_x = random.uniform(200, ARENA_WIDTH - 200)
            start_y = random.uniform(200, ARENA_HEIGHT - 200)
            start_theta = random.uniform(0, 2 * math.pi)
            target_x = random.uniform(100, ARENA_WIDTH - 100)
            target_y = random.uniform(100, ARENA_HEIGHT - 100)
        robots[i] = SimRobot(
            robot_id=i,
            x=start_x,
            y=start_y,
            theta=start_theta,
            target_x=target_x,
            target_y=target_y
        )
    
    # Start mock position server
    pos_server = MockPositionServer(robots)
    pos_thread = threading.Thread(target=pos_server.run, daemon=True)
    pos_thread.start()
    
    # Start state monitor
    monitor = StateMonitor()
    monitor_thread = threading.Thread(target=monitor.run, daemon=True)
    monitor_thread.start()
    
    # Fault injector
    injector = FaultInjector(robots)
    
    print("Mock position server started.")
    print("Now start reip_node.py instances in separate terminals:")
    print()
    for i in range(1, num_robots + 1):
        print(f"  python robot/reip_node.py {i} --sim")
    print()
    print("Press Enter when robots are running, or type commands...")
    print()
    
    # Command loop
    try:
        while True:
            cmd = input().strip().lower()
            parts = cmd.split()
            
            if not parts:
                continue
            
            if parts[0] == 'quit':
                break
            
            elif parts[0] == 'fault' and len(parts) >= 3:
                rid = int(parts[1])
                fault = parts[2]
                injector.inject(rid, fault)
                print(f"Injected {fault} on robot {rid}")
            
            elif parts[0] == 'clear' and len(parts) >= 2:
                rid = int(parts[1])
                injector.inject(rid, 'none')
                print(f"Cleared fault on robot {rid}")
            
            elif parts[0] == 'status':
                pass  # Monitor already displays
            
            else:
                print("Commands: fault <id> <type> | clear <id> | quit")
    
    except KeyboardInterrupt:
        pass
    
    print("\nStopping simulation...")
    pos_server.running = False
    monitor.running = False

if __name__ == "__main__":
    main()
