#!/usr/bin/env python3
"""
REIP Robot Controller - Pi Zero 2W
Main controller for multi-robot exploration with trust-based governance.

Hardware:
- 5x VL53L0X ToF sensors via PCA9548A I2C mux
- Motor control via Pico (UART)
- WiFi for multi-robot communication

Run: python3 reip_controller.py --robot-id 0
"""

import time
import math
import json
import socket
import struct
import argparse
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

# Hardware imports
import serial
import smbus2
import board
import busio
import adafruit_vl53l0x


# ============================================================================
# CONFIGURATION
# ============================================================================

@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    robot_id: int = 0
    num_robots: int = 6
    
    # Physical dimensions (mm)
    wheel_diameter: float = 32.0
    wheel_base: float = 85.0       # Distance between wheels
    encoder_ticks_per_rev: int = 360
    
    # Sensor layout (angles in degrees, 0 = forward, CCW positive)
    sensor_angles: Dict[str, float] = field(default_factory=lambda: {
        'front': 0,
        'front_left': 45,
        'front_right': -45,
        'left': 90,
        'right': -90,
    })
    
    # Control parameters
    control_rate_hz: float = 10.0  # Control loop frequency
    max_speed: float = 70.0        # Max motor speed (0-100)
    min_speed: float = 40.0        # Min speed to overcome static friction
    
    # Navigation
    obstacle_threshold_mm: int = 150   # Stop if obstacle closer than this
    wall_follow_distance_mm: int = 100 # Target wall distance
    
    # Communication
    multicast_group: str = "239.255.0.1"
    multicast_port: int = 5007
    comm_radius: float = 2.0  # meters (for trust calculations)
    
    # REIP parameters
    trust_threshold: float = 0.6
    trust_decay_rate: float = 0.1
    trust_recovery_rate: float = 0.02


# ============================================================================
# HARDWARE INTERFACES
# ============================================================================

class ToFSensors:
    """VL53L0X ToF sensor array with I2C multiplexer"""
    
    CHANNELS = {
        'front': 0,
        'front_left': 1,
        'front_right': 2,
        'left': 3,
        'right': 4,
    }
    
    def __init__(self, i2c_bus=1, mux_address=0x70):
        self.bus = smbus2.SMBus(i2c_bus)
        self.mux_addr = mux_address
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensors = {}
        self.last_readings = {}
        
    def select_channel(self, channel: int):
        """Select mux channel"""
        self.bus.write_byte(self.mux_addr, 1 << channel)
        time.sleep(0.01)
    
    def init(self) -> Dict[str, bool]:
        """Initialize all sensors, returns success status for each"""
        results = {}
        for name, channel in self.CHANNELS.items():
            try:
                self.select_channel(channel)
                time.sleep(0.05)
                sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
                self.sensors[name] = sensor
                results[name] = True
                print(f"  [OK] ToF '{name}' (CH{channel})")
            except Exception as e:
                results[name] = False
                print(f"  [FAIL] ToF '{name}' (CH{channel}): {e}")
        return results
    
    def read_all(self) -> Dict[str, int]:
        """Read all sensors, returns distances in mm"""
        readings = {}
        for name, channel in self.CHANNELS.items():
            try:
                self.select_channel(channel)
                sensor = self.sensors.get(name)
                if sensor:
                    readings[name] = sensor.range
                else:
                    readings[name] = 0
            except Exception:
                readings[name] = 0
        self.last_readings = readings
        return readings
    
    def read(self, name: str) -> int:
        """Read single sensor"""
        if name not in self.CHANNELS:
            return 0
        try:
            self.select_channel(self.CHANNELS[name])
            sensor = self.sensors.get(name)
            if sensor:
                return sensor.range
        except Exception:
            pass
        return 0


class PicoInterface:
    """UART interface to Pico motor controller"""
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        time.sleep(0.5)  # Wait for Pico to initialize
        self.ser.reset_input_buffer()
        self.last_encoders = (0, 0)
        
    def ping(self) -> bool:
        """Test connection to Pico"""
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b'PING\n')
            response = self.ser.readline().decode().strip()
            return response == "PONG"
        except Exception:
            return False
    
    def set_motors(self, left: float, right: float) -> bool:
        """Set motor speeds (-100 to 100)"""
        try:
            left = max(-100, min(100, left))
            right = max(-100, min(100, right))
            cmd = f"MOT,{left:.1f},{right:.1f}\n"
            self.ser.write(cmd.encode())
            response = self.ser.readline().decode().strip()
            return response == "OK"
        except Exception:
            return False
    
    def stop(self) -> bool:
        """Stop motors"""
        try:
            self.ser.write(b'STOP\n')
            response = self.ser.readline().decode().strip()
            return response == "OK"
        except Exception:
            return False
    
    def get_encoders(self) -> Tuple[int, int]:
        """Get encoder counts (left, right)"""
        try:
            self.ser.write(b'ENC\n')
            response = self.ser.readline().decode().strip()
            if ',' in response:
                parts = response.split(',')
                left = int(parts[0])
                right = int(parts[1])
                self.last_encoders = (left, right)
                return (left, right)
        except Exception:
            pass
        return self.last_encoders
    
    def reset_encoders(self) -> bool:
        """Reset encoder counts to zero"""
        try:
            self.ser.write(b'RST\n')
            response = self.ser.readline().decode().strip()
            if response == "OK":
                self.last_encoders = (0, 0)
                return True
        except Exception:
            pass
        return False
    
    def close(self):
        """Close serial connection"""
        self.stop()
        self.ser.close()


# ============================================================================
# ODOMETRY
# ============================================================================

class Odometry:
    """Dead reckoning from wheel encoders"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.x = 0.0  # meters
        self.y = 0.0
        self.theta = 0.0  # radians
        
        self.last_left = 0
        self.last_right = 0
        
        # Precompute constants
        self.meters_per_tick = (math.pi * config.wheel_diameter / 1000.0) / config.encoder_ticks_per_rev
        self.wheel_base_m = config.wheel_base / 1000.0
    
    def update(self, left_ticks: int, right_ticks: int):
        """Update pose from encoder ticks"""
        # Calculate deltas
        dl = (left_ticks - self.last_left) * self.meters_per_tick
        dr = (right_ticks - self.last_right) * self.meters_per_tick
        
        self.last_left = left_ticks
        self.last_right = right_ticks
        
        # Differential drive kinematics
        dc = (dl + dr) / 2.0  # Center displacement
        dtheta = (dr - dl) / self.wheel_base_m  # Rotation
        
        # Update pose
        if abs(dtheta) < 1e-6:
            # Straight line
            self.x += dc * math.cos(self.theta)
            self.y += dc * math.sin(self.theta)
        else:
            # Arc motion
            radius = dc / dtheta
            self.x += radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + dtheta))
        
        self.theta += dtheta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get current pose (x, y, theta)"""
        return (self.x, self.y, self.theta)
    
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset pose"""
        self.x = x
        self.y = y
        self.theta = theta
        self.last_left = 0
        self.last_right = 0


# ============================================================================
# BELIEF MAP (Local occupancy grid)
# ============================================================================

class BeliefMap:
    """Local occupancy grid from sensor observations"""
    
    def __init__(self, size: int = 100, resolution: float = 0.02):
        """
        Args:
            size: Grid cells per side
            resolution: Meters per cell
        """
        self.size = size
        self.resolution = resolution
        self.half_size = size // 2
        
        # Occupancy: -1 = unknown, 0 = free, 1 = obstacle
        self.grid = [[-1] * size for _ in range(size)]
        self.update_count = 0
    
    def world_to_grid(self, wx: float, wy: float, robot_x: float, robot_y: float) -> Tuple[int, int]:
        """Convert world coords to grid coords (robot-centered)"""
        gx = int((wx - robot_x) / self.resolution + self.half_size)
        gy = int((wy - robot_y) / self.resolution + self.half_size)
        return (gx, gy)
    
    def update_from_sensor(self, robot_pose: Tuple[float, float, float],
                           sensor_angle: float, distance_mm: int, max_range_mm: int = 2000):
        """
        Update map from single sensor reading.
        
        Args:
            robot_pose: (x, y, theta) in meters/radians
            sensor_angle: Sensor angle relative to robot (degrees)
            distance_mm: Measured distance
            max_range_mm: Max sensor range
        """
        rx, ry, rtheta = robot_pose
        
        # Sensor ray angle in world frame
        ray_angle = rtheta + math.radians(sensor_angle)
        
        # Distance in meters
        dist_m = min(distance_mm, max_range_mm) / 1000.0
        
        # Mark cells as free along the ray
        num_steps = int(dist_m / self.resolution)
        for i in range(num_steps):
            d = i * self.resolution
            wx = rx + d * math.cos(ray_angle)
            wy = ry + d * math.sin(ray_angle)
            gx, gy = self.world_to_grid(wx, wy, rx, ry)
            if 0 <= gx < self.size and 0 <= gy < self.size:
                self.grid[gx][gy] = 0  # Free
        
        # Mark obstacle at endpoint (if within range)
        if distance_mm < max_range_mm:
            ox = rx + dist_m * math.cos(ray_angle)
            oy = ry + dist_m * math.sin(ray_angle)
            gx, gy = self.world_to_grid(ox, oy, rx, ry)
            if 0 <= gx < self.size and 0 <= gy < self.size:
                self.grid[gx][gy] = 1  # Obstacle
        
        self.update_count += 1
    
    def get_unknown_count(self) -> int:
        """Count unknown cells"""
        count = 0
        for row in self.grid:
            for cell in row:
                if cell == -1:
                    count += 1
        return count
    
    def serialize(self) -> bytes:
        """Serialize map for network transmission"""
        # Compact encoding: 2 bits per cell
        data = []
        for row in self.grid:
            for cell in row:
                # -1->0, 0->1, 1->2
                data.append(cell + 1)
        return bytes(data)
    
    def merge(self, other_data: bytes, transform: Tuple[float, float, float] = None):
        """Merge another robot's map data"""
        # TODO: Apply transform and merge
        pass


# ============================================================================
# COMMUNICATION
# ============================================================================

class MultiRobotComm:
    """UDP multicast communication for robot coordination"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.robot_id = config.robot_id
        
        # Outgoing socket
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        
        # Incoming socket
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(('', config.multicast_port))
        
        # Join multicast group
        mreq = struct.pack("4sl", socket.inet_aton(config.multicast_group), socket.INADDR_ANY)
        self.rx_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.rx_sock.setblocking(False)
        
        # State
        self.peers = {}  # robot_id -> {pose, timestamp, ...}
        self.running = True
        self.lock = threading.Lock()
        
        # Start receiver thread
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
    
    def broadcast_state(self, pose: Tuple[float, float, float], 
                        sensors: Dict[str, int],
                        leader_id: int,
                        trust_in_leader: float):
        """Broadcast robot state to peers"""
        msg = {
            'id': self.robot_id,
            'ts': time.time(),
            'pose': pose,
            'sensors': sensors,
            'leader': leader_id,
            'trust': trust_in_leader,
        }
        data = json.dumps(msg).encode()
        self.tx_sock.sendto(data, (self.config.multicast_group, self.config.multicast_port))
    
    def _receive_loop(self):
        """Background thread to receive peer messages"""
        while self.running:
            try:
                data, addr = self.rx_sock.recvfrom(4096)
                msg = json.loads(data.decode())
                peer_id = msg.get('id')
                
                if peer_id is not None and peer_id != self.robot_id:
                    with self.lock:
                        self.peers[peer_id] = msg
            except BlockingIOError:
                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)
    
    def get_peers(self) -> Dict[int, dict]:
        """Get current peer states"""
        with self.lock:
            # Filter stale peers (>2 seconds old)
            now = time.time()
            return {k: v for k, v in self.peers.items() 
                    if now - v.get('ts', 0) < 2.0}
    
    def close(self):
        """Shutdown communication"""
        self.running = False
        self.tx_sock.close()
        self.rx_sock.close()


# ============================================================================
# REIP GOVERNANCE
# ============================================================================

class REIPGovernance:
    """Resilient Election & Impeachment Protocol"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.robot_id = config.robot_id
        self.num_robots = config.num_robots
        
        # Trust matrix: trust[i][j] = i's trust in j
        self.trust = [[1.0] * config.num_robots for _ in range(config.num_robots)]
        
        # Leadership
        self.leader_id = 0
        self.election_count = 0
        
        # Tracking
        self.predicted_gain = 0.0
        self.observed_gain = 0.0
        self.coverage_history = deque(maxlen=20)
    
    def update_trust(self, peer_performances: Dict[int, Tuple[float, float]]):
        """
        Update trust based on predicted vs observed performance.
        
        Args:
            peer_performances: {robot_id: (predicted_gain, observed_gain)}
        """
        leader = self.leader_id
        
        for peer_id, (pred, obs) in peer_performances.items():
            if peer_id == leader:
                # Update trust in leader
                error = max(0.0, pred - obs)
                decay = math.exp(-self.config.trust_decay_rate * error)
                
                for i in range(self.num_robots):
                    if i != leader:
                        self.trust[i][leader] = max(0.1, self.trust[i][leader] * decay)
                        
                        # Recovery for non-leaders
                        for j in range(self.num_robots):
                            if j != leader and self.trust[i][j] < 1.0:
                                self.trust[i][j] = min(1.0, 
                                    self.trust[i][j] + self.config.trust_recovery_rate)
    
    def get_avg_trust_in_leader(self) -> float:
        """Get average trust in current leader"""
        leader = self.leader_id
        total = sum(self.trust[i][leader] for i in range(self.num_robots) if i != leader)
        count = self.num_robots - 1
        return total / count if count > 0 else 1.0
    
    def should_impeach(self) -> bool:
        """Check if impeachment should be triggered"""
        return self.get_avg_trust_in_leader() < self.config.trust_threshold
    
    def elect_new_leader(self) -> int:
        """Elect new leader based on trust scores"""
        # Sum trust toward each candidate
        scores = []
        for candidate in range(self.num_robots):
            score = sum(self.trust[i][candidate] 
                       for i in range(self.num_robots) if i != candidate)
            scores.append((score, candidate))
        
        scores.sort(reverse=True)
        new_leader = scores[0][1]
        
        if new_leader != self.leader_id:
            old_leader = self.leader_id
            self.leader_id = new_leader
            self.election_count += 1
            print(f"[REIP] Election: {old_leader} -> {new_leader}")
        
        return self.leader_id
    
    def am_i_leader(self) -> bool:
        """Check if this robot is the current leader"""
        return self.robot_id == self.leader_id


# ============================================================================
# NAVIGATION BEHAVIORS
# ============================================================================

class Navigator:
    """Simple reactive navigation behaviors"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
    
    def avoid_obstacles(self, sensors: Dict[str, int]) -> Tuple[float, float]:
        """
        Reactive obstacle avoidance.
        Returns (left_speed, right_speed)
        """
        front = sensors.get('front', 2000)
        front_left = sensors.get('front_left', 2000)
        front_right = sensors.get('front_right', 2000)
        left = sensors.get('left', 2000)
        right = sensors.get('right', 2000)
        
        threshold = self.config.obstacle_threshold_mm
        max_speed = self.config.max_speed
        
        # Emergency stop if too close
        if front < threshold * 0.5:
            return (0, 0)
        
        # Weighted obstacle influence
        weights = {
            'front': 1.0,
            'front_left': 0.7,
            'front_right': 0.7,
            'left': 0.3,
            'right': 0.3,
        }
        
        # Calculate turn bias based on obstacles
        left_influence = (weights['front_left'] * max(0, threshold - front_left) +
                         weights['left'] * max(0, threshold - left))
        right_influence = (weights['front_right'] * max(0, threshold - front_right) +
                          weights['right'] * max(0, threshold - right))
        
        turn_bias = (right_influence - left_influence) / threshold
        
        # Forward speed based on front clearance
        forward_factor = min(1.0, front / (threshold * 2))
        base_speed = max_speed * forward_factor
        
        # Apply turn
        left_speed = base_speed * (1.0 - turn_bias * 0.5)
        right_speed = base_speed * (1.0 + turn_bias * 0.5)
        
        return (left_speed, right_speed)
    
    def explore(self, sensors: Dict[str, int], belief_map: BeliefMap) -> Tuple[float, float]:
        """
        Exploration behavior - head toward unknown areas.
        """
        # For now, just avoid obstacles and keep moving
        return self.avoid_obstacles(sensors)


# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class REIPRobotController:
    """Main robot controller"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.running = True
        
        print(f"[REIP] Initializing Robot {config.robot_id}")
        
        # Initialize hardware
        print("[INIT] ToF sensors...")
        self.tof = ToFSensors()
        self.tof.init()
        
        print("[INIT] Pico interface...")
        self.pico = PicoInterface()
        if self.pico.ping():
            print("  [OK] Pico connected")
        else:
            print("  [WARN] Pico not responding")
        
        # Initialize subsystems
        self.odom = Odometry(config)
        self.belief = BeliefMap()
        self.nav = Navigator(config)
        self.reip = REIPGovernance(config)
        
        print("[INIT] Multi-robot comm...")
        self.comm = MultiRobotComm(config)
        
        print(f"[REIP] Robot {config.robot_id} ready!")
    
    def update_sensors(self) -> Dict[str, int]:
        """Read all sensors"""
        return self.tof.read_all()
    
    def update_odometry(self):
        """Update pose from encoders"""
        left, right = self.pico.get_encoders()
        self.odom.update(left, right)
    
    def update_belief(self, sensors: Dict[str, int]):
        """Update local map from sensor readings"""
        pose = self.odom.get_pose()
        
        for name, distance in sensors.items():
            if name in self.config.sensor_angles:
                angle = self.config.sensor_angles[name]
                self.belief.update_from_sensor(pose, angle, distance)
    
    def control_step(self, sensors: Dict[str, int]) -> Tuple[float, float]:
        """Compute motor commands"""
        # Simple exploration for now
        return self.nav.explore(sensors, self.belief)
    
    def governance_step(self):
        """REIP governance update"""
        # Get peer states
        peers = self.comm.get_peers()
        
        # Update trust based on observed performance
        # (simplified - full implementation would track predictions vs observations)
        
        # Check for impeachment
        if self.reip.should_impeach():
            self.reip.elect_new_leader()
        
        return self.reip.leader_id
    
    def run(self):
        """Main control loop"""
        period = 1.0 / self.config.control_rate_hz
        
        print(f"[REIP] Starting control loop at {self.config.control_rate_hz} Hz")
        print("Press Ctrl+C to stop")
        
        try:
            while self.running:
                start = time.time()
                
                # 1. Update sensors
                sensors = self.update_sensors()
                
                # 2. Update odometry
                self.update_odometry()
                pose = self.odom.get_pose()
                
                # 3. Update local map
                self.update_belief(sensors)
                
                # 4. Governance (REIP)
                leader_id = self.governance_step()
                
                # 5. Compute control
                left, right = self.control_step(sensors)
                
                # 6. Execute motors
                self.pico.set_motors(left, right)
                
                # 7. Broadcast state
                trust = self.reip.get_avg_trust_in_leader()
                self.comm.broadcast_state(pose, sensors, leader_id, trust)
                
                # 8. Status print (every 1s)
                if int(time.time()) % 1 == 0:
                    print(f"\r[{self.config.robot_id}] "
                          f"x={pose[0]:.2f} y={pose[1]:.2f} theta={math.degrees(pose[2]):.0f} deg "
                          f"F={sensors.get('front', 0):4d}mm "
                          f"L={leader_id} T={trust:.2f}", end="")
                
                # Sleep for remaining time
                elapsed = time.time() - start
                if elapsed < period:
                    time.sleep(period - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[REIP] Stopping...")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        self.pico.stop()
        self.pico.close()
        self.comm.close()
        print("[REIP] Shutdown complete")


# ============================================================================
# ENTRY POINT
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='REIP Robot Controller')
    parser.add_argument('--robot-id', type=int, default=0, help='Robot ID (0-5)')
    parser.add_argument('--num-robots', type=int, default=6, help='Total number of robots')
    args = parser.parse_args()
    
    config = RobotConfig(
        robot_id=args.robot_id,
        num_robots=args.num_robots,
    )
    
    controller = REIPRobotController(config)
    controller.run()


if __name__ == "__main__":
    main()
