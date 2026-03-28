#!/usr/bin/env python3
"""
REIP Robot UDP Agent
Receives motor commands from coordinator, sends back sensor telemetry.

Usage: python3 udp_agent.py <robot_id>
Example: python3 udp_agent.py 1
"""

import sys
import time
import json
import socket
import threading
import serial
import smbus2
import board
import busio
import adafruit_vl53l0x

# ============================================================================
# CONFIGURATION
# ============================================================================

COORDINATOR_IP = "255.255.255.255"  # Broadcast (or set coordinator's IP)
CMD_PORT = 5001        # Receive motor commands
TELEMETRY_PORT = 5002  # Send sensor data
TELEMETRY_RATE = 10    # Hz

# Safety: stop if no commands received for this long
COMMAND_TIMEOUT = 0.5  # seconds

# ============================================================================
# HARDWARE
# ============================================================================

class ToFSensors:
    CHANNELS = {'right': 0, 'front_right': 1, 'front': 2, 'front_left': 3, 'left': 4}
    
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensors = {}
    
    def init(self):
        for name, ch in self.CHANNELS.items():
            try:
                self.bus.write_byte(0x70, 1 << ch)
                time.sleep(0.05)
                self.sensors[name] = adafruit_vl53l0x.VL53L0X(self.i2c)
                print(f"  [OK] {name} (CH{ch})")
            except:
                print(f"  [FAIL] {name} (CH{ch})")
    
    def read_all(self):
        readings = {}
        for name, ch in self.CHANNELS.items():
            try:
                self.bus.write_byte(0x70, 1 << ch)
                readings[name] = self.sensors[name].range if name in self.sensors else 0
            except:
                readings[name] = 0
        return readings


class Pico:
    def __init__(self):
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=0.5)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        # Motor calibration - adjust per robot
        self.left_scale = 1.15
        self.right_scale = 1.0
        self.min_power = 30
    
    def motors(self, l, r):
        l = l * self.left_scale
        r = r * self.right_scale
        if abs(l) < 10: l = 0
        elif abs(l) < self.min_power: l = self.min_power if l > 0 else -self.min_power
        if abs(r) < 10: r = 0
        elif abs(r) < self.min_power: r = self.min_power if r > 0 else -self.min_power
        l = max(-100, min(100, l))
        r = max(-100, min(100, r))
        self.ser.write(f'MOT,{l:.0f},{r:.0f}\n'.encode())
        self.ser.readline()
    
    def stop(self):
        self.ser.write(b'STOP\n')
        self.ser.readline()
    
    def encoders(self):
        self.ser.write(b'ENC\n')
        r = self.ser.readline().decode().strip()
        if ',' in r:
            p = r.split(',')
            return int(p[0]), int(p[1])
        return 0, 0


# ============================================================================
# UDP AGENT
# ============================================================================

class UDPAgent:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.running = True
        self.last_cmd_time = time.time()
        self.current_cmd = (0, 0)
        
        # Hardware
        print(f"[Agent {robot_id}] Initializing hardware...")
        print("[INIT] ToF sensors...")
        self.tof = ToFSensors()
        self.tof.init()
        print("[INIT] Pico...")
        self.pico = Pico()
        
        # UDP sockets
        # Command receiver
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.cmd_sock.bind(('', CMD_PORT))
        self.cmd_sock.setblocking(False)
        
        # Telemetry sender
        self.tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        
        print(f"[Agent {robot_id}] Ready! Listening on port {CMD_PORT}")
    
    def receive_commands(self):
        """Check for incoming motor commands."""
        try:
            data, addr = self.cmd_sock.recvfrom(1024)
            msg = json.loads(data.decode())
            
            # Check if command is for this robot
            target_id = msg.get('id')
            if target_id == self.robot_id or target_id == 'all':
                left = msg.get('left', 0)
                right = msg.get('right', 0)
                self.current_cmd = (left, right)
                self.last_cmd_time = time.time()
                return True
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[ERR] Command receive: {e}")
        return False
    
    def send_telemetry(self):
        """Send sensor data to coordinator."""
        try:
            sensors = self.tof.read_all()
            enc_l, enc_r = self.pico.encoders()
            
            msg = {
                'id': self.robot_id,
                'ts': time.time(),
                'tof': sensors,
                'enc': [enc_l, enc_r],
            }
            
            data = json.dumps(msg).encode()
            self.tel_sock.sendto(data, ('<broadcast>', TELEMETRY_PORT))
        except Exception as e:
            print(f"[ERR] Telemetry send: {e}")
    
    def check_timeout(self):
        """Stop motors if no commands received recently."""
        if time.time() - self.last_cmd_time > COMMAND_TIMEOUT:
            if self.current_cmd != (0, 0):
                print("\n[SAFETY] Command timeout - stopping")
                self.current_cmd = (0, 0)
                self.pico.stop()
    
    def run(self):
        """Main loop."""
        print(f"[Agent {self.robot_id}] Running... Ctrl+C to stop")
        
        tel_interval = 1.0 / TELEMETRY_RATE
        last_tel = 0
        
        try:
            while self.running:
                # Receive commands
                self.receive_commands()
                
                # Check safety timeout
                self.check_timeout()
                
                # Execute motor command
                self.pico.motors(self.current_cmd[0], self.current_cmd[1])
                
                # Send telemetry at fixed rate
                now = time.time()
                if now - last_tel >= tel_interval:
                    self.send_telemetry()
                    last_tel = now
                    
                    # Status display
                    sensors = self.tof.read_all()
                    print(f"\r[R{self.robot_id}] F={sensors['front']:4d} "
                          f"L={self.current_cmd[0]:+4.0f} R={self.current_cmd[1]:+4.0f}", end="")
                
                time.sleep(0.02)  # 50Hz loop
                
        except KeyboardInterrupt:
            print(f"\n[Agent {self.robot_id}] Shutting down...")
        finally:
            self.pico.stop()
            self.cmd_sock.close()
            self.tel_sock.close()


# ============================================================================
# MAIN
# ============================================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 udp_agent.py <robot_id>")
        print("Example: python3 udp_agent.py 1")
        sys.exit(1)
    
    robot_id = int(sys.argv[1])
    agent = UDPAgent(robot_id)
    agent.run()


if __name__ == "__main__":
    main()
