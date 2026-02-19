#!/usr/bin/env python3
"""REIP Robot Controller - Simple Navigation (Phase 1)
Tested and working on clanker1.
"""
import time
import serial, smbus2, board, busio, adafruit_vl53l0x

class ToFSensors:
    # Sensor layout: 5 sensors in forward arc
    # CH0=-75°(right), CH1=-37.5°, CH2=0°(front), CH3=+37.5°, CH4=+75°(left)
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
                readings[name] = self.sensors[name].range if name in self.sensors else 2000
            except: 
                readings[name] = 2000
        return readings


class Pico:
    def __init__(self):
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=0.5)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        # Motor calibration - tune per robot
        self.left_scale = 1.15    # clanker1: left motor needs 15% boost
        self.right_scale = 1.0
        self.min_power = 30       # Minimum to overcome static friction
        self.last_enc = (0, 0)
    
    def motors(self, l, r):
        l = l * self.left_scale
        r = r * self.right_scale
        # Deadband or minimum (no hum zone)
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
            self.last_enc = (int(p[0]), int(p[1]))
        return self.last_enc


class SmartNavigator:
    """State-machine navigator with corner escape and stuck detection."""
    
    def __init__(self):
        self.threshold = 200      # Obstacle detection distance (mm)
        self.close = 120          # Too close - must turn
        self.max_speed = 55
        self.state = 'explore'    # explore, escape, wall_follow
        self.escape_start = 0
        self.escape_dir = 1       # 1 = right, -1 = left
        self.stuck_count = 0
        self.last_enc = (0, 0)
    
    def compute(self, s, enc):
        f = s.get('front', 2000)
        fl = s.get('front_left', 2000)
        fr = s.get('front_right', 2000)
        left = s.get('left', 2000)
        right = s.get('right', 2000)
        
        # Check if stuck (encoders not changing)
        enc_delta = abs(enc[0] - self.last_enc[0]) + abs(enc[1] - self.last_enc[1])
        self.last_enc = enc
        if enc_delta < 5:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        
        # STUCK - spin to escape
        if self.stuck_count > 10:
            self.state = 'escape'
            self.escape_start = time.time()
            self.escape_dir = 1 if left > right else -1
            self.stuck_count = 0
        
        # CORNER - all front sensors blocked
        if f < self.close and fl < self.threshold and fr < self.threshold:
            self.state = 'escape'
            self.escape_start = time.time()
            self.escape_dir = 1 if left > right else -1
        
        # STATE MACHINE
        if self.state == 'escape':
            # Spin in place until timeout or front clear
            if time.time() - self.escape_start > 1.0 or f > self.threshold * 1.5:
                self.state = 'explore'
                return 0, 0
            spin = 45 * self.escape_dir
            return -spin, spin
        
        elif self.state == 'wall_follow':
            # Follow right wall
            target_dist = 150
            if right < 100:  # Too close to wall
                return self.max_speed * 0.6, self.max_speed * 0.3
            elif right > 200:  # Too far from wall
                return self.max_speed * 0.3, self.max_speed * 0.6
            else:  # Good distance
                if f < self.threshold:
                    self.state = 'explore'  # Wall ended or corner
                return self.max_speed * 0.7, self.max_speed * 0.7
        
        else:  # explore
            # Front blocked - turn toward open space
            if f < self.close:
                if left > right:
                    return -35, 35  # Spin left
                else:
                    return 35, -35  # Spin right
            
            # Front somewhat blocked - curve away
            if f < self.threshold:
                if left > right:
                    return self.max_speed * 0.3, self.max_speed * 0.7
                else:
                    return self.max_speed * 0.7, self.max_speed * 0.3
            
            # Side obstacles - gentle curve
            if fl < self.threshold:
                return self.max_speed * 0.8, self.max_speed * 0.5
            if fr < self.threshold:
                return self.max_speed * 0.5, self.max_speed * 0.8
            
            # Clear - go straight
            return self.max_speed, self.max_speed


def main():
    print("[REIP] clanker1 starting...")
    print("[INIT] ToF sensors...")
    tof = ToFSensors()
    tof.init()
    print("[INIT] Pico...")
    pico = Pico()
    nav = SmartNavigator()
    
    print("[REIP] Running! Ctrl+C to stop")
    try:
        while True:
            s = tof.read_all()
            enc = pico.encoders()
            l, r = nav.compute(s, enc)
            pico.motors(l, r)
            print(f"\r[{nav.state:11s}] L={s['left']:4d} FL={s['front_left']:4d} F={s['front']:4d} FR={s['front_right']:4d} R={s['right']:4d} | {l:+5.0f} {r:+5.0f}", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[REIP] Stopping...")
    finally:
        pico.stop()


if __name__ == "__main__":
    main()
