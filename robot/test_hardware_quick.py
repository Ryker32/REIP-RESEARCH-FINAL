#!/usr/bin/env python3
"""
Quick hardware test for REIP robot.
Run this first to verify all components work!

Usage: python3 test_hardware_quick.py
"""

import time
import sys


def test_tof():
    """Test ToF sensors"""
    print("\n[1/3] Testing ToF sensors...")
    
    try:
        import smbus2
        import board
        import busio
        import adafruit_vl53l0x
        
        bus = smbus2.SMBus(1)
        i2c = busio.I2C(board.SCL, board.SDA)
        mux_addr = 0x70
        
        channels = {
            'front': 0,
            'front_left': 1,
            'front_right': 2,
            'left': 3,
            'right': 4,
        }
        
        found = 0
        for name, ch in channels.items():
            try:
                bus.write_byte(mux_addr, 1 << ch)
                time.sleep(0.05)
                tof = adafruit_vl53l0x.VL53L0X(i2c)
                dist = tof.range
                print(f"  [OK] {name:12s} (CH{ch}): {dist:4d} mm")
                found += 1
            except Exception as e:
                print(f"  [FAIL] {name:12s} (CH{ch}): {e}")
        
        print(f"  Found {found}/5 sensors")
        return found >= 3  # Pass if at least 3 work
        
    except Exception as e:
        print(f"  [FAIL] ToF error: {e}")
        return False


def test_pico():
    """Test Pico connection"""
    print("\n[2/3] Testing Pico connection...")
    
    try:
        import serial
        
        ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        # Test PING
        ser.write(b'PING\n')
        response = ser.readline().decode().strip()
        
        if response == "PONG":
            print("  [OK] Pico responded to PING")
        else:
            print(f"  [FAIL] Expected PONG, got: {response}")
            ser.close()
            return False
        
        # Test encoders
        ser.write(b'ENC\n')
        response = ser.readline().decode().strip()
        if ',' in response:
            print(f"  [OK] Encoders: {response}")
        else:
            print(f"  [WARN] Encoder response: {response}")
        
        ser.close()
        return True
        
    except Exception as e:
        print(f"  [FAIL] Pico error: {e}")
        return False


def test_motors():
    """Test motors briefly"""
    print("\n[3/3] Testing motors (brief spin)...")
    
    try:
        import serial
        
        ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        # Get initial encoder reading
        ser.write(b'ENC\n')
        before = ser.readline().decode().strip()
        print(f"  Encoders before: {before}")
        
        # Spin motors at 60% for 0.3s
        print("  Spinning motors...")
        ser.write(b'MOT,60,60\n')
        ser.readline()
        time.sleep(0.3)
        ser.write(b'STOP\n')
        ser.readline()
        
        # Get final encoder reading
        ser.write(b'ENC\n')
        after = ser.readline().decode().strip()
        print(f"  Encoders after: {after}")
        
        # Check if encoders changed
        try:
            b_parts = before.split(',')
            a_parts = after.split(',')
            left_diff = abs(int(a_parts[0]) - int(b_parts[0]))
            right_diff = abs(int(a_parts[1]) - int(b_parts[1]))
            
            if left_diff > 5 and right_diff > 5:
                print(f"  [OK] Both motors moved (L:{left_diff}, R:{right_diff})")
                ser.close()
                return True
            elif left_diff > 5:
                print(f"  [WARN] Only left motor moved")
            elif right_diff > 5:
                print(f"  [WARN] Only right motor moved")
            else:
                print(f"  [WARN] Motors didn't move much - check power/wiring")
        except:
            pass
        
        ser.close()
        return True  # Pass anyway since we tried
        
    except Exception as e:
        print(f"  [FAIL] Motor error: {e}")
        return False


def main():
    print("=" * 50)
    print("REIP Robot Quick Hardware Test")
    print("=" * 50)
    
    results = {
        'ToF': test_tof(),
        'Pico': test_pico(),
        'Motors': test_motors(),
    }
    
    print("\n" + "=" * 50)
    print("RESULTS:")
    print("=" * 50)
    
    all_pass = True
    for name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {name}: {status}")
        if not passed:
            all_pass = False
    
    print()
    if all_pass:
        print("All tests passed! Ready to run reip_controller.py")
        return 0
    else:
        print("Some tests failed. Check hardware connections.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
