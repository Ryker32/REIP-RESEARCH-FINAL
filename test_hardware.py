#!/usr/bin/env python3
"""
REIP Robot - Hardware Test Script
Run this first to verify all components work!
"""

import sys
import time


def test_i2c():
    """Test I2C bus and mux detection"""
    print("\n[1/4] Testing I2C Bus...")

    try:
        import smbus2

        bus = smbus2.SMBus(1)

        # Try to read mux
        try:
            bus.write_byte(0x70, 0x00)
            print("  [OK] PCA9548A mux detected at 0x70")
        except Exception:
            print("  [FAIL] PCA9548A mux NOT found at 0x70")
            print("    Check: VIN->3.3V, GND, SDA->GPIO2, SCL->GPIO3")
            return False

        # Check each ToF channel
        tof_found = 0
        for ch in range(5):
            try:
                bus.write_byte(0x70, 1 << ch)
                time.sleep(0.01)
                model = bus.read_byte_data(0x29, 0xC0)
                if model == 0xEE:
                    print(f"  [OK] ToF sensor on channel {ch}")
                    tof_found += 1
            except Exception:
                print(f"  [FAIL] No ToF on channel {ch}")

        print(f"  Found {tof_found}/5 ToF sensors")
        return tof_found > 0

    except Exception as e:
        print(f"  [FAIL] I2C error: {e}")
        return False


def test_uart():
    """Test UART communication with Pico"""
    print("\n[2/4] Testing UART to Pico...")

    try:
        import serial
        import json

        ser = serial.Serial("/dev/serial0", 115200, timeout=1)
        time.sleep(0.1)

        # Clear buffer
        ser.reset_input_buffer()

        # Send ping
        ser.write(b'{"cmd":"ping"}\n')
        time.sleep(0.1)

        response = ser.readline().decode().strip()
        if response:
            try:
                data = json.loads(response)
                if data.get("ok"):
                    print(f"  [OK] Pico responded: {response}")
                    return True
            except Exception:
                pass

        print("  [FAIL] No response from Pico")
        print("    Check: TX->Pico RX, RX<-Pico TX, common GND")
        print("    Is Pico running main.py?")
        return False

    except Exception as e:
        print(f"  [FAIL] UART error: {e}")
        return False


def test_motors():
    """Test motors via Pico"""
    print("\n[3/4] Testing Motors...")

    try:
        import serial

        ser = serial.Serial("/dev/serial0", 115200, timeout=1)

        input("  Press Enter to spin motors for 1 second...")

        # Forward
        ser.write(b'{"cmd":"motor","left":150,"right":150,"dur":1000}\n')
        print("  -> Motors should spin forward...")
        time.sleep(1.2)

        # Check if command was received
        ser.write(b'{"cmd":"ping"}\n')
        time.sleep(0.1)
        response = ser.readline().decode().strip()

        if response:
            answer = input("  Did both motors spin? (y/n): ").lower()
            if answer == "y":
                print("  [OK] Motors working")
                return True

        print("  [FAIL] Motor test failed")
        return False

    except Exception as e:
        print(f"  [FAIL] Motor error: {e}")
        return False


def test_encoders():
    """Test encoder reading"""
    print("\n[4/4] Testing Encoders...")

    try:
        import serial
        import json

        ser = serial.Serial("/dev/serial0", 115200, timeout=1)

        # Reset encoders
        ser.write(b'{"cmd":"reset_encoders"}\n')
        time.sleep(0.1)
        ser.readline()  # Clear response

        input("  Spin LEFT wheel by hand, then press Enter...")

        ser.write(b'{"cmd":"encoders"}\n')
        time.sleep(0.1)
        response = ser.readline().decode().strip()

        if response:
            data = json.loads(response)
            left = data.get("left", 0)
            right = data.get("right", 0)
            print(f"  Encoder counts - Left: {left}, Right: {right}")

            if abs(left) > 10:
                print("  [OK] Left encoder working")
            else:
                print("  [FAIL] Left encoder not responding")

        # Reset and test right
        ser.write(b'{"cmd":"reset_encoders"}\n')
        time.sleep(0.1)
        ser.readline()

        input("  Spin RIGHT wheel by hand, then press Enter...")

        ser.write(b'{"cmd":"encoders"}\n')
        time.sleep(0.1)
        response = ser.readline().decode().strip()

        if response:
            data = json.loads(response)
            left = data.get("left", 0)
            right = data.get("right", 0)
            print(f"  Encoder counts - Left: {left}, Right: {right}")

            if abs(right) > 10:
                print("  [OK] Right encoder working")
                return True
            else:
                print("  [FAIL] Right encoder not responding")

        return False

    except Exception as e:
        print(f"  [FAIL] Encoder error: {e}")
        return False


def main():
    print("=" * 50)
    print("REIP Robot Hardware Test")
    print("=" * 50)

    results = {
        "I2C/ToF": test_i2c(),
        "UART": test_uart(),
        "Motors": test_motors(),
        "Encoders": test_encoders(),
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
        print("All tests passed! Ready to run reip_main.py")
    else:
        print("Some tests failed. Fix issues before running REIP.")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
