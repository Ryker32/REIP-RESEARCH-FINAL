#!/usr/bin/env python3
"""
VL53L0X ToF Sensor Driver with PCA9548A Mux Support
For REIP Robot - Pi Zero 2W
"""

import smbus2
import time

# ============================================================================
# PCA9548A I2C MUX
# ============================================================================


class PCA9548A:
    """I2C Multiplexer driver"""

    def __init__(self, bus, address=0x70):
        self.bus = bus
        self.address = address
        self.current_channel = None

    def select(self, channel: int):
        """Select channel 0-7 (or None to disable all)"""
        if channel == self.current_channel:
            return

        if channel is None:
            self.bus.write_byte(self.address, 0)
        else:
            self.bus.write_byte(self.address, 1 << channel)

        self.current_channel = channel
        time.sleep(0.001)


# ============================================================================
# VL53L0X DRIVER
# ============================================================================


class VL53L0X:
    """VL53L0X Time-of-Flight sensor driver"""

    # Default I2C address
    DEFAULT_ADDRESS = 0x29

    # Registers
    SYSRANGE_START = 0x00
    SYSTEM_SEQUENCE_CONFIG = 0x01
    SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
    SYSTEM_INTERRUPT_CLEAR = 0x0B
    RESULT_INTERRUPT_STATUS = 0x13
    RESULT_RANGE_STATUS = 0x14

    IDENTIFICATION_MODEL_ID = 0xC0
    IDENTIFICATION_REVISION_ID = 0xC2

    MSRC_CONFIG_CONTROL = 0x60
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44

    def __init__(self, bus, address=DEFAULT_ADDRESS, mux=None, mux_channel=None):
        self.bus = bus
        self.address = address
        self.mux = mux
        self.mux_channel = mux_channel
        self._initialized = False

    def _select_mux(self):
        """Select mux channel if using mux"""
        if self.mux and self.mux_channel is not None:
            self.mux.select(self.mux_channel)

    def _write_byte(self, reg, value):
        self._select_mux()
        self.bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg):
        self._select_mux()
        return self.bus.read_byte_data(self.address, reg)

    def _read_word(self, reg):
        self._select_mux()
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        return (high << 8) | low

    def init(self) -> bool:
        """Initialize sensor. Returns True on success."""
        try:
            self._select_mux()

            # Check model ID
            model_id = self._read_byte(self.IDENTIFICATION_MODEL_ID)
            if model_id != 0xEE:
                return False

            # Set 2.8V mode
            self._write_byte(0x89, 0x01)
            self._write_byte(0x88, 0x00)

            # Set standard ranging mode
            self._write_byte(self.SYSTEM_SEQUENCE_CONFIG, 0xFF)

            # Set default timing budget (~33ms)
            self._write_byte(self.SYSTEM_INTERMEASUREMENT_PERIOD, 0x04)

            # Configure GPIO interrupt
            self._write_byte(self.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)

            # Clear interrupt
            self._write_byte(self.SYSTEM_INTERRUPT_CLEAR, 0x01)

            self._initialized = True
            return True

        except Exception as e:
            print(f"VL53L0X init error: {e}")
            return False

    def start_continuous(self, period_ms=0):
        """Start continuous ranging mode"""
        if not self._initialized:
            return

        self._write_byte(self.SYSTEM_INTERMEASUREMENT_PERIOD, period_ms // 10)
        self._write_byte(self.SYSRANGE_START, 0x02)  # Continuous back-to-back

    def stop_continuous(self):
        """Stop continuous ranging"""
        self._write_byte(self.SYSRANGE_START, 0x01)  # Single shot (stops continuous)

    def read(self) -> int:
        """
        Read distance in mm.
        Returns 0 on error, 8190 if out of range.
        """
        if not self._initialized:
            return 0

        try:
            self._select_mux()

            # Single shot measurement
            self._write_byte(self.SYSRANGE_START, 0x01)

            # Wait for measurement complete (poll interrupt status)
            timeout = time.time() + 0.1  # 100ms timeout
            while time.time() < timeout:
                status = self._read_byte(self.RESULT_INTERRUPT_STATUS)
                if status & 0x07:
                    break
                time.sleep(0.005)
            else:
                return 0  # Timeout

            # Read range result
            data = []
            self._select_mux()
            for i in range(12):
                data.append(self.bus.read_byte_data(self.address, self.RESULT_RANGE_STATUS + i))

            # Clear interrupt
            self._write_byte(self.SYSTEM_INTERRUPT_CLEAR, 0x01)

            # Extract distance (bytes 10-11)
            distance = (data[10] << 8) | data[11]

            # Check for errors (byte 0, bits 3:0 = error code)
            range_status = (data[0] & 0x78) >> 3
            if range_status != 0x0B:  # 0x0B = valid measurement
                if distance > 2000:
                    return 2000  # Out of range

            return distance

        except Exception:
            return 0


# ============================================================================
# SENSOR ARRAY (5 ToF via mux)
# ============================================================================


class ToFArray:
    """Array of 5 VL53L0X sensors on PCA9548A mux"""

    CHANNELS = {
        "front": 0,
        "front_left": 1,
        "front_right": 2,
        "left": 3,
        "right": 4,
    }

    def __init__(self, i2c_bus=1, mux_address=0x70):
        self.bus = smbus2.SMBus(i2c_bus)
        self.mux = PCA9548A(self.bus, mux_address)
        self.sensors = {}

        for name, channel in self.CHANNELS.items():
            self.sensors[name] = VL53L0X(
                self.bus,
                mux=self.mux,
                mux_channel=channel,
            )

    def init(self) -> dict:
        """Initialize all sensors. Returns dict of {name: success}"""
        results = {}
        for name, sensor in self.sensors.items():
            success = sensor.init()
            results[name] = success
            status = "[OK]" if success else "[FAIL]"
            print(f"  {status} ToF '{name}' (CH{self.CHANNELS[name]})")
        return results

    def read_all(self) -> dict:
        """Read all sensors. Returns {name: distance_mm}"""
        readings = {}
        for name, sensor in self.sensors.items():
            readings[name] = sensor.read()
        return readings

    def read(self, name: str) -> int:
        """Read single sensor by name"""
        if name in self.sensors:
            return self.sensors[name].read()
        return 0


# ============================================================================
# TEST
# ============================================================================


if __name__ == "__main__":
    print("VL53L0X ToF Array Test")
    print("=" * 40)

    array = ToFArray()
    print("\nInitializing sensors...")
    array.init()

    print("\nReading sensors (Ctrl+C to stop)...")
    print()

    try:
        while True:
            readings = array.read_all()

            print(
                f"\rF:{readings['front']:4}  "
                f"FL:{readings['front_left']:4}  "
                f"FR:{readings['front_right']:4}  "
                f"L:{readings['left']:4}  "
                f"R:{readings['right']:4} mm",
                end="",
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nDone.")
