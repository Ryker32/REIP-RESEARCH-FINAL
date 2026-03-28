# Electronic Component Flow - REIP Robot System

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         EXTERNAL SYSTEMS                                │
├─────────────────────────────────────────────────────────────────────────┤
│  Camera (Laptop) → ArUco Detection → Position Server (UDP 5100)        │
│  Fault Injector (UDP 5300)                                              │
│  Peer Robots (UDP 5200)                                                 │
└─────────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI ZERO 2W                                  │
│                    (Main Controller - reip_node.py)                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────┐     │
│  │  NETWORK LAYER (UDP)                                          │     │
│  │  • Position Server (Port 5100) → Position updates            │     │
│  │  • Peer Broadcast (Port 5200) → Trust, maps, elections        │     │
│  │  • Fault Injection (Port 5300) → Start/stop, fault modes      │     │
│  └──────────────────────────────────────────────────────────────┘     │
│                              │                                           │
│                              ▼                                           │
│  ┌──────────────────────────────────────────────────────────────┐     │
│  │  REIP CONTROL LOGIC                                           │     │
│  │  • Trust Assessment (3-tier)                                  │     │
│  │  • Leader Election & Impeachment                               │     │
│  │  • Coverage Merging                                            │     │
│  │  • Frontier Detection                                          │     │
│  │  • Task Assignment                                             │     │
│  │  • Navigation (A* pathfinding)                                 │     │
│  └──────────────────────────────────────────────────────────────┘     │
│                              │                                           │
│                              ▼                                           │
│  ┌──────────────────────────────────────────────────────────────┐     │
│  │  HARDWARE INTERFACE                                           │     │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │     │
│  │  │ I2C Bus      │  │ UART         │  │ GPIO         │        │     │
│  │  │ (ToF Sensors)│  │ (Pico)       │  │ (Future use) │        │     │
│  │  └──────────────┘  └──────────────┘  └──────────────┘        │     │
│  └──────────────────────────────────────────────────────────────┘     │
│         │                    │                                           │
│         │                    │                                           │
└─────────┼────────────────────┼──────────────────────────────────────────┘
          │                    │
          │ I2C                │ UART (115200 baud)
          │ SDA: GPIO2         │ TX: GPIO14 → Pico RX (GP1)
          │ SCL: GPIO3         │ RX: GPIO15 ← Pico TX (GP0)
          │                    │
          ▼                    ▼
┌─────────────────┐  ┌──────────────────────────────────────────┐
│  I2C MUX       │  │  RASPBERRY PI PICO                      │
│  (TCA9548A)    │  │  (Motor Controller - main.py)           │
│  Address: 0x70 │  ├──────────────────────────────────────────┤
└─────────────────┘  │                                          │
         │           │  ┌────────────────────────────────────┐ │
         │           │  │  UART Handler                       │ │
         │           │  │  • Receives: PING, ENC, MOT, STOP  │ │
         │           │  │  • Responds: PONG, encoder counts  │ │
         │           │  └────────────────────────────────────┘ │
         │           │              │                          │
         │           │              ▼                          │
         │           │  ┌────────────────────────────────────┐ │
         │           │  │  Motor Control (PWM)                │ │
         │           │  │  • GP18, GP19 → Left Motor         │ │
         │           │  │  • GP20, GP21 → Right Motor        │ │
         │           │  └────────────────────────────────────┘ │
         │           │              │                          │
         │           │              ▼                          │
         │           │  ┌────────────────────────────────────┐ │
         │           │  │  Encoder Reading (IRQ)               │ │
         │           │  │  • GP3, GP4 → Left Encoder         │ │
         │           │  │  • GP7, GP8 → Right Encoder       │ │
         │           │  └────────────────────────────────────┘ │
         │           └──────────────────────────────────────────┘
         │                    │              │
         │                    │              │
         │                    ▼              ▼
         │           ┌─────────────┐  ┌─────────────┐
         │           │  DRV8833    │  │  Encoders   │
         │           │  Motor      │  │  (Quadrature│
         │           │  Driver     │  │   Optical) │
         │           └─────────────┘  └─────────────┘
         │                    │              │
         │                    ▼              │
         │           ┌─────────────┐         │
         │           │  N20 Motors │◀─────────┘
         │           │  (100:1)    │
         │           └─────────────┘
         │
         ▼
┌─────────────────────────────────┐
│  ToF Sensors (VL53L0X)          │
│  via I2C Mux Channels:         │
│  • Channel 0: Front            │
│  • Channel 1: Front-Right      │
│  • Channel 2: Front-Left       │
│  • Channel 3: Left              │
│  • Channel 4: Right             │
└─────────────────────────────────┘
```

## Detailed Data Flow

### 1. Position Updates (Camera → Pi Zero)

```
Camera (Laptop)
    │
    │ ArUco Detection (OpenCV)
    │ • Detects robot ArUco tags (IDs 1-5)
    │ • Detects corner tags (IDs 40-43)
    │ • Computes homography transform
    │
    ▼
Position Server (aruco_position_server.py)
    │
    │ UDP Broadcast (Port 5100)
    │ JSON: {"robot_id": 1, "x": 500, "y": 750, "theta": 0.5, "timestamp": ...}
    │
    ▼
Pi Zero: Network Thread
    │
    │ receive_position()
    │ • Updates self.x, self.y, self.theta
    │ • Applies EMA filter to theta (reduces jitter)
    │ • Sets position_rx_mono timestamp
    │
    ▼
REIP Control Logic
    │ • Uses position for coverage marking
    │ • Uses position for navigation
    │ • Uses position for peer state
```

### 2. ToF Sensor Reading (Sensors → Pi Zero)

```
ToF Sensor (VL53L0X) #1 (Front)
    │
    │ I2C Communication
    │ Address: 0x29 (default, shared bus)
    │
    ▼
I2C Mux (TCA9548A)
    │
    │ Channel Selection (via SMBus)
    │ Pi Zero writes: bus.write_byte(0x70, 1 << channel)
    │ • Channel 0 → Front sensor
    │ • Channel 1 → Front-Right
    │ • Channel 2 → Front-Left
    │ • Channel 3 → Left
    │ • Channel 4 → Right
    │
    │ Wait 5ms (I2C settling time)
    │
    ▼
Pi Zero: Sensor Thread (sensor_loop)
    │
    │ read_tof_all()
    │ • Selects mux channel
    │ • Reads distance: tof.range (mm)
    │ • Returns dict: {"front": 150, "left": 9999, ...}
    │ • Error handling: returns 9999 on failure
    │
    ▼
REIP Control Logic
    │ • update_tof_obstacles()
    │   - Converts ToF readings to obstacle cells
    │   - Stores in tof_obstacles set
    │   - Expires after 2 seconds
    │ • compute_motor_command()
    │   - Uses ToF for wall avoidance
    │   - Emergency stop if < CRITICAL_DISTANCE (100mm)
    │   - Speed taper if < AVOID_DISTANCE (200mm)
    │ • assess_leader_command()
    │   - Tier 2 trust: checks if target cell has ToF obstacle
```

### 3. Motor Control (Pi Zero → Pico → Motors)

```
REIP Control Logic
    │
    │ compute_motor_command()
    │ • Calculates left/right PWM values (-100 to 100)
    │ • Accounts for: navigation, wall avoidance, stuck detection
    │
    ▼
Pi Zero: Hardware.set_motors(left, right)
    │
    │ UART Serial Communication (115200 baud)
    │ Command: f"MOT,{left:.1f},{right:.1f}\n"
    │ • TX: GPIO14 → Pico RX (GP1)
    │ • RX: GPIO15 ← Pico TX (GP0)
    │
    │ Thread-safe: Uses _uart_lock
    │ Error handling: Tracks _uart_error_count
    │
    ▼
Pico: UART Handler (main.py)
    │
    │ process_text_command() or process_json_command()
    │ • Parses "MOT,80.0,80.0"
    │ • Validates range (-100 to 100)
    │
    ▼
Pico: set_motor(pwms, speed_l, speed_r)
    │
    │ PWM Generation (1000 Hz)
    │ • Left: GP18 (IN1), GP19 (IN2)
    │ • Right: GP20 (IN1), GP21 (IN2)
    │ • Duty cycle: speed * 655.35 (0-65535 range)
    │ • Direction: IN1 high = forward, IN2 high = reverse
    │
    ▼
DRV8833 Motor Driver
    │
    │ H-Bridge Control
    │ • Converts 3.3V PWM to motor voltage (5V from battery)
    │ • Provides current protection
    │
    ▼
N20 Motors (100:1 gear ratio)
    │
    │ Physical Motion
    │ • ~500mm/s max speed
    │ • Differential drive steering
```

### 4. Encoder Feedback (Encoders → Pico → Pi Zero)

```
N20 Motor Shaft
    │
    │ Mechanical Coupling
    │
    ▼
Optical Encoder (Quadrature)
    │
    │ Ch.A and Ch.B Signals (90° phase)
    │ • Left: GP3 (A), GP4 (B)
    │ • Right: GP7 (A), GP8 (B)
    │
    ▼
Pico: Encoder IRQ Handlers
    │
    │ Interrupt on Rising Edge
    │ • Tracks quadrature state
    │ • Increments/decrements count
    │ • left_count, right_count (global)
    │
    │ (Encoders read continuously, counts stored in memory)
    │
    ▼
Pi Zero: Hardware.read_encoders()
    │
    │ UART Request
    │ Command: "ENC\n"
    │
    ▼
Pico: UART Response
    │
    │ Response: f"{left_count},{right_count}\n"
    │
    ▼
Pi Zero: Control Logic
    │
    │ Uses encoders for:
    │ • Stuck detection (if position stale)
    │ • Movement verification
    │ • Debugging motor behavior
```

### 5. Peer Communication (Robot ↔ Robot)

```
Robot 1: REIP Control Logic
    │
    │ network_loop() (every 200ms)
    │ • Builds peer state message:
    │   - Position (x, y, theta)
    │   - Trust scores
    │   - Coverage map digest
    │   - Election votes
    │   - Leader assignments
    │
    │ UDP Broadcast (Port 5200)
    │ • Target: 192.168.20.255 (broadcast)
    │ • JSON: {"robot_id": 1, "x": 500, "trust_in_leader": 0.8, ...}
    │
    ▼
WiFi Network (802.11)
    │
    ▼
Robot 2: Network Thread
    │
    │ receive_peer_state()
    │ • Parses JSON message
    │ • Updates PeerInfo for robot_id
    │ • Applies velocity extrapolation (accounts for 200ms delay)
    │
    ▼
REIP Control Logic
    │ • Uses peer reports for:
    │   - Tier 3 trust assessment (weight 0.3)
    │   - Coverage merging
    │   - Election coordination
    │   - Emergency avoidance
```

### 6. Fault Injection (Laptop → Robots)

```
Fault Injector (Laptop)
    │
    │ UDP Broadcast (Port 5300)
    │ JSON: {"type": "fault_inject", "robot_id": 1, "fault": "bad_leader", ...}
    │
    ▼
WiFi Network
    │
    ▼
All Robots: Network Thread
    │
    │ receive_fault_injection()
    │ • Parses fault command
    │ • Sets injected_fault flag
    │ • Handles: "start", "stop", "bad_leader", "freeze_leader"
    │
    ▼
REIP Control Logic
    │ • "bad_leader": Leader sends wrong assignments
    │ • "freeze_leader": Leader sends stale assignments
    │ • "start": Enables motors (trial_started = True)
    │ • "stop": Disables motors
```

## Communication Protocols

### UART Protocol (Pi Zero ↔ Pico)

| Command | Direction | Format | Response | Description |
|---------|-----------|--------|----------|-------------|
| `PING` | Pi → Pico | Text | `PONG` | Connection test |
| `ENC` | Pi → Pico | Text | `left,right` | Get encoder counts |
| `MOT,L,R` | Pi → Pico | Text | `OK` | Set motor speeds (-100 to 100) |
| `STOP` | Pi → Pico | Text | `OK` | Emergency stop |
| `RST` | Pi → Pico | Text | `OK` | Reset encoder counts |

**Example:**
```python
# Pi Zero sends:
uart.write(b'MOT,80.0,80.0\n')

# Pico receives, parses, sets PWM:
set_motor(pwms, 80.0, 80.0)

# Pico responds:
uart.write(b'OK\n')
```

### I2C Protocol (Pi Zero ↔ ToF Sensors)

**Mux Selection:**
```python
# Select channel 0 (Front sensor)
bus.write_byte(0x70, 1 << 0)  # Write 0x01 to mux
time.sleep(0.005)  # Wait 5ms for I2C to settle

# Read from sensor (default address 0x29)
tof = adafruit_vl53l0x.VL53L0X(i2c)
distance = tof.range  # Returns mm (0-2000, or 9999 on error)
```

**Channel Mapping:**
- Channel 0: Front sensor
- Channel 1: Front-Right
- Channel 2: Front-Left  
- Channel 3: Left
- Channel 4: Right

### UDP Protocol (Network Communication)

**Position Updates (Port 5100):**
```json
{
  "robot_id": 1,
  "x": 500.0,
  "y": 750.0,
  "theta": 0.5,
  "timestamp": 1234567890.123
}
```

**Peer State (Port 5200):**
```json
{
  "robot_id": 1,
  "x": 500.0,
  "y": 750.0,
  "theta": 0.5,
  "state": "leader",
  "trust_in_leader": 0.8,
  "suspicion_of_leader": 0.2,
  "current_leader": 1,
  "coverage_count": 45,
  "visited_cells": [[10, 5], [11, 5], ...],
  "assigned_target": [1200.0, 800.0],
  "navigation_target": [1150.0, 750.0],
  "seq": 123
}
```

**Fault Injection (Port 5300):**
```json
{
  "type": "fault_inject",
  "robot_id": 1,
  "fault": "bad_leader",
  "timestamp": 1234567890.123
}
```

## Timing Characteristics

| Operation | Frequency | Latency | Notes |
|-----------|-----------|---------|-------|
| Position updates | 30 Hz | ~33ms | Camera → UDP → Pi Zero |
| ToF sensor read | 10 Hz | ~100ms | All 5 sensors, 5ms mux switch each |
| Motor command | 10 Hz | ~100ms | Pi Zero → Pico → Motors |
| Encoder read | 10 Hz | ~20ms | UART request/response |
| Peer broadcast | 5 Hz | ~200ms | Network delay + processing |
| Control loop | 10 Hz | ~100ms | Main REIP logic |

## Power Flow

```
Battery (5V, ~2000mAh)
    │
    ├─→ Pi Zero 2W (VSYS, 5V)
    │   • Powers CPU, WiFi, GPIO
    │   • ~500mA typical, ~1A peak
    │
    ├─→ Pico (VSYS, 5V)
    │   • Powers CPU, GPIO, PWM
    │   • ~100mA typical, ~200mA peak
    │
    ├─→ DRV8833 (VM, 5V)
    │   • Powers motor H-bridge
    │   • ~500mA per motor (1A total peak)
    │
    ├─→ ToF Sensors (3.3V via regulator)
    │   • 5 sensors × ~20mA = ~100mA
    │
    └─→ Encoders (5V from Pico)
        • 2 encoders × ~10mA = ~20mA

Total: ~1.5A typical, ~2.5A peak
```

## Error Handling & Safety

### UART Communication
- **Timeout**: 100ms per command
- **Error counting**: Tracks failures, warns after 10 consecutive errors
- **Auto-stop**: Pico stops motors after 500ms with no commands

### ToF Sensors
- **Error value**: Returns 9999 on failure (not -1)
- **Mux timing**: 5ms delay after channel switch (I2C stability)
- **Range check**: Ignores readings > 200mm (TOF_RANGE)

### Position Updates
- **Stale detection**: Motors disabled if position > 2.0s old
- **EMA filtering**: Reduces ArUco jitter in heading
- **Encoder fallback**: Uses encoders for stuck detection if position stale

### Motor Control
- **Dead zone**: Scales up if both motors < MIN_MOTOR_PWM (25)
- **Rate limiting**: MAX_PWM_STEP prevents sudden changes
- **Emergency stop**: Immediate stop on critical ToF reading (< 100mm)

## Summary

The system uses a **hierarchical architecture**:

1. **Laptop**: Position server (camera → ArUco → UDP)
2. **Pi Zero**: Main controller (REIP logic, network, I2C, UART)
3. **I2C Mux**: Routes I2C to 5 ToF sensors
4. **Pico**: Motor controller (PWM, encoders, UART)
5. **DRV8833**: Motor driver (H-bridge, current protection)
6. **Motors/Encoders**: Physical actuators and feedback

All communication is **asynchronous** with appropriate timeouts and error handling for robust operation.
