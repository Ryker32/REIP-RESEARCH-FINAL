# REIP Robot Code - Clanker-1

## Quick Start

### 1. Deploy Pico Firmware

First, flash the Pico with the motor controller firmware:

1. Hold BOOTSEL button on Pico and plug USB
2. Copy `main.py` from `../pico/` to the Pico drive
3. Unplug and reconnect Pico

Or use Thonny:
```bash
# Open Thonny, connect to Pico, open ../pico/main.py
# Save as main.py on the Pico
```

### 2. Copy Code to Pi Zero

**Option A: Manual copy**
```bash
# From your computer
scp reip_controller.py pi@clanker1.local:~/
scp test_hardware_quick.py pi@clanker1.local:~/
scp requirements.txt pi@clanker1.local:~/
```

**Option B: Using deploy script (Linux/Mac)**
```bash
chmod +x deploy.sh
./deploy.sh clanker1.local 0
```

### 3. Install Dependencies

On the Pi Zero:
```bash
pip3 install -r requirements.txt --break-system-packages
```

### 4. Test Hardware

```bash
python3 test_hardware_quick.py
```

Should show:
- 5 ToF sensors working
- Pico connection OK
- Motors spinning

### 5. Run Controller

```bash
python3 reip_controller.py --robot-id 0
```

## Hardware Pin Mapping

### Pico (Motor Controller)

| Function | GPIO Pin |
|----------|----------|
| Motor L IN1 | GP18 |
| Motor L IN2 | GP19 |
| Motor R IN1 | GP20 |
| Motor R IN2 | GP21 |
| Encoder L A | GP3 |
| Encoder L B | GP4 |
| Encoder R A | GP7 |
| Encoder R B | GP8 |
| UART TX | GP0 |
| UART RX | GP1 |

### Pi Zero (Main Controller)

| Function | GPIO Pin |
|----------|----------|
| I2C SDA | GPIO2 |
| I2C SCL | GPIO3 |
| UART TX | GPIO14 |
| UART RX | GPIO15 |

### ToF Sensors (via I2C Mux)

| Sensor | Mux Channel |
|--------|-------------|
| Front | 0 |
| Front-Left | 1 |
| Front-Right | 2 |
| Left | 3 |
| Right | 4 |

## UART Protocol

The Pi Zero communicates with the Pico using simple text commands:

| Command | Response | Description |
|---------|----------|-------------|
| `PING` | `PONG` | Test connection |
| `ENC` | `left,right` | Get encoder counts |
| `MOT,L,R` | `OK` | Set motor speeds (-100 to 100) |
| `STOP` | `OK` | Stop motors |
| `RST` | `OK` | Reset encoders |

Example:
```python
import serial
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ser.write(b'MOT,50,50\n')  # Drive forward at 50%
```

## Troubleshooting

### ToF sensors not detected
- Check I2C mux power (3.3V, GND)
- Check `i2cdetect -y 1` shows mux at 0x70
- Verify each sensor has unique mux channel

### Pico not responding
- Check UART wiring: Pi TX -> Pico RX (GP1), Pi RX <- Pico TX (GP0)
- Make sure Pico is running main.py (LED should blink)
- Try `screen /dev/serial0 115200` and type `PING`

### Motors not moving
- Check battery voltage (should be 5V to Pico VSYS)
- Check DRV8833 VM/VCC power
- Try higher speed: `MOT,80,80`

### Encoders stuck at 0
- Check encoder wiring to GP3,4,7,8
- Verify encoder power (5V from Pico)
- Spin wheel by hand and check with `ENC` command
