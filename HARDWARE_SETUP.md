git # REIP Hardware Setup & Deployment Guide

## Architecture

```
Laptop (garage)                    Pi Zero 2W (x5)              Pi Pico (x5)
┌─────────────────┐               ┌──────────────┐             ┌──────────────┐
│ aruco_position   │──UDP 5100── │ reip_node.py  │──UART──── │ main.py      │
│ _server.py       │              │ (robot brain) │             │ (motors +    │
│                  │              │               │─UART────  │  encoders)   │
│ Camera -> ArUco   │              │ Trust model   │             │              │
│ Homography -> mm  │              │ Map merging   │             │ DRV8833 PWM  │
│ Position broadcast│             │ Frontier nav  │             │ Encoder IRQs │
└─────────────────┘               └──────────────┘             └──────────────┘
      │                                  
      │            UDP 5200              │
      │         (peer broadcast)         │
      │         Robot ↔ Robot            │
      │                                  │
      └──UDP 5300 (fault injection)──────┘
```

## Hardware Checklist

- [x] 5* Pi Pico -- flashed with MicroPython + `pico/main.py`
- [x] 5* Pi Zero 2W -- set up as clanker1--clanker5, `robot/reip_node.py` deployed
- [x] Laptop -- OpenCV + NumPy installed, `pc/aruco_position_server.py` ready
- [x] 4* Corner ArUco tags (IDs 40-43) on blue tape border
- [x] 5* Robot ArUco tags (IDs 1-5) on top of each robot
- [x] Arena: 2000*1500mm black surface with blue tape border
- [x] Interior wall: x=1000mm, 1200mm long (y=0->1200), 300mm passage at top

## Corner ArUco Tag Offsets (measured)

Tags sit on the outer blue-tape border, offset from the playable arena in X only:

| Tag | ID | X (mm) | Y (mm) | Notes |
|-----|-----|--------|--------|-------|
| BL  | 40  | -115   | 0      | 11.5 cm left of origin |
| BR  | 41  | 2110   | 0      | 211 cm from origin (11 cm past right edge) |
| TR  | 42  | 2110   | 1500   | 11 cm past right edge, level with top |
| TL  | 43  | -113   | 1500   | 11.3 cm left of origin, level with top |

These offsets are configured in:
- `pc/aruco_position_server.py` -> `dst_pts` array (line ~151)
- `test/test_aruco_distances.py` -> `CORNER_ARENA` dict

## Pi Pico Firmware

Same `pico/main.py` on all 5 Picos. Key config:
- `PWM_FREQ = 1000`
- `INVERT_LEFT = True`, `INVERT_RIGHT = False`
- UART: 115200 baud, GP0 (TX) -> Pi Zero RX, GP1 (RX) <- Pi Zero TX
- Safety: motors auto-stop after 500ms with no commands

### Flashing a Pico
1. Hold BOOTSEL + plug USB -> `RPI-RP2` drive appears
2. Drag `RPI_PICO-*.uf2` onto drive -> reboots into MicroPython
3. Open Thonny -> File -> Save As -> Raspberry Pi Pico -> save as `main.py`

## Running Experiments

### On the laptop (in garage, next to arena):

```bash
# Start the position server (camera + ArUco -> UDP broadcasts)
python pc/aruco_position_server.py
```

### From your PC (SSH into laptop):

```powershell
ssh ryker@<laptop-ip>
cd reip-sim-public

# Snap starting positions before a run
python test/test_aruco_distances.py <photo.png>
# -> saves *_aruco.json with sim_fixed_positions

# Start robots (on each Pi Zero):
ssh pi@clanker1.local 'python3 ~/reip/reip_node.py 1'
ssh pi@clanker2.local 'python3 ~/reip/reip_node.py 2'
# ... etc
```

### Hardware -> Sim Validation

Capture hardware starting positions, then replay in sim:

```python
import json
from test.isef_experiments import ExperimentRunner

# Load hardware positions from ArUco snapshot
with open('snapshot_aruco.json') as f:
    hw = json.load(f)
fixed = {int(k): tuple(v) for k, v in hw['sim_fixed_positions'].items()}

# Run sim with identical starting positions
runner = ExperimentRunner(output_dir="experiments/hw_validation")
runner.set_layout('multiroom')
runner.start_robots('robot/reip_node.py', seed=42,
                    experiment_name='hw_validate',
                    fixed_positions=fixed)
```

## Network Ports

| Port | Direction | Data |
|------|-----------|------|
| 5100 | Laptop -> Robots | Position (x, y, theta) per robot |
| 5200 | Robot ↔ Robot | Peer state broadcast (maps, trust, elections) |
| 5300 | Laptop -> Robots | Fault injection commands |

## Robot Placement Tips

- Spread across both rooms (2-3 per room)
- Keep ~15cm from walls (ToF sensor range)
- Keep ~20cm between robots (ArUco visibility)
- Exact positions don't matter -- system reads from camera in real-time
- Note starting positions for sim validation (use `test_aruco_distances.py`)

## Troubleshooting

- **Pico "Back-end not ready" in Thonny**: Check Device Manager for correct COM port (Tools -> Options -> Interpreter)
- **No COM port**: Wrong USB cable (charge-only) or MicroPython not installed
- **Homography off**: Re-measure corner tag X offsets and update `CORNER_ARENA` / `dst_pts`
- **Robot not responding**: Check `ping clanker1.local`, verify WiFi connection
- **Motors reversed**: Flip `INVERT_LEFT`/`INVERT_RIGHT` in `pico/main.py` for that robot
