# REIP Distributed System - PC Tools

**GPS Satellite Mode** - Each robot gets only its own position.

## Quick Start

### 1. Position Server (required)
```bash
python aruco_position_server.py
```
- Press 'c' to calibrate (click 3 corners of arena)
- Press 'v' to toggle video
- Press 'q' to quit

### 2. Visualizer (optional)
```bash
python visualizer.py
```
Real-time view of robots, coverage, trust, and leadership.

### 3. Logger (optional)
```bash
python logger.py experiment_name
```
Records to `logs/experiment_name.jsonl`

### 4. Fault Injector (testing)
```bash
python fault_inject.py
```
Commands:
- `spin 2` - Make robot 2 spin
- `stop 3` - Make robot 3 freeze
- `erratic 1` - Random movements
- `clear 2` - Remove fault
- `status` - Show active faults

## Network Ports

**Hardware mode** (default): all robots share ports, messages include robot_id for filtering.

| Port | From | To | Data |
|------|------|-----|------|
| 5100 | PC -> Robots | Position (x, y, theta) -- each robot filters by its ID |
| 5200 | Robots ↔ Robots | Peer state broadcast |
| 5300 | PC -> Robots | Fault injection -- each robot filters by target ID |

**Sim mode** (`--sim`): unique ports per robot for localhost testing.

| Port Range | Description |
|------|------|
| 5100+i | Position for robot i (5101-5105) |
| 5200+i | Peer comms for robot i (5201-5205), 5200 for simulator |
| 5300+i | Fault injection for robot i (5301-5305) |

## Requirements

```bash
pip install opencv-python opencv-contrib-python numpy pygame
```

## Camera Setup

1. Mount overhead, looking down at arena
2. Print ArUco markers (4x4 dict, IDs 1-5) at https://chev.me/arucogen/
3. Stick 50mm markers on robot tops
4. Run with calibration: `python aruco_position_server.py --calibrate`
