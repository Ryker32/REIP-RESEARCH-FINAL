# REIP Distributed Validation System

True distributed multi-agent coordination with REIP (Resilient Election & Impeachment Policy).

## Architecture

```
        ┌──────────────────────┐
        │   Overhead Camera    │
        └──────────┬───────────┘
                   │ USB
                   
        ┌──────────────────────┐
        │         PC           │
        │  aruco_position_     │
        │  server.py           │
        │                      │
        │  "GPS Satellite"     │
        │  - Detects markers   │           ┌─────────────────────────┐
        │  - Sends each robot  │           │      visualizer.py      │
        │    ONLY its own      │           │   (monitoring only)     │
        │    position          │           └─────────────────────────┘
        └──────────┬───────────┘                       
                   │                                   │
          UDP 5003 │ (position: x,y,theta)                 │ UDP 5004
                   │                                   │ (peer broadcasts)
     ┌─────────────┼─────────────┬─────────────┐      │
                                                  │
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │
│ Robot 1 │  │ Robot 2 │  │ Robot 3 │  │ Robot 4 │   │
│         │  │         │  │         │  │         │   │
│reip_node│  │reip_node│  │reip_node│  │reip_node│   │
│  .py    │  │  .py    │  │  .py    │  │  .py    │   │
│         │─┼────────│─┼────────│─┼────────│───┘
│  REIP   │  │  REIP   │  │  REIP   │  │  REIP   │
│ (local) │  │ (local) │  │ (local) │  │ (local) │
└─────────┘  └─────────┘  └─────────┘  └─────────┘
                   ───── UDP 5004 ─────
                    (peer-to-peer state)
```

## What Makes This TRULY Distributed

1. **Each robot runs its own REIP instance** - decisions are made locally
2. **Robots communicate peer-to-peer** - no central coordinator
3. **Each robot only knows its own position** - camera acts as "GPS", not god
4. **Trust computed locally** - each robot assesses peers independently
5. **Elections are distributed** - votes exchanged, majority wins
6. **PC only provides localization** - like GPS satellites, not control

## Quick Start

### 1. PC Setup

```bash
pip install opencv-python opencv-contrib-python numpy pygame
```

### 2. Robot Setup (each Pi Zero)

```bash
# Copy files
scp robot/reip_node.py pi@clanker-X.local:~/

# Ensure dependencies (from earlier setup)
pip3 install adafruit-circuitpython-vl53l0x --break-system-packages
```

### 3. Print ArUco Markers

Go to https://chev.me/arucogen/
- Dictionary: 4x4 (50)
- IDs: 1, 2, 3, 4, 5 (one per robot)
- Size: 50mm

### 4. Run Experiment

**Terminal 1 - Position Server (PC):**
```bash
python3 aruco_position_server.py
# Press 'c' to calibrate arena
```

**Terminal 2 - Visualizer (PC):**
```bash
python3 visualizer.py
```

**Terminal 3 - Logger (PC):**
```bash
python3 logger.py my_experiment
```

**Each Robot (SSH):**
```bash
ssh pi@clanker-1.local
python3 reip_node.py 1

ssh pi@clanker-2.local
python3 reip_node.py 2
# ... etc
```

### 5. Inject Faults

**Terminal 4 (PC):**
```bash
python3 fault_inject.py

> spin 2        # Robot 2 starts spinning
> status        # Show active faults
> clear 2       # Robot 2 returns to normal
```

## Network Ports

| Port | Direction | Purpose |
|------|-----------|---------|
| 5003 | PC -> Robot | Position updates (each robot gets only its own) |
| 5004 | Robot ↔ Robot | Peer state broadcasts (position, trust, vote, coverage) |
| 5005 | PC -> Robot | Fault injection (testing only) |

## What Each Robot Broadcasts (UDP 5004)

```json
{
  "type": "peer_state",
  "robot_id": 1,
  "x": 500.0,
  "y": 300.0,
  "theta": 1.57,
  "state": "leader",
  "trust": 0.95,
  "vote": 1,
  "coverage_count": 42,
  "visited_cells": [[5,3], [5,4], [6,4], ...],
  "timestamp": 1234567890.123
}
```

## REIP Algorithm (per robot)

```
Every 100ms:
  1. Receive my position from camera
  2. Read ToF sensors
  3. Receive peer broadcasts
  4. Update trust scores for each peer
     - Spinning in place? Trust--
     - Stuck? Trust--
     - Moving normally? Trust++
  5. Every 2s: Run election
     - Vote for highest-trust robot (including self)
     - Count votes from peers
     - Majority = leader
  6. Check impeachment
     - If leader's trust < 0.3, don't vote for them
  7. Compute motor command
     - Avoid obstacles (ToF)
     - Head toward unexplored cells
  8. Broadcast my state to peers
```

## Fault Types

| Fault | Behavior | What REIP Should Do |
|-------|----------|---------------------|
| `spin` | Spin in circles | Detect rotation without translation -> trust decays |
| `stop` | Freeze in place | Detect no movement -> trust decays |
| `erratic` | Random movements | Detect inconsistent behavior -> trust decays |

## Expected Results

When fault injected:
1. **0-2s**: Faulty robot starts misbehaving
2. **2-5s**: Peers detect anomaly, trust decays
3. **5-10s**: Trust falls below threshold
4. **10-15s**: If faulty robot was leader -> impeachment votes -> new election
5. **15-20s**: Swarm continues with new leader, excludes faulty robot

## Log Analysis

Logs are in `logs/experiment_name.jsonl`. Each line is a JSON message.

```python
import json
import matplotlib.pyplot as plt

# Load logs
data = [json.loads(line) for line in open('logs/my_experiment.jsonl')]

# Filter to robot 1's broadcasts
r1 = [d for d in data if d.get('robot_id') == 1]

# Plot trust over time
times = [d['_experiment_time'] for d in r1]
trust = [d['trust'] for d in r1]
plt.plot(times, trust)
plt.xlabel('Time (s)')
plt.ylabel('Trust Score')
plt.title('Robot 1 Trust Over Time')
plt.show()
```

## Validation Metrics

1. **Coverage %** - How much of arena explored
2. **Time to detect fault** - When trust starts dropping
3. **Time to impeach** - When leader changes after fault
4. **Recovery time** - When coverage resumes after fault
5. **Resilience** - Coverage with faults vs without

## Troubleshooting

**No position received:**
- Check camera sees ArUco marker
- Check robot ID matches marker ID
- Check UDP 5003 not blocked

**Robots don't see each other:**
- Check all on same WiFi
- Check UDP 5004 not blocked
- Check broadcasts working: `sudo tcpdump -i wlan0 port 5004`

**Robot not moving:**
- Check Pico firmware running (LED blinking)
- Check UART connected
- Check ToF sensors responding

**Trust not decaying:**
- Need 5+ position samples to detect anomaly
- Check peer broadcasts arriving
- Increase TRUST_DECAY_RATE for faster response
