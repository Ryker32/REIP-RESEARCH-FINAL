# Single Robot Test Checklist

## Prerequisites

[x] **Code is ready** - All recent bug fixes are in place:
- ToF sensor error handling (returns 9999)
- UART error counting and warnings
- Position timeout handling (2.0s threshold)
- Encoder-based stuck detection fallback
- ArUco heading filtering
- Motor dead zone protection
- Peer position extrapolation
- Election timeout handling

## Quick Test Setup

### 1. Start Position Server (on laptop/PC)

```bash
python pc/aruco_position_server.py
```

**Check**: Should see camera feed and position updates for robot's ArUco tag.

### 2. Start Robot Node (on Pi Zero)

**Option A: Hardware mode (default)**
```bash
python3 robot/reip_node.py 1
```

**Option B: Decentralized mode (single robot, no leader needed)**
```bash
python3 robot/reip_node.py 1 --decentralized
```

**Option C: Sim mode (if testing on PC without hardware)**
```bash
python robot/reip_node.py 1 --sim
```

### 3. Verify Startup

You should see:
```
=== REIP Node - Robot 1 [HARDWARE (shared ports)] ===
Trust Model: Three-tier confidence-weighted
  Personal: 1.0, ToF: 1.0, Peer: 0.3
  Suspicion threshold: 1.5, Recovery: 0.1

Initializing hardware...
  Hardware: SIMULATED  (or actual hardware init messages)
  Pico: OK
Running... Press Ctrl+C to stop

[R1] Waiting for 'start' command...
```

### 4. Start Trial

Send start command via fault injector (on laptop/PC):

```bash
python -c "
import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = json.dumps({
    'type': 'fault_inject',
    'robot_id': 1,  # or 0 for all
    'fault': 'start',
    'timestamp': time.time()
}).encode()
sock.sendto(msg, ('192.168.20.255', 5300))  # or '127.0.0.1' for sim
print('Start command sent')
"
```

Or use a simple Python script:

```python
# start_robot.py
import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = json.dumps({
    'type': 'fault_inject',
    'robot_id': 1,
    'fault': 'start',
    'timestamp': time.time()
}).encode()
sock.sendto(msg, ('192.168.20.255', 5300))  # Change IP for sim mode
print('Start command sent')
```

### 5. Monitor Status

Every 2 seconds, you should see:
```
[R1] pos=(500,750) state=leader leader=1 trust=1.00 susp=0.00 cov=5.2%/5.2% mot=(80,80) tgt=(1200,800)
```

**Key things to check:**
- [x] Position updates (x, y values changing)
- [x] Motors engaging (mot values non-zero after start)
- [x] Coverage increasing (cov percentage)
- [x] Target assigned (tgt not None)
- [x] No crashes or errors

## Troubleshooting

### "Waiting for 'start' command..."
- **Fix**: Send start command via fault injector (step 4)

### "no_position" or "stale_position"
- **Fix**: Check position server is running and detecting ArUco tag
- **Fix**: Verify UDP 5100 is not blocked by firewall
- **Fix**: Check robot's ArUco tag is visible to camera

### Motors not moving
- **Fix**: Check Pico is connected and responding (should see "Pico: OK" at startup)
- **Fix**: Verify UART connection between Pi Zero and Pico
- **Fix**: Check motor wiring and DRV8833 connections

### ToF sensors showing 9999
- **Fix**: Check I2C connections (SDA/SCL)
- **Fix**: Verify ToF sensors are powered
- **Fix**: Check mux (TCA9548A) connections

### Robot stuck/not moving
- **Fix**: Check stuck detection is working (should see escape behavior)
- **Fix**: Verify wall avoidance is active (check ToF readings)
- **Fix**: Check if robot is physically stuck (motors on but not moving)

## Expected Behavior

### Single Robot (Decentralized Mode)
- Robot elects itself as leader
- Uses local frontier detection (get_my_frontier)
- Explores arena autonomously
- Avoids walls and obstacles
- Marks coverage as it moves

### Single Robot (Normal Mode)
- Robot elects itself as leader (only candidate)
- Computes task assignments (only for itself)
- Same exploration behavior as decentralized

## Log Files

Logs are saved to: `logs/robot_<id>_<timestamp>.json`

Each line contains:
- Position (x, y, theta)
- Trust state
- Navigation targets
- Motor commands
- Coverage counts

## Next Steps After Single Robot Works

1. Test with 2 robots (verify peer communication)
2. Test with 3+ robots (verify leader election)
3. Test fault injection (bad_leader, freeze_leader)
4. Run full experiment matrix
