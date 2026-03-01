# REIP Hardware Experiment Procedure

## Overview

You need **3 terminal windows** on your PC (all SSH'd into the laptop in the garage):
1. **Terminal 1** — Position Server (camera + ArUco)
2. **Terminal 2** — Robot Launcher (starts reip_node on all 5 Pis)
3. **Terminal 3** — Fault Injector (injects faults mid-experiment)

Plus one quick command before each run to snapshot starting positions.

---

## Pre-Experiment Setup (once)

### 1. Power on everything
- Plug in laptop in garage, camera pointed at arena
- Power on all 5 robots (batteries connected)
- Wait ~30 seconds for Pi Zeros to boot

### 2. Verify network connectivity
```bash
# From your PC, SSH into the laptop
ssh ryker@<laptop-ip>

# Verify all robots are reachable
ping -c 1 clanker1.local
ping -c 1 clanker2.local
ping -c 1 clanker3.local
ping -c 1 clanker4.local
ping -c 1 clanker5.local
```

### 3. Place robots
- 2–3 in Room A (left of wall), 2–3 in Room B (right of wall)
- At least 15 cm from walls, 20 cm apart from each other
- Orientation doesn't matter

---

## Per-Trial Procedure

### Step 1: Start Position Server (Terminal 1)
```bash
cd reip-sim-public
python pc/aruco_position_server.py
```
- Confirm HUD shows `HOMOGRAPHY` mode and all 5 robots + 4 corners detected
- **Leave this running for the entire session**

### Step 2: Snapshot Starting Positions
```bash
# Take a photo or use live camera snapshot
python test/test_aruco_distances.py <photo.png>
```
- Saves `*_aruco.json` with `sim_fixed_positions` for later sim validation
- Note: you can also press `s` in live mode to snapshot

### Step 3: Start All Robots (Terminal 2)

**For REIP trials:**
```bash
ssh pi@clanker1.local 'cd ~/reip && python3 reip_node.py 1 &' &
ssh pi@clanker2.local 'cd ~/reip && python3 reip_node.py 2 &' &
ssh pi@clanker3.local 'cd ~/reip && python3 reip_node.py 3 &' &
ssh pi@clanker4.local 'cd ~/reip && python3 reip_node.py 4 &' &
ssh pi@clanker5.local 'cd ~/reip && python3 reip_node.py 5 &' &
```

**For Raft baseline trials:**
```bash
ssh pi@clanker1.local 'cd ~/reip && python3 baselines/raft_node.py 1 &' &
ssh pi@clanker2.local 'cd ~/reip && python3 baselines/raft_node.py 2 &' &
ssh pi@clanker3.local 'cd ~/reip && python3 baselines/raft_node.py 3 &' &
ssh pi@clanker4.local 'cd ~/reip && python3 baselines/raft_node.py 4 &' &
ssh pi@clanker5.local 'cd ~/reip && python3 baselines/raft_node.py 5 &' &
```

**For Decentralized baseline trials:**
```bash
ssh pi@clanker1.local 'cd ~/reip && python3 reip_node.py 1 --decentralized &' &
ssh pi@clanker2.local 'cd ~/reip && python3 reip_node.py 2 --decentralized &' &
ssh pi@clanker3.local 'cd ~/reip && python3 reip_node.py 3 --decentralized &' &
ssh pi@clanker4.local 'cd ~/reip && python3 reip_node.py 4 --decentralized &' &
ssh pi@clanker5.local 'cd ~/reip && python3 reip_node.py 5 --decentralized &' &
```

### Step 4: Let Robots Explore (~10 seconds clean)
- Watch the arena — robots should begin exploring and spreading out
- The elected leader (check Terminal 1 output) coordinates frontier assignments
- **Timer starts now** — note the wall clock time

### Step 5: Inject Fault (Terminal 3) — at t ≈ 10s
```bash
cd reip-sim-public
python pc/fault_inject.py

# In the injector prompt:
> bad_leader 1
```
This tells Robot 1 (or whoever is leader) to start sending followers to already-explored cells.

**Key demo fault:** `bad_leader <leader_id>` — this is the Byzantine leadership fault that REIP detects.

### Step 6: Observe REIP Response
Watch for these events (should happen within seconds):
1. **Suspicion** — followers detect command divergence (~0.3–0.5s after fault)
2. **Trust decay** — trust scores drop below threshold
3. **Impeachment vote** — democratic vote to remove leader (~1–3s after fault)
4. **New election** — merit-based re-election of a new leader
5. **Recovery** — exploration resumes with new leader

### Step 7: Let Experiment Complete
- Total run time: **120 seconds** (matches simulation)
- After 120s, stop all robots:
```bash
ssh pi@clanker1.local 'pkill -f reip_node' &
ssh pi@clanker2.local 'pkill -f reip_node' &
ssh pi@clanker3.local 'pkill -f reip_node' &
ssh pi@clanker4.local 'pkill -f reip_node' &
ssh pi@clanker5.local 'pkill -f reip_node' &
```

### Step 8: Collect Logs
```bash
mkdir -p experiments/hardware/trial_N
for i in 1 2 3 4 5; do
    scp pi@clanker$i.local:~/reip/logs/*.jsonl experiments/hardware/trial_N/
done
```

---

## Experiment Matrix

Run each condition **at least 3 times** (more is better for stats):

| # | Controller | Fault | What You're Showing |
|---|-----------|-------|---------------------|
| 1 | REIP | None (clean) | Baseline coordination performance |
| 2 | REIP | bad_leader @ 10s | **KEY DEMO**: detects + impeaches + recovers |
| 3 | Raft | None (clean) | Consensus overhead baseline |
| 4 | Raft | bad_leader @ 10s | Raft CANNOT detect this — coverage collapses |
| 5 | Decentralized | None (clean) | No-leader baseline |
| 6 | Decentralized | bad_leader @ 10s | Immune (no leader) but lower coordination |

**Minimum trials: 6 conditions × 3 reps = 18 runs (~36 minutes of robot time)**

---

## Fault Injector Commands Reference

```
bad_leader <id>    — Byzantine: leader sends to explored cells (THE KEY FAULT)
spin <id>          — Motor fault: robot spins in circles
stop <id>          — Motor fault: robot freezes
erratic <id>       — Motor fault: random movement
clear <id>         — Remove fault, return to normal
status             — Show active faults
quit               — Exit injector (auto-clears all faults)
```

---

## Between Trials

1. **Stop all robots** (pkill command above)
2. **Clear faults**: `clear <id>` in fault injector, or quit and restart it
3. **Reposition robots** — spread them out again across both rooms
4. **Snapshot new positions** — `python test/test_aruco_distances.py`
5. **Start next trial**

---

## Sim Validation (after hardware runs)

For each hardware trial, replay the same starting positions in simulation:

```python
import json
from test.isef_experiments import ExperimentRunner

# Load the starting positions you snapped before the hardware run
with open('experiments/hardware/trial_1/starting_positions_aruco.json') as f:
    hw = json.load(f)
fixed = {int(k): tuple(v) for k, v in hw['sim_fixed_positions'].items()}

# Run sim with identical starting positions
runner = ExperimentRunner(output_dir="experiments/hw_validation_trial_1")
runner.set_layout('multiroom')
runner.start_robots('robot/reip_node.py', seed=42,
                    experiment_name='reip_clean_hw_val',
                    fixed_positions=fixed)
```

Compare sim vs hardware:
- Final coverage %
- Time to first suspicion
- Time to impeachment
- Recovery time

This is your **sim-to-real transfer** validation story.

---

## Quick Troubleshooting

| Problem | Fix |
|---------|-----|
| Robot not moving | Check battery, verify Pico LED is blinking (heartbeat) |
| Robot drives into wall | Check ArUco tag is visible to camera, verify homography |
| No fault detected | Verify you're injecting on the actual leader ID, not a follower |
| Robot not reachable | `ping clanker1.local`, check WiFi, reboot Pi Zero |
| Camera loses tracking | Check lighting, clean ArUco tags, ensure no glare |
| Position server crash | Restart `python pc/aruco_position_server.py` |
| Motors won't stop | Battery disconnect (emergency), or SSH in and `pkill python3` |

---

## Safety

- **ALWAYS have physical access to battery disconnects**
- The Pico firmware has a 500ms safety timeout — if the Pi Zero crashes, motors stop automatically
- Keep the fault injector terminal open — `quit` auto-clears all faults
- If anything goes wrong: **pull batteries first, debug second**
