# Simulation vs Hardware Validation Checklist

**Goal**: Ensure simulation accurately represents hardware behavior before generating final paper figures.

## Critical Areas to Validate

### 1. Robot Movement & Physics

**Simulation (isef_experiments.py)**:
- Speed: `SPEED_PER_SEC = 500` pixels/sec (~robot body length per second)
- Time-based movement: `total_move = SPEED_PER_SEC * dt`
- Wall collision: `WALL_RADIUS = 75` pixels
- Stuck detection: `STUCK_TIME = 1.5s`, `STUCK_MOVE_EPS = 15` pixels

**Hardware (reip_node.py)**:
- Motor PWM control via Pico
- Physical speed depends on battery voltage, motor wear, surface friction
- ToF sensors detect walls at ~150mm
- Stuck detection: `elapsed > 1.5s` and `moved < 20mm`

**Validation Steps**:
- [ ] Measure hardware robot speed (mm/s) over 10 seconds
- [ ] Compare to sim speed: 500 pixels/sec = ~500mm/sec (if 1 pixel = 1mm)
- [ ] Test wall avoidance: Do hardware robots stop ~75mm from walls like sim?
- [ ] Test stuck detection: Block a hardware robot for 1.5s, does escape trigger?

**Known Issues to Check**:
- Sim uses perfect pathfinding (A*), hardware uses reactive wall avoidance
- Sim has no motor dead zones, hardware motors may stall below 25 PWM
- Sim has no battery voltage drop, hardware speed may decrease over time

---

### 2. Sensor Readings (ToF)

**Simulation**:
- ToF obstacles detected instantly when within range
- No sensor noise or false positives
- Perfect line-of-sight (no occlusions)

**Hardware**:
- VL53L0X ToF sensors: 30-2000mm range, ~20mm accuracy
- I2C bus contention (5 sensors per robot)
- Sensor update rate: ~8-10 Hz (after recent optimizations)
- May have false positives from reflections

**Validation Steps**:
- [ ] Check ToF update rate on hardware: Should be ~8-10 Hz
- [ ] Test obstacle detection: Place obstacle at 150mm, does robot detect it?
- [ ] Test false positives: Shiny surfaces, does robot see phantom obstacles?
- [ ] Compare ToF emergency trigger: Hardware should stop at ~150mm like sim

---

### 3. Coverage Detection

**Simulation**:
- Marks cell as visited when robot center enters cell
- `CELL_SIZE = 125mm`
- Center-cell only marking (no 3x3 brush)

**Hardware**:
- Same logic: `my_visited` tracks cells robot center is in
- Uses same `CELL_SIZE = 125mm`
- Position from ArUco server (camera-based)

**Validation Steps**:
- [ ] Run hardware trial, compare coverage % to sim with same starting positions
- [ ] Check cell marking: Does hardware mark cells correctly?
- [ ] Position accuracy: ArUco should be accurate to ~5mm (good enough for 125mm cells)

---

### 4. Trust Assessment (Three-Tier)

**Simulation & Hardware**:
- Same code: `assess_leader_command()` in `reip_node.py`
- Tier 1: Personal visited (weight 1.0)
- Tier 2: ToF obstacles (weight 1.0)
- Tier 3: Peer-reported (weight 0.3)
- Causality-aware: Only penalize if evidence predates assignment

**Validation Steps**:
- [ ] Inject bad leader on hardware, measure detection time
- [ ] Compare to sim: Should be similar (0.20-0.21s median)
- [ ] Test causality: Visit cell, then receive assignment to that cell — should NOT trigger false positive
- [ ] Test Tier 2: Command robot to wall, does ToF trigger suspicion?

---

### 5. Network Communication

**Simulation**:
- Perfect UDP delivery (no packet loss)
- Instant delivery (no latency)
- All robots receive all broadcasts

**Hardware**:
- Real UDP over WiFi (may have packet loss)
- Network latency: ~10-50ms typical
- Broadcast may not reach all robots if out of range

**Validation Steps**:
- [ ] Check packet loss: Monitor UDP broadcasts, count dropped packets
- [ ] Measure latency: Time from send to receive
- [ ] Test range: Do robots lose communication when far apart?
- [ ] Compare election timing: Hardware elections may be slower due to network delays

---

### 6. Fault Injection

**Simulation**:
- `bad_leader`: Assigns explored cells
- `freeze_leader`: Stops updating assignments
- `motor_fault`: `stop` or `spin` modes

**Hardware**:
- Same fault types via UDP fault injection
- Motor faults: Pico firmware can simulate `stop` or `spin`

**Validation Steps**:
- [ ] Test bad_leader on hardware: Does leader assign explored cells?
- [ ] Test freeze_leader: Does leader stop updating assignments?
- [ ] Test motor_fault: Does robot actually stop/spin?
- [ ] Compare detection times: Should match sim (within ~0.1s)

---

## Quick Validation Test

**Run this on both sim and hardware with identical starting positions:**

1. **Starting positions** (from `test_aruco_distances.py` output):
   ```json
   {
     "1": [x1, y1, theta1],
     "2": [x2, y2, theta2],
     ...
   }
   ```

2. **Run sim with fixed positions**:
   ```bash
   python test/isef_experiments.py --layout multiroom --trials 1 --fixed-positions positions.json
   ```

3. **Run hardware** (place robots at exact positions from JSON)

4. **Compare metrics**:
   - Final coverage: Should be within ~5%
   - Detection time: Should be within ~0.2s
   - Impeachment time: Should be within ~1s
   - Number of elections: Should be similar

---

## Known Differences (Acceptable)

These differences are expected and don't invalidate results:

1. **Pathfinding**: Sim uses A*, hardware uses reactive avoidance
   - **Impact**: Hardware may take longer paths, but coverage should be similar

2. **Motor dead zones**: Hardware motors stall below 25 PWM
   - **Impact**: Hardware may get stuck more often, but stuck detection handles it

3. **Network delays**: Hardware has real UDP latency
   - **Impact**: Elections may be slightly slower, but detection should still be sub-second

4. **Sensor noise**: Hardware ToF has false positives
   - **Impact**: May cause occasional false suspicion, but three-tier weights prevent false impeachments

---

## Red Flags (Need to Fix)

If you see these, fix before generating final figures:

1. **Coverage differs by >10%**: Sim and hardware should match within 5-10%
2. **Detection time differs by >0.5s**: Should be within ~0.2s
3. **Hardware robots crash/get stuck constantly**: Navigation bug
4. **Hardware elections never happen**: Network or trust bug
5. **Hardware false positives >1 per trial**: Trust threshold too sensitive

---

## Validation Command

After validating, run this to generate figures with confidence:

```bash
# 1. Validate sim matches hardware (run both, compare)
python test/isef_experiments.py --layout multiroom --trials 10

# 2. Generate final figures
python test/_generate_paper_figures.py
```
