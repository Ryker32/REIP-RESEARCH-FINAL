# Hardware Bug Analysis: REIP Node

**Based on code review of `robot/reip_node.py` and robot geometry (147mm × 128mm body)**

## Critical Bugs (Will Cause Failures)

### 1. **ToF Sensor Failure Silent Ignore**
**Location**: `read_tof_all()` lines 262-275

**Bug**: If a ToF sensor fails to initialize or throws an exception, it returns `-1` but the code continues. Later code uses `9999` as default, but `-1` values can cause issues.

**Impact**: 
- Robot thinks obstacle is at -1mm (treated as invalid, but logic may break)
- Missing sensor = blind spot = collisions

**Fix**: 
```python
if distances[name] == -1:
    distances[name] = 9999  # Treat as no obstacle
```

---

### 2. **UART Communication Failures Silent**
**Location**: `read_encoders()`, `set_motors()`, `stop()` lines 277-311

**Bug**: All UART operations are wrapped in `try/except: pass`. If Pico disconnects or UART fails, robot silently loses motor control.

**Impact**:
- Motors stop responding (robot freezes)
- Encoders stop updating (stuck detection fails)
- No error indication to user

**Fix**: Add error counting and timeout:
```python
self._uart_error_count = 0
# In each UART call:
except Exception as e:
    self._uart_error_count += 1
    if self._uart_error_count > 10:
        print(f"CRITICAL: UART failed {self._uart_error_count} times")
        # Enter safe mode
```

---

### 3. **Position Server Timeout Not Handled**
**Location**: `receive_position()` lines 533-582, `_check_stuck()` line 1692

**Bug**: If position server stops sending updates, `position_rx_mono` never updates. Code checks `now - self.position_rx_mono > 1.0` but robot continues operating with stale position.

**Impact**:
- Robot uses last known position (could be seconds old)
- Coverage marking stops (thinks it's not moving)
- Stuck detection disabled (line 1692 clears history if position stale)

**Fix**: Add position timeout check in main loop:
```python
if time.monotonic() - self.position_rx_mono > 2.0:
    print("WARNING: Position server timeout")
    # Stop motors, enter safe mode
```

---

### 4. **ToF Sensor MUX Race Condition**
**Location**: `_select_mux()` line 253-260, `read_tof_all()` line 268

**Bug**: MUX switching uses `time.sleep(0.001)` (1ms), but I2C operations may take longer. If sensor loop runs faster than I2C, sensors can read wrong channel.

**Impact**:
- Wrong ToF readings (front sensor reads left, etc.)
- False obstacle detections
- Robot avoids phantom obstacles

**Fix**: Add I2C ready check or increase sleep:
```python
time.sleep(0.005)  # 5ms for I2C to settle
```

---

### 5. **Motor PWM Rate Limiting Can Cause Oscillation**
**Location**: `_finalize_motor_command()` lines 1653-1667

**Bug**: `MAX_PWM_STEP = 20.0` limits PWM changes. If target PWM is far from current, robot takes many steps to reach it. During turns, this can cause oscillation.

**Impact**:
- Robot oscillates around target heading
- Slow response to emergency situations
- Wasted battery

**Fix**: Increase step size for emergency modes:
```python
if stop_reason in ["peer_emergency", "tof_emergency"]:
    MAX_PWM_STEP = 50.0  # Faster response
```

---

### 6. **Stuck Detection Requires Position Updates**
**Location**: `_check_stuck()` lines 1683-1704

**Bug**: Stuck detection clears history if `position_rx_mono == 0.0` or position is >1s stale (line 1692). If position server is slow, stuck detection never triggers.

**Impact**:
- Robot stuck against wall but stuck detection disabled
- Robot spins wheels forever
- Battery drain

**Fix**: Use encoder-based stuck detection as fallback:
```python
if now - self.position_rx_mono > 1.0:
    # Fallback: check encoders
    enc_l, enc_r = self.hw.read_encoders()
    if enc_l == self._last_enc_l and enc_r == self._last_enc_r:
        # Encoders not changing = stuck
        return True
```

---

### 7. **Peer Position Staleness Not Accounted in Collision Avoidance**
**Location**: `compute_motor_command()` lines 1985-2009

**Bug**: Peer positions can be up to 200ms stale (5Hz broadcast), but collision avoidance uses current `peer.x, peer.y` without accounting for movement.

**Impact**:
- Robot A thinks Robot B is at old position
- Robot A drives into Robot B's actual position
- Collisions despite avoidance code

**Fix**: Extrapolate peer position:
```python
# Estimate where peer is NOW based on last velocity
if hasattr(peer, 'last_vx'):
    dt = time.time() - peer.last_seen
    est_x = peer.x + peer.last_vx * dt
    est_y = peer.y + peer.last_vy * dt
else:
    est_x, est_y = peer.x, peer.y
```

---

### 8. **ToF Obstacle Detection Ignores Sensor Height**
**Location**: `update_tof_obstacles()` lines 1071-1102

**Bug**: ToF sensors are 13mm above ground, walls are 150mm tall. Code projects ToF reading directly to ground cell, but sensor might see wall top, not base.

**Impact**:
- Robot marks cells as obstacles when wall is above sensor
- False positives in pathfinding
- Robot avoids valid paths

**Fix**: Check if reading is reasonable for wall height:
```python
# If reading is very short (<50mm), might be seeing wall top
# Project to ground assuming 150mm wall height
if reading < 50:
    # Adjust projection angle
    pass
```

---

### 9. **ArUco Position Jitter Not Filtered**
**Location**: `receive_position()` lines 548-559

**Bug**: Position uses EMA filter (`POS_EMA = 0.5`), but heading (`theta`) is used directly without filtering. ArUco heading can jitter ±5°.

**Impact**:
- Robot heading oscillates
- Motor commands oscillate
- Wasted energy, poor navigation

**Fix**: Filter heading too:
```python
if not self._pos_initialized:
    self.theta = msg['theta']
else:
    # EMA filter for heading
    theta_diff = msg['theta'] - self.theta
    # Normalize to [-pi, pi]
    while theta_diff > math.pi: theta_diff -= 2*math.pi
    while theta_diff < -math.pi: theta_diff += 2*math.pi
    self.theta += POS_EMA * theta_diff
```

---

### 10. **Motor Dead Zone Not Handled for Low Speeds**
**Location**: `compute_motor_command()` lines 2231-2234

**Bug**: Code bumps speeds below `MIN_MOTOR_PWM` to `MIN_MOTOR_PWM`, but if both motors are below threshold, robot might not move at all (both motors at 25 PWM might not overcome friction).

**Impact**:
- Robot commands movement but doesn't move
- Stuck detection doesn't trigger (motors are "on")
- Robot appears frozen

**Fix**: Check if both motors are at minimum:
```python
if abs(left_speed) < MIN_MOTOR_PWM and abs(right_speed) < MIN_MOTOR_PWM:
    # Both too low - increase both proportionally
    scale = MIN_MOTOR_PWM / max(abs(left_speed), abs(right_speed), 1)
    left_speed *= scale
    right_speed *= scale
```

---

### 11. **Battery Voltage Drop Not Accounted**
**Location**: `BASE_SPEED = 80` (line 158)

**Bug**: `BASE_SPEED` is fixed at 80 PWM. As battery drains, same PWM produces less speed. Robot thinks it's moving at 500mm/s but actually moving slower.

**Impact**:
- Robot takes longer to reach targets
- Coverage estimates wrong
- Stuck detection thresholds wrong (expects 20mm in 1.5s, but only moves 10mm)

**Fix**: Adaptive speed based on encoder feedback:
```python
# Measure actual speed from encoders
actual_speed = (enc_l + enc_r) / 2 / dt
if actual_speed < expected_speed * 0.8:
    # Battery low - increase PWM or reduce expectations
    pass
```

---

### 12. **Network Packet Loss Not Handled**
**Location**: UDP broadcasts throughout

**Bug**: UDP has no ACK/retry. If leader assignment packet is lost, follower never receives it. Code has `ASSIGNMENT_STALE_TIME = 1.0`, but if packet is lost, follower uses old assignment.

**Impact**:
- Followers use stale assignments
- Coverage stalls
- Trust model doesn't trigger (assignment is "valid" but old)

**Fix**: Add sequence numbers to assignments:
```python
# Leader includes sequence number
# Follower checks if sequence increased
if assignment_seq <= self._last_assignment_seq:
    # Stale or duplicate - ignore
    return
```

---

### 13. **ToF Sensor Reading Timeout**
**Location**: `read_tof_all()` line 270

**Bug**: `self.tof_sensors[name].range` can hang if I2C bus is stuck. No timeout on sensor read.

**Impact**:
- Sensor loop blocks
- Control loop freezes
- Robot stops responding

**Fix**: Add timeout or non-blocking read:
```python
try:
    # Set I2C timeout
    distances[name] = self.tof_sensors[name].range
except TimeoutError:
    distances[name] = 9999
```

---

### 14. **Wall Slide Logic Can Trap Robot in Corner**
**Location**: `_wall_slide_heading()` and related wall avoidance

**Bug**: Wall slide logic prevents robot from turning into walls, but in corners, robot can get stuck sliding along two walls.

**Impact**:
- Robot stuck in corner
- Stuck detection might not trigger (robot is "moving" along wall)
- Coverage stalls

**Fix**: Detect corner condition:
```python
# If sliding along two walls, force escape
if self._wall_slide_active and time.monotonic() - self._wall_slide_start > 3.0:
    # Been sliding too long - force escape
    return escape_mode
```

---

### 15. **Election Vote Counting Race Condition**
**Location**: Election logic (multiple places)

**Bug**: Votes are collected via UDP broadcasts. If vote packet is lost, election can deadlock or elect wrong leader.

**Impact**:
- No leader elected (all think someone else is leader)
- Multiple leaders elected
- System deadlock

**Fix**: Add election timeout and retry:
```python
if time.monotonic() - self._election_start > 5.0:
    # Election timeout - restart
    self._start_election()
```

---

## Medium Priority Bugs

### 16. **ToF Sensor Initialization Order Matters**
**Location**: `_init_tof_sensors()` line 247

**Bug**: MUX channel selection happens before sensor init. If previous sensor didn't release I2C, new sensor init fails.

**Fix**: Add I2C bus reset between sensors.

---

### 17. **Encoder Overflow Not Handled**
**Location**: `read_encoders()` returns `int(l), int(r)`

**Bug**: Encoders can overflow (wrap around). Code doesn't detect wrap.

**Fix**: Track last encoder values and detect large jumps.

---

### 18. **Position Server Restart Not Handled**
**Location**: `receive_position()` 

**Bug**: If position server restarts, robot keeps old position. No reconnection logic.

**Fix**: Detect position timeout and attempt reconnection.

---

## Summary

**Most Critical** (Fix First):
1. UART failure silent ignore (#2)
2. Position server timeout (#3)
3. ToF sensor failure handling (#1)
4. Stuck detection with stale position (#6)

**Medium Priority**:
5. Peer position staleness (#7)
6. Motor dead zone (#10)
7. Network packet loss (#12)

**Low Priority** (Nice to Have):
8. Battery voltage adaptation (#11)
9. ArUco heading jitter (#9)
10. Wall slide corner trap (#14)
