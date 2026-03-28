# REIP Framework: In-Depth Technical Explanation

## Table of Contents
1. [System Architecture Overview](#system-architecture-overview)
2. [Information Flow Through the System](#information-flow-through-the-system)
3. [Core Components and Functions](#core-components-and-functions)
4. [Process Flows](#process-flows)
5. [Data Structures](#data-structures)
6. [Threading Model](#threading-model)

---

## System Architecture Overview

REIP (Resilient Election and Impeachment Policy) is a distributed governance framework for multi-robot exploration. Each robot runs an identical `REIPNode` instance that:

- **Verifies leader commands before execution** (proactive trust model)
- **Detects compromised leaders** using a three-tier confidence-weighted trust system
- **Democratically impeaches and replaces** faulty leaders through distributed voting
- **Operates without centralized oversight** or ground-truth oracle access

### Key Design Principles

1. **Proactive Verification**: Unlike reactive systems that detect faults after performance degradation, REIP verifies commands *before* execution using local knowledge.
2. **Confidence-Weighted Evidence**: Three tiers of evidence with different reliability weights:
   - **Tier 1** (Weight 1.0): Personal visit history - ground truth
   - **Tier 2** (Weight 1.0): ToF sensor readings - physically verifiable
   - **Tier 3** (Weight 0.3): Peer-reported coverage - may be stale
3. **Causality-Aware Checking**: Accounts for network delays to prevent false positives from concurrent exploration.
4. **Distributed Consensus**: Each robot independently assesses trust and votes, with no central authority.

---

## Information Flow Through the System

### High-Level Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    EXTERNAL INPUTS                           │
├─────────────────────────────────────────────────────────────┤
│ 1. Position Server (ArUco Camera)                           │
│    -> receive_position()                                     │
│    -> Updates: self.x, self.y, self.theta                   │
│    -> Marks: self.my_visited[cell] = timestamp              │
│                                                              │
│ 2. ToF Sensors (Hardware)                                    │
│    -> sensor_loop() -> hw.read_tof_all()                     │
│    -> update_tof_obstacles()                                 │
│    -> Updates: self.tof_obstacles (set of cells)            │
│                                                              │
│ 3. Peer State Broadcasts (UDP)                              │
│    -> receive_peer_states() -> update_peer()                  │
│    -> Updates: self.peers[peer_id] (position, trust, etc.) │
│    -> Merges: self.known_visited (union of all visited)    │
│                                                              │
│ 4. Fault Injection (UDP)                                     │
│    -> receive_fault_injection()                              │
│    -> Sets: bad_leader_mode, oscillate_leader_mode, etc.   │
└─────────────────────────────────────────────────────────────┘
                            v
┌─────────────────────────────────────────────────────────────┐
│                    INTERNAL PROCESSING                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  CONTROL LOOP (10 Hz)                                        │
│  ├─-> check_impeachment() [if trust < 0.3]                  │
│  ├─-> run_election() [every 2s, or 0.5s at startup]          │
│  │   ├─-> Builds candidate list (trust > 0.5)              │
│  │   ├─-> Counts votes from peers                           │
│  │   └─-> Elects new leader if changed                       │
│  │                                                           │
│  └─-> compute_motor_command() [if trial_started]           │
│      ├─-> assess_leader_command() [if follower]             │
│      │   ├─-> THREE-TIER CHECK                               │
│      │   │   ├─-> Tier 1: cell in self.my_visited?         │
│      │   │   ├─-> Tier 2: cell in self.tof_obstacles?      │
│      │   │   └─-> Tier 3: cell in self.known_visited?      │
│      │   │                                                   │
│      │   ├─-> MPC DIRECTION CHECK                             │
│      │   │   └─-> Command direction vs. nearest frontier    │
│      │   │                                                   │
│      │   └─-> Updates: suspicion_of_leader, trust_in_leader  │
│      │                                                       │
│      ├─-> [If Leader] compute_task_assignments()            │
│      │   ├─-> Finds frontiers (unexplored cells)             │
│      │   ├─-> Greedy nearest-frontier assignment            │
│      │   └─-> Returns: {robot_id: (x, y) target}            │
│      │                                                       │
│      ├─-> [If Follower] Uses leader_assigned_target          │
│      │   └─-> OR get_my_frontier() if no assignment         │
│      │                                                       │
│      └─-> Navigation logic (A*, wall avoidance, etc.)       │
│                                                              │
│  NETWORK LOOP (100 Hz polling)                               │
│  ├─-> receive_position() [UDP port 5100]                    │
│  ├─-> receive_peer_states() [UDP port 5200]                 │
│  ├─-> receive_fault_injection() [UDP port 5300]             │
│  └─-> broadcast_state() [every 0.2s = 5 Hz]                 │
│      └─-> Includes: position, trust, vote, assignments      │
│                                                              │
│  SENSOR LOOP (~8-10 Hz)                                      │
│  └─-> hw.read_tof_all() -> update_tof_obstacles()           │
└─────────────────────────────────────────────────────────────┘
                            v
┌─────────────────────────────────────────────────────────────┐
│                    OUTPUTS                                   │
├─────────────────────────────────────────────────────────────┤
│ 1. Motor Commands (UART to Pico)                            │
│    -> hw.set_motors(left, right)                            │
│                                                              │
│ 2. State Broadcasts (UDP)                                   │
│    -> broadcast_state() -> All peers                          │
│                                                              │
│ 3. Logging (File)                                            │
│    -> log_state() -> JSON lines for visualization            │
└─────────────────────────────────────────────────────────────┘
```

---

## Core Components and Functions

### 1. Position and Localization

#### `receive_position()` (Lines 553-612)
**Purpose**: Receives ArUco-based position updates from camera server.

**Information Flow**:
1. Listens on UDP port 5100 for position messages
2. Extracts `x`, `y`, `theta` from JSON message
3. Applies EMA filter (alpha=0.5) to reduce jitter:
   ```python
   self.x += 0.5 * (raw_x - self.x)
   self.y += 0.5 * (raw_y - self.y)
   ```
4. Marks current cell as visited:
   ```python
   cell = self.get_cell(self.x, self.y)
   self.my_visited[cell] = time.monotonic()
   self.known_visited.add(cell)
   ```
5. Updates `self.position_rx_mono` timestamp for staleness detection

**Key Data Structures Updated**:
- `self.x`, `self.y`, `self.theta` (robot pose)
- `self.my_visited[cell] = timestamp` (personal visit history)
- `self.known_visited` (union of all visited cells)
- `self.known_visited_time[cell]` (when we first learned about cell)

---

### 2. Sensor Processing

#### `sensor_loop()` (Lines 2409-2420)
**Purpose**: Continuously reads ToF sensors for obstacle detection.

**Information Flow**:
1. Calls `hw.read_tof_all()` -> Returns `Dict[str, int]` (sensor_name: distance_mm)
2. Calls `update_tof_obstacles()` to convert readings to cell coordinates
3. Sleeps 20ms (targets ~8-10 Hz effective rate)

#### `update_tof_obstacles()` (Lines 1106-1137)
**Purpose**: Converts ToF distance readings into obstacle cell map.

**Information Flow**:
1. Clears `self.tof_obstacles` (obstacles are instantaneous, not accumulated)
2. For each sensor reading:
   - Calculates obstacle position: `obs_x = self.x + reading * cos(angle)`
   - Converts to cell: `cell = self.get_cell(obs_x, obs_y)`
   - Adds to `self.tof_obstacles` set
3. Only processes readings in range [30mm, 200mm] (TOF_RANGE)

**Key Data Structures Updated**:
- `self.tof_obstacles` (set of cells with obstacles detected)

---

### 3. Peer Communication

#### `broadcast_state()` (Lines 663-705)
**Purpose**: Broadcasts robot's state to all peers at 5 Hz.

**Information Flow**:
1. If leader, calls `compute_task_assignments()` to get assignments
2. Builds JSON message containing:
   - Position: `x`, `y`, `theta`
   - State: `state` (idle/leader/follower)
   - Trust: `trust_in_leader`, `suspicion`
   - Election: `vote`, `leader_id`
   - Coverage: `coverage_count`, `visited_cells` (last 100)
   - Navigation: `navigation_target`, `predicted_target`, `commanded_target`
   - Assignments: `assignments` (if leader)
3. Sends via UDP:
   - Sim mode: Individual unicast to each peer's port
   - Hardware mode: Broadcast to `192.168.20.255:5200`

**Key Data Structures Read**:
- `self.x`, `self.y`, `self.theta`
- `self.trust_in_leader`, `self.suspicion_of_leader`
- `self.my_vote`, `self.current_leader`
- `self.my_visited` (last 100 cells)
- `self.current_navigation_target`, `self.my_predicted_target`, `self.leader_assigned_target`

#### `receive_peer_states()` (Lines 707-718)
**Purpose**: Receives state broadcasts from peers.

**Information Flow**:
1. Listens on UDP port 5200
2. Parses JSON message
3. Calls `update_peer(peer_id, msg)` for each message

#### `update_peer()` (Lines 720-768)
**Purpose**: Updates peer information and merges their coverage data.

**Information Flow**:
1. Creates `PeerInfo` object if peer doesn't exist
2. Updates peer state:
   ```python
   peer.x = msg['x']
   peer.y = msg['y']
   peer.theta = msg['theta']
   peer.trust_score = msg['trust_in_leader']  # Their trust in THEIR leader
   peer.vote = msg['vote']
   peer.last_seen = time.time()
   ```
3. Merges visited cells:
   ```python
   for cell in msg.get('visited_cells', []):
       peer.visited_cells.add(cell)
       if cell not in self.known_visited:
           self.known_visited.add(cell)
           self.known_visited_time[cell] = time.monotonic()
   ```
4. If peer is leader, extracts assignment:
   ```python
   if msg.get('leader_id') == peer_id:
       assignments = msg.get('assignments', {})
       if str(self.robot_id) in assignments:
           self.leader_assigned_target = tuple(assignments[str(self.robot_id)])
           self.leader_assignment_rx_mono = time.monotonic()
   ```
5. Calls `assess_peer_behavior()` to detect peer anomalies (stuck, spinning, etc.)

**Key Data Structures Updated**:
- `self.peers[peer_id]` (PeerInfo object)
- `self.known_visited` (union of all visited cells)
- `self.known_visited_time[cell]` (when we learned about cell)
- `self.leader_assigned_target` (if peer is leader)
- `self.leader_assignment_rx_mono` (assignment receive timestamp)

---

### 4. Trust Assessment (Core REIP Mechanism)

#### `assess_leader_command()` (Lines 858-1038)
**Purpose**: Proactively verifies leader commands before execution.

**Information Flow**:

**Step 1: Three-Tier Confidence Check**

For each leader assignment, checks if target cell is already explored:

1. **Tier 1 - Personal Visit History** (Weight 1.0):
   ```python
   if cell in self.my_visited:
       visited_time = self.my_visited[cell]
       if visited_time < causality_cutoff:  # Visited BEFORE assignment
           suspicion_added += WEIGHT_PERSONAL  # 1.0
           reasons.append("personal_visited")
   ```
   - **Causality Check**: Only penalizes if cell was visited *before* the assignment timestamp (minus grace period)
   - **Purpose**: Prevents false positives from network delay

2. **Tier 2 - ToF Sensor Obstacles** (Weight 1.0):
   ```python
   elif dist_to_target <= TOF_RANGE and cell in self.tof_obstacles:
       suspicion_added += WEIGHT_TOF  # 1.0
       reasons.append("tof_obstacle")
   ```
   - **Instantaneous**: ToF readings are current, no causality check needed
   - **Purpose**: Catches commands to cells with visible obstacles

3. **Tier 3 - Peer-Reported Coverage** (Weight 0.3):
   ```python
   elif cell in self.known_visited and cell not in self.my_visited:
       known_time = self.known_visited_time.get(cell, time.monotonic())
       if known_time < causality_cutoff:  # Knew about it BEFORE assignment
           suspicion_added += WEIGHT_PEER  # 0.3
           reasons.append("peer_reported")
   ```
   - **Causality Check**: Only penalizes if we learned about the cell before the assignment
   - **Purpose**: Catches commands to cells explored by peers

**Step 2: MPC Direction Check** (Reinforcing)

```python
mpc_suspicion, mpc_severe = self._compute_mpc_direction_error()
```

Checks if leader's commanded direction aligns with any unexplored area:

- **Severe Mismatch** (>135deg from nearest frontier): Can trigger independently
- **Moderate Mismatch** (90-135deg): Only reinforces three-tier evidence
- **Purpose**: Catches "wrong direction" faults even if target cell is unexplored

**Step 3: Suspicion Accumulation**

```python
if suspicion_added > 0:
    self.suspicion_of_leader += suspicion_added
    self.bad_commands_received += 1
else:
    # Good command - recover slowly
    self.suspicion_of_leader = max(0, self.suspicion_of_leader - RECOVERY_RATE)
```

**Step 4: Trust Decay (Threshold Crossing)**

```python
if self.suspicion_of_leader >= SUSPICION_THRESHOLD:  # 1.5
    self.trust_in_leader = max(MIN_TRUST, self.trust_in_leader - TRUST_DECAY_RATE)  # -0.2
    self.suspicion_of_leader -= SUSPICION_THRESHOLD  # Carry over excess
```

**Key Properties**:
- **Carry-over**: Excess suspicion above threshold is preserved (not reset)
- **Recovery**: Good commands slowly reduce suspicion (RECOVERY_RATE = 0.1)
- **Detection Bounds**:
  - Tier 1: `ceil(1.5 / (1.0 - 0.1)) = 2 commands`
  - Tier 3: `ceil(1.5 / (0.3 - 0.1)) = 8 commands`

**Key Data Structures Updated**:
- `self.suspicion_of_leader` (accumulated suspicion)
- `self.trust_in_leader` (decays on threshold crossings)
- `self.bad_commands_received` (count for metrics)
- `self.first_bad_command_time`, `self.first_decay_time` (timing metrics)

#### `_compute_mpc_direction_error()` (Lines 1040-1104)
**Purpose**: Computes direction consistency error between commanded and predicted directions.

**Information Flow**:
1. Calculates commanded direction: `cmd_dir = atan2(target_y - self.y, target_x - self.x)`
2. Finds all unexplored cells (frontiers)
3. Computes alignment with nearest frontier:
   ```python
   best_alignment = min(angle_diff(cmd_dir, frontier_dir) for all frontiers)
   ```
4. Returns suspicion weight:
   - **Severe** (>135deg): `0.5 + (error - 135deg) / 45deg * 0.3` (can trigger independently)
   - **Moderate** (90-135deg): `(error - 90deg) / 45deg * 0.3` (only reinforces)
   - **Good** (<90deg): `0.0`

---

### 5. Leader Task Assignment

#### `compute_task_assignments()` (Lines 1140-1360)
**Purpose**: Leader computes frontier assignments for all robots using greedy nearest-frontier algorithm.

**Information Flow**:

**Step 1: Find Frontiers**
```python
for cx, cy in all_cells:
    if (cx, cy) not in self.known_visited and not self._is_wall_cell(cx, cy):
        frontiers.append((cx, cy))
```

**Step 2: Collect Robot Positions**
```python
robots = {self.robot_id: (self.x, self.y)}
for pid, peer in self.peers.items():
    if time.time() - peer.last_seen < PEER_TIMEOUT:
        robots[pid] = (peer.x, peer.y)
```

**Step 3: Assignment Persistence (Hysteresis)**
- Keeps previous assignments if target is still unexplored
- Reassigns if robot has reached target (within CELL_SIZE)

**Step 4: Greedy Assignment**
```python
sorted_robots = sorted(need_assignment, key=nearest_frontier_dist)
for rid in sorted_robots:
    best_frontier = min(available_frontiers, key=lambda f: path_distance(robots[rid], f))
    assignments[str(rid)] = self.cell_to_pos(best_frontier)
```

**Step 5: Spatial Diversity**
- Ensures at least one robot targets each room (if passage is clear)
- Prevents all robots from clustering in one room

**Key Data Structures Updated**:
- `self._prev_assignments` (assignment cache for persistence)
- `self.my_assigned_target` (leader's own target)

**Returns**: `Dict[str, Tuple[float, float]]` mapping robot_id to (x, y) target

---

### 6. Election and Impeachment

#### `check_impeachment()` (Lines 1653-1665)
**Purpose**: Checks if leader should be impeached (called before election).

**Information Flow**:
```python
if self.trust_in_leader < IMPEACHMENT_THRESHOLD:  # 0.3
    if self.impeachment_time is None:
        self.impeachment_time = time.time()
    # Leader will be excluded from candidate list in run_election()
```

**Key Data Structures Updated**:
- `self.impeachment_time` (timestamp for metrics)

#### `run_election()` (Lines 1549-1649)
**Purpose**: Trust-weighted leader election with distributed voting.

**Information Flow**:

**Step 1: Update Peer Trust for Current Leader**
```python
if self.current_leader and self.current_leader != self.robot_id:
    if self.current_leader in self.peers:
        self.peers[self.current_leader].my_trust_for_them = self.trust_in_leader
```

**Step 2: Build Candidate List**
```python
candidates = [(self.robot_id, 1.0, self.leader_failures.get(self.robot_id, 0))]
for pid, peer in self.peers.items():
    if peer.my_trust_for_them > TRUST_THRESHOLD:  # 0.5
        candidates.append((pid, peer.my_trust_for_them, self.leader_failures.get(pid, 0)))
```

**Step 3: Sort Candidates**
```python
candidates.sort(key=lambda x: (-x[1], x[2], x[0]))  # Trust desc, failures asc, id asc
self.my_vote = candidates[0][0]
```

**Step 4: Count Votes**
```python
votes = {self.my_vote: 1}
for pid, peer in self.peers.items():
    if peer.vote in eligible_ids:  # Only count votes for eligible candidates
        votes[peer.vote] = votes.get(peer.vote, 0) + 1
```

**Step 5: Elect Winner**
```python
sorted_candidates = sorted(votes.items(), 
    key=lambda x: (-x[1], self.leader_failures.get(x[0], 0), x[0]))
self.current_leader = sorted_candidates[0][0]
```

**Step 6: Handle Leader Change**
```python
if self.current_leader != old_leader:
    if old_leader is not None and self._election_settled:
        self.leader_failures[old_leader] += 1  # Track impeachment history
    # Reset trust for new leader
    self.trust_in_leader = 1.0
    self.suspicion_of_leader = 0.0
```

**Key Data Structures Updated**:
- `self.current_leader` (elected leader ID)
- `self.my_vote` (this robot's vote)
- `self.leader_failures[leader_id]` (impeachment history for tie-breaking)
- `self.trust_in_leader` (reset to 1.0 for new leader)

---

### 7. Motor Command Computation

#### `compute_motor_command()` (Lines 1972-2280)
**Purpose**: Computes left/right motor PWM values based on navigation target and reactive behaviors.

**Information Flow**:

**Step 1: Determine Navigation Target**

If follower:
```python
if self.leader_assigned_target:
    target = self.leader_assigned_target
    # Verify command BEFORE using it
    self.assess_leader_command()
else:
    target = self.get_my_frontier()  # Fallback to local frontier
```

If leader:
```python
if self.my_assigned_target:
    target = self.my_assigned_target
else:
    target = self.get_my_frontier()
```

**Step 2: Reactive Overrides (Priority Order)**

1. **Stuck Detection** (`_check_stuck()`):
   - Position-based: moved < 20mm in 1.5s
   - Encoder-based: encoders unchanged in 1.5s with motors on
   - Triggers escape sequence

2. **Peer Emergency**:
   - If peer within 280mm (PEER_CONTACT_DIST)
   - Flees away from peer

3. **ToF Emergency**:
   - If obstacle < 150mm in front
   - Reverses and pivots

4. **Escape Mode**:
   - Backs out, pivots toward open space, resumes

**Step 3: Normal Navigation**

```python
target_angle = atan2(target_y - self.y, target_x - self.x)
diff = target_angle - self.theta  # Normalize to [-pi, pi]

# Speed factors (0.0 to 1.0)
sf_wall = compute_wall_speed_factor()  # Slows near walls
sf_tof = compute_tof_speed_factor()    # Slows near obstacles
sf_peer = compute_peer_speed_factor()   # Slows near peers

effective_speed = BASE_SPEED * min(sf_wall, sf_tof, sf_peer)

# Turn mixing
turn_mix = 0.55  # Boosted to 0.8 if near obstacles
left_speed = effective_speed * (1 - turn * turn_mix)
right_speed = effective_speed * (1 + turn * turn_mix)
```

**Step 4: Motor Dead Zone Protection**

```python
if abs(turn) > 0.1:
    if abs(left_speed) < MIN_MOTOR_PWM:  # 25
        left_speed = copysign(MIN_MOTOR_PWM, left_speed)
    # Same for right_speed

# If both below minimum, scale both up proportionally
if abs(left_speed) < MIN_MOTOR_PWM and abs(right_speed) < MIN_MOTOR_PWM:
    scale = MIN_MOTOR_PWM / max(abs(left_speed), abs(right_speed), 1.0)
    left_speed *= scale
    right_speed *= scale
```

**Step 5: Rate Limiting**

```python
left, right = _finalize_motor_command(left_speed, right_speed, stop_reason)
# Limits PWM changes to MAX_PWM_STEP (20.0 normal, 50.0 emergency)
```

**Key Data Structures Read**:
- `self.leader_assigned_target` (follower's assigned target)
- `self.my_assigned_target` (leader's own target)
- `self.x`, `self.y`, `self.theta` (current pose)
- `self.tof` (sensor readings)
- `self.peers` (peer positions)

**Key Data Structures Updated**:
- `self.current_navigation_target` (for logging/visualization)
- `self._last_motor_cmd` (for rate limiting)

---

## Process Flows

### Flow 1: Normal Operation (Clean Leader)

```
1. Leader computes assignments:
   compute_task_assignments()
   -> Finds frontiers
   -> Greedy assignment
   -> Returns {robot_id: (x, y)}

2. Leader broadcasts state:
   broadcast_state()
   -> Includes assignments
   -> UDP broadcast to all peers

3. Follower receives assignment:
   receive_peer_states() -> update_peer()
   -> Sets self.leader_assigned_target
   -> Sets self.leader_assignment_rx_mono

4. Follower verifies command:
   compute_motor_command()
   -> assess_leader_command()
   -> Three-tier check: cell not in my_visited, tof_obstacles, known_visited
   -> No suspicion added (good command)
   -> suspicion_of_leader -= RECOVERY_RATE (slow recovery)

5. Follower navigates:
   compute_motor_command()
   -> Uses leader_assigned_target
   -> A* pathfinding, wall avoidance
   -> Returns (left_pwm, right_pwm)

6. Follower marks coverage:
   receive_position()
   -> Marks current cell as visited
   -> Updates my_visited, known_visited
```

### Flow 2: Fault Detection and Impeachment

```
1. Bad leader sends explored cell:
   compute_task_assignments() [bad_leader_mode=True]
   -> Returns explored cell as target

2. Follower receives bad assignment:
   receive_peer_states() -> update_peer()
   -> Sets self.leader_assigned_target = explored_cell

3. Follower verifies command:
   compute_motor_command()
   -> assess_leader_command()
   -> Tier 1 check: cell in self.my_visited
   -> visited_time < causality_cutoff (visited BEFORE assignment)
   -> suspicion_added += 1.0 (WEIGHT_PERSONAL)
   -> suspicion_of_leader += 1.0
   -> bad_commands_received += 1

4. Suspicion accumulates:
   After 2 bad commands (Tier 1):
   -> suspicion_of_leader >= 1.5 (SUSPICION_THRESHOLD)
   -> trust_in_leader -= 0.2 (TRUST_DECAY_RATE)
   -> suspicion_of_leader -= 1.5 (carry over excess)
   -> first_decay_time = time.time()

5. Trust continues to decay:
   After 4 threshold crossings:
   -> trust_in_leader < 0.3 (IMPEACHMENT_THRESHOLD)
   -> check_impeachment() sets impeachment_time

6. Election occurs:
   run_election()
   -> check_impeachment() excludes leader from candidates
   -> Builds candidate list (trust > 0.5)
   -> Old leader excluded (trust < 0.3)
   -> New leader elected
   -> leader_failures[old_leader] += 1

7. New leader takes over:
   -> trust_in_leader = 1.0 (reset)
   -> suspicion_of_leader = 0.0 (reset)
   -> New assignments computed
```

### Flow 3: Election Process

```
1. Election timer triggers:
   control_loop()
   -> time.time() - last_election > ELECTION_INTERVAL (2.0s)
   -> Calls run_election()

2. Update peer trust:
   -> If current_leader in peers:
      peers[current_leader].my_trust_for_them = trust_in_leader

3. Build candidates:
   candidates = []
   -> Add self (trust = 1.0)
   -> For each peer with trust > 0.5:
      candidates.append((peer_id, peer.my_trust_for_them, failures))

4. Sort candidates:
   -> Sort by: (-trust, failures, id)
   -> self.my_vote = candidates[0][0]

5. Count votes:
   votes = {self.my_vote: 1}
   -> For each peer:
      if peer.vote in eligible_ids:
         votes[peer.vote] += 1

6. Elect winner:
   -> Sort by: (-votes, failures, id)
   -> self.current_leader = winner

7. Handle leader change:
   -> If leader changed:
      leader_failures[old_leader] += 1
      trust_in_leader = 1.0 (reset)
      suspicion_of_leader = 0.0 (reset)
```

### Flow 4: Coverage Merging

```
1. Robot visits cell:
   receive_position()
   -> cell = get_cell(self.x, self.y)
   -> self.my_visited[cell] = time.monotonic()
   -> self.known_visited.add(cell)
   -> self.known_visited_time[cell] = time.monotonic()

2. Robot broadcasts state:
   broadcast_state()
   -> Includes visited_cells (last 100)
   -> UDP broadcast

3. Peer receives broadcast:
   receive_peer_states() -> update_peer()
   -> For each cell in msg['visited_cells']:
      peer.visited_cells.add(cell)
      if cell not in self.known_visited:
         self.known_visited.add(cell)
         self.known_visited_time[cell] = time.monotonic()

4. Leader uses merged coverage:
   compute_task_assignments()
   -> For each cell:
      if cell not in self.known_visited:
         frontiers.append(cell)
```

---

## Data Structures

### Core State Variables

```python
# Position
self.x, self.y, self.theta          # Current pose (mm, mm, radians)
self.position_rx_mono              # Timestamp of last position update

# Coverage
self.my_visited: Dict[tuple, float]  # {cell: timestamp} - cells I visited
self.known_visited: Set[tuple]        # All known visited cells (union)
self.known_visited_time: Dict[tuple, float]  # {cell: when_we_learned}

# Leadership
self.current_leader: Optional[int]   # Elected leader ID
self.my_vote: Optional[int]           # My vote in election
self.trust_in_leader: float           # Trust in current leader [0.0, 1.0]
self.suspicion_of_leader: float      # Accumulated suspicion
self.leader_failures: Dict[int, int] # {leader_id: failure_count}

# Task Assignment
self.leader_assigned_target: Optional[Tuple[float, float]]  # Leader's command
self.leader_assignment_rx_mono: float  # When assignment was received
self.my_assigned_target: Optional[Tuple[float, float]]      # Leader's own target
self.my_predicted_target: Optional[Tuple[float, float]]    # Local optimal target

# Sensors
self.tof: Dict[str, int]              # {sensor_name: distance_mm}
self.tof_obstacles: Set[tuple]        # Cells with obstacles detected

# Peers
self.peers: Dict[int, PeerInfo]       # {peer_id: PeerInfo}
```

### PeerInfo Dataclass

```python
@dataclass
class PeerInfo:
    robot_id: int
    x, y, theta: float                # Position
    state: str                         # 'idle', 'leader', 'follower'
    trust_score: float                 # Their reported trust in THEIR leader
    my_trust_for_them: float          # MY trust assessment of them
    vote: Optional[int]                # Their vote in election
    coverage_count: int                # Number of cells they've visited
    visited_cells: Set[tuple]          # Cells they've visited
    assigned_target: Optional[Tuple]   # Leader's assignment for them
    last_seen: float                   # Timestamp of last update
    last_x, last_y: float              # Previous position (for velocity estimation)
    last_update_time: float            # Previous update timestamp
```

---

## Threading Model

REIP runs three concurrent threads:

### 1. `sensor_loop()` Thread
- **Frequency**: ~8-10 Hz (limited by ToF read time)
- **Purpose**: Read ToF sensors, update obstacle map
- **Blocking**: Yes (reads hardware I2C)

### 2. `network_loop()` Thread
- **Frequency**: 100 Hz polling (10ms sleep)
- **Purpose**: Receive position, peer states, fault injection; broadcast state
- **Blocking**: No (UDP non-blocking recv)

### 3. `control_loop()` Thread (Main)
- **Frequency**: 10 Hz (100ms interval)
- **Purpose**: Elections, trust assessment, motor command computation
- **Blocking**: No (CPU-bound)

### Thread Safety

- **Position updates**: Thread-safe (single writer: `network_loop`)
- **Peer updates**: Thread-safe (single writer: `network_loop`)
- **Trust/assignment**: Thread-safe (single writer: `control_loop`)
- **Motor commands**: Thread-safe (UART lock in `hw.set_motors()`)

---

## Key Design Decisions

### 1. Why Proactive Verification?

**Problem**: Reactive systems detect faults after performance degradation (coverage drops).

**Solution**: REIP verifies commands *before* execution using local knowledge, preventing wasted movement on bad commands.

### 2. Why Three Tiers?

**Problem**: Different evidence sources have different reliability.

**Solution**: Weight evidence by source reliability:
- Personal visits (1.0): Ground truth, no staleness
- ToF sensors (1.0): Direct observation, instantaneous
- Peer reports (0.3): May be stale due to network delay

### 3. Why Causality Gating?

**Problem**: Network delay can cause false positives (leader's map is stale, not malicious).

**Solution**: Only penalize if evidence predates assignment timestamp (minus grace period).

### 4. Why Carry-Over Suspicion?

**Problem**: Resetting suspicion on threshold crossing allows persistent faults to evade detection.

**Solution**: Carry excess suspicion above threshold, ensuring persistent faults are eventually detected.

### 5. Why Trust-Weighted Elections?

**Problem**: Simple majority voting can elect untrustworthy leaders.

**Solution**: Only candidates with trust > 0.5 are eligible, and votes are weighted by trust scores.

---

## Summary

REIP is a distributed governance framework that enables multi-robot teams to:

1. **Detect compromised leaders** through proactive command verification
2. **Impeach faulty leaders** through distributed voting
3. **Elect new leaders** based on trust scores and failure history
4. **Operate without centralized oversight** or ground-truth oracle access

The system achieves sub-second fault detection (0.20-0.21s suspicion, 1.21-1.67s impeachment) through a three-tier confidence-weighted trust model that verifies commands before execution, preventing wasted movement on bad commands.
