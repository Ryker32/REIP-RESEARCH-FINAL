# REIP WSSEF Strategy Document
**Date: February 24, 2026**

---

## THE CORE CLAIM

> **"REIP maintains coordinated performance that autonomous agents can't match, even when faults would normally destroy that coordination."**

Everything on the poster — the vector overlays, the baselines, the trust model — serves this single sentence.

---

## JUDGE VULNERABILITY: "Fallback Drives Performance"

**The attack:** "Your ablation shows robots do fine when they ignore the leader. Isn't REIP just expensive autonomy with extra steps?"

**The defense:** REIP is *governed fallback*.

| Component | Role |
|-----------|------|
| **Trust** | The trigger — detects when coordination is failing |
| **Election** | The recovery — restores coordination with new leader |
| **Fallback** | The safety net — maintains progress during transition |

Without governance, you get:
- Fallback ON, Trust OFF → Robots never return to coordination. You lose the efficiency gains of merged maps and partitioned tasks.
- Trust ON, Fallback OFF → One bad command and the robot freezes waiting for impeachment. Brittle.

**REIP = the system that knows when to coordinate, when to fall back, and when to recover.**

---

## REQUIRED ABLATIONS

### Must Run (Judge-Proof)

| Ablation | Trust | Election | Fallback | Expected Result |
|----------|-------|----------|----------|-----------------|
| **Full REIP** | ✓ | ✓ | ✓ | Best overall |
| **No Fallback** | ✓ | ✓ | ✗ | Fragile under faults |
| **No Governance** | ✗ | ✗ | ✓ | Pure autonomy, no recovery |
| **Fixed Leader** | ✗ | ✗ | ✗ | Catastrophic under leader fault |

**Critical addition:** "Fallback ON, Governance OFF" proves that governance does real work beyond just having a fallback mechanism.

### Baseline Comparisons

| Baseline | Description | Why It Matters |
|----------|-------------|----------------|
| **Fixed Leader** | Single leader, no replacement | Shows value of any failover |
| **Round-Robin** | Rotate leader on timer | Shows value of trust-based selection |
| **Decentralized** | No leader, pure autonomy | Shows value of coordination |
| **RAFT Heartbeat** | Replace leader only on heartbeat timeout | **Judge-proof baseline** |

---

## RAFT BASELINE (Critical)

**What it is:** Leader sends heartbeats. If heartbeats stop (leader crashed/disconnected), elect new leader. No trust. No performance reasoning.

**Why it's judge-proof:** RAFT is industry standard. It's what serious distributed systems use. If REIP beats RAFT under adversarial faults (where the leader is *alive* but *misbehaving*), no judge can dismiss that.

**The key insight:** RAFT handles crash faults. REIP handles Byzantine faults (malicious/erratic behavior). A spinning leader still sends heartbeats — RAFT never catches it. REIP's trust model does.

**Implementation:**
```python
class RAFTBaseline:
    def __init__(self):
        self.heartbeat_timeout = 2.0  # seconds
        self.last_heartbeat = time.time()
    
    def receive_heartbeat(self, leader_id):
        self.last_heartbeat = time.time()
    
    def check_leader(self):
        if time.time() - self.last_heartbeat > self.heartbeat_timeout:
            self.trigger_election()  # Leader presumed dead
        # NO trust check. NO performance check.
```

---

## TRUST MODEL (Final Version)

Three-tier confidence-weighted suspicion:

```python
# Weights by source reliability
WEIGHT_PERSONAL = 1.0    # Cells I visited — ground truth
WEIGHT_TOF = 1.0         # Obstacles in sensor range — I see it
WEIGHT_PEER = 0.3        # Peer-reported — might be stale

# Thresholds
SUSPICION_THRESHOLD = 1.5
RECOVERY_RATE = 0.1      # < lowest weight, prevents wash-out

def assess_leader_command(self, target_cell):
    if cell in self.my_visited:
        suspicion += 1.0  # I was there
    elif in_tof_range and is_obstacle:
        suspicion += 1.0  # I see it
    elif cell in self.known_visited:
        suspicion += 0.3  # Peer said so
    else:
        suspicion -= 0.1  # Good command, recover
    
    if suspicion >= 1.5:
        trust -= 0.1
        suspicion -= 1.5  # Carry over, don't reset
```

---

## VISUALIZATION (Poster Frames)

**Vector overlay showing:**
- Green arrow: Robot's predicted vector (local optimal)
- Red arrow: Leader's commanded vector
- Yellow contour: Trust wavering
- Orange contour: About to impeach

**5-frame comparison strip:**

| Frame | WITHOUT REIP | WITH REIP |
|-------|--------------|-----------|
| 1 | Fault injected, bad commands | Fault injected, bad commands |
| 2 | Robots follow blindly | Yellow contours appear |
| 3 | Coverage stalls, dead ends | More robots wavering |
| 4 | System stuck | Impeachment, election |
| 5 | Low coverage, chaos | New leader, recovery, high coverage |

---

## IMPLEMENTATION ORDER

### Phase 1: Hardware Foundation
- [ ] Clone SD card → clanker2, clanker3
- [ ] Flash Picos
- [ ] Mount overhead camera
- [ ] Test ArUco position server
- [ ] Deploy `reip_node.py`
- [ ] Run 3-robot smoke test

### Phase 2: Experiments
- [ ] Full REIP with fault injection
- [ ] Ablation: No Fallback
- [ ] Ablation: No Governance (fallback only)
- [ ] Ablation: Fixed Leader
- [ ] **Baseline: RAFT heartbeat failover**
- [ ] Baseline: Round-Robin
- [ ] Baseline: Decentralized autonomy

### Phase 3: Visualization
- [ ] Collect logs from all experiments
- [ ] Generate vector overlay videos
- [ ] Extract 5-frame comparison strips
- [ ] Create coverage-over-time plots

### Phase 4: Analysis
- [ ] Compare coverage curves (REIP vs baselines)
- [ ] Time-to-recovery after fault injection
- [ ] False positive rate (impeachments of good leaders)
- [ ] Prepare judge responses for "fallback drives it" attack

---

## FILES REFERENCE

| File | Purpose |
|------|---------|
| `robot/reip_node.py` | Main robot brain with three-tier trust |
| `pc/aruco_position_server.py` | Vision-based localization |
| `pc/visualize_vectors.py` | Post-processing for poster frames |
| `pc/fault_inject.py` | Fault injection for experiments |
| `pc/logger.py` | Centralized log collection |
| `pico/main.py` | Low-level motor control |
| `deploy.ps1` | Push code to all robots |

---

## REMEMBER

The poster tells ONE story:

1. **Problem:** Coordination beats autonomy, but coordination has a single point of failure.
2. **Solution:** REIP — trust-based governance that catches bad leaders before damage accumulates.
3. **Evidence:** REIP beats RAFT (the industry standard) under adversarial faults.
4. **Visual:** Vectors diverge → trust decays → impeachment → recovery → vectors align.

That's it. Everything else is detail.
