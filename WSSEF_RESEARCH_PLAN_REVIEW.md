# WSSEF Research Plan Review

## [x] Rationale - EXCELLENT

Your rationale is comprehensive and well-structured:
- [x] Explains importance (time-critical missions, avalanche SAR)
- [x] Describes social/societal impact (search-and-rescue, disaster response)
- [x] Identifies the problem (leader vulnerability)
- [x] Mentions existing solutions (Raft) and their limitations
- [x] Describes your solution (REIP)
- [x] Connects to future applications (RECCO radar, avalanche debris)

**No changes needed.**

---

## [x] Research Question/Hypothesis/Engineering Goals - GOOD

### Engineering Goals:
- [x] Primary goals clearly stated
- [x] Secondary goals included
- [x] All goals relate to rationale

### Hypothesis:
- [x] Clear and testable
- [x] States expected outcomes
- [x] Relates to rationale

**Minor suggestion:** Consider making the hypothesis more concise. Current version is good but could be slightly tightened.

---

## [!] Procedures - NEEDS FORMATTING FIX

**ISSUE:** WSSEF requires a **numbered list**. Your procedures are in sections, not a numbered list.

**REQUIRED FORMAT:**
```
1. [Procedure 1]
2. [Procedure 2]
3. [Procedure 3]
...
```

**CURRENT FORMAT:** Sections with subsections (not numbered list)

**FIX:** Convert your procedure sections into a numbered list. Here's how to restructure:

### Suggested Numbered Procedure List:

1. Assemble five differential-drive mobile robots, each including: Raspberry Pi Zero 2W for computation, Raspberry Pi Pico H for motor control, five VL53L0X time-of-flight sensors, two N20 geared DC motors with encoders, 2S 1000 mAh LiPo battery, and 3D-printed PLA chassis (under 300g per robot).

2. Program each robot with ArUco identification tags (IDs 1-5) and configure WiFi peer-to-peer communication, naming robots clanker1-5.

3. Develop all distributed software including: robot localization input processing, coverage tracking and map merging, peer-state broadcasting and reception, leader election mechanisms, leader-follower coordination protocols, trust evaluation algorithms, and fault-response behaviors.

4. Implement the Resilient Election and Impeachment Policy (REIP), in which follower robots evaluate leader commands before executing them using three tiers of local evidence: personal visit history, sensor readings, and peer-reported coverage.

5. Implement baseline comparison systems from distributed coordination literature (Raft consensus protocol) that respond to unresponsive leaders but do not proactively evaluate whether an active leader is sending harmful commands.

6. Construct an indoor experimental arena (2000 * 1500 mm) using 38 mm EPS foamboard with boundary walls covered in masking tape, a central divider creating a two-room layout, and four ArUco corner markers (IDs 40-43, 80mm) for camera-based localization.

7. Set up overhead imaging using a Logitech C922 webcam positioned approximately 2040 mm above the arena center, with an OpenCV-based script performing real-time homography transformation, per-robot pose tracking, and live visualization overlay.

8. Define the mission objective for each trial as maximizing area coverage over time, with robots placed in selected starting positions and initialized under the same software condition.

9. Conduct repeated no-fault hardware trials using REIP to measure normal coordination, coverage efficiency, leader stability, and false-positive impeachment rates during standard operation.

10. Conduct repeated fault-injection hardware trials using REIP, where the leader remains active but intentionally issues faulty commands including: directing followers toward densely explored regions, repeatedly changing assignments unstably, assigning robots into unsafe peer conflicts, or freezing assignments.

11. Conduct matching hardware trials using baseline policies (Raft) under the same arena layouts, robot counts, starting conditions, and fault scenarios.

12. Collect time-stamped data during every trial through robot-generated logs and supporting experimental software, including: robot positions and headings, assigned and predicted targets, leader identity and election outcomes, trust values and suspicion values, votes and impeachment events, coverage progress over time, mission timing, and safety-related behaviors.

13. Develop a hardware-faithful simulation environment that reproduces the same distributed robot logic, communication structure, and realistic impairments including sensing noise, communication delay and packet loss, packet reordering, and pose uncertainty.

14. Validate selected simulation runs against hardware runs under matching clean conditions, comparing metrics such as coverage, timing, safety events, and leadership behavior.

15. Perform repeated simulation trials (target: 100 trials per condition) for each experimental condition to observe variability and ensure results are not based on single outcomes.

16. Calculate mission-performance metrics from collected data including: total final coverage, time to reach coverage thresholds, leader settle time, time to trust decay, time to impeachment, number of false impeachments, minimum robot-to-robot separation, minimum wall clearance, and emergency-event counts.

17. Analyze and compare REIP and baseline results under matched conditions to determine whether REIP improves resilience to subtle active leader faults while preserving normal exploration performance during no-fault operation.

---

## [x] Data Analysis - EXCELLENT

Your data analysis section is comprehensive:
- [x] Metric calculation clearly described
- [x] Statistical analysis methods specified
- [x] Comparative analysis framework defined
- [x] Data visualization methods listed
- [x] Success criteria quantified
- [x] Simulation validation approach described

**No changes needed.**

---

## [!] Risk and Safety - NEEDS VERIFICATION

You mentioned "risks are done" but I don't see them in what you provided. **Make sure you include:**

- Electrical risks (LiPo batteries, motor drivers)
- Mechanical risks (moving robots, collisions)
- Software risks (robots getting stuck, communication failures)
- Safety precautions (battery handling, arena boundaries, emergency stop procedures)

**If you already have this, great! If not, add it.**

---

## [!] Bibliography - NEEDS ADDITION

You have 24 references, which is excellent. However, check if these are covered:

**Mentioned in Rationale:**
- [x] Avalanche survival (Rauch - [1])
- [x] Multi-robot SAR (Queralta - [2])
- [x] Frontier exploration (Yamauchi - [3], Burgard - [4])
- [x] Leader-follower coordination (Simmons - [11])
- [x] Raft consensus (Ongaro - [16])
- [x] Trust in multi-agent systems (Marsh - [17], Ramchurn - [23])
- [x] Byzantine fault tolerance (Lamport - [14], Castro - [15])
- [x] Disaster response (Murphy - [6])
- [x] DARPA Subterranean (Chung - [8])
- [x] NASA CADRE (de la Croix - [9], Whittaker - [10])
- [x] Mining robotics (Marshall - [22])
- [x] Mars communication (McBrayer - [7])

**MISSING:**
- [ ] **RECCO radar** - You mention "RECCO radar receivers" in rationale but no citation. Consider adding a reference about RECCO technology or victim detection systems.

**Optional additions:**
- Consider adding a reference about leader-follower vulnerabilities in multi-robot systems (if such a paper exists)
- Consider adding a reference about proactive vs. reactive fault detection (if available)

---

## Summary of Required Fixes

1. **CRITICAL:** Convert Procedures section to numbered list format (1, 2, 3...)
2. **VERIFY:** Ensure Risk and Safety section is included
3. **OPTIONAL:** Add RECCO radar citation if available, or remove mention if not needed

---

## Final Checklist

- [x] Rationale - Complete
- [x] Research Question/Hypothesis - Complete
- [ ] **Procedures - NEEDS NUMBERED LIST FORMAT**
- [ ] Risk and Safety - Verify included
- [x] Data Analysis - Complete
- [x] Bibliography - Mostly complete (consider RECCO citation)
