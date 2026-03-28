# Procedures - Numbered List Format (for WSSEF)

1. Assemble five differential-drive mobile robots, each including: Raspberry Pi Zero 2W for onboard computation and wireless communication, Raspberry Pi Pico H for motor control via UART, five VL53L0X time-of-flight sensors for obstacle detection (connected via I2C multiplexer), two N20 geared DC motors with encoders (1:100 gear ratio), 2S 1000 mAh LiPo battery power system, and 3D-printed PLA chassis (under 300g per robot).

2. Program each robot with ArUco identification tags (IDs 1-5, 4*4 library, 50mm) for visual tracking, configure WiFi peer-to-peer communication, and name robots clanker1-5. The system operates autonomously without centralized control during trials.

3. Develop all distributed software used by the robots, including: robot localization input processing, coverage tracking and map merging, peer-state broadcasting and reception, leader election mechanisms, leader-follower coordination protocols, trust evaluation algorithms, and fault-response behaviors.

4. Implement the Resilient Election and Impeachment Policy (REIP), in which follower robots evaluate leader commands before executing them. If locally available evidence suggests a command is harmful, unsafe, or inconsistent with maximizing coverage, the follower reduces its trust in the leader. When trust falls below a threshold, the follower votes to impeach. If a majority reaches consensus, the leader is removed and replaced through a new election.

5. Implement baseline comparison systems from distributed coordination literature (Raft consensus protocol) that respond to unresponsive leaders but do not proactively evaluate whether an active leader is sending harmful or low-quality commands.

6. Construct an indoor experimental arena (2000 * 1500 mm) using 38 mm EPS foamboard to represent a constrained search-and-rescue-style environment. The arena includes: boundary walls covered with masking tape to reduce ToF sensor reflections, a central divider (1000 mm from left edge, 1300 mm tall) creating a two-room layout, and four ArUco corner markers (IDs 40-43, 80mm) for camera-based localization.

7. Set up overhead imaging using a Logitech C922 webcam positioned approximately 2040 mm above the arena center. Configure an OpenCV-based script running on a laptop to perform real-time homography transformation, per-robot pose tracking, and live visualization overlay. The script broadcasts each robot's position via UDP to that robot only (no cross-robot position sharing).

8. Define the mission objective for each trial as maximizing area coverage over time. At trial start, place robots in selected starting positions, initialize them under the same software condition, and start them with the same exploration objective. The robots then explore the arena autonomously.

9. Conduct repeated no-fault hardware trials using REIP. In these trials, the leader behaves normally. Purpose: measure normal coordination, coverage efficiency, leader stability, and false-positive impeachment rates during standard operation.

10. Conduct repeated fault-injection hardware trials using REIP. In these trials, the leader remains active but intentionally issues faulty or mission-degrading commands. Fault types include: directing followers toward densely explored regions, repeatedly changing assignments in an unstable manner, assigning robots into unsafe peer conflicts, freezing assignments (ceasing to update target assignments), or other behaviors that reduce team effectiveness while appearing responsive.

11. Conduct matching hardware trials using baseline policies (Raft consensus) under the same arena layouts, robot counts, starting conditions, and fault scenarios so that REIP can be directly compared against conventional leader-election approaches.

12. Collect time-stamped data during every trial through robot-generated logs and supporting experimental software. Logged data includes: robot positions and heading values, assigned targets and predicted targets, leader identity and election outcomes, trust values and suspicion values, votes and impeachment events, coverage progress over time, mission timing and completion status, peer-emergency events and wall-emergency events, and other safety-related behaviors.

13. Develop a hardware-faithful simulation environment that reproduces the same distributed robot logic, communication structure, and realistic impairments including: sensing noise, communication delay and packet loss, packet reordering, and pose uncertainty.

14. Validate selected simulation runs against hardware runs under matching clean conditions. Compare metrics such as coverage, timing, safety events, and leadership behavior to determine whether the simulation reproduces real hardware behavior closely enough for controlled testing.

15. Perform repeated simulation trials (target: 100 trials per condition) for each experimental condition (REIP clean, REIP bad-leader, REIP freeze-leader, Raft clean, Raft bad-leader, Raft freeze-leader, Decentralized clean, Decentralized bad-leader, Decentralized freeze-leader) to observe variability and ensure results are not based on single outcomes.

16. Calculate mission-performance metrics from the collected data, including: total final coverage (percentage of explorable area), time to reach selected coverage thresholds (e.g., 50%, 60%, 80%), leader settle time (time to stable leadership), time to trust decay (time from fault injection to first trust drop), time to impeachment (time from fault injection to leader removal), mission completion time (where applicable), number of false impeachments (in clean trials), minimum robot-to-robot separation, minimum wall clearance, and emergency-event counts.

17. Analyze and compare REIP and baseline results under matched conditions to determine: whether REIP improves resilience to subtle active leader faults, whether REIP preserves normal exploration performance during no-fault operation, and the consistency and statistical significance of any observed advantages.
