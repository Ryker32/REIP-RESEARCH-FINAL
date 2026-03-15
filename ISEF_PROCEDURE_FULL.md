# Experimental Procedure

## Overview

This study evaluates the Resilient Election and Impeachment Policy (REIP), a distributed governance framework for multi-robot exploration teams. The procedure involves hardware development, software implementation, experimental trials, and simulation validation to compare REIP against conventional leader-election baselines under both normal and fault-injection conditions.

## 1. Hardware Platform Development

I assembled a low-cost multi-robot testbed consisting of five differential-drive mobile robots. Each robot includes:
- Raspberry Pi Zero 2W for onboard computation and wireless communication
- Raspberry Pi Pico H for motor control via UART
- Five VL53L0X time-of-flight sensors for obstacle detection (connected via I2C multiplexer)
- Two N20 geared DC motors with encoders (1:100 gear ratio)
- 2S 1000 mAh LiPo battery power system
- 3D-printed PLA chassis (under 300g per robot)

Each robot is color-coded and carries an ArUco identification tag (IDs 1-5, 4×4 library, 50mm) for visual tracking. Robots communicate via WiFi peer-to-peer and are named clanker1-5. The system operates autonomously without centralized control during trials.

## 2. Software Development

I developed all distributed software used by the robots, including:
- Robot localization input processing
- Coverage tracking and map merging
- Peer-state broadcasting and reception
- Leader election mechanisms
- Leader-follower coordination protocols
- Trust evaluation algorithms
- Fault-response behaviors

The software implements REIP, in which follower robots evaluate leader commands before executing them. If locally available evidence suggests a command is harmful, unsafe, or inconsistent with maximizing coverage, the follower reduces its trust in the leader. When trust falls below a threshold, the follower votes to impeach. If a majority reaches consensus, the leader is removed and replaced through a new election.

I also implemented baseline comparison systems from distributed coordination literature. These baselines include leader-election methods that respond to unresponsive leaders but do not proactively evaluate whether an active leader is sending harmful or low-quality commands.

## 3. Experimental Arena

I constructed an indoor experimental arena (2000 × 1500 mm) using 38 mm EPS foamboard to represent a constrained search-and-rescue-style environment. The arena includes:
- Boundary walls covered with masking tape to reduce ToF sensor reflections
- A central divider (1000 mm from left edge, 1300 mm tall) creating a two-room layout
- Four ArUco corner markers (IDs 40-43, 80mm) for camera-based localization
- Consistent layout maintained across all comparison trials

A Logitech C922 webcam positioned approximately 2040 mm above the arena center provides overhead imaging. An OpenCV-based script running on a laptop performs real-time homography transformation, per-robot pose tracking, and live visualization overlay. The script broadcasts each robot's position via UDP to that robot only (no cross-robot position sharing).

## 4. Experimental Design

### Mission Objective
The mission objective for each trial is to maximize area coverage over time. At trial start, robots are placed in selected starting positions, initialized under the same software condition, and started with the same exploration objective. The robots then explore the arena autonomously.

### Experimental Conditions
Trials are organized into clearly defined conditions:

**Clean (No-Fault) Condition**: The leader behaves normally. Purpose: measure normal coordination, coverage efficiency, leader stability, and false-positive impeachment rates during standard operation.

**Fault-Injection Conditions**: The leader remains active but intentionally issues faulty or mission-degrading commands. Fault types include:
- Directing followers toward densely explored regions
- Repeatedly changing assignments in an unstable manner
- Assigning robots into unsafe peer conflicts
- Freezing assignments (ceasing to update target assignments)
- Other behaviors that reduce team effectiveness while appearing responsive

**Baseline Comparison**: Matching trials using conventional leader-election policies under the same arena layouts, robot counts, starting conditions, and fault scenarios.

### Trial Structure
For each condition, I perform repeated runs (multiple trials) to observe variability and determine whether any REIP advantage is consistent. This repeated-trial structure ensures results are not based on single outcomes.

## 5. Data Collection

I collect time-stamped data during every trial through robot-generated logs and supporting experimental software. Logged data includes:
- Robot positions and heading values
- Assigned targets and predicted targets
- Leader identity and election outcomes
- Trust values and suspicion values
- Votes and impeachment events
- Coverage progress over time
- Mission timing and completion status
- Peer-emergency events and wall-emergency events
- Other safety-related behaviors

## 6. Performance Metrics

I calculate mission-performance metrics from the collected data, including:
- Total final coverage (percentage of explorable area)
- Time to reach selected coverage thresholds (e.g., 50%, 60%, 80%)
- Leader settle time (time to stable leadership)
- Time to trust decay (time from fault injection to first trust drop)
- Time to impeachment (time from fault injection to leader removal)
- Mission completion time (where applicable)
- Number of false impeachments (in clean trials)
- Minimum robot-to-robot separation
- Minimum wall clearance
- Emergency-event counts

## 7. Simulation Validation

I developed a hardware-faithful simulation environment that reproduces the same distributed robot logic, communication structure, and realistic impairments including:
- Sensing noise
- Communication delay and packet loss
- Packet reordering
- Pose uncertainty

Before using simulation results to support broader claims, I validate selected simulation runs against hardware runs under matching clean conditions. I compare metrics such as coverage, timing, safety events, and leadership behavior to determine whether the simulation reproduces real hardware behavior closely enough for controlled testing.

## 8. Analysis

I analyze and compare REIP and baseline results under matched conditions to determine:
- Whether REIP improves resilience to subtle active leader faults
- Whether REIP preserves normal exploration performance during no-fault operation
- The consistency and statistical significance of any observed advantages

## 9. Original Work Statement

All robot software, trust logic, fault-injection logic, simulation procedures, hardware-validation procedures, data-collection methods, and analysis methods described above are my own work. I have not included algorithms, procedures, or analysis designed by mentors or outside adults as part of my own contribution.
