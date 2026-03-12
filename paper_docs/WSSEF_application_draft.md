# WSSEF Application Draft

## Project Title Ideas

- `REIP: A Trust-Based Governance Framework for Fault-Resilient Multi-Robot Search Teams`
- `Democratic Robot Teams: Detecting and Replacing Faulty Leaders in Autonomous Search`
- `Resilient Multi-Robot Search for Time-Critical Missions Using Trust-Based Leader Impeachment`

## Positioning Note

This project is motivated by avalanche and wilderness search-and-rescue, where every minute matters and communication can be limited by terrain. The current work does **not** claim to be a finished avalanche-rescue deployment. Instead, it focuses on one core enabling problem for future autonomous rescue systems: how a low-cost robot team can maintain coordinated search performance when a leader becomes faulty or compromised.

That framing is strong because it is both ambitious and honest:

- `Current contribution`: resilient autonomous multi-robot search coordination
- `Motivating application`: avalanche search-and-rescue
- `Future extension`: victim-detection payloads and UAV-assisted deployment

## Rationale

Avalanche and wilderness search-and-rescue missions are highly time-critical, and survival probability can decrease sharply as rescue is delayed. In mountainous terrain, communication can be unreliable, aerial rescue assets are limited, and search teams often face dangerous conditions that slow coverage. These constraints make avalanche rescue a compelling application for autonomous robot teams that can search hazardous terrain quickly without requiring constant human supervision.

Multi-robot systems could help cover avalanche debris or unstable environments faster than responders on foot while reducing rescuer exposure to danger. However, in many coordinated robot teams, a leader directs the actions of the rest of the group. This improves efficiency, but it also creates a major vulnerability: if the leader becomes faulty or compromised, the rest of the team may continue following poor commands, wasting critical time and reducing mission effectiveness.

Existing distributed protocols such as Raft can detect a leader that stops responding, but they cannot reliably detect a leader that remains active while sending harmful or low-quality commands. My project investigates whether a low-cost robot team can detect this type of subtle leadership failure using only local sensing and peer communication, without centralized supervision or access to a ground-truth map.

To address this problem, I developed the Resilient Election and Impeachment Policy (REIP), a lightweight trust-based governance framework in which follower robots verify leader commands before execution. When commands conflict with local evidence, suspicion accumulates in a trust ledger. If trust in the leader falls below a threshold, the robots can vote to impeach that leader and elect a replacement. This work develops an important autonomy layer for future search-and-rescue robot teams operating in communication-limited, time-critical, and safety-critical environments. In future systems, this governance framework could be combined with victim-detection payloads and rapid UAV-assisted deployment to support avalanche rescue operations.

## Research Question / Hypothesis / Engineering Goals / Expected Outcomes

### Research Question

Can a low-cost multi-robot team detect, remove, and replace a faulty leader using only local sensing and peer communication, while maintaining strong search coverage performance?

### Engineering Goals

1. Design a distributed governance framework that allows follower robots to verify leader commands before execution using local evidence.
2. Enable a robot team to impeach and replace a faulty leader without centralized supervision or access to a ground-truth global map.
3. Compare REIP against a decentralized controller and a heartbeat-based Raft baseline under both normal and adversarial conditions.
4. Measure whether REIP preserves coverage and fault resilience better than the baseline methods.

### Expected Outcomes

I expect REIP to maintain high final coverage under both clean and faulty conditions while outperforming a heartbeat-based Raft baseline under leader-fault scenarios. I also expect REIP to detect harmful leaders faster than reactive approaches because it evaluates commands before execution rather than waiting for damage to accumulate. Finally, I expect the results to show that trust-based governance can preserve the efficiency advantages of leader-follower coordination while improving resilience in time-critical search tasks.

## Procedures

1. Define the multi-room search environment used for evaluation, including arena dimensions, interior wall layout, passage geometry, cell size, and robot starting conditions.
2. Implement the REIP governance framework on a leader-follower frontier-exploration stack.
3. Implement comparison controllers: `REIP`, `Raft baseline`, and `decentralized baseline`.
4. Configure a five-robot experimental setup in simulation using the same robot-controller software stack as the physical platform whenever possible.
5. Model robot motion, wall interactions, peer interactions, localization updates, and communication delays/loss using the simulator.
6. Run clean trials with no leader fault to measure baseline search performance.
7. Run adversarial/fault trials in which the leader issues harmful or degraded assignments while remaining active.
8. Use at least the following fault conditions: `none`, `bad_leader`, and `freeze_leader`.
9. For each condition, run repeated trials with controlled random seeds so that controller comparisons use matched conditions.
10. Record data from each trial, including final coverage, coverage over time, time to first suspicion, time to impeachment, number of leader changes, and safety-related spacing metrics.
11. Run additional ablation trials removing key components of REIP, such as the trust model, to test which mechanisms are necessary.
12. Use a five-robot embedded hardware platform to run physical experiments that demonstrate feasibility on low-cost robots and provide grounding data for comparison with simulation behavior.
13. Compare controller performance across conditions to determine whether REIP improves resilience while maintaining useful search performance.
14. Interpret the results in the context of time-critical search tasks such as avalanche rescue, where coverage loss and coordination failure can significantly reduce mission effectiveness.

## Risk and Safety

1. The project uses mobile wheeled robots that could collide with people, walls, or each other during testing.
2. To reduce this risk, all testing will be performed in a controlled enclosed arena with a clear perimeter and no bystanders inside the test space.
3. Robots are low-mass, low-speed, battery-powered systems using low-voltage electronics.
4. Emergency stop procedures will be available during hardware testing, including manual shutdown and software stop commands.
5. Batteries and electronics will be inspected before operation to reduce electrical or overheating risk.
6. Soldering, wiring, and hardware assembly will be completed using standard lab safety practices, including eye protection and adult supervision when appropriate.
7. No human subjects, vertebrate animals, hazardous biological agents, or hazardous chemicals are involved in the research.
8. Avalanche terrain is part of the project motivation, but no live avalanche-field experimentation is part of this project.

## Data Analysis

1. Calculate final coverage percentage for each controller and condition.
2. Plot coverage-versus-time curves to compare search speed and recovery after faults.
3. Measure time to first suspicion and time to impeachment for REIP under leader-fault conditions.
4. Compute mean, median, and spread across repeated trials for major performance metrics.
5. Compare REIP to the Raft baseline and decentralized baseline using paired trial results when possible.
6. Evaluate resilience by comparing clean-condition coverage to fault-condition coverage for each controller.
7. Analyze ablation results to determine which REIP components most strongly affect performance.
8. Use tables and graphs such as final coverage bar charts, coverage-over-time plots, fault-detection timing plots, and controller comparison tables.
9. For hardware-supported results, compare physical runs to simulation on selected safety and throughput metrics to assess whether the simulation captures important real-world behavior.
10. Determine whether REIP satisfies the engineering goal of maintaining high search performance while detecting and replacing faulty leaders.

## WSSEF Abstract Draft

Leader-follower robot teams can search hazardous environments faster than individual robots, but they are vulnerable if the leader becomes faulty or compromised. This is especially dangerous in time-critical missions such as avalanche search-and-rescue, where delays reduce mission effectiveness. This project investigated whether a low-cost robot team could detect, remove, and replace a faulty leader without centralized supervision or access to a ground-truth map.

To address this problem, I developed the Resilient Election and Impeachment Policy (REIP), a trust-based governance framework for multi-robot teams. In REIP, follower robots verify leader commands before execution using three levels of local evidence: personal visit history, sensor readings, and peer-reported coverage. When commands conflict with local evidence, suspicion accumulates in a trust ledger. If trust falls below a threshold, the robots vote to impeach the leader and elect a replacement. REIP was evaluated in a multi-room simulation across 100 trials per condition against decentralized control and a heartbeat-based Raft baseline, and supported by experiments on a five-robot low-cost embedded hardware platform.

REIP achieved 100\% median coverage under both clean and adversarial simulation conditions, with median first suspicion in 0.20-0.21 seconds and median impeachment in 1.21-1.67 seconds. REIP outperformed the heartbeat-based baseline by 24.5 percentage points under Byzantine assignment faults and 32.2 percentage points under stale-assignment faults. Removing the trust model reduced coverage to 39.7\%, showing that trust-based command verification was the core mechanism behind the system's resilience.

These results show that proactive trust-based governance can make coordinated robot teams substantially more resilient to faulty or adversarial leaders in hazardous, time-critical missions.

## Judge-Facing Two-Sentence Summary

This project is motivated by avalanche search-and-rescue, where every minute matters and robot teams must keep searching even if coordination fails. I developed a trust-based governance system called REIP that allows robots to detect, impeach, and replace a faulty leader without centralized supervision, improving resilience while preserving coordinated search performance.

## Personal Motivation Note

Being genuinely invested in helping avalanche victims is a strength, not a problem. The key is to present that motivation as the reason you chose the problem, while keeping your scientific claims focused on what the current project actually demonstrates. A good rule is:

- `motivation`: avalanche rescue, limited communication, expensive aerial assets, urgency
- `current contribution`: resilient autonomous multi-robot search coordination
- `future vision`: victim-detection payloads, RECCO-like sensing, UAV-assisted deployment

That structure makes the project feel authentic, ambitious, and honest at the same time.

## Future Work Paragraph

Future extensions of this work could integrate victim-detection payloads, including RECCO-like or analogous sensing systems, so that robot teams search not only for coverage completion but also for buried-signal detection. A longer-term vision is UAV-assisted deployment of low-cost ground robots across hazardous terrain. With aerial transport and autonomous docking, robot teams could begin coordinated search earlier than traditional rescue logistics alone, which is especially important in avalanche scenarios where survival probability decreases rapidly over time.

## Talking Points For Judges

### What problem are you solving?

Leader-follower robot teams are efficient, but they are vulnerable if the leader becomes faulty while still communicating. My project asks whether the followers can detect that subtle failure and democratically replace the leader without a human supervisor or a ground-truth map.

### Why does it matter?

In avalanche rescue and other hazardous missions, delay is costly. If a robot team wastes time following a bad leader, search quality drops exactly when reliability matters most.

### What is novel here?

Instead of detecting faults only after damage is done, REIP checks leader commands before execution using local evidence ranked by reliability. That allows the team to respond proactively rather than reactively.

### Why not just use Raft?

Raft can detect a leader that stops responding, but not a leader that is still alive and sending bad commands. My project focuses on that more subtle and more dangerous failure mode.

### Is this a deployed avalanche rescue system?

Not yet. The current project develops the coordination and resilience layer needed for future autonomous rescue systems. Avalanche search-and-rescue is the motivating application, and future work could add victim-detection payloads and UAV-assisted deployment.

## Bibliography Starter List

1. Ongaro, D., and Ousterhout, J. “In Search of an Understandable Consensus Algorithm (Raft).”
2. Yamauchi, B. “A Frontier-Based Approach for Autonomous Exploration.”
3. Burgard, W. et al. “Collaborative Multi-Robot Exploration.”
4. Simmons, R. et al. “Coordination for Multi-Robot Exploration and Mapping.”
5. Lamport, L. et al. “The Byzantine Generals Problem.”
6. Castro, M., and Liskov, B. “Practical Byzantine Fault Tolerance.”
7. Marsh, S. “Formalising Trust as a Computational Concept.”
8. A source on avalanche survival probability / search urgency.
9. Garrido-Jurado, S. et al. on ArUco fiducial markers.
10. A source on disaster robotics or search-and-rescue robotics.

## Notes To Myself

- Do not claim that the current project is already a full avalanche-deployment system.
- Use avalanche SAR as the motivating application and future target domain.
- Keep current claims focused on resilient autonomous search coverage and fault-tolerant coordination.
- If asked about future directions, mention victim-detection payloads and UAV-assisted deployment.
