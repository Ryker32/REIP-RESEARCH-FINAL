# REIP System Description (Non-MAS Audience)

## What problem this system solves
When multiple robots explore an unknown space, they must coordinate who goes where. A common approach is to pick a single leader that assigns tasks to the rest of the team. This is efficient, but it creates a single point of failure: if the leader is compromised or simply wrong, the whole team can waste time or become unsafe.

REIP (Resilient Election and Impeachment Policy) is a coordination system that keeps the leader model (for efficiency) but adds a governance layer so the team can detect bad leadership and replace the leader when needed.

## One-paragraph summary
REIP is a leader-follower exploration system where each robot continuously evaluates the leader's decisions using trust signals. If trust drops below a threshold, the team "impeaches" the leader and elects a new one. A newer component adds a lightweight model-based check (MPC-style) so each follower can compare the leader's assignment to a locally optimal alternative. This hybrid trust mechanism improves resilience against adversarial commands without requiring full decentralized planning.

## System components (plain language)
- **Environment (GridWorld)**: A simulated map with unknown cells and obstacles.
- **Agents**: Robots that sense nearby space, move toward goals, and share maps.
- **Leader**: The agent that assigns tasks (frontiers) to the rest of the team.
- **Followers**: Agents that carry out assignments but also evaluate the leader's quality.
- **Trust system**: Each follower tracks how reliable the leader has been.
- **Governance**: If trust drops too far, the leader is removed and a new leader is chosen.

## How the system works (step-by-step)
1. Each agent senses nearby cells and updates its local map.
2. Agents share low-resolution map updates with nearby teammates.
3. The leader assigns each agent a target frontier (a boundary between known and unknown space).
4. Each follower evaluates the leader's assignment with two trust signals:
   - **Prediction-observation trust**: "Did the leader's plan deliver the expected gain?"
   - **MPC-based trust**: "Is the leader's assignment much worse than my local best option?"
5. If trust drops below a threshold, an impeachment vote is triggered.
6. A new leader is elected (usually the agent with highest average trust).
7. Exploration continues with the new leader.

## The MPC-based trust check (why it matters)
The original trust system only punished leaders when they promised gains that did not happen. This can fail if the leader never "claims" anything big. The MPC check fixes this by letting each follower compute its own local best target and compare it to the leader's assignment. If the leader keeps sending followers to obviously bad areas, trust decays even without inflated promises.

This makes the system more resilient to adversarial commands without fully decentralizing the planning process.

## Faults and adversarial attacks
The simulator can inject different attack patterns, for example:
- **Dense explored**: Leader sends agents to already explored areas.
- **Corner trap**: Leader sends agents to a dead end.
- **Byzantine**: Leader sends random or contradictory commands.

These attacks allow side-by-side comparisons of:
- baseline leader-follower (no governance),
- REIP with prediction-observation trust,
- REIP with hybrid MPC trust.

## What gets measured
Common metrics include:
- **Coverage**: how much of the map is explored.
- **Time to recovery**: how fast the team resumes exploration after an attack.
- **Trust decay and impeachment count**: how quickly bad leaders are detected.
- **Resilience**: performance under attack vs. no-attack conditions.

## Why this is different from standard multi-robot systems
Most leader-follower systems assume the leader is always reliable. REIP does not. It treats leadership as a role that must be continuously earned, which makes the system robust to faults and adversarial behavior.

## Practical constraints and efficiency
REIP is designed to remain lightweight enough for real robots where compute is limited. The MPC trust check uses coarse maps and local windows rather than full global optimization, keeping overhead low.

## Limitations (honest summary)
- This is still a grid-based simulator; real robots have more uncertainty.
- The MPC trust check uses a simplified utility score (unknown coverage).
- Shared maps are coarse; information fusion can be improved with probabilistic methods.

## Where this could go next
The same governance and trust ideas can be extended to heterogeneous teams (rovers, arms, aerial robots) or more realistic continuous environments, with the leader acting as a coordination hub while followers remain capable of local verification.
