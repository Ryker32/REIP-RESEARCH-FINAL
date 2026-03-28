# Data Analysis - Final Version (Past Tense with Results)

## Data Analysis

I analyzed both mission-performance data and fault-response data collected from hardware and simulation trials. The primary data collected in each trial included robot position over time, assigned and predicted targets, leader identity, trust values, suspicion values, impeachment votes, impeachment times, coverage progression, mission timing, peer-emergency events, wall-emergency events, and minimum separation distances between robots and obstacles.

### Metric Calculation

From these logs, I calculated performance metrics including:
- **Coverage metrics**: Final coverage percentage, time to reach selected coverage thresholds (50%, 60%, 80%), coverage rate (cells explored per second)
- **Leadership metrics**: Leader settle time, time to first trust decay, time to leader impeachment, number of leader changes
- **Fault detection metrics**: Detection rate (percentage of fault trials where impeachment occurred), false-positive rate (impeachments in clean trials), time to detection
- **Safety metrics**: Peer-emergency count, wall-emergency count, minimum robot-to-robot separation, minimum wall clearance
- **Mission completion**: Mission completion time (when applicable), cumulative coverage efficiency

### Statistical Analysis

To ensure conclusions were not based on single runs, I:
- Organized trials by control policy (REIP, baseline Raft, Decentralized) and fault type (no-fault, bad-leader, freeze-leader)
- Performed repeated trials for each condition (100 trials per condition for simulation, 5-10 trials per condition for hardware)
- Calculated descriptive statistics: mean, median, standard deviation, and interquartile range for each metric
- Compared conditions using appropriate statistical methods (comparing means across conditions, analyzing variance)
- Identified and reported outliers or anomalous trials that may have indicated experimental issues

### Comparative Analysis

I compared REIP against baseline policies under matched conditions to determine:
- **Exploration effectiveness**: Whether REIP maintained or improved coverage during normal operation
- **Fault resilience**: Whether REIP detected and responded to harmful active leaders faster than baselines
- **False-positive rate**: Whether REIP avoided excessive false impeachments in clean trials
- **Consistency**: Whether observed advantages were consistent across repeated trials

### Data Visualization

I used the following visualizations to present results:
- **Line graphs**: Coverage over time for each condition, showing mean with variability bands (+/-1 standard deviation or interquartile range)
- **Bar graphs**: Average performance metrics across policies and fault conditions, with error bars showing variation
- **Tables**: Summary statistics for leader-impeachment behavior, false-positive rates, timing measurements, and safety-related events
- **Event timelines**: For selected representative trials, showing when trust decay, impeachment, and leader replacement occurred relative to fault injection

### Success Criteria and Results

I interpreted REIP as effective based on the following criteria, all of which were met:

- **Normal operation**: Target was to maintain strong area coverage (>=80% final coverage) during no-fault trials, comparable to or better than baseline policies. **Result**: REIP achieved 91.2% mean coverage under clean conditions, comparable to Decentralized (90.6%) and better than Raft (86.4%).

- **Fault detection**: Target was to detect harmful active leaders within a limited number of command cycles (<2 seconds from fault injection to first suspicion, <5 seconds to impeachment). **Result**: REIP achieved median first suspicion in 0.21 seconds (bad-leader) and 0.20 seconds (freeze-leader), and median impeachment in 1.21 seconds (bad-leader) and 1.67 seconds (freeze-leader), meeting both targets.

- **Fault response**: Target was to successfully replace compromised leaders (>=90% detection rate) without excessive false positives (<1 false impeachment per clean trial). **Result**: REIP achieved 97% detection rate under bad-leader faults and 99% under freeze-leader faults, with 0.7 false positives per clean trial, meeting both targets.

- **Coverage under faults**: Target was to maintain >=80% final coverage under fault conditions. **Result**: REIP achieved 90.9% mean coverage under bad-leader faults and 96.1% mean coverage under freeze-leader faults, exceeding the target.

- **Safety**: Target was to maintain minimum separation distances and avoid excessive emergency events. **Result**: REIP maintained safe robot-to-robot separation and wall clearance throughout all trials, with no excessive emergency events.

### Resilience Metrics

I calculated two formal summary metrics to quantify system resilience:

- **Resilience gap** (Equation 5): The signed difference between clean and faulty coverage for each controller. REIP's median resilience gap was 0.0 percentage points across both fault types (100% in clean, bad-leader, and freeze-leader conditions), demonstrating fault-invariant performance. At the mean, REIP's resilience gap was only -0.3 percentage points (bad-leader) and +4.9 percentage points (freeze-leader), showing minimal performance degradation. In contrast, Raft's worst-case resilience gap was -39.1 percentage points at the median under freeze-leader, reflecting permanent coverage collapse after fault injection.

- **Cumulative coverage efficiency** (Equation 6): The area under the coverage-vs-time curve normalized by trial duration, capturing both convergence speed and final coverage. REIP's efficiency was effectively invariant to the bad-leader fault: 74.8% under attack versus 76.7% clean (-1.9 percentage points). In contrast, Raft lost 11.2 percentage points (70.6% -> 59.4%) because followers spent the majority of the trial chasing explored cells after fault injection.

### Ablation Study

To isolate the contribution of each REIP component, I conducted an ablation study under the bad-leader fault with N=30 trials per variant. Three variants were tested alongside REIP-Full:

- **No Trust**: Trust assessment and impeachment disabled; elections still occur but there is no detection mechanism. **Result**: Coverage collapsed to 39.7% mean with 0% detection rate--worse than Raft's 66.4%--demonstrating that trust is the core mechanism, not an optional enhancement.

- **No Direction**: Direction consistency check disabled; detection relies solely on the three-tier cell-level verification. **Result**: Achieved 86.0% coverage, a 4.9 percentage point penalty versus REIP-Full, showing that the direction check provides supplementary evidence that accelerates suspicion accumulation.

- **No Causality**: Causality grace period set to zero; any cell appearing in the visited set triggers immediate suspicion regardless of when it was explored. **Result**: Achieved 96.1% coverage under bad-leader (higher than REIP-Full's 90.9%) but at 8.1 false positives per clean trial (vs. 0.7 for REIP-Full), rendering it operationally unusable. This demonstrates that causality-aware verification is essential for deployability.

### Simulation Validation

I compared selected simulation results against hardware results under matching clean conditions to determine whether the simulation reproduced hardware behavior closely enough to support additional controlled experimentation. Validation compared:
- Coverage progression over time
- Robot speeds and movement patterns
- Leadership behavior and election timing
- Safety metrics and emergency events

Simulation and hardware results showed consistent relative behavior: REIP outperformed baseline in both environments. Hardware REIP achieved 70% of the simulation coverage rate (1.02 vs. 1.46 cells/s) despite real-world noise, latency, and imperfect localization. The relative REIP advantage over Raft was preserved and amplified on hardware (+30.2 percentage points on hardware versus +4.8 percentage points in clean simulation), confirming that the simulation results are valid for supporting broader statistical claims from larger trial counts.
