# Data Analysis Section - Improved Version

## Data Analysis

I will analyze both mission-performance data and fault-response data collected from hardware and simulation trials. The primary data collected in each trial includes robot position over time, assigned and predicted targets, leader identity, trust values, suspicion values, impeachment votes, impeachment times, coverage progression, mission timing, peer-emergency events, wall-emergency events, and minimum separation distances between robots and obstacles.

### Metric Calculation

From these logs, I will calculate performance metrics including:
- **Coverage metrics**: Final coverage percentage, time to reach selected coverage thresholds (50%, 60%, 80%), coverage rate (cells explored per second)
- **Leadership metrics**: Leader settle time, time to first trust decay, time to leader impeachment, number of leader changes
- **Fault detection metrics**: Detection rate (percentage of fault trials where impeachment occurred), false-positive rate (impeachments in clean trials), time to detection
- **Safety metrics**: Peer-emergency count, wall-emergency count, minimum robot-to-robot separation, minimum wall clearance
- **Mission completion**: Mission completion time (when applicable), cumulative coverage efficiency

### Statistical Analysis

To ensure conclusions are not based on single runs, I will:
- Organize trials by control policy (REIP, baseline) and fault type (no-fault, bad-leader, freeze-leader)
- Perform repeated trials for each condition (target: 30-100 trials per condition for simulation, 5-10 trials per condition for hardware)
- Calculate descriptive statistics: mean, median, standard deviation, and interquartile range for each metric
- Compare conditions using appropriate statistical methods (e.g., comparing means across conditions, analyzing variance)
- Identify and report outliers or anomalous trials that may indicate experimental issues

### Comparative Analysis

I will compare REIP against baseline policies under matched conditions to determine:
- **Exploration effectiveness**: Whether REIP maintains or improves coverage during normal operation
- **Fault resilience**: Whether REIP detects and responds to harmful active leaders faster than baselines
- **False-positive rate**: Whether REIP avoids excessive false impeachments in clean trials
- **Consistency**: Whether observed advantages are consistent across repeated trials

### Data Visualization

I will use the following visualizations to present results:
- **Line graphs**: Coverage over time for each condition, showing mean with variability bands (e.g., +/-1 standard deviation or interquartile range)
- **Bar graphs**: Average performance metrics across policies and fault conditions, with error bars showing variation
- **Tables**: Summary statistics for leader-impeachment behavior, false-positive rates, timing measurements, and safety-related events
- **Event timelines**: For selected representative trials, showing when trust decay, impeachment, and leader replacement occur relative to fault injection

### Success Criteria

I will interpret REIP as effective if it demonstrates:
- **Normal operation**: Maintains strong area coverage (>=80% final coverage) during no-fault trials, comparable to or better than baseline policies
- **Fault detection**: Detects harmful active leaders within a limited number of command cycles (target: <2 seconds from fault injection to first suspicion, <5 seconds to impeachment)
- **Fault response**: Successfully replaces compromised leaders (>=90% detection rate) without excessive false positives (<1 false impeachment per clean trial)
- **Safety**: Maintains minimum separation distances and avoids excessive emergency events

### Simulation Validation

I will compare selected simulation results against hardware results under matching clean conditions to determine whether the simulation reproduces hardware behavior closely enough to support additional controlled experimentation. Validation will compare:
- Coverage progression over time
- Robot speeds and movement patterns
- Leadership behavior and election timing
- Safety metrics and emergency events

If simulation and hardware results show consistent relative behavior (e.g., REIP outperforms baseline in both), I will use simulation results to support broader statistical claims from larger trial counts.
