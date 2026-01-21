# REIP Multi-Agent Exploration System

A research project implementing and evaluating **REIP** (Trust-based Election/Impeachment Protocol) for resilient multi-agent exploration under adversarial conditions.

## 🔬 Research Overview

This project compares REIP against traditional fixed-leader systems in multi-agent exploration scenarios with:
- **Communication faults** (packet loss, corruption, limited range)
- **Byzantine behavior** (hallucinations, malicious actions)
- **Dynamic leadership** through trust-based governance

**Key Finding**: REIP demonstrates **8.1% better coverage** under extreme adversarial conditions (8% hallucination rate, 20% command loss).

## 📁 Repository Structure

```
├── docs/                          # 📚 Research Documentation
│   ├── BENCHMARK_SUMMARY.md       # Statistical analysis results
│   ├── IMPLEMENTATION_COMPLETE.md # Complete feature list
│   ├── CITATIONS.md               # Academic references
│   ├── REAL_WORLD_BENCHMARKS.md   # Real-world validation
│   ├── IMPACT_STRATEGY.md         # Research impact plan
│   ├── FUTURE_EXPERIMENTS.md      # Next research directions
│   ├── T1000_BENCHMARK.md         # Large-scale experiments
│   ├── DEBUGGING_ROOT_CAUSE.md    # Technical debugging notes
│   └── references.bib             # Bibliography
├── src/                           # 🔧 Core Implementation
│   ├── main.py                    # Main simulation entry point
│   ├── agents/                    # Agent implementations
│   ├── policy/                    # REIP and baseline policies
│   ├── comms/                     # Communication/fault models
│   ├── env/                       # Grid environment
│   └── utils/                     # Utility functions
├── configs/                       # ⚙️ Experiment Configurations
│   ├── reip_*.yaml               # REIP system configs
│   └── baseline_*.yaml           # Baseline system configs
├── scripts/                       # 📊 Analysis Scripts
│   ├── compare_reip_vs_baseline.py
│   ├── collect_metrics.py
│   └── plot_metrics.py
├── results/                       # 📈 Experimental Results
│   ├── benchmark_results_*.csv    # Raw benchmark data
│   ├── optimal_benchmark_*.csv    # Optimized runs
│   └── *.png                     # Visualization plots
├── archive/                       # 📦 Legacy/Development Files
│   ├── run_benchmark_*.py        # Old benchmark scripts
│   └── validate_*.py             # Validation utilities
├── runs/                         # 🏃 Simulation Output
└── tools/                        # 🛠️ Development Tools
```

## 🚀 Quick Start

### Running Core Simulations
```bash
# Basic REIP vs Baseline comparison
python src/main.py --config configs/reip_adversarial.yaml
python src/main.py --config configs/baseline_adversarial.yaml

# Visual simulation with real-time plotting
python run_with_viz.py
python visualize_sim.py
```

### Generating Research Results
```bash
# Statistical comparison analysis
python scripts/compare_reip_vs_baseline.py

# Collect performance metrics
python scripts/collect_metrics.py

# Generate publication plots
python scripts/plot_metrics.py
```

## 📊 Key Results

| Metric | Baseline | REIP | Improvement |
|--------|----------|------|-------------|
| **Coverage** | 17.9% ± 2.1% | **19.4% ± 3.3%** | **+8.1%** |
| **Success Rate** | 40% | **60%** | **+50%** |
| **Elections** | 0 | 28.9 ± 0.7 | Adaptive |

*Under extreme adversarial conditions: 8% hallucination, 20% command loss*

## 🔗 Academic Context

### Core Technologies
- **Trust-based Governance**: Dynamic leader election/impeachment
- **Byzantine Fault Tolerance**: Resilience to malicious agents
- **Communication Fault Models**: Realistic network conditions
- **Comparative Analysis**: Statistical validation vs baselines

### Citations
39 academic sources documented in `docs/CITATIONS.md`, including:
- Lamport et al. (Byzantine consensus)
- García-Camino (distributed election)
- Amodei et al. (AI safety/hallucinations)
- Simmons et al. (centralized coordination)

## 📈 Research Impact

This work addresses critical challenges in:
- **Autonomous vehicle coordination** under GPS jamming
- **Drone swarm exploration** with communication interference
- **Robot team resilience** against adversarial attacks
- **Multi-agent AI safety** for real-world deployment

## 🔬 Experimental Validation

- **Statistical rigor**: 10+ trials per configuration
- **Realistic faults**: Based on empirical network studies
- **Comparative methodology**: Direct baseline comparison
- **Reproducible results**: Documented configurations and seeds

## 🎯 Future Directions

See `docs/FUTURE_EXPERIMENTS.md` for detailed research roadmap including:
- Heterogeneous agent capabilities
- Dynamic environment adaptation
- Real-world hardware validation
- Scalability to 100+ agents

---

**Status**: Implementation Complete ✅ | Statistical Validation Complete ✅ | Ready for Publication 📄
 
## 🧭 Initialization-with-Command and Spawn Configuration

To prevent early stagnation when commands are lost, the REIP controller seeds every agent with an initial goal at t=0 and uses a persistent hold mechanism:

- env.min_hold: Number of timesteps an agent keeps its current goal before being eligible for reassignment (default 3; configurable at top level or in env).
- Agent.hold_target / hold_timer: Per-agent fields that persist the current goal across ticks.
- reip.command_loss_rate: Probability a leader command is dropped (1.0 = all commands dropped).
- reip.command_radius: Maximum range for commands from the leader.

This works with belief-based navigation; even with full command loss, agents keep moving toward their held goals for at least min_hold steps.

Spawn clustering options help create consistent early exploration dynamics:

- env.spawn_box: [x0, y0, x1, y1] restricts initial positions to a box.
- env.spawn_center: [x, y] and env.spawn_radius: r restrict initial positions to a Chebyshev-radius box.

Example YAML snippet:

```yaml
N: 6
map_size: 30
min_hold: 5
env:
	obstacle_density: 0.05
	seed: 123
	spawn_box: [2, 2, 8, 8]
agent:
	r_local: 8
	downsample: 4
reip:
	command_loss_rate: 1.0   # drop all commands to test persistence
	command_radius: 10
	trust_decay_rate: 0.5
	trust_threshold: 0.6
	min_trust: 0.1
```

Run a quick simulation with visualization:

```powershell
python -m src.main --config configs\reip_true.yaml --visualize
```

Run the unit test that verifies persistent movement under full command loss:

```powershell
python tests\test_init_with_command.py
```

See also docs/CONFIG_SPAWN_AND_COMMANDS.md for more examples.

## 🧭 Connectivity-aware assignment and regional diversity

To maintain a relay backbone and reduce clustering, the frontier assignment can:

- Reward staying within command/comm range of at least two teammates.
- Limit how many agents can be assigned to the same coarse region per tick.

Key knobs (under `reip:`):

- `connectivity_beta` (float): Strength of the bonus for being within `command_radius` of teammates (0.0 disables).
- `region_size` (int, optional): Coarse grid size used to bucket frontiers into regions; if omitted, it auto-scales from `prox_radius`.
- `region_capacity` (int): Max agents assigned per region in a single tick (default 1).

Example:

```yaml
reip:
	command_radius: 15
	connectivity_beta: 0.8
	prox_radius: 6
	region_size: 8
	region_capacity: 1
```

Tip: Align `command_radius` with your communication radius for consistent behavior when using connectivity bonuses.

## 🧲 Motion-level deconfliction knobs

The environment applies soft deconfliction to reduce criss-crossing and oscillations:

- `env.neighbor_proximity_radius` (int): Penalize steps that move close to nearby agents within this Chebyshev radius.
- `env.neighbor_repulsion` (float): Strength of proximity repulsion (added cost scales as 1/distance).
- `env.reservation_penalty` (float): Extra cost for moving into a cell reserved earlier that tick by another agent.

Example:

```yaml
env:
	neighbor_proximity_radius: 2
	neighbor_repulsion: 0.6
	reservation_penalty: 3.0
```

These are soft costs layered on top of A*'s move cost; they reduce path crossings without freezing agents.

## How to cite

If you use this repository in academic work, please cite it and the foundational methods. A `CITATION.cff` file is included for GitHub’s citation widget.

Example (APA):

Ryker, [Your First Name]. (2025). REIP: Resilient Election & Impeachment Policy for Multi-Agent Exploration (v0.1.0) [Computer software]. https://github.com/Ryker32/reip-sim

Key references underpinning components:

- Frontier exploration: Yamauchi, B. (1997). A Frontier-Based Approach for Autonomous Exploration. IEEE CIRA.
- Information theory: Shannon, C. E. (1948). A Mathematical Theory of Communication. Bell Syst. Tech. J.
- Pathfinding: Hart, Nilsson, Raphael (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE.
- Line-of-sight: Bresenham, J. E. (1965). Algorithm for computer control of a digital plotter. IBM Systems Journal.
- Bursty comms: Gilbert (1960); Elliott (1963). Bell System Technical Journal.
- Byzantine/fault-tolerance background: Lamport et al. (1982); Castro & Liskov (1999).

See `ATTRIBUTIONS.md` for a consolidated list.