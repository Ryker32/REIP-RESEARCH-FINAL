# REIP Supplemental Repository

**Trust-Based Governance for Fault-Resilient Multi-Agent Frontier Exploration: The REIP Framework**

William R. Kollmyer — Olympia High School, Olympia, WA

---

## How to Reproduce Every Figure and Table

### Prerequisites

- Python 3.10+
- `pip install matplotlib numpy pyyaml`

### Step-by-step

1. **Reproduce N=30 multiroom experiments (270 trials):**
   ```bash
   python scripts/isef_experiments.py --layout multiroom --trials 30 --workers 4 --condition all
   ```
   Seeds are loaded from `seeds/seeds.json`. Results are written to `raw_data/multiroom_n30/`.

2. **Generate statistical tables (Tables IV–VII in paper):**
   ```bash
   python scripts/_full_analysis.py raw_data/multiroom_n30/
   ```

3. **Generate convergence and resilience contrast charts (Figs. 3–4):**
   ```bash
   python scripts/_convergence_chart.py raw_data/multiroom_n30/
   ```

4. **Generate vector gallery snapshots (Fig. 5, supplemental gallery):**
   ```bash
   # First run a single trial with snapshots enabled:
   python scripts/isef_experiments.py --layout multiroom --trials 1 --workers 1 --snapshots --condition all
   # Then render the gallery:
   python scripts/_gridworld_snapshots.py <snapshot_dir> --gallery
   ```

5. **Reproduce GridWorld 2,000-trial results (Section V):**
   The original GridWorld simulation data is in `raw_data/gridworld_2000/`.

---

## Repository Structure

```
REIP_Supplemental/
├── raw_data/
│   ├── multiroom_n30/          ← 270-trial results (N=30 × 9 conditions)
│   │   └── logs/               ← per-trial coverage timelines + robot logs
│   ├── snapshots/              ← vector snapshot JSONs for visualization
│   └── gridworld_2000/         ← original 2,000-seed GridWorld sim data
├── seeds/
│   └── seeds.json              ← exact seeds for all 30 trials (1000–30000, step 1000)
├── scripts/
│   ├── isef_experiments.py     ← main experiment harness (parallel execution)
│   ├── _convergence_chart.py   ← IEEE-standard convergence plots
│   ├── _full_analysis.py       ← statistical summary tables (mean, median, IQR)
│   └── _gridworld_snapshots.py ← gridworld vector gallery generator
├── figures/                    ← all charts as PNG + PDF (print-ready)
├── hardware/
│   ├── bom.csv                 ← bill of materials with costs
│   ├── wiring_diagram.png      ← system wiring schematic
│   ├── aruco_markers/          ← printable ArUco marker PDFs
│   │   ├── robot_markers/      ← IDs 0–4 (one per robot)
│   │   └── corner_markers/     ← IDs 10–13 (arena corners for homography)
│   └── cad/                    ← chassis STL files
├── paper/
│   ├── IEEE-conference-template-062824.tex
│   ├── IEEE-conference-template-062824.pdf
│   └── figures/                ← figures referenced by LaTeX
└── README.md                   ← this file
```

## Seeds

The 30 seeds used for all multiroom experiments:

```
[1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000,
 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000,
 21000, 22000, 23000, 24000, 25000, 26000, 27000, 28000, 29000, 30000]
```

These same seeds are used for hardware trials to enable direct sim-to-real comparison.

## ArUco Marker Layout

| Marker ID | Assignment          | Position              |
|-----------|--------------------|-----------------------|
| 0         | Robot 1            | Affixed to robot top  |
| 1         | Robot 2            | Affixed to robot top  |
| 2         | Robot 3            | Affixed to robot top  |
| 3         | Robot 4            | Affixed to robot top  |
| 4         | Robot 5            | Affixed to robot top  |
| 10        | Arena corner (TL)  | (0, 0) mm             |
| 11        | Arena corner (TR)  | (2000, 0) mm          |
| 12        | Arena corner (BL)  | (0, 1500) mm          |
| 13        | Arena corner (BR)  | (2000, 1500) mm       |

Marker size: 50 mm × 50 mm. Dictionary: `DICT_4X4_50`.

The overhead camera (Logitech C922 at 1080p, 30 fps) detects all markers each frame. The four corner markers compute a homography transform that maps pixel coordinates to arena coordinates in millimeters. Robot markers are then localized in arena space via this homography.

## Experiment Conditions

| Condition   | Controller     | Fault Type   | Fault Time(s) |
|-------------|---------------|-------------|----------------|
| NoFault     | REIP/Raft/Dec | None        | —              |
| BadLeader   | REIP/Raft/Dec | bad_leader  | t=10s, t=30s   |
| Spin        | REIP/Raft/Dec | motor spin  | t=10s          |

**Dual sequential fault injection**: Under BadLeader, Robot 1 is compromised at t=10s. At t=30s, a second bad_leader fault targets whoever is currently the leader (testing repeated-attack survival).

## Key Results

- **REIP under bad-leader**: 90.7% mean / 100% median coverage, 0.28±0.16s detection
- **Raft under bad-leader**: 66.5% mean / 68.8% median coverage, 0/30 detection
- **Resilience gap**: REIP median coverage is invariant to fault condition (100% clean = 100% bad-leader)

## Contact

rykerkollmyer@gmail.com
