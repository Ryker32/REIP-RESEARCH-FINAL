# Simulation-Hardware Validation Guide

**Goal**: Validate that simulation accurately represents hardware behavior, enabling strong claims from 100-trial multiroom experiments.

## How It Works

The `isef_experiments.py` system:
1. **Runs real `reip_node.py` scripts** via subprocess (same code as hardware)
2. **Simulates physics** (movement, collisions, sensors) via the harness
3. **Supports validation mode** to compare sim vs hardware results

## Validation Workflow

### Step 1: Capture Hardware Starting Positions

```bash
# Run ArUco detection to get exact robot positions
python test/test_aruco_distances.py
# Press 's' to snapshot, 'q' to quit
# Saves to JSON with `sim_fixed_positions` field
```

### Step 2: Run Hardware Trial

Run your hardware experiment and save results to JSON (same format as sim results).

### Step 3: Run Simulation with Hardware Positions

```bash
# Run sim with exact hardware starting positions
python test/isef_experiments.py \
    --layout multiroom \
    --trials 1 \
    --hardware-positions <aruco_positions.json> \
    --condition BadLeader
```

This runs simulation using the **exact same starting positions** as hardware.

### Step 4: Compare Results

```bash
# Compare sim results to hardware results
python test/isef_experiments.py \
    --validate <hardware_results.json>
```

This will:
- Load hardware results
- Compare key metrics (coverage, detection time, suspicion time)
- Report pass/fail with tolerances:
  - Coverage: +/-10% difference
  - Detection time: +/-0.5s difference
  - Suspicion time: +/-0.3s difference

### Step 5: If Validation Passes

Once validation passes, you can confidently use 100-trial sim results:

```bash
# Run full 100-trial suite
python test/isef_experiments.py --layout multiroom --trials 100

# Generate figures
python test/_generate_paper_figures.py
```

## Key Features

### Fixed Starting Positions

The `--hardware-positions` flag loads positions from ArUco detection JSON:
- Uses `sim_fixed_positions` field if present
- Falls back to `robot_positions` (converts heading from degrees to radians)
- Only applies to first trial (for validation), or all trials if needed

### Validation Mode

The `--validate` flag compares sim vs hardware:
- Loads hardware results from JSON
- Groups by condition (controller * fault type)
- Computes mean differences
- Reports pass/fail with clear tolerances

## Example Output

```
================================================================================
SIMULATION vs HARDWARE VALIDATION
================================================================================

  OK PASS reip|bad_leader:
    Coverage: Sim 91.2% vs HW 89.5% (diff: 1.7%)
    Detection: Sim 1.21s vs HW 1.35s (diff: 0.14s)
    Suspicion: Sim 0.20s vs HW 0.22s (diff: 0.02s)

  OK PASS reip|none:
    Coverage: Sim 92.1% vs HW 90.8% (diff: 1.3%)
    Detection: N/A (no faults)
    Suspicion: N/A (no faults)

================================================================================
VALIDATION PASSED: Simulation matches hardware within tolerances
  -> Safe to use 100-trial sim results for paper
================================================================================
```

## Critical Metrics to Validate

1. **Final Coverage**: Should match within 5-10%
2. **Detection Time**: Should match within 0.2-0.5s
3. **Suspicion Time**: Should match within 0.2-0.3s
4. **Number of Elections**: Should be similar

## Red Flags (Fix Before Using Results)

- Coverage differs by >10%
- Detection time differs by >0.5s
- Hardware robots crash/get stuck constantly
- Hardware elections never happen
- Hardware false positives >1 per trial

## Why This Matters

For ISEF, you need to show that:
1. **Simulation is accurate**: Validation proves sim matches hardware
2. **100-trial results are valid**: Once validated, large-scale sim results are trustworthy
3. **Strong statistical claims**: Can confidently claim "100 trials show X" knowing sim is accurate

This validation enables you to make **substantial claims** from the larger multiroom experiments while maintaining scientific rigor.
