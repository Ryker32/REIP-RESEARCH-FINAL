# REIP — superseded snapshot

**This repository is a March 2026 snapshot and is no longer current. The code
and results here are superseded.**

The canonical repository is:

### https://github.com/Ryker32/reip-sim-public

Go there for the current simulator, the raw per-trial results, the hardware
trial logs, and `verify_from_raw.py`, which reproduces every published statistic
from the raw data in a fresh clone.

---

## Why this repository is kept

It is retained only so that existing links and citations do not break. Nothing
has been deleted, but nothing here should be used as a reference for the
system's design or its measured performance.

## What is wrong with the numbers in this snapshot

The previous README described an earlier 50×50 grid-cell simulator that was
abandoned before the paper's experiments were run. It reported figures such as
17.9% versus 19.4% coverage, an "8.1% better coverage" headline, and "10+ trials
per configuration". **None of those numbers appear in the paper, and none were
produced by the system the paper evaluates.**

The published work uses a different simulator entirely: robot node software
running as separate processes over UDP, five robots in a 2000×1500 mm multiroom
arena, N=100 trials per condition in simulation and N=5 per condition on
hardware. Anyone who lands here first would otherwise form a wrong picture of
both the method and its results.

Its analysis tooling also predates a series of corrections — to the coverage
denominator, the detection-time clock, and the hardware trial aggregation —
documented in `FINDINGS.md` in the canonical repository.

## What is still current here

One thing. The robot SSH password that was previously hardcoded in
`run_trial.py` has been removed; the script now reads
`REIP_ROBOT_SSH_PASSWORD` from the environment (see `.env.example`). That
credential was public in this repository from 2026-03-16 and has been rotated on
all five robots. It remains present in this repository's git history.
