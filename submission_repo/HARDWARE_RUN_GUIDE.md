# Hardware Run Guide - run_trial.py

## Quick Start

### 1. Start Position Server (ONE TIME, leave running)
```bash
python pc/aruco_position_server.py
```
- Press 'r' to start/stop video recording
- Confirm all 5 robots + 4 corners detected
- **Keep this running for entire session**

### 2. Run Single Trial
```bash
python run_trial.py --controller reip --fault bad_leader
```

### 3. Kill All Robots (emergency or between trials)
```bash
python -c "from run_trial import kill_all; kill_all()"
```

---

## Fault Timing (automatic)

`run_trial.py` handles fault injection automatically with dual sequential faults:
- **t=20s** — Fault #1 injected on current leader
- **t=40s** — Fault #2 injected on current leader (may be a new leader after impeachment)
- **t=120s** — Trial ends, robots stopped, logs collected

---

## Complete Hardware Trial Matrix

### REIP Trials

```bash
# REIP clean (no fault)
python run_trial.py --controller reip --fault none --trial 1
python run_trial.py --controller reip --fault none --trial 2
python run_trial.py --controller reip --fault none --trial 3
python run_trial.py --controller reip --fault none --trial 4
python run_trial.py --controller reip --fault none --trial 5

# REIP bad_leader — *** KEY DEMO ***
python run_trial.py --controller reip --fault bad_leader --trial 1 ## trials\reip_bad_leader_t1_20260315_185118
python run_trial.py --controller reip --fault bad_leader --trial 2 ## trials\reip_bad_leader_t2_20260315_185439
python run_trial.py --controller reip --fault bad_leader --trial 3 ## trials\reip_bad_leader_t3_20260315_185801
python run_trial.py --controller reip --fault bad_leader --trial 4 ## trials\reip_bad_leader_t4_20260315_190114
python run_trial.py --controller reip --fault bad_leader --trial 5 ## trials\reip_bad_leader_t5_20260315_191258
## Best: [REC] Stopped — saved to trials\20260315_191301

# REIP freeze_leader
python run_trial.py --controller reip --fault freeze_leader --trial 1 ## trials\reip_freeze_leader_t1_20260315_191636
python run_trial.py --controller reip --fault freeze_leader --trial 2 ## trials\reip_freeze_leader_t2_20260315_192025
## video: 
python run_trial.py --controller reip --fault freeze_leader --trial 3 ##  trials\reip_freeze_leader_t3_20260315_192847
python run_trial.py --controller reip --fault freeze_leader --trial 4 ## trials\reip_freeze_leader_t4_20260315_193148
python run_trial.py --controller reip --fault freeze_leader --trial 5 ## trials\reip_freeze_leader_t5_20260315_193633

##video for run 5 trials\20260315_193637

##  best :  trials\20260315_191637

# REIP self_injure_leader
python run_trial.py --controller reip --fault self_injure_leader --trial 1 ## trials\reip_self_injure_leader_t1_20260315_194004
## video: trials\20260315_194005
python run_trial.py --controller reip --fault self_injure_leader --trial 2 ## trials\reip_self_injure_leader_t2_20260315_194349
## video : trials\20260315_194351
python run_trial.py --controller reip --fault self_injure_leader --trial 3 ## trials\reip_self_injure_leader_t3_20260315_194659
## video trials\20260315_194700
python run_trial.py --controller reip --fault self_injure_leader --trial 4 ## trials\reip_self_injure_leader_t4_20260315_195024
## video trials\20260315_195019
python run_trial.py --controller reip --fault self_injure_leader --trial 5 ## trials\reip_self_injure_leader_t5_20260315_195345
## video trials\20260315_195338
```

### Raft Trials

```bash
# Raft clean (no fault)
python run_trial.py --controller raft --fault none --trial 1 ## trials\raft_none_t1_20260315_182753
python run_trial.py --controller raft --fault none --trial 2 ## trials\raft_none_t2_20260315_195714
## vidoe: trials\20260315_195716
python run_trial.py --controller raft --fault none --trial 3 ## trials\raft_none_t3_20260315_200055
## trials\20260315_200057
python run_trial.py --controller raft --fault none --trial 4 ## trials\raft_none_t4_20260315_200445
##
python run_trial.py --controller raft --fault none --trial 5 ## trials\raft_none_t5_20260315_200835
##

# Raft bad_leader
python run_trial.py --controller raft --fault bad_leader --trial 1 ## trials\raft_bad_leader_t1_20260315_201736
## trials\20260315_201739
python run_trial.py --controller raft --fault bad_leader --trial 2 ## trials\raft_bad_leader_t2_20260315_202146
## trials\20260315_202151
python run_trial.py --controller raft --fault bad_leader --trial 3 ##t trials\raft_bad_leader_t3_20260315_202706
## trials\20260315_202708
python run_trial.py --controller raft --fault bad_leader --trial 4 ## trials\raft_bad_leader_t4_20260315_203033
## 
python run_trial.py --controller raft --fault bad_leader --trial 5 ## trials\raft_bad_leader_t5_20260315_203417
## trials\20260315_203420

##BEST
## trials\20260315_203039

# Raft freeze_leader
python run_trial.py --controller raft --fault freeze_leader --trial 1 ## trials\raft_freeze_leader_t1_20260315_203754
## trials\20260315_203756
python run_trial.py --controller raft --fault freeze_leader --trial 2 ##  trials\raft_freeze_leader_t2_20260315_204127
## 
python run_trial.py --controller raft --fault freeze_leader --trial 3 ## trials\raft_freeze_leader_t3_20260315_204418
## 
python run_trial.py --controller raft --fault freeze_leader --trial 4 ## trials\raft_freeze_leader_t4_20260315_204744
## 
python run_trial.py --controller raft --fault freeze_leader --trial 5 ## trials\raft_freeze_leader_t5_20260315_205032
## 

# Raft self_injure_leader
python run_trial.py --controller raft --fault self_injure_leader --trial 1 ## trials\raft_self_injure_leader_t1_20260315_210036
## trials\20260315_210057
python run_trial.py --controller raft --fault self_injure_leader --trial 2 ##  trials\raft_self_injure_leader_t2_20260315_210344
## 
python run_trial.py --controller raft --fault self_injure_leader --trial 3 ## trials\raft_self_injure_leader_t3_20260315_210650
## 
python run_trial.py --controller raft --fault self_injure_leader --trial 4 ## trials\raft_self_injure_leader_t4_20260315_211209
## 
python run_trial.py --controller raft --fault self_injure_leader --trial 5 ## trials\raft_self_injure_leader_t5_20260315_211523
## 
```

### Priority Order (what to run first)

1. **REIP + bad_leader x5** — the paper's key claim, currently missing from hardware table
2. **REIP + freeze_leader x4** — only have 1 trial
3. **Raft + bad_leader x5** — need the Raft-fails-under-fault comparison
4. **REIP + self_injure x2** — have 3, get to 5
5. **Raft + freeze_leader x5**
6. **Raft + self_injure x5**
7. **Raft + none x1** — have 4, get to 5

**Total: 8 conditions x 5 trials = 40 runs (~80 minutes of robot time)**

---

## What It Does Automatically

1. **Kills** old processes on all robots
2. **Clears** old logs
3. **Deploys** code to all robots (SSH upload)
4. **Launches** robots with stagger
5. **Waits** 5s for localization + leader election
6. **Starts** trial (sends start signal)
7. **Injects** fault #1 at t=20s, fault #2 at t=40s
8. **Stops** robots at 120s
9. **Collects** logs to `trials/<name>/`

## Options

- `--controller reip|raft|decentralized` - Which controller
- `--fault none|bad_leader|freeze_leader|self_injure_leader` - Fault type
- `--trial N` - Trial number (for logging)
- `--duration N` - Trial duration (default 120s)
- `--start-delay N` - Wait time after launch (default 5s)
- `--preflight` - Ping all robots before starting
- `--robots 1,2,3,4,5` - Which robots to use (default all)

## Output

Logs saved to: `trials/<controller>_<fault>_t<N>_<timestamp>/`

Each trial folder contains per-robot JSONL logs with position, state, trust scores, coverage, and fault timing.

## Troubleshooting

- **Robots not starting**: Check WiFi, use `--preflight` to ping
- **Position server not detecting**: Check camera, ArUco markers visible
- **Fault not injecting**: Check fault injector UDP port (5300)
- **Slow startup**: Increase `--start-delay` to 10s
- **Kill all robots**: `python -c "from run_trial import kill_all; kill_all()"`
