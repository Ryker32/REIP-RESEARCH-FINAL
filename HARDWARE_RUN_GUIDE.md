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
# REIP + bad_leader (default)
python run_trial.py

# REIP + specific fault
python run_trial.py --controller reip --fault bad_leader
python run_trial.py --controller reip --fault freeze_leader
python run_trial.py --controller reip --fault self_injure_leader
python run_trial.py --controller reip --fault none          # clean

# Raft + specific fault
python run_trial.py --controller raft --fault none
python run_trial.py --controller raft --fault bad_leader
python run_trial.py --controller raft --fault freeze_leader
python run_trial.py --controller raft --fault self_injure_leader
```

### 3. Run Full Batch (all conditions)
```bash
python run_trial.py --batch --trials-per 5
```

## Paper Timing (t=10s fault injection)

**IMPORTANT**: Default is t=20s, but paper uses t=10s. Always use `--fault-time 10`:
```bash
python run_trial.py --controller reip --fault bad_leader --fault-time 10
python run_trial.py --controller raft --fault bad_leader --fault-time 10
```

## Complete Raft Rerun (5 trials each, t=10s fault timing)

```bash
# Raft clean (no fault)
python run_trial.py --controller raft --fault none --trial 1
python run_trial.py --controller raft --fault none --trial 2
python run_trial.py --controller raft --fault none --trial 3
python run_trial.py --controller raft --fault none --trial 4
python run_trial.py --controller raft --fault none --trial 5

# Raft freeze_leader
python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial 1
python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial 2
python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial 3
python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial 4
python run_trial.py --controller raft --fault freeze_leader --fault-time 10 --trial 5

# Raft self_injure_leader
python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial 1
python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial 2
python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial 3
python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial 4
python run_trial.py --controller raft --fault self_injure_leader --fault-time 10 --trial 5

# Raft bad_leader (if missing)
python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial 1
python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial 2
python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial 3
python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial 4
python run_trial.py --controller raft --fault bad_leader --fault-time 10 --trial 5
```

## What It Does Automatically

1. **Kills** old processes on all robots
2. **Clears** old logs
3. **Deploys** code to all robots (SSH upload)
4. **Launches** robots with stagger
5. **Waits** 5s for localization + leader election
6. **Starts** trial (sends start signal)
7. **Injects** fault at specified time (default t=20s, **paper uses t=10s** - always use `--fault-time 10`)
8. **Stops** robots at 120s
9. **Collects** logs to `hardware_trials/trial_N/`

## Options

- `--controller reip|raft|decentralized` - Which controller
- `--fault none|bad_leader|freeze_leader|self_injure_leader` - Fault type
- `--fault-time N` - When to inject fault (default 20s, paper uses 10s)
- `--trial N` - Trial number (for logging)
- `--duration N` - Trial duration (default 120s)
- `--start-delay N` - Wait time after launch (default 5s)
- `--preflight` - Ping all robots before starting
- `--robots 1,2,3,4,5` - Which robots to use (default all)

## Output

Logs saved to: `hardware_trials/trial_N/robot_1.jsonl ... robot_5.jsonl`

## Quick Scripts

**Windows**: Run `run_all_raft_trials.bat` to rerun all Raft trials (20 total)
**Linux/Mac**: Run `bash run_all_raft_trials.sh` to rerun all Raft trials (20 total)

## Troubleshooting

- **Robots not starting**: Check WiFi, use `--preflight` to ping
- **Position server not detecting**: Check camera, ArUco markers visible
- **Fault not injecting**: Check fault injector UDP port (5300)
- **Slow startup**: Increase `--start-delay` to 10s
