"""Trace a specific robot's behavior over a trial."""
import json, sys, glob, os

trial = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob('trials/*/'))[-1]
rid = int(sys.argv[2]) if len(sys.argv) > 2 else 3

logs = sorted(glob.glob(os.path.join(trial, f'r{rid}_robot_*.jsonl')))
if not logs:
    print(f"No log for R{rid} in {trial}")
    sys.exit(1)

entries = []
with open(logs[0]) as f:
    for line in f:
        if line.strip():
            entries.append(json.loads(line))

t0 = entries[0]['t']
last_t = -3
for e in entries:
    t = e['t'] - t0
    if t - last_t < 5.0:
        continue
    last_t = t
    age = e.get('position_rx_age')
    age_str = f"{age:.2f}s" if age is not None else "None"
    motor = e.get('motor_cmd', [0, 0])
    tof = e.get('tof', {})
    tof_str = ' '.join(f"{k[0]}:{v}" for k, v in sorted(tof.items())) if tof else "none"
    print(
        f"t={t:6.1f}s  "
        f"pos=({e['x']:6.0f},{e['y']:6.0f})  "
        f"pos_age={age_str:8s}  "
        f"state={e['state']:10s}  "
        f"stop={e.get('stop_reason',''):22s}  "
        f"src={e.get('command_source','?'):18s}  "
        f"motor={motor}  "
        f"tof=[{tof_str}]  "
        f"visited={e.get('my_visited_count',0)}"
    )
