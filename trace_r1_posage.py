import json

entries = []
with open('trials/reip_none_t1_20260312_201053/r1_robot_1_20260312_201059.jsonl') as f:
    for line in f:
        if line.strip():
            entries.append(json.loads(line))

t0 = entries[0]['t']
last_t = -3
for e in entries:
    t = e['t'] - t0
    if t < 20 or t - last_t < 3.0:
        continue
    last_t = t
    age = e.get('position_rx_age')
    age_str = f"{age:.2f}s" if age is not None else "None"
    print(
        f"t={t:6.1f}s  "
        f"pos=({e['x']:6.0f},{e['y']:6.0f})  "
        f"pos_age={age_str:8s}  "
        f"stop={e.get('stop_reason',''):22s}  "
        f"src={e.get('command_source','?')}"
    )
