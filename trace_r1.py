import json

entries = []
with open('trials/reip_none_t1_20260312_201053/r1_robot_1_20260312_201059.jsonl') as f:
    for line in f:
        if line.strip():
            entries.append(json.loads(line))

t0 = entries[0]['t']
last_t = -10
for e in entries:
    t = e['t'] - t0
    if t - last_t >= 5.0:
        last_t = t
        nav = e.get('navigation_target')
        ltgt = e.get('leader_self_target')
        motor = e.get('motor_cmd', [0, 0])
        moving = abs(motor[0]) > 0.01 or abs(motor[1]) > 0.01
        print(
            f"t={t:6.1f}s  "
            f"pos=({e['x']:6.0f},{e['y']:6.0f})  "
            f"state={e['state']:10s}  "
            f"src={e.get('command_source','?'):18s}  "
            f"nav={str(nav):24s}  "
            f"self_tgt={str(ltgt):24s}  "
            f"stop={e.get('stop_reason',''):22s}  "
            f"motor={'MOVE' if moving else 'STOP':4s}  "
            f"visited={e.get('my_visited_count',0):3d}  "
            f"stuck={e.get('stuck_count',0)}"
        )
