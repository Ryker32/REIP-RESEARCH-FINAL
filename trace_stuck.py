"""Find robots that got stuck (position not changing despite motors running)."""
import json, glob, os, sys, math

trial = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob('trials/*/'))[-1]
logs = sorted(glob.glob(os.path.join(trial, 'r*_robot_*.jsonl')))

for logfile in logs:
    rid = int(os.path.basename(logfile).split('_')[0][1:])
    entries = []
    with open(logfile) as f:
        for line in f:
            if line.strip():
                entries.append(json.loads(line))

    t0 = entries[0]['t']
    stuck_episodes = []
    stuck_start = None
    stuck_pos = None
    last_moving_t = None

    for i, e in enumerate(entries):
        t = e['t'] - t0
        motor = e.get('motor_cmd', [0, 0])
        moving_motors = abs(motor[0]) > 5 or abs(motor[1]) > 5
        x, y = e['x'], e['y']

        if moving_motors:
            last_moving_t = t
            if stuck_start is None:
                stuck_start = t
                stuck_pos = (x, y)
            else:
                dist = math.hypot(x - stuck_pos[0], y - stuck_pos[1])
                if dist < 20:  # less than 20mm movement = stuck
                    pass  # still stuck
                else:
                    if t - stuck_start > 3.0:  # was stuck for >3s
                        stuck_episodes.append((stuck_start, t, t - stuck_start, stuck_pos))
                    stuck_start = t
                    stuck_pos = (x, y)
        else:
            if stuck_start is not None and last_moving_t:
                dur = last_moving_t - stuck_start
                if dur > 3.0:
                    dist = math.hypot(x - stuck_pos[0], y - stuck_pos[1])
                    if dist < 30:
                        stuck_episodes.append((stuck_start, last_moving_t, dur, stuck_pos))
            stuck_start = None
            stuck_pos = None

    print(f"R{rid}: {len(stuck_episodes)} stuck episodes (>3s, <20mm movement)")
    for s, end, dur, pos in stuck_episodes:
        print(f"  t={s:.1f}-{end:.1f}s ({dur:.1f}s) at ({pos[0]:.0f}, {pos[1]:.0f})")

    # Also find wall-wedge events: ToF < 50mm while motors running
    wedge_count = 0
    for e in entries:
        t = e['t'] - t0
        motor = e.get('motor_cmd', [0, 0])
        tof = e.get('tof', {})
        front_tofs = [v for k, v in tof.items() if 'front' in k and v < 8000]
        if front_tofs and min(front_tofs) < 40 and (motor[0] > 10 or motor[1] > 10):
            wedge_count += 1
    if wedge_count:
        print(f"  Wall-contact while driving forward: {wedge_count} ticks")
    print()
