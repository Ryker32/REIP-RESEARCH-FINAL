"""Check position_rx_age for ALL robots during R1's dropout window (t=23-105s)."""
import json, glob, os

trial = 'trials/reip_none_t1_20260312_201053'
logs = sorted(glob.glob(os.path.join(trial, 'r*_robot_*.jsonl')))

for logfile in logs:
    rid = int(os.path.basename(logfile).split('_')[0][1:])
    entries = []
    with open(logfile) as f:
        for line in f:
            if line.strip():
                entries.append(json.loads(line))
    t0 = entries[0]['t']

    max_age = 0
    max_age_t = 0
    stale_periods = []
    stale_start = None
    for e in entries:
        t = e['t'] - t0
        age = e.get('position_rx_age')
        if age is not None and age > max_age:
            max_age = age
            max_age_t = t
        if age is not None and age > 2.0:
            if stale_start is None:
                stale_start = t
        else:
            if stale_start is not None:
                stale_periods.append((stale_start, t, t - stale_start))
                stale_start = None
    if stale_start is not None:
        stale_periods.append((stale_start, entries[-1]['t'] - t0, entries[-1]['t'] - t0 - stale_start))

    print(f"R{rid}: max_pos_age={max_age:.1f}s at t={max_age_t:.1f}s")
    if stale_periods:
        for s, e_t, dur in stale_periods:
            print(f"     STALE period: t={s:.1f}s to t={e_t:.1f}s ({dur:.1f}s)")
    else:
        print(f"     No stale periods (always fresh)")
    print()
