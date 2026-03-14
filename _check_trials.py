import json, os, math

base = r'c:\Users\ryker\reip-sim-public\trials'
all_dirs = [d for d in os.listdir(base) if os.path.isdir(os.path.join(base, d))]

results = {}
for dirname in sorted(all_dirs):
    d = os.path.join(base, dirname)
    files = [f for f in os.listdir(d) if f.endswith('.jsonl')]
    if not files:
        continue

    parts = dirname.split('_')
    controller = parts[0]

    fault = 'unknown'
    trial = '?'
    if 'none' in dirname: fault = 'none'
    elif 'bad_leader' in dirname: fault = 'bad_leader'
    elif 'freeze_leader' in dirname: fault = 'freeze_leader'
    elif 'self_injure' in dirname: fault = 'self_injure'

    for p in parts:
        if p.startswith('t') and len(p) <= 3 and p[1:].isdigit():
            trial = p

    speeds = []
    team_cov = 0
    n_robots = 0
    n_entries = 0
    tof_total = 0
    tof_valid = 0
    for fname in files:
        entries = []
        for l in open(os.path.join(d, fname)):
            if l.strip():
                try:
                    entries.append(json.loads(l))
                except json.JSONDecodeError:
                    pass
        started = [e for e in entries if e.get('stop_reason') != 'trial_not_started']
        if not started:
            continue
        if 't' not in started[0]:
            continue
        n_robots += 1
        t0, tf = started[0]['t'], started[-1]['t']
        dur = tf - t0
        dist = sum(math.sqrt((started[i].get('x',0)-started[i-1].get('x',0))**2 + (started[i].get('y',0)-started[i-1].get('y',0))**2) for i in range(1, len(started)))
        speeds.append(dist / dur if dur > 0 else 0)
        team_cov = max(team_cov, started[-1].get('known_visited_count', 0))
        n_entries += len(started)
        for e in started:
            for v in e.get('tof', {}).values():
                tof_total += 1
                if v > 0:
                    tof_valid += 1

    avg_speed = sum(speeds) / len(speeds) if speeds else 0
    tof_pct = (tof_valid / tof_total * 100) if tof_total else 0

    key = f'{controller}/{fault}'
    quality = 'GOOD' if (n_robots >= 4 and avg_speed > 25 and team_cov > 50) else 'WEAK'
    if n_robots < 3:
        quality = 'BAD'

    if key not in results:
        results[key] = []
    results[key].append({
        'dir': dirname, 'trial': trial, 'robots': n_robots,
        'speed': avg_speed, 'cov': team_cov, 'tof': tof_pct,
        'quality': quality, 'entries': n_entries
    })

NEEDED = [
    'reip/none', 'reip/bad_leader', 'reip/freeze_leader', 'reip/self_injure',
    'raft/none', 'raft/bad_leader', 'raft/freeze_leader', 'raft/self_injure',
]

for key in NEEDED:
    print(f'=== {key} ===')
    if key in results:
        for r in results[key]:
            flag = ' OK ' if r['quality'] == 'GOOD' else r['quality']
            print(f"  [{flag}] {r['dir']}")
            print(f"        {r['robots']}R  {r['speed']:.0f}mm/s  cov={r['cov']}  tof={r['tof']:.0f}%")
    else:
        print('  ** NO DATA **')
    print()
