"""Quick script to check trust levels from experiment JSONL logs."""
import json, sys, os

LOG_DIR = "logs"
# Find most recent experiment run's logs (matching timestamp)
ts = "20260225_203146"  # First isef_experiments.py trial

for rid in [1, 2, 3, 4, 5]:
    f = os.path.join(LOG_DIR, f"robot_{rid}_{ts}.jsonl")
    if not os.path.exists(f):
        print(f"R{rid}: log not found")
        continue
    lines = open(f).readlines()
    if not lines:
        print(f"R{rid}: empty log")
        continue
    t0 = json.loads(lines[0])['t']
    
    # Sample at key times: t=30 (fault), t=50, t=72 (min trust), end
    for target_t in [30, 50, 72, None]:
        for line in lines:
            rec = json.loads(line)
            elapsed = rec['t'] - t0
            if target_t is None or elapsed >= target_t:
                state = rec.get('state')
                leader = rec.get('leader_id')
                trust = rec.get('trust_in_leader', 'N/A')
                susp = rec.get('suspicion', 0)
                bad = rec.get('bad_commands_received', 0)
                fault = rec.get('fault')
                label = f"t={elapsed:.0f}s" if target_t else "END"
                if target_t is None:
                    # Use last line
                    rec = json.loads(lines[-1])
                    elapsed = rec['t'] - t0
                    state = rec.get('state')
                    leader = rec.get('leader_id')
                    trust = rec.get('trust_in_leader', 'N/A')
                    susp = rec.get('suspicion', 0)
                    bad = rec.get('bad_commands_received', 0)
                    fault = rec.get('fault')
                    label = f"END(t={elapsed:.0f}s)"
                trust_str = f"{trust:.2f}" if isinstance(trust, float) else str(trust)
                susp_str = f"{susp:.2f}" if isinstance(susp, float) else str(susp)
                print(f"  R{rid} {label:>12}: state={state:<10} leader={leader} "
                      f"trust={trust_str} susp={susp_str} bad_cmds={bad} fault={fault}")
                break
    print()
