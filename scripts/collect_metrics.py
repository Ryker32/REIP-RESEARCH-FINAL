import os, sys, yaml, math

# ensure src on path
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
_SRC_DIR = os.path.join(_PROJECT_ROOT, 'src')
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from src.env.gridworld import GridWorld
from src.comms.channel import share_maps
import importlib, inspect


def normalize_policy_path(path: str) -> str:
    if path.startswith("src."): return path
    if path.startswith("policy."): return "src." + path
    if path == "greedy": return "src.policy.frontier.assign_frontiers_greedy"
    if path == "voronoi": return "src.policy.frontier.assign_voronoi"
    if path == "reip": return "src.policy.reip.REIPController"
    if path == "entropy": return "src.policy.frontier.assign_frontiers_entropy"
    if path == "entropy_global": return "src.policy.frontier.assign_frontiers_entropy_global"
    return path


def load_cfg(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def nn_mean_positions(agent_positions):
    # agent_positions: dict id -> (x,y)
    ids = list(agent_positions.keys())
    if len(ids) <= 1:
        return 0.0
    total = 0.0
    for i in ids:
        x,y = agent_positions[i]
        best = float('inf')
        for j in ids:
            if i==j: continue
            x2,y2 = agent_positions[j]
            d = math.hypot(x-x2, y-y2)
            if d < best: best = d
        total += best
    return total/len(ids)


def nn_mean_targets(assigns):
    # assigns: dict agent -> (x,y)
    targets = list(assigns.values())
    if len(targets) <= 1: return 0.0
    total = 0.0
    for i, (x,y) in enumerate(targets):
        best = float('inf')
        for j, (x2,y2) in enumerate(targets):
            if i==j: continue
            d = math.hypot(x-x2, y-y2)
            if d < best: best = d
        total += best
    return total/len(targets)


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--config', default='configs/dual_res.yaml')
    p.add_argument('--T', type=int, default=None)
    args = p.parse_args()

    cfg = load_cfg(args.config)
    if args.T is not None:
        cfg['T'] = args.T

    env = GridWorld(cfg)
    policy_path = normalize_policy_path(cfg.get('policy', 'entropy'))
    mod, name = policy_path.rsplit('.', 1)
    symbol = getattr(importlib.import_module(mod), name)
    controller = symbol(cfg) if inspect.isclass(symbol) else None

    state = {'t':0, 'prev': None, 'leader': 0, 'N': cfg.get('N',1)}

    prev_assigns = None
    total_assign_changes = 0
    total_steps_with_change = 0
    per_step_changes = []
    nn_pos = []
    nn_targets = []

    for t in range(cfg['T']):
        state['t'] = t
        share_maps(env, R=cfg.get('comm',{}).get('radius',2), p_loss=cfg.get('comm',{}).get('p_loss',0.0))
        frontiers = env.detect_frontiers()
        if controller is None:
            raises = symbol(env, frontiers, lam=cfg.get('lam',0.0), prev_assigns=state['prev'])
            assigns = raises[0]
            env.step(assigns, t=t)
            synced = {}
            for i in assigns.keys():
                a = env.agents.get(i)
                held = getattr(a, 'hold_target', None)
                synced[i] = held if held is not None else assigns[i]
            state['prev'] = synced
        else:
            controller.step(env, frontiers, state)
            assigns = state['prev']

        # metrics
        if prev_assigns is not None:
            changes = 0
            for i in assigns.keys():
                if prev_assigns.get(i) != assigns.get(i):
                    changes += 1
            total_assign_changes += changes
            if changes>0:
                total_steps_with_change += 1
            per_step_changes.append(changes)
        prev_assigns = dict(assigns)

        nn_pos.append(nn_mean_positions(env.agent_positions))
        nn_targets.append(nn_mean_targets(assigns))

    # summary
    avg_changes_per_step = sum(per_step_changes)/len(per_step_changes) if per_step_changes else 0.0
    pct_steps_with_changes = 100.0 * total_steps_with_change / cfg['T']
    print('\n=== METRICS ===')
    print(f"T = {cfg['T']}")
    print(f"Total assignment changes (agent-target pairs changed across steps): {total_assign_changes}")
    print(f"Average assignment changes per step (count, excluding t=0): {avg_changes_per_step:.3f}")
    print(f"Percentage of timesteps with any assignment change: {pct_steps_with_changes:.1f}%")
    print(f"Mean nearest-neighbor distance among agents (averaged over timesteps): {sum(nn_pos)/len(nn_pos):.3f}")
    print(f"Mean nearest-neighbor distance among assigned targets (averaged over timesteps): {sum(nn_targets)/len(nn_targets):.3f}")

