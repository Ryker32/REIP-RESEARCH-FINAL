# src/main.py
"""
Main Simulation Entry Point

Orchestrates multi-agent exploration with configurable policies, communication channels,
and visualization. Supports trust-based governance (REIP) and baseline comparisons.

References:
- Yamauchi (1997): A frontier-based approach for autonomous exploration
- Burgard et al. (2000): Collaborative multi-robot exploration
- Fox et al. (2006): Distributed multirobot exploration and mapping
"""

import os, sys
import argparse, yaml, importlib, inspect
from pathlib import Path

# When running this file as a script (python src/main.py) the import search path
# may not allow imports like `src.*` and also imports like `env.*`. Add both the
# project root and the `src` directory to sys.path so either import style works.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from src.env.gridworld import GridWorld
from src.utils.logging import RunLogger
from src.comms.channel import share_maps
# optional plotting helper
try:
    from src.tools.plot import plot_env
except Exception:
    plot_env = None

#some debugging was helped by CHATGPT and also a lot of comments were added by it for documentation (I was honestly too lazy to write them myself lol)
# backbone (for REIP-style controllers that don't implement their own step)
try:
    from src.policy.frontier import assign_greedy as _greedy_assign
except ImportError:
    # fallback if your function is named assign_frontiers_greedy
    from src.policy.frontier import assign_frontiers_greedy as _greedy_assign


def import_symbol(path: str):
    """
    Import a function or class from a dotted path string, e.g.
    'policy.frontier.assign_greedy' or 'policy.reip.REIPController'
    """
    mod, name = path.rsplit(".", 1)
    module = importlib.import_module(mod)
    return getattr(module, name)

def normalize_policy_path(path: str) -> str:
    if path.startswith("src."): return path
    if path.startswith("policy."): return "src." + path
    if path == "greedy": return "src.policy.frontier.assign_frontiers_greedy"
    if path == "voronoi": return "src.policy.frontier.assign_voronoi"
    if path == "reip": return "src.policy.reip.REIPController"
    if path == "reip_true": return "src.policy.reip_true.REIPController"  # TRUE REIP with trust
    if path == "reip_enhanced": return "src.policy.reip_enhanced.REIPEnhancedPolicy"  # Enhanced REIP with loop detection
    if path == "baseline_leader_follower": return "src.policy.baseline_leader_follower.BaselineLeaderFollower"  # No governance
    if path == "baseline": return "src.policy.baseline_leader_follower.BaselineLeaderFollower"  # No governance
    if path == "entropy": return "src.policy.frontier.assign_frontiers_entropy"
    if path == "entropy_global": return "src.policy.frontier.assign_frontiers_entropy_global"
    return path

def run(cfg: dict, visualize: bool = False, debug: bool = False, record_dir: str = None):
    global _greedy_assign

    env = GridWorld(cfg)
    # Propagate optional comm debug flag to the environment for share_maps logging
    try:
        env.debug_comms = bool(cfg.get("env", {}).get("debug_comms", False))
    except Exception:
        env.debug_comms = False
    debug_force_nearest = bool(cfg.get("debug_force_nearest", False))
    # Track simple coverage history for stagnation detection
    cov_hist = []
    # Visualization helper: allow per-agent mini-maps when requested in config
    visualize_light = bool(cfg.get("visualize_light", False))
    env.draw_agent_maps = bool(cfg.get('reip', {}).get('draw_agent_maps', False))
    if visualize_light:
        # light mode keeps plotting lightweight by disabling per-agent worldviews
        env.draw_agent_maps = False
    run_name = cfg.get("name", "default")
    log = RunLogger(cfg, run_name=run_name)
    record_path = None
    if record_dir:
        record_path = Path(record_dir)
        record_path.mkdir(parents=True, exist_ok=True)
        print(f"[record] saving frames to {record_path}")
    
    # Initialize cyber attack scheduler for fair comparisons
    cyber_attack_rate = cfg.get("cyber_attack_rate", cfg.get("reip", {}).get("cyber_attack_rate", 0.0))
    if cyber_attack_rate > 0:
        from src.utils.cyber_attack_scheduler import initialize_scheduler
        total_timesteps = cfg.get("T", 400)
        seed = cfg.get("env", {}).get("seed", 42)  # Get seed from env section
        initialize_scheduler(seed=seed, attack_rate=cyber_attack_rate, total_timesteps=total_timesteps)
    
    policy_path = normalize_policy_path(cfg.get("policy", "src.policy.frontier.assign_frontiers_greedy"))
    print("using policy:", policy_path)  # debug
    symbol = import_symbol(policy_path)


    lam = cfg.get("lam", 0.0)                 # congestion penalty for greedy
    rotation_H = cfg.get("rotation_H")  # for simple rotation baseline
    N = cfg.get("N", 1)


    state = {"t": 0,
        "prev": None,   # previous assignments (for congestion-aware greedy)
        "leader": 0,    # current leader id (for logging/rotation)
        "N": N}

 
    controller = symbol(cfg) if inspect.isclass(symbol) else None


    # prepare plotting if requested
    if visualize and plot_env is not None:
        import matplotlib.pyplot as plt
        plt.ion()
        # Don't create ax here - let plot_env_enhanced create the enhanced layout
        ax = None

    viz_every = max(1, int(cfg.get("viz_every", 1)))
    for t in range(cfg["T"]):
        state["t"] = t
        # keep env time in sync for debug prints (frontiers etc.)
        env.current_time = t
        cov_now = env.coverage()
        cov_hist.append(cov_now)
        cov_hist = cov_hist[-12:]
        state["cov_hist"] = cov_hist
        stagnating = False
        if len(cov_hist) >= 8:
            window = cov_hist[-8:]
            stagnating = (max(window) - min(window)) < 1e-3
        
        # Communication with optional Gilbert-Elliot channel model
        comm_cfg = cfg.get("comm", {})
        use_ge = comm_cfg.get("use_gilbert_elliot", False)
        ge_params = comm_cfg.get("gilbert_elliot_params", None)
        
        share_maps(env,
                   R=comm_cfg.get("radius", 10),
                   p_loss=comm_cfg.get("p_loss", 0.0),
                   p_corrupt=comm_cfg.get("p_corrupt", 0.0),
                   use_gilbert_elliot=use_ge,
                   ge_params=ge_params,
                   timestep=t)  # Pass current timestep for seeding
        

        #periodic rotation baseline (still uses greedy assign)
        if rotation_H and t > 0 and (t % rotation_H == 0):
            state["leader"] = (state["leader"] + 1) % N
        # Determine current leader id for Phase 2 gating and leader-belief frontiers
        leader_id = None
        if controller is not None and hasattr(controller, "leader_id"):
            leader_id = controller.leader_id
        else:
            leader_id = state.get("leader", 0)
        env.leader_id = leader_id if leader_id is not None else 0
        leader_agent = env.agents.get(env.leader_id, None)
        # debug: show how much coarse unknown remains in leader belief
        if debug and leader_agent is not None and hasattr(leader_agent, "belief_lr"):
            blr = leader_agent.belief_lr
            try:
                unk = int(((blr == -1) | (blr == 1)).sum())
                print(f"[DBG] t={t} leader coarse unknown={unk}/{blr.size}")
            except Exception:
                pass
        use_team_frontiers = bool(cfg.get("debug_use_team_frontiers", False))

        if use_team_frontiers:
            frontiers = env.detect_frontiers()
        else:
            frontiers = env.detect_frontiers_from_agent_belief(leader_agent) if leader_agent is not None else env.detect_frontiers()
            # Frontier growth fallback (leader-belief only): if frontier set is tiny or stagnating, 
            # seed from BELIEF-FRONTIERS: unknown cells adjacent to ANY known cell (free or obstacle)
            # This represents the boundary of explored space
            belief_frontiers = []
            if leader_agent is not None and hasattr(leader_agent, "belief_lr"):
                blr = leader_agent.belief_lr
                ds = max(1, int(getattr(leader_agent, "ds", 1)))
                if (len(frontiers) <= 6 or stagnating):
                    sx, sy = blr.shape
                    for gx in range(sx):
                        for gy in range(sy):
                            v = blr[gx, gy]
                            # Must be unknown
                            if v != -1 and v != 1:
                                continue
                            # Must be adjacent to ANY known cell (free=0 or obstacle=2)
                            # This is the boundary of what we've explored
                            has_known_neighbor = False
                            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                                nx, ny = gx + dx, gy + dy
                                if 0 <= nx < sx and 0 <= ny < sy:
                                    nv = blr[nx, ny]
                                    if nv == 0 or nv == 2:  # known-free OR known-obstacle
                                        has_known_neighbor = True
                                        break
                            if has_known_neighbor:
                                wx = gx * ds + ds // 2
                                wy = gy * ds + ds // 2
                                belief_frontiers.append((wx, wy))
                    if belief_frontiers:
                        try:
                            lp = env.agent_positions.get(env.leader_id, None)
                            if lp is not None:
                                belief_frontiers.sort(key=lambda p: abs(p[0]-lp[0]) + abs(p[1]-lp[1]))
                        except Exception:
                            pass
                        need = max(1, len(env.agent_positions))
                        frontiers = belief_frontiers[:need]

            # If still empty, keep leader-only stance; just log for diagnosis (no team map fallback)
            if not frontiers and env.debug_frontiers and env.current_time % env.debug_frontiers_every == 0:
                print(f"[Frontier] t={getattr(env, 'current_time', '?')}: leader frontiers EMPTY (no fallback to team map)")

        if controller is None:
            #Function policy (like greedy)
            assigns, claims, U_pred = symbol(env, frontiers, lam=lam, prev_assigns=state["prev"])
            env.step(assigns, t=t)
            # sync prev assigns with what agents are actually holding (hold_target)
            synced = {}
            for i in assigns.keys():
                a = env.agents.get(i)
                held = getattr(a, 'hold_target', None)
                synced[i] = held if held is not None else assigns[i]
            state["prev"] = synced

            # For controls, fabricate a minimal "leader/election" object for the logger
            dummy = type("L", (), {"leader_id": state["leader"], "election_count": 0})
            log.record(env, dummy, t)

        else:
            #Class controller (E.G. REIPController))
            if hasattr(controller, "step") and callable(controller.step):
                if hasattr(env, "leader_id"):
                    env.leader_id = getattr(controller, "leader_id", env.leader_id)
                controller.step(env, frontiers, state)  
            else:
                if _greedy_assign is None:
                    _greedy_assign = import_symbol("src.policy.frontier.assign_greedy")
                assigns, claims, U_pred = _greedy_assign(env, frontiers, lam=lam, prev_assigns=state["prev"])
                if hasattr(env, "leader_id"):
                    env.leader_id = getattr(controller, "leader_id", env.leader_id)
                env.step(assigns, t=t)
                # sync prev assigns with agents' hold_target to avoid flip-flops
                synced = {}
                for i in assigns.keys():
                    a = env.agents.get(i)
                    held = getattr(a, 'hold_target', None)
                    synced[i] = held if held is not None else assigns[i]
                state["prev"] = synced
                # classic REIP update/election hooks:
                if hasattr(controller, "update_trust_and_signals"):
                    controller.update_trust_and_signals(env, claims, U_pred)
                if hasattr(controller, "should_impeach") and controller.should_impeach():
                    if hasattr(controller, "elect_new_leader"):
                        controller.elect_new_leader(None, None)
                        if hasattr(env, "leader_id"):
                            env.leader_id = getattr(controller, "leader_id", env.leader_id)
            log.record(env,controller,t)


        # Periodic progress prints
        if t % 10 == 0 or t == cfg["T"] - 1:
            coverage_pct = env.coverage() * 100
            leader_info = ""
            if controller is not None and hasattr(controller, 'leader_id'):
                leader_info = f", Leader={controller.leader_id}"
                if hasattr(controller, 'get_average_trust_in_leader'):
                    try:
                        avg_trust = controller.get_average_trust_in_leader()
                        leader_info += f", Trust={avg_trust:.2f}"
                    except:
                        pass
            print(f"t={t:4d}: Coverage={coverage_pct:5.1f}%{leader_info}")
        
        # optional debug prints
        if debug:
            try:
                stucks = {i: getattr(a, 'stuck_count', 0) for i, a in env.agents.items()}
                print(f"t={t} positions={env.agent_positions} assigns={state['prev']} stucks={stucks}")
            except Exception:
                pass

        # visualization hook: non-blocking update of the plot
        if visualize and plot_env is not None and (t % viz_every == 0):
            try:
                # Try to fetch dynamic leader and avg trust from state if present
                leader_id = state.get('leader', None)
                avg_trust = state.get('avg_trust', None)
                cmd_radius = cfg.get('reip', {}).get('command_radius', cfg.get('reip', {}).get('command_radius', None))
                
                # Use enhanced visualization if REIP controller available
                if controller is not None:
                    from src.tools.plot import plot_env_enhanced
                    # Always use enhanced visualization for REIP to show agent worldviews
                    ax = plot_env_enhanced(env, controller, ax=ax, show=False, 
                                          frontiers=frontiers, timestep=t,
                                          leader_id=leader_id, command_radius=cmd_radius, 
                                          avg_trust=avg_trust, show_agent_worldviews=not visualize_light)
                else:
                    # Standard visualization for non-REIP policies
                    ax = plot_env(env, ax=ax, show=False, frontiers=frontiers, timestep=t,
                                  leader_id=leader_id, command_radius=cmd_radius, avg_trust=avg_trust,
                                  draw_local=not visualize_light, draw_frontiers=True)
                import matplotlib.pyplot as plt
                if record_path is not None and ax is not None:
                    frame_file = record_path / f"frame_{t:05d}.png"
                    ax.figure.savefig(frame_file, dpi=140, bbox_inches="tight")
                # Reduce pause time in light mode to speed up large-batch visualization
                plt.pause(0.01 if visualize_light else 0.05)
            except BaseException:
                # Keep simulation running even if plotting is interrupted or fails (including KeyboardInterrupt)
                pass

        if env.coverage() >= 1.0:
            print(f"every cell explored at t={t}")
            break

    log.flush()

    # final plotting cleanup
    if visualize and plot_env is not None:
        try:
            import matplotlib.pyplot as plt
            plt.ioff()
            plt.show()
        except Exception:
            pass

def run_simulation(config_path: str, quiet: bool = False):
    """
    Run simulation and return final metrics for analysis.
    
    Args:
        config_path: Path to YAML configuration file
        quiet: If True, suppress print output
        
    Returns:
        Dictionary with final metrics including coverage, elections, etc.
    """
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    
    # Initialize cyber attack scheduler for fair comparisons
    cyber_attack_rate = cfg.get("cyber_attack_rate", cfg.get("reip", {}).get("cyber_attack_rate", 0.0))
    if cyber_attack_rate > 0:
        from src.utils.cyber_attack_scheduler import initialize_scheduler
        total_timesteps = cfg.get("T", 400)
        seed = cfg.get("env", {}).get("seed", 42)  # Get seed from env section
        initialize_scheduler(seed=seed, attack_rate=cyber_attack_rate, total_timesteps=total_timesteps)
    
    # Temporarily redirect stdout if quiet mode
    if quiet:
        import io
        import contextlib
        stdout = sys.stdout
        sys.stdout = io.StringIO()
    
    try:
        # Run the simulation
        run(cfg, visualize=False, debug=False)
        
        # Extract metrics from log file
        log_path = os.path.join("runs", cfg["name"], "log.csv")
        metrics = extract_final_metrics(log_path, cfg)
        
        return metrics
        
    finally:
        if quiet:
            sys.stdout = stdout

def extract_final_metrics(log_path: str, cfg: dict):
    """
    Extract final metrics from simulation log.
    """
    metrics = {
        'coverage': 0.0,
        'elections': 0,
        'impeachments': 0,
        'loop_detections': 0,
        'aimless_detections': 0
    }
    
    try:
        import csv
        with open(log_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            rows = list(reader)
            if rows:
                last = rows[-1]
                # Final coverage
                if 'coverage' in last and last['coverage'] != '':
                    try:
                        metrics['coverage'] = float(last['coverage'])
                    except ValueError:
                        metrics['coverage'] = 0.0

                # Count elections (changes in leader_id)
                leader_ids = [r.get('leader_id') for r in rows if 'leader_id' in r]
                if leader_ids:
                    # normalize to ints when possible
                    norm = []
                    for v in leader_ids:
                        try:
                            norm.append(int(v))
                        except Exception:
                            norm.append(v)
                    changes = 0
                    prev = None
                    for v in norm:
                        if prev is None:
                            prev = v
                            continue
                        if v != prev:
                            changes += 1
                            prev = v
                    metrics['elections'] = max(0, changes)

                # Enhanced metrics if available
                if 'impeachments' in last and last['impeachments'] != '':
                    try:
                        metrics['impeachments'] = int(last['impeachments'])
                    except ValueError:
                        pass
                if 'loop_detections' in rows[0]:
                    try:
                        metrics['loop_detections'] = sum(int(r.get('loop_detections') or 0) for r in rows)
                    except Exception:
                        pass
                if 'aimless_detections' in rows[0]:
                    try:
                        metrics['aimless_detections'] = sum(int(r.get('aimless_detections') or 0) for r in rows)
                    except Exception:
                        pass
    except Exception as e:
        print(f"Warning: Could not extract metrics from {log_path}: {e}")
    
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--visualize", action="store_true", help="Show live visualization during the run")
    parser.add_argument("--debug", action="store_true", help="Print per-step assignment/agent diagnostics")
    parser.add_argument("--record-dir", type=str, default=None, help="Directory to dump visualization frames")
    args = parser.parse_args()

    if args.config:
        with open(args.config, "r") as f:
            cfg = yaml.safe_load(f)
    else:
        
        cfg = {
            "name": "controls/greedy_fixed",
            "policy": "policy.frontier.assign_greedy",
            "T": 200,"N": 20,"map_size": 30,"lam": 2.0
        }

    run(cfg, visualize=args.visualize, debug=args.debug, record_dir=args.record_dir)
