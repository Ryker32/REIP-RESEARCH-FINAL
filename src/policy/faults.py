"""
Centralized fault injection helpers (hallucinations, etc.).

Design goals:
- Pure corruption: Do NOT mutate env or input assignments; return a new dict.
- Explicit state: The caller passes an object with hallucination_* attributes
    (e.g., a controller); we update only those attributes in update_* functions.
- Belief-only: Reads from agent.local and team_belief; never touches env.grid.

Hallucination profiles supported:
- 'overconfident': push to known-free cells near agent (wastes exploration)
- 'byzantine': random commands; sometimes synchronize to a single target
- 'clustered': cluster agents around a center agent
- 'dense_explored': leader redirects followers into regions that are mostly explored
- 'pingpong': followers toggle between two points for the attack duration (default 30 steps)
"""
from __future__ import annotations
from typing import Dict, Tuple, List, Any, Optional
import random
import numpy as np


def update_hallucination_state(carrier: Any, rate: float, t: int, *,
                               min_dur: int = 5, max_dur: int = 15,
                               profiles: Optional[List[str]] = None,
                               fixed_type: Optional[str] = None,
                               schedule: Optional[List[dict]] = None,
                               rng: random.Random | None = None) -> None:
    """Update hallucination flags on the given carrier.

    carrier: object with attributes
        - hallucination_active: bool
        - hallucination_duration: int
        - hallucination_type: Optional[str]
        
    NEW: If carrier has 'compromised_agent_id' set, hallucinations only trigger
    when the current leader_id matches that ID. This pins the attack to a specific
    compromised agent, so impeaching them actually removes the attack.
    """
    if profiles is None:
        profiles = ['overconfident', 'byzantine', 'clustered', 'dense_explored', 'pingpong']
    if rng is None:
        rng = random
    
    # === KEY FIX: Pin attack to specific compromised agent ===
    # If compromised_agent_id is set, only allow attack when that agent is leader
    compromised_id = getattr(carrier, 'compromised_agent_id', None)
    current_leader = getattr(carrier, 'leader_id', None)
    
    if compromised_id is not None and current_leader != compromised_id:
        # The compromised agent is NOT the leader - attack should not be active
        # Immediately deactivate any ongoing hallucination
        if getattr(carrier, 'hallucination_active', False):
            carrier.hallucination_active = False
            carrier.hallucination_type = None
            # Keep hallucination_origin_leader as a record of who was compromised
        return  # No attack when clean leader is in charge

    # If a deterministic schedule is provided, honor it (but don't suppress random triggers).
    # This allows both scheduled attacks AND continuous random hallucinations for more severe adversarial conditions.
    scheduled_triggered = False
    if schedule:
        for ev in schedule:
            try:
                start = int(ev.get('start', -1))
                if t == start and not getattr(carrier, 'hallucination_active', False):
                    chosen = ev.get('type') or fixed_type or rng.choice(profiles)
                    dur = ev.get('duration')
                    if dur is None:
                        dur = getattr(carrier, 'hallucination_pingpong_duration', 30) if chosen == 'pingpong' else rng.randint(min_dur, max_dur)
                    carrier.hallucination_active = True
                    carrier.hallucination_type = chosen
                    carrier.hallucination_duration = int(max(1, dur))
                    # Record explicit window for robust debugging/logic
                    try:
                        carrier.hallucination_start_t = int(start)
                        carrier.hallucination_end_t = int(start) + int(max(1, dur))
                        carrier.hallucination_active_src = 'schedule'
                    except Exception:
                        carrier.hallucination_start_t = start
                        carrier.hallucination_end_t = start + int(max(1, dur))
                        carrier.hallucination_active_src = 'schedule'
                    # Mark which leader originated this hallucination
                    try:
                        carrier.hallucination_origin_leader = getattr(carrier, 'leader_id', None)
                    except Exception:
                        carrier.hallucination_origin_leader = None
                    if chosen == 'pingpong':
                        setattr(carrier, '_pingpong_pairs', {})
                        setattr(carrier, '_pingpong_flip', {})
                    scheduled_triggered = True
                    break
            except Exception:
                continue

    # Apply continuous random hallucinations (even when schedule exists, for more severe adversarial conditions)
    if not getattr(carrier, 'hallucination_active', False) and rng.random() < max(0.0, float(rate)):
        if not getattr(carrier, 'hallucination_active', False):
            carrier.hallucination_active = True
            # Choose type (fixed if provided)
            chosen = fixed_type if fixed_type is not None else rng.choice(profiles)
            carrier.hallucination_type = chosen
            # Determine duration (special-case pingpong default ~30)
            if chosen == 'pingpong':
                dur = getattr(carrier, 'hallucination_pingpong_duration', 30)
            else:
                dur = rng.randint(min_dur, max_dur)
            carrier.hallucination_duration = int(max(1, dur))
            # Record explicit window for robust debugging/logic
            try:
                carrier.hallucination_start_t = int(t)
                carrier.hallucination_end_t = int(t) + int(max(1, dur))
                carrier.hallucination_active_src = 'random'
            except Exception:
                carrier.hallucination_start_t = t
                carrier.hallucination_end_t = t + int(max(1, dur))
                carrier.hallucination_active_src = 'random'
            # Mark origin leader
            try:
                carrier.hallucination_origin_leader = getattr(carrier, 'leader_id', None)
            except Exception:
                carrier.hallucination_origin_leader = None
            # Initialize per-profile ephemeral state
            if chosen == 'pingpong':
                # Clear prior pingpong state if any, will be (re)built lazily in corruptor
                setattr(carrier, '_pingpong_pairs', {})
                setattr(carrier, '_pingpong_flip', {})

    if getattr(carrier, 'hallucination_active', False):
        carrier.hallucination_duration -= 1
        if carrier.hallucination_duration <= 0:
            carrier.hallucination_active = False
            carrier.hallucination_type = None
            # Clear window metadata
            for attr in ('hallucination_start_t','hallucination_end_t','hallucination_active_src'):
                if hasattr(carrier, attr):
                    try:
                        delattr(carrier, attr)
                    except Exception:
                        pass
            # Clear ephemeral state for clean recovery
            if hasattr(carrier, '_pingpong_pairs'):
                try:
                    delattr(carrier, '_pingpong_pairs')
                except Exception:
                    pass
            if hasattr(carrier, '_pingpong_flip'):
                try:
                    delattr(carrier, '_pingpong_flip')
                except Exception:
                    pass
            if hasattr(carrier, 'hallucination_origin_leader'):
                try:
                    delattr(carrier, 'hallucination_origin_leader')
                except Exception:
                    pass


def corrupt_assignments_by_hallucination(env, assigns: Dict[int, Tuple[int, int]],
                                         frontiers: List[Tuple[int, int]] | None,
                                         carrier: Any,
                                         rng: random.Random | None = None) -> Dict[int, Tuple[int, int]]:
    """Return a new assignments map corrupted per the carrier's hallucination state.

    - Pure: does not mutate env, agents, or the input assigns dict.
    - Reads only belief/local maps (agent.local) and env.size/positions.
    """
    if rng is None:
        rng = random
    if not getattr(carrier, 'hallucination_active', False) or not assigns:
        return assigns

    htype = getattr(carrier, 'hallucination_type', None)
    out = dict(assigns)
    ids = list(out.keys())
    leader_id = getattr(carrier, 'leader_id', None)
    origin = getattr(carrier, 'hallucination_origin_leader', None)

    # Determine which agents to corrupt this step:
    # - If current leader == origin: corrupt followers (leader is broadcasting bad commands)
    # - If current leader != origin: corrupt only the origin agent (now just a bad follower)
    if leader_id is None:
        target_ids = ids
    elif origin is None or leader_id == origin:
        target_ids = [i for i in ids if i != leader_id]
    else:
        target_ids = [i for i in ids if i == origin]

    if htype == 'overconfident':
        for i in target_ids:
            if rng.random() < 0.3:
                known_free: List[Tuple[int, int]] = []
                agent = env.agents.get(i)
                if agent is not None and hasattr(agent, 'local'):
                    for (x, y), val in np.ndenumerate(agent.local):
                        if val == 0:
                            wx = x - agent.r_local + agent.pos[0]
                            wy = y - agent.r_local + agent.pos[1]
                            if 0 <= wx < env.size and 0 <= wy < env.size:
                                known_free.append((wx, wy))
                if known_free:
                    out[i] = rng.choice(known_free)

    elif htype == 'byzantine':
        for i in target_ids:
            if rng.random() < 0.5:
                out[i] = (rng.randint(0, env.size - 1), rng.randint(0, env.size - 1))
        if rng.random() < 0.3 and len(target_ids) > 1:
            tgt = out[target_ids[0]]
            for i in rng.sample(target_ids, k=max(1, len(target_ids)//2)):
                out[i] = tgt

    elif htype == 'clustered':
        if target_ids:
            center = rng.choice(target_ids)
            cx, cy = env.agent_positions.get(center, (env.size//2, env.size//2))
            for i in target_ids:
                ox, oy = rng.randint(-3, 3), rng.randint(-3, 3)
                nx = max(0, min(env.size - 1, cx + ox))
                ny = max(0, min(env.size - 1, cy + oy))
                out[i] = (nx, ny)

    elif htype == 'dense_explored':
        # Redirect FOLLOWERS (not leader) to regions that are majority explored.
        # ORIGINAL LOGIC restored: each agent gets a random choice from top-K densest explored cells
        # This spreads agents across dense explored areas, maximizing wasted effort
        leader_id = getattr(carrier, 'leader_id', None)
        tb = getattr(env, 'team_belief', None)
        corrupted_count = 0
        num_candidates = 0
        
        if tb is not None:
            # Build masks (team_belief indexed [y, x])
            # team_belief: -1 = unknown, 0 = free, 2 = obstacle
            explored_mask = (tb == 0)   # known free
            unknown_mask = (tb == -1)   # BUG FIX: was == 1, now == -1 (correct unknown value)
            ys, xs = np.where(explored_mask)
            explored_positions: List[Tuple[int, int]] = [(int(x), int(y)) for (y, x) in zip(ys, xs)]
            
            if explored_positions:
                # Score a sample of explored positions by local explored density
                sample = explored_positions if len(explored_positions) <= 500 else rng.sample(explored_positions, 500)
                radius = 4
                strict = bool(getattr(carrier, 'dense_explored_strict', True))
                top: List[Tuple[int, int, int]] = []  # (score, x, y)
                for (x, y) in sample:
                    x0 = max(0, x - radius); x1 = min(env.size, x + radius + 1)
                    y0 = max(0, y - radius); y1 = min(env.size, y + radius + 1)
                    # Slice [y, x]
                    window = explored_mask[y0:y1, x0:x1]
                    if strict:
                        # Skip if ANY unknown in a slightly larger window (more strictly wasted)
                        x0s = max(0, x - (radius + 2)); x1s = min(env.size, x + (radius + 2) + 1)
                        y0s = max(0, y - (radius + 2)); y1s = min(env.size, y + (radius + 2) + 1)
                        if unknown_mask[y0s:y1s, x0s:x1s].any():
                            continue
                    score = int(np.count_nonzero(window))
                    top.append((score, x, y))
                
                num_candidates = len(top)
                
                # Take top-K densest explored cells (ORIGINAL BEHAVIOR)
                top.sort(reverse=True)
                K = max(5, min(100, len(top)//5))
                candidates = [(x, y) for (s, x, y) in top[:K]]
                
                # Each agent gets a RANDOM choice from top-K (ORIGINAL BEHAVIOR)
                for i in target_ids:
                    if candidates:
                        out[i] = rng.choice(candidates)
                        corrupted_count += 1
                    
        # Store corruption stats for debugging
        carrier._last_dense_explored_corrupted = corrupted_count
        carrier._last_dense_explored_targets = len(target_ids) if target_ids else 0
        carrier._last_dense_explored_candidates = num_candidates

    elif htype == 'corner_trap':
        # SABOTAGE: Send ALL followers to a corner, wasting their time completely
        # This is a realistic "opponent trying to ruin our mission" attack
        corners = [(0, 0), (0, env.size-1), (env.size-1, 0), (env.size-1, env.size-1)]
        # Pick a corner (deterministic per attack instance for consistency)
        corner_idx = getattr(carrier, '_corner_trap_idx', None)
        if corner_idx is None:
            corner_idx = rng.randint(0, 3)
            carrier._corner_trap_idx = corner_idx
        corner = corners[corner_idx]
        # Send ALL targets to this corner with 100% probability
        corrupted_count = 0
        for i in target_ids:
            out[i] = corner
            corrupted_count += 1
        # Track for debugging
        carrier._last_dense_explored_corrupted = corrupted_count
        carrier._last_dense_explored_targets = len(target_ids) if target_ids else 0

    elif htype == 'pingpong':
        # Followers toggle between two fixed points for duration. Points are initialized lazily.
        leader_id = getattr(carrier, 'leader_id', None)
        pairs = getattr(carrier, '_pingpong_pairs', None)
        flips = getattr(carrier, '_pingpong_flip', None)
        if pairs is None:
            pairs = {}
            setattr(carrier, '_pingpong_pairs', pairs)
        if flips is None:
            flips = {}
            setattr(carrier, '_pingpong_flip', flips)
        for i in target_ids:
            pos = env.agent_positions.get(i, (env.size//2, env.size//2))
            if i not in pairs:
                # Create two distinct points near the agent (within 8 cells), clamped to bounds
                dx1, dy1 = rng.randint(-8, 8), rng.randint(-8, 8)
                dx2, dy2 = rng.randint(-8, 8), rng.randint(-8, 8)
                p1 = (max(0, min(env.size - 1, pos[0] + dx1)), max(0, min(env.size - 1, pos[1] + dy1)))
                p2 = (max(0, min(env.size - 1, pos[0] + dx2)), max(0, min(env.size - 1, pos[1] + dy2)))
                if p2 == p1:
                    p2 = (max(0, min(env.size - 1, p1[0] + 1)), p1[1])
                pairs[i] = (p1, p2)
                flips[i] = False
            # Toggle choice each call
            flips[i] = not flips[i]
            p1, p2 = pairs[i]
            out[i] = p1 if flips[i] else p2

    return out
