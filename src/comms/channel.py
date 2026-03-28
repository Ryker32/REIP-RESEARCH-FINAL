# src/comms/channel.py
"""
Communication channel with realistic fault models.

Implements packet loss, corruption, and command disruption for testing
resilience of coordination protocols under adverse network conditions.

References:
- Gilbert (1960): Capacity of a burst-noise channel
- Elliott (1963): Estimates of error rates for codes on burst-noise channels
- Akyildiz et al. (2002): Wireless sensor network survey
- Jadbabaie et al. (2003): Coordination of groups under limited communication
- NTIA Gilbert-Elliot implementation: https://github.com/NTIA/gilbert-elliot-model
"""
import numpy as np, random
from collections import deque


def _simulate_gilbert_elliot_errors(p, r, k, h, n=1000, seed=None):
    """
    Simulate errors using the Gilbert-Elliot burst error model.
    
    Adapted from NTIA implementation: https://github.com/NTIA/gilbert-elliot-model
    
    Args:
        p: P(Good -> Bad transition)
        r: P(Bad -> Good transition)
        k: P(no error | Good state)
        h: P(no error | Bad state)
        n: Number of packet attempts to simulate
        seed: Random seed for reproducibility
    
    Returns:
        List of 0s and 1s where 0=no error, 1=error
    """
    rng = np.random.default_rng(seed)
    
    # Start in Good or Bad state based on steady-state probability
    pi_bad = p / (p + r)
    current_state = 'BAD' if rng.uniform() < pi_bad else 'GOOD'
    
    errors = []
    for _ in range(n):
        # Determine if error occurs in current state
        if current_state == 'GOOD':
            error = 1 if rng.uniform() > k else 0  # Error if exceeds k
            # Transition to next state
            current_state = 'BAD' if rng.uniform() < p else 'GOOD'
        else:  # BAD state
            error = 1 if rng.uniform() > h else 0  # Error if exceeds h
            # Transition to next state
            current_state = 'GOOD' if rng.uniform() < r else 'BAD'
        
        errors.append(error)
    
    return errors

def _merge_lr(a, b):
    obs = (a.global_lr == 2) | (b.global_lr == 2)
    out = np.where(obs, 2, -1)
    free = ((a.global_lr == 0) | (b.global_lr == 0)) & (~obs)
    out = np.where(free, 0, out)
    a.global_lr[:] = out
    b.global_lr[:] = out

def _corrupt_map(map_data, corruption_rate=0.05):
    """
    Corrupt a fraction of map cells (bit flips, noise).
    Models sensor noise or transmission errors.
    """
    if corruption_rate <= 0:
        return map_data
    
    corrupted = map_data.copy()
    mask = np.random.random(corrupted.shape) < corruption_rate
    # Flip between known states (-1, 0, 2)
    flip_values = np.random.choice([-1, 0, 2], size=corrupted.shape)
    corrupted[mask] = flip_values[mask]
    return corrupted


class GilbertElliotChannel:
    """
    Wrapper for NTIA Gilbert-Elliot channel model.
    
    Provides stateful per-link channels for realistic bursty packet loss.
    Uses the official NTIA implementation of the Gilbert-Elliot model.
    
    Parameters match NTIA conventions:
        p: Probability of transitioning from Good -> Bad state
        r: Probability of transitioning from Bad -> Good state  
        k: Probability of NO error in Good state (1-k = error rate in Good)
        h: Probability of NO error in Bad state (1-h = error rate in Bad)
    
    Example (realistic wireless):
        channel = GilbertElliotChannel(p=0.1, r=0.3, k=0.95, h=0.3)
        # This gives ~21% average packet loss with bursts of 3-4 consecutive drops
    
    References:
        - Gilbert (1960): Capacity of a burst-noise channel
        - Elliott (1963): Estimates of error rates on burst-noise channels
        - NTIA: https://github.com/NTIA/gilbert-elliot-model
    """
    
    def __init__(self, p=0.1, r=0.3, k=0.95, h=0.3, seed=None):
        """
        Initialize Gilbert-Elliot channel.
        
        Args:
            p: P(Good -> Bad transition) [default 0.1 = 10%]
            r: P(Bad -> Good transition) [default 0.3 = 30%]
            k: P(no error | Good state) [default 0.95 = 95% success in Good]
            h: P(no error | Bad state) [default 0.3 = 30% success in Bad]
            seed: Random seed for reproducibility
        """
        self.p = p
        self.r = r
        self.k = k
        self.h = h
        self.seed = seed
        
        # Pre-generate error sequence (will regenerate when exhausted)
        self.error_buffer = []
        self.buffer_index = 0
        self._regenerate_buffer()
    
    def _regenerate_buffer(self, n=1000):
        """Generate new error sequence from Gilbert-Elliot model."""
        self.error_buffer = _simulate_gilbert_elliot_errors(self.p, self.r, self.k, self.h, n=n, seed=self.seed)
        self.buffer_index = 0
        # Don't reuse same seed
        if self.seed is not None:
            self.seed += 1
    
    def transmit(self):
        """
        Attempt packet transmission.
        
        Returns:
            bool: True if packet successfully received, False if dropped
        """
        # Check if we need more errors
        if self.buffer_index >= len(self.error_buffer):
            self._regenerate_buffer()
        
        # Get next error value (0 = success, 1 = error)
        error = self.error_buffer[self.buffer_index]
        self.buffer_index += 1
        
        return error == 0  # Return True if no error (successful transmission)
    
    def get_statistics(self):
        """Get theoretical channel statistics."""
        pi_bad = self.p / (self.p + self.r)
        pi_good = 1 - pi_bad
        error_rate = pi_good * (1 - self.k) + pi_bad * (1 - self.h)
        avg_burst_length = 1 / self.r  # Expected time in Bad state
        
        return {
            'pi_good': pi_good,
            'pi_bad': pi_bad,
            'error_rate': error_rate,
            'success_rate': 1 - error_rate,
            'avg_burst_length': avg_burst_length
        }


def share_maps(env, R=10, p_loss=0.0, p_corrupt=0.0, use_gilbert_elliot=False, ge_params=None, timestep=0):
    """
    Share maps between agents with communication faults.
    
    OLD BEHAVIOR = ORACLE VIEW or gods eye view of maps
    - centralize dmerge which meant that all agents magically see everyone elses data
    - used merge each agent's low res maps (of entire global map) to combine to the global lr map of others

    NEW (distributed)
    -each agent only comms with neighbors in rang eR
    - uses agent.recieve_map_from_neighbor() for p2p transfer of maps
    - gilbert elliot channel simulates bursty packet loss
    -agents build belief lr incrementally from observations and neighbor dataz

    
    Args:
        env: Environment with agents
        R: Communication radius (Manhattan distance)
        p_loss: Probability of packet loss (uniform random, ignored if use_gilbert_elliot=True)
        p_corrupt: Probability of map corruption (noise in data)
        use_gilbert_elliot: If True, use Gilbert-Elliot bursty channel model
        ge_params: Dict with Gilbert-Elliot params {p, r, k, h}, only used if use_gilbert_elliot=True
    """

    #initiallize gilb elliot channel sim if requested  
    channel = None
    if use_gilbert_elliot:
        if ge_params is None:
            ge_params = {'p': 0.1, 'r': 0.3, 'k': 0.95, 'h': 0.3}
        
        #create channel with buffers size = worst case number of transmissoons
        # each agent pair tries two transmissions (i->j and j->i)
        n_agents = len(env.agents)
        max_pairs = n_agents * (n_agents - 1) // 2  # combine withuot replacement
        # Don't pass buffer_size - GilbertElliotChannel regenerates buffer automatically

        channel = GilbertElliotChannel(
            p=ge_params['p'],
            r=ge_params['r'],
            k=ge_params['k'],
            h=ge_params['h'],
            seed=timestep  # Use timestep for reproducible variability
        )

    successful_exchanges = 0

    agent_ids = list(env.agents.keys())


    #p2p comms- each pair tries bidiretional into exchange
    for i_idx, i in enumerate(agent_ids):
        agent_i = env.agents[i]
        for j in agent_ids[i_idx + 1:]:  # Upper triangle only
            agent_j = env.agents[j]
            dist = abs(agent_i.pos[0] - agent_j.pos[0]) + abs(agent_i.pos[1] - agent_j.pos[1])
            if dist > R:
                continue
            if use_gilbert_elliot:
                tx_success = channel.transmit()
            else:
                tx_success = (np.random.rand() >= p_loss)
            
            if tx_success:
                if getattr(env, "debug_comms", False):
                    before = agent_j.belief_lr.copy()
                agent_j.receive_map_from_neighbor(agent_i.belief_lr, i, timestep)
                successful_exchanges += 1
                if getattr(env, "debug_comms", False):
                    changed = int((agent_j.belief_lr != before).sum())
                    print(f"[COMMS] t={timestep} {i}->{j} merged changed={changed}")
            if use_gilbert_elliot:
                tx_success = channel.transmit()
            else:
                tx_success = (np.random.rand() >= p_loss)
            if tx_success:
                if getattr(env, "debug_comms", False):
                    before = agent_i.belief_lr.copy()
                agent_i.receive_map_from_neighbor(agent_j.belief_lr, j, timestep)
                successful_exchanges += 1
                if getattr(env, "debug_comms", False):
                    changed = int((agent_i.belief_lr != before).sum())
                    print(f"[COMMS] t={timestep} {j}->{i} merged changed={changed}")
    return successful_exchanges

def filter_assignments_by_comm(env, assignments, comm_radius=10, p_cmd_loss=0.0, leader_id=0, multi_hop=False):
    """
    Simulate command loss: leader's assignments don't reach all agents.
    
    Agents that don't receive commands stay at current position or use stale assignment.
    This models realistic distributed systems where commands can be lost in transmission.
    
    Args:
        env: Environment with agent positions
        assignments: Dict of {agent_id: target_position}
        comm_radius: Maximum command transmission distance
        p_cmd_loss: Probability of command loss (even within radius)
    
    Returns:
        Filtered assignments dict with unreceived commands removed
    """
    # Always enforce command radius; apply probabilistic loss when p_cmd_loss>0
    if not assignments:
        return assignments
    
    filtered = {}
    # Use the provided leader_id (defaults to 0 for backward compatibility)
    leader_pos = env.agent_positions.get(leader_id, (env.size // 2, env.size // 2))

    # Optional multi-hop relay: build adjacency graph of agents within comm_radius
    # and compute hop distances from the leader. If reachable in H hops, the
    # success probability is approximated as (1 - p_cmd_loss) ** H.
    hop_dist = None
    if multi_hop:
        try:
            positions = env.agent_positions
            ids = list(positions.keys())
            adj = {i: [] for i in ids}
            for i in ids:
                xi, yi = positions[i]
                for j in ids:
                    if i == j:
                        continue
                    xj, yj = positions[j]
                    if abs(xi - xj) + abs(yi - yj) <= comm_radius:
                        adj[i].append(j)
            # BFS from leader to compute minimum hops
            hop_dist = {leader_id: 0}
            q = deque([leader_id])
            while q:
                cur = q.popleft()
                for nb in adj.get(cur, []):
                    if nb not in hop_dist:
                        hop_dist[nb] = hop_dist[cur] + 1
                        q.append(nb)
        except Exception:
            hop_dist = None
    
    for agent_id, target in assignments.items():
        agent_pos = env.agent_positions.get(agent_id)
        if agent_pos is None:
            continue
        
        if multi_hop and hop_dist is not None:
            # Multi-hop: reachable if we computed a hop distance
            if agent_id in hop_dist:
                hops = max(0, int(hop_dist[agent_id]))
                if p_cmd_loss > 0.0 and hops > 0:
                    success_prob = (1.0 - p_cmd_loss) ** hops
                    received = (random.random() < success_prob)
                else:
                    received = True
            else:
                received = False
        else:
            # Single-hop model (legacy): check Manhattan distance to leader directly
            dist = abs(agent_pos[0] - leader_pos[0]) + abs(agent_pos[1] - leader_pos[1])
            received = dist <= comm_radius and (p_cmd_loss <= 0.0 or random.random() >= p_cmd_loss)

        if not received:
            # Command not received: keep following previous hold_target if any
            # IMPORTANT: Set to current position so fallback logic can detect disconnection
            # and provide autonomous exploration target
            try:
                a = env.agents.get(agent_id)
                if a is not None and getattr(a, 'hold_target', None) is not None:
                    # Check if hold_target is stale (agent already arrived)
                    hold_target = a.hold_target
                    if hold_target == agent_pos:
                        # Agent already at hold target - signal for fallback
                        filtered[agent_id] = agent_pos
                    else:
                        # Keep following old hold target
                        filtered[agent_id] = hold_target
                else:
                    # No prior target known: set to current position to trigger fallback
                    filtered[agent_id] = agent_pos
            except Exception:
                filtered[agent_id] = agent_pos
        else:
            # Command received successfully
            filtered[agent_id] = target
    
    return filtered


'''
In agent.blief_lr: 
-1 = Unknown (never seen by me or any neighbor)
 0 = Free space (explored, safe to traverse)
 2 = Obstacle (explored, can't go here)
# Only assigns frontiers to UNKNOWN cells (belief_lr == -1)
# If belief_lr[x, y] == 0 or 2, it's already explored -> won't be assigned
# --- IGNORE ---
'''

def compute_beacon_tree(env, leader_id=0, R=10):
    """
    Compute a BFS spanning tree over the current communication graph (Manhattan radius R),
    returning per-agent parent pointers and hop counts from the leader.

    Purpose:
    - Lightweight "beacon" structure for connectivity-aware behaviors (parent tethers, hop-based policies).
    - No oracle: uses only agent positions and local radius to define edges.

    Returns:
    - parent: dict[int,int|None]  Parent of each node in the BFS tree (leader has None)
    - hops:   dict[int,int]       Minimum hop count from leader to each node

    Citations:
    - Ji, M., Egerstedt, M. (2007). Distributed coordination control of multi-agent systems while preserving connectedness. IEEE TAC.
    - Zavlanos, M.M., Pappas, G.J. (2005). Controlling connectivity of dynamic graphs. IEEE CDC.
    - Jadbabaie, A., Lin, J., Morse, A.S. (2003). Coordination under limited communication. IEEE CDC.
    """
    try:
        positions = env.agent_positions
        ids = list(positions.keys())
    except Exception:
        return {}, {}

    # Build adjacency by local radius R (Manhattan)
    adj = {i: [] for i in ids}
    for i in ids:
        xi, yi = positions[i]
        for j in ids:
            if i == j:
                continue
            xj, yj = positions[j]
            if abs(xi - xj) + abs(yi - yj) <= int(R):
                adj[i].append(j)

    parent = {i: None for i in ids}
    hops = {i: int(1e9) for i in ids}
    if leader_id not in positions:
        return parent, {i: int(1e9) for i in ids}

    # BFS from leader
    from collections import deque
    q = deque([leader_id])
    parent[leader_id] = None
    hops[leader_id] = 0
    while q:
        cur = q.popleft()
        for nb in adj.get(cur, []):
            if hops[nb] > hops[cur] + 1:
                parent[nb] = cur
                hops[nb] = hops[cur] + 1
                q.append(nb)
    return parent, hops