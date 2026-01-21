"""
REIP: Resilient Election & Impeachment Policy

Trust-based governance for multi-agent exploration using belief-only signals.

Key pieces:
- Entropy-based assignment (frontier scoring)
- Belief-only uncertainty measure for trust (configurable)
- Impeachment when avg trust in leader drops below threshold
- Optional hallucination profiles and command-loss filtering

References:
- Shannon, C. (1948). A Mathematical Theory of Communication. Bell Syst. Tech. J. (entropy)
- Yamauchi, B. (1997). A Frontier-Based Approach for Autonomous Exploration. IEEE CIRA. (frontiers)
- Hart, Nilsson, Raphael (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE. (A*)
- Bresenham, J. (1965). Algorithm for computer control of a digital plotter. IBM Systems Journal. (LOS)
- Lamport, Shostak, Pease (1982). The Byzantine Generals Problem. ACM TOPLAS. (fault tolerance)
- Castro, Liskov (1999). Practical Byzantine Fault Tolerance. OSDI. (governance background)
"""

from src.policy.frontier import assign_frontiers_entropy_global, assign_frontiers_connected
from src.policy.cbba_assignment import cbba_frontier_assignment
from src.comms.channel import filter_assignments_by_comm, compute_beacon_tree
from src.policy.faults import update_hallucination_state, corrupt_assignments_by_hallucination
from src.policy.hallucination_detection import HallucinationDetector, AdaptiveThresholdManager
from src.policy.dispersion import (compute_dispersion_reward, compute_information_dispersion,
                                  adaptive_dispersion_strength, compute_voronoi_cells)
import random
import math
import numpy as np
from collections import deque


class REIPController:
    """REIP Controller with trust, elections, and optional adversarial faults."""

    # Citations:
    # - Lamport, Shostak, Pease (1982) Byzantine Generals (fault-tolerant consensus/governance)
    # - Castro & Liskov (1999) PBFT (practical byzantine governance under faults)
    # - Yamauchi (1997) Frontier-based exploration (task decomposition via frontiers)
    # - Shannon (1948) Information theory (entropy signals for utility/trust)
    def __init__(self, cfg):
        """Initialize controller state from config."""
        self.full_cfg = cfg
        self.reip_cfg = cfg.get("reip", {})
        self.N = cfg.get("N", 1)

        # Debug controls
        self.debug_prints = bool(self.reip_cfg.get("debug_prints", False))

        # Governance state
        self.leader_id = 0
        self.election_count = 0
        self.trust = {i: {j: 1.0 for j in range(self.N)} for i in range(self.N)}

        # Info-theoretic tracking
        self.prev_entropy = None
        self.predicted_coverage_gain = 0.0
        self.leader_failures = {i: 0 for i in range(self.N)}

        # Trust parameters
        self.trust_decay_rate = self.reip_cfg.get("trust_decay_rate", 0.5)
        self.trust_threshold = self.reip_cfg.get("trust_threshold", 0.6)
        self.min_trust = self.reip_cfg.get("min_trust", 0.1)
        self.trust_recovery_rate = self.reip_cfg.get("trust_recovery_rate", 0.02)
        # Trust robustness controls
        # - trust_deadband: ignore small pred/obs mismatches |U_pred-U_obs| <= deadband
        # - trust_event_pred_min: only penalize when prediction claimed at least this gain
        # - trust_event_obs_max: and observed gain was at most this value
        # - trust_decay_cap: cap max one-step multiplicative drop (e.g., 0.25 → at most 25% loss/tick)
        # NOTE: Lower defaults than original to work with scaled predicted_gain
        self.trust_deadband = float(self.reip_cfg.get("trust_deadband", 0.05))
        self.trust_event_pred_min = float(self.reip_cfg.get("trust_event_pred_min", 0.05))
        self.trust_event_obs_max = float(self.reip_cfg.get("trust_event_obs_max", 0.30))
        self.trust_decay_cap = float(self.reip_cfg.get("trust_decay_cap", 0.25))
        
        # MPC-based trust evaluation (Model Predictive Control for local verification)
        # Citations:
        # - Rawlings, J.B., Mayne, D.Q. (2009). "Model Predictive Control: Theory and Design."
        #   MPC enables agents to compute locally optimal actions from their own belief.
        # - Camacho, E.F., Bordons, C. (2004). "Model Predictive Control." 2nd ed. Springer.
        #   MPC is computationally efficient when using compressed state representations.
        # NOTE: pred_gain_radius is defined just below; use a safe local default if the
        # config does not provide an explicit mpc_trust_window.
        self.mpc_trust_enabled = bool(self.reip_cfg.get("mpc_trust_enabled", True))
        self.mpc_trust_weight = float(self.reip_cfg.get("mpc_trust_weight", 0.5))  # Weight in hybrid trust (0-1)
        # Use a temporary default; we will overwrite with pred_gain_radius once it is defined.
        tmp_window = self.reip_cfg.get("mpc_trust_window", None)
        self.mpc_trust_window = int(tmp_window) if tmp_window is not None else 0
        self.mpc_trust_threshold = float(self.reip_cfg.get("mpc_trust_threshold", 0.3))  # Minimum difference to trigger trust decay
        
        # Hallucination
        self.hallucination_rate = self.reip_cfg.get("hallucination_rate", 0.0)
        self.hallucination_active = False
        self.hallucination_duration = 0
        self.hallucination_type = None  # 'overconfident' | 'byzantine' | 'clustered' | 'dense_explored' | 'pingpong'
        # Optional fixed profile selection and per-profile durations
        self.hallucination_profile = self.reip_cfg.get("hallucination_profile", None)
        self.hallucination_pingpong_duration = self.reip_cfg.get("hallucination_pingpong_duration", 30)
        # Optional deterministic schedule: list of {start:int, type:str, duration:int}
        self.hallucination_schedule = self.reip_cfg.get("hallucination_schedule", None)
        # Option to make dense_explored strictly avoid unknown borders
        self.dense_explored_strict = bool(self.reip_cfg.get("dense_explored_strict", False))
        # Option to suppress local autonomy fallback during leader-origin attacks
        self.disable_fallback_during_attack = bool(self.reip_cfg.get("disable_fallback_during_attack", False))
        self.fallback_enabled = bool(self.reip_cfg.get("fallback_enabled", True))
        self.fallback_on_command_loss = bool(self.reip_cfg.get("fallback_on_command_loss", True))
        self.fallback_on_noop = bool(self.reip_cfg.get("fallback_on_noop", True))
        
        # === KEY: Pin attack to specific compromised agent ===
        # If set, hallucinations only trigger when this agent is leader.
        # When a clean agent becomes leader (after impeachment), attack stops.
        self.compromised_agent_id = self.reip_cfg.get("compromised_agent_id", None)
        if self.compromised_agent_id is not None:
            self.compromised_agent_id = int(self.compromised_agent_id)
        self.detection_enabled = bool(self.reip_cfg.get("detection_enabled", True))
        self.governance_enabled = bool(self.reip_cfg.get("governance_enabled", True))
        # Tunable overconfidence factor applied to predicted gain during leader-origin hallucinations
        self.pred_gain_inflation_base = float(self.reip_cfg.get("pred_gain_inflation_base", 1.5))
        self.pred_gain_inflation_jitter = float(self.reip_cfg.get("pred_gain_inflation_jitter", 0.5))
        # Predicted gain estimation (unknown-count scale)
        self.pred_gain_alpha = float(self.reip_cfg.get("pred_gain_alpha", 0.2))
        self.pred_gain_radius = int(self.reip_cfg.get("pred_gain_radius", 3))
        # If mpc_trust_window was not explicitly set, default it to pred_gain_radius now
        if self.mpc_trust_window <= 0:
            self.mpc_trust_window = self.pred_gain_radius

        # Communication faults
        self.command_loss_rate = self.reip_cfg.get("command_loss_rate", 0.0)
        self.command_radius = self.reip_cfg.get("command_radius", 100)
        # Optional: allow commands to relay over multiple hops instead of single-hop from leader
        self.multi_hop_commands = bool(self.reip_cfg.get("multi_hop_commands", False))
        # Connectivity preference for assignments (keeps agents within comm range of ≥2 neighbors)
        self.connectivity_beta = float(self.reip_cfg.get("connectivity_beta", 0.0))
        # Regional diversity controls (limit agents per coarse region and control region granularity)
        self.region_size = self.reip_cfg.get("region_size", None)
        self.region_capacity = int(self.reip_cfg.get("region_capacity", 1))
        # Anti-clique shaping and chain dispersion
        self.triangle_penalty = float(self.reip_cfg.get("triangle_penalty", 0.0))
        self.chain_beta = float(self.reip_cfg.get("chain_beta", 0.0))
        self.degree_target = int(self.reip_cfg.get("degree_target", 2))
        self.degree_over_penalty = float(self.reip_cfg.get("degree_over_penalty", 0.0))
        # New: discourage degrees below a minimum target (helps prevent splits)
        self.degree_min_target = int(self.reip_cfg.get("degree_min_target", 2))
        # Encourage maintaining at least minimal neighbor degree by default
        self.degree_under_penalty = float(self.reip_cfg.get("degree_under_penalty", 0.4))
        # Optional spacing between concurrent assignments to reduce bunching
        self.frontier_spacing_radius = int(self.reip_cfg.get("frontier_spacing_radius", 0))
        # Adaptive connectivity scaling (reduce cohesion when graph is healthy)
        # Citations: Adaptive potential-field style weighting; see Ji & Egerstedt (2007) for
        # connectivity constraints and trade-offs; Burgard et al. (2000) for dispersion utility.
        self.adaptive_connectivity = bool(self.reip_cfg.get("adaptive_connectivity", True))
        self.adapt_scale_min = float(self.reip_cfg.get("adapt_scale_min", 0.5))  # 50% strength when safe
        self.adapt_deg_safe = float(self.reip_cfg.get("adapt_deg_safe", 1.8))     # target avg degree considered safe
        self.adapt_ttl_risk_frac = float(self.reip_cfg.get("adapt_ttl_risk_frac", 0.2))
        self.frontier_spacing_extra_safe = int(self.reip_cfg.get("frontier_spacing_extra_safe", 2))

        # Optional parent tether and leader heading bias (connectivity-friendly, low bandwidth)
        # Citations:
        # - Ji & Egerstedt (2007), Zavlanos & Pappas (2005): connectivity preservation via local constraints
        # - Olfati-Saber (2006), Jadbabaie et al. (2003): heading/velocity alignment under limited comms
        self.enable_beacon_tether = bool(self.reip_cfg.get("enable_beacon_tether", False))
        self.tether_lambda = float(self.reip_cfg.get("tether_lambda", 0.0))
        self.tether_margin = int(self.reip_cfg.get("tether_margin", 1))
        self.enable_heading_bias = bool(self.reip_cfg.get("enable_heading_bias", False))
        self.heading_gamma = float(self.reip_cfg.get("heading_gamma", 0.0))
        self.heading_risk_min_degree = int(self.reip_cfg.get("heading_risk_min_degree", 1))
        # Isolation guardrails for assignment scoring
        self.comm_break_penalty = float(self.reip_cfg.get("comm_break_penalty", 1.0))
        self.link_margin = int(self.reip_cfg.get("link_margin", 1))
        self.link_margin_penalty = float(self.reip_cfg.get("link_margin_penalty", 0.2))

        # Optional regroup behavior when an agent risks isolation (degree below threshold)
        # Citations:
        # - Ji, M., Egerstedt, M. (2007). Preserving connectedness with distributed constraints. IEEE TAC.
        # Enable regroup by default so agents proactively rejoin the mesh when degree is low
        self.enable_regroup = bool(self.reip_cfg.get("enable_regroup", True))
        self.regroup_min_degree = int(self.reip_cfg.get("regroup_min_degree", 1))

        # Linked-signature TTL for relay health: agents reachable from leader refresh TTL; others decay
        self.link_signature_ttl = int(self.reip_cfg.get("link_signature_ttl", 6))
        self.link_risk_penalty = float(self.reip_cfg.get("link_risk_penalty", 0.0))
        self.link_risk_ttl_thresh = int(self.reip_cfg.get("link_risk_ttl_thresh", 2))
        self._link_ttl = {i: 0 for i in range(self.N)}

        # Exploration knobs
        self.base_prox = cfg.get("prox_beta", 0.6)
        self.dispersion_alpha = self.reip_cfg.get("dispersion_alpha", 1.0)

        # Assignment algorithm selection
        # 'greedy_bfs' (default): Sequential BFS-ordered connected assignment
        # 'cbba': Consensus-Based Bundle Algorithm (auction-based, optimal spatial allocation)
        # Citation: Choi, Han-Lim, Brunet, Luc, & How, Jonathan P. (2009). "Consensus-Based 
        #           Decentralized Auctions for Robust Task Allocation." IEEE Trans. Robotics, 25(4).
        self.assignment_algorithm = self.reip_cfg.get("assignment_algorithm", "greedy_bfs")

        # Logging
        self.trust_history = []
        self.leader_history = []
        self.command_loss_events = 0
        self.impeachment_count = 0
        # Impeachment cooldown/persistence to avoid rapid leader churn on transient splits
        self.impeachment_cooldown = int(self.reip_cfg.get('impeachment_cooldown', 15))
        self.impeachment_persistence = int(self.reip_cfg.get('impeachment_persistence', 3))
        self._last_impeachment_t = -9999
        self._impeachment_below_count = 0
        self.prev_regions = None

        # Uncertainty metric for trust update
        # 'unknown_count' (default) or 'coarse_shannon'
        self.trust_entropy_mode = self.reip_cfg.get("trust_entropy_mode", "unknown_count")
        
        # CUSUM-based hallucination detection (Page 1954)
        cusum_kappa = float(self.reip_cfg.get("cusum_kappa", 0.1))
        cusum_threshold = float(self.reip_cfg.get("cusum_threshold", 5.0))
        if self.detection_enabled:
            self.hallucination_detector = HallucinationDetector(
                kappa=cusum_kappa,
                threshold=cusum_threshold,
                min_samples=3,
                window_size=10
            )
            self.adaptive_threshold = AdaptiveThresholdManager(
                base_threshold=cusum_threshold,
                min_threshold=2.0,
                max_threshold=10.0,
                adaptation_rate=0.1
            )
        else:
            self.hallucination_detector = None
            self.adaptive_threshold = None

        # Optional Bayesian residual-based detector (lightweight belief filter)
        self.bayes_enabled = bool(self.reip_cfg.get("bayes_enabled", False))
        self.bayes_prior_fault = float(self.reip_cfg.get("bayes_prior_fault", 0.1))
        self.bayes_sigma_fault = float(self.reip_cfg.get("bayes_sigma_fault", 0.2))
        self.bayes_sigma_healthy = float(self.reip_cfg.get("bayes_sigma_healthy", 0.05))
        self.bayes_threshold = float(self.reip_cfg.get("bayes_threshold", 0.8))
        # decay toward prior each tick to avoid permanently flagging a leader after recovery
        self.bayes_decay = float(self.reip_cfg.get("bayes_decay", 0.01))
        if self.bayes_enabled:
            self.bayes_fault_prob = {i: self.bayes_prior_fault for i in range(self.N)}
        else:
            self.bayes_fault_prob = {}
        
        # Dispersion incentives (reduce clustering bottlenecks)
        self.dispersion_enabled = bool(self.reip_cfg.get("dispersion_enabled", True))
        self.dispersion_weight = float(self.reip_cfg.get("dispersion_weight", 0.4))
        self.min_separation_distance = int(self.reip_cfg.get("min_separation_distance", 6))
        
        # Track agent movement history for loop detection
        self.agent_move_history = {i: deque(maxlen=12) for i in range(self.N)}
        self.coverage_history = deque(maxlen=20)

    def _debug(self, message: str):
        if self.debug_prints:
            print(message)

    # -----------------------------
    # Uncertainty measurement (belief-only)
    # -----------------------------
    # Citations:
    # - Shannon (1948): Entropy as information measure; we use belief-only unknown count / coarse entropy.
    def _count_unknown_coarse(self, belief_lr):
        """Count unknown coarse cells (-1) in a belief_lr grid.
        
        Encoding convention (coarse belief maps):
            -1 = unknown, 0 = free, 2 = obstacle
        Value 1 is no longer used for obstacles, so we treat ONLY -1 as unknown.
        """
        if belief_lr is None:
            return 0.0
        unknown = (belief_lr == -1)
        return float(np.count_nonzero(unknown))

    def _measure_uncertainty(self, env, belief_lr=None):
        """Measure remaining uncertainty from a single belief_lr (coarse only)."""
        if belief_lr is None:
            try:
                leader = getattr(self, "leader_id", 0)
                agent = env.agents.get(leader, None)
                belief_lr = getattr(agent, "belief_lr", None) if agent is not None else None
            except Exception:
                belief_lr = None
        return self._count_unknown_coarse(belief_lr)

    # -----------------------------
    # Capacity estimation (belief-only, tether-aware)
    # -----------------------------
    def _estimate_frontier_capacity(self, env, frontiers, center, allow, spacing):
        """
        Estimate how many agents can work in parallel near a center anchor:
        - Restrict to frontiers within L1 distance <= allow of the given center
        - Greedily select a set of "slots" separated by >= spacing in L1

        This is a quick, conservative proxy for "how much room there is" along a
        boundary segment without over-assigning into a tight corner.

        Returns: int slot_count
        """
        try:
            if not frontiers or center is None or allow is None or allow <= 0:
                return 0
            cx, cy = center
            # Filter reachable by tether constraint
            reachable = [f for f in frontiers if (abs(f[0]-cx) + abs(f[1]-cy) <= int(allow))]
            if not reachable:
                return 0
            # Greedy maximal set with separation >= spacing
            sep = max(1, int(spacing))
            slots = []
            for f in sorted(reachable, key=lambda p: (p[0], p[1])):
                if all(abs(f[0]-s[0]) + abs(f[1]-s[1]) > sep for s in slots):
                    slots.append(f)
            return len(slots)
        except Exception:
            return 0

    def _comm_component(self, env, center_id, radius):
        """Return the set of agent IDs connected to center_id via multi-hop within given radius.

        Connectivity uses Manhattan distance between agent positions and transitive closure.
        """
        try:
            positions = env.agent_positions
        except Exception:
            return set()
        if center_id not in positions:
            return set()
        # Build adjacency
        ids = list(positions.keys())
        adj = {i: set() for i in ids}
        for i in ids:
            xi, yi = positions[i]
            for j in ids:
                if i == j:
                    continue
                xj, yj = positions[j]
                d = abs(xi - xj) + abs(yi - yj)
                if d <= radius:
                    adj[i].add(j)
        # BFS from center_id
        comp = set([center_id])
        q = [center_id]
        while q:
            cur = q.pop()
            for nb in adj.get(cur, ()): 
                if nb not in comp:
                    comp.add(nb)
                    q.append(nb)
        return comp

    def _measure_uncertainty_component(self, env, agent_ids, belief_lr, ds, window_r=3):
        """Measure uncertainty using coarse belief_lr in a window around given agents."""
        if belief_lr is None:
            return 0.0
        r = max(0, int(window_r))
        size_x, size_y = belief_lr.shape
        covered = set()
        for aid in agent_ids:
            pos = env.agent_positions.get(aid)
            if pos is None:
                continue
            x, y = pos
            gx0 = max(0, (x - r) // ds); gx1 = min(size_x - 1, (x + r) // ds)
            gy0 = max(0, (y - r) // ds); gy1 = min(size_y - 1, (y + r) // ds)
            for gx in range(gx0, gx1 + 1):
                for gy in range(gy0, gy1 + 1):
                    covered.add((gx, gy))
        unknown = 0
        for (gx, gy) in covered:
            v = belief_lr[gx, gy]
            # Encoding: -1 = unknown, 0 = free, 2 = obstacle
            if v == -1:
                unknown += 1
        return float(unknown)

    # -----------------------------
    # Hallucination injection
    # -----------------------------
    # Citations:
    # - Lamport et al. (1982), Castro & Liskov (1999): adversarial/fault-tolerant setups motivating attack simulation.
    def inject_hallucination(self, t):
        update_hallucination_state(
            self,
            self.hallucination_rate,
            t,
            fixed_type=self.hallucination_profile,
            schedule=self.hallucination_schedule,
        )

    # Citations:
    # - Fault injection for resilience analysis aligns with byzantine/failure modes in distributed governance (Lamport 1982; Castro & Liskov 1999).
    def corrupt_assignments(self, env, assigns, frontiers):
        return corrupt_assignments_by_hallucination(env, assigns, frontiers, self)

    # -----------------------------
    # Trust / election
    # -----------------------------
    # Citations:
    # - Reputation/trust update under performance gaps; exponential decay is a common choice in control/monitoring.
    # - We tie trust to belief-only gains to avoid oracle leakage (cf. Shannon 1948 for info signals).
    def update_trust(self, predicted_gain, observed_gain, mpc_error=None):
        """Update follower trust in the current leader based on predicted vs observed gains.
        
        Hybrid trust mechanism:
        - Prediction-Observation Trust: Detects "failed promises" (leader promised gain but didn't deliver)
        - MPC Trust: Detects "bad assignments" (leader's assignment is worse than local optimum)
        
        Robustness features:
        - Deadband: ignore small mismatches |U_pred-U_obs| <= trust_deadband
        - Event gating: only penalize when leader claimed a meaningful gain but delivered almost none
        - Drop cap: bound the maximum one-step decay to avoid cascading impeachments on noise
        
        Citations:
        - Trust update: exponential decay is a common choice in control/monitoring (Rawlings & Mayne 2009)
        - Hybrid trust: combining multiple trust signals improves robustness (Josang & Ismail 2002)
        """
        leader = self.leader_id
        
        # Prediction-Observation Trust (existing mechanism)
        # Citations: Shannon (1948) - belief-only gains to avoid oracle leakage
        raw_err = (predicted_gain - observed_gain)
        pred_obs_error = max(0.0, raw_err - max(0.0, self.trust_deadband))
        
        # Gate using RAW prediction ratio (not scaled predicted_gain)
        # This decouples gating from pred_gain_alpha scaling
        raw_pred_ratio = getattr(self, '_raw_pred_ratio', predicted_gain)
        if not (raw_pred_ratio >= self.trust_event_pred_min and observed_gain <= self.trust_event_obs_max):
            pred_obs_error = 0.0
        
        # MPC Trust (new mechanism)
        # Citations: Rawlings & Mayne (2009), Camacho & Bordons (2004) - MPC for local verification
        mpc_error_value = float(mpc_error) if mpc_error is not None else 0.0
        
        # Combine trust errors (hybrid mechanism)
        # Citations: Josang & Ismail (2002) - combining multiple trust signals improves robustness
        # Weighted combination: pred_obs_error and mpc_error_value
        # Normalize both errors to [0,1] range for fair combination
        pred_obs_error_norm = min(1.0, pred_obs_error)  # Normalize to [0,1]
        mpc_error_norm = min(1.0, mpc_error_value)  # Normalize to [0,1]
        
        # Weighted combination
        if self.mpc_trust_enabled and mpc_error is not None:
            # Hybrid: combine prediction-observation and MPC trust
            error = (1.0 - self.mpc_trust_weight) * pred_obs_error_norm + self.mpc_trust_weight * mpc_error_norm
        else:
            # Prediction-observation only (original mechanism)
            error = pred_obs_error_norm
    # Exponential decay with a floor to cap one-step loss
        decay = math.exp(-self.trust_decay_rate * error)
        min_decay = max(0.0, 1.0 - max(0.0, self.trust_decay_cap))
        decay = max(min_decay, decay)
        for i in range(self.N):
            if i == leader:
                continue
            # Make the trust only go between the min and 1.0 (so systems don't go out of control)
            self.trust[i][leader] = max(self.min_trust, min(1.0, self.trust[i][leader] * decay))
            # recovery for non-leaders
            for j in range(self.N):
                if j != leader and self.trust[i][j] < 1.0:
                    self.trust[i][j] = min(1.0, self.trust[i][j] + self.trust_recovery_rate)

    # Citations:
    # - Model Predictive Control (MPC) for local verification: Rawlings & Mayne (2009), Camacho & Bordons (2004)
    # - Distributed trust evaluation: enables agents to verify leadership quality without global information
    def compute_mpc_trust_error(self, env, agent_id, leader_assignment, frontiers, component):
        """Compute MPC-based trust error: difference between local optimum and leader assignment.
        
        For each follower agent, compute locally optimal frontier from their own belief_lr,
        then compare to leader's assignment. If leader's assignment is significantly worse
        than local optimum, return a positive error (indicating trust should decay).
        
        Args:
            env: GridWorld environment
            agent_id: ID of follower agent to evaluate
            leader_assignment: Leader's assignment for this agent (x, y) tuple
            frontiers: List of available frontiers [(x, y), ...]
            component: Set of agent IDs in communication component
            
        Returns:
            mpc_error: Float error value (0.0 if leader assignment is good, positive if bad)
            local_optimum: (x, y) tuple of locally optimal frontier, or None
        """
        if not self.mpc_trust_enabled or agent_id == self.leader_id:
            return 0.0, None
        
        agent = env.agents.get(agent_id)
        if agent is None:
            return 0.0, None
        
        # Get agent's local belief map (already compressed/entropy-based)
        # Citations: Shannon (1948) - entropy as information measure
        belief_lr = getattr(agent, 'belief_lr', None)
        if belief_lr is None or not frontiers:
            return 0.0, None
        
        agent_pos = env.agent_positions.get(agent_id)
        if agent_pos is None:
            return 0.0, None
        
        # Compute local optimum: best frontier from agent's belief
        # Use same scoring logic as frontier assignment, but from agent's local belief
        # Citations: Yamauchi (1997) - frontier-based exploration with entropy scoring
        local_optimum = None
        best_score = -float('inf')
        ds = getattr(agent, 'ds', 4)
        window = self.mpc_trust_window
        
        # Score each frontier from agent's local belief
        for f in frontiers:
            fx, fy = f
            gx, gy = fx // ds, fy // ds
            
            # Check if frontier is in local window (computational efficiency)
            dist_to_frontier = abs(fx - agent_pos[0]) + abs(fy - agent_pos[1])
            max_dist = self.mpc_trust_window * ds * 2  # Limit search to local window
            if dist_to_frontier > max_dist:
                continue
            
            # Score frontier based on unknown cells in window around it (from agent's belief)
            score = 0.0
            unknown_count = 0
            total_count = 0
            
            for dx in range(-window, window + 1):
                for dy in range(-window, window + 1):
                    ngx, ngy = gx + dx, gy + dy
                    if 0 <= ngx < belief_lr.shape[0] and 0 <= ngy < belief_lr.shape[1]:
                        v = belief_lr[ngx, ngy]
                        total_count += 1
                        if v == -1 or v == 1:  # Unknown (coarse belief: -1 = unknown, 1 = obstacle-unknown)
                            unknown_count += 1
            
            if total_count > 0:
                # Score = fraction of unknown cells (information gain potential)
                # Distance penalty (prefer nearby frontiers)
                unknown_frac = float(unknown_count) / float(total_count)
                dist_penalty = float(dist_to_frontier) / float(max(1, ds * window * 2))
                score = unknown_frac - 0.2 * dist_penalty  # Weight: information > distance
                
                # Only update best_score when score is valid (total_count > 0)
                if score > best_score:
                    best_score = score
                    local_optimum = f
        
        if local_optimum is None:
            return 0.0, None
        
        # Compare leader's assignment to local optimum
        if leader_assignment == local_optimum:
            # Leader's assignment matches local optimum → no error
            return 0.0, local_optimum
        
        # Compute score for leader's assignment from agent's belief
        leader_fx, leader_fy = leader_assignment
        leader_gx, leader_gy = leader_fx // ds, leader_fy // ds
        leader_unknown_count = 0
        leader_total_count = 0
        
        for dx in range(-window, window + 1):
            for dy in range(-window, window + 1):
                ngx, ngy = leader_gx + dx, leader_gy + dy
                if 0 <= ngx < belief_lr.shape[0] and 0 <= ngy < belief_lr.shape[1]:
                    v = belief_lr[ngx, ngy]
                    leader_total_count += 1
                    if v == -1 or v == 1:
                        leader_unknown_count += 1
        
        leader_score = 0.0
        if leader_total_count > 0:
            leader_unknown_frac = float(leader_unknown_count) / float(leader_total_count)
            leader_dist = abs(leader_fx - agent_pos[0]) + abs(leader_fy - agent_pos[1])
            leader_dist_penalty = float(leader_dist) / float(max(1, ds * window * 2))
            leader_score = leader_unknown_frac - 0.2 * leader_dist_penalty
        
        # Error = difference between local optimum and leader assignment
        # Positive error means leader's assignment is worse than local optimum
        mpc_error = max(0.0, best_score - leader_score)
        
        # Only trigger trust decay if error exceeds threshold (ignore small differences)
        if mpc_error < self.mpc_trust_threshold:
            mpc_error = 0.0
        
        return mpc_error, local_optimum

    # Citations:
    # - Aggregate reputation toward a leader for governance decision-making (Lamport 1982; Castro & Liskov 1999 background).
    def get_average_trust_in_leader(self, voters: set | None = None):
        leader = self.leader_id
        if voters is None:
            voters = set(range(self.N))
        voters = {i for i in voters if i != leader}
        if not voters:
            return 1.0
        trust_sum = sum(self.trust[i][leader] for i in voters)
        return trust_sum / float(len(voters))

    # Citations:
    # - Threshold-based impeachment is a practical governance trigger (fault-tolerance literature; see Lamport 1982).
    def should_impeach(self):
        return self.get_average_trust_in_leader() < self.trust_threshold

    # Citations:
    # - Election via argmax of collective trust with tie-breaks for stability echoes distributed leader election heuristics (Lamport 1982; Castro & Liskov 1999).
    def elect_new_leader(self, voters: set | None = None, candidates: set | None = None):
        if voters is None:
            voters = set(range(self.N))
        if candidates is None:
            candidates = set(range(self.N))
        # Compute average trust toward each candidate from voters only
        scores = {}
        for candidate in candidates:
            vset = {i for i in voters if i != candidate}
            denom = max(1, len(vset))
            scores[candidate] = sum(self.trust[i][candidate] for i in vset) / float(denom)
        # Tie-breaker: among top-scoring candidates, prefer fewest leader_failures, then deterministic RNG
        max_score = max(scores.values())
        top = [c for c, v in scores.items() if abs(v - max_score) < 1e-12]
        if len(top) == 1:
            new_leader = top[0]
        else:
            # prefer candidate with fewest past failures
            fewest = min(self.leader_failures[c] for c in top)
            top2 = [c for c in top if self.leader_failures[c] == fewest]
            if len(top2) == 1:
                new_leader = top2[0]
            else:
                # deterministic RNG seeded from cfg seed
                try:
                    seed = int(self.full_cfg.get('env', {}).get('seed', 42))
                except Exception:
                    seed = 42
                rng = random.Random(seed + len(self.leader_history))
                new_leader = rng.choice(top2)
        if new_leader != self.leader_id:
            self.leader_failures[self.leader_id] += 1
            old = self.leader_id
            self.leader_id = new_leader
            self.election_count += 1
            if self.bayes_enabled:
                self.bayes_fault_prob[new_leader] = self.bayes_prior_fault
            self.leader_history.append({
                'timestep': len(self.leader_history),
                'old_leader': old,
                'new_leader': new_leader,
                'avg_trust': self.get_average_trust_in_leader()
            })
            return True
        return False

    def _gaussian_pdf(self, x, sigma):
        sigma = max(sigma, 1e-3)
        return (1.0 / (math.sqrt(2 * math.pi) * sigma)) * math.exp(-(x ** 2) / (2 * sigma ** 2))

    def _get_bayes_prob(self, agent):
        if not self.bayes_enabled:
            return 0.0
        return self.bayes_fault_prob.get(agent, self.bayes_prior_fault)

    def _update_bayes_fault_prob(self, residual, significant_error=False):
        if not self.bayes_enabled:
            return False
        leader = self.leader_id
        prior = self.bayes_fault_prob.get(leader, self.bayes_prior_fault)
        posterior = prior

        if significant_error:
            gated_residual = max(0.0, residual - self.trust_deadband)
            if gated_residual > 0.0:
                likelihood_fault = self._gaussian_pdf(gated_residual, self.bayes_sigma_fault)
                likelihood_healthy = self._gaussian_pdf(gated_residual, self.bayes_sigma_healthy)
                numerator = likelihood_fault * prior
                denominator = numerator + likelihood_healthy * (1.0 - prior)
                posterior = numerator / max(denominator, 1e-12)

        # Even if no significant error, gently decay toward prior so stale alarms fade.
        posterior = posterior * (1.0 - self.bayes_decay) + self.bayes_prior_fault * self.bayes_decay
        posterior = max(0.0, min(1.0, posterior))
        self.bayes_fault_prob[leader] = posterior
        return posterior >= self.bayes_threshold

    # -----------------------------
    # Main step
    # -----------------------------
    # Citations:
    # - Frontier-based assignment (Yamauchi 1997) with entropy-informed utility (Shannon 1948).
    # - Fault-tolerant governance loop (Lamport 1982; Castro & Liskov 1999).
    # - Pathfinding and LOS are handled in env/agents (Hart et al. 1968; Bresenham 1965) without oracle knowledge.
    def step(self, env, frontiers, state):
        t = state.get('t', 0)
        T = max(1, self.full_cfg.get('T', 200))

        # 1) Hallucination
        self.inject_hallucination(t)

        # 2) Compute current comm component around leader and measure local uncertainty
        component = self._comm_component(env, self.leader_id, self.command_radius)
        # Update link TTLs: refresh for component, decay for others
        for i in range(self.N):
            if i in component:
                self._link_ttl[i] = int(self.link_signature_ttl)
            else:
                self._link_ttl[i] = max(0, int(self._link_ttl.get(i, 0)) - 1)

        # Compute simple graph health metrics for adaptive scaling
        # Citations:
        # - Graph connectivity as a global objective (algebraic connectivity/"Fiedler value"): Fiedler (1973).
        #   We avoid eigen-computation and use light local proxies: component count, average degree, and relay TTL risk.
        # - Connectivity preservation via distributed constraints: Ji & Egerstedt (2007); Zavlanos & Pappas (2005).
        ids = list(env.agent_positions.keys())
        degs = []
        at_risk = 0
        # Build adjacency and also use it to count components (cheap O(N^2))
        adj = {i: set() for i in ids}
        for i in ids:
            pos_i = env.agent_positions[i]
            deg = 0
            for j in ids:
                if j == i: continue
                pos_j = env.agent_positions[j]
                if abs(pos_i[0]-pos_j[0]) + abs(pos_i[1]-pos_j[1]) <= int(self.command_radius):
                    deg += 1
                    adj[i].add(j)
            degs.append(deg)
            if int(self._link_ttl.get(i, 0)) <= int(self.link_risk_ttl_thresh):
                at_risk += 1
        # Count connected components using the adjacency and collect membership
        comp_count = 0
        seen = set()
        components = []
        for i in ids:
            if i in seen:
                continue
            comp_count += 1
            stack = [i]
            seen.add(i)
            comp = {i}
            while stack:
                cur = stack.pop()
                for nb in adj.get(cur, ()): 
                    if nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
                        comp.add(nb)
            components.append(comp)
        avg_deg = (sum(degs) / float(len(degs))) if degs else 0.0
        risk_deg = max(0.0, min(1.0, (self.adapt_deg_safe - avg_deg) / max(1e-6, self.adapt_deg_safe)))
        risk_ttl = (at_risk / float(len(ids))) if ids else 0.0
        # Component risk: 0 when fully connected, increases with more components (proxy for low Fiedler value)
        risk_comp = 0.0
        if comp_count > 1 and len(ids) > 1:
            risk_comp = min(1.0, (comp_count - 1) / float(len(ids) - 1))
        risk = max(risk_deg, risk_ttl, risk_comp)
        scale = self.adapt_scale_min + (1.0 - self.adapt_scale_min) * risk
        # Effective weights
        eff_connectivity_beta = self.connectivity_beta * (scale if self.adaptive_connectivity else 1.0)
        eff_tether_lambda = self.tether_lambda * (scale if self.adaptive_connectivity else 1.0)
        eff_degree_under = self.degree_under_penalty * (scale if self.adaptive_connectivity else 1.0)
        eff_spacing = int(self.frontier_spacing_radius + (self.frontier_spacing_extra_safe * (1.0 - risk) if self.adaptive_connectivity else 0))
        # Expose for telemetry
        self.last_avg_degree = avg_deg
        self.last_component_count = comp_count
        self.last_ttl_risk_frac = risk_ttl
        self.last_connectivity_scale = scale
        self.last_eff_connectivity_beta = eff_connectivity_beta
        self.last_eff_tether_lambda = eff_tether_lambda
        self.last_eff_degree_under = eff_degree_under
        self.last_eff_spacing = eff_spacing
        leader_agent = env.agents.get(self.leader_id, None)
        leader_belief_lr = getattr(leader_agent, "belief_lr", None) if leader_agent is not None else None
        leader_ds = max(1, int(getattr(leader_agent, "ds", 1))) if leader_agent is not None else 1

        # Use pred_gain_radius to set the trust measurement window to a small local area (leader belief only)
        H_before = self._measure_uncertainty_component(env, component, leader_belief_lr, leader_ds, window_r=self.pred_gain_radius)

        # Optional: Component failover. If the leader is not in the largest component,
        # immediately transfer leadership within that largest component so assignments/tethers
        # are made with a connected leader. This avoids a stranded leader skewing global
        # decisions. Constrained to intra-component election to respect no-teleport rule.
        try:
            enable_failover = bool(self.reip_cfg.get('enable_component_failover', True))
        except Exception:
            enable_failover = True
        enable_failover = enable_failover and self.governance_enabled
        if enable_failover and components:
            largest = max(components, key=lambda c: len(c))
            if self.leader_id not in largest and len(largest) > 0:
                # Elect within the largest component using local voters/candidates
                changed = self.elect_new_leader(voters=largest, candidates=largest)
                if changed:
                    # Recompute leader-centric structures for this step (component, parent tree, etc.)
                    component = self._comm_component(env, self.leader_id, self.command_radius)
                    try:
                        env.leader_id = self.leader_id
                    except Exception:
                        pass
                    # Update leader belief snapshot for the new leader
                    leader_agent = env.agents.get(self.leader_id, None)
                    leader_belief_lr = getattr(leader_agent, "belief_lr", None) if leader_agent is not None else None
                    leader_ds = max(1, int(getattr(leader_agent, "ds", 1))) if leader_agent is not None else leader_ds
                    # Note: H_before remains a belief-only local measure; recompute for new component
                    H_before = self._measure_uncertainty_component(env, component, leader_belief_lr, leader_ds, window_r=self.pred_gain_radius)
                    try:
                        env.leader_id = self.leader_id
                    except Exception:
                        pass

    # 3) Leader assignment (entropy-global)
        state.setdefault('prev', {})
        prev_assigns = state['prev']
        prox_beta = self.base_prox * (1.0 + self.dispersion_alpha * (t / float(T)))
        lam = self.full_cfg.get('lam', 0.0)
        window = self.reip_cfg.get('window', 2)
        persist_bonus = self.reip_cfg.get('persist_bonus', 0.2)
        switch_margin = self.reip_cfg.get('switch_margin', 0.05)
        prox_radius = self.reip_cfg.get('prox_radius', 6)

        if frontiers:
            # Optional: compute beacons (parent tree, hops) for connectivity tethers
            # Always compute the beacon tree so connected assignment can enforce tethers
            # Citations in compute_beacon_tree()
            parent_map, hop_map = compute_beacon_tree(env, leader_id=self.leader_id, R=self.command_radius)

            # Optional leader heading vector derived from current leader claim or assignment
            leader_heading = None
            try:
                lp = env.agent_positions.get(self.leader_id, None)
                # Prefer previous held goal as heading target
                if lp is not None:
                    leader_goal = None
                    if prev_assigns and self.leader_id in prev_assigns:
                        leader_goal = prev_assigns.get(self.leader_id)
                    if leader_goal is None and len(frontiers) > 0:
                        # fallback: nearest frontier to leader
                        leader_goal = min(frontiers, key=lambda f: abs(f[0]-lp[0]) + abs(f[1]-lp[1]))
                    if leader_goal is not None:
                        dx = leader_goal[0] - lp[0]; dy = leader_goal[1] - lp[1]
                        norm = max(1.0, (abs(dx) + abs(dy)))
                        leader_heading = (dx / norm, dy / norm)
            except Exception:
                leader_heading = None

            # Assignment Algorithm Selection
            # ===============================
            # Choose between greedy BFS (default) or auction-based CBBA for task allocation
            # 
            # FAIR COMPARISON: In clean conditions (no faults), use same algorithm as baseline
            # (assign_frontiers_entropy_global) to ensure fair comparison. REIP's advantage comes from
            # governance (trust tracking, impeachment), not from a different assignment algorithm.
            # 
            # Citations for multi-robot task allocation:
            # - Choi, Han-Lim, Brunet, Luc, & How, Jonathan P. (2009). "Consensus-Based Decentralized 
            #   Auctions for Robust Task Allocation." IEEE Transactions on Robotics, 25(4), 912-926.
            # - Gerkey, Brian P., & Matarić, Maja J. (2004). "A Formal Analysis and Taxonomy of Task 
            #   Allocation in Multi-Robot Systems." The International Journal of Robotics Research, 23(9).
            # - Zlot, Robert, & Stentz, Anthony (2006). "Market-based Multirobot Coordination for Complex 
            #   Tasks." The International Journal of Robotics Research, 25(1), 73-101.

            # Track whether environment is clean for fallback logic (governance features stay identical)
            is_clean = not self.hallucination_active and not getattr(self, 'attack_active', False)
            self._is_clean_condition = is_clean

            # Normalize requested assignment algorithm so clean/faulted paths stay identical.
            algo_name = (self.assignment_algorithm or "greedy_bfs").lower()

            if algo_name in ("entropy_global", "baseline", "baseline_entropy"):
                assigns, claims, U_pred_entropy = assign_frontiers_entropy_global(
                    env, frontiers,
                    lam=lam,
                    prev_assigns=prev_assigns,
                    window=window,
                    prox_beta=prox_beta,
                    prox_radius=prox_radius,
                    persist_bonus=persist_bonus,
                    switch_margin=switch_margin,
                    leader_id=self.leader_id,
                    leader_belief_lr=leader_belief_lr
                )
            elif algo_name == "cbba":
                # CBBA: Auction-based assignment with spatial exclusion zones
                # Better for parallel spread, eliminates sequential greedy bias
                from src.policy.frontier import compute_frontier_scores
                
                # Build frontier scores (entropy-based utility)
                frontier_scores = compute_frontier_scores(env, frontiers, window, leader_belief_lr=leader_belief_lr, leader_id=self.leader_id)
                
                assigns, claims, U_pred_entropy = cbba_frontier_assignment(
                    env, frontiers, 
                    agents_list=list(env.agent_positions.keys()),
                    comm_radius=self.command_radius,
                    fov_radius=self.reip_cfg.get('fov_radius', getattr(env, 'sensor_radius', 6)),
                    frontier_score_map=frontier_scores,
                    max_bids_per_agent=3,
                    spacing_radius=self.frontier_spacing_radius,
                    strict_fov_exclusion=bool(self.reip_cfg.get('strict_fov_exclusion', False)),
                    fov_shape=self.reip_cfg.get('fov_shape', 'square'),
                    fov_margin=int(self.reip_cfg.get('fov_margin', 0)),
                    connectivity_strictness=float(self.reip_cfg.get('connectivity_strictness', 1.0)),
                    tether_constraint=True,
                    allow_distance=max(0, int(self.command_radius) - int(self.tether_margin)),
                    leader_id=self.leader_id,
                )
                
                # CBBA returns simple assignment dict; wrap for compatibility
                if not isinstance(claims, dict):
                    claims = {i: assigns.get(i, env.agent_positions.get(i, (0,0))) 
                             for i in env.agent_positions.keys()}
                
            else:
                # Default: Greedy BFS-ordered connected assignment
                # Connected assignment: enforce spanning-tree tethers in BFS order from leader.
                # This preserves global connectivity by construction (Ji & Egerstedt 2007; Zavlanos & Pappas 2005).
                # Lightweight capacity telemetry: how much parallel room exists near the leader?
                try:
                    lp = env.agent_positions.get(self.leader_id, None)
                    allow = max(0, int(self.command_radius) - int(self.tether_margin))
                    spacing_est = int(self.reip_cfg.get('frontier_spacing_radius', 3))
                    cap_slots = self._estimate_frontier_capacity(env, frontiers, lp, allow, spacing_est)
                    # Save for logger/plots if desired
                    self.last_capacity_slots = int(cap_slots)
                    # component defined earlier
                    self.last_component_size = int(len(component) if component else len(env.agent_positions))
                    if self.debug_prints and t % 25 == 0:
                        self._debug(f"[CAPACITY] t={t}: reachable_slots={cap_slots}, component_agents={self.last_component_size}, total_frontiers={len(frontiers)}")
                except Exception:
                    pass

                assigns, claims, U_pred_entropy = assign_frontiers_connected(
                    env, frontiers,
                    prev_assigns=prev_assigns,
                    window=window,
                    prox_beta=prox_beta,
                    prox_radius=prox_radius,
                    persist_bonus=persist_bonus,
                    switch_margin=switch_margin,
                    comm_radius=self.command_radius,
                    leader_id=self.leader_id,
                    parent_map=parent_map,
                    tether_margin=self.tether_margin,
                    region_size=self.region_size,
                    region_capacity=self.region_capacity,
                    triangle_penalty=self.triangle_penalty,
                    degree_target=self.degree_target,
                    degree_over_penalty=self.degree_over_penalty,
                    degree_min_target=self.degree_min_target,
                    degree_under_penalty=eff_degree_under,
                    spacing_radius=self.last_eff_spacing if hasattr(self, 'last_eff_spacing') else self.frontier_spacing_radius,
                    sibling_spacing_radius=None,
                    sibling_penalty=0.4,
                    sibling_angle_bins=self.reip_cfg.get('sibling_angle_bins', 8),
                    sibling_angle_penalty=self.reip_cfg.get('sibling_angle_penalty', 0.2),
                    allow_anchor_any=self.reip_cfg.get('allow_anchor_any', True),
                    repulse_beta=self.reip_cfg.get('repulse_beta', 0.06),
                    repulse_radius=self.reip_cfg.get('repulse_radius', 3),
                    sibling_sector_capacity=self.reip_cfg.get('sibling_sector_capacity', 1),
                    unique_gain_beta=self.reip_cfg.get('unique_gain_beta', 0.0),
                    # Default uniqueness radius to sensor FOV if provided
                    unique_radius=self.reip_cfg.get('unique_radius', getattr(env, 'sensor_radius', self.reip_cfg.get('pred_gain_radius', 3))),
                    # FOV-aware anti-overlap between planned agents
                    fov_overlap_beta=self.reip_cfg.get('fov_overlap_beta', 0.0),
                    fov_radius=self.reip_cfg.get('fov_radius', getattr(env, 'sensor_radius', None)),
                    leader_belief_lr=leader_belief_lr,
                )

            # Capacity guard: throttle how many agents actively push the frontier when slots are scarce.
            try:
                guard_enable = bool(self.reip_cfg.get('capacity_guard_enable', True))
                guard_slack = int(self.reip_cfg.get('capacity_guard_slack', 1))
            except Exception:
                guard_enable, guard_slack = True, 1
            if guard_enable and assigns and claims:
                # Determine center and allowable tether distance
                leader_center = claims.get(self.leader_id, env.agent_positions.get(self.leader_id))
                allow = max(0, int(self.command_radius) - int(self.tether_margin))
                spacing_for_slots = self.frontier_spacing_radius if getattr(self, 'frontier_spacing_radius', 0) > 0 else self.reip_cfg.get('prox_radius', 6)
                slots = int(self._estimate_frontier_capacity(env, frontiers, leader_center, allow, spacing_for_slots))
                K = max(1, slots + max(0, guard_slack))
                # Build BFS order from parent_map (leader first)
                ids = list(env.agent_positions.keys())
                order = []
                if parent_map:
                    children = {i: [] for i in ids}
                    for child, par in parent_map.items():
                        if par in children:
                            children[par].append(child)
                    from collections import deque as _dq
                    q = _dq([self.leader_id])
                    seen = set()
                    while q:
                        u = q.popleft()
                        if u in seen:
                            continue
                        seen.add(u)
                        order.append(u)
                        for v in children.get(u, []):
                            q.append(v)
                    for i in ids:
                        if i not in seen:
                            order.append(i)
                else:
                    order = ids
                # If more agents than K, convert the remainder to support targets (parent claims)
                if len(order) > K:
                    active = set(order[:K])
                    for i in order[K:]:
                        if i not in assigns:
                            continue
                        # keep connectivity by asking follower to hold at parent's planned target (or nearest teammate)
                        parent_i = parent_map.get(i) if parent_map else None
                        support = None
                        if parent_i is not None:
                            support = claims.get(parent_i, env.agent_positions.get(parent_i))
                        if support is None:
                            # fallback: nearest teammate inside comm radius
                            pos_i = env.agent_positions.get(i)
                            bestd = 10**9; chosen = None
                            for j, pj in env.agent_positions.items():
                                if j == i:
                                    continue
                                d = abs(pj[0]-pos_i[0]) + abs(pj[1]-pos_i[1])
                                if d < bestd and d <= int(self.command_radius):
                                    bestd = d; chosen = pj
                            support = chosen if chosen is not None else env.agent_positions.get(i)
                        assigns[i] = support
                # Telemetry
                self.last_capacity_slots = K
        else:
            assigns = {i: env.agent_positions[i] for i in env.agent_positions.keys()}
            claims = assigns.copy()
            U_pred_entropy = 0.0
        
        # Store claims (leader's original assignments) for MPC trust computation
        # Citations: MPC trust compares leader's assignments to local optima (Rawlings & Mayne 2009)
        self.last_claims = claims.copy() if claims else {}

        if t == 0 or 'held_cmds' not in state:
            # Initialize held commands and seed each agent's persistent goal without stepping twice
            state['held_cmds'] = {}
            for i, tgt in assigns.items():
                a = env.agents.get(i)
                if a is not None:
                    a.hold_target = tgt
                    a.hold_timer = getattr(env, 'min_hold', 3)
                state['held_cmds'][i] = {
                    'target': tgt,
                    'command_id': 0,
                    'issued_at': t,
                }
            # Persist what agents are expected to hold going into the unified step below
            state['prev'] = assigns.copy()
        else:
            # Respect existing holds; only retask agents whose hold has expired
            next_assigns = state['prev'].copy()
            for i, new_tgt in assigns.items():
                a = env.agents.get(i)
                hold_left = getattr(a, 'hold_timer', 0) if a is not None else 0
                if hold_left <= 0 and next_assigns.get(i) != new_tgt:
                    # Issue a new command and reset hold
                    next_assigns[i] = new_tgt
                    if a is not None:
                        a.hold_target = new_tgt
                        a.hold_timer = getattr(env, 'min_hold', 3)
                    state['held_cmds'][i] = {
                        'target': new_tgt,
                        'command_id': t,
                        'issued_at': t,
                    }
            # Use the held/updated assignments as the current plan
            assigns = next_assigns
        # Predict information gain using leader belief only (no team map leakage)
        # Build a union of coarse cells within pred_gain_radius around each claimed target for agents near their target.
        def _coarse_window(center, radius, ds, shape):
            cx, cy = center
            gx0 = max(0, (cx - radius) // ds)
            gy0 = max(0, (cy - radius) // ds)
            gx1 = min(shape[0] - 1, (cx + radius) // ds)
            gy1 = min(shape[1] - 1, (cy + radius) // ds)
            for gx in range(gx0, gx1 + 1):
                for gy in range(gy0, gy1 + 1):
                    yield (gx, gy)

        pred_union_cells = set()
        if leader_belief_lr is not None and claims:
            for aid, f in claims.items():
                if aid not in component or f is None:
                    continue
                pos_agent = env.agent_positions.get(aid)
                if pos_agent is not None:
                    dist_to_target = abs(f[0] - pos_agent[0]) + abs(f[1] - pos_agent[1])
                    if dist_to_target > 2 * self.pred_gain_radius:
                        continue
                for cell in _coarse_window(f, self.pred_gain_radius, leader_ds, leader_belief_lr.shape):
                    pred_union_cells.add(cell)

        # Helper: check if cell is unknown to entire component (for observed gain)
        def component_unknown(gx, gy, comp):
            for aid in comp:
                a = env.agents.get(aid)
                if a is None:
                    continue
                blr = getattr(a, "belief_lr", None)
                if blr is None:
                    continue
                if gx < blr.shape[0] and gy < blr.shape[1]:
                    v = blr[gx, gy]
                    if v != -1 and v != 1:  # known free or obstacle
                        return False
            return True
        
        # Helper: check if cell is unknown from LEADER's perspective only (for prediction)
        # This is what the leader CLAIMS is unknown when making assignments
        def leader_claims_unknown(gx, gy):
            if leader_belief_lr is None:
                return True
            if gx < leader_belief_lr.shape[0] and gy < leader_belief_lr.shape[1]:
                v = leader_belief_lr[gx, gy]
                if v == -1 or v == 1:  # unknown or obstacle-unknown
                    return True
            return False
        
        # Count unknown cells from LEADER'S perspective for PREDICTION
        # This is what the leader CLAIMS will be gained - their promise to the team
        # If leader is faulty, they may claim areas are unknown when they're not
        predicted_unknown_before = sum(
            1 for (gx, gy) in pred_union_cells 
            if leader_claims_unknown(gx, gy)
        )
        self._last_predicted_unknown = predicted_unknown_before  # Debug tracking
        
        # Store component_unknown for use in observed gain calculation (step 6)
        self._component_unknown = component_unknown
        self._component_for_trust = component

        # Compute RAW prediction ratio (0-1) for trust event gating
        # This is independent of pred_gain_alpha scaling
        denom = max(1, len(pred_union_cells))
        raw_pred_ratio = predicted_unknown_before / denom  # Raw ratio for gating
        
        # Scale for the actual predicted_gain value (used in trust update magnitude)
        predicted_gain = min(1.0, raw_pred_ratio * self.pred_gain_alpha)
        
        # Store raw ratio for trust event gating (separate from scaled gain)
        self._raw_pred_ratio = raw_pred_ratio
        
        attack_window = (getattr(self, 'hallucination_start_t', None) is not None and getattr(self, 'hallucination_end_t', None) is not None)
        attack_active_now = bool(self.hallucination_active and attack_window and (t >= getattr(self, 'hallucination_start_t') and t < getattr(self, 'hallucination_end_t')) and (getattr(self, 'hallucination_origin_leader', None) == self.leader_id))
        if attack_active_now:
            jitter = random.uniform(0, self.pred_gain_inflation_jitter) if self.pred_gain_inflation_jitter > 0 else 0.0
            factor = max(1.0, self.pred_gain_inflation_base + jitter)
            predicted_gain = min(1.0, predicted_gain * factor)
        self.predicted_coverage_gain = predicted_gain

        # 4) Corrupt and filter commands
        # Store assigns before corruption for MPC trust comparison
        # Citations: MPC trust compares leader's original assignments to local optima (Rawlings & Mayne 2009)
        self._last_assigns_before_corruption = assigns.copy()
        assigns = self.corrupt_assignments(env, assigns, frontiers)
        original_assigns = assigns.copy()  # Keep original for comparison
        original_count = len(assigns)
        assigns = filter_assignments_by_comm(
            env, assigns,
            comm_radius=self.command_radius,
            p_cmd_loss=self.command_loss_rate,
            leader_id=self.leader_id,
            multi_hop=self.multi_hop_commands
        )
        # Track which agents lost commands (for fallback detection)
        agents_without_commands = set()
        for agent_id in env.agent_positions.keys():
            if agent_id not in assigns:
                agents_without_commands.add(agent_id)
            elif agent_id in original_assigns:
                # Check if agent's command was filtered (distance or loss)
                original_target = original_assigns.get(agent_id)
                filtered_target = assigns.get(agent_id)
                agent_pos = env.agent_positions.get(agent_id)
                # If filtered target is current position or hold_target, command was lost
                if filtered_target == agent_pos or (filtered_target != original_target and filtered_target == env.agents.get(agent_id).hold_target if env.agents.get(agent_id) else False):
                    agents_without_commands.add(agent_id)
        
        lost_count = original_count - len([a for a in assigns.values() if a in [v for v in assigns.values()]])
        self.command_loss_events += lost_count

        # Optional regroup: if an agent's current neighbor degree < threshold, guide it toward the nearest teammate
        # (local connectivity maintenance; Ji & Egerstedt 2007; Zavlanos & Pappas 2005)
        if self.enable_regroup and len(env.agent_positions) > 1:
            ids = list(env.agent_positions.keys())
            for i in ids:
                pos_i = env.agent_positions[i]
                # Compute current degree within command radius
                deg = 0
                nearest = None; bestd = 10**9
                for j in ids:
                    if j == i:
                        continue
                    pos_j = env.agent_positions[j]
                    d = abs(pos_i[0]-pos_j[0]) + abs(pos_i[1]-pos_j[1])
                    if d <= int(self.command_radius):
                        deg += 1
                    if d < bestd:
                        bestd = d; nearest = pos_j
                if deg < int(self.regroup_min_degree) and nearest is not None:
                    # Override assignment gently toward nearest teammate to rejoin the mesh
                    assigns[i] = nearest
                    a = env.agents.get(i)
                    if a is not None:
                        a.hold_target = nearest
                        a.hold_timer = getattr(env, 'min_hold', 3)

        # Local autonomy fallback to avoid stagnation when no effective command is guiding motion
        fallback_applied = 0
        if self.fallback_enabled:
            block_fallback = bool(attack_active_now and self.disable_fallback_during_attack)
            if frontiers:
                fallback_frontiers = frontiers
            else:
                fallback_frontiers = env.detect_frontiers_from_agent_belief(leader_agent) if leader_agent is not None else env.detect_frontiers()
            
            for i, pos in env.agent_positions.items():
                a = env.agents.get(i)
                assigned = assigns.get(i, pos)
                hold_tgt = getattr(a, 'hold_target', None) if a is not None else None
                
                no_op = (assigned == pos) or (hold_tgt == pos)
                disconnected = (component is not None) and (i not in component)
                command_lost = i in agents_without_commands

                needs_fallback = (
                    disconnected
                    or (self.fallback_on_command_loss and command_lost)
                    or (self.fallback_on_noop and no_op)
                )
                
                if needs_fallback and not getattr(self, '_is_clean_condition', False):
                    if block_fallback:
                        continue
                    try:
                        if fallback_frontiers:
                            nearest = min(fallback_frontiers, key=lambda f: abs(f[0]-pos[0]) + abs(f[1]-pos[1]))
                        else:
                            if a is not None and hasattr(a, 'belief_lr'):
                                best_target = None
                                best_dist = float('inf')
                                ds = a.ds
                                for gx in range(a.belief_lr.shape[0]):
                                    for gy in range(a.belief_lr.shape[1]):
                                        if a.belief_lr[gx, gy] == 1 or a.belief_lr[gx, gy] == -1:
                                            wx = gx * ds + (ds // 2)
                                            wy = gy * ds + (ds // 2)
                                            dist = abs(wx - pos[0]) + abs(wy - pos[1])
                                            if dist < best_dist:
                                                best_dist = dist
                                                best_target = (wx, wy)
                                if best_target:
                                    nearest = best_target
                                else:
                                    dx = env.size // 2 - pos[0]
                                    dy = env.size // 2 - pos[1]
                                    norm = max(1, abs(dx) + abs(dy))
                                    step_size = min(5, max(2, int(self.command_radius * 0.3)))
                                    nearest = (pos[0] + int(dx * step_size / norm),
                                               pos[1] + int(dy * step_size / norm))
                                    nearest = (max(0, min(env.size - 1, nearest[0])),
                                               max(0, min(env.size - 1, nearest[1])))
                            else:
                                center = (env.size // 2, env.size // 2)
                                dx = center[0] - pos[0]
                                dy = center[1] - pos[1]
                                step = min(3, max(1, int(self.command_radius * 0.2)))
                                if abs(dx) + abs(dy) > 0:
                                    norm = max(1, abs(dx) + abs(dy))
                                    nearest = (pos[0] + int(dx * step / norm),
                                               pos[1] + int(dy * step / norm))
                                    nearest = (max(0, min(env.size - 1, nearest[0])),
                                               max(0, min(env.size - 1, nearest[1])))
                                else:
                                    nearest = pos
                        
                        assigns[i] = nearest
                        if a is not None:
                            a.hold_target = nearest
                            a.hold_timer = getattr(env, 'min_hold', 3)
                        fallback_applied += 1
                        if self.debug_prints and t % 10 == 0:
                            reason = []
                            if disconnected:
                                reason.append("disconnected")
                            if self.fallback_on_command_loss and command_lost:
                                reason.append("command_lost")
                            if self.fallback_on_noop and no_op:
                                reason.append("no_op")
                            self._debug(f"[FALLBACK] t={t}: Agent {i} ({', '.join(reason)}), assigned to {nearest}")
                    except Exception:
                        if a is not None and a.hold_target is None:
                            assigns[i] = pos
                    continue
                
                if self.fallback_on_noop and no_op:
                    if block_fallback:
                        continue
                    try:
                        if fallback_frontiers:
                            nearest = min(fallback_frontiers, key=lambda f: abs(f[0]-pos[0]) + abs(f[1]-pos[1]))
                            assigns[i] = nearest
                            if a is not None:
                                a.hold_target = nearest
                                a.hold_timer = getattr(env, 'min_hold', 3)
                            fallback_applied += 1
                    except Exception:
                        pass
        
        # Light telemetry for tuning
        state['fallback_applied'] = fallback_applied
        self.last_fallback_applied = fallback_applied
        if self.debug_prints and fallback_applied > 0 and t % 10 == 0:
            self._debug(f"[FALLBACK STATS] t={t}: {fallback_applied} agents using autonomous fallback")

        # 5) Execute one env step
        # Ensure every agent has a dispatch entry so env.step applies hold logic
        dispatch = {}
        for i in env.agent_positions.keys():
            a = env.agents.get(i)
            held = getattr(a, 'hold_target', None)
            prev_tgt = state.get('prev', {}).get(i, env.agent_positions[i])
            dispatch[i] = assigns.get(i, held if held is not None else prev_tgt)
        env.step(dispatch, t=t)

        # Sync prev assigns with what agents actually hold
        synced = {}
        for i in env.agent_positions.keys():
            a = env.agents.get(i)
            held = getattr(a, 'hold_target', None)
            synced[i] = held if held is not None else dispatch.get(i, env.agent_positions[i])
        state['prev'] = synced

        # 6) Observe info gain re-counting on the same predicted windows to align normalization
        leader_agent_after = env.agents.get(self.leader_id, None)
        leader_belief_lr_after = getattr(leader_agent_after, "belief_lr", None) if leader_agent_after is not None else None
        leader_ds_after = max(1, int(getattr(leader_agent_after, "ds", leader_ds))) if leader_agent_after is not None else leader_ds

        H_after = self._measure_uncertainty_component(env, component, leader_belief_lr_after, leader_ds_after, window_r=self.pred_gain_radius)

        # Use same component_unknown definition for remaining count
        # This ensures before/after are measured consistently
        component_unknown = getattr(self, '_component_unknown', None)
        trust_component = getattr(self, '_component_for_trust', component)
        
        remaining_unknown = 0
        if pred_union_cells and component_unknown is not None:
            remaining_unknown = sum(
                1 for (gx, gy) in pred_union_cells 
                if component_unknown(gx, gy, trust_component)
            )
        elif pred_union_cells:
            # Fallback to old logic if helper not available
            for (gx, gy) in pred_union_cells:
                known_here = False
                for aid in component:
                    a = env.agents.get(aid)
                    if a is None or not hasattr(a, "belief_lr"):
                        continue
                    blr = a.belief_lr
                    if gx < blr.shape[0] and gy < blr.shape[1]:
                        v = blr[gx, gy]
                        if v != -1 and v != 1:
                            known_here = True
                            break
                if not known_here:
                    remaining_unknown += 1

        observed_gain = 0.0
        if predicted_unknown_before > 0:
            observed_gain = max(0.0, min(1.0, (predicted_unknown_before - remaining_unknown) / float(predicted_unknown_before)))
        
        # Store for debugging
        self._last_observed_gain = observed_gain
        self._last_predicted_unknown = predicted_unknown_before
        self._last_remaining_unknown = remaining_unknown

        raw_error = predicted_gain - observed_gain
        
        # Compute MPC trust error (local optimum vs leader assignment)
        # Citations: Rawlings & Mayne (2009), Camacho & Bordons (2004) - MPC for local verification
        mpc_error_aggregate = 0.0
        if self.mpc_trust_enabled and frontiers:
            mpc_errors = []
            claims = getattr(self, 'last_claims', {})
            for aid in component:
                if aid == self.leader_id:
                    continue
                leader_assignment = claims.get(aid, None)
                if leader_assignment is None:
                    continue
                mpc_err, local_opt = self.compute_mpc_trust_error(env, aid, leader_assignment, frontiers, component)
                if mpc_err > 0:
                    mpc_errors.append(mpc_err)
            
            # Aggregate MPC errors (average across followers)
            # Citations: Josang & Ismail (2002) - aggregation methods for trust
            if mpc_errors:
                mpc_error_aggregate = sum(mpc_errors) / float(len(mpc_errors))
            else:
                mpc_error_aggregate = 0.0
        else:
            mpc_error_aggregate = None
        
        # Store for debugging/telemetry
        self._last_mpc_error = mpc_error_aggregate if mpc_error_aggregate is not None else 0.0
        self._last_mpc_error_count = len(mpc_errors) if self.mpc_trust_enabled and frontiers else 0
        
        # Update trust with hybrid mechanism (prediction-observation + MPC)
        # Citations: Hybrid trust mechanisms improve robustness (Josang & Ismail 2002)
        self.update_trust(predicted_gain, observed_gain, mpc_error=mpc_error_aggregate)
        avg_trust = self.get_average_trust_in_leader(voters=component)
        
        # CUSUM-based hallucination detection (Page 1954)
        if self.detection_enabled and self.hallucination_detector is not None:
            cusum_detected, cusum_metrics = self.hallucination_detector.update(
                self.leader_id, predicted_gain, observed_gain, t
            )
        else:
            cusum_detected = False
            cusum_metrics = {}
        
        # Update agent movement history for loop detection
        for agent_id, pos in env.agent_positions.items():
            if agent_id in self.agent_move_history:
                self.agent_move_history[agent_id].append(pos)
        
        # Update coverage history
        current_coverage = env.coverage()
        self.coverage_history.append(current_coverage)
        
        if self.detection_enabled and self.hallucination_detector is not None:
            loop_scores = self.hallucination_detector.compute_loop_score(
                env.agent_positions, self.agent_move_history
            )
            stagnation_detected = self.hallucination_detector.compute_coverage_stagnation(
                self.coverage_history, window=8, threshold=0.01
            )
        else:
            loop_scores = {}
            stagnation_detected = False

        bayes_alarm = False
        if self.bayes_enabled:
            # Use raw_pred_ratio for gating (not scaled predicted_gain)
            raw_pred_ratio = getattr(self, '_raw_pred_ratio', predicted_gain)
            significant_error = (
                raw_pred_ratio >= self.trust_event_pred_min
                and observed_gain <= self.trust_event_obs_max
            )
            bayes_alarm = self._update_bayes_fault_prob(abs(raw_error), significant_error)

        # Debug trust changes for CBBA tuning
        if self.debug_prints and (t % 10 == 0 or avg_trust < 0.9):
            self._debug(f"[TRUST DEBUG] t={t}: pred={predicted_gain:.3f}, obs={observed_gain:.3f}, avg_trust={avg_trust:.3f}, predicted_unknown={predicted_unknown_before:.0f}")
            if cusum_detected:
                self._debug(f"[CUSUM DETECTION] t={t}: Hallucination detected! CUSUM={cusum_metrics['cusum_value']:.2f}, Residual={cusum_metrics['residual']:.3f}")

        self.trust_history.append({
            'timestep': t,
            'leader': self.leader_id,
            'avg_trust': avg_trust,
            'predicted_gain': predicted_gain,
            'observed_gain': observed_gain,
            'hallucinating': attack_active_now,
            'hallucination_type': self.hallucination_type if attack_active_now else None,
            'origin_leader': getattr(self, 'hallucination_origin_leader', None),
            'attack_window': [getattr(self, 'hallucination_start_t', None), getattr(self, 'hallucination_end_t', None)],
            'attack_src': getattr(self, 'hallucination_active_src', None),
            'fallback_blocked': bool(self.hallucination_active and getattr(self, 'hallucination_origin_leader', None) == self.leader_id and self.disable_fallback_during_attack),
            'cusum_detected': cusum_detected,
            'cusum_value': cusum_metrics.get('cusum_value', 0.0),
            'loop_scores': loop_scores,
            'stagnation_detected': stagnation_detected,
            'bayes_fault_prob': self._get_bayes_prob(self.leader_id) if self.bayes_enabled else None
        })

        # 7) Impeach/elect if enabled
        if self.governance_enabled:
            impeachment_triggered = False
            if cusum_detected or bayes_alarm:
                impeachment_triggered = True
                self._debug(f"[REIP] CUSUM-based impeachment trigger at t={t}")
            elif avg_trust < self.trust_threshold:
                self._impeachment_below_count += 1
                if self._impeachment_below_count >= self.impeachment_persistence:
                    impeachment_triggered = True
            else:
                self._impeachment_below_count = 0

            can_impeach_now = impeachment_triggered and ((t - self._last_impeachment_t) > self.impeachment_cooldown)
            if can_impeach_now:
                prev_leader = self.leader_id
                attack_active = attack_active_now
                if self.elect_new_leader(voters=component, candidates=component):
                    self.impeachment_count += 1
                    self._last_impeachment_t = t
                    status = "🚨 DURING ATTACK" if attack_active else "normal"
                    self._debug(f"[REIP IMPEACHMENT] t={t}: Leader {prev_leader} → {self.leader_id} ({status})")
                    self._debug(f"   Trust: {avg_trust:.3f}, Predicted: {predicted_gain:.4f}, Observed: {observed_gain:.4f}")
        else:
            self._impeachment_below_count = 0

        # Persist for next iteration
        self.prev_entropy = H_after
        state['leader'] = self.leader_id
        # Track current goals per agent for visualization/logging
        try:
            state['current_goals'] = {i: getattr(env.agents.get(i), 'hold_target', None) for i in env.agent_positions.keys()}
        except Exception:
            state['current_goals'] = {}
        state['avg_trust'] = avg_trust
        state['election_count'] = self.election_count
        state['hallucinating'] = self.hallucination_active
        self.last_avg_trust = avg_trust
        # Telemetry snapshot for logger
        state['graph_avg_degree'] = avg_deg
        state['graph_component_count'] = comp_count
        state['graph_ttl_risk_frac'] = risk_ttl
        state['cohesion_scale'] = scale
        state['eff_connectivity_beta'] = eff_connectivity_beta
        state['eff_tether_lambda'] = eff_tether_lambda
        state['eff_degree_under'] = eff_degree_under
        state['eff_spacing'] = eff_spacing

    # Citations:
    # - Governance telemetry and auditing are common in resilient distributed systems to ensure accountability and analysis.
    def get_metrics(self):
        return {
            'election_count': self.election_count,
            'impeachment_count': self.impeachment_count,
            'final_leader': self.leader_id,
            'leader_failures': self.leader_failures,
            'trust_history': self.trust_history,
            'leader_history': self.leader_history,
            'final_avg_trust': self.get_average_trust_in_leader(),
            'command_loss_events': self.command_loss_events,
        }
