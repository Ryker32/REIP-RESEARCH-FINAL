"""
Baseline Leader-Follower Controller (No REIP Governance)

Traditional fixed-leader system with no trust tracking, no impeachment, no election.
Used as baseline to demonstrate REIP's advantages under adversarial conditions.

References:
- Burgard et al. (2005): Coordinated multi-robot exploration
- Simmons et al. (2000): Coordination for multi-robot exploration and mapping
- Traditional leader-follower architectures (centralized command & control)
"""

from src.policy.frontier import assign_frontiers_entropy_global
from src.comms.channel import filter_assignments_by_comm
from src.policy.faults import update_hallucination_state, corrupt_assignments_by_hallucination
import random
import math
import numpy as np


class BaselineLeaderFollower:
    """
    Fixed leader with no governance mechanisms.
    
    Leader is permanently Agent 0, regardless of performance.
    No trust tracking, no impeachment, no fault detection.
    """
    
    def __init__(self, cfg):
        self.full_cfg = cfg
        self.reip_cfg = cfg.get("reip", {})
        self.N = cfg.get("N", 1)
        
        # Leader rotation: fixed (default) or round robin
        self.round_robin_interval = cfg.get("round_robin_interval", None)  # None = fixed leader
        self.leader_id = 0
        self.election_count = 0  # Counts rotations for round robin
        self.last_rotation_t = -1
        
        # Hallucination injection (read from top-level for baseline, reip section for consistency)
        self.hallucination_rate = cfg.get("hallucination_rate", self.reip_cfg.get("hallucination_rate", 0.0))
        self.hallucination_active = False
        self.hallucination_duration = 0
        self.hallucination_type = None
        self.hallucination_profile = cfg.get("hallucination_profile", self.reip_cfg.get("hallucination_profile", None))
        self.hallucination_pingpong_duration = cfg.get("hallucination_pingpong_duration", self.reip_cfg.get("hallucination_pingpong_duration", 30))
        self.hallucination_schedule = cfg.get("hallucination_schedule", self.reip_cfg.get("hallucination_schedule", None))
        self.dense_explored_strict = bool(cfg.get("dense_explored_strict", self.reip_cfg.get("dense_explored_strict", False)))
        
        # Cyber attack injection (NEW - same as Enhanced REIP)
        self.cyber_attack_rate = cfg.get("cyber_attack_rate", self.reip_cfg.get("cyber_attack_rate", 0.0))
        self.attack_active = False
        self.attack_duration = 0
        self.attack_targets = []
        
        # Communication faults (read from top-level for baseline)
        self.command_loss_rate = cfg.get("command_loss_rate", self.reip_cfg.get("command_loss_rate", 0.0))
        self.command_radius = cfg.get("command_radius", self.reip_cfg.get("command_radius", 100))
        
        # Exploration parameters
        self.base_prox = cfg.get("prox_beta", 0.6)
        self.dispersion_alpha = self.reip_cfg.get("dispersion_alpha", 1.0)
        
        # Metrics (for comparison with REIP)
        self.command_loss_events = 0
        self.hallucination_history = []
        self.coverage_history = []
        
    def inject_hallucination(self, t):
        """Use centralized helper for deterministic, side-effect-safe state updates."""
        update_hallucination_state(
            self,
            self.hallucination_rate,
            t,
            fixed_type=self.hallucination_profile,
            schedule=self.hallucination_schedule,
        )
    
    def corrupt_assignments(self, env, assigns, frontiers):
        """Pure corruption via centralized helper (no side-effects)."""
        return corrupt_assignments_by_hallucination(env, assigns, frontiers, self)
    
    def corrupt_assignments_cyber(self, env, assigns, frontiers, t):
        """
        Cyber attack using the global attack scheduler.
        This ensures both Enhanced REIP and Baseline face identical attack patterns.
        
        PERSISTENT ATTACKS: Once triggered, attacks persist for multiple timesteps
        to make the damage more obvious and measurable.
        """
        # Import here to avoid circular imports
        from src.utils.cyber_attack_scheduler import should_attack_now
        
        # Check if we should start a new attack
        if should_attack_now(t) and not self.attack_active:
            # Start new attack with persistence
            self.attack_active = True
            self.attack_duration = random.randint(8, 15)  # Attack persists 8-15 timesteps
            print(f"\n[CYBER ATTACK] Baseline leader {self.leader_id} COMPROMISED at timestep {t}!")
            print(f"   Attack will persist for {self.attack_duration} timesteps")
        
        # Decrement attack duration
        if self.attack_active:
            self.attack_duration -= 1
            if self.attack_duration <= 0:
                self.attack_active = False
                self.attack_targets = []
                print(f"[CYBER ATTACK] Attack ended at timestep {t}")
                return assigns
        
        # If no active attack, return normal assignments
        if not self.attack_active or not assigns:
            return assigns
        
        corrupted = assigns.copy()
        agent_ids = list(corrupted.keys())
        
        # Find explored areas to send followers to
        explored_positions = []
        for x in range(env.size):
            for y in range(env.size):
                # Encoding: team_belief[x, y] uses (x, y) ordering consistently
                if env.team_belief[x, y] == 0:  # Explored (known empty)
                    explored_positions.append((x, y))
        
        if len(explored_positions) < 10:  # Need sufficient explored area
            return assigns
        
        # Persist attack targets or select new ones
        if not self.attack_targets:
            self.attack_targets = random.sample(agent_ids, min(4, len(agent_ids)))  # Attack up to 4 agents
        
        # Redirect compromised agents to already-explored areas (WASTED EFFORT)
        for follower_id in self.attack_targets:
            if follower_id in corrupted:
                target_pos = random.choice(explored_positions)
                corrupted[follower_id] = target_pos
        
        if t % 5 == 0:  # Print status every 5 timesteps during attack
            print(f"[ATTACK ACTIVE] t={t}: {len(self.attack_targets)} agents redirected to explored areas (wasted effort)")
        
        return corrupted
    
    def step(self, env, frontiers, state):
        """
        Fixed leader step with NO governance.
        
        Key difference from REIP: Leader never changes regardless of performance.
        """
        t = state.get("t", 0)
        T = max(1, self.full_cfg.get("T", 200))
        
        # Round robin leader rotation (if enabled)
        if self.round_robin_interval is not None and t > 0:
            if (t - self.last_rotation_t) >= self.round_robin_interval:
                # Rotate to next agent
                self.leader_id = (self.leader_id + 1) % self.N
                self.election_count += 1
                self.last_rotation_t = t
        
        # Inject hallucination
        self.inject_hallucination(t)
        
        # Record metrics
        if self.hallucination_active:
            self.hallucination_history.append({
                't': t,
                'type': self.hallucination_type
            })
        
        # Initialize state
        state.setdefault('leader_goals', {})
        state.setdefault('reserved', {})
        
        # Scale proximity penalty
        prox_beta = self.base_prox * (1.0 + self.dispersion_alpha * (t / float(T)))
        
        # Assignment parameters
        lam = self.full_cfg.get("lam", 0.0)
        window = self.reip_cfg.get("window", 2)
        persist_bonus = self.reip_cfg.get("persist_bonus", 0.2)
        switch_margin = self.reip_cfg.get("switch_margin", 0.05)
        prox_radius = self.reip_cfg.get("prox_radius", 6)
        prev_assigns = state.get('prev', {})
        
        # Assign frontiers
        if frontiers:
            assigns, claims, U_pred = assign_frontiers_entropy_global(
                env, frontiers, lam=lam,
                prev_assigns=prev_assigns,
                window=window,
                prox_beta=prox_beta,
                prox_radius=prox_radius,
                persist_bonus=persist_bonus,
                switch_margin=switch_margin
            )
        else:
            assigns = {i: env.agent_positions[i] for i in env.agent_positions.keys()}
        
        # Corrupt assignments if hallucinating OR under cyber attack
        assigns = self.corrupt_assignments(env, assigns, frontiers)
        assigns = self.corrupt_assignments_cyber(env, assigns, frontiers, t)
        
        # Apply command loss
        assigns = filter_assignments_by_comm(
            env, assigns,
            comm_radius=self.command_radius,
            p_cmd_loss=self.command_loss_rate
        )
        
        # Execute movement
        env.step(assigns, t=t)
        
        # Sync state
        synced = {}
        for i in env.agent_positions.keys():
            a = env.agents.get(i)
            held = getattr(a, 'hold_target', None)
            synced[i] = held if held is not None else assigns.get(i, env.agent_positions[i])
        state['prev'] = synced
        
        # Update state (leader rotates if round robin enabled)
        state['leader'] = self.leader_id
        state['election_count'] = self.election_count
        state['hallucinating'] = self.hallucination_active
        
        # Track coverage
        self.coverage_history.append(env.coverage())

    def get_metrics(self):
        """Return metrics for comparison with REIP."""
        return {
            'election_count': self.election_count,  # Rotations for round robin, 0 for fixed
            'final_leader': self.leader_id,
            'leader_failures': {i: 0 for i in range(self.N)},
            'hallucination_history': self.hallucination_history,
            'coverage_history': self.coverage_history,
            'command_loss_events': self.command_loss_events,
            'final_avg_trust': 1.0  # Not applicable for baseline
        }
