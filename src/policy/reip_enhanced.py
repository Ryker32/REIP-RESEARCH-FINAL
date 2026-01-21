"""
Enhanced REIP Implementation with Loop Detection and Impeachment Hash Mechanism

Key Features:
1. Loop-of-death detection: Detects when leader sends agents in circles through explored areas
2. Aimless movement detection: Identifies when agents are moving randomly without purpose
3. Impeachment hash system: Followers send impeachment signals when detecting bad leadership
4. Distributed voting: Each agent votes for highest trust candidate from their POV
5. Leader demotion: Impeached leaders become followers with restricted privileges

References:
- Lamport et al. (1982): The Byzantine Generals Problem
- Parker (1998): ALLIANCE: An architecture for fault tolerant multi-robot cooperation
- Ren et al. (2005): Information consensus in multivehicle cooperative control
- Burgard et al. (2000): Collaborative multi-robot exploration
"""

import math
import random
import numpy as np
from collections import deque, defaultdict
import hashlib
from typing import Dict, List, Tuple, Set, Optional

from src.policy.frontier import assign_frontiers_entropy_global
from src.comms.channel import filter_assignments_by_comm

class REIPEnhancedPolicy:
    """
    Enhanced REIP (Trust-based Election/Impeachment Protocol) with:
    - Loop-of-death detection
    - Aimless movement pattern recognition  
    - Impeachment hash voting system
    - Leader privilege restriction
    """
    
    def __init__(self, cfg):
        # Store configuration
        self.full_cfg = cfg  # Fixed: store as full_cfg like other policies
        
        # === Core REIP Parameters ===
        self.reip_cfg = cfg.get("reip", {})
        self.N = cfg.get("N", 8)
        self.leader_id = 0  # Start with agent 0 as leader
        self.election_count = 0
        
        # === Trust System ===
        # Trust matrix: trust[i][j] = how much agent i trusts agent j
        self.trust = [[1.0 for _ in range(self.N)] for _ in range(self.N)]
        self.trust_decay_rate = self.reip_cfg.get("trust_decay_rate", 0.5)
        self.trust_threshold = self.reip_cfg.get("trust_threshold", 0.6)
        self.min_trust = self.reip_cfg.get("min_trust", 0.1)
        self.trust_recovery_rate = self.reip_cfg.get("trust_recovery_rate", 0.02)
        
        # === Loop Detection System ===
        self.position_history = defaultdict(lambda: deque(maxlen=20))  # Track last 20 positions per agent
        self.assignment_history = defaultdict(lambda: deque(maxlen=15))  # Track last 15 assignments per agent
        self.loop_detection_window = 60  # Check for loops in last 10 steps
        self.loop_threshold = 0.7  # 70% of positions must be revisited to trigger loop detection
        
        # === Aimless Movement Detection ===
        self.coverage_progress = deque(maxlen=10)  # Track coverage progress over time
        self.stagnation_threshold = 0.01  # Less than 1% coverage gain indicates stagnation
        self.aimless_detection_window = 8  # Check for aimlessness over 8 steps
        
        # === Impeachment Hash System ===
        self.impeachment_votes = defaultdict(set)  # agent_id -> set of voter_ids
        self.impeachment_threshold = max(2, self.N // 2)  # Majority needed for impeachment
        self.impeachment_cooldown = 15  # Prevent spam impeachment attempts
        self.last_impeachment_attempt = -999
        
        # Generate stable cryptographic IDs for each agent (persistent throughout simulation)
        self.agent_ids = {}  # agent_index -> permanent_hash_id
        for agent_idx in range(self.N):
            # Use agent index + a simulation seed to create stable ID
            seed_data = f"REIP_AGENT_{agent_idx}_SIM_{id(self)}"
            self.agent_ids[agent_idx] = hashlib.sha256(seed_data.encode()).hexdigest()[:16]
        
        # === Leader Privilege System ===
        self.demoted_leaders = set()  # Agents that have been impeached and demoted
        self.demotion_duration = 50   # How long agents stay demoted (in timesteps)
        self.demotion_timers = {}     # When each agent was demoted
        
        # === Hallucination/Fault Injection ===
        self.hallucination_rate = self.reip_cfg.get("hallucination_rate", 0.0)
        self.hallucination_active = False
        self.hallucination_type = None
        self.hallucination_duration = 0
        
        # === Cyber Attack Injection ===
        self.cyber_attack_rate = self.reip_cfg.get("cyber_attack_rate", 0.0)
        
        # === Logging and Metrics ===
        self.leader_history = []
        self.impeachment_history = []
        self.loop_detections = []
        self.aimless_detections = []
        self.prev_assigns = {}  # Track previous assignments for persistence
        
    def detect_loops_of_death(self, env, t: int) -> List[int]:
        """
        Detect if agents are being sent in loops through already explored areas.
        
        Returns list of agent IDs that are in loops of death.
        """
        loops_detected = []
        
        for agent_id in range(self.N):
            if agent_id == self.leader_id:
                continue  # Don't check leader's own movement
                
            # Get recent position history
            positions = list(self.position_history[agent_id])
            if len(positions) < self.loop_detection_window:
                continue
                
            recent_positions = positions[-self.loop_detection_window:]
            
            # Check for spatial loops (visiting same locations repeatedly)
            position_counts = defaultdict(int)
            for pos in recent_positions:
                position_counts[pos] += 1
            
            # Calculate revisit ratio
            total_moves = len(recent_positions)
            revisited_moves = sum(count - 1 for count in position_counts.values() if count > 1)
            revisit_ratio = revisited_moves / total_moves if total_moves > 0 else 0
            
            # Check if agent is in explored areas
            agent = env.agents.get(agent_id)
            if agent and revisit_ratio > self.loop_threshold:
                # Verify positions are in already-explored areas
                explored_positions = 0
                for pos in recent_positions:
                    x, y = pos
                    if 0 <= x < env.size and 0 <= y < env.size:
                        # Check if this area was already explored
                        if hasattr(env, 'explored') and env.explored[x, y]:
                            explored_positions += 1
                        elif hasattr(agent, 'local'):
                            # Use agent's local map as proxy for explored areas
                            local_x = x - agent.pos[0] + agent.r_local
                            local_y = y - agent.pos[1] + agent.r_local
                            if (0 <= local_x < agent.local.shape[0] and 
                                0 <= local_y < agent.local.shape[1] and
                                agent.local[local_x, local_y] == 0):  # Known free space
                                explored_positions += 1
                
                explored_ratio = explored_positions / total_moves if total_moves > 0 else 0
                
                if explored_ratio > 0.6:  # 60% of loop is in explored areas
                    loops_detected.append(agent_id)
                    self.loop_detections.append({
                        'timestep': t,
                        'agent_id': agent_id,
                        'leader_id': self.leader_id,
                        'revisit_ratio': revisit_ratio,
                        'explored_ratio': explored_ratio
                    })
        
        return loops_detected
    
    def detect_aimless_movement(self, env, t: int) -> bool:
        """
        Detect if the team is moving aimlessly without making exploration progress.
        
        Returns True if aimless movement is detected.
        """
        # Track coverage progress
        current_coverage = env.coverage()
        self.coverage_progress.append(current_coverage)
        
        if len(self.coverage_progress) < self.aimless_detection_window:
            return False
        
        # Check if coverage has stagnated
        coverage_gain = self.coverage_progress[-1] - self.coverage_progress[0]
        time_window = min(len(self.coverage_progress), self.aimless_detection_window)
        
        if coverage_gain < self.stagnation_threshold * time_window:
            # Check if agents are still moving (not just stuck)
            total_movement = 0
            for agent_id in range(self.N):
                positions = list(self.position_history[agent_id])
                if len(positions) >= 2:
                    # Calculate movement distance
                    for i in range(1, min(len(positions), self.aimless_detection_window)):
                        dx = positions[-i][0] - positions[-i-1][0]
                        dy = positions[-i][1] - positions[-i-1][1]
                        total_movement += math.sqrt(dx*dx + dy*dy)
            
            # If agents are moving but not gaining coverage, it's aimless
            if total_movement > self.N * 2:  # Threshold: more than 2 cells movement per agent
                self.aimless_detections.append({
                    'timestep': t,
                    'leader_id': self.leader_id,
                    'coverage_gain': coverage_gain,
                    'total_movement': total_movement
                })
                return True
        
        return False
    
    def generate_impeachment_hash(self, agent_id: int, target_leader: int, t: int) -> str:
        """
        Generate impeachment hash for secure voting.
        
        Uses the agent's STABLE cryptographic ID (not random) combined with
        vote context (target, timestep) to create verifiable vote signature.
        
        This allows vote tracking while preventing manipulation:
        - Same agent voting for same leader at same time = same hash
        - Different agents produce different hashes (via stable IDs)
        - Hash can be verified by any agent in the system
        """
        # Use the persistent agent ID (generated at init) instead of random nonce
        stable_id = self.agent_ids[agent_id]
        data = f"{stable_id}:{target_leader}:{t}"
        return hashlib.sha256(data.encode()).hexdigest()[:16]  # Use first 16 chars
    
    def cast_impeachment_vote(self, voter_id: int, target_leader: int, t: int) -> str:
        """
        Cast an impeachment vote against the current leader.
        
        Returns the impeachment hash for this vote.
        """
        impeachment_hash = self.generate_impeachment_hash(voter_id, target_leader, t)
        self.impeachment_votes[target_leader].add(voter_id)
        
        print(f"Agent {voter_id} (ID: {self.agent_ids[voter_id]}) casts impeachment vote against Leader {target_leader}")
        print(f"Impeachment hash: {impeachment_hash}")
        
        return impeachment_hash
    
    def verify_impeachment_vote(self, voter_id: int, target_leader: int, t: int, claimed_hash: str) -> bool:
        """
        Verify that an impeachment vote hash is valid.
        
        Any agent can verify a vote by recalculating the hash and comparing.
        This prevents vote forgery and ensures transparency.
        """
        expected_hash = self.generate_impeachment_hash(voter_id, target_leader, t)
        is_valid = (expected_hash == claimed_hash)
        
        if not is_valid:
            print(f"WARNING: Invalid vote hash from Agent {voter_id}!")
            print(f"  Expected: {expected_hash}")
            print(f"  Received: {claimed_hash}")
        
        return is_valid
    
    def check_impeachment_threshold(self, t: int) -> bool:
        """
        Check if enough impeachment votes have been cast to trigger impeachment.
        """
        current_votes = len(self.impeachment_votes[self.leader_id])
        
        if current_votes >= self.impeachment_threshold:
            # Prevent spam impeachment attempts
            if t - self.last_impeachment_attempt >= self.impeachment_cooldown:
                self.last_impeachment_attempt = t
                return True
        
        return False
    
    def distributed_leader_election(self, t: int) -> int:
        """
        Conduct distributed election where each agent votes for highest trust candidate.
        
        Each agent votes based on their own POV (personal trust values).
        The candidate with most votes becomes new leader.
        """
        votes = defaultdict(int)
        
        # Each agent (except demoted ones) casts a vote
        for voter_id in range(self.N):
            if voter_id in self.demoted_leaders:
                continue  # Demoted agents cannot vote
            
            # Find candidate with highest trust from this voter's POV
            best_candidate = None
            best_trust = -1
            
            for candidate_id in range(self.N):
                if candidate_id in self.demoted_leaders:
                    continue  # Demoted agents cannot be leaders
                
                candidate_trust = self.trust[voter_id][candidate_id]
                if candidate_trust > best_trust:
                    best_trust = candidate_trust
                    best_candidate = candidate_id
            
            if best_candidate is not None:
                votes[best_candidate] += 1
        
        # Select candidate with most votes
        if votes:
            new_leader = max(votes, key=votes.get)
            
            # Log election results
            self.leader_history.append({
                'timestep': t,
                'old_leader': self.leader_id,
                'new_leader': new_leader,
                'votes': dict(votes),
                'election_type': 'distributed_voting'
            })
            
            return new_leader
        
        # Fallback: keep current leader if no valid votes
        return self.leader_id
    
    def demote_leader(self, leader_id: int, t: int):
        """
        Demote impeached leader to follower status with restricted privileges.
        """
        self.demoted_leaders.add(leader_id)
        self.demotion_timers[leader_id] = t
        
        print(f"Leader {leader_id} has been IMPEACHED and demoted to follower status!")
        print(f"Restricted privileges: Cannot lead, cannot vote for {self.demotion_duration} timesteps")
        
        self.impeachment_history.append({
            'timestep': t,
            'impeached_leader': leader_id,
            'reason': 'impeachment_vote',
            'vote_count': len(self.impeachment_votes[leader_id])
        })
    
    def restore_demoted_agents(self, t: int):
        """
        Restore agents from demotion after timeout period.
        """
        to_restore = []
        for agent_id, demotion_time in self.demotion_timers.items():
            if t - demotion_time >= self.demotion_duration:
                to_restore.append(agent_id)
        
        for agent_id in to_restore:
            self.demoted_leaders.discard(agent_id)
            del self.demotion_timers[agent_id]
            print(f"Agent {agent_id} restored from demotion - can now participate in leadership")
    
    def inject_cyber_attack(self, env, t: int):
        """
        Simulate a cyber attack using the global attack scheduler.
        This ensures both Enhanced REIP and Baseline face identical attack patterns.
        """
        # Import here to avoid circular imports
        from src.utils.cyber_attack_scheduler import should_attack_now
        
        # Check if this timestep should have an attack (using global scheduler)
        if not should_attack_now(t):
            return
        
        print(f"[CYBER ATTACK] Enhanced REIP attack triggered at timestep {t}!")
        
        # Only attack if we have a leader
        if self.leader_id is None:
            return
        
        # Get list of followers (all agents except leader and demoted agents)
        followers = [i for i in range(self.N) if i != self.leader_id and i not in self.demoted_leaders]
        if len(followers) == 0:
            return
        
        # Select target followers for the attack
        attack_targets = random.sample(followers, min(3, len(followers)))
        
        # Find explored areas to send followers to
        explored_positions = []
        for y in range(env.size):
            for x in range(env.size):
                if env.team_belief[y, x] == 0:  # Explored (known empty)
                    explored_positions.append((x, y))
        
        if len(explored_positions) < 5:  # Need sufficient explored area
            return
        
        # Create malicious assignments to explored areas
        for follower_id in attack_targets:
            agent = env.agents.get(follower_id)
            if agent:
                # Choose a random explored position far from current position
                current_pos = agent.pos
                far_explored = [pos for pos in explored_positions 
                              if abs(pos[0] - current_pos[0]) + abs(pos[1] - current_pos[1]) > 10]
                
                if far_explored:
                    target_pos = random.choice(far_explored)
                    
                    # Store the malicious assignment for this agent
                    if not hasattr(self, 'assignments'):
                        self.assignments = {}
                    self.assignments[follower_id] = target_pos
                    
                    print(f"[CYBER ATTACK] Leader {self.leader_id} compromised - "
                          f"sending follower {follower_id} to explored area {target_pos}")
                    
                    # Log the attack for metrics
                    if hasattr(self, 'metrics'):
                        self.metrics.append({
                            'timestep': t,
                            'event': 'cyber_attack',
                            'attacker': 'external',
                            'victim_leader': self.leader_id,
                            'victim_follower': follower_id,
                            'target_position': target_pos,
                            'is_explored': True
                        })

    def inject_hallucination_enhanced(self, t: int):
        """
        Enhanced hallucination injection that creates loops of death and aimless movement.
        """
        if random.random() < self.hallucination_rate:
            if not self.hallucination_active:
                self.hallucination_active = True
                self.hallucination_duration = random.randint(10, 25)  # Longer duration for loops
                # Focus on loop-inducing behaviors
                self.hallucination_type = random.choice(['loop_of_death', 'aimless_hacked', 'cluster_trap'])
        
        if self.hallucination_active:
            self.hallucination_duration -= 1
            if self.hallucination_duration <= 0:
                self.hallucination_active = False
                self.hallucination_type = None
    
    def corrupt_assignments_enhanced(self, env, assigns: Dict[int, Tuple[int, int]], frontiers: List) -> Dict[int, Tuple[int, int]]:
        """
        Enhanced assignment corruption that creates specific problematic patterns.
        """
        if not self.hallucination_active or not assigns:
            return assigns
        
        corrupted = assigns.copy()
        agent_ids = list(corrupted.keys())
        
        if self.hallucination_type == 'loop_of_death':
            # Create circular assignment pattern in explored areas
            if len(agent_ids) >= 3:
                # Create a triangle/circle pattern
                center_x, center_y = env.size // 2, env.size // 2
                radius = 5
                for i, agent_id in enumerate(agent_ids[:6]):  # Up to 6 agents in circle
                    angle = (i / 6) * 2 * math.pi
                    x = int(center_x + radius * math.cos(angle))
                    y = int(center_y + radius * math.sin(angle))
                    x = max(0, min(env.size - 1, x))
                    y = max(0, min(env.size - 1, y))
                    corrupted[agent_id] = (x, y)
        
        elif self.hallucination_type == 'aimless_hacked':
            # Send agents to random already-explored locations
            for agent_id in agent_ids:
                if random.random() < 0.8:  # 80% of agents affected
                    # Pick random position from their known explored areas
                    agent = env.agents.get(agent_id)
                    if agent and hasattr(agent, 'local'):
                        explored_cells = []
                        for i in range(agent.local.shape[0]):
                            for j in range(agent.local.shape[1]):
                                if agent.local[i, j] == 0:  # Known free space
                                    world_x = i - agent.r_local + agent.pos[0]
                                    world_y = j - agent.r_local + agent.pos[1]
                                    if 0 <= world_x < env.size and 0 <= world_y < env.size:
                                        explored_cells.append((world_x, world_y))
                        
                        if explored_cells:
                            corrupted[agent_id] = random.choice(explored_cells)
        
        elif self.hallucination_type == 'cluster_trap':
            # Force all agents to cluster in one small explored region
            if agent_ids:
                # Pick a previously explored area as trap
                trap_x = random.randint(env.size // 4, 3 * env.size // 4)
                trap_y = random.randint(env.size // 4, 3 * env.size // 4)
                
                for agent_id in agent_ids:
                    # Add small random offset to create cluster
                    offset_x = random.randint(-2, 2)
                    offset_y = random.randint(-2, 2)
                    x = max(0, min(env.size - 1, trap_x + offset_x))
                    y = max(0, min(env.size - 1, trap_y + offset_y))
                    corrupted[agent_id] = (x, y)
        
        return corrupted
    
    def assign_frontiers(self, env, frontiers):
        """
        Assign frontiers to agents using entropy-based global assignment.
        """
        if not frontiers:
            return {}
        
        # Use the same frontier assignment logic as REIP true
        assignments, claims, U_pred = assign_frontiers_entropy_global(
            env, frontiers, 
            lam=self.full_cfg.get("lam", 3.0),
            prev_assigns=getattr(self, 'prev_assigns', {}),
            window=self.full_cfg.get("reip", {}).get("window", 2),
            prox_beta=self.full_cfg.get("prox_beta", 0.6),
            prox_radius=self.full_cfg.get("reip", {}).get("prox_radius", 6),
            persist_bonus=self.full_cfg.get("reip", {}).get("persist_bonus", 0.6),
            switch_margin=self.full_cfg.get("reip", {}).get("switch_margin", 0.15)
        )
        
        # Apply communication constraints
        assignments = filter_assignments_by_comm(
            env, assignments, 
            comm_radius=self.full_cfg.get("reip", {}).get("command_radius", 100),
            p_cmd_loss=self.full_cfg.get("reip", {}).get("command_loss_rate", 0.0)
        )
        
        # Update previous assignments for persistence tracking
        self.prev_assigns = assignments.copy()
        
        return assignments
    
    def update_position_tracking(self, env, t: int):
        """
        Update position history for all agents for loop detection.
        """
        for agent_id in range(self.N):
            agent = env.agents.get(agent_id)
            if agent:
                self.position_history[agent_id].append(agent.pos)
    
    def step(self, env, frontiers, state):
        """
        Enhanced REIP step with loop detection and impeachment hash system.
        """
        t = state.get("t", 0)
        
        # === 1. Update tracking systems ===
        self.update_position_tracking(env, t)
        self.restore_demoted_agents(t)
        
        # === 2. Inject hallucinations and cyber attacks ===
        if self.leader_id not in self.demoted_leaders:  # Only if leader is not demoted
            self.inject_hallucination_enhanced(t)
            self.inject_cyber_attack(env, t)
        
        # === 3. Detect problematic behavior ===
        loops_detected = self.detect_loops_of_death(env, t)
        aimless_detected = self.detect_aimless_movement(env, t)
        
        # === 4. Cast impeachment votes if problems detected ===
        for agent_id in range(self.N):
            if agent_id == self.leader_id or agent_id in self.demoted_leaders:
                continue  # Leader doesn't vote against themselves, demoted can't vote
            
            should_vote = False
            
            # Vote if this agent is in a loop of death
            if agent_id in loops_detected:
                should_vote = True
                print(f"Agent {agent_id} detected loop of death - casting impeachment vote")
            
            # Vote if aimless movement detected
            if aimless_detected and random.random() < 0.7:  # 70% chance followers notice
                should_vote = True
                print(f"Agent {agent_id} detected aimless movement - casting impeachment vote")
            
            if should_vote:
                self.cast_impeachment_vote(agent_id, self.leader_id, t)
        
        # === 5. Check for impeachment ===
        if self.check_impeachment_threshold(t):
            print(f"\n[IMPEACHMENT] TRIGGERED at timestep {t}!")
            print(f"Leader {self.leader_id} has been impeached by majority vote")
            
            # Demote current leader
            old_leader = self.leader_id
            self.demote_leader(self.leader_id, t)
            
            # Conduct distributed election
            self.leader_id = self.distributed_leader_election(t)
            
            # Clear impeachment votes for new round
            self.impeachment_votes.clear()
            
            print(f"New leader elected: Agent {self.leader_id}")
            print(f"Demoted agent {old_leader} is now follower-only\\n")
        
        # === 6. Leader assignment (if not demoted) ===
        if self.leader_id in self.demoted_leaders:
            # Emergency: elect new leader immediately if current leader is demoted
            self.leader_id = self.distributed_leader_election(t)
        
        # === 7. Generate assignments ===
        if frontiers:
            assigns = self.assign_frontiers(env, frontiers)
            
            # Apply corruption if leader is hallucinating
            if self.hallucination_active:
                assigns = self.corrupt_assignments_enhanced(env, assigns, frontiers)
                
            return assigns
        
        return {}