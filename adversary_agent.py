"""
Adaptive adversary for REIP cyberattack experiments.

Threat model: attacker monitors public peer_state broadcasts to identify
the current leader, then re-compromises after each recovery. Uses only
bad_leader fault (no new fault types required).

Budget-limited (default 4 attacks across 120s trial) with fixed cooldown
between strikes. No trust-score observation — just public leader identity.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Any


@dataclass
class AdversaryAction:
    target_robot: int
    fault_type: str
    label: str


class AdaptiveAdversary:
    """Budget-limited adaptive attacker.

    Observes only: elapsed time, current leader ID, whether leader changed
    since last attack. Does NOT observe trust scores.

    Behavior:
    1. Wait holdoff_s for election to settle.
    2. Attack current leader.
    3. After impeachment (leader changed), wait cooldown_s for new leader
       to stabilize, then attack again.
    4. Stop after budget exhausted.
    """

    def __init__(self, fault_type="bad_leader", budget=4,
                 holdoff_s=10.0, cooldown_s=12.0):
        self.fault_type = fault_type
        self.budget = budget
        self.holdoff_s = holdoff_s
        self.cooldown_s = cooldown_s
        self.compromises_used = 0
        self.attack_log: List[Dict[str, Any]] = []
        self._last_attack_time: Optional[float] = None
        self._last_target: Optional[int] = None

    def decide(self, elapsed: float, current_leader: int,
               leader_changed_since_last: bool) -> Optional[AdversaryAction]:
        if self.compromises_used >= self.budget:
            return None
        if elapsed < self.holdoff_s:
            return None
        # Our target was deposed — reset so we can re-attack new leader
        if self._last_target is not None and leader_changed_since_last:
            self._last_target = None
        # Cooldown
        if self._last_attack_time is not None:
            if (elapsed - self._last_attack_time) < self.cooldown_s:
                return None
        # Don't re-attack leader we already compromised (fault already active)
        if self._last_target == current_leader:
            return None
        # Attack
        self.compromises_used += 1
        self._last_attack_time = elapsed
        self._last_target = current_leader
        label = f"Adaptive strike {self.compromises_used}/{self.budget}"
        self.attack_log.append({
            "index": self.compromises_used,
            "elapsed": round(elapsed, 2),
            "target": current_leader,
            "label": label,
        })
        return AdversaryAction(
            target_robot=current_leader,
            fault_type=self.fault_type,
            label=label,
        )

    def metadata(self) -> Dict[str, Any]:
        return {
            "agent_type": "AdaptiveAdversary",
            "fault_type": self.fault_type,
            "budget": self.budget,
            "total_compromises": self.compromises_used,
            "attack_log": self.attack_log,
        }