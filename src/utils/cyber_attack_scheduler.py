"""
Cyber Attack Scheduler - Provides deterministic attack timing for fair comparisons

References:
- Lamport et al. (1982): The Byzantine Generals Problem
- Castro & Liskov (1999): Practical Byzantine Fault Tolerance
- Dolev (1982): The Byzantine Generals strike again (timing attacks)
"""
import random

class CyberAttackScheduler:
    """
    Generates a deterministic schedule of cyber attack timesteps.
    Both Enhanced REIP and Baseline systems will use the same attack schedule.
    """
    
    def __init__(self, seed=42, attack_rate=0.08, total_timesteps=400):
        """
        Create a deterministic cyber attack schedule.
        
        Args:
            seed: Random seed for reproducible attack timing
            attack_rate: Probability of attack per timestep (0.08 = 8%)
            total_timesteps: Total simulation length
        """
        self.seed = seed
        self.attack_rate = attack_rate
        self.total_timesteps = total_timesteps
        
        # Generate the attack schedule
        self.attack_timesteps = self._generate_attack_schedule()
    
    def _generate_attack_schedule(self):
        """Generate a list of timesteps when attacks should occur."""
        # Use a separate random instance to avoid interfering with simulation randomness
        attack_rng = random.Random(self.seed)
        
        attack_timesteps = []
        for timestep in range(self.total_timesteps):
            if attack_rng.random() < self.attack_rate:
                attack_timesteps.append(timestep)
        
        return attack_timesteps
    
    def should_attack_at_timestep(self, timestep):
        """Check if an attack should occur at the given timestep."""
        return timestep in self.attack_timesteps
    
    def get_total_attacks(self):
        """Get the total number of planned attacks."""
        return len(self.attack_timesteps)
    
    def get_attack_schedule(self):
        """Get the complete list of attack timesteps."""
        return self.attack_timesteps.copy()

# Global scheduler instance - will be initialized by the simulation
_global_scheduler = None

def initialize_scheduler(seed=42, attack_rate=0.08, total_timesteps=400):
    """Initialize the global attack scheduler."""
    global _global_scheduler
    _global_scheduler = CyberAttackScheduler(seed, attack_rate, total_timesteps)
    print(f"[CYBER SCHEDULER] Initialized with {_global_scheduler.get_total_attacks()} planned attacks")
    print(f"[CYBER SCHEDULER] Attack timesteps: {_global_scheduler.get_attack_schedule()[:10]}{'...' if len(_global_scheduler.get_attack_schedule()) > 10 else ''}")

def should_attack_now(timestep):
    """Check if an attack should occur at the current timestep."""
    if _global_scheduler is None:
        # No scheduler initialized - no attacks
        return False
    return _global_scheduler.should_attack_at_timestep(timestep)

def get_scheduler():
    """Get the global scheduler instance."""
    return _global_scheduler