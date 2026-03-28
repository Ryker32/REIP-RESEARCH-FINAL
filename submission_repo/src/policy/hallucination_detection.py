"""
Principled Hallucination Detection for REIP Governance

Implements CUSUM (Cumulative Sum) sequential change detection to quantitatively
identify when leader behavior deviates from expected patterns, triggering
impeachment based on statistical evidence rather than heuristics.

References:
- Page, E. S. (1954). Continuous inspection schemes. Biometrika, 41(1/2), 100-115.
  Original CUSUM formulation for sequential hypothesis testing
- Basseville, M., & Nikiforov, I. V. (1993). Detection of abrupt changes: theory
  and application. Prentice Hall. (CUSUM theory and applications)
- Gustafsson, F. (2000). Adaptive filtering and change detection. Wiley.
  (Adaptive thresholds and drift parameters)
- Chang, K.-C., & Chen, C.-Y. (2011). On-line adaptive fuzzy neural network
  for anomaly detection. Expert Systems with Applications, 38(4), 4514-4520.
  (Applications in robotic fault detection)

This module provides:
1. CUSUM-based change detection for trust violations
2. Quantifiable hallucination metrics (information gain violations, loop scores)
3. Adaptive thresholds based on system performance
"""

import math
import numpy as np
from typing import Dict, List, Tuple, Optional
from collections import deque


class HallucinationDetector:
    """
    CUSUM-based sequential change detector for leader hallucinations.
    
    Implements Page's (1954) CUSUM algorithm to detect when a leader's
    predicted vs observed performance systematically diverges, indicating
    hallucination or Byzantine behavior.
    """
    
    def __init__(self, 
                 kappa: float = 0.1,
                 threshold: float = 5.0,
                 min_samples: int = 3,
                 window_size: int = 10):
        """
        Initialize CUSUM detector.
        
        Args:
            kappa: Drift parameter - magnitude of change to detect (Page 1954).
                   Smaller kappa = more sensitive to small deviations.
            threshold: Detection threshold h - triggers alarm when CUSUM > h.
                       Higher = fewer false alarms but longer detection delay.
            min_samples: Minimum observations before detection activates.
            window_size: Moving window for computing baseline statistics.
        
        References:
            Page (1954) recommends kappa ~ 0.5sigma where sigma is noise std dev.
            Threshold h trades off false alarm rate vs detection delay.
        """
        self.kappa = float(kappa)
        self.threshold = float(threshold)
        self.min_samples = int(min_samples)
        self.window_size = int(window_size)
        
        # CUSUM state: per-leader tracking
        self.cusum_statistic: Dict[int, float] = {}  # leader_id -> current CUSUM value
        self.baseline_mean: Dict[int, deque] = {}    # leader_id -> sliding window of errors
        self.baseline_std: Dict[int, float] = {}      # leader_id -> std dev estimate
        
        # Detection history for telemetry
        self.detection_history: List[Dict] = []
        
        # Hallucination metrics (quantifiable)
        self.metrics: Dict[int, Dict] = {}  # leader_id -> metrics dict
    
    def update(self, 
               leader_id: int, 
               predicted_gain: float, 
               observed_gain: float,
               timestep: int) -> Tuple[bool, Dict]:
        """
        Update CUSUM statistic and check for hallucination detection.
        
        Args:
            leader_id: Current leader identifier
            predicted_gain: Predicted information gain (U_pred)
            observed_gain: Actual observed information gain (U_obs)
            timestep: Current simulation timestep
        
        Returns:
            (detected, metrics_dict): 
                - detected: True if CUSUM exceeds threshold
                - metrics_dict: Quantified metrics for logging/analysis
        
        Algorithm (Page 1954):
            1. Compute residual: r = predicted - observed
            2. Update baseline statistics (sliding window)
            3. Normalize: r' = (r - mu_baseline) / sigma_baseline
            4. Update CUSUM: S_t = max(0, S_{t-1} + r' - kappa)
            5. Alarm if S_t > threshold
        
        The kappa parameter acts as a "deadband" - small deviations
        (|r'| < kappa) don't accumulate, preventing false alarms from noise.
        """
        if leader_id not in self.cusum_statistic:
            self.cusum_statistic[leader_id] = 0.0
            self.baseline_mean[leader_id] = deque(maxlen=self.window_size)
            self.baseline_std[leader_id] = 1.0  # Initial estimate
            self.metrics[leader_id] = {
                'total_violations': 0,
                'information_gain_violations': 0,
                'cumulative_error': 0.0,
                'detection_count': 0
            }
        
        # Compute residual error
        residual = predicted_gain - observed_gain
        # Only consider positive errors (leader overpromised)
        positive_error = max(0.0, residual)
        
        # Update baseline statistics (adaptive mean/std)
        baseline_window = self.baseline_mean[leader_id]
        baseline_window.append(positive_error)
        
        # Compute baseline statistics
        if len(baseline_window) >= 2:
            mean_baseline = float(np.mean(list(baseline_window)))
            std_baseline = float(np.std(list(baseline_window)))
            # Avoid division by zero
            std_baseline = max(0.1, std_baseline)
        else:
            mean_baseline = positive_error
            std_baseline = 1.0
        
        self.baseline_std[leader_id] = std_baseline
        
        # Normalize residual (z-score)
        if std_baseline > 1e-6:
            normalized_residual = (positive_error - mean_baseline) / std_baseline
        else:
            normalized_residual = positive_error
        
        # Update CUSUM statistic (Page 1954)
        # S_t = max(0, S_{t-1} + (r' - kappa))
        prev_cusum = self.cusum_statistic[leader_id]
        # The kappa acts as a deadband: small normalized residuals don't accumulate
        self.cusum_statistic[leader_id] = max(0.0, prev_cusum + normalized_residual - self.kappa)
        
        # Check if threshold exceeded
        detected = (len(baseline_window) >= self.min_samples and 
                   self.cusum_statistic[leader_id] >= self.threshold)
        
        # Update metrics
        metrics = self.metrics[leader_id]
        if positive_error > 0.01:  # Significant violation threshold
            metrics['total_violations'] += 1
            metrics['information_gain_violations'] += 1
        metrics['cumulative_error'] += positive_error
        
        if detected:
            metrics['detection_count'] += 1
            # Reset CUSUM after detection to avoid double-counting
            self.cusum_statistic[leader_id] = 0.0
        
        # Build comprehensive metrics dict
        metrics_dict = {
            'cusum_value': self.cusum_statistic[leader_id],
            'residual': residual,
            'positive_error': positive_error,
            'normalized_residual': normalized_residual,
            'baseline_mean': mean_baseline,
            'baseline_std': std_baseline,
            'detected': detected,
            **metrics
        }
        
        # Record detection event
        if detected:
            self.detection_history.append({
                'timestep': timestep,
                'leader_id': leader_id,
                'cusum_value': self.cusum_statistic[leader_id],
                'predicted_gain': predicted_gain,
                'observed_gain': observed_gain,
                'residual': residual
            })
        
        return detected, metrics_dict
    
    def compute_loop_score(self, 
                          agent_positions: Dict[int, Tuple[int, int]],
                          agent_history: Dict[int, deque]) -> Dict[int, float]:
        """
        Compute loop formation scores for each agent.
        
        Higher score = more repetitive movement patterns (potential hallucination).
        
        References:
            - Thrun et al. (2005): Probabilistic Robotics - loop closure detection
            - Bender & Fenton (1970): Loop detection in robot navigation
        
        Returns:
            Dict mapping agent_id -> loop_score (0.0 to 1.0)
        """
        loop_scores = {}
        for agent_id, pos in agent_positions.items():
            history = agent_history.get(agent_id, deque(maxlen=12))
            if len(history) < 4:
                loop_scores[agent_id] = 0.0
                continue
            
            # Count unique positions vs total positions
            unique_positions = len(set(history))
            total_positions = len(history)
            
            # Loop score: low uniqueness = high looping
            if total_positions > 0:
                loop_score = 1.0 - (unique_positions / float(total_positions))
            else:
                loop_score = 0.0
            
            # Penalize if agent revisits same position multiple times
            position_counts = {}
            for p in history:
                position_counts[p] = position_counts.get(p, 0) + 1
            
            # Maximum revisit count indicates loop severity
            max_revisits = max(position_counts.values()) if position_counts else 1
            revisit_penalty = min(1.0, (max_revisits - 1) / 3.0)
            
            loop_scores[agent_id] = min(1.0, loop_score + 0.3 * revisit_penalty)
        
        return loop_scores
    
    def compute_coverage_stagnation(self,
                                   coverage_history: deque,
                                   window: int = 8,
                                   threshold: float = 0.01) -> bool:
        """
        Detect aimless movement: coverage doesn't improve over time.
        
        References:
            - Yamauchi (1997): Frontier-based exploration efficiency metrics
        
        Args:
            coverage_history: Recent coverage values (sliding window)
            window: Number of timesteps to examine
            threshold: Minimum coverage gain required (e.g., 1% per window)
        
        Returns:
            True if coverage stagnated (hallucination indicator)
        """
        if len(coverage_history) < window:
            return False
        
        recent = list(coverage_history)[-window:]
        coverage_gain = recent[-1] - recent[0]
        
        return coverage_gain < threshold
    
    def get_detection_summary(self) -> Dict:
        """Return summary statistics for all detected hallucinations."""
        if not self.detection_history:
            return {
                'total_detections': 0,
                'leaders_detected': set(),
                'avg_residual': 0.0,
                'max_cusum': 0.0
            }
        
        residuals = [d['residual'] for d in self.detection_history]
        cusum_values = [d['cusum_value'] for d in self.detection_history]
        leaders = set(d['leader_id'] for d in self.detection_history)
        
        return {
            'total_detections': len(self.detection_history),
            'leaders_detected': leaders,
            'avg_residual': float(np.mean(residuals)) if residuals else 0.0,
            'max_residual': float(np.max(residuals)) if residuals else 0.0,
            'avg_cusum': float(np.mean(cusum_values)) if cusum_values else 0.0,
            'max_cusum': float(np.max(cusum_values)) if cusum_values else 0.0
        }


class AdaptiveThresholdManager:
    """
    Adaptive threshold tuning for CUSUM detector based on system performance.
    
    Adjusts detection sensitivity to balance false alarm rate vs detection delay.
    
    References:
        - Gustafsson (2000): Adaptive filtering with performance-based tuning
        - Basseville & Nikiforov (1993): Adaptive thresholds for change detection
    """
    
    def __init__(self,
                 base_threshold: float = 5.0,
                 min_threshold: float = 2.0,
                 max_threshold: float = 10.0,
                 adaptation_rate: float = 0.1):
        """
        Initialize adaptive threshold manager.
        
        Args:
            base_threshold: Starting detection threshold
            min_threshold: Lower bound (more sensitive)
            max_threshold: Upper bound (less sensitive)
            adaptation_rate: How quickly thresholds adjust (0-1)
        """
        self.base_threshold = float(base_threshold)
        self.min_threshold = float(min_threshold)
        self.max_threshold = float(max_threshold)
        self.adaptation_rate = float(adaptation_rate)
        
        self.current_threshold = base_threshold
        self.false_alarm_count = 0
        self.missed_detection_count = 0
    
    def update(self, detection_occurred: bool, actual_fault: bool):
        """
        Update threshold based on detection performance.
        
        If false alarm: increase threshold (less sensitive)
        If missed detection: decrease threshold (more sensitive)
        """
        if detection_occurred and not actual_fault:
            # False alarm: threshold too low
            self.false_alarm_count += 1
            self.current_threshold = min(
                self.max_threshold,
                self.current_threshold * (1.0 + self.adaptation_rate)
            )
        elif not detection_occurred and actual_fault:
            # Missed detection: threshold too high
            self.missed_detection_count += 1
            self.current_threshold = max(
                self.min_threshold,
                self.current_threshold * (1.0 - self.adaptation_rate)
            )
    
    def get_threshold(self) -> float:
        """Return current adaptive threshold."""
        return self.current_threshold

