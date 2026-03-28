"""
REIP Tuning Framework - Find Optimal Balance Between Security and Performance

This framework systematically explores parameter combinations to find:
1. When Enhanced REIP outperforms Baseline (high attack scenarios)
2. When Baseline outperforms Enhanced REIP (low attack, time-critical scenarios)
3. Sweet spots for different mission profiles
"""

import yaml
import os
import itertools
from pathlib import Path
import pandas as pd
import numpy as np
from typing import Dict, List, Tuple, Any
import time
from datetime import datetime

class REIPTuningFramework:
    """
    Systematic parameter exploration for REIP vs Baseline performance.
    """
    
    def __init__(self, base_config_path: str = "configs/reip_cyber_attack.yaml"):
        self.base_config_path = base_config_path
        self.results_dir = Path("results/tuning_experiments")
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Load base configuration
        with open(base_config_path, 'r') as f:
            self.base_config = yaml.safe_load(f)
    
    def define_parameter_space(self):
        """
        Define the parameter space to explore.
        
        Returns tuning scenarios:
        1. Security-focused (high attack rates)
        2. Performance-focused (low attack rates)
        3. Balanced scenarios
        4. Stress tests (extreme conditions)
        """
        return {
            # Attack intensity scenarios
            'attack_scenarios': {
                'no_attacks': {'cyber_attack_rate': 0.0},
                'light_attacks': {'cyber_attack_rate': 0.02},  # 2% attack rate
                'moderate_attacks': {'cyber_attack_rate': 0.05},  # 5% attack rate
                'heavy_attacks': {'cyber_attack_rate': 0.10},  # 10% attack rate
                'extreme_attacks': {'cyber_attack_rate': 0.20},  # 20% attack rate
            },
            
            # REIP governance tuning
            'reip_tuning': {
                'sensitive': {
                    'trust_decay_rate': 0.25,       # Fast trust loss
                    'trust_threshold': 0.80,        # High trust requirement
                    'loop_detection_window': 8,     # Shorter detection window
                    'loop_threshold': 0.60,         # Lower threshold (more sensitive)
                    'impeachment_cooldown': 15,     # Faster impeachment
                    'demotion_duration': 30,        # Shorter demotion
                },
                'moderate': {
                    'trust_decay_rate': 0.15,       # Moderate trust loss
                    'trust_threshold': 0.70,        # Moderate trust requirement
                    'loop_detection_window': 12,    # Standard detection window
                    'loop_threshold': 0.75,         # Standard threshold
                    'impeachment_cooldown': 25,     # Standard impeachment
                    'demotion_duration': 40,        # Standard demotion
                },
                'tolerant': {
                    'trust_decay_rate': 0.05,       # Slow trust loss
                    'trust_threshold': 0.50,        # Lower trust requirement
                    'loop_detection_window': 20,    # Longer detection window
                    'loop_threshold': 0.90,         # Higher threshold (less sensitive)
                    'impeachment_cooldown': 40,     # Slower impeachment
                    'demotion_duration': 60,        # Longer demotion
                },
                'permissive': {
                    'trust_decay_rate': 0.02,       # Very slow trust loss
                    'trust_threshold': 0.30,        # Very low trust requirement
                    'loop_detection_window': 30,    # Very long detection window
                    'loop_threshold': 0.95,         # Very high threshold
                    'impeachment_cooldown': 60,     # Very slow impeachment
                    'demotion_duration': 80,        # Very long demotion
                }
            },
            
            # Mission scenarios
            'mission_scenarios': {
                'speed_critical': {'T': 200},      # Short mission
                'standard': {'T': 400},            # Standard mission
                'long_endurance': {'T': 800},      # Long mission
            },
            
            # Environment complexity
            'environment_scenarios': {
                'simple': {'map_size': 30, 'N': 4, 'obstacle_density': 0.02},
                'moderate': {'map_size': 60, 'N': 8, 'obstacle_density': 0.05},
                'complex': {'map_size': 90, 'N': 12, 'obstacle_density': 0.08},
            }
        }
    
    def generate_test_scenarios(self) -> List[Dict]:
        """
        Generate all test scenario combinations.
        """
        param_space = self.define_parameter_space()
        scenarios = []
        
        # Generate all combinations
        for attack_name, attack_params in param_space['attack_scenarios'].items():
            for reip_name, reip_params in param_space['reip_tuning'].items():
                for mission_name, mission_params in param_space['mission_scenarios'].items():
                    for env_name, env_params in param_space['environment_scenarios'].items():
                        
                        scenario = {
                            'name': f"{attack_name}_{reip_name}_{mission_name}_{env_name}",
                            'attack_scenario': attack_name,
                            'reip_tuning': reip_name,
                            'mission_scenario': mission_name,
                            'environment_scenario': env_name,
                            'params': {
                                **attack_params,
                                **reip_params,
                                **mission_params,
                                **env_params
                            }
                        }
                        scenarios.append(scenario)
        
        return scenarios
    
    def create_config(self, scenario: Dict, policy_type: str) -> str:
        """
        Create a configuration file for a specific scenario and policy.
        
        Args:
            scenario: Test scenario parameters
            policy_type: 'reip_enhanced' or 'baseline'
        
        Returns:
            Path to created config file
        """
        config = self.base_config.copy()
        
        # Update basic parameters
        config.update(scenario['params'])
        config['policy'] = policy_type
        config['name'] = f"tuning/{scenario['name']}/{policy_type}"
        
        # Update REIP-specific parameters
        if 'reip' not in config:
            config['reip'] = {}
        
        for key, value in scenario['params'].items():
            if key in ['trust_decay_rate', 'trust_threshold', 'loop_detection_window', 
                      'loop_threshold', 'impeachment_cooldown', 'demotion_duration', 
                      'cyber_attack_rate']:
                config['reip'][key] = value
        
        # Create config file
        config_dir = self.results_dir / "configs"
        config_dir.mkdir(exist_ok=True)
        config_path = config_dir / f"{scenario['name']}_{policy_type}.yaml"
        
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        return str(config_path)
    
    def run_experiment(self, scenario: Dict) -> Dict:
        """
        Run both Enhanced REIP and Baseline for a scenario and compare results.
        """
        print(f" Testing scenario: {scenario['name']}")
        
        results = {
            'scenario': scenario['name'],
            'attack_scenario': scenario['attack_scenario'],
            'reip_tuning': scenario['reip_tuning'],
            'mission_scenario': scenario['mission_scenario'],
            'environment_scenario': scenario['environment_scenario'],
            'timestamp': datetime.now().isoformat()
        }
        
        # Test both systems
        for policy_type in ['baseline', 'reip_enhanced']:
            print(f"   Running {policy_type}...")
            
            # Create config
            config_path = self.create_config(scenario, policy_type)
            
            # Run simulation
            try:
                from src.main import run_simulation
                metrics = run_simulation(config_path, quiet=True)
                
                results[f'{policy_type}_completion_time'] = metrics.get('completion_time', 999999)
                results[f'{policy_type}_coverage'] = metrics.get('coverage', 0.0)
                results[f'{policy_type}_cyber_attacks'] = metrics.get('cyber_attacks', 0)
                results[f'{policy_type}_elections'] = metrics.get('elections', 0)
                results[f'{policy_type}_impeachments'] = metrics.get('impeachments', 0)
                results[f'{policy_type}_completed'] = metrics.get('completed', False)
                
            except Exception as e:
                print(f"     Error running {policy_type}: {e}")
                results[f'{policy_type}_completion_time'] = 999999
                results[f'{policy_type}_coverage'] = 0.0
                results[f'{policy_type}_cyber_attacks'] = 0
                results[f'{policy_type}_elections'] = 0
                results[f'{policy_type}_impeachments'] = 0
                results[f'{policy_type}_completed'] = False
        
        # Calculate performance metrics
        self.calculate_performance_metrics(results)
        
        return results
    
    def calculate_performance_metrics(self, results: Dict):
        """
        Calculate comparative performance metrics.
        """
        # Speed advantage
        reip_time = results['reip_enhanced_completion_time']
        baseline_time = results['baseline_completion_time']
        
        if baseline_time > 0 and reip_time > 0:
            results['speed_advantage'] = (baseline_time - reip_time) / baseline_time
        else:
            results['speed_advantage'] = -999  # Failed comparison
        
        # Security advantage (ability to complete despite attacks)
        results['reip_security_score'] = (
            (1.0 if results['reip_enhanced_completed'] else 0.0) * 
            (1.0 - results['reip_enhanced_cyber_attacks'] / max(1, 100))
        )
        
        results['baseline_security_score'] = (
            (1.0 if results['baseline_completed'] else 0.0) * 
            (1.0 - results['baseline_cyber_attacks'] / max(1, 100))
        )
        
        # Overall effectiveness
        results['reip_effectiveness'] = (
            results['reip_enhanced_coverage'] * 
            (1.0 if results['reip_enhanced_completed'] else 0.5)
        )
        
        results['baseline_effectiveness'] = (
            results['baseline_coverage'] * 
            (1.0 if results['baseline_completed'] else 0.5)
        )
        
        # Winner determination
        if results['reip_effectiveness'] > results['baseline_effectiveness']:
            results['winner'] = 'reip_enhanced'
        elif results['baseline_effectiveness'] > results['reip_effectiveness']:
            results['winner'] = 'baseline'
        else:
            results['winner'] = 'tie'
    
    def run_full_experiment_suite(self, subset: str = None) -> pd.DataFrame:
        """
        Run the complete experiment suite.
        
        Args:
            subset: Run subset of tests ('quick', 'security_focus', 'performance_focus', 'full')
        """
        scenarios = self.generate_test_scenarios()
        
        # Filter scenarios based on subset
        if subset == 'quick':
            # Test a few representative scenarios
            scenarios = [s for s in scenarios if 
                        s['mission_scenario'] == 'standard' and 
                        s['environment_scenario'] == 'moderate'][:8]
        elif subset == 'security_focus':
            # Focus on high-attack scenarios
            scenarios = [s for s in scenarios if 
                        s['attack_scenario'] in ['moderate_attacks', 'heavy_attacks', 'extreme_attacks']]
        elif subset == 'performance_focus':
            # Focus on low-attack, time-critical scenarios
            scenarios = [s for s in scenarios if 
                        s['attack_scenario'] in ['no_attacks', 'light_attacks'] and
                        s['mission_scenario'] in ['speed_critical', 'standard']]
        
        print(f" Running {len(scenarios)} tuning experiments...")
        
        results = []
        for i, scenario in enumerate(scenarios):
            print(f"Progress: {i+1}/{len(scenarios)}")
            result = self.run_experiment(scenario)
            results.append(result)
            
            # Save incremental results
            if (i + 1) % 5 == 0:
                df = pd.DataFrame(results)
                df.to_csv(self.results_dir / f"tuning_results_progress_{i+1}.csv", index=False)
        
        # Create final results DataFrame
        df = pd.DataFrame(results)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        df.to_csv(self.results_dir / f"tuning_results_{timestamp}.csv", index=False)
        
        return df
    
    def analyze_results(self, df: pd.DataFrame) -> Dict:
        """
        Analyze results to find optimal configurations.
        """
        analysis = {}
        
        # Find best REIP configurations for different scenarios
        reip_wins = df[df['winner'] == 'reip_enhanced']
        baseline_wins = df[df['winner'] == 'baseline']
        
        analysis['reip_win_rate'] = len(reip_wins) / len(df)
        analysis['baseline_win_rate'] = len(baseline_wins) / len(df)
        
        # Best REIP configurations by attack scenario
        analysis['best_reip_configs'] = {}
        for attack_scenario in df['attack_scenario'].unique():
            scenario_data = reip_wins[reip_wins['attack_scenario'] == attack_scenario]
            if len(scenario_data) > 0:
                best = scenario_data.loc[scenario_data['reip_effectiveness'].idxmax()]
                analysis['best_reip_configs'][attack_scenario] = {
                    'reip_tuning': best['reip_tuning'],
                    'effectiveness': best['reip_effectiveness'],
                    'scenario_name': best['scenario']
                }
        
        # When baseline is better
        analysis['baseline_advantages'] = {}
        for attack_scenario in df['attack_scenario'].unique():
            scenario_data = baseline_wins[baseline_wins['attack_scenario'] == attack_scenario]
            if len(scenario_data) > 0:
                analysis['baseline_advantages'][attack_scenario] = len(scenario_data)
        
        # Performance vs Security trade-offs
        analysis['trade_offs'] = {
            'high_security_scenarios': list(reip_wins[reip_wins['attack_scenario'].isin(['heavy_attacks', 'extreme_attacks'])]['scenario']),
            'high_performance_scenarios': list(baseline_wins[baseline_wins['mission_scenario'] == 'speed_critical']['scenario'])
        }
        
        return analysis
    
    def generate_recommendations(self, analysis: Dict) -> Dict:
        """
        Generate deployment recommendations based on analysis.
        """
        recommendations = {
            'deployment_guidelines': {},
            'optimal_configs': {},
            'mission_profiles': {}
        }
        
        # Security-critical missions
        if analysis['reip_win_rate'] > 0.6:
            recommendations['deployment_guidelines']['security_critical'] = {
                'recommended_system': 'Enhanced REIP',
                'reason': 'Superior performance under attack conditions',
                'best_config': analysis['best_reip_configs'].get('heavy_attacks', {}).get('reip_tuning', 'moderate')
            }
        
        # Time-critical missions
        if analysis['baseline_win_rate'] > 0.3:
            recommendations['deployment_guidelines']['time_critical'] = {
                'recommended_system': 'Baseline',
                'reason': 'Faster completion when attacks are minimal',
                'conditions': 'Low attack environments'
            }
        
        # Balanced missions
        recommendations['deployment_guidelines']['balanced'] = {
            'recommended_system': 'Enhanced REIP',
            'config': 'moderate',
            'reason': 'Good balance of security and performance'
        }
        
        return recommendations


def run_tuning_suite(subset: str = 'quick'):
    """
    Convenience function to run the tuning framework.
    
    Args:
        subset: 'quick', 'security_focus', 'performance_focus', or 'full'
    """
    framework = REIPTuningFramework()
    
    print(f" Starting REIP tuning framework - {subset} test suite")
    
    # Run experiments
    results_df = framework.run_full_experiment_suite(subset)
    
    # Analyze results
    analysis = framework.analyze_results(results_df)
    
    # Generate recommendations
    recommendations = framework.generate_recommendations(analysis)
    
    # Save analysis
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    with open(framework.results_dir / f"analysis_{timestamp}.yaml", 'w') as f:
        yaml.dump(analysis, f, default_flow_style=False)
    
    with open(framework.results_dir / f"recommendations_{timestamp}.yaml", 'w') as f:
        yaml.dump(recommendations, f, default_flow_style=False)
    
    # Print summary
    print("\n" + "="*60)
    print(" REIP TUNING FRAMEWORK - SUMMARY")
    print("="*60)
    print(f" Total experiments: {len(results_df)}")
    print(f"  Enhanced REIP win rate: {analysis['reip_win_rate']:.1%}")
    print(f" Baseline win rate: {analysis['baseline_win_rate']:.1%}")
    
    print("\n DEPLOYMENT RECOMMENDATIONS:")
    for mission_type, rec in recommendations['deployment_guidelines'].items():
        print(f"  {mission_type}: {rec['recommended_system']} - {rec['reason']}")
    
    print(f"\n Results saved to: {framework.results_dir}")
    
    return results_df, analysis, recommendations


if __name__ == "__main__":
    # Run quick test suite by default
    run_tuning_suite('quick')