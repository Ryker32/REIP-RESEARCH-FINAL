"""
Simple REIP Tuning Framework - Find Optimal Balance Points

This demonstrates how different parameter settings affect the balance between
security (attack resistance) and performance (completion speed).
"""

import yaml
import os
import sys
from pathlib import Path
import json
from datetime import datetime

class SimpleREIPTuning:
    """
    Simplified tuning framework to demonstrate optimal parameter finding.
    """
    
    def __init__(self):
        self.results = []
        
    def generate_test_configs(self):
        """
        Generate test configurations exploring different parameter combinations.
        """
        # Base configuration template
        base_config = {
            'T': 400,
            'N': 8,
            'map_size': 60,
            'lam': 3,
            'comm': {'radius': 3, 'p_loss': 0.05, 'p_corrupt': 0.02},
            'agent': {'r_local': 8, 'ds': 4, 'share_interval': 2},
            'env': {'obstacle_density': 0.05, 'seed': 42},
            'min_hold': 3,
            'known_penalty': 0.6,
            'recent_penalty': 1.2,
            'recent_window': 10,
            'prox_beta': 0.6,
        }
        
        # Test scenarios: exploring attack rates vs REIP sensitivity
        scenarios = [
            # No attacks - baseline comparison
            {
                'name': 'no_attacks_baseline',
                'description': 'No cyber attacks - pure performance test',
                'attack_rate': 0.0,
                'reip_sensitivity': 'moderate'
            },
            
            # Light attacks with different REIP settings
            {
                'name': 'light_attacks_tolerant',
                'description': 'Light attacks (2%) with tolerant REIP',
                'attack_rate': 0.02,
                'reip_sensitivity': 'tolerant'
            },
            {
                'name': 'light_attacks_moderate',
                'description': 'Light attacks (2%) with moderate REIP',
                'attack_rate': 0.02,
                'reip_sensitivity': 'moderate'
            },
            {
                'name': 'light_attacks_sensitive',
                'description': 'Light attacks (2%) with sensitive REIP',
                'attack_rate': 0.02,
                'reip_sensitivity': 'sensitive'
            },
            
            # Heavy attacks with different REIP settings
            {
                'name': 'heavy_attacks_tolerant',
                'description': 'Heavy attacks (10%) with tolerant REIP',
                'attack_rate': 0.10,
                'reip_sensitivity': 'tolerant'
            },
            {
                'name': 'heavy_attacks_moderate',
                'description': 'Heavy attacks (10%) with moderate REIP',
                'attack_rate': 0.10,
                'reip_sensitivity': 'moderate'
            },
            {
                'name': 'heavy_attacks_sensitive',
                'description': 'Heavy attacks (10%) with sensitive REIP',
                'attack_rate': 0.10,
                'reip_sensitivity': 'sensitive'
            },
        ]
        
        # REIP sensitivity presets
        reip_presets = {
            'tolerant': {
                'trust_decay_rate': 0.05,      # Slow trust loss
                'trust_threshold': 0.50,       # Lower trust requirement
                'loop_detection_window': 20,   # Longer detection window
                'loop_threshold': 0.90,        # Less sensitive to loops
                'impeachment_cooldown': 40,    # Slower impeachment
                'demotion_duration': 60,       # Longer demotion
            },
            'moderate': {
                'trust_decay_rate': 0.15,      # Standard trust loss
                'trust_threshold': 0.70,       # Standard trust requirement
                'loop_detection_window': 12,   # Standard detection window
                'loop_threshold': 0.75,        # Standard loop sensitivity
                'impeachment_cooldown': 25,    # Standard impeachment
                'demotion_duration': 40,       # Standard demotion
            },
            'sensitive': {
                'trust_decay_rate': 0.25,      # Fast trust loss
                'trust_threshold': 0.80,       # High trust requirement
                'loop_detection_window': 8,    # Shorter detection window
                'loop_threshold': 0.60,        # More sensitive to loops
                'impeachment_cooldown': 15,    # Faster impeachment
                'demotion_duration': 30,       # Shorter demotion
            }
        }
        
        # Generate configurations
        configs = []
        for scenario in scenarios:
            # Enhanced REIP config
            reip_config = base_config.copy()
            reip_config.update({
                'name': f"tuning/{scenario['name']}/reip_enhanced",
                'policy': 'reip_enhanced',
                'reip': {
                    'cyber_attack_rate': scenario['attack_rate'],
                    **reip_presets[scenario['reip_sensitivity']]
                }
            })
            
            # Baseline config
            baseline_config = base_config.copy()
            baseline_config.update({
                'name': f"tuning/{scenario['name']}/baseline",
                'policy': 'baseline',
                'reip': {
                    'cyber_attack_rate': scenario['attack_rate']
                }
            })
            
            configs.append({
                'scenario': scenario,
                'reip_config': reip_config,
                'baseline_config': baseline_config
            })
        
        return configs
    
    def run_single_test(self, config, policy_type):
        """
        Run a single test configuration.
        """
        # Create temporary config file
        config_path = f"temp_config_{policy_type}.yaml"
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        
        try:
            # Import and run simulation
            from src.main import run_simulation
            metrics = run_simulation(config_path, quiet=True)
            
            return {
                'completed': metrics.get('completed', False),
                'completion_time': metrics.get('completion_time', 999999),
                'coverage': metrics.get('coverage', 0.0),
                'cyber_attacks': metrics.get('cyber_attacks', 0),
                'elections': metrics.get('elections', 0),
                'impeachments': metrics.get('impeachments', 0)
            }
            
        except Exception as e:
            print(f"   ❌ Error: {e}")
            return {
                'completed': False,
                'completion_time': 999999,
                'coverage': 0.0,
                'cyber_attacks': 0,
                'elections': 0,
                'impeachments': 0
            }
        finally:
            # Clean up temp file
            if os.path.exists(config_path):
                os.remove(config_path)
    
    def run_comparison(self, config_set):
        """
        Run comparison between Enhanced REIP and Baseline for a configuration set.
        """
        scenario = config_set['scenario']
        print(f"\n🔬 Testing: {scenario['description']}")
        
        # Run Enhanced REIP
        print("   🛡️  Running Enhanced REIP...")
        reip_results = self.run_single_test(config_set['reip_config'], 'reip')
        
        # Run Baseline
        print("   ⚡ Running Baseline...")
        baseline_results = self.run_single_test(config_set['baseline_config'], 'baseline')
        
        # Compare results
        comparison = {
            'scenario_name': scenario['name'],
            'description': scenario['description'],
            'attack_rate': scenario['attack_rate'],
            'reip_sensitivity': scenario['reip_sensitivity'],
            'timestamp': datetime.now().isoformat(),
            
            # Enhanced REIP results
            'reip_completed': reip_results['completed'],
            'reip_time': reip_results['completion_time'],
            'reip_coverage': reip_results['coverage'],
            'reip_attacks': reip_results['cyber_attacks'],
            'reip_impeachments': reip_results['impeachments'],
            
            # Baseline results
            'baseline_completed': baseline_results['completed'],
            'baseline_time': baseline_results['completion_time'],
            'baseline_coverage': baseline_results['coverage'],
            'baseline_attacks': baseline_results['cyber_attacks'],
            
            # Performance comparison
            'speed_advantage': 'N/A',
            'security_advantage': 'N/A',
            'winner': 'tie'
        }
        
        # Calculate advantages
        if baseline_results['completed'] and reip_results['completed']:
            if baseline_results['completion_time'] > 0:
                speed_advantage = (baseline_results['completion_time'] - reip_results['completion_time']) / baseline_results['completion_time']
                comparison['speed_advantage'] = f"{speed_advantage:.1%}"
        
        # Determine winner based on completion and effectiveness
        reip_score = reip_results['coverage'] * (1.0 if reip_results['completed'] else 0.5)
        baseline_score = baseline_results['coverage'] * (1.0 if baseline_results['completed'] else 0.5)
        
        if reip_score > baseline_score * 1.1:  # 10% advantage threshold
            comparison['winner'] = 'Enhanced REIP'
            comparison['winner_reason'] = 'Better task completion despite attacks'
        elif baseline_score > reip_score * 1.1:
            comparison['winner'] = 'Baseline'
            comparison['winner_reason'] = 'Faster completion with acceptable security'
        else:
            comparison['winner'] = 'Tie'
            comparison['winner_reason'] = 'Similar performance'
        
        # Security advantage
        if reip_results['completed'] and not baseline_results['completed']:
            comparison['security_advantage'] = 'Enhanced REIP survived attacks'
        elif baseline_results['completed'] and not reip_results['completed']:
            comparison['security_advantage'] = 'Baseline completed despite vulnerability'
        
        # Print results
        print(f"   📊 Results:")
        print(f"      Enhanced REIP: {'✅' if reip_results['completed'] else '❌'} "
              f"({reip_results['completion_time']} steps, {reip_results['coverage']:.1%} coverage)")
        print(f"      Baseline:      {'✅' if baseline_results['completed'] else '❌'} "
              f"({baseline_results['completion_time']} steps, {baseline_results['coverage']:.1%} coverage)")
        print(f"      Winner: {comparison['winner']} - {comparison['winner_reason']}")
        
        return comparison
    
    def run_full_tuning_suite(self):
        """
        Run the complete tuning suite and analyze results.
        """
        print("🎯 REIP Parameter Tuning Suite")
        print("=" * 50)
        print("Testing optimal balance between security and performance...")
        
        # Generate test configurations
        configs = self.generate_test_configs()
        
        # Run all comparisons
        results = []
        for i, config_set in enumerate(configs):
            print(f"\nProgress: {i+1}/{len(configs)}")
            result = self.run_comparison(config_set)
            results.append(result)
        
        # Analyze results
        self.analyze_and_report(results)
        
        return results
    
    def analyze_and_report(self, results):
        """
        Analyze results and generate recommendations.
        """
        print("\n" + "="*60)
        print("🎯 TUNING ANALYSIS & RECOMMENDATIONS")
        print("="*60)
        
        # Count wins by system
        reip_wins = [r for r in results if r['winner'] == 'Enhanced REIP']
        baseline_wins = [r for r in results if r['winner'] == 'Baseline']
        ties = [r for r in results if r['winner'] == 'Tie']
        
        print(f"\n📊 OVERALL RESULTS:")
        print(f"   Enhanced REIP wins: {len(reip_wins)}/{len(results)}")
        print(f"   Baseline wins:      {len(baseline_wins)}/{len(results)}")
        print(f"   Ties:               {len(ties)}/{len(results)}")
        
        # Analyze by attack rate
        print(f"\n🔍 ANALYSIS BY ATTACK INTENSITY:")
        
        attack_rates = {}
        for result in results:
            rate = result['attack_rate']
            if rate not in attack_rates:
                attack_rates[rate] = {'reip': 0, 'baseline': 0, 'tie': 0}
            
            if result['winner'] == 'Enhanced REIP':
                attack_rates[rate]['reip'] += 1
            elif result['winner'] == 'Baseline':
                attack_rates[rate]['baseline'] += 1
            else:
                attack_rates[rate]['tie'] += 1
        
        for rate, counts in sorted(attack_rates.items()):
            total = sum(counts.values())
            print(f"   Attack rate {rate:.0%}: REIP={counts['reip']}/{total}, "
                  f"Baseline={counts['baseline']}/{total}, Tie={counts['tie']}/{total}")
        
        # Generate recommendations
        print(f"\n🎯 DEPLOYMENT RECOMMENDATIONS:")
        
        if len([r for r in results if r['attack_rate'] == 0.0 and r['winner'] == 'Baseline']) > 0:
            print("   📋 NO ATTACK scenarios: Use Baseline for maximum speed")
        
        if len([r for r in results if r['attack_rate'] >= 0.05 and r['winner'] == 'Enhanced REIP']) > 0:
            print("   📋 HIGH ATTACK scenarios: Use Enhanced REIP with appropriate sensitivity")
        
        # Best configurations
        print(f"\n⚙️  OPTIMAL CONFIGURATIONS:")
        
        # Find best REIP config for high attacks
        high_attack_reip = [r for r in reip_wins if r['attack_rate'] >= 0.05]
        if high_attack_reip:
            best = max(high_attack_reip, key=lambda x: x['reip_coverage'])
            print(f"   🛡️  High attack defense: {best['reip_sensitivity']} sensitivity")
            print(f"      (Scenario: {best['description']})")
        
        # When baseline is better
        baseline_scenarios = [r['description'] for r in baseline_wins]
        if baseline_scenarios:
            print(f"   ⚡ Baseline advantages:")
            for scenario in baseline_scenarios[:3]:  # Show top 3
                print(f"      • {scenario}")
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = f"tuning_results_{timestamp}.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        print(f"\n📁 Detailed results saved to: {results_file}")


def main():
    """
    Run the simple tuning framework demo.
    """
    try:
        tuner = SimpleREIPTuning()
        results = tuner.run_full_tuning_suite()
        
        print("\n✅ Tuning analysis complete!")
        print("This demonstrates how parameter tuning can optimize REIP for different scenarios.")
        
    except KeyboardInterrupt:
        print("\n❌ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("Make sure you have the simulation framework properly set up.")


if __name__ == "__main__":
    main()