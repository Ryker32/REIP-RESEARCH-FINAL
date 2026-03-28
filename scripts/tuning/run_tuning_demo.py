"""
Demo: Run REIP Tuning Experiments

This demonstrates how to use the tuning framework to find optimal
configurations for different scenarios.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from reip_tuning_framework import run_tuning_suite

def main():
    print(" REIP Parameter Tuning Demo")
    print("=" * 50)
    
    # Choose experiment type
    experiment_types = {
        '1': ('quick', 'Quick test (8 scenarios) - Representative sample'),
        '2': ('security_focus', 'Security-focused (High attack scenarios)'),
        '3': ('performance_focus', 'Performance-focused (Low attack scenarios)'),
        '4': ('full', 'Full experiment suite (All combinations)')
    }
    
    print("Choose experiment type:")
    for key, (name, desc) in experiment_types.items():
        print(f"  {key}. {desc}")
    
    choice = input("\nEnter choice (1-4, default=1): ").strip() or '1'
    
    if choice in experiment_types:
        experiment_type, description = experiment_types[choice]
        print(f"\n Running {description}...")
        
        try:
            results_df, analysis, recommendations = run_tuning_suite(experiment_type)
            
            print("\n" + "="*60)
            print(" EXPERIMENT COMPLETE!")
            print("="*60)
            
            # Show key insights
            print("\n KEY INSIGHTS:")
            
            # When REIP is better
            reip_advantages = []
            baseline_advantages = []
            
            for _, row in results_df.iterrows():
                if row['winner'] == 'reip_enhanced':
                    reip_advantages.append(f"{row['attack_scenario']} + {row['reip_tuning']}")
                elif row['winner'] == 'baseline':
                    baseline_advantages.append(f"{row['attack_scenario']} + {row['mission_scenario']}")
            
            print(f"\n  Enhanced REIP wins in {len(reip_advantages)} scenarios:")
            for adv in reip_advantages[:5]:  # Show first 5
                print(f"   • {adv}")
            if len(reip_advantages) > 5:
                print(f"   ... and {len(reip_advantages) - 5} more")
            
            print(f"\n Baseline wins in {len(baseline_advantages)} scenarios:")
            for adv in baseline_advantages[:5]:  # Show first 5
                print(f"   • {adv}")
            if len(baseline_advantages) > 5:
                print(f"   ... and {len(baseline_advantages) - 5} more")
            
            # Show deployment recommendations
            print("\n DEPLOYMENT RECOMMENDATIONS:")
            for mission_type, rec in recommendations['deployment_guidelines'].items():
                print(f"\n   {mission_type.upper()}:")
                print(f"     System: {rec['recommended_system']}")
                print(f"     Reason: {rec['reason']}")
                if 'best_config' in rec:
                    print(f"     Config: {rec['best_config']}")
            
            print(f"\n Detailed results saved to: results/tuning_experiments/")
            
        except Exception as e:
            print(f" Error running experiments: {e}")
            print("Make sure you have the required dependencies and configs")
    
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()