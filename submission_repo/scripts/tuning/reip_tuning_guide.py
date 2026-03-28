"""
REIP Optimal Tuning Guide & Demo

This demonstrates the key principles of tuning Enhanced REIP for different scenarios
and shows when each system performs best.
"""

def demonstrate_tuning_principles():
    """
    Demonstrate the key tuning principles with examples.
    """
    
    print(" REIP OPTIMAL TUNING GUIDE")
    print("=" * 60)
    
    print("\n PARAMETER SPACE EXPLORATION:")
    print("   We tune Enhanced REIP across multiple dimensions:")
    
    # Show parameter relationships
    parameters = {
        'Trust System': {
            'trust_decay_rate': {
                'low (0.02-0.05)': 'Slow trust loss -> More tolerant to occasional bad decisions',
                'medium (0.10-0.20)': 'Balanced trust management -> Standard governance',
                'high (0.25-0.50)': 'Fast trust loss -> Strict governance, quick impeachment'
            },
            'trust_threshold': {
                'low (0.3-0.5)': 'Low bar for leadership -> Faster leader selection',
                'medium (0.6-0.8)': 'Moderate standards -> Balanced selection',
                'high (0.8-0.95)': 'High bar for leadership -> Only highly trusted agents lead'
            }
        },
        'Loop Detection': {
            'loop_detection_window': {
                'short (5-10)': 'Quick detection -> Fast response to attacks',
                'medium (10-20)': 'Balanced detection -> Standard monitoring',
                'long (20-40)': 'Patient monitoring -> Reduces false positives'
            },
            'loop_threshold': {
                'sensitive (0.5-0.7)': 'Detects subtle patterns -> High security',
                'moderate (0.7-0.8)': 'Standard detection -> Balanced approach',
                'tolerant (0.8-0.95)': 'Only obvious loops -> Performance focused'
            }
        },
        'Governance Speed': {
            'impeachment_cooldown': {
                'fast (10-20)': 'Rapid impeachment -> Quick leader changes',
                'moderate (20-40)': 'Balanced impeachment -> Standard governance',
                'slow (40-80)': 'Patient governance -> Stable leadership'
            },
            'demotion_duration': {
                'short (20-40)': 'Quick recovery -> Fast rehabilitation',
                'medium (40-60)': 'Standard recovery -> Balanced punishment',
                'long (60-100)': 'Extended punishment -> Strong deterrent'
            }
        }
    }
    
    for category, params in parameters.items():
        print(f"\n {category}:")
        for param, settings in params.items():
            print(f"    {param}:")
            for setting, description in settings.items():
                print(f"      • {setting}: {description}")
    
    print("\n" + "="*60)
    print(" OPTIMAL CONFIGURATIONS BY SCENARIO")
    print("="*60)
    
    scenarios = {
        'High Security (Heavy Attacks)': {
            'description': 'Mission faces frequent cyber attacks (>8% attack rate)',
            'optimal_config': 'Sensitive',
            'parameters': {
                'trust_decay_rate': 0.25,      # Fast trust loss
                'trust_threshold': 0.80,       # High trust requirement  
                'loop_detection_window': 8,    # Quick detection
                'loop_threshold': 0.60,        # Sensitive to loops
                'impeachment_cooldown': 15,    # Fast impeachment
                'demotion_duration': 30        # Shorter punishment
            },
            'rationale': 'Quick detection and response to compromised leaders',
            'trade_offs': 'May impeach good leaders occasionally, slower task completion'
        },
        
        'Balanced Security-Performance': {
            'description': 'Standard missions with moderate attack risk (2-5% attack rate)',
            'optimal_config': 'Moderate',
            'parameters': {
                'trust_decay_rate': 0.15,      # Moderate trust loss
                'trust_threshold': 0.70,       # Balanced trust requirement
                'loop_detection_window': 12,   # Standard detection
                'loop_threshold': 0.75,        # Balanced sensitivity
                'impeachment_cooldown': 25,    # Standard impeachment
                'demotion_duration': 40        # Standard punishment
            },
            'rationale': 'Good balance between security and task completion speed',
            'trade_offs': 'Moderate protection, reasonable performance'
        },
        
        'High Performance (Low Attacks)': {
            'description': 'Time-critical missions with minimal attack risk (<2% attack rate)',
            'optimal_config': 'Tolerant',
            'parameters': {
                'trust_decay_rate': 0.05,      # Slow trust loss
                'trust_threshold': 0.50,       # Lower trust requirement
                'loop_detection_window': 20,   # Patient detection
                'loop_threshold': 0.90,        # Less sensitive
                'impeachment_cooldown': 40,    # Slower impeachment
                'demotion_duration': 60        # Longer punishment
            },
            'rationale': 'Minimize governance overhead for maximum speed',
            'trade_offs': 'Faster completion but vulnerable to sophisticated attacks'
        },
        
        'Speed-Critical (No Attacks)': {
            'description': 'Emergency missions where speed is paramount',
            'optimal_config': 'Baseline System',
            'parameters': {
                'system': 'baseline_leader_follower',
                'governance': 'none'
            },
            'rationale': 'No governance overhead, maximum exploration speed',
            'trade_offs': 'No attack protection, single point of failure'
        }
    }
    
    for scenario_name, scenario in scenarios.items():
        print(f"\n {scenario_name.upper()}:")
        print(f"    Description: {scenario['description']}")
        print(f"     Optimal Config: {scenario['optimal_config']}")
        
        if 'parameters' in scenario and scenario['optimal_config'] != 'Baseline System':
            print(f"    Key Parameters:")
            for param, value in scenario['parameters'].items():
                print(f"      • {param}: {value}")
        
        print(f"    Rationale: {scenario['rationale']}")
        print(f"     Trade-offs: {scenario['trade_offs']}")
    
    print("\n" + "="*60)
    print(" DEPLOYMENT DECISION MATRIX")
    print("="*60)
    
    decision_matrix = """
    Attack Rate  | Mission Time | Recommended System | Config
    -------------|--------------|-------------------|----------
    0%           | Any          | Baseline          | N/A
    1-3%         | Critical     | Baseline          | N/A  
    1-3%         | Standard     | Enhanced REIP     | Tolerant
    4-7%         | Any          | Enhanced REIP     | Moderate
    8-15%        | Any          | Enhanced REIP     | Sensitive
    >15%         | Any          | Enhanced REIP     | Sensitive+
    """
    
    print(decision_matrix)
    
    print("\n EXPECTED PERFORMANCE CHARACTERISTICS:")
    
    performance_expectations = {
        'Baseline System': {
            'completion_time': '100% (fastest)',
            'attack_survival': '0% (no defense)',
            'best_for': 'No attacks, speed critical'
        },
        'Enhanced REIP (Tolerant)': {
            'completion_time': '110-120% (slight overhead)',
            'attack_survival': '60-70% (basic protection)',
            'best_for': 'Light attacks, performance important'
        },
        'Enhanced REIP (Moderate)': {
            'completion_time': '120-150% (moderate overhead)',
            'attack_survival': '80-90% (good protection)',
            'best_for': 'Moderate attacks, balanced needs'
        },
        'Enhanced REIP (Sensitive)': {
            'completion_time': '150-200% (high overhead)',
            'attack_survival': '95%+ (excellent protection)',
            'best_for': 'Heavy attacks, security critical'
        }
    }
    
    for system, metrics in performance_expectations.items():
        print(f"\n    {system}:")
        for metric, value in metrics.items():
            print(f"      • {metric}: {value}")
    
    print("\n" + "="*60)
    print(" HOW TO RUN TUNING EXPERIMENTS")
    print("="*60)
    
    experimental_process = """
    1. DEFINE TEST SCENARIOS:
       • Create attack rate variations (0%, 2%, 5%, 10%, 20%)
       • Define mission constraints (time limits, complexity)
       • Set environment parameters (map size, agent count)
    
    2. CONFIGURE PARAMETER SWEEPS:
       • Test sensitivity presets (tolerant, moderate, sensitive)
       • Vary individual parameters systematically
       • Use controlled random seeds for reproducibility
    
    3. RUN COMPARATIVE EXPERIMENTS:
       • Test Enhanced REIP vs Baseline for each scenario
       • Use identical attack schedules (controlled cyber_attack_scheduler)
       • Measure completion time, coverage, survival rate
    
    4. ANALYZE RESULTS:
       • Plot performance vs attack rate curves
       • Identify optimal parameter combinations
       • Find break-even points where REIP becomes beneficial
    
    5. VALIDATE FINDINGS:
       • Test optimal configs on new scenarios
       • Verify robustness across different environments
       • Document deployment guidelines
    """
    
    print(experimental_process)
    
    print("\n SUMMARY:")
    print("   The key to optimal REIP tuning is matching governance sensitivity")
    print("   to the threat environment. More attacks = more sensitive governance.")
    print("   Less attacks = more tolerant governance for better performance.")
    print("\n   Run systematic experiments to find the exact break-even points")
    print("   for your specific mission requirements!")


def show_sample_configs():
    """
    Show sample configuration files for different scenarios.
    """
    print("\n" + "="*60)
    print(" SAMPLE CONFIGURATION FILES")
    print("="*60)
    
    configs = {
        'High Security (Sensitive REIP)': {
            'filename': 'reip_high_security.yaml',
            'content': """# Enhanced REIP: High Security Configuration
# Use for heavy attack scenarios (>8% attack rate)

name: "enhanced_reip/high_security"
policy: "reip_enhanced"
T: 400
N: 8
map_size: 60
lam: 3

env:
  obstacle_density: 0.05
  seed: 42

reip:
  # High security parameters
  trust_decay_rate: 0.25       # Fast trust loss
  trust_threshold: 0.80        # High trust requirement
  min_trust: 0.10
  trust_recovery_rate: 0.05
  
  # Sensitive loop detection
  loop_detection_window: 8     # Quick detection
  loop_threshold: 0.60         # Sensitive to loops
  aimless_detection_window: 6
  stagnation_threshold: 0.01
  
  # Rapid governance
  impeachment_cooldown: 15     # Fast impeachment
  demotion_duration: 30        # Short punishment
  
  # Attack parameters
  cyber_attack_rate: 0.10      # 10% attack rate (heavy)
  hallucination_rate: 0.0
"""
        },
        
        'Balanced Performance (Moderate REIP)': {
            'filename': 'reip_balanced.yaml',
            'content': """# Enhanced REIP: Balanced Configuration
# Use for standard missions with moderate attacks (2-5% attack rate)

name: "enhanced_reip/balanced"
policy: "reip_enhanced"
T: 400
N: 8
map_size: 60
lam: 3

env:
  obstacle_density: 0.05
  seed: 42

reip:
  # Balanced parameters
  trust_decay_rate: 0.15       # Moderate trust loss
  trust_threshold: 0.70        # Balanced trust requirement
  min_trust: 0.10
  trust_recovery_rate: 0.05
  
  # Standard loop detection
  loop_detection_window: 12    # Standard detection
  loop_threshold: 0.75         # Balanced sensitivity
  aimless_detection_window: 8
  stagnation_threshold: 0.02
  
  # Standard governance
  impeachment_cooldown: 25     # Standard impeachment
  demotion_duration: 40        # Standard punishment
  
  # Attack parameters
  cyber_attack_rate: 0.05      # 5% attack rate (moderate)
  hallucination_rate: 0.0
"""
        },
        
        'High Performance (Tolerant REIP)': {
            'filename': 'reip_high_performance.yaml',
            'content': """# Enhanced REIP: High Performance Configuration
# Use for time-critical missions with light attacks (<2% attack rate)

name: "enhanced_reip/high_performance"
policy: "reip_enhanced"
T: 400
N: 8
map_size: 60
lam: 3

env:
  obstacle_density: 0.05
  seed: 42

reip:
  # Performance-focused parameters
  trust_decay_rate: 0.05       # Slow trust loss
  trust_threshold: 0.50        # Lower trust requirement
  min_trust: 0.10
  trust_recovery_rate: 0.05
  
  # Tolerant loop detection
  loop_detection_window: 20    # Patient detection
  loop_threshold: 0.90         # Less sensitive
  aimless_detection_window: 10
  stagnation_threshold: 0.03
  
  # Patient governance
  impeachment_cooldown: 40     # Slower impeachment
  demotion_duration: 60        # Longer punishment
  
  # Attack parameters
  cyber_attack_rate: 0.02      # 2% attack rate (light)
  hallucination_rate: 0.0
"""
        },
        
        'Speed Critical (Baseline)': {
            'filename': 'baseline_speed_critical.yaml',
            'content': """# Baseline: Speed Critical Configuration
# Use when maximum speed is required and no attacks expected

name: "baseline/speed_critical"
policy: "baseline"
T: 200                         # Shorter mission for speed
N: 8
map_size: 60
lam: 3

env:
  obstacle_density: 0.05
  seed: 42

reip:
  # No governance overhead
  cyber_attack_rate: 0.0       # No attacks expected
  hallucination_rate: 0.0
  
  # Minimal communication constraints
  command_loss_rate: 0.0
  command_radius: 100
"""
        }
    }
    
    for config_name, config_info in configs.items():
        print(f"\n {config_name}:")
        print(f"   File: {config_info['filename']}")
        print("   Content preview:")
        lines = config_info['content'].split('\n')
        for line in lines[:15]:  # Show first 15 lines
            print(f"   {line}")
        if len(lines) > 15:
            print(f"   ... ({len(lines) - 15} more lines)")


def main():
    """
    Run the complete tuning guide demonstration.
    """
    try:
        demonstrate_tuning_principles()
        show_sample_configs()
        
        print("\n" + "="*60)
        print(" NEXT STEPS FOR IMPLEMENTATION")
        print("="*60)
        
        next_steps = """
        1.  CHOOSE YOUR SCENARIO:
           • Assess your mission's attack risk level
           • Determine time criticality requirements
           • Evaluate environment complexity
        
        2.   SELECT OPTIMAL CONFIGURATION:
           • Use the decision matrix above
           • Start with recommended presets
           • Fine-tune based on initial results
        
        3.  RUN VALIDATION EXPERIMENTS:
           • Test your chosen config against attacks
           • Compare with baseline performance
           • Verify robustness across scenarios
        
        4.  MEASURE & OPTIMIZE:
           • Track completion time vs attack survival
           • Adjust parameters based on results
           • Document optimal settings for future use
        
        5.  DEPLOY WITH CONFIDENCE:
           • Use proven configurations for similar missions
           • Monitor performance in real deployments
           • Continuously refine based on experience
        """
        
        print(next_steps)
        
        print("\n You now have a complete framework for tuning Enhanced REIP!")
        print("   Use these principles to find optimal configurations for your specific scenarios.")
        
    except KeyboardInterrupt:
        print("\n Interrupted by user")


if __name__ == "__main__":
    main()