"""
Practical REIP Tuning Demonstration

This script demonstrates how to test the optimal configurations
and shows you the practical steps for tuning REIP parameters.
"""

import subprocess
import time
import yaml
import os
from pathlib import Path

def test_configuration(config_file, description):
    """
    Test a specific configuration and return basic results.
    """
    print(f"\n Testing: {description}")
    print(f" Config: {config_file}")
    print(" Running simulation...")
    
    try:
        # Run the simulation with timeout
        start_time = time.time()
        result = subprocess.run(
            ["python", "src/main.py", "--config", config_file],
            capture_output=True,
            text=True,
            timeout=30,  # 30 second timeout for demo
            encoding='utf-8',
            errors='replace'
        )
        
        elapsed_time = time.time() - start_time
        
        # Parse basic results from output
        output_lines = result.stdout.split('\n') if result.stdout else []
        
        # Look for completion indicators
        completed = any('100.0%' in line for line in output_lines)
        coverage_line = next((line for line in output_lines if 'Coverage:' in line), None)
        
        status = " COMPLETED" if completed else "⏱ TIMEOUT" 
        
        print(f"⏱  Time: {elapsed_time:.1f}s")
        print(f" Status: {status}")
        
        if coverage_line:
            print(f" {coverage_line.strip()}")
        
        return {
            'config': config_file,
            'description': description,
            'time': elapsed_time,
            'completed': completed,
            'status': status
        }
        
    except subprocess.TimeoutExpired:
        print("⏱  Status: TIMEOUT (30s limit)")
        return {
            'config': config_file,
            'description': description,
            'time': 30.0,
            'completed': False,
            'status': 'TIMEOUT'
        }
    except Exception as e:
        print(f" Error: {e}")
        return {
            'config': config_file,
            'description': description,
            'time': 0,
            'completed': False,
            'status': f'ERROR: {e}'
        }

def demonstrate_tuning_workflow():
    """
    Demonstrate the complete tuning workflow.
    """
    print(" PRACTICAL REIP TUNING DEMONSTRATION")
    print("=" * 60)
    
    # Define test configurations
    test_configs = [
        {
            'file': 'configs/baseline_speed_critical.yaml',
            'description': 'Baseline (Speed Critical)',
            'expected': 'Fastest completion, no attack protection'
        },
        {
            'file': 'configs/reip_high_performance.yaml', 
            'description': 'Enhanced REIP (Tolerant)',
            'expected': 'Good speed, basic attack protection'
        },
        {
            'file': 'configs/reip_balanced.yaml',
            'description': 'Enhanced REIP (Moderate)',
            'expected': 'Balanced speed and security'
        },
        {
            'file': 'configs/reip_high_security.yaml',
            'description': 'Enhanced REIP (Sensitive)',
            'expected': 'Best security, slower completion'
        }
    ]
    
    print("\n TEST PLAN:")
    for i, config in enumerate(test_configs, 1):
        print(f"   {i}. {config['description']}")
        print(f"       Config: {config['file']}")
        print(f"       Expected: {config['expected']}")
    
    print(f"\n⏱  Each test limited to 30 seconds for demonstration")
    print(" In real tuning, you'd run longer tests with multiple seeds")
    
    # Run the tests
    results = []
    for config in test_configs:
        if os.path.exists(config['file']):
            result = test_configuration(config['file'], config['description'])
            results.append(result)
        else:
            print(f"\n Config file not found: {config['file']}")
    
    # Summarize results
    print("\n" + "=" * 60)
    print(" RESULTS SUMMARY")
    print("=" * 60)
    
    for result in results:
        print(f"\n {result['description']}:")
        print(f"   ⏱  Time: {result['time']:.1f}s")
        print(f"    Status: {result['status']}")
        
        # Add interpretation
        if result['completed']:
            if result['time'] < 10:
                print("    Analysis: Very fast completion - good for speed-critical missions")
            elif result['time'] < 20:
                print("    Analysis: Good balance of speed and governance overhead")
            else:
                print("    Analysis: Slower but likely more secure governance")
        else:
            print("    Analysis: May need parameter adjustment or longer runtime")
    
    print("\n" + "=" * 60)
    print(" TUNING INSIGHTS")
    print("=" * 60)
    
    insights = """
    Key Tuning Principles Demonstrated:

    1.  PERFORMANCE TRADE-OFFS:
       • Baseline systems complete fastest (no governance overhead)
       • Enhanced REIP adds security at cost of speed
       • More sensitive settings = more overhead

    2.  PARAMETER SENSITIVITY:
       • trust_threshold: Higher values slow leader selection
       • loop_detection_window: Smaller windows catch attacks faster
       • impeachment_cooldown: Faster impeachment = quicker response

    3.  OPTIMAL SELECTION STRATEGY:
       • No attacks expected -> Use Baseline
       • Light attacks (1-3%) -> Use Tolerant REIP 
       • Moderate attacks (4-7%) -> Use Moderate REIP
       • Heavy attacks (>8%) -> Use Sensitive REIP

    4.  REAL TUNING PROCESS:
       • Test with longer timeouts (5-10 minutes)
       • Use multiple random seeds for statistical significance
       • Gradually increase attack rates to find break-even points
       • Measure both completion time AND attack survival rate
    """
    
    print(insights)

def show_parameter_ranges():
    """
    Show the valid parameter ranges for tuning.
    """
    print("\n" + "=" * 60)
    print(" PARAMETER TUNING RANGES")
    print("=" * 60)
    
    parameter_ranges = {
        'Trust Management': {
            'trust_decay_rate': {
                'range': '0.01 - 0.50',
                'optimal': {
                    'Tolerant': '0.05',
                    'Moderate': '0.15', 
                    'Sensitive': '0.25'
                },
                'effect': 'Higher values = faster trust loss = stricter governance'
            },
            'trust_threshold': {
                'range': '0.30 - 0.95',
                'optimal': {
                    'Tolerant': '0.50',
                    'Moderate': '0.70',
                    'Sensitive': '0.80'
                },
                'effect': 'Higher values = higher bar for leadership'
            }
        },
        'Loop Detection': {
            'loop_detection_window': {
                'range': '5 - 40',
                'optimal': {
                    'Tolerant': '20',
                    'Moderate': '12',
                    'Sensitive': '8'
                },
                'effect': 'Smaller windows = faster attack detection'
            },
            'loop_threshold': {
                'range': '0.50 - 0.95',
                'optimal': {
                    'Tolerant': '0.90',
                    'Moderate': '0.75',
                    'Sensitive': '0.60'
                },
                'effect': 'Lower values = more sensitive to loops'
            }
        },
        'Governance Speed': {
            'impeachment_cooldown': {
                'range': '10 - 80',
                'optimal': {
                    'Tolerant': '40',
                    'Moderate': '25',
                    'Sensitive': '15'
                },
                'effect': 'Lower values = faster impeachment response'
            },
            'demotion_duration': {
                'range': '20 - 100',
                'optimal': {
                    'Tolerant': '60',
                    'Moderate': '40',
                    'Sensitive': '30'
                },
                'effect': 'Shorter values = faster agent rehabilitation'
            }
        }
    }
    
    for category, params in parameter_ranges.items():
        print(f"\n {category}:")
        for param, info in params.items():
            print(f"\n    {param}:")
            print(f"       Valid Range: {info['range']}")
            print(f"       Optimal Values:")
            for config, value in info['optimal'].items():
                print(f"         • {config}: {value}")
            print(f"       Effect: {info['effect']}")

def main():
    """
    Run the complete practical demonstration.
    """
    try:
        demonstrate_tuning_workflow()
        show_parameter_ranges()
        
        print("\n" + "=" * 60)
        print(" HOW TO USE THIS FOR REAL TUNING")
        print("=" * 60)
        
        usage_guide = """
        1.  START WITH PRESETS:
           • Use the provided optimal configurations as starting points
           • Test baseline_speed_critical.yaml for no-attack scenarios
           • Test reip_balanced.yaml for moderate security needs

        2.  SYSTEMATIC TESTING:
           • Create attack rate variations (0%, 2%, 5%, 10%, 15%, 20%)
           • Test each REIP configuration against each attack rate
           • Run multiple seeds (5-10) for statistical significance

        3.  MEASURE KEY METRICS:
           • Completion time (lower is better for performance)
           • Coverage percentage (higher is better)
           • Attack survival rate (higher is better for security)
           • Mission success rate across multiple runs

        4.  FIND BREAK-EVEN POINTS:
           • Plot completion time vs attack rate for each configuration
           • Identify where Enhanced REIP starts outperforming Baseline
           • Document optimal configs for different threat levels

        5.  CUSTOM PARAMETER TUNING:
           • Start with closest preset configuration
           • Adjust one parameter at a time
           • Test thoroughly before moving to next parameter
           • Document parameter interactions and dependencies

        6.  VALIDATION & DEPLOYMENT:
           • Test optimal configs on new scenarios
           • Verify robustness across different environments
           • Create deployment guidelines for operations teams
        """
        
        print(usage_guide)
        
        print("\n You now have practical tools for REIP parameter tuning!")
        print("   Use the provided configurations and follow the systematic process above.")
        
    except KeyboardInterrupt:
        print("\n Interrupted by user")

if __name__ == "__main__":
    main()