"""
REIP vs Baseline: Direct Exploration Comparison

This script directly compares Enhanced REIP and Baseline systems
on identical exploration tasks to measure which is better.
"""

import subprocess
import json
import yaml
import time
import sys
from pathlib import Path
from datetime import datetime

def run_simulation(config_file, description, timeout=300):
    """
    Run a simulation with a specific configuration.
    Returns the key metrics.
    """
    print(f"\n🔄 Running: {description}")
    print(f"📄 Config: {config_file}")
    print(f"⏱️  Timeout: {timeout}s")
    
    start_time = time.time()
    
    try:
        result = subprocess.run(
            ["python", "src/main.py", "--config", config_file],
            capture_output=True,
            text=True,
            timeout=timeout,
            encoding='utf-8',
            errors='replace',
            cwd="C:\\Users\\ryker\\REIP-Research\\reip-sim"
        )
        
        elapsed_time = time.time() - start_time
        
        # Extract run name from config
        cfg_dict = yaml.safe_load(open(config_file))
        run_name = cfg_dict.get("name", "default")
        log_path = Path("runs") / run_name / "log.csv"
        
        metrics = {
            'description': description,
            'config': config_file,
            'elapsed_time': elapsed_time,
            'completed': False,
            'coverage': 0.0,
            'exploration_efficiency': 0.0
        }
        
        # Try to read the log file for accurate metrics
        if log_path.exists():
            try:
                with open(log_path, 'r') as f:
                    lines = f.readlines()
                    if len(lines) > 1:
                        # Read last line to get final coverage
                        last_line = lines[-1].strip()
                        cols = last_line.split(',')
                        if len(cols) >= 2:
                            # Second column is coverage (as decimal 0-1)
                            coverage_decimal = float(cols[1])
                            metrics['coverage'] = coverage_decimal * 100  # Convert to percentage
                            metrics['completed'] = metrics['coverage'] >= 90.0
            except Exception as e:
                print(f"⚠️  Warning: Could not read log file: {e}")
        
        # Calculate exploration efficiency (coverage per second)
        if elapsed_time > 0:
            metrics['exploration_efficiency'] = metrics['coverage'] / elapsed_time
        
        print(f"✅ Completed in {elapsed_time:.1f}s")
        print(f"📊 Coverage: {metrics['coverage']:.1f}%")
        print(f"🚀 Efficiency: {metrics['exploration_efficiency']:.2f}% per second")
        
        return metrics
        
    except subprocess.TimeoutExpired:
        elapsed_time = time.time() - start_time
        print(f"⏱️  TIMEOUT after {elapsed_time:.1f}s")
        return {
            'description': description,
            'config': config_file,
            'elapsed_time': timeout,
            'completed': False,
            'coverage': 0.0,
            'exploration_efficiency': 0.0,
            'error': 'TIMEOUT'
        }
    except Exception as e:
        elapsed_time = time.time() - start_time
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return {
            'description': description,
            'config': config_file,
            'elapsed_time': elapsed_time,
            'completed': False,
            'coverage': 0.0,
            'exploration_efficiency': 0.0,
            'error': str(e)
        }

def create_test_configs():
    """
    Create clean test configurations for fair comparison.
    """
    
    # Baseline configuration (no REIP governance)
    baseline_config = {
        'name': 'baseline/exploration_test',
        'policy': 'baseline_leader_follower',
        'T': 400,  # Same time budget for both
        'N': 8,
        'map_size': 60,
        'lam': 3,
        'env': {
            'obstacle_density': 0.05,
            'seed': 42  # Same seed for identical environment
        },
        'reip': {
            'cyber_attack_rate': 0.0,  # No attacks for pure exploration
            'hallucination_rate': 0.0,
            'command_loss_rate': 0.0,
            'command_radius': 100
        }
    }
    
    # Enhanced REIP configuration (with governance)
    reip_config = {
        'name': 'enhanced_reip/exploration_test',
        'policy': 'reip_enhanced',
        'T': 400,  # Same time budget
        'N': 8,
        'map_size': 60,
        'lam': 3,
        'env': {
            'obstacle_density': 0.05,
            'seed': 42  # Same seed for identical environment
        },
        'reip': {
            'trust_decay_rate': 0.15,
            'trust_threshold': 0.70,
            'min_trust': 0.10,
            'trust_recovery_rate': 0.05,
            'loop_detection_window': 12,
            'loop_threshold': 0.75,
            'aimless_detection_window': 8,
            'stagnation_threshold': 0.02,
            'impeachment_cooldown': 25,
            'demotion_duration': 40,
            'cyber_attack_rate': 0.0,  # No attacks for pure exploration
            'hallucination_rate': 0.0,
            'command_loss_rate': 0.0,
            'command_radius': 8,
            'failed_command_threshold': 3,
            'leader_election_threshold': 3
        }
    }
    
    # Write configs to temp files
    baseline_file = 'configs/test_baseline_exploration.yaml'
    reip_file = 'configs/test_reip_exploration.yaml'
    
    with open(baseline_file, 'w') as f:
        yaml.dump(baseline_config, f, default_flow_style=False)
    
    with open(reip_file, 'w') as f:
        yaml.dump(reip_config, f, default_flow_style=False)
    
    return baseline_file, reip_file

def run_comparison():
    """
    Run the complete exploration comparison.
    """
    
    print("=" * 70)
    print("🔬 ENHANCED REIP vs BASELINE: EXPLORATION COMPARISON")
    print("=" * 70)
    
    print("\n📋 TEST SETUP:")
    print("   • Environment: 60x60 map with 5% obstacles")
    print("   • Agents: 8 explorers with 400 timesteps each")
    print("   • Fairness: Identical seed (42) for same environment")
    print("   • Conditions: No cyber attacks (pure exploration test)")
    print("   • Metric: Coverage percentage achieved")
    print("   • Efficiency: Coverage per second of computation")
    
    print("\n🔧 Creating test configurations...")
    baseline_file, reip_file = create_test_configs()
    
    print("✅ Configs created:")
    print(f"   • Baseline: {baseline_file}")
    print(f"   • Enhanced REIP: {reip_file}")
    
    # Run tests
    print("\n" + "=" * 70)
    print("🚀 RUNNING TESTS")
    print("=" * 70)
    
    print("\n📊 Test 1: Baseline System")
    print("-" * 70)
    baseline_results = run_simulation(
        baseline_file,
        "Baseline (Leader-Follower)",
        timeout=60  # Reduced timeout
    )
    
    print("\n📊 Test 2: Enhanced REIP System")
    print("-" * 70)
    reip_results = run_simulation(
        reip_file,
        "Enhanced REIP (with governance)",
        timeout=60  # Reduced timeout
    )
    
    # Compare results
    print("\n" + "=" * 70)
    print("📊 RESULTS COMPARISON")
    print("=" * 70)
    
    results_table = f"""
    Metric                  | Baseline           | Enhanced REIP      | Winner
    ------------------------|--------------------|--------------------|--------
    Coverage                | {baseline_results['coverage']:>6.1f}%          | {reip_results['coverage']:>6.1f}%          | {"REIP ✅" if reip_results['coverage'] > baseline_results['coverage'] else "Baseline ✅"}
    Time (seconds)          | {baseline_results['elapsed_time']:>8.1f}s        | {reip_results['elapsed_time']:>8.1f}s        | {"REIP ✅" if reip_results['elapsed_time'] < baseline_results['elapsed_time'] else "Baseline ✅"}
    Efficiency (% per sec)  | {baseline_results['exploration_efficiency']:>8.2f}         | {reip_results['exploration_efficiency']:>8.2f}         | {"REIP ✅" if reip_results['exploration_efficiency'] > baseline_results['exploration_efficiency'] else "Baseline ✅"}
    Completed Mission       | {"Yes ✅" if baseline_results['completed'] else "No ❌"}       | {"Yes ✅" if reip_results['completed'] else "No ❌"}       | {"REIP ✅" if reip_results['completed'] and not baseline_results['completed'] else ("Same" if baseline_results['completed'] == reip_results['completed'] else "Baseline ✅")}
    """
    
    print(results_table)
    
    # Analysis
    print("\n" + "=" * 70)
    print("🔍 DETAILED ANALYSIS")
    print("=" * 70)
    
    coverage_diff = reip_results['coverage'] - baseline_results['coverage']
    coverage_pct_change = (coverage_diff / baseline_results['coverage'] * 100) if baseline_results['coverage'] > 0 else 0
    
    time_diff = baseline_results['elapsed_time'] - reip_results['elapsed_time']
    time_pct_change = (time_diff / baseline_results['elapsed_time'] * 100) if baseline_results['elapsed_time'] > 0 else 0
    
    efficiency_ratio = reip_results['exploration_efficiency'] / baseline_results['exploration_efficiency'] if baseline_results['exploration_efficiency'] > 0.001 else 1.0
    
    print(f"\n📈 Coverage Analysis:")
    print(f"   Baseline: {baseline_results['coverage']:.1f}%")
    print(f"   Enhanced REIP: {reip_results['coverage']:.1f}%")
    print(f"   Difference: {coverage_diff:+.1f}% ({coverage_pct_change:+.1f}%)")
    
    if coverage_diff > 2:
        print(f"   🎯 REIP achieves SIGNIFICANTLY BETTER coverage (+{coverage_diff:.1f}%)")
    elif coverage_diff > 0:
        print(f"   ✅ REIP has slightly better coverage (+{coverage_diff:.1f}%)")
    elif coverage_diff < -2:
        print(f"   ⚠️  Baseline achieves SIGNIFICANTLY BETTER coverage ({coverage_diff:.1f}%)")
    else:
        print(f"   ≈ Coverage is essentially equivalent ({coverage_diff:+.1f}%)")
    
    print(f"\n⏱️  Time Analysis:")
    print(f"   Baseline: {baseline_results['elapsed_time']:.1f}s")
    print(f"   Enhanced REIP: {reip_results['elapsed_time']:.1f}s")
    print(f"   Difference: {time_diff:+.1f}s ({time_pct_change:+.1f}%)")
    
    if time_diff > 10:
        print(f"   🚀 Baseline is SIGNIFICANTLY FASTER (+{time_diff:.1f}s)")
    elif time_diff > 1:
        print(f"   ✅ Baseline is slightly faster (+{time_diff:.1f}s)")
    elif time_diff < -10:
        print(f"   🎯 REIP is SIGNIFICANTLY FASTER (+{time_diff:.1f}s)")
    else:
        print(f"   ≈ Time is essentially equivalent ({time_diff:+.1f}s)")
    
    print(f"\n🚀 Efficiency Analysis:")
    print(f"   Baseline: {baseline_results['exploration_efficiency']:.4f}% per second")
    print(f"   Enhanced REIP: {reip_results['exploration_efficiency']:.4f}% per second")
    
    if baseline_results['exploration_efficiency'] > 0.001:
        print(f"   Ratio: {efficiency_ratio:.2f}x")
        
        if efficiency_ratio > 1.1:
            print(f"   ✅ REIP is {efficiency_ratio:.1f}x MORE EFFICIENT")
        elif efficiency_ratio > 0.9:
            print(f"   ≈ Efficiency is essentially equivalent")
        else:
            print(f"   ⚠️  Baseline is {1/efficiency_ratio:.1f}x MORE EFFICIENT")
    else:
        print(f"   ⚠️  Cannot compute ratio (baseline efficiency too low)")
    
    # Overall assessment
    print("\n" + "=" * 70)
    print("🎯 OVERALL ASSESSMENT")
    print("=" * 70)
    
    print("\n💡 Key Findings:")
    
    if coverage_diff > 5:
        print(f"   1. ✅ REIP EXPLORES BETTER: Achieves +{coverage_diff:.1f}% better coverage")
    elif coverage_diff > 0:
        print(f"   1. ✅ REIP slightly better at exploration: +{coverage_diff:.1f}%")
    elif baseline_results['coverage'] > 0:
        print(f"   1. ⚠️  Baseline explores better: REIP is {abs(coverage_diff):.1f}% behind")
    else:
        print(f"   1. ℹ️  Both systems have limited exploration")
    
    if abs(time_diff) > 10:
        faster = "REIP" if time_diff > 0 else "Baseline"
        print(f"   2. {'✅' if time_diff > 0 else '⚠️'} {faster} is faster by {abs(time_diff):.1f}s")
    else:
        print(f"   2. ≈ Both systems take similar time ({abs(time_diff):.1f}s difference)")
    
    if reip_results['completed'] and not baseline_results['completed']:
        print(f"   3. ✅ REIP COMPLETES THE MISSION, Baseline does not")
    elif baseline_results['completed'] and not reip_results['completed']:
        print(f"   3. ⚠️  Baseline completes, REIP does not")
    else:
        print(f"   3. ≈ Both systems {'complete' if reip_results['completed'] else 'fail'} the mission")
    
    print("\n📋 Recommendation:")
    
    if coverage_diff > 5 and efficiency_ratio > 1.05:
        print("   ✅ ENHANCED REIP IS BETTER FOR EXPLORATION")
        print("      • Achieves superior coverage")
        print("      • More efficient exploration")
        print("      • Worth the governance overhead")
    elif coverage_diff > 0:
        print("   ✅ ENHANCED REIP HAS SLIGHT ADVANTAGE")
        print("      • Marginally better coverage")
        print("      • Consider if security needs justify overhead")
    elif abs(coverage_diff) < 5:
        print("   ≈ SYSTEMS ARE ROUGHLY EQUIVALENT")
        print("      • Choose based on other factors (security needs, time constraints)")
    else:
        print("   ⚠️  BASELINE IS BETTER FOR PURE EXPLORATION")
        print("      • Use baseline for speed-critical, low-security missions")
        print("      • Use REIP if you need attack resistance")
    
    print("\n" + "=" * 70)
    
    # Save results
    results_data = {
        'timestamp': datetime.now().isoformat(),
        'baseline': baseline_results,
        'reip': reip_results,
        'analysis': {
            'coverage_diff': coverage_diff,
            'coverage_pct_change': coverage_pct_change,
            'time_diff': time_diff,
            'time_pct_change': time_pct_change,
            'efficiency_ratio': efficiency_ratio
        }
    }
    
    results_file = f"results/exploration_comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    Path('results').mkdir(exist_ok=True)
    
    with open(results_file, 'w') as f:
        json.dump(results_data, f, indent=2)
    
    print(f"\n💾 Results saved to: {results_file}")

def main():
    """
    Main entry point.
    """
    try:
        run_comparison()
    except KeyboardInterrupt:
        print("\n❌ Interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
