"""
Compare Enhanced REIP vs Baseline WITH Cyber Attacks

This shows when Enhanced REIP provides actual value - under attack conditions.
"""

import subprocess
import json
import yaml
import time
import sys
from pathlib import Path
from datetime import datetime

def run_simulation(config_file, description, timeout=120):
    """Run a simulation and extract metrics from log file."""
    print(f"\n🔄 Running: {description}")
    print(f"📄 Config: {config_file}")
    
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
            'impeachments': 0,
            'loop_detections': 0
        }
        
        # Read log file for accurate metrics
        if log_path.exists():
            try:
                with open(log_path, 'r') as f:
                    lines = f.readlines()
                    if len(lines) > 1:
                        # Last line has final metrics
                        last_line = lines[-1].strip()
                        cols = last_line.split(',')
                        if len(cols) >= 5:
                            metrics['coverage'] = float(cols[1]) * 100  # Convert to percentage
                            metrics['impeachments'] = int(cols[4])
                            metrics['loop_detections'] = int(cols[5])
                            metrics['completed'] = metrics['coverage'] >= 90.0
            except Exception as e:
                print(f"⚠️  Warning: Could not read log: {e}")
        
        print(f"✅ Completed in {elapsed_time:.1f}s")
        print(f"📊 Coverage: {metrics['coverage']:.1f}%")
        print(f"🛡️  Impeachments: {metrics['impeachments']}")
        print(f"🔍 Loop Detections: {metrics['loop_detections']}")
        
        return metrics
        
    except subprocess.TimeoutExpired:
        print(f"⏱️  TIMEOUT after {timeout}s")
        return {
            'description': description,
            'elapsed_time': timeout,
            'completed': False,
            'coverage': 0.0,
            'error': 'TIMEOUT'
        }
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return {
            'description': description,
            'elapsed_time': 0,
            'completed': False,
            'coverage': 0.0,
            'error': str(e)
        }

def create_attack_configs():
    """Create configs for cyber attack comparison."""
    
    # Baseline under attack (no defense)
    baseline_attack = {
        'name': 'baseline/under_attack',
        'policy': 'baseline_leader_follower',
        'T': 400,
        'N': 8,
        'map_size': 60,
        'lam': 3,
        'env': {'obstacle_density': 0.05, 'seed': 42},
        'reip': {
            'cyber_attack_rate': 0.08,  # 8% attack rate
            'hallucination_rate': 0.0,
            'command_loss_rate': 0.0,
            'command_radius': 100
        }
    }
    
    # Enhanced REIP under attack (with defense) - LESS SENSITIVE
    reip_attack = {
        'name': 'enhanced_reip/under_attack',
        'policy': 'reip_enhanced',
        'T': 400,
        'N': 8,
        'map_size': 60,
        'lam': 3,
        'env': {'obstacle_density': 0.05, 'seed': 42},
        'reip': {
            # LESS SENSITIVE SETTINGS to reduce false positives
            'trust_decay_rate': 0.10,        # Slower trust decay (was 0.15)
            'trust_threshold': 0.50,         # Lower threshold (was 0.70)
            'min_trust': 0.10,
            'trust_recovery_rate': 0.05,
            
            # Much more tolerant loop detection
            'loop_detection_window': 20,     # Longer window (was 10)
            'loop_threshold': 0.90,          # Much higher threshold (was 0.70)
            'aimless_detection_window': 15,  # Longer window (was 8)
            'stagnation_threshold': 0.005,   # More tolerant (was 0.01)
            
            # Slower governance
            'impeachment_cooldown': 40,      # Longer cooldown (was 15)
            'demotion_duration': 60,         # Longer demotion (was 50)
            
            # Attack parameters
            'cyber_attack_rate': 0.08,       # Same 8% attack rate
            'hallucination_rate': 0.0,
            'command_loss_rate': 0.0,
            'command_radius': 8,
            'failed_command_threshold': 3,
            'leader_election_threshold': 3
        }
    }
    
    # Write configs
    baseline_file = 'configs/baseline_cyber_attack.yaml'
    reip_file = 'configs/reip_cyber_attack.yaml'
    
    with open(baseline_file, 'w') as f:
        yaml.dump(baseline_attack, f, default_flow_style=False)
    
    with open(reip_file, 'w') as f:
        yaml.dump(reip_attack, f, default_flow_style=False)
    
    return baseline_file, reip_file

def main():
    print("=" * 70)
    print("🔬 ENHANCED REIP vs BASELINE: CYBER ATTACK COMPARISON")
    print("=" * 70)
    
    print("\n📋 TEST SETUP:")
    print("   • Environment: 60x60 map with 5% obstacles")
    print("   • Agents: 8 explorers with 400 timesteps")
    print("   • Seed: 42 (identical environment)")
    print("   • Attack Rate: 8% (cyber attacks on leader)")
    print("   • Attack Effect: Compromised leader sends agents to explored areas")
    print("   • REIP Tuning: Less sensitive to reduce false positives")
    
    print("\n🔧 Creating attack configurations...")
    baseline_file, reip_file = create_attack_configs()
    print(f"✅ Configs created")
    
    print("\n" + "=" * 70)
    print("🚀 RUNNING TESTS")
    print("=" * 70)
    
    print("\n📊 Test 1: Baseline Under Attack (No Defense)")
    print("-" * 70)
    baseline_results = run_simulation(baseline_file, "Baseline (No Defense)", timeout=120)
    
    print("\n📊 Test 2: Enhanced REIP Under Attack (With Defense)")
    print("-" * 70)
    reip_results = run_simulation(reip_file, "Enhanced REIP (With Defense)", timeout=120)
    
    # Compare results
    print("\n" + "=" * 70)
    print("📊 RESULTS COMPARISON")
    print("=" * 70)
    
    coverage_diff = reip_results['coverage'] - baseline_results['coverage']
    
    results_table = f"""
    Metric                  | Baseline (No Defense) | Enhanced REIP (Defense) | Winner
    ------------------------|-----------------------|-------------------------|--------
    Coverage                | {baseline_results['coverage']:>6.1f}%             | {reip_results['coverage']:>6.1f}%              | {"REIP ✅" if coverage_diff > 5 else ("Same" if abs(coverage_diff) < 5 else "Baseline ✅")}
    Completion Time (s)     | {baseline_results['elapsed_time']:>8.1f}s            | {reip_results['elapsed_time']:>8.1f}s             | {"REIP ✅" if reip_results['elapsed_time'] < baseline_results['elapsed_time'] else "Baseline ✅"}
    Impeachments            | {baseline_results.get('impeachments', 0):>8d}              | {reip_results.get('impeachments', 0):>8d}               | {"REIP defense" if reip_results.get('impeachments', 0) > 0 else "N/A"}
    Loop Detections         | {baseline_results.get('loop_detections', 0):>8d}              | {reip_results.get('loop_detections', 0):>8d}               | {"REIP detecting" if reip_results.get('loop_detections', 0) > 0 else "N/A"}
    Mission Success         | {"Yes ✅" if baseline_results['completed'] else "No ❌"}     | {"Yes ✅" if reip_results['completed'] else "No ❌"}      | {"REIP ✅" if reip_results['completed'] and not baseline_results['completed'] else ("Same" if baseline_results['completed'] == reip_results['completed'] else "Baseline ✅")}
    """
    
    print(results_table)
    
    print("\n" + "=" * 70)
    print("🔍 DETAILED ANALYSIS")
    print("=" * 70)
    
    print(f"\n📈 Coverage Comparison:")
    print(f"   Baseline (undefended): {baseline_results['coverage']:.1f}%")
    print(f"   Enhanced REIP (defended): {reip_results['coverage']:.1f}%")
    print(f"   Difference: {coverage_diff:+.1f}%")
    
    if coverage_diff > 10:
        print(f"   ✅ REIP SIGNIFICANTLY OUTPERFORMS under attacks (+{coverage_diff:.1f}%)")
        print(f"      Defense mechanisms prevent attack-induced exploration failures")
    elif coverage_diff > 5:
        print(f"   ✅ REIP has better attack resilience (+{coverage_diff:.1f}%)")
    elif abs(coverage_diff) < 5:
        print(f"   ≈ Both systems achieve similar coverage despite attacks")
    else:
        print(f"   ⚠️  Baseline performs better even under attacks ({coverage_diff:.1f}%)")
        print(f"      REIP may still be too sensitive or attacks not severe enough")
    
    print(f"\n🛡️  Defense Mechanism Analysis:")
    reip_impeachments = reip_results.get('impeachments', 0)
    reip_loop_detections = reip_results.get('loop_detections', 0)
    
    print(f"   Enhanced REIP detected and responded to:")
    print(f"      • {reip_loop_detections} suspicious loop patterns")
    print(f"      • {reip_impeachments} leader impeachments")
    
    if reip_impeachments > 0:
        print(f"   ✅ Governance system ACTIVE - removing compromised leaders")
    else:
        print(f"   ⚠️  No impeachments - governance may not be triggering")
    
    if reip_loop_detections > 100:
        print(f"   ⚠️  WARNING: {reip_loop_detections} loop detections may indicate false positives!")
        print(f"   💡 Consider further tuning: increase loop_threshold or loop_detection_window")
    elif reip_loop_detections > 0:
        print(f"   ✅ Loop detection functioning within reasonable bounds")
    
    print("\n" + "=" * 70)
    print("🎯 VERDICT")
    print("=" * 70)
    
    if coverage_diff > 10:
        verdict = """
✅ ENHANCED REIP IS SUPERIOR UNDER CYBER ATTACKS
   
   The governance mechanisms successfully detect and remove compromised
   leaders, maintaining exploration effectiveness despite attacks.
   
   Recommendation: Use Enhanced REIP for security-critical missions
   where adversarial agents or cyber attacks are expected.
"""
    elif coverage_diff > 5:
        verdict = """
✅ ENHANCED REIP PROVIDES MEASURABLE DEFENSE VALUE
   
   The governance overhead is justified by improved attack resilience.
   
   Recommendation: Use Enhanced REIP when attack risk is moderate to high
   and mission success is more important than speed.
"""
    elif abs(coverage_diff) < 5:
        verdict = """
≈ SYSTEMS ARE COMPARABLE UNDER THESE ATTACK CONDITIONS
   
   Either: attacks aren't severe enough to show REIP's advantage,
   or REIP needs further tuning to balance defense vs performance.
   
   Recommendation: Test with higher attack rates (10-15%) or tune REIP
   sensitivity to reduce false positives while maintaining defense.
"""
    else:
        verdict = """
⚠️  BASELINE STILL OUTPERFORMS
   
   REIP's governance overhead outweighs its defense benefits at this
   attack rate, possibly due to false positives or over-sensitivity.
   
   Recommendation: Either reduce REIP sensitivity further, or increase
   attack rate to show where REIP becomes beneficial.
"""
    
    print(verdict)
    
    # Save results
    results_data = {
        'timestamp': datetime.now().isoformat(),
        'attack_rate': 0.08,
        'baseline': baseline_results,
        'reip': reip_results,
        'coverage_difference': coverage_diff
    }
    
    results_file = f"results/cyber_attack_comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    Path('results').mkdir(exist_ok=True)
    
    with open(results_file, 'w') as f:
        json.dump(results_data, f, indent=2)
    
    print(f"\n💾 Results saved to: {results_file}")
    
    # Discussion: Game-based scoring system
    print("\n" + "=" * 70)
    print("💭 RESEARCH INSIGHT: GAME-BASED EVALUATION")
    print("=" * 70)
    
    game_discussion = """
Your idea about using a GAME-BASED SCORING SYSTEM is excellent! Here's why:

🎯 WHY PURE EXPLORATION ISN'T ENOUGH:
   • No penalties for wasted effort (revisiting explored areas)
   • No cost for failed decisions or compromised leaders
   • Doesn't capture real-world mission constraints

🎮 GAME-BASED SCORING ADVANTAGES:
   
   1. POINT DEDUCTION SYSTEM:
      • -X points for each cell revisited (wasted effort)
      • -Y points for each attack that succeeds
      • -Z points for leadership failures
      • +W points for each new cell explored
   
   2. RESOURCE CONSTRAINTS:
      • Limited fuel/energy budget
      • Communication bandwidth costs
      • Time penalties for delays
   
   3. MISSION OBJECTIVES:
      • Primary: Explore target percentage (e.g., 95%)
      • Secondary: Minimize wasted movement
      • Tertiary: Maintain team cohesion
   
   4. ADVERSARIAL SCORING:
      • Attacker gets points for successful compromises
      • Defender (REIP) gets points for detecting attacks
      • Net score = exploration_gain - attack_damage - governance_overhead

📊 EXAMPLE SCORING FORMULA:
   
   Total Score = (New Cells × 10)           # Reward exploration
               - (Revisited Cells × 2)      # Penalize waste
               - (Attack Successes × 50)    # Heavy attack penalty
               - (Governance Actions × 1)    # Light overhead penalty
               + (Attacks Detected × 25)    # Reward defense
               - (Mission Time × 0.1)       # Time pressure

This would show:
   ✅ Baseline wins when attacks are rare (no defense overhead)
   ✅ Enhanced REIP wins when attacks are frequent (defense value)
   ✅ Break-even point becomes clear (when to switch systems)

💡 RECOMMENDATION:
   Implement this game-based evaluation for your research! It provides:
   • More realistic mission scenarios
   • Clear cost-benefit analysis of governance
   • Publishable metrics for comparing systems
   • Demonstrates practical deployment guidelines
"""
    
    print(game_discussion)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n❌ Interrupted")
        sys.exit(1)
