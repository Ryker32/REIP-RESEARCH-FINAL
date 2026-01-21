"""
Enhanced visualization runner that shows simulation statistics in real-time.
Run both REIP and Baseline with side-by-side comparison data.
"""
import sys
import os
import argparse

# Available configurations
CONFIGS = {
    'baseline': 'configs/baseline_viz.yaml',
    'reip': 'configs/reip_viz.yaml',
}

def main():
    parser = argparse.ArgumentParser(description='Run simulation with live visualization')
    parser.add_argument('system', choices=['baseline', 'reip'], 
                       help='Which system to visualize')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')
    parser.add_argument('--steps', type=int, default=150,
                       help='Number of timesteps (default: 150)')
    parser.add_argument('--fast', action='store_true',
                       help='Faster visualization (less pause between frames)')
    
    args = parser.parse_args()
    
    config_file = CONFIGS[args.system]
    
    # Update config with user parameters
    import yaml
    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)
    
    cfg['env']['seed'] = args.seed
    cfg['T'] = args.steps
    
    # Save temporary config
    temp_config = f'temp_{args.system}_viz.yaml'
    with open(temp_config, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False)
    
    # Print info
    print("=" * 80)
    print(f"LIVE VISUALIZATION: {args.system.upper()}")
    print("=" * 80)
    print(f"Configuration: {config_file}")
    print(f"Random Seed: {args.seed}")
    print(f"Timesteps: {args.steps}")
    print(f"Map Size: {cfg['map_size']}x{cfg['map_size']}")
    print(f"Agents: {cfg['N']}")
    print(f"Obstacles: {cfg['obstacle_density']:.1%}")
    
    if args.system == 'reip':
        reip_cfg = cfg.get('reip', {})
        print("\nREIP Governance Parameters:")
        print(f"  Trust Threshold: {reip_cfg.get('trust_threshold', 0.6)}")
        print(f"  Trust Decay Rate: {reip_cfg.get('trust_decay_rate', 0.5)}")
        print(f"  Hallucination Rate: {reip_cfg.get('hallucination_rate', 0.0):.1%}")
        print(f"  Command Loss Rate: {reip_cfg.get('command_loss_rate', 0.0):.1%}")
    else:
        print("\nBaseline: Fixed Leader (Agent 0)")
        print(f"  Hallucination Rate: {cfg.get('hallucination_rate', 0.0):.1%}")
        print(f"  Command Loss Rate: {cfg.get('command_loss_rate', 0.0):.1%}")
    
    print("\n" + "=" * 80)
    print("Starting simulation... (Close plot window when done)")
    print("=" * 80 + "\n")
    
    # Run simulation
    import subprocess
    cmd = [sys.executable, '-m', 'src.main', '--config', temp_config, '--visualize']
    result = subprocess.run(cmd)
    
    # Cleanup
    if os.path.exists(temp_config):
        os.remove(temp_config)
    
    return result.returncode

if __name__ == '__main__':
    sys.exit(main())
