#!/usr/bin/env python3
"""
Quick Trust Dynamics Analyzer

Plots predicted_gain vs observed_gain and avg_trust over time from CSV logs
to diagnose trust decay behavior during hallucinations.

Usage:
    python scripts/analyze_trust_dynamics.py runs/comparison/reip_adversarial/log.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def analyze_trust_dynamics(csv_path):
    """Analyze and visualize trust dynamics from simulation log."""
    
    if not Path(csv_path).exists():
        print(f" CSV file not found: {csv_path}")
        return
    
    df = pd.read_csv(csv_path)
    
    # Create comprehensive visualization
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # Plot 1: Predicted vs Observed Gain
    ax1 = axes[0]
    ax1.plot(df['t'], df['predicted_gain'], label='Predicted Gain', color='blue', alpha=0.7, linewidth=1.5)
    ax1.plot(df['t'], df['observed_gain'], label='Observed Gain', color='green', alpha=0.7, linewidth=1.5)
    
    # Highlight hallucination periods
    if 'hallucinating' in df.columns:
        halluc_periods = df[df['hallucinating'] == 1]
        if not halluc_periods.empty:
            for idx, row in halluc_periods.iterrows():
                ax1.axvspan(row['t'], row['t']+0.5, color='red', alpha=0.1)
    
    ax1.set_xlabel('Timestep', fontsize=11)
    ax1.set_ylabel('Information Gain', fontsize=11)
    ax1.set_title('Predicted vs Observed Information Gain (Red background = hallucination)', fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Error (|predicted - observed|)
    ax2 = axes[1]
    df['error'] = abs(df['predicted_gain'] - df['observed_gain'])
    ax2.plot(df['t'], df['error'], label='Absolute Error', color='orange', linewidth=1.5)
    
    # Highlight hallucination periods
    if 'hallucinating' in df.columns:
        halluc_periods = df[df['hallucinating'] == 1]
        if not halluc_periods.empty:
            for idx, row in halluc_periods.iterrows():
                ax2.axvspan(row['t'], row['t']+0.5, color='red', alpha=0.1)
    
    ax2.set_xlabel('Timestep', fontsize=11)
    ax2.set_ylabel('|Predicted - Observed|', fontsize=11)
    ax2.set_title('Prediction Error Over Time', fontsize=12, fontweight='bold')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Average Trust
    ax3 = axes[2]
    ax3.plot(df['t'], df['avg_trust'], label='Average Trust', color='purple', linewidth=2)
    
    # Highlight hallucination periods
    if 'hallucinating' in df.columns:
        halluc_periods = df[df['hallucinating'] == 1]
        if not halluc_periods.empty:
            for idx, row in halluc_periods.iterrows():
                ax3.axvspan(row['t'], row['t']+0.5, color='red', alpha=0.1)
    
    # Mark impeachment threshold
    if 'elections' in df.columns:
        elections = df[df['elections'] > df['elections'].shift(1, fill_value=0)]
        if not elections.empty:
            for idx, row in elections.iterrows():
                ax3.axvline(row['t'], color='blue', linestyle='--', alpha=0.7, linewidth=1.5)
            ax3.plot([], [], color='blue', linestyle='--', label='Election/Impeachment')
    
    ax3.axhline(0.6, color='red', linestyle=':', alpha=0.5, label='Impeachment Threshold (0.6)')
    ax3.set_xlabel('Timestep', fontsize=11)
    ax3.set_ylabel('Average Trust in Leader', fontsize=11)
    ax3.set_title('Trust Decay and Elections', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 1.05)
    
    plt.tight_layout()
    
    # Print summary statistics
    print("\n" + "="*70)
    print("TRUST DYNAMICS ANALYSIS")
    print("="*70)
    
    if 'hallucinating' in df.columns:
        normal_periods = df[df['hallucinating'] == 0]
        halluc_periods = df[df['hallucinating'] == 1]
        
        if not normal_periods.empty:
            print(f"\n NORMAL EXPLORATION (n={len(normal_periods)} timesteps)")
            print(f"   Avg Predicted Gain:  {normal_periods['predicted_gain'].mean():.6f}")
            print(f"   Avg Observed Gain:   {normal_periods['observed_gain'].mean():.6f}")
            print(f"   Avg Error:           {normal_periods['error'].mean():.6f}")
            print(f"   Trust Range:         {normal_periods['avg_trust'].min():.3f} -> {normal_periods['avg_trust'].max():.3f}")
            print(f"   Trust Decay Rate:    {(normal_periods['avg_trust'].iloc[0] - normal_periods['avg_trust'].iloc[-1]) / len(normal_periods):.6f} per step")
        
        if not halluc_periods.empty:
            print(f"\n HALLUCINATION PERIODS (n={len(halluc_periods)} timesteps)")
            print(f"   Avg Predicted Gain:  {halluc_periods['predicted_gain'].mean():.6f}")
            print(f"   Avg Observed Gain:   {halluc_periods['observed_gain'].mean():.6f}")
            print(f"   Avg Error:           {halluc_periods['error'].mean():.6f}")
            print(f"   Trust Range:         {halluc_periods['avg_trust'].min():.3f} -> {halluc_periods['avg_trust'].max():.3f}")
            print(f"   Trust Decay Rate:    {(halluc_periods['avg_trust'].iloc[0] - halluc_periods['avg_trust'].iloc[-1]) / len(halluc_periods):.6f} per step")
            
            if 'hallucination_type' in df.columns:
                h_type = halluc_periods['hallucination_type'].iloc[0]
                print(f"   Hallucination Type:  {h_type}")
        
        # Comparison
        if not normal_periods.empty and not halluc_periods.empty:
            error_ratio = halluc_periods['error'].mean() / max(normal_periods['error'].mean(), 1e-9)
            print(f"\n  COMPARISON")
            print(f"   Error during hallucination is {error_ratio:.2f}x normal exploration error")
            
            if error_ratio < 1.5:
                print(f"     WARNING: Hallucination error is not significantly higher!")
                print(f"   Consider: increasing pred_gain_inflation_base, using strict mode, or disabling fallback")
    
    if 'elections' in df.columns:
        final_elections = df['elections'].iloc[-1]
        print(f"\n  GOVERNANCE")
        print(f"   Total Elections:     {final_elections}")
        if final_elections > 0:
            print(f"    Democratic fault recovery demonstrated")
        else:
            print(f"     No elections occurred (trust may not have dropped below threshold)")
    
    print(f"\n FINAL METRICS")
    print(f"   Final Coverage:      {df['coverage'].iloc[-1]*100:.2f}%")
    print(f"   Final Trust:         {df['avg_trust'].iloc[-1]:.3f}")
    
    print("\n" + "="*70)
    
    # Save figure
    output_path = Path(csv_path).parent / "trust_dynamics_analysis.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n Saved visualization to: {output_path}")
    
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python scripts/analyze_trust_dynamics.py <path_to_log.csv>")
        print("Example: python scripts/analyze_trust_dynamics.py runs/comparison/reip_adversarial/log.csv")
        sys.exit(1)
    
    csv_path = sys.argv[1]
    analyze_trust_dynamics(csv_path)
