"""Regenerate ablation figure from existing detailed CSV data."""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import csv
import numpy as np
from scripts.ablation_study import plot_ablation_results

# Load existing detailed CSV
detailed_csv = 'results/ablation/ablation_detailed_20251203_074900.csv'
timestamp = '20251203_074900_fixed'

with open(detailed_csv, 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

variants = set(r['variant'] for r in rows)

# Reconstruct stats
stats = {}
all_results = {}

for v in variants:
    variant_rows = [r for r in rows if r['variant'] == v]
    final_covs = [float(r['final_coverage']) for r in variant_rows]
    elections = [int(r['elections']) for r in variant_rows]
    impeachments = [int(r['impeachments']) for r in variant_rows]
    reached_95 = [r['reached_95'] == 'True' for r in variant_rows]
    times_to_95 = [float(r['time_to_95']) for r in variant_rows if r['time_to_95']]
    
    stats[v] = {
        'mean_coverage': np.mean(final_covs),
        'std_coverage': np.std(final_covs),
        'sem_coverage': np.std(final_covs) / np.sqrt(len(final_covs)),
        'median_coverage': np.median(final_covs),
        'mean_elections': np.mean(elections),
        'std_elections': np.std(elections),
        'mean_impeachments': np.mean(impeachments),
        'success_rate': np.mean(reached_95),
        'success_rate_sem': np.std(reached_95) / np.sqrt(len(reached_95)),
        'median_time_to_95': np.median(times_to_95) if times_to_95 else None,
        'mean_time_to_95': np.mean(times_to_95) if times_to_95 else None,
        'std_time_to_95': np.std(times_to_95) if times_to_95 else None,
        'n_runs': len(variant_rows),
        'n_reached_95': sum(reached_95),
    }
    
    all_results[v] = [{
        'run_id': int(r['run_id']),
        'final_coverage': float(r['final_coverage']),
        'elections': int(r['elections']),
        'impeachments': int(r['impeachments']),
        'reached_95': r['reached_95'] == 'True',
        'reached_90': r['reached_90'] == 'True',
        'time_to_95': float(r['time_to_95']) if r['time_to_95'] else None,
        'coverage_history': [],  # Not in detailed CSV
    } for r in variant_rows]

# Regenerate figure with fixed labels
plot_ablation_results(stats, all_results, 'results/ablation', timestamp)
print(f"\nFixed figure saved to: results/ablation/ablation_figure_{timestamp}.png")

