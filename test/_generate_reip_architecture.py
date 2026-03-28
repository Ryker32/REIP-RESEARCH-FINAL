#!/usr/bin/env python3
"""
Generate REIP Detailed Architecture Diagram (Similar to VARnet Figure 8)
Shows internal operations, data structures, and flow
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle
import numpy as np

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 9,
    'axes.linewidth': 0.5,
    'figure.dpi': 300,
})

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111)
ax.set_xlim(0, 14)
ax.set_ylim(0, 10)
ax.axis('off')

# Colors
color_input = '#E0E0E0'
color_process = '#FFE0E0'
color_trust = '#FFCCCC'
color_output = '#E0FFE0'
color_arrow = '#333333'

# ========== PHASE 1: INPUT PROCESSING ==========
ax.text(7, 9.5, 'Phase 1: Input Processing & Coverage Merging', 
        ha='center', va='center', fontsize=12, weight='bold')

# Input boxes
inputs = [
    ('Position\nServer', 1.5, 8.5, 'x, y, theta\nUDP 5100'),
    ('Peer\nBroadcasts', 4.5, 8.5, 'State messages\nUDP 5200'),
    ('ToF\nSensors', 7.5, 8.5, '5x VL53L0X\nDistances'),
]

for label, x, y, desc in inputs:
    box = FancyBboxPatch((x-0.7, y-0.5), 1.4, 1.0,
                         boxstyle='round,pad=0.1', facecolor=color_input,
                         edgecolor='black', linewidth=1.5)
    ax.add_patch(box)
    ax.text(x, y+0.2, label, ha='center', va='center', fontsize=9, weight='bold')
    ax.text(x, y-0.2, desc, ha='center', va='center', fontsize=7)

# Processing functions
procs = [
    ('receive_\nposition()', 1.5, 7.0),
    ('receive_peer_\nstates()', 4.5, 7.0),
    ('update_tof_\nobstacles()', 7.5, 7.0),
]

for label, x, y in procs:
    box = FancyBboxPatch((x-0.6, y-0.3), 1.2, 0.6,
                         boxstyle='round,pad=0.1', facecolor=color_process,
                         edgecolor='black', linewidth=1.2)
    ax.add_patch(box)
    ax.text(x, y, label, ha='center', va='center', fontsize=8)

# State updates
states = [
    ('my_visited\nDict[cell, time]', 1.5, 6.0),
    ('known_visited\nSet[cell]', 4.5, 6.0),
    ('tof_obstacles\nSet[cell]', 7.5, 6.0),
]

for label, x, y in states:
    box = FancyBboxPatch((x-0.6, y-0.3), 1.2, 0.6,
                         boxstyle='round,pad=0.1', facecolor='#FFFFE0',
                         edgecolor='black', linewidth=1.0)
    ax.add_patch(box)
    ax.text(x, y, label, ha='center', va='center', fontsize=7)

# Arrows Phase 1
for x in [1.5, 4.5, 7.5]:
    arrow = FancyArrowPatch((x, 8.0), (x, 7.3),
                           arrowstyle='->', lw=1.5, color=color_arrow)
    ax.add_patch(arrow)
    arrow = FancyArrowPatch((x, 6.7), (x, 6.3),
                           arrowstyle='->', lw=1.5, color=color_arrow)
    ax.add_patch(arrow)

# ========== PHASE 2: TRUST ASSESSMENT ==========
ax.text(7, 5.2, 'Phase 2: Trust Assessment (assess_leader_command)', 
        ha='center', va='center', fontsize=12, weight='bold')

# Leader assignment input
box = FancyBboxPatch((10.5, 5.5), 2.5, 0.8,
                     boxstyle='round,pad=0.1', facecolor=color_input,
                     edgecolor='black', linewidth=1.5)
ax.add_patch(box)
ax.text(11.75, 5.9, 'Leader Assignment', ha='center', va='center', fontsize=9, weight='bold')
ax.text(11.75, 5.5, 'commanded_target: (x, y)', ha='center', va='center', fontsize=7)

# Three-tier checks
tiers = [
    ('Tier 1 Check\ncell in\nmy_visited?\nvisited_time < cutoff?', 2.0, 4.5, 1.0),
    ('Tier 2 Check\ndist <= 200mm AND\ncell in\ntof_obstacles?', 5.0, 4.5, 1.0),
    ('Tier 3 Check\ncell in\nknown_visited?\nknown_time < cutoff?', 8.0, 4.5, 0.3),
]

for label, x, y, weight in tiers:
    box = FancyBboxPatch((x-0.8, y-0.5), 1.6, 1.0,
                         boxstyle='round,pad=0.1', facecolor=color_trust,
                         edgecolor='black', linewidth=1.5)
    ax.add_patch(box)
    ax.text(x, y+0.2, label, ha='center', va='center', fontsize=7)
    ax.text(x, y-0.3, f'Weight: {weight}', ha='center', va='center', 
            fontsize=8, weight='bold', bbox=dict(boxstyle='round,pad=0.2', 
            facecolor='white', alpha=0.9))

# MPC direction check
mpc_box = FancyBboxPatch((11.0, 4.0), 2.0, 0.8,
                        boxstyle='round,pad=0.1', facecolor='#FFE0FF',
                        edgecolor='black', linewidth=1.5)
ax.add_patch(mpc_box)
ax.text(12.0, 4.4, 'MPC Direction Check', ha='center', va='center', fontsize=8, weight='bold')
ax.text(12.0, 4.0, 'cmd_dir vs. frontier_dir', ha='center', va='center', fontsize=7)

# Suspicion accumulation
sus_box = FancyBboxPatch((5.5, 3.0), 3.0, 0.8,
                        boxstyle='round,pad=0.1', facecolor='#CCCCFF',
                        edgecolor='black', linewidth=1.5)
ax.add_patch(sus_box)
ax.text(7.0, 3.4, 'Suspicion Accumulation', ha='center', va='center', fontsize=9, weight='bold')
ax.text(7.0, 3.0, 'suspicion += weight OR suspicion -= recovery_rate', 
        ha='center', va='center', fontsize=7)

# Trust decay
trust_box = FancyBboxPatch((5.5, 2.0), 3.0, 0.8,
                          boxstyle='round,pad=0.1', facecolor=color_output,
                          edgecolor='black', linewidth=1.5)
ax.add_patch(trust_box)
ax.text(7.0, 2.4, 'Trust Decay (if suspicion >= 1.5)', ha='center', va='center', 
        fontsize=9, weight='bold')
ax.text(7.0, 2.0, 'trust -= 0.2, suspicion -= 1.5 (carry-over)', 
        ha='center', va='center', fontsize=7)

# Arrows Phase 2
# From states to tiers
arrow = FancyArrowPatch((1.5, 5.7), (2.0, 5.0),
                       arrowstyle='->', lw=1.5, color=color_arrow)
ax.add_patch(arrow)
arrow = FancyArrowPatch((7.5, 5.7), (5.0, 5.0),
                       arrowstyle='->', lw=1.5, color=color_arrow)
ax.add_patch(arrow)
arrow = FancyArrowPatch((4.5, 5.7), (8.0, 5.0),
                       arrowstyle='->', lw=1.5, color=color_arrow)
ax.add_patch(arrow)

# From assignment to MPC
arrow = FancyArrowPatch((11.75, 5.5), (12.0, 4.8),
                       arrowstyle='->', lw=1.5, color=color_arrow)
ax.add_patch(arrow)

# From tiers to suspicion
for x in [2.0, 5.0, 8.0, 12.0]:
    if x <= 8.0:
        arrow = FancyArrowPatch((x, 4.0), (6.5, 3.4),
                               arrowstyle='->', lw=1.5, color=color_arrow)
        ax.add_patch(arrow)
    else:
        arrow = FancyArrowPatch((x, 3.8), (7.5, 3.4),
                               arrowstyle='->', lw=1.5, color=color_arrow)
        ax.add_patch(arrow)

# Suspicion to trust
arrow = FancyArrowPatch((7.0, 3.0), (7.0, 2.8),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)

# ========== PHASE 3: ELECTION & ASSIGNMENT ==========
ax.text(7, 1.5, 'Phase 3: Election & Task Assignment', 
        ha='center', va='center', fontsize=12, weight='bold')

# Election
elect_box = FancyBboxPatch((1.0, 0.3), 2.5, 1.0,
                           boxstyle='round,pad=0.1', facecolor='#CCFFCC',
                           edgecolor='black', linewidth=1.5)
ax.add_patch(elect_box)
ax.text(2.25, 0.9, 'Election Process', ha='center', va='center', fontsize=9, weight='bold')
ax.text(2.25, 0.5, 'run_election()\nTrust-weighted voting', ha='center', va='center', fontsize=7)

# Assignment
assign_box = FancyBboxPatch((4.5, 0.3), 2.5, 1.0,
                            boxstyle='round,pad=0.1', facecolor='#FFFFCC',
                            edgecolor='black', linewidth=1.5)
ax.add_patch(assign_box)
ax.text(5.75, 0.9, 'Task Assignment', ha='center', va='center', fontsize=9, weight='bold')
ax.text(5.75, 0.5, 'compute_task_assignments()\nGreedy nearest-frontier', 
        ha='center', va='center', fontsize=7)

# Navigation
nav_box = FancyBboxPatch((8.0, 0.3), 2.5, 1.0,
                         boxstyle='round,pad=0.1', facecolor='#CCE0FF',
                         edgecolor='black', linewidth=1.5)
ax.add_patch(nav_box)
ax.text(9.25, 0.9, 'Navigation', ha='center', va='center', fontsize=9, weight='bold')
ax.text(9.25, 0.5, 'compute_motor_command()\nA* pathfinding', 
        ha='center', va='center', fontsize=7)

# Output
out_box = FancyBboxPatch((11.5, 0.3), 2.0, 1.0,
                        boxstyle='round,pad=0.1', facecolor=color_output,
                        edgecolor='black', linewidth=1.5)
ax.add_patch(out_box)
ax.text(12.5, 0.9, 'Motor Output', ha='center', va='center', fontsize=9, weight='bold')
ax.text(12.5, 0.5, 'left, right PWM\nUART -> Pico', ha='center', va='center', fontsize=7)

# Arrows Phase 3
arrow = FancyArrowPatch((7.0, 2.0), (2.25, 1.3),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)
arrow = FancyArrowPatch((2.25, 0.3), (5.75, 0.3),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)
arrow = FancyArrowPatch((5.75, 0.3), (9.25, 0.3),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)
arrow = FancyArrowPatch((9.25, 0.3), (12.5, 0.3),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)

# Title
ax.text(7, 9.8, 'REIP Node Internal Architecture', ha='center', va='center',
        fontsize=16, weight='bold')

plt.tight_layout()
plt.savefig('paper_docs/Ryker_Kollmyer___UPDATED_PAPER/reip_architecture.png',
            dpi=300, bbox_inches='tight', facecolor='white')
print("Saved: paper_docs/Ryker_Kollmyer___UPDATED_PAPER/reip_architecture.png")
