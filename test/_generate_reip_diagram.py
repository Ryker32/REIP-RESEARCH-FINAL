#!/usr/bin/env python3
"""
Generate REIP System Architecture Diagram for Paper
Similar style to VARnet figures - clean, academic, professional
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, ConnectionPatch
import numpy as np

# IEEE style
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 10,
    'axes.linewidth': 0.5,
    'figure.dpi': 300,
})

fig, ax = plt.subplots(figsize=(12, 8))
ax.set_xlim(0, 10)
ax.set_ylim(0, 8)
ax.axis('off')

# Colors (academic style - muted)
color_input = '#E8E8FF'
color_process = '#FFE8E8'
color_trust = '#FFCCCC'
color_election = '#CCFFCC'
color_output = '#F0F0F0'
color_arrow = '#333333'

# Box style
box_style = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='black', linewidth=1.5)

# ========== INPUT LAYER ==========
y_input = 7.5
inputs = [
    ('Position Server\nUDP 5100\nx, y, theta', 1.5, color_input),
    ('Peer Broadcasts\nUDP 5200\nState messages', 3.5, color_input),
    ('ToF Sensors\n5x VL53L0X\nDistance readings', 5.5, color_input),
    ('Fault Injector\nUDP 5300\nFault modes', 7.5, color_input),
]

for text, x, color in inputs:
    box = FancyBboxPatch((x-0.6, y_input-0.4), 1.2, 0.8,
                         boxstyle='round,pad=0.1', facecolor=color,
                         edgecolor='black', linewidth=1.2)
    ax.add_patch(box)
    ax.text(x, y_input, text, ha='center', va='center', fontsize=8, weight='bold')

# ========== PROCESSING LAYER 1: Coverage Merging ==========
y_proc1 = 6.0
proc1 = [
    ('Position\nProcessing', 1.5, color_process),
    ('Peer State\nUpdate', 3.5, color_process),
    ('Obstacle\nDetection', 5.5, color_process),
]

for text, x, color in proc1:
    box = FancyBboxPatch((x-0.5, y_proc1-0.3), 1.0, 0.6,
                         boxstyle='round,pad=0.1', facecolor=color,
                         edgecolor='black', linewidth=1.2)
    ax.add_patch(box)
    ax.text(x, y_proc1, text, ha='center', va='center', fontsize=8)

# ========== PROCESSING LAYER 2: Trust Assessment ==========
y_proc2 = 4.5
trust_box = FancyBboxPatch((2.5, y_proc2-0.6), 5.0, 1.2,
                          boxstyle='round,pad=0.2', facecolor=color_trust,
                          edgecolor='black', linewidth=2)
ax.add_patch(trust_box)
ax.text(5.0, y_proc2+0.3, 'TRUST ASSESSMENT', ha='center', va='center',
        fontsize=10, weight='bold')

# Three tiers
tiers = [
    ('Tier 1:\nPersonal\nWeight 1.0', 3.5, y_proc2-0.1),
    ('Tier 2:\nToF\nWeight 1.0', 5.0, y_proc2-0.1),
    ('Tier 3:\nPeer\nWeight 0.3', 6.5, y_proc2-0.1),
]

for text, x, y in tiers:
    ax.text(x, y, text, ha='center', va='center', fontsize=7,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

ax.text(5.0, y_proc2-0.4, 'MPC Direction Check: Command vs. Frontier', 
        ha='center', va='center', fontsize=7, style='italic')

# ========== PROCESSING LAYER 3: Leadership ==========
y_proc3 = 3.0
leadership_box = FancyBboxPatch((2.5, y_proc3-0.5), 5.0, 1.0,
                                boxstyle='round,pad=0.2', facecolor=color_election,
                                edgecolor='black', linewidth=2)
ax.add_patch(leadership_box)
ax.text(5.0, y_proc3+0.2, 'LEADERSHIP MANAGEMENT', ha='center', va='center',
        fontsize=10, weight='bold')

ax.text(3.5, y_proc3-0.1, 'Impeachment Check\nTrust < 0.3', ha='center', va='center',
        fontsize=7, bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
ax.text(6.5, y_proc3-0.1, 'Election Process\nTrust-weighted voting', ha='center', va='center',
        fontsize=7, bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

# ========== PROCESSING LAYER 4: Task Assignment ==========
y_proc4 = 1.5
assign_box = FancyBboxPatch((2.0, y_proc4-0.4), 6.0, 0.8,
                            boxstyle='round,pad=0.2', facecolor='#FFF8E8',
                            edgecolor='black', linewidth=1.5)
ax.add_patch(assign_box)
ax.text(5.0, y_proc4+0.2, 'TASK ASSIGNMENT (Leader Only)', ha='center', va='center',
        fontsize=9, weight='bold')

ax.text(3.5, y_proc4-0.1, 'Frontier Detection', ha='center', va='center', fontsize=7)
ax.text(5.0, y_proc4-0.1, 'Greedy Assignment', ha='center', va='center', fontsize=7)
ax.text(6.5, y_proc4-0.1, 'Spatial Diversity', ha='center', va='center', fontsize=7)

# ========== OUTPUT LAYER ==========
y_output = 0.3
outputs = [
    ('Motor Commands\nUART -> Pico\nPWM values', 2.5, color_output),
    ('State Broadcast\nUDP 5200\n5 Hz', 5.0, color_output),
    ('Logging\nJSON file\nVisualization', 7.5, color_output),
]

for text, x, color in outputs:
    box = FancyBboxPatch((x-0.6, y_output-0.3), 1.2, 0.6,
                         boxstyle='round,pad=0.1', facecolor=color,
                         edgecolor='black', linewidth=1.2)
    ax.add_patch(box)
    ax.text(x, y_output, text, ha='center', va='center', fontsize=8, weight='bold')

# ========== ARROWS ==========
# Input to Process 1
for x in [1.5, 3.5, 5.5, 7.5]:
    if x <= 5.5:
        arrow = FancyArrowPatch((x, y_input-0.4), (x, y_proc1+0.3),
                               arrowstyle='->', lw=1.5, color=color_arrow,
                               connectionstyle='arc3,rad=0')
        ax.add_patch(arrow)

# Process 1 to Trust
arrow1 = FancyArrowPatch((2.5, y_proc1-0.3), (3.5, y_proc2+0.6),
                        arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow1)
arrow2 = FancyArrowPatch((4.5, y_proc1-0.3), (5.0, y_proc2+0.6),
                        arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow2)
arrow3 = FancyArrowPatch((6.5, y_proc1-0.3), (6.5, y_proc2+0.6),
                        arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow3)

# Trust to Leadership
arrow = FancyArrowPatch((5.0, y_proc2-0.6), (5.0, y_proc3+0.5),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)

# Leadership to Assignment
arrow = FancyArrowPatch((5.0, y_proc3-0.5), (5.0, y_proc4+0.4),
                       arrowstyle='->', lw=2, color=color_arrow)
ax.add_patch(arrow)

# Assignment to Output
for x in [2.5, 5.0, 7.5]:
    arrow = FancyArrowPatch((x, y_proc4-0.4), (x, y_output+0.3),
                           arrowstyle='->', lw=1.5, color=color_arrow)
    ax.add_patch(arrow)

# Title
ax.text(5.0, 7.8, 'REIP Governance Pipeline', ha='center', va='center',
        fontsize=14, weight='bold')

# Save
plt.tight_layout()
plt.savefig('paper_docs/Ryker_Kollmyer___UPDATED_PAPER/reip_pipeline.png',
            dpi=300, bbox_inches='tight', facecolor='white')
print("Saved: paper_docs/Ryker_Kollmyer___UPDATED_PAPER/reip_pipeline.png")
