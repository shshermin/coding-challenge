#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless systems
import matplotlib.pyplot as plt
import numpy as np

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
plt.subplots_adjust(top=0.96)

# Planning Time and Path Length with Velocities
ax = axes[0, 0]
categories = ['Plan Time (s)', 'Path Length', 'Peak Vel (r/s)', 'Avg Vel (r/s)']
original = [2.34, 5.21311, 20.3002, 2.22782]
optimized = [25.0619, 4.99153, 0.210969, 0.199168]
x = np.arange(len(categories))
width = 0.35
ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)
ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)
ax.set_ylabel('Value')
ax.set_title('Performance Metrics')
ax.set_xticks(x)
ax.set_xticklabels(categories, rotation=15, ha='right')
ax.legend()
ax.grid(True, alpha=0.3, axis='y')

# Joint Configuration Metrics
ax = axes[0, 1]
categories = ['Sum Abs Joints']
original = [116.669]
optimized = [1194.84]
x = np.arange(len(categories))
ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)
ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)
ax.set_ylabel('Sum of Absolute Joint Values (rad)')
ax.set_title('Joint Configuration Analysis')
ax.set_xticks(x)
ax.set_xticklabels(categories)
ax.legend()
ax.grid(True, alpha=0.3)

# Optimization Improvements
ax = axes[1, 0]
improvements = [-971.021, 4.2504, 98.9608, 91.06, -924.131]
categories = ['Plan Time', 'Path Length', 'Peak Vel', 'Avg Vel', 'Joint Motion']
colors = ['#2ca02c' if x > 0 else '#d62728' for x in improvements]
bars = ax.bar(categories, improvements, color=colors, alpha=0.7)
ax.set_ylabel('Improvement (%)')
ax.set_title('Optimization Gains')
ax.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
ax.set_xticklabels(categories, rotation=15, ha='right')
for bar in bars:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height,
            f'{height:.1f}%', ha='center', va='bottom' if height > 0 else 'top', fontsize=9)
ax.grid(True, alpha=0.3, axis='y')

# Summary Comparison Table
ax = axes[1, 1]
ax.axis('tight')
ax.axis('off')
summary_data = [
    ['Metric', 'Original', 'Optimized', 'Improvement'],
    ['Planning Time (s)', f'{2.34:.4f}', f'{25.0619:.4f}', f'{-971.021:.1f}%'],
    ['Path Length (rad)', f'{5.21311:.6f}', f'{4.99153:.6f}', f'{4.2504:.1f}%'],
    ['Peak Velocity (r/s)', f'{20.3002:.6f}', f'{0.210969:.6f}', f'{98.9608:.1f}%'],
    ['Avg Velocity (r/s)', f'{2.22782:.6f}', f'{0.199168:.6f}', f'{91.06:.1f}%'],
    ['Sum Abs Joints (rad)', f'{116.669:.4f}', f'{1194.84:.4f}', f'{-924.131:.1f}%'],
]
table = ax.table(cellText=summary_data, cellLoc='center', loc='center',
                colWidths=[0.22, 0.26, 0.26, 0.26])
table.auto_set_font_size(False)
table.set_fontsize(9)
table.scale(1, 1.8)
# Style header row
for i in range(4):
    table[(0, i)].set_facecolor('#4472C4')
    table[(0, i)].set_text_props(weight='bold', color='white')
ax.set_title('Performance Summary', pad=20, fontweight='bold')

plt.tight_layout()
plt.savefig('/home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/trajectory_comparison.png', dpi=150, bbox_inches='tight')
print(f'Plot saved to /home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/trajectory_comparison.png')
plt.close()
