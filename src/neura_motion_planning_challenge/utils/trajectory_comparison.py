#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# Original trajectory data
orig_times = [0, 0.09, 0.18, 0.27, 0.36, 0.45, 0.54, 0.63, 0.72, 0.81, 0.9, 0.99, 1.08, 1.17, 1.26, 1.35, 1.44, 1.53, 1.62, 1.71, 1.8, 1.89, 1.98, 2.07, 2.16, 2.25, 2.34]

# Optimized trajectory data
opt_times = [0, 0.0909144, 0.192349, 0.289118, 0.389118, 0.489118, 0.589118, 0.689118, 0.789118, 0.889118, 0.989118, 1.08912, 1.18912, 1.28912, 1.38912, 1.48912, 1.58912, 1.68912, 1.78912, 1.88912, 1.98912, 2.08912, 2.18912, 2.28912, 2.38912, 2.48912, 2.58912, 2.68912, 2.78912, 2.88912, 2.98912, 3.08912, 3.18912, 3.28912, 3.38912, 3.48912, 3.58912, 3.68912, 3.78912, 3.88912, 3.98912, 4.08912, 4.18912, 4.28912, 4.38912, 4.48912, 4.58912, 4.68912, 4.78912, 4.88912, 4.98912, 5.08912, 5.18912, 5.28912, 5.38912, 5.48912, 5.58912, 5.68912, 5.78912, 5.88912, 5.98912, 6.08912, 6.18912, 6.28912, 6.38912, 6.48912, 6.58912, 6.68912, 6.78912, 6.88912, 6.98912, 7.08912, 7.18912, 7.28912, 7.38912, 7.48912, 7.59232, 7.69092, 7.78542, 7.87823, 7.97823, 8.07823, 8.17823, 8.27823, 8.37823, 8.47823, 8.57823, 8.67823, 8.77823, 8.87823, 8.97823, 9.07823, 9.17823, 9.27823, 9.37823, 9.47823, 9.57823, 9.67823, 9.77823, 9.87823, 9.97823, 10.0782, 10.1782, 10.2782, 10.3782, 10.4782, 10.5782, 10.6782, 10.7782, 10.8782, 10.9782, 11.0782, 11.1782, 11.2782, 11.3782, 11.4782, 11.5782, 11.6782, 11.7782, 11.8782, 11.9782, 12.0782, 12.1782, 12.2782, 12.3782, 12.4782, 12.5782, 12.6782, 12.7782, 12.8782, 12.9782, 13.0782, 13.1782, 13.2782, 13.3782, 13.4782, 13.5782, 13.6782, 13.7782, 13.8782, 13.9782, 14.0782, 14.1782, 14.2782, 14.3782, 14.4782, 14.5782, 14.6782, 14.7782, 14.8782, 14.9782, 15.0782, 15.1782, 15.2782, 15.3782, 15.4782, 15.5782, 15.6782, 15.774, 15.8652, 15.9617, 16.0512, 16.1386, 16.2334, 16.3334, 16.4334, 16.5334, 16.6334, 16.7334, 16.8334, 16.9334, 17.0334, 17.1334, 17.2334, 17.3334, 17.4334, 17.5334, 17.6334, 17.7334, 17.8334, 17.9334, 18.0334, 18.1334, 18.2334, 18.3334, 18.4334, 18.5334, 18.6334, 18.7334, 18.8334, 18.9334, 19.0334, 19.1334, 19.2334, 19.3334, 19.4334, 19.5334, 19.6334, 19.7334, 19.8334, 19.9334, 20.0334, 20.1334, 20.2334, 20.3334, 20.4334, 20.5334, 20.6334, 20.7334, 20.8334, 20.9334, 21.0334, 21.1334, 21.2334, 21.3334, 21.4334, 21.5334, 21.6334, 21.7334, 21.8334, 21.9334, 22.0334, 22.1334, 22.2334, 22.3334, 22.4334, 22.5334, 22.6334, 22.7334, 22.8334, 22.9334, 23.0334, 23.1334, 23.2334, 23.3334, 23.4334, 23.5334, 23.6334, 23.7334, 23.8334, 23.9334, 24.0334, 24.1304, 24.2171, 24.3098, 24.4069, 24.4955, 24.5955, 24.6955, 24.7952, 24.8968, 24.9996, 25.0619]

# Create figure with subplots
num_joints = 7
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Trajectory Comparison: Original vs Optimized', fontsize=16, fontweight='bold', y=0.98)
plt.subplots_adjust(top=0.94)

# Duration and Waypoint Count
ax = axes[0, 0]
categories = ['Duration (s)', 'Waypoints']
original = [2.34, 27]
optimized = [25.0619, 253]
x = np.arange(len(categories))
width = 0.35
ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)
ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)
ax.set_ylabel('Value')
ax.set_title('Performance Metrics')
ax.set_xticks(x)
ax.set_xticklabels(categories)
ax.legend()
ax.grid(True, alpha=0.3)

# Velocity Metrics
ax = axes[0, 1]
categories = ['Avg Velocity', 'Peak Velocity']
original = [2.22782, 20.3002]
optimized = [0.198645, 0.15]
x = np.arange(len(categories))
ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)
ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)
ax.set_ylabel('Velocity (rad/s)')
ax.set_title('Velocity Profiles')
ax.set_xticks(x)
ax.set_xticklabels(categories)
ax.legend()
ax.grid(True, alpha=0.3)

# Improvements (percentage)
ax = axes[1, 0]
improvements = [-971.021, 99.2611, 6.83213e+19]
categories = ['Duration', 'Peak Vel', 'Waypoints']
colors = ['#2ca02c' if x > 0 else '#d62728' for x in improvements]
bars = ax.bar(categories, improvements, color=colors, alpha=0.7)
ax.set_ylabel('Improvement (%)')
ax.set_title('Optimization Gains')
ax.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
for bar in bars:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height,
            f'{height:.1f}%', ha='center', va='bottom' if height > 0 else 'top')
ax.grid(True, alpha=0.3, axis='y')

# Path Length Comparison
ax = axes[1, 1]
categories = ['Original', 'Optimized']
path_lengths = [5.21311, 4.99153]
colors_path = ['#1f77b4', '#ff7f0e']
ax.bar(categories, path_lengths, color=colors_path, alpha=0.8)
ax.set_ylabel('Path Length (radians)')
ax.set_title('Configuration Space Path Length')
for i, v in enumerate(path_lengths):
    ax.text(i, v, f'{v:.6f}', ha='center', va='bottom')
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig('/home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/trajectory_comparison.png', dpi=150, bbox_inches='tight')
print(f'Plot saved to /home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/trajectory_comparison.png')
plt.close()
