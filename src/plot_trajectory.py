#!/usr/bin/env python3
"""
Plot desired vs actual trajectory from trajectory.csv logged by turtlebot_node.

Usage:
    python3 plot_trajectory.py                          # auto-finds csv via ROS2 package
    python3 plot_trajectory.py path/to/trajectory.csv  # explicit path
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ── Load CSV ──────────────────────────────────────────────────────────────────

def find_csv() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    # Default: relative to this script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default = os.path.join(script_dir, "..", "config", "trajectory.csv")
    if os.path.exists(default):
        return default
    raise FileNotFoundError(
        f"No CSV found at {default}. Pass path as argument: python3 plot_trajectory.py <path>"
    )

csv_path = find_csv()
df = pd.read_csv(csv_path)
print(f"Loaded {len(df)} rows from {csv_path}")

t      = df["t"].to_numpy()
des_x  = df["des_x"].to_numpy()
des_y  = df["des_y"].to_numpy()
act_x  = df["act_x"].to_numpy()
act_y  = df["act_y"].to_numpy()
v      = df["v"].to_numpy()
omega  = df["omega"].to_numpy()

# ── Derived ───────────────────────────────────────────────────────────────────

err_x = act_x - des_x
err_y = act_y - des_y
err_norm = np.sqrt(err_x**2 + err_y**2)

# ── Figure 1: Phase plot (x vs y) ────────────────────────────────────────────

fig1, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)

ax.plot(des_x, des_y, 'b--', linewidth=1.5, label="Desired", zorder=2)
ax.plot(act_x, act_y, 'r-',  linewidth=1.5, label="Actual",  zorder=3)

# Time labels every 5s on desired
label_interval = 5.0
t_ticks = np.arange(
    np.ceil(t[0] / label_interval) * label_interval,
    t[-1] + 1e-9,
    label_interval
)
for t_tick in t_ticks:
    i = int(np.argmin(np.abs(t - t_tick)))
    ax.scatter(des_x[i], des_y[i], s=40, color='blue',  zorder=4, linewidths=0)
    ax.scatter(act_x[i], act_y[i], s=40, color='red',   zorder=4, linewidths=0)
    ax.annotate(f"t={t_tick:.0f}s", xy=(des_x[i], des_y[i]),
                xytext=(6, 6), textcoords='offset points', fontsize=8, color='blue')

# Start / end markers
ax.scatter(des_x[0],  des_y[0],  marker='o', s=100, color='green', zorder=5, label="Start")
ax.scatter(des_x[-1], des_y[-1], marker='X', s=100, color='black', zorder=5, label="Goal")

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Desired vs Actual Trajectory — Phase Plot")
ax.set_aspect('equal', adjustable='datalim')
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend()

save_dir = os.path.join(os.path.dirname(csv_path), "..", "media")
os.makedirs(save_dir, exist_ok=True)
fig1.savefig(os.path.join(save_dir, "trajectory_phase.png"), dpi=150, bbox_inches='tight')
print(f"Saved: {os.path.join(save_dir, 'trajectory_phase.png')}")

# ── Figure 2: Per-axis time plots ─────────────────────────────────────────────

fig2, axs = plt.subplots(4, 1, figsize=(11, 10), constrained_layout=True, sharex=True)

axs[0].plot(t, des_x, 'b--', label="des_x")
axs[0].plot(t, act_x, 'r-',  label="act_x")
axs[0].set_ylabel("x (m)")
axs[0].legend(loc='upper right', fontsize='small')
axs[0].grid(True, linestyle='--', alpha=0.6)
axs[0].set_title("Trajectory Tracking")

axs[1].plot(t, des_y, 'b--', label="des_y")
axs[1].plot(t, act_y, 'r-',  label="act_y")
axs[1].set_ylabel("y (m)")
axs[1].legend(loc='upper right', fontsize='small')
axs[1].grid(True, linestyle='--', alpha=0.6)

axs[2].plot(t, err_norm, 'k-', linewidth=1.2, label="||error||")
axs[2].set_ylabel("Position error (m)")
axs[2].legend(loc='upper right', fontsize='small')
axs[2].grid(True, linestyle='--', alpha=0.6)

axs[3].plot(t, v,     label="v (m/s)")
axs[3].plot(t, omega, label="ω (rad/s)")
axs[3].set_ylabel("Control input")
axs[3].set_xlabel("Time (s)")
axs[3].legend(loc='upper right', fontsize='small')
axs[3].grid(True, linestyle='--', alpha=0.6)

fig2.savefig(os.path.join(save_dir, "trajectory_time.png"), dpi=150, bbox_inches='tight')
print(f"Saved: {os.path.join(save_dir, 'trajectory_time.png')}")

plt.show()
