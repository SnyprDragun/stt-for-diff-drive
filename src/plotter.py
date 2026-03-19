#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt

"""
Visualisation
--------------------------------------------------------------------------------------
This module provides a simple plotting utility to visualize the generated trajectories
as a function of time. Each tube is plotted in its own subplot, showing the position
over time for each dimension. The tubes are expected to be represented as lists of
coefficients for polynomial trajectories, along with their start and end times.
"""

class Plotter:
    """Plots a list of solved tubes against time, one subplot per dimension."""

    def __init__(self, dim: int = 2, dt: float = 0.05):
        self.dim = dim
        self.dt  = dt

    @staticmethod
    def _eval(t, coeffs, dim, degree):
        result = np.zeros(dim)
        for i in range(dim):
            power = 0
            for j in range(degree + 1):
                result[i] += coeffs[j + i * (degree + 1)] * (t ** power)
                power += 1
        return result

    def plot(self, tubes: list, title: str = "Generated Trajectory",
             time_label_interval: float = 5.0):
        """
        tubes: list of [coeffs, t_start, t_end]

        Produces two figures:
          1. Per-dimension time plots  (x1 vs t, x2 vs t, ...)
          2. x1 vs x2 phase plot with time labels every `time_label_interval` seconds
             (only for dim >= 2)

        Both figures are saved to `save_dir`.
        """
        save_dir = os.path.join(os.path.dirname(__file__), "..", "media")
        if not tubes or tubes[0][0] is None:
            print("No data to plot.")
            return

        # ── Evaluate all tubes up front ───────────────────────────────────────
        all_t:      list = []
        all_coords: list = []

        for idx, (coeffs, t_start, t_end) in enumerate(tubes):
            n = len(coeffs)
            if n % self.dim != 0:
                print(f"Tube {idx}: {n} coefficients not divisible by dim={self.dim}, skipping.")
                all_t.append(np.array([]))
                all_coords.append([[] for _ in range(self.dim)])
                continue

            degree = n // self.dim - 1
            print(f"Tube {idx}: degree={degree}  t=[{t_start}, {t_end}]")

            t_vals = np.arange(t_start, t_end + self.dt, self.dt)
            coords = [[] for _ in range(self.dim)]
            for t in t_vals:
                pos = self._eval(t, coeffs, self.dim, degree)
                for d in range(self.dim):
                    coords[d].append(pos[d])

            all_t.append(t_vals)
            all_coords.append(coords)

        # ── Figure 1: per-dimension time plots ───────────────────────────────
        fig1, axs = plt.subplots(self.dim, 1,
                                 figsize=(10, 2.5 * self.dim),
                                 constrained_layout=True)
        if self.dim == 1:
            axs = [axs]

        for idx, (coeffs, t_start, t_end) in enumerate(tubes):
            if len(all_t[idx]) == 0:
                continue
            degree = len(coeffs) // self.dim - 1
            for d in range(self.dim):
                axs[d].plot(all_t[idx], all_coords[idx][d],
                            label=f"Tube {idx} (deg {degree})")
                axs[d].set_ylabel(f"x{d+1} (m)")
                axs[d].grid(True, linestyle='--', alpha=0.6)
                axs[d].legend(loc='upper right', fontsize='small')

        axs[0].set_title(title)
        axs[-1].set_xlabel("Time (s)")

        # ── Figure 2: x1 vs x2 phase plot (only for dim >= 2) ────────────────
        if self.dim >= 2:
            fig2, ax_xy = plt.subplots(figsize=(7, 6), constrained_layout=True)

            for idx, (coeffs, t_start, t_end) in enumerate(tubes):
                if len(all_t[idx]) == 0:
                    continue
                degree = len(coeffs) // self.dim - 1
                xs     = all_coords[idx][0]
                ys     = all_coords[idx][1]
                t_vals = all_t[idx]

                ax_xy.plot(xs, ys, label=f"Tube {idx} (deg {degree})", zorder=2)

                # ── Time labels every `time_label_interval` seconds ───────────
                t_ticks = np.arange(
                    np.ceil(t_start / time_label_interval) * time_label_interval,
                    t_end + 1e-9,
                    time_label_interval
                )
                for t_tick in t_ticks:
                    i_near = int(np.argmin(np.abs(t_vals - t_tick)))
                    x_tick, y_tick = xs[i_near], ys[i_near]
                    ax_xy.scatter(x_tick, y_tick, s=40, color='black', zorder=4, linewidths=0)
                    ax_xy.annotate(f"t={t_tick:.0f}s", xy=(x_tick, y_tick),
                                   xytext=(6, 6), textcoords='offset points',
                                   fontsize=8, color='black', zorder=5)

                # ── Start / end markers ───────────────────────────────────────
                ax_xy.scatter(xs[0],  ys[0],  marker='o', s=80, color='green', zorder=5,
                              label="Start" if idx == 0 else "")
                ax_xy.scatter(xs[-1], ys[-1], marker='X', s=80, color='red',   zorder=5,
                              label="End"   if idx == len(tubes) - 1 else "")

            ax_xy.set_xlabel("x1 (m)")
            ax_xy.set_ylabel("x2 (m)")
            ax_xy.set_title(f"{title} — Phase Plot (x1 vs x2)")
            ax_xy.set_aspect('equal', adjustable='datalim')
            ax_xy.grid(True, linestyle='--', alpha=0.6)
            ax_xy.legend(loc='upper left', fontsize='small')

        # ── Save both figures ─────────────────────────────────────────────────
        os.makedirs(save_dir, exist_ok=True)

        path1 = os.path.join(save_dir, f"time_plots.png")
        fig1.savefig(path1, dpi=150, bbox_inches='tight')
        print(f"Saved: {path1}")

        if self.dim >= 2:
            path2 = os.path.join(save_dir, f"phase_plot.png")
            fig2.savefig(path2, dpi=150, bbox_inches='tight')
            print(f"Saved: {path2}")

        plt.show()
