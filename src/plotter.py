#!/usr/bin/env python3
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

    def plot(self, tubes: list, title: str = "Generated Trajectory"):
        """
        tubes: list of [coeffs, t_start, t_end]
        """
        if not tubes or tubes[0][0] is None:
            print("No data to plot.")
            return

        fig, axs = plt.subplots(self.dim, 1, figsize=(10, 2.5 * self.dim), constrained_layout=True)
        if self.dim == 1:
            axs = [axs]

        for idx, (coeffs, t_start, t_end) in enumerate(tubes):
            n = len(coeffs)
            if n % self.dim != 0:
                print(f"Tube {idx}: {n} coefficients not divisible by dim={self.dim}, skipping.")
                continue
            degree = n // self.dim - 1
            print(f"Tube {idx}: degree={degree}  t=[{t_start}, {t_end}]")

            t_vals = np.arange(t_start, t_end + self.dt, self.dt)
            coords = [[] for _ in range(self.dim)]
            for t in t_vals:
                pos = self._eval(t, coeffs, self.dim, degree)
                for d in range(self.dim):
                    coords[d].append(pos[d])

            for d in range(self.dim):
                axs[d].plot(t_vals, coords[d], label=f"Tube {idx} (deg {degree})")
                axs[d].set_ylabel(f"x{d+1} (m)")
                axs[d].grid(True, linestyle='--', alpha=0.6)
                axs[d].legend(loc='upper right', fontsize='small')

        axs[0].set_title(title)
        axs[-1].set_xlabel("Time (s)")
        plt.show()
