#!/usr/bin/env python3
"""
STT (Spatio-Temporal Tube) generation pipeline.

Usage:
    python3 generate_stt.py

Architecture:
    TubeSpec        — data class describing one tube's constraints
    TubePipeline    — orchestrates multi-tube generation with joining
    Generate_STT    — SMT solver core (one tube)
    Plotter         — isolated visualisation

Output Format:
    +----------+----------------+-----------+
    |   Coeff  |    dimension   |   degree  |
    +==========+================+===========+
    |   C_0,0  |        x       |      0    |
    +----------+----------------+-----------+
    |   C_0,1  |        x       |      1    |
    +----------+----------------+-----------+
    |   C_1,0  |        y       |      0    |
    +----------+----------------+-----------+
    |   C_1,1  |        y       |      1    |
    +----------+----------------+-----------+
    |   C_2,2  |        z       |      2    |
    +----------+----------------+-----------+
    |     .    |        .       |      .    |
    |     .    |        .       |      .    |
    |     .    |        .       |      .    |
    +----------+----------------+-----------+
"""

import z3
import csv
import time
import numpy as np
from dataclasses import dataclass, field
from tabulate import tabulate
from typing import Optional
import matplotlib.pyplot as plt

"""
Core SMT solver
------------------------------------------------------------------------------------------
This module defines the Generate_STT class, which serves as the core SMT-based solver for 
generating spatio-temporal tubes. The class is designed to handle the generation of a 
single tube segment, given a set of constraints and specifications. It provides methods 
for defining the polynomial trajectory, evaluating it symbolically and numerically, and 
finding a solution that satisfies the specified constraints. The solver also includes 
functionality for joining consecutive tube segments to ensure continuity in the generated 
trajectory. The class is structured to be flexible and extensible, allowing for the 
integration of various types of constraints and specifications as needed for different 
applications.
"""

class Generate_STT():
    """SMT-based STT solver for a single tube segment."""

    def __init__(self, degree: int, dimension: int, time_step: float):
        self.setpoints = []
        self.obstacles = []
        self._step     = time_step
        self.degree    = degree
        self.dimension = dimension

        self._start = self._finish = self._range = 0

        self.solver   = z3.Solver()
        z3.set_param("parallel.enable", True)
        self.C        = [z3.Real(f'C_x{axis+1},{i}')
                         for axis in range(self.dimension)
                         for i in range(self.degree + 1)]
        self.C_solved = []
        self.flag     = False
        self.go_ahead = True

    # ── Symbolic evaluators ───────────────────────────────────────────────────

    def gamma(self, t):
        tubes = [z3.Real(f'gamma_{i}') for i in range(self.dimension)]
        for i in range(self.dimension):
            tubes[i] = 0
            power = 0
            for j in range(self.degree + 1):
                tubes[i] += self.C[j + i * (self.degree + 1)] * (t ** power)
                power += 1
        return tubes

    def gamma_dot(self, t):
        tubes = [z3.Real(f'gammadot_{i}') for i in range(self.dimension)]
        for i in range(self.dimension):
            tubes[i] = 0
            power = 0
            for j in range(self.degree + 1):
                if power >= 1:
                    tubes[i] += power * self.C[j + i * (self.degree + 1)] * (t ** (power - 1))
                power += 1
        return tubes

    # ── Numeric evaluators ────────────────────────────────────────────────────

    def real_gamma(self, t, C_fin):
        result = np.zeros(self.dimension)
        for i in range(self.dimension):
            power = 0
            for j in range(self.degree + 1):
                result[i] += C_fin[j + i * (self.degree + 1)] * (t ** power)
                power += 1
        return result

    def real_gamma_dot(self, t, C_fin):
        result = np.zeros(self.dimension)
        for i in range(self.dimension):
            power = 0
            for j in range(self.degree + 1):
                if power >= 1:
                    result[i] += power * C_fin[j + i * (self.degree + 1)] * (t ** (power - 1))
                power += 1
        return result

    # ── Solver ────────────────────────────────────────────────────────────────

    def find_solution(self) -> list:
        start = time.time()
        print("Solving...")
        self.setAll()

        if not self.go_ahead:
            print("Blocked: previous solver had no solution.")
            return []

        if self.solver.check() == z3.sat:
            self.flag = True
            model = self.solver.model()
            xi = np.zeros(self.dimension * (self.degree + 1))
            for i, c in enumerate(self.C):
                try:
                    xi[i] = float(model[c].numerator().as_long()) / float(model[c].denominator().as_long())
                except AttributeError:
                    xi[i] = float(model[c])
                self.C_solved.append(xi[i])
        else:
            print("No solution found.")

        self._display_time(start, time.time())
        return self.C_solved

    # ── Continuity join ───────────────────────────────────────────────────────

    def join_constraint(self, prev_coeffs: list, prev_solver: 'Generate_STT', t_join: float):
        if not prev_solver.flag:
            self.go_ahead = False
            print("Previous solver has no solution — cannot join.")
            return
        for i in range(self.dimension):
            self.solver.add(self.gamma(t_join)[i]     == prev_solver.real_gamma(t_join,     prev_coeffs)[i])
            self.solver.add(self.gamma_dot(t_join)[i] == prev_solver.real_gamma_dot(t_join, prev_coeffs)[i])

    # ── Internal helpers ──────────────────────────────────────────────────────

    def setAll(self):
        all_points = self.setpoints + self.obstacles
        t1_all, t2_all = [], []
        for pt in all_points:
            t1_all.append(pt[2 * self.dimension])
            t2_all.append(pt[2 * self.dimension + 1])
        self.setStart(min(t1_all))
        self.setFinish(max(t2_all))
        self.setRange(int((self.getFinish() - self.getStart() + self._step) / self._step))

    def _display_time(self, start, end):
        duration = end - start
        mins = int(duration) // 60
        secs = round(duration - mins * 60, 2)
        print(f"Time taken: {mins}m {secs}s")

    def getStart(self):    return self._start
    def setStart(self, v): self._start = v
    def getFinish(self):   return self._finish
    def setFinish(self, v): self._finish = v
    def getRange(self):    return self._range
    def setRange(self, v): self._range = v


"""
Constraint helpers
------------------------------------------------------------------------------------------
This section defines helper functions for constructing constraints to be added to the SMT 
solver. The `reach` function builds reachability constraints for a box region over a 
specified time window, while the `apply_constraints` function flattens and adds all 
constraint lists to the solver. These helper functions are designed to simplify the 
process of defining spatial and temporal constraints for the trajectory generation, 
allowing for a more declarative approach when specifying the desired properties of the 
generated tubes.
"""

def reach(solver: Generate_STT, *args) -> list:
    """
    Build reach constraints for a box region over a time window.

    args = [x1_lo, x1_hi, x2_lo, x2_hi, ..., t_start, t_end]
    """
    dim = solver.dimension
    assert len(args) == 2 * dim + 2, f"reach() expects {2*dim+2} args, got {len(args)}"
    bounds_flat, t1, t2 = args[:-2], args[-2], args[-1]
    solver.setpoints.append(list(args))

    bounds = [(bounds_flat[i], bounds_flat[i + 1]) for i in range(0, 2 * dim, 2)]
    constraints = []
    for t in np.arange(t1, t2, solver._step):
        gamma_t = solver.gamma(t)
        constraints.append([
            z3.And(gamma_t[d] > bounds[d][0], gamma_t[d] < bounds[d][1])
            for d in range(dim)
        ])
    return constraints

def apply_constraints(solver: Generate_STT, constraint_lists: list):
    """Flatten and add all constraint lists to the solver."""
    for clist in constraint_lists:
        for c in clist:
            solver.solver.add(c)
