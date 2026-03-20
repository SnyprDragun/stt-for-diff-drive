#!/usr/bin/env python3
import csv
import time
from tabulate import tabulate
from typing import Optional
from tube_spec import TubeSpec
from generate_stt import Generate_STT, apply_constraints, reach

"""
Pipeline — orchestrates multi-tube generation
------------------------------------------------------------------------------------------
This module defines the TubePipeline class, which serves as an orchestrator for generating
a sequence of spatio-temporal tubes based on a list of TubeSpec objects. The pipeline 
handles the sequential generation of tubes, ensuring that continuity constraints are 
enforced between consecutive segments. It also manages the persistence of the generated 
tube coefficients to a CSV file and provides a structured output format for visualization. 
The pipeline is designed to be flexible, allowing for the addition of multiple tube 
specifications and the seamless integration of their solutions into a cohesive trajectory.
"""

class TubePipeline:
    """
    Builds a sequence of joined tube segments from TubeSpec objects
    and saves the result to a CSV.

    Example
    -------
    pipeline = TubePipeline(dimension=2, time_step=0.1, output_csv="config/coefficients.csv")
    pipeline.add(TubeSpec(t_start=0, t_end=11, degree=1, reach_constraints=[(0,1, 0,1, 0,1), (5,6, 3,4, 10,11)]))
    pipeline.add(TubeSpec(t_start=11, t_end=20, degree=3, reach_constraints=[(9,10, 9,10, 19,20)]))
    results = pipeline.run()
    """

    def __init__(self, dimension: int, time_step: float, output_csv: str = "config/coefficients.csv"):
        self.dimension   = dimension
        self.time_step   = time_step
        self.output_csv  = output_csv
        self._specs:  list[TubeSpec]       = []
        self._solvers: list[Generate_STT]  = []
        self._coeffs:  list[list]          = []

    def add(self, spec: TubeSpec) -> 'TubePipeline':
        """Append a tube spec. Returns self for method chaining."""
        self._specs.append(spec)
        return self

    def run(self) -> tuple:
        """
        Solve all tubes in order, joining continuity constraints between them.
        Returns:
            tubes:     list of [coeffs, t_start, t_end]
            setpoints: list of [args, tube_idx] for all reach constraints
        """
        global_start = time.time()
        open(self.output_csv, 'w').close()

        prev_solver: Optional[Generate_STT] = None
        prev_coeffs: Optional[list]         = None
        all_setpoints = []

        for idx, spec in enumerate(self._specs):
            print(f"\n{'='*60}")
            print(f"  Tube {idx}  |  degree={spec.degree}  |  t=[{spec.t_start}, {spec.t_end}]")
            print(f"{'='*60}")

            solver = Generate_STT(degree=spec.degree, dimension=self.dimension, time_step=self.time_step)

            for args in spec.reach_constraints:
                apply_constraints(solver, [reach(solver, *args)])

            if prev_solver is not None and prev_coeffs is not None:
                solver.join_constraint(prev_coeffs, prev_solver, spec.t_start)

            coeffs = solver.find_solution()
            self._solvers.append(solver)
            self._coeffs.append(coeffs)

            if solver.flag:
                self._save_tube(coeffs, spec, append=(idx > 0))
                for pt in solver.setpoints:
                    all_setpoints.append({"bounds": pt, "tube_idx": idx, "type": "setpoint"})
                for pt in solver.obstacles:
                    all_setpoints.append({"bounds": pt, "tube_idx": idx, "type": "obstacle"})
            else:
                print(f"Tube {idx} failed — stopping pipeline.")
                break

            prev_solver = solver
            prev_coeffs = coeffs

        print(f"\nTotal pipeline time: ", end="")
        Generate_STT(1, 1, 0.1)._display_time(global_start, time.time())

        tubes = [[self._coeffs[i], self._specs[i].t_start, self._specs[i].t_end]
                for i in range(len(self._coeffs)) if self._solvers[i].flag]

        return tubes, all_setpoints

    # ── CSV persistence ───────────────────────────────────────────────────────

    def _save_tube(self, coeffs: list, spec: TubeSpec, append: bool):
        header = ["Tube Coefficients", "Value"]
        labels = [f'C_x{d+1},{j}'
                  for d in range(self.dimension)
                  for j in range(spec.degree + 1)]
        rows = [[labels[i], coeffs[i]] for i in range(len(coeffs))]

        print(tabulate(rows, headers=header, tablefmt='grid'))

        mode = 'a' if append else 'w'
        with open(self.output_csv, mode=mode, newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        print(f"Saved to {self.output_csv}")
