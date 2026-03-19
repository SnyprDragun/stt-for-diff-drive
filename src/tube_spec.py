#!/usr/bin/env python3
from dataclasses import dataclass, field

"""
Tube specification — describes one segment declaratively
---------------------------------------------------------------------------------------
This module defines the TubeSpec dataclass, which serves as a declarative specification
for a single tube segment in the trajectory generation process. Each TubeSpec includes
a time window (t_start, t_end), a polynomial degree for the trajectory, and a list of 
reachability constraints that define the spatial and temporal bounds for the tube.
The reachability constraints are expected to be provided as tuples of the form 
(x1, x2, y1, y2, t1, t2), which specify the spatial bounds (x1, x2) and (y1, y2) and 
the temporal bound (t1, t2).
"""

@dataclass
class TubeSpec:
    """
    Declarative spec for one tube segment.

    reach_constraints: list of reach(...) arg-tuples e.g. [(x1, x2, y1, y2, t1, t2), ...]
    t_start / t_end:   time window for this tube
    degree:            polynomial degree
    """
    t_start:            float
    t_end:              float
    degree:             int
    reach_constraints:  list = field(default_factory=list)
