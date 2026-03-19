#!/usr/bin/env python3
from tube_pipeline import TubePipeline
from tube_spec import TubeSpec
from plotter import Plotter

"""
Entry point — define your trajectory here
------------------------------------------------------------------------------------------
This is the main entry point for the trajectory generation process. Here, you can define 
the sequence of tube segments that make up the desired trajectory by creating TubeSpec 
objects with the appropriate constraints and specifications. The TubePipeline will then be 
used to generate the tubes based on these specifications, ensuring continuity between 
segments and saving the resulting coefficients to a CSV file. Finally, the generated tubes 
can be visualized using the Plotter class, which will display the trajectory as a function 
of time for each dimension. This modular structure allows for easy modification and 
extension of the trajectory specifications, enabling you to tailor the generated trajectory 
to your specific requirements and constraints.
"""


if __name__ == "__main__":
    pipeline = TubePipeline(dimension=2, time_step=0.1, output_csv="config/coefficients.csv")

    pipeline.add(TubeSpec(
        t_start=0, t_end=11, degree=1,
        reach_constraints=[
            # Format: (x1_lo, x1_hi, x2_lo, x2_hi, t_start, t_end)
            (0.0, 1.0,  0.0, 1.0,  0,  1),    # start region
            (5.0, 6.0,  3.0, 4.0,  10, 11),   # end region
        ]
    ))

    pipeline.add(TubeSpec(
        t_start=11, t_end=20, degree=3,
        reach_constraints=[
            (9.0, 10.0,  9.0, 10.0,  19, 20),  # end region
        ]
    ))

    tubes = pipeline.run()
    Plotter(dim=2).plot(tubes, title="Generated Trajectory for Turtlebot")
