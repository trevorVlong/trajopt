# Created by trevorlong on 8/12/25
# license
# Copyright 2025 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from problems import AircraftTrajectoryProblem2D as Trajprob
from aerodynamics import ThinAirfoilModel
from dynamics import Aircraft2DPointMass
import numpy as np
from cruiseExample import cruiseProblemTime
from copy import deepcopy


def cruiseGustComparison(gust_velocity_vec,
                         time_vec):


    solutions = dict()
    # run problem for other gust conditions
    for idx,gust_velocity in enumerate(gust_velocity_vec):
        problem = cruiseProblemTime(time_vec,gust_velocity)

        problem.solve()

        solutions[f"gust{gust_velocity}_problem"] = problem.CurrentSolution(problem.PhysicsModel)
        problem.cache_filename = f"gust{gust_velocity}_problem"
    return problem, solutions



if __name__ == "__main__":

    from dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt

    gust_velocities = np.arange(0,10,1.5)
    time = np.arange(0,10,0.1)

    problem, pdict = cruiseGustComparison(gust_velocities,time)

    for name,res in pdict.items():
        visualizeRun2D(problem.Time,res)


    plt.show()