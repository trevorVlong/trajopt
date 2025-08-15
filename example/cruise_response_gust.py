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
from problems import AircraftTrajectoryProblem2D as Trajprob


def cruiseGustComparison(problem: Trajprob,
                         gust_velocity_vec,
                         time_vec):


    # run problem for other gust conditions
    gust_vel = problem.parameter(0)
    problem = cruiseProblemTime(problem, time_vec, gust_vel)
    for idx, gust_velocity in enumerate(gust_velocity_vec):

        # solve problem 1
        sol = problem.solve()
        # change something about setup, run problem 1 again from existing solution
        problem.set_initial_from_sol(problem.CurrentSolution)
        problem.set_value(gust_vel, gust_velocity)
        problem.solve()
    return problem


if __name__ == "__main__":

    from dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt

    gust_velocities = [0,6]
    time = np.arange(0,12,0.1)
    problem = Trajprob()
    problem = cruiseGustComparison(Trajprob(),gust_velocities,time)
    figdict = visualizeRun2D(problem.Time,problem.LastSolution(problem.PhysicsModel),case='cruise',casenum=0)
    visualizeRun2D(problem.Time,problem.CurrentSolution(problem.PhysicsModel), fig_dict=figdict,case='$w_g=6m/s$', casenum=1)


    plt.show()