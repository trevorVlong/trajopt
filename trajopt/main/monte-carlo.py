# Created by trevorlong on 11/17/25
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
import numpy as np
from trajopt.example.cruiseExample import cruiseProblemTime
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob


def cruiseGustComparison(problem: 'Trajprob',
                         time_vec:np.ndarray,
                         num_runs:int,
                         initial_conditions:dict[str,dict[str,float]],
                         ):
    initial_condition_vals = dict()

    # monte-carlo all the conditions in the initial condition space
    run_deck = {}
    for run_num in np.arange(0,num_runs):
        for idx,condition in enumerate(initial_conditions):
            condition_vals = initial_conditions[condition]
            mean = condition_vals['avg']
            std = condition_vals['std']

            # if parameter is supposed to be held static use mean, otherwise monte-carlo
            if condition_vals['static'] or run_num==0:
                initial_condition_vals[condition] = np.array([mean])
            else:
                initial_condition_vals[condition] = np.random.normal(loc=mean,scale=std,size=1)
        run_deck[run_num] = initial_condition_vals.copy()
    # create a run deck using given parameters
    # run problem for other gust conditions

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

    from trajopt.dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob

    gust_velocities = [0,1]
    time = np.arange(0,12,0.1)
    conditions = {
        "xe":{"avg": 0,"std": 10,"static":False},
        "ze": {"avg": 0, "std": 10, "static": False},
        "ub": {"avg": 0, "std": 10, "static": False},
        "wb": {"avg": 0, "std": 10, "static": False},
        "pitch": {"avg": 0, "std": 10, "static": False},
        "throttle": {"avg": 0, "std": 10, "static": False},
        "gust_vel": {'avg':0,"std":1,"static":True}
    }
    problem = Trajprob()
    problem = cruiseGustComparison(Trajprob(),time,num_runs=10, initial_conditions=conditions)
    figdict = visualizeRun2D(problem.Time,problem.LastSolution(problem.PhysicsModel),case='cruise',casenum=0)
    visualizeRun2D(problem.Time,problem.CurrentSolution(problem.PhysicsModel), fig_dict=figdict,case='$w_g=6m/s$', casenum=1)


    plt.show()