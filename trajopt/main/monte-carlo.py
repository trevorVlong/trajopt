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
import pathlib as path
if TYPE_CHECKING:
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob


def monteCarloCruise(problem: 'Trajprob',
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
            if condition_vals['static']:
                initial_condition_vals[condition] = mean
            else:
                initial_condition_vals[condition] = np.random.normal(loc=mean,scale=std)
        run_deck[run_num] = initial_condition_vals.copy()
    # create a run deck using given parameters
    # run problem for other gust conditions

    # create problem and problem parameters for initial conditions, gust conditions
    parameters = {}
    for key,valdict in initial_conditions.items():
        parameters[key]=problem.parameter(valdict['avg'])

    problem = cruiseProblemTime(problem,
                                time_vec,
                                parameters
                                )

    # ==================================================================================================================
    # find the solution to the problem center, then performn run_num solves of the monte-carlo problem

    data = dict()
    # do initial solve
    sol = problem.solve(save_solution=True,)
    data[0] = {'results':sol(problem.PhysicsModel),
               'time':sol(problem.Time)}
    # loop through and monte carlo parameters
    for idx,valdict in run_deck.items():
        for key,value in valdict.items():
            problem.set_value(parameters[key],value)
        problem.set_initial_from_sol(problem.ReferenceSolution)
        sol = problem.solve()
        data[idx+1] = {'results':sol(problem.PhysicsModel),
               'time':sol(problem.Time)}
    print('here')
    return data,run_deck

if __name__ == "__main__":

    from trajopt.dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob


    gust_velocities = [0,1]
    time = np.arange(0,15,0.25)
    parameters = {
        "InitialXPosition":{"avg": 0,"std": 10,"static":True},
        "InitialAltitude": {"avg": 100, "std": 10, "static": True},
        "InitialXVelocity": {"avg": 17.5, "std": 1, "static": True},
        "InitialZVelocity": {"avg": 0, "std": 0.05, "static": True},
        "InitialPitch": {"avg": 3, "std": 10, "static": True},
        "gust_vel": {'avg':0,"std":6,"static":True},
        "InitialThrottle": {'avg':0.75,'std':0,'static':True},
        "FlapAngle":{'avg':30,'std':10,'static':False}
    }
    cache_name = path.PosixPath('/Users/TrevorLong/Desktop/test_cache.json')
    problem = Trajprob(save_to_cache_on_solve=True,cache_filename=str(cache_name))
    data,params = monteCarloCruise(problem,time,num_runs=10, initial_conditions=parameters)
    fig,ax = plt.subplots()
    fig2,ax2 = plt.subplots()
    fig3,ax3 = plt.subplots()
    fig4,ax4 = plt.subplots()
    fig5,ax5 = plt.subplots()
    fig6,ax6 = plt.subplots()
    pkeys = list(params.keys())
    for idx,ds in data.items():

        ax.scatter(ds['time'],ds['results'].Altitude,label=f'run {idx})')
        ax3.plot(ds['time'],ds['results'].ElevatorPosition,label=f'run {idx}',
                 marker='+')
        ax4.plot(ds['time'],ds['results'].ThrottlePosition,label=f'run {idx}',marker='*')
        ax5.plot(ds['time'], ds['results'].Pitch, label=f'run {idx}')
        ax6.plot(ds['time'],ds['results'].Airspeed,label=f'run {idx}',marker='*')
        if idx>0:
            pkey = pkeys[idx-1]
            ax2.scatter(idx,params[pkey]['gust_vel'],label=f'run {idx})')
        ax.legend()
        ax2.legend()
        ax3.legend()
        ax4.legend()
        ax5.legend()

    print('done')
    plt.show()