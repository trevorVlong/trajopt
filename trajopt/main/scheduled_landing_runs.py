# Created by trevorlong on 12/8/25
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
from trajopt.example.landingBlown import cruiseProblemTime
from trajopt.aerodynamics.courtinSurrogates import *
from trajopt.aerodynamics.aeroUtility import jetMomentumCoefficient
from typing import TYPE_CHECKING
import pathlib as path
if TYPE_CHECKING:
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob


def monteCarloCruise(problem: 'Trajprob',
                     time_vec:np.ndarray,
                     initial_conditions:dict[str,dict[str,float]],
                     ):
    initial_condition_vals = dict()

    # monte-carlo all the conditions in the initial condition space
    run_deck = initial_conditions
    num_runs = len(run_deck['InitialXPosition'])
    # create problem and problem parameters for initial conditions, gust conditions
    parameters = {}
    IC = dict()
    for key,val in initial_conditions.items():
        print(f"{key}: {val[0]}")
        parameters[key]=problem.parameter(val[0])
        IC[key] = val[0]

    problem = cruiseProblemTime(problem,
                                time_vec,
                                parameters
                                )

    # ==================================================================================================================
    # find the solution to the problem center, then performn run_num solves of the monte-carlo problem

    data = dict()
    # do initial solve
    sol = problem.solve(save_solution=False,max_iter=5000)
    if sol.opti.return_status() == 'Solve_Succeeded':
        print('solve succeeded')
    else:
        print('solve failed')
    data[0] = {'results':sol(problem.PhysicsModel),
               'time':sol(problem.Time),
               'solve_status': sol.opti.return_status(),
               'initial_conditions': IC,
               }
    # loop through and monte carlo parameters
    for idx in range(num_runs-1):
        idx = idx+1
        print(f'--------------------------------------------')
        print(f'This is run {idx}')
        print(f'--------------------------------------------')
        for key,value in run_deck.items():
            print(f"{key}: {value[idx]}")
            problem.set_value(parameters[key],value[idx])
            IC[key] = value[idx]
        if sol.opti.return_status() == 'Solve_Succeeded':
            problem.set_initial_from_sol(problem.ReferenceSolution)

        sol = problem.solve(max_iter=3000,verbose=False)
        data[idx+1] = {'results':sol(problem.PhysicsModel),
                       'time':sol(problem.Time),
                       'solve_status':sol.opti.return_status(),
                       'initial_conditions': IC}
        if sol.opti.return_status():
            print(f"Run:{idx + 1}: Success")
        else:
            print(f"Run:{idx + 1}: Fail")
    print('here')
    return data,run_deck

if __name__ == "__main__":

    from trajopt.dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt
    from trajopt.main import AircraftTrajectoryProblem2D as Trajprob

    file_path = "landing_0.4ms_gust.json"
    num_runs = 50
    time = np.arange(0, 20, 0.2)
    parameters = {
        "InitialXPosition":[0, 0, 0],
        "InitialAltitude": [10, 10, 10],
        "InitialXVelocity": [5, 5, 5],
        "InitialZVelocity": [0,0, 0],
        "InitialPitch": [5, 5, 5],
        "gust_vel": [0, 0.4, -0.4],
        "InitialThrottle": [0.7,0.7, 0.7],
        "FlapAngle": [50, 50, 50]
    }
    cache_name = path.PosixPath('/Users/TrevorLong/Desktop/test_cache.json')
    problem = Trajprob(save_to_cache_on_solve=True,cache_filename=str(cache_name))
    data,params = monteCarloCruise(problem,time, initial_conditions=parameters)
    fig,ax = plt.subplots()
    fig2,ax2 = plt.subplots()
    fig3,ax3 = plt.subplots()
    fig4,ax4 = plt.subplots()
    fig5,ax5 = plt.subplots()
    fig6,ax6 = plt.subplots()
    pkeys = list(params.keys())
    for idx,ds in data.items():

        ax.scatter(ds['results'].EarthXPosition,ds['results'].Altitude,label=f'run {idx})')
        ax2.scatter(ds['time'], problem.AeroModel.DeltaCJ(ds['results']), label=f'run {idx})')
        ax3.plot(ds['time'],ds['results'].ElevatorPosition,label=f'run {idx}',
                 marker='+')
        ax4.plot(ds['time'],ds['results'].ThrottlePosition,label=f'run {idx}',marker='*')
        ax5.plot(ds['time'], ds['results'].Pitch, label=f'run {idx}')
        ax6.plot(ds['time'],ds['results'].Airspeed,label=f'run {idx}',marker='*')
        ax.legend()
        ax2.legend()
        ax3.legend()
        ax4.legend()
        ax5.legend()

    # extract data to use
        sim_results = dict()
    for idx,ds in data.items():
        extracted_results = dict()
        extracted_results['time'] = ds['time'].tolist()
        extracted_results['solve_status'] = ds['solve_status']


        # extract relevant state values
        state_res = ds['results']
        state = {
            'Pitch': state_res.Pitch.tolist(),
            'PitchRate': state_res.PitchRate.tolist(),
            'ThrottlePosition':state_res.ThrottlePosition.tolist(),
            'Airspeed': state_res.Airspeed.tolist(),
            'DeltaCJ': problem.AeroModel.DeltaCJ(state_res).tolist(),
            'ElevatorPosition': state_res.ElevatorPosition.tolist(),
            'EarthXPosition': state_res.EarthXPosition.tolist(),
            'Altitude': state_res.Altitude.tolist()
        }
        extracted_results['state'] = state
        sim_results[int(idx)] = extracted_results.copy()

    print('done')

    import json

    with open(file_path, "w") as json_file:
        json.dump(sim_results, json_file, indent=5)
    plt.show()