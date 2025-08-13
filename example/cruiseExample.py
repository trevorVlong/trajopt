# Created by trevorlong on 8/6/25
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

from problems import AircraftTrajectoryProblem2D as trajp
from weather.WindModel2D import WindModel2D
from aerodynamics import ThinAirfoilModel
from dynamics import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox.numpy.integrate_discrete import integrate_discrete_squared_curvature as int_desc
from typing import Union


def cruiseProblemTime(
        time_array: Union[float,np.ndarray],
        gust_model_velocity: float = 0,
) -> trajp:
    """
    example setup of a cruise problem with a vertical gust
    """
    # initialize problem, add models
    problem = trajp()

    # set up models / containers
    PhysicsModel = Aircraft2DPointMass(
        mass=9,
        Iyy=2,
    )
    # geometry info (still working to make this cleaner)
    PhysicsModel.Span = 3.05
    PhysicsModel.ChordMean = 0.38
    PhysicsModel.TailSpan = 1.27
    PhysicsModel.TailChordMean = 0.25
    PhysicsModel.Area = 1.09
    PhysicsModel.TailArea = 0.2
    AeroModel = ThinAirfoilModel()

    # wind model setup (simple gust)
    wind_model = WindModel2D()
    wind_model.setParameters(model_name='gaussian1D',
                                    **{'STD': 20,
                                       'center': 50,
                                       'MaxGustVelocity': -gust_model_velocity,
                                       'axis': 'z'}
                                    )


    # set state vars

    problem.updateModels(aero_model=AeroModel,
                         rigid_motion_model=PhysicsModel,
                         wind_model=wind_model,
                         )
    problem.Variables['ThrottlePosition'].Freeze=False
    problem.Variables['ThrottlePosition'].setInitialGuess(0.5,len(time_array))
    problem.initializeProblem(
        time=time_array
    )
    # ======
    # general constraints for each variable which I'll include in a setup file later
    dyn = problem.PhysicsModel

    # =================================================================================
    # set problem constraints
    problem.constrainProblem()
    # constrain dynamics


    # Initial Conditions
    problem.subject_to([
        problem.PhysicsModel.Altitude[0] == 200,
        problem.PhysicsModel.EarthXPosition[0] == 0,
        problem.PhysicsModel.PitchRate[0] == 0,
        problem.PhysicsModel.Airspeed[0] >= 10,
        problem.PhysicsModel.AccelXBody[0] == 0,
        problem.PhysicsModel.AccelZBody[0] == 0,
        problem.PhysicsModel.Pitch[0]**2 <= 36
    ])

    # Final Conditions
    problem.subject_to([
        problem.PhysicsModel.PitchRate[-1] == 0,
        (problem.PhysicsModel.Altitude[-1] - problem.PhysicsModel.Altitude[0])**2 <= 100,
        problem.PhysicsModel.AccelXBody[0] == 0,
        problem.PhysicsModel.AccelZBody[0] == 0,
        problem.PhysicsModel.Pitch[-1] ** 2 <= 36

    ])

    # General Constraints
    dThrottle = np.diff(dyn.ThrottlePosition)
    dElevator = np.diff(dyn.ElevatorPosition)
    dTime = np.diff(problem.Time)

    throttle_rate = dThrottle/dTime
    elev_rate = dElevator/dTime

    problem.subject_to([
        throttle_rate**2 <= 0.2,
        elev_rate**2 <= 225,
        problem.PhysicsModel.Altitude >= 50,
    ])






    # optimization problem
    curv = int_desc(dyn.ElevatorPosition, problem.Time) + int_desc(dyn.ThrottlePosition, problem.Time)

    # cost function for the optimizer to work against
    problem.minimize(
        1e-4 * np.sum(curv)
        + np.sum((dyn.Altitude[0]-dyn.Altitude[1:])**2 / 1e2),

    )

    return problem


if __name__=="__main__":

    time_array = np.arange(0,10,.10)
    problem = cruiseProblemTime(time_array)
    problem.solve()

    from dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt

    fig_dict = visualizeRun2D(problem.Time,
                              problem.CurrentSolution(problem.PhysicsModel)
                              )

    # cl,cd,cm = problem.AeroModel.fullDynamicsModel(problem.Solution)
    plt.show()



