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

from trajopt.main import AircraftTrajectoryProblem2D as trajp
from trajopt.weather.WindModel2D import WindModel2D
from trajopt.aerodynamics import BlownAirfoilModel
from trajopt.dynamics import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox.numpy.integrate_discrete import integrate_discrete_squared_curvature as int_desc
from typing import Union,TYPE_CHECKING

if TYPE_CHECKING:
    from trajopt.main import Problem


if TYPE_CHECKING:
    import casadi as cas


def landingProblemTime(
        problem: "Problem",
        time_array: Union[float,np.ndarray],
        parameters:dict[str,"cas.MX"]
) -> trajp:
    """
    example setup of a cruise problem with a vertical gust
    """
    # initialize problem, add models

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
    PhysicsModel.PropulsorArea = 0.06

    AeroModel = BlownAirfoilModel.BlownAirfoilModel()
    # wind model setup (simple gust)
    wind_model = WindModel2D()

    wind_model.setParameters(model_name='gaussian1D',
                                    **{'STD': 10,
                                       'center': 75,
                                       'MaxGustVelocity': -parameters['gust_vel'],
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
    # settings
    problem.PhysicsModel.FlapPosition = parameters['FlapAngle']
    # =================================================================================
    # set problem constraints
    problem.constrainProblem()
    # constrain dynamics


    # Initial Conditions
    problem.subject_to([
        dyn.Altitude[0] == parameters['InitialAltitude'],
        dyn.EarthXPosition[0] == parameters['InitialXPosition'],
        dyn.Airspeed[0] == parameters['InitialXVelocity'],
        dyn.BodyZVelocity[0]**2 > 0.00001,
        # problem.PhysicsModel.Fz_b[0]**2<=0.1,
    ])

    # Final Conditions
    problem.subject_to([
        dyn.PitchRate[-1]**2 <= 2,
        dyn.Pitch[-1] >= 0,
        dyn.Altitude[-1]<=0.5
    ])

    # General Constraints
    dThrottle = np.diff(dyn.ThrottlePosition)
    dElevator = np.diff(dyn.ElevatorPosition)
    dTime = np.diff(problem.Time)
    throttle_rate = dThrottle/dTime
    elev_rate = dElevator/dTime

    problem.subject_to([
        throttle_rate**2 < 0.6,
        dyn.ThrottlePosition < 1,
        dyn.ThrottlePosition > 0.05,
        elev_rate**2 <= 25**2,
        problem.PhysicsModel.Altitude >= 0,
        dyn.Airspeed > 5
    ])

    # optimization problem
    curv = np.sum(int_desc(dyn.ElevatorPosition, problem.Time)
            + int_desc(dyn.ThrottlePosition, problem.Time)
            + int_desc(dyn.Pitch, problem.Time)
            )

    # cost function for the optimizer to work against
    problem.minimize(
        1e-4 * curv
        + dyn.EarthXPosition[-1]**2
    )

    return problem


if __name__=="__main__":

    time_array = np.arange(0,10,.10)
    problem = landingProblemTime(time_array)
    problem.solve()

    from trajopt.dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt

    fig_dict = visualizeRun2D(problem.Time,
                              problem.CurrentSolution(problem.PhysicsModel)
                              )

    # cl,cd,cm = problem.AeroModel.fullDynamicsModel(problem.Solution)
    plt.show()



