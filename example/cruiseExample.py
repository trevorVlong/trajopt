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

from main.Problem import TrajectoryProblem2D
from weather.WindModel2D import WindModel2D
from aerodynamics.SimpleAircraft2D import ThinAirfoilModel
from dynamics import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox.numpy.integrate_discrete import integrate_discrete_squared_curvature as int_desc
from typing import Union


def cruiseProblemTime(
        time_array: Union[float,np.ndarray]
) -> TrajectoryProblem2D:
    """
    example setup of a cruise problem with a vertical gust
    """
    Npoints = len(time_array)
    # initialize problem, add models
    problem = TrajectoryProblem2D()

    # set up models / containers
    problem.PhysicsModel = Aircraft2DPointMass(
        mass=9,
        Iyy=2,
    )
    # geometry info (still working to make this cleaner)
    problem.PhysicsModel.Span = 3.05
    problem.PhysicsModel.ChordMean = 0.38
    problem.PhysicsModel.TailSpan = 1.27
    problem.PhysicsModel.TailChordMean = 0.25
    problem.PhysicsModel.Area = 1.09
    problem.PhysicsModel.TailArea = 0.2
    problem.WindModel = WindModel2D()
    problem.AeroModel = ThinAirfoilModel()

    # wind model setup (simple gust)
    problem.WindModel.setParameters(model_name='gaussian1D',
                                    **{'STD': 20,
                                       'center': 100,
                                       'MaxGustVelocity': 0,
                                       'axis': 'z'}
                                    )


    # set state vars
    problem.Time = time_array
    init_guess = np.ones((Npoints,))
    variables = {
        'EarthXPosition': problem.Opti.variable(init_guess=init_guess,
                                                lower_bound=0,
                                                upper_bound=1e5,
                                                ),
        'EarthZPosition': problem.Opti.variable(init_guess=init_guess,
                                                upper_bound=0,
                                                lower_bound=-1e5,
                                                ),
        'BodyXVelocity': problem.Opti.variable(init_guess=init_guess,
                                               lower_bound=0,
                                               upper_bound=1e2,
                                               ),
        'BodyZVelocity': problem.Opti.variable(init_guess=init_guess,
                                               lower_bound=-1e1,
                                               upper_bound=1e1,
                                               ),
        'Pitch': problem.Opti.variable(init_guess=init_guess,
                                       lower_bound=-45,
                                       upper_bound=45,
                                       ),
        'PitchRate': problem.Opti.variable(init_guess=init_guess,
                                           lower_bound=-20,
                                           upper_bound=20,
                                           ),
        'FlapPosition': problem.Opti.variable(init_guess=init_guess,
                                              lower_bound=0,
                                              upper_bound=50,
                                              freeze=False,
                                              ),
        'ElevatorPosition': problem.Opti.variable(init_guess=init_guess,
                                                  lower_bound=-20,
                                                  upper_bound=20,
                                                  ),
        'ThrottlePosition': problem.Opti.variable(init_guess= 0.5*init_guess,
                                                  lower_bound=0,
                                                  upper_bound=1,
                                                  freeze=False
                                                  ),
    }
    problem.PhysicsModel.setVariables(variables)

    # =================================================================================
    # set problem constraints
    dyn = problem.PhysicsModel

    # Initial Conditions
    problem.Opti.subject_to([
        problem.PhysicsModel.Altitude[0] == 200,
        problem.PhysicsModel.EarthXPosition[0] == 0,
        problem.PhysicsModel.PitchRate[0] == 0,
        problem.PhysicsModel.Pitch[0] >= 0,
        problem.PhysicsModel.ThrottlePosition[0] == 0.4
    ])

    # Final Conditions
    problem.Opti.subject_to([
        problem.PhysicsModel.Altitude[-1] == 250,
        problem.PhysicsModel.Pitch[-1]**2 <= 4,

    ])

    # General Constraints
    dThrottle = np.diff(dyn.ThrottlePosition)
    dElevator = np.diff(dyn.ElevatorPosition)
    dTime = np.diff(problem.Time)

    throttle_rate = dThrottle/dTime
    elev_rate = dElevator/dTime

    problem.Opti.subject_to([
        throttle_rate**2 <= 0.2,
        elev_rate**2 <= 225,
        dyn.Airspeed < 15
    ])


    # constrain dynamics
    problem.constrainProblem()



    # optimization problem
    curv = int_desc(dyn.ElevatorPosition, problem.Time) + int_desc(dyn.ThrottlePosition, problem.Time)

    # cost function for the optimizer to work against
    problem.Opti.minimize(
        1e-4 * np.sum(curv)
    )

    # get solution
    problem.solve(max_iter=2000)

    return problem


if __name__=="__main__":

    time_array = np.arange(0,20,.25)
    problem = cruiseProblemTime(time_array)

    from dynamics.visualization import visualizeRun2D
    import matplotlib.pyplot as plt

    fig_dict = visualizeRun2D(problem.Time,
                              problem.Solution
                              )

    cl,cd,cm = problem.AeroModel.fullDynamicsModel(problem.Solution)

    plt.figure()

    plt.plot(problem.Time,problem.Solution.BodyXVelocity,label='x')
    plt.plot(problem.Time, problem.Solution.BodyZVelocity,label='z')
    plt.plot(problem.Time,cl)
    print(cl)
    plt.legend()
    plt.show()



