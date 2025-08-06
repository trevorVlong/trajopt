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
from dynamics.Aircraft2D import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox.numpy.integrate_discrete import integrate_discrete_squared_curvature as int_desc


def cruiseProblem() -> TrajectoryProblem2D:
    """
    example setup of a cruise problem with a vertical gust
    """

    # initialize problem, add models
    problem = TrajectoryProblem2D()

    # set up models / containers
    problem = TrajectoryProblem2D()
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
    problem.WindModel.setParameters(model_name='gaussian1D',
                                    **{'STD': 5,
                                       'center': 40,
                                       'MaxGustVelocity': -5,
                                       'axis': 'z'}
                                    )


    # set state vars
    N = 11
    problem.Time = np.linspace(0, 10, N)
    init_guess = np.ones((N,))
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
                                              freeze=True,
                                              ),
        'ElevatorPosition': problem.Opti.variable(init_guess=init_guess,
                                                  lower_bound=-20,
                                                  upper_bound=20,
                                                  ),
        'ThrottlePosition': problem.Opti.variable(init_guess=init_guess,
                                                  lower_bound=0,
                                                  upper_bound=1,
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
        problem.PhysicsModel.Pitch[0]**2 <=2,
        problem.PhysicsModel.Airspeed[0] == 10,
    ])

    # Final Conditions
    problem.Opti.subject_to([
        problem.PhysicsModel.Altitude[-1] == 200,
        problem.PhysicsModel.Pitch[-1]**2 <= 2
    ])

    # General Constraints
    dThrottle = np.diff(dyn.ThrottlePosition)
    dTime = np.diff(problem.Time)

    throttle_rate = dThrottle/dTime

    problem.Opti.subject_to([
        throttle_rate**2 <= 0.09
    ])


    # constrain dynamics
    problem.constrainProblem()



    # optimization problem
    curv = int_desc(dyn.ElevatorPosition / 25, problem.Time) + int_desc(dyn.ThrottlePosition, problem.Time)

    # cost function for the optimizer to work against
    problem.Opti.minimize(
        1 * np.sum(curv)
    )

    # get solution
    problem.solve(max_iter=2000)

    return problem


if __name__=="__main__":

    problem = cruiseProblem()

    from dynamics.visualization import visualizeRun2D

    fig_dict = visualizeRun2D(problem.Time,
                              problem.Solution
                              )

