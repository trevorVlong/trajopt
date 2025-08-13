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


def cruiseGustComparison(gust_velocity_vec,
                         time_vec):


    # setup and run the OG case (no gust)
    problem = Trajprob()
    aeromodel = ThinAirfoilModel()
    dyn_model = Aircraft2DPointMass(mass=10, Iyy=2)

    problem.initializeProblem(
        dynamics_model=aeromodel,
        rigid_motion_model=dyn_model,
        time=time_vec
    )
    problem.WindModel.setParameters(model_name='gaussian1D',
                                     **{'STD': 20,
                                        'center': 100,
                                        'MaxGustVelocity': 0,
                                        'axis': 'z'}
                                     )
    # run each of the other cases
    problem_dict = {}
    for case_num in len(gust_velocity_vec):
        new_prob = problem.copy()
        problem_dict[f'wind_vel_{gust_velocity_vec[case_num]}'] = new_prob
        new_prob.WindModel.setParameters(model_name='gaussian1D',
                                        **{'STD': 20,
                                           'center': 100,
                                           'MaxGustVelocity': -gust_velocity_vec[case_num],
                                           'axis': 'z'}
                                        )
        # run case
        new_prob.solve()

    return



if __name__ == "__main__":

    # set up multiple problems
    problem1 = Trajprob()

    # set up the aero model
    aeromodel = ThinAirfoilModel()

    # set up dynamics model
    dyn_model = Aircraft2DPointMass(mass=10,Iyy=2)

    problem1.initializeProblem(dynamics_model=dyn_model,
                               )

    # copy setup
    problem2 = problem1.copy()
    problem2.WindModel.setParameters(model_name='gaussian1D',
                                    **{'STD': 20,
                                       'center': 100,
                                       'MaxGustVelocity': -5,
                                       'axis': 'z'}
                                    )
    problem3 = problem1.copy()
    problem3.WindModel.setParameters(model_name='gaussian1D',
                                     **{'STD': 20,
                                        'center': 100,
                                        'MaxGustVelocity': -5,
                                        'axis': 'z'}
                                     )
    problem4 = problem1.copy()
    problem4.WindModel.setParameters(model_name='gaussian1D',
                                     **{'STD': 20,
                                        'center': 100,
                                        'MaxGustVelocity': -5,
                                        'axis': 'z'}
                                     )
