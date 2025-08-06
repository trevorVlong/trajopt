# Created by trevorlong on 8/5/25
# license
# Copyright 2025 trevorlong
import aerosandbox
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from aerosandbox import Opti
from aerosandbox import numpy as np
from typing import Union
from dynamics.Aircraft2D import Aircraft2DPointMass


class TrajectoryProblem2D():
    """
    A class which provides a framework for setting up trajectory optimizaiton problems using the AeroSandBox Opti
    API. Provides methods for setting up these problems so that users do not need to interface with lower-level info
    """

    def __init__(self):

        # optimzier releated
        self.Opti = Opti()
        self.Solution = None
        self.SolutionState = "not-initialized"

        self._Constraints = dict()
        self._InitialConditions = dict()
        self._EndConditions = dict()

        # properties related to the problem models
        self.PhysicsModel = None
        self.AeroModel = None
        self.WindModel = None

        # other problem setup info
        self.Npoints = 100
        self.IndependentAxis: str = 'time'
        self.IndependentValues: Union[float,np.ndarray] = np.zeros((self.Npoints,0))

        # variable container
        self.Variables = {}
        self.Time = 0

    def addDynamcis(self,
                        dynamics_model:Aircraft2DPointMass,
                        aerodynamics_model
                        ):
        """
        Add dynamcis and aerodynamics models to the problem and set up containers for variables using state/control
        variable lists.
        """

        # add dynamics model, set up state / control vars from that model
        self.PhysicsModel = dynamics_model
        self.Variables.fromkeys(list(dynamics_model.state.keys())
                                + list(dynamics_model.control_variables.keys())
                                )

        self.AeroModel = aerodynamics_model

    def setVariables(self,
                     variables:dict[str,dict],
                     ) -> None:
        """
        Sets up state and opti variables from incoming dict
        """

        for key,var_dict in variables.items():
            for param, param_val in var_dict.items():
                self.PhysicsModel.addVariables(optiVar)

    def constrainProblem(self,
                         method='cubic'):
        """
        Set problem constraints including start/end conditions, physics constraints, etc

        """

        # ========================================================================
        # constraint problem physics
        # get force, coefficient relationships using aerodynamics model
        L,D,M = self.AeroModel.getForcesAndMoments(self.PhysicsModel)
        cl,cd,cm = self.AeroModel.fullDynamicsModel(self.PhysicsModel)
        thrust = self.AeroModel.thrustModel(self.PhysicsModel)

        # add forces and moments to constraints

        # aero forces
        self.PhysicsModel.add_force(D,L,
                                    from_axes='wind',
                                    )
        self.PhysicsModel.add_moment(M)

        # thrust / propulsor
        self.PhysicsModel.add_force(thrust, 0,
                                    from_axes='body',
                                    )
        self.PhysicsModel.add_moment(thrust*0.1)

        # gravity
        self.PhysicsModel.add_force(0, self.PhysicsModel.Mass * 9.81,
                                    from_axes='earth',
                                    )

        # ========================================================================
        # add problem constraints (start/end conditions, constraints on variables, etc)

        # for variable in state variable list
        # do add constraint
        #elf.Opti.subject_to(constraint_in_list)

        # for variable in control variable list
        # do add constraint

        # finally, constraint derivatives using assigned method
        self.PhysicsModel.constrain_derivatives(self.Opti,
                                                 self.Time,
                                                 method=method,
                                                 )

    def solve(self,
              max_iter: int = 1000,
              max_runtime: float = 120,
              verbose: bool = True,
              behavior_on_failure: str = 'return_last',
              visualize: bool = True,
              **options,
              ) -> None:
        """
        Run opti.solve() and depending on outcome update output containers appropriately
        """

        # try solve
        try:
            sol = self.Opti.solve(
                max_iter=max_iter,
                behavior_on_failure=behavior_on_failure,
                verbose=verbose,
            )

            # update solution state, propogate values
            self.SolutionState = 'Converged'
            self.Solution = sol(self.PhysicsModel)
            self.Time = sol(self.Time)

        except RuntimeError:
            # get most recent solution and update variable arrays
            self.SolutionState = 'Error'
            self.Solution = sol(self.PhysicsModel)
            self.Time = sol(self.Time)


if __name__ == "__main__":
    from weather.WindModel2D import WindModel2D
    from aerodynamics.SimpleAircraft2D import ThinAirfoilModel
    # set up models / containers
    problem = TrajectoryProblem2D()
    problem.PhysicsModel = Aircraft2DPointMass(
        mass=9,
        Iyy=2,
    )
    problem.PhysicsModel.Span = 3.05
    problem.PhysicsModel.ChordMean = 0.38
    problem.PhysicsModel.TailSpan = 1.27
    problem.PhysicsModel.TailChordMean = 0.25
    problem.PhysicsModel.Area = 1.09
    problem.PhysicsModel.TailArea = 0.2
    problem.WindModel = WindModel2D()
    problem.AeroModel = ThinAirfoilModel()
    # add opti
    N = 11
    problem.Time = np.linspace(0,10,N)
    init_guess = np.ones((N,))
    variables = {
        'EarthXPosition': problem.Opti.variable(init_guess=init_guess),
        'EarthZPosition': problem.Opti.variable(init_guess=init_guess),
        'BodyXVelocity': problem.Opti.variable(init_guess=init_guess),
        'BodyZVelocity': problem.Opti.variable(init_guess=init_guess),
        'Pitch': problem.Opti.variable(init_guess=init_guess),
        'PitchRate': problem.Opti.variable(init_guess=init_guess),
        'FlapPosition': problem.Opti.variable(init_guess=init_guess),
        'ElevatorPosition': problem.Opti.variable(init_guess=init_guess),
        'ThrottlePosition': problem.Opti.variable(init_guess=init_guess),
    }
    problem.PhysicsModel.setVariables(variables)

    problem.constrainProblem()

    problem.solve()

    print('done')

