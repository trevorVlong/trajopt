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
from dynamics import Aircraft2DPointMass
from matplotlib.pyplot import Figure,axis,axes


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
        self.__WindModel__ = None

        # other problem setup info
        self.Npoints = 100
        self.IndependentAxis: str = 'time'
        self.IndependentValues: Union[float, np.ndarray] = np.zeros((self.Npoints, 0))

        # variable containers for plotting / analysis in post
        self.Time = 0

    @property
    def WindModel(self):
        return self.__WindModel__

    @WindModel.setter
    def WindModel(self, model):
        """
        set wind model at multiple levels
        """
        self.__WindModel__ = model
        self.PhysicsModel.WindModel = model

    def addDynamcis(self,
                    dynamics_model: Aircraft2DPointMass,
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

    def constrainProblem(self,
                         method='cubic'):
        """
        Set problem constraints including start/end conditions, physics constraints, etc

        """

        # ========================================================================
        # constraint problem physics
        # get force, coefficient relationships using aerodynamics model
        L, D, M = self.AeroModel.getForcesAndMoments(self.PhysicsModel)
        cl, cd, cm = self.AeroModel.fullDynamicsModel(self.PhysicsModel)
        thrust = self.AeroModel.thrustModel(self.PhysicsModel)

        # add forces and moments to constraints

        # aero forces
        self.PhysicsModel.add_force(D, L,
                                    from_axes='wind',
                                    )
        self.PhysicsModel.add_moment(M)

        # thrust / propulsor
        self.PhysicsModel.add_force(thrust, 0,
                                    from_axes='body',
                                    )
        self.PhysicsModel.add_moment(thrust * 0.1)

        # gravity
        self.PhysicsModel.add_force(0, self.PhysicsModel.Mass * 9.81,
                                    from_axes='earth',
                                    )

        # ========================================================================
        # add problem constraints (start/end conditions, constraints on variables, etc)

        # for variable in state variable list
        # do add constraint
        # elf.Opti.subject_to(constraint_in_list)

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

    @property
    def Variables(self):
        """
        returns value or combinations of all Opti.Variable combinations
        """

        return self.PhysicsModel.variables | self.AeroModel.variables

    # def plotVariableTrace(self,
    #                       variables: Union[str, list],
    #                       linestyles: Union[str, list] = None,
    #                       colors: Union[str, list] = None,
    #                       markers: Union[str, list] = None,
    #                       linewidths: Union[str, list] = None,
    #                       markersizes: Union[str, list] = None,
    #                       plot_initial_conditions: Union[bool, list] = True,
    #                       plot_constraints: Union[bool, list] = True,
    #                       overlay: bool = False,
    #                       plot_name: str = 'newplot'
    #                       ):
    #     """
    #     plot time and / or position traces of any variable asked for
    #     """
    #
    #     # make sure variables and modifiers all have same length but if
    #     numvars = len(variables)
    #
    #     if len(linestyles) != numvars and linestyles is not None: raise ValueError
    #     if len(colors) != numvars and colors is not None: raise ValueError
    #     if len(markers) != numvars and markers is not None: raise ValueError
    #     if len(plot_initial_conditions) != numvars and plot_initial_conditions is not None: raise ValueError
    #     if len(plot_constraints) != numvars and plot_constraints is not None: raise ValueError
    #     if len(linewidths) != numvars and linewidths is not None: raise ValueError
    #     if len(markersizes) != numvars and linewidths is not None: raise ValueError
    #
    #     # loop through and generate plots
    #
    #     # generate a new plot
    #     for idx, variable in enumerate(variables):
    #
    #         if not overlay:
    #             fig, ax = plotVariable(independentvar, dependentvar,
    #                      )
    #         else: fig,ax = plotVariable()
    #
    #     self.FigDict[f'{fig.}']


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
    N = 100
    problem.Time = np.linspace(0, 100, N)
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
    problem.Opti.subject_to(
        [problem.PhysicsModel.Pitch[-1] >= 340]
    )
    problem.solve()

    print('done')
