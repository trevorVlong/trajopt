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
from typing import Union, List, Dict, Callable, Any, TYPE_CHECKING
from dynamics import Aircraft2DPointMass
from weather import WindModel2D
from dynamics import PointMass2D
from copy import deepcopy
import casadi as cas

class AircraftTrajectoryProblem2D(Opti):
    """
    A class which abstracts away the setup for trajectory optimization problems involving aircraft. For
    details on usage see method
    docstrings and the Aerosandbox.Opti superclass documentation
    """

    def __init__(self,
                 variable_categories_to_freeze: Union[List[str], str] = None,
                 cache_filename: str = None,
                 load_frozen_variables_from_cache: bool = False,
                 save_to_cache_on_solve: bool = False,
                 ignore_violated_parametric_constraints: bool = False,
                 freeze_style: str = "parameter",
                 ):

        # initialize parent opti class
        super().__init__(variable_categories_to_freeze,
                         cache_filename,
                         load_frozen_variables_from_cache,
                         save_to_cache_on_solve,
                         ignore_violated_parametric_constraints,
                         freeze_style,
                         )

        self._Constraints = dict()
        self._InitialConditions = dict()
        self._EndConditions = dict()

        # initialize sub-modules, specific problem set-ups will most likely have you over-write one or the other. See
        # submodules for implementation details
        self.PhysicsModel = Aircraft2DPointMass()  # model for rigid motion of aircraft
        self.AeroModel = None  #
        self.WindModel = WindModel2D()

        # other problem setup info
        self.Npoints = 100
        self.IndependentAxis: str = 'time'
        self.IndependentValues: Union[float, np.ndarray] = np.zeros((self.Npoints, 0))

        # variable containers for plotting / analysis in post
        self.Time = 0

        # solution storage
        self.LastSolution = None
        self.CurrentSolution = None
        self.Variables = None

    def initializeProblem(self,
                          dynamics_model,
                          time: Union[Union[float, np.ndarray]],
                          rigid_motion_model: Union[None, PointMass2D, Aircraft2DPointMass],
                          initial_guesses:Union[None, Dict] = None,
                          ):
        """
        Configure the problem using a dynamics model and a rigid motion model.
        """

        # set models
        self.AeroModel = dynamics_model
        if rigid_motion_model is not None:
            self.PhysicsModel = rigid_motion_model

        # set up initial guesses for all optimization variables
        self.Time = time
        default_limits = self.PhysicsModel.defaultVariableConfiguration
        for var_name in self.PhysicsModel._OptiVars.keys():
            limits = default_limits[var_name]
            if initial_guesses is None:
                setattr(self.PhysicsModel,
                        var_name,
                        self.variable(
                            init_guess=np.ones(time.shape),
                            upper_bound=limits.UpperLimit,
                            lower_bound=limits.LowerLimit
                        )
                )


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
        self.PhysicsModel.constrain_derivatives(self,
                                                self.Time,
                                                method=method,
                                                )

    def solve(self,
              parameter_mapping: Dict[cas.MX, float] = None,
              max_iter: int = 2000,
              max_runtime: float = 1e20,
              callback: Callable[[int], Any] = None,
              verbose: bool = True,
              jit: bool = False,  # see Opti documentation
              detect_simple_bounds: bool = False,  # see Opti documentation
              expand: bool = False,  # see Opti documentation
              options: Dict = None,  # see Opti documentation
              behavior_on_failure: str = "return_last",
              ):

        self.LastSolution = deepcopy(self.CurrentSolution)
        sol = super().solve(
                parameter_mapping,
                max_iter,
                max_runtime,
                callback,
                verbose,
                jit,
                detect_simple_bounds,
                expand,
                options,
                behavior_on_failure
            )

        # store the last solve and update initial guesses from those values. There is a Opti.solve_sweep() but it may
        # not always be the case that you want to run this in a sweep
        self.CurrentSolution = sol
        self.set_initial_from_sol(self.CurrentSolution)

if __name__ == "__main__":
    from aerodynamics.SimpleAircraft2D import ThinAirfoilModel

    # set up models / containers
    problem = AircraftTrajectoryProblem2D()
    PhysicsModel = Aircraft2DPointMass(
        mass=9,
        Iyy=2,
    )
    PhysicsModel.Span = 3.05
    PhysicsModel.ChordMean = 0.38
    PhysicsModel.TailSpan = 1.27
    PhysicsModel.TailChordMean = 0.25
    PhysicsModel.Area = 1.09
    PhysicsModel.TailArea = 0.2
    WindModel = WindModel2D()
    # add opti
    N = 100
    time = np.linspace(0, 100, N)
    problem.initializeProblem(
        dynamics_model=ThinAirfoilModel(),
        rigid_motion_model=PhysicsModel,
        time=time,
    )
    problem.constrainProblem()
    problem.subject_to(
        [problem.PhysicsModel.Pitch[-1] >= 340]
    )
    problem.solve()

    # run again with new initial guess
    problem.set_initial(problem.PhysicsModel.EarthXPosition,100 * np.ones(problem.Time.shape))
    problem.solve()

    print('done')  # for manual debug