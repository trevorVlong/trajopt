# Created by trevorlong on 6/14/24
# license
# Copyright 2024 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import aerosandbox.numpy as np
from typing import Union, Dict, Tuple, Callable, List
from aerosandbox import Opti
class Oscillator1D():
    """
    Class for playing around with the optimizer using a 1-D oscillator. Models a mass in 1-dimension with

        * mass and damping inputs
        * 1
    """

    def __init__(self,
                 mass: float = 1,
                 xpos: Union[float, np.array] = 0,
                 xvel: Union[float, np.array] = 0,

                 ):

        # inertial characteristics
        self.mass = mass

        # state vars
        self.xpos = xpos
        self.xvel = xvel

        # control vars
        self.Fx = 0

    @property
    def state(self) -> Dict[str, Union[float, np.ndarray]]:
        return {
            "xpos": self.xpos,
            "xvel": self.xvel,
        }

    @property
    def control_variables(self) -> Dict[str, Union[float, np.ndarray]]:
        return {
            "Fx": self.Fx,
        }

    def state_derivatives(self) -> Dict[str, Union[float, np.ndarray]]:

        a_x = self.a_x

        return {
            "xpos": self.xvel,
            "xvel": a_x
        }

    @property
    def a_x(self) -> Union[float, np.ndarray]:

        return self.Fx / self.mass

    def addForce(self, F: Union[float, np.ndarray]):
        """
        adds a forcing function to the axis
        :param F:
        :return:
        """
        # no rotation needed
        self.Fx = self.Fx + F

    def constrain_derivatives(self,
                              opti: Opti,
                              time: np.ndarray,
                              method: str = "trapezoidal",
                              which: Union[str, List[str], Tuple[str]] = "all",
                              _stacklevel=1,
                              ):
        """
        Applies the relevant state derivative constraints to a given Opti instance.

        Args:

            opti: the AeroSandbox `Opti` instance that constraints should be applied to.

            time: A vector that represents the time at each discrete point. Should be the same length as any
            vectorized state variables in the `state` property of this Dynamics instance.

            method: The discrete integration method to use. See Opti.constrain_derivative() for options.

            which: Which state variables should be we constrain? By default, constrains all of them.

                Options:

                    * "all", which constrains all state variables (default)

                    * A list of strings that are state variable names (i.e., a subset of `dyn.state.keys()`),
                    that gives the names of state variables to be constrained.

            _stacklevel: Optional and advanced, purely used for debugging. Allows users to correctly track where
            constraints are declared in the event that they are subclassing `aerosandbox.Opti`. Modifies the
            stacklevel of the declaration tracked, which is then presented using
            `aerosandbox.Opti.variable_declaration()` and `aerosandbox.Opti.constraint_declaration()`.

        Returns:

        """
        if which == "all":
            which = self.state.keys()

        state_derivatives = self.state_derivatives()

        for state_var_name in which:

            # If a state derivative has a None value, skip it.
            if state_derivatives[state_var_name] is None:
                continue

            # Try to constrain the derivative
            try:
                opti.constrain_derivative(
                    derivative=state_derivatives[state_var_name],
                    variable=self.state[state_var_name],
                    with_respect_to=time,
                    method=method,
                    _stacklevel=_stacklevel + 1
                )
            except KeyError:
                raise ValueError(f"This dynamics instance does not have a state named '{state_var_name}'!")
            except Exception as e:
                raise ValueError(f"Error while constraining state variable '{state_var_name}': \n{e}")


if __name__ == "__main__":
    dyn = Oscillator1D()