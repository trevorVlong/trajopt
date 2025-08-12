# Created by trevorlong on 8/5/25
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

import aerosandbox
import matplotlib.pyplot as plt
from typing import Union,List
from aerosandbox import numpy as np
import casadi as cas


class ProblemVariable:
    """
    Structured data class used to manage setup and execution of setting up asb.Opti() variables. Keeps track of
    initial / end states, limits, constraints, and uses these to make
    """

    def __init__(self,
                 name: str = 'default_variable',
                 variable_type: str = 'default',
                 initial_guess: Union[float, np.ndarray] = 0,
                 initial_condition: Union[float, None] = None,
                 end_condition: Union[float, None] = None,
                 lower_limit: Union[float, None] = None,
                 upper_limit: Union[float, None] = None,
                 constraints: Union[cas.MX, None, List[cas.MX]] = None
                 ):

        # Identifying information
        self.Name: str = name
        self.Type = variable_type

        # bondary and initial conditions
        self.Value = initial_guess
        self.InitialGuess = initial_guess
        self.InitialCondition = initial_condition
        self.EndCondition = end_condition

        # limits and constraints
        self.LowerLimit = lower_limit
        self.UpperLimit = upper_limit
        self.Constraints = constraints

    def setOptiVar(self,
                   opti: aerosandbox.Opti,
                   _stacklevel: int = 1,
                   ):
        """
        Create an opti.variable with defined
        """

        if self.InitialGuess is None:
            raise ValueError(f"An initial guess is required to initialize opti variables")

        for key, value in self.__dict__.items():
            if value is None:
                if key == 'LowerLimit':
                    self.LowerLimit = -999
                    raise Warning('Lower limit for variable set to -999 without user input')
                if key == 'UpperLimit':
                    self.UpperLimit = 999
                    raise Warning('Upper limit for variable set to 999 without user input')

        return opti.variable(
            init_guess=self.InitialGuess,
            upper_bound=self.UpperLimit,
            lower_bound=self.LowerLimit,
        )

    def plot(self,
             independent_var: Union[float,np.ndarray],
             constraints: bool = False,
             limits: bool = False,
             boundary_conditions: bool = False,
             fig: Union[plt.Figure, None] = None,
             **plot_options
             ):
        """
        plot a trace of this variable against the array fed in
        """

        # =======================================================
        # default values for certain params

        if 'linewidth' not in plot_options.keys():
            plot_options['linewidth'] = 1.5


        # =======================================================
        # generate plot
        if fig is None:
            fig = plt.figure()
            plt.grid()
        line = plt.plot(
            independent_var, self.Value,
            label=self.Name,
            **plot_options
        )

        return fig, line


if __name__ == "__main__":

    N = 10
    t = np.linspace(0,10,N)
    default_val = np.ones((N,))

    var = ProblemVariable(
        name='var1',
        initial_guess=default_val
    )
    var2 = ProblemVariable(
        name='var2',
        initial_guess= 2*default_val
    )

    fig,line = var.plot(t)
    fig,newline = var2.plot(t,
                            fig=fig
                            )

    plt.legend()
    plt.show()