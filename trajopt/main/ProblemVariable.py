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

import matplotlib.pyplot as plt
from typing import Union, List, TYPE_CHECKING
from aerosandbox import numpy as np
if TYPE_CHECKING:
    import casadi as cas


class Variable:
    """
    Structured data class used to make generating aerosandbox.Opti() variables a little more streamlined for
    trajectory optimizaiton problem.
    """

    def __init__(self,
                 name: str = 'default_variable',
                 variable_type: str = 'default',
                 initial_guess: Union[float, np.ndarray] = 0,
                 initial_condition: Union[float, None] = None,
                 end_condition: Union[float, None] = None,
                 lower_limit: Union[float, None] = None,
                 upper_limit: Union[float, None] = None,
                 constraints: Union['cas.MX', None, 'List[cas.MX]'] = None,
                 freeze: Union[bool, None] = False,
                 ):

        # Identifying information
        self.Name: str = name
        self.Type = variable_type

        # bondary and initial conditions
        self.Value = initial_guess
        self.InitialGuess = initial_guess
        self.InitialCondition = initial_condition
        self.EndCondition = end_condition
        self.Freeze = freeze

        # limits and constraints
        self.LowerLimit = lower_limit
        self.UpperLimit = upper_limit
        self.Constraints = constraints

    def setInitialGuess(self,
                        value:Union[float, np.ndarray],
                        array_length: Union[None, int] = None,
                        ):
        """
        set the initial guess of the parameter as either an array or as a constant value
        """

        if array_length is None:
            if type(value) is not np.ndarray:
                raise ValueError('initial guess must be a numpy array')
            self.InitialGuess = value
        else:
            if type(value) is not float:
                raise ValueError('initial guess must be a float')
            self.InitialGuess = value*np.ones((array_length,))

if __name__ == "__main__":

    N = 10
    t = np.linspace(0,10,N)
    default_val = np.ones((N,))

    var = Variable(
        name='var1',
        initial_guess=default_val
    )
    var2 = Variable(
        name='var2',
        initial_guess= 2*default_val
    )

    fig,line = var.plot(t)
    fig,newline = var2.plot(t,
                            fig=fig
                            )

    plt.legend()
    plt.show()