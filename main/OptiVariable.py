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


class OptiStateVariable:
    """
    Structured data class used to manage setup and execution of setting up asb.Opti() variables. Keeps track of
    initial / end states, limits, constraints, and uses these to make
    """

    def __init__(self):
        self.Name = None
        self.Type = None
        self.Value = None
        self.InitialGuess = None
        self.LowerLimit = None
        self.UpperLimit = None
        self.InitialCondition = None
        self.EndCondition = None
        self.Constraints = None

    def setOptiVar(self,
                   opti: aerosandbox.Opti,
                   n_vars: int = None,
                   scale: float = None,
                   freeze: bool = False,
                   log_transform: bool = False,
                   category: str = "Uncategorized",
                   lower_bound: float = None,
                   upper_bound: float = None,
                   _stacklevel: int = 1,
                   ):
        """
        Create an opti.variable with defined
        """

        if self.InitialGuess is None:
            raise ValueError(f"An initial guess is required to initialize opti variables")

        for key,value in self.__dict__.items():
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

