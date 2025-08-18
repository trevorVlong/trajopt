# Created by trevorlong on 6/18/25
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

from abc import ABC, abstractmethod
from typing import Union, TYPE_CHECKING, Tuple
from aerosandbox import numpy as np

if TYPE_CHECKING:
    from trajopt.dynamics import Aircraft2DPointMass


class AeroModel(ABC, object):
    """
    Abstract class for aerodynamics model intended to solve for forces and moments on a pointmass dynamics model that
    uses CASADi variables. Abstract class defines the required methods for the
    """

    @abstractmethod
    def __init__(self, *args):
        pass

    @abstractmethod
    def getForcesAndMoments(self, dynModel):
        pass

    @abstractmethod
    def wingDynamicsModel(self, dynModel):
        pass

    @abstractmethod
    def tailDynamicsModel(self, dynModel):
        pass

    @abstractmethod
    def fullDynamicsModel(self, dynModel):
        pass


class BaseModel(AeroModel):
    """
    Example model
    """

    def __init__(self):
        # wing model subfunctions should be stored in class for easy access
        self.WingLiftCoefficientFunc = lambda x: x
        self.WingDragCoefficientFunc = lambda x: x
        self.WingMomentCoefficientFunc = lambda x: x

        # tail model subfunctions should be stored in class for easy access
        self.TailLiftCoefficientFunc = lambda x: x
        self.TailDragCoefficientFunc = lambda x: x
        self.TailMomentCoefficientFunc = lambda x: x

    def wingDynamicsModel(self,
                          dynModel: "Aircraft2DPointMass",
                          ) ->\
        Tuple[
        Union[float, np.ndarray],
        Union[float, np.ndarray],
        Union[float, np.ndarray]
    ]:
        pass

    def tailDynamicsModel(self,
                          dynModel: "Aircraft2DPointMass",
                          ) ->\
        Tuple[
        Union[float, np.ndarray],
        Union[float, np.ndarray],
        Union[float, np.ndarray]
    ]:
        pass

    def fullDynamicsModel(self,
                          dynModel: "Aircraft2DPointMass",
                          ) ->\
        Tuple[
        Union[float, np.ndarray],
        Union[float, np.ndarray],
        Union[float, np.ndarray]
    ]:
        pass

    def getForcesAndMoments(self,
                          dynModel: "Aircraft2DPointMass",
                          ) ->\
        Tuple[
        Union[float, np.ndarray],
        Union[float, np.ndarray],
        Union[float, np.ndarray]
    ]:
        pass


if __name__ == "__main__":
    c = BaseModel()

    print('done')
