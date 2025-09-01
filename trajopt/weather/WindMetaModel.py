# Created by trevorlong on 7/30/24
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
from typing import Union
from WindModel2D import WindModel2D


class WindMetaModel(object):
    """
    This is a meta-model which contains a series of WindModel classes which are used to generate a wind field for
    simulations. The purpoes of this class is to
    """

    def __init__(self):

        self.ModelDict: dict[str, WindModel2D] = {}

    def addModel(self,
                 name: str,
                 model: WindModel2D,
                 ) -> None:
        """
        add model to the model dict
        :param model:
        :return:
        """

        self.ModelDict[name] = model

    def windSpeed(self,
                  x: Union[np.ndarray,float] = 0,
                  y: Union[np.ndarray,float] = 0,
                  z: Union[np.ndarray,float] = 0,
                  ) -> (Union[np.ndarray,float],Union[np.ndarray,float],Union[np.ndarray,float]):
        """
        Create the meta wind model from all submodels and return a 3x tuple of x windspeed, y- and z-
        :return:
        """
        shape = np.shape(x)
        x_vel = np.zeros(shape)
        y_vel = np.zeros(shape)
        z_vel = np.zeros(shape)

        for key,model in self.ModelDict.items():
            velocities = model.windSpeed(x,y,z)
            x_vel = x_vel + velocities[0]
            y_vel = y_vel + velocities[1]
            z_vel = z_vel + velocities[2]

        return x_vel,y_vel,z_vel

    def removeModel(self,
                    model_name:str,
                    )->None:
        """
        remove a model from the dictionary
        :param model_name:
        :return:
        """

        self.ModelDict.pop(model_name)


if __name__ == "__main__":
    metamodel = WindMetaModel()
