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
from warnings import warn
import matplotlib.pyplot as plt


class WindModel2D:
    """
    Class containing the functional representation of a variable number of wind fields
    """
    def __init__(self):

        # model ID
        self.CurrentModel: str = "gaussian1D"
        self.ModelList = [
            "gaussian1D",
            "none",
        ] # list of available models

        # model parameters
        self.STD:float = 0.01
        self.Center:float = 0
        self.MaxGustVelocity:float = 0
        self.Axis:str = "x"

    def setParameters(self,
                      model_name:str='none',
                      **kwargs:dict[str,float],
                      )->None:
        """
        Calibrate the wind model
        :param model_type:
        :param kwargs:
        :return:
        """
        if model_name in self.ModelList:
            self.CurrentModel = model_name
        else:
            warn(f"Model type {model_name} not in list of available models. Consult the list under the attribute "
                 f"self.ModelList for available model names")
        for param_name,param_value in kwargs.items():

            # set standard deviation
            if param_name in ["STD","std","standard_deviation"]:
                self.STD = param_value
            # set average
            if param_name in ["average","avg","Average","AVG","Center","center"]:
                self.Center = param_value
            if param_name in ["maxgustvelocity","GustMax","MaxGustVelocity"]:
                self.MaxGustVelocity = param_value

            if param_name in ["axis"]:
                self.Axis = param_value
            # TODO when more models are added

    def windSpeed(self,
                  x:Union[np.ndarray,float] = 0,
                  y: Union[np.ndarray,float] = 0,
                  z: Union[np.ndarray,float] = 0,
                  ) -> (Union[np.ndarray,float],Union[np.ndarray,float],Union[np.ndarray,float]):
        """
        Get the wind speed a a particular x,y,z location or a series of x,y,z locations. Returns a np.ndarray of
        length N where N is the length of the input array(s). All inputs must have the same length. This function
        does NOT keep track of dimensionality, and will return your inputs in whatever scale and/or mix of scales
        chosen by the user for model parameters.

        :param x: x-position(s)
        :param y: y-position(s)
        :param z: z-position(s)
        :return: tuple of arrays of length N
        """
        # add positions to
        model = {
            "gaussian1D": self._gaussian1D,
            "none": self._nowind,
        }

        return model[self.CurrentModel](x=x,y=y,z=z)

    def _gaussian1D(self,
                    x: float,
                    y: float = 0, # placeholder so that program works
                    z: float = 0,  # placeholder so that program works
                    ) -> tuple[Union[np.ndarray, float],Union[np.ndarray, float],Union[np.ndarray, float]]:
        """
        Creates a 1-dimensional gaussian model of a gust using the mean and standard deviation set by the user in
        setParameters. See details at https://en.wikipedia.org/wiki/Gaussian_function for function information

        :param x:
        :return:
        """

        windspeed = self.MaxGustVelocity * np.exp(-np.power((x - self.Center) / self.STD, 2.0) / 2)  # normalized so equal to airspeed
        # at peak
        zeros = np.zeros(windspeed.shape)
        if self.Axis == "x":
            return windspeed, zeros, zeros
        elif self.Axis == "y":
            return zeros, windspeed, zeros
        elif self.Axis == "z":
            return zeros, zeros, windspeed
        else:
            warn(f"Axis choice {self.Axis} is not valid, must be x,y, or z")

    def _nowind(self,
                x:Union[np.ndarray, float],
                y: Union[np.ndarray, float],
                z: Union[np.ndarray, float],
                ) -> tuple[Union[np.ndarray, float],Union[np.ndarray, float],Union[np.ndarray, float]]:
        """
        """
        wind_x = np.zeros(np.shape(x))
        wind_y = np.zeros(np.shape(y))
        wind_z = np.zeros(np.shape(z))

        return  wind_x, wind_y, wind_z

    def constantWind(self,
                     xrange:Union[np.ndarray,float],
                     yrange:Union[np.ndarray,float],
                     zrange:Union[np.ndarray,float],
                     axis:str = "y"):
        """
        creates a constant wind in the range provided
        :param xrange:
        :param yrange:
        :param zrange:
        :param axis:
        :return:
        """

    def visualize(self,
                  xpoints:np.ndarray,
                  zpoints:np.ndarray,
                  ):
        """
        Plots the wind model for a grid of x-z locations
        :return:
        """

        # xlim and zlim should be 1-D arrays of points
        xgrid,zgrid = np.meshgrid(xpoints,zpoints)


        # get x and z velocities and store into u,w arrays
        uarray = np.zeros(xgrid.shape)
        warray = np.zeros(xgrid.shape)

        for xi,xpoint in enumerate(xgrid[0]):
            for zi,zpoint in enumerate(zgrid[0]):

                u,v,w = self.windSpeed(xpoint,0,zpoint)

                uarray[zi,xi] = u
                warray[zi,xi] = w
        # make quiver plot

        quiv = plt.quiver(xgrid,zgrid,uarray,warray)

        return quiv




if __name__ == "__main__":

    model = WindModel2D()

    model.setParameters('gaussian1D',**{"center": 50, "std": 5, "MaxGustVelocity": 1, "axis": "z"})

    import seaborn as sns
    import matplotlib.pyplot as plt
    # sns.set_theme()

    x_points = np.linspace(0,100,500)
    wind = model.windSpeed(x_points)

    plt.plot(x_points,wind[2])

    xpts = np.linspace(0,100,100)
    zpts = np.linspace(0,100,100)

    model.visualize(xpts,zpts)

    plt.show()

    print('done')
