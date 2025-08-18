# Created by trevorlong on 7/15/24
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

from typing import Union
import aerosandbox as asb
import aerosandbox.numpy as np
import matplotlib.pyplot as plt


class Airfoil2DSection:
    """
    Performance model of 2-dimensional airfoil data fitted from Xfoil. this model is designed to return the
    performance of airfoils.
    """
    def __init__(self):

        # models and fits

        self.FittedModels: dict[str, asb.FittedModel] = dict()
        self.Models: dict = dict()
        self.Grids: dict[str, dict] = dict()

        # weights

        # data
        self.xdata: dict[str, dict[[str, np.ndarray]]]
        self.ydata: dict[Union[str, np.ndarray]]

        # outputs
        self.parameters: dict[Union[str, float]]

    def liftCoef(self, x: dict[Union[str, float]]):
        return self.Cl_Model(x, self.ClFit.parameters)

    def dragCoef(self, x: dict[Union[str, float]]):
        return self.Cl_Model(x, self.ClFit.parameters)

    def momentCoef(self, x: dict[Union[str, float]]):
        return self.Cl_Model(x, self.ClFit.parameters)

    def fit(self,
            fit_coefficeint: str,
            model,
            xdata,
            ydata,
            ) -> None:
        """
        fit surface to each desired dimension
        :return:
        """

        fit = asb.FittedModel(model=model,
                              x_data=xdata,
                              y_data=ydata,
                              parameter_guesses={"alfa_l": 1,
                                                 "alfa_q": 0,
                                                 "flap_l": 1,
                                                 "flap_q": 0,
                                                 "const": .1,
                                                 "co_l": 0.5
                                                 }
                              )
        self.FittedModels[fit_coefficeint] = fit
        self.Models[fit_coefficeint] = model

    def generate2DGrid(self,
                       coefficient: str,
                       x: dict[Union[str, np.ndarray]] = None,
                       ) -> dict[Union[str, np.ndarray]]:
        """
        currently only works for keywords 'alpha' and 'flap_ang' for x_dict, will work on making this dynamic later
        :param coefficient:
        :param x:
        :return:
        """

        # if no xdata then just use bounds from the existing data
        if not x:
            x = self.FittedModels[coefficient].x_data

        # create output array
        output = np.empty((len(x['alpha']), len(x['flap_ang'])))

        for idx, valx in enumerate(x['alpha']):
            for idy, valy in enumerate(x['flap_ang']):

                output[idx, idy] = self.Models[coefficient](
                    x={'alpha':valx,
                        'flap_ang':valy
                    },
                    p = self.FittedModels[coefficient].parameters
                )

        self.Grids[coefficient] = {
            "x": x,
            "y": output
        }
        return self.Grids[coefficient]

    def plot(self,
             coefficient:str,
             xaxis:str,
             yaxis:str,
             overlay_raw:bool = False,
             )->None:
        """
        plots a 3D surface
        :param coefficient:
        :return:
        """
        try:
            data = self.Grids[coefficient]
        except KeyError:
            data=self.generate2DGrid(coefficient)
        xdata = data['x'][xaxis]
        ydata = data['x'][yaxis]
        zdata = data['y']

        X,Y = np.meshgrid(xdata,ydata)
        Z = zdata

        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

        ax.plot_surface(X,Y,Z)

        if overlay_raw:
            x_raw = self.FittedModels[coefficient].x_data
            xaxis_raw = x_raw[xaxis]
            yaxis_raw = x_raw[yaxis]
            z_raw = self.FittedModels[coefficient].y_data
            ax.scatter(xaxis_raw,yaxis_raw,z_raw)

        ax.set_xlabel(xaxis)
        ax.set_ylabel(yaxis)
        ax.set_zlabel(coefficient)
        return fig,ax

    def plot_err(self,
             coefficient:str,
             xaxis:str,
             yaxis:str,
             )->None:
        """
        plots a 3D surface
        :param coefficient:
        :return:
        """
        raw = self.FittedModels[coefficient]
        model = self.Models[coefficient]
        fit_params = self.FittedModels[coefficient].parameters

        X = raw.x_data[xaxis]
        Y = raw.x_data[yaxis]
        Zraw = raw.y_data

        Zerr = np.empty(Zraw.shape)
        for idx,Zi in enumerate(Zraw):
            x = X[idx]
            y = Y[idx]
            Zmodel = model({f'{xaxis}':x,f'{yaxis}':y},fit_params)
            Zerr[idx] = Zi-Zmodel



        fig, ax = plt.subplots()

        cntr = ax.tricontourf(X,Y,Zerr)
        ax.set_xlabel(xaxis)
        ax.set_ylabel(yaxis)
        fig.colorbar(cntr)

        return fig,ax


if __name__ == "__main__":
    Airfoil2DSection()
