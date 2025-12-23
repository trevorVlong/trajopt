# Created by trevorlong on 12/23/25
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

import aerosandbox.numpy as np
import pathlib as path
from typing import Union
from trajopt.weather import updraft
def compute2DGustField(xlocs:Union[float,np.ndarray],
                       ylocs:Union[float,np.ndarray],
                       xcenter:float,
                       radius:float ,
                       wcore: float,
                       outputfilepath: Union[None,path.Path] = None
                       )-> np.ndarray:
    """
    compute gust field at xlocs,ylocs with identifying information givne by xcenter, radius, wcore
    saves the computed field points to a csv file
    """
    x = np.array([])
    y = np.array([])
    u = np.array([])
    w = np.array([])
    for idx, xi in enumerate(xlocs):
        for idy,yi in enumerate(ylocs):
            ui,wi = updraft(xi,yi,xcenter,radius,gustvel=wcore)
            x = np.append(x, xi)
            y = np.append(y, yi)
            u = np.append(u, np.nan_to_num(ui))
            w = np.append(w, np.nan_to_num(wi))

    outputarray = np.vstack((x,y,u,w)).T
    if outputfilepath is not None:
        with open(outputfilepath,'w+') as f:
            np.savetxt(f,outputarray,delimiter=',')

if __name__=="__main__":
    N = 50
    xrange = [-20,20]
    yrange = [0,20]
    xlocs = np.linspace(xrange[0],xrange[1],N)
    ylocs = np.linspace(yrange[0],yrange[1],N)
    radius = 0.21
    wcore = 1

    outputfilename = path.Path('example_gust.csv')

    compute2DGustField(xlocs,ylocs,0,radius,wcore,outputfilename)