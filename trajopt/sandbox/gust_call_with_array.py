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
from aerosandbox import Opti
import aerosandbox.numpy as np
from trajopt.weather import updraft


def main(N):

    xidx = np.linspace(-5,5,N)
    yidx = np.linspace(0.00,5,N)

    X = np.array([])
    Y = []
    U = []
    W = []
    for idx,xi in enumerate(xidx):
        for idy,yi in enumerate(yidx):
            ui,wi = updraft(xi,yi,0,0.1,1)

            X = np.append(X,xi)
            Y = np.append(Y,yi)
            U = np.append(U,np.nan_to_num(ui))
            W = np.append(W,np.nan_to_num(wi))


    #
    opti = Opti()
    x = opti.variable(init_guess=10)
    y = opti.variable(init_guess=-10,lower_bound=0)
    #
    ufit = np.interpn((xidx,yidx),np.reshape(U,[N,N]),np.array([x,y]),bounds_error=False,method='bspline',fill_value=0)
    wfit = np.interpn((xidx,yidx),np.reshape(W,[N,N]),np.array([x,y]),bounds_error=False,method='bspline',fill_value=0)

    opti.minimize(x**2 + (y-2)**2)
    sol = opti.solve()
    print(f'x=:{sol(x)}')
    print(f'y=:{sol(y)}')
    print(f"u={sol(ufit)}")
    print(f"w={sol(wfit)}")


if __name__ == "__main__":
    N = 40
    p = main(N)


    # integration process with trajopt:

    # 1. precompute gust filed to get X,Y,U,W
    # 2. in the simulation, load in 1. , create interpolants using command similar to:
    #   ufit = np.interpn((xidx,yidx),np.reshape(U,[N,N]),np.array([x,y]),
    #   bounds_error=False, for optimization based on function notes
    #   method='bspline', for optimization based on function notes
    #   fill_value=0 , for extrapolation, outside of gust expect no disturbance
    #   )
    # 3. run simulation where gust model is included