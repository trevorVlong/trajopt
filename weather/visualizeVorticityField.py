# Created by trevorlong on 6/12/25
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

from pointVortex import pointVortexVelocity
from aerosandbox import numpy as np
import matplotlib.pyplot as plt


def visualizeVorticityField(xlocs,zlocs,Gammas:list,vortex_positions:list):
    """
    Go through list of vortices then plot velocity field based on these vortices
    :param xlocs:
    :param zlocs:
    :param vortices:
    :return:
    """

    # create the grids
    X,Z = np.meshgrid(xlocs,zlocs)
    Vx = np.zeros(X.shape)
    Vz = np.zeros(X.shape)

    # loop through all the points and calculate velocity contribution from all vortices
    for xi,x in enumerate(xlocs):
        for zi,z in enumerate(zlocs):

            # loop through all votices
            for idx in range(len(Gammas)):

                # pull out position
                rp = vortex_positions[idx]
                xg = rp[0]
                zg = rp[1]

                # contribution for this vortex
                if (xg,zg) != (x,z):
                    Vxi,Vzi = pointVortexVelocity(xg,zg,x,z,Gammas[idx])
                else:
                    Vxi = 0
                    Vzi = 0


                Vx[zi,xi] += Vxi
                Vz[zi,xi] += Vzi

    return plt.quiver(X,Z,Vx,Vz)


if __name__ == "__main__":

    vortex_positions = [(30,20),(-30,20),(30,-20),(-30,-20)]
    vortex_strengths = .1*np.array([-1,1,1,-1])

    xg = np.linspace(-30,30,20)
    zg = np.linspace(-30,30,20)

    out = visualizeVorticityField(xg,zg,vortex_strengths,vortex_positions)

    plt.show()

    print('done')
