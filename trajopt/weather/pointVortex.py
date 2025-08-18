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

import aerosandbox.numpy as np
from typing import Union,Tuple


def pointVortexKernal(xg:float,
                      zg:float,
                      x:float,
                      z:float,
                      normalvec:Union[float,Tuple],
                      ) -> float:
    """
    Kernal function for point vortex of unit strength located at (xg,zg) on the point (x,z) in direction normalvec.
    Used to construct vortex influence matrix. Units irrelevant but are understood to be all of the same scale

    :param xg
    :param zg
    :param x
    :param z
    :param normalvec
    :return:
    """

    # get useful values
    dx = x-xg
    dz = z-zg
    r = np.sqrt(dx**2 + dz**2)
    th = np.arctan2(dz,dx)

    # contributions as a function of the kernal
    K = 1/(2*np.pi) * 1/r
    Kx = -K*np.sin(th)
    Kz = K*np.cos(th)

    # return dot product
    return Kx * normalvec[0] + Kz * normalvec[1]


def pointVortexVelocity(xg,zg,x,z,Gamma) -> Union[Tuple,float]:
    """
    vortex velocity at point (x,z) as a function of the vortex location and strength
    :param xg:
    :param zg:
    :param x:
    :param z:
    :param Gamma:
    :return:
    """
    # get useful values
    dx = x - xg
    dz = z - zg
    r = np.sqrt(dx ** 2 + dz ** 2)
    th = np.arctan2(dz, dx)

    # contributions as a function of the kernal
    Vmag = Gamma / (2 * np.pi) * 1 / r
    Vz = Vmag * np.cos(th)
    Vx = -Vmag * np.sin(th)

    # return dot product
    return Vx,Vz


if __name__ == "__main__":

    xg = 0
    zg = 0
    x = 1
    z = 0
    nhat = (0,1)
    Gamma = 1

    # make sure all functions run, no need to check  value
    val = pointVortexKernal(xg,zg,x,z,nhat)
    val2 = pointVortexVelocity(xg,zg,x,z,Gamma)


