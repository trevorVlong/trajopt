# Created by trevorlong on 5/20/25
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


from aerosandbox import numpy as np
from typing import Union


def scaledPropulsorPoint(thrust_to_weight_ratio:float):
    """
    creates a proopulsor function which has a thrust that is scaled directly by the given mass, nothing else about
    the engine is assumed
    :param mass:
    :return:
    """

    def engine(mass:float, throttle:Union[float,np.ndarray])->Union[float,np.ndarray]:
        """
        engine model, assumes throttle input is only between 0-1
        :param throttle:
        :return:
        """

        thrust = mass*9.81*throttle*thrust_to_weight_ratio

        return thrust

    return engine

def actuatorDiskJetVelRatio(T,Adisk,rho,Vinf):
    """
    Source with derivation
    https://ocw.mit.edu/ans7870/16/16.unified/propulsionS04/UnifiedPropulsion7/UnifiedPropulsion7.htm
    :param Adisk:
    :param Vinf:
    :return:
    """

    return np.sqrt(T/(0.5*rho*Vinf**2*Adisk) + 1)


def jetMomentumFromThrust(T,Adisk,Vinf,h,c,rho):
    """
    gets jet momentum assuming the forms

    J ~= rho_j * h * Vj^2
    Cj = J/(0.5*rho*Vinf^2 * c)
    rhoj ~= rho

    Cj = (Vj/Vinf)^2 * 2 *h/c
    :param T:
    :param Adisk:
    :param rho:
    :param Vinf:
    :return:
    """
    VjVinf = actuatorDiskJetVelRatio(T,Adisk,rho,Vinf)

    return VjVinf**2 * h/c * 2


def deltaJetCoeffFromThrust(T,Adisk,Vinf,h,c,rho):
    """
    Delta CJ term as defined in Courtin's and my thesis. Amounts to a constant offset from Cj
    :param T:
    :param Adisk:
    :param Vinf:
    :param h:
    :param c:
    :param rho:
    :return:
    """
    Cj = jetMomentumFromThrust(T,Adisk,Vinf,h,c,rho)

    return Cj - 2*h/c


if __name__ == "__main__":

    T = 5*9.81 #N
    Adisk = 0.06 # inch^2? idk need to figure this one out
    h = 0.032 #m
    c = 0.38 #m
    rho = 1.225 # kg/m^3
    Vinf = 17 # m/s

    Cj = jetMomentumFromThrust(T,Adisk,Vinf,h,c,rho)
    dcj = deltaJetCoeffFromThrust(T,Adisk,Vinf,h,c,rho)
    print(Cj)
    print(dcj)
    print(actuatorDiskJetVelRatio(T,Adisk,rho,Vinf))