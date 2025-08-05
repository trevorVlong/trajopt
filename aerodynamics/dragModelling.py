# Created by trevorlong on 4/10/25
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

# HELPER FUNCTIONS AND OTHER USEFUL EQUATIONS


from aerosandbox import numpy as np


def blassiusPlateDrag(x:float,
                      V:float,
                      mu:float,
                      rho:float,
                      ) -> float:
    """
    Computes the drag due to skin friction on a laminar flat plate. Uses standard Blassius soluation which has been
    integrated from 0 -> x

    Blassius cf solution:
        cf = 0.664/sqrt(Re)
        Re = rho*V*x/mu

        D = int{0}{x'} (cf*rhoV^2/2) dx
        D = 0.664 sqrt(mu/(rho*V)) rho V^2 sqrt(x)

    :param x: [m] length of plate or position on plate
    :param Vinf: [m/s] Freestream velocity of plate
    :param mu: [kg/(m*s)] dynamic viscosity of fluid
    :param rho: [kg/m^3] density of air
    :return: Drag in [N]
    """

    return 0.664 * rho * V**2 * x /np.sqrt(ReynoldsNumber(x,V,mu,rho))


def blassiusPlateDragCoefficient(x:float,
                      V:float,
                      mu:float,
                      rho:float,
                      ) -> float:
    """
    Computes the drag coefficient due to skin friction on a laminar flat plate. Uses standard Blassius soluation which
    has been
    integrated from 0 -> c

    Blassius cf solution:
        cf = 0.664/sqrt(Re)
        Re = rho*V*x/mu

        D = int{0}{x'} (cf*rhoV^2/2) dx
        D = 0.664 sqrt(mu/(rho*V)) rho V^2 sqrt(x)
        cd = D/(0.5 * rho *V^2 * c)
           = 2*0.664*1/sqrt(Re_c)
    :param x: [m] length of plate or position on plate
    :param Vinf: [m/s] Freestream velocity of plate
    :param mu: [kg/(m*s)] dynamic viscosity of fluid
    :param rho: [kg/m^3] density of air
    :return: friction drag coefficient in counts
    """

    return 1.328 / np.sqrt(ReynoldsNumber(x,V,mu,rho))

def turbulentPlateDrag():
    #TODO
    return
def ReynoldsNumber(x,
                   Vinf,
                   mu,
                   rho
                   ) -> float:
    """
    Returns reynolds number wrt x for the given conditions. All units in SI standard units
    :param x: [m] length scale
    :param Vinf: [m/s] Fluid Velocity
    :param mu: [kg/(m*s)] Fluid Dynamic Viscosity
    :param rho: [kg/m^3] Fluid Density
    :return: Reynolds Number w.r.t. X
    """

    return (rho*Vinf*x
            /
            mu)


def inducedDrag(lift_coefficient:float,
                AR:float,
                e:float = 1,
                ) -> float:
    """
    Standard approximation for the induced drag from a 3-dimensional wing
    :param lift_coefficient: [-] lift coefficient of wing
    :param AR: [-] wing aspect ratio
    :param e: [-] span efficiency factor, in general e <1 with elliptical lift distribution e=1
    :return:Induced drag coefficient
    """

    return (
            lift_coefficient**2 /
            (np.pi * AR * e)
            )


if __name__=="__main__":
    rho = 1.225
    V = 10
    x = 1
    mu = 1.8e-5

    DL = blassiusPlateDrag(x,V,mu,rho)
    cdl = blassiusPlateDragCoefficient(x,V,mu,rho)

    print(DL)
    print(cdl)

    cl = 1
    S = 7
    b = 8
    e = 1
    print(inducedDrag(cl,S,b,e))