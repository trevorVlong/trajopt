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


"""
Contains surrogate models taken from Chris Courtin's Thesis, Appendices B&E. For details check there

"""

from aerosandbox import numpy as np
from dynamics.AeroModelling.propulsorModels import deltaJetCoeffFromThrust


def liftCoeff(alfa,delta_f,delta_cj):
    """
    Lift coefficient surrogate model fit taking the form given by courtin in E.23->E.29. makes simplification that
    j1,j2 are equal and that both are the flap angle delta_f
    :param alfa:
    :param delta_f:
    :param delta_cj:
    :return:
    """

    # parametrs given
    c1 = 0.5393
    c2 = 0.0281
    c3 = 0.0262
    c4 = 0.0711
    c5 = 0.0036
    c6 = 0.0031
    c7 = -0.005
    c8 = 0.035
    c9 = 0.010
    c10 = 0.0053
    c11 = 0.0264
    c12 = 0.1712

    # write out func in 3 parts to make easier to read

    # const part w.r.t. cj
    f1 =c1 + (c2+c3)*delta_f + c4*alfa

    # square root part w.r.t. cj
    f2 = c5 + (c6+c7)*delta_f + c8*alfa

    # linear part w.r.t. cj
    f3 = c9 + (c10+c11)*delta_f + c12*alfa

    return f1 + np.sqrt(delta_cj)*f2 + delta_cj*f3


def pitchingCoeff(alfa, delta_f, delta_cj):
    """
    pitching coefficient surrogate model fit taking the form given by courtin in E.23->E.29. makes simplification that
    j1,j2 are equal and that both are the flap angle delta_f
    :param alfa:
    :param delta_f:
    :param delta_cj:
    :return:
    """

    # parametrs given
    c1 = -0.1135
    c2 = -0.0033
    c3 = -0.0029
    c4 = 0.0050
    c5 = 2.927e-4
    c6 = 2.302e-4
    c7 = 0.0012
    c8 = 0.1066
    c9 = -0.0028
    c10 = -0.0018
    c11 = 0.0049
    c12 = -0.2769

    # write out func in 3 parts to make easier to read

    # const part w.r.t. cj
    f1 = c1 + (c2 + c3) * delta_f + c4 * alfa

    # square root part w.r.t. cj
    f2 = c5 + (c6 + c7) * delta_f + c8 * alfa

    # linear part w.r.t. cj
    f3 = c9 + (c10 + c11) * delta_f + c12 * alfa

    return f1 + np.sqrt(delta_cj) * f2 + delta_cj * f3


def dragCoeff(CL,dcj,AR,Tc):
    """
    drag coefficient estimate as a sum of a constant zero-lift drag (CD0) and a induced drag component. Negative
    implies net thrust while positive implies net drag
    :param CL:
    :param dcj:
    :param AR:
    :param Tc:
    :return:
    """

    # set constant drag, not sure what to use for this except make it "reasonable"

    CD0 = 1

    # induced drag component

    CDi = CL**2 / (np.pi * AR * 0.35 + 2*dcj)

    return CD0 + CDi - Tc


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    # geometric params from 2025 16.821 plane
    b = 3.05
    c = 0.38
    bt = 1.27
    ct = 0.25
    AR = b/c

    # blowing / other params
    E = 0.3  # flap chord fraction
    Adisk = .065
    h = 0.032
    c = 0.38
    rho = 1.225

    # problem params
    Vinf = 10
    T = np.linspace(0,90,100)
    Tc = T/(0.5*rho*Vinf**2 * Adisk)
    dcj = deltaJetCoeffFromThrust(T,Adisk,Vinf,h,c,rho)
    alfa = -1
    beta = 30

    # plot lift coefficient as function of cj

    cl = liftCoeff(alfa,beta,dcj)
    cd = dragCoeff(cl,dcj,AR,Tc)
    cm = pitchingCoeff(alfa,beta,dcj)

    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(dcj, cl)
    axs[0, 0].set_title('Axis [0, 0]')
    axs[0, 1].plot(dcj, cd, 'tab:orange')
    axs[0, 1].set_title('Axis [0, 1]')
    axs[1, 0].plot(dcj, cm, 'tab:green')
    axs[1, 0].set_title('Axis [1, 0]')
    axs[1, 1].plot(cl, cm, 'tab:red')
    axs[1, 1].set_title('Axis [1, 1]')

    plt.show()
