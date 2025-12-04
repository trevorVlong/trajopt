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
    c1 = 0.1856
    c2 = 0.0334
    c3 = 0.0667
    c4 = 0.0121
    c5 = -0.0085
    c6 = 0.0426
    c7 = 0.0140
    c8 = 0.0226
    c9 = 0.1407

    # write out func in 3 parts to make easier to read

    # const part w.r.t. cj
    f1 = c1 + c2*delta_f + c3*alfa

    # square root part w.r.t. cj
    f2 = delta_cj ** 0.5 * (c4 * delta_f + c5 * alfa + c7)

    # linear part w.r.t. cj
    f3 = delta_cj * (c6*delta_f + c8*alfa + c9)

    return (f1+f2+f3)


def pitchingCoeff(alfa, delta_f, delta_cj,kcm=1):
    """
    pitching coefficient surrogate model fit taking the form given by courtin in E.23->E.29. makes simplification that
    j1,j2 are equal and that both are the flap angle delta_f
    :param alfa:
    :param delta_f:
    :param delta_cj:
    :return:
    """

    # parametrs given
    c1 = -0.128
    c2 = -0.0114
    c3 = -0.0387
    c4 = -0.0143
    c5 = 0.0278
    c6 = 0.0174
    c7 = -0.0120
    c8 = -0.0017
    c9 =  0.0238
    # write out func in 3 parts to make easier to read

    # const part w.r.t. cj
    f1 = c1 + c2*delta_f + c3*alfa

    # square root part w.r.t. cj
    f2 = delta_cj ** 0.5 * (c4 * delta_f + c5 * alfa + c6)

    # linear part w.r.t. cj
    f3 = delta_cj * (c7*delta_f + c8*alfa + c9)

    return kcm * (f1+f2+f3)


def dragCoeff(CL,dcj,AR):
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

    CD0 = 0.05

    # induced drag component

    CDi = CL**2 / (np.pi * AR *0.426 + 2*dcj)

    return CD0 + CDi


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    # geometric params from 2025 16.821 plane
    b = 3.05
    c = 0.38
    bt = 1.27
    ct = 0.25
    Sref = 1.09
    AR = b**2/Sref

    # blowing / other params
    E = 0.3  # flap chord fraction
    Adisk = 10 *.0061
    h = 0.0375
    c = 0.38
    rho = 1.225

    # problem params
    Vinf = 10
    T = np.linspace(0,50,5)

    alfas = np.linspace(-5,30,100)
    flapang = 20

    # plot lift coefficient as function of cj
    fig, axs = plt.subplots(2, 2)
    for Tidx in T:
        vrat = np.sqrt(Tidx / (0.5 * rho * Vinf ** 2 * Adisk) + 1)
        dcj = Adisk/Sref * (vrat**2-1)*(vrat**2 + 1)
        cl = liftCoeff(alfas,flapang,dcj)
        cd = dragCoeff(cl,dcj,AR) - Tidx/(0.5*rho*Vinf**2*Adisk)
        cm = pitchingCoeff(alfas,flapang,dcj)
        print(f"{dcj}")

        axs[0, 0].plot(alfas, cl)

        axs[0, 1].plot(alfas, cd)
        axs[1, 0].plot(alfas, cm)
        axs[1, 1].plot(cl, cm)
    axs[0, 0].grid()
    axs[0, 1].grid()
    axs[1, 1].grid()
    axs[1, 0].grid()
    axs[0, 0].set_ylabel('cl')
    axs[0, 0].set_xlabel('alfa')
    axs[0, 1].set_ylabel('cd')
    axs[0, 1].set_xlabel('alfa')
    axs[1, 0].set_ylabel('cm')
    axs[1, 0].set_xlabel('alfa')
    axs[1, 1].set_ylabel('cm')
    axs[1, 1].set_xlabel('cl')
    plt.show()
