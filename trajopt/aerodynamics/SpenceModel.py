# Created by trevorlong on 5/6/25
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
from typing import Union
from aerosandbox.numpy.integrate_discrete import integrate_discrete_intervals


def a_mn(phi:Union[float, np.ndarray],n) -> Union[float, np.ndarray]:
    """

    :param phi:
    :return:
    """

    if n == 0:

        return np.sin(phi)
    else:

        return (1 + np.cos(phi)) * np.sin(n*phi)


def b_mn(phi,
         n,
         ):

    # precalculate trig functions
    cosphi = np.cos(n*phi)
    tanphi = np.tan(phi/2)
    sinphi = np.sin(n*phi)

    # break up equation to make more readable
    coeff = 4/(4*n**2 - 1)
    b1 = cosphi
    b2 = 2*n * tanphi * sinphi

    return coeff * (b1 + b2)


def f_mn(phi,xi):
    """

    :param phi:
    :param xi:
    :return:
    """

    #  precalculate trig funcs
    cosphi = np.cos(phi/2)
    tanphi = np.tan(phi/2)
    tanxi = np.tan(xi/2)
    sinphi = np.sin(phi/2)

    # equation broken into two summed parts for readability
    fm1 = 2*xi/np.pi * tanphi

    if sinphi != 0:
        fm2 = 4/np.pi * 1/cosphi * np.arctan(tanxi/sinphi)
    else:
        fm2 = 4/np.pi * 1/cosphi * np.pi/2
    return fm1 - fm2



def phi_m(m,N):
    """
    phi from eq 25 of spence
    :param m:
    :param N:
    :return:
    """

    return m*np.pi / N


def xi(E):
    """
    transformation x-term value at hinge point

    th = arccos(2x-1)
    :param cf:
    :param c:
    :return:
    """

    return np.arccos(2*(1-E) - 1)


def e_mn(phi):
    sinphi = np.sin(phi/2)
    cosphi = np.cos(phi/2)

    return -2 * 1/cosphi * (1-sinphi)

def DcoeffSolution(N,E,Cj):

    lam = 4/Cj

    # set up matrix equations
    A = np.empty((N,N))
    f = np.empty((N,1))
    for m in np.arange(0,N,1):
        phi = phi_m(m,N)
        f[m] = f_mn(phi,xi(E))
        for n in np.arange(0,N,1):

            amn = a_mn(phi,n)
            bmn = b_mn(phi,n)
            A[m,n] = amn + lam * bmn

    Dm = np.linalg.solve(A,f)

    return Dm


def BcoeffSolution(N,E,Cj):

    lam = 4 / Cj

    # set up matrix equations
    A = np.empty((N,N))
    emn = np.empty((N,1))
    for m in np.arange(0,N,1):
        phi = phi_m(m,N)
        emn[m] = e_mn(phi)
        for n in np.arange(0,N,1):

            amn = a_mn(phi,n)
            bmn = b_mn(phi,n)
            A[m,n] = amn + lam * bmn

    Bm = np.linalg.solve(A,emn)


    return Bm

def fx(x,D,alfa,E):
    """
    f(x)/(2*beta)
    Eq 33, 34 of '''The Lift on a Thin Aerofoil with a Jet-Augmented Flap'''. All terms but D terms are geometric.
    for vorticity distribution multiply by 2*beta
    :param x:
    :param D:
    :return:
    """
    # compute helpful params
    xi = 2*np.arcsin(np.sqrt(E))
    X = (1-(1-x)**0.5)/(1+(1-x)**0.5)
    x12 = ((1-x)/x)**0.5
    E12 = (E/(1-E))**0.5


    # compute in parts to make more readible
    f1 = xi/np.pi * x12
    f2 = 1/np.pi * np.log(np.abs( (x12 + E12)/(x12-E12) ))
    f3 = x**-1.5 * (D[0]*(2*X)/(1+X) + D[1]*X)
    f4 = 2*alfa*x12

    return f1+f2+f3+f4


def cl(alfa,beta,Dm,E,N=100):
        """

        :param alfa:
        :param beta:
        :param cj:
        :return:
        """

        #grid vorticity dist
        x = np.linspace(0.001,1,N)
        gx = fx(x,Dm,alfa,E)*2*beta
        
        return integrate_discrete_intervals(gx,x)


def cm(alfa, beta, Dm, N=100):
    """

    :param alfa:
    :param beta:
    :param cj:
    :return:
    """

    # grid vorticity dist
    x = np.linspace(0.001, 1, N)
    gx = fx(x, Dm, alfa) * 2 * beta
    #
    return np.trapezoid(gx*x, x)

if __name__ == "__main__":

    E = 0.3
    Cj = 2
    N = 9

    Dm = DcoeffSolution(N,E,Cj)
    Bm = BcoeffSolution(N,E,Cj)


    # print out to compare with paper
    print(Dm)
    print(len(Dm))

    print(Bm)
    print(len(Bm))

    # plot
    import matplotlib.pyplot as plt


    # grid
    cjvec = np.array([0.01, 0.05, 0.1, 0.2, 0.4, 0.5, 1, 2, 5])

    E = 0.3
    E = [0, 0.02, 0.05, 0.1, 0.2,0.3, 0.5, 1]

    plt.figure()
    for e in E:
        x = xi(e)
        dvec = list([])
        clvec = list([])
        for cj in cjvec:
            D = DcoeffSolution(N, e, cj)
            dvec.append(D[0])
            clvec.append(2*(x + np.sin(x) + 2 * np.pi * D[0]))


        plt.plot(cjvec, clvec,label = f'E={e}')

    plt.legend()

    plt.figure()
    alfa = 0*np.pi/180
    beta = 30*np.pi/180
    # check that fx distribution looks right
    E = 0.3
    CJ = 1
    Dm = DcoeffSolution(2, E, Cj)
    x = np.linspace(0.01,1,200)

    g = fx(x,Dm,alfa)

    plt.plot(x,g,label='g(x)/2b')


    plt.legend()
    plt.grid()

    print(cl(alfa,beta,Dm))
    print(-cm(alfa,beta , Dm))


    plt.figure()
    # plot lift, pitching as functions of beta

    alfa = 0
    beta = np.linspace(0,1,100)

    cmdist = []
    cldist = []
    N = 100
    for idx,b in enumerate(beta):

        cmdist.append(cm(alfa,b,Dm,N))
        cldist.append(cl(alfa,b,Dm,N))

    cmdist = np.array([cmdist]).flatten()
    cldist = np.array([cldist]).flatten()

    plt.plot(beta*180/np.pi,-cmdist,label='cmle')
    plt.plot(beta * 180 / np.pi, cldist, label='cl')
    plt.plot(beta*180/np.pi, -(cmdist-cldist*1/4),label='cm1/4')

    plt.legend()



    plt.show()