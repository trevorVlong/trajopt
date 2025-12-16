#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created Sun Apr 16 19:37:47 2023
@author: fdulker

updated for current use Tue 16 Dec, 2025
@author: tlong
"""
import numpy as np
import trajopt.weather.uvsing as uvsing
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#uvsing.__doc__


# 1  Vortex_pan -0.200000     -0.200000       1.00000      0.100000       10.0000      0.100000                 0.21000
# 2  Vortex_pan -0.200000     -0.200000       1.00000      0.300000       10.0000      0.300000                 0.21000
# 3  Vortex_pan -0.200000     -0.200000       1.00000      0.500000       10.0000      0.500000                 0.21000
# 4  Vortex_pan -0.200000     -0.200000       1.00000      0.700000       10.0000      0.700000                 0.21000
# 5  Vortex_pan -0.200000     -0.200000       1.00000      0.900000       10.0000      0.900000                 0.21000
# 6  Vortex_pan  0.200000      0.200000      -1.00000      0.100000      -10.0000      0.100000                 0.21000
# 7  Vortex_pan  0.200000      0.200000      -1.00000      0.300000      -10.0000      0.300000                 0.21000
# 8  Vortex_pan  0.200000      0.200000      -1.00000      0.500000      -10.0000      0.500000                 0.21000
# 9  Vortex_pan  0.200000      0.200000      -1.00000      0.700000      -10.0000      0.700000                 0.21000
# 10 Vortex_pan  0.200000      0.200000      -1.00000      0.900000      -10.0000      0.900000                 0.21000
def updraft(xloc:float,
            zloc:float,
            xc: float,
            radius: float,
            gustvel: float):
    """
    updraft greates a potential flow gust field given the the x-center, gust
    radius and gust core velocity

    :param xloc: xlocation for evaluation
    :param zloc: zlocation for evaluation
    :param xc: core center on x-axis
    :param radius: core radius
    :param gustvel: gust core velocity
    """
    # -------------------------------------------------------
    # set up singularities
    nsing = 10
    ising = [0] * nsing
    rcore = [0] * nsing
    xsing = [0] * nsing
    ysing = [0] * nsing
    xsing2 = [0] * nsing
    ysing2 = [0] * nsing
    asing = [0] * nsing
    sing = [0] * nsing
    sing2 = [0] * nsing

    for k in range(nsing):
        ising[k] = 6
        rcore[k] = radius

    xsing[0] = 1.00000;
    ysing[0] = 0.10000
    xsing[1] = 1.00000;
    ysing[1] = 0.30000
    xsing[2] = 1.00000;
    ysing[2] = 0.50000
    xsing[3] = 1.00000;
    ysing[3] = 0.70000
    xsing[4] = 1.00000;
    ysing[4] = 0.90000
    # image
    xsing[5] = -1.00000;
    ysing[5] = 0.10000
    xsing[6] = -1.00000;
    ysing[6] = 0.30000
    xsing[7] = -1.00000;
    ysing[7] = 0.50000
    xsing[8] = -1.00000;
    ysing[8] = 0.70000
    xsing[9] = -1.00000;
    ysing[9] = 0.90000

    xsing2[0] = 10.00000;
    ysing2[0] = 0.10000
    xsing2[1] = 10.00000;
    ysing2[1] = 0.30000
    xsing2[2] = 10.00000;
    ysing2[2] = 0.50000
    xsing2[3] = 10.00000;
    ysing2[3] = 0.70000
    xsing2[4] = 10.00000;
    ysing2[4] = 0.90000
    # image
    xsing2[5] = -10.00000;
    ysing2[5] = 0.10000
    xsing2[6] = -10.00000;
    ysing2[6] = 0.30000
    xsing2[7] = -10.00000;
    ysing2[7] = 0.50000
    xsing2[8] = -10.00000;
    ysing2[8] = 0.70000
    xsing2[9] = -10.00000;
    ysing2[9] = 0.90000



    sing[0] = 0.200000
    sing2[0] = 0.200000
    sing[1] = 0.200000
    sing2[1] = 0.200000
    sing[2] = 0.200000
    sing2[2] = 0.200000
    sing[3] = 0.200000
    sing2[3] = 0.200000
    sing[4] = 0.200000
    sing2[4] = 0.200000

    sing[5] = -0.200000
    sing2[5] = -0.200000
    sing[6] = -0.200000
    sing2[6] = -0.200000
    sing[7] = -0.200000
    sing2[7] = -0.200000
    sing[8] = -0.200000
    sing2[8] = -0.200000
    sing[9] = -0.200000
    sing2[9] = -0.200000
    # -------------------------------------------------------
    # set velocity
    sing = np.array(sing) * gustvel # + for updraft, - for downdraft

    eps = 1e-5;

    # x,y,z is in global coordinates
    # xs , ys is in sings coordinates

    # xt ,yt location of the thermal in x,y,z global coordinates
    # rhat and zhat 3d vectors
    rhat = [0] * 3
    zhat = [0] * 3

    xt = xc  # pick this location along the path, and you can MC to move
    yt = 0  # set this to zero for only longitudinal dynamics
    yloc = 0 # 2D

    # distance from gust core
    dist = np.sqrt((xloc - xt) ** 2 + (yloc - yt) ** 2)  #


    rhat[0] = (xloc - xt) / dist
    rhat[1] = (yloc - yt) / dist
    rhat[2] = 0.0

    zhat[0] = 0.0
    zhat[1] = 0.0
    zhat[2] = 1.0
    Vg = np.zeros(3)
    us, vs = uvsing.uvsing(zloc, dist, nsing, ising, sing, sing2, xsing, ysing, xsing2, ysing2, asing, rcore, eps)
    for i in range(3):
        Vg[i] = us*zhat[i] + vs*rhat[i]

    return Vg[0],Vg[2]


if __name__ == "__main__":

    xc = 2.5
    zc = 0
    wg = 2
    radius=0.21
    N = 20
    xlocs = np.zeros((N**2))
    zlocs = np.zeros((N**2))
    uvel = np.zeros((N ** 2))
    wvel = np.zeros((N ** 2))
    idx = 0
    for x in np.linspace(-5,5,N):
        for z in np.linspace(0.01,20,N):
            #print(f"idx:{idx}")

            us,ws = updraft(x,z,xc,radius,wg)
            xlocs[idx] = x
            zlocs[idx] = z
            uvel[idx] = us
            wvel[idx] = ws
            idx+=1

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.quiver(xlocs,zlocs,uvel,wvel,scale=10)
    print('done')
    print(f'max uvel {np.max(np.abs(uvel))}')
    print(f'max wvel {np.max(np.abs(wvel))}')
    plt.show()