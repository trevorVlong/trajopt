#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 19:37:47 2023
gfortran -shared -fPIC -o .so .f

gfortran-mp-12 -shared -fPIC -o .so uvsing.f .f .f 
python3 -m numpy.f2py uvsing.f -m uvsing -h uvsing.pyf  
python3 -m numpy.f2py uvsing.f -m uvsing -h --overwrite-signature uvsing.pyf
python3 -m numpy.f2py -c uvsing.pyf .so

@author: fdulker
"""
import numpy as np
import uvsing
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

uvsing.__doc__



#1  Vortex_pan -0.200000     -0.200000       1.00000      0.100000       10.0000      0.100000                 0.21000
#2  Vortex_pan -0.200000     -0.200000       1.00000      0.300000       10.0000      0.300000                 0.21000
#3  Vortex_pan -0.200000     -0.200000       1.00000      0.500000       10.0000      0.500000                 0.21000
#4  Vortex_pan -0.200000     -0.200000       1.00000      0.700000       10.0000      0.700000                 0.21000
#5  Vortex_pan -0.200000     -0.200000       1.00000      0.900000       10.0000      0.900000                 0.21000
#6  Vortex_pan  0.200000      0.200000      -1.00000      0.100000      -10.0000      0.100000                 0.21000
#7  Vortex_pan  0.200000      0.200000      -1.00000      0.300000      -10.0000      0.300000                 0.21000
#8  Vortex_pan  0.200000      0.200000      -1.00000      0.500000      -10.0000      0.500000                 0.21000
#9  Vortex_pan  0.200000      0.200000      -1.00000      0.700000      -10.0000      0.700000                 0.21000
#10 Vortex_pan  0.200000      0.200000      -1.00000      0.900000      -10.0000      0.900000                 0.21000

#------------------------------------------------
# thermal parameters
Rt = 2.0  # thermal radius
xt = 5.0  # center of thermal x location (in earth x,y,z)  
yt = 0.0  # center of thermal y location (in earth x,y,z)  
wscale = 2.0  # thermal vertical velocity scale (+ updraft, - downdraft)
#------------------------------------------------


# Set up thermal up/downdraft of unit radius and unit velocity (will scale to actual Rt, wscale at the end)
 
nsing = 10
ising = [0]*nsing
rcore = [0]*nsing
xsing1 = [0]*nsing
ysing1 = [0]*nsing
xsing2 = [0]*nsing
ysing2 = [0]*nsing
gamma1 = [0]*nsing
gamma2 = [0]*nsing
asing = [0]*nsing

for k in range(nsing):
    ising[k] = 6
    rcore[k] = 0.2

xsing1[0] =   1.00000 ; ysing1[0] =   0.10000
xsing1[1] =   1.00000 ; ysing1[1] =   0.30000
xsing1[2] =   1.00000 ; ysing1[2] =   0.50000   
xsing1[3] =   1.00000 ; ysing1[3] =   0.70000
xsing1[4] =   1.00000 ; ysing1[4] =   0.90000 
# image
xsing1[5] =  -1.00000 ; ysing1[5] =   0.10000
xsing1[6] =  -1.00000 ; ysing1[6] =   0.30000
xsing1[7] =  -1.00000 ; ysing1[7] =   0.50000   
xsing1[8] =  -1.00000 ; ysing1[8] =   0.70000
xsing1[9] =  -1.00000 ; ysing1[9] =   0.90000 

xsing2[0] =  10.00000 ; ysing2[0] =   0.10000
xsing2[1] =  10.00000 ; ysing2[1] =   0.30000
xsing2[2] =  10.00000 ; ysing2[2] =   0.50000   
xsing2[3] =  10.00000 ; ysing2[3] =   0.70000
xsing2[4] =  10.00000 ; ysing2[4] =   0.90000 
# image
xsing2[5] = -10.00000 ; ysing2[5] =   0.10000
xsing2[6] = -10.00000 ; ysing2[6] =   0.30000
xsing2[7] = -10.00000 ; ysing2[7] =   0.50000   
xsing2[8] = -10.00000 ; ysing2[8] =   0.70000
xsing2[9] = -10.00000 ; ysing2[9] =   0.90000 


gamma1[0]   = 0.200000 ; gamma2[0]  = 0.200000 
gamma1[1]   = 0.200000 ; gamma2[1]  = 0.200000 
gamma1[2]   = 0.200000 ; gamma2[2]  = 0.200000 
gamma1[3]   = 0.200000 ; gamma2[3]  = 0.200000 
gamma1[4]   = 0.200000 ; gamma2[4]  = 0.200000 

gamma1[5]   = -0.200000 ; gamma2[5]  = -0.200000 
gamma1[6]   = -0.200000 ; gamma2[6]  = -0.200000 
gamma1[7]   = -0.200000 ; gamma2[7]  = -0.200000 
gamma1[8]   = -0.200000 ; gamma2[8]  = -0.200000 
gamma1[9]   = -0.200000 ; gamma2[9]  = -0.200000 


# scale unit gust to specified size and velocity
xsing1 = np.array(xsing1)*Rt
xsing2 = np.array(xsing2)*Rt

ysing1 = np.array(ysing1)*Rt
ysing2 = np.array(ysing2)*Rt

rcore = np.array(rcore)*Rt

gamma1 = np.array(gamma1)*wscale
gamma2 = np.array(gamma2)*wscale

eps = 1e-5 * Rt

# x,y,z is in global coordinates 
# xs,ys is in sings coordinates

# initialize unit vectors
rhat = [0]*3
zhat = [0]*3
Vg =   [0]*3


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# evaluate velocity vectors at all points on a x,z grid (y=0 plane), for plotting

for k in range(20):
 x = xt + 2*Rt*(k-9.5)/20.0
 y = 0.0 
 for kk in range(20):
  z = 0.0 + 2*Rt*kk/20.0

# compute axisymmetric gust velocities in gust zs,rs coordinates
  zs = z  # altitude along gust axis
  rs = np.sqrt((x-xt)**2 + (y-yt)**2)   # radius from gust axis
  Vz,Vr = uvsing.uvsing(zs,rs,nsing,ising,gamma1,gamma2,xsing1,ysing1,xsing2,ysing2,asing,rcore, eps)

# axial and radial unit vectors of axisymmetric gust, at x,y,z field point
  rhat[0] = (x-xt)/rs 
  rhat[1] = (y-yt)/rs 
  rhat[2] = 0.0 

  zhat[0] = 0.0
  zhat[1] = 0.0
  zhat[2] = 1.0

# x,y,z components of gust velocity
  for i in range(3):
    Vg[i] = Vz*zhat[i] + Vr*rhat[i]
 
# plot it
  ax.quiver(x,y,z, Vg[0], Vg[1],Vg[2],cmap=plt.cm.jet) #  head_width=0.01, head_length=0.01, color='red')
 
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_zlim(0,2*Rt)

plt.show()
