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
 
nsing = 10
ising = [0]*nsing
rcore = [0]*nsing
xsing = [0]*nsing
ysing = [0]*nsing
xsing2 = [0]*nsing
ysing2 = [0]*nsing
asing = [0]*nsing
sing = [0]*nsing
sing2 = [0]*nsing

for k in range(nsing):
    ising[k] = 6
    rcore[k] = 0.21

xsing[0]  =   1.00000 ; ysing[0] =   0.10000
xsing[1]  =   1.00000 ; ysing[1] =   0.30000
xsing[2]  =   1.00000 ; ysing[2] =   0.50000   
xsing[3]  =   1.00000 ; ysing[3] =   0.70000
xsing[4]  =   1.00000 ; ysing[4] =   0.90000 
# image
xsing[5]  =  -1.00000 ; ysing[5] =   0.10000
xsing[6]  =  -1.00000 ; ysing[6] =   0.30000
xsing[7]  =  -1.00000 ; ysing[7] =   0.50000   
xsing[8]  =  -1.00000 ; ysing[8] =   0.70000
xsing[9] =   -1.00000 ; ysing[9]=   0.90000 

xsing2[0] =  10.00000 ; ysing2[0]=   0.10000
xsing2[1] =  10.00000 ; ysing2[1]=   0.30000
xsing2[2] =  10.00000 ; ysing2[2]=   0.50000   
xsing2[3] =  10.00000 ; ysing2[3]=   0.70000
xsing2[4] =  10.00000 ; ysing2[4]=   0.90000 
# image
xsing2[5] = -10.00000 ; ysing2[5]=   0.10000
xsing2[6] = -10.00000 ; ysing2[6]=   0.30000
xsing2[7] = -10.00000 ; ysing2[7]=   0.50000   
xsing2[8] = -10.00000 ; ysing2[8]=   0.70000
xsing2[9]= -10.00000 ; ysing2[9]=   0.90000 

wscale =  2.0  # + updraft 
#wscale = -2.0 # - downdraft 

sing[0]   = 0.200000 ; sing2[0]  = 0.200000 
sing[1]   = 0.200000 ; sing2[1]  = 0.200000 
sing[2]   = 0.200000 ; sing2[2]  = 0.200000 
sing[3]   = 0.200000 ; sing2[3]  = 0.200000 
sing[4]   = 0.200000 ; sing2[4]  = 0.200000 

sing[5]   = -0.200000 ; sing2[5]  = -0.200000 
sing[6]   = -0.200000 ; sing2[6]  = -0.200000 
sing[7]   = -0.200000 ; sing2[7]  = -0.200000 
sing[8]   = -0.200000 ; sing2[8]  = -0.200000 
sing[9]   = -0.200000 ; sing2[9]  = -0.200000 

sing =np.array(sing)*wscale

eps = 1e-5;

# x,y,z is in global coordinates 
# xs , ys is in sings coordinates

# xt ,yt location of the thermal in x,y,z global coordinates
# rhat and zhat 3d vectors 
rhat = [0]*3
zhat = [0]*3
Vg =   [0]*3

xt = 2.0 # pick this location along the path, and you can MC to move 
yt = 0   # set this to zero for only longitudinal dynamics 

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for k in range(20):
 x = 1.0 + k* 0.1 
 y = 0.0 
 for kk in range(20):
  z = 0.0 + kk*0.25  

  xs = z  # altitude 
  ys = np.sqrt((x-xt)**2+(y-yt)**2 )    # 

  rhat[0] = (x-xt)/ys 
  rhat[1] = (y-yt)/ys 
  rhat[2] = 0.0 
  zhat[0] = 0.0
  zhat[1] = 0.0
  zhat[2] = 1.0
  us,vs = uvsing.uvsing(xs,ys,nsing,ising,sing,sing2,xsing,ysing,xsing2,ysing2,asing,rcore, eps)
# 3D velocity component of the gust Vg 
  for i in range(3):
    Vg[i] =us*zhat[i] + vs*rhat[i]
 
  ax.quiver(x,y, z, Vg[0], Vg[1],Vg[2],cmap=plt.cm.jet) #  head_width=0.01, head_length=0.01, color='red')
 
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_zlim(0,5)

plt.show()

# Sensor                                   4.00000      0.500000E-01
# Sensor                                   4.00000      0.150000
# Sensor                                   4.00000      0.200000
# Sensor                                   4.00000      0.250000
# Sensor                                   4.00000      0.350000
# Sensor                                   4.00000      0.400000
# Sensor                                   4.00000      0.450000
# Sensor                                   4.00000      0.550000
# Sensor                                   4.00000      0.600000
# Sensor                                   4.00000      0.650000
# Sensor                                   4.00000      0.750000
# Sensor                                   4.00000      0.800000
# Sensor                                   4.00000      0.850000
# Sensor                                   4.00000      0.950000
# Sensor                                   4.00000       1.00000
# Sensor                                   4.00000       1.05000


