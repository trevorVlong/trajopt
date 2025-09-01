# Created by trevorlong on 7/24/25
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

from trajopt.dynamics import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox import Opti


"""
Purpose of this is to show that the model is working as anticipated
"""

def D1ForcingProblem(N:int=100,dt=0.1):
    """
    get optimizer to solve a 1D problem
    :return:
    """

    # initialize problem and variables
    opti = Opti()
    time = np.arange(start=0,step=dt,stop=N*dt)
    pointmass = Aircraft2DPointMass(
        mass=1,
        Iyy=1,
        x_earth=opti.variable(init_guess=np.zeros((N,)), lower_bound=-100, upper_bound=100),
        z_earth=np.zeros((N,)),
        u_body=opti.variable(init_guess=np.zeros((N,)), lower_bound=-100, upper_bound=100),
        w_body=np.zeros((N,)),
        pitch = np.zeros((N,)),
        elevator_deflection= np.zeros((N,)),
        throttle_position=opti.variable(init_guess=np.zeros((N,)), lower_bound=-0.5 * 9.81, upper_bound=0.5 * 9.81),
        flap_deflection=np.zeros((N,))

    )

    opti.subject_to([
        pointmass.EarthXPosition[0] == -10,
        pointmass.ThrottlePosition[0] == 0,
        pointmass.BodyXVelocity[0] == 10,
    ])

    pointmass.add_force(pointmass.ThrottlePosition, 0, 'body')
    opti.minimize(pointmass.EarthXPosition[-1]**2 + pointmass.BodyXVelocity[-1]**2)
    pointmass.constrain_derivatives(opti,time)

    sol = opti.solve()

    pointmass = sol(pointmass)
    return opti,sol,pointmass

def ForcingProblem(N:int=100,dt=0.1):
    """
    get optimizer to solve a 1D problem
    :return:
    """

    # initialize problem and variables
    opti = Opti()
    time = np.arange(start=0,step=dt,stop=N*dt)
    pointmass = Aircraft2DPointMass(
        mass=1,
        Iyy=1,
        x_earth=opti.variable(init_guess=np.ones((N,)), lower_bound=-100, upper_bound=100),
        z_earth=opti.variable(init_guess=np.ones((N,)), lower_bound=-100, upper_bound=100),
        u_body=opti.variable(init_guess=np.ones((N,)), lower_bound=-100, upper_bound=100),
        w_body=opti.variable(init_guess=np.ones((N,)), lower_bound=-100, upper_bound=100),
        pitch = opti.variable(init_guess=np.ones((N,)),lower_bound=-100,upper_bound=100),
        elevator_deflection= opti.variable(init_guess=np.ones((N,)), lower_bound=-1, upper_bound=1),
        throttle_position=opti.variable(init_guess=np.ones((N,)), lower_bound=-0.5 * 9.81, upper_bound=0.5 * 9.81),
        flap_deflection=np.zeros((N,))

    )

    opti.subject_to([
        pointmass.EarthXPosition[0] == -10,
        pointmass.EarthZPosition[0] == -10,
        pointmass.ThrottlePosition[0] == 0,
        pointmass.BodyXVelocity[0] == 10,
        pointmass.Pitch[0] == 0,
    ])

    opti.subject_to([
        pointmass.BodyXVelocity >= 0,
    ])

    pointmass.add_force(pointmass.ThrottlePosition, 0, 'body')
    pointmass.add_moment(pointmass.ElevatorPosition)
    opti.minimize(pointmass.EarthXPosition[-1] ** 2 + pointmass.EarthZPosition[-1] ** 2 + pointmass.Speed[-1] ** 2)
    pointmass.constrain_derivatives(opti,time)
    try:
        sol = opti.solve()
    except RuntimeError:
        opti.debug()
    pointmass = sol(pointmass)
    return opti,sol,pointmass

if __name__ == "__main__":

    opti,sol,pointmass = ForcingProblem(100,0.1)
    time = np.arange(0,100*0.1,0.1)

    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(pointmass.EarthXPosition, pointmass.Altitude)

    plt.figure()
    plt.plot(time,pointmass.Pitch)

    plt.figure()
    plt.plot(time, pointmass.ThrottlePosition, label='bodyx')
    plt.plot(time, pointmass.ElevatorPosition, label='bodymom')

    plt.figure()
    plt.plot(time, pointmass.EarthXVelocity)
    plt.plot(time, pointmass.EarthZVelocity)
    plt.show()

    print('here')