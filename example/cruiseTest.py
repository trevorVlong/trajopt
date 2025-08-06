# Created by trevorlong on 6/18/25
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
import aerosandbox as asb
import aerosandbox.numpy as np
from dynamics.RigidMotion.Aircraft2D import Aircraft2DPointMass
from dynamics.AeroModelling.SimpleAircraft2D import SimpleAircraft2D
from dynamics.AeroModelling.ThinAirfoilAnalytical2D import liftCoeffFlappedThinAirfoil
from AeroModelling.dragModelling import inducedDrag

from dynamics.AeroModelling.propulsorModels import *


def problem():

    # i don't know why this import needs to sit in here right now, will resolve later
    from dynamics.AeroModelling.courtinSurrogates import dragCoeff, liftCoeff, pitchingCoeff


    # aircraft model, lift,drag,moment models to be used
    aero_model = SimpleAircraft2D()
    aero_model.LiftCoeffFunc = liftCoeff
    aero_model.DragCoeffFunc = dragCoeff
    aero_model.MomentCoeffFunc = pitchingCoeff

    aero_model.LiftCoeffTailFunc = liftCoeffFlappedThinAirfoil(0.3)
    aero_model.DragCoeffTailFunc = inducedDrag
    aero_model.MomentCoeffTailFunc = lambda x,y,z: 0 # very small and not considering currently



    # aircraft parameters
    aero_model.Span = 3.05  # span [m]
    aero_model.MeanChord = 0.38  # mean-chord [m]
    aero_model.TailSpan = 1.27  # tial span [m]

    aero_model.TailMeanChord = 0.25  # tail chord [m]
    aero_model.WingArea = 1.09  # wing area [m^2]
    aero_model.TailArea = 0.32  # tail area [m^2]

    # blowing / other params
    aero_model.Adisk = .065
    aero_model.Hdisk = 0.1
    aero_model.TailLength = 1.65

    # set which models to use for Wing, Tail, Body forces

    # ========================================
    # init optimal control problem by creating opti class and feeding opti variables to dynamics model
    N = 3
    opti = asb.Opti()
    time = np.linspace(0,10,N)
    dyn = Aircraft2DPointMass(
        mass=9.54,  # kg
        Iyy=2.5,  # kg-m^2
        x_earth=opti.variable(init_guess=1 * np.linspace(1, 100, N), upper_bound=1e4, lower_bound=-1),
        z_earth=opti.variable(init_guess=1 * np.linspace(1, 100, N), upper_bound=1e4, lower_bound=-1e4),
        w_body=opti.variable(init_guess=0 * np.linspace(1, 30, N), upper_bound=30, lower_bound=-30),
        u_body=opti.variable(init_guess=1 * np.linspace(1, 30, N), upper_bound=50, lower_bound=0),
        pitch=opti.variable(init_guess=0 * np.linspace(1, 100, N), upper_bound=360 * 2, lower_bound=-180),
        # delta_f=  opti.variable(init_guess=0*np.linspace(1,100,N),upper_bound=45,lower_bound=0),
        flap_deflection=0 * np.ones(N, ),
        elevator_deflection=opti.variable(init_guess=0 * np.linspace(1, 100, N), upper_bound=30, lower_bound=-30),
        throttle_position=opti.variable(init_guess=0 * np.linspace(1, 100, N), upper_bound=0.5 * aero_model.Mass * 9.81,
                                        lower_bound=0),
    )

    dyn.addDynamics(aero_model)

    opti.subject_to([
        dyn.BodyXVelocity > 0,
        dyn.EarthZPosition >= 0,
        dyn.EarthXPosition >= 0,
    ])

    # physical limits
    opti.subject_to([
        dyn.Alpha >= -12,
        dyn.Alpha <= 15,
    ])

    # rate limits
    throttle_rate = np.diff(dyn.ThrottlePosition) / np.diff(time)
    flap_rate = np.diff(dyn.FlapPosition) / np.diff(time)
    tail_rate = np.diff(dyn.ElevatorPosition) / np.diff(time)
    pitch_rate = np.diff(dyn.Pitch, 2) / np.diff(time, 1)

    opti.subject_to([
        dyn.ThrottlePosition >= 0,
        dyn.ThrottlePosition <= 1,
        throttle_rate >= -.2,
        throttle_rate <= .2,
        tail_rate >= -5,
        tail_rate <= 5,
        pitch_rate >= -30,
        pitch_rate <= 30,
    ])

    opti.subject_to([
        dyn.EarthXPosition[0] == 0,  # starting position arbitrarily 0
        dyn.EarthZPosition[0] == 500,  # starting altitude of 2km (~180 ft)
        pitch_rate[0] == 0,  # no pitching rate ye
        dyn.Alpha[0] > 0
    ])

    # end constraints
    opti.subject_to([
        dyn.EarthXPosition[-1] <= 5e4,  # arbitrary x limit
    ])

    opti.minimize(np.sum(dyn.Altitude - dyn.Altitude[0]) ** 2)

    dyn.constrain_derivatives(opti, time, method="cubic")
    sol = opti.solve(max_iter=1000, behavior_on_failure="return_last")
    dyn = sol(dyn)
    time = sol(time)

    from dynamics.RigidMotion.visualization import visualizeRun2D
    fig_dict = visualizeRun2D(time, dyn)

    return None

if __name__=="__main__":
    """
    run the case for cruise
    """
    import matplotlib.pyplot as plt
    problem()
    plt.show()

