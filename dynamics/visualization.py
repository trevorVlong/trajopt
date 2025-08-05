# Created by trevorlong on 8/6/24
# license
# Copyright 2024 trevorlong

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
import aerosandbox.optimization.opti as opti
import matplotlib.pyplot as plt
import seaborn as sns
from dynamics.RigidMotion.Aircraft2D import Aircraft2DPointMass
from typing import Union

sns.set_theme()


def visualizeRun2D(time,
                   dyn:Aircraft2DPointMass,
                   ) -> dict[str, tuple[plt.figure, Union[list,plt.axis]]]:
    """
    visualization for a run of 2D dynamcis run
    :param dyn:
    :return:
    """

    fig_dict = {}
    # pull out any values I know will be useful so I don't have to constantly dyn.() stuff

    # generate time trace subplots
    fig, axes = plt.subplots(4,1,sharex=True,figsize=(6,10),dpi=100)

    # velocity
    axes[0].plot(time,dyn.Airspeed,label="$V_\\infty$")
    axes[0].set_ylabel("$V_\\infty$ [m/s]",rotation=0)
    axes[0].yaxis.set_label_coords(-.15,0.5)
    axes[0].set_ylim([0,1.1*np.max(dyn.Airspeed)])


    # pitch and angle of attack
    aline = axes[1].plot(time, dyn.Alpha, label="$\\alpha$")
    pline = axes[1].plot(time, dyn.Pitch, label="$\\theta$")
    axes[1].set_ylabel("Angle [deg]",rotation=0)
    axes[1].yaxis.set_label_coords(-.2, 0.5)

    # if np.max(dyn.pitch) < 10:
    #     axes[1].set_ylim([-10,10])
    # else:
    #     axes[1].set_ylim([-1.05*np.abs(np.max(dyn.pitch)), 1.05*np.abs(np.max(dyn.pitch))])

    axes[1].legend()

    # pitch rate
    axes[2].plot(time, dyn.PitchRate, label="pitch rate")
    axes[2].set_ylabel("Pitch Rate [deg/s]",rotation=0,)
    axes[2].yaxis.set_label_coords(-.25, 0.5)
    axes[2].set_ylim([-40,40])

    # control motions
    fline = axes[3].plot(time, dyn.FlapPosition, label="flap angle", color="red")
    htline = axes[3].plot(time, dyn.ElevatorPosition, label="horizontal tail", color="blue")
    axes[3].set_ylim([-40,40])
    thrust_ax = axes[3].twinx()
    thrust_ax.set_ylim([-0.1,1.2])
    axes = np.append(axes,thrust_ax)
    tline = thrust_ax.plot(time, dyn.ThrottlePosition, label="throttle position", color="green")


    axes[3].legend(handles=[fline[0],htline[0],tline[0]],loc="upper left",fontsize=10)
    axes[3].set_ylabel("Angle [deg]",rotation=0)

    thrust_ax.set_ylabel("Pos [-]")
    axes[3].set_xlabel("time [s]")
    axes[3].yaxis.set_label_coords(-.25, 0.5)

    # make sure all grids are behind, create all legends
    for axis in axes:
        axis.grid(zorder=0)

    axes[4].grid(False)
    # add to the output dictionary
    fig_dict["time_series_general"] = (fig, axes)


    # ================================================================
    # position plot
    fig,ax = plt.subplots(figsize=(6,3))

    ax.plot(dyn.EarthXPosition, dyn.Altitude, label="position", marker="s", markersize=0.5)
    ax.set_xlabel("$x^e$ [m]")
    ax.set_ylabel("$z^e$ [m]",rotation=0)
    ax.yaxis.set_label_coords(-.15,0.5)
    ax.set_ylim([0, 1.1 * np.max(dyn.Altitude)])

    fig_dict["position"] = (fig,ax)

    # ================================================================
    # energy plots
    fig, ax = plt.subplots(figsize=(6, 3))

    ax.plot(time, dyn.TE/dyn.TE[0], label="TE", marker="s", markersize=0.5)
    ax.plot(time, dyn.PE/dyn.TE[0], label="PE", markersize=0.5)
    ax.plot(time, dyn.KE/dyn.TE[0], label="KE", markersize=0.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Energy [j]", rotation=0)

    fig_dict["energy"] = (fig, ax)
    plt.legend()

    return fig_dict