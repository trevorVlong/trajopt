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
import matplotlib.pyplot as plt
import seaborn as sns
from trajopt.dynamics.Aircraft2D import Aircraft2DPointMass
from typing import Union,Tuple

sns.set_theme()


def visualizeRun2D(time,
                   dyn:Aircraft2DPointMass,
                   fig_dict: Union[dict[str, Tuple[plt.Figure, plt.Axes]],None] = None,
                   case: Union[str, None] = '',
                   casenum: int = 0,
                   ) -> dict[str, tuple[plt.figure, Union[list,plt.axis]]]:
    """
    visualization for a run of 2D dynamcis run
    :param dyn:
    :return:
    """
    linestyles = ['-','--','-.']

    if fig_dict is None:
        fig_dict = {}
    # pull out any values I know will be useful so I don't have to constantly dyn.() stuff

    # generate time trace subplots
    if 'time_series_general' not in list(fig_dict.keys()):
        fig, axes = plt.subplots(4,1,sharex=True,figsize=(6,10),dpi=100)
        fig_dict['time_series_general'] = (fig, axes)
    else:
        fig,axes = fig_dict['time_series_general']


    # velocity
    axes[0].plot(time,dyn.Airspeed,label=f"$V_\\infty$ {case}",color='blue',linestyle=linestyles[casenum])
    axes[0].plot(time, dyn.EarthXVelocity, label=f"groundspeed {case}",color='red',linestyle=linestyles[casenum])
    axes[0].set_ylabel("$V_\\infty$ [m/s]",rotation=0)
    axes[0].yaxis.set_label_coords(-.15,0.5)
    axes[0].set_ylim([0,1.1*np.max(dyn.Airspeed)])
    axes[0].legend()


    # pitch and angle of attack
    aline = axes[1].plot(time, dyn.Alpha, label= f"$\\alpha$ {case}",color='blue',linestyle=linestyles[casenum])
    pline = axes[1].plot(time, dyn.Pitch, label=f"$\\theta$ {case}",color='red',linestyle=linestyles[casenum])
    axes[1].set_ylabel("Angle [deg]")
    axes[1].yaxis.set_label_coords(-.2, 0.5)

    # if np.max(dyn.pitch) < 10:
    #     axes[1].set_ylim([-10,10])
    # else:
    #     axes[1].set_ylim([-1.05*np.abs(np.max(dyn.pitch)), 1.05*np.abs(np.max(dyn.pitch))])

    axes[1].legend()

    # pitch rate
    axes[2].plot(time, dyn.PitchRate, label=f"pitch rate {case}",color='red',linestyle=linestyles[casenum])
    axes[2].set_ylabel("Pitch Rate [deg/s]")
    axes[2].yaxis.set_label_coords(-.25, 0.5)
    axes[2].set_ylim([-40,40])

    # control motions
    htline = axes[3].plot(time, dyn.ElevatorPosition, label=f"horizontal tail {case}",color='blue',linestyle=linestyles[casenum])
    axes[3].set_ylim([-20,20])
    thrust_ax = axes[3].twinx()
    thrust_ax.set_ylim([-0.1,1.2])
    axes = np.append(axes,thrust_ax)
    tline = thrust_ax.plot(time, dyn.ThrottlePosition, label=f"throttle position {case}",color='green',linestyle=linestyles[casenum])


    axes[3].legend(handles=[htline[0],tline[0]],loc="upper left",fontsize=10)
    axes[3].set_ylabel("Angle [deg]")

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
    if 'position' not in list(fig_dict.keys()):
        fig,ax = plt.subplots(figsize=(6,3))
        fig_dict["position"] = (fig, ax)
    else:
        fig,ax = fig_dict["position"]

    ax.plot(dyn.EarthXPosition, dyn.Altitude, label=f'Position {case}', color='blue',linestyle=linestyles[casenum], marker="s", markersize=0.5)
    ax.legend()
    ax.set_xlabel("$x^e$ [m]")
    ax.set_ylabel("$z^e$ [m]")
    ax.yaxis.set_label_coords(-.15,0.5)
    ax.set_ylim([0, 1.1 * np.max(dyn.Altitude)])



    # ================================================================
    # energy plots
    if 'energy' not in list(fig_dict.keys()):
        fig, ax = plt.subplots(figsize=(6, 3))
        fig_dict["energy"] = (fig, ax)
    else:
        fig,ax = fig_dict["energy"]

    ax.plot(time, dyn.TE/dyn.TE[0], label="TE", marker="s", markersize=0.5)
    ax.plot(time, dyn.PE/dyn.TE[0], label="PE", markersize=0.5)
    ax.plot(time, dyn.KE/dyn.TE[0], label="KE", markersize=0.5)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Energy [j]", rotation=0)

    fig_dict["energy"] = (fig, ax)
    plt.legend()

    return fig_dict