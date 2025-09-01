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


from aerosandbox import numpy as np
from typing import Union,Tuple


def An(E:float,
       ) -> callable:
    """
    first fourier coefficient as a function of cf/c = E, AoA, flap deflection angle. Chord line dz/dx is assumed to
    be -tan(df) since flap deflection is defined as positive down

    :param E: cf/c from 0-1
    :param df: flap deflection in deg
    :param alpha: angle of attack in deg
    :return:
    """

    # reparameterization of chord into [0,1]
    theta_f = np.arccos(2*E-1)

    # generate A0 function using theta_f
    def func(alpha,
             delta_f
             ) -> Tuple[Union[float, np.ndarray],Union[float, np.ndarray],Union[float, np.ndarray]]:
        """

        :param alpha:
        :param delta_f:
        :return:
        """

        A0 = alpha + (np.pi - theta_f)/np.pi * np.tan(delta_f)
        A1 = 2/np.pi * np.tan(delta_f) * np.sin(theta_f)
        A2 = 1/np.pi * np.tan(delta_f) * np.sin(2*theta_f)

        return A0, A1, A2

    return func


def liftCoeffFlappedThinAirfoil(E:float):

    """
    generates the function for the thin airfoil theory solution for a flapped thin airfoil. returns a function handle
    that is a funciton of angle of attack and flap deflection angle testing code wrapping why won't it wrap
    :param E: flap chord to chord ratio
    :return: function handle
    """

    # remapped chord position

    def cl(alpha: Union[float, np.ndarray],
           delta_f: Union[float, np.ndarray]
           ) -> Union[float, np.ndarray]:
        """
        lift as a function of angle of attack and flap angle
        :param alpha: angle of attack in deg
        :param delta_f: flap deflection in deg
        :return:
        """

        alpha = np.pi/180 * alpha
        delta_f = np.pi/180 * delta_f

        A0, A1, A2 = An(E)(alpha, delta_f)

        return 2*np.pi * (A0 + 1/2*A1)

    return cl


def pitchCoeffFlappedThinAirfoil(E:float):
    """
    generates function that returns the pithcing coefficient on a TAT airfoil with a flap at position cf/c = E as a
    function of the flap deflection angle
    :param E:
    :return:
    """


    # generate function for pitching at c/4 using An
    def cm(alpha:Union[float, np.ndarray],
           delta_f: Union[float, np.ndarray]
           ) -> Union[float, np.ndarray]:
        """
        cm_c/4 for a thin airfoil theory airfoil. Takes in angle of attack and flap deflection as angles
        :param alpha:
        :param delta_f:
        :return:
        """
        alpha = np.pi / 180 * alpha
        delta_f = np.pi / 180 * delta_f

        A0, A1, A2 = An(E)(alpha, delta_f)

        return -np.pi/4*A1 + A2/4

    return cm
