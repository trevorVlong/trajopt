# Created by trevorlong on 4/29/25
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
Intended to be a repository for support function for estimating Aero forces and coefficients
"""

from aerosandbox import numpy as np
from typing import Union


def example(a:Union[float, np.ndarray],
            b:Union[float, np.ndarray],
            ) -> Union[float,np.ndarray]:
    """
    example function

    :param a: a numpy array of floats or aerosandbox variables
    :param b: a numpy array of floats or aerosandbox variables
    :return: c: a numpy array of floats or aerosandbox variables
    """

    c = a+b

    return c


def liftCoeff2D(lift_force: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_length: Union[float,np.ndarray],
                ) -> Union[float,np.ndarray]:
    """
    Lift Coefficient in 2D. All input units in SI standard units
    :param lift_force:
    :param dynamic_pressure:
    :param reference_length: 
    :return: 
    """

    return lift_force/(dynamic_pressure * reference_length)


def dragCoeff2D(drag_force: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_length: Union[float,np.ndarray],
                ) -> Union[float, np.ndarray]:
    """
    Drag Coefficient in 2D, Cd. All input units in SI standard units
    :param drag_force:
    :param dynamic_pressure:
    :param reference_length:
    :return:
    """

    return drag_force / (dynamic_pressure * reference_length)


def pitchCoeff2D(pitchingMoment: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_length: Union[float,np.ndarray],
                ) -> Union[float, np.ndarray]:
    """
    pitching Coefficient in 2D, Cd. All input units in SI standard units
    :param pitchingMoment:
    :param dynamic_pressure:
    :param reference_length:
    :return:
    """

    return pitchingMoment / (dynamic_pressure * reference_length**2)


def liftCoeff3D(lift_force: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_area: Union[float,np.ndarray],
                ) -> Union[float,np.ndarray]:
    """
    Lift Coefficient in 3D. All input units in SI standard units
    :param lift_force:
    :param dynamic_pressure:
    :param reference_area:
    :return:
    """

    return lift_force/(dynamic_pressure * reference_area)


def dragCoeff3D(drag_force: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_area: Union[float,np.ndarray],
                ) -> Union[float,np.ndarray]:
    """
    Lift Coefficient in 3D. All input units in SI standard units
    :param lift_force:
    :param dynamic_pressure:
    :param reference_area:
    :return:
    """

    return drag_force/(dynamic_pressure * reference_area)


def pitchCoeff3D(pitchingMoment: Union[float,np.ndarray],
                dynamic_pressure: Union[float,np.ndarray],
                reference_length: Union[float,np.ndarray],
                reference_area: Union[float,np.ndarray]
                ) -> Union[float, np.ndarray]:
    """
    pitching Coefficient in 2D, Cd. All input units in SI standard units
    :param pitchingMoment:
    :param dynamic_pressure:
    :param reference_area:
    :param reference_length:
    :return:
    """

    return pitchingMoment / (dynamic_pressure * reference_length * reference_area)


def lift2D(lift_coefficient: Union[float,np.ndarray],
           dynamic_pressure: Union[float,np.ndarray],
           reference_length: Union[float,np.ndarray],
           ) -> Union[float,np.ndarray]:

    return lift_coefficient * dynamic_pressure * reference_length


def jetMomentumCoefficient(Vj:Union[float,np.ndarray],
                           rho:Union[float,np.ndarray],
                           V:Union[float,np.ndarray],
                           rho_j: Union[float,np.ndarray]) -> Union[float,np.ndarray]:
    """
    jet momentum coefficient

    J = 0.5 * rho * Vj^2 / 0.5 * rho * V^2
    :param J:
    :param rho:
    :param V:
    :return:
    """

    J = 0.5 * rho_j * Vj**2  # jet momentum
    M0 = 0.5 * rho * V**2   # freestrem momentum

    return J/M0


def thrustCoefficient(T,Sref,air_density,Vinf):
    """
    Tc', quasi 3D non-dimensionalization of thrust per span
    :param T: thrust
    :param Sref: area of propulsor
    :param air_density:
    :return:
    """
    qinf = 0.5 * air_density * Vinf**2

    return T/(qinf * Sref)

