# Created by trevorlong on 7/15/24
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

import numpy as np
from typing import Union


def liftCoefficientFlapAlfa(x:dict,
                            p:dict,
                            ) -> Union[float,np.ndarray]:
    """
    Lift coefficient model designed to work with aerosandbox's fitting functions. Ingests dictionaries x and p and
    returns a float of the performance
    :param x: dictionary which contains the single operating point (float, float) to return a value for
    :param p: coefficient dictionary containing floats of weights
    :return:
    """

    alfa_l = p["alfa_l"]  # linear AoA
    alfa_q = p["alfa_q"]  # quadratic AoA
    flap_l = p["flap_l"]  # linear flap
    flap_q = p["flap_q"]  # quadratic flap
    const = p["const"]  # constant term
    co_l  = p["co_l"]  # colinear term

    return (
        const
        + alfa_l * x["alpha"]
        + alfa_q * x["alpha"]**2
        + flap_l * x["flap_ang"]
        + flap_q * x["flap_ang"]**2
        + co_l * x["flap_ang"] * x["alpha"]
    )


def dragCoefficientFlapAlfa(x: dict[Union[str,np.ndarray]],
                            p: dict[Union[str,float]],
                            ) -> np.ndarray:
    """
    Lift coefficient model designed to work with aerosandbox's fitting functions. Ingests dictionaries x and p and
    returns a float of the performance
    :param x: dictionary which contains the single operating point (float, float) to return a value for
    :param p: coefficient dictionary containing floats of weights
    :return:
    """

    # alfa_l = p["alfa_l"]  # linear AoA
    alfa_q = p["alfa_q"]  # quadratic AoA
    # flap_l = p["flap_l"]  # linear flap
    flap_q = p["flap_q"]  # quadratic flap
    const = p["const"]  # constant term
    co_l = p["co_l"]  # colinear term

    return (
            const
            # + alfa_l * x["alpha"]
            + alfa_q * x["alpha"] ** 2
            # + flap_l * x["flap_ang"]
            + flap_q * x["flap_ang"] ** 2
            + co_l * x["flap_ang"] * x["alpha"]
    )


def momentCoefficientFlapAlfa(x: dict[Union[str,np.ndarray]],
                            p: dict[Union[str,float]],
                            ) -> np.ndarray:
    """
    Lift coefficient model designed to work with aerosandbox's fitting functions. Ingests dictionaries x and p and
    returns a float of the performance
    :param x: dictionary which contains the single operating point (float, float) to return a value for
    :param p: coefficient dictionary containing floats of weights
    :return:
    """

    alfa_l = p["alfa_l"]  # linear AoA
    alfa_q = p["alfa_q"]  # quadratic AoA
    flap_l = p["flap_l"]  # linear flap
    flap_q = p["flap_q"]  # quadratic flap
    const = p["const"]  # constant term
    co_l = p["co_l"]  # colinear term

    return (
            const
            + alfa_l * x["alpha"]
            + alfa_q * x["alpha"] ** 2
            + flap_l * x["flap_ang"]
            + flap_q * x["flap_ang"] ** 2
            + co_l * x["flap_ang"] * x["alpha"]
    )