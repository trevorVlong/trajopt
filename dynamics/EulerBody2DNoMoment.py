# Created by trevorlong on 6/14/24
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
from typing import Union, Dict, Tuple, List
from aerosandbox import Opti, OperatingPoint, Atmosphere
from aerosandbox.common import AeroSandboxObject


class EulerBody2D(AeroSandboxObject):
    """
    Class for Euler body in 2-Dimensions

        * commanded rotation
    """

    def __init__(self,
                 mass: float = 1,
                 x_e: Union[float, np.ndarray] = 0,  # inertial-x position of the body
                 z_e: Union[float, np.ndarray] = 0,  # inertial-y position of the body
                 u_b: Union[float, np.ndarray] = 0,  # body-x velocity of the body
                 w_b: Union[float, np.ndarray] = 0,  # body-y velocity of the body,
                 pitch: Union[float, np.ndarray] = 0,  # commanded pitch of the aircraft
                 #windModel: WindModel2D = WindModel2D(),  # wind model
                 ):

        # inertial characteristics
        self.mass = mass
        self.Iyy = 0
        self.g = 0

        # state variables
        self.x_e = x_e
        self.z_e = z_e
        self.u = u_b
        self.w = w_b
        self.pitch = pitch

        # control variables
        self.Fx_b = 0
        self.Fz_b = 0
        self.My_b = 0

        #self.WindModel = windModel

    @property
    def state(self) -> Dict[str, Union[float, np.ndarray]]:
        return {
            "x_e": self.x_e,
            "z_e": self.z_e,
            "u": self.u,
            "w": self.w,
            "pitch":self.pitch
        }

    @property
    def control_variables(self) -> Dict[str, Union[float, np.ndarray]]:
        return {
            "Fx_b": self.Fx_b,
            "Fz_b": self.Fz_b,
            "My_b": self.My_b,
        }

    def state_derivatives(self) -> Dict[str, Union[float, np.ndarray]]:

        return {
            "x_e": self.u_e,
            "z_e": self.w_e,
            "u": self.a_x,
            "w": self.a_z,
            'pitch': self.dtheta
        }

    @property
    def a_x(self) -> Union[float, np.ndarray]:
        return self.Fx_b / self.mass

    @property
    def a_z(self) -> Union[float, np.ndarray]:
        return self.Fz_b / self.mass

    @property
    def alpha(self) -> Union[float, np.ndarray]:
        return np.arctan2d(-self.w, self.u)

    def add_force(self,
                  Fx: Union[float, np.ndarray] = 0,
                  Fy: Union[float, np.ndarray] = 0,
                  Fz: Union[float, np.ndarray] = 0,
                  from_axes="body",
                  ) -> None:
        """
        Adds forcing to the body axes, assumes with no input that original math is in body-axes
        :param Fx:
        :param Fy:
        :param Fz:
        :param from_axes:
        :return:
        """
        Fx, Fy, Fz = self.convert_axes(
            x_from=Fx,
            y_from=Fy,
            z_from=Fz,
            from_axes=from_axes,
            to_axes="body"
        )
        self.Fx_b = self.Fx_b + Fx
        self.Fz_b = self.Fz_b + Fz

    def constrain_derivatives(self,
                              opti: Opti,
                              time: np.ndarray,
                              method: str = "trapezoidal",
                              which: Union[str, List[str], Tuple[str]] = "all",
                              _stacklevel=1,
                              ):
        """
        Applies the relevant state derivative constraints to a given Opti instance.

        Args:

            opti: the AeroSandbox `Opti` instance that constraints should be applied to.

            time: A vector that represents the time at each discrete point. Should be the same length as any
            vectorized state variables in the `state` property of this Dynamics instance.

            method: The discrete integration method to use. See Opti.constrain_derivative() for options.

            which: Which state variables should be we constrain? By default, constrains all of them.

                Options:

                    * "all", which constrains all state variables (default)

                    * A list of strings that are state variable names (i.e., a subset of `dyn.state.keys()`),
                    that gives the names of state variables to be constrained.

            _stacklevel: Optional and advanced, purely used for debugging. Allows users to correctly track where
            constraints are declared in the event that they are subclassing `aerosandbox.Opti`. Modifies the
            stacklevel of the declaration tracked, which is then presented using
            `aerosandbox.Opti.variable_declaration()` and `aerosandbox.Opti.constraint_declaration()`.

        Returns:

        """
        if which == "all":
            which = self.state.keys()

        state_derivatives = self.state_derivatives()

        for state_var_name in which:

            # If a state derivative has a None value, skip it.
            if state_derivatives[state_var_name] is None:
                continue

            # Try to constrain the derivative
            try:
                opti.constrain_derivative(
                    derivative=state_derivatives[state_var_name],
                    variable=self.state[state_var_name],
                    with_respect_to=time,
                    method=method,
                    _stacklevel=_stacklevel + 1
                )
            except KeyError:
                raise ValueError(f"This dynamics instance does not have a state named '{state_var_name}'!")
            except Exception as e:
                raise ValueError(f"Error while constraining state variable '{state_var_name}': \n{e}")

    def convert_axes(self,
                     x_from: float,
                     y_from: float,
                     z_from: float,
                     from_axes: str,
                     to_axes: str,
                     ) -> Tuple[float, float, float]:
        """
        Converts axes for [x,y,z]_from to a new [x',y',z'] in the new coordinate system. For conversions between
        body, wind, and stability axes this uses the operating_point class' internal rotation; for rotating between
        body and earth it uses the rotation schemes shown here

        :param x_from: x-value in original coordinate system
        :param y_from: y-value in original coordinate system
        :param z_from: z-value in original coordinate system
        :param from_axes: name of original coordinate system ('earth', 'wind', 'body')
        :param to_axes: name of target coordinate system ('earth', 'wind', 'body')
        :return: [x',y',z'] the vector components in the new coordinate system
        """

        # really only 2 directions: earth to wind and wind to earth
        # could also be body to wind
        # gravity: earth to wind
        # thrust: body to wind
        # position update: body to earth
        #

        # if inputting into the same axes just return the same vector
        if from_axes == to_axes:
            return x_from, y_from, z_from

        # get earth to body rotation matrix
        # pitch cos/sin
        st = np.sind(self.pitch)
        ct = np.cosd(self.pitch)

        # convert any earth to body or body to earth
        if from_axes == "earth" or from_axes == "body":

            if from_axes == 'earth':
                x_from = x_from*ct + z_from*st
                y_from = np.zeros(x_from.shape)
                z_from = z_from*ct - x_from*st

            # if only rotating to body then done
            if to_axes == "body":
                return x_from, y_from, z_from

            #  if rotating to wind use op_point to do the rest
            elif to_axes == "wind":
                return self.op_point.convert_axes(x_from=x_from,
                                                  y_from=y_from,
                                                  z_from=z_from,
                                                  from_axes='body',
                                                  to_axes='wind')
            elif to_axes == "earth":
                pass
            else:
                raise ValueError('Bad value for "from_axes"')

        if from_axes == "wind" or from_axes == "body":

            # rotate back to body using op_point if not body
            if from_axes == "wind":
                x_from, y_from, z_from = self.op_point.convert_axes(x_from=x_from,
                                                                    y_from=y_from,
                                                                    z_from=z_from,
                                                                    from_axes='wind',
                                                                    to_axes='body',
                                                                    )

            if to_axes == "body":
                return x_from, y_from, z_from

            elif to_axes == "earth":
                x_to = x_from * ct - z_from * st
                y_to = np.zeros(x_to.shape)
                z_to = z_from * ct + x_from * st
                return x_to, y_to, z_to

            elif to_axes == 'wind':
                pass
            else:
                raise ValueError('Bad value for "from_axes"')

    @property
    def altitude(self):
        return self.z_e

    @property
    def u_e(self) -> Union[float, np.ndarray]:
        """
        u-velocity of the body in earth coords
        :return:
        """
        V = self.convert_axes(self.u,
                              0,
                              self.w,
                              "body", "earth")
        #wind_x = self.WindModel.windSpeed(self.x_e,np.zeros(self.x_e.shape),self.z_e)[0]
        return V[0]

    @property
    def w_e(self) -> Union[float, np.ndarray]:
        """
        z-velocity of the body in earth coords
        :return:
        """
        V = self.convert_axes(self.u,
                              0,
                              self.w,
                              "body", "earth")
        #wind_z = self.WindModel.windSpeed(self.x_e, np.zeros(self.x_e.shape), self.z_e)[2]
        return V[2]

    @property
    def op_point(self):
        """
        Returns an OperatingPoint object that represents the current state of the dynamics instance.

        This OperatingPoint object is effectively a subset of the state variables, and is used to compute aerodynamic
        forces and moments.
        """
        return OperatingPoint(
            atmosphere=Atmosphere(altitude=self.altitude),
            velocity=self.speed,
            alpha=self.alpha,
            beta=0,
            p=0,
            q=self.dtheta,
            r=0,
        )

    @property
    def dtheta(self) -> Union[float, np.ndarray]:
        """
        convert moment to acceleration using Mx = I * alpha --> alpha = M/I
        :returns alpha_y in deg/s
        :return:
        """
        dtheta = self.My_b / self.Iyy * 180/np.pi

        return dtheta

    @property
    def speed(self) -> Union[float, np.ndarray]:
        return (self.u_e ** 2 + self.w_e ** 2) ** 0.5

    @property
    def KE(self) -> Union[float, np.ndarray]:
        return 0.5*self.mass*self.speed**2

    @property
    def PE(self) -> Union[float, np.ndarray]:
        return self.mass * 9.81 * self.z_e

    @property
    def TE(self) -> Union[float, np.ndarray]:
        return self.KE + self.PE


if __name__ == "__main__":
    dyn = EulerBody2D()
