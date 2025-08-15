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


class PointMass2D(AeroSandboxObject):
    """
    Class for setting up a point-mass representation of a dynamic body in 2-dimensions.

    Format is based on the speed_gamma class in aerosandbox.dynamics.speed_gamma class but has been modified to more
    easily incorporate the direct simulation of control surfaces to the model. Class includes methods for interfacing
    with the aerosanbdox.Opti() class to set up the trajectory optimization problem, methods for rotating between
    body, earth, and aero/stability coordinate systems (for more on axes systems see the convert_axes method).
    """

    def __init__(self,
                 mass: float = 1,
                 Iyy: float = 1,
                 earth_x_position: Union[float, np.ndarray] = 0,
                 earth_z_position: Union[float, np.ndarray] = 0,
                 body_x_velocity: Union[float, np.ndarray] = 0,
                 body_z_velocity: Union[float, np.ndarray] = 0,
                 pitch: Union[float, np.ndarray] = 0,
                 pitch_rate: Union[float, np.ndarray] = 0,
                 Fx_b: Union[float, np.ndarray] = 0,
                 Fz_b: Union[float, np.ndarray] = 0,
                 My_b: Union[float, np.ndarray] = 0,
                 ):
        """
        Initialize the class with option of assigning values for mass, inertia, state and control variables. There
        are no explicit units applied here but it is assumed that all incoming units will be SI.

        :param mass: [kg] mass of the point mass object
        :param Iyy: [kg-m^2] pitching / y-axis rotational inertia
        :param earth_x_position: [m] earth x-position state variable (forward is arbitrary but assumed along direction of travel)
        :param earth_z_position: [m] earth z-position state variable (+ is below ground, - is above ground by convention)
        :param body_x_velocity: [m/s] x-velocity component along the +x body axis (along the nose)
        :param body_z_velocity: [m/s] z-velocity component along the +y body axis (downward)
        :param pitch: [deg] angle of body axis system relative to earth x-axis (defined + upward)
        :param pitch_rate: [deg/s] angle rate of body axis system relative to earth x-axis (defined + upward)
        :param Fx_b: [N] Forcing along the body x-axis
        :param Fz_b: [N] Forcing along the body z-axis
        :param My_b: [Nm] Moment about the body y-axis (defined + nose up)
        """

        # inertial characteristics
        self.Mass = mass
        self.Iyy = Iyy

        # state variables
        self.EarthXPosition = earth_x_position
        self.EarthZPosition = earth_z_position
        self.BodyXVelocity = body_x_velocity  # body x-velocity in [m/s] + forward
        self.BodyZVelocity = body_z_velocity
        self.Pitch = pitch
        self.PitchRate = pitch_rate

        # control variables
        self.Fx_b = Fx_b
        self.Fz_b = Fz_b
        self.My_b = My_b

    @property
    def state(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Property which returns the state vector consisting of:

            earth x,z position (xe,ze) from arbitrarily chosen (x0,z0) point
            Body velocities (ub, wb) which is the motion velocity within the air mass
            pitch and pitch rate (theta, q) of the aircraft

        The state is tracked in standard aircraft axes in 2D systems where
        earth axes([ ]_e) are defined +x forward, +z downward
        body axes ([ ]_b) are defined +x out nose, +z downward

        The earth x,z position is relative to an arbitrary starting point. Note that in this formulation the +z is
        'under' the ground and so a separate property Altitude has been provided to give the positive number.

        The body velocity components u,w are the velocity of the point mass in the body frame and include
        contributions from the motion of the vehicle in the air mass (u_motion, w_motion) as well as the speed of the
        air mass (u_gust, w_gust). Thus,
            u_body = u_inertial + u_gust
            w_body = w_inertial + w_gust

        The pitch and pitch rate are defined as + up which is contrary to the coordinate system of the body axes.
        Methods in the rest of the class handle this discrepency. B

        :return:
        """
        return {
            "EarthXPosition": self.EarthXPosition,
            "EarthZPosition": self.EarthZPosition,
            "BodyXVelocity": self.BodyXVelocity,
            "BodyZVelocity": self.BodyZVelocity,
            "Pitch": self.Pitch,
            "PitchRate": self.PitchRate
        }

    @property
    def control_variables(self) -> Dict[str, Union[float, np.ndarray]]:
        return {
            "Fx_b": self.Fx_b,
            "Fz_b": self.Fz_b,
            "My_b": self.My_b,
        }

    def state_derivatives(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Method to return dictionary of state vector derivatives
        :return:
        """
        return {
            "EarthXPosition": self.EarthXVelocity,
            "EarthZPosition": self.EarthZVelocity,
            "BodyXVelocity": self.AccelXBody,
            "BodyZVelocity": self.AccelZBody,
            "Pitch": self.PitchRate,
            "PitchRate": self.PitchAccel
        }

    @property
    def a_x(self) -> Union[float, np.ndarray]:
        return self.Fx_b / self.Mass

    @property
    def a_z(self) -> Union[float, np.ndarray]:
        return self.Fz_b / self.Mass

    def add_force(self,
                  Fx: Union[float, np.ndarray] = 0,
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
        Fx, Fz = self.convert_axes(
            x_from=Fx,
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
        Applies the relevant state derivative constraints to a given Opti instance. (copied from aerosandbox,
        credit @petersharpe)

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
                     x_from: Union[float, np.ndarray],
                     z_from: Union[float, np.ndarray],
                     from_axes: str,
                     to_axes: str
                     ) -> tuple[Union[float, np.ndarray], Union[float, np.ndarray]]:
        """
        Converts the 2-dimensional vector [x,z] to the new coordinate system [x',z'] using the 2D euler transform
        matrices. This specific implementation does not care about the y-axis even though it's direction is affected
        by separate transformations. Additionaly in 2-dimensions stability and wind axes are precisely the same
        because sideslip does not exist in 2-dimensions


        Requires the use of the state variables of pitch and angle of attack (calculated by a method above)
        :param x_from:
        :param z_from:
        :param from_axes:
        :param to_axes:
        :return:
        """

        # if in and out are the same, simply return
        if from_axes == to_axes:
            return x_from, z_from

        # precalculate sin,cos values and rotation matrices
        st = np.sind(-self.Pitch)
        ct = np.cosd(-self.Pitch)

        # if in and out are not the same, start logic to transform
        if from_axes in ['stability', 'wind']:
            # need to use the negative of the incoming vector since the ordinate system is flipped relative to body orientation
            # Twb = [ cos(alfa) -sin(alfa) ]
            #      [ sin(alfa)  cos(alfa) ]
            sa = np.sind(self.Alpha + 180)  # 180deg offset added because wind frame is +x back, +z up
            ca = np.cosd(self.Alpha + 180)  # see above
            x_body = x_from * ca - z_from * sa
            z_body = x_from * sa + z_from * ca

        elif from_axes == 'geometric':
            # geometric only has the z-flipped to +z up
            x_body = x_from
            z_body = -z_from

        elif from_axes == 'body':
            # if body just return input
            x_body = x_from
            z_body = z_from

        elif from_axes == 'earth':
            # rotation by pitch
            # Teb = [ cos(pitch) -sin(pitch) ]
            #      [ sin(pitch)  cos(pitch) ]
            x_body = x_from * ct + z_from * st
            z_body = -x_from * st + z_from * ct

        else:
            warn('from_axis incorrect / not supported')
            raise ValueError

        # ------------------------------------------------------------------------
        # once transformed to body, perform rotations to the new coordinate system

        if to_axes in ['wind', 'stability']:
            # Twb = [ cos(alfa) sin(alfa) ]
            #      [ -sin(alfa)  cos(alfa) ]
            sa = np.sind(self.Alpha + 180)  # 180deg offset added because wind frame is +x back, +z up
            ca = np.cosd(self.Alpha + 180)  # see above
            x_to = x_body * ca + z_body * sa
            z_to = -x_body * sa + z_body * ca

        elif to_axes == 'geometric':
            x_to = x_body
            z_to = -z_body

        elif to_axes == 'body':
            x_to = x_body
            z_to = z_body

        elif to_axes == 'earth':
            # Tbe = [ cos(pitch) sin(pitch) ]
            #      [ -sin(pitch)  cos(pitch) ]
            x_to = x_body * ct + z_body * st
            z_to = x_body * st + z_body * ct
        else:
            warn('to_axis incorrect / not supported')
            raise ValueError

        return x_to, z_to

    @property
    def altitude(self):
        return self.z_e

    @property
    def EarthXVelocity(self) -> Union[float, np.ndarray]:
        """
        u-velocity of the body in earth coords
        :return:
        """
        ue, ve = self.convert_axes(self.BodyXVelocity, self.BodyZVelocity, from_axes='body', to_axes='earth')
        return ue

    @property
    def EarthZVelocity(self) -> Union[float, np.ndarray]:
        """
        z-velocity of the body in earth coords
        :return:
        """
        ue, ve = self.convert_axes(self.BodyXVelocity, self.BodyZVelocity, from_axes='body', to_axes='earth')
        return ve

    @property
    def speed(self) -> Union[float, np.ndarray]:
        return (self.EarthXVelocity ** 2 + self.EarthZVelocity ** 2) ** 0.5

    @property
    def KE(self) -> Union[float, np.ndarray]:
        return 0.5*self.Mass*self.speed**2

    @property
    def PE(self) -> Union[float, np.ndarray]:
        return -self.Mass * 9.81 * self.EarthZPosition

    @property
    def TE(self) -> Union[float, np.ndarray]:
        return self.KE + self.PE

    @property
    def PitchAccel(self) -> Union[float, np.ndarray]:
        """
        Pitch derivative relationship as a funciton of moment applied and inertia. Returns value in deg/s.

        qdot = My/Iyy
        qdot_deg = qdot * 180/pi

        :return:
        """

        return self.My_b / self.Iyy * 180/np.pi
    
    def add_moment(self,
                  My: Union[float, np.ndarray] = 0,
                  axes="body",
                  ) -> None:
        """
        Adds forcing to the body axes, assumes with no input that original math is in body-axes
        :param Fx:
        :param Fy:
        :param Fz:
        :param axes:
        :return:
        """

        self.My_b = self.My_b + My

    @property
    def AccelXBody(self) -> Union[float, np.ndarray]:
        """
        Property which returns acceleration along the x-axis in the body frame. Body frame defined as +x out of the
        nose of the aircraft.
        :return:
        """
        return self.Fx_b / self.Mass

    @property
    def AccelZBody(self) -> Union[float, np.ndarray]:
        """
        Property which returns acceleration along the x-axis in the body frame. Body frame defined as +z out of the
        bottom of the aircraft.
        :return:
        """
        return self.Fz_b / self.Mass

    @property
    def Alpha(self) -> Union[float, np.ndarray]:
        """
        Property which returns Angle of Attack in [deg] using body velocity components and gust velocities. The
        following equation is defined in the body frame

        AoA = arctan((wb-wg)/(ub-ug))

        :return:
        """
        # get gust velocity
        u_g, w_g = self.gustVelocity(self.EarthXPosition, self.Altitude, to_axes='body')  # get gust velocity in body

        # return AoA in deg
        return np.arctan2d(self.BodyZVelocity - w_g, self.BodyXVelocity - u_g)


if __name__ == "__main__":
    dyn = PointMass2D()
