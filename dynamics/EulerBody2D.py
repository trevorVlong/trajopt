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
from typing import Union, Dict, Tuple, Callable, List
from aerosandbox import Opti, OperatingPoint, Atmosphere
from dynamics.EulerBody2DNoMoment import EulerBody2D
from aerosandbox import AeroSandboxObject
from warnings import warn


class Aicraft2D(AeroSandboxObject):
    """
    Base class for simulating the motion of 2-Dimensional aircraft models using the aerosandbox optimizzation
    framework. Contains useful methods for calculating various quantities including:

        * rotating between body, earth, wind, and stability frames
        * calculating useful quantities like airspeed, groundspeed, energy relationships etc
        * adding force and moment constraint relationships to the model based on state variables
        * state, state derivative, and control relationships
        * setup for feeding to aerosandbox.opti call format

    This class also should encode important geometric quantities of the whatever the point mass represents like the
    mass, inertia, etc.
    """

    def __init__(self,
                 mass: float = 1,
                 Iyy: float = 0,
                 earth_x_position: Union[float, np.ndarray] = 0,
                 earth_z_position: Union[float, np.ndarray] = 0,
                 body_x_velocity: Union[float, np.ndarray] = 0,
                 body_z_velocity: Union[float, np.ndarray] = 0,
                 pitch: Union[float, np.ndarray] = 0,
                 pitch_rate: Union[float, np.ndarray] = 0,
                 ):
        """
        Initialize class and add variable arrays to properties
        :param mass: [kg] model mass
        :param Iyy: [kg*m^2] rotational inertia about the y-axis
        :param earth_x_position: [m] earth-x position relative to arbitrary 0, + to the right
        :param earth_z_position: [m] earth-z position relative to ground (0), + downard, - upward
        :param body_x_velocity: [m] x-component of body velocity in the earth frame + out of the nose
        :param body_z_velocity: [m] z-component of body velocity in the earth frame + out bottom of pointmass
        :param Pitch: [deg] pitch of moddel nose relative to earth axis, + up
        :param pitch_rate: [deg/s] pitch rate
        """

        # inertial info
        self.Mass = mass
        self.Iyy = Iyy

        # state variables
        self.EarthXPosition = earth_x_position
        self.EarthZPosition = earth_z_position
        self.BodyXVelocity = body_x_velocity   # body x-velocity in [m/s] + forward
        self.BodyZVelocity = body_z_velocity
        self.Pitch = pitch
        self.PitchRate = pitch_rate

        # control variables
        self.Fx_b = 0  # [N] x-force aligned with body axes in the earth frame
        self.Fz_b = 0  # [N] z-force aligned with body axes in the earth frame
        self.My_b = 0  # [N] y-moment (pitching) in the body / earth frame (equivalent in 2D)

    @property
    def state(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Property which returns state values as a dictionary. Note for edits that dict key and property must match
        exactly
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
        """
        Property which returns control values as a dictionary. Note that for edits that dict key and property name
        must match
        :return:
        """
        return {
            "Fx_b": self.Fx_b,
            "Fz_b": self.Fz_b,
            "My_b": self.My_b,
        }

    def state_derivatives(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Method which returns state derivative relationships as a dictioanry. Note that dict keys must match
        Property names exactly
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
    def EarthXVelocity(self) -> Union[float, np.ndarray]:
        """
        u-velocity of the body in earth coords
        :return:
        """
        ue,ve = self.convert_axes(self.BodyXVelocity,self.BodyZVelocity,from_axes='body',to_axes='earth')
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
        get alpha in deg from flight u_b and w_b
        :return:
        """
        return np.arctan2d(self.BodyZVelocity, self.BodyXVelocity)

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
            return x_from,z_from

        # precalculate sin,cos values and rotation matrices
        st = np.sind(-self.Pitch)
        ct = np.cosd(-self.Pitch)

        # if in and out are not the same, start logic to transform
        if from_axes in ['stability', 'wind']:
            # need to use the negative of the incoming vector since the ordinate system is flipped relative to body orientation
            #Twb = [ cos(alfa) -sin(alfa) ]
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

        return x_to,z_to

    def old_convert_axes(self,
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
        st = np.sind(self.Pitch)
        ct = np.cosd(self.Pitch)

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
    def op_point(self):
        """
        Returns an OperatingPoint object that represents the current state of the dynamics instance.

        This OperatingPoint object is effectively a subset of the state variables, and is used to compute aerodynamic
        forces and moments.
        """
        return OperatingPoint(
            atmosphere=Atmosphere(altitude=-self.EarthZPosition),
            velocity=self.Speed,
            alpha=self.Alpha,
            beta=0,
            p=0,
            q=self.PitchRate,
            r=0,
        )

    @property
    def Speed(self):
        """
        Speed in the earth inertial frame. This comes from subtracting out any gust velocity from the body in the
        body frame.

        V_earth = Vbody - Vgust
        :return:
        """

        # convert from body to earth coordinates
        return np.sqrt(self.BodyXVelocity ** 2 + self.BodyZVelocity ** 2)

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


    @property
    def KE(self)->Union[float,np.ndarray]:
        """
        return KE of pointmass
        :return:
        """
        return 0.5 * self.Mass * self.Speed**2

    @property
    def PE(self) -> Union[float, np.ndarray]:
        """
        potential energy of system based on alititude above ze = 0
        :return:
        """

        return -self.Mass*self.EarthZPosition*9.81

    @property
    def TE(self) -> Union[float,np.ndarray]:
        """
        system total energy
        :return:

        """

        return self.KE + self.PE

if __name__ == "__main__":
    dyn = EulerBody2D()
