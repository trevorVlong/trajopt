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
from typing import Union, Dict
from dynamics import PointMass2D
from weather.WindModel2D import WindModel2D
import casadi as cas


class Aircraft2DPointMass(PointMass2D):
    """
    Expansion of the Aircraft2D class which allows for arbitrary numbers of control on the flaps / motors. A
    point-mass representation of a 2-dimensional aircraft with arbitrary pitch/thrust control inputs for use in
    trajectory optimizaiton problems with wind disturbances

    This class contains methods and properties pertaining to the rigid body motion of an aircraft in 2-dimensions.
    The class contains methods for coordinate transformations between axes, generating important values, and setting
    up the dynamics systems to send to the optimizer. Additionally the model contains methods for introducing wind
    velocity fields to the problem

    Coordinate Systems:
        - Earth: +x forward, +z down
        -  Body: +x forward, +z down
        -  Aero: +x along freestream, +z up
        - Stability: same as aero in 2D

    """

    def __init__(self, mass: float = 1,
                 Iyy: float = 0,
                 x_earth: Union[float, np.ndarray] = np.zeros(1,),  # earth x-position [m] in Earth Frame
                 z_earth: Union[float, np.ndarray] = np.zeros(1,),  # earth z-position [m] in Earth Frame
                 u_body: Union[float, np.ndarray] = np.zeros(1,),  # x-component of body velocity [m/s] in the Earth Frame
                 w_body: Union[float, np.ndarray] = np.zeros(1,),  # z-component of body velocity [m/s] in the Earth Frame
                 pitch: Union[float, np.ndarray] = np.zeros(1,),  # y-component of Euler angle sequence [deg] + pitch up from earth x-axis
                 pitch_rate: Union[float, np.ndarray] = np.zeros(1,), # pitch rate [deg/s]
                 flap_deflection: Union[float, np.ndarray] = np.zeros(1,),  # flap deflection  [deg] + down
                 elevator_deflection: Union[float, np.ndarray] = np.zeros(1,),  # horizontal tail deflection [deg] + down
                 throttle_position: Union[float, np.ndarray] = np.zeros(1,),  # throttle position [% from 0-1]
                 air_density:Union[float,np.ndarray] = np.zeros(1,),  # temporary stand in
                 windModel: WindModel2D = WindModel2D(),
                 ):

        # inertial characteristics
        super().__init__(mass, Iyy, x_earth, z_earth, u_body, w_body, pitch, pitch_rate)

        # ======================================================================
        # geometric info

        # wing geometric information
        self.Span: float = 0 # aircraft
        self.ChordMean: float = 0
        self.Area: float = 0

        # tail
        self.TailSpan: float = 0
        self.TailChordMean: float = 0
        self.TailArea: float = 0

        # control surfaces and controls
        self.FlapPosition = flap_deflection
        self.ElevatorPosition = elevator_deflection
        self.ThrottlePosition = throttle_position

        # ----------------------------------------------
        # other tracked information
        # air density and other state stuff (temporary messy code until I switch things over to internal operating
        # point class)

        if air_density == 0:
            self.AirDensity = 1.228*np.ones(self.EarthXPosition.shape)
        else:
            self.AirDensity = air_density

        # wind model property
        self.WindModel = windModel
        windModel.setParameters()

    @property
    def control_variables(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Method to return dictionary of the control vector
        :return:
        """
        return {
            "Fx_b": self.Fx_b,
            "Fz_b": self.Fz_b,
            "My_b": self.My_b,
            "FlapPosition": self.FlapPosition,
            "ElevatorPosition": self.ElevatorPosition,
            "ThrottlePosition": self.ThrottlePosition
        }

    @property
    def _OptiVars(self) -> Dict[str,Union[float, cas.MX]]:
        """
        returns a dict containting the name/value pairs for all assumed opti-variables
        """

        return {
            "EarthXPosition": self.EarthXPosition,
            "EarthZPosition": self.EarthZPosition,
            "BodyXVelocity": self.BodyXVelocity,
            "BodyZVelocity": self.BodyZVelocity,
            "Pitch": self.Pitch,
            "PitchRate": self.PitchRate,
            "FlapPosition": self.FlapPosition,
            "ElevatorPosition": self.ElevatorPosition,
            "ThrottlePosition": self.ThrottlePosition
        }

    @property
    def Altitude(self):
        """
        Altitude in [m] as a positive number since earth position is defined as + down and - up. Only applies the -1
        transformation in the vertical earth direction and returns
        :return:
        """
        return -self.EarthZPosition

    @property
    def glide_slope(self):
        """
        glide slope of the body defined as the arctan of vertical speed in earth frame over horizontal speed in earth
         frame, with positive down by convention
        :return:
        """
        return np.arctan2d(self.EarthZVelocity, self.EarthXVelocity)

    @property
    def WingAspectRatio(self):
        """
        get the wing aspect ratio from geometric information. Uses the general AR = b^2 / S formulation
        :return:
        """

        return self.Span**2 / self.Area

    @property
    def TailAspectRatio(self):
        """
        get the tail aspect ratio from geometric information. Uses the general AR = b^2 / S formulation
        :return:
        """

        return self.TailSpan ** 2 / self.TailArea

    def gustVelocity(self,
                     x_earth:Union[float,np.ndarray] = None,
                     altitude:Union[float,np.ndarray] = None,
                     to_axes='earth'
                     ):
        """
        Returns the gust velocity at the field point (x_earth, altitude) in specified axes. User can
        :param x_earth:
        :param z_earth:
        :return:
        """
        if x_earth is None:
            x_earth = self.EarthXPosition
        elif altitude is None:
            altitude = self.Altitude

        gust_x, gust_z, gust_z = self.WindModel.windSpeed(x_earth,altitude,np.zeros(x_earth.shape))
        u_gust, w_gust = self.convert_axes(gust_x,gust_z,from_axes='earth',to_axes=to_axes)

        return u_gust, w_gust

    @property
    def DynamicPressure(self):
        """
        Returns array containing dynamic pressure equations of the problem / values of the solution in SI
        [N/m^2]. Includes the velocity associated with gusts (see self.Airspeed for details)

        qinf = 1/2 * air_density * V_TAS^2
        :return:
        """

        return 0.5 * self.Airspeed**2 * self.AirDensity

    @property
    def Airspeed(self):
        """
        returns the true airspeed of the point mass which in this formulation is the body velocity + gust velocity at
        the point. Does not take into account any compressibility or corrected airspeed effects.

        VTAS = airspeed velocity vector in aircraft frame
        Vb = body velocity vector in body axes in the earth frame
        Vgust = gust velocity vector in earth coords in the earth frame
        Teb = rotation tensor from earth to body coords
        []_b = body components of []

        |VTAS| = |Vb - Teb*Vgust|
        |VTAS| = sqrt( (u_b-u_gb)^2 + (w_b - w_gb)^2 )

        intuitively this makes sense if you consider that if one flies into a unit step-function of tailwind in the
        moment before you enter the tailwind you see x-velocity component u, the moment after you enter your airspeed
         has decreased by u_g but you speed in the earth frame remains u_b while the airspeed decreases by u_g
        :return:
        """

        # unpack useful values
        u_b = self.BodyXVelocity  # body x-velocity without gust present
        w_b = self.BodyZVelocity  # body z-velocity without gust present
        u_g,w_g = self.gustVelocity(self.EarthXPosition, self.Altitude, 'body')  # get gust velocity in body axes

        # return airspeed which is: V_b - V_g
        return np.sqrt((u_b-u_g)**2 + (w_b-w_g)**2)

    @property
    def Alpha(self) -> Union[float, np.ndarray]:
        """
        Property which returns Angle of Attack in [deg] using body velocity components and gust velocities. The
        following equation is defined in the body frame

        AoA = arctan((wb-wg)/(ub-ug))

        :return:
        """
        # get gust velocity
        u_g,w_g = self.gustVelocity(self.EarthXPosition, self.Altitude, to_axes='body')  # get gust velocity in body

        # return AoA in deg
        return np.arctan2d(self.BodyZVelocity - w_g, self.BodyXVelocity - u_g)

    def setVariables(self,
                     variable_dict:dict,
                     ) -> None:
        """
        sets state and control variables from a dictionary of inputs
        """
        for var, val in variable_dict.items():
            self.__dict__[var] = val

if __name__ == "__main__":

    dyn = Aircraft2DPointMass()

    print(dyn.state.keys())
