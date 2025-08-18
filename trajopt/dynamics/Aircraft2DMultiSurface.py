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
from trajopt.weather.WindModel2D import WindModel2D
from trajopt.dynamics import Aircraft2DPointMass


class Aircraft2DMultiSurface(Aircraft2DPointMass):
    """
    A point-mass representation of a 2-dimensional aircraft with arbitrary pitch/thrust control inputs for use in
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
                 x_earth: Union[float, np.ndarray] = 0,  # earth x-position [m] in Earth Frame
                 z_earth: Union[float, np.ndarray] = 0,  # earth z-position [m] in Earth Frame
                 u_body: Union[float, np.ndarray] = 0,  # x-component of body velocity [m/s] in the Earth Frame
                 w_body: Union[float, np.ndarray] = 0,  # z-component of body velocity [m/s] in the Earth Frame
                 pitch: Union[float, np.ndarray] = 0,  # y-component of Euler angle sequence [deg] + pitch up from earth x-axis
                 pitch_rate: Union[float, np.ndarray] = 0, # pitch rate [deg/s]
                 flap_deflection_inner: Union[float, np.ndarray] = 0,  # flap deflection  [deg] + down
                 flap_deflection_outer: Union[float, np.ndarray] = 0,
                 elevator_deflection: Union[float, np.ndarray] = 0,  # horizontal tail deflection [deg] + down
                 throttle_position_inner: Union[float, np.ndarray] = 0,  # throttle position [% from 0-1]
                 throttle_position_outer: Union[float, np.ndarray] = 0,  # throttle position [% from 0-1]
                 air_density:Union[float,np.ndarray] = 0,  # temporary stand in
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
        self.FlapPositionInner = flap_deflection_inner
        self.FlapPositionOuter = flap_deflection_outer
        self.ThrottlePositionInner = throttle_position_inner
        self.ThrottlePositionOuter = throttle_position_outer
        self.ElevatorPosition = elevator_deflection

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
            "EarthXPosition": self.EarthXPosition, # x-position in [m/s] + forward
            "EarthZPosition": self.EarthZPosition,  # z-position in [m/s] + down (use self.Altitude for position above
            # ground)
            "BodyXVelocity": self.BodyXVelocity,
            "BodyZVelocity": self.BodyZVelocity,
            "Pitch": self.Pitch,
            "PitchRate": self.PitchRate
        }

    @property
    def control_variables(self) -> Dict[str, Union[float, np.ndarray]]:
        """
        Method to return dictionary of the control vector
        :return:
        """
        #TODO procedurarlly add flap and motor positions in the control variables
        return {
            "Fx_b": self.Fx_b,
            "Fz_b": self.Fz_b,
            "My_b": self.My_b,
            "FlapPositionInner": self.FlapPositionInner,
            "FlapPositionOuter": self.FlapPositionOuter,
            "ElevatorPosition": self.ElevatorPosition,
            "ThrottlePositionInner": self.ThrottlePositionInner,
            "ThrottlePositionOuter": self.ThrottlePositionOuter
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


if __name__ == "__main__":

    dyn = Aircraft2DPointMass()
