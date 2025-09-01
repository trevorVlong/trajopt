import unittest
from aerosandbox import numpy as np
from aerosandbox import Opti
from trajopt.dynamics import Aircraft2DPointMass

class MyTestCase(unittest.TestCase):

    def setUp(self):

        # problem setup
        N = 10
        opti = Opti()
        self.Time = np.linspace(0,10,N)

        self.PointMassModelNumeric = Aircraft2DPointMass(
            mass=10, # kg
            Iyy = 1, #N-m
            x_earth= np.linspace(0,100,N),
            z_earth=100 * np.ones((N,)),
            w_body= np.zeros((N,)),
            u_body= 10*np.ones((N,)),
            pitch = np.zeros((N,)),
            flap_deflection= np.zeros((N,)),
            elevator_deflection= np.zeros((N,)),
            throttle_position= np.zeros((N,))
        )

        self.PointMassModelOpti = Aircraft2DPointMass(
            mass=9.54,  # kg
            Iyy=2.5,  # kg-m^2
            x_earth= opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            z_earth= opti.variable(init_guess=2 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            w_body= opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            u_body= opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            pitch = opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            flap_deflection= opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            elevator_deflection= opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            throttle_position=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
        )


        # add geometric quantities

        self.PointMassModelOpti.Span = self.PointMassModelNumeric.Span = 3.05
        self.PointMassModelOpti.ChordMean = self.PointMassModelNumeric.ChordMean = 0.38
        self.PointMassModelOpti.TailSpan = self.PointMassModelNumeric.TailSpan = 1.27
        self.PointMassModelOpti.TailChordMean = self.PointMassModelNumeric.TailChordMean = 0.25
        self.PointMassModelOpti.Area = self.PointMassModelNumeric.Area = 1.09
        self.PointMassModelOpti.TailArea = self.PointMassModelNumeric.TailArea = 0.32

        # other
        self.PointMassModelOpti.AirDensity = self.PointMassModelNumeric.AirDensity = 1.225

    def testGeometricProperties(self):
        """
        Warm up just to make sure any derivative properties work as intended
        :return:
        """

        # wing aspect ratio
        AR = self.PointMassModelNumeric.WingAspectRatio
        self.assertAlmostEqual(AR,8.53,delta=0.009)  # add assertion here

        # tail aspect ratio
        AR = self.PointMassModelNumeric.TailAspectRatio
        self.assertAlmostEqual(AR,5.04,delta=0.009)

    def testWindEffect(self):
        """
        test to make sure wind fields act as intended.
        :return:
        """

        # get pointmass class
        pointmass = self.PointMassModelNumeric
        pointmass_old = pointmass.deepcopy()

        # =========================================================================================
        # add a wind model of a vertical gust, see WindModel for description of how the model works

        # vertical gust (- is upward, + is downward to match earth axes)
        pointmass.WindModel.setParameters('gaussian1D',**{'std':10,'center':50,'MaxGustVelocity':-5,'axis':'z'})

        # check that x-velocity doesn't change
        self.assertTrue(all(pointmass.BodyXVelocity == pointmass_old.BodyXVelocity))
        self.assertTrue(any(pointmass.Airspeed > pointmass_old.Airspeed)) # should have several points with vertical
        self.assertTrue(any(pointmass.Alpha > pointmass_old.Alpha))
        # gust where airspeed increases due to gust

        # -------------------------------------------
        # check horizontal gust, (- is adverse, + is along)
        pointmass = pointmass_old.deepcopy()

        # adverse x-gust
        pointmass.WindModel.setParameters('gaussian1D',**{'std':10,'center':50,'MaxGustVelocity':-5,'axis':'x'})
        self.assertTrue(all(pointmass.BodyXVelocity == pointmass_old.BodyXVelocity))
        self.assertTrue(any(pointmass.Airspeed > pointmass_old.Airspeed))
        self.assertTrue(any(pointmass.Alpha == pointmass_old.Alpha))

        # following x-gust
        pointmass = pointmass_old.deepcopy()
        pointmass.WindModel.setParameters('gaussian1D', **{'std': 10, 'center': 50, 'MaxGustVelocity': 5, 'axis': 'x'})
        self.assertTrue(all(pointmass.BodyXVelocity == pointmass_old.BodyXVelocity))
        self.assertTrue(any(pointmass.Airspeed < pointmass_old.Airspeed))
        self.assertTrue(any(pointmass.Alpha == pointmass_old.Alpha))

if __name__ == '__main__':
    unittest.main()
