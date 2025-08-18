import unittest
from aerosandbox import numpy as np
from trajopt.dynamics import Aircraft2DPointMass
from aerosandbox import Opti
from trajopt.dynamics import ThinAirfoilModel


class MyTestCase(unittest.TestCase):

    def setUp(self):
        """
        Set up aircraft to be used for tests
        :return:
        """

        # problem setup
        N = 10
        self.Opti = Opti()
        self.Time = np.linspace(0,10,N)

        self.PointMassModel = Aircraft2DPointMass(
            mass=10, # kg
            Iyy = 1, #N-m
            x_earth= np.ones((N,)),
            z_earth=100 * np.ones((N,)),
            w_body= np.zeros((N,)),
            u_body= np.ones((N,)),
            pitch = np.zeros((N,)),
            flap_deflection= np.zeros((N,)),
            elevator_deflection= np.zeros((N,)),
            throttle_position= np.zeros((N,))
        )

        self.PointMassModel.Span = 3.05
        self.PointMassModel.ChordMean = 0.38
        self.PointMassModel.TailSpan = 1.27
        self.PointMassModel.TailChordMean = 0.25
        self.PointMassModel.Area = 1.09
        self.PointMassModel.TailArea = 0.32

    def testWingModel(self):
        """
        test the results for the wing model
        :return:
        """

        # pull in model
        aeromodel = ThinAirfoilModel()

        # set params
        # set alpha to 0, Vinf to 10
        alpha = np.zeros(self.PointMassModel.BodyXVelocity.shape)
        Vinf = 10 * np.ones(self.PointMassModel.BodyXVelocity.shape)
        self.PointMassModel.BodyZvelocity = Vinf * np.sind(alpha)
        self.PointMassModel.BodyXVelocity = Vinf * np.cosd(alpha)
        cl,cd,cm = aeromodel.wingDynamicsModel(self.PointMassModel)

        print(cl)
        print(cd)
        print(cm)
        print('--------')

        # set alpha to 5, Vinf to 10
        alpha = 5 * np.ones(self.PointMassModel.BodyXVelocity.shape)
        Vinf = 10 * np.ones(self.PointMassModel.BodyXVelocity.shape)
        self.PointMassModel.BodyZvelocity = -Vinf * np.sind(alpha)
        self.PointMassModel.BodyXVelocity = Vinf * np.cosd(alpha)
        cl, cd, cm = aeromodel.wingDynamicsModel(self.PointMassModel)

        print(cl)
        print(cd)
        print(cm)
        print('--------')

        # set alpha to 5, Vinf to 10
        alpha = 10 * np.ones(self.PointMassModel.BodyXVelocity.shape)
        Vinf = 10 * np.ones(self.PointMassModel.BodyXVelocity.shape)
        self.PointMassModel.BodyZvelocity = -Vinf * np.sind(alpha)
        self.PointMassModel.BodyXVelocity = Vinf * np.cosd(alpha)
        self.PointMassModel.FlapPosition = -10 * np.ones(self.PointMassModel.FlapPosition.shape)
        cl, cd, cm = aeromodel.wingDynamicsModel(self.PointMassModel)

        print(cl)
        print(cd)
        print(cm)
        print('--------')


if __name__ == '__main__':
    unittest.main()
