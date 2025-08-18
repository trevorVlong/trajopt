import unittest
from aerosandbox import numpy as np
from trajopt.dynamics import Aicraft2D

class MyTestCase(unittest.TestCase):

    def setUp(self):
        self.N = 5
        self.EulerClass = Aicraft2D(
            mass=1,
            x_e=np.zeros((self.N,)),
            z_e=np.zeros((self.N,)),
            u_b=np.zeros((self.N,)),
            w_b=np.zeros((self.N,)),
            pitch=np.zeros((self.N,))
        )

    def testEulerRotations(self):
        """
        Check that rotations are correct
        """

        # check pitch-only rotations from body to other axes
        self.EulerClass.Pitch = np.array([0, 30, 45, 60, 90])
        self.EulerClass.BodyXVelocity = np.ones((self.N,))
        self.EulerClass.BodyZvelocity = np.zeros((self.N,))

        ue,ve,we = self.EulerClass.newConvertAxes(self.EulerClass.BodyXVelocity, self.EulerClass.BodyZvelocity, 'body', 'earth')
        ub,vb,wb = self.EulerClass.newConvertAxes(ue,we,'earth','body')

        # check transformations
        ue_expected = np.array([1,np.sqrt(3)/2,np.sqrt(2)/2,1/2,0])
        we_expected = np.array([0,1/2,np.sqrt(2)/2,np.sqrt(3)/2,1])
        [self.assertAlmostEqual(ue[idx],ue_expected[idx]) for idx in range(self.N)] # check ue is what we would
        # expected
        [self.assertAlmostEqual(we[idx], we_expected[idx]) for idx in range(self.N)] # check we is what would be
        # expected

        # check inverse transform
        [self.assertEqual(ub[idx], self.EulerClass.BodyXVelocity[idx]) for idx in range(self.N)]
        [self.assertEqual(wb[idx], self.EulerClass.BodyZvelocity[idx]) for idx in range(self.N)]



        # check with angle of attack
        alpha = np.array([0,5,10,25,45])
        self.EulerClass.Pitch = 5 * np.ones((self.N,))
        self.EulerClass.BodyXVelocity = np.cosd(alpha)
        self.EulerClass.BodyZvelocity = np.sind(alpha)

        ue,out2,we = self.EulerClass.newConvertAxes(np.ones((self.N,)),np.zeros((self.N,)),'wind','earth')
        ub,out2,wb = self.EulerClass.newConvertAxes(ue,we,'earth','wind')
        ue_expected = -np.cosd(alpha+5)
        we_expected = -np.sind(alpha + 5)

        # check wind-to-earth rotation and inverse rotation
        [self.assertAlmostEqual(ue[idx], ue_expected[idx]) for idx in range(self.N)]
        [self.assertAlmostEqual(we[idx], we_expected[idx]) for idx in range(self.N)]
        [self.assertAlmostEqual(ub[idx], 1) for idx in range(self.N)]
        [self.assertAlmostEqual(wb[idx], 0) for idx in range(self.N)]
        print('done')




if __name__ == '__main__':
    unittest.main()
