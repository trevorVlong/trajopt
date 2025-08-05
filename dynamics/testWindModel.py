import unittest
from weather.WindModel2D import WindModel2D
from weather.WindMetaModel import WindMetaModel
import numpy as np


class TestWindModel(unittest.TestCase):
    def setUp(self):
        """
        """
        self.SimpleModel = WindModel2D()
        self.SecondModel = WindModel2D()
        self.MetaModel = WindMetaModel()

    def testClassInit(self):
        """

        :return:
        """

        self.assertIsInstance(self.SimpleModel, WindModel2D)

        # check default values
        self.assertEqual(self.SimpleModel.Center,0)
        self.assertEqual(self.SimpleModel.STD,0)
        self.assertEqual(self.SimpleModel.CurrentModel,"gaussian1D")

    def test1DGauss(self):
        params = {"std":2,"center":10}
        x = np.linspace(0,20,100)
        self.SimpleModel.setParameters("gaussian1D",**params)

        self.assertEqual(self.SimpleModel.Center,params["center"])
        self.assertEqual(self.SimpleModel.STD, params["std"])

        # run the gaussian
        wind = self.SimpleModel.windSpeed(x=x)

        self.assertEqual(len(wind[0]),len(x))
        self.assertEqual(len(wind[0]),len(wind[1]))
        self.assertEqual(len(wind[0]), len(wind[2]))

    def testNoWind(self):
        params = {"std": 2, "center": 10}
        x = np.linspace(0, 20, 100)
        self.SimpleModel.setParameters("none", **params)

        self.assertEqual(self.SimpleModel.Center, params["center"])
        self.assertEqual(self.SimpleModel.STD, params["std"])

        # run the gaussian
        wind = self.SimpleModel.windSpeed(x=x,y=x,z=x)

        self.assertEqual(len(wind[0]), len(x))
        self.assertEqual(len(wind[0]), len(wind[1]))
        self.assertEqual(len(wind[0]), len(wind[2]))

        # check that all values are zero for all three axes
        for wind_ax in range(3):
            self.assertTrue(np.equal(wind[wind_ax],np.zeros(x.shape)).all())
    
    def testMetaModel(self):
        """
        test metamodel which links together several regular wind mdoels
        :return: 
        """
        wind_params_1 = {"center":10.,"std":10.,"MaxGustVelocity":20.}
        wind_params_2 = {"center": 10, "std": 10, "MaxGustVelocity": 30}

        # test add models
        self.MetaModel.addModel("model1",self.SimpleModel)
        self.MetaModel.addModel("model2", self.SecondModel)

        # see if adding models together
        self.SimpleModel.setParameters("gaussian1D",**wind_params_1)
        self.SecondModel.setParameters("gaussian1D",**wind_params_2)

        vel = self.MetaModel.windSpeed(10,0,0)

        self.assertEqual(50,vel[0])

        # test remove model
        self.MetaModel.removeModel("model2")
        self.assertEqual(1, len(self.MetaModel.ModelDict))

        # test adding models at different axes
        wind_params_2 = {"avg": 10, "std": 10, "MaxGustVelocity": 30,"axis":"z"}
        self.SecondModel.setParameters("gaussian1D",**wind_params_2)
        self.MetaModel.addModel("model2",self.SecondModel)

        vel = self.MetaModel.windSpeed(10, 0, 10)
        self.assertEqual(20,vel[0])
        self.assertEqual(30, vel[2])


if __name__ == '__main__':
    unittest.main()
