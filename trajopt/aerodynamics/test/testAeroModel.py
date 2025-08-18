import unittest
from AeroModelling.AeroModelAbstract import AeroModel, BaseModel
from trajopt.dynamics import Aircraft2DPointMass
from aerosandbox import numpy as np
from aerosandbox import Opti
from typing import Union


# if TYPE_CHECKING:
#     from dynamics.rigid_body_dynamics.Aircraft2D import Aircraft2DPointMass

class MyTestCase(unittest.TestCase):


    def setUp(self):
        """
        do a standard setup of the testModel
        :return:
        """
        self.model = BaseModel()

    def tearDown(self):

        del self.model

    def test_AeroModelInit(self):
        """
        make sure is
        :return:
        """

        self.assertIsInstance(self.model, object)
        self.assertIsInstance(self.model,AeroModel)
        self.model.wingDynamicsModel = lambda x: 0

    def testImportFunctions(self):
        """
        test importing functions for the different submodels
        :return:
        """

        example_dynamics_model = lambda x,y: x + y + 1
        example_dynamics_model2 = lambda x, y: x + y + 2

        self.model.wingDynamicsModel = example_dynamics_model
        self.model.tailDynamicsModel = example_dynamics_model2

        # make sure function is actually imported
        self.assertTrue(callable(self.model.wingDynamicsModel))
        self.assertTrue(callable(self.model.tailDynamicsModel))


        # make sure function reports what it should
        x = 1
        y = 1

        # for model 1
        self.assertEqual(self.model.wingDynamicsModel(x,0),2)
        self.assertEqual(self.model.wingDynamicsModel(0, y), 2)
        self.assertEqual(self.model.wingDynamicsModel(0, 0), 1)

        # for model 2
        self.assertEqual(self.model.tailDynamicsModel(x, 0), 3)
        self.assertEqual(self.model.tailDynamicsModel(0, y), 3)
        self.assertEqual(self.model.tailDynamicsModel(0, 0), 2)

    def testSimpleOperation(self):
        """
        test setup with CASADi variables
        :return:
        """

        # set up an optimization problem using the 2D point mass model
        opti = Opti()
        N = 2
        dyn = Aircraft2DPointMass(
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

        # ===========================================
        # example function and assign to aero module

        def sumCasadi(dynamics:Aircraft2DPointMass):

            # some really simple math
            const = 1
            func = dynamics.EarthXPosition + dynamics.EarthZPosition + const
            y = np.ndarray(func.shape)
            z = np.ndarray(func.shape)

            return func, y, z

        self.model.fullDynamicsModel = sumCasadi

        # ===========================================
        # call simple function analyze output
        out1,out2,out3 = self.model.fullDynamicsModel(dyn)

        shape = out1.shape

        # make sure shape is correct
        self.assertEqual(shape[0],2)
        self.assertEqual(shape[1], 1)
        self.assertEqual(shape,out2.shape)


        # call again with values for dyn parameters
        arr1 = np.array([[1],[1]])
        arr2 = np.array([[2], [0]])
        dyn.EarthXPosition = arr1
        dyn.EarthZPosition = arr2

        out1, out2, out3 = self.model.fullDynamicsModel(dyn)
        shape_values = out1.shape

        # assert that shapes are consistent with symbolic representation
        self.assertEqual(shape_values[0], 2)
        self.assertEqual(shape_values[1], 1)
        self.assertEqual(shape, shape_values)

        # assert values is what is expected
        expected_result = np.array([[4],[2]])
        self.assertTrue(all(expected_result==out1))

    def testNestedModels(self):
        """
        test models which use multiple functions which are co-dependent to reach solutions. This is representative of
        the coupled aero systems. For instance, induced drag is a function of lift coefficient and so the drag model
        will pull in results from the lift model, and so on
        :return:
        """

        # set up an optimization problem using the 2D point mass model
        opti = Opti()
        N = 2
        dyn = Aircraft2DPointMass(
            mass=10,  # kg
            Iyy=2.5,  # kg-m^2
            x_earth=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            z_earth=opti.variable(init_guess=2 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            w_body=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            u_body=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            pitch=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            flap_deflection=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            elevator_deflection=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
            throttle_position=opti.variable(init_guess=1 * np.ones((N,)), upper_bound=1e4, lower_bound=-1),
        )

        # ===========================================
        def exampleFunc1(dynamics:Aircraft2DPointMass):
            """
            representative of lift-producing function
            :param dynamics:
            :return:
            """
            # some really simple math
            const = 1
            value = dynamics.EarthXPosition + dynamics.EarthZPosition + const

            return value

        def exampleFunc2(dynamics: Aircraft2DPointMass,
                         value: Union[np.ndarray, float],
                         ):
            """
            representative of CDi, which is a function of aircraft geometry (point mass property) and CL (a
            separately computed function)
            :param dynamics:
            :param value:
            :return:
            """

            # some really simple math

            return value**2 + dynamics.Mass

        def complexDynamicsExample(dynamics:Aircraft2DPointMass):
            val = exampleFunc1(dynamics)
            complex_val = exampleFunc2(dynamics,val)

            # other arrays that would be returned
            y = z = np.zeros(dynamics.EarthXPosition.shape)
            return complex_val,y,z

        # set lift and drag functions to be defined here
        self.model.WingLiftCoefficientFunc = exampleFunc1
        self.model.WingDragCoefficientFunc = exampleFunc2
        self.model.fullDynamicsModel = complexDynamicsExample

        # ===========================================
        # run with CASADi variables
        intermediate = self.model.WingLiftCoefficientFunc(dyn)
        final_val = self.model.WingDragCoefficientFunc(dyn,intermediate)
        out1, out2, out3 = self.model.fullDynamicsModel(dyn)

        # check that shapes are consistent

        self.assertTrue(out1.shape[0] == dyn.EarthXPosition.shape[0])
        self.assertTrue(out1.shape[1] == dyn.EarthXPosition.shape[1])

        # try again with values replacing symbolic math
        arr1 = np.array([[1], [1]])
        arr2 = np.array([[2], [0]])
        dyn.EarthXPosition = arr1
        dyn.EarthZPosition = arr2

        # run
        intermediate_valued = self.model.WingLiftCoefficientFunc(dyn)
        final_val_valued = self.model.WingDragCoefficientFunc(dyn, intermediate)
        out1_valued, out2_valued, out3_valued = self.model.fullDynamicsModel(dyn)

        self.assertTrue(out1.shape[0] == out1_valued.shape[0])
        self.assertTrue(out1.shape[1] == out1_valued.shape[1])

        expected_values = np.array([[16],[4]]) + 10
        self.assertTrue(all(expected_values == out1_valued))


if __name__ == '__main__':
    unittest.main()
