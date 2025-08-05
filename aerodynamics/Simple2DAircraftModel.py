# Created by trevorlong on 5/1/25
# license
# Copyright 2025 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from Airfoil2DSection import Airfoil2DSection
from airfoilPerformanceModels import liftCoefficientFlapAlfa,dragCoefficientFlapAlfa,momentCoefficientFlapAlfa
from dynamics.RigidMotion.Aircraft2D import Aircraft2DPointMass

class Simple2DAircraftModel():
    """
    Simple 2D aircraft model that consists of a wing,tail each with flaps and a motor which is mounted on the wing.
    This model operates at a single reynolds number, as such Re is not considered for any calculations
    """
    def __init__(self):

        # surfaces and models
        self.Wing = Airfoil2DSection()
        self.Tail = Airfoil2DSection()

        # geometric information


    def liftCoeff(self,dyn:Aircraft2DPointMass):
        """
        gets lift coefficient from angle of attack, main flap deflection, tail surface deflection,
        :return:
        """

        # calculate wing CL
        params = self.Wing.FittedModels['CL'].parameters
        x = {"alpha": dyn.Alpha, "flap_ang": dyn.FlapPosition}
        clwing = liftCoefficientFlapAlfa(x,params)



        # calculate tail CL

        # calculate wing downwash at tail 1/4 chord

        params = self.Tail.FittedModels['CL'].parameters
        x = {"alpha": dyn.Alpha + downwash, "flap_ang": dyn.ElevatorPosition}
        clwing = liftCoefficientFlapAlfa(x, params)



    def liftForce(self,dyn):
        """
        calculates lift force
        :param dyn:
        :return:
        """

        q = dyn.DynamicPressure


    def dragCoeff(self):

    def dragForce(self):

    def pitchCoeff(self):

    def thrustForce(self):