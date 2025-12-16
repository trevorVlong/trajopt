from trajopt.aerodynamics.AeroModelAbstract import AeroModel
from typing import TYPE_CHECKING,Union,Tuple
from aerosandbox import numpy as np
from trajopt.aerodynamics.dragModelling import inducedDrag
from trajopt.aerodynamics.ThinAirfoilAnalytical2D import liftCoeffFlappedThinAirfoil,pitchCoeffFlappedThinAirfoil
from trajopt.aerodynamics.propulsorModels import scaledPropulsorPoint
from trajopt.aerodynamics.courtinSurrogates import liftCoeff,dragCoeff,pitchingCoeff
if TYPE_CHECKING:
    from trajopt.dynamics.Aircraft2D import Aircraft2DPointMass


class BlownAirfoilModel(AeroModel):
    """
    An aerodynamics model for an aircraft using thin-airfoil theory used to set the dynamics constraints for the
    optimizer model. This particular class is set as an archetypical example of how to build these classes for more
    complicated models. There are methods for getting forces, coefficients, and other values of interest if handed a
    dynamics instance.

    If creating a new model, it is useful to copy this class and fill it out in a new file giving it a new name.
    """
    def __init__(self):

        # ========================================
        # functions for individual components are properties that can then be ingested by the dynamics functions later

        # main wing
        self.WingLiftCoeffFunction: callable = liftCoeff
        self.WingDragCoeffFunction: callable = dragCoeff
        self.WingMomentCoeffFunction: callable = pitchingCoeff

        # tail
        self.TailLiftCoeffFunction: callable = liftCoeffFlappedThinAirfoil(E=0.3)
        self.TailDragCoeffFunction: callable = lambda cl,AR,e,cd0: inducedDrag(cl,AR,e) + cd0
        self.TailMomentCoeffFunction: callable = pitchCoeffFlappedThinAirfoil(E=0.3)

        # engine
        self.PropulsorModel: callable = scaledPropulsorPoint(thrust_to_weight_ratio=0.5)

        # store relationships for L,D,M ; cl,cd,cm
        self.Lift = None
        self.Drag = None
        self.Moment = None
        self.LiftCoeff = None
        self.DragCoeff = None
        self.MomentCoeff = None

        # offsets etc
        self.TailOffsetAngle = 5 # deg
        self.WingCmOffset = -0
    def tailDynamicsModel(self,
                          dynModel:"Aircraft2DPointMass",
                          w_induced: Union[float,np.ndarray] = 0,
                          )->Tuple[Union[float,np.ndarray],Union[float,np.ndarray],Union[float, np.ndarray]]:
        """
        Get tail aerodynamics results/constraints based on dynModel state values. This tail model includes hookups
        for one-way coupling of the wing induced velocity

        :param dynModel: Aircraft2DPointmass instance
        :param w_induced: [m/s] induced vertical velocity / angle of attack
        :return: tuple of cl,cd,cm of the tail
        """
        # unpack geometric variables if needed
        AR = dynModel.TailAspectRatio  # tail aspect ratio
        e = 0.95 # span efficiency of tail

        #unpack state variables and control variables that are needed
        alfa = dynModel.Alpha + self.TailOffsetAngle   # state - some offset angle in deg
        delta_e = dynModel.ElevatorPosition  # elevator defleciton in deg (+ down)
        cd0 = 0.05  # zero-lift drag (set low for now)

        # ===============================================================
        # call stored dynamics functions which can be any callable, polynomials are preferred for speed
        # cl
        cl = self.TailLiftCoeffFunction(alfa,delta_e)
        # cd
        cd = self.TailDragCoeffFunction(cl,AR,e,cd0)
        # cm
        cm = self.TailMomentCoeffFunction(alfa,delta_e)

        return cl,cd,cm # return as tuple

    def wingDynamicsModel(self, dynModel:"Aircraft2DPointMass"):
        """
        Calculate wing cl,cd,cm from stored functions
        :param dynModel:
        :return:
        """
        # unpack geometric variables if needed
        AR = dynModel.WingAspectRatio  # tail aspect ratio

        #unpack state variables and control variables that are needed
        alfa = dynModel.Alpha
        delta_f = dynModel.FlapPosition
        delta_cj = self.DeltaCJ(dynModel)
        # delta_cj = 0.1
        cd0 = 0.05 # zero-lift drag

        # cl
        cl = self.WingLiftCoeffFunction(alfa,delta_f,delta_cj)
        # cd
        cd = self.WingDragCoeffFunction(cl,delta_cj,AR) + cd0
        # cm
        cm = (self.WingMomentCoeffFunction(alfa,delta_f,delta_cj)
              + self.WingCmOffset)

        return cl,cd,cm

    def thrustModel(self,dynModel:"Aircraft2DPointMass"):
        """
        Calculate thrust using stored model
        :param dynModel:
        :return:
        """

        thrust = self.PropulsorModel(dynModel.Mass, dynModel.ThrottlePosition)
        return thrust

    def fullDynamicsModel(self,
                          dynamics_model:"Aircraft2DPointMass"
                          )-> \
                Tuple[Union[np.ndarray,float], Union[np.ndarray,float],Union[np.ndarray,float],Union[np.ndarray,float]]:
        """
        Compute the combined cl,cd,cm coefficients of the model combining all submodels. This does not break out the
        forces by component contribution (wing, tail) but is used to calculate body forces on the point mass model.

        :param dynamics_model:
        :return:
        """

        # calculate wing CL,CM,CD
        wingcl, wingcd, wingcm = self.wingDynamicsModel(dynamics_model)

        # calculate tail CL,CM,CD
        tailcl, tailcd, tailcm = self.tailDynamicsModel(dynamics_model,wingcl)

        # sum contributions, relate through area ratio
        Arat = dynamics_model.TailArea/dynamics_model.Area
        cl = wingcl
        cd = wingcd + tailcd * Arat
        cm = (wingcm
              - tailcl * 1.75 / dynamics_model.ChordMean * Arat
             )

        return cl,cd,cm

    def getForcesAndMoments(self,
                            dynamics_model: "Aircraft2DPointMass",
                            )->Tuple[Union[float,np.ndarray],Union[float,np.ndarray],Union[float,np.ndarray]]:
        """
        compute forces and moments acting on the aircraft. right now this only outputs the sum L,D,M acting on the
        aircraft. first runs fullDynamicsModel, then dimensionalizes using geometric information from dynamicsModel
        :param dynamics_model:
        :return:
        """

        # =========================================================================
        # get cl,cd,cm from dynamics model
        coefficients = self.fullDynamicsModel(dynamics_model=dynamics_model)

        cl = coefficients[0]
        cd = coefficients[1]
        cm = coefficients[2]

        # =========================================================================
        # extract useful params from dynamics point mass model
        q = dynamics_model.DynamicPressure
        S = dynamics_model.Area
        c = dynamics_model.ChordMean

        # convert coefficients to forces in wind frame
        L = q*cl*S
        D = q*cd*S
        M = q*cm*S*c

        # combine and get force in wind frame
        return L, D, M

    def DeltaCJ(self,dynModel:"Aircraft2DPointMass"):
        """
        calculate dcj based on throttle position from model
        """
        Sref = dynModel.Area
        Aprop = dynModel.PropulsorArea
        air_density = dynModel.AirDensity
        Vinf = dynModel.Airspeed


        T = dynModel.ThrottlePosition*dynModel.Mass*9.81
        # vrat = np.sqrt(T/(0.5 * air_density*Vinf**2*Aprop) + 1)
        vrat = 0.5 * (1 + np.sqrt(1+2*T/(0.5*air_density*Vinf**2*Aprop)))

        return Aprop/Sref*(vrat**2-1)*(1/vrat + 1)



if __name__=="__main__":

    model = BlownAirfoilModel()

    print('done')