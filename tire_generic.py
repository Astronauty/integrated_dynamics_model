import math
import numpy as np
from scipy.optimize import fsolve

"""
Coordinate Systems:
    SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html

Units: 
    Length: meters
    Mass: kg
"""
class Tire:
    def __init__(self, pacejka_fit=None):
        self.state = {}

        # Params
        # Pacejka Coefficients, specific to each tire
        self.param.pacejka_fit = pacejka_fit
        self.param.stiffness = 0
        self.param.steerable = None  # Is this a steerable tire, implement abstract class eventually
        self.param.position = np.array([None, None])  # Position of the center of the contact patch relative to vehicle frame
        self.param.radius_unloaded = None  # Unloaded tire radius




        # States (keep in mind these aren't true states, includes some derived)
        self.state["velocity"] = None  # Velocity vector of the tire relative to vehicle
        self.state["camber"] = 0
        self.state["slip_angle"] = 0
        self.state["slip_ratio"] = 0
        self.state["force"] = np.array([0, 0, 0])
        self.state["radius_effective"] = None  # Effective radius considering slip
        self.state["angular_velocity"] = None

        # Vehicle inputs
        self.torque = None

    def get_radius_loaded(self):
        return self.radius_unloaded - self.state["force"][2]/self.stiffness


    # Determines the lateral force on the tire given the pacejka fit coefficients, slip angle, camber, and normal load
    # https://www.edy.es/dev/docs/pacejka-94-parameters-explained-a-comprehensive-guide/
    def get_Fy(self, slip_angle, camber, Fz):
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16,
         a17] = self.pacejka_fit.Fy_coefficients
        C = a0
        D = Fz * (a1 * Fz + a2) * (1 - a15 * camber ** 2)
        BCD = a3 * math.sin(math.atan(Fz / a4) * 2) * (1 - a5 * abs(camber))
        B = BCD / (C * D)
        H = a8 * Fz + a9 + a10 * camber
        E = (a6 * Fz + a7) * (1 - (a16 * camber + a17) * math.copysign(1, slip_angle + H))
        V = a11 * Fz + a12 + (a13 * Fz + a14) * camber * Fz
        Bx1 = B * (slip_angle + H)

        Fy = D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V

        return D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V

    def get_Fy_Optimize(self, slip_angle, *args):
        camber, Fz, Fy_des = args
        [a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16,
         a17] = self.pacejka_fit.Fy_coefficients
        C = a0
        D = Fz * (a1 * Fz + a2) * (1 - a15 * camber ** 2)
        BCD = a3 * math.sin(math.atan(Fz / a4) * 2) * (1 - a5 * abs(camber))
        B = BCD / (C * D)
        H = a8 * Fz + a9 + a10 * camber
        E = (a6 * Fz + a7) * (1 - (a16 * camber + a17) * math.copysign(1, slip_angle + H))
        V = a11 * Fz + a12 + (a13 * Fz + a14) * camber * Fz
        Bx1 = B * (slip_angle + H)

        Fy = D * math.sin(C * math.atan(Bx1 - E * (Bx1 - math.atan(Bx1)))) + V - Fy_des

        return Fy

    def get_Fx(self, slip_angle, camber, Fz):
        # [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16, b17] = self.pacejka_fit.Fx_coefficients
        # C = b0;
        # D = Fz*(b1*Fz+b2)
        # BCD = (b3**Fz+b4*Fz)*math.exp(-b5*Fz)
        # B = BCD/(C*D)
        # H = b9*Fz+b10
        # E = (b6**Fz+b7*Fz+b8)*(1-b13*math.sign(slip_ratio+H))
        # V = b11*Fz+b12
        # Bx1 = B*(slip_ratio+H)

        # return D*math.sin(C*math.atan(Bx1-E*(Bx1-math.atan(Bx1)))) + V
        return 0

    def get_Mz(self, slip_angle, camber, Fz):
        return 0

    # Returns the slip angle needed to produce an Fy input
    def get_SlipAngle(self, Fy):
        data = (0, 800, Fy)  # Camber and Fz
        return fsolve(self.get_Fy_Optimize, 10, args=data)

    def hasTraction(self):
        # Model the friction ellipse as an ellipsoid
        a = 1
        b = 1
        c = 1
        return 0


# Stores predetermined pacejka fits for several tires
class PacejkaFit:
    def __init__(self, Fx_coefficients=None,
                 Fy_coefficients=None,
                 Mz_coefficients=None):
        self.Fx_coefficients = Fx_coefficients
        self.Fy_coefficients = Fy_coefficients
        self.Mz_coefficients = Mz_coefficients


# Stores info on the tractive state of the tire. Approximated using an ellipsoid centered on the origin of Fx vs Fy
# vs Fz. If the tire state is outside of the ellipsoid, it will be characterized as traction loss:
# https://math.stackexchange.com/questions/76457/check-if-a-point-is-within-an-ellipse
class TireEllipse:
    def __init__(self, a=None, b=None, c=None):
        self.a = a
        self.b = b
        self.c = c

    def hasTraction(self, Fx, Fy, Fz):
        if (Fx ** 2 / self.a ** 2) + (Fy ** 2 / self.b ** 2) + (Fz ** 2 / self.c ** 2) <= 1:
            return False
        else:
            return True
