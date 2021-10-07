import math
import types
import numpy as np
from scipy.optimize import fsolve
from tire_generic import Tire, TireEllipse, PacejkaFit

"""
Coordinate Systems:
    SAE z-down wheel/tire centered coordinates: https://www.mathworks.com/help/vdynblks/ug/coordinate-systems-in-vehicle-dynamics-blockset.html

Units: 
    Length: meters
    Mass: kg
"""


class Suspension():
    def __init__(self):

        self.param = types.SimpleNamespace()
        self.state = types.SimpleNamespace()
        self.env = types.SimpleNamespace()

        # Environment variables (temporary for outside of lapsim functionality)
        self.env.g = 9.81


        # Params (eventually move over to parameters file)
        self.param.front_roll_stiffness = 0
        self.param.rear_roll_stiffness = 0
        self.param.pitch_stiffness = 0

        self.param.front_wheelrate_stiffness = 0
        self.param.rear_wheelrate_stiffness = 0

        self.param.front_toe = 0
        self.param.rear_toe = 0

        self.param.front_static_camber = 0
        self.param.rear_static_camber = 0

        self.param.mass_total = 100  # kg
        self.param.ride_height = 0.0762  # m

        self.param.cg_bias = 0.6  # Position of the cg from front to rear, value from 0-1
        self.param.front_track = 1.27
        self.param.rear_track = 1.17
        self.param.wheelbase = 1.55

        front_coeff_Fx = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        front_coeff_Fy = [0.01131, -0.0314, 282.1, -650, -1490, 0.03926, -0.0003027, 0.9385, 5.777 * 10 ** -5, -0.06358,
                          -0.1176, 0.02715, 4.998, 5.5557 * 10 ** -5, 0.05059, 0.005199, 0.001232, 0.004013]
        front_coeff_Mz = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        front_tire_pacejka = PacejkaFit(front_coeff_Fx, front_coeff_Fy, front_coeff_Mz)

        # States
        self.state.tires = {"front_left": Tire(front_tire_pacejka), "front_right": Tire(front_tire_pacejka)
            , "rear_left": Tire(front_tire_pacejka), "rear_right": Tire(front_tire_pacejka)}

        self.state.bodyslip = 0
        # self.state.yaw = 0
        self.state.pitch = 0
        self.state.roll = 0

        self.state.steer_angle = 0

        self.state.position = np.array([0, 0])  # Planar position in the global frame, maybe should be RacecarState
        self.state.velocity = np.array([0, 0])

    # def steady_state_load_transfer(self, Ax, Ay, Az):
    #     longitudinal_reaction_moment = Ax * self.mass_total
    #     lateral_reaction_moment = Ay * self.mass_total
    #
    #     total_roll_stiffness = self.front_roll_stiffness + self.rear_roll_stiffness
    #
    #     self.state.pitch = longitudinal_reaction_moment/self.pitch_stiffness
    #     self.state.roll = lateral_reaction_moment/total_roll_stiffness
    #
    #
    #     self.state.front_left_tire.state.force[2] = (self.mass_total * (1 - self.cg_bias) / 2) \
    #                                                 - (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                 - (lateral_reaction_moment / self.front_track) / ( 2 / self.front_roll_stiffness/total_roll_stiffness)
    #     self.state.front_right_tire.state.force[2] = (self.mass_total * (1 - self.cg_bias) / 2) \
    #                                                 - (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                 + (lateral_reaction_moment / self.front_track) / ( 2 / self.front_roll_stiffness/total_roll_stiffness)
    #     self.state.rear_left_tire.state.force[2] = (self.mass_total * self.cg_bias / 2) \
    #                                                + (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                - (lateral_reaction_moment / self.front_track) / ( 2 / self.rear_roll_stiffness/total_roll_stiffness)
    #     self.state.rear_left_tire.state.force[2] = (self.mass_total * self.cg_bias / 2) \
    #                                                + (longitudinal_reaction_moment / self.wheelbase) / 4 \
    #                                                + (lateral_reaction_moment / self.front_track) / ( 2 / self.rear_roll_stiffness/total_roll_stiffness)
    def get_total_Fy(self):
        total_Fy = 0
        for tire in self.state.tires.values():
            total_Fy += tire.get_Fy()
        return total_Fy

    def update_tires(self):
        # Add the body slip to the slip angle on each tire
        for tire in self.state.tires.values():
            tire.state.slip_angle += self.state.bodyslip

        # Update toe, steering to each tire (following standard toe sign, steered angle direction same as body slip)
        self.state.tires["front_left"].state.slip_angle += (-self.param.front_toe + self.state.steer_angle)
        self.state.tires["front_right"].state.slip_angle += (self.param.front_toe + self.state.steer_angle)
        self.state.tires["rear_left"].state.slip_angle += (-self.param.rear_toe)
        self.state.tires["rear_right"].state.slip_angle += self.param.rear_toe
        
        # Update camber
        # TODO: add camber gain
        self.state.tires["front_left"].camber = self.param.front_static_camber
        self.state.tires["front_right"].camber = self.param.front_static_camber
        self.state.tires["rear_right"].camber = self.param.rear_static_camber
        self.state.tires["rear_right"].camber = self.param.rear_static_camber

        # Update normal force
        # TODO: apply proper static weight transfer
        static_tire_weights = self.get_static_weight_tire_forces()
        self.state.tires["front_left"].state.force[2] = static_tire_weights[0]
        self.state.tires["front_right"].state.force[2] = static_tire_weights[1]
        self.state.tires["rear_left"].state.force[2] = static_tire_weights[2]
        self.state.tires["rear_right"].state.force[2] = static_tire_weights[3]

    def get_static_weight_tire_forces(self):
        weight_front_static = -self.env.g * self.param.mass_total * (1-self.param.cg_bias)
        weight_rear_static = -self.env.g * self.param.mass_total * self.param.cg_bias

        # Returning forces on tires in N
        return [weight_front_static / 2, weight_front_static / 2, weight_rear_static / 2, weight_rear_static / 2]

    def dynamicWeightTransfer(self, state):
        # dW_lats are always 0 unless there is a nonzero lateral acceleration
        # dW_long is positive when accelerating
        a_lon = state.a_long / self.env.g  # converts to g's
        a_lat = state.a_lat / self.env.g

        self.log('a_long_g', a_lon)
        self.log('a_lat_g', a_lat)

        rollAxis = self.ic_geometry()

        # Note: pitch center terms just cancel out. Add moment of inertia term to sprung weight equation?
        dW_spr_x = a_lon * self.WS * \
                   (self.params.car.CG_sZ - rollAxis[3]) / self.params.car.WB
        dW_geo_x = a_lon * self.WS * (rollAxis[3] / self.params.car.WB)

        # pitch calculation
        if state.a_long > 0:
            p_direction = 0
            '''
            dW_uns_fx = 0
            dW_uns_rx = a_lon * (self.WUS_r / 2) * \
                (self.params.car.CG_urZ / self.params.car.WB)
            dW_lon = dW_spr_x + dW_geo_x + dW_uns_rx
            '''

        else:
            p_direction = 1
            '''
            dW_uns_rx = 0
            dW_uns_fx = a_lon * (self.WUS_f / 2) * \
                (self.params.car.CG_ufZ / self.params.car.WB)
            dW_lon = dW_spr_x + dW_geo_x + dW_uns_fx
            '''

        dW_uns_x = a_lon * (self.WUS_f * self.params.car.CG_urZ + self.WUS_r * self.params.car.CG_ufZ)
        dW_lon = dW_spr_x + dW_geo_x + dW_uns_x
        pitch = self.params.springs.p_GR[p_direction] * a_lon

        '''
        # yaw estimate, likely the best that can be done with linear point-mass model
        if abs(state.a_lat) < 0.000001 or state.v == 0:
            yaw = 0
        else:
            yaw = math.atan(self.params.car.WB / (2 * state.v **
                                                  2 / state.a_lat)) * 180 / math.pi
        '''

        # Body Slip (Yaw?) Equation from Millikan

        CS_f = 0  # Replace with front cornering stiffness
        CS_r = 0  # Replace with rear cornering stiffness
        Y_beta = CS_f + CS_r
        if state.v > 0:
            Y_r = (self.params.car.CG_sX * CS_f - (self.params.car.WB - self.params.car.CG_sX) * CS_r) / state.v
        else:
            Y_r = 0
        Y_delta = -CS_f
        N_beta = self.params.car.CG_sX * CS_f - (self.params.car.WB - self.params.car.CG_sX) * CS_r
        N_r = (self.params.car.CG_sX ** 2 * CS_f - (self.params.car.WB - self.params.car.CG_sX) ** 2 * CS_r)
        N_delta = -self.params.car.CG_sX * CS_f
        Q = N_beta * Y_r - N_beta * self.car_mass * state.v - Y_beta * N_r

        # yaw = (Y_delta * N_r - N_delta * (Y_r - self.car_mass * state.v)) * state.steer_angle / Q

        yaw = 0

        # Roll calculation - from M&M 18.4 (pg 682), requires roll stiffnesses to be in terms of radians for small angle
        # to hold
        K_roll = self.WS * rollAxis[2] / ((180 / math.pi) * (self.K_R_f + self.K_R_r) - (
                self.WS * rollAxis[2]))
        roll = a_lat * K_roll * 180 / math.pi

        K_R_f_prime = self.K_R_f * (180 / math.pi) - (self.params.car.WB - self.params.car.CG_sX) * self.WS * rollAxis[
            2] \
                      / self.params.car.WB
        dw_spr_f = (a_lat * self.WS / self.params.car.TK_f) * \
                   (K_roll * K_R_f_prime / self.WS)
        dw_geo_f = (a_lat * self.WS / self.params.car.TK_f) * \
                   ((self.params.car.WB - self.params.car.CG_sX) / self.params.car.WB) * rollAxis[0]
        dw_uns_f = a_lat * self.WUS_f * (self.params.car.CG_ufZ / self.params.car.TK_f)

        K_R_r_prime = self.K_R_r * (180 / math.pi) - self.params.car.CG_sX * self.WS * rollAxis[2] / self.params.car.WB
        dw_spr_r = (a_lat * self.WS / self.params.car.TK_r) * \
                   (K_roll * K_R_r_prime / self.WS)
        dw_geo_r = (a_lat * self.WS / self.params.car.TK_r) * \
                   self.params.car.CG_sX / self.params.car.WB * rollAxis[1]
        dw_uns_r = a_lat * self.WUS_r * (self.params.car.CG_urZ / self.params.car.TK_r)

        dW_lat_f = dw_spr_f + dw_geo_f + dw_uns_f
        dW_lat_r = dw_spr_r + dw_geo_r + dw_uns_r

        self.log('roll', roll)
        self.log('pitch', pitch)

        self.log('dW_lon', dW_lon)
        self.log('dW_lat_f', dW_lat_f)
        self.log('dW_lat_r', dW_lat_r)

        self.log('dW_spr_f', dw_spr_f)
        self.log('dW_geo_f', dw_geo_f)
        self.log('dW_uns_f', dw_uns_f)
        self.log('dW_spr_r', dw_spr_r)
        self.log('dW_geo_r', dw_geo_r)
        self.log('dW_uns_r', dw_uns_r)

        self.log('dW_spr_x', dW_spr_x)
        self.log('dW_geo_x', dW_geo_x)
        '''
        self.log('dW_uns_rx', dW_uns_rx)
        self.log('dW_uns_fx', dW_uns_fx)
        '''
        self.log('dW_uns_x', dW_uns_x)

        return [dW_lon, dW_lat_f, dW_lat_r], [yaw, pitch, roll]

    def tire_normals(self, dW_lon, dW_lat_f, dW_lat_r, downforce):

        FIN = self.static_WD[0] - dW_lon / 2 - dW_lat_f + downforce[0]
        FON = self.static_WD[1] - dW_lon / 2 + dW_lat_f + downforce[1]
        RIN = self.static_WD[2] + dW_lon / 2 - dW_lat_r + downforce[2]
        RON = self.static_WD[3] + dW_lon / 2 + dW_lat_r + downforce[3]

        W_tot = FIN + FON + RIN + RON
        self.log('weight', W_tot)

        return [FIN, FON, RIN, RON]

    def stiffness_arb(self):
        if self.params.arb.enabled_f:
            K_TB_f = self.params.arb.G_TB_f * math.pi / 32 * \
                     (self.params.arb.TB_f_OD ** 4 -
                      self.params.arb.TB_f_ID ** 4) / self.params.arb.TB_f_L

            self.K_ARB_f = K_TB_f * self.params.arb.IR_f ** 2 * (self.params.car.TK_f / self.params.arb.LA_f) ** 2 \
                           * math.pi / 180  # Nm/deg

        if self.params.arb.enabled_r:
            K_TB_r = self.params.arb.G_TB_r * math.pi / 32 * \
                     (self.params.arb.TB_r_OD ** 4 -
                      self.params.arb.TB_r_ID ** 4) / self.params.arb.TB_r_L

            self.K_ARB_r = K_TB_r * self.params.arb.IR_r ** 2 * \
                           (self.params.car.TK_r / self.params.arb.LA_r) ** 2 * math.pi / 180

    def stiffness_suspension(self):
        k_W_f = self.params.wheel.IR_f ** 2 * self.params.wheel.k_SP_f
        k_RIDE_f = k_W_f * self.params.front_tire.k_T / \
                   (k_W_f + self.params.front_tire.k_T)
        self.K_S_f = k_RIDE_f * \
                     (self.params.car.TK_f / 2) ** 2 * 2 * math.pi / 180  # N.m/deg

        k_W_r = self.params.wheel.IR_r ** 2 * self.params.wheel.k_SP_r
        k_RIDE_r = k_W_r * self.params.rear_tire.k_T / \
                   (k_W_r + self.params.rear_tire.k_T)
        self.K_S_r = k_RIDE_r * \
                     (self.params.car.TK_r / 2) ** 2 * 2 * math.pi / 180

    def find_body_slip_and_steer(self):
        self.state.front_left_tire.get_cornering_stiffness()
        self.state.front_right_tire.get_cornering_stiffness()
        self.state.rear_left_tire.get_cornering_stiffness()
        self.state.rear_right_tire.get_cornering_stiffness()
        return 0
