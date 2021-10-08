import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.optimize import fsolve

from tire_generic import Tire, TireEllipse, PacejkaFit
from suspension_new import Suspension
"""
    TODO FOR MMM:
    Method 1:
    - Assume no longitudinal load transfer for now
    - Impose lateral force
        - Find all possible combinations of body slip and steered angle that result in that lateral force
            - Define search space
            - Based on initial toe, calculate slip angle at 4 tires for certain discretization of search space
            - Return combos which produce correct lateral force within certain margin
    - Calculate net moment
    - Return data set of lateral force, moment, bodyslip, and slip angle
    - Plot
    
    
"""

suspension = Suspension()


# suspension.state.accel = np.array([0,10,0])
# suspension.state.bodyslip = -0*math.pi/180
# suspension.state.steer_angle = -2*math.pi/180
# suspension.update_tires()

###

# suspension.state.accel = np.array([0,10,0])

suspension.state.accel = np.array([0, 10, 0])
bodyslip_sweep = np.linspace(-30*math.pi/180, 30*math.pi/180, 100)
steer_angle_sweep = np.linspace(-45*math.pi/180, 10*math.pi/180, 100)
arr = suspension.find_body_slip_and_steer(bodyslip_sweep, steer_angle_sweep)

print(arr.shape)
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter3D(arr[:,0]*180/math.pi, arr[:,1]*180/math.pi, arr[:,2])
ax.set_xlabel('bodyslip (deg)')
ax.set_ylabel('steer angle (deg)')
ax.set_zlabel('lat force (N)')

plt.show()


# bodyslip_sweep = np.linspace(-30*math.pi/180, 30*math.pi/180, 100)
# steer_angle_sweep = np.linspace(-10*math.pi/180, 10*math.pi/180, 100)
# bodyslip_v, steer_angle_v = np.meshgrid(bodyslip_sweep, steer_angle_sweep)
#
# print(bodyslip_v)
#
# cols = bodyslip_sweep.size
# rows = steer_angle_sweep.size
#
# print(rows,cols)
#
# force_v = np.zeros((rows, cols))
# suspension.state.accel = np.array([0,5,0])
# for i in range(rows):
#     for j in range(cols):
#         print(i,j)
#         suspension.state.bodyslip = bodyslip_v[i, j]
#         suspension.state.steer_angle = steer_angle_v[i, j]
#         suspension.update_tires()
#
#         force_v[i, j] = suspension.get_total_Fy()
#         print(suspension.get_total_Fy())
#
# # plt.pcolor(bodyslip_v, steer_angle_v, force_v)
# # plt.colorbar()
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# #ax.plot_wireframe(bodyslip_v, steer_angle_v, force_v, color='black')
#
# ax.plot_surface(bodyslip_v, steer_angle_v, force_v, rstride=1, cstride=1,
#                 cmap='viridis', edgecolor='none')
# ax.set_title('Lateral Force Surface');
# ax.set_xlabel("Body Slip (rad)")
# ax.set_ylabel("Steered Angle (rad)")
# ax.set_zlabel("Lateral Force (N)")
# ax.view_init(40, 35)
# plt.show()
