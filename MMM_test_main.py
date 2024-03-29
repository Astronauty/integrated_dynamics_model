import math
import numpy as np
import matplotlib.pyplot as plt
import csv
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


suspension.state.accel = np.array([0,10,0])
suspension.state.bodyslip = -0*math.pi/180
suspension.state.steer_angle = -2*math.pi/180
suspension.update_tires()

##

lat_accels = np.linspace(0, 6, 20)
print(lat_accels)

bodyslip_sweep = np.linspace(-30*math.pi/180, 30*math.pi/180, 50)
steer_angle_sweep = np.linspace(-80*math.pi/180, 80*math.pi/180, 50)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for lat_accel in lat_accels:
    suspension.state.accel = np.array([0,lat_accel,0])
    suspension.update_tires()
    arr = suspension.find_body_slip_and_steer(bodyslip_sweep, steer_angle_sweep)
    print(arr.shape)
    ax.scatter3D(arr[:,0]*180/math.pi, arr[:,1]*180/math.pi, arr[:,2])

ax.set_xlabel('bodyslip (deg)')
ax.set_ylabel('steer angle (deg)')
ax.set_zlabel('lat force (N)')
ax.view_init(45,45)
plt.show()



# bodyslip_sweep = np.linspace(-30*math.pi/180, 30*math.pi/180, 100)
# steer_angle_sweep = np.linspace(-50*math.pi/180, 50*math.pi/180, 100)
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
# suspension.state.accel = np.array([0,0,0])
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
# ax.view_init(45,45)
# plt.show()



# Slip angle sweep of 4 tires

# suspension.state.accel = np.array([0,0,0])
# suspension.state.bodyslip = 0 * math.pi/180
# suspension.state.steer_angle = -20 * math.pi/180
# suspension.update_tires()
# for tire in suspension.state.tires.values():
#     print(tire.get_Fy())