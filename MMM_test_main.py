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
suspension.front_wheelrate_stiffness = (.574**2) * 400 / (.0254 * .224)
suspension.rear_wheelrate_stiffness = (.747**2) * 450 / (.0254 * .224)
suspension.front_roll_stiffness = 200 # N*m/rad
suspension.rear_roll_stiffness = 400 # N*m/rad

bodyslip_sweep = np.linspace(-30*math.pi/180, 30*math.pi/180, 20)
steer_angle_sweep = np.linspace(-10*math.pi/180, 10*math.pi/180, 20)
bodyslip_v, steer_angle_v = np.meshgrid(bodyslip_sweep, steer_angle_sweep)

print(bodyslip_v)

cols = bodyslip_sweep.size
rows = steer_angle_sweep.size

print(rows,cols)

force_v = np.zeros((rows-1, cols-1))

for i in range(rows - 1):
    for j in range(cols - 1):
        print(i,j)
        suspension.state.bodyslip = bodyslip_v[i, j]
        suspension.state.steer_angle = steer_angle_v[i, j]
        suspension.update_tires()

        force_v[i, j] = suspension.get_total_Fy()
        print(suspension.get_total_Fy())

plt.pcolor(bodyslip_v, steer_angle_v, force_v)
plt.colorbar()
plt.xlabel("Body Slip (rad)")
plt.ylabel("Steered Angle (rad)")
plt.show()