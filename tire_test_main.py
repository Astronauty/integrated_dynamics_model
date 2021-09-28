import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

from tire_generic import Tire, TireEllipse, PacejkaFit

# Construct the front tire pacejka object
front_coeff_Fx = [1, 1, 1, 1, 1, 1 , 1, 1, 1, 1, 1, 1, 1]
front_coeff_Fy = [0.01131, -0.0314, 282.1, -650, -1490, 0.03926, -0.0003027, 0.9385, 5.777*10**-5, -0.06358, -0.1176, 0.02715, 4.998, 5.5557*10**-5, 0.05059, 0.005199, 0.001232, 0.004013]
front_coeff_Mz = [1, 1, 1, 1, 1, 1 , 1, 1, 1, 1, 1, 1, 1]
front_tire_pacejka = PacejkaFit(front_coeff_Fx, front_coeff_Fy, front_coeff_Mz)

front_tire = Tire(front_tire_pacejka)

# Plot the slip angle vs Fy relation for front tires
slip_angle_array = np.linspace(-20, 20, 200)
Fy_array_camber0 = [front_tire.get_Fy(i, 0, 1200) for i in slip_angle_array]

data = (0, 1200, 1000) # Camber, Fz, Fy_des
Fy_array_opt = [front_tire.get_Fy_Optimize(i, *data) for i in slip_angle_array]


fig = plt.figure()
plt.plot(slip_angle_array, Fy_array_camber0, label='0 deg camber')
plt.plot(slip_angle_array, Fy_array_opt)

slip_angle_needed = front_tire.get_SlipAngle(1000)
plt.plot(slip_angle_needed,front_tire.get_Fy(slip_angle_needed,0, 1200),'x', color='red')
plt.grid()
plt.show()

