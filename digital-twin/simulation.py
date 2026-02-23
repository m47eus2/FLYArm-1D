import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

#
# Simulation
#

mb = 0.0335
l = 0.128
ms = 0.02335
xs = 0.120
# b = 0.00011
g = 9.81

I = (1/3)*mb*l**2 + ms*xs**2
I = I/8.8 # Correction based on experimental identificaiton

u=0

Fs = 0.001617342*u
Fc = 0.0006
b = 0.00005

def deg(t,x):
    dx1 = x[1]
    dx2 = -((mb*g*(l/2) + ms*g*xs)/I)*np.sin(x[0]) - (b/I)*x[1] - (Fc/I)*np.tanh(x[1]/0.01) + (Fs*xs)/I
    return [dx1,dx2]

res = solve_ivp(deg, [0,3], [np.pi/4,0], rtol=1e-10, atol=1e-10)

#
# Reference from real device
#

angles = np.load("angle_log.npy")
angles = ((angles[688:]+4) / 360) * 2*np.pi
step = 0.01
t = np.arange(0, len(angles)*step, step)

#
# Linearization in x=(pi/2, 0) without dry friction
#

A = np.array([[0,1],[0,-b/I]])
B = np.array([[0],[xs/I]])

plt.title("Comparison of Simulated and Measured Responses")
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")
plt.xlim(0,2.5)
plt.plot(res.t, res.y[0], label="Simulation")
#plt.plot(res.t, res.y[1], label="x2")
plt.plot(t, angles, "--", label="Measurement")
plt.legend()
plt.grid()
plt.show()