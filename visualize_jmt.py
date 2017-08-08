#Visualizing the Jerk Minimizing Trajectory
#Author: YWiyogo

import numpy as np
from matplotlib import pyplot as plt
import random

def JMT(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2 * a_2 * T
    c_2 = 2 * a_2

    A = np.array([
        [T**3,   T**4,    T**5],
        [3 * T**2, 4 * T**3,  5 * T**4],
        [6 * T,   12 * T**2, 20 * T**3],
    ])
    B = np.array([
        end[0] - c_0,
        end[1] - c_1,
        end[2] - c_2
    ])
    a_3_4_5 = np.linalg.solve(A, B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas


sstart=[124.833900, 0.005005, 0.083501]
sgoal=[147.333900, 15.000000, 10.000000]

T=1.5
Coeffs = JMT(sstart, sgoal, T)

Coeffs[0] = 0  #uncomment to start the position from 0

print("coeffs: ", Coeffs)
C=Coeffs[::-1]
print(C)
s = np.poly1d(C)
v = s.deriv()
a = v.deriv()
x = np.linspace(0, T+0.2, 50)
y = s(x)
y1 = v(x)
y2= a(x)
plt.title("JMT")
plt.plot(x, y, label='pos')
plt.plot(x, y1, label='vel')
plt.plot(x, y2, label='acc')
plt.legend(loc='upper left')
plt.grid()
plt.show()