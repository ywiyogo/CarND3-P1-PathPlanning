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


sstart=[125.03, 20, 0]#[124.833900, 0.005005, 0.083501]
sgoal=[145.03, 20, 0]#[147.333900, 15.000000, 10.000000]

Ts=[2, 3, 100]
fig = plt.figure(figsize=(18, 8))

axes = [fig.add_subplot(131), fig.add_subplot(132), fig.add_subplot(133)]

for i,T in enumerate(Ts):
    Coeffs = JMT(sstart, sgoal, T)

    Coeffs[0] = 0  #uncomment to start the position from 0

    print("coeffs: ", Coeffs)
    C=Coeffs[::-1]
    print(C)
    s = np.poly1d(C)
    v = s.deriv()
    a = v.deriv()
    x = np.linspace(0, max(Ts)+0.1, 50)
    y = s(x)
    y1 = v(x)
    y2= a(x)
    axes[i].plot(x, y, label='pos')
    axes[i].plot(x, y1, label='vel')
    axes[i].plot(x, y2, label='acc')
    axes[i].legend(loc='upper right')
    axes[i].set_ylim([-25,100])
    axes[i].set_xlim([0,max(Ts)])
    title = "JMT T= "+ str(T)
    axes[i].set_title(title)
    axes[i].set_xlabel('T')
    axes[i].set_ylabel('val')
    axes[i].grid(linestyle=":")
fig.tight_layout()

plt.show()