import numpy as np

import control

# Compute LQR controller for an cart-pole

# Define the model parameters
# Reference: https://underactuated.csail.mit.edu/acrobot.html#cart_pole
mc = 3
mp = 5
l = 7
g = 9.81

# Define the state for pendulum upward position, i.e. the stabilizable operating point
theta = np.pi
c = np.cos(theta)

# mass matrix
M = np.array([[mc + mp,     mp * l * c],
              [mp * l * c,  mp * l * l]])

# control mapping matrix
B = np.array([[1],
              [0]])

# Perform LQR
# 1. Compute the A and B matrices of linearized dynamics
# Reference: https://underactuated.csail.mit.edu/acrobot.html#section4

# derivative of gravity torque wrt configuration
dtau_dq = np.array([[0,   0],
                    [0,   mp * g * l]])

A_lin = np.zeros((4, 4))
A_lin[:2, 2:] = np.eye(2)
A_lin[2:, :2] = np.linalg.inv(M) @ dtau_dq

B_lin = np.zeros((4, 1))
B_lin[2:, :] = np.linalg.inv(M) @ B

# 2. Define cost matrices Q and R
Q = np.eye(4)               # Penalize state deviations equally
R = np.array([[1.0]])      # Penalize control effort

# 3. Compute the LQR controller gain K
K, S, E = control.lqr(A_lin, B_lin, Q, R)

print("LQR Gain K:")
print(K)
# print("\nSolution to Riccati Equation S:")
# print(S)
# print("\nClosed-loop eigenvalues E:")
# print(E)