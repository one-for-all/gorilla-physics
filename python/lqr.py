import control
import numpy as np

# Compute LQR controller for an acrobot (simple double pendulum w/ motor only at
# second joint)

# Define the model parameters
# Reference: https://underactuated.csail.mit.edu/acrobot.html#section1
m1 = 5
m2 = 5
l1 = 7
l2 = 7
lc1 = 7
lc2 = 7
I1 = m1*l1*l1
I2 = m2*l2*l2
g = 9.81

# Define the state for upward position, i.e. the stabilizable operating point
theta1 = np.pi
theta2 = 0
c2 = np.cos(theta2)

# mass matrix
M = np.array([[I1 + I2 + m2*l1*l1 + 2*m2*l1*lc2*c2, I2 + m2*l1*lc2*c2],
              [I2 + m2*l1*lc2*c2,                   I2]])

# control mapping matrix
B = np.array([[0],
              [1]])

# Perform LQR
# 1. Compute the A and B matrices of linearized dynamics
# Reference: https://underactuated.csail.mit.edu/acrobot.html#section4

# derivative of gravity torque wrt configuration
dtau_dq = np.array([[g*(m1*lc1 + m2*l1 + m2*lc2),   m2*g*lc2],
                    [m2*g*lc2,                      m2*g*lc2]])

A_lin = np.zeros((4, 4))
A_lin[:2, 2:] = np.eye(2)
A_lin[2:, :2] = np.linalg.inv(M) @ dtau_dq

B_lin = np.zeros((4, 1))
B_lin[2:, :] = np.linalg.inv(M) @ B

# 2. Define cost matrices Q and R
Q = np.eye(4)              # Penalize state deviations equally
R = np.array([[10.0]])      # Penalize control effort

# 3. Compute the LQR controller gain K
K, S, E = control.lqr(A_lin, B_lin, Q, R)

print("LQR Gain K:")
print(K)
print("\nSolution to Riccati Equation S:")
print(S)
print("\nClosed-loop eigenvalues E:")
print(E)