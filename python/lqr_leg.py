import control
import numpy as np

A = np.matrix(
    [[0., 0., 1., 0.],
     [0., 0., 0., 1.],
     [0., 0., 0., 0.],
     [0., 0., 0., 0.]])
B = np.matrix(
    [[0., 0.,],
     [0., 0.,],
     [1., 0.,],
     [0., 1.,]])

C = np.matrix(
    [[1., 0., 0., 0.],
     [0., 1., 0., 0.]])

# z_com = 0.09428090415820636 # up to thigh
# z_com = 0.18970562748477146 # up to pelvis
z_com = 0.23856180831641274;
D = -z_com/9.81 * np.identity(2)

Q = C.T * C
R = D.T * D
N = C.T * D

K, S, E = control.lqr(A, B, Q, R, N)

print("LQR Gain K:")
print(K)
print("\nSolution to Riccati Equation S:")
print(S)
print("\nClosed-loop eigenvalues E:")
print(E)
