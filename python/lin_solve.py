import numpy as np

A = np.array([[1.0001797859296913,      0.5000449444624181],
              [0.5000449444624181,      0.25]]);
b = np.array([0.7500930408242205, 0.3750128109137374]);

x = np.linalg.solve(A, b)

print("A determinant:")
print(np.linalg.det(A))

print("Matrix A:")
print(A)
print("\nVector b:")
print(b)
print("\nSolution x:")
print(x)
