import numpy as np
import control



m = 1
b = 0.2
k = 0

A = np.array([[0, 1], [-k/m, -b/m]])
B = np.array([[0], [1/m]])
C = np.array([1, 0])
D = np.array([0])
print(f'A:\n{A}')
print(f'B:\n{B}')
sys = control.ss(A, B, C, D)
print(f'sys:\n{sys}')

Q = np.array([[1, 0], [0, 1]])
R = np.array([0.01])
print(f'Q:\n{Q}')
print(f'R:\n{R}')
K, S, E = control.lqr(A, B, Q, R)
print(f'K:\n{K}')  # state feedback gains
print(f'S:\n{S}')  # solution to Riccati equation
print(f'E:\n{E}')  # eignevalues of the closed loop system

