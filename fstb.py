import control
import numpy as np

# num = np.array([2])
# den = np.array([5, 1])
# H = control.tf(num, den)
# print('transfer funtion\n', H)

# A = np.array([[0, 3], [2, 4]])
# B = np.array([[-2], [1]])
# C = control.ctrb(A, B)
# print('controlability matrix\n', C)
# print('rank\n', np.linalg.matrix_rank(C))

# p_desire = np.array([-5+2j, -5-2j])
# print('desired pole placements\n', p_desire)
# K = control.place(A, B, p_desire)
# print('controller u\n', K)

# A_cl = A - B*K
# eig = np.linalg.eig(A_cl)
# print('eigenvalues\n', eig)

# u = -Kx(t)