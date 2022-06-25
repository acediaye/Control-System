import control
import numpy as np
import model
import matplotlib.pyplot as plt

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

# ctrb obsv
# u = -Kx(t) 
# k is 2x1

class FSTB(object):
    def __init__(self, eigenvalues: np.ndarray):
        self.poles_desire = eigenvalues
        self.K = 0
        self.K_r = 0
        
        # save response values
        self.time_out = None
        self.y_out = None
        self.x_out = None
        
    def excite(self, plant: control.TransferFunction, time: np.ndarray, reference: np.ndarray) -> tuple:
        ss = control.tf2ss(plant)
        A, B, C, D = control.ssdata(ss)
        # print(f'ss: {A}, {B}, {C}, {D}')
        
        ctrb = control.ctrb(A, B)
        rank_c = np.linalg.matrix_rank(ctrb)
        print(ctrb, rank_c, np.shape(A))
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        print(obsv, rank_o)
        if rank_c != np.shape(A)[0] or rank_c != np.shape(A)[1]:
            raise RuntimeError('not full rank')
        print('full rank')
        
        plt.figure()
        p, z = control.pzmap(plant)
        print(f'poles: {p}, zeros: {z}')
        
        print(f'desired poles: {self.poles_desire}')
        K = control.place(A, B, self.poles_desire)
        print(f'K: {K}')
        A_cl = A - B*K
        w, v = np.linalg.eig(A)
        print(f'eig A_ol: {w}')
        w, v = np.linalg.eig(A_cl)
        print(f'eig A_cl: {w}')
        
        ss2 = control.ss(A_cl, B, C, D)
        plt.figure()
        p, z = control.pzmap(ss2)
        t, yout = control.forced_response(ss2, time, reference)
        plt.figure()
        plt.plot(t, yout)


        dc = control.dcgain(ss2)
        Kr = 1/dc
        sys3 = control.ss(A_cl, B*Kr, C, D)
        t, yout = control.forced_response(sys3, time, reference)
        plt.figure()
        plt.plot(t, yout)
        plt.show()

    def graph(self, save: bool):
        pass
    
    def pzmap(self):
        pass

# mymodel = model.Mass_Spring_Damper_System(1, 20, 10)
# ss = mymodel.plant()
# A, B, C, D = control.ssdata(ss)
# print(f'ss: {A}, {B}, {C}, {D}')

# ctrb = control.ctrb(A, B)
# print(f'ctrb: {ctrb}')
# print(f'rank: {np.linalg.matrix_rank(ctrb)}')
# obsv = control.obsv(A, C)
# print(f'ovsv: {obsv}')
# print(f'rank: {np.linalg.matrix_rank(obsv)}')

# plt.figure()
# p, z = control.pzmap(ss)
# print(f'poles: {p}, zeros: {z}')
# p_desire = np.array([-5 + 2j, -5-2j])
# print(f'desired poles: {p_desire}')
# K = control.place(A, B, p_desire)
# print(f'K: {K}')
# A_cl = A - B*K
# w, v = np.linalg.eig(A)
# print(f'eig A_ol: {w}')
# w, v = np.linalg.eig(A_cl)
# print(f'eig A_cl: {w}')

# TIME_STEP = 0.002
# TIME = np.arange(0+TIME_STEP, 3+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
# print(f'time: {len(TIME)}')
# REFERENCE = 1*np.ones(len(TIME))

# ss2 = control.ss(A_cl, B, C, D)
# plt.figure()
# control.pzmap(ss2, plot=True)
# t, yout = control.forced_response(ss2, TIME, REFERENCE)
# plt.figure()
# plt.plot(t, yout)


# dc = control.dcgain(ss2)
# Kr = 1/dc
# sys3 = control.ss(A_cl, B*Kr, C, D)
# t, yout = control.forced_response(sys3, TIME, REFERENCE)
# plt.figure()
# plt.plot(t, yout)
# # plt.show()

# myfstb = FSTB(ss, np.array([-5+2j, -5-2j]))
# myfstb.controller()