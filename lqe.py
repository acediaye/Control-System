import control
import numpy as np
import matplotlib.pyplot as plt

class LQE(object):
    def __init__(self, Q: np.ndarray, R: np.ndarray):
        self.Q = Q  # Vd?
        self.R = R  # Vn?
        self.ss_plant = None  # state space plant
        self.ss_obsv = None
        
        # save response values
        self.time_out = None
        self.y_ol_out = None
        self.x_ol_out = None
        self.x_hat = None
        self.reference = None
    
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        self.reference = reference
        self.ss_plant = plant
        A, B, C, D = control.ssdata(plant)
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_o != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        
        # open loop response
        self.time_out, self.y_ol_out, self.x_ol_out = control.forced_response(self.ss_plant, time, reference, return_x=True)
        # state observer response
        # L = control.place(A.T, C.T, self.poles_desire).T
        Vd = 1*np.eye(2)  #disturbance 0.1
        Vn = 1  # noise
        L, S, E = control.lqr(A.T, C.T, Vd, Vn)  # duality
        L = L.T
        S = S.T
        A_ob = A-L@C
        B_ob = np.bmat([B, L])
        C_ob = np.eye(2)
        D_ob = np.array([[0, 0],
                         [0, 0]])
        u = np.array([reference])
        y = np.array([self.y_ol_out])
        u_ob = np.bmat([[u],
                       [y]])
        # x0 = np.array([[0.5], [-0.5]])
        self.ss_obsv = control.ss(A_ob, B_ob, C_ob, D_ob)
        self.time_out, self.x_hat = control.forced_response(self.ss_obsv, time, u_ob)
        return self.time_out, self.x_hat
    
    def graph(self, save):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        # plt.plot(self.time_out, self.reference, label='ref')
        plt.plot(self.time_out, self.y_ol_out, label='y (ol)')
        for i in range(len(self.x_ol_out)):
            plt.plot(self.time_out, self.x_ol_out[i, :], '-', label=f'x_ol{i+1}')
        for i in range(len(self.x_hat)):
            plt.plot(self.time_out, self.x_hat[i, :], '--', label=f'x_hat{i+1}')
        plt.legend(loc='right')
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        plt.title('LQE response')
        if save == True:
            plt.savefig('plots3/lqe_response.png')
        plt.show()
    
    def pzmap(self):
        if self.ss_obsv is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.ss_obsv, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
        