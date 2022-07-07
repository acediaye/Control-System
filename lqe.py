import control
import numpy as np
import matplotlib.pyplot as plt

class LQE(object):
    def __init__(self, Vd: np.ndarray, Vn: np.ndarray):
        self.Vd = Vd  # Wd = Vd*d?
        self.Vn = Vn  # Wn = Vn*n?
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
        # find L gain
        # L = control.place(A.T, C.T, self.poles_desire).T
        L, S, E = control.lqr(A.T, C.T, self.Vd, self.Vn)  # duality
        L = L.T
        S = S.T
        print(f'L: {L}')
        # build state observer
        A_ob = A-L@C
        B_ob = np.bmat([B, L])
        C_ob = np.eye(len(A))
        D_ob = np.array(np.zeros((np.shape(C_ob)[0], np.shape(B_ob)[1])))
        u = np.array([reference])
        y = np.array([self.y_ol_out])
        # print(np.shape(u), np.shape(y))
        u_ob = np.bmat([[u],
                       [y]])
        # x0 = np.array([[0.5], [-0.5]])
        # simulate system
        self.ss_obsv = control.ss(A_ob, B_ob, C_ob, D_ob)
        self.time_out, self.x_hat = control.forced_response(self.ss_obsv, time, u_ob)
        return self.time_out, self.x_hat
    
    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        # plt.plot(self.time_out, self.reference, label='ref')
        plt.plot(self.time_out, self.y_ol_out, label='y (ol)')
        for i in range(len(self.x_ol_out)):
            plt.plot(self.time_out, self.x_ol_out[i, :], '-', label=f'x (ol) {i+1}')
        for i in range(len(self.x_hat)):
            plt.plot(self.time_out, self.x_hat[i, :], '--', label=f'xhat {i+1}')
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
    
    def excite_dist_noise(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray):
        self.reference = reference
        # build plant system
        self.ss_plant = plant
        A, B, C, D = control.ssdata(plant)
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_o != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        # build plant with disturbance and noise system
        B_aug = np.bmat([B, self.Vd, np.zeros(np.shape(B))])  # 0 of matrix size B
        D_aug = np.bmat([D, np.zeros(np.shape(C)), self.Vn])  # 0 of matrix size C
        ss_plant_dn = control.ss(A, B_aug, C, D_aug)
        # build plant with disturbance system
        ss_plant_d = control.ss(A, B_aug, C, np.zeros(np.shape(D_aug))) # 0 of matrix size D_aug
        # find L gain
        L, S, E = control.lqr(A.T, C.T, self.Vd, self.Vn)
        L = L.T
        S = S.T
        print(f'L: {L}')
        # build state estimator
        A_ob = A-L@C
        B_ob = np.bmat([B, L])
        C_ob = np.eye(len(A))
        D_ob = np.array([[0, 0],
                         [0, 0]])
        self.ss_obsv = control.ss(A_ob, B_ob, C_ob, D_ob)
        # build u augmented with disturbance and noise
        u = np.array([reference])
        u_dist = np.sqrt(self.Vd)@np.random.randn(2, len(time))
        u_noise = np.sqrt(self.Vn)@np.random.randn(1, len(time))
        u_aug = np.bmat([[u],
                         [u_dist],
                         [u_noise]])
        print(np.shape(u), np.shape(u_dist), np.shape(u_noise), np.shape(u_aug))
        # simulate each system
        tout, yout_dn = control.forced_response(ss_plant_dn, time, u_aug)
        tout, yout_d, xout_d = control.forced_response(ss_plant_d, time, u_aug, return_x=True)
        tout, yout, xout = control.forced_response(plant, time, reference, return_x=True)
        y = np.array(yout_dn)
        # print(np.shape(y))
        u_ob = np.bmat([[u],  # clean input
                        [y]])  # output with disturbance and noise
        print(np.shape(u), np.shape(y), np.shape(u_ob))
        tout, xhat = control.forced_response(self.ss_obsv, time, u_ob)
        # plotting each system
        plt.figure()
        for i in range(len(yout_dn)):
            plt.plot(tout, yout_dn[i,:], '-', label=f'y dist+noise {i+1}')  # with dist and noise
        for i in range(len(xout_d)):
            plt.plot(tout, xout_d[i,:], '-', label=f'x dist {i+1}')  # with dist
        for i in range(len(xout)):
            plt.plot(tout, xout[i, :], '-', label=f'true x {i+1}')  # true
        for i in range(len(xhat)):
            plt.plot(tout, xhat[i,:], '--', label=f'x est {i+1}')  # x hat
        plt.legend(loc='right')
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('LQE with disturbance and noise')
        plt.grid()
        plt.show()
        