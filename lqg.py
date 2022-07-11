import numpy as np
import matplotlib.pyplot as plt
import control

class LQG(object):
    def __init__(self, Q, R, Vd, Vn):
        self.Q = Q
        self.R = R
        self.Vd = Vd
        self.Vn = Vn
        self.ss_ce = None
        
        self.time_out = None
        self.y_out = None
        
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        A, B, C, D = control.ssdata(plant)
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_o != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        # find optimal gains
        K, S, E = control.lqr(A, B, self.Q, self.R)
        L, S, E = control.lqr(A.T, C.T, self.Vd, self.Vn)
        L = L.T
        S = S.T
        # gain for ref, find with closed loop
        K_r = 20.02498439
        # build states
        Ace = np.bmat([[A-B@K, B@K],
                       [np.zeros(np.shape(A)), A-L@C]])
        Bce = np.bmat([[B*K_r],
                       [np.zeros(np.shape(B))]])
        Cce = np.bmat([[C, np.zeros(np.shape(C))]])
        Dce = D
        # print(np.shape(Ace), np.shape(Bce), np.shape(Cce), np.shape(Dce))
        # simulate system
        self.ss_ce = control.ss(Ace, Bce, Cce, Dce)
        # x0 = np.array([[1], [-1], [2], [-2]])
        x0 = np.zeros((4,1))
        self.time_out, self.y_out, self.x_out = control.forced_response(self.ss_ce, time, reference, x0, return_x=True)
        # print(np.shape(self.y_out), np.shape(self.x_out))
        dc = control.dcgain(self.ss_ce)
        K_r = np.array([[1/dc]])
        # print(K_r)
        return self.time_out, self.y_out, self.x_out
        
    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.plot(self.time_out, self.y_out, '-', label=f'y')
        for i in range(len(self.x_out)):
            plt.plot(self.time_out, self.x_out[i, :], '--', label=f'x{i+1}')
        plt.legend(loc='right')
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('LQG response')
        plt.grid()
        if save==True:
            plt.savefig('plots4/lqg_response.png')
        plt.show()
        
    def pzmap(self):
        if self.ss_ce is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.ss_ce, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
    
    def excite_dist_noise(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray):
        A, B, C, D = control.ssdata(plant)
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_o != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        # find K
        K, S, E = control.lqr(A, B, self.Q, self.R)
        print(f'K: {K}')
        # find L
        L, S, E = control.lqr(A.T, C.T, self.Vd, self.Vn)
        L = L.T
        print(f'L: {L}')
        # build plant with disturbance and noise system
        K_r = 20.02498439
        A_aug = np.bmat([[A-B@K, B@K],
                         [np.zeros(np.shape(A)), A-L@C]])
        B_aug = np.bmat([[B*K_r, self.Vd, np.zeros(np.shape(B))],
                         [np.zeros(np.shape(B)), self.Vd, -L@self.Vn]])
        C_aug = np.bmat([C, np.zeros(np.shape(C))])
        D_aug = np.bmat([D, np.zeros(np.shape(C)), self.Vn])
        # print(f'states:\n{A_aug}\n{B_aug}\n{C_aug}\n{D_aug}')
        # print(np.shape(A_aug), np.shape(B_aug), np.shape(C_aug), np.shape(D_aug))
        ss_plant_dn = control.ss(A_aug, B_aug, C_aug, D_aug)
        # build u augmented with disturbance and noise
        u = np.array([reference])
        u_dist = np.sqrt(self.Vd)@np.random.normal(0, 0.1, (2, len(time)))  # gaussian: mean, std dev, size
        u_noise = np.sqrt(self.Vn)@np.random.normal(0, 0.1, (1, len(time)))
        u_aug = np.bmat([[u],
                         [u_dist],
                         [u_noise]])
        # print(np.shape(u), np.shape(u_dist), np.shape(u_noise), np.shape(u_aug))
        # simulate system
        tout, yout, xout = control.forced_response(plant, time, reference, return_x=True)
        tout, yout_dn, xout_dn = control.forced_response(ss_plant_dn, time, u_aug, return_x=True)
        # print(np.shape(tout), np.shape(yout_dn), np.shape(xout_dn))
        # calc K_r
        dc = control.dcgain(ss_plant_dn)
        K_r = np.array(1/dc)
        print(f'K_r: {K_r}, {np.shape(K_r)}') # gains for each output, ref, 2xdist, noise
        # plotting each system
        plt.figure()
        # plt.plot(tout, np.squeeze(yout), '-', label=f'true y')
        plt.plot(tout, np.squeeze(yout_dn), '-', label=f'y dist+noise')
        # for i in range(len(xout)):
        #     plt.plot(tout, np.squeeze(xout[i, :]), '-', label=f'true x {i+1}')
        for i in range(len(xout_dn)):
            plt.plot(tout, np.squeeze(xout_dn[i, :]), '--', label=f'est x {i+1}')  # with dist and noise
        plt.legend(loc='right')
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('LQG with disturbance and noise')
        plt.grid()
        plt.show()
        
        # # poles and zeros of system
        # poles, zeros = control.pzmap(plant, plot=False)
        # print(f'poles: {poles}, zeros: {zeros}')
        