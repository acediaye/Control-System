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
        K, S, E = control.lqr(A, B, self.Q, self.R)
        L, S, E = control.lqr(A.T, C.T, self.Vd, self.Vn)
        L = L.T
        S = S.T
        K_r = 20.02498439
        Ace = np.bmat([[A-B@K, B@K],
                       [np.zeros(np.shape(A)), A-L@C]])
        Bce = np.bmat([[B*K_r],
                       [np.zeros(np.shape(B))]])
        Cce = np.bmat([[C, np.zeros(np.shape(C))]])
        Dce = D
        # print(np.shape(Ace), np.shape(Bce), np.shape(Cce), np.shape(Dce))
        self.ss_ce = control.ss(Ace, Bce, Cce, Dce)
        # x0 = np.array([[1], [-1], [2], [-2]])
        x0 = np.zeros((4,1))
        self.time_out, self.y_out, self.x_out = control.forced_response(self.ss_ce, time, reference, x0, return_x=True)
        # print(np.shape(self.y_out), np.shape(self.x_out))
        dc = control.dcgain(self.ss_ce)
        K_r = np.array([[1/dc]])
        # print(K_r)
        return self.time_out, self.y_out, self.x_out
        
    def graph(self, save):
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
    