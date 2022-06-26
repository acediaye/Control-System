import numpy as np
import matplotlib.pyplot as plt
import control

class LQR(object):
    def __init__(self, Q: np.ndarray, R: np.ndarray):
        self.Q = Q
        self.R = R
        self.ss_plant = None
        self.ss_cl = None
        self.ss_kr = None
        # u = r*K_r - K*x
        
        # save response values
        self.time_out = None
        self.y_ol_out = None
        self.y_cl_out = None
        self.y_kr_out = None
        self.x_kr_out = None
        self.u_kr_out = None
        
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        self.ss_plant = plant
        A, B, C, D = control.ssdata(plant)
        
        # open loop response
        self.time_out, self.y_ol_out = control.forced_response(self.ss_plant, time, reference)
        # close loop response
        K, S, E = control.lqr(A, B, self.Q, self.R)
        print(f'K: {K}, S: {S}, E: {E}')
        A_cl = A - B*K
        self.ss_cl = control.ss(A_cl, B, C, D)
        self.time_out, self.y_cl_out = control.forced_response(self.ss_cl, time, reference)
        # close loop response with gain
        dc = control.dcgain(self.ss_cl)
        K_r = 1/dc
        self.ss_kr = control.ss(A-B*K, B*K_r, C, D)
        self.time_out, self.y_kr_out, self.x_kr_out = control.forced_response(self.ss_kr, time, reference, return_x=True)
        return self.time_out, self.y_kr_out, self.x_kr_out
        
    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.plot(self.time_out, self.y_ol_out, '--', label='y (ol)')
        plt.plot(self.time_out, self.y_cl_out, '--', label='y (cl)')
        plt.plot(self.time_out, self.y_kr_out, '-', label='y (kr)')
        for i in range(len(self.x_kr_out)):
            plt.plot(self.time_out, self.x_kr_out[i, :], ':', label=f'x{i+1}')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        plt.title('LQR response')
        if save == True:
            plt.savefig('plots2/lqr_response.png')
        plt.show()
    
    def pzmap(self):
        if self.ss_kr is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.ss_kr, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
