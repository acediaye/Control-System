import control
import numpy as np
import matplotlib.pyplot as plt

class FSTB(object):
    def __init__(self, eigenvalues: np.ndarray):
        self.poles_desire = eigenvalues
        self.K = 0
        self.K_r = 0
        self.sys_scaled = None
        
        # save response values
        self.time_out = None
        self.y_out = None
        
    def excite(self, plant: control.TransferFunction, time: np.ndarray, reference: np.ndarray) -> tuple:
        ss = control.tf2ss(plant)
        A, B, C, D = control.ssdata(ss)
        if len(self.poles_desire) != np.shape(A)[0] or len(self.poles_desire) != np.shape(A)[1]:
            raise RuntimeError('not square matrix or missing eigenvalues')
        ctrb = control.ctrb(A, B)
        rank_c = np.linalg.matrix_rank(ctrb)
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_c != np.shape(A)[0] or rank_c != np.shape(A)[1]:
            raise RuntimeError('not full rank')
        
        self.K = control.place(A, B, self.poles_desire)
        A_cl = A - B*self.K
        sys_cl = control.ss(A_cl, B, C, D)
        dc = control.dcgain(sys_cl)
        self.Kr = 1/dc
        self.sys_scaled = control.ss(A-B*self.K, B*self.Kr, C, D)
        self.time_out, self.y_out = control.forced_response(self.sys_scaled, time, reference)
        return self.time_out, self.y_out

    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.plot(self.time_out, self.y_out, label='y (pos)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        plt.title('FSTB response')
        if save == True:
            plt.savefig('plots/fstb_response.png')
        plt.show()
    
    def pzmap(self):
        if self.sys_scaled is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.sys_scaled)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
