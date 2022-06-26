import control
import numpy as np
import matplotlib.pyplot as plt

class FSTB(object):
    def __init__(self, eigenvalues: np.ndarray):
        self.poles_desire = eigenvalues
        self.K = 0
        self.K_r = 0
        self.sys_plant = None
        self.sys_cl = None
        self.sys_kr = None
        
        # save response values
        self.time_out = None
        self.y_ol_out = None
        self.y_cl_out = None
        self.y_kr_out = None
        
    def excite(self, plant: control.TransferFunction, time: np.ndarray, reference: np.ndarray) -> tuple:
        self.sys_plant = control.tf2ss(plant)
        A, B, C, D = control.ssdata(self.sys_plant)
        if len(self.poles_desire) != np.shape(A)[0] or len(self.poles_desire) != np.shape(A)[1]:
            raise RuntimeError('not square matrix or missing eigenvalues')
        ctrb = control.ctrb(A, B)
        rank_c = np.linalg.matrix_rank(ctrb)
        if rank_c != np.shape(A)[0] or rank_c != np.shape(A)[1]:
            raise RuntimeError('not full rank')
        
        # open loop response
        self.time_out, self.y_ol_out = control.forced_response(self.sys_plant, time, reference)
        # close loop response
        self.K = control.place(A, B, self.poles_desire)
        A_cl = A - B*self.K
        self.sys_cl = control.ss(A_cl, B, C, D)
        self.time_out, self.y_cl_out = control.forced_response(self.sys_cl, time, reference)
        # close loop response with ref gain
        dc = control.dcgain(self.sys_cl)
        self.K_r = 1/dc
        self.sys_kr = control.ss(A-B*self.K, B*self.K_r, C, D)
        self.time_out, self.y_kr_out = control.forced_response(self.sys_kr, time, reference)
        
        return self.time_out, self.y_kr_out

    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.plot(self.time_out, self.y_ol_out, '--', label='y (ol)')
        plt.plot(self.time_out, self.y_cl_out, '--', label='y (cl)')
        plt.plot(self.time_out, self.y_kr_out, label='y (kr)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        plt.title('FSTB response')
        if save == True:
            plt.savefig('plots2/fstb_response.png')
        plt.show()
    
    def pzmap(self):
        if self.sys_kr is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.sys_plant, plot=False)
        print(f'open loop poles: {poles}, zeros: {zeros}')
        poles, zeros = control.pzmap(self.sys_cl, plot=False)
        print(f'close loop poles: {poles}, zeros: {zeros}')
        poles, zeros = control.pzmap(self.sys_kr, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        print(f'gains K: {self.K}, Kr: {self.K_r}')
        plt.show()
