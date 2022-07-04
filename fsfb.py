import control
import numpy as np
import matplotlib.pyplot as plt

class FSFB(object):
    def __init__(self, eigenvalues: np.ndarray):
        self.poles_desire = eigenvalues
        self.ss_plant = None  # state space plant
        self.ss_cl = None  # state space close loop
        self.ss_kr = None  # state space close loop with gain
        
        # save response values
        self.time_out = None
        self.y_ol_out = None
        self.y_cl_out = None
        self.y_kr_out = None
        self.x_kr_out = None
        self.u_kr_out = None
        self.reference = None
        
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        self.reference = reference
        self.ss_plant = plant
        A, B, C, D = control.ssdata(plant)
        if len(self.poles_desire) != np.shape(A)[0]:  # check n
            raise RuntimeError('missing eigenvalues')
        ctrb = control.ctrb(A, B)
        rank_c = np.linalg.matrix_rank(ctrb)
        if rank_c != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        
        # open loop response
        self.time_out, self.y_ol_out = control.forced_response(self.ss_plant, time, reference)
        # close loop response
        K = control.place(A, B, self.poles_desire)
        A_cl = A - B@K
        self.ss_cl = control.ss(A_cl, B, C, D)
        self.time_out, self.y_cl_out = control.forced_response(self.ss_cl, time, reference)
        # close loop response with ref gain
        dc = control.dcgain(self.ss_cl)
        K_r = np.array([[1/dc]])
        self.ss_kr = control.ss(A-B@K, B@K_r, C, D)
        self.time_out, self.y_kr_out, self.x_kr_out = control.forced_response(self.ss_kr, time, reference, return_x=True)
        self.u_kr_out = K_r*reference - K@self.x_kr_out
        print(f'K: {K}, Kr: {K_r}')
        return self.time_out, self.y_kr_out, self.x_kr_out

    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.time_out, self.reference, label='ref')
        plt.plot(self.time_out, self.y_ol_out, label='y (ol)')
        plt.plot(self.time_out, self.y_cl_out, label='y (cl)')
        plt.plot(self.time_out, self.y_kr_out, label='y (kr)')
        for i in range(len(self.x_kr_out)):
            plt.plot(self.time_out, self.x_kr_out[i, :], '--', label=f'x{i+1}')
        plt.legend(loc='right')
        plt.ylabel('amplitude')
        plt.grid()
        plt.title('FSFB response')
        plt.subplot(2, 1, 2)
        plt.plot(self.time_out, np.squeeze(self.u_kr_out), label='u')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        if save == True:
            plt.savefig('plots2/fsfb_response.png')
        plt.show()
    
    def pzmap(self):
        if self.ss_kr is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.ss_kr, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
