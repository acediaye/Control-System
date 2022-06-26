import numpy as np
import matplotlib.pyplot as plt
import control

class PID(object):
    def __init__(self, KP: float, KI: float, KD: float):
        """
        """
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.C = None  # controller tf
        self.H = None  # system tf
        
        # save response values
        self.time_out = None
        self.y_out = None
        self.x_out = None
        
        # for discrete values
        self.prev_error = 0
        self.prev2_error = 0
        self.prev_time = 0
        self.prev_u = 0
        # save discrete values
        self.time_arr = np.array([])
        self.r_arr = np.array([])
        self.e_arr = np.array([])
        self.u_arr = np.array([])
        self.y_arr = np.array([])
    
    def controller(self) -> control.TransferFunction:
        s = control.tf('s')
        self.C = self.kp + self.ki/s + self.kd*s
        return self.C
    
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        if self.C is None:
            raise RuntimeError('run controller')
        print(f'C: {self.C}')
        P = control.ss2tf(plant)
        print(f'P: {P}')
        L = control.series(self.C, P)
        print(f'L: {L}')
        self.H = control.feedback(L, 1)
        print(f'H: {self.H}')
        self.time_out, self.y_out, self.x_out = control.forced_response(self.H, time, reference, return_x=True)
        return self.time_out, self.y_out, self.x_out

    def graph(self, save: bool):
        if self.time_out is None:
            raise RuntimeError('run excite')
        plt.figure()
        plt.plot(self.time_out, self.y_out, label='y (pos)')
        for i in range(len(self.x_out)):
            plt.plot(self.time_out, self.x_out[i, :], label=f'x{i+1}')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('pid response')
        plt.grid()
        if save == True:
            plt.savefig('plots/pid_response.png')
        plt.show()
        
    def pzmap(self):
        if self.H is None:
            raise RuntimeError('run excite')
        plt.figure()
        poles, zeros = control.pzmap(self.H, plot=True)
        print(f'poles: {poles}, zeros: {zeros}')
        plt.show()
    
    def controller_discrete(self, time: float, reference: float, measured_value: float) -> float:
        T = time - self.prev_time
        error = reference - measured_value
        u_output = (self.prev_u 
                    + (self.kp+self.ki*T+self.kd/T)*error 
                    + (-self.kp-2*self.kd/T)*self.prev_error 
                    + self.kd/T*self.prev2_error)
        
        self.prev2_error = self.prev_error
        self.prev_error = error
        self.prev_time = time
        self.prev_u = u_output
        
        self.time_arr = np.append(self.time_arr, time)
        self.r_arr = np.append(self.r_arr, reference)
        self.e_arr = np.append(self.e_arr, error)
        self.u_arr = np.append(self.u_arr, u_output)
        self.y_arr = np.append(self.y_arr, measured_value)
        
        return u_output
        
    def graph_discrete(self, save: bool):
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.time_arr, self.r_arr, 'b', label='reference')
        plt.plot(self.time_arr, self.y_arr, 'g', label='measured value')
        plt.plot(self.time_arr, self.e_arr, 'r', label='error')
        plt.legend()
        plt.ylabel('amplitude')
        plt.title('pid discrete response')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.plot(self.time_arr, self.u_arr, label='u (control)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        # print(f'control graph: {len(self.time_arr)}, {len(self.r_arr)}, {len(self.y_arr)}, {len(self.e_arr)}, {len(self.u_arr)}')
        if save == True:
            plt.savefig('plots/pid_discrete_response.png')
        plt.show()
