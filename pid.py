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
        
        self.prev_error = 0
        self.prev2_error = 0
        self.prev_time = 0
        self.prev_u = 0
        
        self.time_arr = np.array([])
        self.r_arr = np.array([])
        self.e_arr = np.array([])
        self.u_arr = np.array([])
        self.y_arr = np.array([])
    
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
        
    def graph(self, save: bool):
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
