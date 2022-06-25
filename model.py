import numpy as np
import matplotlib.pyplot as plt
import control

class Mass_Spring_Damper_System(object):
    def __init__(self, M: float, K: float, C: float):
        """
        sum_F(t) = F(t) - c*x(t)_dot - k*x(t) = m*x(t)_dotdot
        x(t)_dotdot = -k/m * x(t) -c/m * x(t)_dot + F(t)/m
        """
        self.m = M  # mass constant
        self.c = C  # dampening constant
        self.k = K  # spring constant

        self.prev_time = 0
        self.x1_curr = 0
        self.x2_curr = 0
        self.y_curr = 0
        self.x1_next = 0
        self.x2_next = 0
        
        self.x1_arr = np.array([])
        self.x2_arr = np.array([])
        self.y_arr = np.array([])
        self.time_arr = np.array([])
    
    def plant(self) -> control.StateSpace:
        """
        2x1
        x_bar = [x]
                [x_dot]

        2x1 = 2x2 * 2x1 + 2x1 + 1x1
        x_bardot = [x_dot]
                   [x_dotdot]
                = [   0    1][    x] + [  0][F]
                  [-k/m -c/m][x_dot]   [1/m]
        
        1x1 = 1x2 * 2x1 + 1x1 * 1x1
        y = [1 0][    x] + [0][F]
                 [x_dot]
        """
        A = np.array([[0, 1], [-self.k/self.m, -self.c/self.m]])
        B = np.array([[0], [1/self.m]])
        C = np.array([1, 0]).reshape((1, 2))
        D = np.array([0]).reshape((1, 1))
        state_space = control.ss(A, B, C, D)
        return state_space

    def plant_discrete(self, time: float, u: float) -> float:
        """
        only takes in 2x2 A matrix
        """
        T = time - self.prev_time
        a11 = 1
        a12 = T
        a21 = -T*self.k/self.m
        a22 = 1 - T*self.c/self.m
        b1 = 0
        b2 = T/self.m
        c1 = 1
        c2 = 0
        d = 0
        
        self.x1_next = a11*self.x1_curr + a12*self.x2_curr + b1*u
        self.x2_next = a21*self.x1_curr + a22*self.x2_curr + b2*u
        self.y_curr = c1*self.x1_curr + c2*self.x2_curr + d*u
        
        self.x1_curr = self.x1_next
        self.x2_curr = self.x2_next
        self.prev_time = time
        
        self.x1_arr = np.append(self.x1_arr, self.x1_curr)
        self.x2_arr = np.append(self.x2_arr, self.x2_curr)
        self.y_arr = np.append(self.y_arr, self.y_curr)
        self.time_arr = np.append(self.time_arr, time)
        
        return self.y_curr
    
    def graph(self, save: bool):
        plt.figure()
        plt.plot(self.time_arr, self.y_arr, label='y (pos)')
        plt.plot(self.time_arr, self.x1_arr, '--', label='x1 (pos)')
        plt.plot(self.time_arr, self.x2_arr, '--', label='x2 (vel)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('model discrete response')
        plt.grid()
        # print(f'model graph: {len(self.time_arr)}, {len(self.x1_arr)}, {len(self.x2_arr)}, {len(self.y_arr)}')
        if save == True:
            plt.savefig('plots/model_discrete_response.png')
        plt.show()
