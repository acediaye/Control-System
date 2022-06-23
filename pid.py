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
        
    def graph(self):
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.time_arr, self.r_arr, 'b', label='reference')
        plt.plot(self.time_arr, self.y_arr, 'g', label='measured value')
        plt.plot(self.time_arr, self.e_arr, 'r', label='error')
        plt.ylabel('position')
        plt.legend()
        plt.subplot(2, 1, 2)
        plt.plot(self.time_arr, self.u_arr, label='u')
        plt.ylabel('u')
        plt.xlabel('time')
        plt.legend()
        print(f'control graph: {len(self.time_arr)}, {len(self.r_arr)}, {len(self.y_arr)}, {len(self.e_arr)}, {len(self.u_arr)}')
        plt.show()

class MASS_SPRING_DAMPER_SYSTEM(object):
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
    
    def plant(self):
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
    
    def graph(self):
        plt.figure()
        plt.plot(self.time_arr, self.x1_arr, label='x1')
        plt.plot(self.time_arr, self.x2_arr, label='x2')
        plt.plot(self.time_arr, self.y_arr, label='y')
        plt.legend()
        plt.ylabel('position')
        plt.xlabel('time')
        print(f'model graph: {len(self.time_arr)}, {len(self.x1_arr)}, {len(self.x2_arr)}, {len(self.y_arr)}')
        plt.show()

if __name__ == '__main__':
    print('hello')
    mymodel = MASS_SPRING_DAMPER_SYSTEM(1, 20, 10)
    ss = mymodel.plant()
    print(f'ss: {ss}')
    
    P = control.ss2tf(ss)
    print(f'P: {P}')
    
    TIME_STEP = 0.002
    TIME = np.arange(0+TIME_STEP, 2+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
    print(len(TIME))
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    mypid = PID(350, 300, 50)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        u = mypid.controller_discrete(t, r, mymodel.y_curr)
        y = mymodel.plant_discrete(t, u)
    mypid.graph()
    mymodel.graph()
    
    Kp = 350
    Ki = 300
    Kd = 50
    s = control.tf('s')
    C = (Kp + Ki/s + Kd*s)
    print(f'C: {C}')
    L = control.series(C, P)
    print(f'L: {L}')
    H = control.feedback(L, 1)
    print(f'H: {H}')
    
    t, yout, xout = control.forced_response(H, TIME, REFERENCE, return_x=True)
    print(f'force: {np.shape(yout)}')
    print(f'force: {np.shape(xout)}')
    print(len(xout))
    plt.figure()
    plt.plot(t, yout, label='y')
    for i in range(len(xout)):
        plt.plot(t, xout[i, :], label=f'x{i+1}')
    plt.legend()
    plt.show()
    