import numpy as np
import matplotlib.pyplot as plt
import control
from scipy.integrate import odeint

class PID(object):
    def __init__(self, KP: float, KI: float, KD: float):
        """
        """
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.prev_error = 0
        self.prev_time = 0
        self.integral_error = 0
        self.max_u = 1000
        # self.max_windup = 1000  # integrator accumulator
        # plotting
        self.time_arr = np.array([])
        self.kpe_arr = np.array([])
        self.kie_arr = np.array([])
        self.kde_arr = np.array([])
        self.r_arr = np.array([])
        self.e_arr = np.array([])
        self.u_arr = np.array([])
        self.y_arr = np.array([])
    
    def controller(self, time:float, reference: float, measured_value: float) -> float:
        """
        input: reference and measured value -> controller -> output: u
        """
        time_step = time - self.prev_time
        error = reference - measured_value
        proportional_error = error
        self.integral_error = (self.integral_error + error) * time_step
        # limit integral windup/accumulation
        # if self.ki * self.integral_error > self.max_windup:
        #     self.integral_error = self.max_windup/self.ki
        # elif self.ki * self.integral_error < -self.max_windup:
        #     self.integral_error = -self.max_windup/self.ki
        derivative_error = (error - self.prev_error) / time_step
        u_output = self.kp * proportional_error + self.ki * self.integral_error + self.kd * derivative_error
        # limit u output/signal to actuator
        if u_output > self.max_u:
            u_output = self.max_u
        elif u_output < 0:
            u_output = 0
        self.prev_error = error
        self.prev_time = time
        # print(f'r:{reference}, y:{measured_value}, e:{error}, u:{u_output}')
        # plotting
        self.time_arr = np.append(self.time_arr, time)
        self.kpe_arr = np.append(self.kpe_arr, self.kp * proportional_error)
        self.kie_arr = np.append(self.kie_arr, self.ki * self.integral_error)
        self.kde_arr = np.append(self.kde_arr, self.kd * derivative_error)
        self.r_arr = np.append(self.r_arr, reference)
        self.e_arr = np.append(self.e_arr, error)
        self.u_arr = np.append(self.u_arr, u_output)
        self.y_arr = np.append(self.y_arr, measured_value)
        
        return u_output
    
    def graph(self):
        plt.figure(1)
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
        plt.show()
    
    def graph_k(self):
        plt.figure(1)
        plt.subplot(3, 1, 1)
        plt.plot(self.time_arr, self.kpe_arr)
        plt.ylabel('kp errors')
        plt.subplot(3, 1, 2)
        plt.plot(self.time_arr, self.kie_arr)
        plt.ylabel('ki errors')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_arr, self.kde_arr)
        plt.ylabel('kd errors')
        plt.xlabel('time')
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
        self.vel = 0
        self.pos = 0
        self.prev_time = 0
    
    def state_space(self):
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
        print(f'A: {np.shape(A)}, B: {np.shape(B)}, C: {np.shape(C)}, D: {np.shape(D)}')
        return A, B, C, D
    
    def ss(self, A, B, C, D):
        result = control.ss(A, B, C, D)
        print(result)
        return result

    def excite(self, time, u):
        F = u
        time_step = time - self.prev_time
        acc = F/self.m - self.c/self.m*self.vel - self.k/self.m*self.pos
        # acc = -9.8 + u/1
        self.vel = self.vel + acc*time_step
        self.pos = self.pos + self.vel*time_step
        self.prev_time = time
        return self.pos
    
    def get_pos(self):
        return self.pos

# http://apmonitor.com/pdc/index.php/Main/ModelSimulation
def model(x_bar: list, t: np.array, m, c, k):
    F = REF(t)
    # print(t, F)
    x = x_bar[0]
    x_dot = x_bar[1]
    x_dotdot = F/m - c/m*x_dot - k/m*x
    x_bardot = [x_dot, x_dotdot]
    return x_bardot

def REF(t):
    if t>50:
        return 0
    else:
        return 1
    
if __name__ == '__main__':
    print('hello')
    mymodel = MASS_SPRING_DAMPER_SYSTEM(1, 20, 10)
    (A, B, C, D) = mymodel.state_space()  
    
    sys = control.ss(A, B, C, D)
    print(f'sys: {sys}')
    sys2 = control.ss2tf(sys)
    print(f'P: {sys2}')
    # print(control.tf('s'))
    
    TIME_STEP = 0.1
    TIME = np.arange(0+TIME_STEP, 100+TIME_STEP, TIME_STEP)
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    
    mypid = PID(300, 0, 0)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        u = mypid.controller(t, r, mymodel.get_pos())
        y = mymodel.excite(t, u)
    mypid.graph()
    
    # print(sys2)
    Kp = 300
    Ki = 0
    Kd = 0
    s = control.tf('s')
    C = (Kp + Ki/s + Kd*s)
    print(f'C: {C}')
    temp = control.series(C, sys2)
    print(temp)
    H = control.feedback(temp, 1)
    print(f'H: {H}, {type(H)}')
    
    # print(len(TIME), len(REFERENCE))
    sys = control.tf2ss(H)
    t, y = control.forced_response(H, TIME, REFERENCE)
    print(f'force: {np.shape(y)}')
    # print(f'force: {np.shape(x)}')
    plt.figure(2)
    plt.subplot(2, 1, 1)
    plt.plot(t, y)
    # plt.plot(t, x)
    
    # as the solver has adaptive step size control, that is, 
    # it will use internal time steps that you have no control over, 
    # and each time step uses several evaluations of the function. 
    # Thus there is no connection between the solver time steps and 
    # the data time steps.
    # internal ode time will differ from user selected time
    y = odeint(model, [0, 0], TIME, args=(1, 10, 20))
    print(np.shape(y))
    print(y)
    y1 = y[:, 0]
    y2 = y[:, 1]
    plt.plot(t, y1, 'g')
    plt.plot(t, y2, 'r')
    
    t, y = control.step_response(H, TIME)
    plt.subplot(2, 1, 2)
    plt.plot(t, y)
    plt.show()
    