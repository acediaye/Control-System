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
        
        self.prev_u = 0
        self.prevprev_error = 0
    
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
    
    def control2(self, time, reference, measured_value):
        Ti = 5
        Ts = time - self.prev_time
        error = reference - measured_value
        # u_output = self.prev_u + self.kp*(error - self.prev_error) + self.kp/Ti*Ts*error
        # u_output = self.kp*error \
        #            + self.ki*Ts/2*(error+self.prev_error) + self.ki*self.prev_u \
        #            + self.kd/Ts*(error-self.prev_error)
        u_output = self.prev_u \
                    + (self.kp+self.ki*Ts+self.kd/Ts)*error \
                    + (-self.kp-2*self.kd/Ts)*self.prev_error \
                    + self.kd/Ts*self.prevprev_error
        print(u_output)
        
        self.prevprev_error = self.prev_error
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
        
        self.x1_next = 0
        self.x2_next = 0
        self.x1_curr = 0
        self.x2_curr = 0
        self.x1 = np.array([0])
        self.x2 = np.array([0])
        self.time = np.array([0])
    
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

    def excite2(self, time , u):
        Ts = time - self.prev_time
        a11 = 1
        a12 = Ts
        a21 = -(Ts*self.k)/self.m
        a22 = 1 - (Ts*self.c/self.m)
        b1 = 0
        b2 = Ts/self.m
        
        self.x1_next = a11*self.x1_curr + a12*self.x2_curr + b1*u
        self.x2_next = a21*self.x1_curr + a22*self.x2_curr + b2*u
        
        self.x1_curr = self.x1_next
        self.x2_curr = self.x2_next
        self.prev_time = time
        
        self.x1 = np.append(self.x1, self.x1_curr)
        self.x2 = np.append(self.x2, self.x2_curr)
        self.time = np.append(self.time, time)
        
        return self.x1_curr
    
    def graph(self):
        plt.figure()
        plt.plot(self.time, self.x1)
        plt.plot(self.time, self.x2)
        print(f'model graph: {len(self.time), len(self.x1), len(self.x2)}')
        plt.show()

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
    
    TIME_STEP = 0.01
    TIME = np.arange(0+TIME_STEP, 10+TIME_STEP, TIME_STEP)
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    # mypid = PID(30, 70, 0)
    mypid = PID(350, 300, 50)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        u = mypid.control2(t, r, mymodel.x1_curr)
        # u = 1
        y = mymodel.excite2(t, u)
    mypid.graph()
    mymodel.graph()
    
    # print(sys2)
    Kp = 350
    Ki = 300
    Kd = 50
    s = control.tf('s')
    C = (Kp + Ki/s + Kd*s)
    print(f'C: {C}')
    temp = control.series(C, sys2)
    print(temp)
    H = control.feedback(temp, 1)
    print(f'H: {H}, {type(H)}')
    
    # print(len(TIME), len(REFERENCE))
    # sys = control.tf2ss(H)
    t, yout, xout = control.forced_response(H, TIME, REFERENCE, return_x=True)
    print(f'force: {np.shape(yout)}')
    print(f'force: {np.shape(xout)}')
    plt.figure(2)
    plt.subplot(2, 1, 1)
    plt.plot(t, yout)
    # plt.plot(t, x)
    
    # as the solver has adaptive step size control, that is, 
    # it will use internal time steps that you have no control over, 
    # and each time step uses several evaluations of the function. 
    # Thus there is no connection between the solver time steps and 
    # the data time steps.
    # internal ode time will differ from user selected time
    # y = odeint(model, [0, 0], TIME, args=(1, 10, 20))
    # print(f'ode: {np.shape(y)}')
    # print(y)
    # y1 = y[:, 0]
    # y2 = y[:, 1]
    # plt.plot(t, y1, 'g')
    # plt.plot(t, y2, 'r')
    
    # t, y = control.step_response(H, TIME)
    # print(f'step: {np.shape(y)}')
    # plt.subplot(2, 1, 2)
    # plt.plot(t, y)
    # plt.show()
    
    