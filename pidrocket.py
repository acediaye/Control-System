import numpy as np
import matplotlib.pyplot as plt

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
        self.max_u = 20
        self.max_windup = 20
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
        if self.ki * self.integral_error > self.max_windup:
            self.integral_error = self.max_windup/self.ki
        elif self.ki * self.integral_error < -self.max_windup:
            self.integral_error = -self.max_windup/self.ki
        derivative_error = (error - self.prev_error) / time_step
        u_output = self.kp * proportional_error + self.ki * self.integral_error + self.kd * derivative_error
        # limit u output/signal to actuator
        if u_output > self.max_u:
            u_output = self.max_u
        elif u_output < 0:
            u_output = 0
        self.prev_error = error
        self.prev_time = time
        print(f'r:{reference}, y:{measured_value}, e:{error}, u:{u_output}')
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
        plt.plot(self.time_arr, self.u_arr, label='thrust')
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
        
class Model(object):
    def __init__(self, G: float, MASS: float):
        self.g = G
        self.mass = MASS
        self.prev_time = 0
        self.pos = 0
        self.vel = 0
        # self.acc = 0
        # plotting
        self.time_arr = np.array([])
        self.pos_arr = np.array([])
        self.vel_arr = np.array([])
        self.acc_arr = np.array([])
    
    def excite(self, time: float, thrust: float) -> float:
        """
        a=F/m
        """
        time_step = time - self.prev_time
        acc = self.g + thrust / self.mass
        self.vel = self.vel + acc*time_step
        self.pos = self.pos + self.vel*time_step
        self.prev_time = time
        # plotting
        self.time_arr = np.append(self.time_arr, time)
        self.pos_arr = np.append(self.pos_arr, self.pos)
        self.vel_arr = np.append(self.vel_arr, self.vel)
        self.acc_arr = np.append(self.acc_arr, acc)
        
        return self.pos
    
    def graph(self):
        plt.figure(2)
        plt.subplot(3, 1, 1)
        plt.plot(self.time_arr, self.pos_arr, label='position')
        plt.ylabel('position')
        plt.subplot(3, 1, 2)
        plt.plot(self.time_arr, self.vel_arr, label='velocity')
        plt.ylabel('velocity')
        plt.subplot(3, 1, 3)
        plt.plot(self.time_arr, self.acc_arr, label='acceleration')
        plt.ylabel('acceleration')
        plt.xlabel('time')
        plt.show()
    
    def get_pos(self):
        return self.pos

if __name__ == '__main__':
    print('rocket main')
    # constants
    # KP = 0.6
    # KI = 0.07
    # KD = 1.275
    # KP = 1
    # KI = 0
    # KD = 0
    KU = 1  # gain to get y to have stable oscillation
    TU = 25.7 - 8.8  # time oscillation of 1 period
    KP = 0.6*KU
    KI = 1.2*KU/TU
    KD = 3/40*KU*TU
    TIME_STEP = 0.1
    # input reference array
    TIME = np.arange(0+TIME_STEP, 100+TIME_STEP, TIME_STEP)
    # REFERENCE = 100 * np.ones(len(TIME))  # constant 100
    REFERENCE = 100 * np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))  # constant 100 then 0
    # REFERENCE = 100 * np.sin(TIME)  # sine wave
    print(len(REFERENCE))
    # instantiate
    mypid = PID(KP, KI, KD)
    mymodel = Model(-9.8, 1)
    # loop
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        u = mypid.controller(t, r, mymodel.get_pos())
        y = mymodel.excite(t, u)
    # plotting
    mypid.graph()
    # mypid.graph_errors()
    # mymodel.graph()
