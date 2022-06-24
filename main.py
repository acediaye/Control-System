import numpy as np
import matplotlib.pyplot as plt
import control
import pid
import model

# turn parts of main on or off
switch = {'open loop': True, 
          'pid_discrete': True,
          'pid': True,
          'fstb': False,
          'lqr': False}

if __name__ == '__main__':
    TIME_STEP = 0.002
    TIME = np.arange(0+TIME_STEP, 3+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
    print(f'time: {len(TIME)}')
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    mymodel = model.MASS_SPRING_DAMPER_SYSTEM(1, 20, 10)
    ss = mymodel.plant()
    # print(f'ss: {ss}')
    P = control.ss2tf(ss)
    print(f'P: {P}')
    
    Kp = 350
    Ki = 300
    Kd = 50
    
    if switch['open loop']:  
        t, yout = control.step_response(P, TIME)
        # print(f'yout: {np.shape(yout)}')
        plt.figure()
        plt.plot(t, yout, label='y')
        plt.legend()
        plt.ylabel('position')
        plt.xlabel('time')
        plt.title('open loop step response')
        plt.grid()
        plt.savefig('pics/open loop step response.png')
        plt.show()

    if switch['pid_discrete']:
        mypid = pid.PID(Kp, Ki, Kd)
        for i in range(len(TIME)):
            t = TIME[i]
            r = REFERENCE[i]
            u = mypid.controller_discrete(t, r, mymodel.y_curr)
            y = mymodel.plant_discrete(t, u)
        mypid.graph()
        # mymodel.graph()
    
    if switch['pid']:
        s = control.tf('s')
        C = (Kp + Ki/s + Kd*s)
        print(f'C: {C}')
        L = control.series(C, P)
        print(f'L: {L}')
        H = control.feedback(L, 1)
        print(f'H: {H}')
        
        t, yout = control.forced_response(H, TIME, REFERENCE)
        # print(f'yout: {np.shape(yout)}')
        # print(f'force: {np.shape(xout)}')
        plt.figure()
        plt.plot(t, yout, label='y')
        # for i in range(len(xout)):
        #     plt.plot(t, xout[i, :], label=f'x{i+1}')
        plt.legend()
        plt.ylabel('position')
        plt.xlabel('time')
        plt.title('pid response')
        plt.grid()
        plt.savefig('pics/pid response.png')
        plt.show()
