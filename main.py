import numpy as np
import matplotlib.pyplot as plt
import control
import pid
import model

if __name__ == '__main__':
    mymodel = model.MASS_SPRING_DAMPER_SYSTEM(1, 20, 10)
    ss = mymodel.plant()
    # print(f'ss: {ss}')
    
    P = control.ss2tf(ss)
    print(f'P: {P}')
    
    TIME_STEP = 0.002
    TIME = np.arange(0+TIME_STEP, 2+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
    print(len(TIME))
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    mypid = pid.PID(350, 300, 50)
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
    
    t, yout = control.forced_response(H, TIME, REFERENCE)
    print(f'yout: {np.shape(yout)}')
    # print(f'force: {np.shape(xout)}')
    plt.figure()
    plt.plot(t, yout, label='y')
    # for i in range(len(xout)):
    #     plt.plot(t, xout[i, :], label=f'x{i+1}')
    plt.legend()
    plt.ylabel('position')
    plt.xlabel('time')
    plt.show()
    