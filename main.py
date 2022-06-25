import numpy as np
import matplotlib.pyplot as plt
import control
import pid
import model

# turn parts of main on or off
switch = {'open loop': False, 
          'pid_discrete': False,
          'pid': True,
          'fstb': True,
          'lqr': False}
# save plots
save = False

if __name__ == '__main__':
    TIME_STEP = 0.002
    TIME = np.arange(0+TIME_STEP, 3+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
    print(f'time: {len(TIME)}')
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    mymodel = model.Mass_Spring_Damper_System(1, 20, 10)
    ss = mymodel.plant()
    # print(f'ss: {ss}')
    P = control.ss2tf(ss)
    print(f'P: {P}')
    
    Kp = 350
    Ki = 300
    Kd = 50
    # Kp, Ki, Kd = 100, 0, 0
    
    if switch['open loop']:  
        t, yout = control.step_response(P, TIME)
        p, z = control.pzmap(P)
        print(f'poles: {p}, zeros: {z}')
        # print(f'yout: {np.shape(yout)}')
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(t, yout, label='y (pos)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.title('open loop step response')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.plot(t, REFERENCE, label='ref (force)')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.grid()
        if save == True:
            plt.savefig('plots/open_loop_step_response.png')
        plt.show()

    if switch['pid_discrete']:
        mypid = pid.PID(Kp, Ki, Kd)
        for i in range(len(TIME)):
            t = TIME[i]
            r = REFERENCE[i]
            u = mypid.controller_discrete(t, r, mymodel.y_curr)
            y = mymodel.plant_discrete(t, u)
        mypid.graph(save)
        mymodel.graph(save)
    
    if switch['pid']:
        mypid = pid.PID(Kp, Ki, Kd)
        C = mypid.controller()
        print(f'C: {C}')
        L = control.series(C, P)
        print(f'L: {L}')
        H = control.feedback(L, 1)
        print(f'H: {H}')
        
        t, yout, xout = control.forced_response(H, TIME, REFERENCE, return_x=True)
        # print(f'yout: {np.shape(yout)}')
        # print(f'force: {np.shape(xout)}')
        p, z = control.pzmap(H)
        print(f'poles: {p}, zeros: {z}')
        plt.figure()
        plt.plot(t, yout, label='y (pos)')
        for i in range(len(xout)):
            plt.plot(t, xout[i, :], label=f'x{i+1}')
        plt.legend()
        plt.ylabel('amplitude')
        plt.xlabel('time')
        plt.title('pid response')
        plt.grid()
        if save == True:
            plt.savefig('plots/pid_response.png')
        plt.show()

    if switch['fstb']:
        pass
        