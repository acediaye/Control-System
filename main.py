import numpy as np
# import matplotlib.pyplot as plt
# import control
import pid
import model
import fstb

# turn parts of main on or off
switch = {'open loop': False, 
          'pid_discrete': False,
          'pid': False,
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
    P = mymodel.plant()
    
    Kp = 350
    Ki = 300
    Kd = 50
    # Kp, Ki, Kd = 100, 0, 0
    
    if switch['open loop']:  
        t, y = mymodel.excite(TIME, REFERENCE)
        mymodel.graph(save)
        mymodel.pzmap()

    if switch['pid_discrete']:
        mypid = pid.PID(Kp, Ki, Kd)
        for i in range(len(TIME)):
            t = TIME[i]
            r = REFERENCE[i]
            u = mypid.controller_discrete(t, r, mymodel.y_curr)
            y = mymodel.plant_discrete(t, u)
        mypid.graph_discrete(save)
        mymodel.graph_discrete(save)
    
    if switch['pid']:
        mypid = pid.PID(Kp, Ki, Kd)
        C = mypid.controller()
        tout, yout, xout = mypid.excite(P, TIME, REFERENCE)
        mypid.graph(save)
        mypid.pzmap()
        
    if switch['fstb']:
        p_desire = np.array([-5 + 2j, -5-2j])
        myfstb = fstb.FSTB(p_desire)
        myfstb.excite(P, TIME, REFERENCE)