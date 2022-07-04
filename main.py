import numpy as np
import model
import pid
import fsfb
import lqr
import fsob
import lqe

# turn parts of main on or off
switch = {'open loop': False, 
          'pid_discrete': False,
          'pid': False,
          'fsfb': False,
          'lqr': False,
          'fsob': False,
          'lqe': True}
# save plots
save = False

if __name__ == '__main__':
    TIME_STEP = 0.003
    TIME = np.arange(0+TIME_STEP, 3+TIME_STEP, TIME_STEP)  # used t-t_prev. cannot start at 0 or else divide by 0-0
    print(f'time: {len(TIME)}')
    REFERENCE = 1*np.ones(len(TIME))
    # REFERENCE = 1*np.append(np.ones(len(TIME)//2), np.zeros(len(TIME)//2))
    # REFERENCE = 1*np.sin(10*TIME)
    
    mymodel = model.Mass_Spring_Damper_System(1, 20, 10)  # 1 or 10 mass
    ss_plant = mymodel.plant()
    
    Kp = 350
    Ki = 300
    Kd = 50
    # Kp, Ki, Kd = 100, 0, 0
    
    if switch['open loop']:  
        tout, yout = mymodel.excite(TIME, REFERENCE)
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
        tout, yout, xout = mypid.excite(ss_plant, TIME, REFERENCE)
        mypid.graph(save)
        mypid.pzmap()
        
    if switch['fsfb']:
        p_desire = np.array([-5+2j, -5-2j])  # no oscillation
        p_desire2 = np.array([5+2j, 5-2j])  # unstable
        p_desire3 = np.array([-2+5j, -2-5j])  # oscillation
        myfsfb = fsfb.FSFB(p_desire)
        tout, yout, xout = myfsfb.excite(ss_plant, TIME, REFERENCE)
        myfsfb.graph(save)
        myfsfb.pzmap()
        
    if switch['lqr']:
        Q = np.array([[1, 0], [0, 1]])  # 2x2
        R = np.array([[1]])  # 1x1
        mylqr = lqr.LQR(Q, R)
        tout, yout, xout = mylqr.excite(ss_plant, TIME, REFERENCE)
        mylqr.graph(save)
        mylqr.pzmap()

    if switch['fsob']:
        p_desire = np.array([-5+2j, -5-2j])  # no oscillation
        p_desire2 = np.array([-0.1+2j, -0.1-2j])  # no oscillation
        myfsob = fsob.FSOB(p_desire)
        tout, xhat = myfsob.excite(ss_plant, TIME, REFERENCE)
        myfsob.graph(save)
        myfsob.pzmap()
    
    if switch['lqe']:
        Q = np.array([[1, 0], [0, 1]])  # 2x2
        R = np.array([[1]])  # 1x1
        mylqe = lqe.LQE(Q, R)
        tout, xhat = mylqe.excite(ss_plant, TIME, REFERENCE)
        mylqe.graph(save)
        mylqe.pzmap()
        