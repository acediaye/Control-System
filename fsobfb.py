import numpy as np
import matplotlib.pyplot as plt
import control

# model constants
m = 10
b = 10
k = 20
F = 1

# reference
time = np.arange(0, 10, 0.01)
ref = 1*np.ones(np.shape(time))

# state space
A = np.array([[   0,    1], 
              [-k/m, -b/m]])
B = np.array([[0], 
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])
sys = control.ss(A, B, C, D)
print(f'sizes: A:{np.shape(A)} B:{np.shape(B)}, C:{np.shape(C)}, D:{np.shape(D)}')

# ol, fb, ob, obfb
mode = 'obfb'
if __name__ == '__main__':
    # open loop response
    x0 = np.array([[1],
                    [-1]])
    tout, yout = control.forced_response(sys, time, ref, x0)
    youtnoise = yout+np.random.normal(0, 0.1, np.shape(time))  # mu, sigma, shape
    if mode == 'ol':
        # check poles
        p, z = control.pzmap(sys, plot=False)
        print(f'ol poles: {p}')
        # plotting
        plt.plot(tout, youtnoise, label='yout_noise')
        plt.plot(tout, yout, label='yout')
        plt.title('open loop response')
        plt.legend()
        plt.show()

    elif mode == 'fb':
        # check controlability
        ctrb = control.ctrb(A, B)
        rank_c = np.linalg.matrix_rank(ctrb)
        print(f'ctrb rank: {rank_c}')
        # choosing and placing poles
        poles_c = [-5+2j, -5-2j]
        K = control.place(A, B, poles_c)
        print(f'K: {K}')
        # full state feedback
        A_fb = A-B@K
        # find reference gain
        sys_fb = control.ss(A_fb, B, C, D)
        dc = control.dcgain(sys_fb)
        K_r = np.array([[1/dc]])
        print(f'ref gain: {K_r}')
        B_fb = B*K_r
        sys_fb = control.ss(A_fb, B_fb, C, D)
        print(f'sizes: A_fb:{np.shape(A_fb)} B_fb:{np.shape(B_fb)}, C:{np.shape(C)}, D:{np.shape(D)}')
        x0 = np.array([[1],
                       [-1]])
        tout, yout_fb = control.forced_response(sys_fb, time, ref, x0)
        # check poles
        p, z = control.pzmap(sys_fb, plot=False)
        print(f'fb poles: {p}')
        # plotting
        plt.plot(tout, yout_fb, label='yout_fb')
        plt.title('full state feedback')
        plt.legend()
        
        plt.show()
        
    if mode == 'ob':
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        print(f'obsb rank: {rank_o}')
        # choosing and placing poles
        poles_o = [-6+2j, -6-2j]
        L = control.place(A.T, C.T, poles_o).T
        print(f'L: {L}')
        # full state observer
        A_ob = A-L@C
        B_ob = np.bmat([B, L])
        C_ob = np.eye(len(A))
        D_ob = np.array(np.zeros((np.shape(C_ob)[0], np.shape(B_ob)[1])))
        sys_ob = control.ss(A_ob, B_ob, C_ob, D_ob)
        print(f'sizes: A_ob:{np.shape(A_ob)} B_ob:{np.shape(B_ob)}, C_ob:{np.shape(C_ob)}, D_ob:{np.shape(D_ob)}')
        u_ob = np.bmat([[np.array([[ref]])],
                        [np.array([[yout]])]])
        x0 = np.array([[1], 
                       [-1]])
        tout, xhat = control.forced_response(sys_ob, time, u_ob, x0)
        # check poles
        p, z = control.pzmap(sys_ob, plot=False)
        print(f'ob poles: {p}')
        # plotting
        plt.plot(tout, yout, label='yout')
        plt.plot(tout, xhat[0,:], '--', label='xhat1')
        plt.plot(tout, xhat[1,:], '--', label='xhat2')
        plt.title('full state observer')
        plt.legend()
        plt.show()

    if mode == 'obfb':
        # check observability
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        print(f'obsv rank: {rank_o}')
        # choosing and placing poles
        poles_c = [-5+2j, -5-2j]
        K = control.place(A, B, poles_c)
        print(f'K: {K}')
        # choosing and placing poles
        poles_o = [-6+2j, -6-2j]
        L = control.place(A.T, C.T, poles_o).T
        print(f'L: {L}')
        # full state observer feedback
        A_ce = np.bmat([[A-B@K, B@K],
                        [np.zeros(np.shape(A)), A-L@C]])
        B_temp = np.bmat([[B]],
                         [[np.zeros(np.shape(B))]])
        C_ce = np.bmat([[C, np.zeros(np.shape(C))]])
        D_ce = np.zeros(np.shape(D))
        print(np.shape(A_ce), np.shape(B_temp), np.shape(B))
        # find reference gain
        sys_temp = control.ss(A_ce, B_temp, C_ce, D_ce)
        dc = control.dcgain(sys_temp)
        K_r = np.array([[1/dc]])
        print(f'ref gain: {K_r}')
        B_ce = np.bmat([[B*K_r],
                        [np.zeros(np.shape(B))]])  # <-- fix not correct shape
        print(f'sizes: A_ce:{np.shape(A_ce)} B_ce:{np.shape(B_ce)}, C_ce:{np.shape(C_ce)}, D_ce:{np.shape(D_ce)}')
        sys_ce = control.ss(A_ce, B_ce, C_ce, D_ce)
        x0 = np.array([[1],
                       [-1],
                       [1],
                       [-1]])
        tout, yout_ce, xout_ce = control.forced_response(sys_ce, time, ref, x0, return_x=True)
        # check poles
        p, z = control.pzmap(sys_ce, plot=False)
        print(f'ce poles: {p}')
        # plotting
        plt.plot(tout, yout_ce, label='yout_ce')
        for i in range(np.shape(xout_ce)[0]):
            plt.plot(tout, xout_ce[i,:], '--', label=f'xhat{i+1}')
        plt.legend()
        plt.title('full state observer feedback')
        plt.show()
