import numpy as np
import matplotlib.pyplot as plt
import control

# c = 4
# k = 2
# m = 20
# F = 5

# Ts = 0.1
# Tstart = 0
# Tstop = 60
# N = int((Tstop-Tstart)/Ts)  # 600
# print(N)
# x1 = np.zeros(N+1)  # 602
# x2 = np.zeros(N+1)

# a11 = 1
# a12 = Ts
# a21 = -(Ts*k)/m
# a22 = 1 - (Ts*c)/m

# b1 = 0
# b2 = Ts/m

# for k in range(N):  # for all index
#     x1[k+1] = a11*x1[k] + a12*x2[k] + b1*F
#     x2[k+1] = a21*x1[k] + a22*x2[k] + b2*F
#     print(f'loop: {k}')
    
# t = np.arange(Tstart, Tstop+1*Ts, Ts)
# print(len(t))
# print(x1)
# print(len(x1))

# ref = F*np.ones(len(t))
# plt.plot(t, ref, label='ref')
# plt.plot(t, x1, label='position')
# plt.plot(t, x2, label='velocity')
# plt.xlabel('time')
# plt.ylabel('x')
# plt.legend()
# plt.show()

#-------------------------------------------------------------

# c = 4
# k = 2
# m = 20
# # F = 5

# tstart = 0
# tstop = 60
# t = np.arange(tstart, tstop+1, 0.1)
# print(len(t))
# F = 5*np.ones(len(t))

# A = np.array([[0, 1], [-k/m, -c/m]])
# B = np.array([[0], [1/m]])
# C = np.array([[1, 0]])
# D = np.array([[0]])
# sys = control.ss(A, B, C, D)
# print(f'sys: {sys}')

# x0 = np.array([1, 0])
# t, y = control.forced_response(sys, t, F, x0)  # doesnt give x states out
# print(f't: {np.shape(t)}')
# print(f'y: {np.shape(y)}')
# # print(f'x: {np.shape(x)}')
# # x1 = x[0, :]
# # x2 = x[1, :]
# # plt.plot(t, x1, t, x2)
# plt.plot(t, y)
# plt.xlabel('time')
# plt.ylabel('X')
# plt.show()

# ------------------------------------------

# m = 10 # [ kg ]
# k = 4 # [ N / m ]
# d = 2 # [ N /( m / s )]
# # %% System matrices as 2 D arrays :
# A = np.array([[0 , 1] , [ - k /m , -d / m ]])
# B = np.array([[0] , [1/ m ]])
# C = np.array([[1 , 0]])
# D = np.array([[0]])
# # %% Creating the state space model :
# S = control.ss(A , B , C , D )
# # %% Defining signals :
# t0 = 0 # [ s ]
# t1 = 50 # [ s ]
# dt = 0.01 # [ s ]
# nt = int( t1 / dt ) + 1 # Number of points of sim time
# t = np.linspace( t0 , t1 , nt )
# F = 10*np.ones( nt ) # [ N ]
# # %% Initial state :
# x1_0 = 1 # [ m ]
# x2_0 = 0 # [ m / s ]
# x0 = np.array([ x1_0 , x2_0 ])

# (t , y , x ) = control.forced_response (S , t , F , x0 )  # not x states out
# # %% Extracting individual states :
# x1 = x[0 ,:]
# x2 = x[1 ,:]
# # %% Plotting :
# plt.figure(1 , figsize =(12 , 9))
# plt.subplot(3 , 1 , 1)
# plt.plot(t , x1 , 'b')
# plt.grid()
# plt.legend( labels =( 'x1 [ m ] ' ,))
# plt.subplot (3 , 1 , 2)
# plt.plot(t , x2 , 'g')
# plt.grid()
# plt.legend( labels =( 'x2 [ m / s ] ' ,))
# plt.subplot (3 , 1 , 3)
# plt.plot(t , F , 'r')
# plt.grid()
# plt.legend( labels =( ' F [ N ] ' ,))
# plt.xlabel( ' t [ s ] ')
# plt.show()

# -----------------------------------------------

# MIMO
# A = np.array([[0, 1], [-1, -3]])
# B = np.array([[0, 0], [2, 4]])
# C = np.array([[1, 0], [0, 1]])
# D = np.array([[0, 0], [0, 0]])
# ssmodel = control.ss(A, B, C, D)
# print(ssmodel)
# H = control.ss2tf(ssmodel)  # no mimo???
# print(H)
# H1 = H[0,0]
# print(H1)
# H2 = H[0,1]
# print(H2)
# H3 = H[1,0]
# print(H3)
# H4 = H[1,1]
# print(H4)

# # Step response for the system
# t, y = control.step_response(H1)
# plt.plot(t, y)
# t, y = control.step_response(H2)
# plt.plot(t, y)
# t, y = control.step_response(H3)
# plt.plot(t, y)
# t, y = control.step_response(H4)
# plt.plot(t, y)
# plt.title("Step Response H")
# plt.xlabel("t")
# plt.ylabel("y")
# plt.legend(["H1", "H2", "H3", "H4"])
# plt.grid()
# plt.show()

# ----------------------------------
# pid

# Model Parameters
# K = 3
# T = 4
# a = -(1/T)
# b = K/T
# # Simulation Parameters
# Ts = 0.1 # Sampling Time
# Tstop = 20 # End of Simulation Time
# N = int(Tstop/Ts) # Simulation length
# y = np.zeros(N+2) # Initialization the Tout vector
# y[0] = 0 # Initial Vaue
# # PI Controller Settings
# Kp = 0.5
# Ti = 5
# r = 5 # Reference value
# e = np.zeros(N+2) # Initialization
# u = np.zeros(N+2) # Initialization
# # Simulation
# for k in range(N+1):
#     e[k] = r - y[k]
#     u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + (Kp/Ti)*Ts*e[k]
#     y[k+1] = (1+Ts*a)*y[k] + Ts*b*u[k]
# # Plot the Simulation Results
# t = np.arange(0,Tstop+2*Ts,Ts) #Create the Time Series
# # Plot Process Value
# plt.figure(1)
# plt.plot(t,y)
# # Formatting the appearance of the Plot
# plt.title('Control of Dynamic System')
# plt.xlabel('t [s]')
# plt.ylabel('y')
# plt.grid()
# xmin = 0
# xmax = Tstop
# ymin = 0
# ymax = 8
# plt.axis([xmin, xmax, ymin, ymax])
# plt.show()
# # Plot Control Signal
# plt.figure(2)
# plt.plot(t,u)
# # Formatting the appearance of the Plot
# plt.title('Control Signal')
# plt.xlabel('t [s]')
# plt.ylabel('u [V]')
# plt.grid()
# plt.show()

# ----------------------------

# Parameters defining the system
# c = 4 # Damping constant
# k = 2 # Stiffness of the spring
# m = 20 # Mass
# F = 5 # Force
# # Simulation Parameters
# tstart = 0
# tstop = 60
# increment = 0.1
# t = np.arange(tstart,tstop+1,increment)
# # System matrices
# A = [[0, 1], [-k/m, -c/m]]
# B = [[0], [1/m]]
# C = [[1, 0]]
# sys = control.ss(A, B, C, 0)

# Kp = 20
# Ki = 0
# Kd = 0
# s = control.tf('s')
# CC = Kp + Ki/s + Kd*s
# print(CC)
# L = control.series(CC, sys)
# print(L)
# H = control.feedback(L, 1)
# print(H)

# # Step response for the system
# t, y, x = control.forced_response(H, t, F, return_x=True)
# x1 = x[0 ,:]
# x2 = x[1 ,:]
# plt.plot(t, x1, t, x2)
# plt.plot(t, y, '-.')
# plt.title('Simulation of Mass-Spring-Damper System')
# plt.xlabel('t')
# plt.ylabel('x(t)')
# plt.grid()
# plt.show()

# --------------------------------------------
# test PI 

# Parameters defining the system
c = 10 # Damping constant
k = 20 # Stiffness of the spring
m = 1 # Mass
F = 1 # Force
# Simulation Parameters
tstart = 0
tstop = 4
increment = 0.01
t = np.arange(tstart,tstop+1,increment)
# System matrices
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]
P = control.ss(A, B, C, 0)
P = control.ss2tf(P)
print(P)
# Step response for the system

Kp = 30
Ki = 70
s = control.tf('s')
C = Kp + Ki/s
print(C)

L = control.series(C, P)
H = control.feedback(L, 1)
print(H)
print(control.tf2ss(H))

t, y, x = control.forced_response(H, t, F, return_x=True)
print(np.shape(t), np.shape(y), np.shape(x))
x1 = x[0, :]
x2 = x[1, :]
x3 = x[2, :]
plt.plot(t, x1, '-.', label='x1')
plt.plot(t, x2, '--', label='x2')
plt.plot(t, x3, ':', label='x3')
plt.plot(t, y, label='pos')
plt.legend()
plt.title('Simulation of Mass-Spring-Damper System')
plt.xlabel('t')
plt.ylabel('x(t)')
plt.grid()
plt.show()