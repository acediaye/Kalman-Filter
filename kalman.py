import numpy as np
import matplotlib.pyplot as plt
import control

# tuning constants
m = 10
b = 10
k = 20
F = 10

# state space
A = np.array([[   0,    1],
              [-k/m, -b/m]])
B = np.array([[  0],
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])
sys = control.ss(A, B, C, D)

# reference
dt = 0.01
time = np.arange(0, 10, dt)
print(f'time: {np.shape(time)}')
ref = F*np.ones(np.shape(time))

# tuning matrix
P0 = np.array([[1, 0],
               [0, 1]])  # covariance matrix
# 0 == good, 1 == bad
w = 0.2  # std dev process noise
Q = np.array([[w**2, 0],
              [0, w**2]])  # covariance process noise
v = 1 - w  # std dev measurement noise
R = np.array([[v**2]])  # covariance measurement noise
x0 = np.array([[0],
               [0]])  # initial state

# simulate response data with noise
tout, yout = control.forced_response(sys, time, ref, x0)
noise = np.random.normal(0, 0.1, np.shape(time))  # assume guassian noise: mu, sigma
youtn = yout + noise  # 1x100

# saving array
x = np.zeros((2, len(time)))  # 2xn
xdot = np.zeros((2, len(time)))
P = np.zeros((len(time),2,2))  # n 2x2 array
Pdot = np.zeros((len(time),2,2))

# init
x[:,[0]] = x0
P[[0],:,:] = P0
for i in range(0, len(time)-1):
    xdot[:,[i]] = A@x[:,[i]] + B@ref[i].reshape(1,1)
    P[[i],:,:] = A@P[[i],:,:]@A.T + Q
    K = P[[i],:,:]@C.T@np.linalg.pinv(C@P[[i],:,:]@C.T + R)
    x[:,[i]] = x[:,[i]] + K@(youtn[i].reshape(1,1) - C@x[:,[i]])
    P[[i],:,:] = P[[i],:,:] - K@C@P[[i],:,:]
    # integrate
    x[:,[i+1]] = x[:,[i]] + xdot[:,[i]]*dt

# plotting
plt.figure(1)
plt.plot(tout, yout, label='y')
plt.plot(tout, youtn, label='y+noise')
plt.plot(time, x[0,:], label='x1')
plt.plot(time, x[1,:], label='x2')
plt.title('kalman filter')
plt.xlabel('time')
plt.ylabel('magnitude')
plt.legend()
plt.show()
