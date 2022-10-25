import numpy as np
import matplotlib.pyplot as plt
import control

# tuning constants
m = 10
b = 3
k = 5
F = 1

# state space
A = np.array([[   0,    1],
              [-k/m, -b/m]])
B = np.array([[  0],
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])

# reference
time = np.arange(0, 10, 0.1)
ref = F*np.ones(np.shape(time))

# tuning matrix
P = np.array([[1, 0],
              [0, 1]])  # covariance matrix
w = 0.1  # std dev process noise
Q = np.array([[w**2, 0],
              [0, w**2]])  # covariance process noise
v = 0.1  # std dev measurement noise
R = np.array([[v**2]])  # covariance measurement noise
x = np.array([[1],
              [0]])  # initial state

# simulate response data
sys = control.ss(A, B, C, D)
tout, yout = control.forced_response(sys, time, ref, x)
noise = np.random.normal(0, v, np.shape(time))  # assume guassian noise
youtnoise = yout + noise  # 1x100

def predict(A, x, B, u, P, Q):
    x = A@x + B@u  # 2x1
    P = A@P@A.T + Q  # 2x2
    return x, P

def update(C, x, y, P, R):
    K = P@C.T@np.linalg.inv(C@P@C.T + R)  # 2x1
    x = x + K@(y - C@x)  # 2x1
    P = (np.eye(len(P)) - K@C)@P  # 2x2
    return x, P

# x estimate
xhat = np.array([[],
                 []])
for i in range(0, len(time)):
    # single values
    ui = np.array([[ref[i]]])  # reference
    yi = np.array([[youtnoise[i]]])  # measurement + noise
    x, P = predict(A, x, B, ui, P, Q)
    x, P = update(C, x, yi, P, R)
    # save values
    xhat = np.append(xhat, x, axis=1)
print(f'xhat: {np.shape(xhat)}')

# plotting
plt.figure(1)
plt.plot(time, ref, label='r')
plt.plot(time, yout, label='y')
plt.plot(time, youtnoise, label='y+noise')
plt.plot(time, xhat[0,:], label='x')
plt.plot(time, xhat[1,:], label='v')
plt.legend()
plt.show()
