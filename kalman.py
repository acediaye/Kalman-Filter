import numpy as np
import matplotlib.pyplot as plt
import control

# tuning constants
m = 10
b = 10
k = 20
F = 5

# state space
A = np.array([[   0,    1],
              [-k/m, -b/m]])
B = np.array([[  0],
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])

# reference
dt = 0.1
time = np.arange(0, 10, dt)
print(np.shape(time))
ref = F*np.ones(np.shape(time))

# tuning matrix
P = np.array([[1, 0],
              [0, 1]])  # covariance matrix
w = 0.5  # std dev process noise
Q = np.array([[w**2, 0],
              [0, w**2]])  # covariance process noise
v = 0.5  # std dev measurement noise
R = np.array([[v**2]])  # covariance measurement noise
x = np.array([[0],
              [0]])  # initial state

# simulate response data
sys = control.ss(A, B, C, D)
tout, yout = control.forced_response(sys, time, ref, x)
noise = np.random.normal(0, 0.1, np.shape(time))  # assume guassian noise, mu, sigma
youtnoise = yout + noise  # 1x100

def predict(A, x, B, u, P, Q, dt):
    # x = x + dt*(A@x + B@u)  # 2x1
    # P = P + dt*(A@P + P@A.T + Q)  # 2x2
    F = (np.eye(len(A)) + dt*A)
    x = F@x + dt*B@u
    P = F@P@F.T + Q*dt  # 2x2
    return x, P

def update(C, x, y, P, R):
    K = P@C.T@np.linalg.inv(C@P@C.T + R)  # 2x1
    x = x + K@(y - C@x)  # 2x1
    P = P - K@C@P  # 2x2
    return x, P

# x estimate
xhat = np.array([[],
                 []])
for i in range(0, len(time)):
    # single values
    ui = np.array([[ref[i]]])  # reference
    yi = np.array([[youtnoise[i]]])  # measurement + noise
    x, P = predict(A, x, B, ui, P, Q, dt)
    x, P = update(C, x, yi, P, R)
    # save values
    xhat = np.append(xhat, x, axis=1)
print(f'xhat: {np.shape(xhat)}')
print(f'P: {P}')

# plotting
plt.figure(1)
# plt.plot(time, ref, label='r')
plt.plot(time, yout, label='y')
plt.plot(time, youtnoise, label='y+noise')
plt.plot(time, xhat[0,:], label='x')
# plt.plot(time, xhat[1,:], label='v')
plt.legend()
plt.show()
