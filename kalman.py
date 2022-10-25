import numpy as np
import matplotlib.pyplot as plt
import control

m = 10
b = 3
k = 5
F = 1

A = np.array([[   0,    1],
              [-k/m, -b/m]])
B = np.array([[  0],
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])

time = np.arange(0, 10, 0.1)
u = F*np.ones(np.shape(time))

v = 0.1
w = 0.1
P = np.array([[1, 0],
              [0, 1]])
Q = np.array([[w**2, 0],
              [0, w**2]])
R = np.array([[v**2]])

x = np.array([[1],
              [0]])

sys = control.ss(A, B, C, D)
tout, yout = control.forced_response(sys, time, u, x)
noise = np.random.normal(0, v, np.shape(time))
youtnoise = yout + noise  # 1x100

def predict(A, x, B, u, P, Q):
    x = A@x + B@u  # 2x1
    P = A@P@A.T + Q  # 2x2
    return x, P

def update(C, x, y, P, R):
    K = P@C.T@np.linalg.inv(C@P@C.T + R)  # 2x1
    x = x + K@(y - C@x)  # 2x1
    # P = (np.eye(2) - K@C)@P@(np.eye(2) - K@C).T + K@R@K.T  # 2x2
    P = (np.eye(2) - K@C)@P  # 2x2
    return x, P

xhat = np.array([[],
                 []])
for i in range(0, len(time)):
    ui = np.array([[u[i]]])  # input
    yi = np.array([[youtnoise[i]]])  # measurement
    x, P = predict(A, x, B, ui, P, Q)
    x, P = update(C, x, yi, P, R)
    xhat = np.append(xhat, x, axis=1)

# xhat = xhat[:, 1:]  # delete first temp entry 2x100
print(f'xhat: {np.shape(xhat)}')

plt.figure(1)
plt.plot(time, u, label='r')
plt.plot(time, yout, label='y')
plt.plot(time, youtnoise, label='y+noise')
plt.plot(time, xhat[0,:], label='x')
plt.plot(time, xhat[1,:], label='v')
plt.legend()
plt.show()
