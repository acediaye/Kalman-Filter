import numpy as np
import matplotlib.pyplot as plt

m = 1
b = 10
k = 20
F = 1

A = np.array([[   0,    1],
              [-k/m, -b/m]])
B = np.array([[  0],
              [1/m]])
C = np.array([[1, 0]])
D = np.array([[0]])

time = np.arange(0, 10, 0.1)
u = F*np.ones(np.shape(time))
sigma = 0.1
y = u + np.random.normal(0, sigma, np.shape(time))

def predict(A, x, B, u, P, Q):
    # print(np.shape(A), np.shape(x), np.shape(B), np.shape(u), np.shape(P), np.shape(Q))
    x = A@x + B@u  # 2x1
    P = A@P@A.T + Q  # 2x2
    return x, P

def update(C, x, y, P, R):
    K = P@C.T@np.linalg.inv(C@P@C.T + R)  # 2x1
    x = x + K@(y - C@x)  # 2x1
    P = (np.eye(2) - K@C)@P@(np.eye(2) - K@C).T + K@R@K.T  # 2x2
    return x, P

x = np.array([[1], 
              [0]])
P = np.array([[0.1, 0],
              [0, 0.1]])  # covariance matrix
Q = np.array([[0.1, 0],
              [0, 0.1]])  # process noise matrix
R = np.array([[sigma]])  # measurement noise matrix
xhat = x
print(np.shape(xhat))
for i in range(0, len(time)):
    uu = np.array([[u[i]]])  # input
    # print(uu)
    x, P = predict(A, x, B, uu, P, Q)
    yy = np.array([[y[i]]])  # measurement
    x, P = update(C, x, yy, P, R)
    xhat = np.append(xhat, x, axis=1)

xhat = xhat[:,1:]  # delete first temp entry
print(np.shape(xhat))
print(np.shape(y))

plt.figure(1)
plt.plot(time, u, label='true')
plt.plot(time, y, label='noise')
plt.plot(time, xhat[0,:], label='x')
# plt.plot(time, xhat[1,:], label='v')
plt.legend()
plt.show()
