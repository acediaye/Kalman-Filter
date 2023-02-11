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
w = 0  # std dev process noise
Q = np.array([[w**2, 0],
              [0, w**2]])  # covariance process noise
v = 1 - w  # std dev measurement noise
R = np.array([[v**2]])  # covariance measurement noise

# simulate response data with noise
x0 = np.array([[0],
              [0]])  # initial state
tout, yout = control.forced_response(sys, time, ref, x0)
noise = np.random.normal(0, 0.1, np.shape(time))  # assume guassian noise: mu, sigma
youtn = yout + noise  # 1x100

def predict(A, x, B, u, P, Q, dt):
    u = np.array([[u]])
    F = np.eye(len(A)) + dt*A
    x = F@x + dt*B@u
    P = F@P@F.T + Q  # 2x2
    return x, P

def update(C, x, y, P, R):
    y = np.array([[y]])
    K = P@C.T@np.linalg.inv(C@P@C.T + R)  # 2x1
    x = x + K@(y - C@x)  # 2x1
    P = P - K@C@P  # 2x2
    return x, P

mode = 'array'  # function # array
if __name__ == '__main__':
    if mode == 'function':
        # x estimate
        xhat = np.zeros((2, len(time)))
        for i in range(0, len(time)):
            ui = ref[i]  # reference
            yi = youtn[i]  # measurement + noise
            x0, P0 = predict(A, x0, B, ui, P0, Q, dt)
            x0, P0 = update(C, x0, yi, P0, R)
            # save values
            xhat[:,i:i+1] = x0
        print(f'xhat: {np.shape(xhat)}')
        
        # plotting
        plt.figure(1)
        plt.plot(time, yout, label='y')
        plt.plot(time, youtn, label='y+noise')
        plt.plot(time, xhat[0,:], linewidth=2, label='xhat')
        plt.plot(time, xhat[1,:], linewidth=2, label='vhat')
        plt.title('function method')
        plt.legend()
        plt.show()

    elif mode == 'array':
        x = np.zeros((2, len(time)))
        x[:,[0]] = x0
        xdot = np.zeros((2, len(time)))
        P = np.zeros((len(time),2,2))  # many 2x2 array
        P[[0],:,:] = P0
        Pdot = np.zeros((len(time),2,2))
        for i in range(0, len(time)-1):
            # print(np.shape(xdot[:,i:i+1]), np.shape(x[:,i:i+1]), np.shape(ref[[i]]))
            xdot[:,[i]] = A@x[:,[i]] + B@ref[i].reshape(1,1)
            Pdot[[i],:,:] = A@P[[i],:,:]@A.T + Q
            K = P[[i],:,:]@C.T@np.linalg.pinv(C@P[[i],:,:]@C.T + R)
            x[:,[i]] = x[:,[i]] + K@(youtn[i].reshape(1,1) - C@x[:,[i]])
            P[[i],:,:] = P[[i],:,:] - K@C@P[[i],:,:]
            # integrate
            x[:,[i+1]] = x[:,[i]] + xdot[:,[i]]*dt
            P[[i+1],:,:] = P[[i],:,:] + Pdot[[i],:,:]*dt
        # print(np.shape(x), np.shape(xdot), np.shape(P), np.shape(Pdot))
        
        # plotting
        plt.figure(1)
        plt.plot(tout, yout, label='y')
        plt.plot(tout, youtn, label='y+noise')
        plt.plot(time, x[0,:], label='x1')
        plt.plot(time, x[1,:], label='x2')
        plt.title('array method')
        plt.legend()
        plt.show()
