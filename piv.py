import numpy as np
import matplotlib.pyplot as plt

# tuning constants
m = 10
b = 10
k = 20
F = 1

# control constants
kp = 0
ki = 1000
kd = 0
kv = 100
# pid 350 300 50
# piv 10 1 10(ref omega=0)
# piv 0 1000 100(kp=0, ref omega = 4)
ref_omega = 4

# reference
dt = 0.01
time = np.arange(0, 10, dt)
ref = F*np.ones(np.shape(time))

def pid_d(idx, dt, ref, mea):
    global r_arr, e_arr, u_arr, prev_u, prev_error, prev2_error
    error = ref - mea
    u = (prev_u 
         + (kp+ki*dt+kd/dt)*error 
         + (-kp-2*kd/dt)*prev_error 
         + kd/dt*prev2_error)
    prev2_error = prev_error
    prev_error = error
    # prev_time = ti
    prev_u = u
    
    # save values
    r_arr[idx] = ref
    e_arr[idx] = error
    u_arr[idx] = u
    return u

def model_d(idx, dt, u):
    global x_arr, y_arr, k, b, m
    # model discrete mass spring damper
    # u = ref[i]  # check open loop
    u = np.array([[u]])
    A = np.array([[        1,           dt],
                  [-(dt*k)/m, 1 - (dt*b)/m]])
    B = np.array([[   0],
                  [dt/m]])
    C = np.array([[1, 0],
                  [0, 1]])
    D = np.array([[0],
                  [0]])
    x_arr[:, idx+1:idx+2] = A@x_arr[:, idx:idx+1] + B@u
    y_arr[:, idx:idx+1] = C@x_arr[:, idx:idx+1] + D@u
    return y_arr[0, idx:idx+1], y_arr[1, idx:idx+1]

def piv(idx, dt, ref_theta, ref_omega, mea_theta, mea_omega):
    global integral, kp, ki, kv, r_arr, e_arr, u_arr
    error = ref_theta - mea_theta
    u1 = kp*error + ref_omega - mea_omega
    integral = integral + u1*dt
    u2 = ki*integral - kv*mea_omega
    # save values
    r_arr[idx] = ref_theta
    e_arr[idx] = error
    u_arr[idx] = u2
    return u2

# initialize constants/arrays
global prev_time, prev_error, prev2_error, prev_u, integral
prev_time = -dt
prev_error = 0
prev2_error = 0
prev_u = 0
integral = 0
global r_arr, e_arr, u_arr, x_arr, y_arr
r_arr = np.zeros(len(time))
e_arr = np.zeros(len(time))
u_arr = np.zeros(len(time))
x_arr = np.zeros((2, len(time)))
y_arr = np.zeros((2, len(time)))

yx = 0
yv = 0
for i in range(0, len(time)):
    dt = time[i] - prev_time
    prev_time = time[i]
    # u = pid_d(i, dt, ref[i], yx)
    u = piv(i, dt, ref[i], ref_omega, yx, yv)
    yx, yv = model_d(i, dt, u)

# plotting
plt.figure(1)
plt.plot(time, r_arr, 'b', label='r')
plt.plot(time, e_arr, 'r', label='e')
# plt.plot(time, u_arr, 'g', label='u')
plt.plot(time, x_arr[0, :], label='x1')
plt.plot(time, x_arr[1, :], label='x2')
plt.plot(time, y_arr[0, :], '--', label='y1')
plt.plot(time, y_arr[1, :], '--', label='y2')
plt.legend()
plt.show()
