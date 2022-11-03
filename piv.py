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
# piv 100 10 10(ref vel=0)
# piv 0 1000 100(kp=0)
ref_omega = 4

# reference
dt = 0.01
time = np.arange(0, 10, dt)
ref = F*np.ones(np.shape(time))

# initialize constants/arrays
prev_time = -dt
prev_error = 0
prev2_error = 0
prev_u = 0
integral = 0
# global r_arr, e_arr, u_arr
r_arr = np.zeros(len(time))
e_arr = np.zeros(len(time))
u_arr = np.zeros(len(time))
x_arr = np.zeros((2, len(time)))
y_arr = np.zeros((2, len(time)))

for i in range(0, len(time)):
    # # pid discrete
    # dt = time[i] - prev_time
    # prev_time = time[i]
    # error = ref[i] - y_arr[0, i]
    # u = (prev_u 
    #      + (kp+ki*dt+kd/dt)*error 
    #      + (-kp-2*kd/dt)*prev_error 
    #      + kd/dt*prev2_error)
    # prev2_error = prev_error
    # prev_error = error
    # prev_u = u

    # piv
    dt = time[i] - prev_time
    prev_time = time[i]
    error = ref[i] - y_arr[0, i]
    u1 = kp*error + ref_omega - y_arr[1, i]
    integral = integral + u1*dt
    u2 = ki*integral - kv*y_arr[1, i]
    u = u2
    
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
    x_arr[:, i+1:i+2] = A@x_arr[:, i:i+1] + B@u
    y_arr[:, i+1:i+2] = C@x_arr[:, i:i+1] + D@u
    
    # save values
    r_arr[i] = ref[i]
    e_arr[i] = error
    u_arr[i] = u

# plotting
# print(np.shape(y_arr), np.shape(y_arr[:, 0]), np.shape(y_arr[0, 0]))
# print(np.shape(y_arr[0,:]), y_arr[:, 0], y_arr[0, 0])
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

# def pid_d(idx, dt, ref, mea):
#     global r_arr, e_arr, u_arr
#     error = ref - mea
#     # print(ref[idx], y_arr[0, idx])
#     u = (prev_u 
#          + (kp+ki*dt+kd/dt)*error 
#          + (-kp-2*kd/dt)*prev_error 
#          + kd/dt*prev2_error)
#     prev2_error = prev_error
#     prev_error = error
#     # prev_time = ti
#     prev_u = u
#     r_arr[idx] = ref
#     e_arr[idx] = error
#     u_arr[idx] = u
#     return u
