import numpy as np
import matplotlib.pyplot as plt

# reference
dt = 0.001  # under sampling can cause instability 
time = np.arange(0, 1, dt)
control = 'piv'  # pid, piv
mode = 'pos'  # pos, vel
ref_theta = 1*np.ones(np.shape(time))
if mode == 'pos':
    ref_omega = 0
elif mode == 'vel':
    ref_omega = 4

# model constants
Kt = 0.6
b = 0.1e-3
J = 50e-6
# control constants
if control == 'piv':
    BW = 20
    zeta = 1
    if mode == 'pos':
        kp = 2*np.pi*BW/(2*zeta+1)
    elif mode == 'vel':
        kp = 0
    ki = J*(2*zeta+1)*(2*np.pi*BW)**2
    kv = J*(2*zeta+1)*(2*np.pi*BW) - b
elif control == 'pid':
    ko = 1e-2
    fo = 0.6
    # kp = 0.01
    # ki = 0
    # kd = 0
    kp = 0.6*ko
    ki = 2*fo*kp
    kd = kp/(8*fo)

def pid_d(idx, dt, ref, mea):
    global r_arr, e_arr, u_arr, prev_u, prev_error, prev2_error
    error = ref - mea
    u = (prev_u 
         + (kp+ki*dt+kd/dt)*error 
         + (-kp-2*kd/dt)*prev_error 
         + kd/dt*prev2_error)
    prev2_error = prev_error
    prev_error = error
    prev_u = u
    # save values
    r_arr[idx] = ref
    e_arr[idx] = error
    u_arr[idx] = u
    return u

def pid(idx, dt, ref, mea):
    global r_arr, e_arr, u_arr, integral, prev_error
    error = ref - mea
    integral = integral + error*dt
    u = kp*error + ki*integral + kd*(error-prev_error)/dt
    prev_error = error
    # save values
    r_arr[idx] = ref
    e_arr[idx] = error
    u_arr[idx] = u
    return u

def model_d(idx, dt, u):
    global x_arr, y_arr
    m = 10
    b = 10
    k = 20
    # model discrete mass spring damper
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
    global r_arr, e_arr, u_arr, integral
    error = ref_theta - mea_theta
    u1 = kp*error + ref_omega - mea_omega
    integral = integral + u1*dt
    u2 = ki*integral - kv*mea_omega
    # save values
    r_arr[idx] = ref_theta
    e_arr[idx] = error
    u_arr[idx] = u2
    return u2

def servo_d(idx, dt, u):
    global x_arr, y_arr
    u = np.array([[u]])
    A = np.array([[1,           dt],
                  [0, 1 - (dt*b)/J]])
    B = np.array([[      0],
                  [dt*Kt/J]])
    C = np.array([[1, 0],
                  [0, 1]])
    D = np.array([[0],
                  [0]])
    x_arr[:, idx+1:idx+2] = A@x_arr[:, idx:idx+1] + B@u
    y_arr[:, idx:idx+1] = C@x_arr[:, idx:idx+1] + D@u
    return y_arr[0, idx:idx+1], y_arr[1, idx:idx+1]

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

if __name__ == '__main__':
    yx = 0
    yv = 0
    for idx in range(0, len(time)):
        dt = time[idx] - prev_time
        prev_time = time[idx]

        if control == 'piv':
            u = piv(idx, dt, ref_theta[idx], ref_omega, yx, yv)
        elif control == 'pid':
            u = pid(idx, dt, ref_theta[idx], yx)
        else:
            u = ref_theta[idx]  # open loop
        # yx, yv = model_d(i, dt, u)
        yx, yv = servo_d(idx, dt, u)

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
