import numpy as np
import matplotlib.pyplot as plt
import control

m = 20
b = 4
k = 2
F = 5
# kp = 10 #350
# ki = 5 #300
# kd = 1 #50

# A = np.array([[   0,    1],
#               [-k/m, -b/m]])
# B = np.array([[  0],
#               [1/m]])
# C = np.array([[1, 0]])
# D = np.array([[0]])

time = np.arange(0, 60, 0.1)
# print(len(time))
ref = F*np.ones(np.shape(time))

# x = np.array([[0],
#               [0]])
prev_error = 0
prev_time = -0.1
integral = 0
y = 0
r_arr = []
e_arr = []
u_arr = []
y_arr = []
x1_curr = 0
x2_curr = 0
y_curr = 0
prev_u = 0
prev2_error = 0
x1_arr = []
x2_arr = []
for i in range(0, len(time)):
    ri = ref[i]
    ti = time[i]
    
    # dt = ti - prev_time
    # error = ri - y
    # proportional = error
    # integral = integral + error*dt
    # derivative = (error-prev_error)/dt
    # u = np.array([[kp*proportional + ki*integral + kd*derivative]])
    # prev_error = error
    # prev_time = ti
    
    dt = ti - prev_time
    # error = ri - y
    # u = (prev_u 
    #      + (kp+ki*dt+kd/dt)*error 
    #      + (-kp-2*kd/dt)*prev_error 
    #      + kd/dt*prev2_error)
    # prev2_error = prev_error
    # prev_error = error
    # prev_time = ti
    # prev_u = u
    # u = np.array([[u]])
    
    # print(np.shape(A), np.shape(x), np.shape(B), np.shape(u))
    # print(np.shape(C), np.shape(x), np.shape(D), np.shape(u))
    # u = np.array([[ri]])
    # print(x)
    # x = A@x + B@u
    # y = C@x + D@u
    
    u = ri
    a11 = 1
    a12 = dt
    a21 = -(dt*k)/m
    a22 = 1 - (dt*b)/m
    b1 = 0
    b2 = dt/m
    c1 = 1
    c2 = 0
    d = 0
    x1_next = a11*x1_curr + a12*x2_curr + b1*u
    x2_next = a21*x1_curr + a22*x2_curr + b2*u
    y_curr = c1*x1_curr + c2*x2_curr + d*u
    x1_curr = x1_next
    x2_curr = x2_next
    y = y_curr
    # print(np.shape(x1_next), np.shape(x2_next), np.shape(y_curr))
    x1_arr= np.append(x1_arr, x1_curr)
    x2_arr= np.append(x2_arr, x2_curr)
    
    r_arr = np.append(r_arr, ri)
    e_arr = np.append(e_arr, error)
    u_arr = np.append(u_arr, u)
    y_arr = np.append(y_arr, y)
    # print(np.shape(x), np.shape(y))
# print(np.shape(y_arr))

plt.figure(1)
# plt.plot(time, r_arr, 'b', label='r')
# plt.plot(time, e_arr, 'r', label='e')
# plt.plot(time, u_arr, 'y', label='u')
# plt.plot(time, y_arr, 'c--', label='y')
plt.plot(time, x1_arr, label='x1')
plt.plot(time, x2_arr, label='x2')
plt.legend()
plt.show()
