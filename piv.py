import numpy as np
import matplotlib.pyplot as plt
import control

m = 1
b = 10
k = 20
F = 1
kp = 350 #350
ki = 300 #300
kd = 50 #50

# A = np.array([[   0,    1],
#               [-k/m, -b/m]])
# B = np.array([[  0],
#               [1/m]])
# C = np.array([[1, 0]])
# D = np.array([[0]])

dt = 0.01
time = np.arange(0, 3, dt)
print(len(time))
ref = F*np.ones(np.shape(time))

prev_error = 0
prev_time = -dt
r_arr = []
e_arr = []
u_arr = []
x1_curr = 0
x2_curr = 0
y1_curr = 0
y2_curr = 0
prev_u = 0
prev2_error = 0
x1_arr = []
x2_arr = []
y1_arr = []
y2_arr = []

for i in range(0, len(time)):
    ri = ref[i]
    ti = time[i]
    
    dt = ti - prev_time
    error = ri - y1_curr
    u = (prev_u 
         + (kp+ki*dt+kd/dt)*error 
         + (-kp-2*kd/dt)*prev_error 
         + kd/dt*prev2_error)
    prev2_error = prev_error
    prev_error = error
    prev_time = ti
    prev_u = u
   
    # u = ri
    a11, a12 =         1,           dt
    a21, a22 = -(dt*k)/m, 1 - (dt*b)/m
    b1 = 0
    b2 = dt/m
    c11, c12 = 1, 0
    c21, c22 = 0, 1
    d1 = 0
    d2 = 0
    x1_next = a11*x1_curr + a12*x2_curr + b1*u
    x2_next = a21*x1_curr + a22*x2_curr + b2*u
    y1_curr = c11*x1_curr + c12*x2_curr + d1*u
    y2_curr = c21*x1_curr + c22*x2_curr + d2*u
    x1_curr = x1_next
    x2_curr = x2_next
    
    # print(np.shape(x1_next), np.shape(x2_next), np.shape(y_curr))
    x1_arr= np.append(x1_arr, x1_curr)
    x2_arr= np.append(x2_arr, x2_curr)
    y1_arr = np.append(y1_arr, y1_curr)
    y2_arr = np.append(y2_arr, y2_curr)
    
    r_arr = np.append(r_arr, ri)
    e_arr = np.append(e_arr, error)
    u_arr = np.append(u_arr, u)

plt.figure(1)
plt.plot(time, r_arr, 'b', label='r')
plt.plot(time, e_arr, 'r', label='e')
# plt.plot(time, u_arr, 'y', label='u')
plt.plot(time, x1_arr, label='x1')
plt.plot(time, x2_arr, label='x2')
plt.plot(time, y1_arr, '--', label='y1')
plt.plot(time, y2_arr, '--', label='y2')
plt.legend()
plt.show()
