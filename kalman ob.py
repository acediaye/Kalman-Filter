import numpy as np
import matplotlib.pyplot as plt
import control

class mass_spring_damper(object):
    def __init__(self, M: float, K: float, C: float, v):
        self.m = M
        self.k = K
        self.c = C
        self.sys = None
        self.A = None
        self.B = None
        self.C = None
        self.D = None
        self.x0 = np.array([[1],
                            [0]])
        self.v = v
        self.time = None
        self.tout = None
        self.yout = None
        self.youtnoise = None
    
    def plant(self):
        self.A = np.array([[             0,              1],
                           [-self.k/self.m, -self.c/self.m]])
        self.B = np.array([[       0],
                           [1/self.m]])
        self.C = np.array([[1, 0]])
        self.D = np.array([[0]])
        self.sys = control.ss(self.A, self.B, self.C, self.D)
        return self.sys

    def excite(self, time: np.ndarray, u: np.ndarray) -> np.ndarray:
        self.time = time
        self.tout, self.yout = control.forced_response(self.sys, time, u, self.x0)
        noise = np.random.normal(0, self.v, np.shape(time))
        self.youtnoise = self.yout + noise  # 1x100
        return self.youtnoise
        
    def graph(self):
        plt.figure(1)
        plt.plot(self.time, self.yout, label='y')
        plt.plot(self.time, self.youtnoise, label='y+noise')
        plt.legend()
        
class kalman(object):
    def __init__(self, sys: control.StateSpace, w: float, v: float):
        self.sys = sys
        self.A, self.B, self.C, self.D = control.ssdata(self.sys)
        self.w = w
        self.v = v
        self.Q = np.array([[self.w**2,         0],
                           [        0, self.w**2]])
        self.R = np.array([[v**2]])
        self.x = np.array([[1],
                           [0]])
        self.P = np.array([[1, 0],
                           [0, 1]])
        self.time = None
        self.xhat = np.array([[],
                              []])
        print(self.x)
        
    def predict(self, u: float):
        self.x = self.A@self.x + self.B@u  # 2x1
        self.P = self.A@self.P@self.A.T + self.Q  # 2x2
        return self.x, self.P

    def update(self, y:float):
        K = self.P@self.C.T@np.linalg.inv(self.C@self.P@self.C.T + self.R)  # 2x1
        self.x = self.x + K@(y - self.C@self.x)  # 2x1
        self.P = (np.eye(2) - K@self.C)@self.P  # 2x2
        self.xhat = np.append(self.xhat, self.x, axis=1)
        return self.x, self.P
    
    def graph(self, time):
        plt.figure(1)
        plt.plot(time, self.xhat[0,:], label='x')
        plt.plot(time, self.xhat[1,:], label='v')
        plt.legend()
        
    def temp(self):
        print(control.ssdata(self.sys))
        print(self.A, self.B, self.C, self.D)
    

mymodel = mass_spring_damper(10, 5, 3, 0.5)
sys = mymodel.plant()
mykalman = kalman(sys, 0.01, 0.5)
mykalman.temp()

time = np.arange(0, 10, 0.1)
u = 1*np.ones(np.shape(time))
ynoise = mymodel.excite(time, u)
for i in range(0, len(time)):
    ui = np.array([[u[i]]])  # input
    yi = np.array([[ynoise[i]]])
    mykalman.predict(ui)
    mykalman.update(yi)
mymodel.graph()
mykalman.graph(time)
plt.show()
