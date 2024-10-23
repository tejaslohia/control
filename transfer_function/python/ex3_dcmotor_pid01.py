import numpy as np 
import control as ct 
from control.matlab import *
#import slycot 
import matplotlib.pyplot as plt

print("Welcome to Control Theory")

def pid_tf(Kp=0, Ki=0, Kd=0):
    # PID transfer function: Kp + Ki/s + Kd*s
    s = ct.TransferFunction.s
    
    return Kp + Ki / s + Kd * s

J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K**2);

Kp = 100;
pid = pid_tf(Kp);
print(pid)

openloop_tf= ct.series(P_motor,pid)
closeloop_tf=ct.feedback(openloop_tf)

dt = 0.01
t = np.arange(0, 5, dt)  # Time from 0 to 10 seconds

#a=ct.step_info(sys)
#print(a['Peak'])

#Getting Step Response
t,yout=ct.step_response(closeloop_tf,t)

a=ct.step_info(closeloop_tf)
print(a)


#Ploting Step Response
plt.plot(t,yout)
plt.grid()
plt.show()
