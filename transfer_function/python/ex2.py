import numpy as np 
import control as ct 
from control.matlab import *
#import slycot 
import matplotlib.pyplot as plt

print("Welcome to Control Theory")

num=np.array([1,2])
den=np.array([1,2,5])

k=5
num=num*k

dt = 0.1
t = np.arange(0, 10, dt)  # Time from 0 to 10 seconds

sys=ct.tf(num,den)
print(sys)
#a=ct.step_info(sys)
#print(a['Peak'])

#Getting Step Response
t,yout=ct.step_response(sys,t)

#Ploting Step Response
plt.plot(t,yout)
plt.grid()
plt.show()
