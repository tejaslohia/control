import numpy as np 
import control as ct 
from control.matlab import *
#import slycot 
import matplotlib.pyplot as plt


print("Welcome to Control Theory")

num=np.array([1,2])
den=np.array([1,2,5])

sys=ct.tf(num,den)

print(sys)

a=ct.step_info(sys)
print(a['Peak'])

t,yout=ct.step_response(sys)


plt.plot(t,yout)
plt.grid()
plt.show()
