# This example shows Root Locus of transfer function

# Import Library
import numpy as np 
import control as clt 
from control.matlab import *
import matplotlib.pyplot as plt

#Transfer Function
s = clt.tf('s');
sys_tf = (s + 7)/(s*(s + 5)*(s + 15)*(s + 20));

#gain array and root locus
gains=np.linspace(1,5000,5000)
r,g=clt.root_locus(sys_tf,kvect=gains,xlim=(-25, 2), ylim=(-15, 15))

#Plot Root Locus
plt.title('Root Locus of the Second Order System')
plt.xlabel('Real')
plt.ylabel('Imaginary')
plt.grid()
plt.show()


