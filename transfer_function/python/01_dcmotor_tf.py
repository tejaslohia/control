import matplotlib.pyplot as plt 
import control as clt 

# DC motor spec
J = 3.2284E-6
b = 3.5077E-6
K = 0.0274
R = 4
L = 2.75E-6

#Transfer Function
s=clt.tf('s')
p_motor=  K/(s*((J*s+b)*(L*s+R)+K*K))
print(p_motor)
