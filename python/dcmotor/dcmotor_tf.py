import matplotlib.pyplot as plt 
import control as clt 

#Dc motor specificaiton
J = 0.01
b = 0.1
K = 0.1
R = 1.0
L = 0.5
s = clt.tf('s')
P_motor = K/((J*s+b)*(L*s+R)+K*K)


#Step Response 1
plt.figure(1)
t,y=clt.step_response(P_motor)
plt.grid()
plt.plot(t,y)
#plt.show()

#Pole Zero Plot
plt.figure(2)
clt.pole_zero_plot(P_motor)
plt.grid()
plt.show()

plt.figure(2)
clt.pole_zero_plot(P_motor)
plt.grid()
plt.show()
