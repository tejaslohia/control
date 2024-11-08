# Import Library
import numpy as np
import matplotlib.pyplot as plt

# Motor parameters (replace with actual values)
Rs = 0.5             # Stator resistance (ohms)
Ld = 0.00015          # d-axis inductance (H)
Lq = 0.00015          # q-axis inductance (H)
J = 0.01             # Rotor inertia (kg.m^2)
B = 0.001            # Damping coefficient (N.m.s)
lambda_f = 0.1       # Flux linkage (Wb)
p = 4                # Number of pole pairs

# Function to convert Vq,Vd to Va,Vb,Vc 
def dq_to_abc(Vd, Vq, theta):
    # Inverse Park Transformation (d-q to alpha-beta frame)
    V_alpha = Vd * np.cos(theta) - Vq * np.sin(theta)
    V_beta = Vd * np.sin(theta) + Vq * np.cos(theta)
    
    # Inverse Clarke Transformation (alpha-beta to a-b-c frame)
    Va = V_alpha
    Vb = -0.5 * V_alpha + (np.sqrt(3) / 2) * V_beta
    Vc = -0.5 * V_alpha - (np.sqrt(3) / 2) * V_beta
    
    return Va, Vb, Vc

# Function to convert Va,Vb,Vc to Vq,Vd 
def abc_to_dq(Va, Vb, Vc, theta):
    # Clarke Transformation (3-phase to alpha-beta frame)
    V_alpha = Va
    V_beta = (1 / np.sqrt(3)) * (Va + 2 * Vb)
    
    # Park Transformation (alpha-beta to d-q frame)
    Vd = V_alpha * np.cos(theta) + V_beta * np.sin(theta)
    Vq = -V_alpha * np.sin(theta) + V_beta * np.cos(theta)
    
    return Vd, Vq

# PMSM motor equations and solution using Eular Method
# Input :   1.  Pmsm motor state Iq and omega_m
#           2.  sampel time dt
#           3.  Vq_ref
#           4.  Load Torque Tl
# Ouput :  New Values of Iq (Q axis current) and omega_m (Motor Speed)

def pmsm_dynamics(dt,state, Vq,Tl=0):
    # State variables
    Iq, omega_m = state
    
    # Convert Mech omega to Electricl omega
    omega_e = p * omega_m

    # Calculate dq/dt and Iq
    Iq_dt = (Vq - Rs * Iq - omega_e*lambda_f) / Lq
    Iq = Iq + Iq_dt*dt

    # Calculate Electromagnetic torque
    Te = (3/2) * p * lambda_f * Iq

    # Mechanical dynamics
    domega_dt = (Te - Tl - B * omega_m) / J
    omega_m = omega_m + domega_dt*dt
    
    return [Iq,omega_m]

# Simulation parameters
t_span = 0.2  # Simulation time
dt = 1e-6  # Time step
t_eval=np.arange(0, t_span, dt)
time_len=len(t_eval)

# Initial conditions [Is, omega]
state0 = [0, 0]  

# np arrays for storing result
Iq=np.zeros(time_len) 
omega=np.zeros(time_len)
theta=np.zeros(time_len)

# Motor input voltage ref Vq_ref and load torque
Vq_ref=100
Tl=1

# Calculate motor performance 
index=0
for t in t_eval[:(time_len-1)]:
    state0=pmsm_dynamics(dt,state0,Vq_ref,Tl)
   
    index=index+1
    Iq[index],omega[index] = state0
    theta[index]=theta[index-1]+omega[index]*dt

    

# Calculate Ia,Ib and Ic for Iq, theta arrary and Id=0  
Ia,Ib,Ic=dq_to_abc(Vd=0,Vq=Iq,theta=theta) 

# Plot PMSM speed/Theta/Currents vs Time
plt.figure(1)
plt.tight_layout()

plt.subplot(3,1,1)
plt.plot(t_eval, omega*60/(2*np.pi), label='omega in rpm', color="g")
plt.title("PMSM Motor Simulation considering only Lq/Iq and Id=0")
plt.ylabel("PMSM Motor Speed (RPM)")
plt.grid()

plt.subplot(3,1,2)
plt.plot(t_eval, theta, label='omega in rpm', color="g")
plt.ylabel("Angle (Rad)")
plt.grid()

plt.subplot(3,1,3)
plt.plot(t_eval, Ia, label='i_a')
plt.plot(t_eval, Ib, label='i_b')
plt.plot(t_eval, Ic, label='i_c')

plt.xlabel("Time (Second)")
plt.ylabel("Ia,Ib,Ic")
plt.grid()

plt.show()
