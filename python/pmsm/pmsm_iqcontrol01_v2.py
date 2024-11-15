# Import Library
import numpy as np
import matplotlib.pyplot as plt

# Motor parameters (replace with actual values)
Rs = 0.5             # Stator resistance (ohms)
Ld = 0.01         # d-axis inductance (H)
Lq = 0.01          # q-axis inductance (H)
J = 0.01             # Rotor inertia (kg.m^2)
B = 0.001            # Damping coefficient (N.m.s)
lambda_f = 0.1       # Flux linkage (Wb)
p = 4                # Number of pole pairs
Tl=1

class PIController:
    def __init__(self, kp, ki, setpoint=0.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.setpoint = setpoint  # Desired target value
        self.integral = 0.0  # Integral sum of errors
        self.last_error = 0.0  # Store the last error for smooth integration

    def change_setpoint(self,setpoint=0.0):
         self.setpoint = setpoint  # Desired target value
        
    def update(self, measured_value, dt):
        # Calculate the error between setpoint and measured value
        error = self.setpoint - measured_value
       
        # Update integral term with current error over time dt
        self.integral += error * dt

        # Calculate control output
        control_output = self.kp * error + self.ki * self.integral
        
        # Store the error for future use
        self.last_error = error
        
        return control_output

# PMSM motor equations and solution using Eular Method
# Input :   1.  Pmsm motor state Iq and omega_m
#           2.  sampel time dt
#           3.  Vq_ref
#           4.  Load Torque Tl
# Ouput :  New Values of Iq (Q axis current) and omega_m (Motor Speed)

def pmsm_dynamics(dt,motor_state,motor_input,load_torque=0):
    # State variables
    did_dt,Id, diq_dt,Iq,omega_m, theta = motor_state
    #input Va,Vb,VC
    Vd,Vq=motor_input
    # Load Torque
    Tl=load_torque

    # Convert Mech omega to Electricl omega
    omega_e = p * omega_m

    # d-q axis currents (id and iq) dynamics
    did_dt = (Vd - Rs * Id + omega_e * Lq * Iq) / Ld
    diq_dt = (Vq - Rs * Iq - omega_e * Ld * Id - omega_e * lambda_f) / Lq
    Id = Id + did_dt*dt
    Iq = Iq + diq_dt*dt

    # Electromagnetic torque
    Te = (3/2) * p * (lambda_f * Iq + (Ld-Lq)*Id*Iq)

    # Mechanical dynamics
    domega_dt = (Te - Tl - B * omega_m) / J
    omega_m = omega_m + domega_dt*dt
    theta = theta + omega_m * dt
    
    return [did_dt,Id,diq_dt,Iq,omega_m,theta]


# Simulation parameters
t_span = 0.2 # Simulation time
dt = 1e-4 # Time step
t_eval=np.arange(0, t_span, dt)
time_len=len(t_eval)

# Initial conditions [id,iq,omega,theta]
state0 = [0, 0, 0, 0, 0, 0]  

# np arrays for storing result
id=np.zeros(time_len)
iq=np.zeros(time_len) 
omega=np.zeros(time_len)
theta=np.zeros(time_len)
diq_dt=np.zeros(time_len)
did_dt=np.zeros(time_len)

iq_res=np.zeros(time_len)
id_res=np.zeros(time_len)
vq_res=np.zeros(time_len)
vd_res=np.zeros(time_len)


kp_iq = 10# Proportional gain
ki_iq = 50000 # Integral gain
kp_id = 2 # Proportional gain
ki_id = 500  # Integral gain

Iq_ref=5
Id_ref=0.1
PIController_iq=PIController(kp=kp_iq, ki=ki_iq, setpoint=Iq_ref) 
PIController_id=PIController(kp=kp_id, ki=ki_id, setpoint=Id_ref) 


# Input voltages
##Vd = 0.1
Tl=1 

index=0
i_w=0
for t in t_eval[:(time_len-1)]:

    Vq = PIController_iq.update(iq[index], dt)
    Vd = PIController_id.update(id[index], dt)

    vq_res[index]=Vq
    vd_res[index]=Vd
        
    input=[Vd,Vq]
    state0=pmsm_dynamics(dt=dt,motor_state=state0,motor_input=input,load_torque=Tl)
    index=index+1
    did_dt[index], id[index], diq_dt[index],iq[index], omega[index], theta[index] = state0
    

# Plot PMSM speed/Currents vs Time
plt.figure(1)
plt.tight_layout()

plt.subplot(3,1,1)
plt.plot(t_eval, omega*60/(2*np.pi), label='omega in rpm', color="g")
plt.title("PMSM Motor Simulation considering Lq/Ld,Iq,Id")
plt.ylabel("PMSM Motor Speed (RPM)")
plt.grid()

plt.subplot(3,1,2)
plt.plot(t_eval, id, label='Id')
plt.plot(t_eval, iq, label='Iq')
plt.ylabel("Id,Iq,")
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(t_eval, theta, label='theta')
plt.plot(t_eval,vd_res, label='vd_res')
plt.plot(t_eval,vq_res, label='vq_res')

plt.xlabel("Time (Second)")
plt.ylabel("Theta")
plt.grid()
plt.legend()

plt.show()


print(iq[0])