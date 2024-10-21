**1. Program : 01_dcmotor_tf.py**

      a) Import library
           import matplotlib.pyplot as plt 
           import control as clt 
      b) Assign variables to dc motor parameters
            # DC motor spec
            J = 3.2284E-6
            b = 3.5077E-6
            K = 0.0274
            R = 4
            L = 2.75E-6
      c) Trasfer Function of DC Motor
            #Transfer Function
            s=clt.tf('s')
            p_motor=  K/(s*((J*s+b)*(L*s+R)+K*K))
            print(p_motor)
      d) Output on screen

      ![image](/transfer_function/python/image/dc_motor_tf1.png)


**2. Program :**
3. Program
