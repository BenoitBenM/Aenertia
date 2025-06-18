from matplotlib import pyplot as plt
from math import sin, cos, asin, radians
#Definition of samping frequency and simulation time
f_s = 500
time = 30
timescale = 1/f_s
t = [i*timescale for i in range(0,time*f_s)]

#Definition of variables in the simulation
k_p = 50
k_i = 0.01
k_d = 4

k_p_vel = 0.015  # 建议初始值
k_d_vel = 0

e_v = 0

e_i = 0
e_d = 0
theta_d = radians(0)
velocity_desired = 0
e_theta = 0
sensor_bias = radians(0)
delay_sensor_ms = 0
delay_torque_ms = 10
delay_s = delay_sensor_ms/(1000*timescale)
delay_t = delay_torque_ms/(1000*timescale)

mass = 2
height = 0.3
g = 9.81
r = 0.05

#Definition of Initial Values of the simulation
theta_init = radians(0)
torque = [0]
omega = [0]
correction_angle = [0]
theta = [theta_init]
w = [0]
motor_torque = [0]
velocity = [0]
distance = [0]
theta_desired = [0]
desired_velocity = [0]
acc = [0]


#Simulation Loop
for i in range(1,time*f_s):

    if((int(i/(time*f_s/3)))%2==0):
        velocity_desired = 1
    else:
        velocity_desired = -1

    # Outer Loop
    e_d_v = e_v
    e_v = velocity_desired - velocity[i-1]
    e_d_v = (e_v-e_d_v)/timescale
    theta_d_raw = k_p_vel*e_v + k_d_vel*e_d_v

    if theta_d_raw<0:
        theta_d_raw = max(theta_d_raw, radians(-10))
    else:
        theta_d_raw = min(theta_d_raw, radians(10)) # Clamp

    alpha_theta_d = 0.05  # smaller = more filtering (lower bandwidth)
    theta_d = (1 - alpha_theta_d) * theta_d + alpha_theta_d * theta_d_raw

    #Inner Loop
    e_d = e_theta
    if(i-delay_s < 0):
        e_theta = 0
    else:
        e_theta = theta_d - (theta[i - 1 - int(delay_s)]+sensor_bias)  # error in angle
    e_i += e_theta
    e_d = (e_theta-e_d)/timescale
    # w_new = k_p*e_theta + k_i*e_i + k_d*e_d
    
    motor_torque_new = -(k_p*e_theta + k_i*e_i + k_d*e_d)
    acc_new = motor_torque_new/(mass*r**2+1/2*0.05*r**2)
    if(acc_new > 10):
        acc_new = 10
    elif(acc_new < -10):
        acc_new = -10
    else:
        acc_new = acc_new

    w_new = w[i-1]+acc_new*timescale
    if(w_new > 20):
        w_new = 20
    elif(w_new < -20):
        w_new = -20
    else:
        w_new = w_new

    

    if(i-delay_t < 0):
        torque_control = 0
    else:
        torque_control = (w[i-1]-w[i-2])/timescale*(mass*r**2+1/2*0.05*r**2)

    # w.append(w_new)
    if(i-delay_t < 0):
        torque_control = 0
    else:
        torque_control = motor_torque_new
        # torque_control = -w[i-int(delay_t)]*r    # reaction torque from wheel
    
    # Total torque = gravity torque + wheel reaction torque
    torque_new = mass * g * height * sin(theta[i - 1]) - torque_control

    # Angular acceleration
    alpha = torque_new / (mass * height ** 2)
    

    # Integrate
    omega_new = omega[i - 1] + alpha * timescale
    theta_new = theta[i - 1] + omega_new * timescale
    alpha_motor_new = motor_torque_new/(1/2*0.05*r**2+mass*r**2)
    w_new = w[i-1] + alpha_motor_new*timescale
    velocity_new = w_new*r
    distance_new = distance[i-1]+velocity_new * timescale

    # Optional fallover clamp
    if abs(theta_new) >= radians(90):
        theta_new = max(min(theta_new, radians(90)), radians(-90))
        omega_new = 0

    # Store data
    torque.append(torque_new)
    omega.append(omega_new)
    theta.append(theta_new)
    w.append(w_new)
    velocity.append(velocity_new)
    distance.append(distance_new)
    theta_desired.append(theta_d)
    desired_velocity.append(velocity_desired)
    acc.append(acc_new)
    

plt.plot(t[100:-1], theta[100:-1])
plt.plot(t[100:-1], theta_desired[100:-1])
plt.title("Tilt Angle (rad)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()

plt.plot(t[100:-1], acc[100:-1])
plt.title("Wheel Angular Velocity (rad/s)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()

plt.plot(t[100:-1], distance[100:-1])
plt.title("Distance (m)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()

plt.plot(t[100:-1], velocity[100:-1])
plt.plot(t[100:-1], desired_velocity[100:-1])
plt.title("Linear Velocity (m/s)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()
