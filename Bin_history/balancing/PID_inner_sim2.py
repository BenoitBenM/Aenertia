from matplotlib import pyplot as plt
from math import sin, cos, asin, radians
from scipy.signal import chirp
import numpy as np
from scipy.fft import fft, fftfreq
#Definition of samping frequency and simulation time
f_s = 500
time = 10
timescale = 1/f_s
t = [i*timescale for i in range(0,time*f_s)]

#Definition of variables in the simulation
k_p = 1000
k_i = 0
k_d = 5

e_i = 0
e_d = 0
theta_d = radians(0)
e_theta = 0
e_dist = 0
w_real = 0
sensor_bias = radians(0)
delay_sensor_ms = 0
delay_torque_ms = 0
delay_s = delay_sensor_ms/(1000*timescale)
delay_t = delay_torque_ms/(1000*timescale)

mass = 0.7
height = 0.1
g = 9.81
r = 0.035
mass_wheel = 0.05
acc_max = 20
moi = mass*height**2

#Definition of Initial Values of the simulation
theta_init = radians(3)
torque = [0]
omega = [0]
correction_angle = [0]
theta = [theta_init]
w = [0]
w_desired = [0]
motor_torque = [0]
velocity = [0]
distance = [0]
theta_desired = [0]
realVelocity = [0]

chirp_signal = chirp(np.array(t), f0=0.1, f1=10, t1=time, method='linear')


#Simulation Loop
for i in range(1,time*f_s):

    # theta_d = radians(5) * sin(2 * 3.1416 * 10 * i * timescale)
    # theta_d = radians(5) * chirp_signal[i]
    # if i == 1000:
    #     theta_d = radians(20)
    # else:
    #     theta_d = 0
    
    e_d = e_theta
    if(i-delay_s < 0):
        e_theta = 0
    else:
        e_theta =  (theta[i - 1 - int(delay_s)]+sensor_bias) - theta_d  # error in angle
    e_i += e_theta*timescale
    e_d = (e_theta-e_d)/timescale

    w_d = k_p*e_theta + k_i*e_i + k_d*e_d
    if(w_real < w_d):
        acc = acc_max
    else:
        acc = -acc_max
    w_real += acc*timescale

    torque_control = acc*r*cos(theta[i-1])*mass*height
    
    # Total torque = gravity torque + wheel reaction torque
    torque_new = mass * g * height * sin(theta[i - 1]) - torque_control

    # Angular acceleration
    alpha = torque_new / (mass * height ** 2)

    # Integrate
    omega_new = omega[i - 1] + alpha * timescale
    theta_new = theta[i - 1] + omega_new * timescale
    velocity_new = w_real*r
    distance_new = distance[i-1]+velocity_new * timescale

    # Optional fallover clamp
    if abs(theta_new) >= radians(90):
        theta_new = max(min(theta_new, radians(90)), radians(-90))
        omega_new = 0

    # Store data
    torque.append(torque_new)
    omega.append(omega_new)
    theta.append(theta_new)
    w.append(w_real)
    w_desired.append(w_d)
    velocity.append(velocity_new)
    distance.append(distance_new)
    theta_desired.append(theta_d)
    

plt.plot(t[1:-1], theta[1:-1])
plt.plot(t[1:-1], theta_desired[1:-1])
plt.title("Tilt Angle (rad)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()

plt.plot(t[100:-1], w[100:-1])
plt.plot(t[100:-1], w_desired[100:-1])
plt.title("Wheel Angular Velocity (rad/s)")
plt.xlabel("Time (s)")
plt.grid()
plt.show()

# plt.plot(t[100:-1], distance[100:-1])
# plt.title("Distance (m)")
# plt.xlabel("Time (s)")
# plt.grid()
# plt.show()

# plt.plot(t[100:-1], velocity[100:-1])
# plt.title("Linear Velocity (m/s)")
# plt.xlabel("Time (s)")
# plt.grid()
# plt.show()

# plt.plot(t[100:-1], torque[100:-1])
# plt.title("Torque")
# plt.xlabel("Time (s)")
# plt.grid()
# plt.show()


# # Use a trimmed steady-state region to avoid startup transients
# start_idx = int(2 * f_s)  # Skip first 2 seconds
# theta_d_trim = np.array(theta_desired[start_idx:])
# theta_trim = np.array(theta[start_idx:])
# n = len(theta_trim)
# T = timescale

# # Apply FFT
# Theta_d_f = fft(theta_d_trim)
# Theta_f = fft(theta_trim)
# freqs = fftfreq(n, T)

# # Use only the positive frequencies
# mask = freqs > 0
# freqs = freqs[mask]
# gain = np.abs(Theta_f[mask] / Theta_d_f[mask])

# # Normalize and find crossover frequency (gain = 1)
# gain_db = 20 * np.log10(gain)
# idx_cross = np.argmin(np.abs(gain - 1))  # closest to gain = 1
# f_crossover = freqs[idx_cross]

# print(f"Crossover frequency ≈ {f_crossover:.2f} Hz")

# # Optional: Plot Bode magnitude
# plt.semilogx(freqs, gain_db)
# plt.axhline(0, color='r', linestyle='--', label="0 dB")
# plt.axvline(f_crossover, color='g', linestyle='--', label=f"Crossover: {f_crossover:.2f} Hz")
# plt.title("Estimated Bode Plot (Magnitude)")
# plt.xlabel("Frequency (Hz)")
# plt.ylabel("Gain (dB)")
# plt.grid(True, which="both")
# plt.legend()
# plt.show()