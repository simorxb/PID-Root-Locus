import matplotlib.pyplot as plt
import control as ct

s = ct.TransferFunction.s

# Define the system (unstable)
G = 1/(s - 5)

# Define a PID in a form suited for root locus
# C = Kp + Ki/s + Kd*s/(tau*s + 1) = (s^2*(Kp*tau+Kd) + s*(Kp+Ki*tau) + Ki)/(s*(tau*s+1))
# Two poles and two zeros, then C can be written as:
# C = K*(s/z1-1)*(s/z2-1)/(s*(tau*s+1)) = (s^2*(K/(z1*z2)) + s*K*(-1/z1-1/z2) + K)/(s*(tau*s+1))
# As both controllers are the same:
# Ki = K
# Kp = K*(1/z1+1/z2) - Ki*tau
# Kd = K/(z1*z2) - Kp*tau

# Derivative filter time
tau = 0.1

# Controller without numerator - to choose poles
C = 1/(s*(tau*s + 1))

# Plot root-locus of C*P
plt.figure()
ct.root_locus(C*G)


# Choose poles to end up with poles around -5
z1 = -2.5
z2 = -7.5
C = ((s/z1 - 1)*(s/z2 - 1))/(s*(tau*s + 1))

# Plot root-locus of C*P
plt.figure()
ct.root_locus(C*G)

# Choose K to have real poles
K = 73

# Calculate PID gains
Ki = K
Kp = K*(-1/z1-1/z2) - Ki*tau
Kd = K/(z1*z2) - Kp*tau

print(f"Kp: {Kp} - Kd: {Kd} - Ki: {Ki}")

C = Kp + Ki/s + Kd*s/(tau*s + 1)

# Plot Nyquist diagram
plt.figure()
ct.nyquist_plot(C*G)

# Close loop transfer function setpoint to output
T = ct.feedback(C*G)

# Close loop transfer function setpoint to control input
S = 1/(1/C + G)

# Print transfer functions
print(f"T(s): {T}")
print(f"S(s): {S}")

# Find poles
poles = ct.pole(T)
zeros = ct.zero(T)
print(f"Poles of T: {poles}")
print(f"Zeros of T: {zeros}")

# Plot step response
plt.figure()

# Step response - setpoint to output
t, yout = ct.step_response(T, T_num=100)

plt.subplot(2, 1, 1)
plt.plot(t, yout)
plt.ylabel("Output")
plt.grid()

# Step response - setpoint to control input
t, uout = ct.step_response(S, T_num=100)

plt.subplot(2, 1, 2)
plt.plot(t, uout)
plt.grid()
plt.ylabel("Control input")
plt.xlabel("Time [s]")

plt.show()