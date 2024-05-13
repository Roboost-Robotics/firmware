import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import pandas as pd

# Initial system parameters
K = 1.0  # Gain
zeta_init = 0.5  # Initial damping ratio
omega_n_init = 2.0  # Initial natural frequency (rad/s)
bearing_clearance_init = 0.1  # Initial bearing clearance in radians 0.1 rad = 5.7 degrees

# Vibration parameters
vibration_amplitude_init = 0.05  # Amplitude of the vibration component
rollers_init = 12  # Number of rollers on the wheel

# Time vector from 0 to 10 seconds, 1000 points
t = np.linspace(0, 10, num=1000)

# Function to update the plot
def update(val):
    zeta = s_zeta.val
    omega_n = s_omega_n.val
    bearing_clearance = s_clearance.val
    vibration_amplitude = s_vibration_amplitude.val
    rollers = s_rollers.val
    
    y = np.zeros_like(t)
    for i in range(1, len(t)):
        if zeta < 1:  # Underdamped
            omega_d = omega_n * np.sqrt(1 - zeta**2)
            y_temp = K * (1 - (np.exp(-zeta * omega_n * t[i]) * (np.cos(omega_d * t[i]) + (zeta / np.sqrt(1 - zeta**2)) * np.sin(omega_d * t[i]))))
        elif zeta == 1:  # Critically damped
            y_temp = K * (1 - (1 + omega_n * t[i]) * np.exp(-omega_n * t[i]))
        else:  # Overdamped
            r1 = -omega_n * (zeta + np.sqrt(zeta**2 - 1))
            r2 = -omega_n * (zeta - np.sqrt(zeta**2 - 1))
            y_temp = K * (1 - (np.exp(r1 * t[i]) * (r2 - r1) - np.exp(r2 * t[i]) * (r1 - r2)) / (r2 - r1))
        
        # Bearing clearance effect
        if abs(y_temp - y[i-1]) > bearing_clearance:
            y[i] = y_temp
        else:
            y[i] = y[i-1]

    vibration_frequencies = y * rollers / (2 * np.pi)
    vibrations = vibration_amplitude * np.sin(2 * np.pi * vibration_frequencies * t)
    y += vibrations
    line.set_ydata(y)
    fig.canvas.draw_idle()

# Setup the figure and axis
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)

# Plot the initial data
y = np.zeros_like(t)
line, = ax.plot(t, y, label='Step Response with Dynamic Vibrations and Bearing Clearance')
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Output')
ax.grid(True)
ax.legend()

# Set initial axis limits
ax.set_ylim(-0.5, 2.0)  # Adjust this as needed based on your expected range

# Add sliders for real-time parameter adjustment
axcolor = 'lightgoldenrodyellow'
ax_zeta = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_omega_n = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_clearance = plt.axes([0.25, 0.2, 0.65, 0.03], facecolor=axcolor)
ax_vibration_amplitude = plt.axes([0.25, 0.25, 0.65, 0.03], facecolor=axcolor)
ax_rollers = plt.axes([0.25, 0.3, 0.65, 0.03], facecolor=axcolor)

s_zeta = widgets.Slider(ax_zeta, 'Damping Ratio', 0.0, 1.0, valinit=zeta_init)
s_omega_n = widgets.Slider(ax_omega_n, 'Natural Frequency', 0.1, 5.0, valinit=omega_n_init)
s_clearance = widgets.Slider(ax_clearance, 'Bearing Clearance', 0.0, 0.5, valinit=bearing_clearance_init)
s_vibration_amplitude = widgets.Slider(ax_vibration_amplitude, 'Vibration Amplitude', 0.0, 0.1, valinit=vibration_amplitude_init)
s_rollers = widgets.Slider(ax_rollers, 'Number of Rollers', 1, 20, valinit=rollers_init, valstep=1)

update(None)

# Update the plot when sliders are changed
s_zeta.on_changed(update)
s_omega_n.on_changed(update)
s_clearance.on_changed(update)
s_vibration_amplitude.on_changed(update)
s_rollers.on_changed(update)

plt.show()
