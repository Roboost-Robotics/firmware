import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.integrate import solve_ivp

# Initial system parameters
V_init = 12.0  # Initial voltage
R_init = 2.0  # Initial resistance in Ohms
k_e_init = 0.015  # Back EMF constant in V-s/rad
k_t_init = 0.015  # Torque constant in Nm/A
GR_init = 100.0 / 260.0  # Initial gear reduction ratio
r_init = 0.06  # Initial wheel radius in meters
m_init = 0.5  # Initial wheel mass in kg

# Time vector
t = np.linspace(0, 5, 1000)  # From 0 to 5 seconds
def update(val):
    R = s_R.val
    V = s_V.val
    k_e = s_k_e.val
    k_t = s_k_t.val
    GR = s_GR.val
    r = s_r.val
    m = s_m.val

    # Update derived parameters
    tau_max = k_t * (V / R)
    omega_no_load = V / k_e
    J_wheel = 0.5 * m * r**2
    J_total = J_wheel

    # System dynamics function
    def system_dynamics(t, y):
        theta, omega = y
        omega_motor = omega * GR
        tau_motor = tau_max - k_t * omega_motor
        tau_wheel = tau_motor * GR
        domega_dt = tau_wheel / J_total
        dtheta_dt = omega
        return [dtheta_dt, domega_dt]

    # Solve the system
    sol = solve_ivp(system_dynamics, [t[0], t[-1]], [0, 0], t_eval=t, method='RK45')
    
    line_position.set_ydata(sol.y[0])
    line_velocity.set_ydata(sol.y[1])
    
    # Set dynamic axes limits based on the data
    ax_position.set_ylim(np.min(sol.y[0]) - 0.1 * np.abs(np.min(sol.y[0])), np.max(sol.y[0]) + 0.1 * np.abs(np.max(sol.y[0])))
    ax_velocity.set_ylim(np.min(sol.y[1]) - 0.1 * np.abs(np.min(sol.y[1])), np.max(sol.y[1]) + 0.1 * np.abs(np.max(sol.y[1])))
    
    fig.canvas.draw_idle()

# Setup the figure and axis for plots
fig, (ax_position, ax_velocity) = plt.subplots(1, 2, figsize=(12, 6))

# Plot the initial data
line_position, = ax_position.plot(t, np.zeros_like(t), label='Angular Position (rad)')
ax_position.set_xlabel('Time (s)')
ax_position.set_ylabel('Position (rad)')
ax_position.legend()

line_velocity, = ax_velocity.plot(t, np.zeros_like(t), label='Angular Velocity (rad/s)')
ax_velocity.set_xlabel('Time (s)')
ax_velocity.set_ylabel('Velocity (rad/s)')
ax_velocity.legend()

# Add sliders for parameters
axcolor = 'lightgoldenrodyellow'
ax_V = plt.axes([0.2, 0.01, 0.65, 0.03], facecolor=axcolor)
ax_R = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor=axcolor)
ax_k_e = plt.axes([0.2, 0.09, 0.65, 0.03], facecolor=axcolor)
ax_k_t = plt.axes([0.2, 0.13, 0.65, 0.03], facecolor=axcolor)
ax_GR = plt.axes([0.2, 0.17, 0.65, 0.03], facecolor=axcolor)
ax_r = plt.axes([0.2, 0.21, 0.65, 0.03], facecolor=axcolor)
ax_m = plt.axes([0.2, 0.25, 0.65, 0.03], facecolor=axcolor)

s_V = Slider(ax_V, 'Voltage (V)', 0.0, 24.0, valinit=V_init)
s_R = Slider(ax_R, 'Resistance (Ohms)', 0.1, 10.0, valinit=R_init)
s_k_e = Slider(ax_k_e, 'Back EMF Constant (k_e)', 0.001, 0.03, valinit=k_e_init)
s_k_t = Slider(ax_k_t, 'Torque Constant (k_t)', 0.001, 0.03, valinit=k_t_init)
s_GR = Slider(ax_GR, 'Gear Ratio (GR)', 0.1, 1.0, valinit=GR_init)
s_r = Slider(ax_r, 'Wheel Radius (m)', 0.01, 0.1, valinit=r_init)
s_m = Slider(ax_m, 'Wheel Mass (kg)', 0.1, 2.0, valinit=m_init)

# Initialize with default values
update(None)

# Connect the update function to sliders
s_V.on_changed(update)
s_R.on_changed(update)
s_k_e.on_changed(update)
s_k_t.on_changed(update)
s_GR.on_changed(update)
s_r.on_changed(update)
s_m.on_changed(update)

plt.show()