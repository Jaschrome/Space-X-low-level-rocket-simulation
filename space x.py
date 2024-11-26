import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants for Falcon 9 (real-world values)
g = 9.81  # Gravity constant at sea level (m/s^2)
Cd = 0.75  # Drag coefficient (assumed for Falcon 9)
A = 16.2  # Cross-sectional area of Falcon 9 (m^2)
R_earth = 6371000  # Radius of Earth (m)
rho_sea_level = 1.225  # Air density at sea level (kg/m^3)

# Real-world Falcon 9 values
thrust_main = 7607000  # Thrust from first stage (N)
thrust_second = 1100000  # Thrust from second stage (N)
dry_mass_first_stage = 30000  # Dry mass of Falcon 9 first stage (kg)
fuel_capacity_first_stage = 500000  # Fuel capacity (kg)
total_mass_at_launch = 549000  # Total mass at launch (kg)
burn_rate_ascent = 250  # Ascent fuel burn rate (kg/s)
burn_rate_landing = 30  # Descent fuel burn rate (kg/s)
time_limit = 600  # Total simulation time (seconds)
time_step = 1  # Time step for simulation (seconds)

# Get user input for launch and landing coordinates, and fuel amount
launch_lat = float(input("Enter the launch latitude (degrees): "))
launch_lon = float(input("Enter the launch longitude (degrees): "))
landing_lat = float(input("Enter the landing latitude (degrees): "))
landing_lon = float(input("Enter the landing longitude (degrees): "))
fuel_fill = float(input(f"Enter the amount of fuel to fill up (max {fuel_capacity_first_stage} kg): "))

# Ensure fuel does not exceed capacity
fuel_fill = min(fuel_fill, fuel_capacity_first_stage)

# Initial rocket conditions
initial_lat = launch_lat
initial_lon = launch_lon
initial_alt = 0  # Initial altitude in meters (rocket starts at the ground)
initial_vx = 0  # Initial horizontal velocity (m/s)
initial_vy = 0  # Initial horizontal velocity (m/s)
initial_vz = 0  # Initial vertical velocity (m/s)
mass = total_mass_at_launch - fuel_capacity_first_stage + fuel_fill  # Total mass including fuel

# Initial state: [latitude, longitude, altitude, vx, vy, vz, mass]
state = [initial_lat, initial_lon, initial_alt, initial_vx, initial_vy, initial_vz]

# Lists to store data for plotting
trajectory_lat = [initial_lat]
trajectory_lon = [initial_lon]
trajectory_alt = [initial_alt]
trajectory_vx = [initial_vx]
trajectory_vy = [initial_vy]
trajectory_vz = [initial_vz]
trajectory_thrust = [0]
trajectory_mass = [mass]
trajectory_speed = [0]
trajectory_fuel = [fuel_fill]

# Calculate gravity at a given altitude
def get_gravity(altitude):
    return g / (1 + (altitude / R_earth)) ** 2

# Calculate drag force based on speed and altitude
def get_drag_force(speed, altitude):
    air_density = rho_sea_level * np.exp(-altitude / 8500)  # Simplified model of air density
    return 0.5 * Cd * A * air_density * speed ** 2

# Rocket dynamics function with projectile motion
def rocket_dynamics(t, state, m, fuel_fill):
    lat, lon, alt, vx, vy, vz = state
    speed = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    # Get gravity at the current altitude
    gravity = get_gravity(alt)

    # Calculate drag force
    drag_force = get_drag_force(speed, alt)
    drag_acceleration = drag_force / m

    # Calculate accelerations
    ax = -drag_acceleration * (vx / speed) if speed > 0 else 0
    ay = -drag_acceleration * (vy / speed) if speed > 0 else 0
    az = -gravity - drag_acceleration * (vz / speed) if speed > 0 else -gravity

    # Apply thrust during ascent and descent
    if vz < 0 and t < 150:  # Ascent phase (burning fuel)
        az += np.sign(vz) * thrust_main / m
        m -= burn_rate_ascent * time_step  # Burn fuel during ascent
        fuel_fill -= burn_rate_ascent * time_step  # Decrease fuel during ascent

    if vz > 0 and t > 150:  # Descent phase (landing)
        az += np.sign(vz) * thrust_second / m  # Apply smaller thrust for descent control
        m -= burn_rate_landing * time_step  # Burn fuel during descent
        fuel_fill -= burn_rate_landing * time_step  # Decrease fuel during descent

    # Update latitude and longitude
    dlat = vx / R_earth * (180 / np.pi)
    dlon = vy / (R_earth * np.cos(np.radians(lat))) * (180 / np.pi)

    # Update altitude based on vertical velocity
    dz = vz * time_step
    alt += dz  # Altitude follows vertical motion, including gravity and thrust

    # Ensure altitude is never negative
    alt = max(alt, 0)

    return [dlat, dlon, dz, ax, ay, az, m, fuel_fill]

# Simulation loop
def simulate_trajectory():
    global state, mass, fuel_fill
    for t in range(1, time_limit):
        state_dot = rocket_dynamics(t, state, mass, fuel_fill)
        state = [state[i] + state_dot[i] * time_step for i in range(6)]  # Update state
        mass = state_dot[6]  # Update mass
        fuel_fill = state_dot[7]  # Update fuel

        # Append trajectory data for plotting
        trajectory_lat.append(state[0])
        trajectory_lon.append(state[1])
        trajectory_alt.append(state[2])
        trajectory_vx.append(state[3])
        trajectory_vy.append(state[4])
        trajectory_vz.append(state[5])
        trajectory_thrust.append(thrust_main if t < 150 else 0)  # Thrust during ascent and descent
        trajectory_mass.append(mass)
        trajectory_fuel.append(fuel_fill)
        trajectory_speed.append(np.sqrt(state[3] ** 2 + state[4] ** 2 + state[5] ** 2))

# Setup for real-time plotting
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

# Plot for rocket trajectory (latitude vs longitude)
ax1.set_title('Rocket Trajectory (Latitude vs Longitude)')
ax1.set_xlabel('Longitude (degrees)')
ax1.set_ylabel('Latitude (degrees)')

# Plot for fuel and thrust data
ax2.set_title('Fuel and Thrust Data')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Values')

# Plot for rocket speed (velocity) changes
ax3.set_title('Rocket Speed Over Time')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Speed (m/s)')

# Animation function to update the plots
def update_plot(t):
    ax1.clear()
    ax2.clear()
    ax3.clear()

    ax1.set_title('Rocket Trajectory (Latitude vs Longitude)')
    ax1.set_xlabel('Longitude (degrees)')
    ax1.set_ylabel('Latitude (degrees)')

    ax2.set_title('Fuel and Thrust Data')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Values')

    ax3.set_title('Rocket Speed Over Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (m/s)')

    # Plot latitude vs longitude (trajectory)
    ax1.plot(trajectory_lon[:t], trajectory_lat[:t], label='Rocket Trajectory', color='blue')

    # Plot fuel, thrust, and speed
    ax2.plot(range(t), trajectory_fuel[:t], label='Fuel Mass', color='green')
    ax2.plot(range(t), trajectory_thrust[:t], label='Thrust (N)', color='red')
    ax3.plot(range(t), trajectory_speed[:t], label='Rocket Speed', color='purple')

    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax3.legend(loc="lower left")

# Run the simulation and animation
simulate_trajectory()

# Create the FuncAnimation object
ani = FuncAnimation(fig, update_plot, frames=range(1, len(trajectory_lat)), interval=100)

plt.tight_layout()
plt.show()
