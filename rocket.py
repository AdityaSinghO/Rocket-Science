import numpy as np
import matplotlib.pyplot as plt

def simulate_flight(water_vol_fraction=0.33, pressure_psi=60, launch_angle=85):
    mass_empty = 0.262
    bottle_volume = 2.0e-3
    nozzle_diameter = 0.021
    rocket_diameter = 0.10
    drag_coeff_rocket = 0.45
    parachute_diameter = 0.40
    drag_coeff_chute = 1.5
    g = 9.81
    rho_water = 1000.0
    rho_air_amb = 1.225
    P_atm = 101325.0
    gamma = 1.4
    R_gas = 287.05

    area_rocket = np.pi * (rocket_diameter / 2.0) ** 2
    area_parachute = np.pi * (parachute_diameter / 2.0) ** 2
    area_nozzle = np.pi * (nozzle_diameter / 2.0) ** 2

    dt = 0.001
    P_initial = (pressure_psi * 6894.76) + P_atm
    V_water = bottle_volume * water_vol_fraction
    V_air = bottle_volume - V_water
    V_air_start = V_air
    T_inital = 300.0

    mass_air = (P_initial * V_air) / (R_gas * T_inital)
    total_mass = mass_empty + (V_water * rho_water) + mass_air
    x, y = 0.0, 0.0
    theta = np.radians(launch_angle)
    vx, vy = 0.0, 0.0
    t = 0.0

    P_current = P_initial
    T_current = T_inital
    current_phase = "Water Boost"
    current_cd = drag_coeff_rocket
    current_area = area_rocket

    data = {'t':[], 'y': [], 'x': [], 'vy': [], 'thrust': []}

    while y >= 0:
        thrust_force = 0.0

        if V_water > 0: # Phase 1: Water
            current_phase = "Water Boost"

            pressure_delta = np.maximum(0, P_current - P_atm)
            v_exhaust = np.sqrt(2 * pressure_delta / rho_water)

            dm_dt = area_nozzle * rho_water * v_exhaust
            thrust_force = dm_dt * v_exhaust

            dV = (dm_dt / rho_water) * dt
            V_water -= dV
            V_air += dV
            P_current = P_initial * (V_air_start / V_air) ** gamma
        elif P_current > P_atm: # Phase 2: Air Pulse
            current_phase = "Air Pulse"
            V_water = 0
            crit_ratio = (2 / (gamma+1)**(gamma/gamma-1))

            if (P_atm / P_current) < crit_ratio: # Choked
                P_exit = P_current * crit_ratio
                T_exit = T_current * (2/(gamma+1))
                v_exhaust = np.sqrt(gamma * R_gas * T_exit)
                rho_exit = P_exit / (R_gas * T_exit)
                thrust_force = (rho_exit * area_nozzle * v_exhaust**2) + (P_exit - P_atm) * area_nozzle
                dm_dt = rho_exit * area_nozzle * v_exhaust
            else: # Unchoked
                pressure_term = np.maximum(0, (P_current/P_atm)**((gamma-1)/gamma))
                M_e = np.sqrt((2/(gamma-1)) * pressure_term)
                T_exit = T_current / (1 + ((gamma-1)/2) * M_e**2)
                v_exhaust = M_e * np.sqrt(gamma * R_gas * T_exit)
                rho_exit = P_atm / (R_gas * T_exit)
                thrust_force = rho_exit * area_nozzle * v_exhaust**2
                dm_dt = rho_exit * area_nozzle * v_exhaust

            mass_air = dm_dt * dt
            if mass_air > 0:
                rho_current = mass_air / bottle_volume
                T_current = T_inital * (rho_current / (mass_air/bottle_volume)) ** (gamma-1)
                P_current = rho_current * R_gas * T_current
            else: # Phase 3: Coast
                thrust_force = 0.0
                dm_dt = 0.0
                if vy < 0 and current_phase != "Parachute Descent":
                    current_phase = "Parachute Descent"
                    current_cd = drag_coeff_chute
                    current_area = area_parachute
                elif current_phase != "Parachute Descent":
                    current_phase = "Coasting"

            v_total = np.sqrt(vx**2 + vy**2)
            flight_path_angle = np.arctan2(vy, vx) if v_total > 0.01 else theta

            drag_force = 0.5 * rho_air_amb * (v_total**2) * current_cd * current_area

            Fx = (thrust_force * np.cos(theta)) - (drag_force * np.cos(flight_path_angle))
            Fy = (thrust_force * np.sin(theta)) - (drag_force * np.sin(flight_path_angle)) - (total_mass * g)

            vx += (Fx / total_mass) * dt
            vy += (Fy / total_mass) * dt
            x+= vx * dt
            y+= vy * dt
            total_mass -= dm_dt * dt
            t += dt

            data['t'].append(t)
            data['x'].append(x)
            data['y'].append(y)
            data['vy'].append(vy)
            data['thrust'].append(thrust_force)

            if t > 20.0 and y <= 0: 
                break
    return data


print("Running Simulation: ")
data = simulate_flight(water_vol_fraction=0.33, pressure_psi=60, launch_angle=85)

print(f"Max Altitude: {max(data['y']):.2f} m")

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize(10, 15))

#Trajectory Plot
ax1.plot(data['x'], data['y'], '-b', linewidth=2)
ax1.set_title(f"1. Flight Trajectory (Side View) - Max Alt: {max(data['y']):.1f} m")
ax1.set_xlabel("Distance Downrange (m)")
ax1.set_ylabel("Altitude (m)")
ax1.axhline(0, color = 'k', linewidth = 2)
ax1.grid(True)

#Apogee Mark
if len(data['y']) > 0:
    apogee_idx = np.argmax(data['y'])
    ax1.plot(data['x'][apogee_idx], data['y'][apogee_idx], 'ro')
    ax1.text(data['x'][apogee_idx], data['y'][apogee_idx], 'Apogee', color = 'red')

# Velocity Plot
ax2.plot(data['t'], data['vy'], '-r', linewidth = 2)
ax2.set_title("2. Vertcial Velocity VS Time")
ax2.set_ylabel("Velocity (m/s)")
ax2.set_xlabel("Time (s)")
ax2.grid(True)
ax2.axhline(0, color = 'black', linestyle = '--')

# Thrust Plot
ax3.plot(data['t'], data['thrust'], '-g', linewidth = 2)
ax3.set_title("3. Thrust Profile")
ax3.set_ylabel("Thrust Force (N)")
ax3.set_xlabel("Time (s)")
ax3.grid(True)

plt.tight_layout()
plt.show()

results = []
ang = []
for angle in range(1, 90):
    ang.append(angle)
    sim_data = simulate_flight(0.33, 60, angle)
    max_alt = max(sim_data['y'])
    results.append(max_alt)


plt.figure(figsize=(8, 5))
plt.plot(ang, results, '-ro')
plt.title("Optimization: Launch Angle VS Altitude")
plt.xlabel("Launch Angle (degrees)")
plt.ylabel("Max Altitude (m)")
plt.grid(True)
plt.show()