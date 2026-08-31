# Bottle Rocket Flight Simulator

A physics-based simulation of a water bottle rocket's flight, modeling the water-expulsion boost phase, compressed-air pulse phase, coasting, and parachute descent. Produces flight trajectory, velocity, and thrust plots, plus a launch-angle optimization sweep.

## What It Does

`rocket.py` numerically integrates the flight of a water rocket through four phases:

1. **Water Boost** — pressurized water is expelled through the nozzle, generating thrust from the water jet.
2. **Air Pulse** — once the water is exhausted, the remaining compressed air continues to expand and thrust the rocket, using choked or unchoked nozzle flow depending on the pressure ratio.
3. **Coasting** — after the air pressure drops to ambient, the rocket flies unpowered under gravity and drag.
4. **Parachute Descent** — once the rocket starts falling, drag is recalculated using the parachute's larger area and drag coefficient.

The simulation uses a fixed timestep (`dt = 0.001 s`) forward-Euler integration of the 2D equations of motion (x, y position and velocity), accounting for thrust, aerodynamic drag, and gravity.

## Usage

```bash
python rocket.py
```

Running the script will:

1. Simulate a single flight with default parameters and print the max altitude.
2. Display a 3-panel figure: flight trajectory (with apogee marked), vertical velocity vs. time, and thrust profile vs. time.
3. Sweep launch angle from 1° to 89° (holding water fraction and pressure fixed) and plot max altitude vs. launch angle to find the optimal angle.

## Function: `simulate_flight`

```python
simulate_flight(water_vol_fraction=0.33, pressure_psi=60, launch_angle=85)
```

| Parameter | Description | Default |
|---|---|---|
| `water_vol_fraction` | Fraction of the bottle filled with water (0–1) | `0.33` |
| `pressure_psi` | Initial gauge pressure inside the bottle (psi) | `60` |
| `launch_angle` | Launch angle from horizontal (degrees) | `85` |

Returns a `dict` with time-series data:
- `t` — time (s)
- `x` — downrange distance (m)
- `y` — altitude (m)
- `vy` — vertical velocity (m/s)
- `thrust` — thrust force (N)

## Fixed Rocket/Environment Parameters

Set as constants inside `simulate_flight`:

| Parameter | Value |
|---|---|
| Empty rocket mass | 0.262 kg |
| Bottle volume | 2.0 L |
| Nozzle diameter | 21 mm |
| Rocket body diameter | 100 mm |
| Rocket drag coefficient | 0.45 |
| Parachute diameter | 0.40 m |
| Parachute drag coefficient | 1.5 |
| Ambient temperature | 300 K |

## Dependencies

- `numpy`
- `matplotlib`

Install with:

```bash
pip install numpy matplotlib
```

## Known Issues

Before relying on this for accurate results, note a few bugs in the current code:

- **`figsize(10, 15)` should be `figsize=(10, 15)`** in the `plt.subplots(...)` call — as written this will raise a `NameError`/`TypeError` since `figsize` isn't a defined function.
- **Choked-flow critical ratio formula** has an operator precedence bug: `(2 / (gamma+1)**(gamma/gamma-1))` evaluates `gamma/gamma-1` as `(gamma/gamma) - 1 = 0` (due to Python operator precedence and likely a typo), rather than the intended `gamma/(gamma-1)`. This makes the choked/unchoked branch decision incorrect.
- **Air-phase density/temperature update looks circular**: `rho_current / (mass_air/bottle_volume)` simplifies to `1`, so `T_current` never actually updates from `T_inital` inside the Air Pulse phase — the isentropic expansion of the air charge isn't being tracked correctly.
- **Position/velocity integration and phase transitions only happen inside the `elif P_current > P_atm` branch** — once pressure drops to ambient (end of Air Pulse), the loop's `while y >= 0` condition is still checked, but nothing updates `x`, `y`, `vx`, `vy`, or `t` anymore, so the simulation will likely hang in an infinite loop (or stop appending data) once air pressure reaches ambient before apogee is reached, rather than properly transitioning into "Coasting"/"Parachute Descent" physics.

You may want to fix these before trusting the plotted trajectories and altitude sweep.
