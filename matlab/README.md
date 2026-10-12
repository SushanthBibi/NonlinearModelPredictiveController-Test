# MATLAB / Simulink versions

Two self-contained folders, each with MATLAB code and a script that **builds the
Simulink model for you** (no manual block dragging):

| Folder | System | Baseline | Predictive controller |
|---|---|---|---|
| `diesel_engine/` | Diesel engine cooling: engine/coolant, radiator, oil | Wax thermostat, crank-driven pump, on/off viscous fan | NMPC with route preview (fmincon) |
| `electric_battery/` | E-truck battery pack and thermal loop: cell core, cell surface, coolant, SOC | Rule-based BMS (threshold relays) | Digital twin (EKF virtual core sensor) + NMPC with route preview (fmincon) |

**Requirements:** MATLAB R2021a or newer, Simulink, and the Optimization Toolbox (`fmincon`).

## Quick start

```matlab
cd matlab/diesel_engine
run_diesel_comparison          % builds diesel_cooling_nmpc.slx, runs both controllers, prints KPIs, plots

cd ../electric_battery
scenario = 'summer';           % or 'mild' / 'winter'
run_battery_comparison         % builds battery_thermal_twin.slx, runs both controllers, KPIs, plots
```

To only open and explore a model:

```matlab
setup_diesel;  build_diesel_model;      % or: setup_battery; build_battery_model;
```

Then press **Run** in Simulink. `ctrl_mode = 1` selects the conventional
controller and `ctrl_mode = 2` the NMPC. Double-click any subsystem to see its blocks.

## How the models are drawn

The physics is drawn with basic blocks in the same style as a textbook
mass-spring diagram. Each temperature is a heat balance:

```
heat flows --> [Sum] --> [Gain 1/C] --> [Integrator] --> T  --> Scope
                  ^                                      |
                  +----------- feedback -----------------+
```

* **Sum** blocks add the heat flows into each node (heat generated, heat to the
  radiator, heat to the oil cooler, losses, ...).
* **Gain** `1/C` turns heat flow [W] into a temperature rate [K/s].
* **Integrator** blocks hold the states (temperatures, SOC). Their initial
  condition is `x0`.
* **Product / Subtract** blocks build the transfer terms, for example
  `mdot_rad × cp × (T_e − T_r)`.
* **Fcn** blocks hold the nonlinear correlations: radiator effectiveness,
  oil-viscosity friction, actuator power `∝ speed³`, chiller COP, battery
  resistance `R(T, SOC)`, and charge derating.
* **Scopes** are on every state and the main power flows. **To Workspace**
  blocks log the data that the run scripts use for KPIs.

The baseline controllers are drawn the same way. They use Relay blocks with
switch-on/off points for the fan, chiller and heater hysteresis, a Saturation
block for the wax thermostat, and logic blocks for the BMS pump and radiator rules.

The **NMPC** is a MATLAB Function block, sampled by Zero-Order Holds, that
calls the optimiser in `diesel_nmpc_step.m` / `battery_twin_step.m`. Inside,
`fmincon` (SQP) simulates the same equations from `diesel_model.m` /
`battery_model.m` over the horizon and picks the actuator moves.

## Files

| Diesel | Electric | Purpose |
|---|---|---|
| `diesel_params.m` | `battery_params.m` | Parameters (engineering estimates); battery also has the "reality gap" |
| `diesel_drive_cycle.m` | `battery_mission.m` | Route: speed, grade, power demand, charger |
| `diesel_model.m`, `diesel_rk4.m` | `battery_model.m`, `battery_rk4.m` | Same equations as the Simulink plant (NMPC prediction model) |
| `diesel_nmpc_step.m` | `battery_twin_step.m` | NMPC (and EKF for the battery) |
| `setup_diesel.m` | `setup_battery.m` | Puts every parameter in the workspace for the blocks |
| `build_diesel_model.m` | `build_battery_model.m` | Builds the `.slx` with blocks and connections |
| `run_diesel_comparison.m` | `run_battery_comparison.m` | Runs both controllers, prints KPIs, plots |

## Notes

* **Not run in MATLAB yet.** These files were written without access to
  MATLAB. They pass a static syntax check (MISS_HIT), but the first build may
  need small fixes, for example a block parameter name that differs in your
  release. If `build_*_model` stops with an error, the message names the block
  and parameter.
* **Simulation time:** the diesel NMPC run takes a few minutes; the battery
  run is longer because it does one fmincon solve per simulated minute over 2.5
  hours. Lower `nmpc_cfg.max_iter` / `twin_cfg.max_iter` for faster, rougher runs.
* **The battery plant includes the reality gap** (aged cold plates, chiller
  loss in heat, e-drive heat leak), which the twin does not know about. That is
  why the EKF is needed. Set `use_reality_gap = false;` before `setup_battery`
  to remove it.
* **Same equations as the Python version** in this repository, so results
  should be comparable with `nmpc_thermal/` and `battery_twin/`.
