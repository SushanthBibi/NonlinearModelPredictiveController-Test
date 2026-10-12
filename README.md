# Predictive Thermal Management for Heavy-Duty Trucks (NMPC)

This repository holds two applications of the same NMPC approach:

1. **`battery_twin/`: AI-enabled digital twin for an electric truck's battery**
   (entry for the VIT × ISHRAE *AI for Thermal Management: Digital Twin Challenge*,
   application area *Battery Thermal Management*). See the next section.
2. **`nmpc_thermal/`: diesel engine cooling NMPC** (the original Volvo project). See
   [Diesel engine cooling NMPC](#diesel-engine-cooling-nmpc) below.

## Digital Twin Challenge: e-truck battery thermal twin

Open `demo/index.html` in any browser for the interactive demo. It works offline.

The challenge pipeline, as implemented:

| Stage | Implementation |
|---|---|
| Data from sensors | `battery_twin/model.py` `truth()`: a 540 kWh pack with effects the physics twin does not model (cold-plate ageing, chiller capacity loss in heat, heat leak from the e-drive), sampled with sensor noise |
| Digital twin model | `physics()`: four-state electro-thermal model (cell core, cell surface, coolant, SOC), plus an EKF (`estimator.py`) that keeps it synchronised with the sensors and estimates the **unmeasured cell-core temperature** |
| AI/ML prediction | `ml_residual.py`: a neural network trained on fleet-style logs learns the physics residual. It is rebuilt as a CasADi expression, so the NMPC optimises through the **hybrid physics + ML twin** |
| Optimisation & control | `controllers.py` `TwinNMPC`: 40-min horizon over the route preview. Minimises cooling/heating energy and ageing, maximises charge accepted, under a soft temperature window and a hard limit. Five actuators: pump, chiller, radiator valve, fan, PTC heater |
| Efficient operation | KPIs vs a rule-based BMS: thermal-system energy, energy accepted in a 30-min megawatt-charging stop, peak temperature, Arrhenius ageing index |

Mission: a 40 t truck drives 80 min (with climbs), stops for 30 min at a 750 kW
charger during the driver's break, then drives 40 min.

```bash
pip install -r requirements.txt
python scripts/twin_train.py     # trains the ML residual (~30 s)
python scripts/twin_run_all.py   # 3 climates x 3 controllers, rebuilds demo/index.html (~8 min)
```

**Results** (simulated; change against the rule-based BMS):

| Scenario | Controller | Thermal energy | Charged in 30 min | Peak core | Ageing index |
|---|---|---|---|---|---|
| Summer 42 °C | Rule-based BMS | 10.00 kWh | 349.4 kWh | 39.3 °C | 1.51 |
| | NMPC + physics twin | 8.07 kWh (−19 %) | 361.6 kWh (+3.5 %) | 38.0 °C | 1.60 |
| | NMPC + AI hybrid twin | 8.41 kWh (−16 %) | 361.7 kWh (+3.5 %) | 38.0 °C | 1.61 |
| Mild 22 °C | Rule-based BMS | 2.29 kWh | 355.0 kWh | 39.0 °C | 1.27 |
| | NMPC + AI hybrid twin | 1.44 kWh (−37 %) | 359.9 kWh (+1.4 %) | 37.7 °C | 1.26 |
| Winter −5 °C | Rule-based BMS | 7.99 kWh | 362.7 kWh | 34.8 °C | 0.90 |
| | NMPC + AI hybrid twin | 6.86 kWh (−14 %) | 362.4 kWh | 34.1 °C | 0.86 |

AI twin accuracy, as 60-minute free-run prediction RMSE on unseen data: surface
0.93 → 0.50 K, coolant 1.17 → 0.42 K. The core-temperature virtual sensor is
accurate to 0.2–0.5 K.

How to read this:

* **The NMPC gains** by pre-cooling before the charging stop and running the chiller
  at partial load instead of in on/off bursts. In summer the pack accepts
  12 kWh more in the same 30 minutes, using less cooling energy.
* **The AI makes the twin 2–3× more accurate.** In closed loop its effect on the
  KPIs is small here, because the EKF already corrects most of the model drift.
  It matters more as the gap between model and pack grows.
* **Trade-off:** in summer the NMPC runs the pack slightly warmer, so its ageing
  index is about 6 % higher. `Weights.w_age` sets this balance.
* **Solve time:** about 0.3 s per step for the physics twin and about 1 s for the
  hybrid twin, against a 60 s control interval.
* **All data is simulated** and the parameters are estimates. Real fleet or
  test-rig data is the next step.

---

# Diesel engine cooling NMPC

Volvo project: *Sustainable, Compact and Lightweight Thermal Management System for
Heavy-Duty Trucks to Improve Fuel Economy and Reduce CO2 Emissions*.
Sushanth Bibi, Ganishk Mohanraj, Saradia Mathur.

This repository is a working first version of the three proposed deliverables:

| Deliverable | Where |
|---|---|
| 1. Nonlinear thermal model + calibration/validation (RMSE, R²) | `nmpc_thermal/model.py`, `nmpc_thermal/identification.py`, `scripts/validate_model.py` |
| 2. NMPC controller | `nmpc_thermal/controllers.py` (`NMPC`) |
| 3. Simulation / validation platform, comparison with conventional control | `nmpc_thermal/simulate.py`, `nmpc_thermal/metrics.py`, `scripts/run_comparison.py` |

> **Status: all parameters are engineering estimates, not measured data.** The
> model structure is physically based, but until it is calibrated against
> test-cell or vehicle data (step 4 of the methodology) the numbers below show
> *how the controller behaves*, not how much fuel a real Volvo truck will save.

## Quick start

```bash
pip install -r requirements.txt
python scripts/run_comparison.py               # 25 degC cold start, nominal model
python scripts/run_comparison.py --ambient 38  # hot day
python scripts/run_comparison.py --mismatch --noise 0.3  # plant != NMPC model, noisy sensors
python scripts/validate_model.py               # synthetic self-check of the calibration pipeline
python -m pytest
```

Results (KPI JSON + plots) are written to `results/`.

## 1. System and variables

Mapped from the problem statement:

| Role | Variables |
|---|---|
| **States** (measured) | coolant/engine temperature `T_e`, radiator outlet `T_r`, oil `T_o` |
| **Manipulated** | coolant-pump speed `u_pump`, cooling-fan speed `u_fan`, thermostat (valve) position `u_th`, all in [0, 1] |
| **Measured disturbances** | engine speed `N`, brake power `P_b` (from load/torque), vehicle speed `v`, ambient `T_amb` |
| **Preview** | predicted `N`, `P_b`, `v` over the horizon (route + speed plan) |
| **Outputs / KPIs** | coolant & oil stability, peak temperatures, fan & pump energy, fuel, warm-up time |

The NMPC assumes **electrified/variable actuators**: an electric pump, a
variable-speed fan and an electronically controlled thermostat valve. Intake-air
temperature is not yet modelled (charge-air cooler is a natural extension).

## 2. Nonlinear thermal model (`nmpc_thermal/model.py`)

Three lumped thermal masses:

```
C_e dT_e/dt = Q_cool + hA_oc(u_pump)(T_o - T_e) - m_rad c_p (T_e - T_r) - hA_amb (T_e - T_amb)
C_r dT_r/dt = m_rad c_p (T_e - T_r) - Q_rad
C_o dT_o/dt = Q_oil - hA_oc(u_pump)(T_o - T_e)
```

The nonlinearities that make an NMPC worthwhile:

* **Flows**: `m_dot = m_max u_pump`, radiator flow `m_rad = m_dot u_th` (rest bypasses).
* **Radiator**: air-side effectiveness `Q_rad = (1 - exp(-UA/C_air)) C_air (T_r - T_amb)`,
  with `UA` from series air/coolant films that scale with `m^0.8`.
  Air flow = ram air (`k_ram v`) + fan (`v_fan_max u_fan`).
* **Actuator power** follows the fan/pump affinity laws, `P ∝ speed³`, so half
  fan speed costs one eighth of full power.
* **Oil-temperature-dependent friction**: viscosity factor
  `phi = exp(B (1/T_o - 1/T_ref))`. Cold oil means more friction, so more fuel. This is
  what makes fast warm-up and running warm worth fuel.
* **Fuel**: `P_fuel = max(P_b + P_fric + P_fan + P_pump, 0) / eta_ind` (smooth max).
  Fan and pump are driven from the crank, so they cost fuel, except under engine
  braking, where they are free.
* **Heat release**: fixed fractions of fuel power to coolant and oil, plus friction heat.
* **Liner metal temperature** `T_metal = T_e + Q_cool / (k_wall m_dot^0.8)`. Low pump
  flow at high load overheats the liner even if bulk coolant looks fine. This
  is what forces the pump up under load.

The same CasADi expressions are used symbolically by the NMPC and numerically
by the simulated plant (RK4), so there is one model definition.

## 3. NMPC formulation (`nmpc_thermal/controllers.py`)

Multiple shooting, horizon `Np = 24` steps × `dt = 5 s` = 2 min, solved with IPOPT
every 5 s (warm-started from the shifted previous solution).

```
min   Σ_k  w_T (T_e - 92)² + w_To (T_o - 100)²
         + w_fuel (P_fric + P_fan + P_pump)/eta_ind        <- cooling-attributable fuel [kW]
         + Δuᵀ W Δu + w_s |s|²
s.t.  x_{k+1} = RK4(x_k, u_k, d_k) + dt·b                 (b = disturbance estimate)
      u_pump ∈ [0.12, 1],  u_fan, u_th ∈ [0, 1]
      T_e ≤ 102 - margin + s₁,  T_o ≤ 125 - margin + s₂
      T_metal(x, u, d_peak) ≤ 130 - margin + s₃            (soft constraints, slack s ≥ 0)
```

Design choices worth knowing:

* **Preview** (`NMPC.preview`): the disturbance trajectory comes from the
  route/speed plan. Dynamics use the interval-mean load. The metal-temperature
  constraint uses the *peak* load inside each interval, at both ends of the
  interval. Without this, fast load ramps between 5 s samples slipped past the
  constraint.
* **Offset-free MPC**: a constant additive disturbance `b` (K/s per state) is
  integrated from the one-step prediction error. This is what keeps the
  controller inside its limits when the real plant differs from its model.
* **Soft constraints** keep the problem feasible when no input can avoid a
  violation (e.g. a long climb at 45 °C ambient).

## 4. Baselines

* **Conventional**: wax thermostat (opens 82 → 95 °C), crank-driven pump
  (speed ∝ engine rpm), on/off viscous fan (on at 97 °C, off at 92 °C).
* **PI**: the *same variable actuators* as the NMPC, with PI loops on coolant
  temperature and the pump scheduled on load, but **no preview**. This separates
  the gain from electrifying the actuators from the gain from prediction.

## 5. Results on the synthetic 36-min mission

Mission: 40 t truck, cold start, urban, highway, a 4.5 % climb (5 min), descent,
highway, a 5 % climb (4 min), highway, stop.

**25 °C, nominal model**

| KPI | Conventional | PI | NMPC |
|---|---|---|---|
| Warm-up to 85 °C [s] | 722 | 552 | 553 |
| Coolant RMS dev. from 92 °C [K] | 5.98 | 2.31 | 3.91 |
| Peak coolant [°C] | 97.5 | 97.3 | 101.3 |
| Peak liner metal [°C] (limit 130) | 120.3 | 113.1 | 128.6 |
| Fan + pump energy [kWh] | 4.22 | 1.39 | 0.51 |
| Fuel [kg] | 17.65 | 17.14 | 16.98 |

**38 °C ambient**

| KPI | Conventional | PI | NMPC |
|---|---|---|---|
| Time above coolant limit [s] | 265 | 0 | 0 |
| Peak coolant [°C] | 105.9 | 100.8 | 101.4 |
| Fan + pump energy [kWh] | 5.15 | 4.59 | 2.03 |
| Fuel [kg] | 17.74 | 17.65 | 17.22 |

**Model mismatch** (plant radiator 15 % weaker and 10 % more heat to coolant than
the NMPC assumes, 0.3 K sensor noise)

| KPI | Conventional | PI | NMPC |
|---|---|---|---|
| Time above coolant limit [s] | 148 | 0 | 0 |
| Peak liner metal [°C] (limit 130) | 129.9 | 118.3 | **133.2** |
| Fan + pump energy [kWh] | 4.90 | 4.33 | 1.81 |
| Fuel [kg] | 17.73 | 17.64 | 17.20 |

How to read this:

* **Where the NMPC wins is in using the preview.** Before each climb it starts
  the fan early and gently (about 0.4 instead of 1.0). It uses the engine's
  thermal mass as a buffer, letting coolant rise toward the limit only by the
  end of the climb. Under the cubic fan law that is far cheaper than reacting
  late with full fan (see `results/comparison_*.png`).
* **Most of the saving against the conventional system comes from
  variable actuators** (PI column). Prediction adds roughly another 1 % of fuel
  at 25 °C and about 2.4 % at 38 °C, where cooling demand is high.
* **Trade-off**: the economic NMPC deliberately runs the coolant closer to its
  limit, so its temperature *tracking* is looser than PI's. `w_T` and `w_fuel` in
  `NMPCWeights` move this trade-off.
* **Known gap**: under model mismatch the liner-metal temperature overshoots its
  limit by about 3 K. The bias estimator corrects bulk temperatures but only
  approximately infers the extra heat behind the film temperature drop. Fixes:
  estimate a heat-rejection scale factor online (EKF / moving-horizon
  estimation), or add a liner thermocouple to the measurements.
* The fuel figures include only cooling-system effects (actuator power and
  oil-temperature friction) on a 36-min synthetic cycle. Treat the percentages
  as indicative until the model is calibrated.

## 6. Model validation workflow (`scripts/validate_model.py`)

1. Log test-cell or vehicle data as CSV with columns
   `t,T_e,T_r,T_o,u_pump,u_fan,u_th,N,P_b,v,T_amb`, one run for identification
   and a different run for validation.
2. `python scripts/validate_model.py --ident ident.csv --valid valid.csv`
   fits scale factors on `C_e, C_r, C_o, k_air, f_cool_fuel, hA_oc0` by least
   squares on the open-loop simulated temperatures, then reports RMSE and R² on
   the validation run.

With no CSV given, the script runs a **synthetic self-check**. Data come from a
deliberately different "true" model plus noise, and identification recovers
it: R² rises from 0.85 to 0.999 on an unseen cycle at a different ambient.
That only shows the pipeline works. It is not a validation against a real engine.

## 7. Suggested next steps

1. Get real data (Volvo test cell, or published D13 maps) and calibrate; replace
   the synthetic cycle with a measured or VECTO long-haul cycle.
2. Add a state/parameter estimator (EKF or MHE) to close the metal-temperature
   gap and handle unmeasured `T_r`.
3. Add the charge-air cooler (intake-air temperature) and the EGR cooler.
4. Study preview quality: noisy or shortened speed/load forecasts, and
   horizon length against solve time (currently about 60 ms mean, 220 ms worst).
5. Real-time implementation: acados (SQP-RTI) or CasADi code generation for
   an ECU-class target.

## Repository layout

```
battery_twin/        e-truck battery digital twin (model, EKF, ML residual, NMPC)
demo/                interactive dashboard (index.html works offline)
scripts/twin_*.py    train the ML twin, run all scenarios
scripts/build_demo.py  rebuild the dashboard from results/twin/
nmpc_thermal/
  params.py          physical parameters (placeholders until calibrated)
  model.py           nonlinear thermal model (CasADi), RK4 discretisation
  drive_cycle.py     synthetic mission -> engine speed / brake power
  controllers.py     Conventional, PI and NMPC controllers
  simulate.py        closed-loop simulator
  metrics.py         KPIs from the problem statement
  identification.py  calibration (least squares) and RMSE / R² validation
  plotting.py
scripts/
  run_comparison.py  conventional vs PI vs NMPC
  validate_model.py  calibration + validation
tests/               pytest suite
results/             KPI JSON and plots from the last runs
```

## References (from the problem statement)

* NMPC thermal management for fuel-cell heavy-duty trucks: https://www.mdpi.com/1996-1073/19/13/3123
* MPC for auxiliary fuel reduction in 40 t / 60 t trucks: https://www.sciencedirect.com/science/article/abs/pii/S0306261916317494
* https://www.mdpi.com/1996-1073/12/2/259, https://www.sciencedirect.com/science/article/pii/S2405896318325357, https://www.sciencedirect.com/science/article/pii/S147466701536818X
