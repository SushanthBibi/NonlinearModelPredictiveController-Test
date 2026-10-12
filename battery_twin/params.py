"""Parameters for a ~540 kWh heavy-duty e-truck battery pack and its thermal loop.

Pack-level lumped values, engineering estimates (placeholders until calibrated).
SI units, temperatures in degC, powers in W.
"""
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class PackParams:
    # --- electrical ---
    E_cap: float = 540e3 * 3600     # usable energy [J]
    V0: float = 560.0               # open-circuit voltage at SOC=0 [V]
    V1: float = 180.0               # OCV slope [V per unit SOC]
    R_ref: float = 0.035            # pack resistance at 25 degC, SOC 0.5 [ohm]
    B_R: float = 3500.0             # Arrhenius sensitivity of resistance [K]

    # --- thermal capacities [J/K] ---
    C_c: float = 2.4e6              # cell cores
    C_s: float = 0.6e6              # cell surfaces + module structure + cold plates
    C_w: float = 0.3e6              # coolant loop

    # --- conductances [W/K] ---
    hA_cs: float = 4000.0           # core -> surface
    k_sw: float = 1720.0            # surface -> coolant: k_sw * mdot^0.8 + hA_sw0
    hA_sw0: float = 150.0
    hA_env: float = 60.0            # pack enclosure -> ambient

    # --- coolant pump ---
    mdot_max: float = 2.0           # [kg/s]
    P_pump_max: float = 0.8e3
    u_pump_min: float = 0.05
    cp_w: float = 3600.0

    # --- chiller (refrigerant loop) ---
    Q_chill_max: float = 30e3       # cooling capacity [W]
    COP_eta: float = 0.40           # fraction of Carnot COP
    dT_evap: float = 8.0            # evaporator approach [K]
    dT_cond: float = 12.0           # condenser approach [K]

    # --- low-temperature radiator + fan ---
    A_front: float = 0.6
    k_ram: float = 0.25
    v_fan_max: float = 10.0
    P_fan_max: float = 2.5e3
    rho_a: float = 1.16
    cp_a: float = 1006.0
    k_air: float = 500.0
    k_cool_rad: float = 2500.0

    # --- PTC heater ---
    Q_heat_max: float = 12e3

    # --- BMS charging behaviour ---
    soc_taper: float = 0.80         # charge power tapers linearly above this SOC
    soc_full: float = 0.97
    T_derate_hot: float = 43.0      # 50 % charge-power derate point (hot)
    T_derate_cold: float = 12.0     # 50 % derate point (cold, lithium plating)

    # --- limits / targets ---
    T_max: float = 50.0             # absolute core limit
    T_band_hi: float = 38.0         # preferred operating window
    T_band_lo: float = 18.0

    # --- ageing (Arrhenius, normalised to 1 at 25 degC) ---
    B_age: float = 3700.0

    def perturbed(self, **kw) -> "PackParams":
        return replace(self, **kw)


@dataclass(frozen=True)
class RealityGap:
    """Effects present in the 'real' pack that the physics twin does not model.

    They act on the measured nodes (cell surface, coolant), so a data-driven
    residual model can learn them from sensor logs.
    """
    plate_factor: float = 0.75       # cold-plate conductance is 25 % lower (TIM ageing)
    plate_temp_coeff: float = 0.006  # ... and falls a further 0.6 %/K above 25 degC
    chiller_amb_coeff: float = 0.012  # chiller capacity drops 1.2 %/K above 25 degC ambient
    edrive_leak: float = 0.012       # 1.2 % of traction power leaks into the battery loop


DEFAULT = PackParams()
GAP = RealityGap()
