"""Physical parameters of the lumped thermal model.

All values are first-cut engineering estimates for a ~13 L, ~400 kW
heavy-duty diesel in a 40 t tractor-trailer. They are placeholders until the
model is calibrated against measured data (see identification.py).
SI units throughout, temperatures in degC, powers in W.
"""
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class ThermalParams:
    # --- thermal capacities [J/K] ---
    C_e: float = 4.0e5      # engine block + coolant in the engine jacket
    C_r: float = 0.8e5      # radiator core + coolant in the radiator
    C_o: float = 0.9e5      # engine oil + sump

    # --- fluids ---
    cp_c: float = 3600.0    # coolant (50/50 glycol) specific heat [J/kg/K]
    cp_a: float = 1006.0    # air specific heat [J/kg/K]
    rho_a: float = 1.16     # air density [kg/m^3]

    # --- coolant pump (electric / variable) ---
    mdot_max: float = 6.0   # coolant mass flow at full pump speed [kg/s]
    P_pump_max: float = 3.0e3   # hydraulic+drive power at full speed [W]

    # --- cooling fan ---
    A_front: float = 1.2    # radiator frontal area [m^2]
    k_ram: float = 0.30     # fraction of vehicle speed seen as ram air through the core
    v_fan_max: float = 12.0  # equivalent face velocity from the fan at full speed [m/s]
    P_fan_max: float = 30.0e3   # fan shaft power at full speed [W]

    # --- radiator heat transfer: hA = k * mdot^0.8 (Dittus-Boelter scaling) ---
    k_air: float = 728.0
    k_cool: float = 2864.0

    # --- engine <-> oil, engine <-> ambient ---
    hA_oc0: float = 1500.0  # oil-cooler conductance at zero pump flow [W/K]
    hA_oc1: float = 1000.0  # extra conductance at full pump flow [W/K]
    hA_amb: float = 100.0   # engine block natural losses [W/K]

    # --- in-cylinder wall-side film, used for the metal-temperature constraint ---
    k_wall: float = 3000.0  # hA_wall = k_wall * mdot^0.8

    # --- engine energy split ---
    eta_ind: float = 0.47   # indicated efficiency
    f_cool_fuel: float = 0.24   # share of fuel power rejected to coolant
    f_oil_fuel: float = 0.03    # share of fuel power rejected to oil (piston jets)
    f_cool_fric: float = 0.30   # share of friction power into coolant (rest -> oil)
    LHV: float = 42.8e6     # diesel lower heating value [J/kg]

    # --- friction: P_fric = P_fric_ref*(N/1000)^1.5 * (0.5 + 0.5*phi(T_oil)) ---
    P_fric_ref: float = 6.5e3   # friction power at 1000 rpm with hot oil [W]
    visc_B: float = 1500.0      # Arrhenius-like viscosity sensitivity [K]
    T_oil_ref: float = 100.0    # oil temperature where phi = 1 [degC]

    # --- operating limits ---
    T_e_max: float = 102.0      # coolant (engine outlet) upper limit [degC]
    T_o_max: float = 125.0      # oil upper limit [degC]
    T_metal_max: float = 130.0  # cylinder-liner metal temperature limit [degC]
    u_pump_min: float = 0.12    # minimum pump speed (flow through EGR cooler etc.)

    def perturbed(self, **kw) -> "ThermalParams":
        return replace(self, **kw)


DEFAULT = ThermalParams()
