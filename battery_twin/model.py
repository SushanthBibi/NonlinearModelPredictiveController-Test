"""Electro-thermal battery-pack model.

States      x = [T_c, T_s, T_w, SOC]   cell core, cell surface, coolant [degC], SOC [-]
Inputs      u = [u_pump, u_chill, u_fan, u_rv, u_heat]   all in [0, 1]
Disturbance d = [P_drive, P_chg_max, v, T_amb]
            P_drive    traction + auxiliary power drawn from the pack [W] (<0 = regen)
            P_chg_max  charger capability [W] (0 while driving)
            v          vehicle speed [m/s] (ram air through the radiator)
            T_amb      ambient [degC]

Three variants share the same code:
  * physics  - the first-principles digital twin
  * truth    - physics + RealityGap (stands in for the real truck)
  * hybrid   - physics + learned residual (ML) on the measured nodes
"""
import casadi as ca

from .params import DEFAULT, GAP, PackParams, RealityGap

NX, NU, ND = 4, 5, 4
STATE_NAMES = ["T_c", "T_s", "T_w", "SOC"]
INPUT_NAMES = ["u_pump", "u_chill", "u_fan", "u_rv", "u_heat"]
DIST_NAMES = ["P_drive", "P_chg_max", "v", "T_amb"]
OUTPUT_NAMES = ["P_pump", "P_comp", "P_fan", "P_heat", "P_thermal", "P_chg", "P_pack",
                "Q_gen", "Q_chill", "Q_rad", "COP", "age_rate", "I"]
K0 = 273.15


def _pos(x, eps=1e-4):
    return ca.sqrt(x * x + eps * eps)


def _sig(z):
    return 1.0 / (1.0 + ca.exp(-z))


def _clip01(z, s=0.02):
    """Smooth clamp to [0, 1]."""
    lo = 0.5 * (z + ca.sqrt(z * z + s * s))
    return lo - 0.5 * ((lo - 1) + ca.sqrt((lo - 1) ** 2 + s * s))


def features(x, u, d):
    """Inputs of the ML residual model (all measurable, roughly unit-scaled)."""
    P_drive, P_chg_max, v, T_amb = d[0], d[1], d[2], d[3]
    return ca.vertcat(
        (x[1] - 30) / 10, (x[2] - 30) / 10, (T_amb - 20) / 15,
        u[0], u[1], u[2], u[3], u[4],
        _pos(P_drive, 1e3) / 2e5, P_chg_max / 5e5, v / 20,
    )


N_FEATURES = 11


def _expressions(x, u, d, p: PackParams, gap: RealityGap | None, residual):
    T_c, T_s, T_w, SOC = x[0], x[1], x[2], x[3]
    u_pump, u_chill, u_fan, u_rv, u_heat = u[0], u[1], u[2], u[3], u[4]
    P_drive, P_chg_max, v, T_amb = d[0], d[1], d[2], d[3]

    # --- thermal-system actuators ---
    P_pump = p.P_pump_max * u_pump ** 3
    P_fan = p.P_fan_max * u_fan ** 3
    Q_chill_cmd = p.Q_chill_max * u_chill
    T_ev = T_w - p.dT_evap + K0
    T_cd = T_amb + p.dT_cond + K0
    lift = _pos(T_cd - T_ev, 1.0) + 5.0
    COP = 1.0 + p.COP_eta * T_ev / lift            # >= 1, falls as ambient rises
    P_comp = Q_chill_cmd / COP
    Q_heat = p.Q_heat_max * u_heat
    P_heat = Q_heat
    P_thermal = P_pump + P_fan + P_comp + P_heat

    # --- BMS charge acceptance: SOC taper x temperature derate ---
    taper = _clip01((p.soc_full - SOC) / (p.soc_full - p.soc_taper))
    derate_hot = _sig(-(T_c - p.T_derate_hot) / 1.5)
    derate_cold = 0.15 + 0.85 * _sig((T_c - p.T_derate_cold) / 3.0)
    P_chg = P_chg_max * taper * derate_hot * derate_cold

    # --- electrical / heat generation ---
    P_pack = P_drive + P_thermal - P_chg           # >0 discharge
    V = p.V0 + p.V1 * SOC
    I = P_pack / V
    R = p.R_ref * ca.exp(p.B_R * (1 / (T_c + K0) - 1 / (25 + K0))) * (1 + 0.6 * (SOC - 0.5) ** 2)
    Q_gen = I * I * R

    # --- heat paths ---
    mdot = p.mdot_max * u_pump
    hA_sw = p.k_sw * _pos(mdot, 1e-3) ** 0.8 + p.hA_sw0
    Q_chill = Q_chill_cmd
    Q_leak = 0.0
    if gap is not None:
        hA_sw = hA_sw * gap.plate_factor * (1 - gap.plate_temp_coeff * (T_s - 25))
        Q_chill = Q_chill_cmd * (1 - gap.chiller_amb_coeff * _pos(T_amb - 25, 0.5))
        Q_leak = gap.edrive_leak * _pos(P_drive, 1e3)

    mdot_air = p.rho_a * p.A_front * (p.k_ram * v + p.v_fan_max * u_fan) + 0.05
    hA_air = p.k_air * _pos(mdot_air) ** 0.8
    hA_cool = p.k_cool_rad * _pos(mdot * u_rv, 1e-3) ** 0.8
    UA = 1 / (1 / hA_air + 1 / hA_cool)
    C_air = mdot_air * p.cp_a
    Q_rad = (1 - ca.exp(-UA / C_air)) * C_air * (T_w - T_amb)

    Q_cs = p.hA_cs * (T_c - T_s)
    Q_sw = hA_sw * (T_s - T_w)
    dT_c = (Q_gen - Q_cs) / p.C_c
    dT_s = (Q_cs - Q_sw - p.hA_env * (T_s - T_amb)) / p.C_s
    dT_w = (Q_sw - Q_chill - Q_rad + Q_heat + Q_leak) / p.C_w
    dSOC = -P_pack / p.E_cap
    if residual is not None:                       # learned correction [K/s]
        r = residual(features(x, u, d))
        dT_s = dT_s + r[0]
        dT_w = dT_w + r[1]

    age_rate = ca.exp(p.B_age * (1 / (25 + K0) - 1 / (T_c + K0)))

    xdot = ca.vertcat(dT_c, dT_s, dT_w, dSOC)
    out = ca.vertcat(P_pump, P_comp, P_fan, P_heat, P_thermal, P_chg, P_pack,
                     Q_gen, Q_chill, Q_rad, COP, age_rate, I)
    return xdot, out


class PackModel:
    """CasADi dynamics, outputs and RK4 step for one model variant."""

    def __init__(self, params: PackParams = DEFAULT, gap: RealityGap | None = None,
                 residual: ca.Function | None = None, name="physics"):
        self.p, self.gap, self.residual, self.name = params, gap, residual, name
        x, u, d = ca.SX.sym("x", NX), ca.SX.sym("u", NU), ca.SX.sym("d", ND)
        dt = ca.SX.sym("dt")
        xdot, out = _expressions(x, u, d, params, gap, residual)
        self.f = ca.Function("f", [x, u, d], [xdot])
        self.h = ca.Function("h", [x, u, d], [out])
        k1 = self.f(x, u, d)
        k2 = self.f(x + dt / 2 * k1, u, d)
        k3 = self.f(x + dt / 2 * k2, u, d)
        k4 = self.f(x + dt * k3, u, d)
        self.rk4 = ca.Function("rk4", [x, u, d, dt], [x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])

    def step(self, x, u, d, dt, substeps=1):
        h = dt / substeps
        for _ in range(substeps):
            x = self.rk4(x, u, d, h)
        return x

    def outputs(self, x, u, d):
        return dict(zip(OUTPUT_NAMES, self.h(x, u, d).full().ravel()))


def physics(params=DEFAULT):
    return PackModel(params, name="physics")


def truth(params=DEFAULT, gap=GAP):
    return PackModel(params, gap=gap, name="truth")


def hybrid(residual, params=DEFAULT):
    return PackModel(params, residual=residual, name="hybrid")
