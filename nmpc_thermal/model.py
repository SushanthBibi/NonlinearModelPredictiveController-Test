"""Nonlinear lumped-parameter thermal model of engine + cooling system.

States      x = [T_e, T_r, T_o]         engine/coolant, radiator, oil  [degC]
Inputs      u = [u_pump, u_fan, u_th]   pump speed, fan speed, thermostat valve  [0..1]
Disturbance d = [N, P_b, v, T_amb]      engine speed [rpm], brake power [W],
                                        vehicle speed [m/s], ambient [degC]

The same CasADi expressions are used by the NMPC (symbolically) and by the
simulated plant (numerically), so there is exactly one model definition.
"""
import casadi as ca

from .params import DEFAULT, ThermalParams

NX, NU, ND = 3, 3, 4
STATE_NAMES = ["T_e", "T_r", "T_o"]
INPUT_NAMES = ["u_pump", "u_fan", "u_th"]
DIST_NAMES = ["N", "P_b", "v", "T_amb"]


def _pos(x, eps=1e-4):
    """Smooth |x| floor so fractional powers stay real during solver iterations."""
    return ca.sqrt(x * x + eps * eps)


def smooth_max0(x, eps=1.0e3):
    """Differentiable max(x, 0); eps in the units of x (here W)."""
    return 0.5 * (x + ca.sqrt(x * x + eps * eps))


def _expressions(x, u, d, p: ThermalParams):
    T_e, T_r, T_o = x[0], x[1], x[2]
    u_pump, u_fan, u_th = u[0], u[1], u[2]
    N, P_b, v, T_amb = d[0], d[1], d[2], d[3]

    # Actuator power (affinity laws: power ~ speed^3)
    P_pump = p.P_pump_max * u_pump ** 3
    P_fan = p.P_fan_max * u_fan ** 3

    # Oil-temperature dependent friction
    phi = ca.exp(p.visc_B * (1.0 / (T_o + 273.15) - 1.0 / (p.T_oil_ref + 273.15)))
    P_fric = p.P_fric_ref * (N / 1000.0) ** 1.5 * (0.5 + 0.5 * phi)

    # Auxiliaries and friction are taken from the crankshaft -> extra fuel.
    # During engine braking (P_b < 0) they are covered by vehicle kinetic energy.
    P_ind = smooth_max0(P_b + P_fric + P_pump + P_fan)
    P_fuel = P_ind / p.eta_ind
    mdot_fuel = P_fuel / p.LHV

    # Heat sources
    Q_cool = p.f_cool_fuel * P_fuel + p.f_cool_fric * P_fric
    Q_oil = p.f_oil_fuel * P_fuel + (1.0 - p.f_cool_fric) * P_fric

    # Coolant flows: thermostat splits pump flow between radiator and bypass
    mdot = p.mdot_max * u_pump
    mdot_rad = mdot * u_th

    # Radiator: effectiveness on the air side, eps = 1 - exp(-NTU)
    mdot_air = p.rho_a * p.A_front * (p.k_ram * v + p.v_fan_max * u_fan) + 0.05
    hA_air = p.k_air * _pos(mdot_air) ** 0.8
    hA_cool = p.k_cool * _pos(mdot_rad, 1e-3) ** 0.8
    UA = 1.0 / (1.0 / hA_air + 1.0 / hA_cool)
    C_air = mdot_air * p.cp_a
    eff = 1.0 - ca.exp(-UA / C_air)
    Q_rad = eff * C_air * (T_r - T_amb)

    # Couplings
    hA_oc = p.hA_oc0 + p.hA_oc1 * u_pump
    Q_oil_to_cool = hA_oc * (T_o - T_e)
    Q_adv = mdot_rad * p.cp_c * (T_e - T_r)          # advected to radiator
    Q_loss = p.hA_amb * (T_e - T_amb)

    dT_e = (Q_cool + Q_oil_to_cool - Q_adv - Q_loss) / p.C_e
    dT_r = (Q_adv - Q_rad) / p.C_r
    dT_o = (Q_oil - Q_oil_to_cool) / p.C_o

    # Liner metal temperature: film resistance falls with coolant flow
    T_metal = T_e + Q_cool / (p.k_wall * _pos(mdot, 1e-3) ** 0.8)

    xdot = ca.vertcat(dT_e, dT_r, dT_o)
    outputs = dict(
        P_pump=P_pump, P_fan=P_fan, P_fric=P_fric, P_fuel=P_fuel,
        mdot_fuel=mdot_fuel, Q_cool=Q_cool, Q_oil=Q_oil, Q_rad=Q_rad,
        T_metal=T_metal, mdot=mdot, mdot_air=mdot_air,
    )
    return xdot, outputs


class ThermalModel:
    """CasADi functions for the dynamics, outputs and an RK4 discretisation."""

    OUTPUT_NAMES = [
        "P_pump", "P_fan", "P_fric", "P_fuel", "mdot_fuel",
        "Q_cool", "Q_oil", "Q_rad", "T_metal", "mdot", "mdot_air",
    ]

    def __init__(self, params: ThermalParams = DEFAULT):
        self.p = params
        x = ca.SX.sym("x", NX)
        u = ca.SX.sym("u", NU)
        d = ca.SX.sym("d", ND)
        dt = ca.SX.sym("dt")
        xdot, out = _expressions(x, u, d, params)
        self.f = ca.Function("f", [x, u, d], [xdot], ["x", "u", "d"], ["xdot"])
        self.h = ca.Function(
            "h", [x, u, d], [ca.vertcat(*[out[k] for k in self.OUTPUT_NAMES])],
            ["x", "u", "d"], ["y"],
        )
        k1 = self.f(x, u, d)
        k2 = self.f(x + dt / 2 * k1, u, d)
        k3 = self.f(x + dt / 2 * k2, u, d)
        k4 = self.f(x + dt * k3, u, d)
        self.rk4 = ca.Function(
            "rk4", [x, u, d, dt], [x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)],
            ["x", "u", "d", "dt"], ["xn"],
        )

    def step(self, x, u, d, dt, substeps=1):
        h = dt / substeps
        for _ in range(substeps):
            x = self.rk4(x, u, d, h)
        return x

    def outputs(self, x, u, d):
        y = self.h(x, u, d).full().ravel()
        return dict(zip(self.OUTPUT_NAMES, y))
