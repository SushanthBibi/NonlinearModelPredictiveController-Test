"""Conventional (reactive) thermal controller and the NMPC."""
from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from .drive_cycle import disturbance
from .model import ND, NU, NX, ThermalModel
from .params import DEFAULT, ThermalParams


class ConventionalController:
    """Wax thermostat + engine-driven pump + on/off viscous fan.

    This mirrors a production cooling system: nothing is predictive, the pump
    turns with the crankshaft and the fan engages on a coolant-temperature
    threshold with hysteresis.
    """

    def __init__(self, T_open=82.0, T_full=95.0, fan_on=97.0, fan_off=92.0,
                 N_pump_full=1800.0, p: ThermalParams = DEFAULT):
        self.T_open, self.T_full = T_open, T_full
        self.fan_on, self.fan_off = fan_on, fan_off
        self.N_pump_full = N_pump_full
        self.p = p
        self.fan_state = False

    def __call__(self, x, k, cycle):
        T_e = x[0]
        N = cycle["N"][k]
        u_th = np.clip((T_e - self.T_open) / (self.T_full - self.T_open), 0.0, 1.0)
        u_pump = np.clip(N / self.N_pump_full, self.p.u_pump_min, 1.0)
        if T_e >= self.fan_on:
            self.fan_state = True
        elif T_e <= self.fan_off:
            self.fan_state = False
        return np.array([u_pump, 1.0 if self.fan_state else 0.0, u_th])


class PIController:
    """Reactive controller on the same variable actuators the NMPC uses.

    Valve and fan are PI loops on coolant temperature (fan engages only once
    the valve is saturated); the pump is scheduled on current brake power so
    the liner is protected. No preview - this isolates the benefit of
    prediction from the benefit of having variable-speed actuators.
    """

    def __init__(self, T_ref=92.0, T_fan=95.0, kp_th=0.15, ki_th=0.004,
                 kp_fan=0.12, ki_fan=0.003, dt=1.0, p: ThermalParams = DEFAULT,
                 P_rated=380e3):
        self.T_ref, self.T_fan = T_ref, T_fan
        self.kp_th, self.ki_th, self.kp_fan, self.ki_fan = kp_th, ki_th, kp_fan, ki_fan
        self.dt, self.p, self.P_rated = dt, p, P_rated
        self.i_th = 0.0
        self.i_fan = 0.0

    def __call__(self, x, k, cycle):
        T_e = x[0]
        e_th = T_e - self.T_ref
        u_th_raw = self.kp_th * e_th + self.i_th
        u_th = float(np.clip(u_th_raw, 0.0, 1.0))
        if u_th_raw == u_th:  # anti-windup: integrate only when unsaturated
            self.i_th += self.ki_th * e_th * self.dt

        e_fan = T_e - self.T_fan
        u_fan_raw = self.kp_fan * e_fan + self.i_fan
        u_fan = float(np.clip(u_fan_raw, 0.0, 1.0))
        if u_fan_raw == u_fan or (u_fan_raw < 0 and e_fan > 0) or (u_fan_raw > 1 and e_fan < 0):
            self.i_fan += self.ki_fan * e_fan * self.dt
        self.i_fan = min(max(self.i_fan, -0.2), 1.0)

        load = max(cycle["P_b"][k], 0.0) / self.P_rated
        u_pump = float(np.clip(0.25 + 0.8 * load, self.p.u_pump_min, 1.0))
        if T_e < self.T_ref - 10.0:
            u_pump = self.p.u_pump_min + 0.6 * load  # warm-up: low flow unless loaded
        return np.array([u_pump, u_fan, u_th])


@dataclass
class NMPCWeights:
    T_ref: float = 92.0       # coolant target [degC]
    w_T: float = 0.05         # coolant tracking  [per K^2]
    w_To: float = 0.01        # oil tracking around T_o_ref
    T_o_ref: float = 100.0
    w_fuel: float = 1.0       # cooling-attributable fuel power [per kW]
    w_du: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    w_slack: float = 1.0e3    # soft-constraint violation [per K^2]


class NMPC:
    """Multiple-shooting NMPC solved with IPOPT.

    Decision variables: state trajectory X (NX x N+1), inputs U (NU x N),
    slacks S (3 x N) on the temperature limits. The disturbance preview over
    the horizon (engine speed, brake power, vehicle speed, ambient) is a
    parameter, which is what makes the controller predictive.

    Cost per stage:
        w_T  (T_e - T_ref)^2 + w_To (T_o - T_o_ref)^2
      + w_fuel * (P_fric + P_fan + P_pump)/eta_ind    [kW of fuel]
      + du' W du + w_slack |s|^2
    """

    def __init__(self, model: ThermalModel, horizon=24, dt=5.0, substeps=1,
                 weights: NMPCWeights | None = None, ipopt_opts=None,
                 offset_free=True, bias_gain=0.3, margin=1.0):
        self.model, self.p = model, model.p
        self.Np, self.dt, self.substeps = horizon, dt, substeps
        self.w = weights or NMPCWeights()
        self.offset_free, self.bias_gain = offset_free, bias_gain
        self.margin = margin  # back-off from the hard limits [K]
        self._build(ipopt_opts or {})
        self._last = None
        self.bias = np.zeros(NX)   # estimated unmodelled heating rate [K/s]
        self._x_pred = None

    def update_bias(self, x_meas):
        """Offset-free MPC: integrate the one-step prediction error into a
        constant additive disturbance on the state derivatives."""
        if self.offset_free and self._x_pred is not None:
            e = np.asarray(x_meas) - self._x_pred
            self.bias = np.clip(self.bias + self.bias_gain * e / self.dt, -0.05, 0.05)

    def _build(self, ipopt_opts):
        p, w, Np = self.p, self.w, self.Np
        opti = ca.Opti()
        X = opti.variable(NX, Np + 1)
        U = opti.variable(NU, Np)
        S = opti.variable(3, Np)
        x0 = opti.parameter(NX)
        D = opti.parameter(ND, Np)      # interval-mean disturbance (dynamics)
        Dpk = opti.parameter(ND, Np)    # interval-peak disturbance (constraints)
        u_prev = opti.parameter(NU)
        b = opti.parameter(NX)          # disturbance estimate [K/s]

        J = 0
        opti.subject_to(X[:, 0] == x0)
        for k in range(Np):
            xk, uk, dk = X[:, k], U[:, k], D[:, k]
            xn = xk
            for _ in range(self.substeps):
                xn = self.model.rk4(xn, uk, dk, self.dt / self.substeps)
            opti.subject_to(X[:, k + 1] == xn + self.dt * b)

            y = self.model.h(X[:, k + 1], uk, dk)
            P_pump, P_fan, P_fric = y[0], y[1], y[2]
            fuel_kW = (P_fric + P_fan + P_pump) / p.eta_ind / 1e3

            du = uk - (u_prev if k == 0 else U[:, k - 1])
            J += (w.w_T * (X[0, k + 1] - w.T_ref) ** 2
                  + w.w_To * (X[2, k + 1] - w.T_o_ref) ** 2
                  + w.w_fuel * fuel_kW
                  + ca.dot(ca.DM(w.w_du), du * du) * 10.0
                  + w.w_slack * ca.sumsqr(S[:, k]))

            opti.subject_to(opti.bounded(p.u_pump_min, uk[0], 1.0))
            opti.subject_to(opti.bounded(0.0, uk[1], 1.0))
            opti.subject_to(opti.bounded(0.0, uk[2], 1.0))
            opti.subject_to(S[:, k] >= 0)
            mg = self.margin
            opti.subject_to(X[0, k + 1] <= p.T_e_max - mg + S[0, k])
            opti.subject_to(X[2, k + 1] <= p.T_o_max - mg + S[1, k])
            # Metal limit is checked against the worst load inside the interval,
            # at both ends, so fast load ramps between samples are covered.
            # Unmodelled engine heat (bias on T_e) also raises the liner film drop.
            for xm in (X[:, k], X[:, k + 1]):
                yk = self.model.h(xm, uk, Dpk[:, k])
                T_metal_pk = yk[8] + ca.fmax(b[0], 0) * p.C_e / (p.k_wall * yk[9] ** 0.8 + 1.0)
                opti.subject_to(T_metal_pk <= p.T_metal_max - mg + S[2, k])

        opti.minimize(J)
        opts = {"ipopt.print_level": 0, "print_time": False, "ipopt.sb": "yes",
                "ipopt.max_iter": 200, "ipopt.tol": 1e-6,
                "ipopt.warm_start_init_point": "yes"}
        opts.update(ipopt_opts)
        opti.solver("ipopt", opts)

        self.opti, self.X, self.U, self.S = opti, X, U, S
        self.x0, self.D, self.Dpk, self.u_prev, self.b = x0, D, Dpk, u_prev, b
        self.lb = np.array([p.u_pump_min, 0.0, 0.0])

    def preview(self, cycle, k):
        """Disturbance preview over the horizon (perfect knowledge of the route).

        Returns (D_mean, D_peak), one column per NMPC interval: the mean of the
        cycle samples inside the interval, and the same with brake power
        replaced by its maximum inside the interval.
        """
        step = int(round(self.dt / cycle["dt"]))
        D_mean, D_peak = [], []
        for i in range(self.Np):
            block = np.column_stack([disturbance(cycle, k + i * step + j) for j in range(step)])
            dm = block.mean(axis=1)
            dp = dm.copy()
            dp[1] = block[1].max()
            D_mean.append(dm)
            D_peak.append(dp)
        return np.column_stack(D_mean), np.column_stack(D_peak)

    def solve(self, x, D, u_prev):
        opti = self.opti
        D_mean, D_peak = D if isinstance(D, tuple) else (D, D)
        self.update_bias(x)
        opti.set_value(self.b, self.bias)
        opti.set_value(self.x0, x)
        opti.set_value(self.D, D_mean)
        opti.set_value(self.Dpk, D_peak)
        opti.set_value(self.u_prev, u_prev)
        if self._last is None:
            opti.set_initial(self.X, np.tile(np.asarray(x).reshape(-1, 1), self.Np + 1))
            opti.set_initial(self.U, np.tile(np.asarray(u_prev).reshape(-1, 1), self.Np))
            opti.set_initial(self.S, 0.0)
        else:  # shift previous solution by one interval
            Xl, Ul, Sl = self._last
            opti.set_initial(self.X, np.hstack([Xl[:, 1:], Xl[:, -1:]]))
            opti.set_initial(self.U, np.hstack([Ul[:, 1:], Ul[:, -1:]]))
            opti.set_initial(self.S, np.hstack([Sl[:, 1:], Sl[:, -1:]]))
        try:
            sol = opti.solve()
            Xs, Us, Ss = sol.value(self.X), sol.value(self.U), sol.value(self.S)
            ok = True
        except RuntimeError:  # fall back to the last iterate, still bounded
            Xs, Us, Ss = (opti.debug.value(v) for v in (self.X, self.U, self.S))
            ok = False
        Us = np.atleast_2d(Us)
        self._last = (np.atleast_2d(Xs), Us, np.atleast_2d(Ss))
        stats = opti.stats()
        u0 = np.clip(Us[:, 0], self.lb, 1.0)
        # what the model expects the next measurement to be (for the bias update)
        x_next = self.model.step(np.asarray(x), u0, D_mean[:, 0], self.dt, self.substeps)
        self._x_pred = x_next.full().ravel() + self.dt * self.bias
        return u0, dict(ok=ok, iters=stats.get("iter_count", -1),
                              t_wall=stats.get("t_wall_total", np.nan),
                              X_pred=np.atleast_2d(Xs), U_pred=Us)
