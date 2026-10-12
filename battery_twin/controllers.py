"""Rule-based BMS thermal strategy (baseline) and the digital-twin NMPC."""
from dataclasses import dataclass

import casadi as ca
import numpy as np

from .mission import disturbance
from .model import ND, NU, NX, OUTPUT_NAMES, PackModel

O = {n: i for i, n in enumerate(OUTPUT_NAMES)}


class RuleBasedBMS:
    """Typical production logic: reactive thresholds on measured temperatures,
    no knowledge of the route or the upcoming charging stop."""

    name = "Rule-based BMS"

    def __init__(self, chill_on=35.0, chill_off=31.0, heat_on=10.0, heat_off=15.0):
        self.chill_on, self.chill_off = chill_on, chill_off
        self.heat_on, self.heat_off = heat_on, heat_off
        self.chill = self.heat = False

    def __call__(self, y, d):
        T_s, T_w = y[0], y[1]
        P_drive, P_chg, _, T_amb = d
        if T_s >= self.chill_on:
            self.chill = True
        elif T_s <= self.chill_off:
            self.chill = False
        if T_s <= self.heat_on:
            self.heat = True
        elif T_s >= self.heat_off:
            self.heat = False
        active = P_chg > 0 or abs(P_drive) > 20e3
        u_pump = 0.6 if (active or self.chill or self.heat) else 0.3
        radiate = (not self.chill) and T_s > 28 and T_w > T_amb + 3
        u_rv = 1.0 if radiate else 0.0
        u_fan = float(np.clip((T_w - T_amb - 3) / 10, 0, 1)) if radiate else 0.0
        return np.array([u_pump, 1.0 if self.chill else 0.0, u_fan, u_rv,
                         1.0 if self.heat else 0.0])


@dataclass
class Weights:
    w_energy: float = 1.0     # per kW of thermal-system electrical power
    w_charge: float = 0.25    # reward per kW of charge power accepted
    w_age: float = 2.0        # per unit Arrhenius ageing rate (1 at 25 degC)
    w_band: float = 2.0       # per K^2 outside the preferred window
    w_limit: float = 1e3      # per K^2 above the hard limit
    w_du: float = 2.0


class TwinNMPC:
    """NMPC on top of the digital twin (physics or hybrid physics+ML model).

    Decision variables: X (NX x N+1), U (NU x N), slacks. Parameters: the EKF
    state estimate and the route preview (traction power, charger power,
    speed, ambient) over the horizon. Horizon N x dt (default 40 x 60 s = 40 min)
    is long enough to see a charging stop coming and pre-condition the pack.
    """

    def __init__(self, model: PackModel, horizon=40, dt=60.0, weights: Weights | None = None,
                 label=None):
        self.model, self.p = model, model.p
        self.N, self.dt = horizon, dt
        self.w = weights or Weights()
        self.name = label or f"NMPC ({model.name} twin)"
        self._build()
        self._last = None

    def _build(self):
        p, w, N = self.p, self.w, self.N
        opti = ca.Opti()
        X, U = opti.variable(NX, N + 1), opti.variable(NU, N)
        S = opti.variable(3, N)  # [above band, below band, above limit]
        x0, D, u_prev = opti.parameter(NX), opti.parameter(ND, N), opti.parameter(NU)
        J = 0
        opti.subject_to(X[:, 0] == x0)
        lb = np.array([p.u_pump_min, 0, 0, 0, 0])
        for k in range(N):
            xk, uk, dk = X[:, k], U[:, k], D[:, k]
            opti.subject_to(X[:, k + 1] == self.model.step(xk, uk, dk, self.dt, 2))
            y = self.model.h(xk, uk, dk)
            du = uk - (u_prev if k == 0 else U[:, k - 1])
            J += (w.w_energy * y[O["P_thermal"]] / 1e3
                  - w.w_charge * y[O["P_chg"]] / 1e3
                  + w.w_age * y[O["age_rate"]]
                  + w.w_band * (S[0, k] ** 2 + 0.2 * S[1, k] ** 2)
                  + w.w_limit * S[2, k] ** 2
                  + w.w_du * ca.sumsqr(du))
            Tc = X[0, k + 1]
            opti.subject_to(Tc <= p.T_band_hi + S[0, k])
            opti.subject_to(Tc >= p.T_band_lo - S[1, k])
            opti.subject_to(Tc <= p.T_max - 2.0 + S[2, k])
            opti.subject_to(opti.bounded(lb, uk, 1.0))
            opti.subject_to(S[:, k] >= 0)
            # chiller and heater should not fight each other
            opti.subject_to(uk[1] * uk[4] <= 0.01)
        opti.minimize(J)
        opti.solver("ipopt", {"ipopt.print_level": 0, "print_time": False, "ipopt.sb": "yes",
                              "ipopt.max_iter": 300, "ipopt.tol": 1e-5})
        self.opti, self.X, self.U, self.S = opti, X, U, S
        self.x0, self.D, self.u_prev, self.lb = x0, D, u_prev, lb

    def preview(self, mission, k):
        """Interval-mean route preview over the horizon (from navigation / route plan)."""
        step = int(round(self.dt / mission["dt"]))
        cols = []
        for i in range(self.N):
            block = np.column_stack([disturbance(mission, k + i * step + j) for j in range(step)])
            cols.append(block.mean(axis=1))
        return np.column_stack(cols)

    def solve(self, x_hat, D, u_prev):
        o = self.opti
        o.set_value(self.x0, x_hat)
        o.set_value(self.D, D)
        o.set_value(self.u_prev, u_prev)
        if self._last is None:
            o.set_initial(self.X, np.tile(np.reshape(x_hat, (-1, 1)), self.N + 1))
            o.set_initial(self.U, np.tile(np.reshape(np.maximum(u_prev, self.lb), (-1, 1)), self.N))
        else:
            Xl, Ul = self._last
            o.set_initial(self.X, np.hstack([Xl[:, 1:], Xl[:, -1:]]))
            o.set_initial(self.U, np.hstack([Ul[:, 1:], Ul[:, -1:]]))
        try:
            sol = o.solve()
            Xs, Us, ok = sol.value(self.X), sol.value(self.U), True
        except RuntimeError:
            Xs, Us, ok = o.debug.value(self.X), o.debug.value(self.U), False
        self._last = (np.atleast_2d(Xs), np.atleast_2d(Us))
        st = o.stats()
        return np.clip(Us[:, 0], self.lb, 1.0), dict(ok=ok, X_pred=Xs, iters=st.get("iter_count", -1))
