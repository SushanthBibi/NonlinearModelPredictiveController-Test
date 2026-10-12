"""Closed loop: real pack (truth model) -> noisy sensors -> EKF twin -> controller."""
import time

import numpy as np

from .controllers import TwinNMPC
from .estimator import EKF
from .mission import disturbance
from .model import OUTPUT_NAMES, PackModel


def run(controller, mission, plant: PackModel, twin: PackModel, x0, ctrl_dt=60.0,
        noise=(0.1, 0.1, 0.003), seed=0, verbose=False):
    rng = np.random.default_rng(seed)
    dt = mission["dt"]
    n = mission["t"].size
    every = int(round(ctrl_dt / dt))
    x = np.asarray(x0, float)
    x_guess = x.copy()
    x_guess[0] = x[1]                      # core unknown at start: assume = surface
    ekf = EKF(twin, ctrl_dt, x_guess)
    u = np.array([0.3, 0, 0, 0, 0], float)
    keys = ["T_c", "T_s", "T_w", "SOC", "T_c_hat", "u_pump", "u_chill", "u_fan", "u_rv",
            "u_heat", "solve_ms"] + OUTPUT_NAMES
    log = {k: np.zeros(n) for k in keys}
    n_fail = 0
    d_acc = np.zeros(4)
    for k in range(n):
        if k % every == 0:
            y = x[1:] + rng.standard_normal(3) * np.asarray(noise)
            if k > 0:
                ekf.predict(u, d_acc / every)
                d_acc[:] = 0
            x_hat = ekf.update(y)
            t0 = time.perf_counter()
            if isinstance(controller, TwinNMPC):
                u, info = controller.solve(x_hat, controller.preview(mission, k), u)
                n_fail += not info["ok"]
            else:
                u = controller(y, disturbance(mission, k))
            log["solve_ms"][k] = 1e3 * (time.perf_counter() - t0)
            if verbose and k % (every * 15) == 0:
                print(f"  t={k*dt/60:5.0f} min  Tc={x[0]:5.1f} (est {x_hat[0]:5.1f})  "
                      f"SOC={x[3]:.2f}  u={np.round(u, 2)}")
        d = disturbance(mission, k)
        d_acc += d
        for i, s in enumerate(["T_c", "T_s", "T_w", "SOC"]):
            log[s][k] = x[i]
        log["T_c_hat"][k] = ekf.x[0]
        for i, s in enumerate(["u_pump", "u_chill", "u_fan", "u_rv", "u_heat"]):
            log[s][k] = u[i]
        for s, val in plant.outputs(x, u, d).items():
            log[s][k] = val
        x = plant.step(x, u, d, dt).full().ravel()
    log["t"] = mission["t"]
    log["n_fail"] = n_fail
    return log


def kpis(log, mission, p):
    dt = mission["dt"]
    E = lambda P: float(np.sum(P) * dt / 3.6e6)
    chg = mission["P_chg_max"] > 0
    return {
        "thermal-system energy [kWh]": E(log["P_thermal"]),
        "  chiller compressor [kWh]": E(log["P_comp"]),
        "  heater [kWh]": E(log["P_heat"]),
        "  pump + fan [kWh]": E(log["P_pump"] + log["P_fan"]),
        "energy charged in 30-min stop [kWh]": E(log["P_chg"][chg]),
        "SOC after charging [%]": float(100 * log["SOC"][np.nonzero(chg)[0][-1]]),
        "peak core temp [degC]": float(log["T_c"].max()),
        "min core temp while charging [degC]": float(log["T_c"][chg].min()),
        "time core > 40 degC [min]": float(np.sum(log["T_c"] > 40) * dt / 60),
        "ageing index [-] (1 = 25 degC)": float(np.mean(log["age_rate"])),
        "core-temp estimate RMSE [K]": float(np.sqrt(np.mean((log["T_c_hat"] - log["T_c"])[600:] ** 2))),
    }
