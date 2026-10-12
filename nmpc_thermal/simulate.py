"""Closed-loop simulation of plant + controller over a drive cycle."""
import time

import numpy as np

from .controllers import NMPC
from .drive_cycle import disturbance
from .model import ThermalModel


def run(controller, cycle, plant: ThermalModel, x0, ctrl_dt=5.0,
        meas_noise=0.0, seed=0, verbose=False):
    """Simulate at the cycle sample time; the controller updates every ctrl_dt.

    The plant may use different parameters from the controller's internal
    model (plant-model mismatch). Returns a dict of logged time series.
    """
    rng = np.random.default_rng(seed)
    dt = cycle["dt"]
    n = cycle["t"].size
    every = int(round(ctrl_dt / dt))

    x = np.asarray(x0, dtype=float)
    u = np.array([plant.p.u_pump_min, 0.0, 0.0])
    log = {k: np.zeros(n) for k in ["T_e", "T_r", "T_o", "u_pump", "u_fan", "u_th"]
           + plant.OUTPUT_NAMES + ["solve_ms"]}
    n_fail = 0
    for k in range(n):
        if k % every == 0:
            y_meas = x + meas_noise * rng.standard_normal(x.size)
            t0 = time.perf_counter()
            if isinstance(controller, NMPC):
                u, info = controller.solve(y_meas, controller.preview(cycle, k), u)
                n_fail += not info["ok"]
            else:
                u = controller(y_meas, k, cycle)
            log["solve_ms"][k] = 1e3 * (time.perf_counter() - t0)
            if verbose and k % (every * 60) == 0:
                print(f"  t={cycle['t'][k]:6.0f}s  T_e={x[0]:6.1f}  u={np.round(u, 2)}")
        d = disturbance(cycle, k)
        for name, val in zip(["T_e", "T_r", "T_o"], x):
            log[name][k] = val
        for name, val in zip(["u_pump", "u_fan", "u_th"], u):
            log[name][k] = val
        for name, val in plant.outputs(x, u, d).items():
            log[name][k] = val
        x = plant.step(x, u, d, dt, substeps=2).full().ravel()
    log["t"] = cycle["t"]
    log["n_fail"] = n_fail
    return log
