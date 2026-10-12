"""Model calibration and validation (deliverable 1: validated thermal model).

Workflow with real data:
  1. Log a test-cell or vehicle run to CSV with the columns in CSV_COLUMNS.
  2. Split it into an identification run and a separate validation run.
  3. identify() fits selected parameters on the first by least squares on
     the simulated vs measured temperatures (open-loop simulation driven by the
     measured inputs and disturbances).
  4. validate() reports RMSE and R^2 per temperature on the second run.
"""
import csv

import numpy as np
from scipy.optimize import least_squares

from .model import ThermalModel
from .params import DEFAULT, ThermalParams

CSV_COLUMNS = ["t", "T_e", "T_r", "T_o", "u_pump", "u_fan", "u_th", "N", "P_b", "v", "T_amb"]
DEFAULT_FIT = ("C_e", "C_r", "C_o", "k_air", "f_cool_fuel", "hA_oc0")


def load_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {c: np.array([float(r[c]) for r in rows]) for c in CSV_COLUMNS}


def save_csv(path, data):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        for i in range(data["t"].size):
            w.writerow([f"{data[c][i]:.6g}" for c in CSV_COLUMNS])


def simulate_open_loop(params: ThermalParams, data):
    """Replay the measured inputs/disturbances through the model."""
    m = ThermalModel(params)
    t = data["t"]
    U = np.column_stack([data["u_pump"], data["u_fan"], data["u_th"]])
    D = np.column_stack([data["N"], data["P_b"], data["v"], data["T_amb"]])
    X = np.zeros((t.size, 3))
    X[0] = [data["T_e"][0], data["T_r"][0], data["T_o"][0]]
    for k in range(t.size - 1):
        X[k + 1] = m.step(X[k], U[k], D[k], t[k + 1] - t[k]).full().ravel()
    return X


def fit_metrics(meas, pred):
    err = pred - meas
    rmse = float(np.sqrt(np.mean(err ** 2)))
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((meas - meas.mean()) ** 2))
    return {"RMSE [K]": rmse, "R2": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")}


def validate(params: ThermalParams, data):
    X = simulate_open_loop(params, data)
    return {name: fit_metrics(data[name], X[:, i]) for i, name in enumerate(["T_e", "T_r", "T_o"])}


def identify(data, base: ThermalParams = DEFAULT, names=DEFAULT_FIT, decimate=5, verbose=0):
    """Fit multiplicative scale factors on the named parameters (log-space)."""
    sub = {k: v[::decimate] for k, v in data.items()}
    meas = np.column_stack([sub["T_e"], sub["T_r"], sub["T_o"]])
    x0 = {n: getattr(base, n) for n in names}

    def make(theta):
        return base.perturbed(**{n: x0[n] * float(np.exp(th)) for n, th in zip(names, theta)})

    def resid(theta):
        return (simulate_open_loop(make(theta), sub) - meas).ravel()

    sol = least_squares(resid, np.zeros(len(names)), bounds=(-1.0, 1.0),
                        x_scale=0.1, verbose=verbose, max_nfev=60)
    fitted = make(sol.x)
    return fitted, {n: float(np.exp(th)) for n, th in zip(names, sol.x)}
