"""Calibrate and validate the thermal model (RMSE / R^2).

With measured data:
    python scripts/validate_model.py --ident ident.csv --valid valid.csv

Without data this runs a SYNTHETIC self-check: "measurements" are generated
by a model with deliberately different parameters plus sensor noise, and the
script verifies that identification recovers a model that predicts a
different (validation) cycle. This exercises the pipeline only; it is not
evidence that the model matches a real engine.
"""
import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nmpc_thermal.controllers import PIController  # noqa: E402
from nmpc_thermal.drive_cycle import build_cycle  # noqa: E402
from nmpc_thermal.identification import identify, load_csv, validate  # noqa: E402
from nmpc_thermal.model import ThermalModel  # noqa: E402
from nmpc_thermal.params import DEFAULT  # noqa: E402
from nmpc_thermal.simulate import run  # noqa: E402

SYNTH_SEGMENTS_VALID = [(60, 0, 0), (300, 18, 0), (400, 22, 0.02), (200, 14, 0.05),
                        (300, 22, -0.02), (300, 20, 0), (60, 0, 0)]


def synthetic(params, segments=None, T_amb=25.0, noise=0.3, seed=0):
    cyc = build_cycle(**({"segments": segments} if segments else {}), T_amb=T_amb)
    plant = ThermalModel(params)
    # excite the actuators: PI control plus a pseudo-random dither on the fan and pump
    rng = np.random.default_rng(seed)
    pi = PIController(p=params)
    dither = np.repeat(rng.uniform(-0.15, 0.15, size=(cyc["t"].size // 60 + 1, 2)), 60, axis=0)

    def ctrl(x, k, c):
        u = pi(x, k, c)
        u[0] = np.clip(u[0] + dither[k, 0], params.u_pump_min, 1)
        u[1] = np.clip(u[1] + dither[k, 1], 0, 1)
        return u

    log = run(ctrl, cyc, plant, np.full(3, T_amb), ctrl_dt=1.0)
    data = {k: log[k].copy() for k in ["t", "T_e", "T_r", "T_o", "u_pump", "u_fan", "u_th"]}
    data.update(N=cyc["N"], P_b=cyc["P_b"], v=cyc["v"], T_amb=cyc["T_amb"])
    for k in ["T_e", "T_r", "T_o"]:
        data[k] = data[k] + noise * rng.standard_normal(data[k].size)
    return data


def report(title, res):
    print(f"\n{title}")
    for name, m in res.items():
        print(f"  {name:4s}  RMSE = {m['RMSE [K]']:6.2f} K   R^2 = {m['R2']:.4f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ident")
    ap.add_argument("--valid")
    args = ap.parse_args()

    if args.ident and args.valid:
        d_id, d_val = load_csv(args.ident), load_csv(args.valid)
    else:
        print("No data given -> SYNTHETIC self-check (not a real validation).")
        truth = DEFAULT.perturbed(C_e=1.25 * DEFAULT.C_e, k_air=0.8 * DEFAULT.k_air,
                                  f_cool_fuel=1.12 * DEFAULT.f_cool_fuel, hA_oc0=0.7 * DEFAULT.hA_oc0)
        d_id = synthetic(truth, seed=1)
        d_val = synthetic(truth, segments=SYNTH_SEGMENTS_VALID, T_amb=32.0, seed=2)

    report("Nominal model on validation data:", validate(DEFAULT, d_val))
    fitted, scales = identify(d_id)
    print("\nIdentified scale factors (fitted / nominal):")
    for n, s in scales.items():
        print(f"  {n:12s} x {s:.3f}")
    report("Identified model on validation data:", validate(fitted, d_val))


if __name__ == "__main__":
    main()
