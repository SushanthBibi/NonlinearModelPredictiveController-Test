"""Run conventional control vs NMPC over the drive cycle and report KPIs.

    python scripts/run_comparison.py [--ambient 25] [--mismatch] [--no-plot]
"""
import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from nmpc_thermal.controllers import NMPC, ConventionalController, PIController  # noqa: E402
from nmpc_thermal.drive_cycle import build_cycle  # noqa: E402
from nmpc_thermal.metrics import compare_table, kpis  # noqa: E402
from nmpc_thermal.model import ThermalModel  # noqa: E402
from nmpc_thermal.params import DEFAULT  # noqa: E402

OUT = pathlib.Path(__file__).resolve().parents[1] / "results"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ambient", type=float, default=25.0)
    ap.add_argument("--T0", type=float, default=None, help="initial temperature (default: ambient = cold start)")
    ap.add_argument("--horizon", type=int, default=24)
    ap.add_argument("--ctrl-dt", type=float, default=5.0)
    ap.add_argument("--mismatch", action="store_true",
                    help="plant radiator 15%% weaker and heat rejection 10%% higher than the NMPC model")
    ap.add_argument("--noise", type=float, default=0.0, help="temperature sensor noise std [K]")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    cycle = build_cycle(T_amb=args.ambient)
    T0 = args.ambient if args.T0 is None else args.T0
    x0 = np.array([T0, T0, T0])

    ctrl_model = ThermalModel(DEFAULT)
    plant_p = DEFAULT.perturbed(k_air=0.85 * DEFAULT.k_air, f_cool_fuel=1.1 * DEFAULT.f_cool_fuel) \
        if args.mismatch else DEFAULT
    plant = ThermalModel(plant_p)

    print("Simulating conventional controller ...")
    log_c = run(ConventionalController(p=DEFAULT), cycle, plant, x0, ctrl_dt=1.0, meas_noise=args.noise)
    print("Simulating PI controller (variable actuators, no preview) ...")
    log_p = run(PIController(p=DEFAULT), cycle, plant, x0, ctrl_dt=1.0, meas_noise=args.noise)
    print("Simulating NMPC ...")
    nmpc = NMPC(ctrl_model, horizon=args.horizon, dt=args.ctrl_dt)
    log_n = run(nmpc, cycle, plant, x0, ctrl_dt=args.ctrl_dt, meas_noise=args.noise, verbose=True)

    res = {"conventional": kpis(log_c, cycle["dt"], p=plant_p),
           "PI": kpis(log_p, cycle["dt"], p=plant_p),
           "NMPC": kpis(log_n, cycle["dt"], p=plant_p)}
    print()
    print(compare_table(res))
    solves = log_n["solve_ms"][log_n["solve_ms"] > 0]
    print(f"\nNMPC solve time: mean {solves.mean():.0f} ms, max {solves.max():.0f} ms, "
          f"failed solves: {log_n['n_fail']} / {solves.size}")

    OUT.mkdir(exist_ok=True)
    tag = f"amb{args.ambient:g}" + ("_mismatch" if args.mismatch else "")
    (OUT / f"kpis_{tag}.json").write_text(json.dumps(res, indent=2))
    if not args.no_plot:
        from nmpc_thermal.plotting import plot_comparison
        path = OUT / f"comparison_{tag}.png"
        plot_comparison(cycle, {"conventional": log_c, "PI": log_p, "NMPC": log_n}, plant_p, path)
        print(f"Saved {path}")


from nmpc_thermal.simulate import run  # noqa: E402

if __name__ == "__main__":
    main()
