"""Run every climate scenario with the three controllers and export results.

    python scripts/twin_train.py      # once, trains the ML residual
    python scripts/twin_run_all.py    # ~5-10 min

Writes results/twin/kpis.json and results/twin/dashboard_data.json, then
rebuilds the demo dashboard (demo/index.html).
"""
import json
import pathlib
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from battery_twin import ml_residual as ml  # noqa: E402
from battery_twin.controllers import RuleBasedBMS, TwinNMPC  # noqa: E402
from battery_twin.mission import build_mission, charging_window  # noqa: E402
from battery_twin.model import hybrid, physics, truth  # noqa: E402
from battery_twin.simulate import kpis, run  # noqa: E402
import build_demo  # noqa: E402

OUT = ROOT / "results" / "twin"
SCENARIOS = [  # key, label, ambient, initial pack temp
    ("summer", "Vellore summer, 42 °C", 42.0, 38.0),
    ("mild", "Mild day, 22 °C", 22.0, 24.0),
    ("winter", "Winter, -5 °C", -5.0, 4.0),
]
SERIES = ["T_c", "T_c_hat", "T_s", "T_w", "SOC", "P_thermal", "P_comp", "P_heat", "P_chg",
          "u_pump", "u_chill", "u_fan", "u_rv", "u_heat", "age_rate"]


def main(sample_every=30):
    plant = truth()
    phys = physics()
    hyb = hybrid(ml.load(OUT / "residual_net.json"))
    results, export = {}, {"scenarios": [], "controllers": [], "series": SERIES}
    for key, label, T_amb, T0 in SCENARIOS:
        m = build_mission(T_amb=T_amb)
        x0 = [T0, T0, T0, 0.50]
        ctrls = [("rule", RuleBasedBMS(), phys),
                 ("nmpc_phys", TwinNMPC(phys, label="NMPC + physics twin"), phys),
                 ("nmpc_ai", TwinNMPC(hyb, label="NMPC + AI hybrid twin"), hyb)]
        if not export["controllers"]:
            export["controllers"] = [{"key": k, "name": c.name} for k, c, _ in ctrls]
        sc = {"key": key, "label": label, "T_amb": T_amb,
              "t": (m["t"][::sample_every] / 60).round(2).tolist(),
              "P_drive": (m["P_drive"][::sample_every] / 1e3).round(1).tolist(),
              "P_chg_max": (m["P_chg_max"][::sample_every] / 1e3).round(1).tolist(),
              "v": (m["v"][::sample_every] * 3.6).round(1).tolist(),
              "charge_window": [w / 60 for w in charging_window(m)], "runs": {}, "kpis": {}}
        results[key] = {}
        for ck, c, twin in ctrls:
            t0 = time.time()
            log = run(c, m, plant, twin, x0)
            K = kpis(log, m, plant.p)
            solves = log["solve_ms"][log["solve_ms"] > 0]
            K["mean solve time [ms]"] = float(solves.mean())
            print(f"[{key}] {c.name:24s} {time.time() - t0:5.0f}s  fails={log['n_fail']}")
            results[key][c.name] = K
            sc["kpis"][ck] = {k.strip(): round(v, 3) for k, v in K.items()}
            sc["runs"][ck] = {s: np.round(log[s][::sample_every], 3 if s.startswith(("u_", "SOC", "age")) else 2).tolist()
                              for s in SERIES}
        export["scenarios"].append(sc)

    ml_val = json.loads((OUT / "ml_validation.json").read_text())
    export["ml"] = ml_val
    (OUT / "kpis.json").write_text(json.dumps(results, indent=2))
    (OUT / "dashboard_data.json").write_text(json.dumps(export, separators=(",", ":")))
    build_demo.main()

    for key, _, _, _ in SCENARIOS:
        print(f"\n=== {key}")
        names = list(results[key])
        print(f"{'KPI':42s}" + "".join(f"{n[:22]:>24s}" for n in names))
        for k in results[key][names[0]]:
            print(f"{k:42s}" + "".join(f"{results[key][n][k]:24.2f}" for n in names))


if __name__ == "__main__":
    main()
