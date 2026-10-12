"""Train the ML residual of the battery digital twin and report its accuracy.

    python scripts/twin_train.py
Writes results/twin/residual_net.json and results/twin/ml_validation.json.
"""
import json
import pathlib
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from battery_twin import ml_residual as ml  # noqa: E402
from battery_twin.model import hybrid, physics, truth  # noqa: E402

OUT = ROOT / "results" / "twin"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plant, twin = truth(), physics()
    t0 = time.time()
    train_eps = ml.collect(plant, n_episodes=80, seed=1)
    test_eps = ml.collect(plant, n_episodes=6, seed=99)
    print(f"Collected {sum(len(e['y']) for e in train_eps)} training samples, "
          f"{sum(len(e['y']) for e in test_eps)} test samples ({time.time() - t0:.0f} s)")

    Z, R = ml.residual_dataset(train_eps, twin)
    Zt, Rt = ml.residual_dataset(test_eps, twin)
    net = ml.train(Z, R)
    pred = net.predict(Zt)
    r2 = [1 - np.sum((Rt[:, i] - pred[:, i]) ** 2) / np.sum((Rt[:, i] - Rt[:, i].mean()) ** 2) for i in range(2)]
    print(f"Residual network R^2 on test data: surface {r2[0]:.3f}, coolant {r2[1]:.3f}")
    ml.save(net, OUT / "residual_net.json")

    hyb = hybrid(ml.load(OUT / "residual_net.json"))
    rm_phys = np.mean([ml.open_loop_rmse(twin, e) for e in test_eps], axis=0)
    rm_hyb = np.mean([ml.open_loop_rmse(hyb, e) for e in test_eps], axis=0)
    print("60-min free-run prediction RMSE [K]   surface   coolant")
    print(f"  physics twin                         {rm_phys[0]:7.2f}  {rm_phys[1]:7.2f}")
    print(f"  hybrid AI twin                       {rm_hyb[0]:7.2f}  {rm_hyb[1]:7.2f}")
    res = {"n_train": int(Z.shape[0]), "n_test": int(Zt.shape[0]),
           "residual_r2": {"T_s": r2[0], "T_w": r2[1]},
           "free_run_rmse_60min": {"physics": {"T_s": rm_phys[0], "T_w": rm_phys[1]},
                                   "hybrid": {"T_s": rm_hyb[0], "T_w": rm_hyb[1]}}}
    (OUT / "ml_validation.json").write_text(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
