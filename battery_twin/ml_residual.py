"""AI/ML layer of the digital twin.

A small neural network learns the residual between the physics twin and the
sensor data (what the physics gets wrong: cold-plate degradation, chiller
capacity loss in heat, heat leaking in from the e-drive loop). The trained
network is rebuilt as a CasADi expression so the NMPC can differentiate
through it: physics + ML = hybrid twin.
"""
import json

import casadi as ca
import numpy as np
from sklearn.neural_network import MLPRegressor

from .controllers import RuleBasedBMS
from .estimator import EKF
from .mission import SEGMENTS, build_mission, disturbance
from .model import N_FEATURES, PackModel, features, physics

OUT_SCALE = 1e3  # network predicts mK/s
_feat = None


def _feature_fn():
    global _feat
    if _feat is None:
        x, u, d = ca.SX.sym("x", 4), ca.SX.sym("u", 5), ca.SX.sym("d", 4)
        _feat = ca.Function("feat", [x, u, d], [features(x, u, d)])
    return _feat


def random_mission(rng):
    segs = []
    drive = [s for s in SEGMENTS if s[3] == 0]
    for _ in range(rng.integers(6, 12)):
        dur, v, g, _ = drive[rng.integers(len(drive))]
        segs.append((int(dur * rng.uniform(0.5, 1.5)), v * rng.uniform(0.8, 1.1), g * rng.uniform(0.5, 1.3), 0))
    segs.insert(rng.integers(2, len(segs)), (int(rng.uniform(900, 2400)), 0, 0, rng.uniform(250e3, 800e3)))
    return build_mission(T_amb=float(rng.uniform(-5, 45)), segments=segs)


def collect(plant: PackModel, n_episodes=30, ctrl_dt=60.0, noise=(0.1, 0.1, 0.003), seed=0):
    """Fleet-style sensor logs from the 'real' pack under varied missions and climates,
    with random actuator excitation on top of the production BMS logic."""
    rng = np.random.default_rng(seed)
    episodes = []
    for _ in range(n_episodes):
        m = random_mission(rng)
        T0 = float(np.clip(m["T_amb"][0] + rng.uniform(-5, 10), -5, 45))
        x = np.array([T0, T0, T0, rng.uniform(0.3, 0.8)])
        bms = RuleBasedBMS()
        every = int(ctrl_dt / m["dt"])
        rec = dict(y=[], u=[], d=[], x=[])
        u = np.zeros(5)
        for k in range(m["t"].size):
            if k % every == 0:
                y = x[1:] + rng.standard_normal(3) * np.asarray(noise)
                u = bms(y, disturbance(m, k))
                if rng.random() < 0.5:  # excitation
                    u = np.clip(u + rng.normal(0, 0.3, 5), [0.05, 0, 0, 0, 0], 1)
                    if rng.random() < 0.5:
                        u[1 if rng.random() < 0.5 else 4] = 0.0
                d_blk = np.column_stack([disturbance(m, k + j) for j in range(every)]).mean(axis=1)
                rec["y"].append(y), rec["u"].append(u.copy()), rec["d"].append(d_blk), rec["x"].append(x.copy())
            x = plant.step(x, u, disturbance(m, k), m["dt"]).full().ravel()
        episodes.append({k: np.array(v) for k, v in rec.items()})
    return episodes


def residual_dataset(episodes, twin: PackModel, ctrl_dt=60.0):
    """Targets: (measured next T_s, T_w - physics one-step prediction) / dt."""
    feat = _feature_fn()
    Z, R = [], []
    for ep in episodes:
        y, u, d = ep["y"], ep["u"], ep["d"]
        x0 = np.r_[y[0, 0], y[0]]
        ekf = EKF(twin, ctrl_dt, x0)
        ekf.update(y[0])
        for k in range(len(y) - 1):
            xh = ekf.x.copy()
            xp = twin.step(xh, u[k], d[k], ctrl_dt, 2).full().ravel()
            R.append((y[k + 1, :2] - xp[1:3]) / ctrl_dt * OUT_SCALE)
            xz = np.r_[xh[0], y[k, 0], y[k, 1], y[k, 2]]
            Z.append(feat(xz, u[k], d[k]).full().ravel())
            ekf.predict(u[k], d[k])
            ekf.update(y[k + 1])
    return np.array(Z), np.array(R)


def train(Z, R, hidden=24, seed=0):
    net = MLPRegressor(hidden_layer_sizes=(hidden,), activation="tanh", alpha=1e-3,
                       learning_rate_init=3e-3, max_iter=3000, early_stopping=True,
                       n_iter_no_change=50, random_state=seed)
    net.fit(Z, R)
    return net


def to_casadi(net) -> ca.Function:
    W1, W2 = net.coefs_
    b1, b2 = net.intercepts_
    z = ca.SX.sym("z", N_FEATURES)
    hdn = ca.tanh(ca.mtimes(ca.DM(W1.T), z) + ca.DM(b1))
    out = (ca.mtimes(ca.DM(W2.T), hdn) + ca.DM(b2)) / OUT_SCALE
    return ca.Function("residual", [z], [out])


def save(net, path):
    W1, W2 = net.coefs_
    b1, b2 = net.intercepts_
    with open(path, "w") as f:
        json.dump({"W1": W1.tolist(), "b1": b1.tolist(), "W2": W2.tolist(), "b2": b2.tolist(),
                   "out_scale": OUT_SCALE, "features": N_FEATURES}, f)


def load(path) -> ca.Function:
    with open(path) as f:
        w = json.load(f)

    class _Net:  # minimal stand-in with the sklearn attribute layout
        coefs_ = [np.array(w["W1"]), np.array(w["W2"])]
        intercepts_ = [np.array(w["b1"]), np.array(w["b2"])]
    return to_casadi(_Net())


def open_loop_rmse(model: PackModel, ep, ctrl_dt=60.0, horizon=60):
    """RMSE of free-run predictions over `horizon` steps, from the true state,
    against the measured surface and coolant temperatures."""
    errs = []
    for s in range(0, len(ep["y"]) - horizon, horizon // 2):
        x = ep["x"][s].copy()
        for k in range(s, s + horizon):
            x = model.step(x, ep["u"][k], ep["d"][k], ctrl_dt, 2).full().ravel()
            errs.append(x[1:3] - ep["y"][k + 1, :2])
    e = np.array(errs)
    return np.sqrt(np.mean(e ** 2, axis=0))
