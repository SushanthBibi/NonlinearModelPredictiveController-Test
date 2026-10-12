import numpy as np
from sklearn.neural_network import MLPRegressor

from battery_twin import ml_residual as ml
from battery_twin.controllers import RuleBasedBMS, TwinNMPC
from battery_twin.estimator import EKF
from battery_twin.mission import build_mission, disturbance
from battery_twin.model import N_FEATURES, physics, truth

PHYS, TRUTH = physics(), truth()


def test_charge_derates_when_hot_or_cold():
    d = [5e3, 750e3, 0, 25]
    u = [0.5, 0, 0, 0, 0]
    ok = PHYS.outputs([30, 30, 30, 0.3], u, d)["P_chg"]
    hot = PHYS.outputs([46, 44, 40, 0.3], u, d)["P_chg"]
    cold = PHYS.outputs([2, 2, 2, 0.3], u, d)["P_chg"]
    assert ok > 700e3 and hot < 0.5 * ok and cold < 0.3 * ok


def test_chiller_cop_falls_with_ambient():
    u, x = [0.5, 1, 0, 0, 0], [35, 33, 25, 0.5]
    cop_mild = PHYS.outputs(x, u, [0, 0, 0, 20])["COP"]
    cop_hot = PHYS.outputs(x, u, [0, 0, 0, 42])["COP"]
    assert cop_mild > cop_hot > 1


def test_reality_gap_makes_truth_hotter_under_load():
    x = np.array([30.0, 30, 30, 0.6])
    u, d = [0.5, 0, 0, 0, 0], [400e3, 0, 15, 30]
    xp, xt = x.copy(), x.copy()
    for _ in range(30):
        xp = PHYS.step(xp, u, d, 60.0).full().ravel()
        xt = TRUTH.step(xt, u, d, 60.0).full().ravel()
    assert xt[2] > xp[2]


def test_ekf_recovers_unmeasured_core_temperature():
    rng = np.random.default_rng(0)
    x = np.array([40.0, 34, 30, 0.5])
    ekf = EKF(PHYS, 60.0, np.r_[34.0, x[1:]])
    u, d = np.array([0.6, 0.5, 0, 0, 0]), np.array([300e3, 0, 20, 30])
    for _ in range(40):
        x = PHYS.step(x, u, d, 60.0, 2).full().ravel()
        ekf.predict(u, d)
        ekf.update(x[1:] + rng.normal(0, [0.1, 0.1, 0.003]))
    assert abs(ekf.x[0] - x[0]) < 0.5


def test_residual_net_casadi_export_matches_sklearn():
    rng = np.random.default_rng(0)
    Z = rng.normal(size=(200, N_FEATURES))
    R = np.column_stack([np.sin(Z[:, 0]), Z[:, 1] * 0.5])
    net = MLPRegressor(hidden_layer_sizes=(8,), activation="tanh", max_iter=300, random_state=0).fit(Z, R)
    f = ml.to_casadi(net)
    z = Z[:5]
    np.testing.assert_allclose(np.column_stack([f(zi).full().ravel() for zi in z]).T * ml.OUT_SCALE,
                               net.predict(z), rtol=1e-6, atol=1e-8)


def test_nmpc_precools_before_charging():
    m = build_mission(T_amb=40.0)
    k = int(np.nonzero(m["P_chg_max"] > 0)[0][0]) - 20 * 60  # 20 min before the charger
    mpc = TwinNMPC(PHYS, horizon=30)
    u, info = mpc.solve(np.array([36.0, 35, 32, 0.25]), mpc.preview(m, k), np.array([0.3, 0, 0, 0, 0]))
    assert info["ok"]
    assert u[1] > 0.05          # chiller already running while still driving
    assert np.all(u >= mpc.lb - 1e-9) and np.all(u <= 1 + 1e-9)


def test_rule_based_hysteresis():
    bms = RuleBasedBMS()
    d = disturbance(build_mission(), 100)
    assert bms(np.array([36.0, 30, 0.5]), d)[1] == 1.0
    assert bms(np.array([33.0, 30, 0.5]), d)[1] == 1.0
    assert bms(np.array([30.0, 30, 0.5]), d)[1] == 0.0
