import numpy as np

from nmpc_thermal.controllers import NMPC, ConventionalController, PIController
from nmpc_thermal.drive_cycle import build_cycle
from nmpc_thermal.identification import fit_metrics
from nmpc_thermal.model import ThermalModel
from nmpc_thermal.params import DEFAULT
from nmpc_thermal.simulate import run

MODEL = ThermalModel(DEFAULT)


def settle(u, d, x=(90.0, 80.0, 100.0), steps=6000):
    x = np.array(x)
    for _ in range(steps):
        x = MODEL.step(x, u, d, 1.0).full().ravel()
    return x


def test_steady_state_energy_balance():
    d, u = [1300, 100e3, 22, 25], [0.5, 0.3, 0.6]
    x = settle(u, d)
    y = MODEL.outputs(x, u, d)
    heat_in = y["Q_cool"] + y["Q_oil"]
    heat_out = y["Q_rad"] + DEFAULT.hA_amb * (x[0] - 25)
    assert abs(heat_in - heat_out) / heat_in < 1e-3


def test_fan_is_needed_on_a_climb():
    climb = [1600, 310e3, 15, 30]
    assert settle([1, 0, 1], climb)[0] > DEFAULT.T_e_max
    assert settle([1, 1, 1], climb)[0] < DEFAULT.T_e_max


def test_cold_oil_increases_friction_and_fuel():
    d, u = [1200, 100e3, 20, 20], [0.3, 0, 0]
    cold = MODEL.outputs([40, 30, 40], u, d)
    hot = MODEL.outputs([90, 80, 100], u, d)
    assert cold["P_fric"] > 1.3 * hot["P_fric"]
    assert cold["mdot_fuel"] > hot["mdot_fuel"]


def test_conventional_controller_logic():
    c = ConventionalController()
    cyc = build_cycle()
    assert c(np.array([70.0, 60, 70]), 500, cyc)[2] == 0.0      # thermostat shut
    assert c(np.array([99.0, 80, 100]), 500, cyc)[1] == 1.0     # fan on
    assert c(np.array([94.0, 80, 100]), 500, cyc)[1] == 1.0     # hysteresis holds
    assert c(np.array([90.0, 80, 100]), 500, cyc)[1] == 0.0


def test_nmpc_single_solve_respects_bounds():
    cyc = build_cycle()
    mpc = NMPC(MODEL, horizon=12)
    k = 1000  # mid-cycle, around the first climb
    u, info = mpc.solve(np.array([95.0, 85, 105]), mpc.preview(cyc, k), np.array([0.5, 0.2, 1.0]))
    assert info["ok"]
    assert DEFAULT.u_pump_min - 1e-6 <= u[0] <= 1 + 1e-6
    assert np.all((u[1:] >= -1e-6) & (u[1:] <= 1 + 1e-6))


def test_short_closed_loop_beats_conventional_on_cooling_energy():
    segs = [(120, 22.0, 0.0), (300, 16.0, 0.045), (120, 22.0, 0.0)]
    cyc = build_cycle(segments=segs, T_amb=30)
    x0 = np.array([90.0, 75, 100])
    conv = run(ConventionalController(), cyc, MODEL, x0, ctrl_dt=1.0)
    mpc = run(NMPC(MODEL, horizon=24), cyc, MODEL, x0, ctrl_dt=5.0)
    e = lambda log: np.sum(log["P_fan"] + log["P_pump"])
    assert e(mpc) < e(conv)
    assert mpc["T_e"].max() <= DEFAULT.T_e_max + 0.5
    assert mpc["n_fail"] == 0


def test_pi_controller_runs():
    cyc = build_cycle(segments=[(200, 20.0, 0.0)])
    log = run(PIController(), cyc, MODEL, np.full(3, 80.0), ctrl_dt=1.0)
    assert np.isfinite(log["T_e"]).all()


def test_fit_metrics():
    m = np.array([1.0, 2, 3, 4])
    assert fit_metrics(m, m) == {"RMSE [K]": 0.0, "R2": 1.0}
    assert fit_metrics(m, m + 1)["RMSE [K]"] == 1.0
