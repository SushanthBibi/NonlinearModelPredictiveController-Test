"""Synthetic heavy-duty truck mission and its engine-side disturbances.

The cycle is defined by a speed and road-grade profile; engine brake power and
speed follow from simple longitudinal vehicle dynamics. Replace with a measured
cycle (e.g. VECTO long-haul / regional-delivery) when available.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Vehicle:
    mass: float = 40000.0     # [kg]
    CdA: float = 6.0          # [m^2]
    Crr: float = 0.006
    rho: float = 1.2
    eta_dl: float = 0.92      # driveline efficiency
    P_max: float = 380.0e3    # engine rated power [W]
    P_idle: float = 3.0e3     # accessory load at idle [W]
    g: float = 9.81


# (duration [s], target speed [m/s], grade [-])
DEFAULT_SEGMENTS = [
    (60, 0.0, 0.0),      # cold start, idle
    (240, 12.0, 0.0),    # urban / ramp-up
    (60, 0.0, 0.0),      # stop
    (420, 22.0, 0.0),    # highway cruise
    (300, 16.0, 0.045),  # long climb
    (180, 20.0, -0.04),  # descent (engine braking)
    (360, 22.0, 0.0),    # highway
    (240, 15.0, 0.05),   # second climb
    (240, 22.0, 0.0),    # highway
    (60, 0.0, 0.0),      # stop
]


def engine_speed(v, grade):
    """Crude gear-selection map [rpm]."""
    n = np.where(v < 0.5, 600.0, np.clip(850.0 + 20.0 * v, 900.0, 1500.0))
    return n + np.where(grade > 0.02, 250.0, 0.0)


def build_cycle(segments=DEFAULT_SEGMENTS, dt=1.0, T_amb=25.0,
                veh: Vehicle = Vehicle(), accel_max=0.4):
    """Return a dict of time series sampled every dt seconds."""
    target = np.concatenate([np.full(int(dur / dt), vs) for dur, vs, _ in segments])
    grade = np.concatenate([np.full(int(dur / dt), gr) for dur, _, gr in segments])
    n = target.size

    v = np.zeros(n)
    for k in range(1, n):  # rate-limit speed changes
        dv = np.clip(target[k] - v[k - 1], -accel_max * 1.5 * dt, accel_max * dt)
        v[k] = max(v[k - 1] + dv, 0.0)
    a = np.gradient(v, dt)

    F = (veh.mass * a + 0.5 * veh.rho * veh.CdA * v ** 2
         + veh.mass * veh.g * veh.Crr * (v > 0.1)
         + veh.mass * veh.g * np.sin(np.arctan(grade)))
    P_wheel = F * v
    P_b = np.where(P_wheel > 0, P_wheel / veh.eta_dl, P_wheel * veh.eta_dl)
    P_b = np.clip(P_b, -150e3, veh.P_max) + veh.P_idle

    t = np.arange(n) * dt
    return dict(t=t, v=v, grade=grade, N=engine_speed(v, grade), P_b=P_b,
                T_amb=np.full(n, float(T_amb)), dt=dt)


def disturbance(cycle, k):
    """Disturbance vector d = [N, P_b, v, T_amb] at sample k (clamped to the end)."""
    k = min(k, cycle["t"].size - 1)
    return np.array([cycle["N"][k], cycle["P_b"][k], cycle["v"][k], cycle["T_amb"][k]])
