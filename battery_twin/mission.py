"""Regional-haul mission for a 40 t battery-electric truck with a fast-charging stop.

Drive -> 30-min megawatt charging during the driver's break -> drive.
"""
import numpy as np

MASS, CDA, CRR, RHO, G = 40000.0, 5.5, 0.0055, 1.2, 9.81
ETA_DRIVE, ETA_REGEN, AUX = 0.90, 0.60, 5e3
P_TRACTION_MAX, P_REGEN_MAX = 480e3, 250e3

# (duration [s], speed [m/s], grade [-], charger power [W])
SEGMENTS = [
    (300, 14, 0.00, 0), (1200, 22, 0.00, 0), (600, 17, 0.035, 0), (420, 22, -0.03, 0),
    (900, 22, 0.00, 0), (480, 15, 0.045, 0), (300, 22, -0.035, 0), (600, 22, 0.0, 0),
    (1800, 0, 0.0, 750e3),                                   # MCS charging stop
    (900, 22, 0.0, 0), (540, 16, 0.04, 0), (360, 22, -0.03, 0), (600, 22, 0.0, 0),
]


def build_mission(T_amb=25.0, segments=SEGMENTS, dt=1.0):
    v = np.concatenate([np.full(int(s[0] / dt), s[1], float) for s in segments])
    grade = np.concatenate([np.full(int(s[0] / dt), s[2], float) for s in segments])
    chg = np.concatenate([np.full(int(s[0] / dt), s[3], float) for s in segments])
    # smooth speed transitions (first-order, ~20 s)
    vs = np.copy(v)
    for k in range(1, v.size):
        vs[k] = vs[k - 1] + (v[k] - vs[k - 1]) * min(dt / 20.0, 1.0)
    a = np.gradient(vs, dt)
    F = MASS * a + 0.5 * RHO * CDA * vs ** 2 + MASS * G * (CRR * (vs > 0.5) + grade)
    P_wheel = F * vs
    P_drive = np.where(P_wheel > 0, np.minimum(P_wheel / ETA_DRIVE, P_TRACTION_MAX),
                       np.maximum(P_wheel * ETA_REGEN, -P_REGEN_MAX)) + AUX
    T = np.full(v.size, float(T_amb))
    return dict(t=np.arange(v.size) * dt, v=vs, grade=grade, P_drive=P_drive,
                P_chg_max=chg, T_amb=T, dt=dt)


def disturbance(m, k):
    k = min(max(k, 0), m["t"].size - 1)
    return np.array([m["P_drive"][k], m["P_chg_max"][k], m["v"][k], m["T_amb"][k]])


def charging_window(m):
    idx = np.nonzero(m["P_chg_max"] > 0)[0]
    return (m["t"][idx[0]], m["t"][idx[-1]]) if idx.size else None
