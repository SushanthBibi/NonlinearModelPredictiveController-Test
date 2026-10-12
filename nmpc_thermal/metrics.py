"""Key performance indicators listed in the problem statement."""
import numpy as np


def kpis(log, dt, T_ref=92.0, T_warm=85.0, p=None):
    t = log["t"]
    warm_idx = np.argmax(log["T_e"] >= T_warm) if np.any(log["T_e"] >= T_warm) else None
    warm_time = float(t[warm_idx]) if warm_idx is not None else float("nan")
    hot = slice(warm_idx, None) if warm_idx is not None else slice(0, 0)

    E = lambda P: float(np.sum(P) * dt / 3.6e6)  # W*s -> kWh
    out = {
        "warm-up time to 85 degC [s]": warm_time,
        "coolant RMS dev. from target (warm) [K]": float(np.sqrt(np.mean((log["T_e"][hot] - T_ref) ** 2))),
        "coolant std (warm) [K]": float(np.std(log["T_e"][hot])),
        "oil std (warm) [K]": float(np.std(log["T_o"][hot])),
        "peak coolant temp [degC]": float(np.max(log["T_e"])),
        "peak oil temp [degC]": float(np.max(log["T_o"])),
        "peak liner metal temp [degC]": float(np.max(log["T_metal"])),
        "fan energy [kWh]": E(log["P_fan"]),
        "pump energy [kWh]": E(log["P_pump"]),
        "cooling-system energy [kWh]": E(log["P_fan"] + log["P_pump"]),
        "friction energy [kWh]": E(log["P_fric"]),
        "fuel consumed [kg]": float(np.sum(log["mdot_fuel"]) * dt),
    }
    if p is not None:
        out["time above coolant limit [s]"] = float(np.sum(log["T_e"] > p.T_e_max) * dt)
    return out


def compare_table(results: dict) -> str:
    names = list(results)
    keys = list(results[names[0]])
    w = max(len(k) for k in keys) + 2
    base, last = names[0], names[-1]
    lines = [f"{'KPI':<{w}}" + "".join(f"{n:>14}" for n in names)
             + f"{last + ' vs ' + base:>26}"]
    lines.append("-" * len(lines[0]))
    for k in keys:
        vals = [results[n][k] for n in names]
        row = f"{k:<{w}}" + "".join(f"{v:>14.3f}" for v in vals)
        if np.isfinite(vals[0]) and abs(vals[0]) > 1e-9:
            row += f"{100 * (vals[-1] - vals[0]) / abs(vals[0]):>+25.1f}%"
        lines.append(row)
    return "\n".join(lines)
