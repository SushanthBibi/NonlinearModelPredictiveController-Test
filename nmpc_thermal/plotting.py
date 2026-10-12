import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def plot_comparison(cycle, logs, p, path):
    t = cycle["t"] / 60.0
    fig, ax = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    ax[0].plot(t, cycle["v"] * 3.6, color="0.3")
    ax0b = ax[0].twinx()
    ax0b.plot(t, cycle["P_b"] / 1e3, color="tab:orange", alpha=0.6)
    ax[0].set_ylabel("speed [km/h]")
    ax0b.set_ylabel("brake power [kW]", color="tab:orange")
    ax[0].set_title("Drive cycle")

    styles = {"conventional": dict(color="tab:blue"), "PI": dict(color="tab:green"),
              "NMPC": dict(color="tab:red")}
    for name, log in logs.items():
        s = styles.get(name, {})
        ax[1].plot(t, log["T_e"], label=name, **s)
        ax[2].plot(t, log["T_o"], label=name, **s)
        ax[3].plot(t, log["u_fan"], label=name, **s)
        ax[4].plot(t, log["u_pump"], label=name, **s)
        ax[4].plot(t, log["u_th"], ls="--", label=f"{name} thermostat", **s)
        ax[5].plot(t, (log["P_fan"] + log["P_pump"]) / 1e3, label=name, **s)
    ax[1].axhline(p.T_e_max, color="k", ls=":", label="limit")
    ax[1].set_ylabel("coolant T_e [degC]")
    ax[2].axhline(p.T_o_max, color="k", ls=":")
    ax[2].set_ylabel("oil T_o [degC]")
    ax[3].set_ylabel("fan speed [-]")
    ax[4].set_ylabel("pump (solid)\nthermostat (dashed)")
    ax[5].set_ylabel("fan+pump power [kW]")
    ax[5].set_xlabel("time [min]")
    for a in ax[1:]:
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
