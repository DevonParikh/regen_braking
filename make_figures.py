"""
make_figures.py - every poster / report figure, rebuilt from data/*.csv.
Run `python studies.py` first, then:

    python make_figures.py          -> figures/*.png (200 dpi, poster-ready)

Colors follow the entity everywhere: regen = blue, friction brakes = orange,
demand = black, and one fixed color per regen limit (torque / power / battery / grip).
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.ticker import NullFormatter  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
FIG = os.path.join(HERE, "figures")

INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#a3a29c"
C = {"regen": "#2a78d6", "friction": "#eb6834", "demand": INK,
     "torque": "#1baf7a", "power": "#6250d6", "battery": "#eda100", "grip": "#008300",
     "rear": "#2a78d6", "front": "#e87ba4", "icurve": "#6250d6", "theory": INK2, "blind": "#e34948"}
SERIES = [("rear-axle grip limit", C["rear"], "-"), ("front-axle grip limit", C["front"], "--"),
          ("rear-axle I-curve", C["icurve"], "-.")]
LIMIT_KEYS = [("lim_torque", "motor torque", "torque"), ("lim_power", "motor power", "power"),
              ("lim_battery", "battery charge", "battery"), ("lim_grip", "tire grip", "grip")]

plt.rcParams.update({
    "font.size": 12, "axes.titlesize": 13, "axes.labelsize": 12, "axes.titlelocation": "left",
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": "#e6e5e1", "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "lines.linewidth": 2.0, "axes.axisbelow": True, "figure.dpi": 100, "savefig.dpi": 200, "savefig.bbox": "tight",
})


def load(name):
    path = os.path.join(DATA, name)
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except (ValueError, TypeError):
                pass
    return rows


def col(rows, k):
    return [r[k] for r in rows]


def save(fig, name):
    os.makedirs(FIG, exist_ok=True)
    path = os.path.join(FIG, name)
    fig.savefig(path)
    plt.close(fig)
    print("  " + path)


def end_label(ax, x, y, text, color, dy=0):
    ax.annotate(text, (x, y), xytext=(4, dy), textcoords="offset points",
                color=INK2, fontsize=10, va="center")


# -----------------------------------------------------------------------------
def fig_limit_envelope():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
    for ax, fname, title in [(axes[0], "limit_envelope.csv", "Regen force limits, SOC = 0.60"),
                             (axes[1], "limit_envelope_highsoc.csv", "Same car, nearly full battery (SOC = 0.96)")]:
        rows = load(fname)
        v = col(rows, "speed")
        ymax = 14.0
        for key, label, ck in LIMIT_KEYS[:3]:
            y = [min(q / 1e3, ymax * 1.5) for q in col(rows, key)]
            ax.plot(v, y, color=C[ck], label=label + " limit")
        grip = [q / 1e3 for q in col(rows, "lim_grip_a3.0")]
        ax.plot(v, grip, color=C["grip"], ls="--", label="tire grip limit μN_r (at 3 m/s²)")
        icv = [q / 1e3 for q in col(rows, "icurve_a3.0")]
        ax.plot(v, icv, color=C["icurve"], ls="-.", lw=1.6, label="I-curve rear share (at 3 m/s²)")
        dem = [q / 1e3 for q in col(rows, "demand_a3.0")]
        ax.plot(v, dem, color=C["demand"], lw=2.5, label="braking demand at 3 m/s²")
        regen_max = [min(rows[i]["lim_torque"], rows[i]["lim_power"], rows[i]["lim_battery"],
                         rows[i]["lim_grip_a3.0"], rows[i]["demand_a3.0"]) / 1e3 for i in range(len(rows))]
        ax.fill_between(v, 0, regen_max, color=C["regen"], alpha=0.15, lw=0, label="regen supplied")
        ax.fill_between(v, regen_max, dem, color=C["friction"], alpha=0.18, lw=0, label="friction brakes")
        ax.set_ylim(0, ymax)
        ax.set_xlim(0, 30)
        ax.set_xlabel("speed (m/s)")
        ax.set_title(title)
    axes[0].set_ylabel("force at the tires (kN)")
    axes[0].legend(ncol=4, loc="upper center", bbox_to_anchor=(1.05, -0.17), fontsize=10)
    save(fig, "fig01_limit_envelope.png")


def fig_force_split(kind):
    rows = load("ts_%s_optimal.csv" % kind)
    t = col(rows, "t")
    fig, ax = plt.subplots(figsize=(8, 4.4))
    regen_cap = [min(r["lim_torque"], r["lim_power"], r["lim_battery"], r["lim_grip"]) / 1e3 for r in rows]
    ax.plot(t, [min(q, 15) for q in regen_cap], color=MUTED, ls=":", lw=1.8, label="regen limit (smallest of 4)")
    ax.plot(t, [q / 1e3 for q in col(rows, "F_req")], color=C["demand"], lw=2.5, label="braking demand")
    ax.plot(t, [q / 1e3 for q in col(rows, "F_regen")], color=C["regen"], label="regen")
    ax.plot(t, [q / 1e3 for q in col(rows, "F_fric")], color=C["friction"], label="friction brakes")
    if kind == "corner":
        ax.plot(t, [q / 1e3 for q in col(rows, "F_lat")], color=INK2, lw=1.4, ls="--",
                label="sideways tire force")
    ax.set_ylim(0, 12)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("force (kN)")
    ax.set_title("Optimal braking split - %s" % ("straight-line stop from 30 m/s" if kind == "straight"
                                                  else "90° corner, R = 40 m"))
    ax.legend(loc="upper right", fontsize=10)
    save(fig, "fig02_force_split_%s.png" % kind)


def fig_energy_budget():
    rows = load("headline_summary.csv")
    runs = [rows[0], rows[1]]
    names = ["Straight stop\n(30 → 0 m/s)", "90° corner\n(20 → 9 m/s)"]
    parts = [("E_stored_kJ", "stored in battery", C["regen"]),
             ("E_fric_kJ", "friction-brake heat", C["friction"]),
             ("E_loss_kJ", "conversion loss (1 − η)", "#1baf7a"),
             ("E_drag_kJ", "aerodynamic drag", "#e87ba4"),
             ("E_roll_kJ", "rolling resistance", "#008300"),
             ("KE_f_kJ", "kinetic energy left", "#d8d7d2")]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    for i, r in enumerate(runs):
        left = 0.0
        for k, label, color in parts:
            w = r[k]
            ax.barh(i, w, left=left, color=color, edgecolor="white", linewidth=2, height=0.6,
                    label=label if i == 0 else None)
            if w > 35:
                ax.text(left + w / 2, i, "%.0f" % w, ha="center", va="center", fontsize=10,
                        color="white" if color in (C["regen"], "#008300") else INK)
            left += w
    ax.set_yticks([0, 1])
    ax.set_yticklabels(names)
    ax.invert_yaxis()
    ax.set_xlabel("energy (kJ)")
    ax.grid(axis="y", visible=False)
    ax.set_title("Where the car's kinetic energy goes (optimal strategy)")
    ax.legend(ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.25), fontsize=10)
    save(fig, "fig03_energy_budget.png")


def fig_grip_corner():
    opt = load("ts_tight_optimal.csv")
    icv = load("ts_tight_icurve.csv")
    bl = load("ts_tight_gripblind.csv")
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(col(bl, "t"), col(bl, "grip_use"), color=C["blind"], label="100% regen (grip-blind)")
    ax.plot(col(opt, "t"), col(opt, "grip_use"), color=C["regen"], label="optimal (grip limit μN_r)")
    ax.plot(col(icv, "t"), col(icv, "grip_use"), color=C["icurve"], ls="-.", label="I-curve")
    ax.axhline(1.0, color=INK, lw=1.2)
    ax.text(0.2, 1.015, "rear tires slide (unstable)", fontsize=10, color=INK2, va="bottom")
    ax.set_ylim(0, 1.15)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("rear-axle grip used, √(F_x²+F_y²)/(μN_r)")
    ax.set_title("Rear-axle grip in a 35 m corner")
    ax.legend(loc="center right", fontsize=10, bbox_to_anchor=(1.0, 0.6))
    save(fig, "fig04_grip_corner.png")


def fig_decel_sweep():
    rows = load("sweep_decel.csv")
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    for name, color, ls in SERIES:
        rr = [r for r in rows if r["series"] == name]
        a = col(rr, "a_brake")
        axes[0].plot(a, col(rr, "E_stored_kJ"), color=color, marker="o", ms=5, ls=ls, label=name)
        axes[1].plot(a, col(rr, "eta_regen_pct"), color=color, marker="o", ms=5, ls=ls, label=name)
    rr = [r for r in rows if r["series"] == "rear-axle grip limit"]
    axes[0].plot(col(rr, "a_brake"), col(rr, "E_stored_theory_kJ"), color=C["theory"], ls=":",
                 lw=2, label="theory (EQUATIONS A3)")
    best = max(rr, key=lambda r: r["E_stored_kJ"])
    axes[0].annotate("best: %.2f m/s²\n%.0f kJ" % (best["a_brake"], best["E_stored_kJ"]),
                     (best["a_brake"], best["E_stored_kJ"]), xytext=(25, -5),
                     textcoords="offset points", fontsize=10, color=INK2,
                     arrowprops=dict(arrowstyle="-", color=MUTED))
    axes[0].set_xlabel("braking deceleration (m/s²)")
    axes[0].set_ylabel("energy stored (kJ)")
    axes[0].set_title("Energy stored per stop from 30 m/s")
    axes[0].legend(fontsize=10)
    axes[1].set_xlabel("braking deceleration (m/s²)")
    axes[1].set_ylabel("η_regen = E_stored / E_B (%)")
    axes[1].set_ylim(0, 75)
    axes[1].set_title("Fraction of braking energy recovered")
    save(fig, "fig05_decel_sweep.png")


def fig_radius_sweep():
    rows = load("sweep_radius.csv")
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    R_min = min(r["R"] for r in rows if r["t_infeasible_s"] == 0)
    for name, color, ls in SERIES[:2]:
        rr = [r for r in rows if r["series"] == name and r["t_infeasible_s"] == 0 and r["R"] <= 80]
        ax.plot(col(rr, "R"), col(rr, "E_stored_kJ"), color=color, marker="o", ms=5, ls=ls, label=name)
    icv = [r for r in rows if r["series"] == "rear-axle I-curve" and r["R"] == 40.0][0]
    ax.axvspan(25, R_min - 0.5, color="#e6e5e1", lw=0)
    ax.text(R_min - 1, 0.5, "car can't\nmake the\ncorner", ha="right", va="center", fontsize=10, color=INK2,
            transform=ax.get_xaxis_transform())
    ax.set_xlim(25, 80)
    ax.set_xlabel("corner radius (m)")
    ax.set_ylabel("energy stored (kJ)")
    ax.set_title("Tighter corners leave less rear grip for regen")
    ax.text(79, ax.get_ylim()[0] + 0.5, "(rear I-curve: %.0f kJ at every radius)" % icv["E_stored_kJ"],
            ha="right", va="bottom", fontsize=10, color=INK2)
    ax.legend(fontsize=10, loc="center right")
    save(fig, "fig06_radius_sweep.png")


def fig_soc():
    rows = load("soc_study.csv")
    consts = {r["quantity"]: r["value"] for r in load("derived_constants.csv")}
    soc_c = consts["SOC above which battery always binds"]
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.plot(col(rows, "SOC0"), col(rows, "E_stored_kJ"), color=C["battery"], marker="o", ms=7)
    ax.axvline(soc_c, color=MUTED, ls="--", lw=1.2)
    ax.text(soc_c - 0.004, 120, "battery cap\nstarts binding\n(SOC ≈ %.3f)" % soc_c,
            ha="right", fontsize=10, color=INK2)
    ax.set_xlabel("battery state of charge at start of stop")
    ax.set_ylabel("energy stored (kJ)")
    ax.set_ylim(0, 450)
    ax.set_title("A nearly full battery can't absorb regen")
    save(fig, "fig07_soc.png")


def fig_strategies():
    rows = load("strategy_comparison.csv")
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6), sharey=True)
    for ax, kind, title in [(axes[0], "straight", "Straight stop"), (axes[1], "corner", "90° corner, R = 40 m"),
                            (axes[2], "tight", "90° corner, R = 35 m")]:
        rr = [r for r in rows if r["scenario_kind"] == kind]
        names = [r["strategy"].replace(" (grip-blind)", "\n(grip-blind)").replace(" (grip limit", "\n(grip limit")
                 .replace(" (ideal", "\n(ideal") for r in rr]
        y = list(range(len(rr)))
        colors = [C["regen"] if r["strategy"].startswith("optimal") else
                  C["icurve"] if r["strategy"].startswith("I-curve") else MUTED for r in rr]
        ax.barh(y, col(rr, "E_stored_kJ"), color=colors, height=0.6, edgecolor="white")
        for i, r in enumerate(rr):
            txt = "%.0f kJ" % r["E_stored_kJ"]
            if r["t_grip_violation_s"] > 0:
                txt += "  ⚠ slides"
            ax.text(r["E_stored_kJ"] + 5, i, txt, va="center", fontsize=10, color=INK)
        ax.set_yticks(y)
        ax.set_yticklabels(names, fontsize=10)
        ax.grid(axis="y", visible=False)
        ax.set_xlim(0, max(col(rr, "E_stored_kJ")) * 1.5)
        ax.set_xlabel("energy stored (kJ)")
        ax.set_title(title)
    axes[0].invert_yaxis()
    save(fig, "fig08_strategies.png")


def value_text(r):
    if r["parameter"] == "P_motor_max":
        return "%g kW" % (r["value"] / 1e3)
    return "%g" % r["value"]


def fig_sensitivity():
    rows = [r for r in load("sensitivity.csv") if r["scenario_kind"] in ("straight", "tight")]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), sharey=True)
    labels = []
    for r in rows:
        if r["label"] not in labels:
            labels.append(r["label"])
    # sort by largest effect in the corner
    def eff(lbl):
        return max(abs(r["pct_change_E"]) for r in rows if r["label"] == lbl and r["scenario_kind"] != "corner")
    labels.sort(key=eff)
    for ax, kind, title in [(axes[0], "straight", "Straight stop"), (axes[1], "tight", "90° corner, R = 35 m")]:
        for i, lbl in enumerate(labels):
            rr = [r for r in rows if r["label"] == lbl and r["scenario_kind"] == kind]
            rr.sort(key=lambda r: -abs(r["pct_change_E"]))      # small bars drawn on top
            for r in rr:
                lo = r["value"] < r["base_value"]
                y = i + (-0.18 if lo else 0.18)
                x = r["pct_change_E"]
                if r["t_infeasible_s"] > 0:
                    continue
                ax.barh(y, x, color=C["friction"] if lo else C["regen"], height=0.34)
                if r["t_infeasible_s"] > 0:
                    continue
                if abs(x) >= 1.0:
                    ax.text(x + (0.6 if x >= 0 else -0.6), y, value_text(r), va="center",
                            fontsize=8.5, color=INK2, ha="left" if x >= 0 else "right")
            bad = [value_text(r) for r in rr if r["t_infeasible_s"] > 0]
            if bad:
                ax.text(-1, i - 0.18, "%s = %s: car can't make the corner" % (lbl.split(" ")[-1] if "mu" in lbl else lbl,
                        ", ".join(bad)), ha="right", va="center", fontsize=8.5, color=INK2, style="italic")
        ax.axvline(0, color=INK, lw=1)
        ax.set_yticks(range(len(labels)))
        ax.set_yticklabels(labels, fontsize=10)
        ax.grid(axis="y", visible=False)
        ax.set_xlabel("% change in energy stored")
        ax.set_title(title)
        lo_, hi_ = ax.get_xlim()
        ax.set_xlim(lo_ - 4, hi_ + 4)
    axes[1].legend(handles=[Patch(color=C["friction"], label="parameter lowered"),
                            Patch(color=C["regen"], label="parameter raised")],
                   loc="lower left", fontsize=10)
    fig.suptitle("Sensitivity of the result to each assumed parameter (labels = tested value)",
                 x=0.01, ha="left", fontsize=13)
    save(fig, "fig09_sensitivity.png")


def fig_convergence():
    rows = load("convergence.csv")
    fig, ax = plt.subplots(figsize=(7, 4.2))
    for kind, color, ls in [("straight", C["regen"], "-"), ("corner", C["power"], "--")]:
        rr = [r for r in rows if r["scenario_kind"] == kind][:-1]      # drop the reference point
        ax.loglog(col(rr, "dt"), [max(q, 1e-6) for q in col(rr, "pct_diff_vs_finest")], color=color,
                  ls=ls, marker="o", ms=6, label=kind)
    ticks = [0.0005, 0.001, 0.002, 0.004, 0.008]
    ax.set_xticks(ticks)
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.set_xticklabels(["%g" % q for q in ticks])
    ax.axvline(0.002, color=MUTED, ls=":", lw=1.2)
    ax.text(0.00205, 0.97, "dt used", fontsize=10, color=INK2, transform=ax.get_xaxis_transform(), va="top")
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.set_yticks([0.001, 0.01, 0.1])
    ax.set_yticklabels(["0.001%", "0.01%", "0.1%"])
    ax.set_xlabel("time step dt (s)")
    ax.set_ylabel("% difference from dt = 0.00025 s")
    ax.set_title("Time-step error stays below 0.1% for every dt tested")
    ax.legend(fontsize=10)
    save(fig, "fig10_convergence.png")


def fig_profile():
    rows = load("profile_optimization.csv")
    base = [r for r in rows if r.get("note")][0]
    grid = [r for r in rows if not r.get("note") and r["t_infeasible_s"] == 0]
    vs = sorted(set(col(grid, "v_switch")))
    best_per_v = [max((r for r in grid if r["v_switch"] == v), key=lambda r: r["E_stored_kJ"]) for v in vs]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    ax = axes[0]
    ax.plot(vs, [r["E_stored_kJ"] for r in best_per_v], color=C["regen"], marker="o", ms=6,
            label="best two-phase profile")
    ax.axhline(base["E_stored_kJ"], color=INK2, ls="--", lw=1.4, label="constant 3 m/s²")
    ax.set_xlabel("switch speed v_s (m/s)")
    ax.set_ylabel("energy stored (kJ)")
    ax.set_title("Same 150 m stop, different braking profiles")
    ax.legend(fontsize=10)
    best = max(grid, key=lambda r: r["E_stored_kJ"])
    ts_b = load("ts_straight_best_profile.csv")
    ts_c = load("ts_straight_optimal.csv")
    ax = axes[1]
    ax.plot(col(ts_c, "s"), col(ts_c, "speed"), color=INK2, ls="--", label="constant 3 m/s²")
    ax.plot(col(ts_b, "s"), col(ts_b, "speed"), color=C["regen"],
            label="best: %.1f m/s² above %.0f m/s, then %.2f" % (best["a1"], best["v_switch"], best["a2"]))
    ax.set_xlabel("distance (m)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("Speed profile")
    ax.legend(fontsize=10)
    save(fig, "fig11_profile.png")


def fig_hierarchy():
    rows = load("model_hierarchy.csv")
    fig, ax = plt.subplots(figsize=(9, 3.6))
    y = list(range(len(rows)))
    ax.barh(y, col(rows, "E_stored_kJ"), color=C["regen"], height=0.55, edgecolor="white")
    for i, r in enumerate(rows):
        ax.text(r["E_stored_kJ"] + 6, i, "%.0f kJ (η_regen %.0f%%)" % (r["E_stored_kJ"], r["eta_regen_pct"]),
                va="center", fontsize=10)
    ax.set_yticks(y)
    ax.set_yticklabels([r["level"] for r in rows], fontsize=10)
    ax.invert_yaxis()
    ax.grid(axis="y", visible=False)
    ax.set_xlim(0, 760)
    ax.set_xlabel("energy stored, straight stop (kJ)")
    ax.set_title("Building the model up one effect at a time")
    save(fig, "fig12_model_hierarchy.png")


def main():
    print("Writing figures -> " + FIG)
    fig_limit_envelope()
    fig_force_split("straight")
    fig_force_split("corner")
    fig_energy_budget()
    fig_grip_corner()
    fig_decel_sweep()
    fig_radius_sweep()
    fig_soc()
    fig_strategies()
    fig_sensitivity()
    fig_convergence()
    fig_profile()
    fig_hierarchy()


if __name__ == "__main__":
    main()
