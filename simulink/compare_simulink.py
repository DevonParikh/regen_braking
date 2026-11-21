"""
compare_simulink.py - Milestone 4: compare Simulink results with the VPython model.

1. In MATLAB, export each Simulink run to CSV with export_results.m, giving
       simulink/results_straight.csv
       simulink/results_corner.csv
   Required column: t, speed. Optional (compared if present): F_regen, F_fric, E_stored,
   SOC, N_f, N_r, F_lat. SI units (s, m/s, N, J).
2. From the repo root:   python simulink/compare_simulink.py
   (run studies.py first so data/ts_*_optimal.csv exist)

Writes data/simulink_comparison.csv and figures/simulink_overlay_<scenario>.png.
Metrics (EQUATIONS.md 10): percent difference of final values, and normalized RMS
difference of each time series after interpolating VPython onto the Simulink time grid.
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "data")
FIG = os.path.join(ROOT, "figures")

SIGNALS = [("speed", "speed (m/s)", 1.0), ("F_regen", "regen force (kN)", 1e-3),
           ("F_fric", "friction-brake force (kN)", 1e-3), ("E_stored", "energy stored (kJ)", 1e-3),
           ("SOC", "state of charge", 1.0), ("N_f", "front axle load (kN)", 1e-3),
           ("N_r", "rear axle load (kN)", 1e-3), ("F_lat", "sideways tire force (kN)", 1e-3)]


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for k in rows[0].keys():
        try:
            cols[k.strip()] = np.array([float(r[k]) for r in rows])
        except (ValueError, TypeError):
            pass
    return cols


def compare(vp, sl):
    """vp, sl: dicts of numpy arrays. Returns list of metric dicts."""
    t = sl["t"]
    out = []
    for name, label, _ in SIGNALS:
        if name not in vp or name not in sl:
            continue
        mask = (t >= vp["t"][0]) & (t <= vp["t"][-1])
        if mask.sum() < 2:
            continue
        a = np.interp(t[mask], vp["t"], vp[name])
        b = sl[name][mask]
        rng = max(np.max(a) - np.min(a), 1e-12)
        nrmse = float(np.sqrt(np.mean((a - b) ** 2)) / rng)
        fa, fb = float(vp[name][-1]), float(sl[name][-1])
        out.append({"signal": name, "label": label,
                    "vpython_final": fa, "simulink_final": fb,
                    "pct_diff_final": 100 * (fb - fa) / fa if fa != 0 else "",
                    "vpython_peak": float(np.max(vp[name])), "simulink_peak": float(np.max(sl[name])),
                    "nrmse_pct": 100 * nrmse})
    out.append({"signal": "duration", "label": "run duration (s)",
                "vpython_final": float(vp["t"][-1]), "simulink_final": float(t[-1]),
                "pct_diff_final": 100 * (t[-1] - vp["t"][-1]) / vp["t"][-1]})
    return out


def plot(vp, sl, kind, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = [s for s in SIGNALS if s[0] in vp and s[0] in sl]
    n = len(names)
    cols = 2 if n > 1 else 1
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6.5 * cols, 2.8 * rows), squeeze=False)
    for ax, (name, label, scale) in zip(axes.flat, names):
        ax.plot(vp["t"], vp[name] * scale, color="#2a78d6", lw=2, label="VPython model")
        ax.plot(sl["t"], sl[name] * scale, color="#eb6834", lw=2, ls="--", label="Simulink")
        ax.set_title(label, loc="left", fontsize=11)
        ax.set_xlabel("time (s)")
        ax.grid(alpha=0.25)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    for ax in list(axes.flat)[n:]:
        ax.axis("off")
    axes.flat[0].legend(frameon=False)
    fig.suptitle("VPython vs Simulink - %s scenario" % kind, x=0.01, ha="left", fontsize=13)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main(sim_dir=HERE, data_dir=DATA, fig_dir=FIG):
    all_rows = []
    found = False
    for kind in ["straight", "corner"]:
        sl_path = os.path.join(sim_dir, "results_%s.csv" % kind)
        vp_path = os.path.join(data_dir, "ts_%s_optimal.csv" % kind)
        if not os.path.exists(sl_path):
            print("  (no %s yet - export it from MATLAB with export_results.m)" % sl_path)
            continue
        found = True
        vp, sl = read_csv(vp_path), read_csv(sl_path)
        rows = compare(vp, sl)
        for r in rows:
            r["scenario"] = kind
        all_rows += rows
        plot(vp, sl, kind, os.path.join(fig_dir, "simulink_overlay_%s.png" % kind))
        print("\n%s:" % kind)
        for r in rows:
            pd = r.get("pct_diff_final", "")
            print("  %-26s VPython %10.4g   Simulink %10.4g   diff %s   NRMSE %s" % (
                r["label"], r["vpython_final"], r["simulink_final"],
                ("%+.2f%%" % pd) if pd != "" else "-",
                ("%.2f%%" % r["nrmse_pct"]) if "nrmse_pct" in r else "-"))
    if found:
        cols = ["scenario", "signal", "label", "vpython_final", "simulink_final", "pct_diff_final",
                "vpython_peak", "simulink_peak", "nrmse_pct"]
        os.makedirs(data_dir, exist_ok=True)
        with open(os.path.join(data_dir, "simulink_comparison.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(all_rows)
    return all_rows


if __name__ == "__main__":
    main(*sys.argv[1:])
