"""
studies.py - every study, sweep and validation for the project. Writes data/*.csv and
data/results_summary.md. No graphics (run make_figures.py afterwards for the plots).

    python studies.py            # everything (about 2-4 minutes)
    python studies.py quick      # skips the slowest studies (profile grid, finest dt)

Or from a notebook:
    import studies; studies.run_all()

Who owns what (each section maps to the plan of work / EQUATIONS.md):
    Devon  headline runs, convergence, Simulink export      (EQ 6, 10)
    Alex   analytic check, limit envelope, SOC / efficiency (EQ 7, 8)
    Aiden  grip sensitivity, axle, radius, load transfer    (EQ 3, 5)
    All    strategies, profile optimization, sensitivity    (EQ 4, 9, 11)
"""
import csv
import math
import os
import sys
import time

import regen_core as rc

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
TIGHT_R = 35.0        # m, tighter corner where rear grip actually limits regen
HIGH_SOC = 0.96      # nearly full battery, above the critical SOC (~0.93)


# -----------------------------------------------------------------------------
# CSV helpers
# -----------------------------------------------------------------------------
def write_csv(rows, name, columns=None):
    os.makedirs(DATA, exist_ok=True)
    path = os.path.join(DATA, name)
    if not rows:
        return path
    if columns is None:
        columns = list(rows[0].keys())
        for r in rows[1:]:
            for k in r:
                if k not in columns:
                    columns.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: _clean(r.get(k, "")) for k in columns})
    return path


def _clean(v):
    if isinstance(v, float):
        return float("%.6g" % v)
    return v


def write_timeseries(res, name):
    return write_csv(rc.timeseries_rows(res), name, rc.TIMESERIES_COLUMNS)


def run(car, scen, strat):
    return rc.simulate(car, scen, strat)


# -----------------------------------------------------------------------------
# 1. Headline runs (Milestones 2 and 3)                       Devon
# -----------------------------------------------------------------------------
def headline_runs():
    car = rc.Car()
    opt = rc.optimal_strategy()
    icv = rc.icurve_strategy()
    blind = rc.Strategy("fixed 100% regen (grip-blind)", "fixed", 1.0)
    tight = rc.make_corner_scenario()
    tight.R = TIGHT_R
    tight.name = "90 deg corner (R = %g m), braking 20 -> 9 m/s at 3 m/s^2" % TIGHT_R
    out = {}
    out["straight"] = run(car, rc.make_straight_scenario(), opt)
    out["corner"] = run(car, rc.make_corner_scenario(), opt)
    out["straight_icurve"] = run(car, rc.make_straight_scenario(), icv)
    out["corner_icurve"] = run(car, rc.make_corner_scenario(), icv)
    out["tight"] = run(car, tight, opt)
    out["tight_icurve"] = run(car, tight, icv)
    out["tight_blind"] = run(car, tight, blind)
    write_timeseries(out["straight"], "ts_straight_optimal.csv")
    write_timeseries(out["corner"], "ts_corner_optimal.csv")
    write_timeseries(out["straight_icurve"], "ts_straight_icurve.csv")
    write_timeseries(out["corner_icurve"], "ts_corner_icurve.csv")
    write_timeseries(out["tight"], "ts_tight_optimal.csv")
    write_timeseries(out["tight_icurve"], "ts_tight_icurve.csv")
    write_timeseries(out["tight_blind"], "ts_tight_gripblind.csv")
    write_csv([r.summary_row() for r in out.values()], "headline_summary.csv")
    return out


# -----------------------------------------------------------------------------
# 2. Strategy comparison + best constant split                 All
# -----------------------------------------------------------------------------
def strategy_tables():
    car = rc.Car()
    rows = []
    tight = rc.make_corner_scenario()
    tight.R = TIGHT_R
    for kind, scen in [("straight", rc.make_straight_scenario()), ("corner", rc.make_corner_scenario()),
                       ("tight", tight)]:
        for st in rc.standard_strategies():
            r = run(car, scen, st)
            row = r.summary_row()
            row["scenario_kind"] = kind
            rows.append(row)
    write_csv(rows, "strategy_comparison.csv")

    best = []
    tight = rc.make_corner_scenario()
    tight.R = TIGHT_R
    tight.name = "90 deg corner (R = %g m)" % TIGHT_R
    for scen in [rc.make_straight_scenario(), rc.make_corner_scenario(), tight]:
        for label in ["never exceeds grip"]:
            f, E_best, E_opt = rc.best_constant_split(car, scen)
            best.append({"scenario": scen.name, "scenario_kind": scen.kind, "safety_rule": label,
                         "best_safe_constant_regen_pct": 100 * f,
                         "best_constant_E_stored_kJ": E_best / 1e3,
                         "optimal_E_stored_kJ": E_opt / 1e3,
                         "optimal_gain_pct": 100 * (E_opt - E_best) / E_best if E_best > 0 else ""})
    write_csv(best, "best_constant_split.csv")
    return rows, best


# -----------------------------------------------------------------------------
# 3. Analytic check (theory vs simulation)                      Alex   EQ 7
# -----------------------------------------------------------------------------
def analytic_check():
    rows = []
    for a in [1.0, 2.0, 3.0]:
        car = rc.Car()
        car.include_drag = False
        car.include_roll = False
        sc = rc.make_straight_scenario()
        sc.a_brake = a
        r = run(car, sc, rc.optimal_strategy())
        th = rc.analytic_straight_stop(car, sc.v0, a, sc.v_stop)
        rows.append({"a_brake": a, "v_star": th["v_star"], "v_cut": car.v_cut,
                     "E_stored_theory_kJ": th["E_stored"] / 1e3,
                     "E_stored_sim_kJ": r.E_stored / 1e3,
                     "pct_diff": 100 * (r.E_stored - th["E_stored"]) / th["E_stored"],
                     "eta_regen_theory_pct": 100 * th["eta_regen"],
                     "eta_regen_sim_pct": 100 * r.eta_regen(),
                     "grip_binding_s": r.t_binding[3], "battery_binding_s": r.t_binding[2]})
    write_csv(rows, "analytic_check.csv")
    return rows


# -----------------------------------------------------------------------------
# 4. Limit envelope + crossover numbers                         Alex   EQ 8
# -----------------------------------------------------------------------------
def limit_envelope():
    car = rc.Car()
    speeds = [0.25 * i for i in range(1, 121)]       # 0.25 ... 30 m/s
    decels = [1.0, 3.0, 5.0, 7.0]
    rows = rc.limit_envelope(car, speeds, decels)
    write_csv(rows, "limit_envelope.csv")
    hi = rc.limit_envelope(car, speeds, [3.0], SOC=HIGH_SOC)
    write_csv(hi, "limit_envelope_highsoc.csv")

    F_T0 = car.T_motor_max * car.G / car.r_w
    consts = [
        {"quantity": "max regen force from torque, F_T0 (N)", "value": F_T0},
        {"quantity": "decel at which torque binds, F_T0/m (m/s^2)", "value": F_T0 / car.m},
        {"quantity": "I-curve rear share of braking at 3 m/s^2", "value": rc.icurve_rear_fraction(car, 3.0)},
        {"quantity": "power-limit crossover speed at 3 m/s^2, v* = P/(m a) (m/s)",
         "value": car.P_motor_max / (car.m * 3.0)},
        {"quantity": "SOC above which battery always binds", "value": rc.critical_soc(car)},
        {"quantity": "grip-limited decel, rear regen (m/s^2)",
         "value": rc.grip_limited_decel(car, "rear")},
        {"quantity": "grip-limited decel, front regen (m/s^2)",
         "value": rc.grip_limited_decel(car, "front")},
        {"quantity": "energy-optimal constant decel from 30 m/s, theory (m/s^2)",
         "value": rc.analytic_optimal_decel(car, 30.0)},
        {"quantity": "shortest all-regen stop from 30 m/s, rear (m)",
         "value": rc.min_all_regen_stop_distance(car, 30.0, "rear")[0]},
        {"quantity": "shortest all-regen stop from 30 m/s, front (m)",
         "value": rc.min_all_regen_stop_distance(car, 30.0, "front")[0]},
    ]
    write_csv(consts, "derived_constants.csv")
    return consts


# -----------------------------------------------------------------------------
# 5. Battery state of charge (makes the 4th limit appear)        Alex
# -----------------------------------------------------------------------------
def soc_study():
    rows = []
    for soc in [0.60, 0.90, 0.92, 0.93, 0.94, 0.95, 0.96, 0.98]:
        car = rc.Car()
        car.SOC0 = soc
        r = run(car, rc.make_straight_scenario(), rc.optimal_strategy())
        row = r.summary_row()
        row["SOC0"] = soc
        rows.append(row)
    car = rc.Car()
    car.SOC0 = HIGH_SOC
    write_timeseries(run(car, rc.make_straight_scenario(), rc.optimal_strategy()),
                     "ts_straight_highsoc.csv")
    write_csv(rows, "soc_study.csv")
    return rows


# -----------------------------------------------------------------------------
# 6. Model hierarchy (proposal: start simple, add forces)        All
# -----------------------------------------------------------------------------
def model_hierarchy():
    levels = []

    def lvl(name, **kw):
        car = rc.Car()
        for k, v in kw.items():
            setattr(car, k, v)
        return name, car

    levels.append(lvl("1. Stage 1: point mass, motor torque cap only",
                      include_drag=False, include_roll=False, P_motor_max=1e12, P_batt_max=1e12, v_cut=0.0))
    levels.append(lvl("2. + power, battery caps and V_cut", include_drag=False, include_roll=False))
    levels.append(lvl("3. + aerodynamic drag", include_roll=False))
    levels.append(lvl("4. + rolling resistance (Stage 2-3 model)"))
    rows = []
    for name, car in levels:
        r = run(car, rc.make_straight_scenario(), rc.optimal_strategy())
        row = r.summary_row()
        row["level"] = name
        rows.append(row)
    write_csv(rows, "model_hierarchy.csv")
    return rows


# -----------------------------------------------------------------------------
# 7. Axle choice and braking-intensity sweep                     Aiden
# -----------------------------------------------------------------------------
def axle_and_decel_sweep():
    rows = []
    for axle, strat in [("rear", rc.optimal_strategy()), ("front", rc.optimal_strategy()),
                        ("rear", rc.icurve_strategy())]:
        for a_b in [0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0]:
            car = rc.Car()
            car.regen_axle = axle
            sc = rc.make_straight_scenario()
            sc.a_brake = a_b
            r = run(car, sc, strat)
            row = r.summary_row()
            row["a_brake"] = a_b
            row["series"] = axle + ("-axle I-curve" if strat.kind == "icurve" else "-axle grip limit")
            row["E_stored_theory_kJ"] = rc.analytic_decel_energy(car, sc.v0, a_b)["E_stored"] / 1e3
            top = max(range(4), key=lambda i: r.t_binding[i])
            row["most_binding_limit"] = rc.LIMIT_NAMES[top] if r.t_binding[top] > 0 else "none"
            rows.append(row)
    write_csv(rows, "sweep_decel.csv")

    rows_c = []
    for axle, strat in [("rear", rc.optimal_strategy()), ("front", rc.optimal_strategy()),
                        ("rear", rc.icurve_strategy())]:
        for R in [31.0, 33.0, 35.0, 38.0, 40.0, 45.0, 50.0, 65.0, 80.0, 120.0, 200.0]:
            car = rc.Car()
            car.regen_axle = axle
            sc = rc.make_corner_scenario()
            sc.R = R
            r = run(car, sc, strat)
            row = r.summary_row()
            row["R"] = R
            row["series"] = axle + ("-axle I-curve" if strat.kind == "icurve" else "-axle grip limit")
            rows_c.append(row)
    write_csv(rows_c, "sweep_radius.csv")
    return rows, rows_c


# -----------------------------------------------------------------------------
# 8. Lateral load transfer / tire load sensitivity              Aiden  EQ 5
# -----------------------------------------------------------------------------
def load_transfer_study():
    rows = []
    for R in [TIGHT_R, 40.0]:
        for k in [0.0, 0.05, 0.10, 0.15, 0.20]:
            car = rc.Car()
            car.k_load = k
            sc = rc.make_corner_scenario()
            sc.R = R
            r = run(car, sc, rc.optimal_strategy())
            row = r.summary_row()
            row["R"] = R
            row["k_load"] = k
            rows.append(row)
    write_csv(rows, "load_transfer.csv")
    return rows


# -----------------------------------------------------------------------------
# 9. Sensitivity to every assumed parameter                     All    EQ 11
# -----------------------------------------------------------------------------
SENSITIVITY = [
    # (attribute, label, values)  - first value listed is NOT the base; base = Car() default
    ("mu", "road friction mu", [0.5, 0.8, 1.0]),
    ("h", "CG height h (m)", [0.45, 0.55, 0.60]),
    ("m", "mass m (kg)", [1750.0, 2050.0]),
    ("P_motor_max", "motor power", [50e3, 100e3]),
    ("T_motor_max", "motor torque (N m)", [300.0, 500.0]),
    ("eta", "wheel-to-battery efficiency", [0.6, 0.8]),
    ("Cd", "drag coefficient", [0.20, 0.30]),
    ("Crr", "rolling coefficient", [0.010, 0.015]),
    ("v_cut", "regen cutoff speed (m/s)", [1.0, 2.0]),
    ("a", "CG to front axle (m)", [1.35, 1.65]),
]


def sensitivity_study():
    base_car = rc.Car()
    base = {}
    def tight():
        sc = rc.make_corner_scenario()
        sc.R = TIGHT_R
        return sc
    SCEN = [("straight", rc.make_straight_scenario), ("corner", rc.make_corner_scenario),
            ("tight", tight)]
    for kind, mk in SCEN:
        base[kind] = run(base_car, mk(), rc.optimal_strategy())
    rows = []
    for attr, label, values in SENSITIVITY:
        base_val = getattr(base_car, attr)
        for val in values:
            for kind, mk in SCEN:
                car = rc.Car()
                strat = rc.optimal_strategy()
                setattr(car, attr, val)
                if attr == "a":
                    car.b = car.L - car.a
                if attr == "m":
                    car.k_path = 4.0 * car.m
                    car.c_path = 4.0 * car.m
                r = run(car, mk(), strat)
                E0 = base[kind].E_stored
                rel_p = (val - base_val) / base_val
                rel_E = (r.E_stored - E0) / E0
                rows.append({"parameter": attr, "label": label, "scenario_kind": kind,
                             "base_value": base_val, "value": val,
                             "E_stored_kJ": r.E_stored / 1e3, "E_base_kJ": E0 / 1e3,
                             "pct_change_E": 100 * rel_E,
                             "eta_regen_pct": 100 * r.eta_regen(),
                             "eta_regen_base_pct": 100 * base[kind].eta_regen(),
                             "normalized_sensitivity": rel_E / rel_p if rel_p != 0 else "",
                             "t_grip_violation_s": r.t_grip_violation,
                             "t_infeasible_s": r.t_infeasible})
    write_csv(rows, "sensitivity.csv")
    return rows


# -----------------------------------------------------------------------------
# 10. Time-step convergence                                     Devon  EQ 6
# -----------------------------------------------------------------------------
def convergence_study(quick=False):
    dts = [0.008, 0.004, 0.002, 0.001] + ([] if quick else [0.0005, 0.00025])
    rows = []
    for kind, mk in [("straight", rc.make_straight_scenario), ("corner", rc.make_corner_scenario)]:
        Es = []
        for dt in dts:
            sc = mk()
            sc.dt = dt
            sc.record_every = 10 ** 9            # no time series needed
            r = run(rc.Car(), sc, rc.optimal_strategy())
            Es.append(r.E_stored)
            rows.append({"scenario_kind": kind, "dt": dt, "E_stored_kJ": r.E_stored / 1e3,
                         "E_fric_kJ": r.E_fric / 1e3, "residual_J": r.residual(),
                         "t_end": r.t_end})
        E_ref = Es[-1]
        for i, row in enumerate(r for r in rows if r["scenario_kind"] == kind):
            row["abs_diff_vs_finest_J"] = abs(Es[i] - E_ref)
            row["pct_diff_vs_finest"] = 100 * abs(Es[i] - E_ref) / E_ref
        # observed order of accuracy from three successive dt values (EQ 6.3)
        for i in range(len(Es) - 2):
            d1, d2 = Es[i] - Es[i + 1], Es[i + 1] - Es[i + 2]
            ordr = math.log(abs(d1 / d2), 2) if d2 != 0 and d1 != 0 else ""
            [r for r in rows if r["scenario_kind"] == kind][i]["observed_order"] = ordr
    write_csv(rows, "convergence.csv")
    return rows


# -----------------------------------------------------------------------------
# 11. Braking-profile optimization at a fixed stopping distance  All    EQ 9
# -----------------------------------------------------------------------------
def profile_optimization(quick=False):
    """Stop from v0 = 30 m/s in exactly D = 150 m (the 3 m/s^2 constant-decel distance).
    Two-phase profile: decel a1 while v > v_s, then decel a2 chosen so the total distance is
    D:  a2 = v_s^2 / (2 (D - (v0^2 - v_s^2)/(2 a1))).  Search (a1, v_s) for max E_stored."""
    v0, D, a_max = 30.0, 150.0, 0.9 * rc.Car().mu * rc.g
    car = rc.Car()
    strat = rc.optimal_strategy()
    v_list = [2.0 * i for i in range(1, 15)]                 # 2 ... 28 m/s
    a1_list = [0.1 * i for i in range(10, 31)]               # 1.0 ... 3.0 m/s^2
    if quick:
        v_list = v_list[::2]
        a1_list = a1_list[::3]
    rows = []
    for v_s in v_list:
        for a1 in a1_list:
            x1 = (v0 * v0 - v_s * v_s) / (2 * a1)
            if x1 >= D - 1.0:
                continue
            a2 = v_s * v_s / (2 * (D - x1))
            if a2 > a_max:
                continue
            sc = rc.make_straight_scenario()
            sc.name = "profile a1=%.1f v_s=%.0f" % (a1, v_s)
            sc.profile = [(v_s, a1), (0.0, a2)]
            sc.record_every = 10 ** 9
            r = run(car, sc, strat)
            rows.append({"v_switch": v_s, "a1": a1, "a2": a2, "distance_m": r.s_end,
                         "E_stored_kJ": r.E_stored / 1e3, "eta_regen_pct": 100 * r.eta_regen(),
                         "E_fric_kJ": r.E_fric / 1e3, "E_drag_kJ": r.E_drag / 1e3,
                         "duration_s": r.t_end, "t_infeasible_s": r.t_infeasible,
                         "t_grip_violation_s": r.t_grip_violation})
    base = run(car, rc.make_straight_scenario(), strat)
    rows.append({"v_switch": "", "a1": 3.0, "a2": 3.0, "distance_m": base.s_end,
                 "E_stored_kJ": base.E_stored / 1e3, "eta_regen_pct": 100 * base.eta_regen(),
                 "E_fric_kJ": base.E_fric / 1e3, "E_drag_kJ": base.E_drag / 1e3,
                 "duration_s": base.t_end, "t_infeasible_s": base.t_infeasible,
                 "t_grip_violation_s": base.t_grip_violation, "note": "constant 3 m/s^2 baseline"})
    write_csv(rows, "profile_optimization.csv")
    feas = [r for r in rows if r.get("note") is None and r["t_infeasible_s"] == 0]
    best = max(feas, key=lambda r: r["E_stored_kJ"])
    write_csv([best, rows[-1]], "profile_best.csv")
    sc = rc.make_straight_scenario()
    sc.profile = [(best["v_switch"], best["a1"]), (0.0, best["a2"])]
    write_timeseries(run(car, sc, strat), "ts_straight_best_profile.csv")
    return rows, best, rows[-1]


# -----------------------------------------------------------------------------
# Summary markdown
# -----------------------------------------------------------------------------
def write_summary(H, best_const, analytic, consts, soc, hier, sens, conv, prof):
    L = []
    L.append("# Results summary (auto-generated by studies.py - do not edit by hand)\n")
    L.append("Equations: Bayar et al. (2012) sheet, see EQUATIONS.md. Figure of merit "
             "eta_regen = E_stored / E_B (lumped eta = %.2f).\n" % rc.Car().eta)
    L.append("## Headline runs (rear-axle regen)\n")
    L.append("| scenario | strategy | E stored (kJ) | eta_regen | regen share of E_B | friction heat (kJ) "
             "| caps that limited regen (s) | peak rear grip | time over grip (s) |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for key in ["straight", "straight_icurve", "corner", "corner_icurve", "tight", "tight_icurve",
                "tight_blind"]:
        r = H[key]
        caps = ", ".join("%s %.2f" % (rc.LIMIT_NAMES[i], r.t_binding[i]) for i in range(4)
                         if r.t_binding[i] > 0) or "-"
        if r.t_cutoff > 0:
            caps += ", below V_cut %.2f" % r.t_cutoff
        L.append("| %s | %s | %.1f | %.1f%% | %.1f%% | %.1f | %s | %.0f%% | %.2f |" % (
            r.scen.name, r.strat.name, r.E_stored / 1e3, 100 * r.eta_regen(), 100 * r.regen_share(),
            r.E_fric / 1e3, caps, 100 * r.peak_grip_use, r.t_grip_violation))
    L.append("\n**Energy-stability tradeoff (grip limit vs I-curve):** straight %.1f vs %.1f kJ "
             "(I-curve stores %.0f%% less); R = %g m corner %.1f vs %.1f kJ (%.0f%% less)." % (
                 H["straight"].E_stored / 1e3, H["straight_icurve"].E_stored / 1e3,
                 100 * (1 - H["straight_icurve"].E_stored / H["straight"].E_stored),
                 TIGHT_R, H["tight"].E_stored / 1e3, H["tight_icurve"].E_stored / 1e3,
                 100 * (1 - H["tight_icurve"].E_stored / H["tight"].E_stored)))
    L.append("\n## Best constant split vs. time-varying optimal (never exceeds grip)\n")
    for r in best_const:
        L.append("- %s: best constant split %.0f%% regen -> %.1f kJ; optimal -> %.1f kJ (+%.1f%%)"
                 % (r["scenario"], r["best_safe_constant_regen_pct"],
                    r["best_constant_E_stored_kJ"], r["optimal_E_stored_kJ"], r["optimal_gain_pct"]))
    L.append("\n## Analytic check (Stage 1-2 without drag / rolling)\n")
    for r in analytic:
        L.append("- a = %.0f m/s^2: theory %.2f kJ, simulation %.2f kJ (%.3f%% difference)"
                 % (r["a_brake"], r["E_stored_theory_kJ"], r["E_stored_sim_kJ"], r["pct_diff"]))
    L.append("\n## Derived constants\n")
    for r in consts:
        L.append("- %s: %.3g" % (r["quantity"], r["value"]))
    L.append("\n## Battery state of charge (straight stop)\n")
    for r in soc:
        L.append("- SOC0 = %.2f: stored %.1f kJ, eta_regen %.1f%%, battery cap limiting %.2f s"
                 % (r["SOC0"], r["E_stored_kJ"], r["eta_regen_pct"], r["t_battery_binding_s"]))
    L.append("\n## Model hierarchy (straight stop)\n")
    for r in hier:
        L.append("- %s: stored %.1f kJ, eta_regen %.1f%%, drag %.1f kJ, rolling %.1f kJ"
                 % (r["level"], r["E_stored_kJ"], r["eta_regen_pct"], r["E_drag_kJ"], r["E_roll_kJ"]))
    L.append("\n## Most sensitive parameters (largest |normalized sensitivity|: straight / R=40 / R=%g)\n" % TIGHT_R)
    agg = {}
    for r in sens:
        if r["normalized_sensitivity"] == "" or r["t_infeasible_s"] > 0:
            continue
        agg.setdefault(r["label"], {"straight": 0.0, "corner": 0.0, "tight": 0.0})
        agg[r["label"]][r["scenario_kind"]] = max(agg[r["label"]][r["scenario_kind"]],
                                                  abs(r["normalized_sensitivity"]))
    for k, v in sorted(agg.items(), key=lambda kv: -max(kv[1].values())):
        L.append("- %s: %.2f / %.2f / %.2f" % (k, v["straight"], v["corner"], v["tight"]))
    infeas = sorted(set("%s=%g (%s)" % (r["label"], r["value"], r["scenario_kind"]) for r in sens
                        if r["t_infeasible_s"] > 0))
    if infeas:
        L.append("\nScenario infeasible (demand exceeds total grip) for: " + "; ".join(infeas))
    L.append("\n## Time-step convergence (E stored vs finest dt)\n")
    for r in conv:
        L.append("- %s dt = %g s: %.3f kJ, %.4f%% from finest" % (
            r["scenario_kind"], r["dt"], r["E_stored_kJ"], r["pct_diff_vs_finest"]))
    rows, best, base = prof
    L.append("\n## Braking profile at fixed 150 m stopping distance\n")
    L.append("- constant 3 m/s^2: %.1f kJ stored (eta_regen %.1f%%)" % (base["E_stored_kJ"], base["eta_regen_pct"]))
    L.append("- best two-phase profile: a1 = %.1f m/s^2 above %.0f m/s, then a2 = %.2f m/s^2: %.1f kJ "
             "(eta_regen %.1f%%), +%.1f%%"
             % (best["a1"], best["v_switch"], best["a2"], best["E_stored_kJ"], best["eta_regen_pct"],
                100 * (best["E_stored_kJ"] - base["E_stored_kJ"]) / base["E_stored_kJ"]))
    path = os.path.join(DATA, "results_summary.md")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")
    return path


def run_all(quick=False):
    t0 = time.time()

    def step(name, fn, *a):
        t = time.time()
        out = fn(*a)
        print("  %-28s %5.1f s" % (name, time.time() - t))
        return out

    print("Running studies -> " + DATA)
    H = step("headline runs", headline_runs)
    _, best_const = step("strategy tables", strategy_tables)
    analytic = step("analytic check", analytic_check)
    consts = step("limit envelope", limit_envelope)
    soc = step("SOC study", soc_study)
    hier = step("model hierarchy", model_hierarchy)
    step("axle / decel / radius", axle_and_decel_sweep)
    step("load transfer", load_transfer_study)
    sens = step("sensitivity", sensitivity_study)
    conv = step("convergence", convergence_study, quick)
    prof = step("profile optimization", profile_optimization, quick)
    path = write_summary(H, best_const, analytic, consts, soc, hier, sens, conv, prof)
    print("Done in %.0f s. Summary: %s" % (time.time() - t0, path))
    return path


if __name__ == "__main__":
    run_all(quick=("quick" in sys.argv))
