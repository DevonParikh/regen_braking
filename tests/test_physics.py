"""
Physics checks. Run from the repo root:   python -m pytest -q
Each test is a statement we make in the poster/report, checked automatically.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "simulink"))

import regen_core as rc  # noqa: E402


def straight(**kw):
    sc = rc.make_straight_scenario()
    for k, v in kw.items():
        setattr(sc, k, v)
    return sc


# --- Section 2/3 building blocks ----------------------------------------------------------
def test_normal_forces_sum_to_weight_and_shift_forward():
    car = rc.Car()
    for a in [0.0, 3.0, 7.0]:
        Nf, Nr = rc.axle_normal_forces(car, a)
        assert math.isclose(Nf + Nr, car.m * rc.g, rel_tol=1e-12)
    assert rc.axle_normal_forces(car, 3.0)[1] < rc.axle_normal_forces(car, 0.0)[1]


def test_lateral_split_has_zero_torque_about_cg():
    car = rc.Car()
    Fyf, Fyr = rc.axle_lateral_forces(car, 10000.0)
    assert math.isclose(Fyf + Fyr, 10000.0)
    assert math.isclose(Fyf * car.a, Fyr * car.b)


def test_battery_taper():
    car = rc.Car()
    assert rc.battery_charge_power_limit(car, 0.5) == car.P_batt_max
    assert math.isclose(rc.battery_charge_power_limit(car, 0.9), 0.5 * car.P_batt_max)
    assert rc.battery_charge_power_limit(car, 1.0) == 0.0


def test_constant_mu_capacity_is_mu_N():
    car = rc.Car()
    cap, _ = rc.axle_capacity(car, 0.9, 8000.0, 5000.0, 1)
    assert cap == 0.9 * 8000.0


def test_load_sensitivity_reduces_capacity_when_cornering():
    car = rc.Car()
    car.k_load = 0.1
    N0 = rc.static_tire_load(car, 1)
    cap_straight, _ = rc.axle_capacity(car, 0.9, 2 * N0, 0.0, 1)
    cap_corner, _ = rc.axle_capacity(car, 0.9, 2 * N0, 6000.0, 1)
    assert math.isclose(cap_straight, 0.9 * 2 * N0)
    assert cap_corner < cap_straight


# --- Energy principle and numerics ----------------------------------------------------------
def test_energy_ledger_closes():
    for mk in [rc.make_straight_scenario, rc.make_corner_scenario]:
        r = rc.simulate(rc.Car(), mk(), rc.optimal_strategy())
        assert abs(r.residual()) < 1e-6 * r.KE_i


def test_stored_energy_chain():
    r = rc.simulate(rc.Car(), straight(), rc.optimal_strategy())
    car = rc.Car()
    assert math.isclose(r.E_stored, car.eta_motor * car.eta_batt * r.E_regen, rel_tol=1e-9)


def test_converges_with_dt():
    Es = []
    for dt in [0.004, 0.002, 0.001]:
        Es.append(rc.simulate(rc.Car(), straight(dt=dt, record_every=10 ** 9), rc.optimal_strategy()).E_stored)
    assert abs(Es[2] - Es[1]) < abs(Es[1] - Es[0])
    assert abs(Es[2] - Es[1]) / Es[2] < 1e-4


# --- Theory vs simulation ---------------------------------------------------------------------
def test_matches_analytic_straight_stop():
    for a in [1.0, 2.0, 3.0]:
        car = rc.Car()
        car.include_drag = False
        car.include_roll = False
        r = rc.simulate(car, straight(a_brake=a), rc.optimal_strategy())
        th = rc.analytic_straight_stop(car, 30.0, a, 0.3)
        assert abs(r.E_stored - th["E_stored"]) / th["E_stored"] < 1e-3


def test_decel_theory_within_3_percent_below_grip_limit():
    car = rc.Car()
    for a in [1.0, 1.5, 2.0, 2.5, 3.0]:
        r = rc.simulate(car, straight(a_brake=a), rc.optimal_strategy())
        th = rc.analytic_decel_energy(car, 30.0, a)["E_stored"]
        assert abs(r.E_stored - th) / r.E_stored < 0.03


def test_grip_limited_decel_formula():
    car = rc.Car()
    a = rc.grip_limited_decel(car, 0.85, "rear")
    N_r = rc.axle_normal_forces(car, a)[1]
    assert math.isclose(0.85 * car.mu * N_r, car.m * a, rel_tol=1e-9)


def test_critical_soc_is_where_battery_starts_binding():
    car = rc.Car()
    s = rc.critical_soc(car)
    P = rc.battery_charge_power_limit(car, s)
    assert math.isclose(P, car.eta_motor * car.P_motor_max, rel_tol=1e-9)


# --- Strategy claims ----------------------------------------------------------------------------
def test_optimal_beats_every_safe_fixed_split():
    car = rc.Car()
    for mk in [rc.make_straight_scenario, rc.make_corner_scenario]:
        opt = rc.simulate(car, mk(), rc.optimal_strategy())
        assert opt.t_grip_violation == 0
        for st in rc.standard_strategies():
            r = rc.simulate(car, mk(), st)
            if r.t_grip_violation == 0 and r.peak_grip_use <= opt.peak_grip_use + 1e-9:
                assert r.E_stored <= opt.E_stored + 1e-6


def test_grip_blind_regen_exceeds_grip_in_corner():
    r = rc.simulate(rc.Car(), rc.make_corner_scenario(), rc.Strategy("blind", "fixed", 1.0, 1.0))
    assert r.t_grip_violation > 0


def test_high_soc_makes_battery_bind():
    car = rc.Car()
    car.SOC0 = 0.92
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    assert r.t_binding[2] > 0


def test_default_profile_is_constant_decel():
    a = rc.simulate(rc.Car(), straight(), rc.optimal_strategy())
    b = rc.simulate(rc.Car(), straight(profile=[(0.0, 3.0)]), rc.optimal_strategy())
    assert a.E_stored == b.E_stored


# --- Simulink comparison tool -------------------------------------------------------------------
def test_compare_simulink_on_synthetic_data(tmp_path):
    import csv
    import compare_simulink as cs
    r = rc.simulate(rc.Car(), straight(), rc.optimal_strategy())
    rows = rc.timeseries_rows(r)
    data = tmp_path / "data"
    data.mkdir()
    with open(data / "ts_straight_optimal.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rc.TIMESERIES_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    sim = tmp_path / "sim"
    sim.mkdir()
    with open(sim / "results_straight.csv", "w", newline="") as f:      # Simulink stores 2% more
        w = csv.writer(f)
        w.writerow(["t", "speed", "E_stored"])
        for q in rows[::7]:
            w.writerow([q["t"], q["speed"], 1.02 * q["E_stored"]])
    out = cs.main(str(sim), str(data), str(tmp_path / "fig"))
    e = [o for o in out if o["signal"] == "E_stored"][0]
    assert abs(e["pct_diff_final"] - 2.0) < 0.5
    assert (tmp_path / "fig" / "simulink_overlay_straight.png").exists()
