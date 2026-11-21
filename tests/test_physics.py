"""
Physics checks. Run from the repo root:   python -m pytest -q
Each test checks one equation from the Bayar et al. (2012) sheet (EQUATIONS.md) or one claim
we make in the poster/report.
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


# --- Stage 2: regen caps -------------------------------------------------------------------
def test_stage2_caps_match_sheet():
    car = rc.Car()
    F_T, F_P, F_B = rc.regen_force_limits(car, 20.0, 0.5)
    assert math.isclose(F_T, car.G * car.T_motor_max / car.r_w)
    assert math.isclose(F_P, car.P_motor_max / 20.0)
    assert math.isclose(F_B, car.P_batt_max / (car.eta * 20.0))


def test_battery_charge_cap_constant_to_90_then_linear():
    car = rc.Car()
    assert rc.battery_charge_power_limit(car, 0.89) == car.P_batt_max
    assert math.isclose(rc.battery_charge_power_limit(car, 0.95), 0.5 * car.P_batt_max)
    assert rc.battery_charge_power_limit(car, 1.0) == 0.0


def test_no_regen_at_or_below_v_cut():
    r = rc.simulate(rc.Car(), straight(), rc.optimal_strategy())
    for v, Fr in zip(r.speed, r.F_regen):
        if v <= rc.Car().v_cut:
            assert Fr == 0.0
    assert r.t_cutoff > 0


def test_brake_request_is_demand_minus_drag_and_rolling():
    car = rc.Car()
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    i = 50
    expect = car.m * 3.0 - r.F_drag[i] - r.F_roll[i]
    assert math.isclose(r.F_req[i], expect, rel_tol=1e-9)
    assert math.isclose(r.F_roll[i], car.Crr * car.m * rc.g)


# --- Stage 3: load transfer and I-curve -----------------------------------------------------
def test_stage3_normal_forces():
    car = rc.Car()
    a = 3.0
    Nf, Nr = rc.axle_normal_forces(car, a)
    assert math.isclose(Nf, car.m * (rc.g * car.b + a * car.h) / car.L)
    assert math.isclose(Nr, car.m * (rc.g * car.a - a * car.h) / car.L)
    assert math.isclose(Nf + Nr, car.m * rc.g)


def test_load_transfer_uses_a_equals_Fb_over_M():
    car = rc.Car()
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    i = 50
    Nf, Nr = rc.axle_normal_forces(car, r.F_req[i] / car.m)
    assert math.isclose(r.N_r[i], Nr, rel_tol=1e-12)


def test_icurve_shares_sum_to_Fb_and_load_both_axles_equally():
    car = rc.Car()
    F_b = 5000.0
    Fr = rc.ideal_brake_force(car, F_b, "rear")
    Ff = rc.ideal_brake_force(car, F_b, "front")
    assert math.isclose(Fr + Ff, F_b)
    Nf, Nr = rc.axle_normal_forces(car, F_b / car.m)
    assert math.isclose(Fr / Nr, Ff / Nf, rel_tol=1e-9)        # same friction use on both axles


# --- Stage 4: cornering --------------------------------------------------------------------
def test_stage4_rear_lateral_force_and_friction_circle():
    car = rc.Car()
    V, R = 15.0, 40.0
    Fyf, Fyr = rc.axle_lateral_forces(car, car.m * V ** 2 / R)
    assert math.isclose(Fyr, car.m * V ** 2 / R * car.a / car.L)
    assert math.isclose(Fyf * car.a, Fyr * car.b)               # zero torque about the CoM
    Nr = rc.axle_normal_forces(car, 2.0)[1]
    assert math.isclose(rc.longitudinal_grip(car.mu, Nr, Fyr), math.sqrt((car.mu * Nr) ** 2 - Fyr ** 2))


def test_constant_mu_capacity_is_mu_N():
    cap, _ = rc.axle_capacity(rc.Car(), 0.9, 8000.0, 5000.0, 1)
    assert cap == 0.9 * 8000.0


def test_load_sensitivity_extension_reduces_capacity_when_cornering():
    car = rc.Car()
    car.k_load = 0.1
    N0 = rc.static_tire_load(car, 1)
    cap_straight, _ = rc.axle_capacity(car, 0.9, 2 * N0, 0.0, 1)
    cap_corner, _ = rc.axle_capacity(car, 0.9, 2 * N0, 6000.0, 1)
    assert math.isclose(cap_straight, 0.9 * 2 * N0)
    assert cap_corner < cap_straight


# --- Energy bookkeeping ---------------------------------------------------------------------
def test_energy_check_closes_to_roundoff():
    for mk in [rc.make_straight_scenario, rc.make_corner_scenario]:
        r = rc.simulate(rc.Car(), mk(), rc.optimal_strategy())
        assert abs(r.residual()) < 1e-6 * r.KE_i


def test_stored_loss_and_brake_energy_definitions():
    car = rc.Car()
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    assert math.isclose(r.E_stored, car.eta * r.E_regen, rel_tol=1e-9)
    assert math.isclose(r.E_loss, (1 - car.eta) * r.E_regen, rel_tol=1e-9)
    assert math.isclose(r.E_B, r.E_regen + r.E_fric, rel_tol=1e-9)
    assert math.isclose(r.eta_regen(), r.E_stored / r.E_B)


def test_soc_rises_by_stored_energy():
    car = rc.Car()
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    assert math.isclose(r.SOC_f - car.SOC0, r.E_stored / car.E_batt, rel_tol=1e-9)


def test_converges_with_dt():
    Es = []
    for dt in [0.004, 0.002, 0.001]:
        Es.append(rc.simulate(rc.Car(), straight(dt=dt, record_every=10 ** 9), rc.optimal_strategy()).E_stored)
    assert abs(Es[2] - Es[0]) / Es[2] < 1e-3


# --- Theory vs simulation -------------------------------------------------------------------
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
    a = rc.grip_limited_decel(car, "rear")
    N_r = rc.axle_normal_forces(car, a)[1]
    assert math.isclose(car.mu * N_r, car.m * a, rel_tol=1e-9)


def test_critical_soc_is_where_battery_starts_binding():
    car = rc.Car()
    s = rc.critical_soc(car)
    assert math.isclose(rc.battery_charge_power_limit(car, s) / car.eta, car.P_motor_max, rel_tol=1e-9)


# --- Strategy claims ------------------------------------------------------------------------
def test_optimal_beats_every_fixed_split_that_does_not_slide():
    car = rc.Car()
    tight = rc.make_corner_scenario()
    tight.R = 35.0
    for sc in [rc.make_straight_scenario(), rc.make_corner_scenario(), tight]:
        opt = rc.simulate(car, sc, rc.optimal_strategy())
        assert opt.t_grip_violation == 0
        for st in rc.standard_strategies():
            r = rc.simulate(car, sc, st)
            if r.t_grip_violation == 0:
                assert r.E_stored <= opt.E_stored + 1e-6


def test_icurve_is_more_conservative_than_grip_limit():
    car = rc.Car()
    for sc in [rc.make_straight_scenario(), rc.make_corner_scenario()]:
        a = rc.simulate(car, sc, rc.optimal_strategy())
        b = rc.simulate(car, sc, rc.icurve_strategy())
        assert b.E_stored < a.E_stored
        assert b.peak_grip_use <= a.peak_grip_use + 1e-9


def test_grip_blind_regen_slides_in_tight_corner():
    sc = rc.make_corner_scenario()
    sc.R = 35.0
    r = rc.simulate(rc.Car(), sc, rc.Strategy("blind", "fixed", 1.0))
    assert r.t_grip_violation > 0


def test_high_soc_makes_battery_bind():
    car = rc.Car()
    car.SOC0 = 0.96
    r = rc.simulate(car, straight(), rc.optimal_strategy())
    assert r.t_binding[2] > 0


def test_default_profile_is_constant_decel():
    a = rc.simulate(rc.Car(), straight(), rc.optimal_strategy())
    b = rc.simulate(rc.Car(), straight(profile=[(0.0, 3.0)]), rc.optimal_strategy())
    assert a.E_stored == b.E_stored


# --- Simulink comparison tool ---------------------------------------------------------------
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
