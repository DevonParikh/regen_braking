"""
regen_core.py - physics engine for "Optimizing Regenerative Braking of Electric Vehicles"
Team: Alex Wang, Devon Parikh, Aiden Knapp

This file holds ALL of the physics (sections 1-6 of the original regen_braking.py) with
no graphics, so it can be imported by:
    regen_braking.py      the VPython 3D animation (Milestones 2 and 3)
    studies.py            every study / sweep / validation, writes data/*.csv
    make_figures.py       poster/report figures from data/*.csv
    simulink/*.py         Simulink export + comparison (Milestone 4)

With the default settings this gives exactly the same numbers as the original single-file
model. New, optional features (all OFF by default so the original results are unchanged):
    car.include_drag / car.include_roll   turn drag / rolling resistance off (analytic check,
                                          "model hierarchy" from the proposal)
    car.k_load                            tire load sensitivity (left-right weight transfer
                                          study in the corner); 0 = constant mu
    scen.profile                          piecewise-constant braking profile (braking-profile
                                          optimization at a fixed stopping distance)

Equations for everything are in EQUATIONS.md (section numbers referenced below).
"""
import math
import copy


# =============================================================================
# Small 3D vector so the physics doesn't need VPython. Same idea as vpython.vector.
# =============================================================================
class Vec:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, o):
        return Vec(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o):
        return Vec(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, k):
        return Vec(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def __truediv__(self, k):
        return Vec(self.x / k, self.y / k, self.z / k)

    def __neg__(self):
        return Vec(-self.x, -self.y, -self.z)

    def __repr__(self):
        return "Vec(%g, %g, %g)" % (self.x, self.y, self.z)


def vmag(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def vdot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z


# =============================================================================
# SECTION 1 - PARAMETERS                                         (EQUATIONS.md 1)
# =============================================================================
g = 9.81               # m/s^2
rho_air = 1.225        # kg/m^3, sea-level air
GRIP_MARGIN = 0.85     # optimal strategy uses at most 85% of the regen axle's grip


class Car:
    """Rigid body on two axles. Numbers are roughly a rear-drive Tesla Model 3;
    they are assumptions - cite/replace them in the report (see PARAMETERS table in README)."""
    def __init__(self):
        self.m = 1850.0          # kg, mass with driver
        self.L = 2.875           # m, wheelbase
        self.a = 1.50            # m, center of mass -> front axle   (l_f in EQUATIONS.md)
        self.b = self.L - self.a # m, center of mass -> rear axle    (l_r in EQUATIONS.md)
        self.h = 0.50            # m, center-of-mass height
        self.track = 1.58        # m, left-right distance between tires (lateral transfer only)
        self.length = 4.7        # m (drawing only)
        self.width = 1.9         # m (drawing only)

        self.mu = 0.90           # road friction coefficient, flat dry asphalt (constant)
        self.k_load = 0.0        # tire load sensitivity (0 = constant mu, the proposal assumption)

        self.Cd = 0.23           # drag coefficient
        self.A = 2.22            # m^2 frontal area
        self.Crr = 0.010         # rolling resistance coefficient
        self.include_drag = True
        self.include_roll = True

        self.r_w = 0.34          # m, tire radius
        self.G = 9.0             # gear ratio motor:wheel
        self.T_motor_max = 420.0 # N*m, max motor torque in regen
        self.P_motor_max = 75e3  # W, max mechanical power the motor can absorb
        self.eta_motor = 0.90    # mechanical -> electrical (motor + inverter)
        self.eta_batt = 0.95     # electrical -> chemical (battery charging)
        self.v_fade = 1.5        # m/s, regen fades out below this speed

        self.E_batt = 75.0 * 3.6e6  # J, 75 kWh pack
        self.SOC0 = 0.60            # starting state of charge
        self.P_batt_max = 80e3      # W, max charge power when SOC is low
        self.SOC_taper = 0.80       # above this SOC, charge power tapers linearly to 0 at 100%

        self.regen_axle = "rear"    # "rear" or "front": which axle the motor drives

        # gains for the "driver" that keeps the car on the corner midline
        self.k_path = 4.0 * self.m  # N per m of radial error
        self.c_path = 4.0 * self.m  # N per m/s of radial velocity

    def copy(self):
        return copy.copy(self)


# =============================================================================
# SECTION 2 - MOTOR, BATTERY, ENERGY LIMITS   (OWNER: Alex)      (EQUATIONS.md 2)
# All limits are a FORCE at the tire contact patch (N).
# =============================================================================
def battery_charge_power_limit(car, SOC):
    """Max electrical power (W) the battery accepts. Flat, then linear taper to 0 at full."""
    if SOC >= 1.0:
        return 0.0
    if SOC <= car.SOC_taper:
        return car.P_batt_max
    return car.P_batt_max * (1.0 - SOC) / (1.0 - car.SOC_taper)


def regen_force_limits(car, speed, SOC):
    """Returns [F_torque, F_power, F_battery] in N."""
    F_torque = car.T_motor_max * car.G / car.r_w
    if speed < car.v_fade:
        F_torque = F_torque * speed / car.v_fade
    v = max(speed, 0.1)                       # avoid dividing by zero near a stop
    F_power = car.P_motor_max / v
    F_batt = battery_charge_power_limit(car, SOC) / (car.eta_motor * v)
    return [F_torque, F_power, F_batt]


# =============================================================================
# SECTION 3 - WEIGHT TRANSFER AND TIRE GRIP   (OWNER: Aiden)     (EQUATIONS.md 3, 5)
# =============================================================================
def axle_normal_forces(car, a_decel):
    """[N_front, N_rear] in N. a_decel = deceleration magnitude (+ when braking)."""
    W = car.m * g
    shift = car.m * a_decel * car.h / car.L
    N_f = W * car.b / car.L + shift
    N_r = W * car.a / car.L - shift
    return [max(N_f, 0.0), max(N_r, 0.0)]


def axle_lateral_forces(car, F_lat_total):
    """[F_y_front, F_y_rear]: steady cornering, zero torque about the center of mass."""
    return [F_lat_total * car.b / car.L, F_lat_total * car.a / car.L]


def static_tire_load(car, axle_index):
    """Load on ONE tire of the given axle with the car at rest (N). Reference load N0."""
    W = car.m * g
    if axle_index == 0:
        return 0.5 * W * car.b / car.L
    return 0.5 * W * car.a / car.L


def axle_capacity(car, mu, N_axle, F_y_axle, axle_index):
    """Total friction force (N) the two tires on one axle can supply.             (EQ 5)
    k_load = 0 (default): mu * N_axle, exactly the original model.
    k_load > 0: cornering moves dN = F_y*h/track from the inner to the outer tire and
    each tire's mu drops with load: mu_i = mu*(1 - k*(N_i/N0 - 1)).
    Returns (capacity, inner_tire_lifted)."""
    dN = abs(F_y_axle) * car.h / car.track
    lifted = dN > 0.5 * N_axle
    if car.k_load == 0.0:
        return mu * N_axle, lifted
    N_out = min(0.5 * N_axle + dN, N_axle)
    N_in = max(0.5 * N_axle - dN, 0.0)
    N0 = static_tire_load(car, axle_index)
    cap = 0.0
    for Ni in (N_out, N_in):
        mu_i = mu * max(1.0 - car.k_load * (Ni / N0 - 1.0), 0.0)
        cap += mu_i * Ni
    return cap, lifted


def longitudinal_grip_from_capacity(cap, F_y):
    """Friction circle: Fx^2 + Fy^2 <= cap^2. Braking force left after F_y."""
    if F_y >= cap:
        return 0.0
    return math.sqrt(cap * cap - F_y * F_y)


def longitudinal_grip(mu, N, F_y):
    """Original constant-mu version (kept for the tests / backward compatibility)."""
    return longitudinal_grip_from_capacity(mu * N, F_y)


# =============================================================================
# SECTION 5 - BRAKING STRATEGIES                                 (EQUATIONS.md 4)
# =============================================================================
class Strategy:
    def __init__(self, name, kind, frac=0.0, margin=GRIP_MARGIN):
        self.name = name
        self.kind = kind       # "optimal", "fixed"
        self.frac = frac       # used by "fixed"
        self.margin = margin   # grip margin used by "optimal"


def choose_regen(strat, F_req, F_torque, F_power, F_batt, F_grip_margined):
    """Returns the regen force (N) the strategy commands this instant."""
    motor_cap = min(F_torque, F_power, F_batt)
    if strat.kind == "optimal":
        return min(F_req, motor_cap, F_grip_margined)
    if strat.kind == "fixed":
        return min(strat.frac * F_req, motor_cap)
    return 0.0


def optimal_strategy(margin=GRIP_MARGIN):
    return Strategy("optimal, grip margin " + fmt(margin, 2), "optimal", 0.0, margin)


# =============================================================================
# SECTION 6 - SCENARIOS (the "drive cycle" the car must follow)
# =============================================================================
class Scenario:
    def __init__(self, name, kind):
        self.name = name
        self.kind = kind          # "straight" or "corner"
        self.v0 = 30.0            # m/s initial speed
        self.a_brake = 3.0        # m/s^2 commanded deceleration while braking
        self.v_end = 0.0          # stop braking (hold speed) once below this
        self.s_brake_start = 0.0  # m along the path where braking begins
        self.R = 40.0             # m, corner radius (midline)
        self.approach = 60.0      # m of straight road before the corner
        self.exit_length = 20.0   # m of straight road simulated after the corner
        self.v_stop = 0.3         # m/s, end the run when this slow
        self.dt = 0.002           # s
        self.t_max = 60.0         # s
        self.record_every = 10    # store one time-series row every N steps
        # Optional piecewise braking profile: list of (v_above, a) pairs, sorted from
        # high v_above to low. Deceleration a is used while speed > v_above.
        # None = constant a_brake (the original model).
        self.profile = None

    def copy(self):
        sc = copy.copy(self)
        if self.profile is not None:
            sc.profile = list(self.profile)
        return sc


def make_straight_scenario():
    return Scenario("Straight-line stop, 30 m/s -> 0 at 3 m/s^2", "straight")


def make_corner_scenario():
    sc = Scenario("90 deg corner (R = 40 m), braking 20 -> 9 m/s at 3 m/s^2", "corner")
    sc.v0 = 20.0
    sc.a_brake = 3.0
    sc.v_end = 9.0
    sc.R = 40.0
    sc.approach = 60.0
    sc.s_brake_start = 35.0   # braking starts 25 m before corner entry (trail-braking in)
    return sc


def braking_command(scen, s, speed):
    """Target deceleration (m/s^2). 0 means the driver holds speed."""
    if s >= scen.s_brake_start and speed > scen.v_end:
        if scen.profile:
            for v_above, a in scen.profile:
                if speed > v_above:
                    return a
            return scen.profile[-1][1]
        return scen.a_brake
    return 0.0


# =============================================================================
# SECTION 4 - CORE SIMULATION (Devon)                            (EQUATIONS.md 1, 6)
# =============================================================================
LIMIT_NAMES = ["motor torque", "motor power", "battery charge", "tire grip"]


class Results:
    def __init__(self, scen, strat, car):
        self.scen = scen
        self.strat = strat
        self.regen_axle = car.regen_axle
        self.KE_i = 0.0
        self.KE_f = 0.0
        self.W_drive = 0.0
        self.W_lat = 0.0
        self.E_drag = 0.0
        self.E_roll = 0.0
        self.E_fric = 0.0
        self.E_regen = 0.0
        self.E_motor_loss = 0.0
        self.E_batt_loss = 0.0
        self.E_stored = 0.0
        self.t_end = 0.0
        self.s_end = 0.0
        self.SOC_f = 0.0
        self.t_grip_violation = 0.0
        self.t_infeasible = 0.0
        self.t_wheel_lift = 0.0          # s an inner tire would lift (lateral transfer > N/2)
        self.peak_grip_use = 0.0
        self.t_binding = [0.0, 0.0, 0.0, 0.0]
        # time series (one row every scen.record_every steps)
        self.t = []
        self.s = []
        self.x = []
        self.z = []
        self.speed = []
        self.a_cmd = []
        self.F_req = []
        self.F_regen = []
        self.F_fric = []
        self.F_drag = []
        self.F_roll = []
        self.F_lat = []
        self.N_f = []
        self.N_r = []
        self.frac = []
        self.lim = [[], [], [], []]
        self.E_stored_t = []
        self.E_fric_t = []
        self.SOC_t = []
        self.grip_use = []
        self.pos = []

    def brake_work(self):
        return self.E_regen + self.E_fric

    def regen_share(self):
        if self.brake_work() <= 0:
            return 0.0
        return self.E_regen / self.brake_work()

    def residual(self):
        """(K_f - K_i) - (sum of work by all forces). Bookkeeping check, see EQUATIONS.md 6."""
        work = self.W_drive + self.W_lat - self.E_drag - self.E_roll - self.E_fric - self.E_regen
        return (self.KE_f - self.KE_i) - work

    def summary_row(self):
        """One flat dict of the headline numbers (for CSV)."""
        return {
            "scenario": self.scen.name, "strategy": self.strat.name, "regen_axle": self.regen_axle,
            "duration_s": self.t_end, "distance_m": self.s_end,
            "KE_i_kJ": self.KE_i / 1e3, "KE_f_kJ": self.KE_f / 1e3,
            "E_drag_kJ": self.E_drag / 1e3, "E_roll_kJ": self.E_roll / 1e3,
            "E_fric_kJ": self.E_fric / 1e3, "E_regen_kJ": self.E_regen / 1e3,
            "E_motor_loss_kJ": self.E_motor_loss / 1e3, "E_batt_loss_kJ": self.E_batt_loss / 1e3,
            "E_stored_kJ": self.E_stored / 1e3, "W_drive_kJ": self.W_drive / 1e3,
            "regen_share_pct": 100 * self.regen_share(),
            "stored_over_brake_work_pct": 100 * self.E_stored / max(self.brake_work(), 1),
            "residual_J": self.residual(), "W_lat_J": self.W_lat,
            "peak_grip_use_pct": 100 * self.peak_grip_use,
            "t_grip_violation_s": self.t_grip_violation, "t_infeasible_s": self.t_infeasible,
            "t_wheel_lift_s": self.t_wheel_lift,
            "t_torque_binding_s": self.t_binding[0], "t_power_binding_s": self.t_binding[1],
            "t_battery_binding_s": self.t_binding[2], "t_grip_binding_s": self.t_binding[3],
            "SOC_f": self.SOC_f,
        }


def simulate(car, scen, strat, vis=None):
    """Momentum-principle time stepping (Euler-Cromer) + energy ledger. vis (optional) is
    any object with .every and .update(...) - the VPython animation in regen_braking.py."""
    dt = scen.dt
    m = car.m
    mu = car.mu
    res = Results(scen, strat, car)

    if scen.kind == "corner":
        pos = Vec(-scen.approach, 0, 0)
    else:
        pos = Vec(0, 0, 0)
    p = Vec(scen.v0, 0, 0) * m               # momentum, car drives along +x
    center = Vec(0, 0, -scen.R)              # corner turns left (toward -z)
    phase = "approach"
    s = 0.0
    s_exit = 0.0
    t = 0.0
    SOC = car.SOC0
    res.KE_i = 0.5 * m * scen.v0 * scen.v0
    step = 0
    iR = 0 if car.regen_axle == "front" else 1

    while t < scen.t_max:
        v = p / m
        speed = vmag(v)
        if speed < scen.v_stop:
            break
        vhat = v / speed

        # ---- where are we on the path? ----
        if scen.kind == "corner":
            if phase == "approach" and pos.x >= 0:
                phase = "arc"
            if phase == "arc":
                rel = pos - center
                if math.atan2(rel.x, rel.z) >= math.pi / 2:
                    phase = "exit"
            if phase == "exit" and s_exit > scen.exit_length:
                break

        # ---- resistive forces (opposite velocity) ----
        if car.include_drag:
            F_drag = vhat * (-0.5 * rho_air * car.Cd * car.A * speed * speed)
        else:
            F_drag = Vec(0, 0, 0)
        if car.include_roll:
            F_roll = vhat * (-car.Crr * m * g)
        else:
            F_roll = Vec(0, 0, 0)
        F_resist = vmag(F_drag) + vmag(F_roll)

        # ---- the drive cycle: what deceleration is required? ----
        a_cmd = braking_command(scen, s, speed)
        F_drive = 0.0
        if a_cmd > 0:
            F_req = max(0.0, m * a_cmd - F_resist)
            a_decel = a_cmd
        else:
            F_req = 0.0
            F_drive = F_resist
            a_decel = 0.0

        # ---- sideways (centripetal) tire force in the corner ----
        F_lat_vec = Vec(0, 0, 0)
        if phase == "arc":
            rel = pos - center
            r = vmag(rel)
            rhat_out = rel / r
            v_rad = vdot(v, rhat_out)
            F_c = m * speed * speed / r + car.k_path * (r - scen.R) + car.c_path * v_rad
            F_lat_vec = rhat_out * (-F_c)
        F_lat = vmag(F_lat_vec)

        # ---- grip at each axle ----
        N = axle_normal_forces(car, a_decel)
        Fy = axle_lateral_forces(car, F_lat)
        cap_f, lift_f = axle_capacity(car, mu, N[0], Fy[0], 0)
        cap_r, lift_r = axle_capacity(car, mu, N[1], Fy[1], 1)
        if lift_f or lift_r:
            res.t_wheel_lift += dt
        grip_f = longitudinal_grip_from_capacity(cap_f, Fy[0])
        grip_r = longitudinal_grip_from_capacity(cap_r, Fy[1])
        cap_regen_m, _ = axle_capacity(car, strat.margin * mu, N[iR], Fy[iR], iR)
        F_grip_regen = longitudinal_grip_from_capacity(cap_regen_m, Fy[iR])

        # ---- regen limits and the strategy's choice ----
        F_torque, F_power, F_batt = regen_force_limits(car, speed, SOC)
        F_regen = choose_regen(strat, F_req, F_torque, F_power, F_batt, F_grip_regen)
        F_fric = F_req - F_regen

        all_lims = [F_torque, F_power, F_batt, F_grip_regen]
        F_regen_max = min(all_lims)
        if F_req > F_regen_max + 1e-6 and strat.kind == "optimal":
            for i in range(4):
                if all_lims[i] == F_regen_max:
                    res.t_binding[i] += dt
                    break

        # ---- split friction braking between axles by remaining grip ----
        Fx = [0.0, 0.0]
        Fx[iR] = F_regen
        rem_f = max(grip_f - Fx[0], 0.0)
        rem_r = max(grip_r - Fx[1], 0.0)
        if F_fric > 0:
            if rem_f + rem_r > 0:
                Fx[0] += F_fric * rem_f / (rem_f + rem_r)
                Fx[1] += F_fric * rem_r / (rem_f + rem_r)
            else:
                Fx[0] += F_fric * car.b / car.L
                Fx[1] += F_fric * car.a / car.L
            if F_fric > rem_f + rem_r + 1e-6:
                res.t_infeasible += dt

        # ---- stability check: does any axle need more than its capacity? ----
        use_f = math.sqrt(Fx[0] * Fx[0] + Fy[0] * Fy[0]) / max(cap_f, 1e-9)
        use_r = math.sqrt(Fx[1] * Fx[1] + Fy[1] * Fy[1]) / max(cap_r, 1e-9)
        if use_f > 1.0 + 1e-6 or use_r > 1.0 + 1e-6:
            res.t_grip_violation += dt
        use_regen_axle = use_f if iR == 0 else use_r
        res.peak_grip_use = max(res.peak_grip_use, use_regen_axle)

        # ---- force vectors ----
        F_regen_vec = vhat * (-F_regen)
        F_fric_vec = vhat * (-F_fric)
        F_drive_vec = vhat * F_drive
        F_net = F_regen_vec + F_fric_vec + F_drive_vec + F_drag + F_roll + F_lat_vec

        # ---- MOMENTUM PRINCIPLE ----
        v_old = v
        p = p + F_net * dt
        v_new = p / m
        pos = pos + v_new * dt
        v_avg = (v_old + v_new) * 0.5

        # ---- ENERGY LEDGER ----
        dW_regen = -vdot(F_regen_vec, v_avg) * dt
        res.E_regen += dW_regen
        res.E_fric += -vdot(F_fric_vec, v_avg) * dt
        res.E_drag += -vdot(F_drag, v_avg) * dt
        res.E_roll += -vdot(F_roll, v_avg) * dt
        dW_drive = vdot(F_drive_vec, v_avg) * dt
        res.W_drive += dW_drive
        res.W_lat += vdot(F_lat_vec, v_avg) * dt
        E_elec = car.eta_motor * dW_regen
        dE_stored = car.eta_batt * E_elec
        res.E_motor_loss += dW_regen - E_elec
        res.E_batt_loss += E_elec - dE_stored
        res.E_stored += dE_stored
        SOC += (dE_stored - dW_drive / (car.eta_motor * car.eta_batt)) / car.E_batt

        ds = vmag(v_avg) * dt
        s += ds
        if phase == "exit":
            s_exit += ds
        t += dt
        step += 1

        if step % scen.record_every == 0:
            res.t.append(t)
            res.s.append(s)
            res.x.append(pos.x)
            res.z.append(pos.z)
            res.speed.append(speed)
            res.a_cmd.append(a_cmd)
            res.F_req.append(F_req)
            res.F_regen.append(F_regen)
            res.F_fric.append(F_fric)
            res.F_drag.append(vmag(F_drag))
            res.F_roll.append(vmag(F_roll))
            res.F_lat.append(F_lat)
            res.N_f.append(N[0])
            res.N_r.append(N[1])
            res.frac.append(F_regen / F_req if F_req > 1 else 0)
            for i in range(4):
                res.lim[i].append(all_lims[i])
            res.E_stored_t.append(res.E_stored)
            res.E_fric_t.append(res.E_fric)
            res.SOC_t.append(SOC)
            res.grip_use.append(use_regen_axle)
            res.pos.append(pos)

        if vis is not None and step % vis.every == 0:
            vis.update(pos, vhat, speed, t, F_req, F_regen, F_fric, F_regen_vec, F_fric_vec,
                       F_lat_vec, all_lims, use_regen_axle, res)

    res.KE_f = 0.5 * vdot(p, p) / m
    res.t_end = t
    res.s_end = s
    res.SOC_f = SOC
    return res


TIMESERIES_COLUMNS = ["t", "s", "x", "z", "speed", "a_cmd", "F_req", "F_regen", "F_fric",
                      "F_drag", "F_roll", "F_lat", "N_f", "N_r", "regen_frac",
                      "lim_torque", "lim_power", "lim_battery", "lim_grip",
                      "E_stored", "E_fric", "SOC", "grip_use"]


def timeseries_rows(res):
    """Time series as a list of dicts (SI units: s, m, m/s, N, J)."""
    rows = []
    for i in range(len(res.t)):
        rows.append({
            "t": res.t[i], "s": res.s[i], "x": res.x[i], "z": res.z[i], "speed": res.speed[i],
            "a_cmd": res.a_cmd[i], "F_req": res.F_req[i], "F_regen": res.F_regen[i],
            "F_fric": res.F_fric[i], "F_drag": res.F_drag[i], "F_roll": res.F_roll[i],
            "F_lat": res.F_lat[i], "N_f": res.N_f[i], "N_r": res.N_r[i],
            "regen_frac": res.frac[i], "lim_torque": res.lim[0][i], "lim_power": res.lim[1][i],
            "lim_battery": res.lim[2][i], "lim_grip": res.lim[3][i],
            "E_stored": res.E_stored_t[i], "E_fric": res.E_fric_t[i], "SOC": res.SOC_t[i],
            "grip_use": res.grip_use[i],
        })
    return rows


# =============================================================================
# ANALYTIC RESULTS (theory to check the simulation against)      (EQUATIONS.md 7, 8)
# =============================================================================
def analytic_straight_stop(car, v0, a, v_stop=0.3):
    """Closed-form energy for a constant-deceleration straight stop with NO drag and NO
    rolling resistance, when only motor power (high speed) and the faded torque limit
    (below v_fade) can bind. Valid when battery and grip limits don't bind (check with
    analytic_limits_ok). Returns dict of energies in J."""
    m = car.m
    F_T0 = car.T_motor_max * car.G / car.r_w
    F = m * a
    v_star = min(car.P_motor_max / F, v0)                 # power limit binds above v*
    v_t = min(car.v_fade * F / F_T0, car.v_fade)          # faded torque binds below v_t
    E1 = car.P_motor_max * (v0 - v_star) / a              # constant power phase
    v_lo = max(v_t, v_stop)                               # the run ends at v_stop
    E2 = 0.5 * m * (v_star ** 2 - v_lo ** 2)              # 100% regen phase
    E3 = 0.0
    if v_t > v_stop:                                      # torque-fade phase
        E3 = F_T0 * (v_t ** 3 - v_stop ** 3) / (3.0 * car.v_fade * a)
    E_regen = E1 + E2 + E3
    E_brake = 0.5 * m * (v0 ** 2 - v_stop ** 2)
    return {"v_star": v_star, "v_t": v_t, "E1": E1, "E2": E2, "E3": E3,
            "E_regen": E_regen, "E_fric": E_brake - E_regen,
            "E_stored": car.eta_motor * car.eta_batt * E_regen,
            "regen_share": E_regen / E_brake}


def analytic_decel_energy(car, v0, a):
    """Approximate stored energy (J) for a constant-deceleration straight stop WITH drag and
    rolling resistance (EQUATIONS.md 8.4). Drag/rolling losses use v^2 = v0^2 - 2 a x; the
    friction-brake 'excess' above the power limit treats the resistive force as its value at
    v0, R0. Ignores torque fade, battery and grip limits. Returns dict."""
    m, P = car.m, car.P_motor_max
    k = 0.5 * rho_air * car.Cd * car.A if car.include_drag else 0.0
    Rr = car.Crr * m * g if car.include_roll else 0.0
    E_drag = k * v0 ** 4 / (4.0 * a)
    E_roll = Rr * v0 ** 2 / (2.0 * a)
    F = m * a - (k * v0 ** 2 + Rr)              # brake force needed at the start of the stop
    E_excess = 0.0
    if F > 0 and F * v0 > P:
        E_excess = (F * v0 - P) ** 2 / (2.0 * a * F)
    E_regen = 0.5 * m * v0 ** 2 - E_drag - E_roll - E_excess
    return {"E_drag": E_drag, "E_roll": E_roll, "E_fric": E_excess, "E_regen": E_regen,
            "E_stored": car.eta_motor * car.eta_batt * E_regen}


def analytic_optimal_decel(car, v0, lo=0.3, hi=6.0):
    """Deceleration that maximizes analytic_decel_energy (golden-section search)."""
    gr = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    c, d = b - gr * (b - a), a + gr * (b - a)
    for _ in range(80):
        if analytic_decel_energy(car, v0, c)["E_stored"] > analytic_decel_energy(car, v0, d)["E_stored"]:
            b = d
        else:
            a = c
        c, d = b - gr * (b - a), a + gr * (b - a)
    return 0.5 * (a + b)


def grip_limited_decel(car, margin=GRIP_MARGIN, axle="rear"):
    """Largest straight-line deceleration that regen on ONE axle can supply alone without
    passing margin*mu*N on that axle (drag/rolling ignored).                    (EQ 8.3)"""
    mu = margin * car.mu
    if axle == "rear":
        return mu * g * car.a / (car.L + mu * car.h)
    return mu * g * car.b / (car.L - mu * car.h)


def critical_soc(car):
    """SOC above which the battery (not the motor) caps regen at every speed.      (EQ 8.2)"""
    P_elec = car.eta_motor * car.P_motor_max
    if P_elec >= car.P_batt_max:
        return car.SOC_taper
    return 1.0 - (1.0 - car.SOC_taper) * P_elec / car.P_batt_max


def min_all_regen_stop_distance(car, v0, margin=GRIP_MARGIN, axle="rear"):
    """Shortest distance in which regen ALONE can stop the car from v0 (no drag): brake at
    the motor power limit, then at the grip/torque limit.                        (EQ 9)"""
    a_cap = min(grip_limited_decel(car, margin, axle), car.T_motor_max * car.G / car.r_w / car.m)
    F_cap = car.m * a_cap
    v_s = min(car.P_motor_max / F_cap, v0)
    x1 = car.m * (v0 ** 3 - v_s ** 3) / (3.0 * car.P_motor_max)
    x2 = v_s ** 2 / (2.0 * a_cap)
    return x1 + x2, v_s, a_cap


def limit_envelope(car, speeds, decels, SOC=None, margin=GRIP_MARGIN):
    """Regen limits vs speed (straight line). The grip limit depends on deceleration, so it
    is returned once per deceleration in `decels`. Returns list of dicts."""
    if SOC is None:
        SOC = car.SOC0
    iR = 0 if car.regen_axle == "front" else 1
    rows = []
    for v in speeds:
        F_T, F_P, F_B = regen_force_limits(car, v, SOC)
        row = {"speed": v, "lim_torque": F_T, "lim_power": F_P, "lim_battery": F_B}
        for a in decels:
            N = axle_normal_forces(car, a)
            row["lim_grip_a%.1f" % a] = margin * car.mu * N[iR]
            resist = 0.0
            if car.include_drag:
                resist += 0.5 * rho_air * car.Cd * car.A * v * v
            if car.include_roll:
                resist += car.Crr * car.m * g
            row["demand_a%.1f" % a] = max(car.m * a - resist, 0.0)
        rows.append(row)
    return rows


# =============================================================================
# Text helpers (same output format as the original program)
# =============================================================================
def fmt(x, nd):
    if nd == 0:
        return str(int(round(x)))
    return str(round(x, nd))


def pad(txt, n):
    txt = str(txt)
    while len(txt) < n:
        txt = txt + " "
    return txt


def print_summary(res):
    print("---------------------------------------------------------------")
    print(res.scen.name + "  |  strategy: " + res.strat.name + "  |  regen on " + res.regen_axle + " axle")
    print("  duration " + fmt(res.t_end, 2) + " s,  KE " + fmt(res.KE_i / 1000, 1) + " -> "
          + fmt(res.KE_f / 1000, 1) + " kJ")
    print("  where the energy went (kJ): drag " + fmt(res.E_drag / 1000, 1) + ", rolling "
          + fmt(res.E_roll / 1000, 1) + ", friction brakes " + fmt(res.E_fric / 1000, 1)
          + ", motor/inverter loss " + fmt(res.E_motor_loss / 1000, 1) + ", battery loss "
          + fmt(res.E_batt_loss / 1000, 1) + ", STORED " + fmt(res.E_stored / 1000, 1))
    if res.W_drive > 0:
        print("  work by motor holding speed: " + fmt(res.W_drive / 1000, 1) + " kJ")
    print("  regen share of brake work: " + fmt(100 * res.regen_share(), 1) + "%   stored / brake work: "
          + fmt(100 * res.E_stored / max(res.brake_work(), 1), 1) + "%")
    print("  energy-principle residual: " + fmt(res.residual(), 4) + " J  (W_lat = "
          + fmt(res.W_lat, 2) + " J, should be ~0)")
    print("  peak regen-axle grip use: " + fmt(100 * res.peak_grip_use, 1) + "%,  time over grip: "
          + fmt(res.t_grip_violation, 2) + " s,  time demand infeasible: " + fmt(res.t_infeasible, 2) + " s")
    if res.strat.kind == "optimal":
        line = "  time each limit capped regen:"
        for i in range(4):
            line = line + "  " + LIMIT_NAMES[i] + " " + fmt(res.t_binding[i], 2) + " s"
        print(line)


def standard_strategies():
    return [Strategy("friction only", "fixed", 0.0, 1.0),
            Strategy("fixed 25% regen", "fixed", 0.25, 1.0),
            Strategy("fixed 50% regen", "fixed", 0.50, 1.0),
            Strategy("fixed 75% regen", "fixed", 0.75, 1.0),
            Strategy("fixed 100% regen (grip-blind)", "fixed", 1.0, 1.0),
            Strategy("optimal, grip margin " + fmt(GRIP_MARGIN, 2), "optimal", 0, GRIP_MARGIN),
            Strategy("optimal, no margin", "optimal", 0, 1.0)]


def best_constant_split(car, scen, step=0.05, threshold=None):
    """Best constant regen fraction that is at least as safe as the optimal strategy.
    "Safe" = never over the grip limit AND peak regen-axle grip use <= threshold.
    threshold=None uses the optimal run's own peak grip use (a fair comparison: the friction
    brakes also load the regen axle, so even friction-only braking can pass GRIP_MARGIN in a
    corner). Pass threshold=1.0 for "never slides".
    Returns (best_frac, best_E_stored_J, optimal_E_stored_J)."""
    ropt = simulate(car, scen, optimal_strategy())
    if threshold is None:
        threshold = ropt.peak_grip_use
    best_f, best_E = 0.0, -1.0
    n = int(round(1.0 / step))
    for k in range(n + 1):
        f = k * step
        r = simulate(car, scen, Strategy("fixed", "fixed", f, 1.0))
        safe = r.t_grip_violation == 0 and r.peak_grip_use <= threshold + 1e-9
        if safe and r.E_stored > best_E:
            best_E, best_f = r.E_stored, f
    return best_f, best_E, ropt.E_stored
