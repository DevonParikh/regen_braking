from vpython import *
# Web VPython 3.2
# ^ If you run this on glowscript.org / Web VPython, delete the import line above and
#   make "Web VPython 3.2" (without the #) the very first line. Locally: pip install vpython.

# =============================================================================
#  OPTIMIZING REGENERATIVE BRAKING OF ELECTRIC VEHICLES — core VPython model
#  Team: Alex Wang, Devon Parikh, Aiden Knapp
#
#  How the program is organized (everything plugs into SECTION 4):
#    SECTION 1  Settings + vehicle / world parameters
#    SECTION 2  Motor, battery and energy limits        (Alex owns these)
#    SECTION 3  Weight transfer and tire grip           (Aiden owns these)
#    SECTION 4  Core simulation: momentum principle loop + energy ledger (Devon)
#    SECTION 5  Braking strategies (how braking is split regen vs friction)
#    SECTION 6  Scenarios: straight-line stop (Milestone 2), 90° corner (Milestone 3)
#    SECTION 7  3D animation and live graphs
#    SECTION 8  Studies: strategy comparison, best constant split, sweeps
#    SECTION 9  Main program
#
#  Physics in one paragraph: each time step we find every force on the car
#  (drag, rolling resistance, regen brake, friction brake, sideways tire force in
#  the corner), apply the momentum principle  p_f = p_i + F_net*dt,  then move the
#  car  r_f = r_i + (p_f/m)*dt.  The work done by every force is added up so the
#  energy principle can be checked:  dK = W_drive + W_lat - E_drag - E_roll
#  - E_friction - E_regen.  Regen force is capped by the smallest of four limits:
#  motor torque, motor power, battery charge power, and tire grip on the regen axle.
# =============================================================================


# =============================================================================
# SECTION 1 — SETTINGS AND PARAMETERS
# =============================================================================
VISUAL = True          # 3D animation + live graphs for the two headline runs
PLAYBACK_SPEED = 2     # animation speed multiplier (only matters when VISUAL)
RUN_STUDIES = True     # strategy comparison tables and parameter sweeps
PRINT_CSV = False      # print time series (every 0.1 s) to paste into MATLAB/Simulink

g = 9.81               # m/s^2
rho_air = 1.225        # kg/m^3, sea-level air
mu_road = 0.90         # flat, dry asphalt, constant (proposal assumption)
GRIP_MARGIN = 0.85     # optimal strategy uses at most 85% of the regen axle's grip
                       # (safety margin so rear regen never makes the car unstable)


class Car:
    """Rigid body on two axles. Numbers are roughly a rear-drive Tesla Model 3;
    they are assumptions — cite/replace them in the report."""
    def __init__(self):
        self.m = 1850.0          # kg, mass with driver
        self.L = 2.875           # m, wheelbase
        self.a = 1.50            # m, center of mass -> front axle
        self.b = self.L - self.a # m, center of mass -> rear axle
        self.h = 0.50            # m, center-of-mass height
        self.length = 4.7        # m (drawing only)
        self.width = 1.9         # m (drawing only)

        self.Cd = 0.23           # drag coefficient
        self.A = 2.22            # m^2 frontal area
        self.Crr = 0.010         # rolling resistance coefficient

        self.r_w = 0.34          # m, tire radius
        self.G = 9.0             # gear ratio motor:wheel
        self.T_motor_max = 420.0 # N*m, max motor torque in regen
        self.P_motor_max = 75e3  # W, max mechanical power the motor can absorb
        self.eta_motor = 0.90    # mechanical -> electrical (motor + inverter)
        self.eta_batt = 0.95     # electrical -> chemical (battery charging)
        self.v_fade = 1.5        # m/s, regen fades out below this speed (real EVs do this)

        self.E_batt = 75.0 * 3.6e6  # J, 75 kWh pack
        self.SOC0 = 0.60            # starting state of charge
        self.P_batt_max = 80e3      # W, max charge power when SOC is low
        self.SOC_taper = 0.80       # above this SOC, charge power tapers linearly to 0 at 100%

        self.regen_axle = "rear"    # "rear" or "front": which axle the motor drives

        # gains for the "driver" that keeps the car on the corner midline
        self.k_path = 4.0 * self.m  # N per m of radial error
        self.c_path = 4.0 * self.m  # N per m/s of radial velocity


# =============================================================================
# SECTION 2 — MOTOR, BATTERY, ENERGY LIMITS   (OWNER: Alex — check/replace)
# All limits are returned as a FORCE at the tire contact patch (N), so they can
# be compared directly with each other and with the braking force demand.
# =============================================================================
def battery_charge_power_limit(car, SOC):
    """Max electrical power (W) the battery accepts. Flat, then linear taper to 0 at full."""
    if SOC >= 1.0:
        return 0.0
    if SOC <= car.SOC_taper:
        return car.P_batt_max
    return car.P_batt_max * (1.0 - SOC) / (1.0 - car.SOC_taper)


def regen_force_limits(car, speed, SOC):
    """Returns [F_torque, F_power, F_battery] in N.
    torque:  F = T*G/r_w                 (binds at low speed)
    power:   F*v <= P_motor_max          (binds at high speed)
    battery: eta_motor*F*v <= P_batt     (binds when SOC is high)"""
    F_torque = car.T_motor_max * car.G / car.r_w
    if speed < car.v_fade:
        F_torque = F_torque * speed / car.v_fade
    v = max(speed, 0.1)                       # avoid dividing by zero near a stop
    F_power = car.P_motor_max / v
    F_batt = battery_charge_power_limit(car, SOC) / (car.eta_motor * v)
    return [F_torque, F_power, F_batt]


# =============================================================================
# SECTION 3 — WEIGHT TRANSFER AND TIRE GRIP   (OWNER: Aiden — check/replace)
# =============================================================================
def axle_normal_forces(car, a_decel):
    """[N_front, N_rear] in N. a_decel = deceleration magnitude (m/s^2, + when braking).
    Torque balance about the contact patches: braking shifts m*a*h/L onto the front."""
    W = car.m * g
    shift = car.m * a_decel * car.h / car.L
    N_f = W * car.b / car.L + shift
    N_r = W * car.a / car.L - shift
    return [max(N_f, 0.0), max(N_r, 0.0)]


def axle_lateral_forces(car, F_lat_total):
    """[F_y_front, F_y_rear]: steady cornering, no yaw acceleration, so the sideways
    force splits so its torque about the center of mass is zero."""
    return [F_lat_total * car.b / car.L, F_lat_total * car.a / car.L]


def longitudinal_grip(mu, N, F_y):
    """Friction circle: Fx^2 + Fy^2 <= (mu*N)^2. Returns the braking force still
    available on an axle after the sideways force F_y has used its share."""
    cap = mu * N
    if F_y >= cap:
        return 0.0
    return sqrt(cap * cap - F_y * F_y)


# =============================================================================
# SECTION 5 — BRAKING STRATEGIES
# "optimal": every instant, regen = min(demand, smallest of the 4 limits).
#   Why this is optimal: stored energy = sum of eta*F_regen*v*dt, every term is
#   increasing in F_regen, and each limit only depends on the current instant, so
#   maximizing each instant maximizes the total. (The only coupling is SOC rising,
#   which is tiny over one stop.)
# "fixed": regen asks for a constant fraction of demand. The motor still can't
#   exceed its torque/power/battery limits, but it does NOT know about grip, so
#   it can over-brake the regen axle — that's the instability we flag.
# =============================================================================
class Strategy:
    def __init__(self, name, kind, frac, margin):
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


# =============================================================================
# SECTION 6 — SCENARIOS (the "drive cycle" the car must follow)
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


def make_straight_scenario():
    sc = Scenario("Straight-line stop, 30 m/s -> 0 at 3 m/s^2", "straight")
    return sc


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
        return scen.a_brake
    return 0.0


# =============================================================================
# SECTION 4 — CORE SIMULATION (Devon)
# =============================================================================
LIMIT_NAMES = ["motor torque", "motor power", "battery charge", "tire grip"]


class Results:
    def __init__(self, scen, strat, car):
        self.scen = scen
        self.strat = strat
        self.regen_axle = car.regen_axle
        self.KE_i = 0.0
        self.KE_f = 0.0
        self.W_drive = 0.0      # J, work by motor holding speed (from battery)
        self.W_lat = 0.0        # J, work by sideways tire force (should be ~0)
        self.E_drag = 0.0
        self.E_roll = 0.0
        self.E_fric = 0.0       # J, lost as heat in friction brakes
        self.E_regen = 0.0      # J, mechanical energy into the motor
        self.E_motor_loss = 0.0
        self.E_batt_loss = 0.0
        self.E_stored = 0.0     # J, chemical energy actually stored
        self.t_end = 0.0
        self.SOC_f = 0.0
        self.t_grip_violation = 0.0   # s that any axle needs more than mu*N
        self.t_infeasible = 0.0       # s the brakes couldn't supply the demand within grip
        self.peak_grip_use = 0.0      # max fraction of regen axle's grip used
        self.t_binding = [0.0, 0.0, 0.0, 0.0]   # s each limit was the one capping regen
        # time series
        self.t = []
        self.speed = []
        self.F_req = []
        self.F_regen = []
        self.F_fric = []
        self.frac = []
        self.lim = [[], [], [], []]
        self.E_stored_t = []
        self.E_fric_t = []
        self.grip_use = []
        self.pos = []

    def brake_work(self):
        return self.E_regen + self.E_fric

    def regen_share(self):
        if self.brake_work() <= 0:
            return 0.0
        return self.E_regen / self.brake_work()

    def residual(self):
        """Energy principle check: (K_f - K_i) - (sum of work by all forces). ~0 if consistent."""
        work = self.W_drive + self.W_lat - self.E_drag - self.E_roll - self.E_fric - self.E_regen
        return (self.KE_f - self.KE_i) - work


def simulate(car, scen, strat, vis):
    dt = scen.dt
    m = car.m
    res = Results(scen, strat, car)

    # --- initial state ---
    if scen.kind == "corner":
        pos = vector(-scen.approach, 0, 0)
    else:
        pos = vector(0, 0, 0)
    p = m * vector(scen.v0, 0, 0)            # momentum, car drives along +x
    center = vector(0, 0, -scen.R)           # corner turns left (toward -z)
    phase = "approach"
    s = 0.0          # distance along path
    s_exit = 0.0     # distance after corner exit
    t = 0.0
    SOC = car.SOC0
    res.KE_i = 0.5 * m * scen.v0 * scen.v0
    step = 0

    while t < scen.t_max:
        v = p / m
        speed = mag(v)
        if speed < scen.v_stop:
            break
        vhat = v / speed

        # ---- where are we on the path? ----
        if scen.kind == "corner":
            if phase == "approach" and pos.x >= 0:
                phase = "arc"
            if phase == "arc":
                rel = pos - center
                if atan2(rel.x, rel.z) >= pi / 2:
                    phase = "exit"
            if phase == "exit" and s_exit > scen.exit_length:
                break

        # ---- resistive forces (opposite velocity) ----
        F_drag = -0.5 * rho_air * car.Cd * car.A * speed * speed * vhat
        F_roll = -car.Crr * m * g * vhat
        F_resist = mag(F_drag) + mag(F_roll)

        # ---- the drive cycle: what deceleration is required? ----
        a_cmd = braking_command(scen, s, speed)
        F_drive = 0.0
        if a_cmd > 0:
            F_req = max(0.0, m * a_cmd - F_resist)   # brakes supply what drag+rolling don't
            a_decel = a_cmd
        else:
            F_req = 0.0
            F_drive = F_resist                        # driver holds speed
            a_decel = 0.0

        # ---- sideways (centripetal) tire force in the corner ----
        F_lat_vec = vector(0, 0, 0)
        if phase == "arc":
            rel = pos - center
            r = mag(rel)
            rhat_out = rel / r
            v_rad = dot(v, rhat_out)
            F_c = m * speed * speed / r + car.k_path * (r - scen.R) + car.c_path * v_rad
            F_lat_vec = -F_c * rhat_out
        F_lat = mag(F_lat_vec)

        # ---- grip at each axle (Aiden's functions) ----
        N = axle_normal_forces(car, a_decel)
        Fy = axle_lateral_forces(car, F_lat)
        grip_f = longitudinal_grip(mu_road, N[0], Fy[0])
        grip_r = longitudinal_grip(mu_road, N[1], Fy[1])
        if car.regen_axle == "front":
            iR = 0
        else:
            iR = 1
        F_grip_regen = longitudinal_grip(strat.margin * mu_road, N[iR], Fy[iR])

        # ---- regen limits (Alex's functions) and the strategy's choice ----
        lims = regen_force_limits(car, speed, SOC)
        F_torque = lims[0]
        F_power = lims[1]
        F_batt = lims[2]
        F_regen = choose_regen(strat, F_req, F_torque, F_power, F_batt, F_grip_regen)
        F_fric = F_req - F_regen

        # which limit is capping regen? (only counts when demand exceeds it)
        all_lims = [F_torque, F_power, F_batt, F_grip_regen]
        F_regen_max = min(F_torque, F_power, F_batt, F_grip_regen)
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

        # ---- stability check: does any axle need more than mu*N? ----
        use_f = sqrt(Fx[0] * Fx[0] + Fy[0] * Fy[0]) / max(mu_road * N[0], 1e-9)
        use_r = sqrt(Fx[1] * Fx[1] + Fy[1] * Fy[1]) / max(mu_road * N[1], 1e-9)
        if use_f > 1.0 + 1e-6 or use_r > 1.0 + 1e-6:
            res.t_grip_violation += dt
        if iR == 0:
            use_regen_axle = use_f
        else:
            use_regen_axle = use_r
        res.peak_grip_use = max(res.peak_grip_use, use_regen_axle)

        # ---- force vectors ----
        F_regen_vec = -F_regen * vhat
        F_fric_vec = -F_fric * vhat
        F_drive_vec = F_drive * vhat
        F_net = F_regen_vec + F_fric_vec + F_drive_vec + F_drag + F_roll + F_lat_vec

        # ---- MOMENTUM PRINCIPLE ----
        v_old = v
        p = p + F_net * dt
        v_new = p / m
        pos = pos + v_new * dt
        v_avg = 0.5 * (v_old + v_new)

        # ---- ENERGY LEDGER: work by each force = F . v_avg * dt ----
        dW_regen = -dot(F_regen_vec, v_avg) * dt      # energy into the motor (>= 0)
        res.E_regen += dW_regen
        res.E_fric += -dot(F_fric_vec, v_avg) * dt
        res.E_drag += -dot(F_drag, v_avg) * dt
        res.E_roll += -dot(F_roll, v_avg) * dt
        dW_drive = dot(F_drive_vec, v_avg) * dt
        res.W_drive += dW_drive
        res.W_lat += dot(F_lat_vec, v_avg) * dt
        E_elec = car.eta_motor * dW_regen
        dE_stored = car.eta_batt * E_elec
        res.E_motor_loss += dW_regen - E_elec
        res.E_batt_loss += E_elec - dE_stored
        res.E_stored += dE_stored
        SOC += (dE_stored - dW_drive / (car.eta_motor * car.eta_batt)) / car.E_batt

        ds = mag(v_avg) * dt
        s += ds
        if phase == "exit":
            s_exit += ds
        t += dt
        step += 1

        # ---- record every 10 steps ----
        if step % 10 == 0:
            res.t.append(t)
            res.speed.append(speed)
            res.F_req.append(F_req)
            res.F_regen.append(F_regen)
            res.F_fric.append(F_fric)
            if F_req > 1:
                res.frac.append(F_regen / F_req)
            else:
                res.frac.append(0)
            for i in range(4):
                res.lim[i].append(all_lims[i])
            res.E_stored_t.append(res.E_stored)
            res.E_fric_t.append(res.E_fric)
            res.grip_use.append(use_regen_axle)
            res.pos.append(pos)

        if vis is not None and step % vis.every == 0:
            vis.update(pos, vhat, speed, t, F_req, F_regen, F_fric, F_regen_vec, F_fric_vec,
                       F_lat_vec, all_lims, use_regen_axle, res)

    res.KE_f = 0.5 * mag(p) * mag(p) / m
    res.t_end = t
    res.SOC_f = SOC
    return res


# =============================================================================
# SECTION 7 — 3D ANIMATION AND LIVE GRAPHS
# =============================================================================
class Visuals:
    def __init__(self, car, scen, title):
        self.every = 10                                  # draw every 10 physics steps
        self.rate_hz = PLAYBACK_SPEED / (scen.dt * self.every)
        self.car = car
        self.scen = scen
        self.ymax = 1.6 * car.m * scen.a_brake           # limit curves clipped here so graph is readable
        sc = canvas(title="<b>" + title + "</b>", width=900, height=420,
                    background=vector(0.75, 0.85, 1.0))
        self.sc = sc

        # path points along the midline
        pts = []
        if scen.kind == "corner":
            pts.append(vector(-scen.approach - 10, 0, 0))
            pts.append(vector(0, 0, 0))
            n = 30
            for i in range(1, n + 1):
                th = (pi / 2) * i / n
                pts.append(vector(scen.R * sin(th), 0, -scen.R + scen.R * cos(th)))
            pts.append(vector(scen.R, 0, -scen.R - scen.exit_length - 10))
        else:
            d_stop = scen.v0 * scen.v0 / (2 * scen.a_brake)
            pts.append(vector(-20, 0, 0))
            pts.append(vector(d_stop + 30, 0, 0))

        # ground, road (boxes along the path), midline
        xs = [q.x for q in pts]
        zs = [q.z for q in pts]
        cx = 0.5 * (min(xs) + max(xs))
        cz = 0.5 * (min(zs) + max(zs))
        box(canvas=sc, pos=vector(cx, -0.06, cz),
            size=vector(max(xs) - min(xs) + 60, 0.1, max(zs) - min(zs) + 60),
            color=vector(0.35, 0.6, 0.3))
        for i in range(len(pts) - 1):
            d = pts[i + 1] - pts[i]
            box(canvas=sc, pos=0.5 * (pts[i] + pts[i + 1]), axis=d,
                size=vector(mag(d) + 0.4, 0.04, 7.0), color=vector(0.3, 0.3, 0.32))
        mid = curve(canvas=sc, color=color.white, radius=0.08)
        for q in pts:
            mid.append(q + vector(0, 0.05, 0))

        # car and force arrows
        start = vector(0, 0, 0)
        if scen.kind == "corner":
            start = vector(-scen.approach, 0, 0)
        self.body = box(canvas=sc, pos=start + vector(0, 0.75, 0),
                        size=vector(car.length, 1.4, car.width), color=color.red)
        up = vector(0, 2.2, 0)
        self.up = up
        self.a_v = arrow(canvas=sc, pos=start + up, axis=vector(0, 0, 0), color=color.yellow, shaftwidth=0.25)
        self.a_regen = arrow(canvas=sc, pos=start + up, axis=vector(0, 0, 0), color=color.green, shaftwidth=0.35)
        self.a_fric = arrow(canvas=sc, pos=start + up, axis=vector(0, 0, 0), color=color.orange, shaftwidth=0.35)
        self.a_lat = arrow(canvas=sc, pos=start + up, axis=vector(0, 0, 0), color=color.blue, shaftwidth=0.35)
        self.trail = curve(canvas=sc, color=color.red, radius=0.1)

        if scen.kind == "corner":
            sc.center = vector(scen.R * 0.35, 0, -scen.R * 0.35)
            sc.range = scen.R * 1.25
            sc.forward = vector(0.25, -1.0, -0.45)
        else:
            sc.camera.follow(self.body)
            sc.range = 22
            sc.forward = vector(0.6, -0.45, -0.65)

        # graphs
        gw = 445
        self.g1 = graph(width=gw, height=250, title="Speed", xtitle="t (s)", ytitle="m/s", align="left")
        self.c_speed = gcurve(graph=self.g1, color=color.black, label="speed")
        self.g2 = graph(width=gw, height=250, title="Braking force split (limits clipped)",
                        xtitle="t (s)", ytitle="N", align="left")
        self.c_req = gcurve(graph=self.g2, color=color.black, label="demand")
        self.c_regen = gcurve(graph=self.g2, color=color.green, label="regen")
        self.c_fric = gcurve(graph=self.g2, color=color.orange, label="friction")
        lim_colors = [color.magenta, color.purple, color.cyan, color.blue]
        self.c_lim = []
        for i in range(4):
            self.c_lim.append(gcurve(graph=self.g2, color=lim_colors[i], label=LIMIT_NAMES[i] + " limit"))
        self.g3 = graph(width=gw, height=250, title="Energy", xtitle="t (s)", ytitle="kJ", align="left")
        self.c_stored = gcurve(graph=self.g3, color=color.green, label="stored in battery")
        self.c_heat = gcurve(graph=self.g3, color=color.orange, label="friction-brake heat")
        self.g4 = graph(width=gw, height=250, title="Regen-axle grip used (1 = sliding)",
                        xtitle="t (s)", ytitle="fraction of mu*N", align="left", ymin=0, ymax=1.2)
        self.c_grip = gcurve(graph=self.g4, color=color.blue, label="regen axle")

    def update(self, pos, vhat, speed, t, F_req, F_regen, F_fric, F_regen_vec, F_fric_vec,
               F_lat_vec, lims, grip_use, res):
        rate(self.rate_hz)
        self.body.pos = pos + vector(0, 0.75, 0)
        self.body.axis = vhat * self.car.length
        self.body.size = vector(self.car.length, 1.4, self.car.width)
        self.trail.append(pos + vector(0, 0.1, 0))
        base = pos + self.up
        self.a_v.pos = base
        self.a_v.axis = 0.25 * speed * vhat
        self.a_regen.pos = base
        self.a_regen.axis = F_regen_vec * 0.001          # 1 m of arrow = 1000 N
        self.a_fric.pos = base + vector(0, 0.5, 0)
        self.a_fric.axis = F_fric_vec * 0.001
        self.a_lat.pos = base
        self.a_lat.axis = F_lat_vec * 0.001

        self.c_speed.plot(t, speed)
        self.c_req.plot(t, F_req)
        self.c_regen.plot(t, F_regen)
        self.c_fric.plot(t, F_fric)
        for i in range(4):
            self.c_lim[i].plot(t, min(lims[i], self.ymax))
        self.c_stored.plot(t, res.E_stored / 1000)
        self.c_heat.plot(t, res.E_fric / 1000)
        self.c_grip.plot(t, grip_use)
        share = 0
        if F_req > 1:
            share = 100 * F_regen / F_req
        self.sc.caption = ("t = " + fmt(t, 2) + " s   speed = " + fmt(speed, 1) + " m/s   regen share now = "
                           + fmt(share, 0) + "%   stored = " + fmt(res.E_stored / 1000, 1) + " kJ"
                           + "\nArrows: yellow = velocity, green = regen force, orange = friction brake, "
                           + "blue = sideways tire force (1 m = 1000 N)")


# =============================================================================
# SECTION 8 — STUDIES
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


def print_csv(res):
    print("t,speed,F_req,F_regen,F_fric,E_stored")
    for i in range(0, len(res.t), 5):       # recorded every 10 steps (0.02 s at dt=0.002) -> every 5th = 0.1 s
        print(fmt(res.t[i], 2) + "," + fmt(res.speed[i], 3) + "," + fmt(res.F_req[i], 1) + ","
              + fmt(res.F_regen[i], 1) + "," + fmt(res.F_fric[i], 1) + "," + fmt(res.E_stored_t[i], 1))


def strategy_table(car, scen):
    print("")
    print("=== Strategy comparison: " + scen.name + " (regen on " + car.regen_axle + ") ===")
    print(pad("strategy", 34) + pad("stored kJ", 11) + pad("regen %", 9) + pad("heat kJ", 9)
          + pad("peak grip", 11) + "unstable s")
    strats = [Strategy("friction only", "fixed", 0.0, 1.0),
              Strategy("fixed 25% regen", "fixed", 0.25, 1.0),
              Strategy("fixed 50% regen", "fixed", 0.50, 1.0),
              Strategy("fixed 75% regen", "fixed", 0.75, 1.0),
              Strategy("fixed 100% regen (grip-blind)", "fixed", 1.0, 1.0),
              Strategy("optimal, grip margin " + fmt(GRIP_MARGIN, 2), "optimal", 0, GRIP_MARGIN),
              Strategy("optimal, no margin", "optimal", 0, 1.0)]
    for st in strats:
        r = simulate(car, scen, st, None)
        print(pad(st.name, 34) + pad(fmt(r.E_stored / 1000, 1), 11) + pad(fmt(100 * r.regen_share(), 1), 9)
              + pad(fmt(r.E_fric / 1000, 1), 9) + pad(fmt(100 * r.peak_grip_use, 0) + "%", 11)
              + fmt(r.t_grip_violation, 2))


def best_constant_split(car, scen):
    """Grid search: best constant regen fraction that stays inside the SAME grip margin
    the optimal strategy is held to (so the comparison is fair), vs. optimal."""
    best_f = 0
    best_E = -1
    f = 0.0
    while f <= 1.0001:
        r = simulate(car, scen, Strategy("fixed", "fixed", f, 1.0), None)
        safe = r.t_grip_violation == 0 and r.peak_grip_use <= GRIP_MARGIN + 1e-9
        if safe and r.E_stored > best_E:
            best_E = r.E_stored
            best_f = f
        f += 0.05
    ropt = simulate(car, scen, Strategy("optimal", "optimal", 0, GRIP_MARGIN), None)
    print("  " + scen.name + ": best safe CONSTANT split = " + fmt(100 * best_f, 0) + "% regen -> "
          + fmt(best_E / 1000, 1) + " kJ;  time-varying optimal -> " + fmt(ropt.E_stored / 1000, 1) + " kJ")


def run_studies(car, straight, corner, r_s, r_c):
    strategy_table(car, straight)
    strategy_table(car, corner)

    print("")
    print("=== Best constant split vs. time-varying optimal ===")
    best_constant_split(car, straight)
    best_constant_split(car, corner)

    opt = Strategy("optimal", "optimal", 0, GRIP_MARGIN)

    print("")
    print("=== Front-axle vs rear-axle regen (optimal strategy) ===")
    for axle in ["rear", "front"]:
        car.regen_axle = axle
        a = simulate(car, straight, opt, None)
        b = simulate(car, corner, opt, None)
        print("  " + pad(axle, 6) + " straight: " + pad(fmt(a.E_stored / 1000, 1) + " kJ (" + fmt(100 * a.regen_share(), 0)
              + "% regen)", 22) + "  corner: " + fmt(b.E_stored / 1000, 1) + " kJ (" + fmt(100 * b.regen_share(), 0) + "% regen)")
    car.regen_axle = "rear"

    print("")
    print("=== Straight-line braking intensity sweep (optimal, rear regen) ===")
    gs = graph(width=445, height=250, title="Regen share vs braking intensity (straight)",
               xtitle="deceleration (m/s^2)", ytitle="% of brake work by regen", align="left")
    cs = gcurve(graph=gs, color=color.green, label="rear regen")
    for a_b in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]:
        sc = make_straight_scenario()
        sc.a_brake = a_b
        r = simulate(car, sc, opt, None)
        cs.plot(a_b, 100 * r.regen_share())
        top = 0
        for i in range(4):
            if r.t_binding[i] > r.t_binding[top]:
                top = i
        print("  a = " + fmt(a_b, 1) + " m/s^2: stored " + pad(fmt(r.E_stored / 1000, 1) + " kJ", 10) + " regen share "
              + pad(fmt(100 * r.regen_share(), 1) + "%", 7) + " most-binding limit: " + LIMIT_NAMES[top]
              + ",  unstable s: " + fmt(r.t_grip_violation, 2))

    print("")
    print("=== Corner radius sweep (optimal, rear regen; same speeds) ===")
    gr = graph(width=445, height=250, title="Regen share vs corner radius",
               xtitle="radius (m)", ytitle="% of brake work by regen", align="left")
    cr = gcurve(graph=gr, color=color.blue, label="rear regen")
    for R in [35.0, 40.0, 50.0, 65.0, 80.0, 120.0, 200.0]:
        sc = make_corner_scenario()
        sc.R = R
        r = simulate(car, sc, opt, None)
        cr.plot(R, 100 * r.regen_share())
        print("  R = " + pad(fmt(R, 0) + " m", 6) + ": stored " + pad(fmt(r.E_stored / 1000, 1) + " kJ", 10)
              + " regen share " + pad(fmt(100 * r.regen_share(), 1) + "%", 7) + " grip-capped for "
              + fmt(r.t_binding[3], 2) + " s,  infeasible s: " + fmt(r.t_infeasible, 2))

    # regen share vs time for the two headline runs
    gf = graph(width=445, height=250, title="Optimal regen share of braking vs time",
               xtitle="t (s)", ytitle="F_regen / F_demand", align="left", ymin=0, ymax=1.05)
    c1 = gcurve(graph=gf, color=color.green, label="straight")
    c2 = gcurve(graph=gf, color=color.blue, label="corner")
    for i in range(0, len(r_s.t), 5):
        if r_s.F_req[i] > 1:
            c1.plot(r_s.t[i], r_s.frac[i])
    for i in range(0, len(r_c.t), 5):
        if r_c.F_req[i] > 1:
            c2.plot(r_c.t[i], r_c.frac[i])


# =============================================================================
# SECTION 9 — MAIN
# =============================================================================
car = Car()
straight = make_straight_scenario()
corner = make_corner_scenario()
optimal = Strategy("optimal (max regen within limits)", "optimal", 0, GRIP_MARGIN)

vis = None
if VISUAL:
    vis = Visuals(car, straight, "Milestone 2 — " + straight.name)
r_straight = simulate(car, straight, optimal, vis)
print_summary(r_straight)

vis = None
if VISUAL:
    vis = Visuals(car, corner, "Milestone 3 — " + corner.name)
r_corner = simulate(car, corner, optimal, vis)
print_summary(r_corner)

if PRINT_CSV:
    print_csv(r_straight)
    print_csv(r_corner)

if RUN_STUDIES:
    run_studies(car, straight, corner, r_straight, r_corner)
