from vpython import *
# =============================================================================
#  OPTIMIZING REGENERATIVE BRAKING OF ELECTRIC VEHICLES - VPython animation
#  Team: Alex Wang, Devon Parikh, Aiden Knapp
#
#  Equations: Bayar et al. (2012) sheet - see EQUATIONS.md.
#  The physics now lives in regen_core.py (one copy, shared with studies.py and the
#  Simulink comparison). This file is the 3D animation + live graphs for the two headline
#  runs (Milestones 2 and 3) and the original text tables.
#
#  Run:   python regen_braking.py          (opens a browser tab)
#    or   in Jupyter:  %run regen_braking.py
#  Needs regen_core.py in the same folder. (It no longer runs as a single file on
#  glowscript.org / Web VPython.)
#
#  Section numbers match the original program:
#    SECTIONS 1-6  ->  regen_core.py (parameters, limits, grip, core loop, strategies, scenarios)
#    SECTION 7     3D animation and live graphs    (this file)
#    SECTION 8     studies printed as text tables  (this file; full studies -> studies.py)
#    SECTION 9     main program                    (this file)
# =============================================================================
from regen_core import (Car, Strategy, Scenario, make_straight_scenario, make_corner_scenario,
                        simulate, print_summary, standard_strategies, best_constant_split,
                        timeseries_rows, LIMIT_NAMES, optimal_strategy,
                        icurve_strategy, fmt, pad)

VISUAL = True          # 3D animation + live graphs for the two headline runs
PLAYBACK_SPEED = 2     # animation speed multiplier (only matters when VISUAL)
RUN_STUDIES = True     # strategy comparison tables and parameter sweeps
PRINT_CSV = False      # print time series (every 0.1 s); studies.py writes real CSV files


def vv(p):
    """regen_core.Vec -> vpython.vector"""
    return vector(p.x, p.y, p.z)


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
        # the physics engine uses its own small Vec class; convert to VPython vectors
        pos, vhat = vv(pos), vv(vhat)
        F_regen_vec, F_fric_vec, F_lat_vec = vv(F_regen_vec), vv(F_fric_vec), vv(F_lat_vec)
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


# =============================================================================
# SECTION 8 - STUDIES (text tables + VPython graphs). Full set: python studies.py
# =============================================================================
def print_csv(res):
    print("t,speed,F_req,F_regen,F_fric,E_stored")
    rows = timeseries_rows(res)
    for i in range(0, len(rows), 5):
        r = rows[i]
        print(fmt(r["t"], 2) + "," + fmt(r["speed"], 3) + "," + fmt(r["F_req"], 1) + ","
              + fmt(r["F_regen"], 1) + "," + fmt(r["F_fric"], 1) + "," + fmt(r["E_stored"], 1))


def strategy_table(car, scen):
    print("")
    print("=== Strategy comparison: " + scen.name + " (regen on " + car.regen_axle + ") ===")
    print(pad("strategy", 34) + pad("stored kJ", 11) + pad("eta_reg%", 9) + pad("heat kJ", 9)
          + pad("peak grip", 11) + "unstable s")
    for st in standard_strategies():
        r = simulate(car, scen, st, None)
        print(pad(st.name, 34) + pad(fmt(r.E_stored / 1000, 1), 11) + pad(fmt(100 * r.eta_regen(), 1), 9)
              + pad(fmt(r.E_fric / 1000, 1), 9) + pad(fmt(100 * r.peak_grip_use, 0) + "%", 11)
              + fmt(r.t_grip_violation, 2))


def print_best_constant_split(car, scen):
    f, E_best, E_opt = best_constant_split(car, scen)
    print("  " + scen.name + ": best CONSTANT split (never exceeds grip) = " + fmt(100 * f, 0)
          + "% regen -> " + fmt(E_best / 1000, 1) + " kJ;  time-varying optimal -> "
          + fmt(E_opt / 1000, 1) + " kJ")

def run_studies(car, straight, corner, r_s, r_c):
    strategy_table(car, straight)
    strategy_table(car, corner)

    print("")
    print("=== Best constant split vs. time-varying optimal ===")
    print_best_constant_split(car, straight)
    print_best_constant_split(car, corner)

    opt = optimal_strategy()

    print("")
    print("=== Front-axle vs rear-axle regen (optimal strategy) ===")
    for axle in ["rear", "front"]:
        car.regen_axle = axle
        a = simulate(car, straight, opt, None)
        b = simulate(car, corner, opt, None)
        print("  " + pad(axle, 6) + " straight: " + pad(fmt(a.E_stored / 1000, 1) + " kJ (" + fmt(100 * a.eta_regen(), 0)
              + "% eta_regen)", 26) + "  corner: " + fmt(b.E_stored / 1000, 1) + " kJ (" + fmt(100 * b.eta_regen(), 0) + "% eta_regen)")
    car.regen_axle = "rear"

    print("")
    print("=== Straight-line braking intensity sweep (optimal, rear regen) ===")
    gs = graph(width=445, height=250, title="Regen share vs braking intensity (straight)",
               xtitle="deceleration (m/s^2)", ytitle="eta_regen = E_stored/E_B (%)", align="left")
    cs = gcurve(graph=gs, color=color.green, label="rear regen")
    for a_b in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]:
        sc = make_straight_scenario()
        sc.a_brake = a_b
        r = simulate(car, sc, opt, None)
        cs.plot(a_b, 100 * r.eta_regen())
        top = 0
        for i in range(4):
            if r.t_binding[i] > r.t_binding[top]:
                top = i
        print("  a = " + fmt(a_b, 1) + " m/s^2: stored " + pad(fmt(r.E_stored / 1000, 1) + " kJ", 10) + " eta_regen "
              + pad(fmt(100 * r.eta_regen(), 1) + "%", 7) + " most-binding limit: "
              + (LIMIT_NAMES[top] if r.t_binding[top] > 0 else "none")
              + ",  unstable s: " + fmt(r.t_grip_violation, 2))

    print("")
    print("=== Corner radius sweep (optimal, rear regen; same speeds) ===")
    gr = graph(width=445, height=250, title="Regen share vs corner radius",
               xtitle="radius (m)", ytitle="eta_regen = E_stored/E_B (%)", align="left")
    cr = gcurve(graph=gr, color=color.blue, label="rear regen")
    for R in [33.0, 35.0, 38.0, 40.0, 50.0, 80.0, 200.0]:
        sc = make_corner_scenario()
        sc.R = R
        r = simulate(car, sc, opt, None)
        cr.plot(R, 100 * r.eta_regen())
        print("  R = " + pad(fmt(R, 0) + " m", 6) + ": stored " + pad(fmt(r.E_stored / 1000, 1) + " kJ", 10)
              + " eta_regen " + pad(fmt(100 * r.eta_regen(), 1) + "%", 7) + " grip-capped for "
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
# SECTION 9 - MAIN
# =============================================================================
car = Car()
straight = make_straight_scenario()
corner = make_corner_scenario()
optimal = optimal_strategy()

vis = None
if VISUAL:
    vis = Visuals(car, straight, "Milestone 2 - " + straight.name)
r_straight = simulate(car, straight, optimal, vis)
print_summary(r_straight)

vis = None
if VISUAL:
    vis = Visuals(car, corner, "Milestone 3 - " + corner.name)
r_corner = simulate(car, corner, optimal, vis)
print_summary(r_corner)

if PRINT_CSV:
    print_csv(r_straight)
    print_csv(r_corner)

if RUN_STUDIES:
    run_studies(car, straight, corner, r_straight, r_corner)
