# Equations: Optimizing Regenerative Braking of Electric Vehicles

Alex Wang, Devon Parikh, Aiden Knapp

**Primary source:** Bayar, Biasini, Onori & Rizzoni (2012), *Modelling and control of a
brake system for an extended range electric vehicle equipped with axle motors*, Int. J.
Vehicle Design 58(2/3/4), 399–426. We use them as simplified in our sheet *Regenerative
Braking: Equations for Our Model*.

**Part I** is the model, Stages 1–4 plus the energy bookkeeping, exactly as on the sheet.
**Part II** is the analysis built on top of it: theory checks, crossovers, optimization,
validation and sensitivity. Every equation names the code that implements it.

**Model assumptions (sheet):** rigid body on two axles, rear-axle motor, flat dry road,
constant μ, no tire slip, no suspension.

## What changed from our first version

| | first version (team code) | now (Bayar sheet) |
|---|---|---|
| efficiency | motor $\eta_m$ = 0.90 × battery $\eta_b$ = 0.95 | one lumped wheel-to-battery $\eta$ = 0.70 |
| low-speed regen | torque faded linearly to 0 below 1.5 m/s | hard cutoff: $F_{reg}=0$ for $V \le V_{cut}$ = 1.5 m/s |
| battery cap | $P_{chg}/(\eta_m V)$, taper from 80% SOC | $P_{chg,max}(SOC)/(\eta V)$, constant to 90% SOC then linear to 0 |
| rolling resistance | $C_{rr}$ = 0.010 | $C_{rr}$ = 0.012 |
| grip cap | 85% of $\mu N_r$ (a safety margin) | $\mu N_r$ (friction circle in a corner) |
| deceleration in load transfer | commanded $a$ (includes drag and rolling) | $a = F_b/M$ (braking force only) |
| stability comparison | grip margin | **I-curve** (ideal front/rear distribution) vs grip limit |
| figure of merit | regen share $E_{regen}/E_B$ | $\eta_{regen} = E_{stored}/E_B$ |

---

## Notation

| symbol | meaning | code | value (our car) |
|---|---|---|---|
| $M$ | vehicle mass | `car.m` | 1850 kg |
| $V$ | forward speed | `speed` | |
| $l_f,\ l_r$ | front / rear axle to CoM (paper's $a$, $b$) | `car.a`, `car.b` | 1.50, 1.375 m |
| $L = l_f + l_r$ | wheelbase | `car.L` | 2.875 m |
| $h$ | CoM height | `car.h` | 0.50 m |
| $\mu$ | tire–road friction coefficient | `car.mu` | 0.90 |
| $\rho,\ C_d,\ A$ | air density, drag coefficient, frontal area | `rho_air`, `car.Cd`, `car.A` | 1.225 kg/m³, 0.23, 2.22 m² |
| $C_{rr}$ | rolling-resistance coefficient | `car.Crr` | 0.012 |
| $G,\ r_w$ | motor-to-wheel gear ratio, tire rolling radius | `car.G`, `car.r_w` | 9, 0.34 m |
| $T_{max},\ P_{max}$ | max motor torque / power | `car.T_motor_max`, `car.P_motor_max` | 420 N·m, 75 kW |
| $P_{chg,max}(SOC)$ | battery charge-power limit | `battery_charge_power_limit` | 80 kW to 90% SOC |
| $\eta$ | wheel-to-battery efficiency | `car.eta` | 0.70 |
| $V_{cut}$ | low-speed regen cutoff | `car.v_cut` | 1.5 m/s |
| $E_{cap}$ | battery energy capacity | `car.E_batt` | 75 kWh |
| $F_b$ | total braking force required | `F_req` | |
| $F_{reg},\ F_{fric}$ | regen / friction-brake force | `F_regen`, `F_fric` | |
| $F_a,\ F_r$ | drag, rolling resistance | `F_drag`, `F_roll` | |
| $N_f,\ N_r$ | front / rear normal force | `N_f`, `N_r` | |
| $a$ | braking deceleration magnitude, $a = F_b/M$ | `a_lt` | |
| $R,\ F_{y,r}$ | turn radius, rear-axle lateral force | `scen.R`, `Fy[1]` | |

The geometry, motor and battery values are for a rear-drive Tesla Model 3. The paper does
not give them, so **each needs a source in the report**.

---

# Part I: The model (Bayar sheet)

## Stage 1: point mass, straight line, motor torque only  [Bayar eq. 1]

$$M\frac{dV}{dt} = -F_{reg} - F_{fric},\qquad F_{reg} = \min\!\left(F_b,\ \frac{G\,T_{max}}{r_w}\right),\qquad F_{fric} = F_b - F_{reg}$$

This is $d\vec p/dt = \vec F_{net}$ along $\hat x$. The VPython update is
`p = p + F_net*dt`, then `r = r + (p/M)*dt` (Euler–Cromer).
Code: `simulate()`, **MOMENTUM PRINCIPLE** block.

## Stage 2: resistive forces, all regen caps, drive cycle  [Bayar eq. 2, 7, 8, 10]

$$F_a = \tfrac12\rho C_d A V^2,\qquad F_r = C_{rr} M g,\qquad
F_b(t) = -M\frac{dV_{cyc}}{dt} - F_a - F_r\quad(\text{only when } > 0)$$

$$F_{reg} = \begin{cases}\min\!\left(F_b,\ \dfrac{G T_{max}}{r_w},\ \dfrac{P_{max}}{V},\ \dfrac{P_{chg,max}(SOC)}{\eta V},\ F_{grip}\right) & V > V_{cut}\\[2mm] 0 & V \le V_{cut}\end{cases}$$

$$P_{chg,max}(SOC) = \begin{cases} P_{b} & SOC \le 0.90\\ P_{b}\,\dfrac{1-SOC}{1-0.90} & 0.90 < SOC < 1\end{cases}$$

The four caps are torque (binds at low speed), power (binds at high speed, ∝ 1/V),
battery, and grip. Drag and rolling resistance brake for free, so the brakes supply only
the remainder. In the corner, the drive cycle holds speed after braking ends, so
$F_b = 0$ there and the motor supplies $F_a + F_r$.
Code: `regen_force_limits`, `battery_charge_power_limit`, `choose_regen`, `braking_command`.

## Stage 3: two axles, load transfer sets the rear grip cap  [Bayar eq. 5, 6]

$$N_f = \frac{M(g\,l_r + a\,h)}{L},\qquad N_r = \frac{M(g\,l_f - a\,h)}{L},\qquad a = \frac{F_b}{M},\qquad F_{grip} = \mu N_r$$

This comes from $\sum\vec\tau_{CoM} = \vec 0$ (no pitching). Harder braking moves load
$Mah/L$ from rear to front, shrinking rear grip and therefore rear regen.
Code: `axle_normal_forces`.

**I-curve (stricter stability limit, optional comparison):**

$$F_{grip} = F_{br}^{ideal} = F_b\,\frac{M g\,l_f - h F_b}{M g\,L}\qquad\left(F_{bf}^{ideal} = F_b\,\frac{M g\,l_r + h F_b}{M g\,L},\quad F_{bf}+F_{br}=F_b\right)$$

On the I-curve, $F_{bf}/F_{br} = N_f/N_r$, so both axles use the same fraction of their
grip and reach the limit together. The grip limit keeps the rear from locking. The I-curve
also keeps the rear from being used harder than the front, so the car stays straight, but
it recovers less energy. **The energy difference between them is the energy–stability
trade-off.**
Code: `ideal_brake_force`, strategy `icurve_strategy()`.

## Stage 4: braking while cornering, friction circle on the rear axle  [Bayar ext. 12]

$$F_{y,r} = \frac{M V^2}{R}\,\frac{l_f}{L},\qquad F_{grip} = \sqrt{(\mu N_r)^2 - F_{y,r}^2}$$

The share $l_f/L$ comes from zero net torque about the CoM in steady cornering.
Cornering uses up part of the rear tire's grip and lowers the regen cap. The car slows
through the corner (enters at $V_1$, exits at $V_2 < V_1$); at constant speed $\Delta K = 0$
and there is nothing to recover.
In the code, the total sideways force is $M V^2/r$ plus a small spring–damper "driver"
term that keeps the car on the midline: $F_y = MV^2/r + k(r-R) + c\,v_r$.
Code: `axle_lateral_forces`, `longitudinal_grip`.

## Energy bookkeeping and validation (every stage)  [Bayar eq. 3, 11]

$$\frac{d\,SOC}{dt} = \frac{\eta F_{reg} V}{E_{cap}},\qquad E_{stored} = \int \eta F_{reg} V\,dt,\qquad E_B = \int F_b V\,dt,\qquad \eta_{regen} = \frac{E_{stored}}{E_B}$$

$$\textbf{Check: }\ -\Delta K = E_{stored} + E_{loss} + E_{fric} + E_{drag} + E_{roll}$$

$$E_{loss} = \int (1-\eta)F_{reg}V\,dt,\quad E_{fric} = \int F_{fric}V\,dt,\quad E_{drag} = \int F_a V\,dt,\quad E_{roll} = \int F_r V\,dt$$

In the corner the motor also does work $W_{drive}$ holding speed after braking, so the
check there reads $-\Delta K + W_{drive} = \dots$. That draws $W_{drive}/\eta$ from the
battery. If the two sides differ by more than round-off, the code has a bug.
Code: `simulate()`, **ENERGY BOOKKEEPING** block; `Results.residual()`, `Results.eta_regen()`.

---

# Part II: Analysis built on the model

## A1. Analytic solution for a straight stop (theory check, Alex)

Take a stop from $V_0$ at constant $a$ with **no drag or rolling resistance**, so
$F_b = Ma$ and $dV = -a\,dt$. Assume $G T_{max}/r_w \ge Ma$ and that the battery and grip
caps don't bind. Then regen is power-capped above $V^*$, does all the braking between $V_{cut}$
and $V^*$, and is zero below $V_{cut}$:

$$V^* = \frac{P_{max}}{M a},\qquad
E_1 = \int P_{max}\,dt = \frac{P_{max}(V_0 - V^*)}{a},\qquad
E_2 = \tfrac12 M\left(V^{*2} - V_{cut}^2\right)$$

$$E_{stored} = \eta\,(E_1 + E_2),\qquad E_{fric} = \tfrac12 M (V_0^2 - V_{stop}^2) - E_1 - E_2$$

(If $V^* > V_0$, then $E_1 = 0$.) For $V_0$ = 30 m/s and $a$ = 3 m/s², $V^*$ = 13.5 m/s.
Theory gives **405.30 kJ** and the simulation gives **405.27 kJ (0.007%)**.
Code: `analytic_straight_stop`; data: `data/analytic_check.csv`.

## A2. When each cap binds (closed-form crossovers, Alex + Aiden)

- **Power vs. full regen:** regen can supply all of $F_b$ only below $V^* = P_{max}/F_b$.
- **Battery:** the battery cap is below the power cap at every speed once $P_{chg,max}(SOC)/\eta < P_{max}$:
$$SOC_{crit} = 1 - (1-0.90)\,\frac{\eta P_{max}}{P_b} = 0.934$$
  At SOC₀ = 0.60 the battery cap never binds. At 0.96, the car stores 270 kJ instead of 400 kJ (`fig07`).
- **Grip (straight line):** rear regen alone can supply $a$ while $Ma \le \mu N_r(a)$:
$$a_{grip,rear} = \frac{\mu g\,l_f}{L + \mu h} = 3.98\ \text{m/s}^2,\qquad a_{grip,front} = \frac{\mu g\,l_r}{L - \mu h} = 5.01\ \text{m/s}^2$$
  Load transfer helps front regen and hurts rear regen.
- **Torque:** binds only above $G T_{max}/(r_w M)$ = 6.0 m/s², so grip runs out first.
- **I-curve:** the rear may take at most $F_{br}^{ideal}/F_b = (g\,l_f - h a)/(gL)$, which is 52% at rest and **47% at 3 m/s²**. Regen can never do more than about half the braking on the I-curve.

Code: `critical_soc`, `grip_limited_decel`, `icurve_rear_fraction`; data: `data/derived_constants.csv`.

## A3. Energy-optimal braking intensity (team)

Gentle braking stays under the power cap but takes longer, so drag and rolling resistance
take more energy. With $V^2 = V_0^2 - 2as$ over the stopping distance $V_0^2/2a$:

$$E_{drag} = \frac{\rho C_d A\,V_0^4}{8a},\qquad E_{roll} = \frac{C_{rr} M g\,V_0^2}{2a}$$

Friction must supply the force above the power cap. Treat the resistive force as its value
at $V_0$, $R_0$, and write $F = Ma - R_0$:

$$E_{fric,P} \approx \int_{V^*}^{V_0}\!\Big(F - \frac{P_{max}}{V}\Big)\frac{V\,dV}{a} = \frac{(F V_0 - P_{max})^2}{2aF},\qquad E_{fric,cut} = \tfrac12 M\left(V_{cut}^2 - V_{stop}^2\right)$$

$$E_{stored}(a) \approx \eta\left[\tfrac12 M (V_0^2 - V_{stop}^2) - E_{drag} - E_{roll} - E_{fric,P} - E_{fric,cut}\right]$$

The drag and rolling terms fall like $1/a$ and $E_{fric,P}$ rises with $a$, so there is a
best deceleration. Theory gives **$a_{opt}$ = 1.71 m/s²**. The simulation's best point is
1.75 m/s² at **512 kJ**, which is 28% more than the 3 m/s² stop (400 kJ). Theory is within
2.2% of the simulation up to 3 m/s² (`fig05`).
Code: `analytic_decel_energy`, `analytic_optimal_decel`; data: `data/sweep_decel.csv`.

## A4. Braking profile at a fixed stopping distance (team)

The shortest stop regen can do *alone* brakes at the power cap down to $V_s$, then at the
grip/torque cap $a_c$:

$$M V\frac{dV}{ds} = -\frac{P_{max}}{V}\ \Rightarrow\ s_1 = \frac{M(V_0^3 - V_s^3)}{3P_{max}},\qquad s_2 = \frac{V_s^2}{2a_c},\qquad V_s = \frac{P_{max}}{M a_c}$$

From 30 m/s this is **226 m** (rear regen). Our 3 m/s² stop takes $D$ = 150 m, so friction
is unavoidable. Two-phase profiles ($a_1$ while $V > V_s$, then $a_2$) over the same
$D$ must satisfy

$$\frac{V_0^2 - V_s^2}{2a_1} + \frac{V_s^2}{2a_2} = D$$

The best one brakes gently while the power cap binds and harder at low speed, up to the grip
limit: $a_1$ = 2.8 m/s² above 14 m/s, then $a_2$ = 4.04 m/s². That stores **412 vs 400 kJ (+3.1%)**.
Code: `studies.profile_optimization`; `fig11`.

## A5. Numerical check and convergence (Devon)

**Why the energy check closes exactly.** Work per step uses
$\vec V_{avg} = \tfrac12(\vec V_n + \vec V_{n+1})$, and $\vec p_{n+1} - \vec p_n = \vec F_{net}\Delta t$, so

$$\Delta K = \frac{(\vec p_{n+1}-\vec p_n)\cdot(\vec p_{n+1}+\vec p_n)}{2M} = \vec F_{net}\cdot\vec V_{avg}\,\Delta t$$

The residual is therefore zero to round-off for **any** $\Delta t$. This shows that every
force is counted once. It does not show accuracy.

**Accuracy** comes from convergence: Euler–Cromer error is $O(\Delta t)$. With
$\varepsilon(\Delta t) = |E(\Delta t) - E(\Delta t_{min})|/E(\Delta t_{min})$, we get
$\varepsilon$ ≤ 0.06% for every $\Delta t$ from 0.008 s down to 0.0005 s, and 0.006% / 0.02%
(straight / corner) at our $\Delta t$ = 0.002 s.
Code: `studies.convergence_study`; `fig10`.

## A6. Simulink comparison metrics (Milestone 4, Devon)

$$\%\,\text{diff} = 100\,\frac{X_{SL} - X_{VP}}{X_{VP}},\qquad
\text{NRMSE} = \frac{\sqrt{\frac1n\sum_k (X_{VP}(t_k) - X_{SL}(t_k))^2}}{\max X_{VP} - \min X_{VP}}$$

Use these for $X$ = $V$, $F_{reg}$, $E_{stored}$, $\eta_{regen}$, SOC, $N_f$, $N_r$ and $F_{y,r}$.
Code: `simulink/compare_simulink.py`.

## A7. Sensitivity to assumed parameters (team)

$$S_p = \frac{\Delta E_{stored}/E_{stored}}{\Delta p/p_0}$$

The results are in `data/sensitivity.csv` and `fig09`:
- **η** has $S$ = 1 exactly, because $E_{stored} \propto \eta$.
- **$P_{max}$** has $S$ ≈ 0.8: the power cap is the main limit.
- **μ** matters only when grip binds: $S$ ≈ 0.2 on the straight stop (μ = 0.5 costs 7%) and 0.15–0.3 in the corners. Lowering μ quickly makes the corner itself impossible. At μ = 0.5 (wet road), and at μ = 0.8 in the 35 m corner, braking + cornering exceed total grip, so the car **can't make the corner at all**. Those runs show the limit of the scenario, not a change in stored energy, and are left out of the sensitivity numbers.
- **$C_d$, $C_{rr}$, $V_{cut}$** have $|S|$ < 0.05. Extra drag in the power-capped phase replaces friction braking, not regen.

## E1. Extension (assumption test): left–right load transfer

The sheet drops the left/right split. To test that, each axle's load is split between its
tires, with $\Delta N = F_{y,axle}\,h/t_w$ moved to the outer tire, and tire friction is
made to fall with load, $\mu_j = \mu[1 - k(N_j/N_0 - 1)]$:

$$C_{axle} = \mu\left[N(1+k) - \frac{k}{N_0}\Big(\frac{N^2}{2} + 2\Delta N^2\Big)\right]\quad\text{replaces}\quad \mu N$$

With constant μ ($k$ = 0) this is exactly $\mu N$, so the sheet's model is unchanged. Even
at $k$ = 0.2, stored energy in the 35 m corner falls by only 0.9%, and no inner wheel lifts.
**Dropping left/right is justified.**
Code: `axle_capacity`, `car.k_load`; data: `data/load_transfer.csv`.

---

## Which Bayar equations are in the code

| # | paper equation | sheet verdict | where |
|---|---|---|---|
| 1 | momentum principle | KEEP | `simulate()` momentum block |
| 2 | drag + rolling resistance | SIMPLIFY ($C_{rr}$ constant) | `simulate()` Stage 2 block |
| 3 | power balance | KEEP (as check) | `Results.residual()` |
| 4 | wheel rotation / $M_{eq}$ | DROP | not used (would add ≈2.5% to $M$) |
| 5 | load transfer | KEEP (per axle) | `axle_normal_forces` |
| 6 | ideal (I-curve) distribution | OPTIONAL | `ideal_brake_force`, `icurve_strategy` |
| 7 | brake force request | SIMPLIFY (drive cycle) | `braking_command`, `F_req` |
| 8 | layer-2 argmax | SIMPLIFY (min of caps) | `choose_regen` |
| 9 | battery circuit | SIMPLIFY ($\eta$ + $P_{chg,max}$) | `battery_charge_power_limit` |
| 10 | battery power from motor | SIMPLIFY ($\eta F_{reg} V$) | energy block |
| 11 | performance metrics | KEEP (force form) | `E_B`, `E_stored`, `eta_regen()` |
| 12 | cornering friction circle | KEEP | `axle_lateral_forces`, `longitudinal_grip` |

## Assumptions and how each is tested

| assumption | effect | test |
|---|---|---|
| constant μ, dry road | sets every grip cap | A7: μ = 0.5–1.0 |
| no tire slip (friction circle) | grip is a sharp limit | Simulink with Magic Formula tires (A6) |
| no left/right load transfer | could overestimate corner grip | E1: −0.9% at k = 0.2 |
| rigid body, no suspension | load transfer is instant | Simulink (A6) |
| one lumped η = 0.7 | scales $E_{stored}$ directly | A7 ($S$ = 1); Simulink mapped motor |
| hard cutoff at $V_{cut}$ | ≈2 kJ lost per stop | A7: $V_{cut}$ = 1–2 m/s, < 0.3% |
| constant $C_{rr}$ (no $f_s V^{2.5}$ term) | < 10% of $F_r$ below 50 kph | A7: $C_{rr}$ 0.010–0.015, < 0.5% |
| no wheel inertia ($M_{eq}$) | +2.5% effective mass | A7: mass ±5–10% |
| min-of-caps optimum ignores SOC coupling | ΔSOC ≈ 0.15% per stop | negligible |
| abrupt corner entry | sideways force jumps at entry | visible in `fig02_force_split_corner` |
| Euler–Cromer, Δt = 0.002 s | numerical error | A5: < 0.02% |
