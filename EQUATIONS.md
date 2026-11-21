# Equations: Optimizing Regenerative Braking of Electric Vehicles

Alex Wang, Devon Parikh, Aiden Knapp

Sections 1–4 summarize the equations already in the model (Milestones 1–3) so that all
the notation is in one place. **Sections 5–12 are new**: they cover the validation,
theory and analysis behind Milestones 4–5. Each section names the code that uses it
(`regen_core.py`, `studies.py`) and the data file it produces (`data/…`).
The numbers in the examples use the default car (`regen_core.Car`).

## 0. Notation

| symbol | meaning | code | default |
|---|---|---|---|
| $m$ | mass (with driver) | `car.m` | 1850 kg |
| $L$ | wheelbase | `car.L` | 2.875 m |
| $\ell_f,\ \ell_r$ | CG to front / rear axle ($\ell_f+\ell_r=L$) | `car.a`, `car.b` | 1.50, 1.375 m |
| $h$ | CG height | `car.h` | 0.50 m |
| $t_w$ | track width (left–right tire spacing) | `car.track` | 1.58 m |
| $\mu$ | road friction coefficient | `car.mu` | 0.90 |
| $\lambda$ | grip margin used by regen | `GRIP_MARGIN` | 0.85 |
| $C_d A$ | drag coefficient × frontal area | `car.Cd*car.A` | 0.23 × 2.22 m² |
| $C_{rr}$ | rolling-resistance coefficient | `car.Crr` | 0.010 |
| $r_w,\ G$ | tire radius, gear ratio | `car.r_w`, `car.G` | 0.34 m, 9 |
| $T_{max},\ P_{max}$ | motor regen torque / power limits | `car.T_motor_max`, `car.P_motor_max` | 420 N·m, 75 kW |
| $\eta_m,\ \eta_b$ | motor+inverter, battery-charging efficiency | `car.eta_motor`, `car.eta_batt` | 0.90, 0.95 |
| $v_f$ | regen fade speed | `car.v_fade` | 1.5 m/s |
| $E_{pack},\ P_b,\ S_t$ | pack energy, max charge power, taper SOC | `car.E_batt`, `car.P_batt_max`, `car.SOC_taper` | 75 kWh, 80 kW, 0.80 |
| $a$ | commanded deceleration magnitude | `a_cmd` | 3 m/s² |
| $v,\ s$ | speed, distance along path | | |
| $F_{req}$ | brake force the brakes must supply | `F_req` | |
| $F_{rg},\ F_{fr}$ | regen force, friction-brake force | `F_regen`, `F_fric` | |
| $N_f,\ N_r$ | front / rear axle normal force | `N` | |
| $F_y$ | sideways (centripetal) tire force | `F_lat` | |

---

## 1. Momentum principle and forces (Milestones 1–2, recap)

Each time step $\Delta t$ (Euler–Cromer):

$$\vec p_{n+1} = \vec p_n + \vec F_{net}\,\Delta t,\qquad \vec r_{n+1} = \vec r_n + \frac{\vec p_{n+1}}{m}\Delta t$$

$$\vec F_{net} = \vec F_{drag} + \vec F_{roll} + \vec F_{rg} + \vec F_{fr} + \vec F_{drive} + \vec F_{y}$$

$$\vec F_{drag} = -\tfrac12\rho C_d A\, v^2\,\hat v,\qquad \vec F_{roll} = -C_{rr}\, m g\,\hat v$$

The drive cycle commands a deceleration $a$. Drag and rolling resistance supply part of
it, and the brakes supply the rest:

$$F_{req} = \max\!\big(0,\ m a - |F_{drag}| - |F_{roll}|\big),\qquad F_{fr} = F_{req} - F_{rg}$$

In the corner, the sideways force keeps the car on the midline (radius $R$). It includes a
small spring–damper "driver" correction on the radial error:

$$F_y = \frac{m v^2}{r} + k\,(r-R) + c\,v_r$$

## 2. The four regen limits (Milestone 1, recap)

Every limit is written as a force at the tire contact patch:

$$F_T = \frac{T_{max} G}{r_w}\cdot\min\!\left(1,\frac{v}{v_f}\right),\qquad
F_P = \frac{P_{max}}{v},\qquad
F_B = \frac{P_{batt}(SOC)}{\eta_m\, v},\qquad
F_{grip} = \sqrt{(\lambda\mu N_{reg})^2 - F_{y,reg}^2}$$

$$P_{batt}(SOC) = \begin{cases} P_b & SOC \le S_t \\ P_b\,\dfrac{1-SOC}{1-S_t} & S_t < SOC < 1\end{cases}$$

Energy chain: mechanical → electrical → chemical.

$$dE_{regen} = F_{rg}\, v\, dt,\qquad dE_{stored} = \eta_m\eta_b\, dE_{regen},\qquad
dSOC = \frac{dE_{stored}}{E_{pack}}$$

## 3. Weight transfer and the friction circle (Milestone 3, recap)

Torque balance about the CG during braking moves load forward:

$$N_f = \frac{m g\,\ell_r}{L} + \frac{m a h}{L},\qquad N_r = \frac{m g\,\ell_f}{L} - \frac{m a h}{L}$$

In steady cornering (no yaw acceleration), the sideways force splits so that it has zero
torque about the CG:

$$F_{y,f} = F_y\frac{\ell_r}{L},\qquad F_{y,r} = F_y\frac{\ell_f}{L}$$

Friction circle on each axle (braking and cornering share the grip):

$$F_{x,i}^2 + F_{y,i}^2 \le (\mu N_i)^2 \quad\Rightarrow\quad F_{x,i}^{avail} = \sqrt{(\mu N_i)^2 - F_{y,i}^2}$$

The **grip use** of an axle is $\sqrt{F_x^2+F_y^2}/(\mu N)$. A value above 1 means the
tires slide, and on the rear axle that makes the car unstable.

## 4. The optimal split (Milestone 5)

The total stored energy is
$E_{stored} = \eta_m\eta_b\int F_{rg}(t)\,v(t)\,dt$. The drive cycle fixes $v(t)$, and
each limit depends only on the present instant. The integrand increases with $F_{rg}$, so
the total is maximized by maximizing the integrand at every instant:

$$\boxed{F_{rg}^{*}(t) = \min\big(F_{req},\ F_T,\ F_P,\ F_B,\ F_{grip}\big)}$$

The only coupling between instants is that SOC rises, which changes $F_B$. Over one stop
SOC changes by less than 0.2%, so this effect is negligible.
The **regen share** is $E_{regen}/(E_{regen}+E_{fric})$.

---

## 5. NEW: Left–right weight transfer and tire load sensitivity (Aiden)

Cornering also moves load from the inner tire to the outer tire on each axle. Taking moments
about the outer contact patch, and assuming each axle carries the transfer in proportion
to its own sideways force:

$$\Delta N_i = \frac{F_{y,i}\, h}{t_w},\qquad N_{out} = \tfrac12 N_i + \Delta N_i,\qquad N_{in} = \tfrac12 N_i - \Delta N_i$$

The inner tire lifts off if $\Delta N_i > \tfrac12 N_i$ (tracked as `t_wheel_lift`; it
never happens in our scenarios).

**With constant $\mu$, the transfer does not change the axle's grip**, because
$\mu N_{out} + \mu N_{in} = \mu N_i$. It only matters because real tires lose friction
coefficient as load rises (*load sensitivity*). We model that as

$$\mu_j = \mu\left[1 - k\left(\frac{N_j}{N_0}-1\right)\right],\qquad N_0 = \text{static load on one tire}$$

$$C_i = \mu_{out}N_{out} + \mu_{in}N_{in} = \mu\left[N_i(1+k) - \frac{k}{N_0}\left(\frac{N_i^2}{2} + 2\Delta N_i^2\right)\right]$$

The capacity $C_i$ replaces $\mu N_i$ in the friction circle. The capacity lost to the
transfer is $2\mu k\,\Delta N_i^2/N_0$, which grows with the **square** of the cornering
force.

- Code: `regen_core.axle_capacity`, parameter `car.k_load` (0 = proposal assumption).
- Study: `studies.load_transfer_study` → `data/load_transfer.csv`.
- Result: for $k$ up to 0.2, the stored energy in the R = 40 m corner drops by only 0.4%. Neglecting it was a safe assumption.

## 6. NEW: Energy-principle check and numerical convergence (Devon)

### 6.1 The ledger
Work done by each force over a step uses the average velocity
$\vec v_{avg} = \tfrac12(\vec v_n + \vec v_{n+1})$:

$$W_j = \sum_n \vec F_j\cdot\vec v_{avg}\,\Delta t,\qquad
\text{residual} = \Delta K - \Big(W_{drive} + W_y - E_{drag} - E_{roll} - E_{fric} - E_{regen}\Big)$$

### 6.2 Why the residual is exactly zero
With $\vec p_{n+1} - \vec p_n = \vec F_{net}\Delta t$,

$$\Delta K = \frac{p_{n+1}^2 - p_n^2}{2m} = \frac{(\vec p_{n+1}-\vec p_n)\cdot(\vec p_{n+1}+\vec p_n)}{2m} = \vec F_{net}\cdot\vec v_{avg}\,\Delta t$$

So the residual is zero up to rounding error for any $\Delta t$. That makes it a
**bookkeeping** check: it shows every force is counted exactly once. It does not check
accuracy, which is what 6.3 is for. (State this on the poster; it's a common
misunderstanding.)

### 6.3 Convergence
Euler–Cromer has global error $O(\Delta t)$. Run with $\Delta t,\ \Delta t/2,\ \Delta t/4,\dots$:

$$\varepsilon(\Delta t) = \frac{|E(\Delta t) - E(\Delta t_{min})|}{E(\Delta t_{min})},\qquad
\text{observed order } p = \log_2\frac{E(4\Delta t)-E(2\Delta t)}{E(2\Delta t)-E(\Delta t)}$$

- Study: `studies.convergence_study` → `data/convergence.csv`, `figures/fig10_convergence.png`.
- Result: at the $\Delta t$ we use (0.002 s), the error is 0.008% (straight) and 0.02% (corner). The error is below 0.12% for every $\Delta t$ tested. In the straight case it halves as $\Delta t$ halves ($p\approx1$). The corner case is noisier because the corner entry, the start of braking and the limit switches are discrete events that land on a time step.

## 7. NEW: Analytic solution for a straight stop (theory check, Alex)

Take a stop from $v_0$ at constant $a$ with **no drag or rolling resistance**, so
$F_{req} = ma$ and $dv = -a\,dt$. Assume the battery and grip limits don't bind. Then regen is
limited by motor power above $v^*$ and by the faded torque below $v_t$:

$$v^* = \frac{P_{max}}{m a},\qquad v_t = v_f\,\frac{m a}{F_{T0}},\qquad F_{T0} = \frac{T_{max}G}{r_w}$$

Energy into the motor, in three phases:

$$E_1 = \int P_{max}\,dt = \frac{P_{max}\,(v_0 - v^*)}{a}\qquad (v^* < v < v_0,\ \text{power-limited})$$

$$E_2 = \tfrac12 m\left(v^{*2} - v_t^2\right)\qquad (\text{all braking by regen})$$

$$E_3 = \int_{v_{stop}}^{v_t} \frac{F_{T0}\,v}{v_f}\, v\,\frac{dv}{a} = \frac{F_{T0}\left(v_t^3 - v_{stop}^3\right)}{3 v_f\, a}\qquad (\text{torque fade})$$

$$E_{stored} = \eta_m\eta_b\,(E_1+E_2+E_3),\qquad E_{fric} = \tfrac12 m (v_0^2 - v_{stop}^2) - (E_1+E_2+E_3)$$

If $v^* > v_0$, phase 1 doesn't happen. If $v_t < v_{stop}$, phase 3 doesn't happen.

- Example ($v_0$ = 30 m/s, $a$ = 3 m/s²): $v^*$ = 13.5 m/s, $v_t$ = 0.75 m/s. Theory gives 496.66 kJ and the simulation gives 496.61 kJ (**0.01% difference**).
- Code: `regen_core.analytic_straight_stop`. Study: `studies.analytic_check` → `data/analytic_check.csv`.

## 8. NEW: When each limit binds (closed-form crossovers, Alex + Aiden)

### 8.1 Power vs. full regen
Regen can supply all of the braking only below $v^* = P_{max}/F_{req}$. At 3 m/s², that's 13.5 m/s.

### 8.2 Battery
The motor can deliver at most $\eta_m P_{max}$ (67.5 kW) of electrical power. The battery
caps regen only once $P_{batt}(SOC) < \eta_m P_{max}$:

$$SOC_{crit} = 1 - (1-S_t)\,\frac{\eta_m P_{max}}{P_b} = 0.831$$

**At our default SOC₀ = 0.60, the battery limit never binds.** This is why the SOC study
exists (`data/soc_study.csv`, `fig07`): from SOC₀ = 0.86 up, stored energy falls steeply
(at 0.95 only 176 kJ is stored instead of 491 kJ).

### 8.3 Grip (straight line)
Regen on the rear axle alone can supply deceleration $a$ only while
$m a \le \lambda\mu N_r(a)$. Substituting $N_r$ from §3:

$$a_{grip,rear} = \frac{\lambda\mu g\,\ell_f}{L + \lambda\mu h} = 3.46\ \text{m/s}^2,\qquad
a_{grip,front} = \frac{\lambda\mu g\,\ell_r}{L - \lambda\mu h} = 4.14\ \text{m/s}^2$$

Weight transfer **helps front regen and hurts rear regen**, which is why front-axle regen
stores more energy in hard stops (`fig05`). The torque limit binds only above
$F_{T0}/m$ = 6.0 m/s², so in practice grip always runs out before torque does, apart from
the fade zone below $v_f$.

### 8.4 Energy-optimal braking intensity
Gentle braking stays under the power limit but takes longer, so drag and rolling
resistance take more energy. Hard braking hits the power limit. With $v^2 = v_0^2 - 2 a s$
and stopping distance $v_0^2/2a$:

$$E_{drag} = \int_0^{v_0^2/2a}\tfrac12\rho C_dA\,v^2\,ds = \frac{\rho C_d A\, v_0^4}{8a},\qquad
E_{roll} = \frac{C_{rr} m g\, v_0^2}{2a}$$

The friction brakes must supply the force above the power limit. Treat the resistive force
as its value at $v_0$, $R_0$, and write $F = ma - R_0$ and $v^* = P_{max}/F$:

$$E_{fric} \approx \int_{v^*}^{v_0}\Big(F - \frac{P_{max}}{v}\Big)\frac{v\,dv}{a} = \frac{(F v_0 - P_{max})^2}{2aF}$$

$$E_{stored}(a) \approx \eta_m\eta_b\left[\tfrac12 m v_0^2 - E_{drag}(a) - E_{roll}(a) - E_{fric}(a)\right]$$

The first two losses fall like $1/a$, and the last one rises with $a$, so there is a best
deceleration. Maximizing gives $a_{opt}$ ≈ **1.68 m/s²** from 30 m/s. The simulation's best
grid point is 1.75 m/s², at **634 kJ**. Our 3 m/s² headline stop stores 491 kJ, 23% less.
The theory curve stays within 3% of the simulation up to 3 m/s² (`fig05`). Above the
grip limit (§8.3) it overestimates, because it ignores grip.

- Code: `regen_core.analytic_decel_energy`, `analytic_optimal_decel`. Data: `data/sweep_decel.csv`.

## 9. NEW: Braking profile at a fixed stopping distance (team)

Real drivers must stop within a set distance $D$. **Shortest stop that regen can do alone**
(brake at the power limit, then at the grip/torque cap $a_c$; no drag):

$$m v\frac{dv}{ds} = -\frac{P_{max}}{v}\ \Rightarrow\ s_1 = \frac{m\,(v_0^3 - v_s^3)}{3P_{max}},\qquad
s_2 = \frac{v_s^2}{2a_c},\qquad v_s = \frac{P_{max}}{m a_c}$$

From 30 m/s, $s_1 + s_2$ = 229 m with rear regen. Any shorter stop **must** use the friction
brakes. Our 3 m/s² stop takes $D$ = 150 m.

For $D$ = 150 m, we test two-phase profiles: deceleration $a_1$ while $v > v_s$, then $a_2$
to the stop. The distance constraint fixes $a_2$:

$$\frac{v_0^2 - v_s^2}{2a_1} + \frac{v_s^2}{2a_2} = D\quad\Rightarrow\quad a_2 = \frac{v_s^2}{2\left(D - \frac{v_0^2-v_s^2}{2a_1}\right)}$$

The theory predicts that you should brake gently while the power limit binds (high speed)
and harder at low speed, where $P_{max}/v$ is large, **up to but not past $a_{grip}$**.

- Result: the best profile is $a_1$ = 2.9 m/s² above 14 m/s, then $a_2$ = 3.42 m/s². That's just under $a_{grip,rear}$ = 3.46, as predicted. It stores **501 kJ vs 491 kJ (+2.0%)** over the same distance.
- Study: `studies.profile_optimization` → `data/profile_optimization.csv`, `fig11`.

## 10. NEW: Simulink comparison metrics (Milestone 4, Devon)

Interpolate the VPython time series onto the Simulink time grid $t_k$, then compute:

$$\text{\% difference (final)} = 100\,\frac{X_{SL} - X_{VP}}{X_{VP}},\qquad
\text{NRMSE} = \frac{\sqrt{\frac1n\sum_k \left(X_{VP}(t_k) - X_{SL}(t_k)\right)^2}}{\max X_{VP} - \min X_{VP}}$$

Use these for $X$ = speed, $F_{rg}$, $F_{fr}$, $E_{stored}$, SOC, $N_f$, $N_r$ and $F_y$
(whatever Simulink logs). Then explain any difference using §12: tire slip, the motor
efficiency map, battery model, left–right transfer.

- Code: `simulink/compare_simulink.py` → `data/simulink_comparison.csv`, `figures/simulink_overlay_*.png`.

## 11. NEW: Sensitivity to assumed parameters (team)

Change one parameter $p$ at a time from its default $p_0$:

$$S_p = \frac{\Delta E_{stored}/E_{stored}}{\Delta p/p_0}$$

$S_p = 1$ means a 1% change in $p$ gives a 1% change in the result. Efficiencies have
$S = 1$ exactly, because $E_{stored} \propto \eta_m\eta_b$.

- Results (`data/sensitivity.csv`, `fig09`):
  - Straight stop: the result is set by motor power ($S$ ≈ 0.8) and the efficiencies. It is **insensitive to $\mu$, $h$ and the grip margin** unless $\mu$ drops to 0.5, because grip doesn't bind at 3 m/s².
  - Corner: the result is most sensitive to $\mu$ ($S$ ≈ 1.2), the grip margin ($S$ ≈ 1.1) and CG position. At $\mu$ = 0.5 (wet road) the corner itself is **infeasible**: the total braking + cornering demand exceeds the grip of both axles for 1.1 s, so the car could not follow the path at this speed. Report that case as a limit of the scenario, not as a −51% change in stored energy.
  - Drag coefficient has $S \approx 0$. In the power-limited phase, extra drag replaces friction braking, not regen.

## 12. Assumptions and how each is tested

| assumption | effect on the answer | how we test it |
|---|---|---|
| constant $\mu$, dry road | sets every grip limit | sensitivity μ = 0.5–1.0 (§11) |
| no tire slip dynamics (friction circle) | grip limit is a sharp cutoff | Simulink with a Magic Formula tire (§10) |
| no left–right load transfer / load sensitivity | overestimates corner grip | §5: −0.4% at k = 0.2 |
| rigid body, no suspension pitch | weight transfer is instant | Simulink (§10) |
| constant efficiencies (no motor map) | scales $E_{stored}$ directly ($S$ = 1) | sensitivity η (§11); Simulink mapped motor |
| greedy optimum ignores SOC coupling | negligible (ΔSOC < 0.2% per stop) | §4 |
| abrupt corner entry (no transition curve) | sideways force jumps at entry | visible in `fig02_force_split_corner` |
| regen fade below 1.5 m/s | small loss at the very end | sensitivity $v_f$ (§11): ≈ 0% |
| Euler–Cromer, Δt = 0.002 s | numerical error | §6: < 0.02% |
