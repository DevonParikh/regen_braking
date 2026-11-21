# Milestone 4: Simulink cross-validation

Goal: run the **same car and the same two scenarios** in Simulink, then compare against the
VPython model. Differences should be explained by the physics Simulink includes and we don't
(EQUATIONS.md §10 and §12).

## 1. Export our parameters and scenarios (Python, repo root)

```bash
python studies.py                       # makes data/ts_*_optimal.csv
python simulink/export_for_simulink.py  # makes params_regen.m + drive_cycle_*.csv
```

## 2. Load them in MATLAB (from the `simulink/` folder)

```matlab
load_drive_cycles      % runs params_regen.m, makes cycle_straight / cycle_corner timeseries
```

## 3. Build the Simulink runs

**Recommended:** Powertrain Blockset EV reference application (straight stop), and Vehicle
Dynamics Blockset (corner). These include physics we left out (tire slip, mapped motor,
battery model), so differences test our assumptions.
1. Open the EV reference application and run it unchanged once.
2. Set its vehicle, motor and battery parameters from `veh`, `mot`, `bat` (mass, CG,
   wheelbase, Cd·A, C_rr, tire radius, gear ratio, torque/power limits, pack size, SOC0).
3. Drive Cycle Source block → Source = workspace variable `cycle_straight`.
4. Look at how its controller blends regen and friction braking, and compare with our rule
   `F_regen = min(F_req, F_T, F_P, F_B, F_grip)`.
5. For the corner, use `cycle_corner` for speed and `path_corner` (or R = `sc2.R`) for the
   path in Vehicle Dynamics Blockset.

**Fallback** (no blocksets): rebuild our equations (EQUATIONS.md §1–3) with plain Simulink
blocks and integrate with ode45. This checks our numerics only, not our assumptions. Say so
in the report.

## 4. Export the results and compare

Log at least `speed`. Also log `F_regen`, `E_stored` (J), and if possible `SOC`, `N_f`,
`N_r`, `F_lat`. Then:

```matlab
out = sim('your_model');
export_results('results_straight.csv', out.speed.Time, out.speed.Data, ...
               'F_regen', out.F_regen.Data, 'E_stored', out.E_stored.Data);
```

Back in Python (repo root):

```bash
python simulink/compare_simulink.py   # -> data/simulink_comparison.csv, figures/simulink_overlay_*.png
```

Report the % difference of final stored energy and the NRMSE of each signal, and explain
each difference.
