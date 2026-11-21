# Optimizing Regenerative Braking of Electric Vehicles

Alex Wang, Devon Parikh, Aiden Knapp. Physics project, Matter & Interactions.

**Question:** how should braking be split between regen and friction brakes to store the
most energy, and how does that split change between straight-line braking and a corner?

## Repo layout

```
regen_core.py          physics engine: car, limits, grip, strategies, scenarios, simulate()
regen_braking.py       VPython 3D animation + live graphs (Milestones 2-3)
studies.py             every study / sweep / validation  -> data/*.csv, data/results_summary.md
make_figures.py        poster/report figures from data/   -> figures/*.png
EQUATIONS.md           all equations, with variables defined (read this first)
simulink/              Milestone 4: export params + drive cycles, compare results
notebooks/analysis.ipynb   runs the pipeline and shows the figures
tests/test_physics.py  automatic checks of every physics claim
data/  figures/        generated outputs (committed so everyone has them)
```

## Setup (once)

```bash
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements.txt
nbstripout --install                 # keeps notebook outputs out of git
```

## Everyday use

```bash
python studies.py          # ~20 s: re-run every study, rewrite data/
python make_figures.py     # rebuild every figure in figures/
python -m pytest -q        # all physics checks should pass
python regen_braking.py    # 3D animation (opens a browser tab)
jupyter lab                # or open notebooks/analysis.ipynb
```

**Rule:** change parameters only in `regen_core.Car` / the scenario functions. Then rerun
`studies.py` → `make_figures.py` → `simulink/export_for_simulink.py`, so every output and
the Simulink model use the same numbers.

In a notebook, VPython scenes sometimes freeze when a cell is re-run. Restart the kernel,
or create `scene = canvas()` first.

## Headline results (from `data/results_summary.md`)

- **Straight stop from 30 m/s at 3 m/s²:** optimal split stores 491 kJ (73% of brake work by regen). Motor *power* is the binding limit above 13.5 m/s. Grip and battery never bind.
- **90° corner (R = 40 m):** 202 kJ (84% regen). **Tire grip** becomes a binding limit (1.1 s), because sideways force uses up rear grip. Grip-blind 100% regen briefly exceeds rear grip, which makes the car unstable.
- **Corner split must vary in time:** the best *constant* split that is as safe as optimal stores 108 kJ, vs 202 kJ for the time-varying optimum.
- **Battery:** the 4th limit only binds above SOC ≈ 0.83. At SOC 0.95 only 176 kJ is stored.
- **Braking intensity:** stored energy peaks at about 1.7 m/s² (634 kJ). Theory: 1.68 m/s² (EQUATIONS.md §8.4).
- **Validation:** analytic solution matches to 0.01%. Time-step error is below 0.02% at the Δt we use.

## Parameters (cite sources in the report)

The defaults are approximately a rear-drive Tesla Model 3. **Each needs a source** in the report:
mass, wheelbase, CG position/height, Cd·A, C_rr, tire radius, gear ratio, motor torque and
power limits, efficiencies, pack size and charge-power limits, μ (dry asphalt).

## Who owns what

| | owner | code | equations |
|---|---|---|---|
| motor / battery / energy limits, analytic theory | Alex | `regen_core` §2, `studies.analytic_check`, `limit_envelope`, `soc_study` | EQ 2, 7, 8 |
| weight transfer, grip, cornering, load transfer | Aiden | `regen_core` §3, `axle_and_decel_sweep`, `load_transfer_study` | EQ 3, 5, 8.3 |
| core simulation, convergence, Simulink | Devon | `regen_core.simulate`, `convergence_study`, `simulink/` | EQ 1, 6, 10 |
| strategies, profile optimization, sensitivity | all | `strategy_tables`, `profile_optimization`, `sensitivity_study` | EQ 4, 9, 11 |
