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
EQUATIONS.md           all equations (Bayar et al. 2012 sheet + our analysis), variables defined
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

Equations: Bayar et al. (2012), Stages 1-4 (see `EQUATIONS.md`). Figure of merit:
η_regen = E_stored / E_B, with lumped wheel-to-battery efficiency η = 0.7.

- **Straight stop from 30 m/s at 3 m/s²:** the optimal split stores 400 kJ (η_regen 51%). The motor *power* cap binds above 13.5 m/s; grip and battery never bind.
- **90° corner (R = 40 m):** 176 kJ (η_regen 63%). The rear axle reaches 100% of its grip but stays just inside the friction circle. At **R = 35 m**, grip binds for 0.5 s (173 kJ), and grip-blind 100% regen slides the rear tires.
- **Energy–stability trade-off:** the I-curve stores 36% less on the straight stop (257 kJ) and 47% less in the 35 m corner (92 kJ).
- **Constant vs time-varying split:** in the 35 m corner, the best constant split that never slides stores 117 kJ, vs 173 kJ for the optimal split.
- **Battery:** the 4th cap only binds above SOC ≈ 0.93. At SOC 0.96 only 270 kJ is stored.
- **Braking intensity:** stored energy peaks near 1.7 m/s² (512 kJ). Theory: 1.71 m/s².
- **Validation:** the analytic solution matches to 0.007%. Time-step error is below 0.02% at the Δt we use.

## Parameters (cite sources in the report)

From the Bayar sheet: C_rr ≈ 0.012, lumped η ≈ 0.7 (Bayar ≈79% recovery on FUDS; Sun et al. 2018),
V_cut 1-2 m/s, P_chg,max constant to 90% SOC. The rest are approximately a rear-drive Tesla Model 3,
because the paper does not give them. **Each needs a source** in the report: mass, l_f, l_r, h,
Cd·A, tire radius, gear ratio, T_max, P_max, P_chg,max, E_cap, μ (dry asphalt).

## Who owns what

| | owner | code | equations |
|---|---|---|---|
| motor / battery caps, energy bookkeeping, analytic theory | Alex | `regen_core` §2, `studies.analytic_check`, `limit_envelope`, `soc_study` | Stage 1-2, bookkeeping, A1-A3 |
| load transfer, I-curve, grip, cornering | Aiden | `regen_core` §3, `axle_and_decel_sweep`, `load_transfer_study` | Stage 3-4, A2, E1 |
| core simulation, convergence, Simulink | Devon | `regen_core.simulate`, `convergence_study`, `simulink/` | Stage 1, A5, A6 |
| strategies, profile optimization, sensitivity | all | `strategy_tables`, `profile_optimization`, `sensitivity_study` | Stage 2-3, A4, A7 |
