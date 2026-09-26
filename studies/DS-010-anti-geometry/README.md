# DS-010 Anti-Dive / Anti-Squat Geometry

This study selects target anti-dive and anti-squat by trading the aero benefit
of holding ride height under longitudinal load against the contact-patch load
variation (CPLV) cost of reacting that load through the rigid control arms.

## Question

What ride-height response per g under braking and acceleration maximizes
available grip, and which control-arm inboard hardpoints realize it?

## Approach

The aero map is still changing, so the study is split in two layers joined at
the ride-height response.

The Orion FRH/RRH force table is already present in the WIP vehicle YAML. Its
[source comparison and existing-frame sweep](AEROMAP_VALIDATION.md) matched the
21 accepted CFD rows but failed the pitch-moment/aero-balance frame check.
The [provisional CoP calibration](AEROMAP_COP_CALIBRATION.md) recenters the
middle grid point at 50% front downforce and scales the original CoP migrations
to keep the entire stored grid inside the wheelbase. Four cells still lack a
converged source. Treat this as an assumption-driven surrogate until Aero
confirms the CFD reference convention and Dynamics resolves the 2027 datum.

The active `study.yml` vehicle is `vehicles/design/vehicle.yml`, the front V19 /
rear V35 SHARK-derived WIP definition. Its paired `vehicle.datum.json` records
the unresolved rear vertical datum. The study-local WIP YAML and datum sidecar
are matching provenance copies. Run `verify_shark_vehicle.py` with the retained SHARK exports before
using it. Its 28 represented suspension, steering, and actuation pickup points
must match the two exports within 0.001 mm. The anti-roll-bar pickups and rates
are Orion carryovers because SHARK does not define them. The existing
`calibrate_orion_cop.py` regenerates the provisional aero moment rescaling for
the design YAML and study copy; its before/after plot and per-cell data are
`orion_cop_calibration.png` and `orion_cop_sweep.csv`.

From this study directory, run
`python verify_shark_vehicle.py <front-v19.shk> <rear-v35.shk>` in the BobSim
Docker image (with the two SHARK files mounted). The checker fails on a changed
source hash, wrong active manifest, wrong architecture, or a point mismatch.

1. **Response -> grip.** Independent variables are front and rear ride-height
   change per g (`dz_f/g`, `dz_r/g` in braking; `dz_r/g` in RWD acceleration).
   Aero comes from the map in the vehicle YAML: for each speed x a_x bin, ride
   heights are solved from the response and the aero load, and the map gives
   downforce and balance at those heights. A new CFD map is a rerun.
2. **Response -> hardpoints.** With axle wheel rates `K = 2 k_w` and tire rates
   `K_t = 2 k_t`, target response inverts in closed form to anti percentage:

   ```
   braking: dz_f = -dW * [ (1 - AD_f) / K_f + 1 / K_t,f ]
   RWD:     dz_r = -dW * [ (1 - AS_r) / K_r + 1 / K_t,r ]
   dW       = m * g * |a_x| * h / l
   ```

   Anti percentage sets the side-view slope c from the contact patch to the
   instant centre (IC). Drive torque goes through the halfshafts, so it reacts
   at the wheel centre:

   ```
   AD_f = beta_f * (l/h) * c_f
   AS_r = (l/h) * (c_r - R_r / d_r)
   ```

   Inboard pickup points are solved at constant side-view swing-arm length d
   by rotating each pivot axis about the wheel-centre station, which leaves
   front-view geometry unchanged to first order.

   Anti geometry makes the wheel move fore-aft in heave. The toe links run
   fore-aft (0.25 m in x at the rear), so that motion becomes bump steer:
   -227 deg/m at 90 % anti-squat with the toe-link pickup left in place.
   `run.py` therefore re-heights the toe-link/rack inner pickup so the static
   toe rate stays at baseline. A first-order corner model gives the height in
   closed form; it matches the rig within 2 %. The move counts toward the
   pickup-shift limit, and the rig's toe gain is still checked against its
   limit.

   The design vehicle's wishbones are effectively parallel in side view: the
   front IC is about 1,160 km behind the axle and the rear about 270 m ahead
   of it. `run.py` therefore holds 1/d constant and works with the IC in
   homogeneous coordinates. Parallel arms need no special case, and R/d
   becomes R * (1/d), which is close to zero for this car.

## Objective

Available grip is `E[sum(mu_x(Fz) * Fz)]` over the tires carrying Fx, with
mu_x from the tire file's PDX1/PDX2 load sensitivity. mu_x * Fz is quadratic in
Fz, so the expectation is exact in closed form. Aero gain and load-variation
loss share units and need no weighting.

The load variation is a stand-in for contact-patch load variation (CPLV). The
rigid-link path turns a longitudinal force variation into a vertical one,
`sigma_Fz = |c_eff| * sigma_Fx`:

- `sigma_Fx` is a placeholder fraction (study.yml `sigma_fx_fraction`) of the
  mean Fx per wheel.
- `c_eff = c` for contact-patch forces (braking friction).
- `c_eff = c - R/d` for wheel-centre forces (drive, regen).

Scenarios are weighted over speed x a_x bins. Toe gain and caster change in
heave, anti percentage across heave, and pickup shift are constraints, not
terms in the objective. Kickback isn't checked because the rig applies no
steer.

## Held Fixed

- Ride frequency (3 Hz front and rear); spring rates follow from each
  variant's motion ratio
- Dampers, anti-roll bars, tires
- CG, wheelbase, brake bias, regen split
- Static ride height (the aero grid anchor)
- Side-view swing-arm length (as 1/d)

## Sequence

1. **Anti-squat** (RWD acceleration). Sweep rear anti-squat on the baseline
   front and choose the best feasible rear.
2. **Anti-dive** (braking) at the chosen rear. Sweep front anti-dive the same
   way. Each front variant is re-scored with the chosen rear's sweep
   neighbours as a rear sensitivity, using the stage-1 rig results.
3. **Validation.** Rig runs of the chosen car and the four sweep corners,
   comparing achieved values against the stage-1/2 predictions.

## Engine

BobSim FourPostEval (`BobSim/_3_StandardSim/FourPostEval/four_post_eval_sim.py`),
with one `BobLib.Standards.FourPostSim` build per hardpoint variant. The rig
supplies each variant's achieved side-view slope (jacking over Fx at each heave
pose), motion ratios, and toe and caster over heave. `run.py` does the scoring
in quasi-static Python.

## How to Run

The pinned BobSim (c45940e) needs Modelica 3.2.3. `bobdyn/bobsim:latest` was
built from a newer BobSim and only has Modelica 4.1.0. Build the image once
from the submodule:

```
make init
docker build -t bobdyn/bobsim:c45940e BobSim
```

Then, from the repo root:

```
docker run --rm -v "$PWD":/workspace -w /workspace bobdyn/bobsim:c45940e \
  python studies/DS-010-anti-geometry/run.py
```

Options:

- `--stage anti-squat|anti-dive|validation` runs one stage. Later stages read
  the earlier stages' summaries from `outputs/`.
- `--reuse` keeps cached builds and rig results in `work/` for variants that
  haven't changed.
- `--jobs N` sets how many variants are built and run in parallel (default 3).
  Each build takes about 100 s and peaks at about 1.3 GB of memory.
- `--vehicle YML` overrides the study.yml vehicle.

The container runs as root because the image installs Modelica under `/root`,
so the files it writes are owned by root. `chown` `work/`, `outputs/`,
`plots/`, `RESULTS.md` and the report afterwards if needed.

Unit tests need no OpenModelica:

```
docker run --rm -v "$PWD":/workspace -w /workspace bobdyn/bobsim:c45940e \
  python -m pytest -p no:cacheprovider studies/DS-010-anti-geometry/test_run.py
```

## Known Issues

- The original `my_table_nm` / `aero_ref_m` convention yielded CoP behind the
  rear axle throughout the grid. The calibrated map passes a static frame
  sanity check by construction; CFD origin and dynamic vehicle validation
  remain unresolved.
- The rear SHARK vertical datum is unresolved. The ride-height reference points
  and aero force tables are Orion carryovers, not a verified 2027 aero package.
- Roll has no aero effect in the current map (axle-averaged ride height, zero
  `mx`/`mz`), so roll-center / roll-gradient work is out of scope here.
