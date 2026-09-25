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

The active `study.yml` vehicle is the front V19 / rear V35 SHARK-derived WIP
definition. Run `verify_shark_vehicle.py` with the retained SHARK exports before
using it. Its 28 represented suspension, steering, and actuation pickup points
must match the two exports within 0.001 mm. The anti-roll-bar pickups and rates
are Orion carryovers because SHARK does not define them. The existing
`calibrate_orion_cop.py` regenerates the provisional aero moment rescaling for
this same WIP vehicle; its before/after plot and per-cell data are
`orion_cop_calibration.png` and `orion_cop_sweep.csv`.

From this study directory, run
`python verify_shark_vehicle.py <front-v19.shk> <rear-v35.shk>` in the BobSim
Docker image (with the two SHARK files mounted). The checker fails on a changed
source hash, wrong active manifest, wrong architecture, or a point mismatch.

1. **Response -> grip.** Independent variables are front and rear ride-height
   change per g (`dz_f/g`, `dz_r/g` in braking; `dz_r/g` in RWD acceleration).
   Aero enters through a parametric quadratic surrogate of ClA and balance
   about static ride height, not a specific CFD map. The output is a design
   chart of optimal response versus aero sensitivity; a new CFD map is reduced
   to its surrogate coefficients and read off the chart.
2. **Response -> hardpoints.** With springs and tires frozen, target response
   inverts in closed form to anti percentage:

   ```
   AD_f = 1 - K_w,f * (dz_f / dW - 1 / K_t,f)
   AL_r = 1 - K_w,r * (dz_r / dW - 1 / K_t,r)
   dW   = m * g * a * h / L
   ```

   Anti percentage sets side-view IC angle; inboard pickup points are solved at
   constant side-view swing-arm length by rotating each pivot axis about the
   wheel-center station, which leaves front-view geometry unchanged to first
   order.

## Objective

Available grip is the expectation of `sum(mu(Fz) * Fz)` over the contact-patch
load distribution, so aero gain and CPLV loss share units and need no
arbitrary weighting. Scenarios are weighted by lap-time fraction in speed x g
bins. Jacking, kickback, bump steer, and packaging are constraints, not terms
in the objective.

## Held Fixed

- Springs, dampers, anti-roll bars, tires
- CG, wheelbase, brake bias, regen split
- Static ride height (enters the surrogate as distance to aero peak)
- Side-view swing-arm length

## Sequence

1. Anti-squat (RWD acceleration, rear only). Fix rear geometry.
2. Anti-dive with rear anti-lift from step 1; sensitivity at 2-3 rear values.
3. Validate chosen points and sweep extremes in BobSim Modelica with the real
   CFD map and nonlinear kinematics (FourPostEval reports achieved anti %).

## Engine

BobSim reduced-order `dyn_py` `VehicleModel14DOF`, linearized about the
`qss.solve_acceleration_trim` operating point for frequency-domain CPLV.
Hardpoint inversion uses the `kin_py` side-view IC solver.

Required `dyn_py` extensions before results are meaningful:

- Ride-height-dependent aero hook in `_aero_load` (currently constant ClA/CoP).
- Separate brake and drive longitudinal jacking coefficients; drive torque
  through halfshafts reacts at the wheel center, not the contact patch.
- Linearize-about-trim utility (finite-difference Jacobian of `derivative`).

## Known Issues

- The original `my_table_nm` / `aero_ref_m` convention yielded CoP behind the
  rear axle throughout the grid. The calibrated map passes a static frame
  sanity check by construction; CFD origin and dynamic vehicle validation
  remain unresolved.
- The rear SHARK vertical datum is unresolved. The ride-height reference points
  and aero force tables are Orion carryovers, not a verified 2027 aero package.
- Roll has no aero effect in the current map (axle-averaged ride height, zero
  `mx`/`mz`), so roll-center / roll-gradient work is out of scope here.
