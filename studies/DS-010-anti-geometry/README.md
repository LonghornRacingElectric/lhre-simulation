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

The Orion FRH/RRH table is already present in the WIP vehicle YAML. Its
[source comparison and existing-frame sweep](AEROMAP_VALIDATION.md) match the
21 accepted CFD rows but fail the pitch-moment/aero-balance frame check. Four
table cells lack a converged source. Treat the map as an unvalidated surrogate
until Aero confirms the CFD reference convention and Dynamics resolves the
2027 ride-height datum.

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

- The `my_table_nm` / `aero_ref_m` convention in the current CFD map yields an
  implausible aero balance (CoP behind the rear axle over most of the grid).
  The surrogate sidesteps this, but it must be resolved before validation.
- Roll has no aero effect in the current map (axle-averaged ride height, zero
  `mx`/`mz`), so roll-center / roll-gradient work is out of scope here.
