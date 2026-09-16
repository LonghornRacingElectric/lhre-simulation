# Rear toe response to 42 N m aligning moment per wheel

The user confirmed **42 N m per rear wheel**. Apply a pure road-normal
aligning moment Mz, with no incremental Fx/Fy/Fz or other moments.

## Result

Using the notebook's steel properties and saved rear geometry:

- Incremental wheel yaw: **0.01213769 deg per wheel** at +42 N m.
- Moment compliance: **0.000288993 deg/(N m)**.
- Equivalent rotational stiffness: **3460.30 N m/deg**.
- Toe rod: **474.96 N tension**, **0.013002 mm extension**.
- Toe-rod contribution: **0.00842428 deg (69.4%)**.
- Four wishbone legs together: **0.00371302 deg (30.6%)**.
- Pullrod contribution: **0.0000003885 deg**.

This is incremental wheel heading, not pre-existing static toe. For an
initially x-aligned wheel at positive y (left corner, x forward), +Mz produces
toe-out. Reversing Mz reverses the response. Mirroring the corner produces
the same global yaw for the same global Mz, but reverses the local toe-in/out
label. Do not automatically double the per-wheel angle into an axle toe
change: same-sign global moments produce common rear steer in this symmetric,
uncoupled model; opposite-sign moments change total toe.

## Reproduction and source

Run `python rear_toe_compliance.py` from this directory. Dependencies: numpy,
pandas, matplotlib, scipy. The script uses the original notebook's reviewed
geometry, matrix and tube-property definition cells. The original notebook
remains unchanged. Outputs are in `rear_toe_results/`.

The notebook requires `hardpoints.txt` but the attachment omitted it. The
included file was recovered from the local source
`Tools/computa-slack-bot/data/downloads/19f66c61/hardpoints.txt` in the LHRe
workspace. All 36 rear equilibrium-matrix entries match the notebook's saved
output to its printed precision (maximum difference 4.98e-9). This is matching
historical notebook geometry, not the current BobSim vehicle YAML. Input
SHA-256 hashes are recorded in results.json.

## Method and checks

The notebook solves A F + w = 0, with compression-positive member forces F.
For uniform axial link stiffness k = EA/L, extensions are dL = -F/k. Virtual
work gives compatibility dL = A^T q, where q contains translations in inches
and rotations in radians about the reference contact patch. Therefore
K = A diag(k) A^T and K q = w. The wheel's small-angle toe change is q[5].
42 N m is converted to lbf in before entering w. All six links retain the
notebook's E = 29.7 million psi and their respective listed steel areas.

Independent equilibrium/compatibility and stiffness solutions agree; the
maximum equilibrium residual is 1.14e-13 in the notebook's mixed force/moment
units. Strain-energy/work equality, moment-sign reversal, and the mirrored
corner are checked. A finite-rotation compatibility solve with the linear
member extensions gives 0.01213783 deg, a difference of 1.43e-7 deg. That
check is not a geometrically nonlinear equilibrium solution.

## Scope

This extends a rigid equilibrium solver with **axial tube compliance only**.
Inboard points, including the pullrod inner point, are fixed; the upright and
wheel are rigid. Joint-centre spans are treated as uniform tube. The pullrod
retains the notebook's idealized direct-upright attachment. Its line misses
the upper outer ball joint by 0.000449 in at this pose; this near concurrency
supports the first-order idealization but does not model arm/apex flexibility.

No bearing, spherical-joint, insert, adhesive, arm bending, chassis, upright,
rim, tire, rocker/spring, or full-axle coupling compliance is included. No
preload-dependent geometric stiffness or combined cornering loads are
included. The result is a model contribution, not measured full-wheel
compliance or a validated design limit. No carbon properties are substituted
from the separate earlier steel/carbon study.

## Rock West 45526 carbon substitution

Manufacturer page checked September 15, 2026:
https://www.rockwestcomposites.com/45526.html

Nominal OD 0.707 in, ID 0.625 in, wall 0.041 in; axial EX = 13.9 million psi.
Area = pi/4 (OD^2-ID^2) = 0.08578566 in2; axial EA = 5.30407 MN.
These are manufacturer CLT reference properties, not guaranteed test values.
Use axial EX, not transverse EY or an isotropic carbon modulus.

With the same geometry, 42 Nm per wheel and original boundary conditions:

| Replacement | Toe change per wheel | Compliance deg/Nm | Change from steel |
|---|---:|---:|---:|
| Original steel | 0.01213769 deg | 0.000288993 | baseline |
| Carbon toe rod only, steel arms/pullrod | 0.02231863 deg | 0.000531396 | +83.9% |
| Carbon arms and toe rod, steel pullrod | 0.02536462 deg | 0.000603920 | +109.0% |
| All six links carbon | 0.02536509 deg | 0.000603931 | +109.0% |

Primary interpretation: four arm legs and toe rod become 45526, pullrod retains
the notebook's steel properties. Carbon toe rod contributes 0.01860522 deg
(73.35%) and carbon arms 0.00675901 deg (26.65%). Toe-rod extension is
0.0287147 mm under the same 474.959 N tension. Forces do not change with EA
in this statically determinate fixed-geometry model. Greater extension accounts
for the increased toe; lower mass does not imply greater axial stiffness.

Reproduce with `python rear_toe_compliance.py --material carbon_arms_toe`.
Other options: `steel`, `carbon_toe_only`, `carbon_all`. Separate result folders
preserve each case. `carbon_comparison.json` collects all four runs.
All equilibrium, independent stiffness, energy, mirrored-corner and reversal
checks pass. Finite-rotation compatibility gives 0.02536531 deg for the primary
carbon case. All previously stated model limits still apply, including no
bonded-insert/joint compliance, strength signoff or full vehicle validation.
