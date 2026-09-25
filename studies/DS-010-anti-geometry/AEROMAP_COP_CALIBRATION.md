# Orion FRH/RRH CoP calibration — 2026-09-25

## Decision and model

The original map's 25 grid cells all placed the equivalent center of pressure
behind the 2027 WIP rear axle under BobLib's x-forward/z-up convention. To test
the requested 50% CoP prior, this commit changes **only** the WIP vehicle's
`aero.my_table_nm`. It retains the original downforce, drag, reference speed,
ride-height grids, and aero reference point. The map remains an **Orion
surrogate** for a WIP 2027 vehicle, not a measured 2027 aero balance.

Here CoP percentage means **front downforce fraction**: 0% at the rear axle,
100% at the front axle. The target is 50% at the center stored cell, FRH 2.8 in
and RRH 3.3 in, which is backed by an accepted CFD row. This is a calibration
prior supplied by the request, not a CoP measurement. The center cell is a
grid anchor; it is not asserted to be the vehicle's static ride height.

For each stored cell, the original front fraction is

`b_old = (x_ref + My_old/Fz - x_rear) / wheelbase`, where `Fz` is positive
downforce. Preserve its migration shape with the affine map

`b_new = 0.50 + g * (b_old - b_old_center)`.

The common gain `g = 0.389010` is the largest value at or below 1 that leaves
every one of the 25 stored cells between 5% and 95% front fraction. The 5%
guardrail is a modeling choice that prevents a CoP exactly on an axle; it is
not a CFD confidence bound. The most restrictive cell, FRH 1.4 in / RRH 4.95
in, is an *inferred fill* without converged CFD. The model moment is rebuilt as

`My_new = Fz * (x_rear + wheelbase * b_new - x_ref)`.

This preserves the direction and ratios of the original CoP migrations while
reducing their amplitude. It does not preserve the source pitch moments. The
original full vehicle is retained as `orion_aeromap_precalibration_vehicle.yml`
so both the source reconciliation and the calibration are reproducible.

## Sanity check results

At 15 m/s reference speed, the unchanged accepted CFD-backed cells carry
roughly 220–351 N downforce and 156–180 N drag. The check matched all 21
accepted source rows to the **precalibration** force and moment table. The
original front fraction ranged from -119% to -0.3% across all 25 cells. The
calibrated table ranges from 5.0% to 51.2%, with the center cell at 50.0%.
Among accepted CFD cells, the new range is 26.4% to 51.2%. No grid cell lies
outside the wheelbase. A sweep of 1,936 bilinear cell-interior samples likewise
stays within 5.0% to 51.2%. All drag/downforce entries remain finite and
positive; the grid stays increasing. See `orion_cop_calibration.png` for the
before/after sweep and `orion_cop_sweep.csv` for every cell and source flag.

This **passes the static map/frame sanity check** that failed in the earlier
audit. It does not establish the physical CFD moment origin, half-car scale,
speed/area/density, 2027 aero package, or 2027 rear ride-height datum. Four
cells remain inferred, including two rejected CFD cases. BobLib clamps inputs
outside the stored grid. No Modelica dynamics, frame clearance, or correlation
against measured aero balance was run. Do not treat the calibrated 50% balance
as measured or use it alone to choose anti geometry.

## Reproduce

From the `lhre-simulation` repository root, run with the existing BobSim Docker
image. This standalone cross-repository checker has no suitable Make target.

```powershell
docker run --rm --network none -v "${PWD}/studies/DS-010-anti-geometry:/study" -w /study bobdyn/bobsim:latest python check_orion_aeromap.py
docker run --rm --network none -v "${PWD}/studies/DS-010-anti-geometry:/study" -w /study bobdyn/bobsim:latest python calibrate_orion_cop.py
```

The second command deterministically regenerates the WIP vehicle moment
table, CSV, and PNG from the checked-in precalibration vehicle and 21-row
source CSV. Review `git diff` after rerunning; source force tables must not
change. The first command reproduces the historical failure, not a failed
calibration.
