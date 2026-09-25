# Orion FRH/RRH aero map audit — 2026-09-24

> Historical audit of `orion_aeromap_precalibration_vehicle.yml`. The current
> study vehicle's provisional moment calibration and rerun are documented in
> [AEROMAP_COP_CALIBRATION.md](AEROMAP_COP_CALIBRATION.md).

## Source and scope

The [Orion `aeromap_frh_rrh` SharePoint folder](https://utexas.sharepoint.com/:f:/r/sites/ENGR-LonghornRacing/LHR%20Electric/Design/%5BARCHIVE%5D%20Past%20Cars/Orion%202025-2026/_AER/Analysis/CFD%20Files/2026/Full%20Car%20Sims/aeromap_frh_rrh?d=webb79f21cedd4b8f8a31ad9ef16e6d58&csf=1&web=1&e=BHy6Cl) contains `master_report.pdf`, generated 17 April 2026, plus the underlying run outputs. The PDF lists 23 processed cases and accepts 21 under a 5% relative standard-deviation criterion over the final 20 iterations. `orion_2026_aeromap_converged.csv` transcribes only its accepted result table. This audit compares the source table with the aero block of `vehicle_wip_2027_frontv19_rearv35.yml` on `abatra/anti-studies` at `2a844e6`.

The numerical Orion map **was already in this vehicle definition** before this audit. The audit adds provenance and a reproducible check rather than replacing an identical table.

## Conversion check

At every one of the 21 reported points, the existing BobSim tables satisfy:

- `drag_table_n = 2 * reported Fx`
- `downforce_table_n = 2 * reported Fz`
- `my_table_nm = 2 * (reported My - reported Fz * 1 m)`; `aero_ref_m[0] = 1 m`

The maximum numerical residual is below `1.2e-13` in the stored precision. Front grid 1.4–4.2 in and rear grid 1.65–4.95 in exactly match the report's coordinates after inch-to-metre conversion. The stored tables use 15 m/s as reference speed. The reported force/coefficient ratio is consistent with dynamic pressure at 15 m/s, air density 1.225 kg/m³, and a 1 m² coefficient reference area, but the PDF does not state the CFD reference speed, area, density, coordinate origin, or half-car symmetry convention. Those remain assumptions, not validated source metadata.

Four of the 25 stored cells have **no accepted CFD row**: FRH/RRH (in) `(1.4, 4.95)`, `(2.1, 2.5)`, `(4.2, 2.5)`, and `(4.2, 3.3)`. The first two were run but rejected for nonconvergence; the last two were not among the report's 23 cases. The existing values in these cells are inferred fills. BobLib's `Bilinear2D` clamps outside the rectangular grid, so missing/bad cells affect interpolation and the boundary is not an independently validated extrapolation.

## Existing-frame sweep

`check_orion_aeromap.py` audited all 25 stored grid points with the WIP vehicle's wheelbase of 1.5494 m. The direct CFD-backed rows produce 220–351 N downforce and 156–180 N drag in the stored full-car map at its assumed 15 m/s reference. All 25 entries are positive and finite; the grid is strictly increasing and table dimensions match. This establishes data-shape validity, not physical validity.

Under BobLib's x-forward, z-up convention, the pitch moment about the front axle is `My_front = my_table_nm + (aero_ref_x - front_axle_x) * downforce`. The implied front downforce fraction `(x_CoP - x_rear) / wheelbase` is outside `[0,1]` for **all 25 grid points**. For the 21 CFD-backed points it ranges from about `-0.64` to `-0.003`; the stored fills extend to about `-1.19`. Negative means the equivalent CoP lies behind the rear axle. Every bilinear interpolation within this grid also has negative `My_front`, since all cell-corner values do. This reproduces the concern already noted in the DS-010 README and fails the frame sanity check. It does not prove the physical car has that balance; the CFD moment origin/sign and model transform need confirmation.

The WIP aero ride-height references still come from Orion. Relative to the WIP wheel centers, their Z coordinates differ by `-0.032 mm` front and `-1.190 mm` rear. The rear difference coincides with the unresolved rear SHARK vertical-datum discrepancy in the paired `.datum.json`. Actual 2027 frame hardpoints, undertray surfaces, and CFD ride-height measurement locations were not verified against this Orion reference. This is a surrogate map, not a validated map for the 2027 package or its frame clearance.

## Decision gate

Keep this map available as a labeled Orion surrogate for range and sensitivity exploration only. Do not use its pitch moment, aero balance, or filled cells to select anti geometry. Aero should confirm the CFD axes, moment origin, force signs, speed, reference area and density, whether the report is a half-car calculation, and the two rejected cases. Dynamics should resolve the rear vertical datum and the 2027 frame ride-height reference points. Then regenerate the map in the BobSim frame and rerun a dynamic ride-height sweep over the achieved vehicle envelope; the present audit is a static table/frame sweep, not a Modelica vehicle run.

Run the audit in the existing BobSim Docker image (the repository has no Make target for this standalone cross-repo audit):

```bash
docker run --rm --network none -v <this-study-directory>:/study:ro bobdyn/bobsim:latest python /study/check_orion_aeromap.py
```
