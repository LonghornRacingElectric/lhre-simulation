# DS-006 Integrated Tire Design

Generated UTC: 2026-05-24T11:12:40+00:00

## Purpose

This study compares every Round 9 full-UM14 tire fit on the current vehicle architecture. Each tire variant updates tire radius, rim radius, rim width, vertical tire properties, and the chassis-layout z coordinates by the tire-radius delta from the reference tire. Each candidate is evaluated in EnvelopeSim and, unless explicitly skipped, StandardSim SteadyStateEval. The current hybrid tire is included as a non-candidate reference.

## Source Of Results

| Result type | Source |
| --- | --- |
| Envelope limits | `BobSim/_2_EnvelopeSim/GGV/ggv_generation.py` |
| Steady-state vehicle response | `BobSim/_3_StandardSim/SteadyStateEval/steady_state_eval_sim.py` |
| Tire fit diagnostics | `vehicles/current/tires/round_9_fitted_full_um14/manifest.csv` and diagnostics CSV |
| Transient tire temperature | `RunData_Cornering_Matlab_SI_Round9.zip` transient runs |
| Initial/final tire degradation | `RunGuide_Round9.pdf` 12 psi repeats in the cornering and drive/brake archives |
| Report generator | `studies/DS-006-integrated-tire-design/run.py` |

## Coverage

| Item | Value |
| --- | ---: |
| Round 9 candidates | 56 |
| Reference tires | 1 |
| EnvelopeSim cases | 57 |
| StandardSim successful cases | 57 |
| StandardSim errors | 0 |
| Vehicle mass | 261.07 kg |
| CG height | 0.280 m |
| Reference tire radius | 0.2032 m |
| Front static fraction | 0.483 |
| StandardSim QA-pass cases | 51 |
| StandardSim QA-fail cases | 6 |

## Architecture Adjustment

Tire OD is treated as a real vehicle architecture input, not just a tire-file swap. For each candidate, the tire `UNLOADED_RADIUS`, `RIM_RADIUS`, `RIM_WIDTH`, vertical stiffness, and vertical damping are pushed into the StandardSim vehicle record. The chassis-layout z coordinate fields are translated upward by `candidate UNLOADED_RADIUS - reference UNLOADED_RADIUS`, so the sprung/driver/unsprung mass locations, suspension points, wheel centers, and ride-height reference points all move consistently with the tire package.

EnvelopeSim does not carry the full suspension geometry, so the same architecture correction is represented by increasing the GGV vehicle CG height by the tire-radius delta before generating the candidate envelope.

| Tire size | Radius | Radius delta | Envelope CG height | CG delta |
| --- | ---: | ---: | ---: | ---: |
| 16x6.0-10 | 0.2032 m | 0.0 mm | 0.2796 m | 0.0% |
| 16x7.5-10 | 0.2032 m | 0.0 mm | 0.2796 m | 0.0% |
| 18.0x6.5-10 | 0.2286 m | 25.4 mm | 0.3050 m | 9.1% |
| 18.0x6.0-10 | 0.2286 m | 25.4 mm | 0.3050 m | 9.1% |
| 18x6.0-10 | 0.2286 m | 25.4 mm | 0.3050 m | 9.1% |
| 20.0x7.0-13 | 0.2540 m | 50.8 mm | 0.3304 m | 18.2% |
| 20.5x7.0-13 | 0.2604 m | 57.2 mm | 0.3368 m | 20.4% |

## Tire Fit Pedigree

All Round 9 candidate records resolve to generated full-UM14 PAC2002 `.tir` files and were renderable into StandardSim vehicle records. The remaining tire-fit caveats are provenance and fit quality, not missing or undefined TIR fields.

Longitudinal/combined-source counts:

- `direct_drive_brake`: `39`
- `pressure_extrapolated_lateral_direct_drive_brake`: `1`
- `scaled_18in_hoosier_long_combined`: `16`

Lateral-relaxation-source counts:

- `transient_step_steer_fit`: `41`
- `transient_step_steer_pressure_extrapolated_from:10,12`: `2`
- `transient_step_steer_pressure_extrapolated_from:10,12,14`: `13`

| Fit metric | Median | P90 | Count |
| --- | ---: | ---: | ---: |
| Lateral force NRMSE | 1.2% | 1.5% | 55 |
| Longitudinal force NRMSE | 2.3% | 3.5% | 56 |
| Combined Fx NRMSE | 3.9% | 5.4% | 56 |
| Combined Fy NRMSE | 5.6% | 6.7% | 56 |
| Lateral relaxation sigma RMSE | 0.028 m | 0.077 m | 41 |

## Scoring Method

Envelope score is a candidate-normalized weighted score: mean lateral g 35%, 25 m/s lateral g 20%, mean GGV area 25%, mean acceleration 10%, mean braking 10%.
StandardSim score is evaluated in a deliberately stable `8.0 m/s^2` ramp-steer window. It scores closeness to +0.5 deg/g understeer 40%, closeness to +1.0 deg/g roll gradient 25%, low absolute sideslip gradient 20%, and low peak handwheel torque 15%. StandardSim `ay_max` is retained only as a measured-ramp diagnostic; it is not scored and is not used as the tire limit.
Integrated design score is 65% EnvelopeSim score and 35% StandardSim score. This score is a transparent decision aid, not a replacement for reviewing the raw response metrics.
Response flags mark StandardSim outliers: absolute understeer gradient above 5 deg/g, absolute sideslip gradient above 5 deg/g, non-positive roll gradient, or absolute roll gradient above 5 deg/g. Flagged rows are still shown because they are findings, but they should be treated as stability/fit-review warnings rather than clean design wins.
StandardSim QA is a hard score gate. Any row with failed maneuver speeds, missing QA metrics, wrong metric-source velocity, or excessive fit/noise diagnostics is retained in the tables but excluded from StandardSim and integrated scoring.

## Recommendation

Integrated first choice: **Hoosier 43075 16x7.5-10 7in 8 psi** with integrated score `0.895`.

It combines EnvelopeSim rank `2` with stable-window StandardSim score `0.947`, understeer gradient `0.390 deg/g`, and roll gradient `0.934 deg/g`.

Against the current reference, the winner changes mean EnvelopeSim lateral capability by `+6.9%`, understeer gradient by `-13.1%`, peak handwheel torque by `-11.8%`, and mean GGV area by `+1.2%`.

Best clean positive-understeer stability finalist: **Hoosier 43075 16x7.5-10 7in 8 psi** with integrated rank `1`, StandardSim score `0.947`, understeer gradient `0.390 deg/g`, and roll gradient `0.934 deg/g`.

Best direct longitudinal/combined-fit tire: **Goodyear D2704 20.0x7.0-13 7in 8 psi** (`0.877`).
Best scaled-longitudinal/combined tire: **Hoosier 43075 16x7.5-10 7in 8 psi** (`0.895`).

Practical common-tire lens: **Hoosier 43075 16x7.5-10 7in 8 psi** is the best Hoosier 43075 by integrated score (`0.895`), while **Hoosier 43075 16x7.5-10 8in 14 psi** is the fastest-relaxing 43075 (`sigma_alpha = 0.212 m`).
That is the important 43075 story: it is not selected by popularity or by assuming it must win. It earns consideration only after the decision is constrained to the current zero-radius-delta 16x7.5-10 vehicle package, where it is a clean, stable candidate family with no response flags across its tested pressure/rim set.
The raw initial/final tire-data check also behaves nicely for this family: the worst 43075 lateral peak-mu change is `+0.6%`, and the worst cornering-stiffness change is `-0.5%` over the repeated 12 psi sweep.

## Architecture-Constrained Read

The tire selection must be read as a set of nested design problems. The all-tire ranking answers `what wins after the current architecture correction is applied?`; the zero-radius-delta and 16x7.5-10 tables answer `what should be selected if the current architecture is the target?`. This prevents the 43075 from being magically selected while also preventing larger tires from receiving free packaging credit.

| Decision lens | Candidate | dR | Integrated | Envelope | Std | Std ay diag [g] | US grad | Flags |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Unconstrained integrated leader | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |
| Best clean all-tire finalist | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |
| Best zero-radius-delta candidate | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |
| Best clean zero-radius-delta candidate | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |
| Best clean 16x7.5-10 package candidate | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |
| Best Hoosier 43075 candidate | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.895 | 0.868 | 0.947 | 1.658 | 0.390 | ok |

Within the clean zero-radius-delta 16x7.5-10 package, the leading candidate is **Hoosier 43075 16x7.5-10 7in 8 psi**. That is the only lens under which a 43075 selection can be claimed from this study. If a 13 in or larger-OD tire remains the preferred direction after the radius/CG correction, the honest next step is a vehicle architecture study, not a tire-only selection.

## Integrated Ranking

| Rank | Candidate | dR | Envelope rank | Envelope score | Std score | Integrated | Std ay diag [g] | US grad | Roll grad | Resp flags | Std QA | Long source |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 2 | 0.868 | 0.947 | 0.895 | 1.658 | 0.390 | 0.934 | ok | ok | scaled_18in_hoosier_long_combined |
| 2 | Goodyear D2704 20.0x7.0-13 7in 8 psi | 50.8 mm | 1 | 0.922 | 0.793 | 0.877 | 1.698 | -0.006 | 1.042 | ok | ok | direct_drive_brake |
| 3 | Hoosier 43075 16x7.5-10 8in 8 psi | 0.0 mm | 3 | 0.831 | 0.881 | 0.849 | 1.651 | 0.454 | 0.937 | ok | ok | scaled_18in_hoosier_long_combined |
| 4 | Hoosier 43100 18.0x6.0-10 7in 8 psi | 25.4 mm | 5 | 0.797 | 0.910 | 0.836 | 1.552 | 0.674 | 0.995 | ok | ok | direct_drive_brake |
| 5 | Hoosier 43100 18.0x6.0-10 6in 8 psi | 25.4 mm | 4 | 0.804 | 0.760 | 0.789 | 1.513 | 0.829 | 0.976 | ok | ok | direct_drive_brake |
| 6 | Hoosier 43070 16x6.0-10 6in 14 psi | 0.0 mm | 17 | 0.682 | 0.977 | 0.785 | 1.495 | 0.496 | 0.933 | ok | ok | scaled_18in_hoosier_long_combined |
| 7 | Hoosier 43075 16x7.5-10 7in 10 psi | 0.0 mm | 14 | 0.702 | 0.903 | 0.773 | 1.617 | 0.307 | 0.935 | ok | ok | scaled_18in_hoosier_long_combined |
| 8 | Hoosier 43100 18.0x6.0-10 7in 10 psi | 25.4 mm | 12 | 0.718 | 0.866 | 0.770 | 1.630 | 0.213 | 1.000 | ok | ok | direct_drive_brake |
| 9 | Hoosier 43070 16x6.0-10 7in 8 psi | 0.0 mm | 18 | 0.664 | 0.937 | 0.760 | 1.577 | 0.586 | 0.940 | ok | ok | scaled_18in_hoosier_long_combined |
| 10 | Hoosier 43075 16x7.5-10 8in 10 psi | 0.0 mm | 15 | 0.691 | 0.870 | 0.754 | 1.609 | 0.407 | 0.937 | ok | ok | scaled_18in_hoosier_long_combined |
| 11 | Hoosier 43070 16x6.0-10 6in 8 psi | 0.0 mm | 10 | 0.728 | 0.798 | 0.752 | 1.599 | 0.609 | 0.918 | ok | ok | scaled_18in_hoosier_long_combined |
| 12 | Goodyear D2704 20.0x7.0-13 7in 10 psi | 50.8 mm | 8 | 0.761 | 0.735 | 0.752 | 1.663 | 0.313 | 1.059 | ok | ok | direct_drive_brake |

## EnvelopeSim Findings

| Rank | Candidate | dR | CG height | Score | Mean lat | 25 m/s lat | Mean area | Mean accel | Mean brake | Fz excess |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | Goodyear D2704 20.0x7.0-13 7in 8 psi | 50.8 mm | 0.330 | 0.922 | 2.052 | 2.422 | 8.026 | 1.332 | 1.580 | 0.0% |
| 2 | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 mm | 0.280 | 0.868 | 1.919 | 2.232 | 8.297 | 1.332 | 1.753 | 10.2% |
| 3 | Hoosier 43075 16x7.5-10 8in 8 psi | 0.0 mm | 0.280 | 0.831 | 1.891 | 2.232 | 8.144 | 1.332 | 1.735 | 14.2% |
| 4 | Hoosier 43100 18.0x6.0-10 6in 8 psi | 25.4 mm | 0.305 | 0.804 | 1.909 | 2.232 | 7.920 | 1.332 | 1.678 | 4.6% |
| 5 | Hoosier 43100 18.0x6.0-10 7in 8 psi | 25.4 mm | 0.305 | 0.797 | 1.891 | 2.232 | 7.877 | 1.332 | 1.703 | 3.1% |
| 6 | Hoosier 43100 18.0x6.0-10 6in 10 psi | 25.4 mm | 0.305 | 0.785 | 1.909 | 2.232 | 7.803 | 1.332 | 1.649 | 3.0% |
| 7 | Goodyear D0571 18.0x6.5-10 6in 8 psi | 25.4 mm | 0.305 | 0.782 | 1.900 | 2.232 | 7.825 | 1.332 | 1.652 | 0.0% |
| 8 | Goodyear D2704 20.0x7.0-13 7in 10 psi | 50.8 mm | 0.330 | 0.761 | 1.909 | 2.232 | 7.601 | 1.332 | 1.631 | 0.0% |
| 9 | Goodyear D0571 18.0x6.5-10 7in 8 psi | 25.4 mm | 0.305 | 0.737 | 1.862 | 2.185 | 7.675 | 1.332 | 1.667 | 2.0% |
| 10 | Hoosier 43070 16x6.0-10 6in 8 psi | 0.0 mm | 0.280 | 0.728 | 1.833 | 2.137 | 7.778 | 1.332 | 1.703 | 10.4% |

EnvelopeSim pressure deltas compare 14 psi against 8 psi for the same family/size/rim.

| Metric | Median delta | Largest drop | Largest rise |
| --- | ---: | --- | --- |
| Mean lateral g | -6.2% | Goodyear D2704 20.0x7.0-13 7in (-12.5%) | MRF ZTD1 18x6.0-10 7in (+0.0%) |
| Mean GGV area | -9.9% | Hoosier 43075 16x7.5-10 7in (-13.0%) | Hoosier 43070 16x6.0-10 6in (-2.3%) |
| Mean acceleration g | +0.0% | MRF ZTD1 18x6.0-10 6in (-2.0%) | Goodyear D0571 18.0x6.5-10 6in (+0.0%) |
| Mean braking g | -6.7% | Hoosier 43075 16x7.5-10 7in (-11.5%) | Goodyear D2704 20.0x7.0-13 7in (+0.5%) |

## StandardSim Findings

StandardSim uses a stable handling sweep with commanded maxAy `8.0 m/s^2`; limit capability is intentionally judged by EnvelopeSim above.

| Std rank | Candidate | Std score | Std ay diag [g] | US grad | Sideslip grad | Roll grad | Handwheel torque | Resp flags | Std QA |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | Hoosier 43070 16x6.0-10 6in 14 psi | 0.977 | 1.495 | 0.496 | 0.020 | 0.933 | 15.439 | ok | ok |
| 2 | Hoosier 43070 16x6.0-10 7in 12 psi | 0.948 | 1.527 | 0.487 | 0.119 | 0.936 | 16.490 | ok | ok |
| 3 | Hoosier 43075 16x7.5-10 7in 8 psi | 0.947 | 1.658 | 0.390 | 0.046 | 0.934 | 16.010 | ok | ok |
| 4 | Hoosier 43070 16x6.0-10 6in 12 psi | 0.940 | 1.542 | 0.408 | -0.116 | 0.936 | 15.587 | ok | ok |
| 5 | Hoosier 43070 16x6.0-10 7in 10 psi | 0.938 | 1.570 | 0.453 | 0.108 | 0.939 | 17.201 | ok | ok |
| 6 | Hoosier 43070 16x6.0-10 7in 8 psi | 0.937 | 1.577 | 0.586 | 0.113 | 0.940 | 16.313 | ok | ok |
| 7 | Hoosier 43100 18.0x6.0-10 6in 14 psi | 0.932 | 1.550 | 0.401 | 0.043 | 0.996 | 19.045 | ok | ok |
| 8 | Hoosier 43070 16x6.0-10 7in 14 psi | 0.922 | 1.484 | 0.583 | 0.189 | 0.934 | 16.260 | ok | ok |
| 9 | Hoosier 43100 18.0x6.0-10 7in 8 psi | 0.910 | 1.552 | 0.674 | -0.139 | 0.995 | 17.683 | ok | ok |
| 10 | Hoosier 43075 16x7.5-10 7in 12 psi | 0.904 | 1.583 | 0.376 | 0.204 | 0.933 | 17.180 | ok | ok |

StandardSim pressure deltas compare 14 psi against 8 psi for the same family/size/rim.
Gradient percentage deltas can be dominated by flagged outlier cases; use the row-level flags when judging stability.

| Metric | Median delta | Largest drop | Largest rise |
| --- | ---: | --- | --- |
| StandardSim score | -0.9% | MRF ZTD1 18x6.0-10 6in (-15.8%) | Hoosier 43100 18.0x6.0-10 6in (+22.6%) |
| Understeer gradient | +5.7% | Goodyear D2704 20.0x7.0-13 7in (-6751.5%) | Goodyear D0571 18.0x6.5-10 6in (+343.3%) |
| Roll gradient | +0.1% | Hoosier 43164 20.5x7.0-13 7in (-0.9%) | Goodyear D0571 18.0x6.5-10 7in (+10.7%) |
| Peak handwheel torque | +8.0% | Hoosier 43164 20.5x7.0-13 7in (-5.9%) | Hoosier 43100 18.0x6.0-10 6in (+32.9%) |

### StandardSim QA Gate

The following rows produced StandardSim metrics but were excluded from integrated scoring because the steady-state evidence failed QA. This is intentional: noisy or partial steady-state reports are findings, not design winners.

| Candidate | QA flags | Failures | Metric V | Roadwheel fit | Steer-excess fit | Mean rad err |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Goodyear D0571 18.0x6.5-10 6in 10 psi | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;sideslip_fit_nrmse_high | 0 | 15.0 | 0.168 | 0.197 | 40.298 |
| Goodyear D0571 18.0x6.5-10 6in 8 psi | steer_excess_fit_nrmse_high | 0 | 15.0 | 0.023 | 0.155 | 23.905 |
| Goodyear D0571 18.0x6.5-10 7in 8 psi | failed_maneuver;metric_velocity_mismatch | 1 | 12.5 | 0.009 | 0.045 | 16.732 |
| Hoosier 43100 18.0x6.0-10 6in 10 psi | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;roll_fit_nrmse_high;sideslip_fit_nrmse_high | 0 | 15.0 | 0.351 | 0.306 | 296.105 |
| Hoosier 43100 18.0x6.0-10 6in 12 psi | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;roll_fit_nrmse_high;sideslip_fit_nrmse_high | 0 | 15.0 | 0.232 | 0.253 | 199.908 |

## Relaxation And Response

`sigma_alpha_m` is the fitted lateral relaxation length from the Round 9 transient tire data. Lower values mean the tire builds lateral force over a shorter distance, so the car should feel more immediate for the same steady-state capability. This is a tire-fit diagnostic, not an EnvelopeSim or StandardSim response metric, so it is used here as a response/feel cross-check.

Fastest fitted relaxation lengths:

| Rank | Candidate | sigma_alpha | Integrated | Std ay diag [g] | US grad | Flags |
| ---: | --- | ---: | ---: | ---: | ---: | --- |
| 1 | Hoosier 43164 20.5x7.0-13 8in 14 psi | 0.162 | 0.504 | 1.576 | 0.499 | ok |
| 2 | Hoosier 43164 20.5x7.0-13 7in 14 psi | 0.184 | 0.516 | 1.573 | 0.503 | ok |
| 3 | Hoosier 43070 16x6.0-10 7in 14 psi | 0.186 | 0.711 | 1.484 | 0.583 | ok |
| 4 | Hoosier 43164 20.5x7.0-13 8in 12 psi | 0.198 | 0.514 | 1.594 | 0.480 | ok |
| 5 | Hoosier 43070 16x6.0-10 7in 12 psi | 0.209 | 0.708 | 1.527 | 0.487 | ok |
| 6 | Hoosier 43075 16x7.5-10 8in 14 psi | 0.212 | 0.692 | 1.532 | 0.519 | ok |
| 7 | Hoosier 43164 20.5x7.0-13 8in 10 psi | 0.219 | 0.549 | 1.640 | 0.409 | ok |
| 8 | Goodyear D2704 20.0x7.0-13 8in 14 psi | 0.219 | 0.528 | 1.564 | 0.511 | ok |
| 9 | Hoosier 43164 20.5x7.0-13 7in 12 psi | 0.225 | 0.520 | 1.593 | 0.478 | ok |
| 10 | Goodyear D0571 18.0x6.5-10 7in 14 psi | 0.225 | 0.613 | 1.580 | 0.487 | ok |
| 11 | Hoosier 43164 20.5x7.0-13 7in 10 psi | 0.227 | 0.574 | 1.644 | 0.401 | ok |
| 12 | Goodyear D2704 20.0x7.0-13 7in 14 psi | 0.231 | 0.639 | 1.579 | 0.428 | ok |

Best relaxation case by tire family:

| Family | Best case | sigma_alpha | Integrated | Std ay diag [g] | Flags |
| --- | --- | ---: | ---: | ---: | --- |
| Goodyear D0571 | Goodyear D0571 18.0x6.5-10 7in 14 psi | 0.225 | 0.613 | 1.580 | ok |
| Goodyear D2704 | Goodyear D2704 20.0x7.0-13 8in 14 psi | 0.219 | 0.528 | 1.564 | ok |
| Hoosier 43070 | Hoosier 43070 16x6.0-10 7in 14 psi | 0.186 | 0.711 | 1.484 | ok |
| Hoosier 43075 | Hoosier 43075 16x7.5-10 8in 14 psi | 0.212 | 0.692 | 1.532 | ok |
| Hoosier 43100 | Hoosier 43100 18.0x6.0-10 7in 14 psi | 0.257 | 0.687 | 1.531 | ok |
| Hoosier 43164 | Hoosier 43164 20.5x7.0-13 8in 14 psi | 0.162 | 0.504 | 1.576 | ok |
| MRF ZTD1 | MRF ZTD1 18x6.0-10 7in 14 psi | 0.236 | 0.279 | 1.513 | ok |

Clean positive-understeer response finalists:

| Int rank | Candidate | Integrated | sigma_alpha | Std ay diag [g] | US grad | Roll grad |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | Hoosier 43075 16x7.5-10 7in 8 psi | 0.895 | 0.410 | 1.658 | 0.390 | 0.934 |
| 3 | Hoosier 43075 16x7.5-10 8in 8 psi | 0.849 | 0.294 | 1.651 | 0.454 | 0.937 |
| 4 | Hoosier 43100 18.0x6.0-10 7in 8 psi | 0.836 | 0.372 | 1.552 | 0.674 | 0.995 |
| 5 | Hoosier 43100 18.0x6.0-10 6in 8 psi | 0.789 | 0.491 | 1.513 | 0.829 | 0.976 |
| 6 | Hoosier 43070 16x6.0-10 6in 14 psi | 0.785 | 0.258 | 1.495 | 0.496 | 0.933 |
| 7 | Hoosier 43075 16x7.5-10 7in 10 psi | 0.773 | 0.361 | 1.617 | 0.307 | 0.935 |
| 8 | Hoosier 43100 18.0x6.0-10 7in 10 psi | 0.770 | 0.333 | 1.630 | 0.213 | 1.000 |
| 9 | Hoosier 43070 16x6.0-10 7in 8 psi | 0.760 | 0.255 | 1.577 | 0.586 | 0.940 |
| 10 | Hoosier 43075 16x7.5-10 8in 10 psi | 0.754 | 0.267 | 1.609 | 0.407 | 0.937 |
| 11 | Hoosier 43070 16x6.0-10 6in 8 psi | 0.752 | 0.394 | 1.599 | 0.609 | 0.918 |
| 12 | Goodyear D2704 20.0x7.0-13 7in 10 psi | 0.752 | 0.299 | 1.663 | 0.313 | 1.059 |
| 13 | Hoosier 43070 16x6.0-10 6in 12 psi | 0.748 | 0.310 | 1.542 | 0.408 | 0.936 |

Hoosier 43075 practical lens:

Within the common 16x7.5-10 Hoosier 43075 family, the 7in/8psi case maximizes vehicle-level score, while the wider-rim and higher-pressure cases reduce relaxation length at the cost of some EnvelopeSim capability. This creates a real tuning choice: 7in/8psi for peak simulated vehicle score, or 8in/8psi to keep most of that score while cutting relaxation length substantially.

| 43075 case | Integrated | Envelope | Std | sigma_alpha | Std ay diag [g] | US grad | Roll grad | Flags |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Hoosier 43075 16x7.5-10 7in 8 psi | 0.895 | 0.868 | 0.947 | 0.410 | 1.658 | 0.390 | 0.934 | ok |
| Hoosier 43075 16x7.5-10 8in 8 psi | 0.849 | 0.831 | 0.881 | 0.294 | 1.651 | 0.454 | 0.937 | ok |
| Hoosier 43075 16x7.5-10 7in 10 psi | 0.773 | 0.702 | 0.903 | 0.361 | 1.617 | 0.307 | 0.935 | ok |
| Hoosier 43075 16x7.5-10 8in 10 psi | 0.754 | 0.691 | 0.870 | 0.267 | 1.609 | 0.407 | 0.937 | ok |
| Hoosier 43075 16x7.5-10 7in 12 psi | 0.740 | 0.652 | 0.904 | 0.310 | 1.583 | 0.376 | 0.933 | ok |
| Hoosier 43075 16x7.5-10 8in 12 psi | 0.725 | 0.638 | 0.885 | 0.239 | 1.572 | 0.439 | 0.934 | ok |
| Hoosier 43075 16x7.5-10 8in 14 psi | 0.692 | 0.588 | 0.886 | 0.212 | 1.532 | 0.519 | 0.931 | ok |
| Hoosier 43075 16x7.5-10 7in 14 psi | 0.668 | 0.552 | 0.884 | 0.263 | 1.545 | 0.511 | 0.931 | ok |

## Transient Temperature Evidence

The Round 9 transient step-steer runs include tread inner/center/outer temperature channels (`TSTI`, `TSTC`, `TSTO`), rim surface temperature (`RST`), and ambient temperature (`AMBTMP`). These temperatures are not EnvelopeSim or StandardSim response outputs, so they are not part of the vehicle score. They are used as a test-condition and operating-window check for the tire data.

Transient temperature rows exist for the 10, 12, and 14 psi transient pressure windows. The 8 psi vehicle finalists do not have direct transient-temperature windows in this dataset, so their thermal behavior still needs track validation.

| Candidate | Pressure | Mean tread | Peak tread | Rise | I-C-O spread | Inner-outer | Rim | Ambient |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Goodyear D0571 18.0x6.5-10 6in 10 psi | 10 | 31.8 C | 33.1 C | -0.3 C | 0.9 C | -0.8 C | 33.3 C | 26.7 C |
| Goodyear D0571 18.0x6.5-10 6in 12 psi | 12 | 32.5 C | 34.1 C | 4.4 C | 1.5 C | -1.3 C | 33.4 C | 26.8 C |
| Goodyear D0571 18.0x6.5-10 6in 14 psi | 14 | 32.0 C | 32.6 C | -0.3 C | 0.9 C | -0.5 C | 32.9 C | 26.9 C |
| Goodyear D0571 18.0x6.5-10 7in 10 psi | 10 | 29.7 C | 30.4 C | -0.1 C | 2.1 C | -2.1 C | 30.3 C | 25.6 C |
| Goodyear D0571 18.0x6.5-10 7in 12 psi | 12 | 30.3 C | 32.1 C | 2.8 C | 3.0 C | -3.0 C | 29.6 C | 25.5 C |
| Goodyear D0571 18.0x6.5-10 7in 14 psi | 14 | 29.6 C | 30.6 C | -0.1 C | 1.8 C | -1.7 C | 30.6 C | 25.6 C |
| Goodyear D2704 20.0x7.0-13 7in 10 psi | 10 | 31.4 C | 32.9 C | 0.4 C | 0.8 C | -0.7 C | 32.2 C | 26.6 C |
| Goodyear D2704 20.0x7.0-13 7in 12 psi | 12 | 31.6 C | 33.5 C | 4.1 C | 1.0 C | -0.6 C | 32.4 C | 27.0 C |
| Goodyear D2704 20.0x7.0-13 7in 14 psi | 14 | 31.7 C | 32.6 C | -0.2 C | 0.6 C | -0.2 C | 32.1 C | 27.0 C |
| Goodyear D2704 20.0x7.0-13 8in 10 psi | 10 | 31.8 C | 32.8 C | -0.3 C | 1.2 C | -0.8 C | 32.9 C | 26.8 C |
| Goodyear D2704 20.0x7.0-13 8in 12 psi | 12 | 32.0 C | 33.7 C | 4.7 C | 1.4 C | -0.8 C | 32.9 C | 26.7 C |
| Goodyear D2704 20.0x7.0-13 8in 14 psi | 14 | 31.9 C | 33.1 C | 0.5 C | 0.8 C | -0.4 C | 32.4 C | 26.8 C |
| Hoosier 43070 16x6.0-10 6in 10 psi | 10 | 31.1 C | 32.2 C | 0.5 C | 2.1 C | -1.7 C | 32.6 C | 26.7 C |
| Hoosier 43070 16x6.0-10 6in 12 psi | 12 | 30.8 C | 32.3 C | 4.2 C | 2.6 C | -2.4 C | 32.6 C | 26.6 C |
| Hoosier 43070 16x6.0-10 6in 14 psi | 14 | 31.2 C | 32.3 C | 0.2 C | 1.6 C | -1.2 C | 32.3 C | 26.8 C |
| Hoosier 43070 16x6.0-10 7in 10 psi | 10 | 31.8 C | 33.0 C | 0.3 C | 1.8 C | -1.5 C | 33.1 C | 26.9 C |
| Hoosier 43070 16x6.0-10 7in 12 psi | 12 | 31.7 C | 33.6 C | 4.5 C | 2.6 C | -2.5 C | 33.2 C | 26.8 C |
| Hoosier 43070 16x6.0-10 7in 14 psi | 14 | 31.9 C | 32.9 C | 0.1 C | 1.4 C | -1.0 C | 33.0 C | 27.0 C |
| Hoosier 43075 16x7.5-10 7in 10 psi | 10 | 28.5 C | 29.5 C | 0.2 C | 2.2 C | -2.2 C | 29.2 C | 25.5 C |
| Hoosier 43075 16x7.5-10 7in 12 psi | 12 | 28.9 C | 30.5 C | 2.3 C | 3.4 C | -3.4 C | 28.9 C | 25.7 C |
| Hoosier 43075 16x7.5-10 7in 14 psi | 14 | 28.5 C | 29.6 C | 0.3 C | 1.8 C | -1.8 C | 29.7 C | 25.3 C |
| Hoosier 43075 16x7.5-10 8in 10 psi | 10 | 31.1 C | 32.2 C | 0.4 C | 2.2 C | -2.1 C | 32.7 C | 26.6 C |
| Hoosier 43075 16x7.5-10 8in 12 psi | 12 | 30.7 C | 32.3 C | 4.2 C | 3.0 C | -3.0 C | 32.8 C | 26.6 C |
| Hoosier 43075 16x7.5-10 8in 14 psi | 14 | 31.2 C | 32.2 C | 0.2 C | 1.6 C | -1.5 C | 32.6 C | 26.7 C |
| Hoosier 43100 18.0x6.0-10 6in 10 psi | 10 | 30.3 C | 31.4 C | 0.4 C | 2.0 C | -1.0 C | 30.9 C | 26.4 C |
| Hoosier 43100 18.0x6.0-10 6in 12 psi | 12 | 30.2 C | 31.5 C | 3.3 C | 2.0 C | -1.2 C | 30.6 C | 25.8 C |
| Hoosier 43100 18.0x6.0-10 6in 14 psi | 14 | 30.5 C | 31.6 C | 0.2 C | 1.8 C | -0.9 C | 30.9 C | 26.6 C |
| Hoosier 43100 18.0x6.0-10 7in 10 psi | 10 | 31.3 C | 32.3 C | -0.2 C | 1.9 C | -1.3 C | 32.1 C | 26.7 C |
| Hoosier 43100 18.0x6.0-10 7in 12 psi | 12 | 31.4 C | 32.7 C | 3.2 C | 2.0 C | -1.4 C | 32.0 C | 26.8 C |
| Hoosier 43100 18.0x6.0-10 7in 14 psi | 14 | 31.4 C | 32.1 C | 0.3 C | 1.5 C | -0.8 C | 31.9 C | 26.9 C |
| Hoosier 43164 20.5x7.0-13 7in 10 psi | 10 | 28.8 C | 29.7 C | 0.5 C | 1.6 C | -1.5 C | 28.8 C | 26.6 C |
| Hoosier 43164 20.5x7.0-13 7in 12 psi | 12 | 28.6 C | 29.8 C | 2.1 C | 1.7 C | -1.7 C | 28.2 C | 26.5 C |
| Hoosier 43164 20.5x7.0-13 7in 14 psi | 14 | 28.9 C | 29.9 C | 0.4 C | 1.3 C | -1.3 C | 28.9 C | 26.6 C |
| Hoosier 43164 20.5x7.0-13 8in 10 psi | 10 | 31.0 C | 31.8 C | -0.0 C | 1.2 C | -0.9 C | 32.7 C | 26.7 C |
| Hoosier 43164 20.5x7.0-13 8in 12 psi | 12 | 30.7 C | 32.0 C | 4.2 C | 0.8 C | -0.4 C | 32.4 C | 26.8 C |
| Hoosier 43164 20.5x7.0-13 8in 14 psi | 14 | 31.2 C | 32.4 C | 0.7 C | 0.7 C | -0.5 C | 32.1 C | 26.7 C |
| MRF ZTD1 18x6.0-10 6in 10 psi | 10 | 30.8 C | 31.7 C | 0.4 C | 1.9 C | -1.4 C | 32.5 C | 26.4 C |
| MRF ZTD1 18x6.0-10 6in 12 psi | 12 | 30.5 C | 31.8 C | 3.3 C | 2.2 C | -1.9 C | 32.6 C | 26.2 C |
| MRF ZTD1 18x6.0-10 6in 14 psi | 14 | 30.9 C | 32.1 C | 0.4 C | 1.6 C | -1.2 C | 32.4 C | 26.6 C |
| MRF ZTD1 18x6.0-10 7in 10 psi | 10 | 30.7 C | 31.6 C | 0.4 C | 2.4 C | -1.4 C | 32.4 C | 26.9 C |
| MRF ZTD1 18x6.0-10 7in 12 psi | 12 | 30.3 C | 31.4 C | 3.3 C | 2.5 C | -2.0 C | 32.5 C | 26.6 C |
| MRF ZTD1 18x6.0-10 7in 14 psi | 14 | 31.0 C | 32.1 C | 0.3 C | 2.0 C | -0.9 C | 32.5 C | 27.0 C |

## Tire Degradation Evidence

The Round 9 RunGuide repeats the 12 psi slip-angle or slip-ratio sweep after the pressure sequence. DS-006 compares those initial and final 12 psi runs at the nominal 25 mph test speed window (`34-47 km/h`) using robust 95th-percentile measured force/load and a small-slip linear stiffness fit. This is raw tire-data evidence for wear, warmup, and run-order drift; it is not included in the EnvelopeSim, StandardSim, or integrated score.

Cornering degradation by tire/rim setup:

| Setup | Initial runs | Final runs | Peak mu_y delta | Ky/Fz delta | Tread delta | Samples i/f |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Goodyear D0571 18.0x6.5-10 6in | 34 | 35 | -1.1% | -2.5% | 0.8 C | 20066/19161 |
| Goodyear D0571 18.0x6.5-10 7in | 37 | 38 | -2.9% | -1.0% | -0.2 C | 20870/19839 |
| Goodyear D2704 20.0x7.0-13 7in | 23 | 24 | -3.2% | -6.1% | 1.7 C | 20404/19219 |
| Goodyear D2704 20.0x7.0-13 8in | 46 | 49 | -2.2% | -3.7% | -1.7 C | 18977/19803 |
| Hoosier 43070 16x6.0-10 6in | 11 | 12 | -0.5% | -0.7% | 0.9 C | 21066/19768 |
| Hoosier 43070 16x6.0-10 7in | 14 | 15 | -1.0% | -1.0% | 0.5 C | 21170/19930 |
| Hoosier 43075 16x7.5-10 7in | 4,5 | 6 | +1.6% | +1.3% | 0.7 C | 21176/17345 |
| Hoosier 43075 16x7.5-10 8in | 8 | 9 | +0.6% | -0.5% | 0.0 C | 21168/19932 |
| Hoosier 43100 18.0x6.0-10 6in | 28 | 29 | +0.7% | -0.0% | 0.7 C | 19636/18593 |
| Hoosier 43100 18.0x6.0-10 7in | 31 | 32 | -1.7% | -2.5% | 0.4 C | 20212/19684 |
| Hoosier 43164 20.5x7.0-13 7in | 17 | 18 | +1.3% | -1.9% | 1.6 C | 21070/19780 |
| Hoosier 43164 20.5x7.0-13 8in | 20 | 21 | +0.4% | -1.7% | 0.8 C | 21171/19926 |
| MRF ZTD1 18x6.0-10 6in | 40 | 41 | +7.3% | +3.0% | 0.6 C | 21172/19929 |
| MRF ZTD1 18x6.0-10 7in | 43 | 44 | +1.8% | +0.4% | 0.1 C | 21169/19920 |

Drive/brake degradation by tire/rim setup:

| Setup | Initial runs | Final runs | Peak mu_x delta | Kx/Fz delta | Tread delta | Samples i/f |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Goodyear D0571 18.0x6.5-10 6in | 75 | 76 | -9.4% | -8.1% | 1.1 C | 9060/9504 |
| Goodyear D0571 18.0x6.5-10 7in | 78 | 79 | -8.7% | -6.9% | 2.1 C | 9321/9231 |
| Goodyear D2704 20.0x7.0-13 7in | 57 | 58 | -16.6% | -13.1% | 2.7 C | 7388/8289 |
| Goodyear D2704 20.0x7.0-13 8in | 60 | 61 | -15.0% | -11.0% | 2.1 C | 7452/8239 |
| Hoosier 43100 18.0x6.0-10 6in | 69 | 70 | -19.4% | -11.2% | 0.8 C | 9551/9472 |
| Hoosier 43100 18.0x6.0-10 7in | 72 | 73 | -12.8% | -12.9% | 1.9 C | 9498/9737 |
| Hoosier 43164 20.5x7.0-13 7in | 51 | 52 | -8.4% | -14.5% | 3.7 C | 7467/7945 |
| Hoosier 43164 20.5x7.0-13 8in | 54 | 55 | -8.2% | -10.8% | 2.5 C | 7468/8028 |
| MRF ZTD1 18x6.0-10 6in | 63 | 64 | -1.6% | -3.4% | 5.1 C | 9679/9695 |
| MRF ZTD1 18x6.0-10 7in | 66 | 67 | +2.2% | -0.3% | 4.2 C | 9603/9602 |

Hoosier 43075 degradation read:

Both 43075 rim setups show negligible measured lateral degradation in the repeated 12 psi cornering sweep. Direct drive/brake degradation is not available for the 16 inch Hoosiers because their longitudinal/combined fits are scaled from the 18 inch Hoosier donor data.

| 43075 setup | Peak mu_y delta | Ky/Fz delta | Tread delta | Drive/brake note |
| --- | ---: | ---: | ---: | --- |
| Hoosier 43075 16x7.5-10 7in | +1.6% | +1.3% | 0.7 C | not available for 16in scaled longitudinal |
| Hoosier 43075 16x7.5-10 8in | +0.6% | -0.5% | 0.0 C | not available for 16in scaled longitudinal |

## Rim Effects

Rim deltas compare the widest fitted rim against the narrowest fitted rim for the same family/size/pressure.

| Metric | Median delta | Largest drop | Largest rise |
| --- | ---: | --- | --- |
| Envelope mean lateral g | -1.6% | Goodyear D2704 20.0x7.0-13 8 psi (-10.6%) | Hoosier 43075 16x7.5-10 14 psi (+1.6%) |
| StandardSim score | -2.8% | MRF ZTD1 18x6.0-10 8 psi (-15.7%) | Hoosier 43070 16x6.0-10 10 psi (+34.5%) |
| Understeer gradient | +11.4% | Goodyear D2704 20.0x7.0-13 8 psi (-7280.3%) | Hoosier 43070 16x6.0-10 10 psi (+380.7%) |
| Tire lateral relaxation sigma | -19.7% | Hoosier 43070 16x6.0-10 8 psi (-35.2%) | Hoosier 43164 20.5x7.0-13 8 psi (-2.1%) |
| Integrated score | -4.3% | MRF ZTD1 18x6.0-10 8 psi (-27.2%) | Hoosier 43070 16x6.0-10 10 psi (+9.0%) |

## Group Reads

| Group | Value | N | Median env score | Median std score | Median integrated | Best candidate |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| family | Goodyear D0571 | 8 | 0.673 | 0.794 | 0.692 | Goodyear D0571 18.0x6.5-10 7in 10 psi |
| family | Goodyear D2704 | 8 | 0.604 | 0.735 | 0.643 | Goodyear D2704 20.0x7.0-13 7in 8 psi |
| family | Hoosier 43070 | 8 | 0.654 | 0.937 | 0.742 | Hoosier 43070 16x6.0-10 6in 14 psi |
| family | Hoosier 43075 | 8 | 0.672 | 0.886 | 0.747 | Hoosier 43075 16x7.5-10 7in 8 psi |
| family | Hoosier 43100 | 8 | 0.703 | 0.890 | 0.752 | Hoosier 43100 18.0x6.0-10 7in 8 psi |
| family | Hoosier 43164 | 8 | 0.455 | 0.685 | 0.534 | Hoosier 43164 20.5x7.0-13 7in 8 psi |
| family | MRF ZTD1 | 8 | 0.071 | 0.728 | 0.299 | MRF ZTD1 18x6.0-10 6in 8 psi |
| longitudinal_combined_source | direct_drive_brake | 39 | 0.583 | 0.735 | 0.617 | Goodyear D2704 20.0x7.0-13 7in 8 psi |
| longitudinal_combined_source | pressure_extrapolated_lateral_direct_drive_brake | 1 | 0.640 | 0.737 | 0.674 | Goodyear D2704 20.0x7.0-13 8in 8 psi |
| longitudinal_combined_source | scaled_18in_hoosier_long_combined | 16 | 0.658 | 0.903 | 0.744 | Hoosier 43075 16x7.5-10 7in 8 psi |
| pressure_psi | 10.0 | 14 | 0.678 | 0.747 | 0.705 | Hoosier 43075 16x7.5-10 7in 10 psi |
| pressure_psi | 12.0 | 14 | 0.609 | 0.789 | 0.692 | Hoosier 43070 16x6.0-10 6in 12 psi |
| pressure_psi | 14.0 | 14 | 0.551 | 0.812 | 0.644 | Hoosier 43070 16x6.0-10 6in 14 psi |
| pressure_psi | 8.0 | 14 | 0.732 | 0.796 | 0.756 | Hoosier 43075 16x7.5-10 7in 8 psi |
| rim_width_in | 6.0 | 16 | 0.654 | 0.814 | 0.690 | Hoosier 43100 18.0x6.0-10 6in 8 psi |
| rim_width_in | 7.0 | 28 | 0.613 | 0.793 | 0.692 | Hoosier 43075 16x7.5-10 7in 8 psi |
| rim_width_in | 8.0 | 12 | 0.591 | 0.736 | 0.634 | Hoosier 43075 16x7.5-10 8in 8 psi |
| tire_size | 16x6.0-10 | 8 | 0.654 | 0.937 | 0.742 | Hoosier 43070 16x6.0-10 6in 14 psi |
| tire_size | 16x7.5-10 | 8 | 0.672 | 0.886 | 0.747 | Hoosier 43075 16x7.5-10 7in 8 psi |
| tire_size | 18.0x6.0-10 | 8 | 0.703 | 0.890 | 0.752 | Hoosier 43100 18.0x6.0-10 7in 8 psi |
| tire_size | 18.0x6.5-10 | 8 | 0.673 | 0.794 | 0.692 | Goodyear D0571 18.0x6.5-10 7in 10 psi |
| tire_size | 18x6.0-10 | 8 | 0.071 | 0.728 | 0.299 | MRF ZTD1 18x6.0-10 6in 8 psi |
| tire_size | 20.0x7.0-13 | 8 | 0.604 | 0.735 | 0.643 | Goodyear D2704 20.0x7.0-13 7in 8 psi |
| tire_size | 20.5x7.0-13 | 8 | 0.455 | 0.685 | 0.534 | Hoosier 43164 20.5x7.0-13 7in 8 psi |

## Current Reference Comparison

| Metric | Reference |
| --- | ---: |
| Envelope mean lateral g | 1.795 |
| Envelope 25 m/s lateral g | 2.090 |
| Envelope mean GGV area | 8.198 |
| StandardSim ay diagnostic [g] | 1.591 |
| StandardSim understeer gradient | 0.449 |
| StandardSim roll gradient | 0.890 |
| StandardSim peak handwheel torque | 18.143 |

## Complete Candidate Result Table

This table contains every candidate in the integrated study. The reference tire is listed separately above.

| Int rank | Env rank | Candidate | dR | Pressure | Rim | mu_y | mu_x | sigma_a | Lat mu degr | Lat Ky degr | Drive mu degr | Env score | Std ay diag [g] | US grad | Roll | Std score | Integrated | Resp flags | Std QA | Long source |
| ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 | 2 | Hoosier 43075 16x7.5-10 7in 8 psi | 0.0 | 8 | 7 | 2.701 | 2.310 | 0.410 | +1.6% | +1.3% | n/a | 0.868 | 1.658 | 0.390 | 0.934 | 0.947 | 0.895 | ok | ok | scaled_18in_hoosier_long_combined |
| 2 | 1 | Goodyear D2704 20.0x7.0-13 7in 8 psi | 50.8 | 8 | 7 | 3.469 | 2.275 | 0.328 | -3.2% | -6.1% | -16.6% | 0.922 | 1.698 | -0.006 | 1.042 | 0.793 | 0.877 | ok | ok | direct_drive_brake |
| 3 | 3 | Hoosier 43075 16x7.5-10 8in 8 psi | 0.0 | 8 | 8 | 2.638 | 2.256 | 0.294 | +0.6% | -0.5% | n/a | 0.831 | 1.651 | 0.454 | 0.937 | 0.881 | 0.849 | ok | ok | scaled_18in_hoosier_long_combined |
| 4 | 5 | Hoosier 43100 18.0x6.0-10 7in 8 psi | 25.4 | 8 | 7 | 2.792 | 2.387 | 0.372 | -1.7% | -2.5% | -12.8% | 0.797 | 1.552 | 0.674 | 0.995 | 0.910 | 0.836 | ok | ok | direct_drive_brake |
| 5 | 4 | Hoosier 43100 18.0x6.0-10 6in 8 psi | 25.4 | 8 | 6 | 2.782 | 2.734 | 0.491 | +0.7% | -0.0% | -19.4% | 0.804 | 1.513 | 0.829 | 0.976 | 0.760 | 0.789 | ok | ok | direct_drive_brake |
| 6 | 17 | Hoosier 43070 16x6.0-10 6in 14 psi | 0.0 | 14 | 6 | 2.515 | 2.313 | 0.258 | -0.5% | -0.7% | n/a | 0.682 | 1.495 | 0.496 | 0.933 | 0.977 | 0.785 | ok | ok | scaled_18in_hoosier_long_combined |
| 7 | 14 | Hoosier 43075 16x7.5-10 7in 10 psi | 0.0 | 10 | 7 | 2.532 | 2.152 | 0.361 | +1.6% | +1.3% | n/a | 0.702 | 1.617 | 0.307 | 0.935 | 0.903 | 0.773 | ok | ok | scaled_18in_hoosier_long_combined |
| 8 | 12 | Hoosier 43100 18.0x6.0-10 7in 10 psi | 25.4 | 10 | 7 | 2.717 | 2.309 | 0.333 | -1.7% | -2.5% | -12.8% | 0.718 | 1.630 | 0.213 | 1.000 | 0.866 | 0.770 | ok | ok | direct_drive_brake |
| 9 | 18 | Hoosier 43070 16x6.0-10 7in 8 psi | 0.0 | 8 | 7 | 2.416 | 2.066 | 0.255 | -1.0% | -1.0% | n/a | 0.664 | 1.577 | 0.586 | 0.940 | 0.937 | 0.760 | ok | ok | scaled_18in_hoosier_long_combined |
| 10 | 15 | Hoosier 43075 16x7.5-10 8in 10 psi | 0.0 | 10 | 8 | 2.514 | 2.136 | 0.267 | +0.6% | -0.5% | n/a | 0.691 | 1.609 | 0.407 | 0.937 | 0.870 | 0.754 | ok | ok | scaled_18in_hoosier_long_combined |
| 11 | 10 | Hoosier 43070 16x6.0-10 6in 8 psi | 0.0 | 8 | 6 | 2.565 | 2.521 | 0.394 | -0.5% | -0.7% | n/a | 0.728 | 1.599 | 0.609 | 0.918 | 0.798 | 0.752 | ok | ok | scaled_18in_hoosier_long_combined |
| 12 | 8 | Goodyear D2704 20.0x7.0-13 7in 10 psi | 50.8 | 10 | 7 | 3.059 | 2.434 | 0.299 | -3.2% | -6.1% | -16.6% | 0.761 | 1.663 | 0.313 | 1.059 | 0.735 | 0.752 | ok | ok | direct_drive_brake |
| 13 | 22 | Hoosier 43070 16x6.0-10 6in 12 psi | 0.0 | 12 | 6 | 2.403 | 2.110 | 0.310 | -0.5% | -0.7% | n/a | 0.645 | 1.542 | 0.408 | 0.936 | 0.940 | 0.748 | ok | ok | scaled_18in_hoosier_long_combined |
| 14 | 20 | Hoosier 43075 16x7.5-10 7in 12 psi | 0.0 | 12 | 7 | 2.453 | 2.130 | 0.310 | +1.6% | +1.3% | n/a | 0.652 | 1.583 | 0.376 | 0.933 | 0.904 | 0.740 | ok | ok | scaled_18in_hoosier_long_combined |
| 15 | 28 | Hoosier 43070 16x6.0-10 7in 10 psi | 0.0 | 10 | 7 | 2.425 | 2.061 | 0.232 | -1.0% | -1.0% | n/a | 0.628 | 1.570 | 0.453 | 0.939 | 0.938 | 0.736 | ok | ok | scaled_18in_hoosier_long_combined |
| 16 | 13 | Goodyear D0571 18.0x6.5-10 7in 10 psi | 25.4 | 10 | 7 | 2.661 | 2.425 | 0.265 | -2.9% | -1.0% | -8.7% | 0.706 | 1.620 | 0.411 | 1.006 | 0.789 | 0.735 | ok | ok | direct_drive_brake |
| 17 | 23 | Hoosier 43100 18.0x6.0-10 7in 12 psi | 25.4 | 12 | 7 | 2.600 | 2.258 | 0.299 | -1.7% | -2.5% | -12.8% | 0.645 | 1.569 | 0.422 | 1.000 | 0.902 | 0.735 | ok | ok | direct_drive_brake |
| 18 | 27 | Hoosier 43075 16x7.5-10 8in 12 psi | 0.0 | 12 | 8 | 2.433 | 2.113 | 0.239 | +0.6% | -0.5% | n/a | 0.638 | 1.572 | 0.439 | 0.934 | 0.885 | 0.725 | ok | ok | scaled_18in_hoosier_long_combined |
| 19 | 24 | Goodyear D0571 18.0x6.5-10 6in 12 psi | 25.4 | 12 | 6 | 2.530 | 2.358 | 0.270 | -1.1% | -2.5% | -9.4% | 0.641 | 1.628 | 0.304 | 0.998 | 0.853 | 0.715 | ok | ok | direct_drive_brake |
| 20 | 30 | Hoosier 43070 16x6.0-10 7in 14 psi | 0.0 | 14 | 7 | 2.416 | 2.147 | 0.186 | -1.0% | -1.0% | n/a | 0.598 | 1.484 | 0.583 | 0.934 | 0.922 | 0.711 | ok | ok | scaled_18in_hoosier_long_combined |
| 21 | 36 | Hoosier 43070 16x6.0-10 7in 12 psi | 0.0 | 12 | 7 | 2.354 | 2.044 | 0.209 | -1.0% | -1.0% | n/a | 0.579 | 1.527 | 0.487 | 0.936 | 0.948 | 0.708 | ok | ok | scaled_18in_hoosier_long_combined |
| 22 | 34 | Hoosier 43100 18.0x6.0-10 6in 14 psi | 25.4 | 14 | 6 | 2.540 | 2.336 | 0.326 | +0.7% | -0.0% | -19.4% | 0.583 | 1.550 | 0.401 | 0.996 | 0.932 | 0.705 | ok | ok | direct_drive_brake |
| 23 | 33 | Hoosier 43075 16x7.5-10 8in 14 psi | 0.0 | 14 | 8 | 2.405 | 2.138 | 0.212 | +0.6% | -0.5% | n/a | 0.588 | 1.532 | 0.519 | 0.931 | 0.886 | 0.692 | ok | ok | scaled_18in_hoosier_long_combined |
| 24 | 26 | Goodyear D0571 18.0x6.5-10 7in 12 psi | 25.4 | 12 | 7 | 2.520 | 2.430 | 0.243 | -2.9% | -1.0% | -8.7% | 0.640 | 1.594 | 0.464 | 1.003 | 0.789 | 0.692 | ok | ok | direct_drive_brake |
| 25 | 35 | Hoosier 43100 18.0x6.0-10 7in 14 psi | 25.4 | 14 | 7 | 2.594 | 2.305 | 0.257 | -1.7% | -2.5% | -12.8% | 0.583 | 1.531 | 0.540 | 1.000 | 0.879 | 0.687 | ok | ok | direct_drive_brake |
| 26 | 19 | Hoosier 43070 16x6.0-10 6in 10 psi | 0.0 | 10 | 6 | 2.488 | 1.954 | 0.347 | -0.5% | -0.7% | n/a | 0.664 | 1.680 | 0.094 | 0.925 | 0.697 | 0.675 | ok | ok | scaled_18in_hoosier_long_combined |
| 27 | 25 | Goodyear D2704 20.0x7.0-13 8in 8 psi | 50.8 | 8 | 8 | 2.820 | 2.303 | 0.282 | -2.2% | -3.7% | -15.0% | 0.640 | 1.632 | 0.463 | 1.058 | 0.737 | 0.674 | ok | ok | pressure_extrapolated_lateral_direct_drive_brake |
| 28 | 38 | Hoosier 43075 16x7.5-10 7in 14 psi | 0.0 | 14 | 7 | 2.373 | 2.109 | 0.263 | +1.6% | +1.3% | n/a | 0.552 | 1.545 | 0.511 | 0.931 | 0.884 | 0.668 | ok | ok | scaled_18in_hoosier_long_combined |
| 29 | 21 | Hoosier 43164 20.5x7.0-13 7in 8 psi | 57.2 | 8 | 7 | 2.726 | 2.672 | 0.256 | +1.3% | -1.9% | -8.4% | 0.651 | 1.633 | 0.404 | 1.084 | 0.670 | 0.658 | ok | ok | direct_drive_brake |
| 30 | 39 | Goodyear D0571 18.0x6.5-10 6in 14 psi | 25.4 | 14 | 6 | 2.421 | 2.229 | 0.281 | -1.1% | -2.5% | -9.4% | 0.551 | 1.579 | 0.498 | 0.998 | 0.831 | 0.649 | ok | ok | direct_drive_brake |
| 31 | 29 | Goodyear D2704 20.0x7.0-13 8in 10 psi | 50.8 | 10 | 8 | 2.696 | 2.436 | 0.261 | -2.2% | -3.7% | -15.0% | 0.613 | 1.609 | 0.439 | 1.065 | 0.708 | 0.646 | ok | ok | direct_drive_brake |
| 32 | 31 | Goodyear D2704 20.0x7.0-13 7in 14 psi | 50.8 | 14 | 7 | 2.707 | 2.218 | 0.231 | -3.2% | -6.1% | -16.6% | 0.596 | 1.579 | 0.428 | 1.061 | 0.719 | 0.639 | ok | ok | direct_drive_brake |
| 33 | 37 | Goodyear D2704 20.0x7.0-13 7in 12 psi | 50.8 | 12 | 7 | 2.723 | 2.308 | 0.247 | -3.2% | -6.1% | -16.6% | 0.579 | 1.603 | 0.413 | 1.065 | 0.701 | 0.622 | ok | ok | direct_drive_brake |
| 34 | 32 | Hoosier 43164 20.5x7.0-13 8in 8 psi | 57.2 | 8 | 8 | 2.601 | 2.754 | 0.250 | +0.4% | -1.7% | -8.2% | 0.594 | 1.636 | 0.418 | 1.083 | 0.670 | 0.621 | ok | ok | direct_drive_brake |
| 35 | 42 | Goodyear D0571 18.0x6.5-10 7in 14 psi | 25.4 | 14 | 7 | 2.400 | 2.258 | 0.225 | -2.9% | -1.0% | -8.7% | 0.515 | 1.580 | 0.487 | 1.001 | 0.794 | 0.613 | ok | ok | direct_drive_brake |
| 36 | 40 | Goodyear D2704 20.0x7.0-13 8in 12 psi | 50.8 | 12 | 8 | 2.611 | 2.359 | 0.238 | -2.2% | -3.7% | -15.0% | 0.536 | 1.572 | 0.506 | 1.061 | 0.740 | 0.607 | ok | ok | direct_drive_brake |
| 37 | 41 | Hoosier 43164 20.5x7.0-13 7in 10 psi | 57.2 | 10 | 7 | 2.592 | 2.280 | 0.227 | +1.3% | -1.9% | -8.4% | 0.525 | 1.644 | 0.401 | 1.082 | 0.664 | 0.574 | ok | ok | direct_drive_brake |
| 38 | 43 | Hoosier 43164 20.5x7.0-13 8in 10 psi | 57.2 | 10 | 8 | 2.559 | 2.301 | 0.219 | +0.4% | -1.7% | -8.2% | 0.490 | 1.640 | 0.409 | 1.080 | 0.659 | 0.549 | ok | ok | direct_drive_brake |
| 39 | 45 | Goodyear D2704 20.0x7.0-13 8in 14 psi | 50.8 | 14 | 8 | 2.468 | 1.990 | 0.219 | -2.2% | -3.7% | -15.0% | 0.416 | 1.564 | 0.511 | 1.061 | 0.734 | 0.528 | ok | ok | direct_drive_brake |
| 40 | 44 | Hoosier 43164 20.5x7.0-13 7in 12 psi | 57.2 | 12 | 7 | 2.437 | 2.252 | 0.225 | +1.3% | -1.9% | -8.4% | 0.420 | 1.593 | 0.478 | 1.077 | 0.705 | 0.520 | ok | ok | direct_drive_brake |
| 41 | 47 | Hoosier 43164 20.5x7.0-13 7in 14 psi | 57.2 | 14 | 7 | 2.418 | 2.165 | 0.184 | +1.3% | -1.9% | -8.4% | 0.403 | 1.573 | 0.503 | 1.075 | 0.724 | 0.516 | ok | ok | direct_drive_brake |
| 42 | 46 | Hoosier 43164 20.5x7.0-13 8in 12 psi | 57.2 | 12 | 8 | 2.431 | 2.217 | 0.198 | +0.4% | -1.7% | -8.2% | 0.414 | 1.594 | 0.480 | 1.077 | 0.700 | 0.514 | ok | ok | direct_drive_brake |
| 43 | 48 | Hoosier 43164 20.5x7.0-13 8in 14 psi | 57.2 | 14 | 8 | 2.386 | 2.118 | 0.162 | +0.4% | -1.7% | -8.2% | 0.388 | 1.576 | 0.499 | 1.074 | 0.718 | 0.504 | ok | ok | direct_drive_brake |
| 44 | 49 | MRF ZTD1 18x6.0-10 6in 8 psi | 25.4 | 8 | 6 | 1.981 | 1.773 | 0.328 | +7.3% | +3.0% | -1.6% | 0.183 | 1.565 | 0.561 | 0.997 | 0.864 | 0.421 | ok | ok | direct_drive_brake |
| 45 | 51 | MRF ZTD1 18x6.0-10 6in 12 psi | 25.4 | 12 | 6 | 1.932 | 1.692 | 0.287 | +7.3% | +3.0% | -1.6% | 0.080 | 1.542 | 0.807 | 0.998 | 0.755 | 0.316 | ok | ok | direct_drive_brake |
| 46 | 52 | MRF ZTD1 18x6.0-10 7in 8 psi | 25.4 | 8 | 7 | 1.917 | 1.744 | 0.284 | +1.8% | +0.4% | +2.2% | 0.080 | 1.479 | 0.952 | 0.996 | 0.728 | 0.307 | ok | ok | direct_drive_brake |
| 47 | 50 | MRF ZTD1 18x6.0-10 7in 12 psi | 25.4 | 12 | 7 | 1.894 | 1.733 | 0.251 | +1.8% | +0.4% | +2.2% | 0.085 | 1.520 | 0.887 | 0.997 | 0.706 | 0.303 | ok | ok | direct_drive_brake |
| 48 | 53 | MRF ZTD1 18x6.0-10 6in 14 psi | 25.4 | 14 | 6 | 1.896 | 1.701 | 0.274 | +7.3% | +3.0% | -1.6% | 0.063 | 1.533 | 0.850 | 0.996 | 0.728 | 0.296 | ok | ok | direct_drive_brake |
| 49 | 55 | MRF ZTD1 18x6.0-10 6in 10 psi | 25.4 | 10 | 6 | 1.891 | 1.696 | 0.312 | +7.3% | +3.0% | -1.6% | 0.041 | 1.515 | 0.823 | 0.998 | 0.759 | 0.292 | ok | ok | direct_drive_brake |
| 50 | 54 | MRF ZTD1 18x6.0-10 7in 14 psi | 25.4 | 14 | 7 | 1.864 | 1.718 | 0.236 | +1.8% | +0.4% | +2.2% | 0.057 | 1.513 | 0.960 | 0.994 | 0.691 | 0.279 | ok | ok | direct_drive_brake |
| 51 | 56 | MRF ZTD1 18x6.0-10 7in 10 psi | 25.4 | 10 | 7 | 1.907 | 1.682 | 0.268 | +1.8% | +0.4% | +2.2% | 0.003 | 1.484 | 0.990 | 0.996 | 0.698 | 0.246 | ok | ok | direct_drive_brake |
|  | 6 | Hoosier 43100 18.0x6.0-10 6in 10 psi | 25.4 | 10 | 6 | 2.859 | 2.245 | 0.428 | +0.7% | -0.0% | -19.4% | 0.785 | 4.989 | 2.653 | -2.522 | nan | nan | roll_outlier | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;roll_fit_nrmse_high;sideslip_fit_nrmse_high | direct_drive_brake |
|  | 7 | Goodyear D0571 18.0x6.5-10 6in 8 psi | 25.4 | 8 | 6 | 2.787 | 2.339 | 0.384 | -1.1% | -2.5% | -9.4% | 0.782 | 1.730 | 0.112 | 0.955 | nan | nan | ok | steer_excess_fit_nrmse_high | direct_drive_brake |
|  | 9 | Goodyear D0571 18.0x6.5-10 7in 8 psi | 25.4 | 8 | 7 | 2.722 | 2.378 | 0.285 | -2.9% | -1.0% | -8.7% | 0.737 | 1.867 | -1.330 | 0.904 | nan | nan | ok | failed_maneuver;metric_velocity_mismatch | direct_drive_brake |
|  | 11 | Goodyear D0571 18.0x6.5-10 6in 10 psi | 25.4 | 10 | 6 | 2.653 | 2.415 | 0.361 | -1.1% | -2.5% | -9.4% | 0.721 | 2.326 | -0.418 | 0.983 | nan | nan | ok | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;sideslip_fit_nrmse_high | direct_drive_brake |
|  | 16 | Hoosier 43100 18.0x6.0-10 6in 12 psi | 25.4 | 12 | 6 | 2.613 | 2.294 | 0.414 | +0.7% | -0.0% | -19.4% | 0.688 | 2.058 | -26.109 | -2.008 | nan | nan | understeer_outlier;sideslip_outlier;roll_outlier | roadwheel_fit_nrmse_high;steer_excess_fit_nrmse_high;roll_fit_nrmse_high;sideslip_fit_nrmse_high | direct_drive_brake |

## Caveats

- EnvelopeSim uses the tire pure-slip/load coefficients and the current vehicle-level assumptions; it is excellent for capability screening, not final handling sign-off.
- StandardSim SteadyStateEval captures vehicle response and balance, but it is still a steady-state maneuver set; transient feel should be checked separately before final tire sign-off.
- Larger tire candidates are adjusted for radius-driven CG/ride-height changes, but they are not fully re-optimized for suspension kinematics, aero map quality, gearing, packaging, or wheel/brake hardware.
- The 16 inch Hoosier candidates use scaled 18 inch Hoosier longitudinal/combined fits, so their longitudinal/braking conclusions carry less confidence than direct drive/brake Round 9 candidates.
- Longitudinal relaxation is still estimated because the Round 9 data does not include an equivalent slip-ratio transient run.

## Figure Gallery

These figures are generated from the same DS-006 outputs summarized above.

### Integrated score ranking across scoreable tire candidates

![Integrated score ranking across scoreable tire candidates](../studies/DS-006-integrated-tire-design/plots/integrated_score_rank.png)

### Current-package zero-radius-delta integrated ranking

![Current-package zero-radius-delta integrated ranking](../studies/DS-006-integrated-tire-design/plots/current_package_integrated_rank.png)

### Best scoreable setup by tire family

![Best scoreable setup by tire family](../studies/DS-006-integrated-tire-design/plots/family_best_score_comparison.png)

### Fitted peak lateral mu versus cornering stiffness

![Fitted peak lateral mu versus cornering stiffness](../studies/DS-006-integrated-tire-design/plots/tire_fit_mu_stiffness_map.png)

### Peak lateral mu and cornering stiffness by candidate

![Peak lateral mu and cornering stiffness by candidate](../studies/DS-006-integrated-tire-design/plots/tire_fit_mu_stiffness_rank.png)

### EnvelopeSim tire ranking

![EnvelopeSim tire ranking](../studies/DS-006-integrated-tire-design/plots/envelope_score_rank.png)

### StandardSim stable-window handling ranking

![StandardSim stable-window handling ranking](../studies/DS-006-integrated-tire-design/plots/standardsim_score_rank.png)

### Integrated tire trade space

![Integrated tire trade space](../studies/DS-006-integrated-tire-design/plots/integrated_trade_space.png)

### Pressure trends in EnvelopeSim and StandardSim score

![Pressure trends in EnvelopeSim and StandardSim score](../studies/DS-006-integrated-tire-design/plots/pressure_trends_vehicle_metrics.png)

### Lateral relaxation length ranking

![Lateral relaxation length ranking](../studies/DS-006-integrated-tire-design/plots/relaxation_rank.png)

### Relaxation length versus vehicle-level performance

![Relaxation length versus vehicle-level performance](../studies/DS-006-integrated-tire-design/plots/relaxation_trade_space.png)

### Transient tire-temperature evidence

![Transient tire-temperature evidence](../studies/DS-006-integrated-tire-design/plots/transient_temperature.png)

### Initial-to-final 12 psi cornering degradation

![Initial-to-final 12 psi cornering degradation](../studies/DS-006-integrated-tire-design/plots/tire_degradation_cornering.png)

### Initial-to-final 12 psi drive/brake degradation

![Initial-to-final 12 psi drive/brake degradation](../studies/DS-006-integrated-tire-design/plots/tire_degradation_drive_brake.png)

## Generated Files

- `outputs/candidate_registry.csv`
- `outputs/tire_characterization.csv`
- `outputs/transient_temperature_summary.csv`
- `outputs/degradation_summary.csv`
- `outputs/envelope_metrics.csv`
- `outputs/standardsim_metrics.csv`
- `outputs/standardsim_errors.csv`
- `outputs/integrated_results.csv`
- `outputs/group_summary.csv`
- `outputs/run_provenance.csv`
- `plots/integrated_score_rank.png`
- `plots/envelope_score_rank.png`
- `plots/standardsim_score_rank.png`
- `plots/current_package_integrated_rank.png`
- `plots/family_best_score_comparison.png`
- `plots/tire_fit_mu_stiffness_map.png`
- `plots/tire_fit_mu_stiffness_rank.png`
- `plots/pressure_trends_vehicle_metrics.png`
- `plots/integrated_trade_space.png`
- `plots/relaxation_rank.png`
- `plots/relaxation_trade_space.png`
- `plots/transient_temperature.png`
- `plots/tire_degradation_cornering.png`
- `plots/tire_degradation_drive_brake.png`

## Run Provenance

| Item | Value |
| --- | --- |
| Elapsed time | 108.7 s |
| Python | /tmp/lhre-sim-venv/bin/python |
| StandardSim skipped | False |
