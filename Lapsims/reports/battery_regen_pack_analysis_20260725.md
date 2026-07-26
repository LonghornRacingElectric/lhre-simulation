# Battery Regen Pack Analysis

Run date: 2026-07-25<br>
Vehicle and course: fixed LHRe OpenLAP vehicle on the Michigan endurance model<br>
Endurance distance: 22.000 km<br>
Study size: 128 feasible pack configurations<br>
Regeneration scope: **endurance only**

## Executive decision

The native-mesh **Reliance RS50 110s4p** is the strongest performance option in
this study. It:

- wins endurance at **1216.026 s** and receives **275.000/275** endurance
  points;
- has the highest pack-aware timed-event subtotal at **566.476/575**;
- remains first after cohort-normalized efficiency is included at
  **590.209/675**; and
- gains **13.625 kW** of sustainable endurance power and saves **21.532 s**
  relative to its no-regen result.

The total-score ranking is not separated enough to treat every place as a
hardware decision. The first four configurations are within **0.969 point out
of 675**, and two of those four are screening-mesh results. The best supported
interpretation is:

1. Choose **RS50 110s4p** if maximum dynamic-event performance is the primary
   objective.
2. Keep **RS50 140s3p** as the strongest already native-validated compromise
   near the top of the total ranking.
3. Native-rerun the screening-fidelity **RS50 130s3p** and **P50B 125s3p**
   before using their sub-point total-score differences to select hardware.
4. Do not use the efficiency score alone to break a close tie. The winner is
   only **0.008115 kWh (8.1 Wh)** below the 6.776 kWh eligibility limit, so
   small model or measurement changes can materially change that component.
5. Validate cell charge-current capability and cooling before approving any
   pack. The present thermal calculation is deliberately conservative and
   does not model a cooling system or thermal derating.

## Objective and scope

The objective was to repeat the full battery-pack comparison with regenerative
braking grounded in measured car-1001 endurance telemetry, then re-score the
candidate packs.

The scope boundary is important:

- Regeneration is enabled only in the chronological Michigan endurance
  simulation.
- Acceleration, skidpad, and autocross remain regeneration-free. Their prior
  pack-aware simulation results are reused exactly.
- Every endurance candidate uses the same telemetry-calibrated braking command
  policy.
- Each endurance candidate's discharge power ceiling is re-searched so it
  finishes with the same 0.500 kWh usable-chemical-energy reserve target.
- Endurance and the three sprint events are scored against the fastest
  simulated configuration in that event. Efficiency is then normalized within
  the same simulated cohort for the recommended 675-point comparison.

This isolates the effects of pack topology, mass, voltage, resistance,
continuous discharge capability, charge acceptance, and recovered energy
without adding regen to events where it was not requested.

## Telemetry basis and malformed-header repair

The source was the user-provided `car-1001-joined-endurance.csv`, SHA-256:

`49e34e23dd77cdd30aceb4dfb0172dd966d438f4aeb795c5fbeb01123a8c6882`

The file's header contains six names, but all data rows contain eight values.
The parser retained the six declared fields and appended the two energy-counter
fields:

| Original six fields | Appended fields |
|---|---|
| `time_s`, `voltage_v`, `power_kw`, `max_cell_temp_c`, `net_energy_kwh`, `current_a` | `outgoing_energy_kwh`, `incoming_energy_kwh` |

This repair is supported by the final counter identity:

`4.6298 kWh outgoing - 0.4860 kWh incoming = 4.1438 kWh net`

The residual from the recorded net counter is less than
`9e-16 kWh`. Power is time-weighted using a left-sample zero-order hold from
each timestamp to the next. The final sample has zero duration.

### Measured regenerative behavior

| Metric | All negative power | Active regen policy |
|---|---:|---:|
| Selector | `power_kw < 0` | `power_kw < -1.0` |
| Active duration | 308.953 s | 240.390 s |
| Event duty cycle | 17.032% | **13.252%** |
| Conditional active RMS | 7.567 kW | **8.574 kW** |
| Whole-event-equivalent RMS | 3.123 kW | **3.121 kW** |
| Peak regen magnitude | 22.230 kW | 22.230 kW |
| Integrated recovered energy | 0.4781 kWh | **0.4710 kWh** |

The `< -1.0 kW` selector is used to exclude small sign/noise crossings and
match the previously established telemetry convention. Its **8.574267 kW**
conditional active RMS is the calibration target. The telemetry's final
incoming-energy counter, **0.4860 kWh**, provides an independent integrated
energy check.

The 1 kW deadband is part of the definition, not an incidental filter. With a
0.5 kW deadband, the same time-weighted trace gives an active RMS of
approximately **8.234 kW**, while whole-event RMS remains approximately
3.122 kW. The selected 8.574 kW target and the resulting 10.059 kW command are
therefore internally consistent with the frozen `< -1.0 kW` policy, but the
active RMS must not be treated as threshold-independent.

## Frozen regenerative-command policy

The measured RMS is not treated as an instantaneous power clamp. A native
Michigan 22 km calibration was performed on the P30B 130s5p reference pack at
its then-current 26.078125 kW discharge ceiling:

| Terminal regen command | Achieved active RMS | Whole-event RMS | Recovered energy |
|---:|---:|---:|---:|
| 9.800 kW | 8.3948 kW | 4.0974 kW | 0.6671 kWh |
| 10.075 kW | 8.5854 kW | 4.1904 kW | 0.6808 kWh |

Linear interpolation gives a fixed terminal command of **10.059 kW**, with an
expected reference active RMS of 8.57433 kW. That command is below the measured
22.23 kW instantaneous peak.

The **10.059 kW command is held fixed for all 128 packs**. It is not retuned
per candidate. The achieved result is allowed to vary with braking source,
motor/inverter operation, tire limit, terminal voltage, current, SOC, and
charge headroom. Across the final cohort, achieved active RMS is
**8.040-8.893 kW**, with a median of **8.651 kW**.

## Regen, SOC, and thermal model

The endurance solver now supports signed electrical power while its
zero-regen/default path remains unchanged:

- Positive terminal power and current represent battery discharge.
- Negative terminal power and current represent battery charging.
- The pack equivalent-circuit solve is used in both directions. Terminal
  voltage rises above OCV while charging and falls below OCV while discharging.
- SOC decreases under positive current and increases under negative current.
  Maximum cell voltage and available SOC headroom prevent overcharge.
- A reverse motor/inverter/drivetrain path maps allowed wheel-braking power
  into accumulator-terminal charging power.
- Mechanical brakes provide any remaining demanded braking force. Regen
  therefore changes energy flow without weakening the modeled braking envelope.
- Motor, inverter, pack-voltage, pack-current, tire, and command limits are
  applied at every step.
- Terminal discharge, regenerated energy, net terminal energy, chemical
  discharge/storage, and the solver energy balance are integrated separately.
- Pack heat is positive `I^2 R` in both directions. Regen heat is recorded
  separately and is included in total heat.
- Adiabatic temperature rise uses total pack resistive heat and
  `1000 J/(kg K)` cell specific heat.

The thermal result is an adiabatic energy-to-temperature upper bound. It does
not model conduction between cells, a cooling plate, coolant, ambient heat
rejection, temperature-dependent resistance, or thermal power derating.

## Pack sweep and equal-reserve method

The study begins with eight identified cell models and generates series counts
from 110s through 140s in five-cell increments. Parallel counts begin at the
first topology providing at least 9 Ah and end at 6p.

The topology screens are:

- maximum charged pack voltage strictly below 600 V;
- modeled usable chemical energy no greater than 8.0 kWh;
- cylindrical-cell envelope no larger than the P30B 130s5p baseline,
  11.515 L; and
- the study's cell and inverter current constraints.

Of 259 generated topologies, 128 pass the pre-simulation constraints and 131
are rejected. Every accepted configuration completes the full 22 km
endurance distance.

For each accepted pack, the discharge terminal-power ceiling is searched
against a **0.500 kWh final usable-chemical-energy reserve**, with a
**0.010 kWh tolerance**. All 128 results meet the equal-reserve requirement;
the largest absolute reserve error is 0.006268 kWh. None is an 80 kW
power-cap saturation case.

All candidates first run on the factor-8 screening mesh. Twenty-two selected
finalists, including the baseline, cell-level contenders, and report
recommendations, are rerun on the native 0.25 m mesh. A native result replaces
its screening result in the final table.

## Endurance result: top 10

Endurance is scored from the raw 22 km time. The cohort-fastest
1216.026125 s result defines `Tmin` and receives 275 points.

| Rank | Configuration | Fidelity | Power limit (kW) | Time (s) | Endurance points | Recovered / net energy (kWh) | Total / regen heat (kWh) | Active RMS (kW) | Adiabatic rise (deg C) |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | `reliance_rs50_110s4p` | Native | 48.719 | 1216.026 | 275.000 | 0.827 / 6.768 | 0.491 / 0.010 | 8.803 | 60.0 |
| 2 | `tenpower_60xg_125s3p` | Native | 47.672 | 1216.845 | 274.458 | 0.815 / 6.692 | 0.743 / 0.015 | 8.768 | 93.8 |
| 3 | `ampace_jp50_110s4p` | Native | 48.250 | 1218.473 | 273.382 | 0.828 / 6.773 | 0.489 / 0.010 | 8.807 | 55.6 |
| 4 | `tenpower_50xg_110s4p` | Native | 47.938 | 1219.982 | 272.388 | 0.830 / 6.772 | 0.487 / 0.010 | 8.811 | 53.1 |
| 5 | `tenpower_60xg_120s3p` | Native | 42.293 | 1221.928 | 271.109 | 0.794 / 6.438 | 0.685 / 0.015 | 8.740 | 90.2 |
| 6 | `reliance_rs50_140s3p` | Native | 42.625 | 1222.149 | 270.964 | 0.802 / 6.465 | 0.445 / 0.010 | 8.766 | 57.0 |
| 7 | `molicel_p50b_110s4p` | Native | 43.250 | 1224.092 | 269.692 | 0.803 / 6.528 | 0.732 / 0.015 | 8.741 | 84.4 |
| 8 | `ampace_jp50_140s3p` | Native | 42.313 | 1224.557 | 269.388 | 0.802 / 6.470 | 0.444 / 0.011 | 8.764 | 52.8 |
| 9 | `tenpower_50xg_140s3p` | Native | 42.000 | 1226.191 | 268.322 | 0.803 / 6.465 | 0.441 / 0.011 | 8.765 | 50.4 |
| 10 | `ampace_jp50_135s3p` | Screening | 42.344 | 1226.664 | 268.014 | 0.730 / 6.204 | 0.439 / 0.010 | 8.893 | 54.2 |

The first nine endurance positions are native-mesh results. The tenth-place
JP50 135s3p result remains screening fidelity.

## Fully normalized dynamic-event result: top 15

The table below is the recommended pack-aware decision view:

- acceleration: 100 maximum;
- skidpad: 75 maximum;
- autocross: 125 maximum;
- endurance: 275 maximum;
- timed-event performance subtotal: 575 maximum; and
- cohort-normalized efficiency: 100 maximum, producing 675 total.

The fastest simulated configuration in each event receives that event's
maximum score. The sprint-event anchors are unchanged no-regen runs; only
endurance is regenerated.

| Rank | Configuration | Fidelity | Accel | Skidpad | Autox | Endurance | Performance /575 | Efficiency /100 | Total /675 |
|---:|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | `reliance_rs50_110s4p` | Native | 97.234 | 71.794 | 122.448 | 275.000 | 566.476 | 23.733 | **590.209** |
| 2 | `reliance_rs50_130s3p` | Screening | 95.827 | 72.503 | 123.042 | 265.663 | 557.036 | 32.897 | **589.933** |
| 3 | `reliance_rs50_140s3p` | Native | 96.785 | 72.076 | 122.702 | 270.964 | 562.528 | 26.908 | **589.435** |
| 4 | `molicel_p50b_125s3p` | Screening | 99.330 | 72.398 | 123.270 | 255.932 | 550.930 | 38.310 | **589.240** |
| 5 | `reliance_rs50_135s3p` | Native | 96.347 | 72.289 | 122.879 | 267.372 | 558.887 | 29.488 | **588.375** |
| 6 | `ampace_jp50_135s3p` | Screening | 95.906 | 71.861 | 122.385 | 268.014 | 558.167 | 29.939 | **588.106** |
| 7 | `molicel_p50b_140s3p` | Native | 99.159 | 71.723 | 122.534 | 265.353 | 558.768 | 29.148 | **587.915** |
| 8 | `molicel_p50b_110s4p` | Native | 98.585 | 71.425 | 122.171 | 269.692 | 561.873 | 26.013 | **587.886** |
| 9 | `molicel_p50b_135s3p` | Native | 99.366 | 71.947 | 122.790 | 261.791 | 555.893 | 31.702 | **587.595** |
| 10 | `molicel_p50b_120s3p` | Screening | 99.079 | 72.625 | 123.491 | 250.771 | 545.966 | 41.454 | **587.420** |
| 11 | `tenpower_30xg_120s5p` | Screening | 99.353 | 71.937 | 122.778 | 255.525 | 549.594 | 37.666 | **587.260** |
| 12 | `tenpower_60xg_125s3p` | Native | 93.954 | 72.000 | 122.294 | 274.458 | 562.705 | 24.544 | **587.250** |
| 13 | `reliance_rs50_120s3p` | Screening | 94.506 | 72.934 | 123.317 | 257.126 | 547.883 | 39.152 | **587.035** |
| 14 | `ampace_jp50_110s4p` | Native | 96.765 | 71.334 | 121.919 | 273.382 | 563.400 | 23.521 | **586.921** |
| 15 | `ampace_jp50_130s3p` | Screening | 95.399 | 72.089 | 122.565 | 264.192 | 554.245 | 32.673 | **586.918** |

The P30B 130s5p reference scores 549.630/575 timed-event points,
32.883/100 cohort efficiency points, and **582.514/675 total**. The winner
therefore gains 16.846 performance points and 7.695 total points over the
reference.

## Change from the no-regen study

Across all 128 matched configurations:

| Metric | Cohort result |
|---|---:|
| Sustainable endurance-power gain | 1.844-13.625 kW; 4.359 kW median |
| Endurance-time reduction | 17.405-61.693 s; 37.367 s median |
| Recovered terminal energy | 0.373-0.830 kWh; 0.570 kWh median |
| Net terminal-energy reduction | 0.029-0.243 kWh; 0.081 kWh median |
| Total pack-heat increase | 0.032-0.236 kWh; 0.082 kWh median |
| Direct regen pack heat | 0.0077-0.0157 kWh; 0.0104 kWh median |

Three representative changes show why recovered energy improves pace but does
not automatically reduce total heat:

| Configuration | Power gain (kW) | Time saved (s) | Recovered (kWh) | Net energy reduction (kWh) | Total heat change (kWh) | Endurance rank |
|---|---:|---:|---:|---:|---:|---:|
| `reliance_rs50_110s4p` | +13.625 | 21.532 | 0.827 | 0.165 | +0.165 | 2 -> 1 |
| `tenpower_60xg_125s3p` | +12.469 | 19.551 | 0.815 | 0.243 | +0.236 | 1 -> 2 |
| `molicel_p30b_130s5p` | +7.648 | 29.188 | 0.751 | 0.154 | +0.153 | 28 -> 24 |

For the winning RS50 110s4p, direct regen `I^2 R` heat is only 0.0104 kWh.
Most of its 0.1651 kWh total-heat increase comes from using recovered energy to
support a higher 48.719 kW discharge ceiling, rather than from the charge pulse
itself.

## Thermal and charge-acceptance findings

Across the final cohort:

| Metric | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Total pack resistive heat | 0.0869 kWh | 0.2530 kWh | 0.7427 kWh |
| Regen-only pack heat | 0.0077 kWh | 0.0104 kWh | 0.0157 kWh |
| Adiabatic cell-temperature rise | 18.9 deg C | 39.6 deg C | 93.8 deg C |
| Peak cell charge current | 4.49 A | 7.66 A | 12.86 A |

The endurance runner includes regen heating correctly, but the resulting
temperature rise is not a prediction of on-car peak cell temperature. It is a
screening metric for comparing heat generation under a common no-cooling
assumption.

The largest top-five thermal concern is **Tenpower 60XG 125s3p**:
0.7427 kWh total resistive heat and a 93.8 deg C adiabatic rise. That result is
nearly tied for endurance pace but is not comparable to the RS50 winner on
thermal robustness.

The candidate cell dataset does not provide a validated maximum charge-current
map for every cell. Missing limits are therefore unbounded in the model rather
than guessed. The modeled peak cell charge-current range must be checked
against supplier data and team pulse testing before any configuration is
approved.

## Interpretation limits

### Track duty does not match telemetry duty

The command is calibrated to measured **conditional active RMS**, not to
measured event duty or recovered energy. The measured `< -1 kW` duty is
13.252%, whole-event RMS is 3.121 kW, and integrated recovered energy is
0.471 kWh.

Conditional active RMS is also sensitive to the chosen 1 kW deadband: reducing
the deadband to 0.5 kW changes it from 8.574 kW to about 8.234 kW even though
whole-event RMS remains about 3.122 kW. Any future change to the active-regen
selector requires recalibrating the command rather than reusing 10.059 kW.

The simulated cohort has:

- active duty of **12.631-29.263%**, with a 19.931% median;
- whole-event RMS of **2.897-4.763 kW**; and
- recovered energy of **0.373-0.830 kWh**.

The winner is at the high end: 29.263% active duty, 4.762 kW whole-event RMS,
and 0.827 kWh recovered. This means the Michigan model presents substantially
more useful braking opportunity than the supplied telemetry event even though
active RMS is similar. The result is valid for a fixed active-RMS command on
the modeled track; it is not a claim that the car will recover 0.827 kWh on
every real endurance course.

### Mixed mesh fidelity

The final table contains 22 native-mesh results and 106 screening-mesh results.
The endurance top nine are native, but seven of the fully normalized top 15
remain screening fidelity. Sub-point ordering involving a screening result is
provisional.

### Efficiency is a threshold-sensitive secondary view

Strict 2026 EV efficiency eligibility uses a 6.776 kWh terminal-energy limit
and a time limit of 1.45 times the cohort endurance `Tmin`. All 128 current
results are eligible, but the largest net energy in the cohort is only
0.00287 kWh below the energy limit. The recommended hardware decision should
therefore lead with the 575-point performance subtotal and use the
cohort-efficiency score as a sensitivity rather than a precise tie-breaker.

### Thermal and electrical data limits

- The thermal model is adiabatic and has no cooling or derating feedback.
- Resistance is fixed at the study's 25 deg C input condition apart from the
  shared SOC multiplier.
- Several non-Molicel DCIR values are estimated from ACIR and remain
  provisional.
- Cell-specific charge-current limits and SOC/temperature-dependent charge
  maps require supplier or test data.
- The fixed command represents one driver/control policy. It does not optimize
  regen by corner, pack, SOC, or temperature.

## Validation and tests

The final artifacts pass the following automated checks:

- **23 endurance-study boolean checks**, including topology coverage, voltage,
  energy, packaging, power/current limits, exact 22 km distance, signed energy
  identity, nonnegative regen energy/heat, command and peak enforcement,
  maximum charge voltage, RMS threshold use, equal reserve, native finalist
  coverage, ranking, Pareto recomputation, and sensitivity cases.
- **128/128 equal-reserve configurations**, with no 80 kW saturated cases.
- **5 comparison-level checks** plus exact arithmetic for all 11 recorded
  regen-minus-no-regen delta fields.
- **44 cohort-scoring checks** over 128 candidates and 896 timed-event rows.
  These verify anchors, monotonic score behavior, eligibility, bounds, and all
  point-total arithmetic.
- **56 unit tests** covering the telemetry parser, signed pack solve,
  zero-regen regression, SOC increase under charging, mechanical-brake
  remainder, energy/heat balances, dynamic refresh, comparison, and cohort
  scoring.

The no-regen results remain in their original versioned output directory and
are not overwritten by this study.

## Exact reproduction commands

Run these commands from `Lapsims`. The telemetry CSV is not committed; place
the source at the path shown or substitute the local path to an identical file
with the recorded SHA-256.

### 1. Verify the telemetry analysis

```powershell
python .\tools\analyze_regen_telemetry.py `
  C:\Users\Abishek\Downloads\car-1001-joined-endurance.csv `
  --active-threshold-kw 1.0 `
  --output `
    .\outputs\battery_trade_study_regen_endurance_only_20260725\source_telemetry_regen_analysis.json
```

The checked-in telemetry policy and native calibration files contain the
selected interpretation and frozen 10.059 kW command.

### 2. Run the full endurance pack sweep

```powershell
python .\tools\run_battery_trade_study.py `
  --regen-policy `
    .\inputs\battery_trade_study\endurance_regen_telemetry_policy.json `
  --regen-command-power-kw 10.059 `
  --output-root `
    .\outputs\battery_trade_study_regen_endurance_only_20260725
```

### 3. Validate the endurance study

```powershell
python .\tools\validate_battery_trade_study.py `
  --output-root `
    .\outputs\battery_trade_study_regen_endurance_only_20260725
```

### 4. Compare with the no-regen results

```powershell
python .\tools\compare_battery_regen_results.py `
  --prior-results `
    .\outputs\battery_trade_study_20260725_final\all_configuration_results.csv `
  --regen-results `
    .\outputs\battery_trade_study_regen_endurance_only_20260725\all_configuration_results.csv `
  --output-dir `
    .\outputs\battery_regen_comparison_20260725 `
  --scenario-name regen_endurance_only_vs_no_regen
```

### 5. Reuse sprint results and refresh endurance

```powershell
python .\tools\refresh_battery_dynamic_points_endurance.py `
  --sprint-source-dir .\outputs\battery_dynamic_points_20260725 `
  --battery-results `
    .\outputs\battery_trade_study_regen_endurance_only_20260725\all_configuration_results.csv `
  --output-dir `
    .\outputs\battery_dynamic_points_regen_endurance_only_20260725 `
  --scenario-name regen_endurance_only
```

### 6. Re-score the complete simulated cohort

```powershell
python .\tools\run_battery_cohort_scoring.py `
  --source-summary `
    .\outputs\battery_dynamic_points_regen_endurance_only_20260725\battery_dynamic_points_summary.csv `
  --battery-results `
    .\outputs\battery_trade_study_regen_endurance_only_20260725\all_configuration_results.csv `
  --output-dir `
    .\outputs\battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725
```

### 7. Run the unit tests

```powershell
python -m unittest discover -s .\tests -p "test_*.py" -q
```

For an intentional rerun into an already populated comparison or dynamic
refresh directory, add `--overwrite` to that corresponding command.

## Artifact index

### Calibration inputs

- [Telemetry interpretation and target](../inputs/battery_trade_study/endurance_regen_telemetry_policy.json)
- [Native command calibration](../inputs/battery_trade_study/endurance_regen_simulation_calibration.json)
- [Battery-cell inputs](../inputs/battery_trade_study/cells.json)
- [OpenLAP vehicle](../inputs/openlap_vehicle.json)
- [Michigan track](../inputs/michigan_openlap_track.csv)

### Endurance sweep

- [Telemetry analysis output](../outputs/battery_trade_study_regen_endurance_only_20260725/source_telemetry_regen_analysis.json)
- [Study inputs](../outputs/battery_trade_study_regen_endurance_only_20260725/study_inputs.json)
- [All 128 configuration results](../outputs/battery_trade_study_regen_endurance_only_20260725/all_configuration_results.csv)
- [Native refinement results](../outputs/battery_trade_study_regen_endurance_only_20260725/refined_results.csv)
- [Endurance study report](../outputs/battery_trade_study_regen_endurance_only_20260725/battery_trade_study_report.md)
- [Endurance validation](../outputs/battery_trade_study_regen_endurance_only_20260725/validation_report.json)
- [Endurance plots](../outputs/battery_trade_study_regen_endurance_only_20260725/plots/)

### Regen versus no-regen comparison

- [Candidate-by-candidate comparison](../outputs/battery_regen_comparison_20260725/battery_regen_comparison.csv)
- [Comparison report](../outputs/battery_regen_comparison_20260725/battery_regen_comparison_report.md)
- [Comparison headline summary](../outputs/battery_regen_comparison_20260725/headline_summary.json)
- [Comparison validation](../outputs/battery_regen_comparison_20260725/validation_report.json)
- [Comparison plots](../outputs/battery_regen_comparison_20260725/plots/)

### Dynamic-event scores

- [Fully normalized score table](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/battery_cohort_points_summary.csv)
- [Event-level score table](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/battery_cohort_event_results_long.csv)
- [Cohort scoring inputs and anchors](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/cohort_scoring_inputs.json)
- [Dynamic points report](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/battery_cohort_points_report.md)
- [Dynamic points validation](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/validation_report.json)
- [Dynamic points plots](../outputs/battery_dynamic_points_cohort_normalized_regen_endurance_only_20260725/plots/)

### Implementation and validation code

- [Powertrain model](../src/powertrain_model.py)
- [Chronological endurance solver](../src/endurance_solver.py)
- [Battery trade-study model](../src/battery_trade_study.py)
- [Endurance dynamic-point refresh](../src/battery_dynamic_refresh.py)
- [Cohort scoring](../src/battery_cohort_scoring.py)
- [Regen comparison](../src/battery_regen_comparison.py)
- [Telemetry analyzer](../tools/analyze_regen_telemetry.py)
- [Full study runner](../tools/run_battery_trade_study.py)
- [Test suite](../tests/)
