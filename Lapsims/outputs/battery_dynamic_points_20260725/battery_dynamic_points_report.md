# Battery configuration dynamic-points study

## Scope

All 128 accepted battery configurations from the 22 km trade study
were run through acceleration, skidpad, and autocross. The primary result uses
each pack's full-SOC voltage, resistance, continuous-current limit, and the
common 80 kW battery-terminal ceiling. The diagnostic result changes vehicle
mass only while keeping one common 80 kW OpenLAP tractive curve.

The performance subtotal is 575 points maximum (acceleration 100, skidpad 75,
autocross 125, endurance 275). The full dynamic total adds the inherited
100-point efficiency projection for a 675-point maximum.

## Headline results

- Highest pack-aware full dynamic score: **molicel_p50b_130s3p** at
  **675.00 / 675**.
- Baseline **molicel_p30b_130s5p**:
  **675.00 / 675** pack-aware and
  **675.00 / 675** in
  the hybrid that combines mass-isolated sprint scores with actual
  pack-specific endurance and efficiency.
- 13 of 128 packs reach
  at least 79.9 kW battery-terminal power at full SOC somewhere on the speed
  curve. The others are limited first by their electrical envelope.
- The mass-only non-energy subtotal changes
  **-1.61 points**
  from the lightest pack (18.43 kg) to the
  heaviest pack (41.25 kg). Because lighter-than-baseline
  candidates clip at the event maxima, use the uncapped raw-time columns for
  comparisons within that clipped region.
- Lightest-to-heaviest mass-only raw time changes are
  **0.0559 s acceleration**,
  **0.0479 s skidpad**,
  and **0.6150 s autocross**.
- At the baseline vehicle mass, the one-sided adverse mass-only sensitivity is
  **0.528 non-energy points lost per added kg**:
  acceleration
  0.165, skidpad
  0.170, and autocross
  0.193 point/kg.

## Primary ranking

| Candidate | Pack kg | Accel | Skidpad | Autox | Endurance | Efficiency | Total |
|---|---:|---:|---:|---:|---:|---:|---:|
| molicel_p50b_130s3p | 34.612 | 100.00 | 75.00 | 125.00 | 275.00 | 100.00 | 675.00 |
| tenpower_30xg_125s5p | 37.500 | 100.00 | 75.00 | 125.00 | 275.00 | 100.00 | 675.00 |
| molicel_p30b_130s5p | 38.188 | 100.00 | 75.00 | 125.00 | 275.00 | 100.00 | 675.00 |
| ampace_jp30_125s5p | 39.062 | 100.00 | 74.85 | 124.93 | 275.00 | 100.00 | 674.78 |
| molicel_p50b_135s3p | 35.944 | 100.00 | 75.00 | 125.00 | 275.00 | 99.05 | 674.05 |
| molicel_p50b_125s3p | 33.281 | 100.00 | 75.00 | 125.00 | 274.05 | 100.00 | 674.05 |
| molicel_p30b_125s5p | 36.719 | 99.69 | 75.00 | 125.00 | 273.84 | 100.00 | 673.53 |
| reliance_rs50_130s3p | 32.663 | 98.04 | 75.00 | 125.00 | 275.00 | 99.55 | 672.59 |
| reliance_rs50_125s3p | 31.406 | 97.42 | 75.00 | 125.00 | 275.00 | 100.00 | 672.42 |
| tenpower_30xg_130s5p | 39.000 | 100.00 | 74.86 | 124.94 | 275.00 | 97.44 | 672.24 |

## Scoring and normalization

- Formula SAE 2026 time-score equations are used with event-winner anchors:
  acceleration 3.697 s, skidpad 4.782 s, autocross 43.937 s, and endurance
  1312.281 s.
- For each sprint model and event, the P30B 130s5p baseline's converted-track
  time is assigned that event's 2026 winner time. Every candidate retains its
  simulated time ratio to the P30B baseline before the official formula is
  applied. Raw converted-track and projected competition times are both saved.
- The 22 km endurance simulation is mapped by time ratio: the P30B 130s5p
  baseline is assigned the 1312.281 s winner time, and every candidate keeps
  its simulated time ratio to that baseline. A completed candidate receives
  the 25 lap/completion points plus up to 250 time points.
- Efficiency is explicitly an inherited ratio-normalized projection, not a
  fresh competition energy-meter reconstruction. Candidate average-lap-time
  and terminal-energy ratios to P30B are mapped onto the established winner
  inputs (65.192 s average lap and 3.263 kWh over 22 laps), then scored with
  EFmin 0.289 and EFmax 0.797.

## Interpretation

Use the pack-aware result to compare real candidate concepts because it includes
the full-SOC electrical power constraint. Use the common-curve result to answer
the narrower question, "what does battery mass alone cost?" The signed
`pack_aware_minus_mass_isolated` difference reports how a candidate's
electrical envelope changes its score *relative to P30B*, because each model is
independently ratio-normalized to the P30B baseline. A negative value means the
candidate is electrically disadvantaged relative to P30B; a positive value
means it is less constrained than P30B. It is not an absolute point penalty.

Only the 300-point short-event subtotal is cleanly mass-isolated. Any 575- or
675-point column carrying the `mass_isolated_sprint_hybrid` prefix combines
mass-only sprint scores with the actual pack-specific chronological endurance
and inherited efficiency results.

The short-event calculation starts each run at full SOC and does not integrate
SOC or temperature over the few-second event. Endurance remains the prior
chronological, equal-reserve simulation, where each candidate's terminal power
ceiling was tuned independently up to 80 kW to finish 22 km with 0.500 kWh
remaining.

## Artifacts

- `battery_dynamic_points_summary.csv`: one row per pack, both models, all event
  scores, subtotals, totals, and ranks.
- `battery_dynamic_event_results_long.csv`: one row per pack/model/short event.
- `inherited_endurance_efficiency_scores.csv`: shared endurance and efficiency
  projections.
- `scoring_inputs.json`: formulas, anchors, and normalization inputs.
- `headline_summary.json`: best configurations and mass sensitivities.
- `validation_report.json`: automated execution checks.
- `plots/`: decision plots.
