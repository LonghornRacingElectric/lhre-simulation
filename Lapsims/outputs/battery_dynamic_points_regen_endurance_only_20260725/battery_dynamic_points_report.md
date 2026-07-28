# Battery dynamic points: endurance-only refresh

## Scope

Scenario: `regen_endurance_only`.

Acceleration, skidpad, autocross, pack-aware full-SOC diagnostics, and the
mass-isolated sprint model are reused from `C:\Users\Abishek\Documents\LHR VMOD Stuff\References\lhre-simulation\Lapsims\outputs\battery_dynamic_points_20260725`. The long-form
sprint CSV is copied byte-for-byte (SHA-256 `83aab2a3c86d3111579b932bf0050d0d3139f81eadb3ff51de02d7bcaf0f86cd`), and all
sprint-invariant summary columns are required to remain exactly equal.

Only endurance raw time, net accumulator-terminal energy, the sustainable
endurance power limit, inherited endurance/efficiency scores, combined totals,
and their affected ranks are refreshed from `C:\Users\Abishek\Documents\LHR VMOD Stuff\References\lhre-simulation\Lapsims\outputs\battery_trade_study_regen_endurance_only_20260725\all_configuration_results.csv`.

## Headline results

- Highest refreshed pack-aware full dynamic score: **molicel_p30b_130s5p** at
  **675.000 / 675**.
- Baseline **molicel_p30b_130s5p**: **1242.199 s**,
  **5.915547 kWh net terminal energy**, and
  **33.727 kW** sustainable terminal
  power.
- The P30B baseline is remapped to the established 2026 endurance `Tmin`, and
  every candidate retains its refreshed endurance-time ratio to that baseline.
- The inherited efficiency projection is also recalculated relative to the
  refreshed P30B time and net terminal energy.
- Winning pack recovered terminal energy: **0.750609 kWh**; regen-created pack heat: **0.012940 kWh**; achieved active regen RMS: **8.671 kW**.


## Top refreshed configurations

| Candidate | Pack kg | Accel | Skidpad | Autox | Endurance | Efficiency | Total |
|---|---:|---:|---:|---:|---:|---:|---:|
| molicel_p30b_130s5p | 38.188 | 100.00 | 75.00 | 125.00 | 275.00 | 100.00 | 675.00 |
| ampace_jp30_125s5p | 39.062 | 100.00 | 74.85 | 124.93 | 275.00 | 100.00 | 674.78 |
| molicel_p50b_130s3p | 34.612 | 100.00 | 75.00 | 125.00 | 274.54 | 100.00 | 674.54 |
| tenpower_30xg_125s5p | 37.500 | 100.00 | 75.00 | 125.00 | 273.99 | 100.00 | 673.99 |
| molicel_p30b_125s5p | 36.719 | 99.69 | 75.00 | 125.00 | 273.75 | 100.00 | 673.45 |
| reliance_rs50_130s3p | 32.663 | 98.04 | 75.00 | 125.00 | 275.00 | 100.00 | 673.04 |
| molicel_p50b_135s3p | 35.944 | 100.00 | 75.00 | 125.00 | 275.00 | 97.87 | 672.87 |
| molicel_p50b_125s3p | 33.281 | 100.00 | 75.00 | 125.00 | 272.86 | 100.00 | 672.86 |
| tenpower_30xg_120s5p | 36.000 | 100.00 | 75.00 | 125.00 | 272.44 | 100.00 | 672.44 |
| reliance_rs50_125s3p | 31.406 | 97.42 | 75.00 | 125.00 | 275.00 | 100.00 | 672.42 |

## Artifacts

- `battery_dynamic_points_summary.csv`: refreshed one-row-per-pack results.
- `battery_dynamic_event_results_long.csv`: byte-identical sprint-event source.
- `inherited_endurance_efficiency_scores.csv`: refreshed shared scoring inputs.
- `endurance_efficiency_delta.csv`: previous-to-refreshed score and rank deltas.
- `scoring_inputs.json`: scenario, source paths, formulas, and hashes.
- `validation_report.json`: preservation and arithmetic checks.
- `headline_summary.json`, `plots/`, and `artifact_manifest.json`: decision
  outputs.
