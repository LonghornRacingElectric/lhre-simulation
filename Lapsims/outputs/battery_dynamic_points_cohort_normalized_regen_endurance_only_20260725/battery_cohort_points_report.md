# Battery dynamic points: simulated-cohort normalization

## What changed

This scoring step reads and re-scores 128 completed source rows; it
does not run a lap solver. The source may be the original dynamic study or a
versioned endurance-only refresh that reused its sprint results. For every
timed event, the fastest raw simulated configuration now defines that event's
`Tmin` and receives the official maximum score. All slower configurations are
scored directly from that raw time using the 2026 Formula SAE equation and the
event's official `Tmax/Tmin` factor.

Acceleration, skidpad, and autocross receive separate cohort anchors for the
pack-aware and mass-isolated sprint models. Endurance uses the fastest raw 22 km
time across the cohort.

Two efficiency interpretations are included:

1. **Recommended internal-design score:** strict 2026 EV eligibility followed
   by cohort normalization of efficiency factor. This makes every component of
   the 675-point total relative to the same simulated design cohort.
2. **Inherited comparison score:** the prior competition-calibrated efficiency
   projection is preserved unchanged, along with its original total and rank.

Source: `C:\Users\Abishek\Documents\LHR VMOD Stuff\References\lhre-simulation\Lapsims\outputs\battery_dynamic_points_regen_endurance_only_20260725\battery_dynamic_points_summary.csv`

## Cohort anchors

| Model | Event | Cohort Tmin | Max points | Fastest config |
|---|---|---:|---:|---|
| pack_aware | Acceleration | 4.250934 s | 100.0 | tenpower_30xg_130s4p |
| pack_aware | Skidpad | 4.758553 s | 75.0 | reliance_rs50_110s2p |
| pack_aware | Autocross | 51.884322 s | 125.0 | tenpower_30xg_115s3p |
| mass_isolated | Acceleration | 4.205352 s | 100.0 | reliance_rs50_110s2p |
| mass_isolated | Skidpad | 4.758553 s | 75.0 | reliance_rs50_110s2p |
| mass_isolated | Autocross | 51.704094 s | 125.0 | reliance_rs50_110s2p |
| shared | Endurance | 1216.026125 s | 275.0 | reliance_rs50_110s4p |

### Efficiency anchors and eligibility

- Maximum permitted terminal energy:
  **6.776000 kWh**, from
  20.02 kgCO2/100 km * 22 km / 100 / 0.65 kgCO2/kWh.
- Overall endurance `Tmin`: **1216.026125 s**
  (reliance_rs50_110s4p).
- Efficiency time limit: **1763.237882 s**
  (1.45 times overall endurance `Tmin`).
- Fastest eligible `Tmin`: **1216.026125 s**
  (reliance_rs50_110s4p).
- Eligible minimum `Emin`: **2.852480038 kWh**
  (molicel_p30b_110s3p).
- `EFmin = 0.290322847` and
  `EFmax = 0.842938351`.
- 128 eligible configurations;
  0 energy-ineligible and
  0 time-ineligible.
- Energy-ineligible:
  .

## Headline rankings

- **Recommended fully cohort-normalized winner:**
  **reliance_rs50_110s4p**,
  **590.209 /
  675**.
- Winner when retaining inherited competition-calibrated efficiency:
  **reliance_rs50_130s3p**,
  **657.036 /
  675**.

- Best pack-aware timed-event performance subtotal (efficiency excluded):
  **reliance_rs50_110s4p**,
  **566.476 /
  575**.
- Baseline **molicel_p30b_130s5p** scores
  **549.630 /
  575** timed-event points and
  **582.514 /
  675** with cohort efficiency versus
  **649.630 /
  675** with inherited efficiency.

## Recommended top 15 pack-aware configurations

| Rank | Candidate | Pack kg | Accel | Skidpad | Autox | Endurance | Cohort eff. | Recommended total | Inherited-eff. total |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | reliance_rs50_110s4p | 36.850 | 97.23 | 71.79 | 122.45 | 275.00 | 23.73 | 590.21 | 649.71 |
| 2 | reliance_rs50_130s3p | 32.663 | 95.83 | 72.50 | 123.04 | 265.66 | 32.90 | 589.93 | 657.04 |
| 3 | reliance_rs50_140s3p | 35.175 | 96.79 | 72.08 | 122.70 | 270.96 | 26.91 | 589.44 | 651.59 |
| 4 | molicel_p50b_125s3p | 33.281 | 99.33 | 72.40 | 123.27 | 255.93 | 38.31 | 589.24 | 650.93 |
| 5 | reliance_rs50_135s3p | 33.919 | 96.35 | 72.29 | 122.88 | 267.37 | 29.49 | 588.38 | 652.69 |
| 6 | ampace_jp50_135s3p | 36.450 | 95.91 | 71.86 | 122.39 | 268.01 | 29.94 | 588.11 | 652.80 |
| 7 | molicel_p50b_140s3p | 37.275 | 99.16 | 71.72 | 122.53 | 265.35 | 29.15 | 587.92 | 651.95 |
| 8 | molicel_p50b_110s4p | 39.050 | 98.59 | 71.43 | 122.17 | 269.69 | 26.01 | 587.89 | 649.29 |
| 9 | molicel_p50b_135s3p | 35.944 | 99.37 | 71.95 | 122.79 | 261.79 | 31.70 | 587.60 | 653.77 |
| 10 | molicel_p50b_120s3p | 31.950 | 99.08 | 72.63 | 123.49 | 250.77 | 41.45 | 587.42 | 645.97 |
| 11 | tenpower_30xg_120s5p | 36.000 | 99.35 | 71.94 | 122.78 | 255.53 | 37.67 | 587.26 | 649.59 |
| 12 | tenpower_60xg_125s3p | 35.625 | 93.95 | 72.00 | 122.29 | 274.46 | 24.54 | 587.25 | 647.43 |
| 13 | reliance_rs50_120s3p | 30.150 | 94.51 | 72.93 | 123.32 | 257.13 | 39.15 | 587.04 | 647.88 |
| 14 | ampace_jp50_110s4p | 39.600 | 96.77 | 71.33 | 121.92 | 273.38 | 23.52 | 586.92 | 646.24 |
| 15 | ampace_jp50_130s3p | 35.100 | 95.40 | 72.09 | 122.56 | 264.19 | 32.67 | 586.92 | 653.90 |

## Scoring details

- Acceleration: 100 maximum, 4.5 completion, `Tmax = 1.50 * Tmin`.
- Skidpad: 75 maximum, 3.5 completion, `Tmax = 1.25 * Tmin`, squared
  time-ratio equation.
- Autocross: 125 maximum, 6.5 completion, `Tmax = 1.45 * Tmin`.
- Endurance: 275 maximum, 25 completion, `Tmax = 1.45 * Tmin`.
- Timed-event performance subtotal: 575 points.
- Cohort efficiency:
  `EF=(eligible_Tmin/T)*(eligible_Emin/E)` and
  `points=clip(100*(EF-EFmin)/(EFmax-EFmin),0,100)`. Ineligible
  configurations receive zero efficiency points.
- Inherited competition-calibrated efficiency: retained as a comparison,
  up to 100 points.
- Full dynamic total: 675 points.

The pack-aware result is the decision model: it includes pack mass plus the
pack's full-SOC voltage, resistance, continuous-current limit, and the common
80 kW battery-terminal ceiling. The mass-isolated result is a diagnostic that
holds the 80 kW tractive curve constant and changes mass only. Because each
sprint model has its own fastest candidate, compare ranks and point losses
within a model; do not interpret their absolute cross-model point difference as
a physical powertrain penalty.

## Files

- `battery_cohort_points_summary.csv`: one row per configuration with all
  cohort scores, totals, and ranks.
- `battery_cohort_event_results_long.csv`: seven timed-event rows per
  configuration (three events in each sprint model plus shared endurance).
- `inherited_efficiency_scores.csv`: explicit unchanged efficiency inputs.
- `cohort_efficiency_scores.csv`: terminal energy, strict eligibility, cohort
  factor, and cohort efficiency points.
- `cohort_scoring_inputs.json`: formulas, cohort anchors, and policy.
- `headline_summary.json`: decision-ready winners and baseline.
- `validation_report.json`: automated score and arithmetic checks.
- `plots/`: cohort-normalized decision plots.
