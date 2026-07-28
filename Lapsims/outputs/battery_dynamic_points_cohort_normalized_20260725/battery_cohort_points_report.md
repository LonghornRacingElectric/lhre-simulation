# Battery dynamic points: simulated-cohort normalization

## What changed

This is a re-score of the same 128 completed simulations. No lap was
rerun and the competition-anchored output remains unchanged. For every timed
event, the fastest raw simulated configuration now defines that event's
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

Source: `C:\Users\Abishek\Documents\LHR VMOD Stuff\References\lhre-simulation\Lapsims\outputs\battery_dynamic_points_20260725\battery_dynamic_points_summary.csv`

## Cohort anchors

| Model | Event | Cohort Tmin | Max points | Fastest config |
|---|---|---:|---:|---|
| pack_aware | Acceleration | 4.250934 s | 100.0 | tenpower_30xg_130s4p |
| pack_aware | Skidpad | 4.758553 s | 75.0 | reliance_rs50_110s2p |
| pack_aware | Autocross | 51.884322 s | 125.0 | tenpower_30xg_115s3p |
| mass_isolated | Acceleration | 4.205352 s | 100.0 | reliance_rs50_110s2p |
| mass_isolated | Skidpad | 4.758553 s | 75.0 | reliance_rs50_110s2p |
| mass_isolated | Autocross | 51.704094 s | 125.0 | reliance_rs50_110s2p |
| shared | Endurance | 1236.396184 s | 275.0 | tenpower_60xg_125s3p |

### Efficiency anchors and eligibility

- Maximum permitted terminal energy:
  **6.776000 kWh**, from
  20.02 kgCO2/100 km * 22 km / 100 / 0.65 kgCO2/kWh.
- Overall endurance `Tmin`: **1236.396184 s**
  (tenpower_60xg_125s3p).
- Efficiency time limit: **1792.774467 s**
  (1.45 times overall endurance `Tmin`).
- Fastest eligible `Tmin`: **1244.122620 s**
  (tenpower_60xg_120s3p).
- Eligible minimum `Emin`: **2.903145020 kWh**
  (molicel_p30b_110s3p).
- `EFmin = 0.295479483` and
  `EFmax = 0.829488982`.
- 124 eligible configurations;
  4 energy-ineligible and
  0 time-ineligible.
- Energy-ineligible:
  tenpower_60xg_125s3p, reliance_rs50_110s4p, ampace_jp50_110s4p, tenpower_50xg_110s4p.

## Headline rankings

- **Recommended fully cohort-normalized winner:**
  **molicel_p50b_140s3p**,
  **589.937 /
  675**.
- Winner when retaining inherited competition-calibrated efficiency:
  **molicel_p50b_135s3p**,
  **656.072 /
  675**.

- Best pack-aware timed-event performance subtotal (efficiency excluded):
  **reliance_rs50_110s4p**,
  **565.720 /
  575**.
- Baseline **molicel_p30b_130s5p** scores
  **544.433 /
  575** timed-event points and
  **576.745 /
  675** with cohort efficiency versus
  **644.433 /
  675** with inherited efficiency.

## Recommended top 15 pack-aware configurations

| Rank | Candidate | Pack kg | Accel | Skidpad | Autox | Endurance | Cohort eff. | Recommended total | Inherited-eff. total |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | molicel_p50b_140s3p | 37.275 | 99.16 | 71.72 | 122.53 | 267.40 | 29.12 | 589.94 | 655.15 |
| 2 | molicel_p50b_135s3p | 35.944 | 99.37 | 71.95 | 122.79 | 262.92 | 31.76 | 588.78 | 656.07 |
| 3 | molicel_p50b_130s3p | 34.612 | 99.50 | 72.17 | 123.04 | 257.53 | 34.50 | 586.75 | 652.25 |
| 4 | molicel_p50b_110s4p | 39.050 | 98.59 | 71.43 | 122.17 | 269.47 | 24.96 | 586.61 | 648.54 |
| 5 | reliance_rs50_140s3p | 35.175 | 96.79 | 72.08 | 122.70 | 268.35 | 26.66 | 586.57 | 649.83 |
| 6 | ampace_jp50_135s3p | 36.450 | 95.91 | 71.86 | 122.39 | 266.39 | 29.87 | 586.41 | 652.21 |
| 7 | molicel_p50b_125s3p | 33.281 | 99.33 | 72.40 | 123.27 | 251.90 | 37.44 | 584.34 | 646.90 |
| 8 | tenpower_60xg_120s3p | 34.200 | 93.23 | 72.24 | 122.44 | 270.00 | 26.37 | 584.28 | 647.31 |
| 9 | reliance_rs50_135s3p | 33.919 | 96.35 | 72.29 | 122.88 | 263.28 | 29.28 | 584.08 | 649.42 |
| 10 | reliance_rs50_125s3p | 31.406 | 95.22 | 72.72 | 123.19 | 256.97 | 35.87 | 583.96 | 648.09 |
| 11 | tenpower_60xg_110s3p | 31.350 | 91.41 | 72.73 | 122.65 | 262.98 | 33.52 | 583.29 | 649.77 |
| 12 | ampace_jp50_140s3p | 37.800 | 96.33 | 71.63 | 122.19 | 266.59 | 26.50 | 583.25 | 646.38 |
| 13 | tenpower_30xg_125s5p | 37.500 | 99.12 | 71.68 | 122.49 | 254.63 | 34.61 | 582.54 | 647.93 |
| 14 | molicel_p50b_120s3p | 31.950 | 99.08 | 72.63 | 123.49 | 245.91 | 40.68 | 581.78 | 641.10 |
| 15 | reliance_rs50_130s3p | 32.663 | 95.83 | 72.50 | 123.04 | 258.09 | 32.04 | 581.50 | 649.01 |

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
