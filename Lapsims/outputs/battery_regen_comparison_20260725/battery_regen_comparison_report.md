# Battery Regen Comparison: regen_endurance_only_vs_no_regen

## Scope and conventions

This report compares 128 matched battery candidates by `candidate_id`. It reads versioned result artifacts and does not rerun either endurance study.

- Every `delta_*` column is `regen - prior`.
- Positive power gain, time reduction, recovered-energy gain, net-energy reduction, reserve change, and rank improvement are beneficial by definition.
- Positive gross-discharge, total-heat, regen-heat, and active-RMS changes indicate increases, not automatically benefits.
- Missing no-regen recovered energy, regen heat, and regen-active RMS are physically reconstructed as zero; missing no-regen gross discharge is reconstructed from its net terminal energy.

## Cohort-level result

- Sustainable power gain: median 4.359 kW, range 1.844 to 13.625 kW.
- Endurance time reduction: median 37.367 s, range 17.405 to 61.693 s.
- Recovered terminal energy gain: median 0.5697 kWh.
- Net terminal energy reduction: median 0.0813 kWh.
- Total pack heat change: median 0.0823 kWh; positive means more heat.
- Final reserve change: median 0.0002 kWh.

## Regen ranking

| Regen rank | Candidate | Prior rank | Rank improvement | Power gain (kW) | Time reduction (s) |
|---|---|---|---|---|---|
| 1 | reliance_rs50_110s4p | 2 | +1 | +13.625 | +21.532 |
| 2 | tenpower_60xg_125s3p | 1 | -1 | +12.469 | +19.551 |
| 3 | ampace_jp50_110s4p | 3 | -0 | +13.383 | +21.739 |
| 4 | tenpower_50xg_110s4p | 4 | -0 | +13.172 | +21.736 |
| 5 | tenpower_60xg_120s3p | 5 | -0 | +10.277 | +22.194 |
| 6 | reliance_rs50_140s3p | 7 | +1 | +11.117 | +24.544 |
| 7 | molicel_p50b_110s4p | 6 | -1 | +10.289 | +20.852 |
| 8 | ampace_jp50_140s3p | 9 | +1 | +11.008 | +24.885 |
| 9 | tenpower_50xg_140s3p | 11 | +2 | +10.797 | +24.832 |
| 10 | ampace_jp50_135s3p | 10 | -0 | +10.469 | +23.095 |

## Largest sustainable-power gains

| Candidate | Power gain (kW) | Time reduction (s) | Net energy reduction (kWh) | Pack heat change (kWh) |
|---|---|---|---|---|
| reliance_rs50_110s4p | +13.625 | +21.532 | +0.1654 | +0.1651 |
| ampace_jp50_110s4p | +13.383 | +21.739 | +0.1617 | +0.1641 |
| tenpower_50xg_135s3p | +13.320 | +30.627 | +0.1699 | +0.1640 |
| tenpower_50xg_110s4p | +13.172 | +21.736 | +0.1663 | +0.1624 |
| tenpower_60xg_125s3p | +12.469 | +19.551 | +0.2434 | +0.2360 |
| reliance_rs50_130s3p | +11.469 | +32.628 | +0.1578 | +0.1488 |
| ampace_jp50_130s3p | +11.422 | +33.060 | +0.1502 | +0.1494 |
| tenpower_50xg_130s3p | +11.266 | +32.930 | +0.1548 | +0.1479 |
| reliance_rs50_140s3p | +11.117 | +24.544 | +0.1522 | +0.1484 |
| ampace_jp50_140s3p | +11.008 | +24.885 | +0.1455 | +0.1484 |

## Interpretation cautions

- Rank changes combine the regen model with the study's existing equal-reserve constraint and ranking rules; they are not a standalone cell-selection recommendation.
- Compare `prior_simulation_fidelity` and `regen_simulation_fidelity` in the CSV if the study contains mixed native and resampled runs.
- Active RMS is computed only over the model's declared regen-active window, so it should not be compared with a whole-event RMS value.

## Metric source resolution

```json
{
  "prior": {
    "sustainable_power_kw": {
      "source_column": "power_limit_kw",
      "fallback": false
    },
    "endurance_time_s": {
      "source_column": "elapsed_time_s",
      "fallback": false
    },
    "gross_discharge_kwh": {
      "source_column": "terminal_energy_kwh",
      "fallback": true,
      "reason": "No-regen gross discharge equals net terminal energy"
    },
    "recovered_energy_kwh": {
      "source_column": null,
      "fallback": true,
      "constant": 0.0,
      "reason": "No-regen source has no regenerative contribution"
    },
    "net_terminal_energy_kwh": {
      "source_column": "terminal_energy_kwh",
      "fallback": false
    },
    "total_pack_heat_kwh": {
      "source_column": "pack_resistive_heat_kwh",
      "fallback": false
    },
    "regen_pack_heat_kwh": {
      "source_column": null,
      "fallback": true,
      "constant": 0.0,
      "reason": "No-regen source has no regenerative contribution"
    },
    "final_reserve_kwh": {
      "source_column": "remaining_usable_chemical_kwh",
      "fallback": false
    },
    "active_rms_kw": {
      "source_column": null,
      "fallback": true,
      "constant": 0.0,
      "reason": "No-regen source has no regenerative contribution"
    },
    "overall_rank": {
      "source_column": "overall_rank",
      "fallback": false
    },
    "cell_rank": {
      "source_column": "cell_rank",
      "fallback": false
    }
  },
  "regen": {
    "sustainable_power_kw": {
      "source_column": "power_limit_kw",
      "fallback": false
    },
    "endurance_time_s": {
      "source_column": "elapsed_time_s",
      "fallback": false
    },
    "gross_discharge_kwh": {
      "source_column": "terminal_discharge_energy_kwh",
      "fallback": false
    },
    "recovered_energy_kwh": {
      "source_column": "terminal_regenerated_energy_kwh",
      "fallback": false
    },
    "net_terminal_energy_kwh": {
      "source_column": "terminal_energy_kwh",
      "fallback": false
    },
    "total_pack_heat_kwh": {
      "source_column": "pack_resistive_heat_kwh",
      "fallback": false
    },
    "regen_pack_heat_kwh": {
      "source_column": "regen_pack_resistive_heat_kwh",
      "fallback": false
    },
    "final_reserve_kwh": {
      "source_column": "remaining_usable_chemical_kwh",
      "fallback": false
    },
    "active_rms_kw": {
      "source_column": "regen_active_rms_terminal_power_kw",
      "fallback": false
    },
    "overall_rank": {
      "source_column": "overall_rank",
      "fallback": false
    },
    "cell_rank": {
      "source_column": "cell_rank",
      "fallback": false
    }
  }
}
```
