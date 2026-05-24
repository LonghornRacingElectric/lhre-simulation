# DS-005 Tire Selection

Generated UTC: 2026-05-22T20:34:54+00:00

## Source of Results

All simulated response metrics in this report are from EnvelopeSim. No non-EnvelopeSim simulation output is used for the ranking.

## Tire Data Caveat

The Round 8 candidates preserve real lateral/vertical/alignment tire fits, but their pure longitudinal coefficients were copied from the current hybrid reference tire. Combined-slip coefficients were intentionally zeroed. Therefore the ranking emphasizes lateral EnvelopeSim capability; acceleration, braking, and combined-slip differences are not tire-selection evidence here.

## Recommendation

First-pass selection winner: **R25B 16x6_10 on 6in 14 psi** (`Round_8_Hoosier_R25B_16x6_10_on_6in_14psi_PAC02_UM2`), with EnvelopeSim lateral score 1.000.

This is a performance winner, not a final sign-off tire: EnvelopeSim loaded it 28.5% above the fit's stated maximum vertical load. Treat it as the first finalist, then confirm with a load-range-aware tire fit, StandardSim, or test data.

The strongest pattern is pressure and fit dependent, not just compound-name dependent. Use the top candidates as the next StandardSim/track-test shortlist, not as a final purchasing decision.

## Top EnvelopeSim Candidates

| Rank | Candidate | Score | Mean lat g | Lat 25 g | Mean area g^2 | Fz extrap? | Fz excess % |
| ---: | --- | ---: | ---: | ---: | ---: | --- | ---: |
| 1 | R25B 16x6_10 on 6in 14 psi | 1.0000 | 1.9817 | 2.3314 | 8.8579 | yes | 28.5 |
| 2 | R25B 16x7p5_10 on 8in 8 psi | 0.7878 | 1.9234 | 2.2343 | 8.7377 | yes | 28.3 |
| 3 | R25B 16x6_10 on 6in 8 psi | 0.7834 | 1.9234 | 2.2343 | 8.7131 | yes | 28.7 |
| 4 | R25B 16x7p5_10 on 7in 8 psi | 0.7585 | 1.9137 | 2.2343 | 8.6684 | yes | 28.5 |
| 5 | R25B 16x6_10 on 6in 10 psi | 0.7553 | 1.9137 | 2.2343 | 8.6455 | yes | 28.4 |
| 6 | R25B 16x7p5_10 on 8in 10 psi | 0.7545 | 1.9137 | 2.2343 | 8.6449 | yes | 28.6 |
| 7 | R25B 16x6_10 on 7in 10 psi | 0.7535 | 1.9137 | 2.2343 | 8.6415 | yes | 28.5 |
| 8 | LC0 16x7p5_10 on 7in 10 psi | 0.7329 | 1.9040 | 2.2343 | 8.6118 | yes | 28.4 |
| 9 | LC0 16x7p5_10 on 8in 10 psi | 0.7314 | 1.9040 | 2.2343 | 8.6070 | yes | 28.4 |
| 10 | R25B 16x6_10 on 7in 8 psi | 0.6626 | 1.8846 | 2.1857 | 8.6431 | yes | 28.5 |

## Fit-Range Confidence

Every top-ten Round 8 candidate exceeds its fitted Fz range in the current EnvelopeSim load case. That does not invalidate the directional ranking, but it lowers confidence in the absolute margin between the fastest candidates.

| Candidate | Fz max seen | Fit Fz max | Excess |
| --- | ---: | ---: | ---: |
| R25B 16x6_10 on 6in 14 psi | 1403.6 N | 1092.0 N | 28.5% |
| R25B 16x7p5_10 on 8in 8 psi | 1402.4 N | 1093.0 N | 28.3% |
| R25B 16x6_10 on 6in 8 psi | 1402.4 N | 1090.0 N | 28.7% |
| R25B 16x7p5_10 on 7in 8 psi | 1401.8 N | 1091.0 N | 28.5% |
| R25B 16x6_10 on 6in 10 psi | 1401.8 N | 1092.0 N | 28.4% |

## Current Reference Tire

| Item | Value |
| --- | ---: |
| Label | current hybrid reference |
| Mean lateral | 1.7971 g |
| Lat 25 m/s | 2.0886 g |
| Mean GGV area | 8.1934 g^2 |
| Fz max seen | 1397.4 N |

## Against Current Reference Tire

Relative to `vehicles/current/tires/16x7p5_10_12psi.tir`, the leading Round 8 candidates show lateral-envelope upside, but with lower confidence because the fitted Fz range is exceeded. Acceleration and braking deltas are intentionally excluded because the Round 8 longitudinal coefficients are fabricated from the reference tire.

| Rank | Candidate | Mean lat | Lat 25 m/s | Mean GGV area | Fz confidence |
| ---: | --- | ---: | ---: | ---: | --- |
| 1 | R25B 16x6_10 on 6in 14 psi | +10.3% | +11.6% | +8.1% | extrapolated |
| 2 | R25B 16x7p5_10 on 8in 8 psi | +7.0% | +7.0% | +6.6% | extrapolated |
| 3 | R25B 16x6_10 on 6in 8 psi | +7.0% | +7.0% | +6.3% | extrapolated |
| 4 | R25B 16x7p5_10 on 7in 8 psi | +6.5% | +7.0% | +5.8% | extrapolated |
| 5 | R25B 16x6_10 on 6in 10 psi | +6.5% | +7.0% | +5.5% | extrapolated |
| 6 | R25B 16x7p5_10 on 8in 10 psi | +6.5% | +7.0% | +5.5% | extrapolated |
| 7 | R25B 16x6_10 on 7in 10 psi | +6.5% | +7.0% | +5.5% | extrapolated |
| 8 | LC0 16x7p5_10 on 7in 10 psi | +5.9% | +7.0% | +5.1% | extrapolated |
| 9 | LC0 16x7p5_10 on 8in 10 psi | +5.9% | +7.0% | +5.0% | extrapolated |
| 10 | R25B 16x6_10 on 7in 8 psi | +4.9% | +4.7% | +5.5% | extrapolated |

## Group Reads

| Group | Value | N | Mean score | Best candidate |
| --- | --- | ---: | ---: | --- |
| compound | LC0 | 16 | 0.3390 | LC0 16x7p5_10 on 7in 10 psi |
| compound | R25B | 16 | 0.5823 | R25B 16x6_10 on 6in 14 psi |
| tire_size | 16x6_10 | 16 | 0.4527 | R25B 16x6_10 on 6in 14 psi |
| tire_size | 16x7p5_10 | 16 | 0.4685 | R25B 16x7p5_10 on 8in 8 psi |
| rim | 6in | 8 | 0.4850 | R25B 16x6_10 on 6in 14 psi |
| rim | 7in | 16 | 0.4406 | R25B 16x7p5_10 on 7in 8 psi |
| rim | 8in | 8 | 0.4762 | R25B 16x7p5_10 on 8in 8 psi |
| pressure_psi | 8.0 | 8 | 0.6425 | R25B 16x7p5_10 on 8in 8 psi |
| pressure_psi | 10.0 | 8 | 0.6676 | R25B 16x6_10 on 6in 10 psi |
| pressure_psi | 12.0 | 8 | 0.2701 | R25B 16x7p5_10 on 8in 12 psi |
| pressure_psi | 14.0 | 8 | 0.2623 | R25B 16x6_10 on 6in 14 psi |

## StandardSim Finalist Check

Not run in this pass. EnvelopeSim generated the selection ranking.

## Generated Files

- `outputs/candidate_registry.csv`
- `outputs/tire_characterization.csv`
- `outputs/envelope_metrics.csv`
- `outputs/candidate_scores.csv`
- `outputs/group_summary.csv`
- `outputs/standardsim_metrics.csv`
- `outputs/standardsim_errors.csv`
- `outputs/run_provenance.csv`
- `plots/envelope_score_rank.png`
- `plots/envelope_capability_map.png`
- `plots/pressure_trends_mean_lateral.png`

## Run Provenance

| Item | Value |
| --- | --- |
| Elapsed time | 53.3 s |
| Python | /tmp/lhre-sim-venv/bin/python |
