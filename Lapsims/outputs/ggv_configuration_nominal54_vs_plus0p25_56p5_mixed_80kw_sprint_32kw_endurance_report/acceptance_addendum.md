# Two-configuration acceptance addendum

## Status

- Mixed-power scoring/report validation: **passed**.
- Physical terminal-prefix/topology sidecar: **passed**.
- Strict sampled-QSS production acceptance: **blocked**.
- Selected event solves: 8/8 converged with zero speed-cap segments.

The current score-model result is therefore suitable as a screening comparison,
not final production signoff.

## Result retained for screening

- Config 1, 11.5 in model-CG coordinate and 54% rear: **465.320627 / 575**.
- Config 2, +0.25 in and 56.5% rear: **455.001902 / 575**.
- Config 2 minus Config 1: **-10.318725 points**.

## Strict QSS blockers

1. Config 2 skidpad reconstructs the inside-front load at
   **99.4587659 N** versus the fitted-tire minimum of 100 N. The trim residual is
   `4.81e-13`; the issue is a small interpolation excursion outside the fitted
   load domain, not wheel lift.
2. Config 2 autocross has one maximum-lateral sample at 13.6881408 m/s with
   residual **0.00146663**, above the frozen `0.0001` gate. Direct continuation
   converges through approximately 14.166 m/s2, implying only about
   **0.0044 m/s2** of local envelope overstatement.
3. Config 1 32 kW endurance has one sampled inside-front load of
   **99.9952960 N**, 0.004704 N below the fitted-tire minimum.

No successful sampled state had wheel lift or exceeded the 1800 N fitted-tire
maximum. The Config 2 skidpad correction would reduce Config 2 performance, so
it cannot reverse the observed winner. A bounded strict-domain refresh is still
required before treating the exact point totals as signed off.

## Speed-domain serialization

- 80 kW maps contain the exact 0 through 42 m/s prefix; only the empty requested
  42.0539983 m/s terminal-RPM slice is omitted.
- 32 kW maps contain the exact 0 through 34 m/s physical prefix. The analytical
  drag-power terminal is **34.8953203 m/s**, and every omitted requested slice is
  above it.
- All tracks remain strictly inside their serialized map domains.

## Provenance

- Frozen source/input lock SHA-256:
  `518c2f954b12d3fedad9a0b17b36c684d50aaa9e933fe14a487fdd386c67863c`
- Terminal-prefix sidecar SHA-256:
  `3b92fc65a36d0b786c3888c28aeb4143ad23f60bc991ef48486f0c5917e4c9f2`
- Scoring/report provenance SHA-256:
  `2650ff1fd883653044bfe78584e5698f00e4c9e5170c12363730c8bff4ca559b`
