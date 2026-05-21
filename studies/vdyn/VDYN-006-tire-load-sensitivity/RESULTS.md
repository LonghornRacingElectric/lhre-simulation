# VDYN-006 Results

## Finding

**PASS:** the baseline tire file produces meaningful load-sensitive peak-force curves across the valid vertical-load range.

## Key Metrics

- Valid vertical-load range: `100` to `1800 N`
- Nominal-load mu_x/mu_y near `659 N`: `2.589` / `2.398`
- High-load mu_x/mu_y at `1800 N`: `1.503` / `1.795`
- High-load lateral peak force: `3231 N` per tire

![Load sensitivity](plots/load_sensitivity.png)

## Design Implication

LLTD, aero load, mass, and setup changes must be interpreted through tire load sensitivity. More normal load increases peak force but does not preserve the same coefficient of friction.
