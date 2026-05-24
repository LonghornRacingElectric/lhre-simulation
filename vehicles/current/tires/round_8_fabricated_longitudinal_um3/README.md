# Fabricated Longitudinal UM3 Tire Fits

Generated from the original zip archives in `tire_fits/`.

These files preserve each Round 8 lateral tire fit, copy only the pure
longitudinal Magic Formula coefficients from:

`vehicles/current/tires/16x7p5_10_12psi.tir`

Important caveats:

- The original Round 8 files were `USE_MODE = 2` lateral-only fits.
- These generated files are `USE_MODE = 3`, meaning uncombined steady
  `Fx, Fy, Mx, My, Mz` calculation.
- Longitudinal coefficients are fabricated by copying the reference tire fit.
- Combined-slip longitudinal coefficients are intentionally zeroed.
- Do not use these files to make tire-specific braking, drive, or combined-slip
  claims.

Generated files:

- `32` unique unarchived `.tir` files in this directory
- `36` source `.tir` entries processed
- `4` duplicate source entries folded into existing generated files
- `9` source zip archives processed
- `manifest.csv` with source-to-generated provenance
