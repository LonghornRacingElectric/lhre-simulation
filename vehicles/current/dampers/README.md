# Current Vehicle Dampers

This folder stores damper source material for the current vehicle model.

## Files

- `dyno_ttx25_mkii_n_vs_mmps.pdf`: Ohlins TTX25 MkII force-vs-velocity dyno
  plots for low-speed and high-speed adjuster sweeps.

## Modeling Notes

The current `vehicle.yml` damper tables are simplified linear force-velocity
tables. The dyno PDF should be treated as the source artifact for replacing
those proxy tables with setting-specific, digressive damper curves.

The PDF is plotted, not stored as machine-readable CSV data, so the next step is
digitization into setting-specific compression/rebound tables before using it as
a quantitative StandardSim input.
