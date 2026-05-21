# AERO-001 Results

## Decision Question

Is the aero map referenced to defensible vehicle geometry, dimensions, and sign conventions before aero performance claims are made?

## Finding

**PASS:** the aero map is coherent enough to proceed to platform sensitivity studies.

This study does not claim the aero package is optimal. It only verifies that the map can be interpreted and cited.

## Reference And Shape Checks

- Front/rear ride-height grids monotonic: `True`
- Force/moment table shapes valid: `True`
- Front reference error: `3.469447e-18 m`
- Rear reference error: `2.220446e-16 m`

## Baseline Map Point

- Baseline front/rear ride height: `0.03556 m` / `0.04191 m`
- Downforce/drag at 15 m/s: `323.5 N` / `161.6 N`
- ClA/CdA: `2.347 m^2` / `1.173 m^2`
- Downforce/drag ratio: `2.002`

## Map Ranges

- Downforce range: `163.4` to `351.3 N`
- Drag range: `154.5` to `180.1 N`
- Equivalent vertical-load x range from pitch moment: `-3.395` to `-1.553 m`

![Downforce map](plots/downforce_map.png)

![Drag map](plots/drag_map.png)

## What This Enables

`AERO-002-platform-sensitivity` may now evaluate ride-height, rake, downforce, and drag effects with the map references and dimensions established.

## Correlation Closure

Use coastdown, aero-on/off, and ride-height-vs-speed testing to validate drag and load trends. Convert pitch moment into a front/rear load split before making final balance claims.
