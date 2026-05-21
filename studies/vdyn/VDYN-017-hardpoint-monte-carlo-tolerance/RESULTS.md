# VDYN-017 Results

## Finding

**PASS:** inboard hardpoint Monte Carlo variation has been translated into geometry and aero-reference tolerance bands.

This is the study's own geometry Monte Carlo. It is not an EnvelopeSim output and it is not a StandardSim output. Dynamic-response claims require measured hardpoints to be pushed back into the source vehicle model and rerun through the appropriate simulator.

## Run Provenance

- Engine: `study_geometry_monte_carlo`
- Compiled models: `0`
- Simulated geometry cases: `25000`
- Runtime: `14.75 s`

## Key Metrics

- Samples per tolerance level: `5000`
- Tolerance levels swept: `0.5, 1.0, 1.5, 2.0, 3.0 mm` combined machined-plus-welded coordinate sigma
- Largest passing tolerance by geometry thresholds: `1.0 mm` sigma (`+/-2.0 mm` approximate two-sigma band)
- At `1.0 mm` sigma, p95 aero reference z delta: `1.40 mm`
- At `1.0 mm` sigma, front aero-reference z mean/std: `+0.003` / `0.698 mm`
- At `1.0 mm` sigma, rear aero-reference z mean/std: `+0.009` / `0.709 mm`
- At `1.0 mm` sigma, p95 roll-center delta: `5.15 mm`
- At `1.0 mm` sigma, front roll-center mean/std: `-0.009` / `2.429 mm`
- At `1.0 mm` sigma, rear roll-center mean/std: `+0.025` / `2.638 mm`
- At `1.0 mm` sigma, p95 front/rear camber-gain delta: `8.03 %` / `6.87 %`
- At `1.0 mm` sigma, p95 inboard span delta: `2.75 mm`

![Tolerance summary](plots/tolerance_summary.png)

![Front aero-reference z delta histogram](plots/front_aero_ref_z_delta_hist.png)

![Front roll-center diagnostic histogram](plots/front_roll_center_delta_hist.png)

![Front camber-gain delta histogram](plots/front_camber_gain_delta_hist.png)

## Design Implication

The chassis tolerance target should be expressed statistically: hold combined machined-plus-welded frame-side suspension hardpoint coordinate variation near the largest passing geometry sigma, measure the built frame, update `vehicle.yml` with the measured hardpoints, then rerun EnvelopeSim or compile and rerun StandardSim variants before claiming a dynamic-response effect.
