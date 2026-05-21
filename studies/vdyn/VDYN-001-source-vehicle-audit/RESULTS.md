# VDYN-001 Results

## Decision Question

What exact vehicle definition is the dynamics report allowed to use as its source of truth?

## Finding

**PASS:** the source vehicle definition is coherent enough to be the starting point for vehicle dynamics studies.

This study does not prove performance. It only establishes the source-of-truth vehicle that later performance studies are allowed to use.

## Source Vehicle Metrics

- Total audited mass: `261.073 kg`
- Audited CG: `x=-0.8003 m`, `y=0.0000 m`, `z=0.2796 m`
- Wheelbase: `1.5494 m`
- Front/rear track: `1.2122 m` / `1.2122 m`
- Static load split: `48.35 %` front / `51.65 %` rear
- Body torsional stiffness input: `300000 N*m/rad`

## Tire Source Metrics

- Tire template: `16x7p5_10_12psi`
- Unloaded tire radius: `0.2032 m`
- Nominal vertical load: `650.0 N`
- Vertical load range: `100.0` to `1800.0 N`
- Lateral relaxation coefficients: `PTY1=3.330134`, `PTY2=3.587160`

## Aero Reference Audit

- Front lower-inboard average: `[0.012699999999999996, 0.226314, 0.08001]`
- YAML front aero reference: `[0.0127, 0.226314, 0.08001]`
- Rear lower-inboard average: `[-1.4070203000000001, 0.2834894, 0.0870458]`
- YAML rear aero reference: `[-1.4070203, 0.2834894, 0.0870458]`
- Max aero reference error: `2.220446e-16 m`

![Mass breakdown](plots/mass_breakdown.png)

## What This Enables

- `VDYN-002` may use these mass, CG, wheelbase, track, tire, brake, power, and aero inputs for baseline envelope work.
- `AERO-001` may use the audited lower-inboard reference points as the starting point for aero map convention checks.
- `CHASSIS-001` may use this audit as the first hardpoint and mass-property source check, but should still add structural load-path detail.

## Correlation Closure

Before first dynamic correlation, measure total mass, corner weights, wheelbase, track, ride heights, and static alignment. If the built car differs materially, update the source YAML or explicitly document why design-intent values remain in the model.
