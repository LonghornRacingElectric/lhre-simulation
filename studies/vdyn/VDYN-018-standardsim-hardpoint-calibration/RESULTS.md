# VDYN-018 Results

## Finding

**SMOKE:** hardpoint calibration subset is defined and has been run through compiled StandardSim variants.

## Run Provenance

- Engine: `StandardSim`
- Requested vehicle configurations: `1`
- Compiled StandardSim models: `1`
- Successful TransientEval cases: `1`
- Max parallel builds: `1`
- Aggregate compile runtime: `223.03 s`
- Aggregate eval runtime: `27.39 s`
- Runtime: `250.76 s`

## Key Metrics

- Calibration sigma: `1.00 mm`
- Design cases written: `1`

## Design Implication

Use this compiled subset as the gate before claiming a 25000-case hardpoint response Monte Carlo. If the simplified mapping is weak, the large Monte Carlo may still support geometry/aero-reference tolerance, but not StandardSim response risk.
