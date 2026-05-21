# VDYN-012 Results

## Finding

**PASS:** aero scaling has different effects on lateral capability and acceleration capability, so aero must remain a vehicle dynamics variable.

## Key Metrics

- 25 m/s downforce-scale lateral span: `1.573` to `2.347 g`
- 25 m/s drag-scale acceleration span: `0.980` to `1.111 g`

![Aero scaling](plots/aero_scaling_25mps.png)

## Design Implication

Downforce, drag, and aero balance must be correlated separately. A single aero performance number is not enough.
