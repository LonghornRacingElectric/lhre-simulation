# Current Vehicle Tires

Tire property files used by `vehicles/current/vehicle.yml`.

The current reference tire is:

- `16x7p5_10_12psi.tir`

BobSim expects tire templates under
`BobSim/_0_Utils/external/BobLib/Generation/tire_templates/` when running from
the submodule. Stage the current vehicle inputs there with:

```bash
make sync-inputs
```

## Relaxation Length Fit

The lateral MF5.2 relaxation coefficients in `16x7p5_10_12psi.tir` come from
the saved outputs in `Analysis.ipynb` for `B2356run2.mat`; the notebook was not
rerun for this fit.

The fit uses the first two slip-angle sweeps and excludes the largest
slip-angle sweep because the force response is already nonlinear enough to bias
the first-order 63.2% timing metric low. With `FNOMIN = 650 N` and
`UNLOADED_RADIUS = 0.2032 m`, the fitted lateral relaxation model is:

```text
SigAlp0 = PTY1 * sin(2 * atan(Fz / (PTY2 * FNOMIN))) * UNLOADED_RADIUS
PTY1 = 3.330134
PTY2 = 3.587160
```

This gives `SigAlp0 ~= 0.35 m` at nominal load. Longitudinal relaxation
coefficients remain unset because the notebook output only supports a lateral
force relaxation fit.
