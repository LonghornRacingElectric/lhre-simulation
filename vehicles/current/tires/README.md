# Current Vehicle Tires

Tire property files used by `vehicles/current/vehicle.yml`.

The current reference tire is:

- `16x7p5_10_12psi.tir`

## Round 9 Fitted Tires

`round_9_fitted_full_um14/` contains the current preferred MF5.2/PAC2002 tire
files generated from the Round 9 TTC RunGuide and Matlab SI data archives at
the repository root. These files use `USE_MODE = 14` and include overlay plots,
fit-quality tables, source manifests, and fit assumptions in the generated
folder.

These are the preferred tire-study candidates when combined-slip behavior is
needed. Pure lateral behavior is fit from each tire's own cornering data. The
16 inch Hoosier files use their own Round 9 lateral fits, with longitudinal and
combined-slip sections scaled from the nearest Hoosier 18.0x6.0-10 Round 9
drive/brake fit because the RunGuide does not provide 16 inch drive/brake runs.
The scaling preserves the donor longitudinal/lateral friction relationship
while matching the 16 inch tire's fitted lateral capability.

One Goodyear D2704 20.0x7.0-13 on 8 inch rim pressure case is missing 8 psi
cornering data in the available Round 9 channels. Its lateral coefficients are
linearly pressure-extrapolated from the same tire/rim's 10, 12, and 14 psi
fits, while its longitudinal and combined-slip coefficients are fit from the
available direct 8 psi drive/brake data. The generated manifest labels this
case explicitly.

`round_9_fitted_combined_um4/` is retained as the previous steady combined-slip
generation pass.

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

The generated Round 9 `round_9_fitted_full_um14/` files fit lateral relaxation
from the Round 9 free-rolling transient step-steer runs where those pressure
blocks exist. The 8 psi files, plus any missing transient pressure block, use a
same-tire/rim pressure extrapolation of the fitted lateral relaxation terms.
Longitudinal relaxation remains reference/radius/friction estimated because
the RunGuide does not provide an equivalent transient slip-ratio test. BobLib's
generator path should be checked before assuming every simulation model
consumes `.tir` PTX/PTY values as tire-specific transient-slip parameters.
