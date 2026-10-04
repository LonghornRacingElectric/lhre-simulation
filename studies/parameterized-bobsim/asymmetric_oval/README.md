# Asymmetric oval parameter sweep

This study carries the three oval definitions and the 20-case anti/roll-center
grid from the [LHRe anti-geometry study](https://github.com/LonghornRacingElectric/lhre/tree/310e3487cd9ff62eadfde91a7d66c5a7096b28a6/simulation/studies/anti-geometry).
`oval.py` is copied from that commit's `simulation/tools/oval.py` (MIT license).
It keeps the two variable-radius corners, tangent straights, 4 m corridor,
0.5 m gate spacing, centerline driver target, and the source study's
1.3 g lateral / 1.3 g braking / 0.6 g drive limits.

The car is the published mechanical-only 2027 parameterized BobSim input. The
anti grid sets front anti-dive and rear anti-squat independently at 0, 20, 40,
and 60%. The roll-center controls set both axles to 0, 25, or 50 mm at zero
anti. `baseline` leaves the vehicle input unchanged. All cars share one QSS
speed target on each track. The forward model is BobSimParametric 6DOF, so its
results cannot reproduce the upstream hardpoint-based 14DOF study's tire-load
timing or wheel-hop results. Lap time here measures tracking of a shared driver
target; it is not an optimized performance comparison. The model has no current
aero map, tire relaxation, or physical force-path validation.

Run from the `lhre-simulation` root with Docker:

```sh
git submodule update --init --recursive BobSimParametric
docker compose build oval-study
docker compose run --rm -T oval-study
```

Select a small subset or change the output directory by overriding the
container command:

```sh
docker compose run --rm -T oval-study python ../studies/parameterized-bobsim/asymmetric_oval/run.py --case baseline --track template --output ../studies/parameterized-bobsim/asymmetric_oval/results_pilot
```

Use `--geometry-only` to inspect track lengths and the case grid without
loading the car. `--sample-period` changes the output interval; the default is
5 ms as in the source study. Driver limits can be changed explicitly with
`--ay-g`, `--brake-g`, and `--drive-g`. Each run writes `manifest.json` and
`summary.csv` to an ignored results directory. Every requested car/track pair
gets a row. An incomplete or out-of-domain case records the error and gives a
nonzero process exit after the remaining cases have been screened.

## Validation status

The source oval layout is reproduced: hairpin 196.55 m, template 195.98 m,
and sweeper 208.18 m. A full 20-car × 3-track screening run at 20 ms output
interval completed its case enumeration, but all 60 cases stopped at the
2027 input's provisional ±30 mm corner-jounce boundary. The baseline first
reported −34.0, −30.3, and −43.7 mm for hairpin, template, and sweeper.
These are attempted solver states outside the declared model domain, not
validated physical travel predictions. No lap or ranking is accepted from this
run. The next engineering gate is a physically justified travel/ride-height
domain and a feasible driver envelope, followed by transient lap verification.
