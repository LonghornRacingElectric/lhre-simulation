# DS-010 run.py on FourPostEval: Design

Date: 2026-09-25
Status: approved in chat; awaiting spec review

## Goal

Give DS-010 (anti-dive / anti-squat geometry) a `run.py` like the other
studies: given a study directory and a vehicle YAML, it runs all three stages
in `study.yml` and writes the study outputs. Every stage uses BobSim
FourPostEval (Modelica `BobLib.Standards.FourPostSim`). Nothing uses `dyn_py`.

The physics follows `studies/DS-010-anti-geometry/PITCH_ROLL_RATE_FORMULATION.md`
§4.2–4.3 (called "the formulation note" below).

## Scope

In scope:

- `studies/DS-010-anti-geometry/run.py`, `test_run.py`, updated `study.yml`
  and `README.md`.
- Outputs under the study directory and `reports/DS-010-anti-geometry.md`.

Out of scope:

- `dyn_py`, dynamic CPLV, lateral or combined loading, roll-centre work.
- Changes to BobSim or BobLib, or to the BobSim submodule pin.
- Writing over `vehicles/design/vehicle.yml`. The chosen vehicle goes to
  `outputs/` only.
- Steering kickback. The rig applies no steer, so it can't be measured.

## BobSim integration

- **Pin.** Keep BobSim at c45940e (BobLib 849566d). It has FourPostEval and
  `BobLib.Standards.FourPostSim` for `bellcrank_stabar` front and rear, which
  matches the design vehicle. Run `make init` once to check out the submodule.
- **Runtime.** Run inside `bobdyn/bobsim:c45940e` with the repo mounted:

  ```
  docker build -t bobdyn/bobsim:c45940e BobSim      # once, after make init
  docker run --rm -v "$PWD":/workspace -w /workspace bobdyn/bobsim:c45940e \
    python studies/DS-010-anti-geometry/run.py
  ```

  The pinned BobLib's `FourPostSim.mo` imports `Modelica.SIunits`, so it needs
  Modelica 3.2.3. `bobdyn/bobsim:latest` was built from a newer BobSim and
  only has Modelica 4.1.0, so the build fails there. The image built from the
  pinned `BobSim/Dockerfile` installs 3.2.3 under `/root`, so the container
  runs as root.

- **Build and run.** Same pipeline as DS-002:
  1. `render_record(vehicle, vehicle_path)` produces `variant.mo`.
  2. `omc` builds it with a `.mos` script using DS-002's `FOURPOST_CFG`
     (stop time 113 s, dassl).
  3. `FourPostEvalSim(config).run()` runs it.
- **Copied helpers.** DS-002's build helpers are copied rather than imported.
  DS-002's `run.py` imports `_4_OptSim.pipeline.steady_state_eval_report`,
  which moved at the current pin, so importing it fails. The copied helpers
  are `make_mos_content`, `find_executable`, `compile_variant`,
  `stage_variant_text`, `spring_rate_from_vehicle` and `FOURPOST_CFG`.
- **Setup loader.** The pinned `FourPostEvalSim.run()` calls `build_setup()`,
  which reads BobLib's own `Generation/vehicle.yml`. Before each run, `run.py`
  sets `four_post_eval_sim._load_active_vehicle_yaml` to return the variant
  vehicle, so the setup block describes the simulated car.
- **Lazy imports.** BobSim imports happen inside the functions that build or
  run the rig, so the pure functions can be tested without the submodule.
- **FourPostEval config.** `vehicle.mass` and `vehicle.h_cg` are the **total**
  mass and CG height from the mass roll-up. Tracks and wheelbase come from the
  wheel centres. Spring and ARB rates come from the vehicle YAML.
  - The rig holds each pose with actuators, so springs don't affect the slope
    c or the motion ratio it measures. Its spring rate is only used for
    FourPostEval's roll-stiffness metrics, which the scoring doesn't use.
- **Variant directory.** Each variant lives in `work/<stage>/<variant_id>/` and
  holds:
  - `vehicle.yml`, the modified vehicle
  - `variant.mo`
  - `build/FourPostEval/`
  - `fourpost_eval_config.yml`
  - `metrics.csv`, the rig summary
  - `series.csv`, the rig heave and roll series

  `--reuse` reads `metrics.csv` and `series.csv` when `variant.mo` is
  unchanged. Otherwise the variant is rebuilt.

## Inputs

### Vehicle YAML fields used

| Quantity | Field |
| --- | --- |
| Mass roll-up (m, h, CG x) | `sprung_mass`, `driver_mass`, `<axle>.masses` |
| Wheel centre, wishbone pickups | `<axle>.suspension.{upper,lower}_{fore_i,aft_i,o}_m`, `wheel_center_m` |
| Spring rate k_s | slope of `<axle>.actuation.shock.spring_table` |
| Tire rate k_t | `<axle>.tire.vertical_stiffness_n_per_m` |
| Loaded radius R | `<axle>.wheel.radius_m` |
| Brake bias β_f | `brake.front_bias` |
| Aero map | `aero.*` (ride-height grids, `downforce_table_n`, `my_table_nm`, `aero_ref_m`, `reference_speed_m_per_s`) |
| μ(Fz) | `.tir` named by `<axle>.tire.template`, under BobSim `_0_Utils/tire_templates`: `PDX1`, `PDX2`, `FNOMIN`, `LFZO`, `LMUX` |

### study.yml (replaces the `dyn_py` fields)

The independent variable is still dz/g, but each sweep is given as an anti %
range. `run.py` converts the range to evenly spaced dz/g targets at the
active axle rates, so the targets are reachable whatever the rates are. At
3 Hz with the design vehicle:

- rear anti-squat −20% to 90% is −12.7 to −3.2 mm/g
- front anti-dive −20% to 90% is −14.0 to −3.3 mm/g

```yaml
id: DS-010
title: Anti-Dive / Anti-Squat Geometry
status: planned
vehicle: vehicles/design/vehicle.yml
engine: BobSim/_3_StandardSim/FourPostEval/four_post_eval_sim.py
stages:
  - id: anti-squat
    maneuver: rwd_acceleration
    independent: [dz_rear_per_g]
    sweep_anti_pct: {min: -20, max: 90, points: 7}   # AS_r
  - id: anti-dive
    maneuver: braking
    independent: [dz_front_per_g, dz_rear_per_g]
    depends_on: anti-squat
    sweep_anti_pct: {min: -20, max: 90, points: 7}   # AD_f
    rear_sensitivity_neighbors: 1     # chosen rear +/- 1 sweep step
  - id: validation
    depends_on: anti-dive
config:
  fourpost:
    heave_magnitude_m: 0.03
    roll_magnitude_rad: 0.035
    force_magnitude_n: 1000.0
  assumptions:                        # provisional, flagged in RESULTS.md
    ride_frequency_hz: {front: 3.0, rear: 3.0}  # sets K; null = vehicle springs
    static_ride_height_m: {front: 0.07112, rear: 0.08382}  # aero grid anchor
    regen_share_of_rear_braking: 0.0
    sigma_fx_fraction: 0.10           # sigma_Fx = 0.10 * |mean Fx| per wheel
    scenarios:
      rwd_acceleration:
        speeds_m_per_s: [10, 15, 20, 25]
        ax_g: [0.3, 0.6, 0.9]
        weights: uniform
      braking:
        speeds_m_per_s: [10, 15, 20, 25]
        ax_g: [0.5, 1.0, 1.4]
        weights: uniform
  constraints:                        # provisional placeholders
    max_pickup_shift_mm: 30
    max_abs_toe_gain_deg_per_m: 4.0   # about 0.1 deg per 25 mm
    max_caster_change_deg: 2.0        # over the heave sweep
    anti_pct_range: [-20, 100]        # at every heave pose
  ic_check_tolerance: 0.02            # |c_rig - c_ic|
outputs:
  - RESULTS.md
  - ../../reports/DS-010-anti-geometry.md
```

`weights` is either `uniform` or an explicit 2-D list (speeds × ax_g), and is
normalised per maneuver.

## Physics

### Conventions

- Axes follow `vehicle.yml`: x forward, y left, z up.
- dz is the ride-height change at an axle, positive up. It is reported in
  mm per g of |a_x|.
- m and h are the **total** mass and CG height, including driver and unsprung
  mass. l is the wheelbase from the wheel centres.
- Axle rates are always `K_i = 2 * k_w,i` and `K_t,i = 2 * k_t,i`. The README's
  inversion only holds with these axle rates; per-wheel rates are off by 2×.
- `ΔW = m * g * |a_x| * h / l`.

### Closing K: ride-frequency hold (default)

The steady-state response depends on anti and springs only through
`(1 − anti)/K`. The scoring penalises anti (link-path load variation), but
nothing penalises stiffness, because road-input load variation is dynamic and
not modelled. With K free, the optimum would run to infinite K and zero anti.
K must therefore be fixed from outside. The default fixes it from a
ride-frequency target, which stands in for the ride trade-off the
steady-state model can't see.

With `ride_frequency_hz` set, ride rate means wheel rate in series with the
tire, per corner:

```
m_c,i  = sprung corner mass (sprung_mass + driver_mass, split by the sprung CG x)
k_rd,i = m_c,i * (2π f_i)^2                      ride rate
k_w,i  = k_rd,i * k_t,i / (k_t,i - k_rd,i)       wheel rate
k_s,i  = k_w,i * MR_i^2                          implied spring rate, per variant
```

- **Invalid target.** `run.py` stops with an error if `k_rd,i ≥ k_t,i`.
- **Constant K.** K does not depend on the hardpoints, so the chain
  dz/g → anti % → target instant centre → pivot rotations is solved in one
  pass.
- **Implied spring.** Each variant reports the spring rate `k_s,i` it needs.
  MR_i is the rig's `avg_motion_ratio_*`.

With `ride_frequency_hz: null`, `k_w,i = k_s,i / MR_i^2`, using the vehicle
YAML springs and each variant's rig MR. K then drifts slightly between
variants, so achieved dz/g can differ from the target.

With the design vehicle, the current springs give about 2.90 Hz front and
2.34 Hz rear. Holding 3 Hz raises the rear wheel rate by 81% (front by 9%),
which roughly halves anti-squat's effect on rear squat.

### Degrees of freedom

Once K and the swing-arm length d are fixed:

| | Unknowns | Equations | Result |
| --- | --- | --- | --- |
| Front | slope `c_f` | braking dz_f target | unique |
| Target instant centre → hardpoints | 2 pivot rotations | the centre's 2 coordinates | unique |
| Rear | slope `c_r` | acceleration squat target, braking lift target | overdetermined |

For the rear, stage 1 sets `c_r` from acceleration, and stage 2 reports the
braking consequence as a sensitivity. Holding d constant (the README's rule)
removes the one freedom that could meet both rear targets. That is kept as
is in this spec.

### Side-view slope c

c is the side-view slope from the contact patch to the instant centre (IC),
`e / d`. e is the IC height above the contact patch and d is its horizontal
distance from the wheel centre towards the middle of the car.

- **Rig value.** Read at the zero-heave pose. The rig series `fr_anti_vs_heave`
  and `rr_anti_vs_heave` are converted back to c with the h and l given in the
  FourPostEval config: `c = s * anti_pct / 100 * h_cfg / l_cfg`. The value is
  interpolated at heave = 0. The rig reports jacking / F_x for a single F_x
  pulse. A rearward brake force on an anti-dive front lifts the car, so the
  front sign is s = −1 and the rear is s = +1. At 90 % anti, the smoke run gave
  front c_ic +0.193 vs rig −0.192, and rear +0.162 vs +0.161.
- **Rig outputs not used for scoring.** The rig's `avg_anti_*_pct` leave out
  brake bias and depend on the h passed in.
- **Python value.** Each wishbone plane is defined by `fore_i`, `aft_i` and
  `o`. Intersecting it with the vertical plane `y = y_wc` gives a side-view
  line. The two lines meet at the IC. This also gives d. c_ic is computed
  for every variant and compared with c_rig.
- **Parallel arms.** The design vehicle's arms are effectively parallel in
  side view (front IC about 1,160 km behind the axle, rear about 270 m ahead),
  so d is unbounded or negative. The IC is kept in homogeneous coordinates:
  each side-view line is `(n_x, n_z, −c0)` and the IC is their cross product
  `(X, Z, W)`. Then `c = (Z − z_cp W) / (inward (X − x_wc W))` and
  `1/d = W / (inward (X − x_wc W))`, both finite when W = 0. Everywhere below,
  `R/d` is computed as `R · (1/d)`, and 1/d, not d, is the quantity held
  constant.

Anti percentages, from the formulation note §4.2:

```
AD_f = β_f * (l/h) * c_f
AL_r = (l/h) * [ s_fric * c_r + s_regen * (c_r - R_r/d_r) ]
       s_regen = regen_share * (1 - β_f),  s_fric = (1 - β_f) - s_regen
AS_r = (l/h) * (c_r - R_r/d_r)
```

### Ride-height response (formulation note §4.3)

```
braking:          dz_f = -ΔW * [ (1 - AD_f)/K_f + 1/K_t,f ]
                  dz_r = +ΔW * [ (1 - AL_r)/K_r + 1/K_t,r ]
RWD acceleration: dz_f = +ΔW * [ 1/K_f + 1/K_t,f ]
                  dz_r = -ΔW * [ (1 - AS_r)/K_r + 1/K_t,r ]
```

The inverse maps a target dz/g to a target anti %:

```
AD_f = 1 - K_f * (|dz_f|/ΔW - 1/K_t,f)
AS_r = 1 - K_r * (|dz_r|/ΔW - 1/K_t,r)
```

## Hardpoint variants

For each sweep point on an axle:

1. **Target anti % and dz/g.**
   - The anti % comes from the stage's `sweep_anti_pct`.
   - Its dz/g, the independent variable, follows from the ride-height response
     at the active K and K_t.
   - With the ride-frequency hold, K is exact for every variant. With vehicle
     springs, it uses the baseline MR.
2. **Target c.**
   - Front: `c = AD_f * h / (l * β_f)`.
   - Rear (drive): `c = AS_r * h / l + R_r / d_r`.
3. **Target IC.** `Q = (x_wc − d_0, z_cp + c * d_0)` for the front and
   `Q = (x_wc + d_0, z_cp + c * d_0)` for the rear, so the IC is always
   towards the middle of the car. d_0 is the baseline swing-arm length (held
   constant) and `z_cp = z_wc − R`. In homogeneous form, with `w = 1/d_0`,
   `Q = (x_wc w + inward, z_cp w + c, w)`; with parallel arms (w = 0) Q is the
   direction the two arms must share.
4. **Solve each wishbone** (upper and lower):
   - Let P0 be the point on the pivot axis (`fore_i`→`aft_i`) at `x = x_wc`.
   - Rotate `fore_i` and `aft_i` by δ about the y-direction through P0. This
     keeps their spacing and y coordinates.
   - Solve for δ so the arm's side-view line passes through Q. Use `brentq`
     in [−20°, 20°] on the residual `(n_x X + n_z Z − c0 W) / hypot(n_x, n_z)`.
   - If no root exists in that bracket, the variant is infeasible.
5. **Hold bump steer** by re-heighting the toe-link/rack inner pickup
   (`<axle>.steering.rack_pickup_m`); its x and y are unchanged.
   - A static first-order model of the corner gives toe rate per unit
     wheel-centre rise. It uses the wishbone rotations, the kingpin axis
     through the ball joints, and the toe-link length constraint.
   - Fixing the toe rate at the baseline value fixes the steer rate, so the
     toe-link constraint is linear in the pickup and its height has a closed
     form. If the link is tangent to the path it would need, the variant is
     infeasible.
   - Anti geometry adds fore-aft wheel motion in heave. The design car's toe
     links run fore-aft (0.25 m in x at the rear), so that motion couples
     directly into toe. With the pickup left in place, the rig gave −227 deg/m
     at 90 % anti-squat and −12 deg/m at 90 % anti-dive. Keeping its height
     fraction between the pivot axes gave +39 and −31 deg/m.
   - The model matches all of those rig values, and both baselines, within 2 %.
6. **Write the variant.** The four inboard wishbone pickups and the toe-link/rack
   inner pickup of that axle change, left side only (the renderer mirrors).
   Outboard joints, rod mount, springs, dampers and ARBs are unchanged.
7. **Record** δ_upper, δ_lower, the toe-pickup shift, the maximum pickup shift
   (mm, which includes the toe pickup) and the target IC.

Each variant is scored with what the rig measured (c_rig, MR), not with its
targets.

## Stages

0. **Baseline.** One rig run of the unmodified vehicle. It supplies d_0 (from
   the Python IC) and the baseline MR. MR gives the implied spring rates, and
   the targets too when `ride_frequency_hz` is null.
1. **anti-squat.** For each rear target, one rig run: rear variant, baseline
   front.
   - Score over the `rwd_acceleration` bins.
   - The chosen rear is the feasible variant with the highest objective.
   - If no variant is feasible, stop and report the constraint margins.
2. **anti-dive.** For each front target, one rig run: front variant, chosen
   rear.
   - Score over the `braking` bins.
   - Rear sensitivity: re-score each front variant with the chosen rear
     swapped for its ± `rear_sensitivity_neighbors` sweep neighbours.
   - Those neighbours use the rear rig results from stage 1, with no new
     runs. This is valid because the rig drives front and rear axles
     independently; validation checks it.
   - The chosen front is the feasible variant with the highest objective at
     the chosen rear.
3. **validation.** Rig runs of five cars:
   - the chosen front + chosen rear
   - the four sweep corners (front min/max × rear min/max)

   For each car, report:
   - achieved against target dz/g and anti %
   - achieved against the stage-1/2 combined prediction
   - anti % against heave across the sweep
   - constraint margins
   - c_rig against c_ic

   Write `outputs/chosen_vehicle.yml`.

Default cost is 1 + 7 + 7 + 5 builds. The chosen combined car can reuse a
stage-2 build.

## Scoring (objective B)

For each variant and each (speed v, a_x) bin of the stage's maneuver:

1. **Mass roll-up.** Static axle loads W_f and W_r.
2. **Ride heights.** Solve by fixed-point iteration:

   ```
   RH_i = RH_static,i + dz_i(a_x) - F_aero,i / K_ride,i
   K_ride,i = K_i * K_t,i / (K_i + K_t,i)
   ```

   - Aero comes from bilinear interpolation of the vehicle map at
     (RH_f, RH_r), scaled by `(v / v_ref)^2`.
   - The axle split uses `x_CoP = aero_ref_x + My / DF`, with front fraction
     `(x_CoP − x_rear) / l`, the same as `calibrate_orion_cop.fraction`.
   - Ride heights outside the grid are clamped to the grid edge, and the bin
     is flagged.
3. **Mean corner loads.** `F̄z = (W_i ± ΔW + F_aero,i) / 2`.
4. **Mean Fx per wheel.**
   - Braking: `β_f * m|a_x| / 2` front and `(1 − β_f) * m|a_x| / 2` rear.
   - RWD: `m|a_x| / 2` rear.
5. **Link-path load variation.**

   ```
   σFx   = sigma_fx_fraction * |F̄x|
   σFz   = |c_eff| * σFx
   c_eff = c              contact-patch force (braking friction)
         = c - R/d        wheel-centre force (drive, regen)
   ```

   For rear braking, c_eff is the s_fric / s_regen weighted mix.
6. **Grip.**

   ```
   μx(Fz)   = LMUX * (PDX1 + PDX2 * (Fz - Fz0) / Fz0),   Fz0 = FNOMIN * LFZO
   E[μx Fz] = μx(F̄z) * F̄z + LMUX * PDX2 / Fz0 * σFz^2      (exact; μx·Fz is quadratic)
   ```

7. **Sum over tires carrying Fx.** All four in braking, the two rears in RWD
   acceleration.
8. **Objective.** The bin-weighted sum over the maneuver's bins. It is also
   reported relative to the baseline vehicle.

## Constraints

A variant is feasible only if all of these hold:

- Pickup shift ≤ `max_pickup_shift_mm`.
- |toe gain in heave| ≤ `max_abs_toe_gain_deg_per_m`, from a linear fit of the
  rig's `*_toe_vs_heave`, on the axle being varied.
- Caster range over the heave sweep ≤ `max_caster_change_deg` (front).
- Anti % at every heave pose within `anti_pct_range`.
- The hardpoint solve found a root.

Every variant's constraint margins are reported. If `|c_rig − c_ic|` exceeds
`ic_check_tolerance`, the variant is flagged in the outputs; it is not
rejected, because scoring uses c_rig. If c_rig and c_ic both exceed
`ic_check_tolerance` in magnitude but have opposite signs, the run stops,
because d and the targets would be wrong. The baseline is near zero anti, so
only the swept variants can trigger this.

## CLI

```
python studies/DS-010-anti-geometry/run.py
    [--study DIR]          # default: this script's directory
    [--vehicle YML]        # default: study.yml `vehicle`, relative to repo root
    [--stage all|anti-squat|anti-dive|validation]   # default: all
    [--reuse]              # reuse cached builds and rig results in work/
    [--jobs N]             # variants built and run in parallel (default 3; ~1.3 GB, ~100 s each)
```

- **Stage dependencies.** `--stage anti-dive` reads the stage-1 choice from
  `outputs/anti_squat_summary.csv`. `--stage validation` reads both stage
  summaries. A missing dependency stops the run with the command to run
  first.
- **Output locations.** Output paths are relative to the study directory. The
  report path comes from study.yml `outputs`.
- **Startup checks.** `omc` must be available, and `require_dependencies()`
  gives the same pip hint as the other studies.

## Outputs

`outputs/`:

- `run_provenance.csv`: BobSim and BobLib commits, SHA-256 of the vehicle and
  study.yml, timestamp and CLI arguments.
- `variants.csv`: stage, variant_id, axle, target dz/g, target anti %,
  δ_upper, δ_lower, pickup shift and target IC.
- `fourpost_metrics.csv`: per variant:
  - c_rig and c_ic per axle
  - d
  - MR per axle
  - wheel rate `k_w` and implied spring rate `k_s` per axle
  - toe gain and caster range
  - the rig `avg_*` values
- `bin_scores.csv`: per variant × bin: RH_f and RH_r, aero DF and front
  fraction, F̄z and σFz per axle, grip, weight and a clamp flag.
- `anti_squat_summary.csv` and `anti_dive_summary.csv`: per variant:
  achieved dz/g, achieved anti %, objective, Δ objective against baseline,
  feasibility, constraint margins, and chosen flag. The anti-dive summary
  also carries a rear-sensitivity column.
- `validation.csv`: predicted against achieved for the chosen car and the
  corners.
- `chosen_vehicle.yml`: the chosen hardpoints. With the ride-frequency hold,
  each axle's `actuation.shock.spring_table` is replaced by the linear table
  `[[0, 0], [1, k_s]]` at the implied spring rate, with a comment giving the
  target frequency.

`plots/`:

- Objective against dz_r/g (stage 1).
- Objective against dz_f/g, one line per rear sensitivity value (stage 2).
- Anti % against heave for the validation cars.
- Ride height against speed, chosen car against baseline, per maneuver.

`RESULTS.md` and `reports/DS-010-anti-geometry.md`:

- Chosen dz/g and anti % per axle, and the hardpoint changes.
- Objective gain against baseline.
- Validation deltas and constraint margins.
- The assumptions block below, with each assumption marked provisional.

## README changes

- **Engine.** Replace with FourPostEval, and the run command above. Remove
  "Required dyn_py extensions".
- **Approach 1.** Aero now comes from the vehicle YAML's map, not a quadratic
  surrogate chart. A new map is a rerun.
- **Approach 2.** The inversion uses axle rates, and drive anti-squat uses the
  wheel-centre force line.
- **Objective.** Describe the CPLV stand-in (`σFz = |c_eff|·σFx`).
- **Held Fixed.** Replace "Springs" with "Ride frequency (3 Hz front and
  rear); spring rates follow from each variant's motion ratio".
- **How to run.** Add a short section.

## Testing

`studies/DS-010-anti-geometry/test_run.py` (pytest) loads `run.py` with
`importlib`. It needs no BobSim checkout.

1. Zero rotation returns identical hardpoints. A solved δ keeps pivot spacing
   and y coordinates, and puts the side-view line through Q (residual below
   1e-9 m).
2. The Python IC of a hand-computable synthetic double wishbone matches its
   analytic IC.
3. The mass roll-up of `vehicles/design/vehicle.yml` gives m = 261.07 kg,
   h = 0.2796 m and a = 0.8003 m (formulation note §6).
4. With §6 inputs, the ride-height response gives:
   - ΔW = 462.2 N/g
   - zero anti: front drop 12.9 mm/g, rear rise 18.0 mm/g
   - 30% anti: 9.7 mm/g and 13.3 mm/g

   The dz → anti → dz round trip is exact.
5. The ride-frequency hold at 3 Hz, with §6 masses and k_t = 98,947 N/m,
   gives:
   - sprung corner mass: 54.0 kg front, 59.2 kg rear
   - wheel rate: 23,811 N/m front, 26,706 N/m rear
   - squat or dive at zero anti: 11.0 mm/g rear, 12.0 mm/g front
   - the sweep endpoints (−20% and 90% anti): −12.7 and −3.2 mm/g rear
     (acceleration), −14.0 and −3.3 mm/g front (braking)

   The §6 wheel rates give 2.90 Hz front and 2.34 Hz rear. With `null`, test 4's
   numbers come back unchanged. If `k_rd ≥ k_t`, the run stops with an error.
6. The closed-form `E[μx Fz]` matches a Monte Carlo estimate (10^6 samples,
   relative error below 1e-3).
7. The aero front fraction at the middle grid point is 0.5, the calibration
   target.
8. The ride-height fixed point converges, and out-of-grid bins are clamped and
   flagged.
9. CLI path resolution: default study directory, study.yml `vehicle` relative
   to the repo root, and the report path from `outputs`.

End-to-end check: run the baseline and one rear variant in Docker. Confirm
c_rig and c_ic agree within tolerance with the same sign, and that `--reuse`
skips the rebuild.

## Provisional assumptions and known limits

- **Aero map.** `AEROMAP_VALIDATION.md` says not to select anti geometry from
  this map's balance. The calibrated CoP is a provisional prior, so results
  depend on it until Aero confirms the map. `run.py` uses whatever map the
  vehicle YAML holds.
- **Static ride height.** Taken as the aero grid anchor, which
  `AEROMAP_COP_CALIBRATION.md` says is not asserted to be the static ride
  height.
- **Rear vertical datum.** Unresolved (`vehicle.datum.json`), which affects h,
  IC heights and R/d.
- **Scenario weights.** Uniform. Nothing in the repo gives lap-time fractions.
  A Lapsims Michigan endurance trace could supply them later.
- **σFx.** 10% of the mean Fx is a placeholder for the longitudinal force
  variation.
- **Regen share of rear braking.** 0. The vehicle YAML doesn't define it.
- **Ride frequency.** 3 Hz front and rear, as wheel rate in series with the
  tire. It replaces the vehicle's placeholder springs, and the implied spring
  rates are written to `chosen_vehicle.yml`. With `null`, the vehicle springs
  are used instead.
- **Constraint limits.** Placeholders. Kickback isn't checked.
- **Quasi-static model.** No damper or frequency content. The CPLV term only
  stands in for the rigid-link load path.
- **Loaded radius.** Taken as `wheel.radius_m`, ignoring static tire
  deflection (about 6 mm).
- **Drag.** a_x is the total longitudinal acceleration, and drag's line of
  action is ignored in ΔW.
