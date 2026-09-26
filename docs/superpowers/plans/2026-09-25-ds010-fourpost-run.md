# DS-010 FourPostEval run.py Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give DS-010 a `run.py` that runs all three stages in `study.yml`
(anti-squat, anti-dive, validation) as BobSim FourPostEval rig runs, given a
study directory and a vehicle YAML.

**Architecture:** One `run.py` following the DS-00X pattern.
- Pure functions (mass roll-up, rates, anti and ride-height response, side-view
  IC, pivot rotation, tire grip, aero map, bin scoring) import only numpy,
  scipy and yaml. `test_run.py` tests them with no BobSim checkout.
- BobSim (`render_record`, `FourPostEvalSim`) is imported lazily inside the rig
  runner. Stage functions chain the rig results into scores and outputs.

**Tech Stack:** Python 3.11, numpy, scipy (`brentq`), PyYAML, matplotlib,
pytest. OpenModelica `omc` and BobSim c45940e (BobLib 849566d) run inside the
`bobdyn/bobsim:c45940e` Docker image, built from the pinned `BobSim/Dockerfile`
(Modelica 3.2.3; `latest` has only 4.1.0).

**Implementation note:** the design vehicle's side-view arms are effectively
parallel, so every `d`/`d_r`/`d0` in the interfaces below is carried as its
reciprocal `inv_d` (1/m), with the IC in homogeneous coordinates. See the spec's
"Parallel arms" paragraph.

**Spec:** `docs/superpowers/specs/2026-09-25-ds010-fourpost-run-design.md`

## Global Constraints

- BobSim pin stays at c45940e; no BobSim or BobLib edits.
- Everything comes from `study.yml` and the vehicle YAML; no stashed DS-010 code.
- `vehicles/design/vehicle.yml` is never written; the chosen vehicle goes to `outputs/chosen_vehicle.yml`.
- `g = 9.80665 m/s²` (matches FourPostEval).
- Axle rates `K = 2*k_w`, `K_t = 2*k_t`; `ΔW = m*g*|a_x|*h/l` with total m and h.
- Variant directories are `work/<stage>/<variant_id>/`.
- Commits happen only when the user asks (session instruction); commit steps below are skipped.

## Review Focus

1. **No root in ±20° for a wishbone rotation.** The variant is marked infeasible and skipped; the run continues. Test: `test_solve_rotation_no_root_returns_none`.
2. **Ride height leaves the aero grid.** The value is clamped to the grid edge, the bin is flagged, and the score is still finite. Test: `test_ride_height_clamped_and_flagged`.
3. **`ride_frequency_hz: null`.** Axle rates fall back to `k_s/MR²` and reproduce formulation note §6. Test: `test_null_frequency_reproduces_section6`.
4. **Ride-frequency target above the tire rate.** The run stops with a clear error instead of producing a negative wheel rate. Test: `test_frequency_above_tire_rate_raises`.
5. **`--stage anti-dive` before stage 1 has run.** The run stops and prints the command to run first. Test: `test_missing_stage_dependency_message`.

---

### Task 1: study.yml, CLI and path resolution

**Files:**
- Modify: `studies/DS-010-anti-geometry/study.yml` (replace with the spec's block)
- Create: `studies/DS-010-anti-geometry/run.py`
- Create: `studies/DS-010-anti-geometry/test_run.py`

**Interfaces:**
- Produces: `StudyPaths(study_dir, repo_root, vehicle, outputs, plots, work, report_paths)`; `resolve_paths(study_dir: Path, vehicle_arg: str | None, study: dict) -> StudyPaths`; `load_study(study_dir) -> dict`; `parse_args(argv) -> Namespace`; `stage_dependency_error(stage_id, summary_path) -> str`.

- [ ] Write tests: default study dir is `run.py`'s directory; `vehicle` from study.yml resolves against the repo root; `--vehicle` overrides it; report paths come from `outputs` relative to the study dir; the missing-dependency message names the file and the command.
- [ ] Run them (fail), implement, run them (pass).

### Task 2: Mass roll-up, ride-frequency hold, anti and ride-height response

**Interfaces:**
- Produces: `MassProps` (m, h, x_cg, a, l, x_front, x_rear, m_s, x_s, h_s); `mass_rollup(vehicle) -> MassProps`; `static_axle_loads(mp) -> (W_f, W_r)`; `sprung_corner_mass(mp, axle)`; `wheel_rate_for_frequency(m_c, f_hz, k_t)`; `ride_frequency(m_c, k_w, k_t)`; `delta_w(mp, ax_g)`; `anti_dive(c_f, beta_f, mp)`; `anti_lift(c_r, d_r, R, beta_f, regen_share, mp)`; `anti_squat(c_r, d_r, R, mp)`; `c_for_anti_dive`, `c_for_anti_squat`; `dz_braking(dW, AD_f, AL_r, K_f, Kt_f, K_r, Kt_r) -> (dz_f, dz_r)`; `dz_accel(dW, AS_r, ...) -> (dz_f, dz_r)`; `anti_from_dz(dz_abs, dW, K, Kt)`.

- [ ] Tests (§6 numbers): m = 261.07 kg, h = 0.2796 m, a = 0.8003 m; ΔW = 462.2 N/g; zero-anti braking 12.9/18.0 mm/g; 30% anti 9.7/13.3 mm/g; dz → anti → dz round trip; 3 Hz corner masses 54.0/59.2 kg, wheel rates 23,811/26,706 N/m; 11.0 mm/g rear squat and 12.0 mm/g front dive at zero anti; sweep endpoints −12.7/−3.2 and −14.0/−3.3 mm/g; §6 wheel rates → 2.90/2.34 Hz; `k_rd ≥ k_t` raises.
- [ ] Fail, implement, pass.

### Task 3: Side-view IC and pivot-axis rotation

**Interfaces:**
- Produces: `side_view_line(fore, aft, outer, y) -> (nx, nz, c)`; `side_view_ic(susp, y) -> (x, z)`; `axle_geometry(vehicle, axle) -> AxleGeometry(c, d, e, x_wc, z_wc, z_cp, R, ic)`; `rotate_pivot(fore, aft, x_wc, delta) -> (fore', aft')`; `solve_rotation(fore, aft, outer, y, x_wc, Q) -> float | None`; `make_hardpoint_variant(vehicle, axle, target_c, d0) -> (vehicle', info)`.
- `inward = -1` front, `+1` rear; `d = inward*(x_ic − x_wc)`; `e = z_ic − z_cp`; `c = e/d`.

- [ ] Tests: zero rotation is an identity; solved δ keeps spacing and y and puts the line through Q (< 1e-9 m); synthetic wishbone IC matches the analytic IC; `make_hardpoint_variant` hits the target c through `axle_geometry` and leaves the outboard and rod points unchanged; an unreachable Q returns `None`. The toe-link/rack inner pickup is re-heighted so the static toe rate stays at baseline, and the corner model's toe rate is pinned to the FourPostEval smoke values. (Added after the smoke run: with the pickup left in place, the 90 % anti variants gave -227 and -12 deg/m.)
- [ ] Fail, implement, pass.

### Task 4: Tire grip, aero map, ride-height fixed point, bin scoring

**Interfaces:**
- Produces: `TireParams(pdx1, pdx2, fz0, lmux)`; `read_tire_params(path)`; `mu_x(fz, tp)`; `expected_grip(fz_mean, sigma, tp)`; `AeroMap.from_vehicle(vehicle)` with `.evaluate(rh_f, rh_r, v) -> (df, front_frac, clamped)`; `solve_ride_heights(...) -> (rh_f, rh_r, df, frac, clamped, iterations)`; `AxleState(c, d, R, K, Kt)`; `score_bin(maneuver, v, ax_g, front: AxleState, rear: AxleState, ctx) -> dict`; `score_maneuver(...) -> (objective, rows)`; `scenario_bins(cfg) -> list[(v, ax_g, w)]`.

- [ ] Tests: closed form matches Monte Carlo (10^6 samples, rel err < 1e-3); middle-grid front fraction is 0.5; fixed point converges and matches a direct re-evaluation; out-of-grid heights are clamped and flagged; uniform weights sum to 1.
- [ ] Fail, implement, pass.

### Task 5: Rig integration (BobSim)

**Interfaces:**
- Copied from DS-002: `FOURPOST_CFG`, `make_mos_content`, `find_executable`, `compile_variant`, `stage_variant_text`, `spring_rate_from_vehicle`.
- Produces: `make_fourpost_config(vehicle, mp, build_dir, metrics_csv, fourpost_cfg) -> dict`; `run_rig(variant_dir, vehicle, fourpost_cfg, reuse) -> RigResult(summary, series)`; `write_series_csv` / `read_series_csv`; `rig_c(series, axle, h_cfg, l_cfg)`; `rig_metrics(result, axle) -> dict` (toe gain deg/m, caster range deg, anti-vs-heave).
- Lazy BobSim imports; `four_post_eval_sim._load_active_vehicle_yaml` is patched to return the variant vehicle.

- [ ] Test: series CSV round trip; `rig_c` interpolates at zero heave and applies the per-axle sign.
- [ ] Docker smoke: baseline rig run completes; summary has `avg_motion_ratio_*`.

### Task 6: Stages 0–3 and outputs

**Interfaces:**
- Produces: `run_baseline`, `run_anti_squat`, `run_anti_dive`, `run_validation`; CSV writers for `run_provenance.csv`, `variants.csv`, `fourpost_metrics.csv`, `bin_scores.csv`, `anti_squat_summary.csv`, `anti_dive_summary.csv`, `validation.csv`; `write_chosen_vehicle(...)`.
- Stage 2 and 3 read the earlier stage's summary CSV and `work/<stage>/<id>/vehicle.yml`.

- [ ] Test: constraint evaluation (margins and feasibility); sign-check stop; chosen-vehicle spring tables `[[0,0],[1,k_s]]` with the frequency comment.
- [ ] Fail, implement, pass.

### Task 7: Plots, RESULTS.md, report, README

- [ ] Four plots from the spec; RESULTS.md and `reports/DS-010-anti-geometry.md` with chosen values, gain against baseline, validation deltas, constraint margins and the provisional-assumptions block.
- [ ] README edits listed in the spec.

### Task 8: End-to-end in Docker

- [ ] Baseline and one rear variant: c_rig and c_ic agree within tolerance with the same sign (sets the per-axle rig sign); `--reuse` skips the rebuild.
- [ ] Full run: `python studies/DS-010-anti-geometry/run.py` in Docker; check the outputs and report.
