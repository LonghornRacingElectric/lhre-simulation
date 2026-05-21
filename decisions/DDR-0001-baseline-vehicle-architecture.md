# DDR-0001: Baseline Vehicle Architecture

Date: 2026-05-20
Status: Proposed
Owners: LHRE Vehicle Dynamics
Linked studies: `studies/2026-baseline-envelope-justification/`, `studies/2026-vehicle-geometry-reference-study/`, `studies/2026-design-evidence-validation-matrix/`, `studies/2026-design-briefing-score-sheet-review/`, `studies/2026-powertrain-endurance-integration-study/`, `studies/2026-chassis-loadpath-integration-study/`, `studies/2026-design-tradeoff-sensitivities/`, `studies/2026-standardsim-baseline-characterization/`, `studies/2026-standardsim-setup-sensitivities/`, `studies/2026-tire-response-envelope-study/`, `studies/2026-standardsim-tire-response-sensitivities/`, `studies/2026-standardsim-tire-arb-interaction-study/`, `studies/2026-aero-platform-envelope-study/`, `studies/2026-damper-dyno-characterization/`, `studies/2026-standardsim-alignment-sensitivities/`

## Decision

Proceed with the current double-wishbone, front/rear bellcrank + stabar vehicle
architecture as the 2026 LHRE baseline simulation design.

## Context

The team goal is a car that drives sooner, builds driver confidence, survives
endurance, and remains tunable after first drive. The simulation question is
whether the current vehicle model is coherent enough to justify detailed setup
work instead of revisiting the architecture.

## Options Considered

| Option | Pros | Cons | Notes |
| --- | --- | --- | --- |
| Keep current bellcrank + stabar architecture | Known packaging, tunable roll balance, clean kinematics, supported by current BobLib model | Requires ARB rate convention reconciliation and damping/setup work | Selected baseline |
| Redesign suspension actuation architecture | Could improve specific packaging or tuning constraints | High schedule risk and would reset validation effort | Not justified by current evidence |
| Keep architecture but freeze setup now | Fastest path to build | Ignores overshoot and LLTD mapping findings | Rejected |

## Evidence

- Source geometry/reference study: source YAML checks give `1.5494 m`
  wheelbase, `1.2122 m` track, `261.07 kg` mass rollup, `48.35 %` front static
  split, and zero aero-reference error after tying aero ride-height references
  to lower inboard hardpoint averages.
- EnvelopeSim baseline: `1.770 g` max lateral at 15 m/s, `1.340 g`
  acceleration, `1.828 g` braking.
- SteadyStateEval baseline: `0.402 deg/g` understeer gradient and
  `0.894 deg/g` roll gradient.
- TransientEval baseline: `0.069 s` ay rise time, `0.059 s` yaw rise time,
  `20.6 %` ay overshoot, `18.6 %` yaw overshoot.
- FourPostEval: low anti-dive/squat, low toe coupling, and `45.29 %` front
  elastic LLTD using corrected physical rates.
- Setup sensitivity sweep: ARB scaling strongly moves LLTD and roll angle while
  only weakly changing ay/yaw gain in the 5 deg step-steer test.
- Tire response sweep: peak friction changes max lateral by `0.343 g`; cornering
  stiffness changes dAy/ddelta by `98.7 (m/s^2)/rad`.
- StandardSim tire response sweep: global lateral stiffness changes ay
  overshoot by `14.49` percentage points, while global lateral friction is
  nearly flat in the 15 m/s, 5 deg step-steer maneuver.
- StandardSim tire stiffness x ARB sweep: tire stiffness dominates the tested
  ARB balance cases, and none of the tested cases meet both preferred overshoot
  targets.
- Aero platform sweep: downforce scaling moves max lateral from `1.579 g` to
  `1.856 g` at 15 m/s and from `1.557 g` to `2.304 g` at 25 m/s.
- Powertrain endurance integration: the selected `7.02 kWh` pack gives
  `6.67 kWh` usable energy after the briefing efficiency assumption,
  `15.0 %` endurance energy margin, and enough modeled drive-force capacity
  for `1.340 g` baseline acceleration at 15 m/s.
- Chassis loadpath integration: tire-limit resultant is `2657.1 N` per corner,
  static-FOS design resultant is `5314.3 N` per corner, and the study ties
  tire, brake, aero, and torsional-stiffness loads to validation tests.
- Damper dyno source review: TTX25 curves are available, but the current model
  still uses simplified linear damper tables.
- Alignment sensitivity sweep: static alignment parameters are exposed, but the
  current StandardSim step-response path is effectively insensitive to them.
- Evidence/validation matrix: `10` core claims are traced to evidence,
  confidence, residual risks, and first-drive validation signals; `7` are high
  or medium-high confidence.
- Design briefing and score-sheet review: current simulation/validation work
  supports at least `125` points of cross-functional Design Event discussion,
  with vehicle dynamics acting as the contact-patch-centered integration story.

## Consequences

This decision enables detailed setup work: ARB setting calibration, damping
DOE, tire-response studies, aero/platform studies, and alignment-model
verification.

Accepted risks:

- The physical BobLib ARB-rate model and the team LLTD calculator are not yet
  numerically aligned.
- Tire peak friction, cornering stiffness, pressure, temperature, and relaxation
  length remain major uncertainties.
- Aero pitch moment still needs conversion into a validated front/rear aero load
  split before making final aero balance claims.
- Damper curves need to be digitized before using the damping study for final
  adjuster choices.
- Static alignment propagation through BobLib/StandardSim must be verified
  before using StandardSim to choose toe or camber.
- No track correlation data exists yet.
- Evidence matrix must be converted into a first-drive validation sheet before
  the decision moves from proposed to validated.

Revisit this decision if track data or updated tire/ARB modeling shows that the
baseline cannot deliver mild understeer, low lag, and manageable overshoot
without impractical setup compromises.
