# Study Catalog

This directory is intentionally reset around a purpose-first workflow.

No simulation or sensitivity study should be added just because the tooling can
run it. Every study must answer a design question and must feed exactly one or
more report claims.

## Required Study Contract

Each study must include:

- Decision question: the specific design choice or claim being tested.
- Subsystem lane: `vdyn`, `aero`, or `chassis`.
- Inputs: vehicle config, tire file, aero map, Modelica record, raw test data,
  or external briefing source.
- Fixed assumptions: what is intentionally held constant.
- Swept variables: what is intentionally varied.
- Metrics: the plotted and tabulated outputs used to make the decision.
- Acceptance logic: what result supports, weakens, or rejects the claim.
- Report destination: the report section that consumes the result.
- Correlation plan: the measurement that will validate or correct the model.

Start each new study from `studies/STUDY_CONTRACT_TEMPLATE.yml`, then add the
runner and results only after the contract is clear.

## Report Lanes

- `studies/vdyn/`: vehicle dynamics, tires, controls-facing handling response,
  setup authority, braking as a dynamics limit, and DAQ channels needed to
  correlate the dynamic model.
- `studies/aero/`: aero map interpretation, ride-height sensitivity, drag,
  balance, platform coupling, cooling/package consequences, and aero
  correlation.
- `studies/chassis/`: source geometry, mass properties, hardpoints, load cases,
  stiffness, compliance, structural validation, and chassis consequences of
  tire/aero/brake loads.
- `studies/design-event/`: systems integration, score-sheet traceability,
  requirements, interface control, validation planning, and risk/correlation
  priority.

Powertrain stays connected through the report claims it affects. If a
powertrain study is primarily about delivered acceleration or regen/brake
balance, it belongs in `vdyn`. If it is primarily about energy, thermal, or
endurance scoring, it should become a separate future report lane rather than
being hidden inside chassis or aero.

## Fresh Study Backlog

The current intended starting backlog is:

| ID | Lane | Purpose | Report Claim |
| --- | --- | --- | --- |
| `VDYN-001` | vdyn | Establish source vehicle mass, CG, wheelbase, track, tire, brake, power, and aero inputs before any dynamic claims. | The model represents the intended vehicle. |
| `VDYN-002` | vdyn | Baseline GGV/YMD capability and handling balance. | The architecture has enough dynamic envelope to justify tuning. |
| `VDYN-003` | vdyn | StandardSim baseline steady-state, transient, and FourPost metrics. | The full vehicle model is quick, mild, and mechanically tunable. |
| `VDYN-004` | vdyn | Tire operating-window sensitivity: peak mu, cornering stiffness, relaxation, pressure/camber proxies. | Tire behavior is the dominant uncertainty and must drive test planning. |
| `VDYN-005` | vdyn | Setup authority: ARB, damping, alignment, and tire x ARB interactions. | The car has useful tuning knobs and known model gaps. |
| `VDYN-015` | vdyn | EnvelopeSim paired interaction DOE. | Vehicle-level capability depends on coupled tire/aero, drive/drag, brake/CG, and LLTD/aero-balance choices. |
| `VDYN-016` | vdyn | StandardSim-baseline-anchored surrogate response DOE. | Driver-facing compiled-run priorities are ranked by gain, rise time, overshoot, roll, and settling effects, with zero compiled variants claimed. |
| `VDYN-017` | vdyn | Inboard hardpoint Monte Carlo tolerance. | Frame manufacturing tolerance is statistically linked to roll-center, camber-gain, inboard-span, and aero-reference variation. |
| `VDYN-018` | vdyn | StandardSim hardpoint calibration subset. | A small compiled StandardSim truth set gates any simplified 25000-case hardpoint response Monte Carlo. |
| `AERO-001` | aero | Audit aero reference points, map convention, ride-height grid, force/moment sign convention. | The aero map is traceable to vehicle geometry and sign conventions. |
| `AERO-002` | aero | Ride-height/rake/downforce/drag/platform sensitivity. | Aero is meaningful but platform-sensitive. |
| `AERO-003` | aero | Aero-to-vehicle integration: balance, drag-energy consequence, suspension platform consequence. | Aero choices are defended by vehicle-level tradeoffs, not isolated downforce. |
| `CHASSIS-001` | chassis | Hardpoint, mass, and load-path source audit. | Chassis claims use one source of truth. |
| `CHASSIS-002` | chassis | Tire/brake/aero load-case generation from simulation outputs. | Structural loads come from vehicle behavior, not arbitrary factors alone. |
| `CHASSIS-003` | chassis | Stiffness/compliance sensitivity and validation plan. | The chassis preserves modeled contact-patch behavior. |
| `DE-001` | design-event | Rubric crosswalk. | Every score-sheet category maps to evidence and next validation. |
| `DE-002` | design-event | Requirements traceability. | Vehicle goals cascade into requirements, evidence, owners, and verification. |
| `DE-003` | design-event | Interface control matrix. | The vehicle is managed through measurable subsystem exchange variables. |
| `DE-004` | design-event | Validation and correlation plan. | Every major claim maps to a test, channel list, and model update action. |
| `DE-005` | design-event | Risk and correlation priority. | The team knows which assumptions can move the conclusion and what to correlate first. |
| `DE-006` | design-event | Judge question bank coverage. | Uploaded judge questions map to answer stances, evidence, and closure actions. |

## Output Standard

Results belong in each study folder as:

- `study.yml`: contract and assumptions.
- `run.py`: repeatable generation script when applicable.
- `RESULTS.md`: human-readable findings.
- `outputs/`: CSV/JSON artifacts.
- `outputs/run_provenance.csv`: runtime/provenance record with engine,
  compiled model count, evaluated case count, runtime, and caveats.
- `plots/`: report-ready figures.

Reports must cite only studies that have run and produced `RESULTS.md`.
StandardSim variant studies must only claim variant results for configurations
that were actually generated, compiled, and run. EnvelopeSim studies should
report compiled models as `0`.
