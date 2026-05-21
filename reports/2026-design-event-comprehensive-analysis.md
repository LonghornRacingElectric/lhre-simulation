# 2026 Design Event Comprehensive Analysis

This file is the judge-facing packet entry point. The detailed technical
reports are split by subsystem:

- [Vehicle Dynamics](2026-vdyn-design-report.md)
- [Aerodynamics](2026-aero-design-report.md)
- [Chassis](2026-chassis-design-report.md)
- [Design Report Index](2026-design-report-index.md)
- [Judge Question Bank](2026-judge-question-bank.md)

## Executive Story

The 2026 car is justified because the team can trace its goals into a source
vehicle model, a credible dynamic envelope, a platform-aware aero package,
chassis load paths that preserve contact-patch behavior, and a first-drive
correlation plan.

The current package strongly supports Overall Vehicle, Vehicle Dynamics,
Aerodynamics, and Chassis discussion. It also defines the vehicle-level
interfaces that Powertrain, Driver Interface, and LV/DAQ need to close with
owner artifacts and test data.

## Rubric Coverage

![Rubric coverage](../studies/design-event/DE-001-rubric-crosswalk/plots/rubric_coverage.png)

- Strong direct coverage: `95` score-sheet points
- Interface coverage needing owner artifacts: `55` score-sheet points
- Unmapped categories: `0`

## Technical Evidence

- `VDYN-001` through `VDYN-017`: source vehicle, baseline envelope,
  StandardSim response, tire operating window, setup authority, tire load
  sensitivity, pure-slip curves, cornering stiffness, relaxation response,
  combined slip, envelope DOE, aero scaling, torsional stiffness, and static
  alignment, EnvelopeSim interaction DOE, and StandardSim-anchored response
  DOE, and hardpoint tolerance Monte Carlo.
- `AERO-001` through `AERO-003`: map/reference audit, platform sensitivity,
  vehicle integration.
- `CHASSIS-001` through `CHASSIS-003`: hardpoint audit, vehicle-derived load
  cases, stiffness and validation framing.
- `DE-001` through `DE-006`: score-sheet crosswalk, requirements
  traceability, interface control, validation/correlation plan, and risk
  priority, plus uploaded judge-question coverage.

## Systems Engineering Layer

![Requirements readiness](../studies/design-event/DE-002-requirements-traceability/plots/requirements_readiness.png)

![Interface criticality](../studies/design-event/DE-003-interface-control-matrix/plots/interface_criticality_matrix.png)

![Validation priority](../studies/design-event/DE-004-validation-correlation-plan/plots/validation_priority.png)

![Risk priority](../studies/design-event/DE-005-risk-correlation-priority/plots/risk_priority.png)

The OEM-style layer is the glue: requirements explain why each study exists,
interface control explains which subsystem exchanges matter, validation defines
the first-drive data spine, and risk priority explains what can still move the
conclusion.

## Fundamental Answer

Given the team's goals and constraints, this is the right vehicle because it is
traceable, buildable, tunable, and credible enough to validate. The model does
not claim the car is finished. It identifies exactly what should be tested
first and what data would change the design.

For the complete narrative, use [2026 Design Report Index](2026-design-report-index.md).
