# Chassis Study Lane

Chassis is responsible for preserving the modeled vehicle behavior on the
physical car:

- Are hardpoints, mass properties, and load paths traceable?
- Which tire, brake, and aero loads define structural cases?
- What stiffness or compliance level is required for setup changes to remain
  meaningful?
- What validation test proves the structure did what the model assumed?

## Planned Study Sequence

1. `CHASSIS-001-source-and-hardpoint-audit`
2. `CHASSIS-002-load-case-generation`
3. `CHASSIS-003-stiffness-and-validation`

## Completed

- `CHASSIS-001-source-and-hardpoint-audit`: passed hardpoint/source check.
- `CHASSIS-002-load-case-generation`: passed vehicle-derived tire/brake/aero
  load generation.
- `CHASSIS-003-stiffness-and-validation`: passed stiffness and validation
  traceability framing.

No chassis claim should enter the chassis report unless it connects a load,
stiffness, or hardpoint assumption to a vehicle-level consequence.
