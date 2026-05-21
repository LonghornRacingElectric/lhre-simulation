# Aero Study Lane

Aero is responsible for the platform-sensitive force, drag, balance, and
correlation story:

- Is the aero map referenced to the correct vehicle geometry?
- How sensitive are downforce and drag to ride height and rake?
- What vehicle-level benefit survives the drag, cooling, packaging, and
platform-control tradeoffs?
- How will coastdown, aero-on/off, and ride-height data close the loop?

## Planned Study Sequence

1. `AERO-001-map-and-reference-audit`
2. `AERO-002-platform-sensitivity`
3. `AERO-003-vehicle-integration`

## Completed

- `AERO-001-map-and-reference-audit`: passed map shape, ride-height grid,
  lower-inboard reference, baseline force, and coefficient checks.
- `AERO-002-platform-sensitivity`: passed ride-height/rake force sensitivity
  screening.
- `AERO-003-vehicle-integration`: passed force, drag-power, and vehicle
  envelope integration framing.

No aero plot should enter the aero report unless the sign convention, reference
point, and vehicle-level interpretation are documented.
