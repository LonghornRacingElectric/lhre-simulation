# Vehicle Dynamics Study Lane

Vehicle dynamics is responsible for the contact-patch-centered story:

- What vehicle are we modeling?
- What dynamic capability does the architecture have?
- What knobs move the car?
- What uncertainties dominate?
- What first-drive data will prove or correct the model?

## Planned Study Sequence

1. `VDYN-001-source-vehicle-audit`
2. Preliminary design with EnvelopeSim: `VDYN-002`, `VDYN-011`,
   `VDYN-012`, `VDYN-015`
3. Tire model foundation: `VDYN-004`, `VDYN-006` through `VDYN-010`
4. Advanced characterization with StandardSim: `VDYN-003`, `VDYN-005`,
   `VDYN-013`, `VDYN-014`, `VDYN-016`
5. Manufacturing tolerance and correlation bridge: `VDYN-017`, `VDYN-018`

## Completed

- `VDYN-001-source-vehicle-audit`: passed source mass, CG, wheelbase, track,
  tire metadata, and aero reference checks.
- `VDYN-002-baseline-envelope`: passed baseline GGV capability and tire
  vertical-load range checks.
- `VDYN-003-standardsim-baseline`: passed StandardSim metric ingestion for
  steady-state, transient, and FourPost evidence.
- `VDYN-004-tire-operating-window`: passed tire-source screening and
  correlation-channel definition.
- `VDYN-005-setup-authority`: passed setup-authority framing from roll
  stiffness, LLTD, and transient response metrics.
- `VDYN-006-tire-load-sensitivity`: passed tire load-sensitivity screening.
- `VDYN-007-tire-pure-slip-curves`: passed representative pure-slip curve
  generation.
- `VDYN-008-tire-cornering-stiffness`: passed cornering-stiffness screening
  across observed load range.
- `VDYN-009-tire-relaxation-response`: passed relaxation-length response-scale
  screening.
- `VDYN-010-tire-combined-slip-budget`: passed combined-slip budget conversion
  from admitted GGV outputs.
- `VDYN-011-envelope-doe-importance`: passed vehicle-level EnvelopeSim DOE
  importance ranking.
- `VDYN-012-aero-scaling-doe`: passed aero downforce/drag/balance scaling
  screening.
- `VDYN-013-torsional-stiffness-authority`: passed chassis torsional stiffness
  to setup-authority screening.
- `VDYN-014-static-alignment-screening`: passed static camber/toe tire-response
  screening.
- `VDYN-015-envelopesim-interaction-doe`: passed paired EnvelopeSim
  interaction surfaces.
- `VDYN-016-standardsim-response-surface-doe`: passed
  StandardSim-baseline-anchored surrogate response-surface ranking with zero
  compiled variants claimed.
- `VDYN-017-hardpoint-monte-carlo-tolerance`: passed inboard hardpoint
  geometry/aero-reference tolerance Monte Carlo and chassis manufacturing
  tolerance framing.
- `VDYN-018-standardsim-hardpoint-calibration`: added compiled StandardSim
  calibration subset workflow for hardpoint tolerance response claims.

No result should enter the vehicle dynamics report until its study folder
contains a repeatable script or clearly documented manual calculation, plots,
and a `RESULTS.md` decision statement.
