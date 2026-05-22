# DS-004 Vehicle Design Synthesis and Justification

This study converts the first-pass simulation sensitivity work into a design
argument for the current vehicle.

## Inputs

- DS-001: EnvelopeSim parameter sensitivity
- DS-002: StandardSim SteadyStateEval sensitivity
- DS-003: StandardSim TransientEval sensitivity

## Output

The report is intentionally a synthesis document, not a new simulation. It
answers: given the current baseline and sensitivity rankings, which vehicle
design choices are already well-supported, which knobs should be treated as
late-stage tuning variables, and which questions deserve a follow-up study?
It also separates the ground-up design logic from the platform tuning evidence
so the report can be read as a design-review justification.

Run from the repository root:

```bash
/tmp/lhre-sim-venv/bin/python studies/DS-004-vehicle-design-synthesis/run.py
```
