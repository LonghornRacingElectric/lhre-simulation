# lhre-simulation

Longhorn Racing Electric's simulation workspace for vehicle design exploration,
tradeoff studies, optimization, and simulation-backed engineering decisions.

This repository wraps BobSim as the simulation backend and owns LHRE-specific
vehicle definitions, study configurations, selected results, reports, and
decision records.

## Repository Layout

- `BobSim/`: simulation backend, included as a Git submodule
- `vehicles/`: LHRE vehicle definitions and design variants
- `studies/`: self-contained design studies and tradeoff analyses
- `configs/`: shared simulation, sweep, and workflow configuration
- `reports/`: committed summaries, plots, and design-study deliverables
- `decisions/`: design decision records backed by studies and assumptions
- `docs/`: workflow notes, assumptions, and modeling guidance

## Getting Started

Clone with submodules:

```bash
git clone --recurse-submodules git@github.com:LonghornRacingElectric/lhre-simulation.git
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Set up the workspace:

```bash
make setup
```

This initializes submodules and delegates BobSim's Docker setup through the
`BobSim/` submodule.

The current LHRE baseline vehicle config lives at
`vehicles/current/vehicle.yml`. It uses the `DWBCStabar_DWBCStabar` architecture:
bellcrank plus stabar in the front and rear.

To stage the selected LHRE vehicle config into BobSim before running BobSim
workflows:

```bash
make sync-inputs
```

This stages the selected vehicle config and repo-owned tire property files into
the locations BobSim expects. Aero maps remain in the vehicle YAML because they
are vehicle-specific model assumptions. Tire files currently live with the
vehicle config that uses them, such as `vehicles/current/tires/`.

## Purpose

This repo exists to turn simulation work into engineering decisions. A useful
study should make clear what question was asked, what assumptions were made,
what configurations were compared, what metrics mattered, and what decision the
results supported.

## Study Pattern

Studies are now organized around a purpose-first contract. Before a simulation
or sensitivity sweep exists, it needs a decision question, fixed assumptions,
swept variables, metrics, acceptance logic, report destination, and correlation
plan.

The fresh study lanes are:

- `studies/vdyn/`: vehicle dynamics, tires, setup authority, braking/drive
  limits, and dynamic correlation.
- `studies/aero/`: aero map audit, ride-height/platform sensitivity, drag,
  balance, and aero validation.
- `studies/chassis/`: hardpoints, mass properties, load paths, stiffness,
  compliance, and structural validation.

Reports are split at the same level:

- `reports/2026-vdyn-design-report.md`
- `reports/2026-aero-design-report.md`
- `reports/2026-chassis-design-report.md`
- `reports/2026-design-report-index.md`

Use `make study-catalog` to view the current intended study backlog.
