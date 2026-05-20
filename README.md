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

## Purpose

This repo exists to turn simulation work into engineering decisions. A useful
study should make clear what question was asked, what assumptions were made,
what configurations were compared, what metrics mattered, and what decision the
results supported.
