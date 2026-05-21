# Current Vehicle

This is the current Longhorn Racing Electric baseline vehicle configuration.

- BobSim/BobLib record: `DWBCStabar_DWBCStabar`
- Front architecture: double wishbone, bellcrank actuation, stabar
- Rear architecture: double wishbone, bellcrank actuation, stabar
- LHRE config: `vehicles/current/vehicle.yml`
- Reference tire: `vehicles/current/tires/16x7p5_10_12psi.tir`
- BobSim seed config: `BobSim/vehicle.yml`
- BobSim template: `BobSim/_0_Utils/vehicle_templates/DWBCStabar_DWBCStabarRecord.yml`

To stage this configuration into the BobSim submodule before running BobSim
workflows from `BobSim/`, run from the repository root:

```bash
make sync-inputs
```

Use `VEHICLE_CONFIG=...` to stage a different vehicle config:

```bash
make sync-vehicle VEHICLE_CONFIG=vehicles/concepts/example/vehicle.yml
```
