# 2027 design vehicle for DS-010

`vehicle.yml` is the active front V19 / rear V35 SHARK-derived vehicle selected
by `studies/DS-010-anti-geometry/study.yml`. It includes the provisional Orion
FRH/RRH aeromap with the middle cell anchored to 50% front downforce fraction.
The study-local WIP YAML is a byte-identical provenance copy; the calibration
script regenerates both from the retained pre-calibration input.

`vehicle.datum.json` records the unresolved rear SHARK vertical datum. The
stabilizer-bar pickups and rates are Orion carryovers because the SHARK exports
do not supply them. Other mass, tire, spring/damper, and powertrain fields also
remain WIP carryovers. This file is a study/design input, not a validated 2027
vehicle definition. Run the study's `verify_shark_vehicle.py` gate with the
retained V19 and V35 SHARK files before using it.
