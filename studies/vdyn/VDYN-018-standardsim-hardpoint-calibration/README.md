# VDYN-018 StandardSim Hardpoint Calibration

Compiles a small StandardSim truth set for hardpoint perturbations, then
compares the actual transient metric deltas against simplified geometry
diagnostics. The intended workflow is:

1. Generate a 25-point hardpoint perturbation design.
2. Compile and run those 25 StandardSim vehicle configurations, up to four at
   a time.
3. Fit/check a simplified mapping from geometry diagnostics to StandardSim
   response deltas.
4. Use that calibrated mapping, not raw guesswork, for large 25000-case
   manufacturing tolerance statistics.
