# VDYN-005 Results

## Finding

**PASS:** the current model exposes real setup authority, with overshoot and physical setup correlation as the key next risks.

## Key Metrics

- Front/rear elastic roll stiffness: `67846` / `62559 N*m/rad`
- Front/rear ARB roll stiffness contribution: `46328` / `49753 N*m/rad`
- Front LLTD: `52.06 %`
- Ay/yaw overshoot: `21.6 %` / `18.6 %`
- Settling time: `1.302 s`

## Design Implication

Setup work should focus on damping, ARB mapping, tire operating window, and alignment propagation. The architecture has knobs; the first-drive task is proving the knobs move the car as predicted.
