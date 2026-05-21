# VDYN-004 Results

## Finding

**PASS:** tire behavior is the dominant remaining model-confidence item and must be treated as a first-drive test objective.

## Key Tire Source Metrics

- Nominal load: `650.0 N`; vertical range: `100.0` to `1800.0 N`
- Lateral peak coefficients: `PDY1=-2.402750`, `PDY2=0.343535`
- Lateral stiffness coefficients: `PKY1=-53.2421`, `PKY2=2.3821`
- Lateral relaxation coefficients: `PTY1=3.330134`, `PTY2=3.587160`
- Nominal relaxation scale from PTY1*R0: `0.677 m`
- Baseline ay overshoot needing tire/setup correlation: `21.6 %`

## Design Implication

The tire plan must log hot pressure, tire temperature spread, steering, yaw, ay, speed, and driver comments. The current model can support screening, but final setup confidence requires measured tire operating-window correlation.
