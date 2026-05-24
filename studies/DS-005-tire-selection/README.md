# DS-005 Tire Selection

This study compares tire candidates for the current vehicle architecture.

The generated Round 8 tire files contain real lateral/vertical/alignment fits
with fabricated pure-longitudinal support copied from the current hybrid
reference tire. For that reason, this study ranks tire candidates primarily on
lateral EnvelopeSim capability and treats longitudinal/combined-slip behavior as
non-selection data.

Run from the repository root:

```bash
/tmp/lhre-sim-venv/bin/python studies/DS-005-tire-selection/run.py
```

Optional StandardSim finalist check:

```bash
/tmp/lhre-sim-venv/bin/python studies/DS-005-tire-selection/run.py --standardsim-top 6
```
