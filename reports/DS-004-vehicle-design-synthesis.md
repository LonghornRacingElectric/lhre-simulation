# DS-004 Vehicle Design Synthesis and Justification

Generated UTC: 2026-05-22T12:47:53+00:00

## Thesis

The current vehicle is justified as a low-CG, aero-forward, tire-limited, rear-drive formula-style car whose primary architecture is set by mass properties, tire capability, aero load, and longitudinal force limits. StandardSim then shows that the late-stage handling knobs should be alignment, steering rack travel, roll-platform rates, aero balance, and small CG-placement adjustments.

In plain human terms: the big rocks are low/central mass, downforce, tire quality, and usable power/brake force. The spicy knobs are toe and rack travel. Springs and bars are real, but they are platform/balance tools, not magic grip buttons.

## Simulation Basis

| Study | Role | Source |
| --- | --- | --- |
| DS-001 | Capability envelope and first-order architecture | `studies/DS-001-envelopesim-parameter-sensitivity/` |
| DS-002 | Steady-state handling response sensitivities | `studies/DS-002-standardsim-steady-state-sensitivity/` |
| DS-003 | Transient response sensitivities | `studies/DS-003-standardsim-transient-sensitivity/` |

## Baseline Position

| Item | Value |
| --- | ---: |
| Mass | 261.07 kg |
| CG height | 0.2796 m |
| Front static fraction | 0.4835 |
| Wheelbase | 1.5494 m |
| Track front/rear | 1.2122 / 1.2122 m |
| ClA / CdA | 2.3472 / 1.1726 m^2 |
| Aero balance front | 0.5000 |
| Power / drive cap | 80000 W / 3735 N |
| Brake cap / front brake bias | 14000 N / 0.620 |

## Baseline Response Summary

| Response | Value |
| --- | ---: |
| Envelope lat 25 m/s | 2.080 g |
| Envelope mean lateral | 1.803 g |
| Envelope mean accel | 1.327 g |
| Envelope mean brake | 1.932 g |
| Envelope mean GGV area | 8.178 g^2 |
| StandardSim ay_max | 18.575 m/s^2 |
| StandardSim understeer gradient | 0.394 deg/g |
| StandardSim roll gradient | 0.894 deg/g |
| Transient step ay peak | 3.413 m/s^2 |
| Transient yaw gain DC | 2.220 (rad/s)/rad |
| Transient ay lag at 1 Hz | 0.0157 s |

## Repeated First-Order Levers

| Lever | Top-response count |
| --- | ---: |
| front rack travel per rev | 6 |
| sprung CG x | 5 |
| CG height | 4 |
| front static toe | 4 |
| downforce area | 2 |
| rear static toe | 2 |
| sprung mass | 2 |
| max drive power | 1 |

This count is not an optimization score; it is a sanity check for which variables keep reappearing as the strongest single-factor levers.

## Ground-Up Design Logic

| Layer | Simulation read | Design consequence |
| --- | --- | --- |
| 1. Capability envelope | EnvelopeSim shows high-speed lateral and GGV area are driven by downforce, CG height, total mass, tire scale, and longitudinal force limits. | Set mass, CG, aero area, tire selection, power limit, and brake capacity before tuning feel. |
| 2. Steady-state balance | SteadyStateEval shows ay_max follows tire scale, while understeer/roll/sideslip follow aero load, CG placement, roll platform, and toe. | Use aero/CG/roll distribution to place the car in the right balance window. |
| 3. Transient response | TransientEval shows rack travel, toe, CG x, and mass placement dominate gain, phase, lag, and initial yaw/ay response. | Tune driver command authority and response speed after the balance target is sane. |
| 4. Track trim | Toe and brake bias are high-authority knobs, but broad sweeps show they can overpower interpretation if used too early. | Keep the baseline neutral/adjustable, then use small sweeps for final event-specific trim. |

## Design Decisions

### 1. Treat low mass and low CG as first-order architecture constraints.

| Field | Read |
| --- | --- |
| Status | strongly supported |
| Simulation basis | DS-001 mean accel: total mass -20.58% (1.4805 -> 1.2075); DS-001 area 25: total mass -16.85% (10.8697 -> 9.1933); DS-001 mean area: CG height -33.14% (9.5733 -> 6.8629); DS-002 roll_gradient_deg_per_g: sprung CG z 19.74% (0.8160 -> 0.9924) |
| Design implication | Packaging, driver placement, ballast, accumulator/fuel placement, and upright/unsprung mass are not housekeeping details; they directly set the usable envelope and roll response. |
| Current vehicle position | Baseline mass rollup is 261.07 kg with CG height 0.2796 m and 48.35% front static load. |

### 2. Keep the vehicle concept aero-forward, with aero balance owned as a primary design variable.

| Field | Read |
| --- | --- |
| Status | strongly supported |
| Simulation basis | DS-001 lat 25: downforce area 20.51% (1.8667 -> 2.2933); DS-001 area 25: downforce area 36.75% (8.1737 -> 11.8308); DS-002 understeer_gradient_deg_per_g: aero downforce scale 38.13% (0.2867 -> 0.4370) |
| Design implication | Downforce is not cosmetic. It grows high-speed lateral capability and changes steady-state balance, so aero package and aero balance deserve early design ownership. |
| Current vehicle position | Baseline EnvelopeSim uses ClA=2.347 m^2, CdA=1.173 m^2, and 50% front aero balance. |

### 3. Protect tire quality and tire-load management before chasing secondary geometry changes.

| Field | Read |
| --- | --- |
| Status | strongly supported |
| Simulation basis | DS-002 ay_max: lateral tire friction scale 10.76% (17.4552 -> 19.4542); DS-001 mean area: lateral tire peak scale 12.79% (7.6371 -> 8.6834); DS-001 mean area: lateral tire load sensitivity scale 3.45% (8.0263 -> 8.3083) |
| Design implication | The tire model/test data and the platform that keeps tires in their usable load range are foundational. Tire scale moving ay_max by double-digit percent means tire uncertainty can overwhelm many chassis tweaks. |
| Current vehicle position | Current tire source is vehicles/current/tires/16x7p5_10_12psi.tir. |

### 4. Use springs and anti-roll bars as platform-control and balance tools, not as the main grip source.

| Field | Read |
| --- | --- |
| Status | supported |
| Simulation basis | DS-002 roll_gradient_deg_per_g: front anti-roll bar rate -16.00% (0.9760 -> 0.8330); DS-002 roll_gradient_deg_per_g: rear anti-roll bar rate -16.96% (0.9831 -> 0.8315); DS-003 step.roll_gain_dc: front anti-roll bar rate -16.35% (0.0480 -> 0.0409); DS-003 step.roll_gain_dc: rear anti-roll bar rate -16.74% (0.0482 -> 0.0409) |
| Design implication | Spring and bar rates should be selected for aero platform, roll control, ride, and balance. They should not be expected to create large raw lateral capability by themselves. |
| Current vehicle position | Baseline rates: front spring 26.27 kN/m, rear spring 43.78 kN/m, front bar 258.94 N*m/rad, rear bar 535.36 N*m/rad. DS-002 spring variants now preserve free length using FourPost motion ratios. |

### 5. Keep static toe near zero for the baseline, and reserve toe as a late-stage trim knob.

| Field | Read |
| --- | --- |
| Status | strongly supported |
| Simulation basis | DS-002 roadwheel_angle_gradient_deg_per_g: rear static toe 6.83% (4.7054 -> 4.9954); DS-003 step.ay_peak: front static toe 75.07% (3.1220 -> 5.6842); DS-003 step.yaw_peak: front static toe 70.19% (0.2209 -> 0.3787) |
| Design implication | Toe is powerful enough to shape steady and transient behavior, but also powerful enough to destabilize interpretation. Zero static toe is a sane baseline; final toe should be set by small, response-targeted sweeps. |
| Current vehicle position | Baseline front and rear static toe are both 0 deg. |

### 6. Treat rack travel as the driver-interface gain knob.

| Field | Read |
| --- | --- |
| Status | strongly supported |
| Simulation basis | DS-002 handwheel_angle_gradient_deg_per_g: front rack travel per rev -30.20% (20.4677 -> 15.2004); DS-002 handwheel_torque_peak_abs: front rack travel per rev 30.78% (15.9006 -> 21.6841); DS-003 step.ay_gain_dc: front rack travel per rev 29.91% (28.5461 -> 38.5872); DS-003 frequency.yaw_gain_peak: front rack travel per rev 29.91% (1.9085 -> 2.5798) |
| Design implication | Rack travel directly scales driver command authority, handwheel angle, and effort. Changing it is not a hidden performance gain; it is a deliberate HMI/control-authority choice. |
| Current vehicle position | Baseline front rack travel is 0.0889 m/rev. |

### 7. Hold chassis torsional stiffness target unless structural packaging forces a change.

| Field | Read |
| --- | --- |
| Status | supported as sufficient for first baseline |
| Simulation basis | DS-002 ay_max: body torsional stiffness 1.34% (18.3791 -> 18.6280); DS-002 roll_gradient_deg_per_g: body torsional stiffness -3.78% (0.9204 -> 0.8866); DS-003 step.roll_gain_dc: body torsional stiffness -6.89% (0.0463 -> 0.0433) |
| Design implication | Current torsional stiffness is worth preserving, but the first-order handling story is elsewhere: tires, aero, CG, toe, rack travel, and roll platform. |
| Current vehicle position | Baseline body torsional stiffness is 300 kN*m/rad. |

### 8. Use brake bias and drive limits as envelope-shaping systems.

| Field | Read |
| --- | --- |
| Status | supported by EnvelopeSim |
| Simulation basis | DS-001 accel 25: max drive power 49.18% (0.7525 -> 1.2775); DS-001 brake 25: CG height -21.05% (2.5200 -> 2.0400); DS-001 mean brake: front brake distribution 18.43% (1.7720 -> 2.1280) |
| Design implication | Power/force limits and brake bias are not downstream details; they define longitudinal envelope area and should remain adjustable during validation. |
| Current vehicle position | Baseline assumptions: 80 kW, 3735 N drive cap, RWD, 14 kN brake cap, 62% front brake distribution. |

## Platform Evidence

Signed values are low-to-high parameter spans as a percent of baseline response. This table is why springs and bars are treated as platform/balance tools rather than first-order raw-grip generators.

| Lever | ay_max | understeer | steady roll | step roll peak | step roll gain | Read |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| front spring | -2.50% | 10.30% | -9.82% | -9.49% | -9.88% | strong roll/platform lever, modest raw ay lever |
| rear spring | 3.59% | -2.94% | -10.18% | -9.60% | -9.49% | strong roll/platform lever, modest raw ay lever |
| front anti-roll bar | -3.38% | 11.71% | -16.00% | -15.69% | -16.35% | strong roll/platform lever, modest raw ay lever |
| rear anti-roll bar | 4.63% | -7.95% | -16.96% | -16.70% | -16.74% | strong roll/platform lever, modest raw ay lever |
| body torsional stiffness | 1.34% | -1.93% | -3.78% | -4.87% | -6.89% | secondary structure lever in this baseline sweep |

## Control Map

| Objective | Primary knobs | Design use |
| --- | --- | --- |
| Increase high-speed lateral envelope | downforce area, CG height, tire lateral capability | Aero and tire/load-transfer design problem, not a steering-rack problem. |
| Tune steady-state understeer | aero downforce scale/balance, CG x, roll stiffness distribution, static toe | Use aero/CG/roll distribution for architecture; use toe for final trim. |
| Reduce roll response | CG z, springs, anti-roll bars, torsional stiffness | Springs/bars are legitimate roll-platform tools; torsion is a secondary preservation target. |
| Shape driver steering feel and command authority | rack travel per revolution | Set rack travel to driver target after vehicle balance is known. |
| Improve transient phase/lag | sprung CG x, sprung mass, toe | Use mass placement as the architecture lever; use toe carefully for track tuning. |
| Improve acceleration/braking envelope | drive power, drive force cap, brake force, brake distribution, CG height | Maintain brake-bias adjustability and validate power/traction limits. |

## Open Questions

| Question | Why it matters | Next study |
| --- | --- | --- |
| What aero balance gives the best understeer/phase tradeoff? | Downforce area and downforce scale are high-value, but DS-002 varied scale rather than balance. | StandardSim aero balance and ride-height/downforce map sweep. |
| What is the fine static toe window? | Toe strongly moves steady and transient metrics; +/-1 deg is intentionally broad. | Small-range toe sweep around zero, likely +/-0.2 deg with both axles. |
| How should camber be treated with tire data uncertainty? | Static camber was not a top first-pass driver, but real tires may make camber window important. | Tire model validation plus camber/load sensitivity sweep. |
| Where is the spring/bar optimum after aero platform constraints are imposed? | Springs/bars affect roll and understeer, but need ride-height/aero constraints to become a design optimum. | Coupled ride/roll/aero-platform optimization using StandardSim and FourPost motion ratios. |
| Can transient threshold metrics be made robust enough for optimization? | Some rise/overshoot metrics are sign-sensitive in broad sweeps, so active DS-003 metrics avoided them. | Refine transient response metrics using absolute response or monotonic target definitions. |

## Generated Files

- `outputs/source_top_sensitivities.csv`
- `outputs/architecture_layers.csv`
- `outputs/platform_summary.csv`
- `outputs/design_decisions.csv`
- `outputs/control_map.csv`
- `outputs/open_questions.csv`
- `RESULTS.md`
