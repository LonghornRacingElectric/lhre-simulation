# OpenLAP Battery Cell and Pack Trade Study

Run date: 2026-07-25<br>
Event distance: 22.000 km<br>
Vehicle/track: fixed LHRe OpenLAP Michigan endurance model<br>
Configurations simulated: 128 feasible topologies within the stated study grid<br>
Topologies rejected before simulation: 131

## Executive result

The fastest constraint-valid result is **Reliance INR21700-RS50 110s4p** at **1216.03 s** (20.27 min), using a **48.72 kW** terminal-power ceiling and finishing with **0.504 kWh** of modeled usable chemical energy.
Against the team 7.2 kWh-class 130s5p P30B baseline (1242.20 s), this is **2.107% faster** (26.17 s saved).
The requested 7.2 kWh baseline is instantiated from the actual 130s5p P30B cell data as **7.020 kWh nominal** and **6.881 kWh modeled usable** from 100% to the 5% SOC floor.

Regeneration is enabled in endurance only. The fixed 10.059 kW terminal command was calibrated on the native P30B baseline to the measured 8.574 kW active-period RMS and is held constant across packs.

## Fastest-configuration metrics

| Metric | Result |
|---|---:|
| Cell count | 440 (110s4p) |
| Maximum charged voltage | 462.0 V |
| Nominal / modeled usable energy | 7.920 / 7.763 kWh |
| Pack / vehicle mass | 36.850 / 259.735 kg |
| Total endurance / equivalent average lap | 1216.026 / 59.141 s |
| Fastest / slowest completed lap | 59.101 / 60.269 s |
| Terminal power, average / peak | 20.036 / 48.719 kW |
| Motor-shaft power, average / peak | 16.981 / 44.382 kW |
| Cell current, RMS / peak | 19.649 / 37.031 A |
| Peak pack current | 148.122 A |
| Terminal / chemical / mechanical energy | 6.768 / 7.259 / 5.736 kWh |
| Gross discharge / recovered / net terminal energy | 7.595 / 0.827 / 6.768 kWh |
| Regen active / whole-event RMS | 8.803 / 4.762 kW |
| Total / regen pack resistive heat | 0.4913 / 0.0104 kWh |
| Pack resistive heat | 0.4913 kWh |
| Battery discharge efficiency | 94.044% |
| Adiabatic cell temperature rise | 60.0 degC |
| Remaining modeled usable energy | 0.504 kWh |

## Recommended configurations

| Category | Configuration | Time (s) | Pack mass (kg) | Heat (kWh) | Reserve (kWh) |
|---|---|---:|---:|---:|---:|
| Best overall | Reliance INR21700-RS50 110s4p | 1216.03 | 36.85 | 0.4913 | 0.504 |
| Best lightweight within 1% of fastest | Reliance INR21700-RS50 135s3p | 1227.65 | 33.92 | 0.4128 | 0.499 |
| Lowest heat within 2% of fastest | Ampace JP30P1 (JP30) 130s5p | 1238.55 | 40.62 | 0.2881 | 0.500 |
| Best simulated performance per pack kg | Reliance INR21700-RS50 110s2p | 1402.56 | 18.43 | 0.1309 | 0.503 |

## Sensitivities

The most decision-useful local sensitivities below come from centered reruns around the best configuration while retuning power to the same reserve target:

- Usable energy: **-10.72 s/kWh**.
- Pack mass: **0.875 s/kg**.
- Cell DCIR: **1.217 s per mOhm/cell** (**0.042 s/mOhm** on an initial pack-resistance basis).
- These are centered derivative probes with power retuned to the same reserve target, not additional selectable pack designs. In particular, the +5% capacity probe exceeds the study's 8.0 kWh usable-energy cap.

A cross-configuration linear fit gives R² = 0.938 over 128 constraint-valid screening cases. Because topology variables are correlated, the local finite differences should drive design decisions.

## Engineering observations

- The terminal-power limit is capped at 80 kW. A configuration that still has more than 0.5 kWh at 80 kW is labeled `power_cap_extra_reserve`; it is physically valid but not an exact equal-reserve comparison.
- Pack current is limited by each cell's conservative published continuous rating and the unchanged 250 A inverter bus-current limit. Published pulse values are reported but not used.
- The thermal result is an adiabatic cell-temperature-rise upper bound from I²R heat and an assumed 1000 J/kg-K cell heat capacity. It is not a cooling-loop or peak-cell-temperature prediction.
- Screening uses every 8th native track element and every accepted topology runs the full 22 km. Per-cell winners, the baseline, and leading Pareto cases are rerun on the native 0.25 m mesh.
- The Pareto sheet is therefore a screening-level design-space frontier with native results substituted where available. Use it to select hardware-test candidates, not as a sub-second final ranking of every non-dominated topology.
- `Best value` means the largest reciprocal endurance-time-per-pack-mass metric in this study. It is not a purchase-cost ranking because comparable cell pricing was not available, and it can favor a very light pack with materially slower pace.

## Constraints and pack generation

- Maximum charged voltage: strictly below 600 V.
- Maximum modeled usable chemical energy: 8.0 kWh.
- Series counts: 110–140 in 5-cell module increments. Parallel counts begin at the first topology providing 9 Ah and stop at 6p. These are explicit study-grid bounds, not an exhaustive enumeration of every possible accumulator architecture.
- Packaging screen: cylindrical cell envelope no larger than the 130s5p P30B baseline (11.515 L). This is a topology screen, not enclosure CAD validation.

## Cell-data assumptions and uncertainty

- The live Notion shortlist supplied eight fully identifiable cell models. The ambiguous `Tenpower 18650 4000mAh` entry is excluded until Battery supplies an exact model/datasheet.
- Molicel P30B/P50B use published room-temperature DCIR. Cells with ACIR only use DCIR = 2.047 × ACIR, derived from the two Molicel ACIR/DCIR ratios. Treat those resistance rankings as provisional and validate on the team's pulse fixture.
- One common normalized NMC OCV-vs-SOC shape and one common SOC resistance multiplier are used because comparable maps are not published. Ambient/cell resistance is held at 25 °C.
- Pack mass uses 1.25 × total cell mass, matching the existing team model. Fixed non-cell pack mass and cooling strategy are otherwise identical.
- Endurance regeneration is enabled from the measured terminal-RMS policy; acceleration, skidpad, and autocross remain regeneration-free. No driver-change stop, thermal derating, or SOC-dependent motor map beyond the coupled pack-voltage/current and EMRAX field-weakening model.

## Source register

| Cell | Resistance basis | Datasheet |
|---|---|---|
| Ampace JP30P1 (JP30) | Estimated as 2.047 x published maximum 5.0 mOhm ACIR; no DCIR value is published. | [source](https://www.nkon.nl/en/amfile/file/download/file/739/product/5848/) |
| Ampace JP50P1 (JP50) | Estimated as 2.047 x published maximum 4.0 mOhm ACIR; no DCIR value is published. | [source](https://www.dnkpower.com/wp-content/uploads/2025/09/Full-Tab-Battery-Cell-3.58V-5000mAh-21700B-datasheet.pdf) |
| Tenpower INR21700-50XG | Estimated as 2.047 x published maximum 4.0 mOhm ACIR; no DCIR value is published. | [source](https://www.lithiumlifepo4-battery.com/photo/lithiumlifepo4-battery/document/95856/INR21700-50XG%20specification.pdf) |
| Tenpower INR18650-30XG | Estimated as 2.047 x published maximum 5.0 mOhm ACIR; no DCIR value is published. | [source](https://static.dianchi.cn/uploads/2025/08/14/dc24ug2qundv6vmnaf.pdf) |
| Tenpower INR21700-60XG | Estimated as 2.047 x published maximum 5.0 mOhm ACIR; no DCIR value is published. | [source](https://www.lithiumlifepo4-battery.com/photo/lithiumlifepo4-battery/document/103029/TENPOWER%C2%A0PRODUCT%C2%A0SPECIFICATION%C2%A0INR21700-60XG%C2%A0%C2%A0V0.1%C2%A0251209%281%29%281%29%281%29.pdf) |
| Reliance INR21700-RS50 | Estimated as 2.047 x published maximum 4.0 mOhm ACIR; no DCIR value is published. | [source](https://birikimpilleri.net/documents/contract/AB9F4D23-D6A7-CB6F-D0E2EB2F8468BB35.PDF) |
| Molicel INR-18650-P30B | Published typical DCIR at 50% SOC and 23 degC. | [source](https://www.molicel.com/wp-content/uploads/Product-Data-Sheet-of-INR-18650-P30B-80111-2.pdf) |
| Molicel INR-21700-P50B | Published typical DCIR at 50% SOC and 23 degC. | [source](https://www.molicel.com/wp-content/uploads/Product-Data-Sheet-of-INR-21700-P50B-80122.pdf) |

## Output guide

- `all_configuration_results.csv`: all simulated accepted topologies.
- `ranked_constraint_valid_results.csv`: completed packs with at least the reserve target.
- `pareto_frontier.csv`: non-dominated time/mass/heat results.
- `sensitivity_cases.csv`: centered local sensitivity reruns.
- `workbooks/`: one four-sheet workbook per identifiable cell.
- `plots/`: consolidated comparison plots.
