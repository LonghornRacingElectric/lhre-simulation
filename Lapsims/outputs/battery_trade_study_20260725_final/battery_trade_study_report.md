# OpenLAP Battery Cell and Pack Trade Study

Run date: 2026-07-25<br>
Event distance: 22.000 km<br>
Vehicle/track: fixed LHRe OpenLAP Michigan endurance model<br>
Configurations simulated: 128 feasible topologies within the stated study grid<br>
Topologies rejected before simulation: 131

## Executive result

The fastest constraint-valid result is **Tenpower INR21700-60XG 125s3p** at **1236.40 s** (20.61 min), using a **35.20 kW** terminal-power ceiling and finishing with **0.497 kWh** of modeled usable chemical energy.
Against the team 7.2 kWh-class 130s5p P30B baseline (1271.39 s), this is **2.752% faster** (34.99 s saved).
The requested 7.2 kWh baseline is instantiated from the actual 130s5p P30B cell data as **7.020 kWh nominal** and **6.881 kWh modeled usable** from 100% to the 5% SOC floor.

**Decision caution:** the raw pace winner is based on Tenpower's V0.1 draft specification and an ACIR-derived DCIR estimate. Do not freeze the design around it until pulse resistance, capacity, and thermal behavior are reproduced on the team fixture.

The winner is **8.100 kWh nominal** but **7.939 kWh modeled usable** from 100% to the 5% SOC floor. It passes this study's literal 8.0 kWh *usable-energy* constraint; if Battery intended an 8.0 kWh nominal/nameplate cap, exclude it and use **Reliance INR21700-RS50 110s4p** as the fastest remaining native result (1237.56 s).
The winner's **64.0 degC** adiabatic rise would imply about **89 degC** from a 25 degC start, above the draft specification's 80 degC cell-surface discharge range. This is an uncooled upper bound, not a cooled peak prediction, but the model has no thermal feedback or derating; cooling validation is mandatory and the ranking can change.
The better-documented, non-draft A0 Reliance result is only 1.16 s slower and produces 0.1805 kWh less modeled heat. Its DCIR is still ACIR-derived. The fastest candidate with manufacturer-published DCIR is **Molicel INR-21700-P50B 110s4p** at **1244.94 s**.

## Fastest-configuration metrics

| Metric | Result |
|---|---:|
| Cell count | 375 (125s3p) |
| Maximum charged voltage | 525.0 V |
| Nominal / modeled usable energy | 8.100 / 7.939 kWh |
| Pack / vehicle mass | 35.625 / 258.510 kg |
| Total endurance / equivalent average lap | 1236.396 / 60.132 s |
| Fastest / slowest completed lap | 60.085 / 61.254 s |
| Terminal power, average / peak | 20.194 / 35.203 kW |
| Motor-shaft power, average / peak | 17.440 / 31.863 kW |
| Cell current, RMS / peak | 19.167 / 31.726 A |
| Peak pack current | 95.178 A |
| Terminal / chemical / mechanical energy | 6.936 / 7.442 / 5.990 kWh |
| Pack resistive heat | 0.5067 kWh |
| Battery discharge efficiency | 93.192% |
| Adiabatic cell temperature rise | 64.0 degC |
| Remaining modeled usable energy | 0.497 kWh |

## Recommended configurations

| Category | Configuration | Time (s) | Pack mass (kg) | Heat (kWh) | Reserve (kWh) |
|---|---|---:|---:|---:|---:|
| Best overall | Tenpower INR21700-60XG 125s3p | 1236.40 | 35.62 | 0.5067 | 0.497 |
| Best lightweight within 1% of fastest | Tenpower INR21700-60XG 120s3p | 1244.12 | 34.20 | 0.4645 | 0.503 |
| Lowest heat within 2% of fastest | Tenpower INR21700-50XG 135s3p | 1258.68 | 37.97 | 0.2739 | 0.496 |
| Best simulated performance per pack kg | Reliance INR21700-RS50 110s2p | 1454.68 | 18.43 | 0.0862 | 0.499 |

## Sensitivities

The most decision-useful local sensitivities below come from centered reruns around the best configuration while retuning power to the same reserve target:

- Usable energy: **-21.29 s/kWh**.
- Pack mass: **0.980 s/kg**.
- Cell DCIR: **1.614 s per mOhm/cell** (**0.037 s/mOhm** on an initial pack-resistance basis).
- These are centered derivative probes with power retuned to the same reserve target, not additional selectable pack designs. In particular, the +5% capacity probe exceeds the study's 8.0 kWh usable-energy cap.

A cross-configuration linear fit gives R² = 0.951 over 128 constraint-valid screening cases. Because topology variables are correlated, the local finite differences should drive design decisions.

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
- No regeneration, driver-change stop, thermal derating, or SOC-dependent motor map beyond the coupled pack-voltage/current and EMRAX field-weakening model.

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
