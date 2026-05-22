#!/usr/bin/env python3
"""Run DS-004: synthesize simulation sensitivity studies into design decisions."""

from __future__ import annotations

import csv
import math
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
REPORT_PATH = REPO_ROOT / "reports" / "DS-004-vehicle-design-synthesis.md"

DS001 = REPO_ROOT / "studies" / "DS-001-envelopesim-parameter-sensitivity"
DS002 = REPO_ROOT / "studies" / "DS-002-standardsim-steady-state-sensitivity"
DS003 = REPO_ROOT / "studies" / "DS-003-standardsim-transient-sensitivity"

ENVELOPE_KEY_METRICS = (
    "max_lateral_g__25mps",
    "max_accel_g__25mps",
    "max_brake_g__25mps",
    "ggv_area_g2__25mps",
    "mean_max_lateral_g",
    "mean_max_accel_g",
    "mean_max_brake_g",
    "mean_ggv_area_g2",
)

PLATFORM_LEVERS = (
    ("front_spring_rate_n_per_m", "front spring"),
    ("rear_spring_rate_n_per_m", "rear spring"),
    ("front_stabar_rate_n_m_per_rad", "front anti-roll bar"),
    ("rear_stabar_rate_n_m_per_rad", "rear anti-roll bar"),
    ("torsional_stiffness_n_m_per_rad", "body torsional stiffness"),
)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def as_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def format_float(value: Any, digits: int = 3) -> str:
    value = as_float(value)
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def table_line(values: list[str]) -> str:
    return "| " + " | ".join(values) + " |"


def metric_catalog(path: Path) -> dict[str, dict[str, str]]:
    rows = read_csv(path)
    return {row["metric"]: row for row in rows}


def active_metrics(path: Path, fallback: tuple[str, ...] = ()) -> list[str]:
    rows = read_csv(path)
    active = [
        row["metric"]
        for row in rows
        if str(row.get("active_in_current_report", "")).strip() in {"1", "true", "True"}
    ]
    return active or list(fallback)


def top_sensitivity_rows(
    study_id: str,
    study_label: str,
    sensitivity_csv: Path,
    metric_names: list[str],
    catalog: dict[str, dict[str, str]],
) -> list[dict[str, Any]]:
    rows = read_csv(sensitivity_csv)
    output = []
    for metric in metric_names:
        metric_rows = [row for row in rows if row["metric"] == metric]
        metric_rows.sort(
            key=lambda row: as_float(row.get("abs_effect_pct_span")),
            reverse=True,
        )
        if not metric_rows:
            continue
        row = metric_rows[0]
        catalog_row = catalog.get(metric, {})
        output.append(
            {
                "study": study_id,
                "study_label": study_label,
                "metric": metric,
                "display_name": catalog_row.get("display_name", metric),
                "unit": catalog_row.get("unit", catalog_row.get("units", "")),
                "top_parameter": row["parameter_label"],
                "top_parameter_name": row["parameter"],
                "direction": row["direction"],
                "signed_effect_pct_span": as_float(row["signed_effect_pct_span"]),
                "response_low": as_float(row["response_low"]),
                "response_baseline": as_float(row["response_baseline"]),
                "response_high": as_float(row["response_high"]),
            }
        )
    return output


def baseline_lookup(path: Path, key_field: str, value_field: str) -> dict[str, str]:
    return {row[key_field]: row[value_field] for row in read_csv(path)}


def sensitivity_lookup(paths: dict[str, Path]) -> dict[tuple[str, str, str], dict[str, str]]:
    lookup = {}
    for study_id, path in paths.items():
        for row in read_csv(path):
            lookup[(study_id, row["parameter"], row["metric"])] = row
    return lookup


def evidence_text(rows: list[dict[str, Any]], study: str, metric: str) -> str:
    row = next((r for r in rows if r["study"] == study and r["metric"] == metric), None)
    if row is None:
        return "evidence missing"
    return (
        f"{row['study']} {row['display_name']}: {row['top_parameter']} "
        f"{format_float(row['signed_effect_pct_span'], 2)}% "
        f"({format_float(row['response_low'], 4)} -> {format_float(row['response_high'], 4)})"
    )


def specific_evidence_text(
    row_lookup: dict[tuple[str, str, str], dict[str, str]],
    catalogs: dict[str, dict[str, dict[str, str]]],
    study: str,
    parameter: str,
    metric: str,
) -> str:
    row = row_lookup.get((study, parameter, metric))
    if row is None:
        return "evidence missing"
    catalog_row = catalogs.get(study, {}).get(metric, {})
    display_name = catalog_row.get("display_name", metric)
    return (
        f"{study} {display_name}: {row['parameter_label']} "
        f"{format_float(row['signed_effect_pct_span'], 2)}% "
        f"({format_float(row['response_low'], 4)} -> {format_float(row['response_high'], 4)})"
    )


def make_architecture_layers() -> list[dict[str, str]]:
    return [
        {
            "layer": "1. Capability envelope",
            "simulation_read": (
                "EnvelopeSim shows high-speed lateral and GGV area are driven by downforce, "
                "CG height, total mass, tire scale, and longitudinal force limits."
            ),
            "design_consequence": (
                "Set mass, CG, aero area, tire selection, power limit, and brake capacity before tuning feel."
            ),
        },
        {
            "layer": "2. Steady-state balance",
            "simulation_read": (
                "SteadyStateEval shows ay_max follows tire scale, while understeer/roll/sideslip "
                "follow aero load, CG placement, roll platform, and toe."
            ),
            "design_consequence": (
                "Use aero/CG/roll distribution to place the car in the right balance window."
            ),
        },
        {
            "layer": "3. Transient response",
            "simulation_read": (
                "TransientEval shows rack travel, toe, CG x, and mass placement dominate gain, "
                "phase, lag, and initial yaw/ay response."
            ),
            "design_consequence": (
                "Tune driver command authority and response speed after the balance target is sane."
            ),
        },
        {
            "layer": "4. Track trim",
            "simulation_read": (
                "Toe and brake bias are high-authority knobs, but broad sweeps show they can "
                "overpower interpretation if used too early."
            ),
            "design_consequence": (
                "Keep the baseline neutral/adjustable, then use small sweeps for final event-specific trim."
            ),
        },
    ]


def make_platform_summary(
    row_lookup: dict[tuple[str, str, str], dict[str, str]],
) -> list[dict[str, str]]:
    rows = []
    for parameter, label in PLATFORM_LEVERS:
        steady_ay = row_lookup[("DS-002", parameter, "ay_max")]
        steady_understeer = row_lookup[("DS-002", parameter, "understeer_gradient_deg_per_g")]
        steady_roll = row_lookup[("DS-002", parameter, "roll_gradient_deg_per_g")]
        transient_roll_peak = row_lookup[("DS-003", parameter, "step.roll_peak")]
        transient_roll_gain = row_lookup[("DS-003", parameter, "step.roll_gain_dc")]
        rows.append(
            {
                "lever": label,
                "steady_ay_max_pct": format_float(steady_ay["signed_effect_pct_span"], 2),
                "steady_understeer_pct": format_float(steady_understeer["signed_effect_pct_span"], 2),
                "steady_roll_gradient_pct": format_float(steady_roll["signed_effect_pct_span"], 2),
                "transient_roll_peak_pct": format_float(transient_roll_peak["signed_effect_pct_span"], 2),
                "transient_roll_gain_pct": format_float(transient_roll_gain["signed_effect_pct_span"], 2),
            }
        )
    return rows


def platform_read(row: dict[str, str]) -> str:
    ay = abs(as_float(row["steady_ay_max_pct"]))
    roll = max(abs(as_float(row["steady_roll_gradient_pct"])), abs(as_float(row["transient_roll_gain_pct"])))
    understeer = abs(as_float(row["steady_understeer_pct"]))
    if roll >= 8 and ay < 6:
        return "strong roll/platform lever, modest raw ay lever"
    if roll < 8 and ay < 3 and understeer < 3:
        return "secondary structure lever in this baseline sweep"
    return "balance lever with measurable ay/platform coupling"


def make_design_decisions(
    evidence_rows: list[dict[str, Any]],
    row_lookup: dict[tuple[str, str, str], dict[str, str]],
    catalogs: dict[str, dict[str, dict[str, str]]],
) -> list[dict[str, str]]:
    e = lambda study, metric: evidence_text(evidence_rows, study, metric)
    s = lambda study, parameter, metric: specific_evidence_text(row_lookup, catalogs, study, parameter, metric)
    return [
        {
            "decision": "Treat low mass and low CG as first-order architecture constraints.",
            "status": "strongly supported",
            "simulation_basis": (
                f"{s('DS-001', 'mass_kg', 'mean_max_accel_g')}; "
                f"{s('DS-001', 'mass_kg', 'ggv_area_g2__25mps')}; "
                f"{e('DS-001', 'mean_ggv_area_g2')}; "
                f"{e('DS-002', 'roll_gradient_deg_per_g')}"
            ),
            "design_implication": (
                "Packaging, driver placement, ballast, accumulator/fuel placement, and upright/unsprung mass "
                "are not housekeeping details; they directly set the usable envelope and roll response."
            ),
            "current_vehicle_position": (
                "Baseline mass rollup is 261.07 kg with CG height 0.2796 m and 48.35% front static load."
            ),
        },
        {
            "decision": "Keep the vehicle concept aero-forward, with aero balance owned as a primary design variable.",
            "status": "strongly supported",
            "simulation_basis": (
                f"{e('DS-001', 'max_lateral_g__25mps')}; "
                f"{e('DS-001', 'ggv_area_g2__25mps')}; "
                f"{e('DS-002', 'understeer_gradient_deg_per_g')}"
            ),
            "design_implication": (
                "Downforce is not cosmetic. It grows high-speed lateral capability and changes steady-state "
                "balance, so aero package and aero balance deserve early design ownership."
            ),
            "current_vehicle_position": (
                "Baseline EnvelopeSim uses ClA=2.347 m^2, CdA=1.173 m^2, and 50% front aero balance."
            ),
        },
        {
            "decision": "Protect tire quality and tire-load management before chasing secondary geometry changes.",
            "status": "strongly supported",
            "simulation_basis": (
                f"{e('DS-002', 'ay_max')}; "
                f"{s('DS-001', 'lateral_mu_scale', 'mean_ggv_area_g2')}; "
                f"{s('DS-001', 'lateral_load_sensitivity_scale', 'mean_ggv_area_g2')}"
            ),
            "design_implication": (
                "The tire model/test data and the platform that keeps tires in their usable load range are "
                "foundational. Tire scale moving ay_max by double-digit percent means tire uncertainty can "
                "overwhelm many chassis tweaks."
            ),
            "current_vehicle_position": "Current tire source is vehicles/current/tires/16x7p5_10_12psi.tir.",
        },
        {
            "decision": "Use springs and anti-roll bars as platform-control and balance tools, not as the main grip source.",
            "status": "supported",
            "simulation_basis": (
                f"{s('DS-002', 'front_stabar_rate_n_m_per_rad', 'roll_gradient_deg_per_g')}; "
                f"{s('DS-002', 'rear_stabar_rate_n_m_per_rad', 'roll_gradient_deg_per_g')}; "
                f"{s('DS-003', 'front_stabar_rate_n_m_per_rad', 'step.roll_gain_dc')}; "
                f"{s('DS-003', 'rear_stabar_rate_n_m_per_rad', 'step.roll_gain_dc')}"
            ),
            "design_implication": (
                "Spring and bar rates should be selected for aero platform, roll control, ride, and balance. "
                "They should not be expected to create large raw lateral capability by themselves."
            ),
            "current_vehicle_position": (
                "Baseline rates: front spring 26.27 kN/m, rear spring 43.78 kN/m, front bar 258.94 N*m/rad, "
                "rear bar 535.36 N*m/rad. DS-002 spring variants now preserve free length using FourPost motion ratios."
            ),
        },
        {
            "decision": "Keep static toe near zero for the baseline, and reserve toe as a late-stage trim knob.",
            "status": "strongly supported",
            "simulation_basis": (
                f"{e('DS-002', 'roadwheel_angle_gradient_deg_per_g')}; "
                f"{e('DS-003', 'step.ay_peak')}; "
                f"{e('DS-003', 'step.yaw_peak')}"
            ),
            "design_implication": (
                "Toe is powerful enough to shape steady and transient behavior, but also powerful enough to "
                "destabilize interpretation. Zero static toe is a sane baseline; final toe should be set by "
                "small, response-targeted sweeps."
            ),
            "current_vehicle_position": "Baseline front and rear static toe are both 0 deg.",
        },
        {
            "decision": "Treat rack travel as the driver-interface gain knob.",
            "status": "strongly supported",
            "simulation_basis": (
                f"{e('DS-002', 'handwheel_angle_gradient_deg_per_g')}; "
                f"{e('DS-002', 'handwheel_torque_peak_abs')}; "
                f"{e('DS-003', 'step.ay_gain_dc')}; "
                f"{e('DS-003', 'frequency.yaw_gain_peak')}"
            ),
            "design_implication": (
                "Rack travel directly scales driver command authority, handwheel angle, and effort. Changing it "
                "is not a hidden performance gain; it is a deliberate HMI/control-authority choice."
            ),
            "current_vehicle_position": "Baseline front rack travel is 0.0889 m/rev.",
        },
        {
            "decision": "Hold chassis torsional stiffness target unless structural packaging forces a change.",
            "status": "supported as sufficient for first baseline",
            "simulation_basis": (
                f"{s('DS-002', 'torsional_stiffness_n_m_per_rad', 'ay_max')}; "
                f"{s('DS-002', 'torsional_stiffness_n_m_per_rad', 'roll_gradient_deg_per_g')}; "
                f"{s('DS-003', 'torsional_stiffness_n_m_per_rad', 'step.roll_gain_dc')}"
            ),
            "design_implication": (
                "Current torsional stiffness is worth preserving, but the first-order handling story is elsewhere: "
                "tires, aero, CG, toe, rack travel, and roll platform."
            ),
            "current_vehicle_position": "Baseline body torsional stiffness is 300 kN*m/rad.",
        },
        {
            "decision": "Use brake bias and drive limits as envelope-shaping systems.",
            "status": "supported by EnvelopeSim",
            "simulation_basis": (
                f"{e('DS-001', 'max_accel_g__25mps')}; "
                f"{e('DS-001', 'max_brake_g__25mps')}; "
                f"{s('DS-001', 'brake_distribution_front', 'mean_max_brake_g')}"
            ),
            "design_implication": (
                "Power/force limits and brake bias are not downstream details; they define longitudinal envelope "
                "area and should remain adjustable during validation."
            ),
            "current_vehicle_position": "Baseline assumptions: 80 kW, 3735 N drive cap, RWD, 14 kN brake cap, 62% front brake distribution.",
        },
    ]


def make_control_map() -> list[dict[str, str]]:
    return [
        {
            "objective": "Increase high-speed lateral envelope",
            "primary_knobs": "downforce area, CG height, tire lateral capability",
            "simulation_basis": "DS-001 lat 25 and area 25; DS-002 ay_max",
            "design_use": "Aero and tire/load-transfer design problem, not a steering-rack problem.",
        },
        {
            "objective": "Tune steady-state understeer",
            "primary_knobs": "aero downforce scale/balance, CG x, roll stiffness distribution, static toe",
            "simulation_basis": "DS-002 understeer gradient and sideslip gradient sensitivities",
            "design_use": "Use aero/CG/roll distribution for architecture; use toe for final trim.",
        },
        {
            "objective": "Reduce roll response",
            "primary_knobs": "CG z, springs, anti-roll bars, torsional stiffness",
            "simulation_basis": "DS-002 roll gradient and DS-003 roll peak/gain",
            "design_use": "Springs/bars are legitimate roll-platform tools; torsion is a secondary preservation target.",
        },
        {
            "objective": "Shape driver steering feel and command authority",
            "primary_knobs": "rack travel per revolution",
            "simulation_basis": "DS-002 handwheel angle/torque and DS-003 gain metrics",
            "design_use": "Set rack travel to driver target after vehicle balance is known.",
        },
        {
            "objective": "Improve transient phase/lag",
            "primary_knobs": "sprung CG x, sprung mass, toe",
            "simulation_basis": "DS-003 frequency phase/lag metrics",
            "design_use": "Use mass placement as the architecture lever; use toe carefully for track tuning.",
        },
        {
            "objective": "Improve acceleration/braking envelope",
            "primary_knobs": "drive power, drive force cap, brake force, brake distribution, CG height",
            "simulation_basis": "DS-001 acceleration/braking sensitivities",
            "design_use": "Maintain brake-bias adjustability and validate power/traction limits.",
        },
    ]


def make_open_questions() -> list[dict[str, str]]:
    return [
        {
            "question": "What aero balance gives the best understeer/phase tradeoff?",
            "why_it_matters": "Downforce area and downforce scale are high-value, but DS-002 varied scale rather than balance.",
            "next_study": "StandardSim aero balance and ride-height/downforce map sweep.",
        },
        {
            "question": "What is the fine static toe window?",
            "why_it_matters": "Toe strongly moves steady and transient metrics; +/-1 deg is intentionally broad.",
            "next_study": "Small-range toe sweep around zero, likely +/-0.2 deg with both axles.",
        },
        {
            "question": "How should camber be treated with tire data uncertainty?",
            "why_it_matters": "Static camber was not a top first-pass driver, but real tires may make camber window important.",
            "next_study": "Tire model validation plus camber/load sensitivity sweep.",
        },
        {
            "question": "Where is the spring/bar optimum after aero platform constraints are imposed?",
            "why_it_matters": "Springs/bars affect roll and understeer, but need ride-height/aero constraints to become a design optimum.",
            "next_study": "Coupled ride/roll/aero-platform optimization using StandardSim and FourPost motion ratios.",
        },
        {
            "question": "Can transient threshold metrics be made robust enough for optimization?",
            "why_it_matters": "Some rise/overshoot metrics are sign-sensitive in broad sweeps, so active DS-003 metrics avoided them.",
            "next_study": "Refine transient response metrics using absolute response or monotonic target definitions.",
        },
    ]


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)

    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")

    ds001_catalog = metric_catalog(DS001 / "outputs" / "metric_catalog.csv")
    ds002_catalog = metric_catalog(DS002 / "outputs" / "metric_catalog.csv")
    ds003_catalog = metric_catalog(DS003 / "outputs" / "metric_catalog.csv")
    catalogs = {
        "DS-001": ds001_catalog,
        "DS-002": ds002_catalog,
        "DS-003": ds003_catalog,
    }
    row_lookup = sensitivity_lookup(
        {
            "DS-001": DS001 / "outputs" / "metric_sensitivity_matrix.csv",
            "DS-002": DS002 / "outputs" / "metric_sensitivity_matrix.csv",
            "DS-003": DS003 / "outputs" / "metric_sensitivity_matrix.csv",
        }
    )

    source_top_rows = []
    source_top_rows.extend(
        top_sensitivity_rows(
            "DS-001",
            "EnvelopeSim",
            DS001 / "outputs" / "metric_sensitivity_matrix.csv",
            list(ENVELOPE_KEY_METRICS),
            ds001_catalog,
        )
    )
    source_top_rows.extend(
        top_sensitivity_rows(
            "DS-002",
            "StandardSim SteadyStateEval",
            DS002 / "outputs" / "metric_sensitivity_matrix.csv",
            active_metrics(DS002 / "outputs" / "metric_catalog.csv"),
            ds002_catalog,
        )
    )
    source_top_rows.extend(
        top_sensitivity_rows(
            "DS-003",
            "StandardSim TransientEval",
            DS003 / "outputs" / "metric_sensitivity_matrix.csv",
            active_metrics(DS003 / "outputs" / "metric_catalog.csv"),
            ds003_catalog,
        )
    )

    write_csv(
        OUTPUT_DIR / "source_top_sensitivities.csv",
        source_top_rows,
        [
            "study",
            "study_label",
            "metric",
            "display_name",
            "unit",
            "top_parameter",
            "top_parameter_name",
            "direction",
            "signed_effect_pct_span",
            "response_low",
            "response_baseline",
            "response_high",
        ],
    )

    architecture_layers = make_architecture_layers()
    platform_summary = make_platform_summary(row_lookup)
    for row in platform_summary:
        row["design_read"] = platform_read(row)
    design_decisions = make_design_decisions(source_top_rows, row_lookup, catalogs)
    control_map = make_control_map()
    open_questions = make_open_questions()

    write_csv(
        OUTPUT_DIR / "architecture_layers.csv",
        architecture_layers,
        ["layer", "simulation_read", "design_consequence"],
    )
    write_csv(
        OUTPUT_DIR / "platform_summary.csv",
        platform_summary,
        [
            "lever",
            "steady_ay_max_pct",
            "steady_understeer_pct",
            "steady_roll_gradient_pct",
            "transient_roll_peak_pct",
            "transient_roll_gain_pct",
            "design_read",
        ],
    )
    write_csv(
        OUTPUT_DIR / "design_decisions.csv",
        design_decisions,
        [
            "decision",
            "status",
            "simulation_basis",
            "design_implication",
            "current_vehicle_position",
        ],
    )
    write_csv(
        OUTPUT_DIR / "control_map.csv",
        control_map,
        ["objective", "primary_knobs", "simulation_basis", "design_use"],
    )
    write_csv(
        OUTPUT_DIR / "open_questions.csv",
        open_questions,
        ["question", "why_it_matters", "next_study"],
    )

    baseline_characterization = baseline_lookup(
        DS001 / "outputs" / "baseline_characterization.csv",
        "item",
        "value",
    )
    envelope_baseline = baseline_lookup(
        DS001 / "outputs" / "baseline_metrics.csv",
        "metric",
        "value",
    )
    steady_baseline = baseline_lookup(
        DS002 / "outputs" / "baseline_metrics.csv",
        "metric",
        "value",
    )
    transient_baseline = baseline_lookup(
        DS003 / "outputs" / "baseline_metrics.csv",
        "metric",
        "value",
    )

    top_counter = Counter(row["top_parameter"] for row in source_top_rows)
    top_counter_rows = top_counter.most_common(8)

    lines: list[str] = []
    lines.append("# DS-004 Vehicle Design Synthesis and Justification")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Thesis")
    lines.append("")
    lines.append(
        "The current vehicle is justified as a low-CG, aero-forward, tire-limited, "
        "rear-drive formula-style car whose primary architecture is set by mass "
        "properties, tire capability, aero load, and longitudinal force limits. "
        "StandardSim then shows that the late-stage handling knobs should be "
        "alignment, steering rack travel, roll-platform rates, aero balance, and "
        "small CG-placement adjustments."
    )
    lines.append("")
    lines.append(
        "In plain human terms: the big rocks are low/central mass, downforce, tire "
        "quality, and usable power/brake force. The spicy knobs are toe and rack "
        "travel. Springs and bars are real, but they are platform/balance tools, "
        "not magic grip buttons."
    )
    lines.append("")
    lines.append("## Simulation Basis")
    lines.append("")
    lines.append(table_line(["Study", "Role", "Source"]))
    lines.append(table_line(["---", "---", "---"]))
    lines.append(table_line(["DS-001", "Capability envelope and first-order architecture", "`studies/DS-001-envelopesim-parameter-sensitivity/`"]))
    lines.append(table_line(["DS-002", "Steady-state handling response sensitivities", "`studies/DS-002-standardsim-steady-state-sensitivity/`"]))
    lines.append(table_line(["DS-003", "Transient response sensitivities", "`studies/DS-003-standardsim-transient-sensitivity/`"]))
    lines.append("")
    lines.append("## Baseline Position")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---:"]))
    lines.append(table_line(["Mass", f"{format_float(baseline_characterization.get('mass'), 2)} kg"]))
    lines.append(table_line(["CG height", f"{format_float(baseline_characterization.get('cg_height'), 4)} m"]))
    lines.append(table_line(["Front static fraction", format_float(baseline_characterization.get("front_static_frac"), 4)]))
    lines.append(table_line(["Wheelbase", f"{format_float(baseline_characterization.get('wheelbase'), 4)} m"]))
    lines.append(table_line(["Track front/rear", f"{format_float(baseline_characterization.get('track_front'), 4)} / {format_float(baseline_characterization.get('track_rear'), 4)} m"]))
    lines.append(table_line(["ClA / CdA", f"{format_float(baseline_characterization.get('cl_a'), 4)} / {format_float(baseline_characterization.get('cd_a'), 4)} m^2"]))
    lines.append(table_line(["Aero balance front", format_float(baseline_characterization.get("aero_balance_front"), 4)]))
    lines.append(table_line(["Power / drive cap", f"{format_float(baseline_characterization.get('max_drive_power'), 0)} W / {format_float(baseline_characterization.get('max_drive_force'), 0)} N"]))
    lines.append(table_line(["Brake cap / front brake bias", f"{format_float(baseline_characterization.get('max_brake_force'), 0)} N / {format_float(baseline_characterization.get('brake_distribution_front'), 3)}"]))
    lines.append("")
    lines.append("## Baseline Response Summary")
    lines.append("")
    lines.append(table_line(["Response", "Value"]))
    lines.append(table_line(["---", "---:"]))
    lines.append(table_line(["Envelope lat 25 m/s", f"{format_float(envelope_baseline.get('max_lateral_g__25mps'), 3)} g"]))
    lines.append(table_line(["Envelope mean lateral", f"{format_float(envelope_baseline.get('mean_max_lateral_g'), 3)} g"]))
    lines.append(table_line(["Envelope mean accel", f"{format_float(envelope_baseline.get('mean_max_accel_g'), 3)} g"]))
    lines.append(table_line(["Envelope mean brake", f"{format_float(envelope_baseline.get('mean_max_brake_g'), 3)} g"]))
    lines.append(table_line(["Envelope mean GGV area", f"{format_float(envelope_baseline.get('mean_ggv_area_g2'), 3)} g^2"]))
    lines.append(table_line(["StandardSim ay_max", f"{format_float(steady_baseline.get('ay_max'), 3)} m/s^2"]))
    lines.append(table_line(["StandardSim understeer gradient", f"{format_float(steady_baseline.get('understeer_gradient_deg_per_g'), 3)} deg/g"]))
    lines.append(table_line(["StandardSim roll gradient", f"{format_float(steady_baseline.get('roll_gradient_deg_per_g'), 3)} deg/g"]))
    lines.append(table_line(["Transient step ay peak", f"{format_float(transient_baseline.get('step.ay_peak'), 3)} m/s^2"]))
    lines.append(table_line(["Transient yaw gain DC", f"{format_float(transient_baseline.get('step.yaw_gain_dc'), 3)} (rad/s)/rad"]))
    lines.append(table_line(["Transient ay lag at 1 Hz", f"{format_float(transient_baseline.get('frequency.ay_lag_1hz'), 4)} s"]))
    lines.append("")
    lines.append("## Repeated First-Order Levers")
    lines.append("")
    lines.append(table_line(["Lever", "Top-response count"]))
    lines.append(table_line(["---", "---:"]))
    for lever, count in top_counter_rows:
        lines.append(table_line([lever, str(count)]))
    lines.append("")
    lines.append("This count is not an optimization score; it is a sanity check for which variables keep reappearing as the strongest single-factor levers.")
    lines.append("")
    lines.append("## Ground-Up Design Logic")
    lines.append("")
    lines.append(table_line(["Layer", "Simulation read", "Design consequence"]))
    lines.append(table_line(["---", "---", "---"]))
    for row in architecture_layers:
        lines.append(table_line([row["layer"], row["simulation_read"], row["design_consequence"]]))
    lines.append("")
    lines.append("## Design Decisions")
    lines.append("")
    for idx, decision in enumerate(design_decisions, start=1):
        lines.append(f"### {idx}. {decision['decision']}")
        lines.append("")
        lines.append(table_line(["Field", "Read"]))
        lines.append(table_line(["---", "---"]))
        lines.append(table_line(["Status", decision["status"]]))
        lines.append(table_line(["Simulation basis", decision["simulation_basis"]]))
        lines.append(table_line(["Design implication", decision["design_implication"]]))
        lines.append(table_line(["Current vehicle position", decision["current_vehicle_position"]]))
        lines.append("")
    lines.append("## Platform Evidence")
    lines.append("")
    lines.append(
        "Signed values are low-to-high parameter spans as a percent of baseline response. "
        "This table is why springs and bars are treated as platform/balance tools rather than first-order raw-grip generators."
    )
    lines.append("")
    lines.append(table_line([
        "Lever",
        "ay_max",
        "understeer",
        "steady roll",
        "step roll peak",
        "step roll gain",
        "Read",
    ]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:", "---"]))
    for row in platform_summary:
        lines.append(table_line([
            row["lever"],
            f"{row['steady_ay_max_pct']}%",
            f"{row['steady_understeer_pct']}%",
            f"{row['steady_roll_gradient_pct']}%",
            f"{row['transient_roll_peak_pct']}%",
            f"{row['transient_roll_gain_pct']}%",
            row["design_read"],
        ]))
    lines.append("")
    lines.append("## Control Map")
    lines.append("")
    lines.append(table_line(["Objective", "Primary knobs", "Design use"]))
    lines.append(table_line(["---", "---", "---"]))
    for row in control_map:
        lines.append(table_line([row["objective"], row["primary_knobs"], row["design_use"]]))
    lines.append("")
    lines.append("## Open Questions")
    lines.append("")
    lines.append(table_line(["Question", "Why it matters", "Next study"]))
    lines.append(table_line(["---", "---", "---"]))
    for row in open_questions:
        lines.append(table_line([row["question"], row["why_it_matters"], row["next_study"]]))
    lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative_path in [
        "outputs/source_top_sensitivities.csv",
        "outputs/architecture_layers.csv",
        "outputs/platform_summary.csv",
        "outputs/design_decisions.csv",
        "outputs/control_map.csv",
        "outputs/open_questions.csv",
        "RESULTS.md",
    ]:
        lines.append(f"- `{relative_path}`")
    lines.append("")

    text = "\n".join(lines)
    (STUDY_DIR / "RESULTS.md").write_text(text, encoding="utf-8")
    REPORT_PATH.write_text(text, encoding="utf-8")
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}")
    print(f"Top-level report: {REPORT_PATH}")


if __name__ == "__main__":
    main()
