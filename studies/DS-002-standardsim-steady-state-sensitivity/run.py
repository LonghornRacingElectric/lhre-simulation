#!/usr/bin/env python3
"""Run DS-002: StandardSim SteadyStateEval parameter sensitivity."""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
WORK_DIR = STUDY_DIR / "work"
POPULATION_DIR = WORK_DIR / "population"
REPORT_PATH = REPO_ROOT / "reports" / "DS-002-standardsim-steady-state-sensitivity.md"

BOBSIM_ROOT = REPO_ROOT / "BobSim"
BOBLIB_PACKAGE = BOBSIM_ROOT / "_0_Utils" / "external" / "BobLib" / "BobLib" / "package.mo"
GENERATION_SCRIPTS = (
    BOBSIM_ROOT / "_0_Utils" / "external" / "BobLib" / "Generation" / "scripts"
)

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))


def require_dependencies() -> tuple[Any, Any, Any, Any, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        import pandas as pd
        import scipy
        import yaml
    except ModuleNotFoundError as exc:
        missing = exc.name or "a required package"
        raise SystemExit(
            f"Missing Python dependency: {missing}\n"
            "Install the StandardSim study dependencies in the external environment:\n"
            "  /tmp/lhre-sim-venv/bin/python -m pip install PyYAML numpy matplotlib pandas scipy"
        ) from exc

    return np, plt, pd, scipy, yaml


np, plt, pd, scipy, yaml = require_dependencies()

sys.path.insert(0, str(GENERATION_SCRIPTS))
sys.path.insert(0, str(BOBSIM_ROOT))

from build_common import load_yaml  # noqa: E402
from build_records import render_record  # noqa: E402
from _3_StandardSim.FourPostEval.four_post_eval_sim import FourPostEvalSim  # noqa: E402
from _4_OptSim.pipeline.steady_state_eval_report import build_report_config  # noqa: E402


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    label: str
    unit: str
    baseline: float
    low: float
    high: float
    group: str
    description: str
    apply: Callable[[str, float], str]


@dataclass
class CaseResult:
    case_id: str
    parameter: str
    level: str
    value: float | str
    variant_dir: Path
    status: str
    metrics: dict[str, float]
    error: str = ""
    elapsed_s: float = 0.0


@dataclass(frozen=True)
class AxleSpringSetup:
    axle: str
    motion_ratio_wheel_per_spring: float
    installed_length_m: float
    sprung_corner_load_n: float
    spring_force_n: float
    baseline_rate_n_per_m: float
    baseline_free_length_vehicle_m: float
    baseline_compression_m: float
    configured_static_length_m: float
    fourpost_free_length_at_baseline_rate_m: float

    def free_length_for_rate(self, spring_rate_n_per_m: float) -> float:
        if spring_rate_n_per_m <= 0.0:
            raise ValueError("Spring rate must be positive.")
        return self.configured_static_length_m + self.spring_force_n / spring_rate_n_per_m


STANDARD_CFG = {
    "model": "BobLib.Standards.VehicleSim",
    "start_time": 0.5,
    "stop_time": 10,
    "intervals": 0,
    "tolerance": 1e-6,
    "solver": "dassl",
    "command_line_options": ["--simCodeTarget=C --maxSizeLinearTearing=5000"],
}

FOURPOST_CFG = {
    "model": "BobLib.Standards.FourPostSim",
    "start_time": 0,
    "stop_time": 113,
    "intervals": None,
    "tolerance": 1e-6,
    "solver": "dassl",
    "command_line_options": [
        "--simCodeTarget=C",
        "--maxSizeLinearTearing=5000",
        "--indexReductionMethod=dynamicStateSelection",
        "--matchingAlgorithm=PFPlusExt",
        "-d=NLSanalyticJacobian",
    ],
}

IMPORTANT_RESPONSE_METRICS = (
    "ay_max",
    "roadwheel_angle_gradient_deg_per_g",
    "handwheel_angle_gradient_deg_per_g",
    "sideslip_gradient_deg_per_g",
    "understeer_gradient_deg_per_g",
    "roll_gradient_deg_per_g",
    "handwheel_torque_peak_abs",
)

DERIVED_METRIC_UNITS = {
    "handwheel_torque_peak_abs": "N*m",
}

DERIVED_METRIC_DESCRIPTIONS = {
    "handwheel_torque_peak_abs": (
        "Maximum absolute handwheel torque over the SteadyStateEval sweep, "
        "derived from StandardSim torque extrema"
    ),
}


def as_repo_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def format_float(value: float, digits: int = 4) -> str:
    if not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.{digits}f}"


def modelica_value(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return f"{float(value):.12g}"
    if isinstance(value, (list, tuple)):
        return "{" + ", ".join(modelica_value(v) for v in value) + "}"
    raise TypeError(f"Cannot convert to Modelica value: {value!r}")


def find_block_span(text: str, block: str) -> tuple[int, int, int, int]:
    pattern = re.compile(rf"\b{re.escape(block)}\b")
    match = pattern.search(text)
    if match is None:
        raise ValueError(f"Block {block!r} not found in Modelica record.")

    paren_open = text.find("(", match.end())
    if paren_open == -1:
        raise ValueError(f"Block {block!r} has no opening parenthesis.")

    depth = 1
    index = paren_open + 1
    while index < len(text) and depth > 0:
        char = text[index]
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
        index += 1

    if depth != 0:
        raise ValueError(f"Block {block!r} has unbalanced parentheses.")

    paren_close = index - 1
    return match.start(), paren_open + 1, paren_close, index


def find_assignment_span(block_body: str, param: str) -> tuple[int, int, int]:
    pattern = re.compile(rf"\b{re.escape(param)}\b\s*=")
    match = pattern.search(block_body)
    if match is None:
        raise ValueError(f"Parameter {param!r} not found in block body.")

    value_start = match.end()
    while value_start < len(block_body) and block_body[value_start].isspace():
        value_start += 1

    paren_depth = 0
    brace_depth = 0
    bracket_depth = 0
    value_end = value_start

    while value_end < len(block_body):
        char = block_body[value_end]
        if char == "(":
            paren_depth += 1
        elif char == ")":
            if paren_depth == 0 and brace_depth == 0 and bracket_depth == 0:
                break
            paren_depth -= 1
        elif char == "{":
            brace_depth += 1
        elif char == "}":
            brace_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif (
            char == ","
            and paren_depth == 0
            and brace_depth == 0
            and bracket_depth == 0
        ):
            break
        value_end += 1

    return match.start(), value_start, value_end


def get_block_param(text: str, block: str, param: str) -> str:
    _block_start, body_start, body_end, _block_end = find_block_span(text, block)
    block_body = text[body_start:body_end]
    _assign_start, value_start, value_end = find_assignment_span(block_body, param)
    return block_body[value_start:value_end].strip()


def set_block_param_text(text: str, block: str, param: str, value_text: str) -> str:
    _block_start, body_start, body_end, _block_end = find_block_span(text, block)
    block_body = text[body_start:body_end]
    _assign_start, value_start, value_end = find_assignment_span(block_body, param)
    new_body = block_body[:value_start] + value_text + block_body[value_end:]
    return text[:body_start] + new_body + text[body_end:]


def set_block_param(text: str, block: str, param: str, value: Any) -> str:
    return set_block_param_text(text, block, param, modelica_value(value))


def set_record_parameter(text: str, param: str, value: Any) -> str:
    pattern = re.compile(
        rf"(\bparameter\b[^;\n]*\b{re.escape(param)}\b\s*=\s*)([^;]+)(;)"
    )
    replacement = rf"\g<1>{modelica_value(value)}\3"
    new_text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise ValueError(f"Top-level record parameter {param!r} not found.")
    return new_text


def scale_numbers(value_text: str, scale: float) -> str:
    number_re = re.compile(
        r"(?<![A-Za-z_])[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"
    )

    def repl(match: re.Match[str]) -> str:
        return f"{float(match.group(0)) * scale:.12g}"

    return number_re.sub(repl, value_text)


def scale_block_param_numbers(text: str, block: str, param: str, scale: float) -> str:
    value_text = get_block_param(text, block, param)
    return set_block_param_text(text, block, param, scale_numbers(value_text, scale))


def set_spring_rate(
    text: str,
    block: str,
    spring_rate: float,
    spring_free_length: float | None = None,
) -> str:
    text = set_block_param_text(
        text,
        block,
        "springTable",
        f"[0, 0; 1, {spring_rate:.12g}]",
    )
    if spring_free_length is not None:
        text = set_block_param(text, block, "springFreeLength", spring_free_length)
    return text


def set_mass_record_r_cm_component(
    text: str,
    block: str,
    component_index: int,
    value: float,
) -> str:
    current = get_block_param(text, block, "rCM")
    numbers = [float(token) for token in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", current)]
    if len(numbers) != 3:
        raise ValueError(f"Expected a three-component rCM in {block}, found {current!r}")
    numbers[component_index] = value
    return set_block_param(text, block, "rCM", numbers)


def apply_lateral_tire_mu_scale(text: str, scale: float) -> str:
    for block in ("pFrTireModel", "pRrTireModel"):
        text = set_block_param(text, block, "LMUY", scale)
    return text


def apply_aero_downforce_scale(text: str, scale: float) -> str:
    text = scale_block_param_numbers(text, "pAero", "downforceTable", scale)
    text = scale_block_param_numbers(text, "pAero", "myTable", scale)
    return text


def apply_aero_drag_scale(text: str, scale: float) -> str:
    return scale_block_param_numbers(text, "pAero", "dragTable", scale)


def read_vehicle(path: Path) -> dict[str, Any]:
    data = load_yaml(path)
    if not isinstance(data, dict):
        raise TypeError(f"Expected mapping at top level: {path}")
    return data


def spring_rate_from_vehicle(vehicle: dict[str, Any], axle: str) -> float:
    table = vehicle[axle]["actuation"]["shock"]["spring_table"]["table"]
    return float(table[-1][1])


def spring_free_length_from_vehicle(vehicle: dict[str, Any], axle: str) -> float:
    return float(vehicle[axle]["actuation"]["shock"]["free_length_m"])


def nested_value(data: dict[str, Any], *keys: str) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            raise KeyError(".".join(keys))
        cur = cur[key]
    return cur


def combine_sprung_mass(vehicle: dict[str, Any]) -> tuple[float, np.ndarray]:
    base_m = float(nested_value(vehicle, "sprung_mass", "mass_kg"))
    base_cg = np.asarray(nested_value(vehicle, "sprung_mass", "cg_m"), dtype=float)

    driver = vehicle.get("driver_mass")
    if isinstance(driver, dict):
        driver_m = float(driver["mass_kg"])
        driver_cg = np.asarray(driver["cg_m"], dtype=float)
    else:
        driver_m = 0.0
        driver_cg = np.zeros(3, dtype=float)

    total_m = base_m + driver_m
    if total_m <= 0.0:
        raise ValueError("Combined sprung mass must be positive.")

    total_cg = (base_m * base_cg + driver_m * driver_cg) / total_m
    return total_m, total_cg


def shock_installed_length(vehicle: dict[str, Any], axle: str) -> float:
    actuation = nested_value(vehicle, axle, "actuation")
    shock_mount = np.asarray(nested_value(actuation, "shock", "mount_m"), dtype=float)
    if "bellcrank" in actuation:
        shock_pickup = np.asarray(
            nested_value(actuation, "bellcrank", "pickups_m", "shock"),
            dtype=float,
        )
        return float(np.linalg.norm(shock_mount - shock_pickup))

    rod_mount = np.asarray(nested_value(actuation, "rod_mount_m"), dtype=float)
    return float(np.linalg.norm(shock_mount - rod_mount))


def wheelbase_and_tracks(vehicle: dict[str, Any]) -> tuple[float, float, float]:
    front_wc = np.asarray(nested_value(vehicle, "front", "suspension", "wheel_center_m"), dtype=float)
    rear_wc = np.asarray(nested_value(vehicle, "rear", "suspension", "wheel_center_m"), dtype=float)
    wheelbase = abs(float(front_wc[0] - rear_wc[0]))
    track_front = 2.0 * abs(float(front_wc[1]))
    track_rear = 2.0 * abs(float(rear_wc[1]))
    return wheelbase, track_front, track_rear


def axle_sprung_corner_loads(vehicle: dict[str, Any], axle: str) -> tuple[float, float]:
    sprung_mass_kg, sprung_cg_m = combine_sprung_mass(vehicle)
    front_wc = np.asarray(nested_value(vehicle, "front", "suspension", "wheel_center_m"), dtype=float)
    rear_wc = np.asarray(nested_value(vehicle, "rear", "suspension", "wheel_center_m"), dtype=float)

    front_x = float(front_wc[0])
    rear_x = float(rear_wc[0])
    left_y = 0.5 * (float(front_wc[1]) + float(rear_wc[1]))
    right_y = -left_y

    if abs(front_x - rear_x) <= 1e-12:
        raise ValueError("Vehicle wheelbase must be positive.")
    if abs(left_y - right_y) <= 1e-12:
        raise ValueError("Vehicle track width must be positive.")

    front_fraction = (float(sprung_cg_m[0]) - rear_x) / (front_x - rear_x)
    rear_fraction = 1.0 - front_fraction
    left_fraction = (float(sprung_cg_m[1]) - right_y) / (left_y - right_y)
    right_fraction = 1.0 - left_fraction

    axle_fraction = front_fraction if axle == "front" else rear_fraction
    left_load = sprung_mass_kg * axle_fraction * left_fraction * 9.80665
    right_load = sprung_mass_kg * axle_fraction * right_fraction * 9.80665
    return float(left_load), float(right_load)


def make_fourpost_vehicle_config(vehicle: dict[str, Any]) -> dict[str, float]:
    _sprung_mass_kg, sprung_cg_m = combine_sprung_mass(vehicle)
    wheelbase, track_front, track_rear = wheelbase_and_tracks(vehicle)
    return {
        "mass": float(nested_value(vehicle, "sprung_mass", "mass_kg")),
        "h_cg": float(sprung_cg_m[2]),
        "track_front": track_front,
        "track_rear": track_rear,
        "wheelbase": wheelbase,
    }


def make_spring_setups(
    vehicle: dict[str, Any],
    motion_ratios: dict[str, float],
) -> dict[str, AxleSpringSetup]:
    setups: dict[str, AxleSpringSetup] = {}
    for axle in ("front", "rear"):
        motion_ratio = float(motion_ratios[axle])
        if not math.isfinite(motion_ratio) or motion_ratio <= 0.0:
            raise ValueError(f"Invalid FourPost motion ratio for {axle}: {motion_ratio}")

        left_load, right_load = axle_sprung_corner_loads(vehicle, axle)
        sprung_corner_load = 0.5 * (left_load + right_load)
        spring_force = sprung_corner_load * motion_ratio
        baseline_rate = spring_rate_from_vehicle(vehicle, axle)
        installed_length = shock_installed_length(vehicle, axle)
        baseline_free_length = spring_free_length_from_vehicle(vehicle, axle)
        baseline_compression = spring_force / baseline_rate
        configured_static_length = baseline_free_length - baseline_compression
        fourpost_free_length = installed_length + baseline_compression

        setups[axle] = AxleSpringSetup(
            axle=axle,
            motion_ratio_wheel_per_spring=motion_ratio,
            installed_length_m=installed_length,
            sprung_corner_load_n=sprung_corner_load,
            spring_force_n=spring_force,
            baseline_rate_n_per_m=baseline_rate,
            baseline_free_length_vehicle_m=baseline_free_length,
            baseline_compression_m=baseline_compression,
            configured_static_length_m=configured_static_length,
            fourpost_free_length_at_baseline_rate_m=fourpost_free_length,
        )
    return setups


def make_baseline_record(vehicle_path: Path) -> tuple[str, dict[str, Any]]:
    vehicle = read_vehicle(vehicle_path)
    record_text = render_record(vehicle, vehicle_path)
    return record_text, vehicle


def make_parameter_specs(
    vehicle: dict[str, Any],
    spring_setups: dict[str, AxleSpringSetup] | None = None,
) -> list[ParameterSpec]:
    front_spring = spring_rate_from_vehicle(vehicle, "front")
    rear_spring = spring_rate_from_vehicle(vehicle, "rear")
    front_stabar = float(vehicle["front"]["actuation"]["stabar"]["rate_n_m_per_rad"])
    rear_stabar = float(vehicle["rear"]["actuation"]["stabar"]["rate_n_m_per_rad"])
    front_toe = float(vehicle["front"]["wheel"]["toe_deg"])
    rear_toe = float(vehicle["rear"]["wheel"]["toe_deg"])
    front_camber = float(vehicle["front"]["wheel"]["camber_deg"])
    rear_camber = float(vehicle["rear"]["wheel"]["camber_deg"])
    sprung_mass = float(vehicle["sprung_mass"]["mass_kg"])
    sprung_cg = [float(value) for value in vehicle["sprung_mass"]["cg_m"]]
    torsional = float(vehicle["body"]["torsional_stiff_n_m_per_rad"])
    rack_c = float(vehicle["front"]["steering"]["rack_travel_per_rev_m"])

    return [
        ParameterSpec(
            "front_spring_rate_n_per_m",
            "front spring rate",
            "N/m",
            front_spring,
            front_spring * 0.75,
            front_spring * 1.25,
            "ride/roll",
            (
                "Front shock spring table terminal rate. Spring free length is "
                "updated from baseline FourPost motion ratio to preserve the "
                "configured static spring length."
                if spring_setups
                else "Front shock spring table terminal rate."
            ),
            lambda text, value: set_spring_rate(
                text,
                "pFrAxleDW",
                value,
                spring_setups["front"].free_length_for_rate(value)
                if spring_setups
                else None,
            ),
        ),
        ParameterSpec(
            "rear_spring_rate_n_per_m",
            "rear spring rate",
            "N/m",
            rear_spring,
            rear_spring * 0.75,
            rear_spring * 1.25,
            "ride/roll",
            (
                "Rear shock spring table terminal rate. Spring free length is "
                "updated from baseline FourPost motion ratio to preserve the "
                "configured static spring length."
                if spring_setups
                else "Rear shock spring table terminal rate."
            ),
            lambda text, value: set_spring_rate(
                text,
                "pRrAxleDW",
                value,
                spring_setups["rear"].free_length_for_rate(value)
                if spring_setups
                else None,
            ),
        ),
        ParameterSpec(
            "front_stabar_rate_n_m_per_rad",
            "front anti-roll bar rate",
            "N*m/rad",
            front_stabar,
            front_stabar * 0.50,
            front_stabar * 1.50,
            "ride/roll",
            "Front anti-roll bar torsional stiffness.",
            lambda text, value: set_block_param(text, "pFrStabar", "barRate", value),
        ),
        ParameterSpec(
            "rear_stabar_rate_n_m_per_rad",
            "rear anti-roll bar rate",
            "N*m/rad",
            rear_stabar,
            rear_stabar * 0.50,
            rear_stabar * 1.50,
            "ride/roll",
            "Rear anti-roll bar torsional stiffness.",
            lambda text, value: set_block_param(text, "pRrStabar", "barRate", value),
        ),
        ParameterSpec(
            "front_toe_deg",
            "front static toe",
            "deg",
            front_toe,
            -1.0,
            1.0,
            "alignment",
            "Front static toe angle in the BobLib partial wheel record.",
            lambda text, value: set_block_param(text, "pFrPartialWheel", "staticAlpha", value),
        ),
        ParameterSpec(
            "rear_toe_deg",
            "rear static toe",
            "deg",
            rear_toe,
            -1.0,
            1.0,
            "alignment",
            "Rear static toe angle in the BobLib partial wheel record.",
            lambda text, value: set_block_param(text, "pRrPartialWheel", "staticAlpha", value),
        ),
        ParameterSpec(
            "front_camber_deg",
            "front static camber",
            "deg",
            front_camber,
            -2.5,
            0.5,
            "alignment",
            "Front static inclination angle in the BobLib partial wheel record.",
            lambda text, value: set_block_param(text, "pFrPartialWheel", "staticGamma", value),
        ),
        ParameterSpec(
            "rear_camber_deg",
            "rear static camber",
            "deg",
            rear_camber,
            -2.5,
            0.5,
            "alignment",
            "Rear static inclination angle in the BobLib partial wheel record.",
            lambda text, value: set_block_param(text, "pRrPartialWheel", "staticGamma", value),
        ),
        ParameterSpec(
            "sprung_mass_kg",
            "sprung mass",
            "kg",
            sprung_mass,
            sprung_mass * 0.90,
            sprung_mass * 1.10,
            "mass properties",
            "Sprung mass before driver mass is combined.",
            lambda text, value: set_block_param(text, "pBaseSprungMass", "m", value),
        ),
        ParameterSpec(
            "sprung_cg_x_m",
            "sprung CG x",
            "m",
            sprung_cg[0],
            sprung_cg[0] - 0.08,
            sprung_cg[0] + 0.08,
            "mass properties",
            "Longitudinal sprung-mass CG coordinate.",
            lambda text, value: set_mass_record_r_cm_component(text, "pBaseSprungMass", 0, value),
        ),
        ParameterSpec(
            "sprung_cg_z_m",
            "sprung CG z",
            "m",
            sprung_cg[2],
            max(0.16, sprung_cg[2] - 0.04),
            sprung_cg[2] + 0.05,
            "mass properties",
            "Vertical sprung-mass CG coordinate.",
            lambda text, value: set_mass_record_r_cm_component(text, "pBaseSprungMass", 2, value),
        ),
        ParameterSpec(
            "torsional_stiffness_n_m_per_rad",
            "body torsional stiffness",
            "N*m/rad",
            torsional,
            100_000.0,
            600_000.0,
            "structure",
            "Body torsional stiffness used by the compliant chassis.",
            lambda text, value: set_record_parameter(text, "pTorsionalStiff", value),
        ),
        ParameterSpec(
            "aero_downforce_scale",
            "aero downforce scale",
            "scale",
            1.0,
            0.65,
            1.35,
            "aero",
            "Scale applied to downforce and pitch-moment aero tables.",
            apply_aero_downforce_scale,
        ),
        ParameterSpec(
            "aero_drag_scale",
            "aero drag scale",
            "scale",
            1.0,
            0.75,
            1.25,
            "aero",
            "Scale applied to the drag aero table.",
            apply_aero_drag_scale,
        ),
        ParameterSpec(
            "lateral_tire_mu_scale",
            "lateral tire friction scale",
            "scale",
            1.0,
            0.90,
            1.10,
            "tires",
            "Scale applied to front and rear MF52 lateral friction multipliers.",
            apply_lateral_tire_mu_scale,
        ),
        ParameterSpec(
            "front_rack_travel_per_rev_m",
            "front rack travel per rev",
            "m/rev",
            rack_c,
            rack_c * 0.85,
            rack_c * 1.15,
            "steering",
            "Front steering rack travel per handwheel revolution.",
            lambda text, value: set_block_param(text, "pFrRack", "cFactor", value),
        ),
    ]


def baseline_values(specs: list[ParameterSpec]) -> dict[str, float]:
    return {spec.name: spec.baseline for spec in specs}


def apply_values(record_text: str, specs: list[ParameterSpec], values: dict[str, float]) -> str:
    spec_lookup = {spec.name: spec for spec in specs}
    text = record_text
    for name, value in values.items():
        text = spec_lookup[name].apply(text, float(value))
    return text


def generate_lhs(
    specs: list[ParameterSpec],
    sample_count: int,
    seed: int,
) -> list[dict[str, float]]:
    rng = np.random.default_rng(seed)
    matrix = np.zeros((sample_count, len(specs)), dtype=float)

    for col, spec in enumerate(specs):
        bins = (np.arange(sample_count, dtype=float) + rng.random(sample_count)) / sample_count
        rng.shuffle(bins)
        matrix[:, col] = spec.low + bins * (spec.high - spec.low)

    return [
        {spec.name: float(row[idx]) for idx, spec in enumerate(specs)}
        for row in matrix
    ]


def make_mos_content(variant_mo: Path, build_dir: Path, cfg: dict[str, Any]) -> str:
    option_lines = "\n".join(
        f'OpenModelica.Scripting.setCommandLineOptions("{option}");'
        for option in cfg.get("command_line_options", [])
    )
    intervals = cfg.get("intervals")
    intervals_line = (
        f"  numberOfIntervals={intervals},\n"
        if intervals is not None
        else ""
    )
    return f"""{option_lines}

clear();
loadModel(Modelica);
loadFile("{BOBLIB_PACKAGE.resolve().as_posix()}");
loadFile("{variant_mo.resolve().as_posix()}");

cd("{build_dir.resolve().as_posix()}");

buildModel(
  {cfg["model"]},
  startTime={cfg["start_time"]},
  stopTime={cfg["stop_time"]},
  outputFormat="csv",
{intervals_line}  tolerance={cfg["tolerance"]},
  method="{cfg["solver"]}",
  cflags="-O3 -march=native -mtune=native"
);

print(getErrorString());
"""


def find_executable(build_dir: Path, model_name: str = str(STANDARD_CFG["model"])) -> Path | None:
    model = str(model_name)
    short = model.split(".")[-1]
    for candidate in (
        build_dir / model,
        build_dir / f"{model}.exe",
        build_dir / short,
        build_dir / f"{short}.exe",
    ):
        if candidate.exists():
            return candidate
    return None


def compile_variant(
    variant_dir: Path,
    *,
    standard_name: str = "SteadyStateEval",
    cfg: dict[str, Any] = STANDARD_CFG,
) -> None:
    variant_mo = variant_dir / "variant.mo"
    build_dir = variant_dir / "build" / standard_name
    build_dir.mkdir(parents=True, exist_ok=True)
    mos_path = variant_dir / f"build_{standard_name}.mos"
    mos_path.write_text(make_mos_content(variant_mo, build_dir, cfg), encoding="utf-8")

    completed = subprocess.run(
        ["omc", str(mos_path)],
        cwd=str(variant_dir),
        capture_output=True,
        text=True,
    )
    log_path = variant_dir / f"compile_{standard_name}.log"
    log_path.write_text(
        f"returncode={completed.returncode}\n\nSTDOUT:\n{completed.stdout}\n\nSTDERR:\n{completed.stderr}",
        encoding="utf-8",
    )

    if find_executable(build_dir, str(cfg["model"])) is None:
        raise RuntimeError(
            f"OpenModelica did not produce {cfg['model']} for {variant_dir.name}. "
            f"See {as_repo_path(log_path)}"
        )


def read_metrics_csv(csv_path: Path) -> tuple[dict[str, float], list[dict[str, Any]]]:
    rows: list[dict[str, Any]] = []
    metrics: dict[str, float] = {}
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
            metric = str(row["metric"])
            try:
                metrics[metric] = float(row["value"])
            except (TypeError, ValueError):
                metrics[metric] = math.nan
    add_derived_metrics(metrics)
    return metrics, rows


def add_derived_metrics(metrics: dict[str, float]) -> None:
    torque_candidates = [
        abs(float(metrics[name]))
        for name in ("handwheel_torque_min", "handwheel_torque_max")
        if name in metrics and math.isfinite(float(metrics[name]))
    ]
    if torque_candidates:
        metrics["handwheel_torque_peak_abs"] = max(torque_candidates)


def run_standard_report(variant_dir: Path) -> Path:
    build_dir = variant_dir / "build" / "SteadyStateEval"
    config_path, metrics_csv = build_report_config(
        variant_dir=variant_dir,
        build_dir=build_dir,
        exec_name=str(STANDARD_CFG["model"]),
    )

    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise TypeError(f"Expected generated StandardSim config mapping: {config_path}")

    execution = config.setdefault("execution", {})
    if not isinstance(execution, dict):
        raise TypeError("Generated StandardSim execution block must be a mapping.")

    # Keep this study sandbox-friendly and deterministic. The compile/run loop
    # already gives us plenty of parallelism at the process level when needed.
    execution["parallel"] = False
    execution["max_workers"] = 1
    execution["stream_logs"] = False

    with config_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "_3_StandardSim.SteadyStateEval.steady_state_eval_sim",
            str(config_path),
        ],
        cwd=str(BOBSIM_ROOT),
        capture_output=True,
        text=True,
        timeout=900,
    )

    report_log = variant_dir / "run_SteadyStateEval.log"
    report_log.write_text(
        f"returncode={completed.returncode}\n\nSTDOUT:\n{completed.stdout}\n\nSTDERR:\n{completed.stderr}",
        encoding="utf-8",
    )

    if completed.returncode != 0:
        raise RuntimeError(
            "SteadyStateEval report generation failed. "
            f"See {as_repo_path(report_log)}"
        )

    if not metrics_csv.exists():
        raise FileNotFoundError(f"Metrics CSV not produced: {metrics_csv}")

    canonical_metrics_csv = metrics_csv.with_name("metrics.csv")
    shutil.copyfile(metrics_csv, canonical_metrics_csv)
    return canonical_metrics_csv


def stage_variant_text(
    variant_dir: Path,
    variant_text: str,
    *,
    rebuild: bool,
) -> bool:
    if rebuild and variant_dir.exists():
        shutil.rmtree(variant_dir)

    variant_dir.mkdir(parents=True, exist_ok=True)
    variant_mo = variant_dir / "variant.mo"
    existing_text = variant_mo.read_text(encoding="utf-8") if variant_mo.exists() else None
    variant_changed = existing_text != variant_text

    if rebuild or variant_changed:
        variant_mo.write_text(variant_text, encoding="utf-8")

    if variant_changed and not rebuild:
        shutil.rmtree(variant_dir / "build", ignore_errors=True)
        shutil.rmtree(variant_dir / "results", ignore_errors=True)

    return rebuild or variant_changed


def make_fourpost_config(
    vehicle: dict[str, Any],
    build_dir: Path,
    metrics_csv: Path,
) -> dict[str, Any]:
    return {
        "standard": "FourPostEval",
        "simulation": {
            "build_dir": str(build_dir),
            "exec_name": str(FOURPOST_CFG["model"]),
            "solver": FOURPOST_CFG["solver"],
            "stepSize": 0.5,
            "output_format": "csv",
            "log_level": "LOG_STATS,LOG_SOLVER,LOG_INIT",
            "no_grid": False,
            "no_event_emit": True,
            "stop_time": FOURPOST_CFG["stop_time"],
        },
        "execution": {
            "parallel": False,
            "cleanup": True,
            "stream_logs": False,
        },
        "vehicle": make_fourpost_vehicle_config(vehicle),
        "suspension": {
            "front": {
                "spring_rate": spring_rate_from_vehicle(vehicle, "front"),
                "arb_rate": float(vehicle["front"]["actuation"]["stabar"]["rate_n_m_per_rad"]),
            },
            "rear": {
                "spring_rate": spring_rate_from_vehicle(vehicle, "rear"),
                "arb_rate": float(vehicle["rear"]["actuation"]["stabar"]["rate_n_m_per_rad"]),
            },
        },
        "procedure": {
            "steerMagnitude": 0.0,
            "heaveMagnitude": 0.03,
            "rollMagnitude": 0.035,
            "forceMagnitude": 1000.0,
        },
        "report": {
            "enabled": False,
            "metrics_csv_path": str(metrics_csv),
        },
    }


def read_simple_metrics_csv(path: Path) -> dict[str, float]:
    metrics: dict[str, float] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            metric = str(row["metric"])
            try:
                metrics[metric] = float(row["value"])
            except (TypeError, ValueError):
                metrics[metric] = math.nan
    return metrics


def run_fourpost_motion_ratio(
    *,
    base_record_text: str,
    vehicle: dict[str, Any],
    rebuild: bool,
) -> tuple[dict[str, float], dict[str, AxleSpringSetup]]:
    variant_dir = WORK_DIR / "fourpost_baseline"
    build_dir = variant_dir / "build" / "FourPostEval"
    metrics_csv = OUTPUT_DIR / "fourpost_eval_metrics.csv"
    config_path = variant_dir / "fourpost_eval_config.yml"

    variant_changed = stage_variant_text(
        variant_dir,
        base_record_text,
        rebuild=rebuild,
    )

    if (
        rebuild
        or variant_changed
        or find_executable(build_dir, str(FOURPOST_CFG["model"])) is None
    ):
        compile_variant(
            variant_dir,
            standard_name="FourPostEval",
            cfg=FOURPOST_CFG,
        )

    config = make_fourpost_config(vehicle, build_dir, metrics_csv)
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    if rebuild or variant_changed or not metrics_csv.exists():
        result = FourPostEvalSim(config).run()
        summary = {
            key: float(value)
            for key, value in result["summary"].items()
            if isinstance(value, (int, float, np.floating))
        }
    else:
        summary = read_simple_metrics_csv(metrics_csv)

    motion_ratios = {
        "front": float(summary["avg_motion_ratio_front"]),
        "rear": float(summary["avg_motion_ratio_rear"]),
    }
    spring_setups = make_spring_setups(vehicle, motion_ratios)
    return motion_ratios, spring_setups


def write_spring_setup_csv(
    spring_setups: dict[str, AxleSpringSetup],
    specs: list[ParameterSpec],
) -> None:
    spec_lookup = {spec.name: spec for spec in specs}
    rows = []
    for axle in ("front", "rear"):
        setup = spring_setups[axle]
        spec = spec_lookup[f"{axle}_spring_rate_n_per_m"]
        rows.append(
            {
                "axle": axle,
                "motion_ratio_wheel_per_spring": setup.motion_ratio_wheel_per_spring,
                "installed_length_m": setup.installed_length_m,
                "sprung_corner_load_n": setup.sprung_corner_load_n,
                "spring_force_n": setup.spring_force_n,
                "baseline_rate_n_per_m": setup.baseline_rate_n_per_m,
                "baseline_free_length_vehicle_m": setup.baseline_free_length_vehicle_m,
                "baseline_compression_m": setup.baseline_compression_m,
                "configured_static_length_m": setup.configured_static_length_m,
                "geometric_static_length_m": setup.installed_length_m,
                "fourpost_free_length_at_baseline_rate_m": (
                    setup.fourpost_free_length_at_baseline_rate_m
                ),
                "baseline_free_length_delta_m": (
                    setup.fourpost_free_length_at_baseline_rate_m
                    - setup.baseline_free_length_vehicle_m
                ),
                "low_rate_n_per_m": spec.low,
                "low_free_length_m": setup.free_length_for_rate(spec.low),
                "high_rate_n_per_m": spec.high,
                "high_free_length_m": setup.free_length_for_rate(spec.high),
            }
        )

    write_csv(
        OUTPUT_DIR / "fourpost_spring_setup.csv",
        rows,
        [
            "axle",
            "motion_ratio_wheel_per_spring",
            "installed_length_m",
            "sprung_corner_load_n",
            "spring_force_n",
            "baseline_rate_n_per_m",
            "baseline_free_length_vehicle_m",
            "baseline_compression_m",
            "configured_static_length_m",
            "geometric_static_length_m",
            "fourpost_free_length_at_baseline_rate_m",
            "baseline_free_length_delta_m",
            "low_rate_n_per_m",
            "low_free_length_m",
            "high_rate_n_per_m",
            "high_free_length_m",
        ],
    )


def run_case(
    *,
    case_index: int,
    case_id: str,
    parameter: str,
    level: str,
    value: float | str,
    base_record_text: str,
    specs: list[ParameterSpec],
    values: dict[str, float],
    rebuild: bool,
) -> CaseResult:
    variant_dir = POPULATION_DIR / f"variant_{case_index:04d}_{case_id}"
    metrics_path = variant_dir / "results" / "SteadyStateEval" / "metrics.csv"
    started = time.perf_counter()

    try:
        variant_changed = stage_variant_text(
            variant_dir,
            apply_values(base_record_text, specs, values),
            rebuild=rebuild,
        )

        if (
            rebuild
            or variant_changed
            or find_executable(variant_dir / "build" / "SteadyStateEval") is None
        ):
            compile_variant(variant_dir)

        if rebuild or variant_changed or not metrics_path.exists():
            run_standard_report(variant_dir)

        metrics, _metric_rows = read_metrics_csv(metrics_path)
        status = "ok"
        error = ""
    except Exception as exc:  # noqa: BLE001 - keep the sweep moving and log failures.
        metrics = {}
        status = "failed"
        error = str(exc)
        (variant_dir / "case_error.txt").write_text(error, encoding="utf-8")

    return CaseResult(
        case_id=case_id,
        parameter=parameter,
        level=level,
        value=value,
        variant_dir=variant_dir,
        status=status,
        metrics=metrics,
        error=error,
        elapsed_s=time.perf_counter() - started,
    )


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def finite_pct_delta(value: float, baseline: float) -> float:
    if not math.isfinite(value) or not math.isfinite(baseline) or abs(baseline) < 1e-12:
        return math.nan
    return 100.0 * (value - baseline) / abs(baseline)


def finite_span_pct(high_value: float, low_value: float, baseline: float) -> float:
    if (
        not math.isfinite(high_value)
        or not math.isfinite(low_value)
        or not math.isfinite(baseline)
        or abs(baseline) < 1e-12
    ):
        return math.nan
    return 100.0 * (high_value - low_value) / abs(baseline)


def local_sensitivity_rows(
    specs: list[ParameterSpec],
    metric_names: list[str],
    baseline_metrics: dict[str, float],
    low_by_parameter: dict[str, dict[str, float]],
    high_by_parameter: dict[str, dict[str, float]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []

    for spec in specs:
        low_metrics = low_by_parameter.get(spec.name, {})
        high_metrics = high_by_parameter.get(spec.name, {})
        parameter_span = spec.high - spec.low

        for metric in metric_names:
            base_response = float(baseline_metrics.get(metric, math.nan))
            low_response = float(low_metrics.get(metric, math.nan))
            high_response = float(high_metrics.get(metric, math.nan))
            signed_span = high_response - low_response
            signed_effect_pct_span = finite_span_pct(
                high_response,
                low_response,
                base_response,
            )

            if high_response > low_response:
                direction = "increases_with_parameter"
            elif high_response < low_response:
                direction = "decreases_with_parameter"
            else:
                direction = "flat_or_failed"

            rows.append(
                {
                    "parameter": spec.name,
                    "parameter_label": spec.label,
                    "parameter_group": spec.group,
                    "parameter_unit": spec.unit,
                    "metric": metric,
                    "parameter_low": spec.low,
                    "parameter_baseline": spec.baseline,
                    "parameter_high": spec.high,
                    "response_low": low_response,
                    "response_baseline": base_response,
                    "response_high": high_response,
                    "signed_response_span": signed_span,
                    "slope_per_unit": signed_span / parameter_span
                    if abs(parameter_span) > 1e-12
                    else math.nan,
                    "signed_effect_pct_span": signed_effect_pct_span,
                    "abs_effect_pct_span": abs(signed_effect_pct_span)
                    if math.isfinite(signed_effect_pct_span)
                    else math.nan,
                    "low_delta_from_baseline_pct": finite_pct_delta(low_response, base_response),
                    "high_delta_from_baseline_pct": finite_pct_delta(high_response, base_response),
                    "direction": direction,
                }
            )

    for metric in metric_names:
        metric_rows = [row for row in rows if row["metric"] == metric]
        metric_rows.sort(
            key=lambda row: float(row["abs_effect_pct_span"])
            if math.isfinite(float(row["abs_effect_pct_span"]))
            else -1.0,
            reverse=True,
        )
        for rank, row in enumerate(metric_rows, start=1):
            row["rank_within_metric"] = rank

    return rows


def pearson(x_values: list[float], y_values: list[float]) -> float:
    x = np.asarray(x_values, dtype=float)
    y = np.asarray(y_values, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(mask) < 3:
        return math.nan
    x = x[mask]
    y = y[mask]
    if float(np.std(x)) <= 1e-12 or float(np.std(y)) <= 1e-12:
        return math.nan
    return float(np.corrcoef(x, y)[0, 1])


def metric_units_from_rows(metric_rows: list[dict[str, Any]]) -> dict[str, str]:
    units = {str(row["metric"]): str(row.get("units", "")) for row in metric_rows}
    units.update(DERIVED_METRIC_UNITS)
    return units


def metric_descriptions_from_rows(metric_rows: list[dict[str, Any]]) -> dict[str, str]:
    descriptions = {
        str(row["metric"]): str(row.get("description", "")) for row in metric_rows
    }
    descriptions.update(DERIVED_METRIC_DESCRIPTIONS)
    return descriptions


def is_active_report_metric(metric: str) -> bool:
    """Return True for metrics included in the current design read."""
    return metric in IMPORTANT_RESPONSE_METRICS


def ignored_metric_reason(metric: str) -> str:
    if metric in {"metric_target_velocity_mps", "metric_source_velocity_mps"}:
        return "run metadata, not a design response"
    if "_per_mps" in metric:
        return (
            "velocity-slope _per_mps metric excluded from current findings/plots"
        )
    if metric == "ay_min":
        return "low-end lateral sweep coverage metric; ay_max is the active capability response"
    if metric in {"handwheel_torque_min", "handwheel_torque_max"}:
        return "raw torque extremum; handwheel_torque_peak_abs is the active steering-effort response"
    if metric.endswith("_rad_per_mps2"):
        return "unit duplicate; deg/g version is the active response"
    return "excluded from current findings/plots"


def plot_baseline_metrics(metrics: dict[str, float], units: dict[str, str]) -> None:
    preferred = list(IMPORTANT_RESPONSE_METRICS)
    labels = [name for name in preferred if name in metrics]
    values = [metrics[name] for name in labels]

    fig, ax = plt.subplots(figsize=(10.5, 5.8))
    ax.bar(range(len(labels)), values, color="#426a5a")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("Metric value")
    ax.set_title("DS-002 Baseline StandardSim SteadyStateEval Metrics")
    ax.grid(True, axis="y", linestyle="--", alpha=0.3)

    for idx, name in enumerate(labels):
        ax.text(
            idx,
            values[idx],
            units.get(name, ""),
            ha="center",
            va="bottom" if values[idx] >= 0 else "top",
            fontsize=8,
        )

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "baseline_metrics.png", dpi=220)
    plt.close(fig)


def plot_heatmap(
    matrix: np.ndarray,
    row_labels: list[str],
    column_labels: list[str],
    title: str,
    colorbar_label: str,
    output_path: Path,
) -> None:
    finite = matrix[np.isfinite(matrix)]
    limit = max(1e-9, float(np.nanmax(np.abs(finite)))) if finite.size else 1.0
    fig_width = max(12.0, 0.55 * len(column_labels))
    fig_height = max(7.0, 0.35 * len(row_labels))
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    image = ax.imshow(matrix, aspect="auto", cmap="coolwarm", vmin=-limit, vmax=limit)
    ax.set_xticks(np.arange(len(column_labels)))
    ax.set_xticklabels(column_labels, rotation=55, ha="right", fontsize=8)
    ax.set_yticks(np.arange(len(row_labels)))
    ax.set_yticklabels(row_labels, fontsize=8)
    ax.set_title(title)
    ax.set_xlabel("Response variable")
    ax.set_ylabel("Parameter")
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label(colorbar_label)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def plot_local_sensitivity_heatmap(
    specs: list[ParameterSpec],
    metric_names: list[str],
    sensitivity_rows: list[dict[str, Any]],
) -> None:
    lookup = {
        (str(row["parameter"]), str(row["metric"])): float(row["signed_effect_pct_span"])
        for row in sensitivity_rows
    }
    matrix = np.array(
        [
            [lookup.get((spec.name, metric), math.nan) for metric in metric_names]
            for spec in specs
        ],
        dtype=float,
    )
    plot_heatmap(
        matrix,
        [spec.label for spec in specs],
        metric_names,
        "Local StandardSim Response Sensitivity",
        "Signed response span from low to high (% of baseline)",
        PLOT_DIR / "response_sensitivity_heatmap.png",
    )


def plot_top_local_sensitivities(
    sensitivity_rows: list[dict[str, Any]],
    active_metric_names: list[str],
) -> None:
    active_metrics = set(active_metric_names)
    sensitivity_rows = [
        row for row in sensitivity_rows if str(row["metric"]) in active_metrics
    ]
    sorted_rows = sorted(
        sensitivity_rows,
        key=lambda row: float(row["abs_effect_pct_span"])
        if math.isfinite(float(row["abs_effect_pct_span"]))
        else -1.0,
        reverse=True,
    )[:24]
    labels = [f"{row['parameter_label']} -> {row['metric']}" for row in sorted_rows][::-1]
    values = [float(row["signed_effect_pct_span"]) for row in sorted_rows][::-1]
    colors = ["#247ba0" if value >= 0.0 else "#d1495b" for value in values]

    fig, ax = plt.subplots(figsize=(12.0, 8.5))
    ax.barh(labels, values, color=colors)
    ax.axvline(0.0, color="0.2", linewidth=0.8)
    ax.set_xlabel("Signed response span from low to high (% of baseline)")
    ax.set_title("Largest Local StandardSim Sensitivities")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "top_local_sensitivities.png", dpi=220)
    plt.close(fig)


def plot_filename_for_metric(metric: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", metric).strip("_").lower()
    return f"{slug or 'response'}.png"


def plot_per_response_sensitivities(
    sensitivity_rows: list[dict[str, Any]],
    active_metric_names: list[str],
    units: dict[str, str],
) -> list[Path]:
    response_dir = PLOT_DIR / "responses"
    response_dir.mkdir(parents=True, exist_ok=True)
    output_paths: list[Path] = []

    for metric in active_metric_names:
        rows = [
            row
            for row in sensitivity_rows
            if str(row["metric"]) == metric
            and math.isfinite(float(row["abs_effect_pct_span"]))
        ]
        rows.sort(key=lambda row: float(row["abs_effect_pct_span"]))
        if not rows:
            continue

        labels = [str(row["parameter_label"]) for row in rows]
        values = [float(row["signed_effect_pct_span"]) for row in rows]
        colors = ["#247ba0" if value >= 0.0 else "#d1495b" for value in values]
        limit = max(1.0, max(abs(value) for value in values) * 1.18)
        fig_height = max(6.0, 0.42 * len(rows) + 1.8)

        fig, ax = plt.subplots(figsize=(11.5, fig_height))
        y = np.arange(len(rows))
        ax.barh(y, values, color=colors)
        ax.axvline(0.0, color="0.2", linewidth=0.8)
        ax.set_yticks(y)
        ax.set_yticklabels(labels)
        ax.set_xlim(-limit, limit)
        ax.set_xlabel("Signed response span from parameter low to high (% of baseline)")
        unit_label = units.get(metric, "")
        title_suffix = f" [{unit_label}]" if unit_label else ""
        ax.set_title(f"{metric}{title_suffix}")
        ax.grid(True, axis="x", linestyle="--", alpha=0.3)

        for idx, value in enumerate(values):
            ha = "left" if value >= 0.0 else "right"
            offset = 0.01 * limit if value >= 0.0 else -0.01 * limit
            ax.text(
                value + offset,
                idx,
                f"{value:+.1f}%",
                va="center",
                ha=ha,
                fontsize=8,
            )

        ax.text(
            0.5,
            -0.13,
            "Positive means the response increases as the parameter moves from low to high.",
            transform=ax.transAxes,
            ha="center",
            va="top",
            fontsize=9,
            color="0.35",
        )
        fig.tight_layout()

        output_path = response_dir / plot_filename_for_metric(metric)
        fig.savefig(output_path, dpi=220)
        plt.close(fig)
        output_paths.append(output_path)

    return output_paths


def table_line(values: list[str]) -> str:
    return "| " + " | ".join(values) + " |"


def write_report(
    *,
    vehicle_path: Path,
    specs: list[ParameterSpec],
    spring_setups: dict[str, AxleSpringSetup],
    metric_names: list[str],
    active_metric_names: list[str],
    baseline_metrics: dict[str, float],
    units: dict[str, str],
    sensitivity_rows: list[dict[str, Any]],
    case_results: list[CaseResult],
    global_samples: int,
    started_at: str,
    elapsed_s: float,
) -> None:
    ignored_metrics = [
        metric for metric in metric_names if metric not in set(active_metric_names)
    ]
    summary_metrics = list(IMPORTANT_RESPONSE_METRICS)
    top_summary_rows = []
    for metric in summary_metrics:
        rows = [row for row in sensitivity_rows if row["metric"] == metric]
        rows.sort(
            key=lambda row: int(row.get("rank_within_metric", 999999))
            if str(row.get("rank_within_metric", "")).isdigit()
            else 999999
        )
        if rows:
            top_summary_rows.append(rows[0])

    ok_cases = [case for case in case_results if case.status == "ok"]
    failed_cases = [case for case in case_results if case.status != "ok"]

    lines: list[str] = []
    lines.append("# DS-002 StandardSim SteadyStateEval Sensitivity")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Source of Results")
    lines.append("")
    lines.append(
        "All response metrics in this report are generated by BobSim StandardSim "
        "using `BobSim/_3_StandardSim/SteadyStateEval/steady_state_eval_sim.py`."
    )
    lines.append("")
    lines.append("## Sensitivity Definition")
    lines.append("")
    lines.append(
        "Each local sensitivity is one response variable with respect to one "
        "varied StandardSim parameter."
    )
    lines.append("")
    lines.append(table_line(["Term", "Meaning"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Independent variable", "The parameter listed in `outputs/parameter_registry.csv`."]))
    lines.append(table_line(["Response variable", "The metric listed in `outputs/metric_catalog.csv`."]))
    lines.append(table_line(["Local cases", "One low-parameter StandardSim build/run, one baseline build/run, and one high-parameter StandardSim build/run."]))
    lines.append(table_line(["Raw response span", "`response_high - response_low`."]))
    lines.append(table_line(["Slope per unit", "`(response_high - response_low) / (parameter_high - parameter_low)`."]))
    lines.append(table_line(["Normalized span", "`100 * (response_high - response_low) / abs(response_baseline)`."]))
    lines.append("")
    lines.append(
        "The normalized span is intentionally design-envelope dependent. "
        "Use `slope_per_unit` for derivative-style comparisons."
    )
    lines.append("")
    lines.append("## Baseline")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Vehicle source", as_repo_path(vehicle_path)]))
    lines.append(table_line(["Model", str(STANDARD_CFG["model"])]))
    lines.append(table_line(["Standard", "SteadyStateEval"]))
    lines.append(table_line(["Spring free-length policy", "FourPost motion-ratio adjusted for spring-rate variants"]))
    lines.append(table_line(["Parameters swept", str(len(specs))]))
    lines.append(table_line(["Successful cases", f"{len(ok_cases)} / {len(case_results)}"]))
    lines.append(table_line(["Active response metrics", str(len(active_metric_names))]))
    lines.append(table_line(["Ignored response metrics", str(len(ignored_metrics))]))
    lines.append(table_line(["Global samples", str(global_samples)]))
    lines.append("")
    lines.append("## Baseline Response Metrics")
    lines.append("")
    lines.append(table_line(["Metric", "Value", "Units"]))
    lines.append(table_line(["---", "---:", "---"]))
    for metric in summary_metrics:
        if metric in baseline_metrics:
            lines.append(
                table_line(
                    [
                        metric,
                        format_float(baseline_metrics[metric], 5),
                        units.get(metric, ""),
                    ]
                )
            )
    lines.append("")
    lines.append("## FourPost Spring Setup")
    lines.append("")
    lines.append(
        "Before the spring-rate sensitivity cases, the study runs the baseline "
        "FourPostSim/FourPostEval model to extract motion ratio. Low/high spring "
        "rate variants then update `springFreeLength` so the same configured "
        "static spring length and sprung corner load are preserved."
    )
    lines.append("")
    lines.append(table_line(["Axle", "MR wheel/spring", "Config static m", "Spring force N", "Low FL m", "High FL m"]))
    lines.append(table_line(["---", "---:", "---:", "---:", "---:", "---:"]))
    spec_lookup = {spec.name: spec for spec in specs}
    for axle in ("front", "rear"):
        if axle not in spring_setups:
            continue
        setup = spring_setups[axle]
        spec = spec_lookup.get(f"{axle}_spring_rate_n_per_m")
        low_fl = setup.free_length_for_rate(spec.low) if spec else math.nan
        high_fl = setup.free_length_for_rate(spec.high) if spec else math.nan
        lines.append(
            table_line(
                [
                    axle,
                    format_float(setup.motion_ratio_wheel_per_spring, 5),
                    format_float(setup.configured_static_length_m, 5),
                    format_float(setup.spring_force_n, 2),
                    format_float(low_fl, 5),
                    format_float(high_fl, 5),
                ]
            )
        )
    lines.append("")
    lines.append("## Local Sensitivity Read")
    lines.append("")
    lines.append(table_line(["Response", "Top local parameter", "Direction", "Signed span %", "Low", "High"]))
    lines.append(table_line(["---", "---", "---", "---:", "---:", "---:"]))
    for row in top_summary_rows:
        lines.append(
            table_line(
                [
                    str(row["metric"]),
                    str(row["parameter_label"]),
                    str(row["direction"]),
                    format_float(float(row["signed_effect_pct_span"]), 2),
                    format_float(float(row["response_low"]), 5),
                    format_float(float(row["response_high"]), 5),
                ]
            )
        )
    lines.append("")
    lines.append(
        "The complete per-response local sensitivity matrix is in "
        "`outputs/metric_sensitivity_matrix.csv`."
    )
    if ignored_metrics:
        lines.append("")
        lines.append(
            "Diagnostic and duplicate metrics were computed and retained in the "
            "raw CSVs, but are ignored in the current findings and plots so the "
            "figures focus on the important design responses."
        )
    lines.append("")
    if failed_cases:
        lines.append("## Failed Cases")
        lines.append("")
        lines.append(table_line(["Case", "Parameter", "Level", "Error"]))
        lines.append(table_line(["---", "---", "---", "---"]))
        for case in failed_cases[:12]:
            lines.append(
                table_line(
                    [
                        case.case_id,
                        case.parameter,
                        case.level,
                        case.error.replace("\n", " ")[:160],
                    ]
                )
            )
        lines.append("")
    lines.append("## Generated Files")
    lines.append("")
    for relative_path in [
        "outputs/baseline_metrics.csv",
        "outputs/fourpost_eval_metrics.csv",
        "outputs/fourpost_spring_setup.csv",
        "outputs/parameter_registry.csv",
        "outputs/ofat_cases.csv",
        "outputs/metric_sensitivity_matrix.csv",
        "outputs/ignored_metrics.csv",
        "outputs/case_manifest.csv",
        "outputs/case_errors.csv",
        "outputs/run_provenance.csv",
        "plots/baseline_metrics.png",
        "plots/response_sensitivity_heatmap.png",
        "plots/top_local_sensitivities.png",
    ]:
        lines.append(f"- `{relative_path}`")
    for metric in active_metric_names:
        lines.append(f"- `plots/responses/{plot_filename_for_metric(metric)}`")
    lines.append("")
    lines.append("## Run Provenance")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Elapsed time", f"{elapsed_s:.1f} s"]))
    lines.append(table_line(["Work dir", "`work/` (ignored; reproducible compiled artifacts)"]))
    lines.append(table_line(["Python", sys.executable]))
    lines.append("")

    text = "\n".join(lines)
    (STUDY_DIR / "RESULTS.md").write_text(text, encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "vehicles" / "current" / "vehicle.yml",
        help="Vehicle YAML file used to render the baseline Modelica record.",
    )
    parser.add_argument(
        "--global-samples",
        type=int,
        default=0,
        help="Optional global LHS sample count. Default 0 because each sample compiles.",
    )
    parser.add_argument("--seed", type=int, default=4107)
    parser.add_argument(
        "--limit-parameters",
        type=int,
        default=0,
        help="Limit swept parameters for smoke testing. 0 means all parameters.",
    )
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="Reuse existing compiled cases/results in work/ when available.",
    )
    args = parser.parse_args()

    if shutil.which("omc") is None:
        raise SystemExit("OpenModelica `omc` was not found on PATH.")

    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    start_time = time.perf_counter()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    POPULATION_DIR.mkdir(parents=True, exist_ok=True)

    print("DS-002 StandardSim SteadyStateEval sensitivity", flush=True)
    print(f"  Vehicle: {args.vehicle.relative_to(REPO_ROOT)}", flush=True)
    print(f"  Work dir: {WORK_DIR.relative_to(REPO_ROOT)}", flush=True)

    base_record_text, vehicle = make_baseline_record(args.vehicle)

    print("Running baseline FourPostSim motion-ratio characterization", flush=True)
    motion_ratios, spring_setups = run_fourpost_motion_ratio(
        base_record_text=base_record_text,
        vehicle=vehicle,
        rebuild=not args.reuse,
    )
    print(
        "  Motion ratios: "
        f"front={format_float(motion_ratios['front'], 5)}, "
        f"rear={format_float(motion_ratios['rear'], 5)}",
        flush=True,
    )

    all_specs = make_parameter_specs(vehicle, spring_setups)
    specs = all_specs[: args.limit_parameters] if args.limit_parameters else all_specs
    base_values = baseline_values(specs)

    print(f"  Parameters: {len(specs)}", flush=True)
    print(f"  Local cases: {1 + 2 * len(specs)}", flush=True)
    print(f"  Global samples: {args.global_samples}", flush=True)

    write_spring_setup_csv(spring_setups, all_specs)

    write_csv(
        OUTPUT_DIR / "parameter_registry.csv",
        [
            {
                key: value
                for key, value in dataclasses.asdict(spec).items()
                if key != "apply"
            }
            for spec in specs
        ],
        ["name", "label", "unit", "baseline", "low", "high", "group", "description"],
    )

    case_results: list[CaseResult] = []
    case_index = 0

    print("Running baseline StandardSim case", flush=True)
    baseline_case = run_case(
        case_index=case_index,
        case_id="baseline",
        parameter="",
        level="baseline",
        value="",
        base_record_text=base_record_text,
        specs=specs,
        values=base_values,
        rebuild=not args.reuse,
    )
    case_results.append(baseline_case)
    if baseline_case.status != "ok":
        raise SystemExit(f"Baseline StandardSim case failed: {baseline_case.error}")

    baseline_metrics = baseline_case.metrics
    baseline_metrics_path = baseline_case.variant_dir / "results" / "SteadyStateEval" / "metrics.csv"
    _baseline_metric_values, baseline_metric_rows = read_metrics_csv(baseline_metrics_path)
    units = metric_units_from_rows(baseline_metric_rows)
    descriptions = metric_descriptions_from_rows(baseline_metric_rows)
    metric_names = list(baseline_metrics.keys())
    active_metric_names = [
        metric for metric in metric_names if is_active_report_metric(metric)
    ]
    ignored_metric_names = [
        metric for metric in metric_names if metric not in active_metric_names
    ]

    write_csv(
        OUTPUT_DIR / "metric_catalog.csv",
        [
            {
                "metric": metric,
                "unit": units.get(metric, ""),
                "description": descriptions.get(metric, ""),
                "active_in_current_report": int(metric in active_metric_names),
            }
            for metric in metric_names
        ],
        ["metric", "unit", "description", "active_in_current_report"],
    )
    write_csv(
        OUTPUT_DIR / "ignored_metrics.csv",
        [
            {
                "metric": metric,
                "reason": ignored_metric_reason(metric),
            }
            for metric in ignored_metric_names
        ],
        ["metric", "reason"],
    )
    write_csv(
        OUTPUT_DIR / "baseline_metrics.csv",
        [
            {
                "metric": metric,
                "value": baseline_metrics[metric],
                "unit": units.get(metric, ""),
                "description": descriptions.get(metric, ""),
            }
            for metric in metric_names
        ],
        ["metric", "value", "unit", "description"],
    )
    plot_baseline_metrics(baseline_metrics, units)

    print("Running local one-factor StandardSim sweep", flush=True)
    ofat_rows: list[dict[str, Any]] = []
    low_by_parameter: dict[str, dict[str, float]] = {}
    high_by_parameter: dict[str, dict[str, float]] = {}

    baseline_row = {
        "case_id": "baseline",
        "parameter": "",
        "level": "baseline",
        "value": "",
        "status": baseline_case.status,
    }
    baseline_row.update(baseline_metrics)
    ofat_rows.append(baseline_row)

    for spec in specs:
        for level, value in (("low", spec.low), ("high", spec.high)):
            case_index += 1
            values = dict(base_values)
            values[spec.name] = value
            print(
                f"  OFAT {case_index:02d}/{2 * len(specs)}: "
                f"{spec.name}={format_float(value, 6)}",
                flush=True,
            )
            result = run_case(
                case_index=case_index,
                case_id=f"{spec.name}_{level}",
                parameter=spec.name,
                level=level,
                value=value,
                base_record_text=base_record_text,
                specs=specs,
                values=values,
                rebuild=not args.reuse,
            )
            case_results.append(result)
            if level == "low":
                low_by_parameter[spec.name] = result.metrics
            else:
                high_by_parameter[spec.name] = result.metrics

            row = {
                "case_id": result.case_id,
                "parameter": spec.name,
                "level": level,
                "value": value,
                "status": result.status,
            }
            row.update({metric: result.metrics.get(metric, math.nan) for metric in metric_names})
            ofat_rows.append(row)

    sensitivity = local_sensitivity_rows(
        specs,
        metric_names,
        baseline_metrics,
        low_by_parameter,
        high_by_parameter,
    )

    case_fields = ["case_id", "parameter", "level", "value", "status"] + metric_names
    write_csv(OUTPUT_DIR / "ofat_cases.csv", ofat_rows, case_fields)
    sensitivity_fields = [
        "parameter",
        "parameter_label",
        "parameter_group",
        "parameter_unit",
        "metric",
        "parameter_low",
        "parameter_baseline",
        "parameter_high",
        "response_low",
        "response_baseline",
        "response_high",
        "signed_response_span",
        "slope_per_unit",
        "signed_effect_pct_span",
        "abs_effect_pct_span",
        "low_delta_from_baseline_pct",
        "high_delta_from_baseline_pct",
        "direction",
        "rank_within_metric",
    ]
    write_csv(OUTPUT_DIR / "metric_sensitivity_matrix.csv", sensitivity, sensitivity_fields)
    plot_local_sensitivity_heatmap(specs, active_metric_names, sensitivity)
    plot_top_local_sensitivities(sensitivity, active_metric_names)
    plot_per_response_sensitivities(sensitivity, active_metric_names, units)

    global_correlation_rows: list[dict[str, Any]] = []
    if args.global_samples > 0:
        print("Running optional global StandardSim sample", flush=True)
        samples = generate_lhs(specs, args.global_samples, args.seed)
        global_sample_rows: list[dict[str, Any]] = []
        global_metric_rows: list[dict[str, Any]] = []
        parameter_series = {spec.name: [] for spec in specs}
        metric_series = {metric: [] for metric in metric_names}

        for sample_idx, values in enumerate(samples, start=1):
            case_index += 1
            print(f"  Global sample {sample_idx:02d}/{args.global_samples}", flush=True)
            result = run_case(
                case_index=case_index,
                case_id=f"global_{sample_idx:04d}",
                parameter="global_sample",
                level="lhs",
                value=sample_idx,
                base_record_text=base_record_text,
                specs=specs,
                values=values,
                rebuild=not args.reuse,
            )
            case_results.append(result)

            sample_row = {"sample_id": sample_idx}
            sample_row.update(values)
            global_sample_rows.append(sample_row)

            metric_row = {"sample_id": sample_idx, "status": result.status}
            metric_row.update({metric: result.metrics.get(metric, math.nan) for metric in metric_names})
            global_metric_rows.append(metric_row)

            for spec in specs:
                parameter_series[spec.name].append(values[spec.name])
            for metric in metric_names:
                metric_series[metric].append(result.metrics.get(metric, math.nan))

        for spec in specs:
            for metric in metric_names:
                r_value = pearson(parameter_series[spec.name], metric_series[metric])
                global_correlation_rows.append(
                    {
                        "parameter": spec.name,
                        "parameter_label": spec.label,
                        "parameter_group": spec.group,
                        "metric": metric,
                        "pearson_r": r_value,
                        "abs_pearson_r": abs(r_value) if math.isfinite(r_value) else math.nan,
                    }
                )

        write_csv(
            OUTPUT_DIR / "global_samples.csv",
            global_sample_rows,
            ["sample_id"] + [spec.name for spec in specs],
        )
        write_csv(
            OUTPUT_DIR / "global_metrics.csv",
            global_metric_rows,
            ["sample_id", "status"] + metric_names,
        )
        write_csv(
            OUTPUT_DIR / "global_metric_correlations.csv",
            global_correlation_rows,
            [
                "parameter",
                "parameter_label",
                "parameter_group",
                "metric",
                "pearson_r",
                "abs_pearson_r",
            ],
        )

    manifest_rows = [
        {
            "case_id": case.case_id,
            "parameter": case.parameter,
            "level": case.level,
            "value": case.value,
            "status": case.status,
            "elapsed_s": case.elapsed_s,
            "variant_dir": as_repo_path(case.variant_dir),
        }
        for case in case_results
    ]
    write_csv(
        OUTPUT_DIR / "case_manifest.csv",
        manifest_rows,
        ["case_id", "parameter", "level", "value", "status", "elapsed_s", "variant_dir"],
    )
    write_csv(
        OUTPUT_DIR / "case_errors.csv",
        [
            {
                "case_id": case.case_id,
                "parameter": case.parameter,
                "level": case.level,
                "error": case.error,
            }
            for case in case_results
            if case.status != "ok"
        ],
        ["case_id", "parameter", "level", "error"],
    )

    elapsed_s = time.perf_counter() - start_time
    write_csv(
        OUTPUT_DIR / "run_provenance.csv",
        [
            {"item": "started_at_utc", "value": started_at},
            {"item": "engine", "value": "BobSim StandardSim SteadyStateEval"},
            {"item": "engine_path", "value": "BobSim/_3_StandardSim/SteadyStateEval/steady_state_eval_sim.py"},
            {"item": "setup_engine", "value": "BobSim StandardSim FourPostEval"},
            {"item": "setup_engine_path", "value": "BobSim/_3_StandardSim/FourPostEval/four_post_eval_sim.py"},
            {"item": "model", "value": STANDARD_CFG["model"]},
            {"item": "fourpost_model", "value": FOURPOST_CFG["model"]},
            {"item": "fourpost_motion_ratio_front", "value": motion_ratios["front"]},
            {"item": "fourpost_motion_ratio_rear", "value": motion_ratios["rear"]},
            {"item": "parameter_count", "value": len(specs)},
            {"item": "response_metric_count", "value": len(metric_names)},
            {"item": "ofat_case_count", "value": 1 + 2 * len(specs)},
            {"item": "global_sample_count", "value": args.global_samples},
            {"item": "elapsed_seconds", "value": f"{elapsed_s:.3f}"},
        ],
        ["item", "value"],
    )

    write_report(
        vehicle_path=args.vehicle,
        specs=specs,
        spring_setups=spring_setups,
        metric_names=metric_names,
        active_metric_names=active_metric_names,
        baseline_metrics=baseline_metrics,
        units=units,
        sensitivity_rows=sensitivity,
        case_results=case_results,
        global_samples=args.global_samples,
        started_at=started_at,
        elapsed_s=elapsed_s,
    )

    print(f"Complete in {elapsed_s:.1f} s", flush=True)
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}", flush=True)
    print(f"Top-level report: {REPORT_PATH}", flush=True)


if __name__ == "__main__":
    main()
