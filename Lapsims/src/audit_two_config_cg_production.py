"""Fail-closed provenance lock and acceptance audit for the two-config CG study.

``lock`` is a read-only production preflight apart from writing one immutable
JSON lock file.  It validates the exact coupled geometry, nominal masses,
per-case tuning, 80/32 kW split, pinned BobSim snapshot, tire/FourPost inputs,
track inputs, and all study-local source hashes.  It refuses roots that already
contain production artifacts.

``audit`` launches no simulation.  It verifies the lock, completed root
topology, map symmetry/feasibility, event convergence and speed-domain use,
sampled 6DOF QSS loads/states, 80/32 lateral-boundary equality, and performs an
independent mixed-power competition-points recomputation.  If a report root is
provided, its numerical rows must reproduce that recomputation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from audit_two_config_qss_samples import (
    CONFIGURATION_AXIS,
    EXPECTED_AERO_BALANCE_FRONT,
    EXPECTED_CASES,
    EXPECTED_MODEL_DOF,
    EXPECTED_MU_SCALE,
    EXPECTED_SPRUNG_MASS_KG,
    EXPECTED_TOTAL_MASS_KG,
    EXPECTED_WHEELBASE_M,
    MODEL_FAMILY,
    MODEL_KEY,
    validate_configuration_metadata,
    validate_configuration_sweep,
)

LAPSIMS_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = LAPSIMS_ROOT.parents[2]
DEFAULT_BOBSIM_ROOT = (
    WORKSPACE_ROOT / "tmp" / "bobsim-two-config-cg-9e1af8a"
)
DEFAULT_SCORING_JSON = LAPSIMS_ROOT / "inputs/fsae_ev_michigan_2026_scoring.json"
DEFAULT_TRACK_MANIFEST = LAPSIMS_ROOT / "inputs/events/openlap_event_suite_manifest.json"
DEFAULT_RUN_CONTRACT = LAPSIMS_ROOT / (
    "inputs/ggv_configuration_nominal54_vs_plus0p25_56p5_"
    "production_run_contract.json"
)
PINNED_BOBSIM_COMMIT = "9e1af8a797e663377b7efdce10dd49ac8648bc64"
CRITICAL_BOBSIM_HASHES = {
    "vehicle.yml": "ef2daaf81289e59d063d33e3f140a498f7d39ce3515efc2e5809d297d5d8adab",
    "_0_Utils/tire_templates/16x7p5_10_12psi.tir": (
        "328b9666c71a164685d7429ef00517fbe0a6f15d3f5d031714bae4ce642bc7b1"
    ),
    "_3_StandardSim/generated_results/four_post_eval_report_metrics.csv": (
        "ab42f195305c605c19bf1390c795054c68de697aa63f687ae6365a1fe74fec87"
    ),
    "_2_EnvelopeSim/GGV/ggv_generation.py": (
        "fabddc7d17831eee74ef7c7055750384d93efb0e411abe91f4459da8c69ec349"
    ),
}
EXPECTED_EVENTS = (
    "acceleration",
    "skidpad",
    "autocross",
    "michigan_endurance",
)
SPRINT_EVENTS = EXPECTED_EVENTS[:3]
ENDURANCE_EVENT = EXPECTED_EVENTS[3]
EXPECTED_POWER_W = {"80kw": 80_000.0, "32kw": 32_000.0}
EXPECTED_POWER_KW = {"80kw": 80.0, "32kw": 32.0}
NORMAL_LOAD_NUMERICAL_TOLERANCE_N = 1e-5
LOCK_SCHEMA = "lapsims.two-config-cg-production-lock.v1"
AUDIT_SCHEMA = "lapsims.two-config-cg-production-acceptance.v1"
MAP_COLUMNS = (
    "speed_mps",
    "ay_mps2",
    "ax_accel_mps2",
    "ax_brake_mps2",
    "accel_feasible",
    "brake_feasible",
)
QSS_COLUMNS = (
    "cg_case",
    "sweep_axis",
    "model_dof",
    "event_slug",
    "source_kind",
    "trim_success",
    "trim_residual_norm",
    "beta_rad",
    "steering_rad",
    "fz_fl_n",
    "fz_fr_n",
    "fz_rl_n",
    "fz_rr_n",
    "normal_load_min_n",
    "normal_load_max_n",
    "cg_height_in",
    "sprung_mass_kg",
    "total_mass_kg",
    "rear_static_weight_fraction",
    "front_static_weight_fraction",
    "front_antiroll_stiffness_fraction",
    "brake_distribution_front",
    "aero_balance_front",
    "tire_mu_scale",
    "tir_valid_load_min_n",
    "tir_valid_load_max_n",
)


@dataclass(frozen=True)
class ScoreRule:
    maximum_points: float
    performance_points: float
    completion_points: float
    tmax_factor: float
    exponent: float = 1.0

    def score(self, time_s: float, tmin_s: float) -> float:
        if time_s == tmin_s:
            return self.maximum_points
        tmax_s = tmin_s * self.tmax_factor
        if time_s >= tmax_s:
            return self.completion_points
        numerator = (tmax_s / time_s) ** self.exponent - 1.0
        denominator = (tmax_s / tmin_s) ** self.exponent - 1.0
        value = self.performance_points * numerator / denominator
        return float(
            np.clip(
                value + self.completion_points,
                self.completion_points,
                np.nextafter(self.maximum_points, -math.inf),
            )
        )


SCORE_RULES = {
    "acceleration": ScoreRule(100.0, 95.5, 4.5, 1.50),
    "skidpad": ScoreRule(75.0, 71.5, 3.5, 1.25, 2.0),
    "autocross": ScoreRule(125.0, 118.5, 6.5, 1.45),
    "michigan_endurance": ScoreRule(275.0, 250.0, 25.0, 1.45),
}


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(payload, dict):
        raise TypeError(f"{path} must contain a JSON object.")
    return payload


def _json_default(value: object) -> object:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}.")


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_columns(frame: pd.DataFrame, columns: Sequence[str], context: str) -> None:
    missing = sorted(set(columns).difference(frame.columns))
    if missing:
        raise ValueError(f"{context} is missing required columns: {missing}.")


def _as_bool(value: object) -> bool:
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if (
        isinstance(value, (int, float, np.integer, np.floating))
        and math.isfinite(float(value))
        and float(value) in (0.0, 1.0)
    ):
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise ValueError(f"Cannot interpret {value!r} as a strict boolean.")


def _close(
    actual: float,
    expected: float,
    *,
    context: str,
    tolerance: float = 1e-9,
) -> None:
    if not math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=tolerance):
        raise ValueError(
            f"{context} mismatch: expected {expected:.12g}, got {actual:.12g}."
        )


def _equivalent(left: Any, right: Any, *, tolerance: float = 1e-10) -> bool:
    if isinstance(left, Mapping) or isinstance(right, Mapping):
        if not isinstance(left, Mapping) or not isinstance(right, Mapping):
            return False
        return set(left) == set(right) and all(
            _equivalent(left[key], right[key], tolerance=tolerance) for key in left
        )
    if isinstance(left, Sequence) and not isinstance(left, (str, bytes)):
        if not isinstance(right, Sequence) or isinstance(right, (str, bytes)):
            return False
        return len(left) == len(right) and all(
            _equivalent(a, b, tolerance=tolerance) for a, b in zip(left, right)
        )
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(
            float(left), float(right), rel_tol=0.0, abs_tol=tolerance
        )
    return left == right


def _git_head(root: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip().lower()


def validate_run_contract(payload: Mapping[str, Any]) -> dict[str, Any]:
    """Validate the hash-locked numerical generation and audit contract."""

    expected_schema = "lapsims.two-config-cg-production-run-contract.v1"
    if payload.get("schema") != expected_schema:
        raise ValueError(f"Run contract schema must be {expected_schema}.")
    ggv = payload.get("ggv")
    qss = payload.get("sampled_qss_audit")
    acceptance = payload.get("acceptance")
    if not isinstance(ggv, Mapping) or not isinstance(qss, Mapping) or not isinstance(
        acceptance, Mapping
    ):
        raise TypeError("Run contract ggv, sampled_qss_audit, and acceptance must be objects.")

    integer_fields = {
        "ggv.ay_points": (ggv, "ay_points", 3),
        "ggv.ax_search_points": (ggv, "ax_search_points", 3),
        "ggv.ax_binary_iterations": (ggv, "ax_binary_iterations", 1),
        "qss.maximum_binned_trace_samples_per_case": (
            qss,
            "maximum_binned_trace_samples_per_case",
            1,
        ),
        "qss.speed_bins": (qss, "speed_bins", 1),
        "qss.lateral_bins": (qss, "lateral_bins", 1),
        "qss.longitudinal_bins": (qss, "longitudinal_bins", 1),
        "qss.trim_max_nfev": (qss, "trim_max_nfev", 1),
    }
    for context, (block, field, minimum) in integer_fields.items():
        value = block.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
            raise ValueError(f"Run contract {context} must be an integer >= {minimum}.")
    if int(ggv["ay_points"]) % 2 == 0:
        raise ValueError("Run contract ggv.ay_points must be odd and include ay=0.")

    positive_fields = (
        (ggv, "ay_max_g", "ggv.ay_max_g"),
        (ggv, "top_speed_mps", "ggv.top_speed_mps"),
        (ggv, "speed_step_mps", "ggv.speed_step_mps"),
        (ggv, "qss_zero_proxy_mps", "ggv.qss_zero_proxy_mps"),
        (qss, "pure_lateral_speed_mps", "qss.pure_lateral_speed_mps"),
        (qss, "trim_tolerance", "qss.trim_tolerance"),
        (
            acceptance,
            "maximum_final_speed_change_mps",
            "acceptance.maximum_final_speed_change_mps",
        ),
        (
            acceptance,
            "maximum_trim_residual_norm",
            "acceptance.maximum_trim_residual_norm",
        ),
        (acceptance, "maximum_abs_beta_rad", "acceptance.maximum_abs_beta_rad"),
        (
            acceptance,
            "maximum_abs_steering_rad",
            "acceptance.maximum_abs_steering_rad",
        ),
        (acceptance, "tir_valid_load_min_n", "acceptance.tir_valid_load_min_n"),
        (acceptance, "tir_valid_load_max_n", "acceptance.tir_valid_load_max_n"),
        (
            acceptance,
            "lateral_equality_tolerance_mps2",
            "acceptance.lateral_equality_tolerance_mps2",
        ),
    )
    for block, field, context in positive_fields:
        value = block.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"Run contract {context} must be numeric.")
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"Run contract {context} must be finite and positive.")
    if float(ggv["qss_zero_proxy_mps"]) >= float(ggv["speed_step_mps"]):
        raise ValueError("QSS zero proxy must be below the first regular speed step.")
    if float(acceptance["tir_valid_load_max_n"]) <= float(
        acceptance["tir_valid_load_min_n"]
    ):
        raise ValueError("TIR maximum load must exceed its minimum load.")
    return {
        "schema": expected_schema,
        "ggv": dict(ggv),
        "sampled_qss_audit": dict(qss),
        "acceptance": dict(acceptance),
    }


def _case_tuning_from_artifact(
    tuning_artifact: Mapping[str, Any],
) -> dict[str, dict[str, float]]:
    if str(tuning_artifact.get("sweep_axis")) != CONFIGURATION_AXIS:
        raise ValueError("Tuning artifact must declare sweep_axis=configuration.")
    _close(
        float(tuning_artifact.get("aero_balance_front", math.nan)),
        EXPECTED_AERO_BALANCE_FRONT,
        context="tuning aero balance",
    )
    tire = tuning_artifact.get("tire")
    if not isinstance(tire, Mapping):
        raise TypeError("Tuning artifact tire block must be an object.")
    _close(
        float(tire.get("mu_scale", math.nan)),
        EXPECTED_MU_SCALE,
        context="tuning tire mu scale",
    )
    method = tuning_artifact.get("roll_tuning_method")
    if not isinstance(method, Mapping):
        raise TypeError("Tuning artifact roll_tuning_method must be an object.")
    endpoint = str(method.get("endpoint_solver", ""))
    if "solve_lateral_limit" not in endpoint or "interval" not in endpoint.lower():
        raise ValueError("Tuning artifact does not identify the robust lateral endpoint.")
    if method.get("cold_monotonic_binary_assumption") is not False:
        raise ValueError("Tuning artifact must explicitly disable cold monotonic binary.")

    models = tuning_artifact.get("models")
    if not isinstance(models, Mapping) or MODEL_KEY not in models:
        raise ValueError(f"Tuning artifact must contain models.{MODEL_KEY}.")
    model = models[MODEL_KEY]
    if not isinstance(model, Mapping) or int(model.get("model_dof", -1)) != 6:
        raise ValueError("Tuning artifact model block is not the 6DOF model.")
    cases = model.get("cases")
    if not isinstance(cases, list):
        raise TypeError("Tuning artifact model cases must be a list.")
    expected_names = [item[0] for item in EXPECTED_CASES]
    if [str(case.get("name")) for case in cases if isinstance(case, Mapping)] != expected_names:
        raise ValueError("Tuning artifact cases do not match the exact configuration order.")

    output: dict[str, dict[str, float]] = {}
    for case, (name, height_in, rear_fraction) in zip(cases, EXPECTED_CASES):
        assert isinstance(case, Mapping)
        context = f"tuning case {name}"
        expected_scalars = {
            "model_dof": 6.0,
            "cg_height_in": height_in,
            "cg_height_m": height_in * 0.0254,
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "rear_static_weight_fraction": rear_fraction,
            "front_static_weight_fraction": 1.0 - rear_fraction,
            "cg_x_m": -rear_fraction * EXPECTED_WHEELBASE_M,
            "tire_mu_scale": EXPECTED_MU_SCALE,
        }
        for field, expected in expected_scalars.items():
            _close(
                float(case.get(field, math.nan)),
                expected,
                context=f"{context} {field}",
                tolerance=1e-8,
            )
        front_arb = float(case.get("front_antiroll_stiffness_fraction", math.nan))
        brake = float(case.get("brake_distribution_front", math.nan))
        if not (math.isfinite(front_arb) and 0.0 <= front_arb <= 1.0):
            raise ValueError(f"{context} has invalid front ARB fraction.")
        if not (math.isfinite(brake) and 0.0 <= brake <= 1.0):
            raise ValueError(f"{context} has invalid front brake fraction.")
        if _as_bool(case.get("roll_optimization_at_search_bound", True)):
            raise ValueError(f"{context} roll optimum lies on its search bound.")
        if _as_bool(case.get("brake_optimization_at_search_bound", True)):
            raise ValueError(f"{context} brake optimum lies on its search bound.")
        trim = case.get("pure_lateral_limit_trim")
        if not isinstance(trim, Mapping):
            raise TypeError(f"{context}.pure_lateral_limit_trim must be an object.")
        if not _as_bool(trim.get("solver_success", False)):
            raise ValueError(f"{context} pure-lateral endpoint did not solve.")
        residual = float(trim.get("residual_norm", math.nan))
        if not math.isfinite(residual) or residual > 1e-4:
            raise ValueError(f"{context} pure-lateral trim residual is unacceptable.")
        if abs(float(trim.get("beta_rad", math.inf))) > 0.25 + 1e-8:
            raise ValueError(f"{context} pure-lateral beta exceeds the racing bound.")
        if abs(float(trim.get("steering_rad", math.inf))) > 0.5 + 1e-8:
            raise ValueError(f"{context} pure-lateral steer exceeds the racing bound.")
        loads = np.asarray(trim.get("normal_loads_n", []), dtype=float)
        if loads.shape != (4,) or not np.all(np.isfinite(loads)) or np.any(loads <= 0.0):
            raise ValueError(f"{context} pure-lateral wheel loads are invalid.")
        if np.any(loads < 100.0 - NORMAL_LOAD_NUMERICAL_TOLERANCE_N):
            raise ValueError(f"{context} pure-lateral loads are below the TIR fit range.")
        if np.any(loads > 1800.0 + NORMAL_LOAD_NUMERICAL_TOLERANCE_N):
            raise ValueError(f"{context} pure-lateral loads are above the TIR fit range.")
        robustness = case.get("robustness_lateral_limit_g_by_speed_mps")
        if not isinstance(robustness, Mapping) or len(robustness) < 4:
            raise ValueError(f"{context} omits the multi-speed lateral robustness check.")
        output[name] = {
            "front_antiroll_stiffness_fraction": front_arb,
            "brake_distribution_front": brake,
        }
    return output


def validate_design_inputs(
    sweep_80kw: Mapping[str, Any],
    sweep_32kw: Mapping[str, Any],
    tuning_artifact: Mapping[str, Any],
) -> dict[str, dict[str, float]]:
    """Validate exact geometry/mass/tuning and the intended power-only split."""

    tuning_80 = validate_configuration_sweep(sweep_80kw)
    tuning_32 = validate_configuration_sweep(sweep_32kw)
    for label, sweep in (("80kw", sweep_80kw), ("32kw", sweep_32kw)):
        _close(
            float(sweep.get("drive_power_limit_kw", math.nan)),
            EXPECTED_POWER_KW[label],
            context=f"{label} sweep drive power limit",
        )
    if not _equivalent(tuning_80, tuning_32):
        raise ValueError("80 and 32 kW sweep case tuning differs.")
    cases_80 = [dict(case) for case in sweep_80kw["cases"]]
    cases_32 = [dict(case) for case in sweep_32kw["cases"]]
    for case in cases_80:
        case.pop("drive_power_limit_kw", None)
    for case in cases_32:
        case.pop("drive_power_limit_kw", None)
    if not _equivalent(cases_80, cases_32):
        raise ValueError("80 and 32 kW sweeps differ beyond their power limit.")
    tuned = _case_tuning_from_artifact(tuning_artifact)
    if not _equivalent(tuning_80, tuned):
        raise ValueError("Runner sweep tuning does not match the physical tuning artifact.")
    return tuning_80


def _load_track_manifest(path: Path) -> tuple[dict[str, float], list[Path]]:
    payload = _read_json(path)
    events = payload.get("Events")
    if not isinstance(events, list):
        raise TypeError("Track manifest Events must be a list.")
    observed = [str(event.get("Slug")) for event in events if isinstance(event, Mapping)]
    if set(observed) != set(EXPECTED_EVENTS) or len(observed) != len(EXPECTED_EVENTS):
        raise ValueError("Track manifest does not contain exactly the four study events.")
    lengths: dict[str, float] = {}
    paths: list[Path] = []
    for event in events:
        assert isinstance(event, Mapping)
        slug = str(event["Slug"])
        length = float(event.get("TrackLength_m", math.nan))
        if not math.isfinite(length) or length <= 0.0:
            raise ValueError(f"Track manifest {slug} length is invalid.")
        csv_value = Path(str(event.get("OpenLAPTrackCsv", "")))
        csv_path = csv_value if csv_value.is_absolute() else LAPSIMS_ROOT / csv_value
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
        lengths[slug] = length
        paths.append(csv_path.resolve())
    return lengths, paths


def _source_manifest(
    *,
    bobsim_root: Path,
    sweep_80kw_path: Path,
    sweep_32kw_path: Path,
    tuning_path: Path,
    scoring_path: Path,
    track_manifest_path: Path,
    run_contract_path: Path,
) -> dict[str, dict[str, str]]:
    _lengths, track_paths = _load_track_manifest(track_manifest_path)
    lapsims_files = [
        sweep_80kw_path,
        sweep_32kw_path,
        tuning_path,
        scoring_path,
        track_manifest_path,
        run_contract_path,
        *track_paths,
        LAPSIMS_ROOT / "src/run_ggv_cg_smoke.py",
        LAPSIMS_ROOT / "src/reduced_ggv_bridge.py",
        LAPSIMS_ROOT / "src/ggv_lookup_solver.py",
        LAPSIMS_ROOT / "src/tune_dyn_py_cg_setups.py",
        LAPSIMS_ROOT / "src/audit_two_config_qss_samples.py",
        LAPSIMS_ROOT / "src/build_two_config_mixed_power_report.py",
        LAPSIMS_ROOT / "src/build_longitudinal_cg_mixed_power_report.py",
        LAPSIMS_ROOT / "src/build_longitudinal_cg_points_correlation.py",
        LAPSIMS_ROOT / "src/build_reduced_dof_cg_comparison.py",
        LAPSIMS_ROOT / "src/battery_dynamic_points.py",
        Path(__file__).resolve(),
    ]
    bobsim_files = [
        bobsim_root / "vehicle.yml",
        bobsim_root / "_0_Utils/tire_templates/16x7p5_10_12psi.tir",
        bobsim_root
        / "_3_StandardSim/generated_results/four_post_eval_report_metrics.csv",
        bobsim_root / "_2_EnvelopeSim/GGV/ggv_generation.py",
        bobsim_root / "_2_EnvelopeSim/GGV/ggv_config.yml",
        bobsim_root / "_2_EnvelopeSim/vehicle_yaml.py",
        bobsim_root / "_2_EnvelopeSim/YMD/ymd_generation.py",
        bobsim_root / "_0_Utils/vehicle_io.py",
        *sorted((bobsim_root / "_0_Utils/dyn_py").glob("*.py")),
        *sorted((bobsim_root / "_0_Utils/kin_py").rglob("*.py")),
    ]
    manifest: dict[str, dict[str, str]] = {}
    for prefix, root, paths in (
        ("lapsims", LAPSIMS_ROOT, lapsims_files),
        ("bobsim", bobsim_root, bobsim_files),
    ):
        for path in paths:
            resolved = path.resolve()
            if not resolved.is_file():
                raise FileNotFoundError(resolved)
            try:
                relative = resolved.relative_to(root.resolve()).as_posix()
            except ValueError as exc:
                raise ValueError(f"Locked source {resolved} is outside {root}.") from exc
            source_id = f"{prefix}:{relative}"
            manifest[source_id] = {
                "path": str(resolved),
                "sha256": _sha256(resolved),
            }
    return dict(sorted(manifest.items()))


def _validate_pinned_bobsim(bobsim_root: Path) -> dict[str, str]:
    head = _git_head(bobsim_root)
    if head != PINNED_BOBSIM_COMMIT:
        raise ValueError(
            f"BobSim HEAD must be {PINNED_BOBSIM_COMMIT}; got {head}."
        )
    observed: dict[str, str] = {}
    for relative, expected_hash in CRITICAL_BOBSIM_HASHES.items():
        path = bobsim_root / relative
        actual = _sha256(path)
        if actual != expected_hash:
            raise ValueError(
                f"Pinned BobSim critical hash drift for {relative}: {actual}."
            )
        observed[relative] = actual
    return observed


def _assert_preproduction_root(root: Path) -> None:
    if not root.exists():
        return
    forbidden = [root / "event_results.csv", root / "smoke_summary.json"]
    forbidden.extend(root.glob("*/ggv.csv"))
    existing = [path for path in forbidden if path.exists()]
    if existing:
        raise ValueError(
            f"Production root {root} already contains study artifacts: {existing}."
        )


def create_source_lock(
    *,
    bobsim_root: Path,
    sweep_80kw_path: Path,
    sweep_32kw_path: Path,
    tuning_path: Path,
    scoring_path: Path,
    track_manifest_path: Path,
    run_contract_path: Path,
    root_80kw: Path,
    root_32kw: Path,
    lock_path: Path,
) -> dict[str, Any]:
    if lock_path.exists():
        raise FileExistsError(f"Refusing to overwrite existing source lock {lock_path}.")
    sweep_80 = _read_json(sweep_80kw_path)
    sweep_32 = _read_json(sweep_32kw_path)
    tuning_artifact = _read_json(tuning_path)
    case_tuning = validate_design_inputs(sweep_80, sweep_32, tuning_artifact)
    run_contract = validate_run_contract(_read_json(run_contract_path))
    _load_scoring_reference(scoring_path)
    track_lengths, _track_paths = _load_track_manifest(track_manifest_path)
    _assert_preproduction_root(root_80kw)
    _assert_preproduction_root(root_32kw)
    critical = _validate_pinned_bobsim(bobsim_root)
    manifest = _source_manifest(
        bobsim_root=bobsim_root,
        sweep_80kw_path=sweep_80kw_path,
        sweep_32kw_path=sweep_32kw_path,
        tuning_path=tuning_path,
        scoring_path=scoring_path,
        track_manifest_path=track_manifest_path,
        run_contract_path=run_contract_path,
    )
    payload = {
        "schema": LOCK_SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "pinned_bobsim_root": str(bobsim_root.resolve()),
        "pinned_bobsim_commit": PINNED_BOBSIM_COMMIT,
        "critical_bobsim_hashes": critical,
        "planned_roots": {
            "80kw": str(root_80kw.resolve()),
            "32kw": str(root_32kw.resolve()),
        },
        "design": {
            "sweep_axis": CONFIGURATION_AXIS,
            "model_dof": EXPECTED_MODEL_DOF,
            "tire_mu_scale": EXPECTED_MU_SCALE,
            "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "drive": "RWD",
            "limited_slip_differential_model": False,
            "cases": [
                {
                    "name": name,
                    "cg_height_in": height,
                    "rear_static_weight_fraction": rear,
                    **case_tuning[name],
                }
                for name, height, rear in EXPECTED_CASES
            ],
            "power_limit_kw": EXPECTED_POWER_KW,
        },
        "track_lengths_m": track_lengths,
        "run_contract": run_contract,
        "source_manifest": manifest,
    }
    _write_json(lock_path, payload)
    return payload


def verify_source_lock(
    *,
    lock_path: Path,
    bobsim_root: Path,
    sweep_80kw_path: Path,
    sweep_32kw_path: Path,
    tuning_path: Path,
    scoring_path: Path,
    track_manifest_path: Path,
    run_contract_path: Path,
    root_80kw: Path,
    root_32kw: Path,
) -> dict[str, Any]:
    lock = _read_json(lock_path)
    if lock.get("schema") != LOCK_SCHEMA:
        raise ValueError(f"Unsupported source-lock schema in {lock_path}.")
    expected_roots = {
        "80kw": str(root_80kw.resolve()),
        "32kw": str(root_32kw.resolve()),
    }
    if lock.get("planned_roots") != expected_roots:
        raise ValueError("Audit roots do not match the pre-production source lock.")
    if str(bobsim_root.resolve()) != str(lock.get("pinned_bobsim_root")):
        raise ValueError("Audit BobSim root does not match the pre-production lock.")
    _validate_pinned_bobsim(bobsim_root)
    validate_design_inputs(
        _read_json(sweep_80kw_path),
        _read_json(sweep_32kw_path),
        _read_json(tuning_path),
    )
    run_contract = validate_run_contract(_read_json(run_contract_path))
    if not _equivalent(lock.get("run_contract"), run_contract):
        raise ValueError("Current run contract differs from the locked contract.")
    current = _source_manifest(
        bobsim_root=bobsim_root,
        sweep_80kw_path=sweep_80kw_path,
        sweep_32kw_path=sweep_32kw_path,
        tuning_path=tuning_path,
        scoring_path=scoring_path,
        track_manifest_path=track_manifest_path,
        run_contract_path=run_contract_path,
    )
    validate_locked_manifest(lock.get("source_manifest"), current)
    return lock


def validate_locked_manifest(
    locked: object,
    current: Mapping[str, Mapping[str, str]],
) -> None:
    """Fail on membership, absolute-path, or content-hash drift."""

    if not isinstance(locked, Mapping) or set(current) != set(locked):
        raise ValueError("Current source-manifest members differ from the lock.")
    drift = [
        source_id
        for source_id, current_entry in current.items()
        if not isinstance(locked[source_id], Mapping)
        or str(locked[source_id].get("path")) != current_entry["path"]
        or str(locked[source_id].get("sha256")) != current_entry["sha256"]
    ]
    if drift:
        raise ValueError(f"Source hash/path drift after lock: {drift}.")


def validate_ggv_map(
    path: Path,
    *,
    expected_speed_slices: Sequence[float] | None = None,
    symmetry_tolerance: float = 1e-9,
) -> dict[str, Any]:
    frame = pd.read_csv(path)
    _require_columns(frame, MAP_COLUMNS, str(path))
    for column in MAP_COLUMNS:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    if frame[["speed_mps", "ay_mps2"]].isna().any().any():
        raise ValueError(f"{path} contains nonnumeric speed/ay coordinates.")
    if frame.duplicated(["speed_mps", "ay_mps2"]).any():
        raise ValueError(f"{path} contains duplicate speed/ay coordinates.")
    speeds = np.sort(frame["speed_mps"].unique().astype(float))
    if len(speeds) < 2 or speeds[0] < -1e-12 or np.any(np.diff(speeds) <= 0.0):
        raise ValueError(f"{path} has an invalid speed topology.")
    if expected_speed_slices is not None:
        expected = np.asarray(expected_speed_slices, dtype=float)
        if expected.shape != speeds.shape or not np.allclose(
            expected, speeds, rtol=0.0, atol=1e-12
        ):
            raise ValueError(f"{path} speed slices disagree with case metadata.")

    endpoints: dict[str, float] = {}
    positive_drive_found = False
    negative_brake_found = False
    rows_per_speed: dict[str, int] = {}
    for speed, group in frame.groupby("speed_mps", sort=True):
        ordered = group.sort_values("ay_mps2", kind="mergesort").reset_index(drop=True)
        ay = ordered["ay_mps2"].to_numpy(dtype=float)
        if len(ay) < 5 or len(ay) % 2 == 0 or np.any(np.diff(ay) <= 0.0):
            raise ValueError(f"{path} speed {speed:g} has an invalid lateral grid.")
        if float(np.min(np.abs(ay))) > symmetry_tolerance:
            raise ValueError(f"{path} speed {speed:g} omits ay=0.")
        if not np.allclose(ay, -ay[::-1], rtol=0.0, atol=symmetry_tolerance):
            raise ValueError(f"{path} speed {speed:g} lateral grid is asymmetric.")
        for value_column, flag_column in (
            ("ax_accel_mps2", "accel_feasible"),
            ("ax_brake_mps2", "brake_feasible"),
        ):
            values = ordered[value_column].to_numpy(dtype=float)
            flags = ordered[flag_column].to_numpy(dtype=float)
            if not np.all(np.isin(flags, [0.0, 1.0])):
                raise ValueError(f"{path} {flag_column} must be binary.")
            if not np.array_equal(flags > 0.5, np.isfinite(values)):
                raise ValueError(
                    f"{path} {flag_column} disagrees with {value_column} finiteness."
                )
            if not np.array_equal(flags, flags[::-1]):
                raise ValueError(f"{path} {flag_column} is not laterally symmetric.")
            if not np.allclose(
                values, values[::-1], rtol=0.0, atol=symmetry_tolerance, equal_nan=True
            ):
                raise ValueError(f"{path} {value_column} is not laterally symmetric.")
        zero = ordered.iloc[int(np.argmin(np.abs(ay)))]
        positive_drive_found |= bool(
            zero["accel_feasible"] > 0.5 and zero["ax_accel_mps2"] > 0.0
        )
        negative_brake_found |= bool(
            zero["brake_feasible"] > 0.5 and zero["ax_brake_mps2"] < 0.0
        )
        endpoints[f"{float(speed):.12g}"] = float(np.max(np.abs(ay)))
        rows_per_speed[f"{float(speed):.12g}"] = len(ordered)
    if not positive_drive_found or not negative_brake_found:
        raise ValueError(f"{path} lacks a feasible ay=0 drive or brake branch.")
    return {
        "path": str(path.resolve()),
        "sha256": _sha256(path),
        "row_count": len(frame),
        "speed_slices_mps": speeds.tolist(),
        "rows_per_speed": rows_per_speed,
        "lateral_endpoints_mps2": endpoints,
        "symmetric_topology": True,
        "feasibility_masks_match_finiteness": True,
    }


def _root_case_dirs(root: Path) -> list[str]:
    return sorted(
        path.name
        for path in root.iterdir()
        if path.is_dir() and (path / "case_metadata.json").is_file()
    )


def _expected_requested_speed_slices(ggv_contract: Mapping[str, Any]) -> list[float]:
    top = float(ggv_contract["top_speed_mps"])
    step = float(ggv_contract["speed_step_mps"])
    regular = np.arange(step, top, step, dtype=float).tolist()
    speeds = [0.0, *regular]
    if not speeds or not math.isclose(speeds[-1], top, rel_tol=0.0, abs_tol=1e-12):
        speeds.append(top)
    return speeds


def validate_completed_root(
    root: Path,
    *,
    expected_power_w: float,
    expected_tuning: Mapping[str, Mapping[str, float]],
    track_lengths_m: Mapping[str, float],
    run_contract: Mapping[str, Any],
) -> dict[str, Any]:
    event_path = root / "event_results.csv"
    summary_path = root / "smoke_summary.json"
    if not event_path.is_file() or not summary_path.is_file():
        raise FileNotFoundError(f"{root} is missing event_results.csv or smoke_summary.json.")
    expected_names = [item[0] for item in EXPECTED_CASES]
    if _root_case_dirs(root) != sorted(expected_names):
        raise ValueError(f"{root} does not contain exactly the two configuration dirs.")
    events = pd.read_csv(event_path)
    required = (
        "cg_case",
        "event_slug",
        "lap_time_s",
        "track_length_m",
        "converged",
        "final_max_speed_change_mps",
        "ggv_source_csv",
        "ggv_source_sha256",
        "ggv_max_speed_domain_fraction",
        "ggv_speed_cap_segments",
        "model_family",
        "model_key",
        "model_dof",
        "sweep_axis",
        "effective_drive_power_limit_w",
    )
    _require_columns(events, required, str(event_path))
    if len(events) != len(EXPECTED_CASES) * len(EXPECTED_EVENTS):
        raise ValueError(f"{event_path} must contain exactly eight rows.")
    if events.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError(f"{event_path} contains duplicate configuration/event rows.")
    expected_pairs = {(name, event) for name in expected_names for event in EXPECTED_EVENTS}
    observed_pairs = set(
        zip(events["cg_case"].astype(str), events["event_slug"].astype(str))
    )
    if observed_pairs != expected_pairs:
        raise ValueError(f"{event_path} does not contain the exact case/event product.")
    if not events["converged"].map(_as_bool).all():
        raise ValueError(f"{event_path} contains a nonconverged event solve.")
    final_change = pd.to_numeric(
        events["final_max_speed_change_mps"], errors="raise"
    ).to_numpy(dtype=float)
    maximum_final_speed_change_mps = float(
        run_contract["acceptance"]["maximum_final_speed_change_mps"]
    )
    if not np.all(np.isfinite(final_change)) or np.any(
        final_change > maximum_final_speed_change_mps
    ):
        raise ValueError(f"{event_path} exceeds the final speed-change tolerance.")
    domain = pd.to_numeric(
        events["ggv_max_speed_domain_fraction"], errors="raise"
    ).to_numpy(dtype=float)
    if not np.all(np.isfinite(domain)) or np.any(domain < 0.0) or np.any(domain > 1.0 + 1e-12):
        raise ValueError(f"{event_path} exceeds the GGV speed domain.")
    caps = pd.to_numeric(events["ggv_speed_cap_segments"], errors="raise")
    if not bool((caps == 0).all()):
        raise ValueError(f"{event_path} contains GGV speed-cap segments.")
    if set(events["model_family"].astype(str)) != {MODEL_FAMILY}:
        raise ValueError(f"{event_path} has the wrong model family.")
    if set(events["model_key"].astype(str)) != {MODEL_KEY}:
        raise ValueError(f"{event_path} has the wrong model key.")
    if set(pd.to_numeric(events["model_dof"], errors="raise").astype(int)) != {6}:
        raise ValueError(f"{event_path} is not uniformly 6DOF.")
    if set(events["sweep_axis"].astype(str)) != {CONFIGURATION_AXIS}:
        raise ValueError(f"{event_path} has the wrong sweep axis.")
    power = pd.to_numeric(
        events["effective_drive_power_limit_w"], errors="raise"
    ).to_numpy(dtype=float)
    if not np.allclose(power, expected_power_w, rtol=0.0, atol=1e-8):
        raise ValueError(f"{event_path} has the wrong power limit.")
    times = pd.to_numeric(events["lap_time_s"], errors="raise").to_numpy(dtype=float)
    if not np.all(np.isfinite(times)) or np.any(times <= 0.0):
        raise ValueError(f"{event_path} contains invalid lap times.")
    for event_slug, length in track_lengths_m.items():
        observed = pd.to_numeric(
            events.loc[events["event_slug"] == event_slug, "track_length_m"],
            errors="raise",
        ).to_numpy(dtype=float)
        if observed.size != 2 or not np.allclose(observed, length, rtol=0.0, atol=1e-6):
            raise ValueError(f"{event_path} {event_slug} track length mismatch.")

    map_summaries: dict[str, Any] = {}
    metadata_rows: dict[str, dict[str, Any]] = {}
    artifact_hashes: dict[str, dict[str, str]] = {
        "event_results.csv": {
            "path": str(event_path.resolve()),
            "sha256": _sha256(event_path),
        },
        "smoke_summary.json": {
            "path": str(summary_path.resolve()),
            "sha256": _sha256(summary_path),
        },
    }
    for case_name in expected_names:
        case_dir = root / case_name
        required_artifacts = [case_dir / "ggv.csv", case_dir / "case_metadata.json"]
        for event_slug in EXPECTED_EVENTS:
            required_artifacts.extend(
                (
                    case_dir / f"{event_slug}_trace.csv",
                    case_dir / f"{event_slug}_summary.json",
                )
            )
        missing = [path for path in required_artifacts if not path.is_file()]
        if missing:
            raise FileNotFoundError(f"{case_dir} missing production artifacts: {missing}.")
        for artifact in required_artifacts:
            relative = artifact.relative_to(root).as_posix()
            artifact_hashes[relative] = {
                "path": str(artifact.resolve()),
                "sha256": _sha256(artifact),
            }
        metadata = _read_json(case_dir / "case_metadata.json")
        validate_configuration_metadata(metadata, expected_tuning=expected_tuning)
        _close(
            float(metadata.get("effective_drive_power_limit_w", math.nan)),
            expected_power_w,
            context=f"{case_name} metadata power limit",
            tolerance=1e-8,
        )
        ggv_contract = run_contract["ggv"]
        exact_generation_fields = {
            "ggv_ay_points": int(ggv_contract["ay_points"]),
            "ggv_ay_max_g": float(ggv_contract["ay_max_g"]),
            "ggv_ax_search_points": int(ggv_contract["ax_search_points"]),
            "ggv_ax_binary_iterations": int(ggv_contract["ax_binary_iterations"]),
            "qss_zero_speed_proxy_mps": float(ggv_contract["qss_zero_proxy_mps"]),
        }
        for field, expected in exact_generation_fields.items():
            _close(
                float(metadata.get(field, math.nan)),
                float(expected),
                context=f"{case_name} metadata {field}",
                tolerance=1e-12,
            )
        requested_speeds = metadata.get("ggv_requested_speed_slices_mps")
        expected_requested_speeds = _expected_requested_speed_slices(ggv_contract)
        observed_requested = (
            np.asarray(requested_speeds, dtype=float)
            if isinstance(requested_speeds, list)
            else np.asarray([], dtype=float)
        )
        expected_requested = np.asarray(expected_requested_speeds, dtype=float)
        if observed_requested.shape != expected_requested.shape or not np.allclose(
            observed_requested,
            expected_requested,
            rtol=0.0,
            atol=1e-12,
        ):
            raise ValueError(f"{case_name} requested GGV speed grid violates the run contract.")
        map_path = (case_dir / "ggv.csv").resolve()
        serialized_speeds = metadata.get("ggv_speed_slices")
        if not isinstance(serialized_speeds, list) or len(serialized_speeds) < 2:
            raise ValueError(f"{case_name} metadata omits the serialized GGV speed grid.")
        map_summary = validate_ggv_map(
            map_path,
            expected_speed_slices=serialized_speeds,
        )
        map_hash = map_summary["sha256"]
        if str(metadata.get("ggv_source_sha256", "")) != map_hash:
            raise ValueError(f"{case_name} metadata GGV hash mismatch.")
        group = events.loc[events["cg_case"].astype(str) == case_name]
        row_paths = {str(Path(str(path)).resolve()) for path in group["ggv_source_csv"]}
        if row_paths != {str(map_path)}:
            raise ValueError(f"{case_name} event rows do not use their root-local map.")
        if set(group["ggv_source_sha256"].astype(str)) != {map_hash}:
            raise ValueError(f"{case_name} event rows have a stale map hash.")
        map_summaries[case_name] = map_summary
        metadata_rows[case_name] = metadata

    summary = _read_json(summary_path)
    if str(summary.get("sweep_axis")) != CONFIGURATION_AXIS:
        raise ValueError(f"{summary_path} has the wrong sweep axis.")
    if str(summary.get("model_key")) != MODEL_KEY or int(summary.get("model_dof", -1)) != 6:
        raise ValueError(f"{summary_path} has the wrong model identity.")
    return {
        "root": str(root.resolve()),
        "event_results_sha256": _sha256(event_path),
        "smoke_summary_sha256": _sha256(summary_path),
        "events": events,
        "metadata": metadata_rows,
        "maps": map_summaries,
        "artifact_hashes": dict(sorted(artifact_hashes.items())),
        "maximum_final_speed_change_mps": float(np.max(final_change)),
        "maximum_ggv_speed_domain_fraction": float(np.max(domain)),
        "ggv_speed_cap_segments": int(caps.sum()),
    }


def validate_power_independent_match(
    study_80: Mapping[str, Any],
    study_32: Mapping[str, Any],
    *,
    lateral_equality_tolerance_mps2: float = 1e-9,
) -> dict[str, Any]:
    controlled_paths = (
        "model_family",
        "model_key",
        "model_dof",
        "sweep_axis",
        "cg_case",
        "cg_height_m",
        "cg_height_in",
        "sprung_mass_kg",
        "total_mass_kg",
        "rear_static_weight_fraction",
        "front_static_weight_fraction",
        "front_antiroll_stiffness_fraction",
        "front_elastic_roll_stiffness_fraction",
        "effective_brake_distribution_front",
        "effective_aero_balance_front",
        "effective_cop_from_front_m",
        "tire_mu_scale",
        "drive_distribution_front",
        "limited_slip_differential_model",
        "case_tuning",
        "vehicle.mass",
        "vehicle.wheelbase",
        "vehicle.track_front",
        "vehicle.track_rear",
        "vehicle.cg_height",
        "vehicle.front_static_frac",
        "vehicle.lltd",
        "vehicle.rho",
        "vehicle.cl_a",
        "vehicle.cd_a",
        "vehicle.aero_balance_front",
        "vehicle.max_drive_force",
        "vehicle.max_drive_speed",
        "vehicle.max_brake_force",
        "vehicle.drive_distribution_front",
        "vehicle.brake_distribution_front",
        "vehicle.fz_ref",
        "vehicle.fz_min_valid",
        "vehicle.fz_max_valid",
        "vehicle.pdx1",
        "vehicle.pdx2",
        "vehicle.pdy1",
        "vehicle.pdy2",
        "vehicle.mu_min",
        "reduced_parameter_overrides.absolute_cg_height_m",
        "reduced_parameter_overrides.sprung_mass_kg",
        "reduced_parameter_overrides.total_mass_kg",
        "reduced_parameter_overrides.fixed_unsprung_mass_kg",
        "reduced_parameter_overrides.rear_static_weight_fraction",
        "reduced_parameter_overrides.front_static_weight_fraction",
        "reduced_parameter_overrides.center_of_gravity_m",
        "reduced_parameter_overrides.aero_balance_front",
        "reduced_parameter_overrides.aero_cop_body_m",
        "reduced_parameter_overrides.brake_distribution_front",
        "reduced_parameter_overrides.front_antiroll_stiffness_fraction",
        "reduced_parameter_overrides.front_elastic_roll_stiffness_fraction",
        "reduced_parameter_overrides.tire_mu_scale",
        "reduced_parameter_overrides.effective_tire_peak_parameters",
        "reduced_vehicle_parameters.mass_kg",
        "reduced_vehicle_parameters.sprung_mass_kg",
        "reduced_vehicle_parameters.center_of_gravity_m",
        "reduced_vehicle_parameters.inertia_kg_m2",
        "reduced_vehicle_parameters.sprung_inertia_kg_m2",
        "reduced_vehicle_parameters.corner_positions_m",
        "reduced_vehicle_parameters.static_wheel_loads_n",
        "reduced_vehicle_parameters.wheel_radius_m",
        "reduced_vehicle_parameters.wheel_inertia_kg_m2",
        "reduced_vehicle_parameters.unsprung_mass_kg",
        "reduced_vehicle_parameters.suspension_stiffness_n_per_m",
        "reduced_vehicle_parameters.suspension_damping_n_s_per_m",
        "reduced_vehicle_parameters.antiroll_stiffness_nm_per_rad",
        "reduced_vehicle_parameters.double_wishbone",
        "reduced_vehicle_parameters.tire_vertical_stiffness_n_per_m",
        "reduced_vehicle_parameters.tire_vertical_damping_n_s_per_m",
        "reduced_vehicle_parameters.tire",
        "reduced_vehicle_parameters.rho_air_kg_m3",
        "reduced_vehicle_parameters.cl_area_m2",
        "reduced_vehicle_parameters.cd_area_m2",
        "reduced_vehicle_parameters.aero_balance_front",
        "reduced_vehicle_parameters.aero_cop_m",
        "reduced_vehicle_parameters.aero_drag_application_m",
        "reduced_vehicle_parameters.peak_drive_force_n",
        "reduced_vehicle_parameters.continuous_drive_force_n",
        "reduced_vehicle_parameters.maximum_drive_speed_mps",
        "reduced_vehicle_parameters.drive_distribution_front",
        "reduced_vehicle_parameters.brake_distribution_front",
    )

    def nested(payload: Mapping[str, Any], path: str) -> Any:
        value: Any = payload
        for token in path.split("."):
            if not isinstance(value, Mapping) or token not in value:
                return None
            value = value[token]
        return value

    comparisons: list[dict[str, Any]] = []
    lateral_comparisons: list[dict[str, Any]] = []
    for case_name, _height, _rear in EXPECTED_CASES:
        metadata_80 = study_80["metadata"][case_name]
        metadata_32 = study_32["metadata"][case_name]
        for path in controlled_paths:
            if not _equivalent(nested(metadata_80, path), nested(metadata_32, path)):
                raise ValueError(f"80/32 controlled metadata mismatch at {case_name}.{path}.")
        comparisons.append({"cg_case": case_name, "controlled_metadata_match": True})

        endpoints_80 = study_80["maps"][case_name]["lateral_endpoints_mps2"]
        endpoints_32 = study_32["maps"][case_name]["lateral_endpoints_mps2"]
        lateral_comparisons.append(
            validate_lateral_endpoint_equality(
                endpoints_80,
                endpoints_32,
                case_name=case_name,
                tolerance_mps2=lateral_equality_tolerance_mps2,
            )
        )
    return {
        "controlled_metadata": comparisons,
        "lateral_boundary": lateral_comparisons,
    }


def validate_lateral_endpoint_equality(
    endpoints_80: Mapping[str, float],
    endpoints_32: Mapping[str, float],
    *,
    case_name: str,
    tolerance_mps2: float,
) -> dict[str, Any]:
    """Compare lateral boundaries wherever the 32 kW map remains sustainable."""

    speeds_80 = set(endpoints_80)
    speeds_32 = set(endpoints_32)
    if not speeds_32 or not speeds_32.issubset(speeds_80):
        raise ValueError(
            f"32 kW lateral speed domain is not a nonempty subset of the "
            f"80 kW domain for {case_name}."
        )
    deltas = {
        speed: float(endpoints_80[speed]) - float(endpoints_32[speed])
        for speed in speeds_32
    }
    maximum = max(abs(value) for value in deltas.values())
    if maximum > tolerance_mps2:
        raise ValueError(
            f"80/32 lateral boundary differs for {case_name} by "
            f"{maximum:.3g} m/s^2."
        )
    return {
        "cg_case": case_name,
        "common_speed_slice_count": len(deltas),
        "80kw_speed_slice_count": len(speeds_80),
        "32kw_speed_slice_count": len(speeds_32),
        "32kw_domain_is_subset_of_80kw_domain": True,
        "80kw_only_power_sustainable_speed_slices_mps": sorted(
            float(speed) for speed in speeds_80.difference(speeds_32)
        ),
        "maximum_abs_80kw_minus_32kw_lateral_endpoint_mps2": maximum,
        "lateral_endpoints_equal_at_all_common_speeds": True,
    }


def validate_qss_audit(
    root: Path,
    *,
    expected_tuning: Mapping[str, Mapping[str, float]],
    run_contract: Mapping[str, Any],
) -> dict[str, Any]:
    audit_dir = root / "reduced_qss_audit"
    sample_path = audit_dir / "sampled_trim_states.csv"
    summary_path = audit_dir / "case_qss_audit_summary.csv"
    json_path = audit_dir / "reduced_qss_audit.json"
    for path in (sample_path, summary_path, json_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    frame = pd.read_csv(sample_path)
    _require_columns(frame, QSS_COLUMNS, str(sample_path))
    expected_names = {item[0] for item in EXPECTED_CASES}
    if set(frame["cg_case"].astype(str)) != expected_names:
        raise ValueError(f"{sample_path} has the wrong configuration set.")
    if set(frame["sweep_axis"].astype(str)) != {CONFIGURATION_AXIS}:
        raise ValueError(f"{sample_path} has the wrong sweep axis.")
    if set(pd.to_numeric(frame["model_dof"], errors="raise").astype(int)) != {6}:
        raise ValueError(f"{sample_path} is not uniformly 6DOF.")

    numeric = (
        "trim_residual_norm",
        "beta_rad",
        "steering_rad",
        "fz_fl_n",
        "fz_fr_n",
        "fz_rl_n",
        "fz_rr_n",
        "normal_load_min_n",
        "normal_load_max_n",
        "cg_height_in",
        "sprung_mass_kg",
        "total_mass_kg",
        "rear_static_weight_fraction",
        "front_static_weight_fraction",
        "front_antiroll_stiffness_fraction",
        "brake_distribution_front",
        "aero_balance_front",
        "tire_mu_scale",
        "tir_valid_load_min_n",
        "tir_valid_load_max_n",
    )
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    if not np.all(np.isfinite(frame[list(numeric)].to_numpy(dtype=float))):
        raise ValueError(f"{sample_path} contains non-finite state/load values.")
    if not frame["trim_success"].map(_as_bool).all():
        raise ValueError(f"{sample_path} contains failed trims.")
    acceptance = run_contract["acceptance"]
    maximum_residual_norm = float(acceptance["maximum_trim_residual_norm"])
    if (frame["trim_residual_norm"] > maximum_residual_norm).any():
        raise ValueError(f"{sample_path} exceeds the trim residual tolerance.")
    maximum_beta = float(acceptance["maximum_abs_beta_rad"])
    maximum_steer = float(acceptance["maximum_abs_steering_rad"])
    if (frame["beta_rad"].abs() > maximum_beta + 1e-8).any():
        raise ValueError(f"{sample_path} exceeds the sideslip bound.")
    if (frame["steering_rad"].abs() > maximum_steer + 1e-8).any():
        raise ValueError(f"{sample_path} exceeds the steering bound.")
    load_columns = ["fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n"]
    loads = frame[load_columns].to_numpy(dtype=float)
    if np.any(loads <= 0.0):
        raise ValueError(f"{sample_path} contains wheel lift/nonpositive load.")
    tir_min = float(acceptance["tir_valid_load_min_n"])
    tir_max = float(acceptance["tir_valid_load_max_n"])
    load_tolerance = float(acceptance["normal_load_numerical_tolerance_n"])
    if not math.isclose(
        load_tolerance,
        NORMAL_LOAD_NUMERICAL_TOLERANCE_N,
        rel_tol=0.0,
        abs_tol=0.0,
    ):
        raise ValueError("Run contract has the wrong normal-load numerical tolerance.")
    if not np.allclose(frame["tir_valid_load_min_n"], tir_min, rtol=0.0, atol=1e-10):
        raise ValueError(f"{sample_path} does not use the locked TIR lower fit bound.")
    if not np.allclose(frame["tir_valid_load_max_n"], tir_max, rtol=0.0, atol=1e-10):
        raise ValueError(f"{sample_path} does not use the locked TIR upper fit bound.")
    if np.any(
        loads
        < frame["tir_valid_load_min_n"].to_numpy()[:, None] - load_tolerance
    ):
        raise ValueError(f"{sample_path} samples below the TIR fitted load range.")
    if np.any(
        loads
        > frame["tir_valid_load_max_n"].to_numpy()[:, None] + load_tolerance
    ):
        raise ValueError(f"{sample_path} samples above the TIR fitted load range.")

    for name, height, rear in EXPECTED_CASES:
        group = frame.loc[frame["cg_case"].astype(str) == name]
        trace_events = set(group.loc[group["event_slug"] != "GGV", "event_slug"].astype(str))
        if trace_events != set(EXPECTED_EVENTS):
            raise ValueError(f"{sample_path} {name} lacks one or more event samples.")
        if not group["source_kind"].astype(str).str.contains("boundary_pure_lateral").any():
            raise ValueError(f"{sample_path} {name} lacks a pure-lateral boundary sample.")
        exact = {
            "cg_height_in": height,
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
            "rear_static_weight_fraction": rear,
            "front_static_weight_fraction": 1.0 - rear,
            "front_antiroll_stiffness_fraction": expected_tuning[name][
                "front_antiroll_stiffness_fraction"
            ],
            "brake_distribution_front": expected_tuning[name][
                "brake_distribution_front"
            ],
            "aero_balance_front": EXPECTED_AERO_BALANCE_FRONT,
            "tire_mu_scale": EXPECTED_MU_SCALE,
        }
        for column, expected in exact.items():
            if not np.allclose(group[column], expected, rtol=0.0, atol=1e-8):
                raise ValueError(f"{sample_path} {name} {column} identity mismatch.")

    summary = pd.read_csv(summary_path)
    if set(summary["cg_case"].astype(str)) != expected_names or len(summary) != 2:
        raise ValueError(f"{summary_path} must contain exactly two case summaries.")
    summary_columns = (
        "sample_count",
        "successful_trim_count",
        "failed_trim_count",
        "maximum_abs_beta_rad",
        "maximum_abs_steering_rad",
        "maximum_residual_norm",
        "sampled_normal_load_min_n",
        "sampled_normal_load_max_n",
    )
    _require_columns(summary, summary_columns, str(summary_path))
    for case_name in expected_names:
        samples = frame.loc[frame["cg_case"].astype(str) == case_name]
        row = summary.loc[summary["cg_case"].astype(str) == case_name].iloc[0]
        exact_counts = {
            "sample_count": len(samples),
            "successful_trim_count": len(samples),
            "failed_trim_count": 0,
        }
        for column, expected in exact_counts.items():
            if int(row[column]) != expected:
                raise ValueError(f"{summary_path} {case_name} {column} is stale.")
        recomputed = {
            "maximum_abs_beta_rad": float(samples["beta_rad"].abs().max()),
            "maximum_abs_steering_rad": float(samples["steering_rad"].abs().max()),
            "maximum_residual_norm": float(samples["trim_residual_norm"].max()),
            "sampled_normal_load_min_n": float(samples[load_columns].min().min()),
            "sampled_normal_load_max_n": float(samples[load_columns].max().max()),
        }
        for column, expected in recomputed.items():
            _close(
                float(row[column]),
                expected,
                context=f"{summary_path} {case_name} {column}",
                tolerance=1e-10,
            )
    payload = _read_json(json_path)
    if payload.get("schema") != "lapsims.two-config-reduced-qss-sampled-audit.v1":
        raise ValueError(f"{json_path} has the wrong schema.")
    sampling = payload.get("sampling")
    if not isinstance(sampling, Mapping):
        raise TypeError(f"{json_path} sampling must be an object.")
    for field, expected in run_contract["sampled_qss_audit"].items():
        if field not in sampling or not _equivalent(sampling[field], expected):
            raise ValueError(f"{json_path} sampling.{field} violates the run contract.")
    json_summaries = payload.get("case_summaries")
    if not isinstance(json_summaries, list) or {
        str(item.get("cg_case"))
        for item in json_summaries
        if isinstance(item, Mapping)
    } != expected_names:
        raise ValueError(f"{json_path} does not contain both case summaries.")
    return {
        "sample_csv": str(sample_path.resolve()),
        "sample_csv_sha256": _sha256(sample_path),
        "case_summary_csv": str(summary_path.resolve()),
        "case_summary_csv_sha256": _sha256(summary_path),
        "audit_json": str(json_path.resolve()),
        "audit_json_sha256": _sha256(json_path),
        "sample_count": len(frame),
        "case_sample_counts": frame.groupby("cg_case").size().to_dict(),
        "maximum_trim_residual_norm": float(frame["trim_residual_norm"].max()),
        "maximum_abs_beta_rad": float(frame["beta_rad"].abs().max()),
        "maximum_abs_steering_rad": float(frame["steering_rad"].abs().max()),
        "minimum_sampled_wheel_load_n": float(np.min(loads)),
        "maximum_sampled_wheel_load_n": float(np.max(loads)),
        "all_samples_within_tir_fit_load_range": True,
        "all_four_events_and_pure_lateral_boundary_sampled_per_case": True,
    }


def _load_scoring_reference(path: Path) -> dict[str, Any]:
    payload = _read_json(path)
    events = payload.get("events")
    if not isinstance(events, Mapping) or set(events) != set(EXPECTED_EVENTS):
        raise ValueError("Scoring reference must contain exactly the four events.")
    expected_rule_slugs = {
        "acceleration": "acceleration",
        "skidpad": "skidpad",
        "autocross": "autocross",
        "michigan_endurance": "endurance",
    }
    for event_slug, reference in events.items():
        if not isinstance(reference, Mapping):
            raise TypeError(f"Scoring reference {event_slug} must be an object.")
        if str(reference.get("rule_slug")) != expected_rule_slugs[event_slug]:
            raise ValueError(f"Scoring reference {event_slug} has the wrong rule.")
        times = np.asarray(reference.get("valid_adjusted_times_s", []), dtype=float)
        if times.ndim != 1 or times.size == 0 or not np.all(np.isfinite(times)):
            raise ValueError(f"Scoring reference {event_slug} times are invalid.")
        if np.any(times <= 0.0) or np.any(np.diff(times) < 0.0):
            raise ValueError(f"Scoring reference {event_slug} times are not sorted positive.")
        _close(
            float(reference.get("fastest_adjusted_time_s", math.nan)),
            float(times[0]),
            context=f"scoring reference {event_slug} fastest time",
            tolerance=0.0,
        )
    return payload


def independent_mixed_scoring(
    events_80kw: pd.DataFrame,
    events_32kw: pd.DataFrame,
    scoring_reference: Mapping[str, Any],
) -> tuple[pd.DataFrame, pd.DataFrame]:
    sprint = events_80kw.loc[
        events_80kw["event_slug"].astype(str).isin(SPRINT_EVENTS)
    ].copy()
    endurance = events_32kw.loc[
        events_32kw["event_slug"].astype(str) == ENDURANCE_EVENT
    ].copy()
    sprint["audit_source_power_kw"] = 80.0
    endurance["audit_source_power_kw"] = 32.0
    mixed = pd.concat((sprint, endurance), ignore_index=True, sort=False)
    if len(mixed) != 8 or mixed.duplicated(["cg_case", "event_slug"]).any():
        raise ValueError("Mixed-power selection did not produce eight unique rows.")
    mixed["audit_competition_time_s"] = np.nan
    mixed["audit_tmin_s"] = np.nan
    mixed["audit_tmax_s"] = np.nan
    mixed["audit_projected_points"] = np.nan
    mixed["audit_is_exact_tmin"] = False
    mixed["audit_simulated_rank_in_real_plus_two_sim_field"] = 0
    for event_slug in EXPECTED_EVENTS:
        mask = mixed["event_slug"].astype(str) == event_slug
        group = mixed.loc[mask]
        reference = scoring_reference["events"][event_slug]
        multiplier = 1.0
        if str(reference.get("simulation_time_conversion")) == "distance_scaled":
            multiplier = float(reference["official_event_distance_m"]) / pd.to_numeric(
                group["track_length_m"], errors="raise"
            ).to_numpy(dtype=float)
        raw = pd.to_numeric(group["lap_time_s"], errors="raise").to_numpy(dtype=float)
        competition = raw * multiplier
        official = np.asarray(reference["valid_adjusted_times_s"], dtype=float)
        tmin = float(min(np.min(official), np.min(competition)))
        rule = SCORE_RULES[event_slug]
        points = [rule.score(float(time), tmin) for time in competition]
        ranks = [1 + int(np.count_nonzero(np.concatenate((official, competition)) < time)) for time in competition]
        mixed.loc[mask, "audit_competition_time_s"] = competition
        mixed.loc[mask, "audit_tmin_s"] = tmin
        mixed.loc[mask, "audit_tmax_s"] = tmin * rule.tmax_factor
        mixed.loc[mask, "audit_projected_points"] = points
        mixed.loc[mask, "audit_is_exact_tmin"] = competition == tmin
        mixed.loc[mask, "audit_simulated_rank_in_real_plus_two_sim_field"] = ranks
    if "projected_points" in mixed:
        source_points = pd.to_numeric(mixed["projected_points"], errors="coerce")
        finite = np.isfinite(source_points)
        if finite.any() and not np.allclose(
            source_points[finite],
            mixed.loc[finite, "audit_projected_points"],
            rtol=0.0,
            atol=1e-8,
        ):
            raise ValueError("Selected source projected points disagree with recomputation.")
    order = pd.Categorical(mixed["cg_case"], [item[0] for item in EXPECTED_CASES], ordered=True)
    event_order = pd.Categorical(mixed["event_slug"], EXPECTED_EVENTS, ordered=True)
    mixed = (
        mixed.assign(_case_order=order, _event_order=event_order)
        .sort_values(["_case_order", "_event_order"])
        .drop(columns=["_case_order", "_event_order"])
        .reset_index(drop=True)
    )
    totals = (
        mixed.groupby("cg_case", sort=False)["audit_projected_points"]
        .sum()
        .rename("audit_timed_event_points")
        .reset_index()
    )
    baseline = float(
        totals.loc[
            totals["cg_case"] == EXPECTED_CASES[0][0],
            "audit_timed_event_points",
        ].iloc[0]
    )
    totals["audit_points_delta_vs_config_nominal_54"] = (
        totals["audit_timed_event_points"] - baseline
    )
    totals["audit_rank"] = (
        totals["audit_timed_event_points"].rank(ascending=False, method="min").astype(int)
    )
    return mixed, totals


def compare_report_recomputation(
    report_root: Path,
    mixed: pd.DataFrame,
    totals: pd.DataFrame,
) -> dict[str, Any]:
    event_path = report_root / "two_config_event_results.csv"
    totals_path = report_root / "two_config_case_points.csv"
    provenance_path = report_root / "two_config_provenance.json"
    validation_path = report_root / "validation_summary.json"
    for path in (event_path, totals_path, provenance_path, validation_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    report_events = pd.read_csv(event_path)
    report_totals = pd.read_csv(totals_path)
    joined = mixed.merge(
        report_events,
        on=["cg_case", "event_slug"],
        how="inner",
        validate="one_to_one",
        suffixes=("_audit_source", "_report"),
    )
    if len(joined) != 8:
        raise ValueError("Report does not contain the same eight selected event rows.")
    comparisons = (
        ("audit_competition_time_s", "common_projected_competition_time_s"),
        ("audit_tmin_s", "common_field_tmin_s"),
        ("audit_projected_points", "common_projected_points"),
        ("audit_source_power_kw", "mixed_power_power_limit_w"),
    )
    for audit_column, report_column in comparisons:
        if report_column not in joined:
            raise ValueError(f"Report omits {report_column}.")
        report_values = pd.to_numeric(joined[report_column], errors="raise").to_numpy(dtype=float)
        if report_column == "mixed_power_power_limit_w":
            report_values /= 1000.0
        audit_values = pd.to_numeric(joined[audit_column], errors="raise").to_numpy(dtype=float)
        if not np.allclose(audit_values, report_values, rtol=0.0, atol=1e-8):
            raise ValueError(f"Report {report_column} disagrees with independent audit.")
    totals_joined = totals.merge(
        report_totals,
        on="cg_case",
        how="inner",
        validate="one_to_one",
    )
    if len(totals_joined) != 2 or not np.allclose(
        totals_joined["audit_timed_event_points"],
        pd.to_numeric(totals_joined["timed_event_points"], errors="raise"),
        rtol=0.0,
        atol=1e-8,
    ):
        raise ValueError("Report case totals disagree with independent audit.")
    validation = _read_json(validation_path)
    if validation.get("status") != "passed":
        raise ValueError("Report validation_summary.json is not passed.")
    return {
        "report_root": str(report_root.resolve()),
        "event_results_sha256": _sha256(event_path),
        "case_points_sha256": _sha256(totals_path),
        "provenance_sha256": _sha256(provenance_path),
        "validation_sha256": _sha256(validation_path),
        "event_rows_match_independent_recomputation": True,
        "case_totals_match_independent_recomputation": True,
    }


def run_acceptance_audit(args: argparse.Namespace) -> dict[str, Any]:
    bobsim_root = Path(args.bobsim_root).resolve()
    root_80 = Path(args.root_80kw).resolve()
    root_32 = Path(args.root_32kw).resolve()
    sweep_80_path = Path(args.sweep_80kw).resolve()
    sweep_32_path = Path(args.sweep_32kw).resolve()
    tuning_path = Path(args.tuning_json).resolve()
    scoring_path = Path(args.scoring_json).resolve()
    track_manifest_path = Path(args.track_manifest).resolve()
    run_contract_path = Path(args.run_contract).resolve()
    output_dir = Path(args.output_dir).resolve()
    lock = verify_source_lock(
        lock_path=Path(args.lock_file).resolve(),
        bobsim_root=bobsim_root,
        sweep_80kw_path=sweep_80_path,
        sweep_32kw_path=sweep_32_path,
        tuning_path=tuning_path,
        scoring_path=scoring_path,
        track_manifest_path=track_manifest_path,
        run_contract_path=run_contract_path,
        root_80kw=root_80,
        root_32kw=root_32,
    )
    expected_tuning = validate_design_inputs(
        _read_json(sweep_80_path),
        _read_json(sweep_32_path),
        _read_json(tuning_path),
    )
    run_contract = validate_run_contract(_read_json(run_contract_path))
    track_lengths, _paths = _load_track_manifest(track_manifest_path)
    study_80 = validate_completed_root(
        root_80,
        expected_power_w=EXPECTED_POWER_W["80kw"],
        expected_tuning=expected_tuning,
        track_lengths_m=track_lengths,
        run_contract=run_contract,
    )
    study_32 = validate_completed_root(
        root_32,
        expected_power_w=EXPECTED_POWER_W["32kw"],
        expected_tuning=expected_tuning,
        track_lengths_m=track_lengths,
        run_contract=run_contract,
    )
    power_independent = validate_power_independent_match(
        study_80,
        study_32,
        lateral_equality_tolerance_mps2=float(
            run_contract["acceptance"]["lateral_equality_tolerance_mps2"]
        ),
    )
    scoring_reference = _load_scoring_reference(scoring_path)
    mixed, totals = independent_mixed_scoring(
        study_80["events"], study_32["events"], scoring_reference
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    event_output = output_dir / "independent_mixed_event_recompute.csv"
    totals_output = output_dir / "independent_mixed_case_totals_recompute.csv"
    mixed.to_csv(event_output, index=False)
    totals.to_csv(totals_output, index=False)
    report = None
    if args.report_root is not None:
        report = compare_report_recomputation(
            Path(args.report_root).resolve(), mixed, totals
        )
    qss_80 = validate_qss_audit(
        root_80,
        expected_tuning=expected_tuning,
        run_contract=run_contract,
    )
    qss_32 = validate_qss_audit(
        root_32,
        expected_tuning=expected_tuning,
        run_contract=run_contract,
    )
    payload = {
        "schema": AUDIT_SCHEMA,
        "status": "passed",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "source_lock": {
            "path": str(Path(args.lock_file).resolve()),
            "sha256": _sha256(Path(args.lock_file).resolve()),
            "schema": lock["schema"],
            "all_locked_sources_unchanged": True,
        },
        "exact_design_and_tuning_validated": True,
        "root_80kw": {
            key: value
            for key, value in study_80.items()
            if key not in {"events", "metadata"}
        },
        "root_32kw": {
            key: value
            for key, value in study_32.items()
            if key not in {"events", "metadata"}
        },
        "power_independent_validation": power_independent,
        "qss_80kw": qss_80,
        "qss_32kw": qss_32,
        "independent_scoring": {
            "event_csv": str(event_output),
            "event_csv_sha256": _sha256(event_output),
            "totals_csv": str(totals_output),
            "totals_csv_sha256": _sha256(totals_output),
            "efficiency_points_included": False,
            "maximum_timed_event_points": 575.0,
            "case_totals": totals.to_dict(orient="records"),
        },
        "report_recomputation": report,
        "acceptance_claim": (
            "Passed only for the exact locked two-case 6DOF categorical "
            "comparison; this is not empirical vehicle correlation or a "
            "one-variable CG sensitivity."
        ),
    }
    acceptance_path = output_dir / "two_config_production_acceptance.json"
    _write_json(acceptance_path, payload)
    return payload


def _add_common_inputs(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bobsim-root", type=Path, default=DEFAULT_BOBSIM_ROOT)
    parser.add_argument("--sweep-80kw", type=Path, required=True)
    parser.add_argument("--sweep-32kw", type=Path, required=True)
    parser.add_argument("--tuning-json", type=Path, required=True)
    parser.add_argument("--scoring-json", type=Path, default=DEFAULT_SCORING_JSON)
    parser.add_argument("--track-manifest", type=Path, default=DEFAULT_TRACK_MANIFEST)
    parser.add_argument("--run-contract", type=Path, default=DEFAULT_RUN_CONTRACT)
    parser.add_argument("--root-80kw", type=Path, required=True)
    parser.add_argument("--root-32kw", type=Path, required=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    lock = subparsers.add_parser("lock", help="Validate and hash-lock preproduction inputs.")
    _add_common_inputs(lock)
    lock.add_argument("--lock-file", type=Path, required=True)
    audit = subparsers.add_parser("audit", help="Audit completed roots without simulation.")
    _add_common_inputs(audit)
    audit.add_argument("--lock-file", type=Path, required=True)
    audit.add_argument("--output-dir", type=Path, required=True)
    audit.add_argument("--report-root", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.command == "lock":
        payload = create_source_lock(
            bobsim_root=Path(args.bobsim_root).resolve(),
            sweep_80kw_path=Path(args.sweep_80kw).resolve(),
            sweep_32kw_path=Path(args.sweep_32kw).resolve(),
            tuning_path=Path(args.tuning_json).resolve(),
            scoring_path=Path(args.scoring_json).resolve(),
            track_manifest_path=Path(args.track_manifest).resolve(),
            run_contract_path=Path(args.run_contract).resolve(),
            root_80kw=Path(args.root_80kw).resolve(),
            root_32kw=Path(args.root_32kw).resolve(),
            lock_path=Path(args.lock_file).resolve(),
        )
    else:
        try:
            payload = run_acceptance_audit(args)
        except Exception as exc:
            failure = {
                "schema": AUDIT_SCHEMA,
                "status": "failed",
                "generated_utc": datetime.now(timezone.utc).isoformat(),
                "error_type": type(exc).__name__,
                "error": str(exc),
                "simulations_launched": False,
            }
            failure_path = Path(args.output_dir).resolve() / (
                "two_config_production_acceptance.json"
            )
            _write_json(failure_path, failure)
            raise
    print(json.dumps(payload, indent=2, default=_json_default), flush=True)


if __name__ == "__main__":
    main()
