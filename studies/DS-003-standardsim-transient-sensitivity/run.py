#!/usr/bin/env python3
"""Run DS-003: StandardSim TransientEval sensitivity using DS-002 builds."""

from __future__ import annotations

import argparse
import csv
import dataclasses
import importlib.util
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
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
STUDY_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = STUDY_DIR / "outputs"
PLOT_DIR = STUDY_DIR / "plots"
REPORT_PATH = REPO_ROOT / "reports" / "DS-003-standardsim-transient-sensitivity.md"

DS002_DIR = REPO_ROOT / "studies" / "DS-002-standardsim-steady-state-sensitivity"
DS002_RUN = DS002_DIR / "run.py"
DS002_CASE_MANIFEST = DS002_DIR / "outputs" / "case_manifest.csv"
DS002_PARAMETER_REGISTRY = DS002_DIR / "outputs" / "parameter_registry.csv"

BOBSIM_ROOT = REPO_ROOT / "BobSim"

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))


def require_dependencies() -> tuple[Any, Any, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        import yaml
    except ModuleNotFoundError as exc:
        missing = exc.name or "a required package"
        raise SystemExit(
            f"Missing Python dependency: {missing}\n"
            "Install the study dependencies in the external environment:\n"
            "  /tmp/lhre-sim-venv/bin/python -m pip install PyYAML numpy matplotlib pandas scipy"
        ) from exc

    return np, plt, yaml


np, plt, yaml = require_dependencies()

sys.path.insert(0, str(BOBSIM_ROOT))

from _3_StandardSim.TransientEval.transient_eval_sim import TransientEvalSim  # noqa: E402


def load_ds002_module() -> Any:
    spec = importlib.util.spec_from_file_location("ds002_run", DS002_RUN)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load DS-002 helper module: {DS002_RUN}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[str(spec.name)] = module
    spec.loader.exec_module(module)
    return module


DS002 = load_ds002_module()


@dataclass(frozen=True)
class ParameterRow:
    name: str
    label: str
    unit: str
    baseline: float
    low: float
    high: float
    group: str
    description: str


@dataclass
class TransientCaseResult:
    case_id: str
    parameter: str
    level: str
    value: float | str
    variant_dir: Path
    status: str
    metrics: dict[str, float]
    error: str = ""
    elapsed_s: float = 0.0


IMPORTANT_TRANSIENT_RESPONSE_METRICS = (
    "step.ay_peak",
    "step.ay_gain_dc",
    "step.yaw_peak",
    "step.yaw_gain_dc",
    "step.roll_peak",
    "step.roll_gain_dc",
    "step.settling_time_s",
    "frequency.ay_gain_peak",
    "frequency.yaw_gain_peak",
    "frequency.ay_phase_1hz",
    "frequency.yaw_phase_1hz",
    "frequency.ay_lag_1hz",
    "frequency.yaw_lag_1hz",
    "frequency.yaw_to_ay_lag_1hz",
    "frequency.gain_variation_pct",
)


def as_repo_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def format_float(value: float, digits: int = 4) -> str:
    if not math.isfinite(float(value)):
        return "nan"
    return f"{float(value):.{digits}f}"


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def table_line(values: list[str]) -> str:
    return "| " + " | ".join(values) + " |"


def plot_filename_for_metric(metric: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", metric).strip("_").lower()
    return f"{slug or 'response'}.png"


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def read_parameter_rows(path: Path) -> list[ParameterRow]:
    rows = []
    for row in read_csv_rows(path):
        rows.append(
            ParameterRow(
                name=row["name"],
                label=row["label"],
                unit=row["unit"],
                baseline=float(row["baseline"]),
                low=float(row["low"]),
                high=float(row["high"]),
                group=row["group"],
                description=row["description"],
            )
        )
    return rows


def to_ds002_spec(row: ParameterRow) -> Any:
    return DS002.ParameterSpec(
        name=row.name,
        label=row.label,
        unit=row.unit,
        baseline=row.baseline,
        low=row.low,
        high=row.high,
        group=row.group,
        description=row.description,
        apply=lambda text, value: text,
    )


def read_case_manifest(path: Path, specs: list[ParameterRow]) -> list[dict[str, Any]]:
    spec_names = {spec.name for spec in specs}
    cases: list[dict[str, Any]] = []
    for row in read_csv_rows(path):
        level = row["level"]
        parameter = row["parameter"]
        if level not in {"baseline", "low", "high"}:
            continue
        if level != "baseline" and parameter not in spec_names:
            continue
        if row["status"] != "ok":
            continue
        cases.append(
            {
                "case_id": row["case_id"],
                "parameter": parameter,
                "level": level,
                "value": row["value"],
                "variant_dir": REPO_ROOT / row["variant_dir"],
            }
        )
    return cases


def transient_config(variant_dir: Path, metrics_dir: Path) -> dict[str, Any]:
    build_dir = variant_dir / "build" / "SteadyStateEval"
    return {
        "standard": "TransientEval",
        "simulation": {
            "backend": "modelica",
            "build_dir": str(build_dir),
            "exec_name": "BobLib.Standards.VehicleSim",
            "start_time": 0.0,
            "solver": "dassl",
            "output_format": "csv",
            "log_level": "LOG_STATS",
            "no_grid": True,
            "no_event_emit": True,
        },
        "execution": {
            "parallel": False,
            "max_workers": 1,
            "cleanup": True,
            "stream_logs": False,
        },
        "test": {
            "testVel": [15.0, 20.0],
            "stepTime": 1.0,
            "run_step": True,
            "run_continuous_sine": True,
            "directions": ["left"],
            "representative_step_deg": 5.0,
            "representative_step_direction": "left",
            "steerStep_deg": [5.0],
            "stepDuration": 0.02,
            "representative_cont_freq_hz": 1.0,
            "representative_cont_amp_deg": 5.0,
            "representative_cont_direction": "left",
            "sweep_freq_hz": [0.5, 0.75, 1.0],
            "sweep_amp_deg": [5.0],
            "n_cycles": 4,
            "analyze_cycles_after": 1,
            "freq_response_amp_deg": 5.0,
            "freq_response_direction": "left",
        },
        "report": {
            "enabled": False,
            "brand": "BobSim",
            "title": "TransientEval Lateral Transient Response",
            "subtitle": "Step response and sustained-sine FRF",
            "output_path": str(metrics_dir / "transient_eval_report.pdf"),
            "metric_target_velocity_mps": 15.0,
        },
    }


def metric_key(row: dict[str, str]) -> str:
    return f"{row.get('group', '').strip()}.{row.get('metric', '').strip()}"


def read_transient_metrics_csv(path: Path) -> tuple[dict[str, float], list[dict[str, Any]]]:
    metrics: dict[str, float] = {}
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            key = metric_key(row)
            if key in metrics:
                continue
            raw_value = row.get("value", "")
            try:
                value = float(raw_value) if raw_value != "" else math.nan
            except (TypeError, ValueError):
                value = math.nan
            metrics[key] = value
            rows.append(
                {
                    "metric": key,
                    "source_group": row.get("group", ""),
                    "source_metric": row.get("metric", ""),
                    "value": value,
                    "unit": row.get("units", ""),
                    "description": row.get("description", ""),
                }
            )
    return metrics, rows


def run_transient_case(case: dict[str, Any], *, reuse: bool) -> TransientCaseResult:
    variant_dir = Path(case["variant_dir"])
    metrics_dir = variant_dir / "results" / "TransientEval"
    metrics_path = metrics_dir / "metrics.csv"
    generated_metrics_path = metrics_dir / "transient_eval_report_metrics.csv"
    config_path = metrics_dir / "transient_eval_config.yml"
    log_path = metrics_dir / "run_TransientEval.log"
    started = time.perf_counter()

    try:
        build_dir = variant_dir / "build" / "SteadyStateEval"
        if DS002.find_executable(build_dir, "BobLib.Standards.VehicleSim") is None:
            raise FileNotFoundError(
                f"Missing compiled VehicleSim executable under {as_repo_path(build_dir)}"
            )

        metrics_dir.mkdir(parents=True, exist_ok=True)
        config = transient_config(variant_dir, metrics_dir)
        config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

        if not reuse or not metrics_path.exists():
            stdout_chunks: list[str] = []
            try:
                result = TransientEvalSim(config).run()
                stdout_chunks.append(f"metrics_csv_path={result.get('metrics_csv_path')}\n")
            except Exception as exc:
                stdout_chunks.append(f"error={exc}\n")
                raise
            finally:
                log_path.write_text("".join(stdout_chunks), encoding="utf-8")

            if not generated_metrics_path.exists():
                raise FileNotFoundError(
                    f"TransientEval metrics CSV was not produced: {generated_metrics_path}"
                )
            shutil.copyfile(generated_metrics_path, metrics_path)

        metrics, _rows = read_transient_metrics_csv(metrics_path)
        status = "ok"
        error = ""
    except Exception as exc:  # noqa: BLE001 - keep the sweep moving and log failures.
        metrics = {}
        status = "failed"
        error = str(exc)
        metrics_dir.mkdir(parents=True, exist_ok=True)
        (metrics_dir / "case_error.txt").write_text(error, encoding="utf-8")

    return TransientCaseResult(
        case_id=str(case["case_id"]),
        parameter=str(case["parameter"]),
        level=str(case["level"]),
        value=case["value"],
        variant_dir=variant_dir,
        status=status,
        metrics=metrics,
        error=error,
        elapsed_s=time.perf_counter() - started,
    )


def finite_span_pct(high_value: float, low_value: float, baseline: float) -> float:
    if (
        not math.isfinite(high_value)
        or not math.isfinite(low_value)
        or not math.isfinite(baseline)
        or abs(baseline) < 1e-12
    ):
        return math.nan
    return 100.0 * (high_value - low_value) / abs(baseline)


def finite_pct_delta(value: float, baseline: float) -> float:
    if not math.isfinite(value) or not math.isfinite(baseline) or abs(baseline) < 1e-12:
        return math.nan
    return 100.0 * (value - baseline) / abs(baseline)


def local_sensitivity_rows(
    specs: list[ParameterRow],
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


def is_active_report_metric(metric: str) -> bool:
    return metric in IMPORTANT_TRANSIENT_RESPONSE_METRICS


def ignored_metric_reason(metric: str) -> str:
    if metric.startswith("general."):
        return "run metadata, not a transient design response"
    if metric.startswith("quality."):
        return "fit quality diagnostic, not a design response"
    if metric.startswith("trend."):
        return "velocity-slope diagnostic excluded from first-pass plots"
    if metric.endswith("_rise_time_s") or metric.endswith("_peak_response_time_s"):
        return "threshold timing metric can become sign-sensitive in some variants; retained as diagnostic"
    if metric.endswith("_overshoot_pct") or metric.endswith("_overshoot_rad") or metric.endswith("_overshoot_rad_per_s"):
        return "overshoot metric can blow up when steady-state response is small; retained as diagnostic"
    if metric == "frequency.bandwidth_hz":
        return "coarse frequency-grid locator; constant in this first TransientEval sweep"
    if metric.endswith("_freq"):
        return "frequency locator retained for diagnostics; response magnitude/phase/lag is active"
    return "excluded from current findings/plots"


def plot_baseline_metrics(metrics: dict[str, float], units: dict[str, str]) -> None:
    labels = [name for name in IMPORTANT_TRANSIENT_RESPONSE_METRICS if name in metrics]
    values = [metrics[name] for name in labels]

    fig, ax = plt.subplots(figsize=(12.0, 6.2))
    ax.bar(range(len(labels)), values, color="#355c7d")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=55, ha="right", fontsize=8)
    ax.set_ylabel("Metric value")
    ax.set_title("DS-003 Baseline StandardSim TransientEval Metrics")
    ax.grid(True, axis="y", linestyle="--", alpha=0.3)
    for idx, name in enumerate(labels):
        ax.text(
            idx,
            values[idx],
            units.get(name, ""),
            ha="center",
            va="bottom" if values[idx] >= 0 else "top",
            fontsize=7,
        )
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "baseline_metrics.png", dpi=220)
    plt.close(fig)


def plot_local_sensitivity_heatmap(
    specs: list[ParameterRow],
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
    DS002.plot_heatmap(
        matrix,
        [spec.label for spec in specs],
        metric_names,
        "Local StandardSim TransientEval Response Sensitivity",
        "Signed response span from low to high (% of baseline)",
        PLOT_DIR / "response_sensitivity_heatmap.png",
    )


def plot_top_local_sensitivities(
    sensitivity_rows: list[dict[str, Any]],
    active_metric_names: list[str],
) -> None:
    active_metrics = set(active_metric_names)
    rows = [
        row
        for row in sensitivity_rows
        if str(row["metric"]) in active_metrics
        and math.isfinite(float(row["abs_effect_pct_span"]))
    ]
    rows = sorted(
        rows,
        key=lambda row: float(row["abs_effect_pct_span"]),
        reverse=True,
    )[:28]
    labels = [f"{row['parameter_label']} -> {row['metric']}" for row in rows][::-1]
    values = [float(row["signed_effect_pct_span"]) for row in rows][::-1]
    colors = ["#247ba0" if value >= 0.0 else "#d1495b" for value in values]

    fig, ax = plt.subplots(figsize=(12.5, 9.0))
    ax.barh(labels, values, color=colors)
    ax.axvline(0.0, color="0.2", linewidth=0.8)
    ax.set_xlabel("Signed response span from low to high (% of baseline)")
    ax.set_title("Largest Local StandardSim TransientEval Sensitivities")
    ax.grid(True, axis="x", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "top_local_sensitivities.png", dpi=220)
    plt.close(fig)


def plot_per_response_sensitivities(
    sensitivity_rows: list[dict[str, Any]],
    active_metric_names: list[str],
    units: dict[str, str],
) -> None:
    response_dir = PLOT_DIR / "responses"
    response_dir.mkdir(parents=True, exist_ok=True)
    for stale_plot in response_dir.glob("*.png"):
        stale_plot.unlink()

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
        fig.savefig(response_dir / plot_filename_for_metric(metric), dpi=220)
        plt.close(fig)


def write_report(
    *,
    specs: list[ParameterRow],
    metric_names: list[str],
    active_metric_names: list[str],
    baseline_metrics: dict[str, float],
    units: dict[str, str],
    sensitivity_rows: list[dict[str, Any]],
    case_results: list[TransientCaseResult],
    started_at: str,
    elapsed_s: float,
) -> None:
    ignored_metrics = [
        metric for metric in metric_names if metric not in set(active_metric_names)
    ]
    top_summary_rows = []
    for metric in active_metric_names:
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
    lines.append("# DS-003 StandardSim TransientEval Sensitivity")
    lines.append("")
    lines.append(f"Generated UTC: {started_at}")
    lines.append("")
    lines.append("## Source of Results")
    lines.append("")
    lines.append(
        "All response metrics in this report are generated by BobSim StandardSim "
        "using `BobSim/_3_StandardSim/TransientEval/transient_eval_sim.py`."
    )
    lines.append("")
    lines.append(
        "This study reuses the compiled DS-002 `VehicleSim` population under "
        "`studies/DS-002-standardsim-steady-state-sensitivity/work/population/`."
    )
    lines.append("")
    lines.append("## Baseline")
    lines.append("")
    lines.append(table_line(["Item", "Value"]))
    lines.append(table_line(["---", "---"]))
    lines.append(table_line(["Model", "BobLib.Standards.VehicleSim"]))
    lines.append(table_line(["Standard", "TransientEval"]))
    lines.append(table_line(["Parameters swept", str(len(specs))]))
    lines.append(table_line(["Successful cases", f"{len(ok_cases)} / {len(case_results)}"]))
    lines.append(table_line(["Active response metrics", str(len(active_metric_names))]))
    lines.append(table_line(["Ignored response metrics", str(len(ignored_metrics))]))
    lines.append("")
    lines.append("## Baseline Response Metrics")
    lines.append("")
    lines.append(table_line(["Metric", "Value", "Units"]))
    lines.append(table_line(["---", "---:", "---"]))
    for metric in active_metric_names:
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
        "The complete per-response transient sensitivity matrix is in "
        "`outputs/metric_sensitivity_matrix.csv`."
    )
    if ignored_metrics:
        lines.append("")
        lines.append(
            "Diagnostic, quality, locator, and velocity-slope metrics are retained "
            "in CSV outputs but ignored in the current findings and plots."
        )
    lines.append("")
    if failed_cases:
        lines.append("## Failed Cases")
        lines.append("")
        lines.append(table_line(["Case", "Parameter", "Level", "Error"]))
        lines.append(table_line(["---", "---", "---", "---"]))
        for case in failed_cases[:20]:
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
        "outputs/metric_catalog.csv",
        "outputs/ignored_metrics.csv",
        "outputs/transient_cases.csv",
        "outputs/metric_sensitivity_matrix.csv",
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
    lines.append(table_line(["DS-002 source", as_repo_path(DS002_DIR)]))
    lines.append(table_line(["Python", sys.executable]))
    lines.append("")

    text = "\n".join(lines)
    (STUDY_DIR / "RESULTS.md").write_text(text, encoding="utf-8")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="Reuse existing TransientEval metrics under DS-002 variant results.",
    )
    parser.add_argument(
        "--limit-cases",
        type=int,
        default=0,
        help="Limit number of manifest cases for smoke testing. 0 means all cases.",
    )
    args = parser.parse_args()

    if shutil.which("omc") is None:
        raise SystemExit("OpenModelica `omc` was not found on PATH.")
    if not DS002_CASE_MANIFEST.exists():
        raise SystemExit(
            f"Missing DS-002 case manifest: {as_repo_path(DS002_CASE_MANIFEST)}"
        )
    if not DS002_PARAMETER_REGISTRY.exists():
        raise SystemExit(
            f"Missing DS-002 parameter registry: {as_repo_path(DS002_PARAMETER_REGISTRY)}"
        )

    started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    start_time = time.perf_counter()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    specs = read_parameter_rows(DS002_PARAMETER_REGISTRY)
    cases = read_case_manifest(DS002_CASE_MANIFEST, specs)
    if args.limit_cases:
        cases = cases[: args.limit_cases]

    print("DS-003 StandardSim TransientEval sensitivity", flush=True)
    print(f"  Source study: {DS002_DIR.relative_to(REPO_ROOT)}", flush=True)
    print(f"  Cases: {len(cases)}", flush=True)
    print(f"  Reuse: {args.reuse}", flush=True)

    case_results: list[TransientCaseResult] = []
    for idx, case in enumerate(cases, start=1):
        print(f"  TransientEval {idx:02d}/{len(cases)}: {case['case_id']}", flush=True)
        result = run_transient_case(case, reuse=args.reuse)
        case_results.append(result)

    baseline_case = next(
        (case for case in case_results if case.level == "baseline" and case.status == "ok"),
        None,
    )
    if baseline_case is None:
        first_error = next((case.error for case in case_results if case.error), "unknown error")
        raise SystemExit(f"Baseline TransientEval case failed: {first_error}")

    baseline_metrics = baseline_case.metrics
    _baseline_metrics, baseline_metric_rows = read_transient_metrics_csv(
        baseline_case.variant_dir / "results" / "TransientEval" / "metrics.csv"
    )
    metric_names = list(baseline_metrics.keys())
    units = {row["metric"]: row["unit"] for row in baseline_metric_rows}
    descriptions = {row["metric"]: row["description"] for row in baseline_metric_rows}
    source_groups = {row["metric"]: row["source_group"] for row in baseline_metric_rows}
    source_metrics = {row["metric"]: row["source_metric"] for row in baseline_metric_rows}

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
                "source_group": source_groups.get(metric, ""),
                "source_metric": source_metrics.get(metric, ""),
                "unit": units.get(metric, ""),
                "description": descriptions.get(metric, ""),
                "active_in_current_report": int(metric in active_metric_names),
            }
            for metric in metric_names
        ],
        [
            "metric",
            "source_group",
            "source_metric",
            "unit",
            "description",
            "active_in_current_report",
        ],
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

    transient_rows: list[dict[str, Any]] = []
    low_by_parameter: dict[str, dict[str, float]] = {}
    high_by_parameter: dict[str, dict[str, float]] = {}

    for result in case_results:
        if result.status == "ok" and result.level == "low":
            low_by_parameter[result.parameter] = result.metrics
        elif result.status == "ok" and result.level == "high":
            high_by_parameter[result.parameter] = result.metrics

        row = {
            "case_id": result.case_id,
            "parameter": result.parameter,
            "level": result.level,
            "value": result.value,
            "status": result.status,
        }
        row.update({metric: result.metrics.get(metric, math.nan) for metric in metric_names})
        transient_rows.append(row)

    sensitivity = local_sensitivity_rows(
        specs,
        metric_names,
        baseline_metrics,
        low_by_parameter,
        high_by_parameter,
    )

    write_csv(
        OUTPUT_DIR / "transient_cases.csv",
        transient_rows,
        ["case_id", "parameter", "level", "value", "status"] + metric_names,
    )
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

    write_csv(
        OUTPUT_DIR / "case_manifest.csv",
        [
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
        ],
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
            {"item": "engine", "value": "BobSim StandardSim TransientEval"},
            {"item": "engine_path", "value": "BobSim/_3_StandardSim/TransientEval/transient_eval_sim.py"},
            {"item": "model", "value": "BobLib.Standards.VehicleSim"},
            {"item": "source_study", "value": "DS-002"},
            {"item": "source_case_manifest", "value": as_repo_path(DS002_CASE_MANIFEST)},
            {"item": "parameter_count", "value": len(specs)},
            {"item": "response_metric_count", "value": len(metric_names)},
            {"item": "active_response_metric_count", "value": len(active_metric_names)},
            {"item": "case_count", "value": len(case_results)},
            {"item": "elapsed_seconds", "value": f"{elapsed_s:.3f}"},
        ],
        ["item", "value"],
    )

    write_report(
        specs=specs,
        metric_names=metric_names,
        active_metric_names=active_metric_names,
        baseline_metrics=baseline_metrics,
        units=units,
        sensitivity_rows=sensitivity,
        case_results=case_results,
        started_at=started_at,
        elapsed_s=elapsed_s,
    )

    print(f"Complete in {elapsed_s:.1f} s", flush=True)
    print(f"Study report: {STUDY_DIR / 'RESULTS.md'}", flush=True)
    print(f"Top-level report: {REPORT_PATH}", flush=True)


if __name__ == "__main__":
    main()
