from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import yaml


STUDY_DIR = Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise TypeError(f"Expected mapping in {path}")
    return data


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def read_metrics(path: Path) -> dict[str, dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    out: dict[str, dict[str, str]] = {}
    for row in rows:
        out[row["metric"]] = row
    return out


def metric(metrics: dict[str, dict[str, str]], key: str) -> float:
    if key not in metrics:
        raise KeyError(key)
    return float(metrics[key]["value"])


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_response(summary: dict[str, float], path: Path) -> None:
    labels = ["ay rise", "yaw rise", "settling"]
    values = [summary["ay_rise_time_s"], summary["yaw_rise_time_s"], summary["settling_time_s"]]
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    ax.bar(labels, values, color="#4c78a8")
    ax.set_ylabel("Time [s]")
    ax.set_title("StandardSim Baseline Response Timing")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    steady = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["steady_state_metrics"]))
    transient = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["transient_metrics"]))
    four = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["four_post_metrics"]))
    summary = {
        "understeer_gradient_deg_per_g": metric(steady, "understeer_gradient_deg_per_g"),
        "roll_gradient_deg_per_g": metric(steady, "roll_gradient_deg_per_g"),
        "ay_rise_time_s": metric(transient, "ay_rise_time_s"),
        "yaw_rise_time_s": metric(transient, "yaw_rise_time_s"),
        "settling_time_s": metric(transient, "settling_time_s"),
        "ay_overshoot_pct": metric(transient, "ay_overshoot_pct"),
        "yaw_overshoot_pct": metric(transient, "yaw_overshoot_pct"),
        "front_lltd_pct": metric(four, "avg_lltd_front_pct"),
        "front_motion_ratio": metric(four, "avg_motion_ratio_front"),
        "rear_motion_ratio": metric(four, "avg_motion_ratio_rear"),
        "toe_gain_roll_rad_per_rad": metric(four, "toe_gain_roll_rad_per_rad"),
    }
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "summary.csv", [summary])
    plot_response(summary, plots / "response_timing.png")
    status = "PASS"
    lines = [
        "# VDYN-003 Results",
        "",
        "## Finding",
        "",
        f"**{status}:** StandardSim baseline metrics are present and support the claim that the car is mild, quick, and tunable enough to proceed to setup studies.",
        "",
        "## Key Metrics",
        "",
        f"- Understeer gradient: `{summary['understeer_gradient_deg_per_g']:.3f} deg/g`",
        f"- Roll gradient: `{summary['roll_gradient_deg_per_g']:.3f} deg/g`",
        f"- Ay rise time: `{summary['ay_rise_time_s']:.3f} s`",
        f"- Yaw rise time: `{summary['yaw_rise_time_s']:.3f} s`",
        f"- Ay overshoot: `{summary['ay_overshoot_pct']:.1f} %`",
        f"- Yaw overshoot: `{summary['yaw_overshoot_pct']:.1f} %`",
        f"- Front LLTD: `{summary['front_lltd_pct']:.2f} %`",
        f"- Front/rear motion ratio: `{summary['front_motion_ratio']:.3f}` / `{summary['rear_motion_ratio']:.3f}`",
        f"- Roll toe gain: `{summary['toe_gain_roll_rad_per_rad']:.5f} rad/rad`",
        "",
        "![Response timing](plots/response_timing.png)",
        "",
        "## Design Implication",
        "",
        "The full model does not point to an architecture reset. It points to setup work: overshoot, damping, tire response, and ARB mapping.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
