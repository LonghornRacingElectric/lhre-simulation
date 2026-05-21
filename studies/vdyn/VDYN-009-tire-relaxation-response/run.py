from __future__ import annotations

import csv
import math
import os
import re
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
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def parse_tir(path: Path) -> dict[str, float]:
    vals: dict[str, float] = {}
    pat = re.compile(r"^([A-Za-z0-9_]+)\s*=\s*([-+0-9.Ee]+)")
    for raw in path.read_text(encoding="utf-8").splitlines():
        m = pat.match(raw.split("$", 1)[0].strip())
        if m:
            vals[m.group(1).upper()] = float(m.group(2))
    return vals


def read_metrics(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return {row["metric"]: float(row["value"]) for row in csv.DictReader(f)}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    tire = parse_tir(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"]))
    transient = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["standardsim_transient_metrics"]))
    sigma_nominal = tire["PTY1"] * tire["UNLOADED_RADIUS"]
    speeds = [5.0, 10.0, 15.0, 20.0, 25.0]
    rows = [
        {
            "speed_mps": speed,
            "relaxation_length_m": sigma_nominal,
            "time_constant_s": sigma_nominal / speed,
            "approx_95pct_distance_m": 3.0 * sigma_nominal,
            "approx_95pct_time_s": 3.0 * sigma_nominal / speed,
        }
        for speed in speeds
    ]
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "relaxation_response.csv", rows)

    plots.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    ax.plot([r["speed_mps"] for r in rows], [r["time_constant_s"] for r in rows], marker="o", label="1 tau")
    ax.plot([r["speed_mps"] for r in rows], [r["approx_95pct_time_s"] for r in rows], marker="o", label="~95%")
    ax.set_xlabel("Speed [m/s]")
    ax.set_ylabel("Response time [s]")
    ax.set_title("VDYN-009 Relaxation Time vs Speed")
    ax.grid(True, alpha=0.28)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots / "relaxation_time_vs_speed.png", dpi=220)
    plt.close(fig)

    r15 = next(r for r in rows if r["speed_mps"] == 15.0)
    lines = [
        "# VDYN-009 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the fitted lateral relaxation length is now expressed as a transient response scale for correlation.",
        "",
        "## Key Metrics",
        "",
        f"- PTY1 / PTY2: `{tire['PTY1']:.6f}` / `{tire['PTY2']:.6f}`",
        f"- Nominal relaxation length scale `PTY1 * R0`: `{sigma_nominal:.3f} m`",
        f"- 15 m/s relaxation time constant: `{r15['time_constant_s']:.3f} s`",
        f"- 15 m/s approximate 95% force-build time: `{r15['approx_95pct_time_s']:.3f} s`",
        f"- StandardSim ay/yaw rise time for comparison: `{transient['ay_rise_time_s']:.3f}` / `{transient['yaw_rise_time_s']:.3f} s`",
        "",
        "![Relaxation time](plots/relaxation_time_vs_speed.png)",
        "",
        "## Design Implication",
        "",
        "Relaxation length should be correlated with step steer and sine steer phase/lag. It is the bridge between tire data and transient driver confidence.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
