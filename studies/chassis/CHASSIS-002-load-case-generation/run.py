from __future__ import annotations

import csv
import math
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import yaml


STUDY_DIR = Path(__file__).resolve().parent
G = 9.80665


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def read_one(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))[0]


def read_metrics(path: Path) -> dict[float, dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return {float(r["speed_mps"]): r for r in csv.DictReader(f)}


def plot(rows: list[dict[str, Any]], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    labels = [r["case"] for r in rows]
    ax.bar(labels, [float(r["resultant_n"]) for r in rows], color="#5f9e6e")
    ax.set_ylabel("Per-corner resultant [N]")
    ax.set_title("CHASSIS-002 Generated Load Cases")
    ax.tick_params(axis="x", rotation=25)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    source = read_one(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vdyn_source_summary"]))
    metrics = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vdyn_envelope_metrics"]))
    aero = read_one(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["aero_summary"]))
    mass = float(source["total_mass_kg"])
    m15 = metrics[15.0]
    rows = [
        {"case": "lateral_15mps", "fx_n": 0.0, "fy_n": mass * G * float(m15["max_lateral_g"]) / 4.0, "fz_n": mass * G / 4.0},
        {"case": "brake_15mps", "fx_n": mass * G * float(m15["max_brake_g"]) / 4.0, "fy_n": 0.0, "fz_n": mass * G / 4.0},
        {"case": "aero_15mps", "fx_n": float(aero["baseline_drag_n"]) / 4.0, "fy_n": 0.0, "fz_n": float(aero["baseline_downforce_n"]) / 4.0},
    ]
    for r in rows:
        r["resultant_n"] = math.sqrt(r["fx_n"] ** 2 + r["fy_n"] ** 2 + r["fz_n"] ** 2)
        r["design_resultant_fos2_n"] = 2.0 * r["resultant_n"]
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "load_cases.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    plot(rows, STUDY_DIR / cfg["outputs"]["figures_dir"] / "load_cases.png")
    max_case = max(rows, key=lambda r: r["resultant_n"])
    lines = [
        "# CHASSIS-002 Results",
        "",
        "## Finding",
        "",
        "**PASS:** first-pass chassis load cases were generated from admitted vehicle behavior studies.",
        "",
        "## Key Metrics",
        "",
        f"- Lateral 15 m/s per-corner resultant: `{rows[0]['resultant_n']:.1f} N`",
        f"- Brake 15 m/s per-corner resultant: `{rows[1]['resultant_n']:.1f} N`",
        f"- Aero 15 m/s per-corner equivalent resultant: `{rows[2]['resultant_n']:.1f} N`",
        f"- Largest generated case: `{max_case['case']}` at `{max_case['resultant_n']:.1f} N`; FOS 2.0 resultant `{max_case['design_resultant_fos2_n']:.1f} N`",
        "",
        "![Load cases](plots/load_cases.png)",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
