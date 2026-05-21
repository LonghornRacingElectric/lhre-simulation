from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import yaml


STUDY_DIR = Path(__file__).resolve().parent


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def read_one(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))[0]


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    aero = read_one(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["aero_summary"]))
    vdyn_rows_path = resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vdyn_metrics"])
    with vdyn_rows_path.open("r", encoding="utf-8", newline="") as f:
        vdyn = {float(r["speed_mps"]): r for r in csv.DictReader(f)}
    drag_n = float(aero["baseline_drag_n"])
    speed = 15.0
    drag_power_kw = drag_n * speed / 1000.0
    m15 = vdyn[15.0]
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["baseline_drag_power_kw", "baseline_downforce_n", "baseline_lateral_g"])
        writer.writeheader()
        writer.writerow({
            "baseline_drag_power_kw": drag_power_kw,
            "baseline_downforce_n": aero["baseline_downforce_n"],
            "baseline_lateral_g": m15["max_lateral_g"],
        })
    lines = [
        "# AERO-003 Results",
        "",
        "## Finding",
        "",
        "**PASS:** baseline aero can be tied to vehicle-level force, drag power, and envelope context.",
        "",
        "## Key Metrics",
        "",
        f"- Baseline downforce at 15 m/s: `{float(aero['baseline_downforce_n']):.1f} N`",
        f"- Baseline drag at 15 m/s: `{drag_n:.1f} N`",
        f"- Drag power at 15 m/s: `{drag_power_kw:.2f} kW`",
        f"- Baseline 15 m/s lateral envelope context: `{float(m15['max_lateral_g']):.3f} g`",
        "",
        "## Design Implication",
        "",
        "Aero claims should be presented with platform control, drag power, vehicle envelope, and validation channels. The next closure is coastdown plus aero-on/off testing.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
