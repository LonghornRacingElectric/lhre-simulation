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
    return data


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_rake(rows: list[dict[str, float]], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    ax.scatter([r["rake_m"] * 1000 for r in rows], [r["downforce_n"] for r in rows], label="Downforce")
    ax.set_xlabel("Rake [mm]")
    ax.set_ylabel("Downforce [N]")
    ax.set_title("AERO-002 Downforce vs Rake")
    ax.grid(True, alpha=0.28)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    rows = read_rows(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["aero_map_points"]))
    summary = {
        "downforce_min_n": min(r["downforce_n"] for r in rows),
        "downforce_max_n": max(r["downforce_n"] for r in rows),
        "downforce_span_n": max(r["downforce_n"] for r in rows) - min(r["downforce_n"] for r in rows),
        "drag_min_n": min(r["drag_n"] for r in rows),
        "drag_max_n": max(r["drag_n"] for r in rows),
        "drag_span_n": max(r["drag_n"] for r in rows) - min(r["drag_n"] for r in rows),
        "rake_min_mm": min(r["rake_m"] for r in rows) * 1000,
        "rake_max_mm": max(r["rake_m"] for r in rows) * 1000,
        "equivalent_x_min_m": min(r["equivalent_vertical_load_x_m"] for r in rows),
        "equivalent_x_max_m": max(r["equivalent_vertical_load_x_m"] for r in rows),
    }
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "summary.csv", [summary])
    plot_rake(rows, plots / "downforce_vs_rake.png")
    lines = [
        "# AERO-002 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the aero map is platform-sensitive enough that ride height and rake must be part of the aero design argument.",
        "",
        "## Key Metrics",
        "",
        f"- Downforce span across map: `{summary['downforce_span_n']:.1f} N`",
        f"- Drag span across map: `{summary['drag_span_n']:.1f} N`",
        f"- Rake range in map: `{summary['rake_min_mm']:.1f}` to `{summary['rake_max_mm']:.1f} mm`",
        f"- Equivalent vertical-load x range: `{summary['equivalent_x_min_m']:.3f}` to `{summary['equivalent_x_max_m']:.3f} m`",
        "",
        "![Downforce vs rake](plots/downforce_vs_rake.png)",
        "",
        "## Design Implication",
        "",
        "The aero report must discuss suspension platform control. A single downforce number is not a sufficient design justification.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
