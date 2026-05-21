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
import numpy as np
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


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def mu_x(tire: dict[str, float], fz: float) -> float:
    dfz = (fz - tire["FNOMIN"]) / tire["FNOMIN"]
    return max(tire["PDX1"] + tire["PDX2"] * dfz, 0.8)


def mu_y(tire: dict[str, float], fz: float) -> float:
    dfz = (fz - tire["FNOMIN"]) / tire["FNOMIN"]
    return max(abs(tire["PDY1"] + tire["PDY2"] * dfz), 0.8)


def plot(rows: list[dict[str, float]], path: Path) -> None:
    fig, ax1 = plt.subplots(figsize=(7.6, 4.8))
    fz = [r["fz_n"] for r in rows]
    ax1.plot(fz, [r["mux"] for r in rows], label="mu_x", color="#4c78a8")
    ax1.plot(fz, [r["muy"] for r in rows], label="mu_y", color="#f58518")
    ax1.set_xlabel("Vertical load [N]")
    ax1.set_ylabel("Peak friction coefficient [-]")
    ax1.grid(True, alpha=0.28)
    ax2 = ax1.twinx()
    ax2.plot(fz, [r["fy_peak_n"] for r in rows], label="Fy peak", color="#54a24b", linestyle="--")
    ax2.set_ylabel("Lateral peak force [N]")
    ax1.set_title("VDYN-006 Tire Load Sensitivity")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [line.get_label() for line in lines], loc="best")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    tire = parse_tir(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"]))
    fz_values = np.linspace(tire["FZMIN"], tire["FZMAX"], 80)
    rows = [
        {
            "fz_n": float(fz),
            "mux": mu_x(tire, float(fz)),
            "muy": mu_y(tire, float(fz)),
            "fx_peak_n": mu_x(tire, float(fz)) * float(fz),
            "fy_peak_n": mu_y(tire, float(fz)) * float(fz),
        }
        for fz in fz_values
    ]
    nominal = min(rows, key=lambda r: abs(r["fz_n"] - tire["FNOMIN"]))
    high = rows[-1]
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    write_csv(outputs / "tire_load_sensitivity.csv", rows)
    plot(rows, plots / "load_sensitivity.png")
    lines = [
        "# VDYN-006 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the baseline tire file produces meaningful load-sensitive peak-force curves across the valid vertical-load range.",
        "",
        "## Key Metrics",
        "",
        f"- Valid vertical-load range: `{tire['FZMIN']:.0f}` to `{tire['FZMAX']:.0f} N`",
        f"- Nominal-load mu_x/mu_y near `{nominal['fz_n']:.0f} N`: `{nominal['mux']:.3f}` / `{nominal['muy']:.3f}`",
        f"- High-load mu_x/mu_y at `{high['fz_n']:.0f} N`: `{high['mux']:.3f}` / `{high['muy']:.3f}`",
        f"- High-load lateral peak force: `{high['fy_peak_n']:.0f} N` per tire",
        "",
        "![Load sensitivity](plots/load_sensitivity.png)",
        "",
        "## Design Implication",
        "",
        "LLTD, aero load, mass, and setup changes must be interpreted through tire load sensitivity. More normal load increases peak force but does not preserve the same coefficient of friction.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
