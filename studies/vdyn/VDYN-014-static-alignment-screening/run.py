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


def cornering_stiffness(t: dict[str, float], fz: float, gamma: float) -> float:
    return abs(t["PKY1"] * t["FNOMIN"] * math.sin(2.0 * math.atan(fz / (t["PKY2"] * t["FNOMIN"]))) * (1.0 - t["PKY3"] * abs(gamma)))


def lateral_peak_mu(t: dict[str, float], fz: float, gamma: float) -> float:
    dfz = (fz - t["FNOMIN"]) / t["FNOMIN"]
    return abs((t["PDY1"] + t["PDY2"] * dfz) * (1.0 - t["PDY3"] * gamma * gamma))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    t = parse_tir(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"]))
    fz = t["FNOMIN"]
    camber_deg_values = np.linspace(0.0, 4.0, 41)
    rows = [
        {
            "camber_deg": float(deg),
            "camber_rad": math.radians(float(deg)),
            "cornering_stiffness_n_per_rad": cornering_stiffness(t, fz, math.radians(float(deg))),
            "lateral_peak_mu": lateral_peak_mu(t, fz, math.radians(float(deg))),
        }
        for deg in camber_deg_values
    ]
    toe_rows = [
        {
            "total_toe_deg": toe,
            "per_wheel_toe_deg": toe / 2.0,
            "slip_preload_rad": math.radians(toe / 2.0),
            "approx_lateral_preload_n_per_front_pair": 2.0 * cornering_stiffness(t, fz, 0.0) * math.radians(toe / 2.0),
        }
        for toe in [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]
    ]
    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "camber_screening.csv", rows)
    write_csv(outputs / "toe_screening.csv", toe_rows)
    fig, ax1 = plt.subplots(figsize=(7.4, 4.8))
    ax1.plot([r["camber_deg"] for r in rows], [r["cornering_stiffness_n_per_rad"] for r in rows], label="Cornering stiffness")
    ax1.set_xlabel("Static camber magnitude [deg]")
    ax1.set_ylabel("Cornering stiffness [N/rad]")
    ax2 = ax1.twinx()
    ax2.plot([r["camber_deg"] for r in rows], [r["lateral_peak_mu"] for r in rows], color="#f58518", label="Peak mu_y")
    ax2.set_ylabel("Peak mu_y [-]")
    ax1.set_title("VDYN-014 Static Camber Screening")
    ax1.grid(True, alpha=0.28)
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [line.get_label() for line in lines], loc="best")
    fig.tight_layout()
    plots.mkdir(parents=True, exist_ok=True)
    fig.savefig(plots / "camber_screening.png", dpi=220)
    plt.close(fig)
    c0 = rows[0]
    c4 = rows[-1]
    toe2 = toe_rows[-1]
    lines_out = [
        "# VDYN-014 Results",
        "",
        "## Finding",
        "",
        "**PASS:** static alignment has been screened as tire-response variables before full StandardSim alignment sweeps.",
        "",
        "## Key Metrics",
        "",
        f"- Cornering stiffness at 0 deg camber: `{c0['cornering_stiffness_n_per_rad']:.0f} N/rad`",
        f"- Cornering stiffness at 4 deg camber magnitude: `{c4['cornering_stiffness_n_per_rad']:.0f} N/rad`",
        f"- Peak mu_y at 0/4 deg camber: `{c0['lateral_peak_mu']:.3f}` / `{c4['lateral_peak_mu']:.3f}`",
        f"- Approx front-pair lateral preload for 2 deg total toe: `{toe2['approx_lateral_preload_n_per_front_pair']:.0f} N`",
        "",
        "![Camber screening](plots/camber_screening.png)",
        "",
        "## Design Implication",
        "",
        "Static alignment should be correlated through tire temperatures, steering response, yaw/ay gain, and scrub/drag observations. A full StandardSim alignment DOE remains a required follow-up.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines_out) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
