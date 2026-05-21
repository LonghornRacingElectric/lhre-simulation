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


def mf(slip: np.ndarray, b: float, c: float, d: float, e: float) -> np.ndarray:
    return d * np.sin(c * np.arctan(b * slip - e * (b * slip - np.arctan(b * slip))))


def lateral_curve(t: dict[str, float], fz: float, alpha: np.ndarray) -> np.ndarray:
    dfz = (fz - t["FNOMIN"]) / t["FNOMIN"]
    c = t["PCY1"]
    d = abs(t["PDY1"] + t["PDY2"] * dfz) * fz
    k = abs(t["PKY1"] * t["FNOMIN"] * math.sin(2.0 * math.atan(fz / (t["PKY2"] * t["FNOMIN"]))))
    b = k / max(c * d, 1.0)
    e = t["PEY1"] + t["PEY2"] * dfz
    return mf(alpha, b, c, d, e)


def longitudinal_curve(t: dict[str, float], fz: float, kappa: np.ndarray) -> np.ndarray:
    dfz = (fz - t["FNOMIN"]) / t["FNOMIN"]
    c = t["PCX1"]
    d = max(t["PDX1"] + t["PDX2"] * dfz, 0.8) * fz
    stiffness = fz * (t["PKX1"] + t["PKX2"] * dfz) * math.exp(t["PKX3"] * dfz)
    b = abs(stiffness) / max(c * d, 1.0)
    e = t["PEX1"] + t["PEX2"] * dfz + t["PEX3"] * dfz * dfz
    return mf(kappa, b, c, d, e)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    t = parse_tir(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"]))
    loads = [350.0, 650.0, 1000.0, 1400.0]
    alpha = np.deg2rad(np.linspace(-15.0, 15.0, 181))
    kappa = np.linspace(-0.15, 0.15, 181)
    lat_rows: list[dict[str, Any]] = []
    long_rows: list[dict[str, Any]] = []
    fig_lat, ax_lat = plt.subplots(figsize=(7.2, 4.8))
    fig_long, ax_long = plt.subplots(figsize=(7.2, 4.8))
    for fz in loads:
        fy = lateral_curve(t, fz, alpha)
        fx = longitudinal_curve(t, fz, kappa)
        ax_lat.plot(np.rad2deg(alpha), fy, label=f"{fz:.0f} N")
        ax_long.plot(kappa, fx, label=f"{fz:.0f} N")
        for a, y in zip(alpha, fy):
            lat_rows.append({"fz_n": fz, "alpha_deg": math.degrees(float(a)), "fy_n": float(y)})
        for k, x in zip(kappa, fx):
            long_rows.append({"fz_n": fz, "kappa": float(k), "fx_n": float(x)})
    ax_lat.set_xlabel("Slip angle [deg]")
    ax_lat.set_ylabel("Lateral force [N]")
    ax_lat.set_title("VDYN-007 Pure Lateral Curves")
    ax_lat.grid(True, alpha=0.28)
    ax_lat.legend(title="Fz")
    ax_long.set_xlabel("Longitudinal slip [-]")
    ax_long.set_ylabel("Longitudinal force [N]")
    ax_long.set_title("VDYN-007 Pure Longitudinal Curves")
    ax_long.grid(True, alpha=0.28)
    ax_long.legend(title="Fz")
    plots = STUDY_DIR / cfg["outputs"]["figures_dir"]
    plots.mkdir(parents=True, exist_ok=True)
    fig_lat.tight_layout()
    fig_long.tight_layout()
    fig_lat.savefig(plots / "pure_lateral_curves.png", dpi=220)
    fig_long.savefig(plots / "pure_longitudinal_curves.png", dpi=220)
    plt.close(fig_lat)
    plt.close(fig_long)
    outputs = STUDY_DIR / cfg["outputs"]["data_dir"]
    write_csv(outputs / "pure_lateral_curves.csv", lat_rows)
    write_csv(outputs / "pure_longitudinal_curves.csv", long_rows)
    lines = [
        "# VDYN-007 Results",
        "",
        "## Finding",
        "",
        "**PASS:** pure-slip screening curves were generated at representative tire loads.",
        "",
        "These curves are tire-file screening artifacts, not track-correlated final tire behavior.",
        "",
        "## Representative Loads",
        "",
        "- `350 N`, `650 N`, `1000 N`, `1400 N`",
        "",
        "![Pure lateral curves](plots/pure_lateral_curves.png)",
        "",
        "![Pure longitudinal curves](plots/pure_longitudinal_curves.png)",
        "",
        "## Design Implication",
        "",
        "The tire should be discussed as a curve shape and stiffness source, not only as a peak coefficient. Driver response happens before the tire reaches peak force.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
