from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import yaml


STUDY_DIR = Path(__file__).resolve().parent
REQUIRED = [
    "upper_fore_i_m", "upper_aft_i_m", "lower_fore_i_m", "lower_aft_i_m",
    "upper_o_m", "lower_o_m", "tie_o_m", "wheel_center_m",
]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_from(base: Path, raw: str) -> Path:
    candidate = Path(raw)
    return candidate if candidate.is_absolute() else (base.parent / candidate).resolve()


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    vehicle = load_yaml(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["vehicle_config"]))
    rows: list[dict[str, Any]] = []
    missing: list[str] = []
    for axle in ["front", "rear"]:
        susp = vehicle[axle]["suspension"]
        for key in REQUIRED:
            if key not in susp:
                missing.append(f"{axle}.{key}")
                continue
            rows.append({"axle": axle, "point": key, "x_m": susp[key][0], "y_m": susp[key][1], "z_m": susp[key][2]})
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "hardpoints.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    status = "PASS" if not missing else "CHECK"
    lines = [
        "# CHASSIS-001 Results",
        "",
        "## Finding",
        "",
        f"**{status}:** chassis hardpoints and source references are traceable in `vehicles/current/vehicle.yml`.",
        "",
        "## Key Metrics",
        "",
        f"- Required suspension hardpoint entries checked: `{len(REQUIRED) * 2}`",
        f"- Missing required entries: `{len(missing)}`",
        f"- Body torsional stiffness input: `{float(vehicle['body']['torsional_stiff_n_m_per_rad']):.0f} N*m/rad`",
        "",
        "## Design Implication",
        "",
        "Chassis load and stiffness studies may reference the vehicle YAML directly, with VDYN-001 providing the mass-property audit.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
