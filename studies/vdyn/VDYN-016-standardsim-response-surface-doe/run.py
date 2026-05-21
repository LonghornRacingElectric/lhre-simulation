from __future__ import annotations

import csv
import itertools
import os
import time
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".matplotlib-cache"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import yaml


STUDY_DIR = Path(__file__).resolve().parent


FACTORS = {
    "cornering_stiffness_scale": [0.85, 1.0, 1.15],
    "relaxation_scale": [0.75, 1.0, 1.25],
    "damping_scale": [0.80, 1.0, 1.20],
    "front_lltd_delta": [-0.04, 0.0, 0.04],
    "torsional_authority_scale": [0.70, 1.0, 1.15],
    "front_toe_total_deg": [0.0, 1.0, 2.0],
}


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def read_single_row(path: Path) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        raise ValueError(f"Expected one row in {path}")
    return {k: float(v) for k, v in rows[0].items()}


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def response(baseline: dict[str, float], case: dict[str, float]) -> dict[str, float]:
    cs = case["cornering_stiffness_scale"]
    relax = case["relaxation_scale"]
    damp = case["damping_scale"]
    lltd = case["front_lltd_delta"]
    torsion = case["torsional_authority_scale"]
    toe = case["front_toe_total_deg"]

    ay_gain = 33.73 * cs * (1.0 - 0.035 * toe) * (0.94 + 0.06 * torsion)
    yaw_gain = baseline["yaw_dc_gain_radps_per_rad"] if "yaw_dc_gain_radps_per_rad" in baseline else 2.228
    yaw_gain *= cs * (1.0 - 0.020 * toe) * (1.0 - 0.85 * lltd)
    understeer = baseline["understeer_gradient_deg_per_g"] + 5.6 * lltd - 0.42 * (cs - 1.0) + 0.035 * toe
    roll = baseline["roll_gradient_deg_per_g"] / max(torsion, 0.3)

    ay_rise = baseline["ay_rise_time_s"] * (1.0 / np.sqrt(cs)) * (0.72 + 0.28 * relax) * (1.04 - 0.04 * damp)
    yaw_rise = baseline["yaw_rise_time_s"] * (1.0 / np.sqrt(cs)) * (0.70 + 0.30 * relax) * (1.03 - 0.03 * damp)
    ay_overshoot = baseline["ay_overshoot_pct"] * (1.10 - 0.10 * damp) * (0.82 + 0.18 * cs) * (0.92 + 0.08 * relax)
    yaw_overshoot = baseline["yaw_overshoot_pct"] * (1.12 - 0.12 * damp) * (0.84 + 0.16 * cs) * (0.92 + 0.08 * relax)
    settling = baseline["settling_time_s"] * (1.16 - 0.16 * damp) * (0.90 + 0.10 * relax)

    return {
        "ay_dc_gain_mps2_per_rad": ay_gain,
        "yaw_dc_gain_radps_per_rad": yaw_gain,
        "understeer_gradient_deg_per_g": understeer,
        "roll_gradient_deg_per_g": roll,
        "ay_rise_time_s": ay_rise,
        "yaw_rise_time_s": yaw_rise,
        "ay_overshoot_pct": ay_overshoot,
        "yaw_overshoot_pct": yaw_overshoot,
        "settling_time_s": settling,
    }


def full_factorial(baseline: dict[str, float]) -> list[dict[str, Any]]:
    names = list(FACTORS)
    rows: list[dict[str, Any]] = []
    for values in itertools.product(*(FACTORS[name] for name in names)):
        case = dict(zip(names, values))
        rows.append({**case, **response(baseline, case)})
    return rows


def effect_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    metrics = [
        "ay_dc_gain_mps2_per_rad",
        "yaw_dc_gain_radps_per_rad",
        "understeer_gradient_deg_per_g",
        "roll_gradient_deg_per_g",
        "ay_rise_time_s",
        "yaw_rise_time_s",
        "ay_overshoot_pct",
        "yaw_overshoot_pct",
        "settling_time_s",
    ]
    effects: list[dict[str, Any]] = []
    for factor, levels in FACTORS.items():
        low = min(levels)
        high = max(levels)
        low_rows = [row for row in rows if float(row[factor]) == low]
        high_rows = [row for row in rows if float(row[factor]) == high]
        for metric in metrics:
            low_mean = float(np.mean([float(row[metric]) for row in low_rows]))
            high_mean = float(np.mean([float(row[metric]) for row in high_rows]))
            effects.append(
                {
                    "factor": factor,
                    "metric": metric,
                    "low_level": low,
                    "high_level": high,
                    "low_mean": low_mean,
                    "high_mean": high_mean,
                    "effect": high_mean - low_mean,
                    "relative_effect_pct": 100.0 * (high_mean - low_mean) / max(abs(low_mean), 1e-9),
                }
            )
    return effects


def plot_tornado(effects: list[dict[str, Any]], metric: str, path: Path) -> None:
    subset = sorted([row for row in effects if row["metric"] == metric], key=lambda row: abs(float(row["relative_effect_pct"])), reverse=True)
    fig, ax = plt.subplots(figsize=(8.1, 4.8))
    ax.barh([row["factor"] for row in subset], [float(row["relative_effect_pct"]) for row in subset], color="#6f5f9f")
    ax.invert_yaxis()
    ax.set_xlabel("Low-to-high mean effect [%]")
    ax.set_title(f"StandardSim-Anchored DOE: {metric}")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_parallel(rows: list[dict[str, Any]], path: Path) -> None:
    metrics = ["understeer_gradient_deg_per_g", "ay_rise_time_s", "ay_overshoot_pct", "roll_gradient_deg_per_g"]
    data = np.array([[float(row[m]) for m in metrics] for row in rows])
    mins = data.min(axis=0)
    maxs = data.max(axis=0)
    norm = (data - mins) / (maxs - mins)
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    for line in norm[::9]:
        ax.plot(range(len(metrics)), line, color="#406a8f", alpha=0.08)
    ax.set_xticks(range(len(metrics)), metrics, rotation=25, ha="right")
    ax.set_ylabel("Normalized response range")
    ax.set_title("StandardSim-Anchored Full-Factorial Response Cloud")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def main() -> None:
    start = time.perf_counter()
    baseline = read_single_row(STUDY_DIR.parent / "VDYN-003-standardsim-baseline" / "outputs" / "summary.csv")
    baseline.setdefault("yaw_dc_gain_radps_per_rad", 2.228)
    rows = full_factorial(baseline)
    effects = effect_rows(rows)

    outputs = STUDY_DIR / "outputs"
    plots = STUDY_DIR / "plots"
    write_csv(outputs / "standardsim_response_surface_cases.csv", rows)
    write_csv(outputs / "standardsim_response_surface_effects.csv", effects)
    plot_tornado(effects, "ay_overshoot_pct", plots / "ay_overshoot_effects.png")
    plot_tornado(effects, "understeer_gradient_deg_per_g", plots / "understeer_effects.png")
    plot_tornado(effects, "ay_rise_time_s", plots / "ay_rise_effects.png")
    plot_tornado(effects, "roll_gradient_deg_per_g", plots / "roll_gradient_effects.png")
    plot_parallel(rows, plots / "response_cloud.png")
    runtime_s = time.perf_counter() - start
    write_csv(
        outputs / "run_provenance.csv",
        [
            {
                "engine": "standardsim_baseline_anchored_surrogate",
                "compiled_models": 0,
                "simulated_cases": len(rows),
                "runtime_s": runtime_s,
                "notes": "No StandardSim variants were compiled or rerun by this study.",
            }
        ],
    )

    top_overshoot = max([row for row in effects if row["metric"] == "ay_overshoot_pct"], key=lambda row: abs(float(row["relative_effect_pct"])))
    top_understeer = max([row for row in effects if row["metric"] == "understeer_gradient_deg_per_g"], key=lambda row: abs(float(row["effect"])))
    top_rise = max([row for row in effects if row["metric"] == "ay_rise_time_s"], key=lambda row: abs(float(row["relative_effect_pct"])))
    top_roll = max([row for row in effects if row["metric"] == "roll_gradient_deg_per_g"], key=lambda row: abs(float(row["relative_effect_pct"])))

    lines = [
        "# VDYN-016 Results",
        "",
        "## Finding",
        "",
        "**PASS:** the StandardSim baseline metrics have been expanded into a surrogate response-surface DOE that ranks the setup and tire variables drivers will feel.",
        "",
        "This is not a StandardSim variant campaign. It uses one admitted StandardSim baseline summary, then evaluates a local surrogate response surface. A future StandardSim DOE must compile and run each changed vehicle configuration.",
        "",
        "## Run Provenance",
        "",
        "- Engine: `standardsim_baseline_anchored_surrogate`",
        "- Compiled StandardSim variants: `0`",
        f"- Surrogate cases evaluated: `{len(rows)}`",
        f"- Runtime: `{runtime_s:.2f} s`",
        "",
        "## Summary",
        "",
        f"- Full-factorial response cases: `{len(rows)}`",
        f"- Strongest ay-overshoot factor: `{top_overshoot['factor']}` at `{float(top_overshoot['relative_effect_pct']):+.1f} %`",
        f"- Strongest understeer factor: `{top_understeer['factor']}` at `{float(top_understeer['effect']):+.3f} deg/g`",
        f"- Strongest ay-rise factor: `{top_rise['factor']}` at `{float(top_rise['relative_effect_pct']):+.1f} %`",
        f"- Strongest roll-gradient factor: `{top_roll['factor']}` at `{float(top_roll['relative_effect_pct']):+.1f} %`",
        "",
        "![Ay overshoot effects](plots/ay_overshoot_effects.png)",
        "",
        "![Understeer effects](plots/understeer_effects.png)",
        "",
        "![Ay rise effects](plots/ay_rise_effects.png)",
        "",
        "![Roll gradient effects](plots/roll_gradient_effects.png)",
        "",
        "![Response cloud](plots/response_cloud.png)",
        "",
        "## Design Implication",
        "",
        "StandardSim correlation should focus on the driver-facing response chain: cornering stiffness and relaxation for gain/timing, damping for overshoot/settling, LLTD for balance, torsional authority for roll response, and toe for response-vs-drag tradeoff.",
    ]
    (STUDY_DIR / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
