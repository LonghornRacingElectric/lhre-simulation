from __future__ import annotations

import csv
import re
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
        return {r["metric"]: float(r["value"]) for r in csv.DictReader(f)}


def main() -> None:
    cfg = load_yaml(STUDY_DIR / "study.yml")
    tire = parse_tir(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["tire_file"]))
    transient = read_metrics(resolve_from(STUDY_DIR / "study.yml", cfg["inputs"]["standardsim_transient_metrics"]))
    relaxation_m = tire["PTY1"] * tire["UNLOADED_RADIUS"]
    rows = [{
        "fnomin_n": tire["FNOMIN"],
        "fzmin_n": tire["FZMIN"],
        "fzmax_n": tire["FZMAX"],
        "pdy1": tire["PDY1"],
        "pdy2": tire["PDY2"],
        "pky1": tire["PKY1"],
        "pky2": tire["PKY2"],
        "pty1": tire["PTY1"],
        "pty2": tire["PTY2"],
        "nominal_relaxation_length_m": relaxation_m,
        "baseline_ay_overshoot_pct": transient["ay_overshoot_pct"],
    }]
    out = STUDY_DIR / cfg["outputs"]["data_dir"]
    out.mkdir(parents=True, exist_ok=True)
    with (out / "summary.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    r = rows[0]
    lines = [
        "# VDYN-004 Results",
        "",
        "## Finding",
        "",
        "**PASS:** tire behavior is the dominant remaining model-confidence item and must be treated as a first-drive test objective.",
        "",
        "## Key Tire Source Metrics",
        "",
        f"- Nominal load: `{r['fnomin_n']:.1f} N`; vertical range: `{r['fzmin_n']:.1f}` to `{r['fzmax_n']:.1f} N`",
        f"- Lateral peak coefficients: `PDY1={r['pdy1']:.6f}`, `PDY2={r['pdy2']:.6f}`",
        f"- Lateral stiffness coefficients: `PKY1={r['pky1']:.4f}`, `PKY2={r['pky2']:.4f}`",
        f"- Lateral relaxation coefficients: `PTY1={r['pty1']:.6f}`, `PTY2={r['pty2']:.6f}`",
        f"- Nominal relaxation scale from PTY1*R0: `{r['nominal_relaxation_length_m']:.3f} m`",
        f"- Baseline ay overshoot needing tire/setup correlation: `{r['baseline_ay_overshoot_pct']:.1f} %`",
        "",
        "## Design Implication",
        "",
        "The tire plan must log hot pressure, tire temperature spread, steering, yaw, ay, speed, and driver comments. The current model can support screening, but final setup confidence requires measured tire operating-window correlation.",
    ]
    (STUDY_DIR / cfg["outputs"]["results_markdown"]).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
