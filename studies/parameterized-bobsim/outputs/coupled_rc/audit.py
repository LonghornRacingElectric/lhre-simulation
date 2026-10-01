import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from _0_Utils.plotting.plot_engine import PlotEngine


root = Path("_3_StandardSim/generated_results/coupled_rc")
refined = Path("_3_StandardSim/generated_results/coupled_rc_refined")
columns = ["ay_mps2", "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR"]


def history(base, pair):
    return pd.read_csv(base / f"pair_{pair:03d}/results/case_000/transient.csv")


def lltd(frame):
    front = frame.Fz_FR - frame.Fz_FL
    rear = frame.Fz_RR - frame.Fz_RL
    return 100 * front / (front + rear)


coarse_summary = pd.read_csv(root / "summary.csv")
fine_summary = pd.read_csv(refined / "summary.csv")
audit: dict[str, Any] = {
    "comparisons": [],
    "interpretation": "Numerical convergence only; illustrative uncorrelated vehicle.",
}
for pair, fine_pair in [(0, 0), (4, 1)]:
    coarse, fine = history(root, pair), history(refined, fine_pair)
    item: dict[str, Any] = {"pair": pair}
    for col in columns:
        aligned = np.interp(coarse.time_s, fine.time_s, fine[col])
        item[f"max_abs_{col}_difference"] = float(np.max(np.abs(coarse[col] - aligned)))
    valid = (coarse.ay_mps2.abs() >= 1).to_numpy()
    fine_valid = fine.ay_mps2.abs() >= 1
    aligned = np.interp(coarse.time_s[valid], fine.time_s[fine_valid], lltd(fine)[fine_valid])
    item["max_abs_lltd_difference_pp"] = float(np.max(np.abs(lltd(coarse)[valid] - aligned)))
    for col in ["rise_10_90_s", "final_ay_mps2", "overshoot_percent"]:
        item[f"{col}_difference"] = float(coarse_summary.iloc[pair][col] - fine_summary.iloc[fine_pair][col])
    audit["comparisons"].append(item)

baseline, high = history(root, 0), history(root, 4)
effect: dict[str, Any] = {}
for col in columns:
    delta = high[col] - baseline[col]
    i = delta.abs().idxmax()
    effect[col] = {
        "max_abs_difference": abs(float(delta[i])),
        "signed_difference": float(delta[i]),
        "time_s": float(high.time_s[i]),
    }
valid = (baseline.ay_mps2.abs() >= 1) & (high.ay_mps2.abs() >= 1)
delta = (lltd(high) - lltd(baseline))[valid]
i = delta.abs().idxmax()
effect["lltd"] = {
    "max_abs_difference_pp": abs(float(delta[i])),
    "signed_difference_pp": float(delta[i]),
    "time_s": float(high.time_s[i]),
    "baseline_pct": float(lltd(baseline)[i]),
    "high_pct": float(lltd(high)[i]),
    "ay_mps2": float(high.ay_mps2[i]),
}
fine_base, fine_high = history(refined, 0), history(refined, 1)
valid_fine = (fine_base.ay_mps2.abs() >= 1) & (fine_high.ay_mps2.abs() >= 1)
effect["fine_max_abs_lltd_difference_pp"] = float((lltd(fine_high) - lltd(fine_base))[valid_fine].abs().max())
effect["coarse_rise_time_change_s"] = float(coarse_summary.iloc[-1].rise_10_90_s - coarse_summary.iloc[0].rise_10_90_s)
effect["fine_rise_time_change_s"] = float(fine_summary.iloc[-1].rise_10_90_s - fine_summary.iloc[0].rise_10_90_s)
audit["endpoint_effect"] = effect
(root / "convergence.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n")

series: dict[str, dict[str, Any]] = {k: {} for k in ["time", "ay", "lltd", "front", "rear"]}
for i, row in coarse_summary.iterrows():
    frame = history(root, i)
    frame = frame[(frame.time_s >= 1) & (frame.time_s <= 1.8)]
    label = f"F {row.front_rc_mm:03.0f} / R {row.rear_rc_mm:.1f} mm"
    series["time"][label] = frame.time_s.tolist()
    series["ay"][label] = frame.ay_mps2.tolist()
    series["lltd"][label] = lltd(frame).where(frame.ay_mps2.abs() >= 1).tolist()
    series["front"][label] = (frame.Fz_FR - frame.Fz_FL).tolist()
    series["rear"][label] = (frame.Fz_RR - frame.Fz_RL).tolist()
fields = [
    ("ay", "Lateral acceleration", "Ay (m/s²)"),
    ("lltd", "Transient front LLTD (|Ay| >= 1)", "Front share (%)"),
    ("front", "Front load difference", "FR - FL (N)"),
    ("rear", "Rear load difference", "RR - RL (N)"),
]
plots = {
    "turn_in": {
        "layout": "quad",
        "title": "RC-only paired sweep: turn-in at 15 m/s",
        "subplots": [
            {"title": title, "x": {"key": "time", "label": "Time (s)"}, "y": {"key": key, "label": unit}}
            for key, title, unit in fields
        ],
    }
}
PlotEngine({"plots": plots}).save_pngs({"series": series}, root)
print(json.dumps(audit, indent=2))
