"""Compare native OptimumLap and OpenLAP-equation results for every event."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


RAW_TIME_LIMIT_PERCENT = 3.0
NORMALIZED_TIME_LIMIT_PERCENT = 0.5
SPEED_RMSE_LIMIT_MPS = 0.30
SPEED_CORRELATION_LIMIT = 0.99
CONSTANT_SPEED_LIMIT_PERCENT = 0.5


def time_from_profile(
    dx: np.ndarray,
    speed: np.ndarray,
    is_closed: bool,
) -> float:
    time = dx / speed
    if not is_closed:
        time[0] *= 2.0
    return float(time.sum())


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    event_input_dir = root / "inputs" / "events"
    event_output_dir = root / "outputs" / "events"
    manifest = json.loads(
        (event_input_dir / "openlap_event_suite_manifest.json").read_text(
            encoding="utf-8"
        )
    )

    rows = []
    detailed_results = []
    plot_data = []
    for event in manifest["Events"]:
        slug = event["Slug"]
        is_closed = bool(event["IsClosed"])
        optimum_summary = json.loads(
            (event_output_dir / f"{slug}_optimumlap_summary.json").read_text(
                encoding="utf-8-sig"
            )
        )
        openlap_summary = json.loads(
            (event_output_dir / f"{slug}_openlap_summary.json").read_text(
                encoding="utf-8"
            )
        )
        optimum_trace = pd.read_csv(
            event_output_dir / f"{slug}_optimumlap_trace.csv"
        )
        openlap_trace = pd.read_csv(
            event_output_dir / f"{slug}_openlap_trace.csv"
        )
        if len(optimum_trace) != len(openlap_trace):
            raise ValueError(f"{slug}: trace lengths differ")

        distance = openlap_trace["distance_m"].to_numpy(dtype=float)
        distance_delta = (
            optimum_trace["elapsedDistance"].to_numpy(dtype=float) - distance
        )
        maximum_distance_delta = float(np.max(np.abs(distance_delta)))
        if maximum_distance_delta > 1e-8:
            raise ValueError(f"{slug}: distance meshes differ")

        dx = openlap_trace["dx_m"].to_numpy(dtype=float)
        optimum_speed = optimum_trace["speed"].to_numpy(dtype=float)
        openlap_speed = openlap_trace["speed_mps"].to_numpy(dtype=float)
        speed_delta = openlap_speed - optimum_speed
        optimum_time = float(optimum_summary["LapTime_s"])
        openlap_time = float(openlap_summary["lap_time_s"])
        raw_delta = openlap_time - optimum_time
        raw_delta_percent = 100.0 * raw_delta / optimum_time
        optimum_profile_time = time_from_profile(dx, optimum_speed, is_closed)
        openlap_profile_time = time_from_profile(dx, openlap_speed, is_closed)
        normalized_delta = openlap_profile_time - optimum_profile_time
        normalized_delta_percent = 100.0 * normalized_delta / optimum_profile_time
        speed_rmse = float(np.sqrt(np.mean(speed_delta**2)))
        speed_mae = float(np.mean(np.abs(speed_delta)))
        maximum_speed_delta = float(np.max(np.abs(speed_delta)))

        optimum_variation = float(np.ptp(optimum_speed))
        openlap_variation = float(np.ptp(openlap_speed))
        is_constant_speed = max(optimum_variation, openlap_variation) < 1e-6
        if is_constant_speed:
            speed_correlation = None
            constant_speed_delta_percent = (
                100.0 * float(np.mean(speed_delta)) / float(np.mean(optimum_speed))
            )
            profile_check = (
                abs(constant_speed_delta_percent)
                < CONSTANT_SPEED_LIMIT_PERCENT
            )
        else:
            speed_correlation = float(
                np.corrcoef(optimum_speed, openlap_speed)[0, 1]
            )
            constant_speed_delta_percent = None
            profile_check = speed_correlation > SPEED_CORRELATION_LIMIT

        checks = {
            "solver_converged": bool(openlap_summary["converged"]),
            "raw_time_delta_below_3_percent": (
                abs(raw_delta_percent) < RAW_TIME_LIMIT_PERCENT
            ),
            "same_formula_time_delta_below_0_5_percent": (
                abs(normalized_delta_percent) < NORMALIZED_TIME_LIMIT_PERCENT
            ),
            "speed_rmse_below_0_30_mps": speed_rmse < SPEED_RMSE_LIMIT_MPS,
            "speed_profile_shape_or_constant_speed_check": bool(profile_check),
            "distance_mesh_below_1e_8_m": maximum_distance_delta < 1e-8,
        }
        passed = all(checks.values())
        detail = {
            "event_slug": slug,
            "event_name": event["Name"],
            "configuration": event["Configuration"],
            "optimumlap_time_s": optimum_time,
            "openlap_time_s": openlap_time,
            "raw_openlap_minus_optimumlap_s": raw_delta,
            "raw_openlap_minus_optimumlap_percent": raw_delta_percent,
            "optimumlap_profile_openlap_formula_s": optimum_profile_time,
            "openlap_profile_openlap_formula_s": openlap_profile_time,
            "same_formula_delta_s": normalized_delta,
            "same_formula_delta_percent": normalized_delta_percent,
            "speed_rmse_mps": speed_rmse,
            "speed_mae_mps": speed_mae,
            "maximum_absolute_speed_delta_mps": maximum_speed_delta,
            "speed_profile_correlation": speed_correlation,
            "constant_speed_delta_percent": constant_speed_delta_percent,
            "maximum_absolute_distance_delta_m": maximum_distance_delta,
            "checks": checks,
            "passed": passed,
        }
        detailed_results.append(detail)
        rows.append(
            {
                "Event": event["Name"],
                "Configuration": event["Configuration"],
                "OptimumLap_s": optimum_time,
                "OpenLAP_s": openlap_time,
                "Raw_delta_s": raw_delta,
                "Raw_delta_percent": raw_delta_percent,
                "Same_formula_delta_s": normalized_delta,
                "Same_formula_delta_percent": normalized_delta_percent,
                "Speed_RMSE_mps": speed_rmse,
                "Speed_correlation": speed_correlation,
                "Constant_speed_delta_percent": constant_speed_delta_percent,
                "Passed": passed,
            }
        )
        plot_data.append(
            (event["Name"], distance, optimum_speed, openlap_speed, detail)
        )

    all_passed = all(result["passed"] for result in detailed_results)
    suite = {
        "thresholds": {
            "absolute_raw_time_delta_percent_less_than": RAW_TIME_LIMIT_PERCENT,
            "absolute_same_formula_time_delta_percent_less_than": (
                NORMALIZED_TIME_LIMIT_PERCENT
            ),
            "speed_rmse_mps_less_than": SPEED_RMSE_LIMIT_MPS,
            "speed_profile_correlation_greater_than": SPEED_CORRELATION_LIMIT,
            "absolute_constant_speed_delta_percent_less_than": (
                CONSTANT_SPEED_LIMIT_PERCENT
            ),
        },
        "events": detailed_results,
        "all_checks_passed": all_passed,
    }
    (event_output_dir / "event_correlation_summary.json").write_text(
        json.dumps(suite, indent=2),
        encoding="utf-8",
    )
    pd.DataFrame(rows).to_csv(
        event_output_dir / "event_correlation_summary.csv",
        index=False,
    )

    fig, axes = plt.subplots(
        len(plot_data),
        2,
        figsize=(13, 3.2 * len(plot_data)),
        gridspec_kw={"width_ratios": [3.3, 1.3]},
    )
    for row_index, (name, distance, optimum, openlap, detail) in enumerate(
        plot_data
    ):
        speed_axis = axes[row_index, 0]
        delta_axis = axes[row_index, 1]
        speed_axis.plot(
            distance,
            optimum * 3.6,
            label=f"OptimumLap {detail['optimumlap_time_s']:.3f} s",
            linewidth=1.5,
        )
        speed_axis.plot(
            distance,
            openlap * 3.6,
            label=f"OpenLAP {detail['openlap_time_s']:.3f} s",
            linewidth=1.2,
            alpha=0.85,
        )
        speed_axis.set_title(name)
        speed_axis.set_ylabel("Speed (km/h)")
        speed_axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        speed_axis.grid(alpha=0.25)
        speed_axis.legend(loc="best", fontsize=8)

        delta_axis.plot(
            distance,
            (openlap - optimum) * 3.6,
            color="#7b3294",
            linewidth=1.0,
        )
        delta_axis.axhline(0.0, color="black", linewidth=0.7)
        delta_axis.set_title(
            f"RMSE {detail['speed_rmse_mps']:.3f} m/s"
        )
        delta_axis.set_ylabel("OpenLAP - OptimumLap\n(km/h)")
        delta_axis.grid(alpha=0.25)
        if row_index == len(plot_data) - 1:
            speed_axis.set_xlabel("Distance (m)")
            delta_axis.set_xlabel("Distance (m)")
    fig.suptitle("Matched OptimumLap vs OpenLAP Event Correlation", y=1.002)
    fig.tight_layout()
    fig.savefig(
        event_output_dir / "optimumlap_vs_openlap_all_events.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(fig)

    print(json.dumps(suite, indent=2))
    if not all_passed:
        raise SystemExit("One or more event correlation checks failed")


if __name__ == "__main__":
    main()
