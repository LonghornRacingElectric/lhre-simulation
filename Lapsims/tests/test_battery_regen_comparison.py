"""Synthetic tests for the versioned regen comparison workflow."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_regen_comparison import (  # noqa: E402
    build_comparison,
    run_comparison,
    validate_comparison,
)


def _row(
    candidate_id: str,
    cell_id: str,
    series: int,
    parallel: int,
    pack_mass: float,
    power: float,
    time: float,
    net_energy: float,
    heat: float,
    reserve: float,
    overall_rank: float,
    cell_rank: float,
) -> dict[str, object]:
    return {
        "candidate_id": candidate_id,
        "cell_id": cell_id,
        "manufacturer": "Synthetic Cells",
        "cell_model": cell_id.upper(),
        "series_cells": series,
        "parallel_cells": parallel,
        "total_cells": series * parallel,
        "pack_mass_kg": pack_mass,
        "vehicle_mass_kg": 220.0 + pack_mass,
        "nominal_pack_energy_kwh": 7.0 + 0.1 * parallel,
        "model_usable_energy_kwh": 6.5 + 0.1 * parallel,
        "power_limit_kw": power,
        "elapsed_time_s": time,
        "terminal_energy_kwh": net_energy,
        "pack_resistive_heat_kwh": heat,
        "remaining_usable_chemical_kwh": reserve,
        "overall_rank": overall_rank,
        "cell_rank": cell_rank,
        "simulation_fidelity": "synthetic",
        "constraint_valid": True,
        "equal_reserve_eligible": True,
    }


def _prior() -> pd.DataFrame:
    return pd.DataFrame(
        [
            _row(
                "alpha_120s4p",
                "alpha",
                120,
                4,
                32.0,
                32.0,
                1250.0,
                6.2,
                0.45,
                0.50,
                2.0,
                1.0,
            ),
            _row(
                "alpha_125s4p",
                "alpha",
                125,
                4,
                33.0,
                34.0,
                1235.0,
                6.3,
                0.48,
                0.50,
                1.0,
                2.0,
            ),
            _row(
                "beta_130s3p",
                "beta",
                130,
                3,
                29.0,
                30.0,
                1270.0,
                5.9,
                0.40,
                0.50,
                3.0,
                1.0,
            ),
        ]
    )


def _regen() -> pd.DataFrame:
    rows = []
    for prior_row in _prior().to_dict(orient="records"):
        row = dict(prior_row)
        candidate_id = str(row["candidate_id"])
        if candidate_id == "alpha_120s4p":
            power_gain, time_reduction, recovered, rank = 2.0, 12.0, 0.42, 1.0
        elif candidate_id == "alpha_125s4p":
            power_gain, time_reduction, recovered, rank = 0.5, 4.0, 0.37, 2.0
        else:
            power_gain, time_reduction, recovered, rank = 1.0, 7.0, 0.35, 3.0
        row["power_limit_kw"] = float(row["power_limit_kw"]) + power_gain
        row["elapsed_time_s"] = float(row["elapsed_time_s"]) - time_reduction
        row["terminal_discharge_energy_kwh"] = (
            float(row["terminal_energy_kwh"]) + 0.10
        )
        row["terminal_regenerated_energy_kwh"] = recovered
        row["terminal_energy_kwh"] = (
            float(row["terminal_discharge_energy_kwh"]) - recovered
        )
        row["pack_resistive_heat_kwh"] = float(row["pack_resistive_heat_kwh"]) + 0.03
        row["regen_pack_resistive_heat_kwh"] = 0.015
        row["remaining_usable_chemical_kwh"] = (
            float(row["remaining_usable_chemical_kwh"]) + 0.02
        )
        row["regen_active_rms_terminal_power_kw"] = 18.0 + power_gain
        row["overall_rank"] = rank
        rows.append(row)
    return pd.DataFrame(rows).iloc[::-1].reset_index(drop=True)


class BatteryRegenComparisonTest(unittest.TestCase):
    def test_join_defaults_and_delta_signs(self) -> None:
        comparison, provenance = build_comparison(_prior(), _regen())
        self.assertEqual(
            comparison["candidate_id"].tolist(),
            ["alpha_120s4p", "alpha_125s4p", "beta_130s3p"],
        )
        alpha = comparison.loc[
            comparison["candidate_id"] == "alpha_120s4p"
        ].iloc[0]
        self.assertAlmostEqual(alpha["prior_gross_discharge_kwh"], 6.2)
        self.assertAlmostEqual(alpha["prior_recovered_energy_kwh"], 0.0)
        self.assertAlmostEqual(alpha["prior_regen_pack_heat_kwh"], 0.0)
        self.assertAlmostEqual(alpha["prior_active_rms_kw"], 0.0)
        self.assertAlmostEqual(alpha["sustainable_power_gain_kw"], 2.0)
        self.assertAlmostEqual(alpha["endurance_time_reduction_s"], 12.0)
        self.assertAlmostEqual(alpha["gross_discharge_change_kwh"], 0.1)
        self.assertAlmostEqual(alpha["recovered_energy_gain_kwh"], 0.42)
        self.assertAlmostEqual(alpha["net_terminal_energy_reduction_kwh"], 0.32)
        self.assertAlmostEqual(alpha["total_pack_heat_change_kwh"], 0.03)
        self.assertAlmostEqual(alpha["regen_pack_heat_increase_kwh"], 0.015)
        self.assertAlmostEqual(alpha["final_reserve_change_kwh"], 0.02)
        self.assertAlmostEqual(alpha["active_rms_change_kw"], 20.0)
        self.assertAlmostEqual(alpha["overall_rank_improvement"], 1.0)
        self.assertTrue(
            provenance["metric_sources"]["prior"]["gross_discharge_kwh"][
                "fallback"
            ]
        )
        validation = validate_comparison(comparison, 3)
        self.assertTrue(validation["all_checks_passed"], validation)

    def test_current_and_expected_regen_aliases_are_supported(self) -> None:
        regen = _regen().rename(
            columns={
                "terminal_discharge_energy_kwh": (
                    "gross_discharge_terminal_energy_kwh"
                ),
                "terminal_regenerated_energy_kwh": (
                    "recovered_terminal_energy_kwh"
                ),
            }
        )
        comparison, provenance = build_comparison(_prior(), regen)
        self.assertEqual(len(comparison), 3)
        self.assertEqual(
            provenance["metric_sources"]["regen"]["gross_discharge_kwh"][
                "source_column"
            ],
            "gross_discharge_terminal_energy_kwh",
        )

    def test_candidate_mismatch_is_rejected(self) -> None:
        regen = _regen().iloc[:-1].copy()
        with self.assertRaisesRegex(ValueError, "candidate sets differ"):
            build_comparison(_prior(), regen)

    def test_duplicate_candidate_is_rejected(self) -> None:
        regen = pd.concat([_regen(), _regen().iloc[[0]]], ignore_index=True)
        with self.assertRaisesRegex(ValueError, "duplicate candidate_id"):
            build_comparison(_prior(), regen)

    def test_topology_change_is_rejected(self) -> None:
        regen = _regen()
        regen.loc[regen["candidate_id"] == "alpha_120s4p", "series_cells"] = 121
        with self.assertRaisesRegex(ValueError, "changed topology"):
            build_comparison(_prior(), regen)

    def test_full_synthetic_artifact_bundle_with_plots(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            prior_path = root / "prior.csv"
            regen_path = root / "regen.csv"
            output_dir = root / "comparison_v1"
            _prior().to_csv(prior_path, index=False)
            _regen().to_csv(regen_path, index=False)

            result = run_comparison(
                prior_results_csv=prior_path,
                regen_results_csv=regen_path,
                output_dir=output_dir,
                scenario_name="synthetic_regen_v1",
                write_plot_files=True,
            )

            self.assertTrue(result["validation"]["all_checks_passed"])
            expected = {
                "battery_regen_comparison.csv",
                "top_rank_changes.csv",
                "headline_summary.json",
                "validation_report.json",
                "comparison_inputs.json",
                "battery_regen_comparison_report.md",
                "artifact_manifest.json",
            }
            self.assertTrue(
                expected.issubset(
                    {path.name for path in output_dir.iterdir() if path.is_file()}
                )
            )
            plots = list((output_dir / "plots").glob("*.png"))
            self.assertEqual(len(plots), 5)
            manifest = json.loads(
                (output_dir / "artifact_manifest.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(manifest["candidate_count"], 3)
            self.assertEqual(len(manifest["artifacts"]), 11)


if __name__ == "__main__":
    unittest.main()
