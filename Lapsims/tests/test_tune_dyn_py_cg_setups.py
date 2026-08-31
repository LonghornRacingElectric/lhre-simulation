"""Regression tests for dyn_py physical setup tuning configuration."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from tune_dyn_py_cg_setups import (
    G,
    _lateral_limit,
    _load_configuration_definition,
    _runner_payload,
    _tune_case,
)


class DynPyTuningConfigurationTest(unittest.TestCase):
    def _write_payload(self, payload: object) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "configuration.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_runner_payload_preserves_scaled_mu_and_selected_model(self) -> None:
        payload = _runner_payload(
            6,
            [
                {
                    "name": "cg_8p0in",
                    "cg_height_in": 8.0,
                    "front_antiroll_stiffness_fraction": 0.53,
                    "brake_distribution_front": 0.71,
                }
            ],
            tire_mu_scale=0.6225437130779028,
        )

        self.assertEqual(payload["model_dof"], 6)
        self.assertEqual(payload["tire_mu_scale"], 0.6225437130779028)
        self.assertEqual(payload["aero_balance_front"], 0.5)
        self.assertIn("no LSD", payload["drive_model"])
        self.assertEqual(payload["cases"][0]["cg_height_in"], 8.0)

    def test_tune_case_passes_mu_scale_to_both_physical_optimizers(self) -> None:
        roll = {
            "front_antiroll_stiffness_fraction": 0.54,
            "pure_lateral_limit_g": 1.2,
        }
        braking = {
            "brake_distribution_front": 0.72,
            "weighted_pure_braking_limit_g": 1.1,
        }
        task = ("C:/BobSim", 6, 11.6, 0.6225437130779028, [10.0], [1.0])

        with (
            patch(
                "tune_dyn_py_cg_setups._tune_front_arb_fraction",
                return_value=roll,
            ) as tune_roll,
            patch(
                "tune_dyn_py_cg_setups._tune_fixed_brake_bias",
                return_value=braking,
            ) as tune_brake,
        ):
            result = _tune_case(task)

        self.assertEqual(result["model_dof"], 6)
        self.assertEqual(result["cg_height_in"], 11.6)
        self.assertEqual(result["tire_mu_scale"], 0.6225437130779028)
        self.assertEqual(
            tune_roll.call_args.kwargs["tire_mu_scale"],
            0.6225437130779028,
        )
        self.assertEqual(
            tune_brake.call_args.kwargs["tire_mu_scale"],
            0.6225437130779028,
        )

    def test_mass_case_passes_target_mass_and_serializes_effective_masses(self) -> None:
        target_mass = 250.0
        task = (
            "C:/BobSim",
            6,
            11.5,
            0.6225437130779028,
            [10.0],
            [1.0],
            target_mass,
            "mass_plus_50lb",
        )
        with (
            patch(
                "tune_dyn_py_cg_setups._tune_front_arb_fraction",
                return_value={
                    "front_antiroll_stiffness_fraction": 0.54,
                    "pure_lateral_limit_g": 1.2,
                },
            ) as tune_roll,
            patch(
                "tune_dyn_py_cg_setups._tune_fixed_brake_bias",
                return_value={
                    "brake_distribution_front": 0.72,
                    "weighted_pure_braking_limit_g": 1.1,
                },
            ) as tune_brake,
            patch(
                "tune_dyn_py_cg_setups._load_context",
                return_value=(
                    object(),
                    SimpleNamespace(sprung_mass_kg=250.0, mass_kg=280.0),
                ),
            ),
        ):
            result = _tune_case(task)

        self.assertEqual(result["name"], "mass_plus_50lb")
        self.assertEqual(result["sprung_mass_kg"], 250.0)
        self.assertEqual(result["total_mass_kg"], 280.0)
        self.assertEqual(
            tune_roll.call_args.kwargs["target_sprung_mass_kg"], target_mass
        )
        self.assertEqual(
            tune_brake.call_args.kwargs["target_sprung_mass_kg"], target_mass
        )

    def test_runner_payload_serializes_mass_axis(self) -> None:
        payload = _runner_payload(
            6,
            [
                {
                    "name": "mass_nominal",
                    "cg_height_in": 11.5,
                    "sprung_mass_kg": 230.0,
                    "total_mass_kg": 260.0,
                    "mass_offset_lb": 0.0,
                    "front_antiroll_stiffness_fraction": 0.54,
                    "brake_distribution_front": 0.72,
                }
            ],
            tire_mu_scale=0.6225437130779028,
            sweep_axis="sprung_mass_kg",
        )

        self.assertEqual(payload["sweep_axis"], "sprung_mass_kg")
        self.assertEqual(payload["reference_case"], "mass_nominal")
        self.assertEqual(payload["cases"][0]["sprung_mass_kg"], 230.0)
        self.assertEqual(payload["cases"][0]["total_mass_kg"], 260.0)

    def test_longitudinal_cg_case_passes_rear_fraction_and_serializes_axis(
        self,
    ) -> None:
        task = (
            "C:/BobSim",
            6,
            11.5,
            0.6225437130779028,
            [10.0],
            [1.0],
            None,
            0.55,
            "rear_55pct",
        )
        selected = SimpleNamespace(
            sprung_mass_kg=230.0,
            mass_kg=260.0,
            static_rear_weight_fraction=0.55,
            static_front_weight_fraction=0.45,
            center_of_gravity_m=(0.123, 0.0, 11.5 * 0.0254),
        )
        with (
            patch(
                "tune_dyn_py_cg_setups._tune_front_arb_fraction",
                return_value={"front_antiroll_stiffness_fraction": 0.54},
            ) as tune_roll,
            patch(
                "tune_dyn_py_cg_setups._tune_fixed_brake_bias",
                return_value={"brake_distribution_front": 0.72},
            ) as tune_brake,
            patch(
                "tune_dyn_py_cg_setups._load_context",
                return_value=(object(), selected),
            ),
        ):
            result = _tune_case(task)

        self.assertEqual(
            tune_roll.call_args.kwargs["static_rear_weight_fraction"], 0.55
        )
        self.assertEqual(
            tune_brake.call_args.kwargs["static_rear_weight_fraction"], 0.55
        )
        self.assertEqual(result["rear_static_weight_fraction"], 0.55)
        self.assertEqual(result["front_static_weight_fraction"], 0.45)

        payload = _runner_payload(
            6,
            [result],
            tire_mu_scale=0.6225437130779028,
            sweep_axis="rear_static_weight_fraction",
        )
        self.assertEqual(payload["sweep_axis"], "rear_static_weight_fraction")
        self.assertEqual(payload["reference_case"], "rear_55pct")
        self.assertEqual(payload["cases"][0]["rear_static_weight_fraction"], 0.55)
        self.assertEqual(payload["cases"][0]["total_mass_kg"], 260.0)

    def test_configuration_definition_preserves_two_explicit_cg_coordinates(
        self,
    ) -> None:
        payload = {
            "sweep_axis": "configuration",
            "reference_case": "config_nominal_54",
            "cases": [
                {
                    "name": "config_nominal_54",
                    "cg_height_in": 11.5,
                    "rear_static_weight_fraction": 0.54,
                },
                {
                    "name": "config_plus0p25_56p5",
                    "cg_height_in": 11.75,
                    "rear_static_weight_fraction": 0.565,
                },
            ],
        }

        raw, cases, reference = _load_configuration_definition(
            self._write_payload(payload)
        )

        self.assertEqual(raw, payload)
        self.assertEqual(reference, "config_nominal_54")
        self.assertEqual([case["height_in"] for case in cases], [11.5, 11.75])
        self.assertEqual(
            [case["static_rear_weight_fraction"] for case in cases],
            [0.54, 0.565],
        )
        self.assertTrue(all(case["target_sprung_mass_kg"] is None for case in cases))

    def test_runner_payload_serializes_categorical_configuration_comparison(
        self,
    ) -> None:
        cases = [
            {
                "name": "config_nominal_54",
                "cg_height_in": 11.5,
                "sprung_mass_kg": 230.72288408,
                "total_mass_kg": 261.07265114,
                "rear_static_weight_fraction": 0.54,
                "front_static_weight_fraction": 0.46,
                "cg_x_m": -0.836676,
                "front_antiroll_stiffness_fraction": 0.64,
                "brake_distribution_front": 0.70,
            },
            {
                "name": "config_plus0p25_56p5",
                "cg_height_in": 11.75,
                "sprung_mass_kg": 230.72288408,
                "total_mass_kg": 261.07265114,
                "rear_static_weight_fraction": 0.565,
                "front_static_weight_fraction": 0.435,
                "cg_x_m": -0.875411,
                "front_antiroll_stiffness_fraction": 0.68,
                "brake_distribution_front": 0.68,
            },
        ]

        payload = _runner_payload(
            6,
            cases,
            tire_mu_scale=0.6225437130779028,
            sweep_axis="configuration",
            reference_case="config_nominal_54",
        )

        self.assertEqual(payload["sweep_axis"], "configuration")
        self.assertEqual(payload["reference_case"], "config_nominal_54")
        self.assertEqual(payload["cases"][1]["cg_height_in"], 11.75)
        self.assertEqual(payload["cases"][1]["rear_static_weight_fraction"], 0.565)
        self.assertEqual(payload["cases"][1]["total_mass_kg"], 261.07265114)

    def test_lateral_tuning_uses_envelopesim_robust_endpoint_contract(self) -> None:
        solve_endpoint = Mock(return_value=(1.8 * G, 0.0))
        dyn_py = SimpleNamespace(create_model=Mock(return_value="model"))
        generator = SimpleNamespace(solve_lateral_limit=solve_endpoint)
        with (
            patch(
                "tune_dyn_py_cg_setups._load_context",
                return_value=("outer_vehicle", "parameters"),
            ),
            patch.dict(
                sys.modules,
                {
                    "_0_Utils.dyn_py": dyn_py,
                    "_2_EnvelopeSim.GGV.ggv_generation": generator,
                },
            ),
        ):
            result = _lateral_limit(
                Path("C:/BobSim"),
                model_dof=6,
                height_m=11.5 * 0.0254,
                static_rear_weight_fraction=0.54,
                front_arb_fraction=0.64,
                tire_mu_scale=0.6225437130779028,
                speed_mps=11.8,
                binary_iterations=18,
            )

        self.assertAlmostEqual(result, 1.8 * G)
        self.assertEqual(solve_endpoint.call_args.kwargs["reduced_model"], "model")
        self.assertNotIn("trim_multistart", solve_endpoint.call_args.kwargs)
        self.assertNotIn("enforce_tire_load_range", solve_endpoint.call_args.kwargs)


if __name__ == "__main__":
    unittest.main()
