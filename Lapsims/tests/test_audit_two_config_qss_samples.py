"""Focused tests for coupled-configuration reduced-QSS reconstruction support."""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PINNED_BOBSIM_ROOT = (
    ROOT.parents[2] / "tmp" / "bobsim-two-config-cg-9e1af8a"
)
sys.path.insert(0, str(ROOT / "src"))

from audit_two_config_qss_samples import (
    EXPECTED_MU_SCALE,
    EXPECTED_SPRUNG_MASS_KG,
    EXPECTED_TOTAL_MASS_KG,
    validate_configuration_metadata,
    validate_configuration_sweep,
)


def _sweep() -> dict[str, object]:
    return {
        "sweep_axis": "configuration",
        "model_dof": 6,
        "reference_case": "config_nominal_54",
        "aero_balance_front": 0.5,
        "tire_mu_scale": EXPECTED_MU_SCALE,
        "drive_model": "RWD with equal rear-wheel force split; no LSD model",
        "cases": [
            {
                "name": "config_nominal_54",
                "cg_height_in": 11.5,
                "rear_static_weight_fraction": 0.54,
                "front_static_weight_fraction": 0.46,
                "cg_x_m": -0.54 * 1.5494,
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "front_antiroll_stiffness_fraction": 0.61,
                "brake_distribution_front": 0.70,
            },
            {
                "name": "config_plus0p25_56p5",
                "cg_height_in": 11.75,
                "rear_static_weight_fraction": 0.565,
                "front_static_weight_fraction": 0.435,
                "cg_x_m": -0.565 * 1.5494,
                "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
                "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
                "front_antiroll_stiffness_fraction": 0.66,
                "brake_distribution_front": 0.68,
            },
        ],
    }


def _metadata(
    *,
    name: str = "config_nominal_54",
    height_in: float = 11.5,
    rear: float = 0.54,
    front_arb: float = 0.61,
    brake: float = 0.70,
) -> dict[str, object]:
    wheelbase = 1.5494
    front = 1.0 - rear
    cg_x = -rear * wheelbase
    cg_z = height_in * 0.0254
    total = EXPECTED_TOTAL_MASS_KG
    loads = [
        0.5 * front * total * 9.80665,
        0.5 * front * total * 9.80665,
        0.5 * rear * total * 9.80665,
        0.5 * rear * total * 9.80665,
    ]
    case_tuning = {
        "name": name,
        "cg_height_in": height_in,
        "rear_static_weight_fraction": rear,
        "front_antiroll_stiffness_fraction": front_arb,
        "brake_distribution_front": brake,
    }
    return {
        "model_family": "dyn_py_reduced_order_qss",
        "model_key": "dyn_py_6dof_qss",
        "model_dof": 6,
        "sweep_axis": "configuration",
        "cg_case": name,
        "cg_height_in": height_in,
        "cg_height_m": cg_z,
        "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
        "total_mass_kg": total,
        "rear_static_weight_fraction": rear,
        "front_static_weight_fraction": front,
        "effective_aero_balance_front": 0.5,
        "drive_distribution_front": 0.0,
        "limited_slip_differential_model": False,
        "tire_mu_scale": EXPECTED_MU_SCALE,
        "front_antiroll_stiffness_fraction": front_arb,
        "effective_brake_distribution_front": brake,
        "case_tuning": case_tuning,
        "vehicle": {
            "mass": total,
            "front_static_frac": front,
            "aero_balance_front": 0.5,
            "drive_distribution_front": 0.0,
            "brake_distribution_front": brake,
        },
        "reduced_parameter_overrides": {
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "total_mass_kg": total,
            "rear_static_weight_fraction": rear,
            "front_static_weight_fraction": front,
            "center_of_gravity_m": [cg_x, 0.0, cg_z],
            "aero_balance_front": 0.5,
            "brake_distribution_front": brake,
            "front_antiroll_stiffness_fraction": front_arb,
        },
        "reduced_vehicle_parameters": {
            "mass_kg": total,
            "sprung_mass_kg": EXPECTED_SPRUNG_MASS_KG,
            "center_of_gravity_m": [cg_x, 0.0, cg_z],
            "corner_positions_m": [
                [-cg_x, 0.6, -cg_z],
                [-cg_x, -0.6, -cg_z],
                [-wheelbase - cg_x, 0.6, -cg_z],
                [-wheelbase - cg_x, -0.6, -cg_z],
            ],
            "static_wheel_loads_n": loads,
            "aero_balance_front": 0.5,
            "aero_cop_m": [-0.5 * wheelbase - cg_x, 0.0, -cg_z],
            "drive_distribution_front": 0.0,
            "brake_distribution_front": brake,
        },
    }


class ConfigurationSchemaTest(unittest.TestCase):
    def test_accepts_exact_coupled_height_and_rear_fraction_cases(self) -> None:
        tuning = validate_configuration_sweep(_sweep())
        self.assertEqual(set(tuning), {"config_nominal_54", "config_plus0p25_56p5"})
        self.assertAlmostEqual(
            tuning["config_plus0p25_56p5"]["brake_distribution_front"], 0.68
        )

        for metadata in (
            _metadata(),
            _metadata(
                name="config_plus0p25_56p5",
                height_in=11.75,
                rear=0.565,
                front_arb=0.66,
                brake=0.68,
            ),
        ):
            result = validate_configuration_metadata(
                metadata, expected_tuning=tuning
            )
            self.assertAlmostEqual(
                result["cg_x_m"],
                -float(metadata["rear_static_weight_fraction"]) * 1.5494,
            )

    def test_rejects_desynchronized_coupled_design_or_tuning(self) -> None:
        swapped = _sweep()
        swapped["cases"][1]["cg_height_in"] = 11.5
        with self.assertRaisesRegex(ValueError, "CG height"):
            validate_configuration_sweep(swapped)

        tuning = validate_configuration_sweep(_sweep())
        metadata = _metadata()
        metadata["reduced_parameter_overrides"]["center_of_gravity_m"][0] = -0.9
        with self.assertRaisesRegex(ValueError, "CG x"):
            validate_configuration_metadata(metadata, expected_tuning=tuning)

        metadata = _metadata()
        metadata["case_tuning"]["brake_distribution_front"] = 0.71
        with self.assertRaisesRegex(ValueError, "brake_distribution_front"):
            validate_configuration_metadata(metadata, expected_tuning=tuning)


@unittest.skipUnless(
    (PINNED_BOBSIM_ROOT / "vehicle.yml").is_file(),
    "Pinned detached BobSim snapshot is not available.",
)
class PinnedReconstructionTest(unittest.TestCase):
    def test_pinned_override_api_reconstructs_both_explicit_cases(self) -> None:
        code = f"""
import numpy as np
from _0_Utils.dyn_py import (
    ReducedVehicleOverrides,
    apply_reduced_vehicle_overrides,
    load_reduced_vehicle_parameters,
)
base = load_reduced_vehicle_parameters(r'{PINNED_BOBSIM_ROOT / "vehicle.yml"}')
assert abs(base.sprung_mass_kg - {EXPECTED_SPRUNG_MASS_KG!r}) < 1e-8
assert abs(base.mass_kg - {EXPECTED_TOTAL_MASS_KG!r}) < 1e-8
for height, rear, arb, brake in (
    (11.5, 0.54, 0.61, 0.70),
    (11.75, 0.565, 0.66, 0.68),
):
    p = apply_reduced_vehicle_overrides(
        base,
        ReducedVehicleOverrides(
            absolute_cg_height_m=height * 0.0254,
            static_rear_weight_fraction=rear,
            aero_balance_front=0.5,
            brake_distribution_front=brake,
            front_antiroll_stiffness_fraction=arb,
            tire_mu_scale={EXPECTED_MU_SCALE!r},
        ),
    )
    assert abs(p.center_of_gravity_m[2] - height * 0.0254) < 1e-10
    assert abs(p.center_of_gravity_m[0] + rear * 1.5494) < 1e-10
    assert abs(p.static_rear_weight_fraction - rear) < 1e-10
    assert abs(p.sprung_mass_kg - {EXPECTED_SPRUNG_MASS_KG!r}) < 1e-8
    assert abs(p.mass_kg - {EXPECTED_TOTAL_MASS_KG!r}) < 1e-8
    assert np.all(np.asarray(p.static_wheel_loads_n) > 0.0)
"""
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(PINNED_BOBSIM_ROOT)
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=PINNED_BOBSIM_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
