"""Regression tests for the matched OptimumLap/OpenLAP baseline."""

from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

import pandas as pd
from scipy.io import loadmat


class BaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.root = Path(__file__).resolve().parents[1]
        cls.inputs = cls.root / "inputs"
        cls.outputs = cls.root / "outputs"
        cls.vehicle = json.loads(
            (cls.inputs / "openlap_vehicle.json").read_text(encoding="utf-8")
        )
        cls.optimum = json.loads(
            (cls.inputs / "optimumlap_baseline.json").read_text(
                encoding="utf-8-sig"
            )
        )

    def test_mass_matches_bobsim_and_optimumlap(self) -> None:
        self.assertAlmostEqual(
            self.vehicle["mass_kg"],
            self.optimum["Vehicle"]["Runtime"]["Mass"],
            places=10,
        )

    def test_track_mesh_is_exact(self) -> None:
        track = pd.read_csv(self.inputs / "michigan_openlap_track.csv")
        self.assertEqual(len(track), 4280)
        self.assertAlmostEqual(track["dx_m"].sum(), 1069.968773, places=8)
        self.assertAlmostEqual(
            track["distance_m"].iloc[-1], 1069.968773, places=8
        )

    def test_mat_files_are_readable_and_complete(self) -> None:
        track = loadmat(
            self.inputs / "OpenTRACK_FSAE_Michigan_Endurance_2014.mat",
            squeeze_me=True,
            struct_as_record=False,
        )
        vehicle = loadmat(
            self.inputs / "OpenVEHICLE_LHRe_Matched_Baseline.mat",
            squeeze_me=True,
            struct_as_record=False,
        )
        for key in ("x", "dx", "n", "r", "bank", "incl", "factor_grip"):
            self.assertIn(key, track)
        for key in (
            "M",
            "Cl",
            "Cd",
            "A",
            "mu_x",
            "mu_y",
            "sens_x",
            "sens_y",
            "vehicle_speed",
            "fx_engine",
        ):
            self.assertIn(key, vehicle)

    def test_load_sensitive_mu_mapping_is_exact(self) -> None:
        validation = pd.read_csv(
            self.outputs / "tire_load_sensitivity_validation.csv"
        )
        self.assertLess(validation["mu_x_difference"].abs().max(), 1e-12)
        self.assertLess(validation["mu_y_difference"].abs().max(), 1e-12)
        self.assertGreater(
            validation["openlap_mu_x"].iloc[0],
            validation["openlap_mu_x"].iloc[-1],
        )
        self.assertGreater(
            validation["openlap_mu_y"].iloc[0],
            validation["openlap_mu_y"].iloc[-1],
        )

    def test_solver_converged_and_comparison_is_close(self) -> None:
        openlap = json.loads(
            (self.outputs / "openlap_summary.json").read_text(encoding="utf-8")
        )
        comparison = json.loads(
            (self.outputs / "comparison_summary.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertTrue(openlap["converged"])
        self.assertLess(
            abs(
                comparison["difference_openlap_minus_optimumlap"][
                    "lap_time_percent"
                ]
            ),
            2.0,
        )
        self.assertGreater(
            comparison["difference_openlap_minus_optimumlap"][
                "speed_profile_correlation"
            ],
            0.99,
        )
        self.assertLess(
            abs(
                comparison["difference_openlap_minus_optimumlap"][
                    "same_openlap_time_formula_delta_percent"
                ]
            ),
            0.1,
        )


if __name__ == "__main__":
    unittest.main()
