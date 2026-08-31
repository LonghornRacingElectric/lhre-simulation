"""Unit tests for the EnvelopeSim GGV lookup track solver."""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ggv_lookup_solver import GGVMap, solve_track


class GGVLookupSolverTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.temp = Path(self.temporary_directory.name)
        self.ggv_path = self.temp / "synthetic_ggv.csv"
        self._write_synthetic_ggv(self.ggv_path)
        self.ggv = GGVMap.from_csv(self.ggv_path)

    @staticmethod
    def _write_synthetic_ggv(path: Path) -> None:
        rows: list[dict[str, float | int]] = []
        for speed in (0.0, 10.0, 20.0):
            for ay in (-4.0, -2.0, 0.0, 2.0, 4.0):
                # The drive branch has a +/-4 m/s^2 lateral domain. The brake
                # branch is intentionally narrower to prove that sustainable
                # corner speed is not spuriously limited by fixed brake bias.
                brake_feasible = abs(ay) <= 2.0
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": 4.0 - 0.05 * speed - 0.2 * abs(ay),
                        "ax_brake_mps2": (
                            -6.0 + 0.10 * speed + 0.30 * abs(ay)
                            if brake_feasible
                            else math.nan
                        ),
                        "accel_feasible": 1,
                        "brake_feasible": int(brake_feasible),
                    }
                )
        pd.DataFrame(rows).to_csv(path, index=False)

    def test_exact_grid_nodes_reproduce_csv_boundaries(self) -> None:
        self.assertAlmostEqual(self.ggv.acceleration(10.0, 2.0), 3.1)
        self.assertAlmostEqual(self.ggv.braking_deceleration(10.0, 2.0), 4.4)

    def test_speed_interpolation_is_linear(self) -> None:
        self.assertAlmostEqual(self.ggv.acceleration(5.0, 2.0), 3.35)
        self.assertAlmostEqual(self.ggv.braking_deceleration(5.0, 2.0), 4.9)

    def test_zero_width_low_speed_boundary_retains_standing_start_accel(
        self,
    ) -> None:
        zero_width_path = self.temp / "zero_width_low_speed.csv"
        rows: list[dict[str, float | int]] = []
        for speed, center_accel, lateral_nodes in (
            (0.0, 13.62205, (0.0,)),
            (0.1, 13.62205, (0.0,)),
            (20.0, 10.0, (-2.0, 0.0, 2.0)),
        ):
            for ay in lateral_nodes:
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": center_accel - 0.25 * abs(ay),
                        "ax_brake_mps2": -8.0 + 0.25 * abs(ay),
                        "accel_feasible": 1,
                        "brake_feasible": 1,
                    }
                )
        pd.DataFrame(rows).to_csv(zero_width_path, index=False)
        ggv = GGVMap.from_csv(zero_width_path)

        self.assertAlmostEqual(ggv.acceleration(0.0, 0.0), 13.62205)
        self.assertAlmostEqual(ggv.acceleration(0.05, 0.0), 13.62205)
        self.assertAlmostEqual(ggv.acceleration(0.1, 0.0), 13.62205)
        self.assertAlmostEqual(ggv.braking_deceleration(0.0, 0.0), 8.0)
        self.assertEqual(ggv.acceleration(0.0, 1e-6), 0.0)
        self.assertEqual(ggv.braking_deceleration(0.0, -1e-6), 0.0)

        track = pd.DataFrame(
            {
                "distance_m": [1.0, 2.0],
                "dx_m": [1.0, 1.0],
                "curvature_1pm": [0.0, 0.0],
            }
        )
        trace, summary = solve_track(ggv, track, is_closed=False)
        self.assertTrue(summary["converged"])
        self.assertAlmostEqual(
            trace["speed_mps"].iloc[0],
            math.sqrt(2.0 * 13.62205),
        )

    def test_two_dimensional_interpolation_uses_normalized_lateral_demand(
        self,
    ) -> None:
        variable_domain_path = self.temp / "variable_domain.csv"
        rows = []
        for speed, lateral_limit, center_accel, edge_accel in (
            (0.0, 4.0, 4.0, 2.0),
            (10.0, 8.0, 3.0, 1.0),
        ):
            for ay, fraction in (
                (-lateral_limit, 1.0),
                (0.0, 0.0),
                (lateral_limit, 1.0),
            ):
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": center_accel
                        + fraction * (edge_accel - center_accel),
                        "ax_brake_mps2": -6.0 + 2.0 * fraction,
                        "accel_feasible": 1,
                        "brake_feasible": 1,
                    }
                )
        pd.DataFrame(rows).to_csv(variable_domain_path, index=False)
        ggv = GGVMap.from_csv(variable_domain_path)

        # At 5 m/s, ay=3 is 50% of the interpolated 6 m/s^2 lateral
        # domain. Each bounding speed slice must therefore be sampled at
        # 50% of its own domain before the two values are blended.
        self.assertAlmostEqual(ggv.acceleration(5.0, 3.0), 2.5)
        self.assertAlmostEqual(ggv.braking_deceleration(5.0, 3.0), 5.0)

    def test_lateral_speed_limit_uses_drive_not_narrower_brake_domain(self) -> None:
        limit = self.ggv.lateral_speed_limits(np.asarray([0.04]))[0]
        self.assertAlmostEqual(limit, 10.0, places=8)
        self.assertGreater(limit, math.sqrt(2.0 / 0.04))

    def test_lateral_limit_refines_to_zero_ax_inside_nan_bracket(self) -> None:
        edge_path = self.temp / "edge_ggv.csv"
        rows = []
        for speed in (0.0, 10.0):
            for ay in (-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0):
                accel_feasible = abs(ay) <= 4.0
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": 5.0 - abs(ay) if accel_feasible else math.nan,
                        "ax_brake_mps2": -6.0,
                        "accel_feasible": int(accel_feasible),
                        "brake_feasible": 1,
                    }
                )
        pd.DataFrame(rows).to_csv(edge_path, index=False)
        ggv = GGVMap.from_csv(edge_path)

        positive, negative = ggv.lateral_limits(np.asarray([0.0, 5.0, 10.0]))
        np.testing.assert_allclose(positive, 5.0)
        np.testing.assert_allclose(negative, 5.0)
        self.assertAlmostEqual(ggv.acceleration(5.0, 2.0), 3.0)
        self.assertAlmostEqual(ggv.acceleration(5.0, 4.0), 1.0)
        self.assertAlmostEqual(ggv.acceleration(5.0, 4.5), 0.5)
        self.assertAlmostEqual(ggv.acceleration(5.0, 5.0), 0.0)

    def test_lateral_limit_does_not_extrapolate_unbracketed_csv(self) -> None:
        positive, negative = self.ggv.lateral_limits(np.asarray([10.0]))
        self.assertAlmostEqual(positive[0], 4.0)
        self.assertAlmostEqual(negative[0], 4.0)

    def test_explicit_zero_ax_edge_is_authoritative(self) -> None:
        exact_edge_path = self.temp / "exact_edge_ggv.csv"
        rows = []
        for speed in (0.0, 10.0):
            for ay in (-6.0, -5.0, -4.0, -2.0, 0.0, 2.0, 4.0, 5.0, 6.0):
                accel_feasible = abs(ay) <= 5.0
                edge_ax = {
                    0.0: 5.0,
                    2.0: 4.0,
                    4.0: 3.0,
                    5.0: 0.0,
                }.get(abs(ay), math.nan)
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": edge_ax if accel_feasible else math.nan,
                        "ax_brake_mps2": -6.0,
                        "accel_feasible": int(accel_feasible),
                        "brake_feasible": 1,
                    }
                )
        pd.DataFrame(rows).to_csv(exact_edge_path, index=False)
        ggv = GGVMap.from_csv(exact_edge_path)

        positive, negative = ggv.lateral_limits(np.asarray([5.0]))
        self.assertAlmostEqual(positive[0], 5.0)
        self.assertAlmostEqual(negative[0], 5.0)
        self.assertAlmostEqual(ggv.acceleration(5.0, 5.0), 0.0)

    def test_lateral_edge_fit_shrinks_past_actuator_distortion(self) -> None:
        distorted_path = self.temp / "distorted_edge_ggv.csv"
        rows = []
        boundary = {0.0: 5.0, 2.0: 5.0, 4.0: 2.0, 5.0: 1.0}
        for speed in (0.0, 10.0):
            for ay in (-6.0, -5.0, -4.0, -2.0, 0.0, 2.0, 4.0, 5.0, 6.0):
                accel_feasible = abs(ay) <= 5.0
                rows.append(
                    {
                        "speed_mps": speed,
                        "ay_mps2": ay,
                        "ax_accel_mps2": (
                            boundary[abs(ay)] if accel_feasible else math.nan
                        ),
                        "ax_brake_mps2": -6.0,
                        "accel_feasible": int(accel_feasible),
                        "brake_feasible": 1,
                    }
                )
        pd.DataFrame(rows).to_csv(distorted_path, index=False)
        ggv = GGVMap.from_csv(distorted_path)

        # The four-point fit crosses at 6.51 m/s^2, outside the [5, 6]
        # bracket. The outermost three-point fit crosses at 5.6315789.
        expected_edge = 5.631578947368421
        positive, negative = ggv.lateral_limits(np.asarray([5.0]))
        self.assertAlmostEqual(positive[0], expected_edge)
        self.assertAlmostEqual(negative[0], expected_edge)

    def test_open_track_retains_standing_start_endpoint_convention(self) -> None:
        track = pd.DataFrame(
            {
                "distance_m": [1.0, 2.0, 3.0],
                "dx_m": [1.0, 1.0, 1.0],
                "curvature_1pm": [0.0, 0.0, 0.0],
            }
        )
        trace, summary = solve_track(self.ggv, track, is_closed=False)
        expected_first_speed = math.sqrt(2.0 * 4.0 * 1.0)
        expected_second_speed = math.sqrt(
            expected_first_speed**2 + 2.0 * (4.0 - 0.05 * expected_first_speed)
        )
        expected_third_speed = math.sqrt(
            expected_second_speed**2 + 2.0 * (4.0 - 0.05 * expected_second_speed)
        )
        expected_time = (
            2.0 / expected_first_speed
            + 1.0 / expected_second_speed
            + 1.0 / expected_third_speed
        )
        self.assertTrue(summary["converged"])
        self.assertAlmostEqual(trace["speed_mps"].iloc[0], expected_first_speed)
        self.assertAlmostEqual(
            trace["time_in_segment_s"].iloc[0],
            2.0 / expected_first_speed,
        )
        self.assertAlmostEqual(summary["lap_time_s"], expected_time)

    def test_open_track_constraint_diagnostics_align_with_segment_start(self) -> None:
        curvature = np.asarray([0.0] * 15 + [0.25] * 5 + [0.0] * 20)
        track = pd.DataFrame(
            {
                "distance_m": np.arange(1.0, len(curvature) + 1.0),
                "dx_m": np.ones_like(curvature),
                "curvature_1pm": curvature,
            }
        )
        trace, summary = solve_track(self.ggv, track, is_closed=False)
        self.assertTrue(summary["converged"])
        np.testing.assert_allclose(
            trace["ggv_constraint_speed_mps"].to_numpy()[1:],
            trace["speed_mps"].to_numpy()[:-1],
        )
        np.testing.assert_allclose(
            trace["ggv_constraint_curvature_1pm"].to_numpy()[1:],
            curvature[:-1],
        )
        self.assertEqual(trace["ggv_constraint_speed_mps"].iloc[0], 0.0)
        self.assertEqual(trace["ggv_constraint_lateral_accel_mps2"].iloc[0], 0.0)
        self._assert_longitudinal_accel_within_reported_limits(trace)

    def test_closed_track_converges_and_respects_lateral_cap(self) -> None:
        track = pd.DataFrame(
            {
                "distance_m": np.arange(1.0, 21.0),
                "dx_m": np.ones(20),
                "curvature_1pm": np.full(20, 0.04),
            }
        )
        trace, summary = solve_track(self.ggv, track, is_closed=True)
        self.assertTrue(summary["converged"])
        self.assertLessEqual(trace["speed_mps"].max(), 10.0 + 1e-8)
        self.assertLessEqual(summary["ggv_max_lateral_utilization"], 1.0 + 1e-8)
        self.assertLessEqual(summary["ggv_max_speed_domain_fraction"], 1.0)
        self.assertTrue(
            np.all(
                trace["speed_mps"].to_numpy()
                <= trace["lateral_speed_limit_mps"].to_numpy() + 1e-8
            )
        )

    def test_closed_track_constraint_diagnostics_match_current_state(self) -> None:
        curvature = np.asarray([0.0] * 20 + [0.25] * 10 + [0.0] * 20 + [0.25] * 10)
        track = pd.DataFrame(
            {
                "distance_m": np.arange(1.0, len(curvature) + 1.0),
                "dx_m": np.ones_like(curvature),
                "curvature_1pm": curvature,
            }
        )
        trace, summary = solve_track(self.ggv, track, is_closed=True)
        self.assertTrue(summary["converged"])
        np.testing.assert_allclose(
            trace["ggv_constraint_speed_mps"], trace["speed_mps"]
        )
        np.testing.assert_allclose(trace["ggv_constraint_curvature_1pm"], curvature)
        self._assert_longitudinal_accel_within_reported_limits(trace)

    def _assert_longitudinal_accel_within_reported_limits(
        self, trace: pd.DataFrame
    ) -> None:
        actual = trace["longitudinal_accel_mps2"].to_numpy()
        accel_limit = trace["ggv_accel_limit_mps2"].to_numpy()
        brake_limit = trace["ggv_brake_limit_mps2"].to_numpy()
        self.assertTrue(np.any(actual > 1e-6))
        self.assertTrue(np.any(actual < -1e-6))
        self.assertTrue(np.all(actual <= accel_limit + 1e-8))
        self.assertTrue(np.all(actual >= brake_limit - 1e-8))

    def test_missing_zero_speed_slice_is_rejected(self) -> None:
        frame = pd.read_csv(self.ggv_path)
        no_zero = self.temp / "no_zero.csv"
        frame.loc[frame["speed_mps"] > 0.0].to_csv(no_zero, index=False)
        with self.assertRaisesRegex(ValueError, "0 m/s"):
            GGVMap.from_csv(no_zero)

    def test_internal_nan_gap_is_rejected(self) -> None:
        frame = pd.read_csv(self.ggv_path)
        row = (frame["speed_mps"] == 10.0) & (frame["ay_mps2"] == 0.0)
        frame.loc[row, "ax_accel_mps2"] = math.nan
        gap = self.temp / "gap.csv"
        frame.to_csv(gap, index=False)
        with self.assertRaisesRegex(ValueError, "internal infeasible gap"):
            GGVMap.from_csv(gap)


if __name__ == "__main__":
    unittest.main()
