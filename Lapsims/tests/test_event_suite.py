"""Regression tests for the multi-event OptimumLap/OpenLAP comparison."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import pandas as pd
from scipy.io import loadmat


EXPECTED_EVENTS = {
    "acceleration": (75.0, 750, False),
    "autocross": (791.0, 7910, False),
    "skidpad": (57.33406592801372, 574, True),
    "michigan_endurance": (1069.968773, 4280, True),
}


class EventSuiteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.root = Path(__file__).resolve().parents[1]
        cls.inputs = cls.root / "inputs" / "events"
        cls.outputs = cls.root / "outputs" / "events"
        cls.manifest = json.loads(
            (cls.inputs / "openlap_event_suite_manifest.json").read_text(
                encoding="utf-8"
            )
        )
        cls.comparison = json.loads(
            (cls.outputs / "event_correlation_summary.json").read_text(
                encoding="utf-8"
            )
        )

    def test_expected_track_suite_and_exact_meshes(self) -> None:
        events = {event["Slug"]: event for event in self.manifest["Events"]}
        self.assertEqual(set(events), set(EXPECTED_EVENTS))
        for slug, (length, segments, is_closed) in EXPECTED_EVENTS.items():
            event = events[slug]
            track = pd.read_csv(self.root / event["OpenLAPTrackCsv"])
            self.assertEqual(len(track), segments)
            self.assertAlmostEqual(track["dx_m"].sum(), length, places=8)
            self.assertAlmostEqual(track["distance_m"].iloc[-1], length, places=8)
            self.assertEqual(bool(event["IsClosed"]), is_closed)

    def test_native_openlap_mat_tracks_are_readable(self) -> None:
        for event in self.manifest["Events"]:
            track = loadmat(
                self.root / event["OpenLAPTrackMat"],
                squeeze_me=True,
                struct_as_record=False,
            )
            for key in ("info", "x", "dx", "n", "r", "factor_grip", "sector"):
                self.assertIn(key, track)
            expected_points = int(event["SegmentCount"])
            if not event["IsClosed"]:
                expected_points += 1
            self.assertEqual(int(track["n"]), expected_points)

    def test_all_event_solvers_converged(self) -> None:
        for event in self.comparison["events"]:
            self.assertTrue(event["checks"]["solver_converged"])

    def test_all_event_correlation_checks_pass(self) -> None:
        self.assertTrue(self.comparison["all_checks_passed"])
        for event in self.comparison["events"]:
            self.assertTrue(event["passed"], msg=event["event_slug"])
            self.assertLess(abs(event["same_formula_delta_percent"]), 0.5)
            self.assertLess(event["speed_rmse_mps"], 0.30)

    def test_skidpad_remains_the_calibration_anchor(self) -> None:
        events = {
            event["event_slug"]: event for event in self.comparison["events"]
        }
        skidpad = events["skidpad"]
        self.assertAlmostEqual(skidpad["optimumlap_time_s"], 4.8, places=6)
        self.assertLess(abs(skidpad["same_formula_delta_percent"]), 0.01)


if __name__ == "__main__":
    unittest.main()
