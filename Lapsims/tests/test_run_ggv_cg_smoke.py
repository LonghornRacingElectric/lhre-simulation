"""Regression tests for CG-smoke 2026 Michigan points projection."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from dataclasses import dataclass, replace
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from battery_dynamic_points import EVENT_RULES
from reduced_ggv_bridge import (
    DEFAULT_TOP_SPEED_MPS,
    adapt_envelope_for_lapsims,
    cache_fingerprint,
    default_output_root,
    dense_speed_slices,
    generate_envelopes,
    model_identity,
)
from run_ggv_cg_smoke import (
    _finite_metadata_float,
    _load_case_tuning,
    _load_fsae_2026_results_reference,
    _load_sweep_definition,
    _points_correlation,
    _resolve_drive_power_limit_w,
    _resolve_tire_mu_scale,
    _scale_ggv_tire_peak_parameters,
    _score_rows,
    _tire_mu_fingerprint_fields,
    _tire_mu_source,
    _validate_existing_case_metadata,
    _validate_existing_ggv_speed_grid,
)


@dataclass(frozen=True)
class _FakeConfig:
    speeds: tuple[float, ...]


@dataclass(frozen=True)
class _FakeEnvelope:
    speed: float
    marker: str = "generated"
    ay: object = (-1.0, 0.0, 1.0)
    ax_accel: object = (0.0, 1.0, 0.0)
    ax_brake: object = (-1.0, -1.0, -1.0)


@dataclass(frozen=True)
class _FakeGgvVehicle:
    pdx1: float = 2.6
    pdx2: float = -0.62
    pdy1: float = -2.4
    pdy2: float = 0.34
    mu_min: float = 0.8
    stiffness_marker: float = 53.0


class ReducedGgvBridgeTest(unittest.TestCase):
    def test_dense_speeds_include_zero_and_exact_driveline_ceiling(self) -> None:
        speeds = dense_speed_slices()
        self.assertEqual(speeds[:3], (0.0, 1.0, 2.0))
        self.assertEqual(speeds[-2], 42.0)
        self.assertEqual(speeds[-1], DEFAULT_TOP_SPEED_MPS)
        self.assertEqual(len(speeds), 44)

    def test_reduced_generation_explicitly_passes_model_and_adds_zero_proxy(
        self,
    ) -> None:
        calls: list[tuple[_FakeConfig, dict[str, object]]] = []

        def fake_generate(
            _vehicle: object, config: _FakeConfig, **kwargs: object
        ) -> list[_FakeEnvelope]:
            calls.append((config, kwargs))
            return [_FakeEnvelope(speed=value) for value in config.speeds]

        model = object()
        config = _FakeConfig((0.0, 1.0, DEFAULT_TOP_SPEED_MPS))
        envelopes, trim_speeds = generate_envelopes(
            fake_generate,
            vehicle=object(),
            config=config,
            reduced_model=model,
            zero_proxy_mps=0.1,
        )

        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0][1]["reduced_model"], model)
        self.assertEqual(calls[0][0].speeds, (0.1, 1.0, DEFAULT_TOP_SPEED_MPS))
        self.assertEqual(trim_speeds, calls[0][0].speeds)
        self.assertEqual(
            [envelope.speed for envelope in envelopes],
            [0.0, 0.1, 1.0, DEFAULT_TOP_SPEED_MPS],
        )

    def test_legacy_generation_retains_backward_compatible_call(self) -> None:
        calls: list[dict[str, object]] = []

        def fake_generate(
            _vehicle: object, config: _FakeConfig, **kwargs: object
        ) -> list[_FakeEnvelope]:
            calls.append(kwargs)
            return [_FakeEnvelope(speed=value) for value in config.speeds]

        config = _FakeConfig((0.0, 1.0))
        envelopes, generation_speeds = generate_envelopes(
            fake_generate,
            vehicle=object(),
            config=config,
            reduced_model=None,
        )
        self.assertEqual(calls, [{}])
        self.assertEqual(generation_speeds, config.speeds)
        self.assertEqual([item.speed for item in envelopes], [0.0, 1.0])

    def test_drive_branch_uses_interpolated_zero_ax_edge_not_coast_point(self) -> None:
        envelope = _FakeEnvelope(
            speed=10.0,
            ay=(-2.0, -1.0, 0.0, 1.0, 2.0),
            ax_accel=(-0.2, 0.8, 1.0, 0.8, -0.2),
            ax_brake=(-0.2, -1.0, -1.2, -1.0, -0.2),
        )
        adapted = adapt_envelope_for_lapsims(envelope)
        ay = list(adapted.ay)
        accel = list(adapted.ax_accel)
        self.assertIn(-1.8, ay)
        self.assertIn(1.8, ay)
        self.assertAlmostEqual(accel[ay.index(-1.8)], 0.0)
        self.assertAlmostEqual(accel[ay.index(1.8)], 0.0)
        self.assertTrue(math.isnan(accel[ay.index(-2.0)]))
        self.assertTrue(math.isnan(accel[ay.index(2.0)]))

    def test_drive_branch_discards_islands_disconnected_from_zero_ay(self) -> None:
        envelope = _FakeEnvelope(
            speed=0.1,
            ay=(-2.0, -1.0, 0.0, 1.0, 2.0),
            ax_accel=(0.5, math.nan, 1.0, math.nan, 0.5),
            ax_brake=(-0.2, -0.4, -0.6, -0.4, -0.2),
        )

        adapted = adapt_envelope_for_lapsims(envelope)

        self.assertTrue(math.isnan(adapted.ax_accel[0]))
        self.assertTrue(math.isnan(adapted.ax_accel[1]))
        self.assertEqual(adapted.ax_accel[2], 1.0)
        self.assertTrue(math.isnan(adapted.ax_accel[3]))
        self.assertTrue(math.isnan(adapted.ax_accel[4]))
        self.assertEqual(tuple(adapted.ax_brake), envelope.ax_brake)

    def test_model_identity_output_roots_and_fingerprints_are_stable(self) -> None:
        family, key = model_identity(6)
        self.assertEqual(family, "dyn_py_reduced_order_qss")
        self.assertEqual(key, "dyn_py_6dof_qss")
        base = Path("outputs") / "ggv_cg_smoke"
        self.assertEqual(
            default_output_root(base, 6),
            Path("outputs") / "ggv_cg_smoke_dyn_py_6dof",
        )
        self.assertEqual(
            cache_fingerprint({"b": [2, 3], "a": 1}),
            cache_fingerprint({"a": 1, "b": [2, 3]}),
        )

    def test_tire_mu_scale_resolves_one_study_wide_source(self) -> None:
        self.assertEqual(
            _resolve_tire_mu_scale(None, None),
            (1.0, "default_raw_tir"),
        )
        self.assertEqual(
            _resolve_tire_mu_scale(None, {"tire_mu_scale": 0.6225}),
            (0.6225, "sweep_json"),
        )
        self.assertEqual(
            _resolve_tire_mu_scale(0.6225, {"tire_mu_scale": 0.6225}),
            (0.6225, "command_line"),
        )
        with self.assertRaisesRegex(ValueError, "conflicts"):
            _resolve_tire_mu_scale(0.7, {"tire_mu_scale": 0.8})
        for invalid in (0.0, -0.1, math.nan, math.inf, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                _resolve_tire_mu_scale(invalid, None)

    def test_tire_mu_scale_matches_prior_envelopesim_coefficient_semantics(
        self,
    ) -> None:
        scale = 0.6225437130779028
        raw = _FakeGgvVehicle()
        scaled = _scale_ggv_tire_peak_parameters(raw, scale)

        self.assertAlmostEqual(scaled.pdx1, raw.pdx1 * scale)
        self.assertAlmostEqual(scaled.pdx2, raw.pdx2 * scale)
        self.assertAlmostEqual(scaled.pdy1, raw.pdy1 * scale)
        self.assertAlmostEqual(scaled.pdy2, raw.pdy2 * scale)
        self.assertAlmostEqual(scaled.mu_min, raw.mu_min * scale)
        self.assertEqual(scaled.stiffness_marker, raw.stiffness_marker)
        self.assertIs(_scale_ggv_tire_peak_parameters(raw, 1.0), raw)
        self.assertEqual(_tire_mu_fingerprint_fields(1.0), {})
        self.assertEqual(
            _tire_mu_fingerprint_fields(scale),
            {"tire_mu_scale": scale},
        )
        self.assertIn("vehicle.yml and TIR remain unchanged", _tire_mu_source(scale))

    def test_drive_power_limit_resolves_one_study_wide_source(self) -> None:
        self.assertEqual(
            _resolve_drive_power_limit_w(None, None),
            (None, "live_vehicle_projection"),
        )
        self.assertEqual(
            _resolve_drive_power_limit_w(None, {"drive_power_limit_kw": 32.0}),
            (32_000.0, "sweep_json"),
        )
        self.assertEqual(
            _resolve_drive_power_limit_w(32.0, {"drive_power_limit_kw": 32.0}),
            (32_000.0, "command_line"),
        )
        with self.assertRaisesRegex(ValueError, "conflicts"):
            _resolve_drive_power_limit_w(
                31.0,
                {"drive_power_limit_kw": 32.0},
            )
        for invalid in (0.0, -1.0, math.nan, math.inf, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                _resolve_drive_power_limit_w(invalid, None)

    def test_existing_ggv_recovery_metadata_rejects_input_mismatch(self) -> None:
        metadata = {
            "model_key": "dyn_py_3dof_qss",
            "model_dof": 3,
            "brake_distribution_front": 0.81,
            "wheel_load_min_n": 180.0,
        }
        validated = _validate_existing_case_metadata(
            metadata,
            expected_fields={
                "model_key": "dyn_py_3dof_qss",
                "model_dof": 3,
                "brake_distribution_front": 0.81,
            },
        )
        self.assertIs(validated, metadata)
        self.assertEqual(_finite_metadata_float(metadata, "wheel_load_min_n"), 180.0)

        with self.assertRaisesRegex(ValueError, "brake_distribution_front"):
            _validate_existing_case_metadata(
                metadata,
                expected_fields={"brake_distribution_front": 0.82},
            )
        with self.assertRaisesRegex(ValueError, "missing"):
            _validate_existing_case_metadata(
                metadata,
                expected_fields={"ggv_speed_slices": [0.0, 0.1]},
            )

    def test_existing_ggv_recovery_accepts_only_optional_empty_terminal_slice(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            expected = (0.0, 0.1, 1.0, 2.0, 2.05)
            terminal_empty = root / "terminal_empty.csv"
            pd.DataFrame({"speed_mps": expected[:-1]}).to_csv(
                terminal_empty,
                index=False,
            )
            self.assertEqual(
                _validate_existing_ggv_speed_grid(
                    terminal_empty,
                    expected_speeds=expected,
                ),
                expected[:-1],
            )

            interior_missing = root / "interior_missing.csv"
            pd.DataFrame({"speed_mps": (0.0, 0.1, 2.0, 2.05)}).to_csv(
                interior_missing,
                index=False,
            )
            with self.assertRaisesRegex(ValueError, "dense grid"):
                _validate_existing_ggv_speed_grid(
                    interior_missing,
                    expected_speeds=expected,
                )


class GgvCgSmokeScoringTest(unittest.TestCase):
    def test_fixed_reference_matches_official_2026_field_counts(self) -> None:
        events = _load_fsae_2026_results_reference()["events"]
        expected = {
            "acceleration": (34, 3.697),
            "skidpad": (38, 4.782),
            "autocross": (34, 43.937),
            "michigan_endurance": (23, 1312.281),
        }
        for event_slug, (count, fastest) in expected.items():
            event = events[event_slug]
            self.assertEqual(len(event["valid_adjusted_times_s"]), count)
            self.assertAlmostEqual(event["fastest_adjusted_time_s"], fastest)

    def test_scores_against_fastest_combined_real_or_simulated_result(self) -> None:
        event_times = {
            "acceleration": {"low": 3.6, "baseline": 4.4, "high": 4.8},
            "skidpad": {"low": 5.6, "baseline": 5.2, "high": 4.7},
            "autocross": {"low": 50.0, "baseline": 52.0, "high": 43.0},
            "michigan_endurance": {
                "low": 61.0,
                "baseline": 60.0,
                "high": 59.0,
            },
        }
        input_rows = [
            {
                "event_slug": event_slug,
                "cg_case": cg_case,
                "lap_time_s": lap_time,
            }
            for event_slug, case_times in event_times.items()
            for cg_case, lap_time in case_times.items()
        ]
        for row in input_rows:
            row["track_length_m"] = (
                1069.968773 if row["event_slug"] == "michigan_endurance" else 75.0
            )

        scored = _score_rows(input_rows)

        for event_slug, case_times in event_times.items():
            rule_slug = (
                "endurance" if event_slug == "michigan_endurance" else event_slug
            )
            rule = EVENT_RULES[rule_slug]
            baseline_time = case_times["baseline"]
            event_rows = [row for row in scored if row["event_slug"] == event_slug]
            effective_tmin = min(
                rule.tmin_s,
                min(float(row["projected_competition_time_s"]) for row in event_rows),
            )
            effective_rule = replace(rule, tmin_s=effective_tmin)
            for row in event_rows:
                lap_time = case_times[row["cg_case"]]
                self.assertAlmostEqual(row["baseline_lap_time_s"], baseline_time)
                self.assertAlmostEqual(row["delta_time_s"], lap_time - baseline_time)
                self.assertAlmostEqual(
                    row["delta_time_pct"],
                    100.0 * (lap_time / baseline_time - 1.0),
                )
                multiplier = (
                    22000.0 / 1069.968773 if event_slug == "michigan_endurance" else 1.0
                )
                scoring_time = lap_time * multiplier
                self.assertAlmostEqual(
                    row["projected_competition_time_s"], scoring_time
                )
                is_fastest = abs(scoring_time - effective_tmin) <= 1e-12
                expected = (
                    rule.maximum_points
                    if is_fastest
                    else effective_rule.score(scoring_time)
                )
                self.assertAlmostEqual(row["projected_points"], expected)
                self.assertEqual(
                    row["at_or_faster_than_fsae_2026_winner"],
                    scoring_time <= rule.tmin_s,
                )
                self.assertEqual(row["is_effective_event_fastest"], is_fastest)
                self.assertAlmostEqual(row["effective_points_tmin_s"], effective_tmin)

            baseline = next(row for row in event_rows if row["cg_case"] == "baseline")
            self.assertAlmostEqual(baseline["delta_time_s"], 0.0)
            self.assertAlmostEqual(baseline["delta_time_pct"], 0.0)

    def test_only_fastest_combined_field_time_receives_maximum_points(self) -> None:
        rows = [
            {"event_slug": "acceleration", "cg_case": "low", "lap_time_s": 3.6},
            {
                "event_slug": "acceleration",
                "cg_case": "baseline",
                "lap_time_s": 3.697,
            },
            {"event_slug": "acceleration", "cg_case": "high", "lap_time_s": 3.8},
        ]

        scored = _score_rows(rows)
        winner = next(row for row in scored if row["cg_case"] == "low")
        self.assertTrue(winner["is_effective_event_fastest"])
        self.assertAlmostEqual(
            winner["projected_points"],
            EVENT_RULES["acceleration"].maximum_points,
        )
        real_winner_tie = next(row for row in scored if row["cg_case"] == "baseline")
        self.assertTrue(real_winner_tie["at_or_faster_than_fsae_2026_winner"])
        self.assertFalse(real_winner_tie["is_effective_event_fastest"])
        self.assertLess(
            real_winner_tie["projected_points"],
            EVENT_RULES["acceleration"].maximum_points,
        )
        slower = next(row for row in scored if row["cg_case"] == "high")
        self.assertFalse(slower["at_or_faster_than_fsae_2026_winner"])
        self.assertLess(
            slower["projected_points"],
            EVENT_RULES["acceleration"].maximum_points,
        )

    def test_identical_fastest_simulated_times_can_share_maximum(self) -> None:
        rows = [
            {"event_slug": "acceleration", "cg_case": "low", "lap_time_s": 3.6},
            {
                "event_slug": "acceleration",
                "cg_case": "baseline",
                "lap_time_s": 3.8,
            },
            {"event_slug": "acceleration", "cg_case": "high", "lap_time_s": 3.6},
        ]

        scored = _score_rows(rows)
        winners = [row for row in scored if row["is_effective_event_fastest"]]
        self.assertEqual({row["cg_case"] for row in winners}, {"low", "high"})
        for winner in winners:
            self.assertAlmostEqual(
                winner["projected_points"],
                EVENT_RULES["acceleration"].maximum_points,
            )

    def test_endurance_scales_modeled_pace_to_22_km_zero_penalty_total(self) -> None:
        rows = [
            {
                "event_slug": "michigan_endurance",
                "cg_case": "low",
                "lap_time_s": 58.0,
                "track_length_m": 1069.968773,
            },
            {
                "event_slug": "michigan_endurance",
                "cg_case": "baseline",
                "lap_time_s": 60.0,
                "track_length_m": 1069.968773,
            },
            {
                "event_slug": "michigan_endurance",
                "cg_case": "high",
                "lap_time_s": 65.0,
                "track_length_m": 1069.968773,
            },
        ]

        scored = _score_rows(rows)
        multiplier = 22000.0 / 1069.968773
        for row in scored:
            self.assertAlmostEqual(row["simulation_time_multiplier"], multiplier)
            self.assertAlmostEqual(
                row["projected_competition_time_s"],
                multiplier * row["lap_time_s"],
            )
        winners = [row for row in scored if row["is_effective_event_fastest"]]
        self.assertEqual([row["cg_case"] for row in winners], ["low"])
        for row in scored:
            if row["cg_case"] != "low":
                self.assertLess(
                    row["projected_points"],
                    EVENT_RULES["endurance"].maximum_points,
                )


class GgvCgSmokeCaseTuningTest(unittest.TestCase):
    def _write_payload(self, payload: object) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "case_tuning.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_loads_exact_three_case_tuning(self) -> None:
        payload = {
            "method": "test fixture",
            "cases": {
                "low": {"lltd": 0.64, "brake_distribution_front": 0.66},
                "baseline": {"lltd": 0.60, "brake_distribution_front": 0.70},
                "high": {"lltd": 0.58, "brake_distribution_front": 0.74},
            },
        }
        path = self._write_payload(payload)
        self.assertEqual(_load_case_tuning(path), payload)

    def test_rejects_missing_case_or_out_of_range_value(self) -> None:
        missing = self._write_payload(
            {
                "cases": {
                    "low": {"lltd": 0.64, "brake_distribution_front": 0.66},
                    "baseline": {
                        "lltd": 0.60,
                        "brake_distribution_front": 0.70,
                    },
                }
            }
        )
        with self.assertRaisesRegex(ValueError, "exactly low, baseline, and high"):
            _load_case_tuning(missing)

        out_of_range = self._write_payload(
            {
                "cases": {
                    "low": {"lltd": 1.01, "brake_distribution_front": 0.66},
                    "baseline": {
                        "lltd": 0.60,
                        "brake_distribution_front": 0.70,
                    },
                    "high": {"lltd": 0.58, "brake_distribution_front": 0.74},
                }
            }
        )
        with self.assertRaisesRegex(ValueError, "low.lltd"):
            _load_case_tuning(out_of_range)


class GgvCgSweepDefinitionTest(unittest.TestCase):
    def _write_payload(self, payload: object) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "sweep.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def test_loads_arbitrary_absolute_heights_and_tuning(self) -> None:
        heights_in = [8.0, 9.2, 10.4, 11.6, 12.8, 14.0]
        payload = {
            "reference_case": "cg_11p6in",
            "cases": [
                {
                    "name": f"cg_{height:g}in".replace(".", "p"),
                    "cg_height_in": height,
                    "lltd": 0.60,
                    "brake_distribution_front": 0.70,
                }
                for height in heights_in
            ],
        }
        raw, cases, reference_case = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=0.28,
        )

        self.assertEqual(raw, payload)
        self.assertEqual(reference_case, "cg_11p6in")
        self.assertEqual(len(cases), 6)
        self.assertAlmostEqual(cases[0]["cg_height_m"], 8.0 * 0.0254)
        self.assertAlmostEqual(cases[-1]["cg_height_in"], 14.0)
        self.assertAlmostEqual(cases[2]["tuning"]["lltd"], 0.60)

    def test_loads_explicit_reduced_model_antiroll_fraction(self) -> None:
        payload = {
            "reference_case": "low",
            "cases": [
                {
                    "name": "low",
                    "cg_height_in": 8.0,
                    "front_antiroll_stiffness_fraction": 0.72,
                    "brake_distribution_front": 0.66,
                },
                {
                    "name": "high",
                    "cg_height_in": 14.0,
                    "front_antiroll_stiffness_fraction": 0.44,
                    "brake_distribution_front": 0.75,
                },
            ],
        }
        _, cases, _ = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=0.28,
        )
        self.assertAlmostEqual(
            cases[0]["tuning"]["front_antiroll_stiffness_fraction"], 0.72
        )
        self.assertNotIn("lltd", cases[0]["tuning"])

    def test_chooses_height_nearest_nominal_when_reference_omitted(self) -> None:
        payload = {
            "cases": [
                {"name": "eight", "cg_height_in": 8.0},
                {"name": "eleven", "cg_height_in": 11.0},
                {"name": "fourteen", "cg_height_in": 14.0},
            ]
        }
        _, _, reference_case = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=10.8 * 0.0254,
        )
        self.assertEqual(reference_case, "eleven")

    def test_loads_fixed_height_sprung_mass_sweep(self) -> None:
        nominal_sprung_mass_kg = 230.0
        masses = [190.0, 210.0, 230.0, 250.0, 270.0]
        payload = {
            "sweep_axis": "sprung_mass_kg",
            "cases": [
                {
                    "name": f"mass_{index}",
                    "cg_height_in": 11.5,
                    "sprung_mass_kg": mass,
                    "front_antiroll_stiffness_fraction": 0.55,
                    "brake_distribution_front": 0.72,
                }
                for index, mass in enumerate(masses)
            ],
        }
        _, cases, reference_case = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=0.28,
            nominal_sprung_mass_kg=nominal_sprung_mass_kg,
        )

        self.assertEqual(reference_case, "mass_2")
        self.assertTrue(all(case["sweep_axis"] == "sprung_mass_kg" for case in cases))
        self.assertTrue(all(case["cg_height_in"] == 11.5 for case in cases))
        self.assertEqual([case["sprung_mass_kg"] for case in cases], masses)
        self.assertAlmostEqual(cases[2]["mass_offset_lb"], 0.0)

    def test_mass_sweep_rejects_varying_cg_height(self) -> None:
        payload = {
            "cases": [
                {"name": "low", "cg_height_in": 11.5, "sprung_mass_kg": 200.0},
                {"name": "high", "cg_height_in": 11.6, "sprung_mass_kg": 250.0},
            ]
        }
        with self.assertRaisesRegex(ValueError, "hold total CG height fixed"):
            _load_sweep_definition(
                self._write_payload(payload),
                nominal_cg_height_m=0.28,
                nominal_sprung_mass_kg=230.0,
            )

    def test_loads_fixed_height_longitudinal_cg_sweep(self) -> None:
        payload = {
            "sweep_axis": "rear_static_weight_fraction",
            "cases": [
                {
                    "name": f"rear_{100 * fraction:g}pct",
                    "cg_height_in": 11.5,
                    "rear_static_weight_fraction": fraction,
                    "total_mass_kg": 260.0,
                    "front_antiroll_stiffness_fraction": 0.55,
                    "brake_distribution_front": 0.72,
                }
                for fraction in (0.45, 0.50, 0.55)
            ],
        }
        _, cases, reference_case = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=0.28,
            nominal_sprung_mass_kg=230.0,
            nominal_rear_static_weight_fraction=0.49,
        )

        self.assertEqual(reference_case, "rear_50pct")
        self.assertTrue(
            all(case["sweep_axis"] == "rear_static_weight_fraction" for case in cases)
        )
        self.assertEqual(
            [case["rear_static_weight_fraction"] for case in cases],
            [0.45, 0.50, 0.55],
        )
        self.assertTrue(all(case["cg_height_in"] == 11.5 for case in cases))

    def test_loads_explicit_two_coordinate_configuration_comparison(self) -> None:
        payload = {
            "sweep_axis": "configuration",
            "reference_case": "config_nominal_54",
            "cases": [
                {
                    "name": "config_nominal_54",
                    "cg_height_in": 11.5,
                    "sprung_mass_kg": 230.72288408,
                    "total_mass_kg": 261.07265114,
                    "rear_static_weight_fraction": 0.54,
                    "front_antiroll_stiffness_fraction": 0.64,
                    "brake_distribution_front": 0.70,
                },
                {
                    "name": "config_plus0p25_56p5",
                    "cg_height_in": 11.75,
                    "sprung_mass_kg": 230.72288408,
                    "total_mass_kg": 261.07265114,
                    "rear_static_weight_fraction": 0.565,
                    "front_antiroll_stiffness_fraction": 0.68,
                    "brake_distribution_front": 0.68,
                },
            ],
        }

        raw, cases, reference = _load_sweep_definition(
            self._write_payload(payload),
            nominal_cg_height_m=0.28,
            nominal_sprung_mass_kg=230.72288408,
            nominal_rear_static_weight_fraction=0.49,
        )

        self.assertEqual(raw, payload)
        self.assertEqual(reference, "config_nominal_54")
        self.assertTrue(all(case["sweep_axis"] == "configuration" for case in cases))
        self.assertEqual([case["cg_height_in"] for case in cases], [11.5, 11.75])
        self.assertEqual(
            [case["rear_static_weight_fraction"] for case in cases],
            [0.54, 0.565],
        )
        self.assertTrue(all(abs(case["mass_offset_lb"]) < 1e-12 for case in cases))

    def test_configuration_comparison_requires_explicit_reference(self) -> None:
        payload = {
            "sweep_axis": "configuration",
            "cases": [
                {
                    "name": "first",
                    "cg_height_in": 11.5,
                    "rear_static_weight_fraction": 0.54,
                },
                {
                    "name": "second",
                    "cg_height_in": 11.75,
                    "rear_static_weight_fraction": 0.565,
                },
            ],
        }

        with self.assertRaisesRegex(ValueError, "explicit reference_case"):
            _load_sweep_definition(
                self._write_payload(payload),
                nominal_cg_height_m=0.28,
                nominal_rear_static_weight_fraction=0.49,
            )

    def test_rejects_partial_tuning_and_duplicate_names(self) -> None:
        partial_tuning = {
            "cases": [
                {
                    "name": "low",
                    "cg_height_in": 8.0,
                    "lltd": 0.60,
                    "brake_distribution_front": 0.70,
                },
                {"name": "high", "cg_height_in": 14.0},
            ]
        }
        with self.assertRaisesRegex(ValueError, "every case"):
            _load_sweep_definition(
                self._write_payload(partial_tuning),
                nominal_cg_height_m=0.28,
            )

        duplicate_names = {
            "cases": [
                {"name": "same", "cg_height_in": 8.0},
                {"name": "same", "cg_height_in": 14.0},
            ]
        }
        with self.assertRaisesRegex(ValueError, "duplicated"):
            _load_sweep_definition(
                self._write_payload(duplicate_names),
                nominal_cg_height_m=0.28,
            )

    def test_arbitrary_reference_case_drives_raw_time_deltas(self) -> None:
        rows = [
            {"event_slug": "acceleration", "cg_case": "eight", "lap_time_s": 4.0},
            {"event_slug": "acceleration", "cg_case": "ten", "lap_time_s": 3.9},
            {
                "event_slug": "acceleration",
                "cg_case": "twelve",
                "lap_time_s": 3.8,
            },
        ]
        scored = _score_rows(rows, reference_case="ten")
        by_case = {row["cg_case"]: row for row in scored}
        self.assertAlmostEqual(by_case["ten"]["delta_time_s"], 0.0)
        self.assertAlmostEqual(by_case["eight"]["delta_time_s"], 0.1)
        self.assertAlmostEqual(by_case["twelve"]["delta_time_s"], -0.1)
        self.assertTrue(all(row["reference_cg_case"] == "ten" for row in scored))

    def test_points_correlation_reports_linear_slope(self) -> None:
        totals = [
            {"cg_height_in": 8.0, "projected_timed_event_points": 500.0},
            {"cg_height_in": 10.0, "projected_timed_event_points": 496.0},
            {"cg_height_in": 12.0, "projected_timed_event_points": 492.0},
        ]
        correlation = _points_correlation(totals)
        self.assertAlmostEqual(correlation["linear_slope_points_per_in"], -2.0)
        self.assertAlmostEqual(correlation["pearson_r"], -1.0)
        self.assertAlmostEqual(correlation["linear_r_squared"], 1.0)
        self.assertAlmostEqual(correlation["spearman_rho"], -1.0)
        self.assertAlmostEqual(correlation["quadratic_r_squared"], 1.0)
        self.assertEqual(correlation["adjacent_slopes_points_per_in"], [-2.0, -2.0])

    def test_mass_points_correlation_reports_points_per_lb(self) -> None:
        totals = [
            {
                "sprung_mass_kg": mass_lb * 0.45359237,
                "projected_timed_event_points": 500.0 - 0.2 * mass_lb,
            }
            for mass_lb in (400.0, 450.0, 500.0, 550.0, 600.0)
        ]
        correlation = _points_correlation(
            totals,
            sweep_axis="sprung_mass_kg",
        )

        self.assertAlmostEqual(correlation["linear_slope_points_per_lb"], -0.2)
        self.assertAlmostEqual(correlation["linear_r_squared"], 1.0)
        self.assertEqual(correlation["sweep_axis"], "sprung_mass_kg")

    def test_longitudinal_cg_correlation_reports_points_per_rear_percent(
        self,
    ) -> None:
        totals = [
            {
                "rear_static_weight_fraction": rear_percent / 100.0,
                "projected_timed_event_points": 400.0 + 2.0 * rear_percent,
            }
            for rear_percent in (45.0, 50.0, 55.0)
        ]
        correlation = _points_correlation(
            totals,
            sweep_axis="rear_static_weight_fraction",
        )

        self.assertAlmostEqual(
            correlation["linear_slope_points_per_rear_weight_pct"], 2.0
        )
        self.assertAlmostEqual(correlation["linear_r_squared"], 1.0)
        self.assertEqual(correlation["sweep_axis"], "rear_static_weight_fraction")

    def test_configuration_comparison_does_not_report_false_axis_correlation(
        self,
    ) -> None:
        totals = [
            {
                "cg_case": "config_nominal_54",
                "is_reference_case": True,
                "cg_height_in": 11.5,
                "rear_static_weight_fraction": 0.54,
                "sprung_mass_kg": 230.72288408,
                "projected_timed_event_points": 500.0,
            },
            {
                "cg_case": "config_plus0p25_56p5",
                "is_reference_case": False,
                "cg_height_in": 11.75,
                "rear_static_weight_fraction": 0.565,
                "sprung_mass_kg": 230.72288408,
                "projected_timed_event_points": 503.5,
            },
        ]

        comparison = _points_correlation(totals, sweep_axis="configuration")

        self.assertFalse(comparison["correlation_computed"])
        self.assertEqual(comparison["reference_case"], "config_nominal_54")
        self.assertAlmostEqual(
            comparison["comparisons"][1]["points_delta_vs_reference"], 3.5
        )
        self.assertNotIn("linear_slope_points_per_axis_unit", comparison)


if __name__ == "__main__":
    unittest.main()
