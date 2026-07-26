"""Unit tests for the standalone regenerative-telemetry analyzer."""

from __future__ import annotations

import hashlib
import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_regen_telemetry import (  # noqa: E402
    REPAIRED_HEADER,
    analyze_telemetry,
    negative_power_metrics,
    parse_telemetry_csv,
)


class NegativePowerMetricTest(unittest.TestCase):
    def test_time_weighted_rms_duty_peak_and_energy(self) -> None:
        time_s = [0.0, 1.0, 3.0, 4.0]
        power_kw = [-0.5, -2.0, 1.0, 0.0]

        all_negative = negative_power_metrics(
            time_s,
            power_kw,
            threshold_kw=0.0,
        )
        active = negative_power_metrics(
            time_s,
            power_kw,
            threshold_kw=-1.0,
        )

        self.assertAlmostEqual(all_negative["active_duration_s"], 3.0)
        self.assertAlmostEqual(all_negative["duty_cycle_fraction"], 0.75)
        self.assertAlmostEqual(
            all_negative["conditional_active_rms_kw"],
            math.sqrt(8.25 / 3.0),
        )
        self.assertAlmostEqual(
            all_negative["whole_event_equivalent_rms_kw"],
            math.sqrt(8.25 / 4.0),
        )
        self.assertAlmostEqual(
            all_negative["integrated_recovered_energy_kwh"],
            4.5 / 3600.0,
        )
        self.assertEqual(all_negative["peak_negative_power_kw"], -2.0)
        self.assertEqual(all_negative["peak_regen_magnitude_kw"], 2.0)

        self.assertEqual(active["active_interval_count"], 1)
        self.assertAlmostEqual(active["active_duration_s"], 2.0)
        self.assertAlmostEqual(active["duty_cycle_fraction"], 0.5)
        self.assertAlmostEqual(active["conditional_active_rms_kw"], 2.0)
        self.assertAlmostEqual(
            active["whole_event_equivalent_rms_kw"],
            math.sqrt(2.0),
        )
        self.assertAlmostEqual(
            active["integrated_recovered_energy_kwh"],
            4.0 / 3600.0,
        )

    def test_empty_selection_returns_zero_metrics(self) -> None:
        result = negative_power_metrics(
            [0.0, 1.0, 2.0],
            [1.0, 0.0, 0.0],
            threshold_kw=-1.0,
        )
        self.assertEqual(result["active_interval_count"], 0)
        self.assertEqual(result["active_duration_s"], 0.0)
        self.assertEqual(result["conditional_active_rms_kw"], 0.0)
        self.assertEqual(result["whole_event_equivalent_rms_kw"], 0.0)
        self.assertEqual(result["integrated_recovered_energy_kwh"], 0.0)


class MalformedCsvTest(unittest.TestCase):
    def test_repairs_six_header_eight_data_columns(self) -> None:
        content = (
            "time_s,voltage_v,power_kw,max_cell_temp_c,"
            "net_energy_kwh,current_a\n"
            "0,500,-2,,0,-4,0,0\n"
            "1,501,4,,0.001,8,0.0015,0.0005\n"
            "2,500,0,,0.001,0,0.0015,0.0005\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "malformed.csv"
            path.write_text(content, encoding="utf-8")
            expected_hash = hashlib.sha256(path.read_bytes()).hexdigest()

            parsed = parse_telemetry_csv(path)
            result = analyze_telemetry(path)

        self.assertTrue(parsed.header_repaired)
        self.assertEqual(parsed.repaired_header, REPAIRED_HEADER)
        self.assertEqual(parsed.data_column_count, 8)
        self.assertEqual(parsed.columns["incoming_energy_kwh"][-1], 0.0005)
        self.assertEqual(result["source"]["sha256"], expected_hash)
        self.assertEqual(result["parser"]["original_header_column_count"], 6)
        self.assertEqual(result["parser"]["data_column_count"], 8)
        self.assertTrue(result["parser"]["header_repaired"])
        self.assertEqual(
            result["final_energy_counters"]["incoming_energy_kwh"],
            0.0005,
        )
        self.assertAlmostEqual(
            result["final_energy_counters"]["counter_balance_residual_kwh"],
            0.0,
        )

    def test_rejects_unknown_header_width_mismatch(self) -> None:
        content = "time_s,power_kw\n0,-1,extra\n1,0,extra\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unsupported.csv"
            path.write_text(content, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Unsupported telemetry"):
                parse_telemetry_csv(path)


if __name__ == "__main__":
    unittest.main()
