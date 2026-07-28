"""Analyze accumulator-terminal regenerative power from an endurance CSV.

The source telemetry export used by this project has a six-column header but
eight-column data rows.  The two omitted headings are the cumulative outgoing
and incoming energy counters.  This utility repairs that known schema defect,
validates the time series, and reports time-weighted negative-power metrics.

Power follows the source sign convention: positive is accumulator discharge
and negative is energy entering the accumulator.  Each sample is treated as a
zero-order hold over the interval ending at the next timestamp.  The last
sample has no duration and therefore contributes only its final counter values.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


BASE_HEADER = (
    "time_s",
    "voltage_v",
    "power_kw",
    "max_cell_temp_c",
    "net_energy_kwh",
    "current_a",
)
MISSING_COUNTER_HEADERS = (
    "outgoing_energy_kwh",
    "incoming_energy_kwh",
)
REPAIRED_HEADER = BASE_HEADER + MISSING_COUNTER_HEADERS


@dataclass(frozen=True)
class ParsedTelemetry:
    """Numeric telemetry columns plus schema-repair metadata."""

    columns: dict[str, list[float]]
    original_header: tuple[str, ...]
    repaired_header: tuple[str, ...]
    data_column_count: int
    header_repaired: bool

    @property
    def sample_count(self) -> int:
        return len(self.columns["time_s"])


def sha256_file(path: Path) -> str:
    """Return the lowercase SHA256 digest of *path*."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _numeric_cell(value: str, *, row_number: int, column_name: str) -> float:
    stripped = value.strip()
    if stripped == "":
        return math.nan
    try:
        result = float(stripped)
    except ValueError as exc:
        raise ValueError(
            f"Row {row_number} column {column_name!r} is not numeric: "
            f"{value!r}"
        ) from exc
    if not math.isfinite(result):
        raise ValueError(
            f"Row {row_number} column {column_name!r} must be finite"
        )
    return result


def parse_telemetry_csv(path: Path) -> ParsedTelemetry:
    """Parse the known six-header/eight-data-column telemetry export.

    A normal, already-repaired eight-column file is accepted as well.  Other
    header/data width mismatches are rejected rather than guessed.
    """

    path = path.resolve()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.reader(stream)
        try:
            original_header = tuple(cell.strip() for cell in next(reader))
        except StopIteration as exc:
            raise ValueError("Telemetry CSV is empty") from exc

        raw_rows = [
            (row_number, row)
            for row_number, row in enumerate(reader, start=2)
            if row and any(cell.strip() for cell in row)
        ]

    if not raw_rows:
        raise ValueError("Telemetry CSV contains no data rows")
    row_widths = {len(row) for _, row in raw_rows}
    if len(row_widths) != 1:
        raise ValueError(
            f"Telemetry data rows have inconsistent widths: "
            f"{sorted(row_widths)}"
        )
    data_width = row_widths.pop()

    if original_header == BASE_HEADER and data_width == len(REPAIRED_HEADER):
        repaired_header = REPAIRED_HEADER
        header_repaired = True
    elif (
        original_header == REPAIRED_HEADER
        and data_width == len(REPAIRED_HEADER)
    ):
        repaired_header = REPAIRED_HEADER
        header_repaired = False
    else:
        raise ValueError(
            "Unsupported telemetry schema: expected either the known "
            "six-column header with eight-column rows or the repaired "
            f"eight-column schema; header={original_header!r}, "
            f"data_width={data_width}"
        )

    columns = {name: [] for name in repaired_header}
    for row_number, row in raw_rows:
        for name, value in zip(repaired_header, row, strict=True):
            columns[name].append(
                _numeric_cell(
                    value,
                    row_number=row_number,
                    column_name=name,
                )
            )

    time_s = columns["time_s"]
    if len(time_s) < 2:
        raise ValueError("Telemetry requires at least two time samples")
    if any(not math.isfinite(value) for value in time_s):
        raise ValueError("Telemetry timestamps may not be blank")
    deltas = [right - left for left, right in zip(time_s, time_s[1:])]
    if any(delta <= 0.0 for delta in deltas):
        raise ValueError("Telemetry timestamps must be strictly increasing")
    if any(not math.isfinite(value) for value in columns["power_kw"]):
        raise ValueError("Telemetry power values may not be blank")

    return ParsedTelemetry(
        columns=columns,
        original_header=original_header,
        repaired_header=repaired_header,
        data_column_count=data_width,
        header_repaired=header_repaired,
    )


def negative_power_metrics(
    time_s: Iterable[float],
    power_kw: Iterable[float],
    *,
    threshold_kw: float,
) -> dict[str, Any]:
    """Return left-hold, time-weighted metrics below *threshold_kw*.

    ``threshold_kw`` uses the signed source convention and must be zero or
    negative.  For example, ``-1.0`` selects intervals whose power is strictly
    less than -1 kW.
    """

    if not math.isfinite(threshold_kw) or threshold_kw > 0.0:
        raise ValueError("threshold_kw must be finite and nonpositive")
    times = [float(value) for value in time_s]
    powers = [float(value) for value in power_kw]
    if len(times) != len(powers):
        raise ValueError("time and power arrays must have the same length")
    if len(times) < 2:
        raise ValueError("At least two samples are required")
    if any(not math.isfinite(value) for value in (*times, *powers)):
        raise ValueError("time and power arrays must be finite")

    event_duration_s = times[-1] - times[0]
    if event_duration_s <= 0.0:
        raise ValueError("Event duration must be positive")

    active_duration_s = 0.0
    square_integral_kw2_s = 0.0
    recovered_energy_kw_s = 0.0
    active_interval_count = 0
    peak_negative_power_kw: float | None = None
    for index, (left, right) in enumerate(zip(times, times[1:])):
        dt = right - left
        if dt <= 0.0:
            raise ValueError("Telemetry timestamps must be strictly increasing")
        power = powers[index]
        if power < threshold_kw:
            active_interval_count += 1
            active_duration_s += dt
            square_integral_kw2_s += power**2 * dt
            recovered_energy_kw_s += -power * dt
            if peak_negative_power_kw is None:
                peak_negative_power_kw = power
            else:
                peak_negative_power_kw = min(peak_negative_power_kw, power)

    if active_duration_s > 0.0:
        conditional_rms_kw = math.sqrt(
            square_integral_kw2_s / active_duration_s
        )
    else:
        conditional_rms_kw = 0.0
    whole_event_rms_kw = math.sqrt(
        square_integral_kw2_s / event_duration_s
    )
    peak_value = peak_negative_power_kw or 0.0
    return {
        "selector": f"power_kw < {threshold_kw:g}",
        "threshold_kw": threshold_kw,
        "active_interval_count": active_interval_count,
        "active_duration_s": active_duration_s,
        "duty_cycle_fraction": active_duration_s / event_duration_s,
        "conditional_active_rms_kw": conditional_rms_kw,
        "whole_event_equivalent_rms_kw": whole_event_rms_kw,
        "peak_negative_power_kw": peak_value,
        "peak_regen_magnitude_kw": -peak_value,
        "integrated_recovered_energy_kwh": (
            recovered_energy_kw_s / 3600.0
        ),
    }


def analyze_telemetry(
    path: Path,
    *,
    active_threshold_magnitude_kw: float = 1.0,
) -> dict[str, Any]:
    """Parse and analyze one telemetry export."""

    if (
        not math.isfinite(active_threshold_magnitude_kw)
        or active_threshold_magnitude_kw < 0.0
    ):
        raise ValueError(
            "active_threshold_magnitude_kw must be finite and nonnegative"
        )
    path = path.resolve()
    parsed = parse_telemetry_csv(path)
    time_s = parsed.columns["time_s"]
    power_kw = parsed.columns["power_kw"]
    deltas = [right - left for left, right in zip(time_s, time_s[1:])]

    all_negative = negative_power_metrics(
        time_s,
        power_kw,
        threshold_kw=0.0,
    )
    threshold = -active_threshold_magnitude_kw
    active_negative = negative_power_metrics(
        time_s,
        power_kw,
        threshold_kw=threshold,
    )
    final_counters = {
        "net_energy_kwh": parsed.columns["net_energy_kwh"][-1],
        "outgoing_energy_kwh": parsed.columns["outgoing_energy_kwh"][-1],
        "incoming_energy_kwh": parsed.columns["incoming_energy_kwh"][-1],
    }

    return {
        "schema_version": 1,
        "source": {
            "path": str(path),
            "file_name": path.name,
            "size_bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        },
        "parser": {
            "original_header": list(parsed.original_header),
            "repaired_header": list(parsed.repaired_header),
            "original_header_column_count": len(parsed.original_header),
            "data_column_count": parsed.data_column_count,
            "header_repaired": parsed.header_repaired,
            "repair_interpretation": (
                "Appended outgoing_energy_kwh and incoming_energy_kwh to "
                "the known six-column export header."
                if parsed.header_repaired
                else "No header repair required."
            ),
        },
        "time_weighting": {
            "method": "left_sample_zero_order_hold",
            "interpretation": (
                "Each power sample applies from its timestamp until the next "
                "timestamp; the final sample has zero duration."
            ),
        },
        "event": {
            "sample_count": parsed.sample_count,
            "interval_count": parsed.sample_count - 1,
            "start_time_s": time_s[0],
            "end_time_s": time_s[-1],
            "duration_s": time_s[-1] - time_s[0],
            "median_interval_s": statistics.median(deltas),
            "minimum_interval_s": min(deltas),
            "maximum_interval_s": max(deltas),
        },
        "all_negative_power": all_negative,
        "active_negative_power": active_negative,
        "final_energy_counters": {
            **final_counters,
            "outgoing_minus_incoming_kwh": (
                final_counters["outgoing_energy_kwh"]
                - final_counters["incoming_energy_kwh"]
            ),
            "counter_balance_residual_kwh": (
                final_counters["net_energy_kwh"]
                - (
                    final_counters["outgoing_energy_kwh"]
                    - final_counters["incoming_energy_kwh"]
                )
            ),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute time-weighted regenerative-power metrics from the "
            "malformed endurance telemetry export."
        )
    )
    parser.add_argument("csv_path", type=Path)
    parser.add_argument(
        "--active-threshold-kw",
        type=float,
        default=1.0,
        help=(
            "Positive regen magnitude defining active samples; 1.0 selects "
            "source power strictly below -1 kW."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON output path; stdout is always populated.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = analyze_telemetry(
        args.csv_path,
        active_threshold_magnitude_kw=args.active_threshold_kw,
    )
    rendered = json.dumps(result, indent=2, allow_nan=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
