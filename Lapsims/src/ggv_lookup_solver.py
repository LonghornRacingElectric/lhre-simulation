"""Forward/backward track solver driven by an EnvelopeSim GGV boundary.

The GGV CSV is the vehicle model. Aero, tire, load-transfer, braking, and
powertrain limits must already be represented in its acceleration and braking
boundaries; this module deliberately does not apply any of them a second time.

The velocity-envelope iteration follows the contained OpenLAP-equation port's
track conventions so the existing event meshes and time formulas remain
directly comparable. The legacy correlated solver is not imported or changed.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

REQUIRED_GGV_COLUMNS = {
    "speed_mps",
    "ay_mps2",
    "ax_accel_mps2",
    "ax_brake_mps2",
    "accel_feasible",
    "brake_feasible",
}
REQUIRED_TRACK_COLUMNS = {"dx_m", "curvature_1pm"}
LATERAL_EDGE_FIT_POINTS = 4
ZERO_AX_EDGE_TOLERANCE_MPS2 = 1e-9
ZERO_LATERAL_DEMAND_TOLERANCE_MPS2 = 1e-10


@dataclass(frozen=True)
class _Boundary:
    ay_mps2: np.ndarray
    ax_mps2: np.ndarray
    positive_ay_limit_mps2: float
    negative_ay_limit_mps2: float

    def value_at_fraction(self, signed_fraction: float) -> float:
        """Interpolate at a signed fraction of this slice's lateral limit."""

        fraction = float(np.clip(signed_fraction, -1.0, 1.0))
        if fraction >= 0.0:
            target_ay = fraction * self.positive_ay_limit_mps2
        else:
            target_ay = -abs(fraction) * self.negative_ay_limit_mps2
        return float(np.interp(target_ay, self.ay_mps2, self.ax_mps2))


@dataclass(frozen=True)
class _GGVSlice:
    speed_mps: float
    accel: _Boundary | None
    brake: _Boundary | None


class GGVMap:
    """Validated, interpolated EnvelopeSim GGV acceleration boundary."""

    def __init__(self, slices: list[_GGVSlice], source_path: Path):
        if not slices:
            raise ValueError("GGV map contains no speed slices")
        self.source_path = source_path
        self.source_sha256 = hashlib.sha256(source_path.read_bytes()).hexdigest()
        self._slices = tuple(sorted(slices, key=lambda item: item.speed_mps))
        self.csv_speed_min_mps = float(self._slices[0].speed_mps)
        self.csv_speed_max_mps = float(self._slices[-1].speed_mps)

        accel_slices = [item for item in self._slices if item.accel is not None]
        brake_slices = [item for item in self._slices if item.brake is not None]
        if not accel_slices:
            raise ValueError("GGV map has no feasible acceleration boundary")
        if not brake_slices:
            raise ValueError("GGV map has no feasible braking boundary")

        self._accel_slices = tuple(accel_slices)
        self._brake_slices = tuple(brake_slices)
        self._accel_speeds = np.asarray(
            [item.speed_mps for item in accel_slices], dtype=float
        )
        self._brake_speeds = np.asarray(
            [item.speed_mps for item in brake_slices], dtype=float
        )

        # A generated slice may extend above the actual tractive-force curve.
        # The highest slice with a feasible drive branch at ay=0 is therefore
        # the usable speed cap rather than the raw maximum CSV speed.
        self.minimum_speed_mps = float(self._accel_speeds[0])
        self.maximum_speed_mps = float(self._accel_speeds[-1])
        if abs(self.minimum_speed_mps) > 1e-12:
            raise ValueError("GGV map must include a 0 m/s speed slice")
        self._validate_branch_speed_coverage("acceleration", self._accel_speeds)
        self._validate_branch_speed_coverage("braking", self._brake_speeds)

        # Sustainable lateral acceleration uses the drive branch: at ax=0 the
        # driven tires still have to cancel aero drag. EnvelopeSim's brake mode
        # deliberately rejects that positive tire-force demand, so intersecting
        # the brake branch would spuriously make cornering brake-bias-limited.
        # A finite drive row means a non-negative ax (including the zero grid
        # point) is feasible. Enforce the generator's configured left/right
        # symmetry by taking the weaker side.
        common_slices = [
            item for item in self._slices if item.speed_mps <= self.maximum_speed_mps
        ]
        self._lateral_speeds = np.asarray(
            [item.speed_mps for item in common_slices], dtype=float
        )
        symmetric_limits = np.asarray(
            [
                min(
                    item.accel.positive_ay_limit_mps2,
                    item.accel.negative_ay_limit_mps2,
                )
                for item in common_slices
            ],
            dtype=float,
        )
        self._lateral_positive_limits = symmetric_limits
        self._lateral_negative_limits = symmetric_limits.copy()

    @classmethod
    def from_csv(cls, path: str | Path) -> GGVMap:
        source_path = Path(path).resolve()
        frame = pd.read_csv(source_path)
        missing = REQUIRED_GGV_COLUMNS.difference(frame.columns)
        if missing:
            raise ValueError(f"GGV CSV is missing required columns: {sorted(missing)}")
        if frame.empty:
            raise ValueError("GGV CSV contains no rows")

        numeric_columns = (
            "speed_mps",
            "ay_mps2",
            "ax_accel_mps2",
            "ax_brake_mps2",
            "accel_feasible",
            "brake_feasible",
        )
        converted = frame.copy()
        for column in numeric_columns:
            converted[column] = pd.to_numeric(converted[column], errors="coerce")
        if converted["speed_mps"].isna().any() or converted["ay_mps2"].isna().any():
            raise ValueError("GGV speed and lateral-acceleration values must be finite")
        if (converted["speed_mps"] < 0.0).any():
            raise ValueError("GGV speeds must be non-negative")
        if converted.duplicated(["speed_mps", "ay_mps2"]).any():
            raise ValueError("GGV CSV has duplicate speed/ay rows")

        slices: list[_GGVSlice] = []
        for speed, group in converted.groupby("speed_mps", sort=True):
            group = group.sort_values("ay_mps2")
            accel = cls._make_boundary(group, "accel")
            brake = cls._make_boundary(group, "brake")
            slices.append(
                _GGVSlice(
                    speed_mps=float(speed),
                    accel=accel,
                    brake=brake,
                )
            )
        return cls(slices, source_path)

    @staticmethod
    def _make_boundary(frame: pd.DataFrame, branch: str) -> _Boundary | None:
        value_column = "ax_accel_mps2" if branch == "accel" else "ax_brake_mps2"
        flag_column = "accel_feasible" if branch == "accel" else "brake_feasible"
        feasible = (frame[flag_column] > 0.5) & np.isfinite(frame[value_column])
        feasible_positions = np.flatnonzero(feasible.to_numpy(dtype=bool))
        if len(feasible_positions) > 1 and np.any(np.diff(feasible_positions) != 1):
            raise ValueError(
                f"GGV {branch} boundary has an internal infeasible gap; "
                "interpolation across NaNs is not allowed"
            )
        branch_frame = frame.loc[feasible, ["ay_mps2", value_column]].copy()
        if branch_frame.empty:
            return None
        branch_frame.sort_values("ay_mps2", inplace=True)
        ay = branch_frame["ay_mps2"].to_numpy(dtype=float)
        ax = branch_frame[value_column].to_numpy(dtype=float)
        if ay[0] > 1e-9 or ay[-1] < -1e-9:
            raise ValueError(
                f"GGV {branch} boundary must include both lateral directions"
            )
        if branch == "accel" and np.nanmin(ax) < -1e-9:
            raise ValueError("GGV acceleration boundary contains negative ax")
        if branch == "brake" and np.nanmax(ax) > 1e-9:
            raise ValueError("GGV braking boundary contains positive ax")

        positive_limit = float(max(ay[-1], 0.0))
        negative_limit = float(max(-ay[0], 0.0))
        if branch == "accel":
            positive_edge = GGVMap._zero_ax_edge(frame, feasible, side="positive")
            negative_edge = GGVMap._zero_ax_edge(frame, feasible, side="negative")
            if positive_edge > positive_limit:
                ay = np.append(ay, positive_edge)
                ax = np.append(ax, 0.0)
                positive_limit = positive_edge
            if negative_edge > negative_limit:
                ay = np.insert(ay, 0, -negative_edge)
                ax = np.insert(ax, 0, 0.0)
                negative_limit = negative_edge
        return _Boundary(
            ay_mps2=ay,
            ax_mps2=ax,
            positive_ay_limit_mps2=positive_limit,
            negative_ay_limit_mps2=negative_limit,
        )

    @staticmethod
    def _zero_ax_edge(
        frame: pd.DataFrame,
        feasible: pd.Series,
        *,
        side: str,
    ) -> float:
        """Estimate the drive boundary's zero-ax lateral intercept.

        EnvelopeSim exports the largest feasible acceleration on a discrete ax
        grid. Its final finite ay row therefore normally retains a small
        positive ax, while the adjacent ay row is infeasible. Fit a local line
        through the last four finite boundary samples and extrapolate to ax=0
        inside that one-row bracket. If actuator distortion pushes that fit
        outside the bracket, shrink the local window to three and then two
        points. This suppresses discrete-ax staircase noise without trusting a
        broad distorted fit. An explicitly serialized zero-ax outer point is
        authoritative. If no fit is bracketed, retain the conservative last
        finite node rather than extrapolating beyond the supplied domain.
        """

        ay = frame["ay_mps2"].to_numpy(dtype=float)
        ax = frame["ax_accel_mps2"].to_numpy(dtype=float)
        feasible_values = feasible.to_numpy(dtype=bool)

        if side == "positive":
            side_positions = np.flatnonzero(feasible_values & (ay >= -1e-12))
            if len(side_positions) < 2:
                return (
                    float(max(ay[side_positions[-1]], 0.0))
                    if len(side_positions)
                    else 0.0
                )
            edge_position = int(side_positions[-1])
            if edge_position + 1 >= len(ay) or feasible_values[edge_position + 1]:
                return float(max(ay[edge_position], 0.0))
            finite_edge = float(ay[edge_position])
            infeasible_edge = float(ay[edge_position + 1])
            direction = 1.0
        elif side == "negative":
            side_positions = np.flatnonzero(feasible_values & (ay <= 1e-12))
            if len(side_positions) < 2:
                return (
                    float(max(-ay[side_positions[0]], 0.0))
                    if len(side_positions)
                    else 0.0
                )
            edge_position = int(side_positions[0])
            if edge_position == 0 or feasible_values[edge_position - 1]:
                return float(max(-ay[edge_position], 0.0))
            finite_edge = float(ay[edge_position])
            infeasible_edge = float(ay[edge_position - 1])
            direction = -1.0
        else:
            raise ValueError(f"Unknown lateral edge side: {side}")

        if ax[edge_position] <= ZERO_AX_EDGE_TOLERANCE_MPS2:
            return abs(finite_edge)

        lower, upper = sorted((finite_edge, infeasible_edge))
        maximum_points = min(LATERAL_EDGE_FIT_POINTS, len(side_positions))
        for point_count in range(maximum_points, 1, -1):
            fit_positions = (
                side_positions[-point_count:]
                if side == "positive"
                else side_positions[:point_count]
            )
            slope, intercept = np.polyfit(ay[fit_positions], ax[fit_positions], 1)
            if not math.isfinite(slope) or direction * slope >= -1e-12:
                continue
            zero_crossing = float(-intercept / slope)
            if lower < zero_crossing < upper:
                return abs(zero_crossing)
        return abs(finite_edge)

    def _validate_branch_speed_coverage(
        self, branch_name: str, branch_speeds: np.ndarray
    ) -> None:
        all_speeds = np.asarray([item.speed_mps for item in self._slices], dtype=float)
        within_cap = all_speeds <= self.maximum_speed_mps + 1e-12
        expected = all_speeds[within_cap]
        actual = branch_speeds[branch_speeds <= self.maximum_speed_mps + 1e-12]
        if len(expected) != len(actual) or not np.allclose(
            expected, actual, rtol=0.0, atol=1e-12
        ):
            raise ValueError(
                f"GGV {branch_name} boundary has an interior missing speed slice; "
                "interpolation across NaNs is not allowed"
            )

    @staticmethod
    def _bracket(
        slices: tuple[_GGVSlice, ...], speeds: np.ndarray, speed_mps: float
    ) -> tuple[_GGVSlice, _GGVSlice, float]:
        speed = float(np.clip(speed_mps, speeds[0], speeds[-1]))
        upper_index = int(np.searchsorted(speeds, speed, side="right"))
        if upper_index == 0:
            return slices[0], slices[0], 0.0
        if upper_index >= len(speeds):
            return slices[-1], slices[-1], 0.0
        lower_index = upper_index - 1
        lower_speed = float(speeds[lower_index])
        upper_speed = float(speeds[upper_index])
        fraction = (speed - lower_speed) / (upper_speed - lower_speed)
        return slices[lower_index], slices[upper_index], float(fraction)

    @staticmethod
    def _side_limit(boundary: _Boundary, ay_mps2: float) -> float:
        if ay_mps2 >= 0.0:
            return boundary.positive_ay_limit_mps2
        return boundary.negative_ay_limit_mps2

    def _boundary_value(self, branch: str, speed_mps: float, ay_mps2: float) -> float:
        if branch == "accel":
            slices = self._accel_slices
            speeds = self._accel_speeds
        else:
            slices = self._brake_slices
            speeds = self._brake_speeds
        lower, upper, speed_fraction = self._bracket(slices, speeds, speed_mps)
        lower_boundary = lower.accel if branch == "accel" else lower.brake
        upper_boundary = upper.accel if branch == "accel" else upper.brake
        if lower_boundary is None or upper_boundary is None:
            return math.nan

        lower_limit = self._side_limit(lower_boundary, ay_mps2)
        upper_limit = self._side_limit(upper_boundary, ay_mps2)
        current_limit = (
            1.0 - speed_fraction
        ) * lower_limit + speed_fraction * upper_limit
        if current_limit <= 0.0:
            if abs(ay_mps2) > ZERO_LATERAL_DEMAND_TOLERANCE_MPS2:
                return math.nan
            # A reduced-order QSS slice can legitimately contain only its
            # straight-line ay=0 solution at very low speed.  Its lateral
            # domain then has zero width, but the stored longitudinal boundary
            # remains authoritative at ay=0 (not zero acceleration).
            lower_value = lower_boundary.value_at_fraction(0.0)
            upper_value = upper_boundary.value_at_fraction(0.0)
            return float(
                (1.0 - speed_fraction) * lower_value
                + speed_fraction * upper_value
            )
        if abs(ay_mps2) > current_limit + 1e-8:
            return math.nan

        signed_fraction = float(np.clip(ay_mps2 / current_limit, -1.0, 1.0))
        lower_value = lower_boundary.value_at_fraction(signed_fraction)
        upper_value = upper_boundary.value_at_fraction(signed_fraction)
        return float(
            (1.0 - speed_fraction) * lower_value + speed_fraction * upper_value
        )

    def acceleration(self, speed_mps: float, ay_mps2: float) -> float:
        """Maximum net forward acceleration at speed and signed ay."""

        if speed_mps > self.maximum_speed_mps + 1e-9:
            return 0.0
        value = self._boundary_value("accel", speed_mps, ay_mps2)
        return max(0.0, value) if math.isfinite(value) else 0.0

    def braking_deceleration(self, speed_mps: float, ay_mps2: float) -> float:
        """Positive magnitude of maximum net braking deceleration."""

        value = self._boundary_value("brake", speed_mps, ay_mps2)
        return max(0.0, -value) if math.isfinite(value) else 0.0

    def lateral_limits(self, speed_mps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return interpolated positive and negative sustainable ay magnitudes."""

        speed = np.clip(
            np.asarray(speed_mps, dtype=float),
            self._lateral_speeds[0],
            self.maximum_speed_mps,
        )
        positive = np.interp(speed, self._lateral_speeds, self._lateral_positive_limits)
        negative = np.interp(speed, self._lateral_speeds, self._lateral_negative_limits)
        return positive, negative

    def lateral_speed_limits(self, curvature_1pm: np.ndarray) -> np.ndarray:
        """Solve v^2*kappa <= GGV lateral capability for each segment."""

        curvature = np.asarray(curvature_1pm, dtype=float)
        limits = np.full_like(curvature, self.maximum_speed_mps, dtype=float)
        turning = np.abs(curvature) > 1e-15
        if not np.any(turning):
            return limits

        active_curvature = curvature[turning]
        lower = np.zeros_like(active_curvature)
        upper = np.full_like(active_curvature, self.maximum_speed_mps)
        for _ in range(64):
            middle = 0.5 * (lower + upper)
            positive, negative = self.lateral_limits(middle)
            ay_limit = np.where(active_curvature >= 0.0, positive, negative)
            feasible = middle**2 * np.abs(active_curvature) <= ay_limit
            lower = np.where(feasible, middle, lower)
            upper = np.where(feasible, upper, middle)
        limits[turning] = lower
        return limits


def _validate_track(track: pd.DataFrame, is_closed: bool) -> None:
    missing = REQUIRED_TRACK_COLUMNS.difference(track.columns)
    if missing:
        raise ValueError(f"Track CSV is missing required columns: {sorted(missing)}")
    if track.empty:
        raise ValueError("Track has no segments")
    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    if not np.all(np.isfinite(dx)) or np.any(dx <= 0.0):
        raise ValueError("Track segment lengths must be finite and positive")
    if not np.all(np.isfinite(curvature)):
        raise ValueError("Track curvature must be finite")
    if is_closed and len(track) < 2:
        raise ValueError("Closed track must contain at least two segments")


def _backward_speed(
    ggv: GGVMap,
    next_speed_mps: float,
    previous_curvature_1pm: float,
    dx_m: float,
    previous_lateral_limit_mps: float,
) -> float:
    proposed = next_speed_mps
    for _ in range(16):
        ay = proposed**2 * previous_curvature_1pm
        deceleration = ggv.braking_deceleration(proposed, ay)
        updated = math.sqrt(max(0.0, next_speed_mps**2 + 2.0 * deceleration * dx_m))
        updated = min(updated, previous_lateral_limit_mps, ggv.maximum_speed_mps)
        if abs(updated - proposed) < 1e-11:
            return updated
        proposed = updated
    return proposed


def _finish_result(
    ggv: GGVMap,
    track: pd.DataFrame,
    speed: np.ndarray,
    lateral_limit: np.ndarray,
    is_closed: bool,
    iteration: int,
    converged: bool,
    max_change: float,
) -> tuple[pd.DataFrame, dict]:
    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    count = len(track)

    if is_closed:
        next_speed = np.roll(speed, -1)
        longitudinal_accel = (next_speed**2 - speed**2) / (2.0 * dx)
        constraint_speed = speed.copy()
        constraint_curvature = curvature.copy()
    else:
        longitudinal_accel = np.empty_like(speed)
        longitudinal_accel[0] = speed[0] ** 2 / (2.0 * dx[0])
        longitudinal_accel[1:] = (speed[1:] ** 2 - speed[:-1] ** 2) / (2.0 * dx[1:])
        # Open-course rows are segment endpoints. The acceleration recorded in
        # row i acts from the preceding endpoint state to endpoint i; row zero
        # starts at the synthetic OpenLAP v=0 point. Keep constraint diagnostics
        # aligned with those segment-start states rather than the row endpoint.
        constraint_speed = np.empty_like(speed)
        constraint_speed[0] = 0.0
        constraint_speed[1:] = speed[:-1]
        constraint_curvature = np.empty_like(curvature)
        constraint_curvature[0] = curvature[0]
        constraint_curvature[1:] = curvature[:-1]
    lateral_accel = speed**2 * curvature
    constraint_lateral_accel = constraint_speed**2 * constraint_curvature
    if not is_closed:
        constraint_lateral_accel[0] = 0.0
    time_in_segment = dx / speed
    if not is_closed:
        time_in_segment[0] *= 2.0
    elapsed_time = np.cumsum(time_in_segment)

    accel_limit = np.asarray(
        [
            ggv.acceleration(v, ay)
            for v, ay in zip(constraint_speed, constraint_lateral_accel)
        ],
        dtype=float,
    )
    brake_limit = -np.asarray(
        [
            ggv.braking_deceleration(v, ay)
            for v, ay in zip(constraint_speed, constraint_lateral_accel)
        ],
        dtype=float,
    )

    result = track.copy()
    result["lateral_speed_limit_mps"] = lateral_limit
    result["speed_mps"] = speed
    result["time_in_segment_s"] = time_in_segment
    result["elapsed_time_s"] = elapsed_time
    result["longitudinal_accel_mps2"] = longitudinal_accel
    result["lateral_accel_mps2"] = lateral_accel
    result["ggv_constraint_speed_mps"] = constraint_speed
    result["ggv_constraint_curvature_1pm"] = constraint_curvature
    result["ggv_constraint_lateral_accel_mps2"] = constraint_lateral_accel
    result["ggv_accel_limit_mps2"] = accel_limit
    result["ggv_brake_limit_mps2"] = brake_limit
    positive_lateral_limit, negative_lateral_limit = ggv.lateral_limits(
        constraint_speed
    )
    signed_lateral_limit = np.where(
        constraint_lateral_accel >= 0.0,
        positive_lateral_limit,
        negative_lateral_limit,
    )
    lateral_utilization = np.divide(
        np.abs(constraint_lateral_accel),
        signed_lateral_limit,
        out=np.zeros_like(constraint_lateral_accel),
        where=signed_lateral_limit > 0.0,
    )
    speed_domain_fraction = constraint_speed / ggv.maximum_speed_mps
    result["ggv_lateral_limit_mps2"] = signed_lateral_limit
    result["ggv_lateral_utilization"] = lateral_utilization
    result["ggv_speed_domain_fraction"] = speed_domain_fraction

    lap_time = float(time_in_segment.sum())
    summary = {
        "solver": "EnvelopeSim GGV lookup, contained forward/backward track solver",
        "lap_time_s": lap_time,
        "track_length_m": float(dx.sum()),
        "segments": count,
        "track_configuration": "closed" if is_closed else "open",
        "standing_start": not is_closed,
        "minimum_speed_mps": float(speed.min()),
        "maximum_speed_mps": float(speed.max()),
        "distance_weighted_average_speed_mps": float(dx.sum() / lap_time),
        "maximum_lateral_accel_mps2": float(np.max(np.abs(lateral_accel))),
        "maximum_longitudinal_accel_mps2": float(longitudinal_accel.max()),
        "maximum_longitudinal_decel_mps2": float(longitudinal_accel.min()),
        "iterations": iteration,
        "converged": converged,
        "final_max_speed_change_mps": max_change,
        "ggv_source_csv": str(ggv.source_path),
        "ggv_source_sha256": ggv.source_sha256,
        "ggv_csv_speed_min_mps": ggv.csv_speed_min_mps,
        "ggv_csv_speed_max_mps": ggv.csv_speed_max_mps,
        "ggv_solver_speed_cap_mps": ggv.maximum_speed_mps,
        "ggv_max_lateral_utilization": float(np.max(lateral_utilization)),
        "ggv_max_speed_domain_fraction": float(np.max(speed_domain_fraction)),
        "ggv_speed_cap_segments": int(
            np.count_nonzero(constraint_speed >= ggv.maximum_speed_mps - 1e-8)
        ),
        "ggv_lookup_interpolation": (
            "linear in speed and signed normalized lateral demand"
        ),
        "ggv_out_of_domain_policy": (
            "track speeds capped at highest feasible GGV drive slice; "
            "drive acceleration is zero above it; no NaN gaps are crossed"
        ),
        "canonical_time_formula": (
            "sum(dx / speed), matching OpenLAP.m for a closed track"
            if is_closed
            else (
                "2*dx[0]/speed[0] + sum(dx[1:] / speed[1:]), matching "
                "OpenLAP.m standing-start integration"
            )
        ),
    }
    return result, summary


def solve_closed_track(
    ggv: GGVMap,
    track: pd.DataFrame,
    tolerance: float = 1e-9,
    max_iterations: int = 2000,
) -> tuple[pd.DataFrame, dict]:
    _validate_track(track, is_closed=True)
    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    count = len(track)
    lateral_limit = ggv.lateral_speed_limits(curvature)
    speed = lateral_limit.copy()

    converged = False
    max_change = math.inf
    for iteration in range(1, max_iterations + 1):
        old_speed = speed.copy()
        for index in range(count):
            next_index = (index + 1) % count
            ay = speed[index] ** 2 * curvature[index]
            acceleration = ggv.acceleration(speed[index], ay)
            proposed = math.sqrt(
                max(0.0, speed[index] ** 2 + 2.0 * acceleration * dx[index])
            )
            speed[next_index] = min(speed[next_index], proposed)

        for index in range(count - 1, -1, -1):
            previous_index = (index - 1) % count
            proposed = _backward_speed(
                ggv,
                next_speed_mps=speed[index],
                previous_curvature_1pm=curvature[previous_index],
                dx_m=dx[previous_index],
                previous_lateral_limit_mps=lateral_limit[previous_index],
            )
            speed[previous_index] = min(speed[previous_index], proposed)

        speed = np.minimum(speed, lateral_limit)
        max_change = float(np.max(np.abs(speed - old_speed)))
        if max_change < tolerance:
            converged = True
            break

    return _finish_result(
        ggv,
        track,
        speed,
        lateral_limit,
        is_closed=True,
        iteration=iteration,
        converged=converged,
        max_change=max_change,
    )


def solve_open_track(
    ggv: GGVMap,
    track: pd.DataFrame,
    tolerance: float = 1e-9,
    max_iterations: int = 2000,
) -> tuple[pd.DataFrame, dict]:
    """Solve endpoint segments with OpenLAP's synthetic zero-speed start."""

    _validate_track(track, is_closed=False)
    dx = track["dx_m"].to_numpy(dtype=float)
    curvature = track["curvature_1pm"].to_numpy(dtype=float)
    count = len(track)
    lateral_limit = ggv.lateral_speed_limits(curvature)
    speed = lateral_limit.copy()

    start_acceleration = ggv.acceleration(0.0, 0.0)
    standing_start_limit = math.sqrt(max(0.0, 2.0 * start_acceleration * dx[0]))
    if standing_start_limit <= 0.0:
        raise ValueError("GGV map provides no standing-start acceleration")
    speed[0] = min(speed[0], standing_start_limit)

    converged = False
    max_change = math.inf
    for iteration in range(1, max_iterations + 1):
        old_speed = speed.copy()
        speed[0] = min(speed[0], standing_start_limit, lateral_limit[0])
        for index in range(count - 1):
            next_index = index + 1
            ay = speed[index] ** 2 * curvature[index]
            acceleration = ggv.acceleration(speed[index], ay)
            proposed = math.sqrt(
                max(
                    0.0,
                    speed[index] ** 2 + 2.0 * acceleration * dx[next_index],
                )
            )
            speed[next_index] = min(speed[next_index], proposed)

        for index in range(count - 1, 0, -1):
            previous_index = index - 1
            proposed = _backward_speed(
                ggv,
                next_speed_mps=speed[index],
                previous_curvature_1pm=curvature[previous_index],
                dx_m=dx[index],
                previous_lateral_limit_mps=lateral_limit[previous_index],
            )
            speed[previous_index] = min(speed[previous_index], proposed)

        speed = np.minimum(speed, lateral_limit)
        speed[0] = min(speed[0], standing_start_limit)
        max_change = float(np.max(np.abs(speed - old_speed)))
        if max_change < tolerance:
            converged = True
            break

    return _finish_result(
        ggv,
        track,
        speed,
        lateral_limit,
        is_closed=False,
        iteration=iteration,
        converged=converged,
        max_change=max_change,
    )


def solve_track(
    ggv: GGVMap,
    track: pd.DataFrame,
    is_closed: bool,
) -> tuple[pd.DataFrame, dict]:
    if is_closed:
        return solve_closed_track(ggv, track)
    return solve_open_track(ggv, track)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--ggv", type=Path, required=True)
    parser.add_argument(
        "--track",
        type=Path,
        default=root / "inputs" / "events" / "michigan_endurance_openlap_track.csv",
    )
    parser.add_argument("--output-root", type=Path, default=root / "outputs" / "ggv")
    parser.add_argument("--configuration", choices=("open", "closed"), default="closed")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = args.output_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    ggv = GGVMap.from_csv(args.ggv)
    track = pd.read_csv(args.track.resolve())
    result, summary = solve_track(
        ggv,
        track,
        is_closed=args.configuration == "closed",
    )
    result.to_csv(output_root / "ggv_trace.csv", index=False)
    (output_root / "ggv_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
