"""Bridge BobSim's reduced-order QSS models into the Lapsims GGV workflow.

The legacy EnvelopeSim generator remains available by passing ``model_dof=None``.
Reduced-order generation always receives an explicitly constructed model; this
module never relies on ``generate_ggv``'s legacy ``reduced_model=None`` default.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Iterable
from dataclasses import asdict, replace
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np

MODEL_FAMILY_LEGACY = "envelopesim_algebraic"
MODEL_FAMILY_REDUCED = "dyn_py_reduced_order_qss"
SUPPORTED_MODEL_DOFS = (3, 6, 10, 14)
DEFAULT_TOP_SPEED_MPS = 42.0539983361957
DEFAULT_SPEED_STEP_MPS = 1.0
DEFAULT_QSS_ZERO_PROXY_MPS = 0.1


def model_identity(model_dof: int | None) -> tuple[str, str]:
    """Return stable family/key identifiers for output joins."""

    if model_dof is None:
        return MODEL_FAMILY_LEGACY, "envelopesim_algebraic_legacy"
    if model_dof not in SUPPORTED_MODEL_DOFS:
        raise ValueError(
            f"model_dof must be one of {SUPPORTED_MODEL_DOFS}, got {model_dof}."
        )
    return MODEL_FAMILY_REDUCED, f"dyn_py_{model_dof}dof_qss"


def default_output_root(base: Path, model_dof: int | None) -> Path:
    """Keep default legacy and reduced-model artifacts in separate roots."""

    if model_dof is None:
        return base
    return base.with_name(f"{base.name}_dyn_py_{model_dof}dof")


def dense_speed_slices(
    *,
    top_speed_mps: float = DEFAULT_TOP_SPEED_MPS,
    step_mps: float = DEFAULT_SPEED_STEP_MPS,
) -> tuple[float, ...]:
    """Return 0, a dense uniform grid, and the exact driveline speed ceiling."""

    if not math.isfinite(top_speed_mps) or top_speed_mps <= 0.0:
        raise ValueError("top_speed_mps must be finite and positive.")
    if not math.isfinite(step_mps) or step_mps <= 0.0:
        raise ValueError("step_mps must be finite and positive.")
    values = [0.0]
    index = 1
    while index * step_mps < top_speed_mps - 1e-12:
        values.append(float(index * step_mps))
        index += 1
    if not math.isclose(values[-1], top_speed_mps, rel_tol=0.0, abs_tol=1e-12):
        values.append(float(top_speed_mps))
    else:
        values[-1] = float(top_speed_mps)
    return tuple(values)


def qss_generation_speeds(
    exported_speeds: Iterable[float],
    *,
    zero_proxy_mps: float = DEFAULT_QSS_ZERO_PROXY_MPS,
) -> tuple[float, ...]:
    """Replace the requested zero slice with a small positive QSS trim speed."""

    speeds = tuple(float(value) for value in exported_speeds)
    if not speeds or not math.isclose(speeds[0], 0.0, abs_tol=1e-12):
        raise ValueError("Reduced-order exported speeds must begin at 0 m/s.")
    if any(right <= left for left, right in pairwise(speeds)):
        raise ValueError("Exported speeds must be strictly increasing.")
    if not math.isfinite(zero_proxy_mps) or zero_proxy_mps <= 0.0:
        raise ValueError("zero_proxy_mps must be finite and positive.")
    if len(speeds) < 2 or zero_proxy_mps >= speeds[1]:
        raise ValueError(
            "zero_proxy_mps must be below the first positive exported speed."
        )
    return (float(zero_proxy_mps), *speeds[1:])


def add_zero_speed_proxy(envelopes: list[Any], *, zero_proxy_mps: float) -> list[Any]:
    """Duplicate the near-zero QSS envelope at 0 m/s for standing starts.

    QSS trim is undefined at exactly zero speed. At the default 0.1 m/s proxy,
    aerodynamic load is negligible and the driveline remains on the same
    constant-torque branch, so copying that boundary is a controlled limiting
    approximation rather than extrapolating from the first 1 m/s map slice.
    """

    matches = [
        envelope
        for envelope in envelopes
        if math.isclose(
            float(envelope.speed), zero_proxy_mps, rel_tol=0.0, abs_tol=1e-12
        )
    ]
    if len(matches) != 1:
        raise ValueError(
            "Reduced-order GGV generation must return exactly one zero-speed "
            f"proxy slice at {zero_proxy_mps:.12g} m/s."
        )
    if any(math.isclose(float(item.speed), 0.0, abs_tol=1e-12) for item in envelopes):
        raise ValueError(
            "Reduced-order GGV unexpectedly already contains a 0 m/s slice."
        )
    zero = replace(matches[0], speed=0.0)
    return [zero, *sorted(envelopes, key=lambda item: float(item.speed))]


def adapt_envelope_for_lapsims(envelope: Any) -> Any:
    """Ensure the drive branch closes at zero body acceleration.

    Current BobSim solves this sustainable-corner endpoint directly, with the
    driven wheels balancing drag.  The interpolation path remains for backward
    compatibility with older coast-closed GGV files; already-correct zero-ax
    endpoints pass through unchanged.
    """

    ay = np.asarray(envelope.ay, dtype=float)
    accel = np.asarray(envelope.ax_accel, dtype=float)
    brake = np.asarray(envelope.ax_brake, dtype=float)
    if not (ay.ndim == accel.ndim == brake.ndim == 1):
        raise ValueError("GGV envelope arrays must be one-dimensional.")
    if not (ay.size == accel.size == brake.size):
        raise ValueError("GGV envelope arrays must have equal lengths.")
    if np.any(np.diff(ay) <= 0.0):
        raise ValueError("GGV lateral-acceleration nodes must be strictly increasing.")

    connected_accel = accel.copy()
    for sign in (-1.0, 1.0):
        positions = np.flatnonzero(sign * ay >= -1e-12)
        positions = positions[np.argsort(np.abs(ay[positions]))]
        disconnected = False
        for position in positions:
            if disconnected:
                connected_accel[position] = np.nan
            elif not math.isfinite(float(connected_accel[position])):
                disconnected = True

    additions: list[tuple[float, float]] = []
    for sign in (-1.0, 1.0):
        positions = np.flatnonzero(sign * ay >= -1e-12)
        positions = positions[np.argsort(np.abs(ay[positions]))]
        for inner, outer in pairwise(positions):
            inner_ax = float(connected_accel[inner])
            outer_ax = float(connected_accel[outer])
            if not (math.isfinite(inner_ax) and math.isfinite(outer_ax)):
                continue
            if inner_ax >= -1e-12 and outer_ax < -1e-12:
                if abs(inner_ax) <= 1e-12:
                    break
                fraction = inner_ax / (inner_ax - outer_ax)
                intercept_ay = float(ay[inner] + fraction * (ay[outer] - ay[inner]))
                intercept_brake = float(
                    brake[inner] + fraction * (brake[outer] - brake[inner])
                )
                additions.append((intercept_ay, intercept_brake))
                break

    sanitized_accel = connected_accel
    sanitized_accel[sanitized_accel < -1e-12] = np.nan
    sanitized_accel[np.isfinite(sanitized_accel) & (sanitized_accel < 0.0)] = 0.0
    if additions:
        ay = np.concatenate((ay, np.asarray([item[0] for item in additions])))
        sanitized_accel = np.concatenate(
            (sanitized_accel, np.zeros(len(additions), dtype=float))
        )
        brake = np.concatenate(
            (brake, np.asarray([item[1] for item in additions], dtype=float))
        )
        order = np.argsort(ay)
        ay = ay[order]
        sanitized_accel = sanitized_accel[order]
        brake = brake[order]
    return replace(
        envelope,
        ay=ay,
        ax_accel=sanitized_accel,
        ax_brake=brake,
    )


def generate_envelopes(
    generate_ggv: Callable[..., list[Any]],
    *,
    vehicle: Any,
    config: Any,
    reduced_model: Any | None,
    zero_proxy_mps: float = DEFAULT_QSS_ZERO_PROXY_MPS,
) -> tuple[list[Any], tuple[float, ...]]:
    """Generate legacy or explicitly reduced-model envelopes.

    The return value contains exported envelopes and the actual trim speeds.
    """

    if reduced_model is None:
        envelopes = [
            adapt_envelope_for_lapsims(item) for item in generate_ggv(vehicle, config)
        ]
        return envelopes, tuple(float(v) for v in config.speeds)

    trim_speeds = qss_generation_speeds(
        config.speeds,
        zero_proxy_mps=zero_proxy_mps,
    )
    qss_config = replace(config, speeds=trim_speeds)
    envelopes = [
        adapt_envelope_for_lapsims(item)
        for item in generate_ggv(
            vehicle,
            qss_config,
            reduced_model=reduced_model,
        )
    ]
    return (
        add_zero_speed_proxy(envelopes, zero_proxy_mps=zero_proxy_mps),
        trim_speeds,
    )


def elastic_roll_stiffness(parameters: Any) -> dict[str, float]:
    """Return spring, anti-roll, and total elastic roll stiffness by axle."""

    rates = tuple(float(value) for value in parameters.suspension_stiffness_n_per_m)
    front_spring = 0.25 * sum(rates[:2]) * float(parameters.track_front_m) ** 2
    rear_spring = 0.25 * sum(rates[2:]) * float(parameters.track_rear_m) ** 2
    front_arb, rear_arb = (
        float(value) for value in parameters.antiroll_stiffness_nm_per_rad
    )
    front_total = front_spring + front_arb
    rear_total = rear_spring + rear_arb
    total = front_total + rear_total
    if total <= 0.0:
        raise ValueError(
            "Reduced model must have positive total elastic roll stiffness."
        )
    return {
        "front_spring_nm_per_rad": front_spring,
        "rear_spring_nm_per_rad": rear_spring,
        "front_arb_nm_per_rad": front_arb,
        "rear_arb_nm_per_rad": rear_arb,
        "front_total_nm_per_rad": front_total,
        "rear_total_nm_per_rad": rear_total,
        "total_nm_per_rad": total,
        "front_elastic_fraction": front_total / total,
        "front_antiroll_fraction": (
            front_arb / (front_arb + rear_arb) if front_arb + rear_arb > 0.0 else 0.5
        ),
    }


def cache_fingerprint(payload: Any) -> str:
    """Return a deterministic hash for validated per-case resume artifacts."""

    def default(value: object) -> object:
        if isinstance(value, Path):
            return value.as_posix()
        tolist = getattr(value, "tolist", None)
        if callable(tolist):
            return tolist()
        try:
            return asdict(value)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            raise TypeError(f"Cannot fingerprint {type(value).__name__}") from None

    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        default=default,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
