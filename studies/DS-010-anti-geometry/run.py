#!/usr/bin/env python3
"""DS-010 anti-dive / anti-squat geometry on BobSim FourPostEval.

Runs the three stages in study.yml (anti-squat, anti-dive, validation). Each
hardpoint variant of the vehicle YAML is one FourPostEval rig run, and is
scored with the quasi-static grip objective described in README.md.

Run it in the Docker image built from the pinned BobSim submodule:

    docker run --rm -v "$PWD":/workspace -w /workspace bobdyn/bobsim:c45940e \
        python studies/DS-010-anti-geometry/run.py
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence


STUDY_DIR = Path(__file__).resolve().parent
REPO_ROOT = STUDY_DIR.parents[1]

BOBSIM_ROOT = REPO_ROOT / "BobSim"
BOBLIB_ROOT = BOBSIM_ROOT / "_0_Utils" / "external" / "BobLib"
BOBLIB_PACKAGE = BOBLIB_ROOT / "BobLib" / "package.mo"
GENERATION_SCRIPTS = BOBLIB_ROOT / "Generation" / "scripts"

MPLCONFIGDIR = Path("/tmp/lhre-sim-matplotlib")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))


def require_dependencies() -> tuple[Any, Any, Any, Any]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
        import yaml
        from scipy.optimize import brentq
    except ModuleNotFoundError as exc:
        missing = exc.name or "a required package"
        raise SystemExit(
            f"Missing Python dependency: {missing}\n"
            "Run the study in the BobSim Docker image, or install:\n"
            "  python -m pip install PyYAML numpy matplotlib scipy"
        ) from exc

    return np, plt, yaml, brentq


np, plt, yaml, brentq = require_dependencies()


G = 9.80665  # m/s^2, as FourPostEval
AXLES = ("front", "rear")
ARMS = ("upper", "lower")
# Hardpoints a variant moves, as (block, key) under the axle.
MOVED_PICKUPS = tuple(("suspension", f"{arm}_{pivot}") for arm in ARMS for pivot in ("fore_i_m", "aft_i_m")) + (
    ("steering", "rack_pickup_m"),
)
STAGES = ("anti-squat", "anti-dive", "validation")
MANEUVERS = ("rwd_acceleration", "braking")
STAGE_SUMMARY = {
    "anti-squat": "anti_squat_summary.csv",
    "anti-dive": "anti_dive_summary.csv",
}
STAGE_DEPENDS = {
    "anti-squat": (),
    "anti-dive": ("anti-squat",),
    "validation": ("anti-squat", "anti-dive"),
}

# x direction from each wheel centre towards the middle of the car.
INWARD = {"front": -1.0, "rear": 1.0}
RIG_PREFIX = {"front": "fr", "rear": "rr"}
# FourPostEval reports jack / Fx for a forward Fx pulse at the contact patches.
# These signs map it to the side-view slope c used here (IC inboard and above
# the contact patch is c > 0). Set from the end-to-end check against the
# Python IC on strongly-anti variants; the opposite-sign stop guards them.
# FourPostEval reports jacking / F_x for one F_x pulse. A rearward brake force on an
# anti-dive front lifts the car, so the front coefficient is negative for anti-dive.
# Smoke check at 90 %: front c_ic +0.193 vs rig -0.192, rear c_ic +0.162 vs rig +0.161.
RIG_C_SIGN = {"front": -1.0, "rear": 1.0}
ROTATION_LIMIT_RAD = math.radians(20.0)

FOURPOST_CFG = {
    "model": "BobLib.Standards.FourPostSim",
    "start_time": 0,
    "stop_time": 113,
    "intervals": None,
    "tolerance": 1e-6,
    "solver": "dassl",
    "command_line_options": [
        "--simCodeTarget=C",
        "--maxSizeLinearTearing=5000",
        "--indexReductionMethod=dynamicStateSelection",
        "--matchingAlgorithm=PFPlusExt",
        "-d=NLSanalyticJacobian",
    ],
}


# ---------------------------------------------------------------------------
# Study files and CLI


@dataclass(frozen=True)
class StudyPaths:
    study_dir: Path
    study_yml: Path
    vehicle: Path
    outputs: Path
    plots: Path
    work: Path
    reports: tuple[Path, ...]


def load_study(study_dir: Path) -> dict[str, Any]:
    path = Path(study_dir) / "study.yml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping.")
    return data


def resolve_paths(study_dir: Path, vehicle_arg: str | None, study: dict[str, Any]) -> StudyPaths:
    study_dir = Path(study_dir).resolve()
    if vehicle_arg:
        vehicle = Path(vehicle_arg).resolve()
    else:
        vehicle = (REPO_ROOT / str(study["vehicle"])).resolve()
    reports = tuple(
        (study_dir / str(entry)).resolve()
        for entry in study.get("outputs", [])
        if str(entry).endswith(".md")
    )
    return StudyPaths(
        study_dir=study_dir,
        study_yml=study_dir / "study.yml",
        vehicle=vehicle,
        outputs=study_dir / "outputs",
        plots=study_dir / "plots",
        work=study_dir / "work",
        reports=reports,
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--study",
        type=Path,
        default=STUDY_DIR,
        help="study directory holding study.yml (default: this script's directory)",
    )
    parser.add_argument(
        "--vehicle",
        help="vehicle YAML (default: study.yml `vehicle`, relative to the repo root)",
    )
    parser.add_argument("--stage", choices=("all",) + STAGES, default="all")
    parser.add_argument(
        "--reuse",
        action="store_true",
        help="reuse cached builds and rig results in work/ when the variant is unchanged",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=3,
        help="variants built and run in parallel (default: 3; about 1.3 GB each)",
    )
    return parser.parse_args(argv)


def as_repo_path(path: Path) -> str:
    try:
        return Path(path).resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(path)


def stage_dependency_error(stage: str, needed: str, missing: Path, study_dir: Path) -> str:
    script = as_repo_path(Path(__file__))
    study_arg ="" if Path(study_dir).resolve() == STUDY_DIR else f" --study {as_repo_path(study_dir)}"
    return (
        f"--stage {stage} needs {as_repo_path(missing)} from the {needed} stage, "
        f"which does not exist. Run that stage first:\n"
        f"  python {script}{study_arg} --stage {needed}"
    )


def check_stage_dependencies(stage: str, outputs: Path, study_dir: Path) -> None:
    for needed in STAGE_DEPENDS[stage]:
        path = outputs / STAGE_SUMMARY[needed]
        if not path.exists():
            raise SystemExit(stage_dependency_error(stage, needed, path, study_dir))


# ---------------------------------------------------------------------------
# Vehicle data


def vec(values: Sequence[float]) -> np.ndarray:
    return np.asarray(values, dtype=float)


def read_vehicle(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise TypeError(f"Expected mapping at top level: {path}")
    return data


def to_builtin(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: to_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(item) for item in value]
    if isinstance(value, np.ndarray):
        return [to_builtin(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    return value


def dump_yaml(data: dict[str, Any]) -> str:
    return yaml.safe_dump(to_builtin(data), sort_keys=False, default_flow_style=False)


def spring_rate_from_vehicle(vehicle: dict[str, Any], axle: str) -> float:
    spring = vehicle[axle]["actuation"]["shock"]["spring_table"]
    table = spring["table"] if isinstance(spring, dict) else spring
    (x0, f0), (x1, f1) = table[0], table[-1]
    return float((f1 - f0) / (x1 - x0))


def wheelbase_and_tracks(vehicle: dict[str, Any]) -> tuple[float, float, float]:
    front_wc = vec(vehicle["front"]["suspension"]["wheel_center_m"])
    rear_wc = vec(vehicle["rear"]["suspension"]["wheel_center_m"])
    return (
        abs(float(front_wc[0] - rear_wc[0])),
        2.0 * abs(float(front_wc[1])),
        2.0 * abs(float(rear_wc[1])),
    )


@dataclass(frozen=True)
class MassProps:
    m: float  # total mass incl. driver and unsprung, kg
    h: float  # total CG height, m
    x_cg: float
    x_front: float  # front wheel-centre x
    x_rear: float
    m_s: float  # sprung + driver, kg
    x_s: float
    h_s: float

    @property
    def l(self) -> float:  # noqa: E743
        return self.x_front - self.x_rear

    @property
    def a(self) -> float:
        """CG distance behind the front axle."""
        return self.x_front - self.x_cg


def mass_rollup(vehicle: dict[str, Any]) -> MassProps:
    sprung = [
        (float(vehicle[key]["mass_kg"]), vec(vehicle[key]["cg_m"]))
        for key in ("sprung_mass", "driver_mass")
        if isinstance(vehicle.get(key), dict)
    ]
    # Every unsprung part is listed for the left side; the right is its mirror,
    # which has the same x and z.
    unsprung = [
        (float(part["mass_kg"]), vec(part["cg_m"]))
        for axle in AXLES
        for part in vehicle[axle]["masses"].values()
        for _ in range(2)
    ]

    def combine(items: list[tuple[float, np.ndarray]]) -> tuple[float, np.ndarray]:
        mass = sum(m for m, _ in items)
        return mass, sum(m * cg for m, cg in items) / mass

    m_s, cg_s = combine(sprung)
    m, cg = combine(sprung + unsprung)
    return MassProps(
        m=float(m),
        h=float(cg[2]),
        x_cg=float(cg[0]),
        x_front=float(vehicle["front"]["suspension"]["wheel_center_m"][0]),
        x_rear=float(vehicle["rear"]["suspension"]["wheel_center_m"][0]),
        m_s=float(m_s),
        x_s=float(cg_s[0]),
        h_s=float(cg_s[2]),
    )


def static_axle_loads(mp: MassProps) -> tuple[float, float]:
    weight = mp.m * G
    return (
        weight * (mp.x_cg - mp.x_rear) / mp.l,
        weight * (mp.x_front - mp.x_cg) / mp.l,
    )


def sprung_corner_mass(mp: MassProps, axle: str) -> float:
    if axle == "front":
        fraction = (mp.x_s - mp.x_rear) / mp.l
    else:
        fraction = (mp.x_front - mp.x_s) / mp.l
    return mp.m_s * fraction / 2.0


# ---------------------------------------------------------------------------
# Ride rates


def wheel_rate_for_frequency(m_c: float, f_hz: float, k_t: float) -> float:
    """Wheel rate whose series combination with the tire gives f_hz on m_c."""
    k_rd = m_c * (2.0 * math.pi * f_hz) ** 2
    if k_rd >= k_t:
        raise ValueError(
            f"A {f_hz:.2f} Hz ride frequency needs a ride rate of {k_rd:,.0f} N/m, "
            f"at or above the tire rate of {k_t:,.0f} N/m. Lower ride_frequency_hz."
        )
    return k_rd * k_t / (k_t - k_rd)


def ride_frequency(m_c: float, k_w: float, k_t: float) -> float:
    k_rd = k_w * k_t / (k_w + k_t)
    return math.sqrt(k_rd / m_c) / (2.0 * math.pi)


@dataclass(frozen=True)
class AxleRate:
    k_w: float  # wheel rate per corner, N/m
    k_s: float  # spring rate per corner, N/m (implied under the frequency hold)
    k_t: float
    mr: float  # FourPostEval avg_motion_ratio (wheel / spring)
    f_hz: float
    held: bool  # True when k_w comes from the ride-frequency target

    @property
    def K(self) -> float:
        return 2.0 * self.k_w

    @property
    def Kt(self) -> float:
        return 2.0 * self.k_t


def ride_frequency_target(assumptions: dict[str, Any], axle: str) -> float | None:
    targets = assumptions.get("ride_frequency_hz")
    if targets is None:
        return None
    value = targets.get(axle) if isinstance(targets, dict) else targets
    return None if value is None else float(value)


def axle_rate(
    vehicle: dict[str, Any], mp: MassProps, axle: str, f_target: float | None, mr: float
) -> AxleRate:
    k_t = float(vehicle[axle]["tire"]["vertical_stiffness_n_per_m"])
    m_c = sprung_corner_mass(mp, axle)
    if f_target is not None:
        k_w = wheel_rate_for_frequency(m_c, f_target, k_t)
        k_s = k_w * mr**2
    else:
        k_s = spring_rate_from_vehicle(vehicle, axle)
        k_w = k_s / mr**2
    return AxleRate(
        k_w=k_w,
        k_s=k_s,
        k_t=k_t,
        mr=mr,
        f_hz=ride_frequency(m_c, k_w, k_t),
        held=f_target is not None,
    )


# ---------------------------------------------------------------------------
# Anti and ride-height response (formulation note §4.2-4.3). Anti values are
# fractions; inv_d is 1/d, zero when the side-view arms are parallel.


def delta_w(mp: MassProps, ax_g: float) -> float:
    return mp.m * G * abs(ax_g) * mp.h / mp.l


def rear_brake_shares(beta_f: float, regen_share: float) -> tuple[float, float]:
    s_regen = regen_share * (1.0 - beta_f)
    return (1.0 - beta_f) - s_regen, s_regen


def anti_dive(c_f: float, beta_f: float, mp: MassProps) -> float:
    return beta_f * mp.l / mp.h * c_f


def anti_lift(
    c_r: float, inv_d_r: float, R_r: float, beta_f: float, regen_share: float, mp: MassProps
) -> float:
    s_fric, s_regen = rear_brake_shares(beta_f, regen_share)
    return mp.l / mp.h * (s_fric * c_r + s_regen * (c_r - R_r * inv_d_r))


def anti_squat(c_r: float, inv_d_r: float, R_r: float, mp: MassProps) -> float:
    return mp.l / mp.h * (c_r - R_r * inv_d_r)


def c_for_anti_dive(anti: float, beta_f: float, mp: MassProps) -> float:
    return anti * mp.h / (mp.l * beta_f)


def c_for_anti_squat(anti: float, inv_d_r: float, R_r: float, mp: MassProps) -> float:
    return anti * mp.h / mp.l + R_r * inv_d_r


def dz_braking(
    dW: float, AD_f: float, AL_r: float, K_f: float, Kt_f: float, K_r: float, Kt_r: float
) -> tuple[float, float]:
    return (
        -dW * ((1.0 - AD_f) / K_f + 1.0 / Kt_f),
        dW * ((1.0 - AL_r) / K_r + 1.0 / Kt_r),
    )


def dz_accel(
    dW: float, AS_r: float, K_f: float, Kt_f: float, K_r: float, Kt_r: float
) -> tuple[float, float]:
    return (
        dW * (1.0 / K_f + 1.0 / Kt_f),
        -dW * ((1.0 - AS_r) / K_r + 1.0 / Kt_r),
    )


def anti_from_dz(dz: float, dW: float, K: float, Kt: float) -> float:
    """Anti that gives the signed drop dz on the loaded axle (front braking, rear drive)."""
    return 1.0 + K * (dz / dW + 1.0 / Kt)


# ---------------------------------------------------------------------------
# Side-view geometry. Lines are n_x x + n_z z = c in the plane y = y_wc; points
# are homogeneous (X, Z, W) so a parallel-arm IC (W = 0) needs no special case.


def side_view_line(
    fore: Sequence[float], aft: Sequence[float], outer: Sequence[float], y: float
) -> tuple[float, float, float]:
    fore, aft, outer = vec(fore), vec(aft), vec(outer)
    n = np.cross(aft - fore, outer - fore)
    return float(n[0]), float(n[2]), float(n @ fore - n[1] * y)


def arm_line(susp: dict[str, Any], arm: str, y: float) -> tuple[float, float, float]:
    return side_view_line(susp[f"{arm}_fore_i_m"], susp[f"{arm}_aft_i_m"], susp[f"{arm}_o_m"], y)


def side_view_ic_h(susp: dict[str, Any], y: float) -> np.ndarray:
    upper, lower = (arm_line(susp, arm, y) for arm in ARMS)
    return np.cross([upper[0], upper[1], -upper[2]], [lower[0], lower[1], -lower[2]])


def side_view_ic(susp: dict[str, Any], y: float) -> tuple[float, float] | None:
    X, Z, W = side_view_ic_h(susp, y)
    if abs(W) <= 1e-15 * max(abs(X), abs(Z)):
        return None
    return float(X / W), float(Z / W)


@dataclass(frozen=True)
class AxleGeometry:
    axle: str
    c: float  # side-view slope, contact patch to IC
    inv_d: float  # 1/d, d = IC distance inboard of the wheel centre, 1/m
    x_wc: float
    y_wc: float
    z_wc: float
    z_cp: float
    R: float

    @property
    def d(self) -> float:
        return math.inf if self.inv_d == 0.0 else 1.0 / self.inv_d


def axle_geometry(vehicle: dict[str, Any], axle: str) -> AxleGeometry:
    susp = vehicle[axle]["suspension"]
    wc = vec(susp["wheel_center_m"])
    R = float(vehicle[axle]["wheel"]["radius_m"])
    z_cp = float(wc[2]) - R
    X, Z, W = side_view_ic_h(susp, float(wc[1]))
    den = INWARD[axle] * (X - wc[0] * W)
    if abs(den) <= 1e-12 * max(abs(X), abs(Z), abs(W)):
        raise ValueError(f"{axle} side-view IC lies directly above the wheel centre.")
    return AxleGeometry(
        axle=axle,
        c=float((Z - z_cp * W) / den),
        inv_d=float(W / den),
        x_wc=float(wc[0]),
        y_wc=float(wc[1]),
        z_wc=float(wc[2]),
        z_cp=z_cp,
        R=R,
    )


def target_point_h(geom: AxleGeometry, c: float, inv_d: float) -> np.ndarray:
    """IC at slope c and 1/d = inv_d: (x_wc + inward d, z_cp + c d) in homogeneous form."""
    return np.array(
        [geom.x_wc * inv_d + INWARD[geom.axle], geom.z_cp * inv_d + c, inv_d], dtype=float
    )


def rotate_pivot(
    fore: Sequence[float], aft: Sequence[float], x_wc: float, delta: float
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate both pivots by delta about the y-direction through the axis point at x_wc."""
    fore, aft = vec(fore), vec(aft)
    if abs(aft[0] - fore[0]) < 1e-9:
        raise ValueError("Pivot axis has no x extent.")
    p0 = fore + (x_wc - fore[0]) / (aft[0] - fore[0]) * (aft - fore)
    cos_d, sin_d = math.cos(delta), math.sin(delta)

    def rotate(point: np.ndarray) -> np.ndarray:
        dx, dz = point[0] - p0[0], point[2] - p0[2]
        return np.array([p0[0] + cos_d * dx - sin_d * dz, point[1], p0[2] + sin_d * dx + cos_d * dz])

    return rotate(fore), rotate(aft)


@dataclass(frozen=True)
class UprightRates:
    """Static first-order upright motion per unit upper-arm rotation, before steer."""

    U: np.ndarray  # upper ball joint
    T: np.ndarray  # toe-link outer joint
    W: np.ndarray  # wheel centre
    v_U: np.ndarray
    omega_perp: np.ndarray  # angular rate normal to the kingpin axis
    k: np.ndarray  # kingpin axis (unit)

    def toe_rate_for(self, omega_s: float) -> float:
        """d(toe about z)/d(wheel-centre z) for a steer rate omega_s about k."""
        omega = self.omega_perp + omega_s * self.k
        v_w = self.v_U + np.cross(omega, self.W - self.U)
        return float(omega[2] / v_w[2])

    def steer_rate_for(self, pickup: Sequence[float]) -> float:
        """Steer rate the toe link allows: (T - P) . v_T = 0."""
        r = self.T - vec(pickup)
        return float(-np.dot(r, self.v_U + np.cross(self.omega_perp, self.T - self.U))
                     / np.dot(r, np.cross(self.k, self.T - self.U)))


def upright_rates(susp: dict[str, Any]) -> UprightRates:
    U, L = vec(susp["upper_o_m"]), vec(susp["lower_o_m"])

    def arm_velocity(arm: str, point: np.ndarray) -> np.ndarray:
        fore, aft = vec(susp[f"{arm}_fore_i_m"]), vec(susp[f"{arm}_aft_i_m"])
        return np.cross((aft - fore) / np.linalg.norm(aft - fore), point - fore)

    v_U = arm_velocity("upper", U)
    v_L1 = arm_velocity("lower", L)
    v_L = np.dot(U - L, v_U) / np.dot(U - L, v_L1) * v_L1  # |U - L| stays constant
    return UprightRates(
        U=U,
        T=vec(susp["tie_o_m"]),
        W=vec(susp["wheel_center_m"]),
        v_U=v_U,
        omega_perp=np.cross(U - L, v_U - v_L) / np.dot(U - L, U - L),
        k=(U - L) / np.linalg.norm(U - L),
    )


def toe_rate(susp: dict[str, Any], pickup: Sequence[float]) -> float:
    """Static bump steer, rad per m of wheel-centre rise (matches FourPostEval's toe vs heave)."""
    rates = upright_rates(susp)
    return rates.toe_rate_for(rates.steer_rate_for(pickup))


def tie_pickup_for_toe_rate(susp: dict[str, Any], pickup: Sequence[float], target: float) -> np.ndarray | None:
    """Toe-link/rack inner pickup height (x, y held) that gives `target` bump steer.

    The target fixes the steer rate, and the toe-link constraint is then linear in the
    pickup. None when the link is tangent to the needed path (no height works).
    """
    rates = upright_rates(susp)
    e = rates.W - rates.U
    dz = np.cross(rates.omega_perp, e)[2]
    omega_s = (target * (rates.v_U[2] + dz) - rates.omega_perp[2]) / (
        rates.k[2] - target * np.cross(rates.k, e)[2]
    )
    g = rates.v_U + np.cross(rates.omega_perp + omega_s * rates.k, rates.T - rates.U)
    if abs(g[2]) < 1e-9 * np.linalg.norm(g):
        return None
    p = vec(pickup).copy()
    p[2] = (np.dot(rates.T, g) - p[0] * g[0] - p[1] * g[1]) / g[2]
    return p


def line_residual(line: tuple[float, float, float], q: np.ndarray) -> float:
    nx, nz, c = line
    return (nx * q[0] + nz * q[1] - c * q[2]) / math.hypot(nx, nz)


def solve_rotation(
    fore: Sequence[float],
    aft: Sequence[float],
    outer: Sequence[float],
    y: float,
    x_wc: float,
    q: np.ndarray,
) -> float | None:
    """Pivot rotation that puts the arm's side-view line through q, or None."""

    def residual(delta: float) -> float:
        new_fore, new_aft = rotate_pivot(fore, aft, x_wc, delta)
        return line_residual(side_view_line(new_fore, new_aft, outer, y), q)

    lo, hi = -ROTATION_LIMIT_RAD, ROTATION_LIMIT_RAD
    r_lo, r_hi = residual(lo), residual(hi)
    if r_lo == 0.0:
        return lo
    if r_hi == 0.0:
        return hi
    if r_lo * r_hi > 0.0:
        return None
    return float(brentq(residual, lo, hi, xtol=1e-15, maxiter=200))


def make_hardpoint_variant(
    vehicle: dict[str, Any], axle: str, target_c: float, inv_d: float
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    """Rotate both wishbone pivot axes so the side-view IC sits at (target_c, inv_d)."""
    geom = axle_geometry(vehicle, axle)
    q = target_point_h(geom, target_c, inv_d)
    info: dict[str, Any] = {
        "target_c": target_c,
        "target_inv_d_per_m": inv_d,
        "target_ic_x_m": geom.x_wc + INWARD[axle] / inv_d if inv_d else math.inf,
        "target_ic_z_m": geom.z_cp + target_c / inv_d if inv_d else math.inf,
        "delta_upper_deg": math.nan,
        "delta_lower_deg": math.nan,
        "tie_pickup_shift_mm": math.nan,
        "max_pickup_shift_mm": math.nan,
        "root_found": False,
    }
    new = copy.deepcopy(vehicle)
    susp = new[axle]["suspension"]
    steering = new[axle]["steering"]
    tie = vec(steering["rack_pickup_m"])
    toe0 = toe_rate(susp, tie)
    shifts = []
    for arm in ARMS:
        fore, aft = susp[f"{arm}_fore_i_m"], susp[f"{arm}_aft_i_m"]
        delta = solve_rotation(fore, aft, susp[f"{arm}_o_m"], geom.y_wc, geom.x_wc, q)
        if delta is None:
            return None, info
        info[f"delta_{arm}_deg"] = math.degrees(delta)
        new_fore, new_aft = rotate_pivot(fore, aft, geom.x_wc, delta)
        shifts += [np.linalg.norm(new_fore - vec(fore)), np.linalg.norm(new_aft - vec(aft))]
        susp[f"{arm}_fore_i_m"] = [float(v) for v in new_fore]
        susp[f"{arm}_aft_i_m"] = [float(v) for v in new_aft]
    # Hold bump steer: re-height the toe-link/rack inner pickup for the baseline toe rate.
    new_tie = tie_pickup_for_toe_rate(susp, tie, toe0)
    if new_tie is None:
        return None, info
    steering["rack_pickup_m"] = [float(v) for v in new_tie]
    info["tie_pickup_shift_mm"] = 1000.0 * float(np.linalg.norm(new_tie - tie))
    shifts.append(np.linalg.norm(new_tie - tie))
    info["max_pickup_shift_mm"] = 1000.0 * float(max(shifts))
    info["root_found"] = True
    return new, info


# ---------------------------------------------------------------------------
# Tire, aero, ride heights, grip


@dataclass(frozen=True)
class TireParams:
    pdx1: float
    pdx2: float
    fz0: float  # FNOMIN * LFZO
    lmux: float


def parse_tir(path: Path) -> dict[str, float | str]:
    params: dict[str, float | str] = {}
    line_re = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*(?:\$.*)?$")
    for line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        match = line_re.match(line)
        if not match:
            continue
        raw = match.group(2).strip().strip("'").strip('"')
        try:
            params[match.group(1).upper()] = float(raw)
        except ValueError:
            params[match.group(1).upper()] = raw
    return params


def tire_path(vehicle: dict[str, Any], axle: str) -> Path:
    root = Path(str(vehicle.get("paths", {}).get("tire_templates", "_0_Utils/tire_templates")))
    root = root if root.is_absolute() else BOBSIM_ROOT / root
    return root / f"{vehicle[axle]['tire']['template']}.tir"


def read_tire_params(path: Path) -> TireParams:
    tir = parse_tir(path)
    return TireParams(
        pdx1=float(tir["PDX1"]),
        pdx2=float(tir["PDX2"]),
        fz0=float(tir["FNOMIN"]) * float(tir.get("LFZO", 1.0)),
        lmux=float(tir.get("LMUX", 1.0)),
    )


def mu_x(fz: float, tp: TireParams) -> float:
    return tp.lmux * (tp.pdx1 + tp.pdx2 * (fz - tp.fz0) / tp.fz0)


def expected_grip(fz_mean: float, sigma_fz: float, tp: TireParams) -> float:
    """E[mu_x(Fz) Fz] for Fz with mean fz_mean and std sigma_fz; exact, mu_x Fz is quadratic."""
    return mu_x(fz_mean, tp) * fz_mean + tp.lmux * tp.pdx2 / tp.fz0 * sigma_fz**2


def _bilinear(grid_a: np.ndarray, grid_b: np.ndarray, table: np.ndarray, a: float, b: float) -> float:
    i = int(np.clip(np.searchsorted(grid_a, a) - 1, 0, len(grid_a) - 2))
    j = int(np.clip(np.searchsorted(grid_b, b) - 1, 0, len(grid_b) - 2))
    u = (a - grid_a[i]) / (grid_a[i + 1] - grid_a[i])
    v = (b - grid_b[j]) / (grid_b[j + 1] - grid_b[j])
    return float(
        (1 - u) * (1 - v) * table[i, j]
        + (1 - u) * v * table[i, j + 1]
        + u * (1 - v) * table[i + 1, j]
        + u * v * table[i + 1, j + 1]
    )


@dataclass(frozen=True)
class AeroMap:
    front_grid: np.ndarray
    rear_grid: np.ndarray
    downforce: np.ndarray  # [front index, rear index], N at v_ref
    my: np.ndarray
    ref_x: float
    v_ref: float
    x_front: float
    x_rear: float

    @classmethod
    def from_vehicle(cls, vehicle: dict[str, Any]) -> "AeroMap":
        aero = vehicle["aero"]
        return cls(
            front_grid=vec(aero["front_ride_height_grid_m"]),
            rear_grid=vec(aero["rear_ride_height_grid_m"]),
            downforce=vec(aero["downforce_table_n"]),
            my=vec(aero["my_table_nm"]),
            ref_x=float(aero["aero_ref_m"][0]),
            v_ref=float(aero["reference_speed_m_per_s"]),
            x_front=float(vehicle["front"]["suspension"]["wheel_center_m"][0]),
            x_rear=float(vehicle["rear"]["suspension"]["wheel_center_m"][0]),
        )

    def evaluate(self, rh_f: float, rh_r: float, v: float) -> tuple[float, float, bool]:
        """Downforce (N), front fraction and whether the ride heights left the grid."""
        f = float(np.clip(rh_f, self.front_grid[0], self.front_grid[-1]))
        r = float(np.clip(rh_r, self.rear_grid[0], self.rear_grid[-1]))
        clamped = bool(f != rh_f or r != rh_r)
        # BobLib interpolates force and moment separately, so CoP is their ratio.
        df = _bilinear(self.front_grid, self.rear_grid, self.downforce, f, r)
        my = _bilinear(self.front_grid, self.rear_grid, self.my, f, r)
        front_fraction = (my / df + self.ref_x - self.x_rear) / (self.x_front - self.x_rear)
        return df * (v / self.v_ref) ** 2, front_fraction, clamped


@dataclass(frozen=True)
class RideHeights:
    rh_f: float
    rh_r: float
    df: float
    front_fraction: float
    f_aero_f: float
    f_aero_r: float
    clamped: bool
    iterations: int


def solve_ride_heights(
    rh_static: tuple[float, float],
    dz: tuple[float, float],
    k_ride: tuple[float, float],
    aero: AeroMap,
    v: float,
    tol: float = 1e-10,
    max_iter: int = 200,
) -> RideHeights:
    """Fixed point of RH_i = RH_static,i + dz_i - F_aero,i(RH) / K_ride,i."""
    rh = np.array(rh_static, dtype=float) + np.array(dz, dtype=float)
    for iteration in range(1, max_iter + 1):
        df, frac, _ = aero.evaluate(rh[0], rh[1], v)
        new = np.array(
            [
                rh_static[0] + dz[0] - df * frac / k_ride[0],
                rh_static[1] + dz[1] - df * (1.0 - frac) / k_ride[1],
            ]
        )
        step = float(np.max(np.abs(new - rh)))
        rh = new
        if step < tol:
            break
    else:
        raise RuntimeError(f"Ride heights did not converge at {v} m/s (last step {step:.3g} m).")
    df, frac, clamped = aero.evaluate(rh[0], rh[1], v)
    return RideHeights(
        rh_f=float(rh[0]),
        rh_r=float(rh[1]),
        df=df,
        front_fraction=frac,
        f_aero_f=df * frac,
        f_aero_r=df * (1.0 - frac),
        clamped=clamped,
        iterations=iteration,
    )


@dataclass(frozen=True)
class AxleState:
    c: float
    inv_d: float
    R: float
    K: float  # axle wheel rate, 2 k_w
    Kt: float  # axle tire rate, 2 k_t

    @property
    def k_ride(self) -> float:
        return self.K * self.Kt / (self.K + self.Kt)


@dataclass(frozen=True)
class ScoreContext:
    mp: MassProps
    beta_f: float
    regen_share: float
    sigma_fx_fraction: float
    tires: dict[str, TireParams]
    aero: AeroMap
    rh_static: dict[str, float]


def maneuver_antis(maneuver: str, front: AxleState, rear: AxleState, ctx: ScoreContext) -> dict[str, float]:
    if maneuver == "braking":
        return {
            "AD_f": anti_dive(front.c, ctx.beta_f, ctx.mp),
            "AL_r": anti_lift(rear.c, rear.inv_d, rear.R, ctx.beta_f, ctx.regen_share, ctx.mp),
        }
    if maneuver == "rwd_acceleration":
        return {"AS_r": anti_squat(rear.c, rear.inv_d, rear.R, ctx.mp)}
    raise ValueError(f"Unknown maneuver {maneuver!r}.")


def response_per_g(
    maneuver: str, front: AxleState, rear: AxleState, ctx: ScoreContext
) -> tuple[float, float]:
    """(dz_f, dz_r) in m per g of |a_x|."""
    dW = delta_w(ctx.mp, 1.0)
    antis = maneuver_antis(maneuver, front, rear, ctx)
    if maneuver == "braking":
        return dz_braking(dW, antis["AD_f"], antis["AL_r"], front.K, front.Kt, rear.K, rear.Kt)
    return dz_accel(dW, antis["AS_r"], front.K, front.Kt, rear.K, rear.Kt)


def score_bin(
    maneuver: str, v: float, ax_g: float, front: AxleState, rear: AxleState, ctx: ScoreContext
) -> dict[str, Any]:
    mp, beta = ctx.mp, ctx.beta_f
    W_f, W_r = static_axle_loads(mp)
    dW = delta_w(mp, ax_g)
    fx_total = mp.m * G * abs(ax_g)
    dz_f, dz_r = (abs(ax_g) * dz for dz in response_per_g(maneuver, front, rear, ctx))
    if maneuver == "braking":
        load_f, load_r = W_f + dW, W_r - dW
        fx = {"front": beta * fx_total / 2.0, "rear": (1.0 - beta) * fx_total / 2.0}
        s_fric, s_regen = rear_brake_shares(beta, ctx.regen_share)
        rear_share = s_fric + s_regen
        c_eff_r = (
            (s_fric * rear.c + s_regen * (rear.c - rear.R * rear.inv_d)) / rear_share
            if rear_share > 0.0
            else rear.c
        )
        c_eff = {"front": front.c, "rear": c_eff_r}
        carrying = AXLES
    else:
        load_f, load_r = W_f - dW, W_r + dW
        fx = {"front": 0.0, "rear": fx_total / 2.0}
        c_eff = {"front": front.c, "rear": rear.c - rear.R * rear.inv_d}
        carrying = ("rear",)

    rh = solve_ride_heights(
        (ctx.rh_static["front"], ctx.rh_static["rear"]),
        (dz_f, dz_r),
        (front.k_ride, rear.k_ride),
        ctx.aero,
        v,
    )
    fz = {"front": (load_f + rh.f_aero_f) / 2.0, "rear": (load_r + rh.f_aero_r) / 2.0}
    sigma_fz = {axle: abs(c_eff[axle]) * ctx.sigma_fx_fraction * fx[axle] for axle in AXLES}
    grip = sum(2.0 * expected_grip(fz[axle], sigma_fz[axle], ctx.tires[axle]) for axle in carrying)
    return {
        "maneuver": maneuver,
        "speed_m_per_s": v,
        "ax_g": ax_g,
        "dz_front_mm": 1000.0 * dz_f,
        "dz_rear_mm": 1000.0 * dz_r,
        "rh_front_m": rh.rh_f,
        "rh_rear_m": rh.rh_r,
        "downforce_n": rh.df,
        "aero_front_fraction": rh.front_fraction,
        "fz_front_n": fz["front"],
        "fz_rear_n": fz["rear"],
        "sigma_fz_front_n": sigma_fz["front"],
        "sigma_fz_rear_n": sigma_fz["rear"],
        "grip_n": grip,
        "rh_clamped": rh.clamped,
    }


def scenario_bins(scenario: dict[str, Any]) -> list[tuple[float, float, float]]:
    speeds = [float(v) for v in scenario["speeds_m_per_s"]]
    accels = [float(a) for a in scenario["ax_g"]]
    weights = scenario.get("weights", "uniform")
    if weights == "uniform":
        table = np.ones((len(speeds), len(accels)))
    else:
        table = np.asarray(weights, dtype=float)
        if table.shape != (len(speeds), len(accels)):
            raise ValueError(f"weights must be {len(speeds)} x {len(accels)} (speeds x ax_g).")
    if np.any(table < 0.0) or table.sum() <= 0.0:
        raise ValueError("weights must be non-negative with a positive sum.")
    table = table / table.sum()
    return [
        (v, a, float(table[i, j]))
        for i, v in enumerate(speeds)
        for j, a in enumerate(accels)
    ]


def score_maneuver(
    maneuver: str,
    bins: list[tuple[float, float, float]],
    front: AxleState,
    rear: AxleState,
    ctx: ScoreContext,
) -> tuple[float, list[dict[str, Any]]]:
    rows = []
    objective = 0.0
    for v, ax_g, weight in bins:
        row = score_bin(maneuver, v, ax_g, front, rear, ctx)
        row["weight"] = weight
        objective += weight * row["grip_n"]
        rows.append(row)
    return objective, rows


# ---------------------------------------------------------------------------
# Rig metrics and constraints


def linear_gain(x: np.ndarray, y: np.ndarray) -> float:
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if mask.sum() < 2 or np.std(x[mask]) < 1e-12:
        return math.nan
    return float(np.polyfit(x[mask], y[mask], 1)[0])


def rig_c_vs_heave(
    series: dict[str, np.ndarray], axle: str, h_cfg: float, l_cfg: float
) -> tuple[np.ndarray, np.ndarray]:
    prefix = RIG_PREFIX[axle]
    x = np.asarray(series[f"{prefix}_jacking_vs_heave_x"], dtype=float)
    anti = np.asarray(series[f"{prefix}_anti_vs_heave"], dtype=float)
    c = RIG_C_SIGN[axle] * anti / 100.0 * h_cfg / l_cfg
    order = np.argsort(x)
    return x[order], c[order]


def rig_c(series: dict[str, np.ndarray], axle: str, h_cfg: float, l_cfg: float) -> float:
    x, c = rig_c_vs_heave(series, axle, h_cfg, l_cfg)
    return float(np.interp(0.0, x, c))


@dataclass
class AxleMeasure:
    axle: str
    c_rig: float
    geom: AxleGeometry
    rate: AxleRate
    heave_m: np.ndarray
    c_vs_heave: np.ndarray
    anti_pct_vs_heave: np.ndarray
    toe_gain_deg_per_m: float
    caster_range_deg: float

    @property
    def state(self) -> AxleState:
        return AxleState(c=self.c_rig, inv_d=self.geom.inv_d, R=self.geom.R, K=self.rate.K, Kt=self.rate.Kt)


def axle_anti(axle: str, c: np.ndarray | float, geom: AxleGeometry, beta_f: float, mp: MassProps) -> Any:
    """The stage's anti for this axle: anti-dive at the front, anti-squat at the rear."""
    if axle == "front":
        return anti_dive(c, beta_f, mp)
    return anti_squat(c, geom.inv_d, geom.R, mp)


def constraint_margins(
    axle: str,
    measure: AxleMeasure | None,
    shift_mm: float,
    root_found: bool,
    limits: dict[str, Any],
) -> tuple[dict[str, float], bool]:
    """Margins (positive = inside the limit) and overall feasibility."""
    if measure is None:
        nan = math.nan
        return {
            "pickup_shift_margin_mm": nan,
            "toe_gain_margin_deg_per_m": nan,
            "caster_margin_deg": nan,
            "anti_range_margin_pct": nan,
        }, False
    lo, hi = (float(v) for v in limits["anti_pct_range"])
    anti = measure.anti_pct_vs_heave
    margins = {
        "pickup_shift_margin_mm": float(limits["max_pickup_shift_mm"]) - shift_mm
        if math.isfinite(shift_mm)
        else math.inf,  # baseline pickups are not moved
        "toe_gain_margin_deg_per_m": float(limits["max_abs_toe_gain_deg_per_m"])
        - abs(measure.toe_gain_deg_per_m),
        "caster_margin_deg": float(limits["max_caster_change_deg"]) - measure.caster_range_deg
        if axle == "front"
        else math.inf,
        "anti_range_margin_pct": float(min(np.min(anti) - lo, hi - np.max(anti)))
        if anti.size
        else math.nan,
    }
    # A NaN margin means the rig could not show the constraint holds.
    feasible = root_found and all(not math.isnan(v) and v >= 0.0 for v in margins.values())
    return margins, feasible


def ic_check(c_rig: float, c_ic: float, tol: float) -> tuple[bool, bool]:
    """(flagged, opposite_signs) for the rig slope against the Python IC slope."""
    flagged = abs(c_rig - c_ic) > tol
    opposite = abs(c_rig) > tol and abs(c_ic) > tol and (c_rig > 0.0) != (c_ic > 0.0)
    return flagged, opposite


# ---------------------------------------------------------------------------
# CSV helpers


def fmt(value: Any) -> Any:
    if isinstance(value, (bool, np.bool_)):
        return str(bool(value))
    if isinstance(value, (float, np.floating)):
        return repr(float(value))
    return value


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        fields += [key for key in row if key not in fields]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(value) for key, value in row.items()})


def read_csv(path: Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def upsert_stage_rows(path: Path, stage: str, rows: list[dict[str, Any]]) -> None:
    """Replace this stage's rows in a CSV shared by all stages."""
    kept: list[dict[str, Any]] = []
    if path.exists():
        kept = [row for row in read_csv(path) if row.get("stage") != stage]
    order = {name: i for i, name in enumerate(("baseline",) + STAGES)}
    merged = sorted(kept + rows, key=lambda row: order.get(str(row.get("stage")), 99))
    write_csv(path, merged)


def fnum(row: dict[str, Any], key: str) -> float:
    value = row.get(key, "")
    return math.nan if value in ("", None) else float(value)


def fbool(row: dict[str, Any], key: str) -> bool:
    return str(row.get(key)) == "True"


def read_simple_metrics_csv(path: Path) -> dict[str, float]:
    metrics: dict[str, float] = {}
    for row in read_csv(path):
        try:
            metrics[str(row["metric"])] = float(row["value"])
        except (TypeError, ValueError):
            metrics[str(row["metric"])] = math.nan
    return metrics


def write_series_csv(path: Path, series: dict[str, Any]) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["key", "index", "value"])
        for key, values in series.items():
            for index, value in enumerate(np.asarray(values, dtype=float).reshape(-1)):
                writer.writerow([key, index, repr(float(value))])


def read_series_csv(path: Path) -> dict[str, np.ndarray]:
    grouped: dict[str, list[tuple[int, float]]] = {}
    for row in read_csv(path):
        grouped.setdefault(row["key"], []).append((int(row["index"]), float(row["value"])))
    return {
        key: np.array([value for _, value in sorted(items)], dtype=float)
        for key, items in grouped.items()
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git_commit(path: Path) -> str:
    try:
        completed = subprocess.run(
            ["git", "-c", "safe.directory=*", "-C", str(path), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return completed.stdout.strip()


# ---------------------------------------------------------------------------
# BobSim build and rig run (helpers copied from DS-002)


def make_mos_content(variant_mo: Path, build_dir: Path, cfg: dict[str, Any]) -> str:
    option_lines = "\n".join(
        f'OpenModelica.Scripting.setCommandLineOptions("{option}");'
        for option in cfg.get("command_line_options", [])
    )
    intervals = cfg.get("intervals")
    intervals_line = (
        f"  numberOfIntervals={intervals},\n"
        if intervals is not None
        else ""
    )
    return f"""{option_lines}

clear();
loadModel(Modelica);
loadFile("{BOBLIB_PACKAGE.resolve().as_posix()}");
loadFile("{variant_mo.resolve().as_posix()}");

cd("{build_dir.resolve().as_posix()}");

buildModel(
  {cfg["model"]},
  startTime={cfg["start_time"]},
  stopTime={cfg["stop_time"]},
  outputFormat="csv",
{intervals_line}  tolerance={cfg["tolerance"]},
  method="{cfg["solver"]}",
  cflags="-O3 -march=native -mtune=native"
);

print(getErrorString());
"""


def find_executable(build_dir: Path, model_name: str = str(FOURPOST_CFG["model"])) -> Path | None:
    model = str(model_name)
    short = model.split(".")[-1]
    for candidate in (
        build_dir / model,
        build_dir / f"{model}.exe",
        build_dir / short,
        build_dir / f"{short}.exe",
    ):
        if candidate.exists():
            return candidate
    return None


def compile_variant(
    variant_dir: Path,
    *,
    standard_name: str = "FourPostEval",
    cfg: dict[str, Any] = FOURPOST_CFG,
) -> None:
    variant_mo = variant_dir / "variant.mo"
    build_dir = variant_dir / "build" / standard_name
    build_dir.mkdir(parents=True, exist_ok=True)
    mos_path = variant_dir / f"build_{standard_name}.mos"
    mos_path.write_text(make_mos_content(variant_mo, build_dir, cfg), encoding="utf-8")

    completed = subprocess.run(
        ["omc", str(mos_path)],
        cwd=str(variant_dir),
        capture_output=True,
        text=True,
    )
    log_path = variant_dir / f"compile_{standard_name}.log"
    log_path.write_text(
        f"returncode={completed.returncode}\n\nSTDOUT:\n{completed.stdout}\n\nSTDERR:\n{completed.stderr}",
        encoding="utf-8",
    )

    if find_executable(build_dir, str(cfg["model"])) is None:
        raise RuntimeError(
            f"OpenModelica did not produce {cfg['model']} for {variant_dir.name}. "
            f"See {as_repo_path(log_path)}"
        )


def stage_variant_text(
    variant_dir: Path,
    variant_text: str,
    *,
    rebuild: bool,
) -> bool:
    if rebuild and variant_dir.exists():
        shutil.rmtree(variant_dir)

    variant_dir.mkdir(parents=True, exist_ok=True)
    variant_mo = variant_dir / "variant.mo"
    existing_text = variant_mo.read_text(encoding="utf-8") if variant_mo.exists() else None
    variant_changed = existing_text != variant_text

    if rebuild or variant_changed:
        variant_mo.write_text(variant_text, encoding="utf-8")

    if variant_changed and not rebuild:
        shutil.rmtree(variant_dir / "build", ignore_errors=True)
        shutil.rmtree(variant_dir / "results", ignore_errors=True)

    return rebuild or variant_changed


def make_fourpost_config(
    vehicle: dict[str, Any],
    mp: MassProps,
    build_dir: Path,
    metrics_csv: Path,
    fourpost: dict[str, Any],
) -> dict[str, Any]:
    _wheelbase, track_front, track_rear = wheelbase_and_tracks(vehicle)
    return {
        "standard": "FourPostEval",
        "simulation": {
            "build_dir": str(build_dir),
            "exec_name": str(FOURPOST_CFG["model"]),
            "solver": FOURPOST_CFG["solver"],
            "stepSize": 0.5,
            "output_format": "csv",
            "log_level": "LOG_STATS,LOG_SOLVER,LOG_INIT",
            "no_grid": False,
            "no_event_emit": True,
            "stop_time": FOURPOST_CFG["stop_time"],
        },
        "execution": {"parallel": False, "cleanup": True, "stream_logs": False},
        # Total mass and CG height: the rig's anti % is referenced to these.
        "vehicle": {
            "mass": mp.m,
            "h_cg": mp.h,
            "track_front": track_front,
            "track_rear": track_rear,
            "wheelbase": mp.l,
        },
        "suspension": {
            axle: {
                "spring_rate": spring_rate_from_vehicle(vehicle, axle),
                "arb_rate": float(vehicle[axle]["actuation"]["stabar"]["rate_n_m_per_rad"]),
            }
            for axle in AXLES
        },
        "procedure": {
            "steerMagnitude": 0.0,
            "heaveMagnitude": float(fourpost["heave_magnitude_m"]),
            "rollMagnitude": float(fourpost["roll_magnitude_rad"]),
            "forceMagnitude": float(fourpost["force_magnitude_n"]),
        },
        "report": {"enabled": False, "metrics_csv_path": str(metrics_csv)},
    }


@dataclass
class RigResult:
    summary: dict[str, float]
    series: dict[str, np.ndarray]
    reused: bool = False


_RIG_VEHICLE = threading.local()


def _rig_vehicle() -> dict[str, Any]:
    return _RIG_VEHICLE.vehicle


def import_bobsim() -> tuple[Any, Callable[..., str]]:
    for path in (GENERATION_SCRIPTS, BOBSIM_ROOT):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from build_records import render_record
    from _3_StandardSim.FourPostEval import four_post_eval_sim

    # build_setup() reads BobLib's Generation/vehicle.yml; describe the
    # simulated variant instead. Thread-local so parallel runs stay separate.
    four_post_eval_sim._load_active_vehicle_yaml = _rig_vehicle
    return four_post_eval_sim, render_record


def load_rig_result(variant_dir: Path) -> RigResult:
    metrics_csv, series_csv = variant_dir / "metrics.csv", variant_dir / "series.csv"
    if not (metrics_csv.exists() and series_csv.exists()):
        raise FileNotFoundError(f"No rig results in {as_repo_path(variant_dir)}.")
    return RigResult(read_simple_metrics_csv(metrics_csv), read_series_csv(series_csv), reused=True)


def run_rig(
    variant_dir: Path,
    vehicle: dict[str, Any],
    mp: MassProps,
    fourpost: dict[str, Any],
    *,
    reuse: bool,
) -> RigResult:
    four_post_eval_sim, render_record = import_bobsim()
    vehicle_path = variant_dir / "vehicle.yml"
    record_text = render_record(vehicle, vehicle_path)
    changed = stage_variant_text(variant_dir, record_text, rebuild=not reuse)
    vehicle_path.write_text(dump_yaml(vehicle), encoding="utf-8")

    build_dir = variant_dir / "build" / "FourPostEval"
    metrics_csv, series_csv = variant_dir / "metrics.csv", variant_dir / "series.csv"
    config_path = variant_dir / "fourpost_eval_config.yml"
    config = make_fourpost_config(vehicle, mp, build_dir, metrics_csv, fourpost)
    config_text = yaml.safe_dump(to_builtin(config), sort_keys=False)
    same_config = config_path.exists() and config_path.read_text(encoding="utf-8") == config_text
    if reuse and not changed and same_config and metrics_csv.exists() and series_csv.exists():
        return load_rig_result(variant_dir)

    if changed or find_executable(build_dir, str(FOURPOST_CFG["model"])) is None:
        compile_variant(variant_dir, cfg=FOURPOST_CFG)
    metrics_csv.unlink(missing_ok=True)
    series_csv.unlink(missing_ok=True)
    config_path.write_text(config_text, encoding="utf-8")

    _RIG_VEHICLE.vehicle = vehicle
    result = four_post_eval_sim.FourPostEvalSim(to_builtin(config)).run()
    write_series_csv(series_csv, result["series"])
    return RigResult(read_simple_metrics_csv(metrics_csv), read_series_csv(series_csv))


# ---------------------------------------------------------------------------
# Stages


@dataclass
class Variant:
    stage: str
    variant_id: str
    axle: str
    sweep_index: int
    target_anti: float
    target_dz_per_g: float
    vehicle: dict[str, Any] | None
    info: dict[str, Any]


@dataclass
class Evaluated:
    variant: Variant
    measures: dict[str, AxleMeasure] | None = None
    rig_error: str = ""
    margins: dict[str, float] = field(default_factory=dict)
    feasible: bool = False
    ic_flags: dict[str, bool] = field(default_factory=dict)


@dataclass
class Baseline:
    measures: dict[str, AxleMeasure]
    objectives: dict[str, float]
    bins: dict[str, list[dict[str, Any]]]


def variant_id(axle: str, index: int, kind: str, anti: float) -> str:
    return f"{axle}{index:02d}_{kind}{100.0 * anti:+.0f}"


def sweep_targets(stage_cfg: dict[str, Any]) -> list[float]:
    sweep = stage_cfg["sweep_anti_pct"]
    return [float(v) / 100.0 for v in np.linspace(sweep["min"], sweep["max"], int(sweep["points"]))]


class Study:
    def __init__(self, paths: StudyPaths, study: dict[str, Any], args: argparse.Namespace):
        self.paths = paths
        self.study = study
        self.args = args
        self.reuse = bool(args.reuse)
        self.jobs = max(1, int(args.jobs))
        cfg = study["config"]
        self.fourpost = cfg["fourpost"]
        self.assumptions = cfg["assumptions"]
        self.limits = cfg["constraints"]
        self.ic_tol = float(cfg["ic_check_tolerance"])
        self.stage_cfg = {entry["id"]: entry for entry in study["stages"]}

        self.vehicle = read_vehicle(paths.vehicle)
        self.mp = mass_rollup(self.vehicle)
        self.beta_f = float(self.vehicle["brake"]["front_bias"])
        self.geom0 = {axle: axle_geometry(self.vehicle, axle) for axle in AXLES}
        self.freq = {axle: ride_frequency_target(self.assumptions, axle) for axle in AXLES}
        self.ctx = ScoreContext(
            mp=self.mp,
            beta_f=self.beta_f,
            regen_share=float(self.assumptions["regen_share_of_rear_braking"]),
            sigma_fx_fraction=float(self.assumptions["sigma_fx_fraction"]),
            tires={axle: read_tire_params(tire_path(self.vehicle, axle)) for axle in AXLES},
            aero=AeroMap.from_vehicle(self.vehicle),
            rh_static={axle: float(self.assumptions["static_ride_height_m"][axle]) for axle in AXLES},
        )
        self.bins = {m: scenario_bins(self.assumptions["scenarios"][m]) for m in MANEUVERS}
        self.dW1 = delta_w(self.mp, 1.0)

    # -- rig -----------------------------------------------------------------

    def rig_runs(self, jobs: list[tuple[str, str, dict[str, Any]]]) -> dict[str, RigResult | Exception]:
        def one(job: tuple[str, str, dict[str, Any]]) -> tuple[str, RigResult | Exception]:
            stage, vid, vehicle = job
            start = time.monotonic()
            try:
                result = run_rig(
                    self.paths.work / stage / vid, vehicle, self.mp, self.fourpost, reuse=self.reuse
                )
            except Exception as exc:  # a failed variant is reported, not fatal
                print(f"  [{stage}] {vid}: FAILED ({exc})", flush=True)
                return vid, exc
            how = "reused" if result.reused else f"built and run in {time.monotonic() - start:.0f} s"
            print(f"  [{stage}] {vid}: {how}", flush=True)
            return vid, result

        if self.jobs > 1 and len(jobs) > 1:
            with ThreadPoolExecutor(max_workers=min(self.jobs, len(jobs))) as pool:
                return dict(pool.map(one, jobs))
        return dict(one(job) for job in jobs)

    def measure(self, vehicle: dict[str, Any], rig: RigResult) -> dict[str, AxleMeasure]:
        measures = {}
        for axle in AXLES:
            prefix = RIG_PREFIX[axle]
            geom = axle_geometry(vehicle, axle)
            mr = rig.summary.get(f"avg_motion_ratio_{axle}", math.nan)
            heave, c_vs_heave = rig_c_vs_heave(rig.series, axle, self.mp.h, self.mp.l)
            measures[axle] = AxleMeasure(
                axle=axle,
                c_rig=float(np.interp(0.0, heave, c_vs_heave)),
                geom=geom,
                rate=axle_rate(vehicle, self.mp, axle, self.freq[axle], mr),
                heave_m=heave,
                c_vs_heave=c_vs_heave,
                anti_pct_vs_heave=100.0 * np.asarray(
                    axle_anti(axle, c_vs_heave, geom, self.beta_f, self.mp)
                ),
                toe_gain_deg_per_m=math.degrees(
                    linear_gain(rig.series["heave"], rig.series[f"{prefix}_l_toe_vs_heave"])
                ),
                caster_range_deg=math.degrees(
                    float(np.ptp(rig.series[f"{prefix}_l_caster_vs_heave"]))
                ),
            )
        return measures

    def check_ic(self, stage: str, vid: str, measures: dict[str, AxleMeasure]) -> dict[str, bool]:
        flags = {}
        for axle, meas in measures.items():
            flagged, opposite = ic_check(meas.c_rig, meas.geom.c, self.ic_tol)
            if opposite:
                raise SystemExit(
                    f"{stage}/{vid} {axle}: rig slope c = {meas.c_rig:+.4f} and IC slope "
                    f"c = {meas.geom.c:+.4f} have opposite signs, so d and the targets "
                    "would be wrong. Check RIG_C_SIGN in run.py."
                )
            flags[axle] = flagged
        return flags

    def metric_row(self, stage: str, vid: str, measures: dict[str, AxleMeasure], rig: RigResult) -> dict[str, Any]:
        row: dict[str, Any] = {"stage": stage, "variant_id": vid}
        for axle, meas in measures.items():
            row.update(
                {
                    f"c_rig_{axle}": meas.c_rig,
                    f"c_ic_{axle}": meas.geom.c,
                    f"d_{axle}_m": meas.geom.d,
                    f"inv_d_{axle}_per_m": meas.geom.inv_d,
                    f"mr_{axle}": meas.rate.mr,
                    f"k_w_{axle}_n_per_m": meas.rate.k_w,
                    f"k_s_{axle}_n_per_m": meas.rate.k_s,
                    f"ride_frequency_{axle}_hz": meas.rate.f_hz,
                    f"toe_gain_{axle}_deg_per_m": meas.toe_gain_deg_per_m,
                    f"caster_range_{axle}_deg": meas.caster_range_deg,
                    f"anti_pct_{axle}_min": float(np.min(meas.anti_pct_vs_heave)),
                    f"anti_pct_{axle}_max": float(np.max(meas.anti_pct_vs_heave)),
                }
            )
        row.update({key: value for key, value in sorted(rig.summary.items()) if key.startswith("avg_")})
        return row

    def bin_rows(self, stage: str, vid: str, rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [{"stage": stage, "variant_id": vid, **row} for row in rows]

    def evaluate(self, variants: list[Variant]) -> list[Evaluated]:
        runnable = [(v.stage, v.variant_id, v.vehicle) for v in variants if v.vehicle is not None]
        rigs = self.rig_runs(runnable)
        self._rigs = rigs
        evaluated = []
        for variant in variants:
            ev = Evaluated(variant)
            rig = rigs.get(variant.variant_id)
            if isinstance(rig, Exception):
                ev.rig_error = str(rig)
            if isinstance(rig, RigResult):
                ev.measures = self.measure(variant.vehicle, rig)
                ev.ic_flags = self.check_ic(variant.stage, variant.variant_id, ev.measures)
            ev.margins, ev.feasible = constraint_margins(
                variant.axle,
                ev.measures[variant.axle] if ev.measures else None,
                variant.info["max_pickup_shift_mm"],
                bool(variant.info["root_found"]),
                self.limits,
            )
            evaluated.append(ev)
        return evaluated

    def variant_rows(self, variants: list[Variant]) -> list[dict[str, Any]]:
        return [
            {
                "stage": v.stage,
                "variant_id": v.variant_id,
                "axle": v.axle,
                "sweep_index": v.sweep_index,
                "target_anti_pct": 100.0 * v.target_anti,
                "target_dz_mm_per_g": 1000.0 * v.target_dz_per_g,
                **v.info,
            }
            for v in variants
        ]

    # -- stage 0 ---------------------------------------------------------------

    def run_baseline(self) -> Baseline:
        print("Stage 0: baseline", flush=True)
        rig = self.rig_runs([("baseline", "baseline", self.vehicle)])["baseline"]
        if isinstance(rig, Exception):
            raise SystemExit(f"Baseline rig run failed: {rig}")
        measures = self.measure(self.vehicle, rig)
        objectives, bins = {}, {}
        for maneuver in MANEUVERS:
            objectives[maneuver], bins[maneuver] = score_maneuver(
                maneuver, self.bins[maneuver], measures["front"].state, measures["rear"].state, self.ctx
            )
        upsert_stage_rows(
            self.paths.outputs / "fourpost_metrics.csv",
            "baseline",
            [self.metric_row("baseline", "baseline", measures, rig)],
        )
        upsert_stage_rows(
            self.paths.outputs / "bin_scores.csv",
            "baseline",
            [row for m in MANEUVERS for row in self.bin_rows("baseline", "baseline", bins[m])],
        )
        for axle, meas in measures.items():
            print(
                f"  {axle}: c_rig {meas.c_rig:+.4f}, c_ic {meas.geom.c:+.4f}, d {meas.geom.d:.3g} m, "
                f"MR {meas.rate.mr:.4f}, k_w {meas.rate.k_w:,.0f} N/m ({meas.rate.f_hz:.2f} Hz), "
                f"k_s {meas.rate.k_s:,.0f} N/m",
                flush=True,
            )
        return Baseline(measures, objectives, bins)

    # -- stage 1 ---------------------------------------------------------------

    def run_anti_squat(self, base: Baseline) -> None:
        stage, cfg = "anti-squat", self.stage_cfg["anti-squat"]
        print(f"Stage 1: {stage}", flush=True)
        geom = self.geom0["rear"]
        front0, rear0 = base.measures["front"].state, base.measures["rear"].state
        variants = []
        for index, anti in enumerate(sweep_targets(cfg)):
            target_c = c_for_anti_squat(anti, geom.inv_d, geom.R, self.mp)
            vehicle, info = make_hardpoint_variant(self.vehicle, "rear", target_c, geom.inv_d)
            variants.append(
                Variant(
                    stage=stage,
                    variant_id=variant_id("rear", index, "as", anti),
                    axle="rear",
                    sweep_index=index,
                    target_anti=anti,
                    target_dz_per_g=dz_accel(self.dW1, anti, front0.K, front0.Kt, rear0.K, rear0.Kt)[1],
                    vehicle=vehicle,
                    info=info,
                )
            )
        evaluated = self.evaluate(variants)

        summary, metrics, bins = [], [], []
        maneuver = cfg["maneuver"]
        for ev in evaluated:
            v = ev.variant
            row: dict[str, Any] = {
                "variant_id": v.variant_id,
                "sweep_index": v.sweep_index,
                "target_anti_squat_pct": 100.0 * v.target_anti,
                "target_dz_rear_mm_per_g": 1000.0 * v.target_dz_per_g,
                "ran": ev.measures is not None,
            }
            if ev.measures:
                front, rear = ev.measures["front"].state, ev.measures["rear"].state
                objective, rows = score_maneuver(maneuver, self.bins[maneuver], front, rear, self.ctx)
                achieved = anti_squat(rear.c, rear.inv_d, rear.R, self.mp)
                row.update(
                    {
                        "achieved_anti_squat_pct": 100.0 * achieved,
                        "achieved_dz_rear_mm_per_g": 1000.0 * response_per_g(maneuver, front, rear, self.ctx)[1],
                        "c_rig_rear": rear.c,
                        "c_ic_rear": ev.measures["rear"].geom.c,
                        "inv_d_rear_per_m": rear.inv_d,
                        "mr_rear": ev.measures["rear"].rate.mr,
                        "k_w_rear_n_per_m": ev.measures["rear"].rate.k_w,
                        "k_s_rear_n_per_m": ev.measures["rear"].rate.k_s,
                        "objective_n": objective,
                        "delta_objective_n": objective - base.objectives[maneuver],
                    }
                )
                metrics.append(self.metric_row(stage, v.variant_id, ev.measures, self._rigs[v.variant_id]))
                bins += self.bin_rows(stage, v.variant_id, rows)
            row.update(
                {
                    "feasible": ev.feasible,
                    **ev.margins,
                    "max_pickup_shift_mm": v.info["max_pickup_shift_mm"],
                    "tie_pickup_shift_mm": v.info["tie_pickup_shift_mm"],
                    "root_found": v.info["root_found"],
                    "ic_flag": any(ev.ic_flags.values()),
                    "rig_error": ev.rig_error,
                    "chosen": False,
                }
            )
            summary.append(row)

        chosen = self.choose(summary)
        self.write_stage(stage, variants, summary, metrics, bins)
        plot_anti_squat(summary, self.paths.plots / "anti_squat_objective.png")
        if chosen is None:
            raise SystemExit(self.no_feasible_message(stage, summary))
        print(
            f"  chosen rear {chosen['variant_id']}: anti-squat {chosen['achieved_anti_squat_pct']:.1f}%, "
            f"{chosen['achieved_dz_rear_mm_per_g']:+.2f} mm/g, objective {chosen['delta_objective_n']:+.2f} N "
            "vs baseline",
            flush=True,
        )

    # -- stage 2 ---------------------------------------------------------------

    def rear_states(self) -> tuple[list[dict[str, str]], dict[str, AxleState]]:
        rows = read_csv(self.paths.outputs / STAGE_SUMMARY["anti-squat"])
        geom = self.geom0["rear"]
        k_t = float(self.vehicle["rear"]["tire"]["vertical_stiffness_n_per_m"])
        states = {
            row["variant_id"]: AxleState(
                c=fnum(row, "c_rig_rear"),
                inv_d=fnum(row, "inv_d_rear_per_m"),
                R=geom.R,
                K=2.0 * fnum(row, "k_w_rear_n_per_m"),
                Kt=2.0 * k_t,
            )
            for row in rows
            if fbool(row, "ran")
        }
        return rows, states

    def run_anti_dive(self, base: Baseline) -> None:
        stage, cfg = "anti-dive", self.stage_cfg["anti-dive"]
        print(f"Stage 2: {stage}", flush=True)
        rear_rows, rear_states = self.rear_states()
        chosen_rear = next(row for row in rear_rows if fbool(row, "chosen"))
        rear = rear_states[chosen_rear["variant_id"]]
        rear_vehicle = read_vehicle(self.paths.work / "anti-squat" / chosen_rear["variant_id"] / "vehicle.yml")
        n = int(cfg.get("rear_sensitivity_neighbors", 1))
        by_index = {int(row["sweep_index"]): row["variant_id"] for row in rear_rows if row["variant_id"] in rear_states}
        chosen_index = int(chosen_rear["sweep_index"])
        neighbors = {
            offset: by_index[chosen_index + offset]
            for offset in range(-n, n + 1)
            if offset != 0 and chosen_index + offset in by_index
        }

        geom = self.geom0["front"]
        front0 = base.measures["front"].state
        AL_r = anti_lift(rear.c, rear.inv_d, rear.R, self.beta_f, self.ctx.regen_share, self.mp)
        variants = []
        for index, anti in enumerate(sweep_targets(cfg)):
            target_c = c_for_anti_dive(anti, self.beta_f, self.mp)
            vehicle, info = make_hardpoint_variant(rear_vehicle, "front", target_c, geom.inv_d)
            variants.append(
                Variant(
                    stage=stage,
                    variant_id=variant_id("front", index, "ad", anti),
                    axle="front",
                    sweep_index=index,
                    target_anti=anti,
                    target_dz_per_g=dz_braking(self.dW1, anti, AL_r, front0.K, front0.Kt, rear.K, rear.Kt)[0],
                    vehicle=vehicle,
                    info=info,
                )
            )
        evaluated = self.evaluate(variants)

        summary, metrics, bins = [], [], []
        maneuver = cfg["maneuver"]
        for ev in evaluated:
            v = ev.variant
            row: dict[str, Any] = {
                "variant_id": v.variant_id,
                "sweep_index": v.sweep_index,
                "rear_variant_id": chosen_rear["variant_id"],
                "target_anti_dive_pct": 100.0 * v.target_anti,
                "target_dz_front_mm_per_g": 1000.0 * v.target_dz_per_g,
                "ran": ev.measures is not None,
            }
            if ev.measures:
                front = ev.measures["front"].state
                objective, rows = score_maneuver(maneuver, self.bins[maneuver], front, rear, self.ctx)
                dz_f, dz_r = response_per_g(maneuver, front, rear, self.ctx)
                row.update(
                    {
                        "achieved_anti_dive_pct": 100.0 * anti_dive(front.c, self.beta_f, self.mp),
                        "achieved_anti_lift_pct": 100.0 * AL_r,
                        "achieved_dz_front_mm_per_g": 1000.0 * dz_f,
                        "achieved_dz_rear_mm_per_g": 1000.0 * dz_r,
                        "c_rig_front": front.c,
                        "c_ic_front": ev.measures["front"].geom.c,
                        "inv_d_front_per_m": front.inv_d,
                        "mr_front": ev.measures["front"].rate.mr,
                        "k_w_front_n_per_m": ev.measures["front"].rate.k_w,
                        "k_s_front_n_per_m": ev.measures["front"].rate.k_s,
                        "c_rig_rear_this_run": ev.measures["rear"].c_rig,
                        "objective_n": objective,
                        "delta_objective_n": objective - base.objectives[maneuver],
                    }
                )
                for offset, rear_id in neighbors.items():
                    other, _ = score_maneuver(maneuver, self.bins[maneuver], front, rear_states[rear_id], self.ctx)
                    row[f"objective_rear_{offset:+d}_n"] = other
                    row[f"delta_objective_rear_{offset:+d}_n"] = other - base.objectives[maneuver]
                    row[f"rear_{offset:+d}_variant_id"] = rear_id
                metrics.append(self.metric_row(stage, v.variant_id, ev.measures, self._rigs[v.variant_id]))
                bins += self.bin_rows(stage, v.variant_id, rows)
            row.update(
                {
                    "feasible": ev.feasible,
                    **ev.margins,
                    "max_pickup_shift_mm": v.info["max_pickup_shift_mm"],
                    "tie_pickup_shift_mm": v.info["tie_pickup_shift_mm"],
                    "root_found": v.info["root_found"],
                    "ic_flag": any(ev.ic_flags.values()),
                    "rig_error": ev.rig_error,
                    "chosen": False,
                }
            )
            summary.append(row)

        chosen = self.choose(summary)
        self.write_stage(stage, variants, summary, metrics, bins)
        plot_anti_dive(summary, neighbors, self.paths.plots / "anti_dive_objective.png")
        if chosen is None:
            raise SystemExit(self.no_feasible_message(stage, summary))
        print(
            f"  chosen front {chosen['variant_id']}: anti-dive {chosen['achieved_anti_dive_pct']:.1f}%, "
            f"{chosen['achieved_dz_front_mm_per_g']:+.2f} mm/g, objective {chosen['delta_objective_n']:+.2f} N "
            "vs baseline",
            flush=True,
        )

    # -- stage 3 ---------------------------------------------------------------

    def run_validation(self, base: Baseline) -> None:
        stage = "validation"
        print(f"Stage 3: {stage}", flush=True)
        rear_rows, rear_states = self.rear_states()
        front_rows = read_csv(self.paths.outputs / STAGE_SUMMARY["anti-dive"])
        front_ran = sorted((r for r in front_rows if fbool(r, "ran")), key=lambda r: int(r["sweep_index"]))
        rear_ran = sorted((r for r in rear_rows if fbool(r, "ran")), key=lambda r: int(r["sweep_index"]))
        chosen_front = next(r for r in front_rows if fbool(r, "chosen"))
        chosen_rear = next(r for r in rear_rows if fbool(r, "chosen"))
        k_t_f = float(self.vehicle["front"]["tire"]["vertical_stiffness_n_per_m"])

        def front_state(row: dict[str, str]) -> AxleState:
            return AxleState(
                c=fnum(row, "c_rig_front"),
                inv_d=fnum(row, "inv_d_front_per_m"),
                R=self.geom0["front"].R,
                K=2.0 * fnum(row, "k_w_front_n_per_m"),
                Kt=2.0 * k_t_f,
            )

        cars = [("chosen", chosen_front, chosen_rear)] + [
            (f"front_{fa}_rear_{ra}", f_row, r_row)
            for fa, f_row in (("min", front_ran[0]), ("max", front_ran[-1]))
            for ra, r_row in (("min", rear_ran[0]), ("max", rear_ran[-1]))
        ]

        # Stage-2 cars already carry the chosen rear, so reuse their rig runs.
        stage2 = self.paths.work / "anti-dive"
        vehicles, rigs, jobs = {}, {}, []
        for car_id, f_row, r_row in cars:
            if r_row["variant_id"] == chosen_rear["variant_id"]:
                vehicles[car_id] = read_vehicle(stage2 / f_row["variant_id"] / "vehicle.yml")
                rigs[car_id] = load_rig_result(stage2 / f_row["variant_id"])
                print(f"  [{stage}] {car_id}: reused anti-dive/{f_row['variant_id']}", flush=True)
                continue
            vehicle = read_vehicle(self.paths.work / "anti-squat" / r_row["variant_id"] / "vehicle.yml")
            front_vehicle = read_vehicle(stage2 / f_row["variant_id"] / "vehicle.yml")
            for block, key in MOVED_PICKUPS:
                vehicle["front"][block][key] = front_vehicle["front"][block][key]
            vehicles[car_id] = vehicle
            jobs.append((stage, car_id, vehicle))
        for car_id, rig in self.rig_runs(jobs).items():
            if isinstance(rig, Exception):
                raise SystemExit(f"Validation rig run {car_id} failed: {rig}")
            rigs[car_id] = rig

        rows, metrics, bins, anti_curves = [], [], [], {}
        chosen_bins: dict[str, list[dict[str, Any]]] = {}
        for car_id, f_row, r_row in cars:
            measures = self.measure(vehicles[car_id], rigs[car_id])
            flags = self.check_ic(stage, car_id, measures)
            predicted = {"front": front_state(f_row), "rear": rear_states[r_row["variant_id"]]}
            achieved = {axle: measures[axle].state for axle in AXLES}
            row: dict[str, Any] = {
                "car": car_id,
                "front_variant_id": f_row["variant_id"],
                "rear_variant_id": r_row["variant_id"],
                "target_anti_dive_pct": fnum(f_row, "target_anti_dive_pct"),
                "achieved_anti_dive_pct": 100.0 * anti_dive(achieved["front"].c, self.beta_f, self.mp),
                "target_anti_squat_pct": fnum(r_row, "target_anti_squat_pct"),
                "achieved_anti_squat_pct": 100.0
                * anti_squat(achieved["rear"].c, achieved["rear"].inv_d, achieved["rear"].R, self.mp),
                "target_dz_front_braking_mm_per_g": fnum(f_row, "target_dz_front_mm_per_g"),
                "target_dz_rear_accel_mm_per_g": fnum(r_row, "target_dz_rear_mm_per_g"),
            }
            for axle in AXLES:
                row[f"c_rig_{axle}_predicted"] = predicted[axle].c
                row[f"c_rig_{axle}"] = achieved[axle].c
                row[f"c_ic_{axle}"] = measures[axle].geom.c
                row[f"ic_flag_{axle}"] = flags[axle]
            for maneuver in MANEUVERS:
                label = "braking" if maneuver == "braking" else "accel"
                pred_obj, _ = score_maneuver(maneuver, self.bins[maneuver], predicted["front"], predicted["rear"], self.ctx)
                ach_obj, ach_rows = score_maneuver(maneuver, self.bins[maneuver], achieved["front"], achieved["rear"], self.ctx)
                pred_dz = response_per_g(maneuver, predicted["front"], predicted["rear"], self.ctx)
                ach_dz = response_per_g(maneuver, achieved["front"], achieved["rear"], self.ctx)
                for i, axle in enumerate(AXLES):
                    row[f"dz_{axle}_{label}_mm_per_g_predicted"] = 1000.0 * pred_dz[i]
                    row[f"dz_{axle}_{label}_mm_per_g"] = 1000.0 * ach_dz[i]
                row[f"objective_{label}_n_predicted"] = pred_obj
                row[f"objective_{label}_n"] = ach_obj
                row[f"delta_objective_{label}_n"] = ach_obj - base.objectives[maneuver]
                bins += self.bin_rows(stage, car_id, ach_rows)
                if car_id == "chosen":
                    chosen_bins[maneuver] = ach_rows
            feasible = True
            for axle, source in (("front", f_row), ("rear", r_row)):
                margins, ok = constraint_margins(
                    axle, measures[axle], fnum(source, "max_pickup_shift_mm"), True, self.limits
                )
                feasible &= ok
                row.update({f"{key}_{axle}": value for key, value in margins.items()})
            row["feasible"] = feasible
            rows.append(row)
            metrics.append(self.metric_row(stage, car_id, measures, rigs[car_id]))
            anti_curves[car_id] = {
                axle: (measures[axle].heave_m, measures[axle].anti_pct_vs_heave) for axle in AXLES
            }

        write_csv(self.paths.outputs / "validation.csv", rows)
        upsert_stage_rows(self.paths.outputs / "fourpost_metrics.csv", stage, metrics)
        upsert_stage_rows(self.paths.outputs / "bin_scores.csv", stage, bins)
        plot_validation_anti(anti_curves, self.paths.plots / "validation_anti_vs_heave.png")
        plot_ride_heights(chosen_bins, base.bins, self.paths.plots / "ride_height_vs_speed.png")

        chosen_measures = self.measure(vehicles["chosen"], rigs["chosen"])
        chosen_vehicle_text = chosen_vehicle_yaml(vehicles["chosen"], chosen_measures, self.freq)
        (self.paths.outputs / "chosen_vehicle.yml").write_text(chosen_vehicle_text, encoding="utf-8")
        write_reports(self, base, rows, rear_rows, front_rows, vehicles["chosen"], chosen_measures)
        print(
            f"  chosen car: braking {rows[0]['delta_objective_braking_n']:+.2f} N, "
            f"acceleration {rows[0]['delta_objective_accel_n']:+.2f} N vs baseline; "
            f"feasible {rows[0]['feasible']}",
            flush=True,
        )

    # -- shared ----------------------------------------------------------------

    @staticmethod
    def choose(summary: list[dict[str, Any]]) -> dict[str, Any] | None:
        feasible = [row for row in summary if row["feasible"]]
        if not feasible:
            return None
        chosen = max(feasible, key=lambda row: row["objective_n"])
        chosen["chosen"] = True
        return chosen

    def no_feasible_message(self, stage: str, summary: list[dict[str, Any]]) -> str:
        lines = [f"No feasible {stage} variant. Constraint margins (negative = violated):"]
        for row in summary:
            margins = ", ".join(
                f"{key} {row[key]:.3g}" for key in row if key.endswith(("_margin_mm", "_margin_deg_per_m", "_margin_deg", "_margin_pct"))
            )
            reason = "" if row["root_found"] else " (no hardpoint root in +/-20 deg)"
            reason += f" (rig failed: {row['rig_error']})" if row["rig_error"] else ""
            lines.append(f"  {row['variant_id']}: {margins}{reason}")
        lines.append(f"See {as_repo_path(self.paths.outputs / STAGE_SUMMARY[stage])}.")
        return "\n".join(lines)

    def write_stage(
        self,
        stage: str,
        variants: list[Variant],
        summary: list[dict[str, Any]],
        metrics: list[dict[str, Any]],
        bins: list[dict[str, Any]],
    ) -> None:
        write_csv(self.paths.outputs / STAGE_SUMMARY[stage], summary)
        upsert_stage_rows(self.paths.outputs / "variants.csv", stage, self.variant_rows(variants))
        upsert_stage_rows(self.paths.outputs / "fourpost_metrics.csv", stage, metrics)
        upsert_stage_rows(self.paths.outputs / "bin_scores.csv", stage, bins)

    def write_provenance(self) -> None:
        rows = [
            ("timestamp_utc", datetime.now(timezone.utc).isoformat(timespec="seconds")),
            ("command", " ".join([as_repo_path(Path(sys.argv[0]))] + sys.argv[1:])),
            ("stage", self.args.stage),
            ("reuse", self.reuse),
            ("study", as_repo_path(self.paths.study_yml)),
            ("study_sha256", sha256(self.paths.study_yml)),
            ("vehicle", as_repo_path(self.paths.vehicle)),
            ("vehicle_sha256", sha256(self.paths.vehicle)),
            ("bobsim_commit", git_commit(BOBSIM_ROOT)),
            ("boblib_commit", git_commit(BOBLIB_ROOT)),
            ("rig_c_sign_front", RIG_C_SIGN["front"]),
            ("rig_c_sign_rear", RIG_C_SIGN["rear"]),
        ]
        write_csv(self.paths.outputs / "run_provenance.csv", [{"key": k, "value": v} for k, v in rows])


# ---------------------------------------------------------------------------
# Chosen vehicle, plots and reports


def chosen_vehicle_yaml(
    vehicle: dict[str, Any], measures: dict[str, AxleMeasure], freq: dict[str, float | None]
) -> str:
    """The chosen car; under the ride-frequency hold, springs are set to the implied rates."""
    data = copy.deepcopy(vehicle)
    comments = {}
    for axle in AXLES:
        rate = measures[axle].rate
        if freq[axle] is None:
            continue
        data[axle]["actuation"]["shock"]["spring_table"] = {"table": [[0.0, 0.0], [1.0, float(rate.k_s)]]}
        comments[axle] = (
            f"# DS-010: {freq[axle]:.2f} Hz ride frequency hold: wheel rate {rate.k_w:,.0f} N/m, "
            f"motion ratio {rate.mr:.4f}, spring rate {rate.k_s:,.0f} N/m."
        )
    lines = dump_yaml(data).splitlines()
    out, axle = [], None
    for line in lines:
        if not line.startswith(" ") and line.rstrip(":") in AXLES:
            axle = line.rstrip(":")
        if line.strip() == "spring_table:" and axle in comments:
            indent = line[: len(line) - len(line.lstrip())]
            out.append(indent + comments[axle])
        out.append(line)
    header = (
        "# DS-010 chosen anti geometry, written by studies/DS-010-anti-geometry/run.py.\n"
        "# Only the inboard wishbone pickups (and, under the ride-frequency hold, the\n"
        "# spring tables) differ from the study vehicle.\n"
    )
    return header + "\n".join(out) + "\n"


def plot_anti_squat(summary: list[dict[str, Any]], path: Path) -> None:
    ran = [row for row in summary if row["ran"]]
    if not ran:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = [row["achieved_dz_rear_mm_per_g"] for row in ran]
    y = [row["delta_objective_n"] for row in ran]
    ax.plot(x, y, color="0.6", zorder=1)
    for row, xi, yi in zip(ran, x, y):
        style = dict(marker="o", s=40, zorder=2)
        if row["feasible"]:
            ax.scatter(xi, yi, color="tab:blue", **style)
        else:
            ax.scatter(xi, yi, facecolors="none", edgecolors="tab:red", **style)
        ax.annotate(f"{row['achieved_anti_squat_pct']:.0f}%", (xi, yi), textcoords="offset points", xytext=(4, 4), fontsize=8)
        if row["chosen"]:
            ax.scatter(xi, yi, marker="*", s=220, color="tab:orange", zorder=3, label="chosen")
    ax.axhline(0.0, color="0.3", linewidth=0.8)
    ax.set_xlabel("rear ride-height change, RWD acceleration (mm/g)")
    ax.set_ylabel("objective vs baseline (N)")
    ax.set_title("Stage 1: anti-squat (labels: anti-squat %; hollow = infeasible)")
    if any(row["chosen"] for row in ran):
        ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_anti_dive(summary: list[dict[str, Any]], neighbors: dict[int, str], path: Path) -> None:
    ran = [row for row in summary if row["ran"]]
    if not ran:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    x = [row["achieved_dz_front_mm_per_g"] for row in ran]
    rear_id = ran[0]["rear_variant_id"]
    ax.plot(x, [row["delta_objective_n"] for row in ran], marker="o", label=f"rear {rear_id} (chosen)")
    for offset, other in sorted(neighbors.items()):
        key = f"delta_objective_rear_{offset:+d}_n"
        ax.plot(x, [row.get(key, math.nan) for row in ran], marker=".", linestyle="--", label=f"rear {other}")
    for row, xi in zip(ran, x):
        if not row["feasible"]:
            ax.scatter(xi, row["delta_objective_n"], facecolors="none", edgecolors="tab:red", s=80, zorder=3)
        if row["chosen"]:
            ax.scatter(xi, row["delta_objective_n"], marker="*", s=220, color="tab:orange", zorder=4)
    ax.axhline(0.0, color="0.3", linewidth=0.8)
    ax.set_xlabel("front ride-height change, braking (mm/g)")
    ax.set_ylabel("objective vs baseline (N)")
    ax.set_title("Stage 2: anti-dive (red ring = infeasible, star = chosen)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_validation_anti(curves: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharex=True)
    for car_id, axles in curves.items():
        width = 2.2 if car_id == "chosen" else 1.0
        for ax, axle in zip(axes, AXLES):
            heave, anti = axles[axle]
            ax.plot(1000.0 * heave, anti, marker=".", linewidth=width, label=car_id)
    for ax, title in zip(axes, ("front anti-dive %", "rear anti-squat %")):
        ax.set_title(title)
        ax.set_xlabel("rig heave (mm)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("anti (%)")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_ride_heights(
    chosen: dict[str, list[dict[str, Any]]], baseline: dict[str, list[dict[str, Any]]], path: Path
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True)
    for r, maneuver in enumerate(MANEUVERS):
        for c, axle in enumerate(AXLES):
            ax = axes[r][c]
            for rows, style, label in ((chosen[maneuver], "-", "chosen"), (baseline[maneuver], "--", "baseline")):
                for i, ax_g in enumerate(sorted({row["ax_g"] for row in rows})):
                    sel = sorted((row for row in rows if row["ax_g"] == ax_g), key=lambda row: row["speed_m_per_s"])
                    ax.plot(
                        [row["speed_m_per_s"] for row in sel],
                        [1000.0 * row[f"rh_{axle}_m"] for row in sel],
                        linestyle=style,
                        color=f"C{i}",
                        marker="o" if style == "-" else None,
                        label=f"{label}, {ax_g:g} g",
                    )
            ax.set_title(f"{maneuver}: {axle} ride height")
            ax.set_ylabel("ride height (mm)")
            ax.grid(alpha=0.3)
    for ax in axes[1]:
        ax.set_xlabel("speed (m/s)")
    axes[0][1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _f(value: Any, digits: int = 2) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(number):
        return "-"
    if math.isinf(number):
        return "inf"
    return f"{number:,.{digits}f}"


def _table(header: Sequence[str], rows: Sequence[Sequence[Any]]) -> list[str]:
    lines = ["| " + " | ".join(header) + " |", "| " + " | ".join("---" for _ in header) + " |"]
    lines += ["| " + " | ".join(str(cell) for cell in row) + " |" for row in rows]
    return lines


def write_reports(
    study: Study,
    base: Baseline,
    validation: list[dict[str, Any]],
    rear_rows: list[dict[str, str]],
    front_rows: list[dict[str, str]],
    chosen_vehicle: dict[str, Any],
    chosen_measures: dict[str, AxleMeasure],
) -> None:
    chosen = validation[0]
    chosen_front = next(r for r in front_rows if fbool(r, "chosen"))
    chosen_rear = next(r for r in rear_rows if fbool(r, "chosen"))
    variants = {row["variant_id"]: row for row in read_csv(study.paths.outputs / "variants.csv")}
    a = study.assumptions
    lines = [
        "# DS-010 Anti-Dive / Anti-Squat Geometry: Results",
        "",
        f"Generated by `{as_repo_path(Path(__file__))}` on "
        f"{datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')} from `{as_repo_path(study.paths.vehicle)}` "
        f"(BobSim {git_commit(BOBSIM_ROOT)[:7]}, BobLib {git_commit(BOBLIB_ROOT)[:7]}). "
        "Every number rests on the provisional assumptions listed at the end.",
        "",
        "## Chosen geometry",
        "",
    ]
    lines += _table(
        ["Axle", "Variant", "Anti % (target / rig)", "dz/g, mm/g (target / rig)", "c rig / IC",
         "δ upper, lower (deg)", "Max pickup shift (mm)"],
        [
            [
                "Front (anti-dive, braking)",
                chosen_front["variant_id"],
                f"{_f(chosen_front['target_anti_dive_pct'], 1)} / {_f(chosen['achieved_anti_dive_pct'], 1)}",
                f"{_f(chosen_front['target_dz_front_mm_per_g'])} / {_f(chosen['dz_front_braking_mm_per_g'])}",
                f"{_f(chosen['c_rig_front'], 4)} / {_f(chosen['c_ic_front'], 4)}",
                f"{_f(variants[chosen_front['variant_id']]['delta_upper_deg'])}, "
                f"{_f(variants[chosen_front['variant_id']]['delta_lower_deg'])}",
                _f(chosen_front["max_pickup_shift_mm"], 1),
            ],
            [
                "Rear (anti-squat, RWD)",
                chosen_rear["variant_id"],
                f"{_f(chosen_rear['target_anti_squat_pct'], 1)} / {_f(chosen['achieved_anti_squat_pct'], 1)}",
                f"{_f(chosen_rear['target_dz_rear_mm_per_g'])} / {_f(chosen['dz_rear_accel_mm_per_g'])}",
                f"{_f(chosen['c_rig_rear'], 4)} / {_f(chosen['c_ic_rear'], 4)}",
                f"{_f(variants[chosen_rear['variant_id']]['delta_upper_deg'])}, "
                f"{_f(variants[chosen_rear['variant_id']]['delta_lower_deg'])}",
                _f(chosen_rear["max_pickup_shift_mm"], 1),
            ],
        ],
    )
    lines += [
        "",
        f"Braking anti-lift at the chosen rear is {_f(chosen_front.get('achieved_anti_lift_pct'), 1)}%. "
        f"The side-view swing-arm length is held at the baseline value: front "
        f"{_f(study.geom0['front'].d, 1)} m, rear {_f(study.geom0['rear'].d, 1)} m.",
        "",
        "### Hardpoint changes (left side; BobLib mirrors the right)",
        "",
    ]
    hp_rows = []
    for axle in AXLES:
        for block, key in MOVED_PICKUPS:
            old = vec(study.vehicle[axle][block][key])
            new = vec(chosen_vehicle[axle][block][key])
            hp_rows.append(
                [axle, key, ", ".join(_f(1000 * v, 1) for v in old), ", ".join(_f(1000 * v, 1) for v in new),
                 ", ".join(_f(1000 * v, 1) for v in new - old)]
            )
    lines += _table(["Axle", "Pickup", "Baseline (mm)", "Chosen (mm)", "Change x, y, z (mm)"], hp_rows)
    lines += ["", "## Objective", "",
              "Bin-weighted expected longitudinal capacity E[Σ μx Fz] over the tires carrying Fx (N).", ""]
    lines += _table(
        ["Maneuver", "Baseline", "Chosen (rig)", "Gain", "Chosen, stage prediction"],
        [
            ["RWD acceleration", _f(base.objectives["rwd_acceleration"]), _f(chosen["objective_accel_n"]),
             _f(chosen["delta_objective_accel_n"]), _f(chosen["objective_accel_n_predicted"])],
            ["Braking", _f(base.objectives["braking"]), _f(chosen["objective_braking_n"]),
             _f(chosen["delta_objective_braking_n"]), _f(chosen["objective_braking_n_predicted"])],
        ],
    )
    lines += ["", "## Stage 1: anti-squat sweep", ""]
    lines += _table(
        ["Variant", "Target AS %", "Rig AS %", "dz_r/g (mm/g)", "Δ objective (N)", "Feasible", "Chosen"],
        [
            [r["variant_id"], _f(r["target_anti_squat_pct"], 1), _f(r.get("achieved_anti_squat_pct"), 1),
             _f(r.get("achieved_dz_rear_mm_per_g")), _f(r.get("delta_objective_n")), r["feasible"], r["chosen"]]
            for r in rear_rows
        ],
    )
    sens_keys = sorted({k for r in front_rows for k in r if k.startswith("delta_objective_rear_") and r[k]})
    lines += ["", "## Stage 2: anti-dive sweep (at the chosen rear)", ""]
    lines += _table(
        ["Variant", "Target AD %", "Rig AD %", "dz_f/g (mm/g)", "Δ objective (N)"]
        + [f"Δ objective, rear {k.split('_')[3]} step (N)" for k in sens_keys]
        + ["Feasible", "Chosen"],
        [
            [r["variant_id"], _f(r["target_anti_dive_pct"], 1), _f(r.get("achieved_anti_dive_pct"), 1),
             _f(r.get("achieved_dz_front_mm_per_g")), _f(r.get("delta_objective_n"))]
            + [_f(r.get(k)) for k in sens_keys]
            + [r["feasible"], r["chosen"]]
            for r in front_rows
        ],
    )
    lines += ["", "## Validation", "",
              "Rig runs of the chosen car and the sweep corners. Predicted values combine the stage-1 rear "
              "and stage-2 front rig results; achieved values come from the car's own rig run.", ""]
    lines += _table(
        ["Car", "c front pred / rig / IC", "c rear pred / rig / IC", "Braking obj. pred / rig (N)",
         "Accel obj. pred / rig (N)", "Feasible"],
        [
            [v["car"],
             f"{_f(v['c_rig_front_predicted'], 4)} / {_f(v['c_rig_front'], 4)} / {_f(v['c_ic_front'], 4)}",
             f"{_f(v['c_rig_rear_predicted'], 4)} / {_f(v['c_rig_rear'], 4)} / {_f(v['c_ic_rear'], 4)}",
             f"{_f(v['objective_braking_n_predicted'])} / {_f(v['objective_braking_n'])}",
             f"{_f(v['objective_accel_n_predicted'])} / {_f(v['objective_accel_n'])}",
             v["feasible"]]
            for v in validation
        ],
    )
    lines += ["", "### Constraint margins, chosen car (positive = inside the limit)", ""]
    lines += _table(
        ["Axle", "Pickup shift (mm)", "Toe gain (deg/m)", "Caster change (deg)", "Anti range (%)"],
        [
            [axle, _f(chosen[f"pickup_shift_margin_mm_{axle}"]), _f(chosen[f"toe_gain_margin_deg_per_m_{axle}"]),
             _f(chosen[f"caster_margin_deg_{axle}"]), _f(chosen[f"anti_range_margin_pct_{axle}"])]
            for axle in AXLES
        ],
    )
    lines += ["", "### Springs", ""]
    lines += _table(
        ["Axle", "Ride frequency (Hz)", "Wheel rate (N/m)", "Motion ratio", "Spring rate (N/m)",
         "Vehicle YAML spring (N/m)"],
        [
            [axle, _f(m.rate.f_hz), _f(m.rate.k_w, 0), _f(m.rate.mr, 4), _f(m.rate.k_s, 0),
             _f(spring_rate_from_vehicle(study.vehicle, axle), 0)]
            for axle, m in chosen_measures.items()
        ],
    )
    freq = a.get("ride_frequency_hz")
    freq_text = (
        f"{_f(freq['front'])} Hz front and {_f(freq['rear'])} Hz rear, as wheel rate in series with the tire. "
        "It replaces the vehicle's placeholder springs; `outputs/chosen_vehicle.yml` carries the implied rates."
        if isinstance(freq, dict)
        else "not held; the vehicle YAML springs are used."
    )
    lines += [
        "",
        "## Provisional assumptions",
        "",
        "- **Aero map.** `AEROMAP_VALIDATION.md` says not to select anti geometry from this map's balance. "
        "The calibrated CoP is a provisional prior, so these results depend on it until Aero confirms the map.",
        f"- **Static ride height.** {_f(1000 * study.ctx.rh_static['front'], 2)} mm front, "
        f"{_f(1000 * study.ctx.rh_static['rear'], 2)} mm rear: the aero grid anchor, not an asserted static "
        "ride height (`AEROMAP_COP_CALIBRATION.md`).",
        "- **Rear vertical datum.** Unresolved (`vehicle.datum.json`); it affects h, IC heights and R/d.",
        "- **Scenario weights.** "
        + ", ".join(f"{m}: {a['scenarios'][m].get('weights', 'uniform')}" for m in MANEUVERS)
        + ". Nothing in the repo gives lap-time fractions.",
        f"- **σFx.** {_f(100 * study.ctx.sigma_fx_fraction, 0)}% of the mean Fx per wheel, a placeholder "
        "for the longitudinal force variation. σFz = |c_eff| σFx stands in for the rigid-link load path only.",
        f"- **Regen share of rear braking.** {_f(study.ctx.regen_share)}; the vehicle YAML doesn't define it.",
        f"- **Ride frequency.** {freq_text}",
        "- **Constraint limits.** Placeholders from study.yml. Kickback isn't checked; the rig applies no steer.",
        "- **Quasi-static model.** No damper or frequency content.",
        "- **Loaded radius.** `wheel.radius_m`, ignoring static tire deflection (about 6 mm).",
        "- **Drag.** a_x is the total longitudinal acceleration; drag's line of action is ignored in ΔW.",
        "",
        "## Outputs",
        "",
    ]
    output_names = [
        "run_provenance.csv", "variants.csv", "fourpost_metrics.csv", "bin_scores.csv",
        "anti_squat_summary.csv", "anti_dive_summary.csv", "validation.csv", "chosen_vehicle.yml",
    ]
    plot_names = [
        "anti_squat_objective.png", "anti_dive_objective.png",
        "validation_anti_vs_heave.png", "ride_height_vs_speed.png",
    ]
    for report in study.paths.reports:
        rel = lambda p: os.path.relpath(p, report.parent)  # noqa: E731
        body = lines + [f"- [`outputs/{name}`]({rel(study.paths.outputs / name)})" for name in output_names]
        body += [""] + [f"![{name}]({rel(study.paths.plots / name)})" for name in plot_names]
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text("\n".join(body) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)
    study_yml = load_study(args.study)
    paths = resolve_paths(args.study, args.vehicle, study_yml)
    if shutil.which("omc") is None:
        raise SystemExit("OpenModelica `omc` is not on PATH. Run the study in the BobSim Docker image.")
    if not BOBLIB_PACKAGE.exists():
        raise SystemExit(f"Missing {as_repo_path(BOBLIB_PACKAGE)}. Run `make init` from the repo root.")
    stages = STAGES if args.stage == "all" else (args.stage,)
    if args.stage != "all":
        check_stage_dependencies(args.stage, paths.outputs, paths.study_dir)

    study = Study(paths, study_yml, args)
    paths.outputs.mkdir(parents=True, exist_ok=True)
    study.write_provenance()
    base = study.run_baseline()
    runners = {
        "anti-squat": study.run_anti_squat,
        "anti-dive": study.run_anti_dive,
        "validation": study.run_validation,
    }
    for stage in stages:
        runners[stage](base)
    if "validation" in stages:
        for report in paths.reports:
            print(f"Wrote {as_repo_path(report)}")


if __name__ == "__main__":
    main()
