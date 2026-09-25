"""Fail if the active DS-010 vehicle diverges from the retained SHARK exports.

Usage: python verify_shark_vehicle.py FRONT_V19.shk REAR_V35.shk
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import yaml


STUDY = Path(__file__).resolve().parent
REPO = STUDY.parents[1]
VEHICLE = "studies/DS-010-anti-geometry/vehicle_wip_2027_frontv19_rearv35.yml"
SHA256 = {
    "front": "361e76f3bf94b01e5c8e9609d0e1790d47b25539fa754923b0f6a0e7e467ae02",
    "rear": "507d4b5b0c7a6006d02b08a08de180b65ae1595ae055af19bb255ef628ed92b0",
}
POINTS = {
    "suspension.lower_fore_i_m": 1,
    "suspension.lower_aft_i_m": 2,
    "suspension.lower_o_m": 3,
    "suspension.upper_fore_i_m": 4,
    "suspension.upper_aft_i_m": 5,
    "suspension.upper_o_m": 6,
    "actuation.rod_mount_m": 7,
    "actuation.bellcrank.pickups_m.rod": 8,
    "suspension.tie_o_m": 9,
    "steering.rack_pickup_m": 10,
    "actuation.shock.mount_m": 11,
    "actuation.bellcrank.pickups_m.shock": 12,
    "suspension.wheel_center_m": 14,
    "actuation.bellcrank.pivot_m": 15,
}
TOLERANCE_MM = 0.001


def shark_points(path: Path) -> list[list[float]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    marker = next(i for i, line in enumerate(lines) if line.strip() == "FRONT SUSPENSION")
    points = [[float(value) / 1000 for value in line.split()]
              for line in lines[marker + 2:marker + 18]]
    if len(points) != 16 or any(len(point) != 3 for point in points):
        raise ValueError(f"Expected 16 three-coordinate SHARK points in {path}")
    return points


def nested(data: dict, dotted: str) -> list[float]:
    value = data
    for key in dotted.split("."):
        value = value[key]
    return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("front_v19", type=Path)
    parser.add_argument("rear_v35", type=Path)
    args = parser.parse_args()
    manifest = yaml.safe_load((STUDY / "study.yml").read_text(encoding="utf-8"))
    if manifest["vehicle"] != VEHICLE:
        raise SystemExit(f"FAIL: DS-010 selects {manifest['vehicle']}, expected {VEHICLE}")
    vehicle = yaml.safe_load((REPO / VEHICLE).read_text(encoding="utf-8"))
    failures = []
    for axle, source in (("front", args.front_v19), ("rear", args.rear_v35)):
        actual_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual_hash != SHA256[axle]:
            failures.append(f"{axle}: SHARK source hash changed: {actual_hash}")
        points = shark_points(source)
        if vehicle["architecture"][axle] != "bellcrank_stabar":
            failures.append(f"{axle}: wrong architecture {vehicle['architecture'][axle]}")
        for field, number in POINTS.items():
            error_mm = 1000 * math.dist(nested(vehicle[axle], field), points[number - 1])
            if error_mm > TOLERANCE_MM:
                failures.append(f"{axle}.{field}: {error_mm:.3f} mm from SHARK")
    if failures:
        raise SystemExit("FAIL: DS-010 SHARK gate\n" + "\n".join(failures))
    print("PASS: active DS-010 vehicle selects V19 front/V35 rear; "
          "28/28 represented points within 0.001 mm; both source hashes match")
    print("LIMIT: stabilizer-bar pickups/rates are Orion carryovers; "
          "rear vertical datum remains unresolved")


if __name__ == "__main__":
    main()
