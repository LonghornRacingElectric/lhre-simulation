"""Audit the Orion CFD ride-height map carried in the DS-010 vehicle YAML.

Run with: python check_orion_aeromap.py
The CSV contains only the 21 cases accepted by the CFD master report.
"""

from __future__ import annotations

import csv
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "orion_2026_aeromap_converged.csv"
VEHICLE = HERE / "vehicle_wip_2027_frontv19_rearv35.yml"


def main() -> None:
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding="utf-8"))
    aero = vehicle["aero"]
    with SOURCE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    front = aero["front_ride_height_grid_m"]
    rear = aero["rear_ride_height_grid_m"]
    assert all(a < b for a, b in zip(front, front[1:]))
    assert all(a < b for a, b in zip(rear, rear[1:]))
    for key in ("drag_table_n", "downforce_table_n", "my_table_nm"):
        assert len(aero[key]) == len(front)
        assert all(len(row) == len(rear) for row in aero[key])

    source_points: set[tuple[int, int]] = set()
    largest_residual = 0.0
    for row in rows:
        f_in = float(row["front_ride_height_in"])
        r_in = float(row["rear_ride_height_in"])
        i = min(range(len(front)), key=lambda n: abs(front[n] - f_in * 0.0254))
        j = min(range(len(rear)), key=lambda n: abs(rear[n] - r_in * 0.0254))
        assert abs(front[i] - f_in * 0.0254) < 1e-9
        assert abs(rear[j] - r_in * 0.0254) < 1e-9
        source_points.add((i, j))
        # Existing BobSim map assumes CFD reports one side of a symmetric car,
        # then translates reported My from x=0 to aero_ref x=1 m.
        fz = float(row["fz_n"])
        expected = (
            ("drag_table_n", 2 * float(row["fx_n"])),
            ("downforce_table_n", 2 * fz),
            ("my_table_nm", 2 * (float(row["my_nm"]) - fz)),
        )
        for key, value in expected:
            residual = abs(aero[key][i][j] - value)
            largest_residual = max(largest_residual, residual)
            assert residual < 0.00011, (f_in, r_in, key, residual)

    front_x = vehicle["front"]["suspension"]["wheel_center_m"][0]
    rear_x = vehicle["rear"]["suspension"]["wheel_center_m"][0]
    wheelbase = front_x - rear_x
    assert wheelbase > 0
    aero_x = aero["aero_ref_m"][0]
    assert abs(aero_x - 1.0) < 1e-12

    outside = []
    missing = []
    direct_fractions = []
    for i, f in enumerate(front):
        for j, r in enumerate(rear):
            downforce = aero["downforce_table_n"][i][j]
            drag = aero["drag_table_n"][i][j]
            my = aero["my_table_nm"][i][j]
            assert downforce > 0 and drag > 0
            # Body x points forward and z upward. Moment about front axle:
            my_front = my + (aero_x - front_x) * downforce
            cop_x = front_x + my_front / downforce
            front_fraction = (cop_x - rear_x) / wheelbase
            if not 0 <= front_fraction <= 1:
                outside.append((f / 0.0254, r / 0.0254, front_fraction))
            if (i, j) not in source_points:
                missing.append((f / 0.0254, r / 0.0254))
            else:
                direct_fractions.append(front_fraction)

    print(f"CFD source cases matched: {len(source_points)}/21")
    print(f"Maximum force/moment residual: {largest_residual:.6g} N or N m")
    print(f"Grid cells without a converged CFD source: {len(missing)}/25: "
          f"{[(round(f, 2), round(r, 2)) for f, r in missing]}")
    print(f"Front aero fraction outside [0,1]: {len(outside)}/25")
    print(f"Direct CFD front aero fraction range: "
          f"{min(direct_fractions):.3f} to {max(direct_fractions):.3f}")
    print(f"Entire stored grid front aero fraction range: "
          f"{min(x[2] for x in outside):.3f} to {max(x[2] for x in outside):.3f}")
    print("Ride-height reference Z minus wheel-center Z (mm): "
          f"front={1000 * (aero['front_left_ride_height_ref_m'][2] - vehicle['front']['suspension']['wheel_center_m'][2]):.3f}, "
          f"rear={1000 * (aero['rear_left_ride_height_ref_m'][2] - vehicle['rear']['suspension']['wheel_center_m'][2]):.3f}")


if __name__ == "__main__":
    main()
