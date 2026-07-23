"""Convert the prior OptimumLap event meshes to OpenLAP inputs.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.io import savemat


def deterministic_savemat(path: Path, payload: dict) -> None:
    """Write a MATLAB v5 file with a stable descriptive header."""

    savemat(path, payload, do_compression=True)
    header = (
        b"MATLAB 5.0 MAT-file, Platform: OpenLAP-compatible, "
        b"Created by LHRe Lapsims"
    )
    with path.open("r+b") as stream:
        stream.write(header.ljust(116, b" "))


def converted_track(segments: pd.DataFrame) -> pd.DataFrame:
    radius = segments["Radius_m"].to_numpy(dtype=float)
    curvature = np.divide(
        1.0,
        radius,
        out=np.zeros_like(radius),
        where=np.abs(radius) > 1e-15,
    )
    return pd.DataFrame(
        {
            "distance_m": segments["TotalLength_m"].to_numpy(dtype=float),
            "dx_m": segments["Length_m"].to_numpy(dtype=float),
            "curvature_1pm": curvature,
            "bank_deg": np.zeros(len(segments)),
            "inclination_deg": -segments["Grade"].to_numpy(dtype=float),
            "grip_factor": np.ones(len(segments)),
            "sector": segments["Sector"].to_numpy(dtype=int),
            "x_m": segments["X_m"].to_numpy(dtype=float),
            "y_m": segments["Y_m"].to_numpy(dtype=float),
            "z_m": segments["Z_m"].to_numpy(dtype=float),
        }
    )


def save_native_openlap_track(
    destination: Path,
    event: dict,
    track: pd.DataFrame,
) -> None:
    is_closed = bool(event["IsClosed"])
    if is_closed:
        x = track["distance_m"].to_numpy(dtype=float)
        dx = track["dx_m"].to_numpy(dtype=float)
        r = track["curvature_1pm"].to_numpy(dtype=float)
        bank = track["bank_deg"].to_numpy(dtype=float)
        incl = track["inclination_deg"].to_numpy(dtype=float)
        grip = track["grip_factor"].to_numpy(dtype=float)
        sector = track["sector"].to_numpy(dtype=int)
        map_x = track["x_m"].to_numpy(dtype=float)
        map_y = track["y_m"].to_numpy(dtype=float)
        map_z = track["z_m"].to_numpy(dtype=float)
    else:
        # Native OpenTRACK files include a standing-start point at x=0.
        x = np.insert(track["distance_m"].to_numpy(dtype=float), 0, 0.0)
        segment_dx = track["dx_m"].to_numpy(dtype=float)
        dx = np.append(np.diff(x), segment_dx[-1])
        r = np.insert(track["curvature_1pm"].to_numpy(dtype=float), 0, track["curvature_1pm"].iloc[0])
        bank = np.insert(track["bank_deg"].to_numpy(dtype=float), 0, track["bank_deg"].iloc[0])
        incl = np.insert(track["inclination_deg"].to_numpy(dtype=float), 0, track["inclination_deg"].iloc[0])
        grip = np.insert(track["grip_factor"].to_numpy(dtype=float), 0, track["grip_factor"].iloc[0])
        sector = np.insert(track["sector"].to_numpy(dtype=int), 0, track["sector"].iloc[0])
        map_x = np.insert(track["x_m"].to_numpy(dtype=float), 0, 0.0)
        map_y = np.insert(track["y_m"].to_numpy(dtype=float), 0, 0.0)
        map_z = np.insert(track["z_m"].to_numpy(dtype=float), 0, track["z_m"].iloc[0])

    info = {
        "name": event["Name"],
        "country": "United States",
        "city": "",
        "type": "FSAE",
        "config": "Closed" if is_closed else "Open",
        "direction": "Forward",
        "mirror": "Off",
    }
    deterministic_savemat(
        destination,
        {
            "info": info,
            "x": x[:, None],
            "dx": dx[:, None],
            "n": np.array([[len(x)]], dtype=np.uint32),
            "r": r[:, None],
            "bank": bank[:, None],
            "incl": incl[:, None],
            "factor_grip": grip[:, None],
            "sector": sector[:, None],
            "r_apex": np.empty((0, 1)),
            "apex": np.empty((0, 1)),
            "X": map_x[:, None],
            "Y": map_y[:, None],
            "Z": map_z[:, None],
            "arrow": np.empty((0, 1)),
        },
    )


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    event_dir = root / "inputs" / "events"
    manifest = json.loads(
        (event_dir / "event_suite_manifest.json").read_text(encoding="utf-8-sig")
    )
    converted_events = []
    for event in manifest["Events"]:
        slug = event["Slug"]
        segments = pd.read_csv(event_dir / f"{slug}_optimumlap_segments.csv")
        track = converted_track(segments)
        track_path = event_dir / f"{slug}_openlap_track.csv"
        mat_path = event_dir / f"OpenTRACK_{slug}.mat"
        track.to_csv(track_path, index=False)
        save_native_openlap_track(mat_path, event, track)

        length = float(track["dx_m"].sum())
        if not math.isclose(
            length,
            float(event["TrackLength_m"]),
            rel_tol=0.0,
            abs_tol=1e-8,
        ):
            raise ValueError(f"{slug}: converted track length mismatch")
        converted_events.append(
            {
                **event,
                "OpenLAPTrackCsv": str(track_path.relative_to(root)),
                "OpenLAPTrackMat": str(mat_path.relative_to(root)),
                "MaximumAbsCurvature_1pm": float(
                    track["curvature_1pm"].abs().max()
                ),
            }
        )

    converted_manifest = {
        "Vehicle": "inputs/openlap_vehicle.json",
        "NativeOpenLAPVehicle": "inputs/OpenVEHICLE_LHRe_Matched_Baseline.mat",
        "Events": converted_events,
    }
    (event_dir / "openlap_event_suite_manifest.json").write_text(
        json.dumps(converted_manifest, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(converted_manifest, indent=2))


if __name__ == "__main__":
    main()
