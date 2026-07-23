"""Run the contained OpenLAP-equation solver for the matched event suite.

SPDX-License-Identifier: GPL-3.0-or-later
"""

from __future__ import annotations

import json
from pathlib import Path

import pandas as pd

from openlap_solver import load_vehicle, solve_track


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    event_dir = root / "inputs" / "events"
    output_dir = root / "outputs" / "events"
    output_dir.mkdir(parents=True, exist_ok=True)

    manifest = json.loads(
        (event_dir / "openlap_event_suite_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    vehicle = load_vehicle(root / manifest["Vehicle"])
    summaries = []
    for event in manifest["Events"]:
        slug = event["Slug"]
        track = pd.read_csv(root / event["OpenLAPTrackCsv"])
        result, summary = solve_track(
            vehicle,
            track,
            is_closed=bool(event["IsClosed"]),
        )
        summary.update(
            {
                "event_slug": slug,
                "event_name": event["Name"],
                "track_configuration": event["Configuration"],
            }
        )
        result.to_csv(output_dir / f"{slug}_openlap_trace.csv", index=False)
        (output_dir / f"{slug}_openlap_summary.json").write_text(
            json.dumps(summary, indent=2),
            encoding="utf-8",
        )
        summaries.append(summary)

    suite_summary = {
        "solver": "OpenLAP equations, contained Python forward/backward port",
        "events": summaries,
    }
    (output_dir / "openlap_event_suite_summary.json").write_text(
        json.dumps(suite_summary, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(suite_summary, indent=2))


if __name__ == "__main__":
    main()
