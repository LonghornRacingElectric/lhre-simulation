import numpy as np
import pytest

from _0_Utils.dyn_py.parametric_projection import with_longitudinal_anti
from _0_Utils.vehicle_io import load_yaml
from oval import oval_centerline, oval_layout
from run import TRACKS, cases, corridor


def test_grid_contains_upstream_cases_once():
    names = [case[0] for case in cases()]
    assert len(names) == len(set(names)) == 20
    assert names[0] == "baseline"
    assert names[-3:] == ["rc00", "rc25", "rc50"]


@pytest.mark.parametrize("track,expected_m", [("hairpin", 196.5487), ("template", 195.9771), ("sweeper", 208.1770)])
def test_source_oval_layout_and_centerline(track, expected_m):
    layout = oval_layout(**TRACKS[track])
    center = oval_centerline(**TRACKS[track], spacing_m=0.5)
    line = corridor(center).line_from_offsets(np.zeros(len(center)))
    assert layout["lap_m"] == pytest.approx(expected_m, abs=0.0001)
    assert line.track_length_m == pytest.approx(expected_m, rel=1e-4)


def test_anti_grid_uses_parametric_2027_force_path():
    data = load_yaml("../studies/parameterized-bobsim/outputs/targets_2027/vehicle.yml")
    changed = with_longitudinal_anti(data, anti_dive=0.4, anti_squat=0.6)
    ratio = data["vehicle"]["cg_height_m"] / data["vehicle"]["wheelbase_m"]
    assert changed["front"]["longitudinal_jacking_coefficient"] == pytest.approx(-0.4 * ratio / 0.84)
    assert changed["rear"]["longitudinal_jacking_coefficient"] == pytest.approx(0.6 * ratio)
    assert data["front"]["longitudinal_jacking_coefficient"] == 0
