"""Tests for the DS-010 run.py pure functions. No OpenModelica needed.

    docker run --rm -v "$PWD":/workspace -w /workspace bobdyn/bobsim:c45940e \
        python -m pytest -p no:cacheprovider studies/DS-010-anti-geometry/test_run.py
"""

from __future__ import annotations

import copy
import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

STUDY_DIR = Path(__file__).resolve().parent
REPO_ROOT = STUDY_DIR.parents[1]
DESIGN_VEHICLE = REPO_ROOT / "vehicles" / "design" / "vehicle.yml"

_spec = importlib.util.spec_from_file_location("ds010_run", STUDY_DIR / "run.py")
run = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = run
_spec.loader.exec_module(run)

# Formulation note §6: motion ratios and the vehicle's springs.
MR = {"front": 1.0944, "rear": 1.7222}


@pytest.fixture(scope="module")
def vehicle():
    return run.read_vehicle(DESIGN_VEHICLE)


@pytest.fixture(scope="module")
def mp(vehicle):
    return run.mass_rollup(vehicle)


def rates(vehicle, mp, freq):
    return {axle: run.axle_rate(vehicle, mp, axle, freq[axle], MR[axle]) for axle in run.AXLES}


# -- Task 1: study files and CLI ---------------------------------------------


def test_default_study_dir_is_script_dir():
    args = run.parse_args([])
    assert Path(args.study).resolve() == STUDY_DIR
    assert args.stage == "all" and not args.reuse


def test_vehicle_from_study_resolves_against_repo_root():
    study = run.load_study(STUDY_DIR)
    paths = run.resolve_paths(STUDY_DIR, None, study)
    assert paths.vehicle == DESIGN_VEHICLE
    assert paths.study_yml == STUDY_DIR / "study.yml"


def test_vehicle_argument_overrides_relative_to_cwd(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    paths = run.resolve_paths(STUDY_DIR, "other.yml", {"vehicle": "vehicles/design/vehicle.yml"})
    assert paths.vehicle == (tmp_path / "other.yml").resolve()


def test_report_paths_come_from_outputs():
    study = run.load_study(STUDY_DIR)
    paths = run.resolve_paths(STUDY_DIR, None, study)
    assert paths.reports == (
        STUDY_DIR / "RESULTS.md",
        REPO_ROOT / "reports" / "DS-010-anti-geometry.md",
    )


def test_missing_stage_dependency_message(tmp_path):
    with pytest.raises(SystemExit) as excinfo:
        run.check_stage_dependencies("anti-dive", tmp_path, STUDY_DIR)
    message = str(excinfo.value)
    assert "anti_squat_summary.csv" in message
    assert "python studies/DS-010-anti-geometry/run.py --stage anti-squat" in message

    (tmp_path / "anti_squat_summary.csv").write_text("variant_id\n", encoding="utf-8")
    with pytest.raises(SystemExit) as excinfo:
        run.check_stage_dependencies("validation", tmp_path, STUDY_DIR)
    assert "--stage anti-dive" in str(excinfo.value)
    run.check_stage_dependencies("anti-squat", tmp_path, STUDY_DIR)


# -- Task 2: mass, rates, anti and ride-height response ------------------------


def test_mass_rollup_matches_section6(mp):
    assert mp.m == pytest.approx(261.07, abs=0.01)
    assert mp.h == pytest.approx(0.2796, abs=1e-4)
    assert mp.a == pytest.approx(0.8003, abs=1e-4)
    assert mp.l == pytest.approx(1.5494, abs=1e-4)
    assert run.delta_w(mp, 1.0) == pytest.approx(462.2, abs=0.3)
    W_f, W_r = run.static_axle_loads(mp)
    assert W_f + W_r == pytest.approx(mp.m * run.G)


def test_null_frequency_reproduces_section6(vehicle, mp):
    r = rates(vehicle, mp, {"front": None, "rear": None})
    assert r["front"].k_w == pytest.approx(21934, abs=2)
    assert r["rear"].k_w == pytest.approx(14761, abs=2)
    assert r["front"].k_s == pytest.approx(run.spring_rate_from_vehicle(vehicle, "front"))
    assert r["front"].f_hz == pytest.approx(2.90, abs=0.005)
    assert r["rear"].f_hz == pytest.approx(2.34, abs=0.005)

    dW = run.delta_w(mp, 1.0)
    args = (r["front"].K, r["front"].Kt, r["rear"].K, r["rear"].Kt)
    dz_f, dz_r = run.dz_braking(dW, 0.0, 0.0, *args)
    assert 1000 * dz_f == pytest.approx(-12.9, abs=0.05)
    assert 1000 * dz_r == pytest.approx(18.0, abs=0.05)
    dz_f, dz_r = run.dz_braking(dW, 0.3, 0.3, *args)
    assert 1000 * dz_f == pytest.approx(-9.7, abs=0.05)
    assert 1000 * dz_r == pytest.approx(13.3, abs=0.05)


def test_three_hz_hold(vehicle, mp):
    assert run.sprung_corner_mass(mp, "front") == pytest.approx(54.02, abs=0.01)
    assert run.sprung_corner_mass(mp, "rear") == pytest.approx(59.19, abs=0.01)
    r = rates(vehicle, mp, {"front": 3.0, "rear": 3.0})
    assert r["front"].k_w == pytest.approx(23811, rel=5e-4)
    assert r["rear"].k_w == pytest.approx(26706, rel=5e-4)
    assert r["front"].k_s == pytest.approx(r["front"].k_w * MR["front"] ** 2)
    for axle in run.AXLES:
        assert r[axle].f_hz == pytest.approx(3.0)

    dW = run.delta_w(mp, 1.0)
    K_f, Kt_f, K_r, Kt_r = r["front"].K, r["front"].Kt, r["rear"].K, r["rear"].Kt
    assert 1000 * run.dz_accel(dW, 0.0, K_f, Kt_f, K_r, Kt_r)[1] == pytest.approx(-11.0, abs=0.05)
    assert 1000 * run.dz_braking(dW, 0.0, 0.0, K_f, Kt_f, K_r, Kt_r)[0] == pytest.approx(-12.0, abs=0.05)
    # Sweep endpoints, -20% and 90% anti.
    rear = [1000 * run.dz_accel(dW, a, K_f, Kt_f, K_r, Kt_r)[1] for a in (-0.2, 0.9)]
    front = [1000 * run.dz_braking(dW, a, 0.0, K_f, Kt_f, K_r, Kt_r)[0] for a in (-0.2, 0.9)]
    assert rear == pytest.approx([-12.7, -3.2], abs=0.05)
    assert front == pytest.approx([-14.0, -3.3], abs=0.05)


def test_frequency_above_tire_rate_raises():
    with pytest.raises(ValueError, match="tire rate"):
        run.wheel_rate_for_frequency(m_c=55.0, f_hz=7.0, k_t=98947.0)


def test_ride_frequency_inverts_wheel_rate():
    k_w = run.wheel_rate_for_frequency(55.0, 2.7, 98947.0)
    assert run.ride_frequency(55.0, k_w, 98947.0) == pytest.approx(2.7)


def test_dz_anti_round_trip(mp):
    dW, K, Kt = run.delta_w(mp, 1.0), 47622.0, 197894.0
    for anti in (-0.2, 0.0, 0.35, 0.9):
        dz_f, _ = run.dz_braking(dW, anti, 0.0, K, Kt, K, Kt)
        assert run.anti_from_dz(dz_f, dW, K, Kt) == pytest.approx(anti)
        _, dz_r = run.dz_accel(dW, anti, K, Kt, K, Kt)
        assert run.anti_from_dz(dz_r, dW, K, Kt) == pytest.approx(anti)


def test_anti_inverse_functions(mp):
    R, inv_d, beta = 0.2045, 1.0 / 3.0, 0.84
    c = run.c_for_anti_squat(0.4, inv_d, R, mp)
    assert run.anti_squat(c, inv_d, R, mp) == pytest.approx(0.4)
    c = run.c_for_anti_dive(0.4, beta, mp)
    assert run.anti_dive(c, beta, mp) == pytest.approx(0.4)
    # No regen: anti-lift uses the full rear friction share of c.
    assert run.anti_lift(c, inv_d, R, beta, 0.0, mp) == pytest.approx(mp.l / mp.h * (1 - beta) * c)
    s_fric, s_regen = run.rear_brake_shares(beta, 0.5)
    assert s_fric + s_regen == pytest.approx(1 - beta)


# -- Task 3: side-view IC and pivot rotation -----------------------------------


def arm_points(vehicle, axle, arm):
    susp = vehicle[axle]["suspension"]
    return susp[f"{arm}_fore_i_m"], susp[f"{arm}_aft_i_m"], susp[f"{arm}_o_m"]


def side_view_line_brute(fore, aft, outer, y0):
    """Two points of the wishbone plane at y = y0, found from the plane's parametrisation."""
    A, B, C = (np.asarray(p, dtype=float) for p in (fore, aft, outer))
    u, v = B - A, C - A
    points = []
    for s in (0.0, 1.0):
        t = (y0 - A[1] - s * u[1]) / v[1]
        p = A + s * u + t * v
        points.append((p[0], p[2]))
    return points


def intersect(line1, line2):
    (x1, z1), (x2, z2) = line1
    (x3, z3), (x4, z4) = line2
    M = np.array([[x2 - x1, -(x4 - x3)], [z2 - z1, -(z4 - z3)]])
    s, _ = np.linalg.solve(M, [x3 - x1, z3 - z1])
    return x1 + s * (x2 - x1), z1 + s * (z2 - z1)


SYNTHETIC = {
    "upper_fore_i_m": [0.15, 0.30, 0.19],
    "upper_aft_i_m": [-0.10, 0.31, 0.16],
    "upper_o_m": [0.01, 0.57, 0.27],
    "lower_fore_i_m": [0.16, 0.22, 0.06],
    "lower_aft_i_m": [-0.08, 0.22, 0.09],
    "lower_o_m": [0.00, 0.59, 0.12],
    "wheel_center_m": [0.0, 0.62, 0.20],
}


def test_side_view_ic_matches_brute_force():
    y0 = SYNTHETIC["wheel_center_m"][1]
    lines = [
        side_view_line_brute(
            SYNTHETIC[f"{arm}_fore_i_m"], SYNTHETIC[f"{arm}_aft_i_m"], SYNTHETIC[f"{arm}_o_m"], y0
        )
        for arm in run.ARMS
    ]
    expected = intersect(*lines)
    assert run.side_view_ic(SYNTHETIC, y0) == pytest.approx(expected, abs=1e-12)

    vehicle = {"front": {"suspension": SYNTHETIC, "wheel": {"radius_m": 0.2}}}
    geom = run.axle_geometry(vehicle, "front")
    d = -(expected[0] - 0.0)  # front: inward is -x
    assert geom.d == pytest.approx(d)
    assert geom.c == pytest.approx((expected[1] - geom.z_cp) / d)


def test_baseline_arms_are_parallel(vehicle):
    front = run.axle_geometry(vehicle, "front")
    rear = run.axle_geometry(vehicle, "rear")
    # Front IC ~1,160 km behind, rear ~270 m ahead: effectively parallel arms.
    for geom in (front, rear):
        assert abs(geom.inv_d) < 1e-2 and abs(geom.c) < 1e-3


def test_zero_rotation_is_identity(vehicle):
    fore, aft, _ = arm_points(vehicle, "rear", "upper")
    new_fore, new_aft = run.rotate_pivot(fore, aft, -1.5494, 0.0)
    assert new_fore == pytest.approx(fore, abs=1e-15)
    assert new_aft == pytest.approx(aft, abs=1e-15)


@pytest.mark.parametrize("axle", run.AXLES)
@pytest.mark.parametrize("arm", run.ARMS)
def test_solved_rotation_puts_line_through_target(vehicle, axle, arm):
    geom = run.axle_geometry(vehicle, axle)
    fore, aft, outer = arm_points(vehicle, axle, arm)
    for inv_d in (0.0, 0.4):  # parallel arms, and a 2.5 m swing arm
        q = run.target_point_h(geom, 0.08, inv_d)
        delta = run.solve_rotation(fore, aft, outer, geom.y_wc, geom.x_wc, q)
        assert delta is not None
        new_fore, new_aft = run.rotate_pivot(fore, aft, geom.x_wc, delta)
        line = run.side_view_line(new_fore, new_aft, outer, geom.y_wc)
        if inv_d:
            assert abs(run.line_residual(line, q / q[2])) < 1e-9
        else:
            assert abs(run.line_residual(line, q)) < 1e-12
        assert np.linalg.norm(new_aft - new_fore) == pytest.approx(
            np.linalg.norm(np.subtract(aft, fore)), abs=1e-12
        )
        assert (new_fore[1], new_aft[1]) == (fore[1], aft[1])


def test_solve_rotation_no_root_returns_none(vehicle):
    geom = run.axle_geometry(vehicle, "front")
    fore, aft, outer = arm_points(vehicle, "front", "upper")
    q = run.target_point_h(geom, 5.0, 0.0)  # a 79 deg arm slope
    assert run.solve_rotation(fore, aft, outer, geom.y_wc, geom.x_wc, q) is None
    new, info = run.make_hardpoint_variant(vehicle, "front", 5.0, 0.0)
    assert new is None and info["root_found"] is False


@pytest.mark.parametrize(("axle", "target_c"), [("front", 0.12), ("rear", -0.03), ("rear", 0.15)])
def test_hardpoint_variant_hits_target(vehicle, axle, target_c):
    geom0 = run.axle_geometry(vehicle, axle)
    new, info = run.make_hardpoint_variant(vehicle, axle, target_c, geom0.inv_d)
    assert new is not None and info["root_found"]
    geom = run.axle_geometry(new, axle)
    assert geom.c == pytest.approx(target_c, abs=1e-9)
    assert geom.inv_d == pytest.approx(geom0.inv_d, abs=1e-9)
    assert 0.0 < info["max_pickup_shift_mm"] < 100.0
    assert 0.0 < info["tie_pickup_shift_mm"] <= info["max_pickup_shift_mm"]

    old_tie, new_tie = vehicle[axle]["steering"]["rack_pickup_m"], new[axle]["steering"]["rack_pickup_m"]
    assert new_tie[:2] == pytest.approx(old_tie[:2], abs=1e-12)
    # MOVED_PICKUPS is exactly what changed: validation cars are assembled from it.
    restored = copy.deepcopy(new)
    for block, key in run.MOVED_PICKUPS:
        assert new[axle][block][key] != vehicle[axle][block][key], key
        restored[axle][block][key] = vehicle[axle][block][key]
    assert restored == vehicle
    assert vehicle == run.read_vehicle(DESIGN_VEHICLE)  # input not mutated


@pytest.mark.parametrize(("axle", "target_c"), [("front", 0.19), ("rear", 0.16), ("rear", -0.04)])
def test_hardpoint_variant_holds_bump_steer(vehicle, axle, target_c):
    geom0 = run.axle_geometry(vehicle, axle)
    new, _ = run.make_hardpoint_variant(vehicle, axle, target_c, geom0.inv_d)
    before = run.toe_rate(vehicle[axle]["suspension"], vehicle[axle]["steering"]["rack_pickup_m"])
    after = run.toe_rate(new[axle]["suspension"], new[axle]["steering"]["rack_pickup_m"])
    assert after == pytest.approx(before, abs=1e-9)


@pytest.mark.parametrize(
    ("axle", "percent", "rack_moved", "rig_deg_per_m"),
    [
        # FourPostEval smoke runs (left toe vs heave, linear fit over +/-30 mm).
        ("front", None, False, 0.129),
        ("rear", None, False, -0.087),
        ("front", 0.9, False, -12.095),
        ("rear", 0.9, False, -226.773),
    ],
)
def test_toe_rate_matches_fourpost(vehicle, axle, percent, rack_moved, rig_deg_per_m):
    """The static bump-steer model against the rig, including a variant whose rack was left in place."""
    variant = vehicle
    if percent is not None:
        mp = run.mass_rollup(vehicle)
        geom0 = run.axle_geometry(vehicle, axle)
        target = (
            run.c_for_anti_dive(percent, float(vehicle["brake"]["front_bias"]), mp)
            if axle == "front"
            else run.c_for_anti_squat(percent, geom0.inv_d, geom0.R, mp)
        )
        variant, _ = run.make_hardpoint_variant(vehicle, axle, target, geom0.inv_d)
        if not rack_moved:
            variant[axle]["steering"]["rack_pickup_m"] = vehicle[axle]["steering"]["rack_pickup_m"]
    model = math.degrees(run.toe_rate(variant[axle]["suspension"], variant[axle]["steering"]["rack_pickup_m"]))
    assert model == pytest.approx(rig_deg_per_m, abs=0.05, rel=0.02)


# -- Task 4: tire, aero, ride heights, scoring ---------------------------------


def test_read_tire_params(vehicle):
    tp = run.read_tire_params(run.tire_path(vehicle, "rear"))
    assert tp == run.TireParams(pdx1=2.597991, pdx2=-0.618826, fz0=650.0, lmux=1.0)


def test_expected_grip_matches_monte_carlo():
    tp = run.TireParams(pdx1=2.597991, pdx2=-0.618826, fz0=650.0, lmux=1.0)
    rng = np.random.default_rng(10)
    fz = rng.normal(800.0, 150.0, 1_000_000)
    sampled = float(np.mean(run.mu_x(fz, tp) * fz))
    assert run.expected_grip(800.0, 150.0, tp) == pytest.approx(sampled, rel=1e-3)
    assert run.expected_grip(800.0, 150.0, tp) < run.expected_grip(800.0, 0.0, tp)


def synthetic_aero(front_fraction=0.4):
    front = np.array([0.03, 0.05, 0.07])
    rear = np.array([0.04, 0.06, 0.08])
    df = 400.0 + 1000.0 * front[:, None] + 2000.0 * rear[None, :]
    x_front, x_rear, ref_x = 0.0, -1.5, 1.0
    my = df * (front_fraction * (x_front - x_rear) + x_rear - ref_x)
    return run.AeroMap(front, rear, df, my, ref_x, 15.0, x_front, x_rear)


def test_aero_map_bilinear_and_balance():
    aero = synthetic_aero(0.4)
    df, frac, clamped = aero.evaluate(0.045, 0.071, 15.0)
    assert df == pytest.approx(400.0 + 1000.0 * 0.045 + 2000.0 * 0.071)
    assert frac == pytest.approx(0.4)
    assert not clamped
    assert aero.evaluate(0.045, 0.071, 30.0)[0] == pytest.approx(4 * df)


def test_design_aero_map_reads(vehicle):
    aero = run.AeroMap.from_vehicle(vehicle)
    df, frac, clamped = aero.evaluate(0.07112, 0.08382, 15.0)
    raw = vehicle["aero"]
    assert df == pytest.approx(raw["downforce_table_n"][2][2])
    assert frac == pytest.approx(
        (raw["my_table_nm"][2][2] / df + 1.0 + 1.5494) / 1.5494
    )
    assert frac == pytest.approx(0.5, abs=1e-9)  # the CoP calibration target
    assert not clamped


def test_ride_height_fixed_point():
    aero = synthetic_aero(0.45)
    rh = run.solve_ride_heights((0.05, 0.06), (-0.004, 0.006), (40000.0, 45000.0), aero, 25.0)
    df, frac, _ = aero.evaluate(rh.rh_f, rh.rh_r, 25.0)
    assert rh.rh_f == pytest.approx(0.05 - 0.004 - df * frac / 40000.0, abs=1e-10)
    assert rh.rh_r == pytest.approx(0.06 + 0.006 - df * (1 - frac) / 45000.0, abs=1e-10)
    assert rh.f_aero_f + rh.f_aero_r == pytest.approx(df)


def score_context(vehicle, mp, rh_static=(0.07112, 0.08382), sigma=0.10):
    tp = run.read_tire_params(run.tire_path(vehicle, "front"))
    return run.ScoreContext(
        mp=mp,
        beta_f=0.84,
        regen_share=0.0,
        sigma_fx_fraction=sigma,
        tires={"front": tp, "rear": tp},
        aero=run.AeroMap.from_vehicle(vehicle),
        rh_static={"front": rh_static[0], "rear": rh_static[1]},
    )


STATE_F = run.AxleState(c=0.05, inv_d=0.0, R=0.2045, K=47622.0, Kt=197894.0)
STATE_R = run.AxleState(c=0.08, inv_d=0.0, R=0.2045, K=53412.0, Kt=197894.0)


def test_ride_height_clamped_and_flagged(vehicle, mp):
    ctx = score_context(vehicle, mp, rh_static=(0.01, 0.20))
    row = run.score_bin("braking", 25.0, 1.4, STATE_F, STATE_R, ctx)
    assert row["rh_clamped"] is True
    assert math.isfinite(row["grip_n"]) and row["grip_n"] > 0.0
    row = run.score_bin("braking", 15.0, 0.5, STATE_F, STATE_R, score_context(vehicle, mp))
    assert row["rh_clamped"] is False


def test_rwd_scores_rear_tires_only(vehicle, mp):
    ctx = score_context(vehicle, mp, sigma=0.0)
    row = run.score_bin("rwd_acceleration", 20.0, 0.6, STATE_F, STATE_R, ctx)
    tp = ctx.tires["rear"]
    assert row["grip_n"] == pytest.approx(2 * run.mu_x(row["fz_rear_n"], tp) * row["fz_rear_n"])
    assert row["sigma_fz_front_n"] == 0.0
    W_f, W_r = run.static_axle_loads(mp)
    total = 2 * (row["fz_front_n"] + row["fz_rear_n"])
    assert total == pytest.approx(W_f + W_r + row["downforce_n"])


def test_cplv_penalises_slope(vehicle, mp):
    ctx = score_context(vehicle, mp)
    flat = run.AxleState(c=0.0, inv_d=0.0, R=0.2045, K=STATE_R.K, Kt=STATE_R.Kt)
    steep = run.AxleState(c=0.3, inv_d=0.0, R=0.2045, K=STATE_R.K, Kt=STATE_R.Kt)
    # Same ride heights via a zero speed, so only the sigma term differs.
    a = run.score_bin("rwd_acceleration", 0.0, 0.9, STATE_F, flat, ctx)
    b = run.score_bin("rwd_acceleration", 0.0, 0.9, STATE_F, steep, ctx)
    assert b["sigma_fz_rear_n"] > a["sigma_fz_rear_n"] == 0.0
    assert b["grip_n"] < a["grip_n"]


def test_scenario_weights():
    bins = run.scenario_bins({"speeds_m_per_s": [10, 20], "ax_g": [0.5, 1.0, 1.4], "weights": "uniform"})
    assert len(bins) == 6 and sum(w for *_, w in bins) == pytest.approx(1.0)
    bins = run.scenario_bins({"speeds_m_per_s": [10, 20], "ax_g": [0.5], "weights": [[1], [3]]})
    assert [w for *_, w in bins] == pytest.approx([0.25, 0.75])
    with pytest.raises(ValueError, match="speeds x ax_g"):
        run.scenario_bins({"speeds_m_per_s": [10, 20], "ax_g": [0.5], "weights": [[1, 2]]})


# -- Task 5: rig results -------------------------------------------------------


def test_series_csv_round_trip(tmp_path):
    series = {"heave": np.array([-0.01, 0.0, 0.01]), "fr_anti_vs_heave": np.array([1.5, np.nan, 2.5])}
    path = tmp_path / "series.csv"
    run.write_series_csv(path, series)
    back = run.read_series_csv(path)
    assert back.keys() == series.keys()
    np.testing.assert_array_equal(back["heave"], series["heave"])
    np.testing.assert_array_equal(back["fr_anti_vs_heave"], series["fr_anti_vs_heave"])


def test_rig_c_interpolates_at_zero_heave(monkeypatch):
    series = {
        "rr_jacking_vs_heave_x": np.array([0.02, -0.02, 0.0]),
        "rr_anti_vs_heave": np.array([30.0, 10.0, 20.0]),
    }
    h, l = 0.28, 1.55
    assert run.rig_c(series, "rear", h, l) == pytest.approx(0.20 * h / l)
    monkeypatch.setitem(run.RIG_C_SIGN, "rear", -1.0)
    assert run.rig_c(series, "rear", h, l) == pytest.approx(-0.20 * h / l)
    x, c = run.rig_c_vs_heave(series, "rear", h, l)
    assert list(x) == [-0.02, 0.0, 0.02]


# -- Task 6: constraints, sign check, chosen vehicle ---------------------------

LIMITS = {
    "max_pickup_shift_mm": 30,
    "max_abs_toe_gain_deg_per_m": 4.0,
    "max_caster_change_deg": 2.0,
    "anti_pct_range": [-20, 100],
}


def measure(axle, c_rig=0.05, c_ic=0.05, toe=1.0, caster=0.5, anti=(10.0, 20.0), k_s=30000.0):
    geom = run.AxleGeometry(axle, c_ic, 0.0, 0.0, 0.6, 0.2, -0.0045, 0.2045)
    rate = run.AxleRate(k_w=25000.0, k_s=k_s, k_t=98947.0, mr=1.1, f_hz=3.0, held=True)
    return run.AxleMeasure(
        axle=axle,
        c_rig=c_rig,
        geom=geom,
        rate=rate,
        heave_m=np.array([-0.01, 0.01]),
        c_vs_heave=np.array([c_rig, c_rig]),
        anti_pct_vs_heave=np.array(anti),
        toe_gain_deg_per_m=toe,
        caster_range_deg=caster,
    )


def test_constraint_margins():
    margins, ok = run.constraint_margins("front", measure("front"), 12.0, True, LIMITS)
    assert ok
    assert margins == pytest.approx(
        {
            "pickup_shift_margin_mm": 18.0,
            "toe_gain_margin_deg_per_m": 3.0,
            "caster_margin_deg": 1.5,
            "anti_range_margin_pct": 30.0,
        }
    )
    # Caster is a front-only limit.
    assert run.constraint_margins("rear", measure("rear", caster=5.0), 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front", caster=5.0), 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front", toe=-4.5), 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front", anti=(-25.0, 5.0)), 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front"), 31.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front", toe=math.nan), 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", None, 12.0, True, LIMITS)[1]
    assert not run.constraint_margins("front", measure("front"), 12.0, False, LIMITS)[1]
    # The baseline moves no pickups.
    assert run.constraint_margins("front", measure("front"), math.nan, True, LIMITS)[1]


def test_ic_sign_check():
    assert run.ic_check(0.050, 0.055, 0.02) == (False, False)
    assert run.ic_check(0.050, 0.080, 0.02) == (True, False)
    assert run.ic_check(0.010, -0.005, 0.02) == (False, False)  # both near zero
    assert run.ic_check(-0.100, 0.100, 0.02) == (True, True)

    study = SimpleNamespace(ic_tol=0.02)
    flags = run.Study.check_ic(study, "anti-squat", "rear01", {"rear": measure("rear", 0.06, 0.09)})
    assert flags == {"rear": True}
    with pytest.raises(SystemExit, match="RIG_C_SIGN"):
        run.Study.check_ic(study, "anti-squat", "rear01", {"rear": measure("rear", -0.1, 0.1)})


def test_chosen_vehicle_springs(vehicle):
    measures = {"front": measure("front", k_s=31000.0), "rear": measure("rear", k_s=77000.0)}
    text = run.chosen_vehicle_yaml(vehicle, measures, {"front": 3.0, "rear": 3.0})
    data = yaml.safe_load(text)
    assert data["front"]["actuation"]["shock"]["spring_table"] == {"table": [[0.0, 0.0], [1.0, 31000.0]]}
    assert data["rear"]["actuation"]["shock"]["spring_table"] == {"table": [[0.0, 0.0], [1.0, 77000.0]]}
    lines = text.splitlines()
    comments = [i for i, line in enumerate(lines) if "ride frequency hold" in line]
    assert len(comments) == 2
    assert all(lines[i + 1].strip() == "spring_table:" for i in comments)
    assert "spring rate 77,000 N/m" in lines[comments[1]]

    unchanged = run.chosen_vehicle_yaml(vehicle, measures, {"front": None, "rear": None})
    assert yaml.safe_load(unchanged)["front"]["actuation"]["shock"] == vehicle["front"]["actuation"]["shock"]


def test_upsert_stage_rows(tmp_path):
    path = tmp_path / "rows.csv"
    run.upsert_stage_rows(path, "anti-dive", [{"stage": "anti-dive", "v": 1}])
    run.upsert_stage_rows(path, "baseline", [{"stage": "baseline", "v": 0}])
    run.upsert_stage_rows(path, "anti-dive", [{"stage": "anti-dive", "v": 2}, {"stage": "anti-dive", "v": 3}])
    assert [(r["stage"], r["v"]) for r in run.read_csv(path)] == [
        ("baseline", "0"),
        ("anti-dive", "2"),
        ("anti-dive", "3"),
    ]
