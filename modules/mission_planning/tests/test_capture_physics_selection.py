# -*- coding: utf-8 -*-
"""FOV DB 조회를 대체하는 물리 기반 실시간 선택의 계약.

계약 요약:
- 요구공간해상도는 모니터링 품질탭(JANT 2024 면적 GSD)과 식·기본값이 같다.
- 선택 FOV = min(GSD 한계 FOV, 카메라 상한) × margin(기본 0.7, "큰거에서 30% 작게").
- 이격(sep)은 진입 WP에서 측정한 값이 최우선이고, 물리는 선택 FOV의 GSD 한계로
  자르기만 한다.  힌트가 없으면 기본 보어사이트 틸트(physics_default_boresight_
  tilt_deg)에서 이격 = 고도 × tan(틸트).
- AREA 는 일반 margin 대신 area 전용 마진(physics_area_fov_margin_ratio,
  기본 0.6)으로 더 보수적으로 선택한다 — 실비행 급선회·뱅킹 대비.
- 속도·스윕 간격은 기존 capture_geometry 법칙이 FOV에서 자동 유도한다.
- 물리 선택이 None을 반환하면(킬스위치/실패) 모든 호출부는 기존 FOV DB 경로로
  그대로 진행해야 한다(fail-open).
"""
from __future__ import annotations

import inspect
import math

from modules.mission_planning.MissionPlanner import capture_physics as cp
from modules.mission_planning.MissionPlanner.capture_geometry import (
    area_vertical_sweep_spacing_m,
    capture_aircraft_speed_kmh,
    capture_altitude_m,
    nadir_footprint_m,
    vertical_sweep_spacing_m,
)

DISABLED = {"values": {"physics_fov_selection_enabled": False}}
BASE_WIDTH = 457.0


# ------------------------------------------------------------- GSD/마진 계약


def test_required_area_matches_the_monitoring_quality_tab() -> None:
    gamma = (6.0 * 3.6) / (38 * 22)
    assert abs(cp.required_footprint_area_m2() - gamma * 1920 * 1080) < 1e-6
    assert abs(cp.required_footprint_area_m2() - 53_576.3) < 1.0


def test_slant_and_area_are_exact_inverses() -> None:
    for fov in (2.0, 4.0, 8.2):
        slant = cp.max_slant_for_fov(fov)
        area = cp.frame_area_m2(fov, slant)
        assert abs(area - cp.required_footprint_area_m2()) < 1e-6, fov


def test_selection_is_feasible_max_times_margin_with_camera_clamp() -> None:
    params = cp.physics_params()
    assert params["fov_margin"] == 0.7
    # 기본 스펙에서는 GSD 한계가 카메라 상한(8.2°)보다 커서 카메라가 제약이다.
    near = cp.select_fov_deg(1_026.0)
    assert abs(near - params["fov_max_deg"] * params["fov_margin"]) < 1e-9
    # 마진 손잡이를 바꾸면 그대로 따라간다.
    tighter = cp.select_fov_deg(1_026.0, {"values": {"physics_fov_margin_ratio": 0.5}})
    assert abs(tighter - params["fov_max_deg"] * 0.5) < 1e-9
    # 스펙을 조이면(표적 픽셀 2배) GSD가 제약이 되어 FOV가 내려간다.
    strict = cp.select_fov_deg(
        2_100.0, {"values": {"physics_obj_min_px_x": 72.0, "physics_obj_min_px_y": 38.0}}
    )
    assert strict < params["fov_max_deg"] * params["fov_margin"]


def test_selected_fov_keeps_a_real_gsd_margin_at_the_far_edge() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 1_609.0)
    meta = row["physics"]
    assert meta["frameAreaFarM2"] < meta["requiredAreaM2"]


# ------------------------------------------------------------- 이격(오프셋) 계약


def test_measured_entry_offset_is_kept_verbatim() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 1_609.0)
    assert abs(row["sep"] - 1_609.0) < 1e-9
    assert abs(row["physics"]["sepHintM"] - 1_609.0) < 1e-9


def test_missing_offset_defaults_to_the_default_boresight_tilt() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 0.0)
    altitude = capture_altitude_m(None)
    tilt = cp.physics_params()["default_tilt_deg"]
    assert abs(row["sep"] - altitude * math.tan(math.radians(tilt))) < 1e-6
    assert abs(row["physics"]["boresightTiltDeg"] - tilt) < 1e-6


def test_larger_offset_never_raises_the_fov() -> None:
    fovs = [cp.physics_line_row(BASE_WIDTH, sep)["fov"] for sep in (0.0, 1_000.0, 2_500.0, 4_000.0)]
    assert all(a >= b - 1e-9 for a, b in zip(fovs, fovs[1:])), fovs


def test_boresight_tilt_follows_the_offset_geometry() -> None:
    altitude = capture_altitude_m(None)
    row = cp.physics_line_row(BASE_WIDTH, 800.0)
    assert abs(
        row["physics"]["boresightTiltDeg"] - math.degrees(math.atan2(800.0, altitude))
    ) < 1e-6


def test_route_offset_cap_preserves_defaults_and_clips_excess() -> None:
    fov = cp.physics_line_row(BASE_WIDTH, 1_609.0)["fov"]
    kept = cp.physics_route_offset_cap_m(fov, 1_609.0)
    assert abs(kept - 1_609.0) < 1e-9
    clipped = cp.physics_route_offset_cap_m(fov, 99_999.0)
    assert 0.0 < clipped < 99_999.0
    assert cp.physics_route_offset_cap_m(fov, 0.0) == 0.0


def test_standoff_is_the_exact_inverse_of_selection() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 0.0)
    cap = row["physics"]["gsdStandoffCapM"]
    # cap 이격에서 다시 선택하면 같은 FOV가 나와야 한다 (마진 걷어낸 역산).
    again = cp.physics_line_row(BASE_WIDTH, cap)
    assert abs(again["fov"] - row["fov"]) < 1e-6


# ------------------------------------------------------------- 행 계약/속도 연동


def test_row_shape_matches_the_db_contract() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 1_609.0)
    for key in ("width", "sep", "fov", "vel"):
        assert isinstance(row[key], float) and row[key] > 0.0, key


def test_vel_comes_from_the_capture_law_in_kmh() -> None:
    row = cp.physics_line_row(BASE_WIDTH, 1_609.0)
    assert abs(row["vel"] - float(capture_aircraft_speed_kmh(row["fov"]))) < 1e-6


def test_area_selection_applies_the_dedicated_stability_margin() -> None:
    """AREA는 일반 margin(0.7) 대신 area 마진(기본 0.6)으로 더 보수적으로.

    평지 GSD 모델은 나디어 근방에서 항상 카메라 상한×0.7 을 허용하지만,
    실비행(짧은 행 급선회·뱅킹)에서는 그 FOV로 요구해상도가 깨졌다.
    """

    params = cp.physics_params()
    fov = cp.physics_area_fov_deg()
    assert fov is not None and fov > 0.0
    # 기본 스펙에서는 카메라 상한이 제약 — 상한 × area 마진.
    assert abs(fov - params["fov_max_deg"] * params["area_fov_margin"]) < 1e-9
    # 일반 margin 선택(나디어)보다 항상 좁다.
    assert fov < cp.select_fov_deg(capture_altitude_m(None))
    # 마진 손잡이를 조이면 그대로 따라간다.
    tighter = cp.physics_area_fov_deg(
        {"values": {"physics_area_fov_margin_ratio": 0.4}}
    )
    assert abs(tighter - params["fov_max_deg"] * 0.4) < 1e-9


def test_area_search_line_spacing_is_one_point_five_density() -> None:
    fov = cp.physics_area_fov_deg()
    footprint = nadir_footprint_m(fov)
    spacing = area_vertical_sweep_spacing_m(fov)
    assert fov is not None and footprint is not None and spacing is not None
    assert abs(float(spacing) - float(footprint["verticalM"]) / 1.5) < 1e-9


def test_area_margin_knob_survives_runtime_canonicalization() -> None:
    """runtime_override 로 조정 가능해야 한다 (defaults 맵 등록 증명)."""

    from modules.mission_planning.MissionPlanner.runtime_settings import (
        runtime_override,
    )

    with runtime_override({"values": {"physics_area_fov_margin_ratio": 0.5}}):
        fov = cp.physics_area_fov_deg()
    params = cp.physics_params()
    assert abs(fov - params["fov_max_deg"] * 0.5) < 1e-9


def test_area_selection_folds_in_the_sweep_row_geometry() -> None:
    """행이 길어지면 끝단 팬 슬랜트가 커져 FOV가 내려간다 (짧으면 그대로)."""

    base = cp.physics_area_fov_deg()
    short_row = cp.physics_area_fov_deg(row_length_m=200.0)
    long_row = cp.physics_area_fov_deg(row_length_m=6_000.0)
    assert abs(short_row - base) < 1e-9  # 카메라 상한 제약 구간
    assert long_row < base               # GSD가 제약으로 넘어온다
    assert long_row >= cp.physics_params()["fov_min_deg"]


def test_width_report_uses_fps15_and_50pct_overlap_knobs() -> None:
    base = cp.physics_line_row(BASE_WIDTH, 1_609.0)["width"]
    faster = cp.physics_line_row(
        BASE_WIDTH, 1_609.0, runtime_cfg={"values": {"physics_capture_fps": 30.0}}
    )["width"]
    tighter = cp.physics_line_row(
        BASE_WIDTH, 1_609.0, runtime_cfg={"values": {"physics_lateral_overlap_ratio": 0.8}}
    )["width"]
    # fps 를 2배로 올리면 폭이 크게 늘지만, 스캔 간격 자체가 촬영 법칙에서
    # 클램프되므로 정확히 2배가 되지는 않는다.
    assert faster > base * 1.4
    assert tighter < base


# ------------------------------------------------------------- 킬스위치/폴백


def test_kill_switch_turns_every_entry_point_off() -> None:
    assert cp.physics_line_row(BASE_WIDTH, 1_609.0, runtime_cfg=DISABLED) is None
    assert cp.physics_area_fov_deg(DISABLED) is None
    assert cp.select_fov_deg(1_000.0, DISABLED) is None
    assert cp.physics_route_offset_cap_m(6.5, 1_609.0, runtime_cfg=DISABLED) == 0.0
    assert cp.physics_standoff_for_fov(6.5, 0.0, DISABLED) == 0.0


def test_signature_tracks_the_knobs_for_cache_invalidation() -> None:
    base = cp.physics_signature()
    changed = cp.physics_signature({"values": {"physics_fov_margin_ratio": 0.65}})
    assert base != changed
    assert base == cp.physics_signature()


# ------------------------------------------------------------- 연결부 무결성


def test_line_runner_resolved_row_is_physics_first_with_db_fallback() -> None:
    from modules.mission_planning.runtime import next_collab_line_runner as lr

    source = inspect.getsource(lr._next_collab_resolved_db_row)
    assert "physics_line_row" in source
    assert "_db_sep_requirement_m" in source  # DB 폴백 유지

    # 물리 경로는 planner를 건드리지 않으므로 planner=None으로도 성립한다.
    row = lr._next_collab_resolved_db_row(None, BASE_WIDTH, 1_609.0)
    assert isinstance(row, dict) and row["fov"] > 0.0
    assert abs(row["sep"] - 1_609.0) < 1e-9


def test_line_runner_tprime_row_scales_the_gsd_cap_by_the_ratio() -> None:
    from modules.mission_planning.runtime import next_collab_line_runner as lr

    row = lr._next_collab_entry_tprime_db_row(None, BASE_WIDTH)
    assert isinstance(row, dict) and row["fov"] > 0.0
    cap = cp.physics_line_row(BASE_WIDTH, 0.0)["physics"]["gsdStandoffCapM"]
    ratio = lr._next_collab_entry_tprime_target_sep_ratio(None)
    assert abs(row["sep"] - cap * ratio) < 1e-6


def test_route_offset_functions_apply_the_physics_cap_first() -> None:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )
    from modules.mission_planning.pipelines import next_collab_path_builder as pb
    from modules.mission_planning.runtime import next_collab_line_runner as lr

    for module, fn in (
        (lr, lr._route_offset_sep_for_fov),
        (pb, pb._route_offset_sep_for_fov),
        (d0303, d0303._route_offset_sep_for_fov),
    ):
        source = inspect.getsource(fn)
        assert "physics_route_offset_cap_m" in source, module.__name__
        assert "_fov_db_min_sep_for_fov" in source, module.__name__  # DB 폴백 유지

    # 측정 이격은 그대로 살아남고, GSD 한계를 넘는 값만 잘린다.  한계와 측정
    # 이격은 같은 SEP 공간의 값이다 — line_route_offset_scale 로 환산하면 상한이
    # 1/scale 배로 부풀어 GSD 보증이 무력화된다(그 커플링이 첫 LINE 임무를 좁은
    # FOV 로 떨어뜨리고 영역을 못 채우게 만든 원인이었다).
    selected_fov = cp.physics_line_row(BASE_WIDTH, 1_609.0)["fov"]
    assert abs(pb._route_offset_sep_for_fov(selected_fov, 1_609.0) - 1_609.0) < 1e-9

    # 호출부는 회랑 폭 없이(0) 이격만 자르므로 같은 조건으로 기대값을 만든다.
    sep_cap = cp.physics_standoff_for_fov(selected_fov, 0.0)
    assert pb._route_offset_sep_for_fov(selected_fov, sep_cap * 0.9) > sep_cap * 0.85
    clipped = pb._route_offset_sep_for_fov(selected_fov, sep_cap * 2.0)
    assert clipped < sep_cap * 2.0
    assert abs(clipped - sep_cap) < 1.0


def test_export_0302_and_d0303_selects_are_physics_first() -> None:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )
    from modules.mission_planning.MissionPlanner.planning_enhanced.io import export_0302

    source = inspect.getsource(export_0302._piece_speed_fov_sep)
    assert "physics_area_fov_deg" in source and "physics_line_row" in source
    assert "_select_balanced_row" in source  # DB 폴백 유지

    source = inspect.getsource(d0303._select_area_db_config)
    assert "physics_area_fov_deg" in source
    assert "max_sweep_row_chord_m_xy" in source  # 행 최대 현을 기하로 전달
    assert "_select_balanced_fov_db_row" in source  # DB 폴백 유지
    # 물리 area cfg는 sep을 싣지 않아 mission_sep_m 기존값이 유지된다.
    meta = d0303._select_area_db_config(
        [(38.0, 127.0), (38.0, 127.03), (38.02, 127.03), (38.02, 127.0)], 90.0
    )
    assert meta is not None and "sep" not in meta["config"]
    # 행이 아주 긴 영역은 끝단 팬 슬랜트 때문에 FOV가 더 내려간다.
    elongated = d0303._select_area_db_config(
        [(38.0, 127.0), (38.0, 127.07), (38.005, 127.07), (38.005, 127.0)], 0.0
    )
    assert elongated is not None
    assert float(elongated["config"]["fov"]) < float(meta["config"]["fov"])


def test_line_fov_uses_assigned_corridor_width_not_route_length() -> None:
    """회랑 길이축은 기체 이동축이므로 카메라 동시 지상거리에 넣지 않는다."""

    from modules.mission_planning.MissionPlanner.planning_enhanced.io import export_0302

    line_source = inspect.getsource(export_0302._piece_speed_fov_sep)
    assert 'data.get("width")' in line_source
    assert "width_ref_m = float(piece_width_m)" in line_source
    assert "piece_width_m < width_ref_m" not in line_source
    assert "physics_line_row" in line_source
    assert "physics_sweep_row_fov_deg" not in line_source
    assert "_line_route_wp_span_m" not in line_source

    # 같은 할당 폭/SEP라면 회랑 길이가 달라도 FOV는 같다. 길이는 속도 계획에만
    # 영향을 줄 수 있고, 줌 선택의 슬랜트에는 들어가면 안 된다.
    short = cp.physics_line_row(497.0, 249.0, line_length_m=500.0)
    long = cp.physics_line_row(497.0, 249.0, line_length_m=8_000.0)
    assert short is not None and long is not None
    assert abs(float(short["fov"]) - float(long["fov"])) < 1e-12


def test_area_mission_info_builder_selects_the_physics_fov() -> None:
    from modules.mission_planning.pipelines import next_collab_path_builder as pb

    info = pb.build_mission_info_from_planned_row(
        {
            "aircraftID": 4,
            "partPolygonXY": [(0.0, 0.0), (1000.0, 0.0), (1000.0, 1000.0), (0.0, 1000.0)],
        },
        template_info={"FOV": 3.7, "individualMissionType": 4, "patternType": 6},
    )
    expected = cp.physics_area_fov_deg()
    assert expected is not None
    assert abs(float(info["FOV"]) - round(float(expected), 3)) < 1e-9

    # 행이 긴 조각은 행 최대 현이 선택에 들어가 FOV가 더 내려간다.
    long_info = pb.build_mission_info_from_planned_row(
        {
            "aircraftID": 4,
            "bearingDeg": 0.0,
            "partPolygonXY": [(0.0, 0.0), (6000.0, 0.0), (6000.0, 500.0), (0.0, 500.0)],
        },
        template_info={"FOV": 3.7, "individualMissionType": 4, "patternType": 6},
    )
    assert float(long_info["FOV"]) < float(info["FOV"])


# ------------------------------------------------ 경로 후 실기하 GSD 검증


def _sweep_waypoint(lat: float, lon: float, alt: float, fov: float, targets) -> dict:
    return {
        "coordinate": {"latitude": lat, "longitude": lon, "altitude": alt},
        "filmingProperty": {
            "fieldOfView": fov,
            "operationMode": 2,
            "lineSearch": {"coordinateList": list(targets), "width": 457.0},
        },
    }


def test_certifier_clamps_the_fov_when_the_real_geometry_violates_gsd() -> None:
    """옆으로 비껴 찍는 실제 기하에서 프레임이 허용면적을 넘으면 FOV가 내려간다."""

    # WP 고도 1500 MSL, 촬영점 지형고 500 → AGL 1000, 횡거리 약 2.5 km.
    targets = [
        {"latitude": 38.0225, "longitude": 127.0, "altitude": 500.0},
        {"latitude": 38.0225, "longitude": 127.005, "altitude": 500.0},
    ]
    waypoints = [_sweep_waypoint(38.0, 127.0, 1500.0, 6.56, targets)]
    mission_info = {"FOV": 6.56}

    before_area = cp.frame_area_m2(6.56, math.hypot(2500.0, 1000.0), altitude_m=1000.0)
    assert before_area is not None and before_area > cp.required_footprint_area_m2()

    summary = cp.certify_waypoint_gsd_inplace(waypoints, mission_info)

    assert summary["checked"] == 2
    assert summary["clamped"] is True
    assert summary["fovAfterDeg"] < summary["fovBeforeDeg"]
    assert abs(float(mission_info["FOV"]) - round(summary["fovAfterDeg"], 3)) < 1e-9
    assert abs(
        float(waypoints[0]["filmingProperty"]["fieldOfView"]) - summary["fovAfterDeg"]
    ) < 1e-9
    # 클램프 후 최악 지점 면적이 허용면적 이하로 내려와야 한다.
    after_area = cp.frame_area_m2(
        summary["fovAfterDeg"], summary["worstSlantM"], altitude_m=summary["worstAglM"]
    )
    assert after_area is not None and after_area <= cp.required_footprint_area_m2()


def test_corridor_certifier_uses_moving_leg_cross_track_distance() -> None:
    """4 km 회랑의 끝점까지를 한 WP 정지 촬영거리로 오인해 1.7°로 낮추지 않는다."""

    orient = {
        "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500.0},
        "filmingProperty": {"fieldOfView": 6.56, "operationMode": 1},
    }
    targets = []
    # 항공기는 남→북 4.4 km를 이동하고, 각 위치에서 폭 약 500 m의 횡 스윕을 본다.
    for latitude in (38.002, 38.012, 38.022, 38.032, 38.04):
        targets.extend(
            [
                {"latitude": latitude, "longitude": 126.9972, "altitude": 500.0},
                {"latitude": latitude, "longitude": 127.0028, "altitude": 500.0},
            ]
        )
    sweep = _sweep_waypoint(38.04, 127.0, 1500.0, 6.56, targets)
    mission_info = {
        "FOV": 6.56,
        "lineList": [
            {
                "width": 497.0,
                "coordinateList": [
                    {"latitude": 38.0, "longitude": 127.0},
                    {"latitude": 38.04, "longitude": 127.0},
                ],
            }
        ],
    }

    summary = cp.certify_waypoint_gsd_inplace([orient, sweep], mission_info)

    assert summary["geometryMode"] == "corridor_moving_leg"
    assert summary["clamped"] is False
    assert summary["fovAfterDeg"] == 6.56
    assert summary["worstGroundM"] < 300.0
    assert {orient["filmingProperty"]["fieldOfView"], sweep["filmingProperty"]["fieldOfView"]} == {6.56}


def test_certifier_leaves_satisfying_geometry_untouched() -> None:
    targets = [{"latitude": 38.0045, "longitude": 127.0, "altitude": 500.0}]
    waypoints = [_sweep_waypoint(38.0, 127.0, 1500.0, 5.0, targets)]
    mission_info = {"FOV": 5.0}

    summary = cp.certify_waypoint_gsd_inplace(waypoints, mission_info)

    assert summary["checked"] == 1
    assert summary["clamped"] is False
    assert abs(float(mission_info["FOV"]) - 5.0) < 1e-9


def test_certifier_uses_one_fov_for_every_filming_waypoint_in_a_path() -> None:
    """DB-free 후보정은 lineSearch WP만 줄이지 않고 path 전체를 동기화한다."""

    target = {"latitude": 38.0045, "longitude": 127.0, "altitude": 500.0}
    waypoints = [
        {
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500.0},
            "filmingProperty": {"fieldOfView": 6.56, "operationMode": 1},
        },
        _sweep_waypoint(38.0, 127.0, 1500.0, 2.5, [target]),
        {
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500.0},
            "filmingProperty": {"fieldOfView": 3.3, "operationMode": 1},
        },
    ]
    mission_info = {"FOV": 6.56}

    summary = cp.certify_waypoint_gsd_inplace(waypoints, mission_info)

    assert summary["clamped"] is False
    assert summary["normalized"] is True
    assert summary["normalizedWaypointCount"] == 2
    assert [wp["filmingProperty"]["fieldOfView"] for wp in waypoints] == [
        2.5,
        2.5,
        2.5,
    ]
    assert mission_info["FOV"] == 2.5


def test_certifier_ignores_non_sweep_waypoints() -> None:
    waypoints = [
        {"coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500.0}},
        {"coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500.0},
         "filmingProperty": {"fieldOfView": 6.56, "operationMode": 1}},
    ]
    summary = cp.certify_waypoint_gsd_inplace(waypoints, {"FOV": 6.56})
    assert summary["checked"] == 0
    assert summary["clamped"] is False


def test_certifier_respects_the_kill_switch() -> None:
    targets = [{"latitude": 38.0225, "longitude": 127.0, "altitude": 500.0}]
    waypoints = [_sweep_waypoint(38.0, 127.0, 1500.0, 6.56, targets)]
    summary = cp.certify_waypoint_gsd_inplace(
        waypoints, {"FOV": 6.56}, runtime_cfg=DISABLED
    )
    assert summary.get("skipped") == "physics_disabled"
    assert waypoints[0]["filmingProperty"]["fieldOfView"] == 6.56


def test_path_builder_runs_the_certifier_after_altitudes_are_final() -> None:
    from modules.mission_planning.pipelines import next_collab_path_builder as pb

    source = inspect.getsource(pb)
    marker = source.index("certify_waypoint_gsd_inplace")
    # 고도 프로파일·촬영점 DEM 정규화 이후, waypoint ID 부여 이전이어야 한다.
    assert source.index("normalize_filming_target_altitudes_in_waypoints") < marker
    assert marker < source.index('metrics["gsdCertify"]')


def test_tilt_stretch_makes_oblique_frames_bigger_than_the_nadir_formula() -> None:
    """이번 결함의 핵심: 경사 프레임은 나디어 공식보다 R/h 배 크다."""

    slant = 2_092.0
    agl = 1_000.0
    tilted = cp.frame_area_m2(6.56, slant, altitude_m=agl)
    nadir_formula = cp.frame_area_m2(6.56, slant, altitude_m=slant)  # h=R → 옛 공식
    assert tilted is not None and nadir_formula is not None
    assert abs(tilted / nadir_formula - slant / agl) < 1e-6


def test_sweep_row_length_tightens_the_fov_beyond_the_offset_alone() -> None:
    """이번 실증 결함의 핵심: 행 끝단이 이격보다 훨씬 먼 최악 슬랜트가 된다."""

    offset_only = cp.physics_line_row(BASE_WIDTH, 1_000.0)["fov"]
    with_row = cp.physics_sweep_row_fov_deg(row_length_m=6_000.0, lateral_offset_m=1_000.0)
    assert with_row is not None
    assert with_row < offset_only

    # 행이 짧으면 이격만 볼 때와 같아진다.
    short_row = cp.physics_sweep_row_fov_deg(row_length_m=1.0, lateral_offset_m=1_000.0)
    assert abs(short_row - offset_only) < 1e-6


def test_initial_planning_certifies_after_target_altitudes_are_final() -> None:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )

    source = inspect.getsource(d0303)
    marker = source.index("certify_waypoint_gsd_inplace")
    # 촬영점 지형고 정규화·바닥 적용 이후, waypoint ID 부여 이전이어야 한다.
    assert source.index("_enforce_filming_target_altitude_floor_inplace(wps)") < marker
    assert marker < source.index('wp["waypointID"] = wp_alloc.alloc()')
    # 동일 프로세스 경로는 확정 FOV를 원본 0302 임무 정보에도 되돌린다.
    assert '"_mission_info": info' in source
    assert 'pkt.get("_mission_info")' in source
    assert 'pkt.pop("_mission_info", None)' in source
    assert "gsdCertifyClampedPaths" in source
    # 최종 실기하에서 정말 FOV가 더 좁아지면 그 값으로 스윕 간격/속도까지
    # 다시 계산한 LINE 경로를 한 번 재생성한다.
    assert "gsdCertifyLineRebuildPaths" in source
    assert "_physics_line_rebuild_pass=1" in source


def test_a_genuinely_narrow_line_fov_produces_dense_search_lines() -> None:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )

    # SEP 4 km는 30% 안전 마진에서 실제로 1.5°대가 필요한 기하다. 이 값이
    # 생성 전에 선택되면 d0303 search-line 간격도 약 10.7 m로 촘촘해져야 한다.
    row = cp.physics_line_row(497.0, 4_000.0)
    assert row is not None and 1.5 < float(row["fov"]) < 1.65
    expected_spacing = vertical_sweep_spacing_m(float(row["fov"]))
    actual_spacing = d0303._sweep_spacing_m(
        separation_m=float(row["sep"]),
        fov_deg=float(row["fov"]),
    )
    assert expected_spacing is not None
    assert abs(actual_spacing - expected_spacing) < 1e-9
    assert 10.0 < actual_spacing < 11.5
    assert actual_spacing < float(vertical_sweep_spacing_m(5.74)) * 0.3


def test_late_line_gsd_clamp_rebuilds_the_search_line_spacing() -> None:
    from unittest.mock import patch

    from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303,
    )

    mission = {
        "aircraftID": 4,
        "pathID": 400000111,
        "individualMissionID": 900000111,
        "relatedMission": {"inputMissionID": 1},
        "individualMissionInfo": {
            "individualMissionType": 6,
            "patternType": 8,
            "SEP": 249.0,
            "routeOffsetSepM": 249.0,
            "FOV": 6.56,
            "SPEED": 144.0,
            "lineList": [
                {
                    "width": 497.0,
                    "coordinateList": [
                        {"latitude": 38.0, "longitude": 127.0, "altitude": 0},
                        {"latitude": 38.01, "longitude": 127.0, "altitude": 0},
                    ],
                }
            ],
        },
    }
    strict_gsd = {
        "values": {
            "physics_fov_selection_enabled": True,
            "physics_obj_min_px_x": 190.0,
            "physics_obj_min_px_y": 110.0,
        }
    }

    d0303.reset_dense_linesearch_metrics()
    with patch.object(d0303, "_dem_alt", return_value=0.0), runtime_override(strict_gsd):
        [flight_path] = d0303.build_flight_plans([mission], cruise_speed=40.0)

    final_fov = float(mission["individualMissionInfo"]["FOV"])
    assert final_fov < 6.56
    assert {
        round(float((wp.get("filmingProperty") or {}).get("fieldOfView")), 3)
        for wp in flight_path["waypointList"]
        if (wp.get("filmingProperty") or {}).get("fieldOfView")
    } == {round(final_fov, 3)}

    coords = []
    interpolation_points = 2
    for waypoint in flight_path["waypointList"]:
        line_search = (waypoint.get("filmingProperty") or {}).get("lineSearch") or {}
        coords.extend(line_search.get("coordinateList") or [])
        interpolation_points = int(
            line_search.get("interpolationPoints") or interpolation_points
        )
    groups = [
        coords[index : index + interpolation_points]
        for index in range(0, len(coords), interpolation_points)
        if len(coords[index : index + interpolation_points]) == interpolation_points
    ]
    lat0 = float(groups[0][0]["latitude"])
    lon0 = float(groups[0][0]["longitude"])
    cos_lat = math.cos(math.radians(lat0))

    def _midpoint(group) -> tuple[float, float]:
        xy = [
            (
                (float(coord["longitude"]) - lon0) * 111_320.0 * cos_lat,
                (float(coord["latitude"]) - lat0) * 111_132.9,
            )
            for coord in group
        ]
        return (
            sum(point[0] for point in xy) / len(xy),
            sum(point[1] for point in xy) / len(xy),
        )

    midpoints = [_midpoint(group) for group in groups]
    gaps = [
        math.hypot(
            midpoints[index][0] - midpoints[index - 1][0],
            midpoints[index][1] - midpoints[index - 1][1],
        )
        for index in range(1, len(midpoints))
    ]
    required_spacing = float(vertical_sweep_spacing_m(final_fov))
    median_gap = sorted(gaps)[len(gaps) // 2]
    assert median_gap <= required_spacing * 1.05
    assert d0303.get_dense_linesearch_metrics().get("gsdCertifyLineRebuildPaths") == 1


def test_certifier_never_raises_an_already_narrower_waypoint_fov() -> None:
    targets = [{"latitude": 38.0225, "longitude": 127.0, "altitude": 500.0}]
    waypoints = [
        _sweep_waypoint(38.0, 127.0, 1500.0, 6.56, targets),
        _sweep_waypoint(38.0, 127.0, 1500.0, 1.5, targets),
    ]
    summary = cp.certify_waypoint_gsd_inplace(waypoints, None)

    assert summary["clamped"] is True
    # 이미 더 좁던 waypoint를 올리지 않고 그 값을 path 공통 FOV로 쓴다.
    assert waypoints[0]["filmingProperty"]["fieldOfView"] == 1.5
    assert waypoints[1]["filmingProperty"]["fieldOfView"] == 1.5
    assert summary["fovAfterDeg"] == 1.5


def test_syncs_worker_selected_path_fov_back_to_initial_mission_info() -> None:
    """0303 워커의 최종 FOV가 부모 프로세스에서 생성할 0302에도 반영된다."""

    target = {"latitude": 38.0045, "longitude": 127.0, "altitude": 500.0}
    missions = [
        {
            "aircraftID": 4,
            "pathID": 400000001,
            "individualMissionInfo": {"FOV": 6.56},
        }
    ]
    flight_plans = [
        {
            "aircraftID": 4,
            "pathID": 400000001,
            "waypointList": [
                {
                    "coordinate": {
                        "latitude": 38.0,
                        "longitude": 127.0,
                        "altitude": 1500.0,
                    },
                    "filmingProperty": {"fieldOfView": 6.56, "operationMode": 1},
                },
                _sweep_waypoint(38.0, 127.0, 1500.0, 2.5, [target]),
            ],
        }
    ]

    summary = cp.sync_mission_fov_from_flight_plans(missions, flight_plans)

    assert summary["matchedPaths"] == 1
    assert summary["updatedMissions"] == 1
    assert summary["normalizedWaypoints"] == 1
    assert missions[0]["individualMissionInfo"]["FOV"] == 2.5
    assert {
        wp["filmingProperty"]["fieldOfView"]
        for wp in flight_plans[0]["waypointList"]
    } == {2.5}


def test_plan_cache_keys_include_the_physics_signature() -> None:
    from modules.mission_planning.runtime import next_collab_line_runner as lr

    assert "physicsFovSignature" in inspect.getsource(lr._line_plan_cache_key)
    module_source = inspect.getsource(lr)
    assert module_source.count('"physicsFovSignature"') >= 3
