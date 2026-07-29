from __future__ import annotations

import math
from collections.abc import Iterable

import pytest

from modules.mission_planning.MissionPlanner.data_def.lah_terminal_cover import (
    select_lah_terminal_cover_point,
)
from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0304 import (
    _constrain_lah_terminal_support_coordinate,
)
from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0304 as d0304_module,
)


ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
METRES_PER_LATITUDE = 111_132.92
METRES_PER_LONGITUDE = METRES_PER_LATITUDE * math.cos(math.radians(ORIGIN_LAT))


def _coordinate(east_m: float, north_m: float, altitude_m: float = 0.0) -> dict:
    return {
        "latitude": ORIGIN_LAT + float(north_m) / METRES_PER_LATITUDE,
        "longitude": ORIGIN_LON + float(east_m) / METRES_PER_LONGITUDE,
        "altitude": float(altitude_m),
    }


def _xy(coordinate: object) -> tuple[float, float]:
    if isinstance(coordinate, dict):
        latitude = float(coordinate["latitude"])
        longitude = float(coordinate["longitude"])
    else:
        latitude, longitude = coordinate  # type: ignore[misc]
        latitude = float(latitude)
        longitude = float(longitude)
    return (
        (longitude - ORIGIN_LON) * METRES_PER_LONGITUDE,
        (latitude - ORIGIN_LAT) * METRES_PER_LATITUDE,
    )


def _area_row(vertices_xy: Iterable[tuple[float, float]], *, is_hole: bool = False) -> dict:
    return {
        "isHole": bool(is_hole),
        "coordinateList": [_coordinate(east, north) for east, north in vertices_xy],
    }


def _rectangle(
    west_m: float,
    south_m: float,
    east_m: float,
    north_m: float,
    *,
    is_hole: bool = False,
) -> dict:
    return _area_row(
        [
            (west_m, south_m),
            (east_m, south_m),
            (east_m, north_m),
            (west_m, north_m),
        ],
        is_hole=is_hole,
    )


def _point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    tolerance_m: float = 0.05,
) -> bool:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px - ax, py - ay) <= tolerance_m
    ratio = ((px - ax) * dx + (py - ay) * dy) / length_sq
    if ratio < 0.0 or ratio > 1.0:
        return False
    closest = (ax + ratio * dx, ay + ratio * dy)
    return math.hypot(px - closest[0], py - closest[1]) <= tolerance_m


def _point_in_polygon(point: tuple[float, float], vertices: list[tuple[float, float]]) -> bool:
    if len(vertices) < 3:
        return False
    if any(
        _point_on_segment(point, start, end)
        for start, end in zip(vertices, vertices[1:] + vertices[:1])
    ):
        return True
    px, py = point
    inside = False
    previous = vertices[-1]
    for current in vertices:
        xi, yi = current
        xj, yj = previous
        if (yi > py) != (yj > py):
            x_intersection = xi + (py - yi) * (xj - xi) / (yj - yi)
            if px < x_intersection:
                inside = not inside
        previous = current
    return inside


def _vertices_xy(area_row: dict) -> list[tuple[float, float]]:
    return [_xy(coordinate) for coordinate in area_row["coordinateList"]]


def _inside_area(coordinate: dict, area_list: list[dict]) -> bool:
    point = _xy(coordinate)
    outers = [row for row in area_list if not bool(row.get("isHole", False))]
    holes = [row for row in area_list if bool(row.get("isHole", False))]
    return any(_point_in_polygon(point, _vertices_xy(row)) for row in outers) and not any(
        _point_in_polygon(point, _vertices_xy(row)) for row in holes
    )


def _point_to_segment_distance_m(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = min(1.0, max(0.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _distance_to_area_boundaries_m(coordinate: dict, area_list: list[dict]) -> float:
    point = _xy(coordinate)
    distances: list[float] = []
    for row in area_list:
        vertices = _vertices_xy(row)
        distances.extend(
            _point_to_segment_distance_m(point, start, end)
            for start, end in zip(vertices, vertices[1:] + vertices[:1])
        )
    return min(distances) if distances else 0.0


def _terrain_coordinate(raw: object) -> tuple[float, float]:
    if isinstance(raw, dict):
        return float(raw["latitude"]), float(raw["longitude"])
    latitude, longitude = raw  # type: ignore[misc]
    return float(latitude), float(longitude)


def _terrain_many_for_cover_case(coordinates: Iterable[object]) -> list[float]:
    elevations: list[float] = []
    for raw in coordinates:
        east_m, _north_m = _xy(_terrain_coordinate(raw))
        if -340.0 <= east_m <= -180.0:
            elevations.append(620.0)  # Threat-facing masking ridge.
        elif east_m < -400.0:
            elevations.append(80.0)  # Concealed valley on the UAV-facing side.
        else:
            elevations.append(140.0)  # Exposed centre and threat side.
    return elevations


def _terrain_altitude(coordinate: dict) -> float:
    return float(_terrain_many_for_cover_case([coordinate])[0])


def _ray_is_clear(start: dict, end: dict, *, samples: int = 41) -> bool:
    start_altitude = float(start["altitude"])
    end_altitude = float(end["altitude"])
    pairs: list[tuple[float, float]] = []
    sightline: list[float] = []
    for index in range(1, samples - 1):
        ratio = index / float(samples - 1)
        pairs.append(
            (
                float(start["latitude"])
                + (float(end["latitude"]) - float(start["latitude"])) * ratio,
                float(start["longitude"])
                + (float(end["longitude"]) - float(start["longitude"])) * ratio,
            )
        )
        sightline.append(start_altitude + (end_altitude - start_altitude) * ratio)
    terrain = _terrain_many_for_cover_case(pairs)
    return all(ground + 1.0 < line for ground, line in zip(terrain, sightline))


def test_selects_concealed_uav_visible_point_instead_of_area_centre() -> None:
    area_list = [_rectangle(-1_000.0, -800.0, 1_000.0, 800.0)]
    fallback = _coordinate(0.0, 0.0, 190.0)
    threats = [
        _coordinate(1_300.0, north_m, 190.0)
        for north_m in (-350.0, 0.0, 350.0)
    ]
    uav = _coordinate(-1_500.0, 0.0, 700.0)

    selected, diagnostics = select_lah_terminal_cover_point(
        area_list,
        fallback,
        threat_coordinates=threats,
        uav_coordinate=uav,
        terrain_elev_many_fn=_terrain_many_for_cover_case,
        hold_agl_m=50.0,
        max_candidates=49,
        max_threats=5,
        max_ray_samples=64,
        minimum_threat_masking_depth_m=30.0,
    )

    selected_east_m, _selected_north_m = _xy(selected)
    assert selected_east_m < -400.0
    assert _inside_area(selected, area_list)
    assert diagnostics["fallbackUsed"] is False
    assert diagnostics["coverFraction"] >= 2.0 / 3.0
    assert diagnostics["selectedThreatMaskingDepthM"] >= 30.0
    assert diagnostics["threatMaskingDepthPreferenceAvailable"] is True
    assert diagnostics["uavLosClear"] is True
    assert diagnostics["uavLosMarginM"] >= 0.0
    assert diagnostics["uavDistanceFeasible"] is True

    selected_with_altitude = dict(selected)
    selected_with_altitude["altitude"] = max(
        float(selected_with_altitude.get("altitude", 0.0)),
        _terrain_altitude(selected) + 50.0,
    )
    centre_with_altitude = dict(fallback)
    centre_with_altitude["altitude"] = _terrain_altitude(fallback) + 50.0
    assert not _ray_is_clear(selected_with_altitude, threats[1])
    assert _ray_is_clear(centre_with_altitude, threats[1])
    assert _ray_is_clear(selected_with_altitude, uav)


def test_hole_and_boundary_inset_reject_the_lowest_invalid_candidate() -> None:
    area_list = [
        _rectangle(-1_200.0, -900.0, 1_200.0, 900.0),
        _rectangle(-260.0, -260.0, 260.0, 260.0, is_hole=True),
    ]
    fallback = _coordinate(-700.0, 0.0, 250.0)

    def terrain_many(coordinates: Iterable[object]) -> list[float]:
        values: list[float] = []
        for raw in coordinates:
            east_m, north_m = _xy(_terrain_coordinate(raw))
            values.append(-500.0 if abs(east_m) < 250.0 and abs(north_m) < 250.0 else 200.0)
        return values

    selected, _diagnostics = select_lah_terminal_cover_point(
        area_list,
        fallback,
        terrain_elev_many_fn=terrain_many,
        max_candidates=49,
    )

    assert _inside_area(selected, area_list)
    # The 1.8 km short span requests the configured, capped 100 m inset.
    assert _distance_to_area_boundaries_m(selected, area_list) >= 90.0


def test_concave_area_never_selects_the_low_excluded_notch() -> None:
    concave_outer = _area_row(
        [
            (-1_200.0, -900.0),
            (1_200.0, -900.0),
            (1_200.0, 900.0),
            (300.0, 900.0),
            (300.0, -150.0),
            (-300.0, -150.0),
            (-300.0, 900.0),
            (-1_200.0, 900.0),
        ]
    )
    area_list = [concave_outer]
    fallback = _coordinate(-800.0, 0.0, 250.0)

    def terrain_many(coordinates: Iterable[object]) -> list[float]:
        values: list[float] = []
        for raw in coordinates:
            east_m, north_m = _xy(_terrain_coordinate(raw))
            in_excluded_notch = -290.0 < east_m < 290.0 and -140.0 < north_m < 890.0
            values.append(-800.0 if in_excluded_notch else 180.0)
        return values

    selected, _diagnostics = select_lah_terminal_cover_point(
        area_list,
        fallback,
        terrain_elev_many_fn=terrain_many,
        max_candidates=49,
    )

    assert _inside_area(selected, area_list)
    assert _distance_to_area_boundaries_m(selected, area_list) >= 85.0


@pytest.mark.parametrize("failure_mode", ["exception", "nan"])
def test_dem_failure_returns_the_safe_internal_fallback(failure_mode: str) -> None:
    area_list = [_rectangle(-1_000.0, -800.0, 1_000.0, 800.0)]
    fallback = _coordinate(125.0, -75.0, 333.0)

    def terrain_many(coordinates: Iterable[object]) -> list[float]:
        rows = list(coordinates)
        if failure_mode == "exception":
            raise RuntimeError("synthetic DEM failure")
        return [float("nan")] * len(rows)

    selected, diagnostics = select_lah_terminal_cover_point(
        area_list,
        fallback,
        uav_coordinate=_coordinate(2_000.0, 0.0, 700.0),
        terrain_elev_many_fn=terrain_many,
    )

    assert selected["latitude"] == pytest.approx(fallback["latitude"])
    assert selected["longitude"] == pytest.approx(fallback["longitude"])
    assert _inside_area(selected, area_list)
    assert diagnostics["fallbackUsed"] is True
    assert isinstance(diagnostics["reason"], str) and diagnostics["reason"]


def test_all_area_candidates_far_from_uav_remain_inside_area() -> None:
    area_list = [_rectangle(-1_000.0, -800.0, 1_000.0, 800.0)]
    fallback = _coordinate(0.0, 0.0, 150.0)
    far_uav = _coordinate(30_000.0, 0.0, 1_000.0)

    selected, diagnostics = select_lah_terminal_cover_point(
        area_list,
        fallback,
        uav_coordinate=far_uav,
        terrain_elev_many_fn=lambda coordinates: [100.0 for _ in coordinates],
        max_uav_distance_m=20_000.0,
    )

    assert _inside_area(selected, area_list)
    assert diagnostics["uavDistanceM"] > 20_000.0
    assert diagnostics["uavDistanceFeasible"] is False


def test_shared_terminal_cover_uses_one_horizontal_site_for_all_lah(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cover_point = _coordinate(-200.0, 0.0, 0.0)
    area_list = [_rectangle(-1_000.0, -800.0, 1_000.0, 800.0)]
    info = {
        "individualMissionType": 9,
        "patternType": 12,
        "autoZoomIn": False,
        "coordinateList": [dict(cover_point)],
        "targetID": None,
        "_lahTerminalCoverEnabled": True,
        "_lahSharedTerminalCoverPoint": True,
        "_lahConstraintAreaList": area_list,
        "_lahTerminalCoverThreatCoordinateList": [
            _coordinate(1_500.0, north_m, 0.0)
            for north_m in (-400.0, 0.0, 400.0)
        ],
        "_lahTerminalCoverFallbackCoordinate": dict(cover_point),
    }
    missions = [
        {
            "aircraftID": aircraft_id,
            "pathID": aircraft_id * 100_000_000 + 1,
            "individualMissionInfo": dict(info),
        }
        for aircraft_id in (1, 2, 3)
    ]
    selector_calls: list[object] = []

    def _unexpected_reselection(*args, **kwargs):
        selector_calls.append((args, kwargs))
        return _coordinate(500.0, 500.0, 0.0), {"reason": "unexpected"}

    monkeypatch.setattr(
        d0304_module,
        "select_lah_terminal_cover_point",
        _unexpected_reselection,
    )
    packets = d0304_module.build_lah_flight_plans_fixed(missions)
    d0304_module.apply_uav_eta_follow_speed_plan(
        packets,
        [],
        lah_missions=missions,
    )

    terminal_sites = {
        (
            round(float(packet["lahWaypointList"][-1]["coordinate"]["latitude"]), 6),
            round(float(packet["lahWaypointList"][-1]["coordinate"]["longitude"]), 6),
        )
        for packet in packets
    }
    assert terminal_sites == {
        (
            round(float(cover_point["latitude"]), 6),
            round(float(cover_point["longitude"]), 6),
        )
    }
    assert selector_calls == []


def test_selection_is_deterministic_and_respects_candidate_and_dem_sample_caps() -> None:
    area_list = [_rectangle(-1_200.0, -900.0, 1_200.0, 900.0)]
    fallback = _coordinate(0.0, 0.0, 250.0)
    threats = [_coordinate(1_400.0, -600.0 + index * 150.0, 250.0) for index in range(9)]
    uav = _coordinate(-1_500.0, 0.0, 800.0)
    max_candidates = 25
    max_threats = 3
    max_ray_samples = 16
    observed_sample_counts: list[int] = []

    def run_once() -> tuple[dict, dict, int]:
        sample_count = 0

        def terrain_many(coordinates: Iterable[object]) -> list[float]:
            nonlocal sample_count
            rows = list(coordinates)
            sample_count += len(rows)
            return _terrain_many_for_cover_case(rows)

        coordinate, diagnostics = select_lah_terminal_cover_point(
            area_list,
            fallback,
            threat_coordinates=threats,
            uav_coordinate=uav,
            terrain_elev_many_fn=terrain_many,
            max_candidates=max_candidates,
            max_threats=max_threats,
            max_ray_samples=max_ray_samples,
        )
        return coordinate, diagnostics, sample_count

    outputs = [run_once() for _ in range(3)]
    coordinate_signatures = [
        (round(float(coordinate["latitude"]), 8), round(float(coordinate["longitude"]), 8))
        for coordinate, _diagnostics, _sample_count in outputs
    ]
    assert len(set(coordinate_signatures)) == 1

    # Candidate ground, local-relief neighbours, capped threat rays and one UAV
    # ray all remain proportional to the public caps rather than the DEM size.
    sample_upper_bound = max_candidates * (max_threats + 2) * (max_ray_samples + 2)
    for _coordinate_out, diagnostics, callback_sample_count in outputs:
        observed_sample_counts.append(callback_sample_count)
        assert diagnostics["candidateCount"] <= max_candidates
        assert diagnostics["demSampleCount"] <= callback_sample_count
        assert callback_sample_count <= sample_upper_bound
    assert len(set(observed_sample_counts)) == 1


def test_d0304_terminal_support_constraint_keeps_outside_desired_point_at_fallback() -> None:
    area_list = [
        _rectangle(-1_000.0, -800.0, 1_000.0, 800.0),
        _rectangle(-200.0, -200.0, 200.0, 200.0, is_hole=True),
    ]
    fallback = _coordinate(-600.0, 0.0, 350.0)
    desired_outside = _coordinate(5_000.0, 0.0, 450.0)
    info = {"_lahConstraintAreaList": area_list}

    constrained = _constrain_lah_terminal_support_coordinate(
        info,
        desired_outside,
        fallback,
    )

    assert constrained["latitude"] == pytest.approx(fallback["latitude"])
    assert constrained["longitude"] == pytest.approx(fallback["longitude"])
    assert constrained["altitude"] == pytest.approx(fallback["altitude"])
    assert _inside_area(constrained, area_list)


def test_far_uav_keeps_cover_terminal_inside_area_and_uses_max_speed(monkeypatch) -> None:
    area_list = [_rectangle(-1_000.0, -800.0, 1_000.0, 800.0)]
    fallback = _coordinate(-200.0, 0.0, 150.0)
    path_id = 100_000_001

    def waypoint(waypoint_id: int, *, terminal: bool = False) -> dict:
        row = {
            "waypointID": waypoint_id,
            "isDone": False,
            "coordinate": dict(fallback),
            "speed": 20.0,
            "eta": 0,
            "ecf": 1.0 if terminal else 0.0,
            "nextWaypointID": 0 if terminal else waypoint_id + 1,
            "attack": {"targetID": 0, "weaponType": 0},
        }
        if terminal:
            row["hovering"] = {"time": 300}
        return row

    lah_packet = {
        "pathID": path_id,
        "aircraftID": 1,
        "lahWaypointList": [waypoint(1), waypoint(2, terminal=True)],
    }
    far_uav = _coordinate(30_000.0, 0.0, 1_000.0)
    uav_packet = {
        "pathID": 400_000_001,
        "aircraftID": 4,
        "uavWaypointList": [
            {"coordinate": dict(far_uav), "eta": 0},
            {"coordinate": dict(far_uav), "eta": 600},
        ],
    }
    info = {
        "_lahTerminalCoverEnabled": True,
        "_lahConstraintAreaList": area_list,
        "_lahTerminalCoverThreatCoordinateList": [],
        "_lahTerminalCoverFallbackCoordinate": dict(fallback),
    }
    missions = [{"pathID": path_id, "aircraftID": 1, "individualMissionInfo": info}]

    monkeypatch.setattr(
        d0304_module,
        "select_lah_terminal_cover_point",
        lambda *_args, **_kwargs: (
            dict(fallback),
            {"uavDistanceFeasible": False, "reason": "synthetic"},
        ),
    )
    monkeypatch.setattr(
        d0304_module,
        "_terrain_profile_many",
        lambda coordinates: [100.0 for _ in coordinates],
    )

    d0304_module.apply_uav_eta_follow_speed_plan(
        [lah_packet],
        [uav_packet],
        lah_missions=missions,
    )

    terminal = lah_packet["lahWaypointList"][-1]
    assert _inside_area(terminal["coordinate"], area_list)
    assert _xy(terminal["coordinate"])[0] == pytest.approx(-200.0, abs=0.2)
    assert terminal["hovering"] == {"time": 300}
    assert all(
        waypoint_row["speed"] == pytest.approx(round(d0304_module._lah_max_speed_mps(), 2))
        for waypoint_row in lah_packet["lahWaypointList"]
    )
    assert "_lahTerminalCover" not in repr(lah_packet)
