"""AREA 전환 out-leg 링크의 계약.

사용자 결정(2026-07-28): out-leg 점(이전 영역 끝에서 다음 영역으로 나가는 선회
접점 WP)은 유용성이 없어 **기본적으로 방출하지 않는다** — 초기계획(d0303)과
다음협업 재계획 모두.  로직은 보존되어 ``area_dubins_entry_links_enabled`` 를
켜면 두 빌더가 같은 링크를 다시 방출한다(공유 기하 ``compute_area_transition_
link``).  켰을 때의 계약: FLYOVER + 이웃 AGL 이월 + 내부 마커 무누출.
"""

from __future__ import annotations

import math

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303,
)
from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override
from modules.mission_planning.replanning.triggers.next_collab import pipeline as ncp

LINKS_ON = {"values": {"area_dubins_entry_links_enabled": True}}

EARTH_RADIUS_M = 6_371_008.8

# The AC5 half-split reversal from Logs/260727: lane A ends northbound, lane B
# starts southbound about 770 m to the east.
PREV_START = {"latitude": 38.17171, "longitude": 127.24816, "altitude": 1203}
PREV_END = {"latitude": 38.188605, "longitude": 127.2465, "altitude": 1213}
NEXT_FIRST = {"latitude": 38.188549, "longitude": 127.255313, "altitude": 1231}
NEXT_SECOND = {"latitude": 38.180400, "longitude": 127.252660, "altitude": 1231}
LAST_CAPTURE = {
    "latitude": 38.1849065556195,
    "longitude": 127.25009154064381,
    "altitude": 183,
}
SPEED_MPS = 52.79


def _distance_m(left, right) -> float:
    lat = math.radians((float(left["latitude"]) + float(right["latitude"])) / 2.0)
    return math.hypot(
        math.radians(float(right["longitude"]) - float(left["longitude"]))
        * EARTH_RADIUS_M
        * math.cos(lat),
        math.radians(float(right["latitude"]) - float(left["latitude"])) * EARTH_RADIUS_M,
    )


def _scan_waypoint(waypoint_id, coordinate, scan_coordinates):
    return {
        "waypointID": int(waypoint_id),
        "coordinate": dict(coordinate),
        "speed": SPEED_MPS,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 1,
        "isDone": False,
        "filmingProperty": {
            "fieldOfView": 7.2,
            "sensorType": 1,
            "operationMode": 2,
            "lineSearch": {
                "coordinateList": [dict(item) for item in scan_coordinates],
                "searchSpeed": 1130.0,
            },
        },
    }


def _consecutive_area_paths():
    previous = {
        "waypointList": [
            _scan_waypoint(1, PREV_START, [PREV_START, {"latitude": 38.18427, "longitude": 127.25004, "altitude": 186}]),
            _scan_waypoint(2, PREV_END, [{"latitude": 38.18427, "longitude": 127.25004, "altitude": 186}, LAST_CAPTURE]),
        ]
    }
    following = {
        "waypointList": [
            _scan_waypoint(3, NEXT_FIRST, [NEXT_FIRST]),
            _scan_waypoint(4, NEXT_SECOND, [NEXT_SECOND]),
        ]
    }
    return {101: previous, 102: following}


def _link_waypoints(payload):
    # 링크는 마지막 스캔 WP 뒤에 순서대로 덧붙는다.  (내부 마커는 더 이상
    # 출력에 남지 않으므로 위치로 식별한다.)
    return [
        waypoint
        for waypoint in payload["waypointList"]
        if not (waypoint.get("filmingProperty") or {}).get("lineSearch")
    ]


def test_out_leg_links_are_not_emitted_by_default() -> None:
    """기본값: out-leg 점은 초기계획·재계획 어디서도 나오지 않는다."""

    generated = _consecutive_area_paths()
    before = len(generated[101]["waypointList"])
    appended = ncp._append_next_collab_area_transition_links(
        generated_fp_by_path=generated,
        ordered_path_ids_by_aircraft={4: [101, 102]},
        emit=lambda _message: None,
    )
    assert appended == 0
    assert len(generated[101]["waypointList"]) == before

    prev_pkt = {
        "wplist": [
            _scan_waypoint(1, PREV_START, [PREV_START]),
            _scan_waypoint(2, PREV_END, [LAST_CAPTURE]),
        ],
        "_mission_cruise_speed_mps": SPEED_MPS,
    }
    next_pkt = {
        "wplist": [
            _scan_waypoint(3, NEXT_FIRST, [NEXT_FIRST]),
            _scan_waypoint(4, NEXT_SECOND, [NEXT_SECOND]),
        ],
    }
    assert (
        d0303._append_area_transition_links_inplace(
            prev_pkt, next_pkt, altitude_fn=lambda _lat, _lon: 1298
        )
        == 0
    )
    assert len(prev_pkt["wplist"]) == 2


def test_sequential_two_area_pair_omits_public_turn_helpers() -> None:
    """Turn geometry may guide direction, but only capture WPs are commanded."""

    generated = _consecutive_area_paths()
    before = len(generated[101]["waypointList"])
    with runtime_override(LINKS_ON):
        appended = ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            suppressed_link_pairs={(101, 102)},
            emit=lambda _message: None,
        )

    assert appended == 0
    assert len(generated[101]["waypointList"]) == before
    assert not _link_waypoints(generated[101])


def test_next_collab_area_replan_emits_a_transition_link_when_enabled() -> None:
    generated = _consecutive_area_paths()
    with runtime_override(LINKS_ON):
        appended = ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            emit=lambda _message: None,
        )
    assert appended > 0
    assert _link_waypoints(generated[101])
    waypoints = generated[101]["waypointList"]
    eta_values = [int(waypoint["eta"]) for waypoint in waypoints]
    assert eta_values == sorted(eta_values)
    assert all(
        int(waypoint["eta"]) > int(waypoints[1]["eta"])
        for waypoint in _link_waypoints(generated[101])
    )


def test_next_collab_link_matches_the_initial_plan_builder() -> None:
    """Same geometry, same link - the two builders may not disagree."""

    radius_m = d0303._turn_radius_m_for_speed(SPEED_MPS)
    reference_coords, reference_speed_mps = d0303.compute_area_transition_link(
        prev_start_coord=PREV_START,
        prev_end_coord=PREV_END,
        next_first_coord=NEXT_FIRST,
        next_second_coord=NEXT_SECOND,
        turn_radius_m=radius_m,
        cruise_speed_mps=SPEED_MPS,
        altitude_fn=lambda _lat, _lon: 1298,
        min_link_gap_m=max(80.0, radius_m * 0.2),
    )
    assert reference_coords

    generated = _consecutive_area_paths()
    with runtime_override(LINKS_ON):
        ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            emit=lambda _message: None,
        )
    links = _link_waypoints(generated[101])

    assert len(links) == len(reference_coords)
    for waypoint, reference in zip(links, reference_coords):
        assert _distance_m(waypoint["coordinate"], reference) <= 1.0
        assert abs(float(waypoint["speed"]) - float(reference_speed_mps)) < 0.05


def test_link_speed_matches_the_radius_the_turn_actually_needs() -> None:
    """A tightened reversal is flown slower, not at the scan speed."""

    generated = _consecutive_area_paths()
    with runtime_override(LINKS_ON):
        ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            emit=lambda _message: None,
        )
    links = _link_waypoints(generated[101])
    assert links
    # The reversal is tighter than the cruise-speed radius allows, so the link
    # is slowed; flying it at scan speed is what the aircraft cannot do.
    assert float(links[0]["speed"]) < SPEED_MPS


def test_link_stares_at_the_last_captured_point() -> None:
    """No observation gap across the transition."""

    generated = _consecutive_area_paths()
    with runtime_override(LINKS_ON):
        ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            emit=lambda _message: None,
        )
    for waypoint in _link_waypoints(generated[101]):
        filming = waypoint.get("filmingProperty") or {}
        orientation = (filming.get("coordinateOrientation") or {}).get("coordinate")
        assert orientation is not None
        assert _distance_m(orientation, LAST_CAPTURE) <= 1.0


def test_single_area_path_is_left_alone() -> None:
    """Nothing to join means nothing is added."""

    generated = _consecutive_area_paths()
    before = len(generated[101]["waypointList"])
    with runtime_override(LINKS_ON):
        appended = ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101]},
            emit=lambda _message: None,
        )
    assert appended == 0
    assert len(generated[101]["waypointList"]) == before


def test_out_leg_is_flyover_and_carries_neighbor_agl(monkeypatch) -> None:
    """진출점(out leg)은 FLYOVER로 실제 통과하고, 지형에 붙지 않아야 한다.

    재계획 링크가 맨 DEM 지면고를 쓰던 회귀: 링크 WP 고도가 지형고 그대로라
    "땅에 붙은 점"이 생겼다.  이웃 WP의 AGL을 이월해야 한다.
    """

    ground_m = 200.0
    monkeypatch.setattr(ncp, "_dem_alt", lambda _lat, _lon: ground_m)
    generated = _consecutive_area_paths()
    with runtime_override(LINKS_ON):
        ncp._append_next_collab_area_transition_links(
            generated_fp_by_path=generated,
            ordered_path_ids_by_aircraft={4: [101, 102]},
            emit=lambda _message: None,
        )
    links = _link_waypoints(generated[101])
    assert links
    # 이웃 AGL: prev_end 1213-200, next_first 1231-200 → 큰 값 1031 이월.
    expected_alt = int(round(ground_m + (float(NEXT_FIRST["altitude"]) - ground_m)))
    for waypoint in links:
        assert waypoint["waypointPassType"] == 3  # FLYOVER
        assert int(waypoint["coordinate"]["altitude"]) == expected_alt
        # 내부 마커는 0303 출력으로 새면 안 된다.
        assert "_flyover_dubins_prefix" not in waypoint
        assert "_area_dubins_link" not in waypoint


def test_initial_plan_out_leg_is_flyover_without_leaked_markers() -> None:
    """d0303 초기계획 링크도 같은 계약: FLYOVER + 마커 무누출."""

    prev_pkt = {
        "wplist": [
            _scan_waypoint(1, PREV_START, [PREV_START]),
            _scan_waypoint(2, PREV_END, [LAST_CAPTURE]),
        ],
        "_mission_cruise_speed_mps": SPEED_MPS,
    }
    next_pkt = {
        "wplist": [
            _scan_waypoint(3, NEXT_FIRST, [NEXT_FIRST]),
            _scan_waypoint(4, NEXT_SECOND, [NEXT_SECOND]),
        ],
    }
    with runtime_override(LINKS_ON):
        appended = d0303._append_area_transition_links_inplace(
            prev_pkt,
            next_pkt,
            altitude_fn=lambda _lat, _lon: 1298,
        )
    assert appended > 0
    links = prev_pkt["wplist"][2:]
    assert len(links) == appended
    for waypoint in links:
        assert waypoint["waypointPassType"] == d0303.PASS_FLYOVER
        assert "_flyover_dubins_prefix" not in waypoint
        assert "_area_dubins_link" not in waypoint


def test_link_runs_before_waypoint_ids_are_assigned() -> None:
    """Link waypoints have to be numbered with the path they belong to."""

    import inspect

    source = inspect.getsource(ncp._prepare_area_replacements)
    link_at = source.find("_append_next_collab_area_transition_links(")
    assign_at = source.find("_assign_replacement_waypoint_ids_in_order(")
    assert link_at != -1 and assign_at != -1
    assert link_at < assign_at


def test_two_pass_area_stays_two_individual_missions_in_order() -> None:
    """A 2-split (forward/reverse) area keeps both passes, OUT before RETURN.

    The link post-pass walks each aircraft's paths in emission order, so the
    turn link is only correct if the OUT pass really precedes the RETURN one.
    """

    rows = [
        {"aircraftID": 5, "areaAssignedCoveragePass": "reverse", "areaComponentIndex": 0, "pieceIndex": 1},
        {"aircraftID": 5, "areaAssignedCoveragePass": "forward", "areaComponentIndex": 0, "pieceIndex": 1},
        {"aircraftID": 4, "areaAssignedCoveragePass": "reverse", "areaComponentIndex": 0, "pieceIndex": 1},
        {"aircraftID": 4, "areaAssignedCoveragePass": "forward", "areaComponentIndex": 0, "pieceIndex": 1},
    ]
    ordered = sorted(
        rows,
        key=lambda row: (
            int(row["aircraftID"]),
            int(ncp._area_assignment_pass_rank(row)),
            int(row["areaComponentIndex"]),
            0,
            int(row["pieceIndex"]),
            0,
            "",
            "",
        ),
    )
    grouped = ncp._group_next_collab_path_rows_by_aircraft(ordered)

    assert sorted(grouped) == [4, 5]
    for aircraft_id, aircraft_rows in grouped.items():
        assert len(aircraft_rows) == 2, aircraft_id
        assert [row["areaAssignedCoveragePass"] for row in aircraft_rows] == [
            "forward",
            "reverse",
        ]


def test_pass_rank_puts_out_before_return() -> None:
    assert ncp._area_assignment_pass_rank({"areaAssignedCoveragePass": "forward"}) < (
        ncp._area_assignment_pass_rank({"areaAssignedCoveragePass": "reverse"})
    )
