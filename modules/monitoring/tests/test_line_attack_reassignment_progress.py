from __future__ import annotations

from unittest.mock import patch

import pytest

from modules.monitoring.logic.line_scan_progress_monitor import LineScanProgressMonitor


def _line_mission(
    *,
    mission_id: int,
    input_id: int,
    path_id: int,
    latitude: float,
    reverse: bool = False,
    source_coords: list[dict[str, float]] | None = None,
) -> dict[str, object]:
    coords = [
        {"latitude": float(latitude), "longitude": 127.0, "altitude": 100},
        {"latitude": float(latitude), "longitude": 127.012, "altitude": 100},
    ]
    if reverse:
        coords.reverse()
    return {
        "individual_mission_id": int(mission_id),
        "input_id": int(input_id),
        "path_id": int(path_id),
        "is_done": False,
        "line_list": [{"width": 300.0, "coordinateList": coords}],
        "input_line_list": [{"width": 900.0, "coordinateList": coords}],
        "input_coordinate_list": list(source_coords or coords),
        "waypoints": [],
    }


def _view(plan_id: int, rows: list[tuple[int, dict[str, object]]]) -> dict[str, object]:
    return {
        "mission_plan_id": int(plan_id),
        "uav_entries": [
            {
                "aircraft_id": int(aircraft_id),
                "current_individual_mission_id": int(mission["individual_mission_id"]),
                "missions": [mission],
            }
            for aircraft_id, mission in rows
        ],
    }


def test_attack_reassignment_retires_departed_owner_and_carries_common_capture() -> None:
    plan1 = _view(
        1,
        [
            (4, _line_mission(mission_id=41, input_id=10, path_id=401, latitude=38.000)),
            (5, _line_mission(mission_id=51, input_id=10, path_id=501, latitude=38.003)),
            (6, _line_mission(mission_id=61, input_id=10, path_id=601, latitude=38.006)),
        ],
    )
    plan2 = _view(
        2,
        [
            (4, _line_mission(mission_id=42, input_id=10, path_id=402, latitude=38.0015)),
            # Reassignment may reverse one generated centerline. Carry must be
            # normalized to the source direction before it is seeded.
            (5, _line_mission(mission_id=52, input_id=10, path_id=502, latitude=38.0045, reverse=True)),
        ],
    )
    views = {1: plan1, 2: plan2}

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        side_effect=lambda plan_id: views[int(plan_id)],
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)

        # All three owners photographed a common prefix, while UAV4/5 each
        # have a little extra. Only the common part is safe to transfer.
        ratios = {4: 0.45, 5: 0.40, 6: 0.30}
        for runtime in monitor._missions.values():
            length_m = float(runtime.line_def.line_lengths_m[0])
            covered_m = length_m * ratios[int(runtime.aircraft_id)]
            runtime.state.covered_intervals_by_line = {0: [(0.0, covered_m)]}
            runtime.state.covered_length_m = covered_m

        monitor.apply_mission_plan(2)

        assert {runtime.aircraft_id for runtime in monitor._missions.values()} == {4, 5}
        for runtime in monitor._missions.values():
            ratio = runtime.state.covered_length_m / runtime.line_def.planned_length_m
            assert ratio == pytest.approx(0.30, abs=0.01)
            assert runtime.carry_source_aircraft_ids == (4, 5, 6)

        payload = monitor.snapshot()
        assert payload["missionPlanID"] == 2
        assert {entry["aircraftID"] for entry in payload["entries"]} == {4, 5}
        assert all(
            entry["coverageCarryPolicy"] == "previous_active_owner_intersection"
            for entry in payload["entries"]
        )


def test_explicit_line_reexecution_clears_carried_progress() -> None:
    view = _view(
        1,
        [(4, _line_mission(mission_id=41, input_id=10, path_id=401, latitude=38.0))],
    )
    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        return_value=view,
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)
        runtime = next(iter(monitor._missions.values()))
        length_m = float(runtime.line_def.line_lengths_m[0])
        runtime.state.covered_intervals_by_line = {0: [(0.0, length_m * 0.5)]}
        runtime.state.covered_length_m = length_m * 0.5

        assert monitor.reset_input_coverage(10) == 1
        assert runtime.state.covered_length_m == 0.0
        assert runtime.state.covered_intervals_by_line == {}


def test_attack_reassignment_does_not_double_count_prefix_already_trimmed_from_new_line() -> None:
    full_source = [
        {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
        {"latitude": 38.0, "longitude": 127.012, "altitude": 100.0},
    ]
    trimmed_rows = []
    for mission_id, path_id, latitude in ((42, 402, 38.0015), (52, 502, 38.0045)):
        mission = _line_mission(
            mission_id=mission_id,
            input_id=10,
            path_id=path_id,
            latitude=latitude,
            source_coords=full_source,
        )
        mission["line_list"][0]["coordinateList"] = [
            {"latitude": float(latitude), "longitude": 127.0048, "altitude": 100.0},
            {"latitude": float(latitude), "longitude": 127.012, "altitude": 100.0},
        ]
        mission["input_line_list"] = [
            {
                "width": 900.0,
                "coordinateList": list(full_source),
            }
        ]
        trimmed_rows.append((4 if mission_id == 42 else 5, mission))

    plan1 = _view(
        1,
        [
            (
                4,
                _line_mission(
                    mission_id=41,
                    input_id=10,
                    path_id=401,
                    latitude=38.000,
                    source_coords=full_source,
                ),
            ),
            (
                5,
                _line_mission(
                    mission_id=51,
                    input_id=10,
                    path_id=501,
                    latitude=38.003,
                    source_coords=full_source,
                ),
            ),
            (
                6,
                _line_mission(
                    mission_id=61,
                    input_id=10,
                    path_id=601,
                    latitude=38.006,
                    source_coords=full_source,
                ),
            ),
        ],
    )
    plan2 = _view(2, trimmed_rows)
    views = {1: plan1, 2: plan2}

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        side_effect=lambda plan_id: views[int(plan_id)],
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)

        for runtime in monitor._missions.values():
            length_m = float(runtime.line_def.line_lengths_m[0])
            covered_m = length_m * 0.30
            runtime.state.covered_intervals_by_line = {0: [(0.0, covered_m)]}
            runtime.state.covered_length_m = covered_m

        monitor.apply_mission_plan(2)

        # The replacement geometry already starts after 40% of the original
        # source line. The old 0-30% prefix does not overlap it, so seeding 30%
        # again would skip untouched work when the third UAV rejoins.
        for runtime in monitor._missions.values():
            assert runtime.state.covered_length_m == pytest.approx(0.0, abs=0.01)
            assert runtime.state.covered_intervals_by_line == {}


def test_retired_attack_owner_is_excluded_from_next_plan_common_frontier() -> None:
    plan1 = _view(
        1,
        [
            (4, _line_mission(mission_id=41, input_id=10, path_id=401, latitude=38.000)),
            (5, _line_mission(mission_id=51, input_id=10, path_id=501, latitude=38.003)),
            (6, _line_mission(mission_id=61, input_id=10, path_id=601, latitude=38.006)),
        ],
    )
    plan2 = _view(
        2,
        [
            (4, _line_mission(mission_id=42, input_id=10, path_id=402, latitude=38.001)),
            (5, _line_mission(mission_id=52, input_id=10, path_id=502, latitude=38.004)),
        ],
    )
    views = {1: plan1, 2: plan2}

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        side_effect=lambda plan_id: views[int(plan_id)],
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)

        ratios = {4: 0.40, 5: 0.30, 6: 0.10}
        for runtime in monitor._missions.values():
            length_m = float(runtime.line_def.line_lengths_m[0])
            covered_m = length_m * ratios[int(runtime.aircraft_id)]
            runtime.state.covered_intervals_by_line = {0: [(0.0, covered_m)]}
            runtime.state.covered_length_m = covered_m
            if int(runtime.aircraft_id) == 6:
                runtime.is_current = False

        monitor.apply_mission_plan(2)

        for runtime in monitor._missions.values():
            ratio = runtime.state.covered_length_m / runtime.line_def.planned_length_m
            assert ratio == pytest.approx(0.30, abs=0.01)
            assert runtime.carry_source_aircraft_ids == (4, 5)


def test_explicit_non_line_waypoint_retires_stale_line_runtime() -> None:
    mission = _line_mission(
        mission_id=41,
        input_id=10,
        path_id=401,
        latitude=38.0,
    )
    mission["waypoint_ids"] = [4101, 4102]
    view = _view(1, [(4, mission)])

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        return_value=view,
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)
        runtime = next(iter(monitor._missions.values()))

        monitor.update_agent_status(
            timestamp_ms=1000,
            agent_states=[
                {
                    "aircraft_id": 4,
                    "current_waypoint_id": 9999,
                    "flying": 1,
                    "filming": 0,
                }
            ],
        )

        assert monitor._aircraft_current_mission[4] is None
        assert runtime.is_current is False
        assert runtime.state.covered_length_m == pytest.approx(0.0)
        assert monitor.snapshot()["entries"][0]["isCurrent"] is False
