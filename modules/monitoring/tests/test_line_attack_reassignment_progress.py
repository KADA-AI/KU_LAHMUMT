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
    independent_line_progress: bool = False,
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
        "independent_line_progress": bool(independent_line_progress),
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


def test_noncurrent_pending_line_does_not_receive_common_coverage() -> None:
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
    # UAV4 is still tracking an attack; path 402 is only its pending resume
    # suffix. UAV5 remains the live LINE owner.
    plan2["uav_entries"][0]["current_individual_mission_id"] = 999
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

        by_aircraft = {
            int(runtime.aircraft_id): runtime
            for runtime in monitor._missions.values()
        }
        assert by_aircraft[4].is_current is False
        assert by_aircraft[4].state.covered_length_m == pytest.approx(0.0)
        assert by_aircraft[4].carry_source_aircraft_ids == ()
        assert (
            by_aircraft[5].state.covered_length_m
            / by_aircraft[5].line_def.planned_length_m
        ) == pytest.approx(0.30, abs=0.01)


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


def test_type2_independent_branch_new_suffix_does_not_inherit_input_progress() -> None:
    plan1 = _view(
        1,
        [
            (
                aircraft_id,
                _line_mission(
                    mission_id=(aircraft_id * 10) + 1,
                    input_id=4,
                    path_id=(aircraft_id * 100) + 1,
                    latitude=38.0 + (aircraft_id * 0.003),
                    independent_line_progress=True,
                ),
            )
            for aircraft_id in (4, 5, 6)
        ],
    )
    plan2 = _view(
        2,
        [
            (
                aircraft_id,
                _line_mission(
                    mission_id=(aircraft_id * 10) + 2,
                    input_id=4,
                    path_id=(aircraft_id * 100) + 2,
                    latitude=38.001 + (aircraft_id * 0.003),
                    independent_line_progress=True,
                ),
            )
            for aircraft_id in (4, 5, 6)
        ],
    )
    views = {1: plan1, 2: plan2}

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        side_effect=lambda plan_id: views[int(plan_id)],
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)
        for runtime in monitor._missions.values():
            length_m = float(runtime.line_def.line_lengths_m[0])
            covered_m = length_m * 0.60
            runtime.state.covered_intervals_by_line = {0: [(0.0, covered_m)]}
            runtime.state.covered_length_m = covered_m

        monitor.apply_mission_plan(2)

        for runtime in monitor._missions.values():
            assert runtime.independent_line_progress is True
            assert runtime.state.covered_length_m == pytest.approx(0.0)
            assert runtime.state.covered_intervals_by_line == {}
            assert runtime.carry_source_aircraft_ids == ()
        assert all(
            "coverageCarryPolicy" not in entry
            for entry in monitor.snapshot()["entries"]
        )


def test_type2_independent_branch_preserves_only_same_aircraft_exact_path() -> None:
    path_id = 401
    first = _line_mission(
        mission_id=41,
        input_id=4,
        path_id=path_id,
        latitude=38.0,
        independent_line_progress=True,
    )
    pending = _line_mission(
        mission_id=42,
        input_id=4,
        path_id=path_id,
        latitude=38.0,
        independent_line_progress=True,
    )
    resumed = _line_mission(
        mission_id=43,
        input_id=4,
        path_id=path_id,
        latitude=38.0,
        independent_line_progress=True,
    )
    plan1 = _view(1, [(4, first)])
    plan2 = _view(2, [(4, pending)])
    plan2["uav_entries"][0]["current_individual_mission_id"] = 999
    plan3 = _view(3, [(4, resumed)])
    views = {1: plan1, 2: plan2, 3: plan3}

    with patch(
        "modules.monitoring.logic.line_scan_progress_monitor.build_uav_mission_view",
        side_effect=lambda plan_id: views[int(plan_id)],
    ), patch.object(LineScanProgressMonitor, "_persist", return_value=None):
        monitor = LineScanProgressMonitor()
        monitor.apply_mission_plan(1)
        runtime = next(iter(monitor._missions.values()))
        length_m = float(runtime.line_def.line_lengths_m[0])
        covered_m = length_m * 0.35
        runtime.state.covered_intervals_by_line = {0: [(0.0, covered_m)]}
        runtime.state.covered_length_m = covered_m
        runtime.visited_line_indexes.add(0)
        runtime.line_visit_sequence.append(0)
        runtime.line_transition_count = 3

        monitor.apply_mission_plan(2)
        pending_runtime = next(iter(monitor._missions.values()))
        assert pending_runtime.is_current is False
        assert pending_runtime.state.covered_length_m == pytest.approx(covered_m)

        monitor.apply_mission_plan(3)
        resumed_runtime = next(iter(monitor._missions.values()))
        assert resumed_runtime.is_current is True
        assert resumed_runtime.state.covered_length_m == pytest.approx(covered_m)
        assert resumed_runtime.line_transition_count == 3
        assert resumed_runtime.carry_source_aircraft_ids == ()
        assert monitor.snapshot()["entries"][0]["coverageCarryPolicy"] == (
            "same_aircraft_exact_path"
        )
