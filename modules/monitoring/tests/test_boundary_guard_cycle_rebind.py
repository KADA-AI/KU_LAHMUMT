from __future__ import annotations

import unittest
from unittest.mock import patch

from shapely.geometry import LineString, box

from modules.monitoring.logic.mission_area_progress_monitor import (
    MissionProgressAreaSnapshotMonitor,
    _MissionAreaState,
)
from modules.monitoring.logic.mission_coverage import MissionCoverageDefinition


class _IdentityTransformer:
    @staticmethod
    def transform(
        x: float,
        y: float,
        *_args: object,
        **_kwargs: object,
    ) -> tuple[float, float]:
        return float(x), float(y)


def _guard_state(
    *,
    mission_id: int = 900000004,
    path_id: int = 500000004,
) -> _MissionAreaState:
    assignment = box(0.0, 0.0, 100.0, 100.0)
    return _MissionAreaState(
        mission_id=mission_id,
        aircraft_id=4,
        input_id=700000005,
        mission_type="area",
        source_plan_id=700000001,
        path_id=path_id,
        coverage_def=MissionCoverageDefinition(
            planned_area_m2=float(assignment.area),
            assignment_geometry=assignment,
            transformer=_IdentityTransformer(),
        ),
        width_hint_m=20.0,
        assignment_geometry=assignment,
        planned_area_m2=float(assignment.area),
        covered_geometry=box(0.0, 0.0, 30.0, 100.0),
        planned_cut_lines=[LineString([(0.0, 50.0), (100.0, 50.0)])],
        done=True,
        boundary_guard_loop=True,
        boundary_guard_loop_version=1,
        boundary_guard_set_id="guard:4",
        boundary_guard_sequence=1,
        boundary_guard_sequence_count=1,
        boundary_guard_duration_s=600.0,
        boundary_guard_cycle_first_waypoint_id=100,
        boundary_guard_cycle_last_waypoint_id=101,
        boundary_guard_cycle_count=1,
        boundary_guard_first_cycle_complete=True,
    )


def _cycle_row(cycle_count: int) -> dict[str, object]:
    return {
        "aircraft_id": 4,
        "current_waypoint_id": 100,
        "flight_mode": 8,
        "boundary_guard_set_id": "guard:4",
        "boundary_guard_cycle_count": cycle_count,
        "boundary_guard_loop_active": True,
    }


class BoundaryGuardCycleRebindTests(unittest.TestCase):
    def test_raw_zero_then_one_after_rebind_advances_logical_one_to_two(self) -> None:
        monitor = MissionProgressAreaSnapshotMonitor(
            snapshot_persist_interval_ms=0
        )
        state = _guard_state()
        state.done = False
        monitor._states = {int(state.mission_id): state}
        monitor._boundary_guard_cycle_by_set = {"guard:4": 1}
        monitor._boundary_guard_rearm_pending_set_ids = {"guard:4"}

        changed_at_zero = monitor._update_boundary_guard_cycle_observations(
            [_cycle_row(0)]
        )

        self.assertFalse(changed_at_zero)
        self.assertEqual(
            monitor._boundary_guard_published_baseline_by_set["guard:4"],
            -1,
        )
        self.assertFalse(state.covered_geometry.is_empty)
        self.assertEqual(state.boundary_guard_cycle_count, 1)

        changed_at_one = monitor._update_boundary_guard_cycle_observations(
            [_cycle_row(1)]
        )

        self.assertTrue(changed_at_one)
        self.assertTrue(state.covered_geometry.is_empty)
        self.assertFalse(state.done)
        self.assertEqual(state.boundary_guard_cycle_count, 2)
        self.assertEqual(monitor._boundary_guard_cycle_by_set["guard:4"], 2)

    def test_counter_regression_after_stale_old_sample_rebases_new_generation(
        self,
    ) -> None:
        monitor = MissionProgressAreaSnapshotMonitor(
            snapshot_persist_interval_ms=0
        )
        state = _guard_state()
        state.done = False
        monitor._states = {int(state.mission_id): state}
        monitor._boundary_guard_cycle_by_set = {"guard:4": 1}
        monitor._boundary_guard_rearm_pending_set_ids = {"guard:4"}

        monitor._update_boundary_guard_cycle_observations([_cycle_row(1)])
        monitor._update_boundary_guard_cycle_observations([_cycle_row(0)])
        changed = monitor._update_boundary_guard_cycle_observations(
            [_cycle_row(1)]
        )

        self.assertTrue(changed)
        self.assertTrue(state.covered_geometry.is_empty)
        self.assertEqual(state.boundary_guard_cycle_count, 2)

    def test_plan_reload_treats_stable_guard_done_as_prior_cycle_history(
        self,
    ) -> None:
        monitor = MissionProgressAreaSnapshotMonitor(
            snapshot_persist_interval_ms=0
        )
        previous = _guard_state()
        monitor._states = {int(previous.mission_id): previous}
        monitor._mission_view = {"mission_plan_id": 700000001}
        monitor._boundary_guard_cycle_by_set = {"guard:4": 1}

        new_mission_id = 900000104
        new_view = {
            "mission_plan_id": 700000002,
            "input_missions": [
                {
                    "input_mission_id": 700000005,
                    "input_mission_type": 3,
                    "is_done": False,
                }
            ],
            "uav_entries": [
                {
                    "aircraft_id": 4,
                    "current_individual_mission_id": new_mission_id,
                    "missions": [
                        {
                            "individual_mission_id": new_mission_id,
                            "input_id": 700000005,
                            "path_id": 500000104,
                            "is_done": True,
                            "sweep_point_count": 2,
                            "area_list": [{"coordinateList": [{}, {}, {}]}],
                            "boundary_guard_loop": True,
                            "boundary_guard_loop_version": 1,
                            "boundary_guard_set_id": "guard:4",
                            "boundary_guard_sequence": 1,
                            "boundary_guard_sequence_count": 1,
                            "boundary_guard_duration_s": 600.0,
                            "boundary_guard_cycle_first_waypoint_id": 200,
                            "boundary_guard_cycle_last_waypoint_id": 201,
                            "waypoints": [
                                {
                                    "waypoint_id": 200,
                                    "line_search_point_count": 1,
                                },
                                {
                                    "waypoint_id": 201,
                                    "line_search_point_count": 1,
                                },
                            ],
                        }
                    ],
                }
            ],
        }
        planned_line = LineString([(0.0, 50.0), (100.0, 50.0)])

        with (
            patch(
                "modules.monitoring.logic.mission_area_progress_monitor."
                "build_uav_mission_view",
                return_value=new_view,
            ),
            patch(
                "modules.monitoring.logic.mission_area_progress_monitor."
                "build_mission_coverage_definition",
                return_value=previous.coverage_def,
            ),
            patch(
                "modules.monitoring.logic.mission_area_progress_monitor."
                "_build_planned_sweep_lines",
                return_value=([planned_line], 10.0),
            ),
            patch.object(
                monitor,
                "_persist_replan_snapshot",
                return_value=None,
            ),
            patch.object(monitor, "_request_refresh", return_value=None),
        ):
            monitor._load_mission_plan(700000002)

        current = monitor._states[new_mission_id]
        self.assertFalse(current.done)
        self.assertAlmostEqual(float(current.covered_geometry.area), 3000.0)
        self.assertIn(
            "guard:4",
            monitor._boundary_guard_rearm_pending_set_ids,
        )


if __name__ == "__main__":
    unittest.main()
