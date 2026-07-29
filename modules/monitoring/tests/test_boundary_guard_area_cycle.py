from __future__ import annotations

import unittest
from unittest.mock import patch

from shapely.geometry import LineString, box

from modules.common import mission_area_replan_store
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
    mission_id: int,
    aircraft_id: int,
    set_id: str,
    first_waypoint_id: int,
    last_waypoint_id: int,
) -> _MissionAreaState:
    assignment = box(0.0, 0.0, 100.0, 100.0)
    planned_line = LineString([(0.0, 50.0), (100.0, 50.0)])
    return _MissionAreaState(
        mission_id=mission_id,
        aircraft_id=aircraft_id,
        input_id=700000005,
        mission_type="area",
        source_plan_id=700000001,
        path_id=500000000 + mission_id,
        coverage_def=MissionCoverageDefinition(
            planned_area_m2=float(assignment.area),
            assignment_geometry=assignment,
            transformer=_IdentityTransformer(),
        ),
        width_hint_m=20.0,
        assignment_geometry=assignment,
        planned_area_m2=float(assignment.area),
        covered_geometry=box(0.0, 0.0, 60.0, 100.0),
        planned_cut_lines=[planned_line],
        completed_cut_line_indexes={0},
        done=True,
        boundary_guard_loop=True,
        boundary_guard_loop_version=1,
        boundary_guard_set_id=set_id,
        boundary_guard_sequence=1,
        boundary_guard_sequence_count=1,
        boundary_guard_duration_s=600.0,
        boundary_guard_cycle_first_waypoint_id=first_waypoint_id,
        boundary_guard_cycle_last_waypoint_id=last_waypoint_id,
    )


def _cycle_row(
    *,
    aircraft_id: int,
    set_id: str,
    waypoint_id: int,
    cycle_count: int,
) -> dict[str, object]:
    return {
        "aircraft_id": aircraft_id,
        "current_waypoint_id": waypoint_id,
        "flight_mode": 8,
        "boundary_guard_set_id": set_id,
        "boundary_guard_cycle_count": cycle_count,
        "boundary_guard_loop_active": True,
    }


class BoundaryGuardAreaCycleTests(unittest.TestCase):
    def _monitor(self) -> tuple[
        MissionProgressAreaSnapshotMonitor,
        _MissionAreaState,
        _MissionAreaState,
    ]:
        monitor = MissionProgressAreaSnapshotMonitor(
            snapshot_persist_interval_ms=0
        )
        first = _guard_state(
            mission_id=900000004,
            aircraft_id=4,
            set_id="guard:4",
            first_waypoint_id=100,
            last_waypoint_id=101,
        )
        second = _guard_state(
            mission_id=900000005,
            aircraft_id=5,
            set_id="guard:5",
            first_waypoint_id=200,
            last_waypoint_id=201,
        )
        monitor._states = {
            int(first.mission_id): first,
            int(second.mission_id): second,
        }
        monitor._boundary_guard_cycle_by_set = {
            "guard:4": 0,
            "guard:5": 0,
        }
        monitor._boundary_guard_set_by_waypoint_id = {
            100: "guard:4",
            101: "guard:4",
            200: "guard:5",
            201: "guard:5",
        }
        return monitor, first, second

    def test_asynchronous_wrap_resets_only_that_uav_set(self) -> None:
        monitor, first, second = self._monitor()
        second_before = second.covered_geometry

        changed = monitor._update_boundary_guard_cycle_observations(
            [
                _cycle_row(
                    aircraft_id=4,
                    set_id="guard:4",
                    waypoint_id=100,
                    cycle_count=1,
                )
            ]
        )

        self.assertTrue(changed)
        self.assertTrue(first.covered_geometry.is_empty)
        self.assertFalse(first.done)
        self.assertEqual(first.boundary_guard_cycle_count, 1)
        self.assertTrue(first.boundary_guard_first_cycle_complete)
        self.assertTrue(second.covered_geometry.equals(second_before))
        self.assertEqual(second.boundary_guard_cycle_count, 0)

        monitor._mission_view = {"mission_plan_id": 700000001}
        snapshot = monitor._build_replan_snapshot()
        mission = snapshot["missions"][0]
        self.assertEqual(
            mission["boundaryGuardCycleCoveragePolicy"],
            "current_cycle_per_set",
        )
        cycle_by_set = {
            row["boundaryGuardSetID"]: row["boundaryGuardCycleCount"]
            for row in mission["boundaryGuardSetProgress"]
        }
        self.assertEqual(cycle_by_set, {"guard:4": 1, "guard:5": 0})
        owner_cycles = {
            row["boundaryGuardSetID"]: row["boundaryGuardCycleCount"]
            for row in mission["areaOwnershipDetails"]
        }
        self.assertEqual(owner_cycles, {"guard:4": 1, "guard:5": 0})

    def test_reexecute_baselines_stale_counter_then_next_increment_resets(self) -> None:
        monitor, first, _second = self._monitor()
        with patch.object(
            mission_area_replan_store,
            "reset_central_area_coverage_entry",
            return_value=True,
        ):
            monitor.reset_input_coverage(700000005)

        first.covered_geometry = box(0.0, 0.0, 30.0, 100.0)
        first.completed_cut_line_indexes = {0}
        monitor._update_boundary_guard_cycle_observations(
            [
                _cycle_row(
                    aircraft_id=4,
                    set_id="guard:4",
                    waypoint_id=100,
                    cycle_count=5,
                )
            ]
        )
        self.assertFalse(first.covered_geometry.is_empty)
        self.assertEqual(first.boundary_guard_cycle_count, 0)

        changed = monitor._update_boundary_guard_cycle_observations(
            [
                _cycle_row(
                    aircraft_id=4,
                    set_id="guard:4",
                    waypoint_id=100,
                    cycle_count=6,
                )
            ]
        )
        self.assertTrue(changed)
        self.assertTrue(first.covered_geometry.is_empty)
        self.assertEqual(first.boundary_guard_cycle_count, 1)


class BoundaryGuardCentralLedgerTests(unittest.TestCase):
    @staticmethod
    def _entry(cycle_count: int, remaining_area_m2: float) -> dict[str, object]:
        return {
            "inputMissionID": 700000005,
            "missionType": "area",
            "boundaryGuardLoop": True,
            "boundaryGuardLoopVersion": 1,
            "boundaryGuardSetProgress": [
                {
                    "boundaryGuardSetID": "guard:4",
                    "boundaryGuardCycleCount": cycle_count,
                }
            ],
            "remainingAreaM2": remaining_area_m2,
            "remainingDetail": {
                "coordinateList": [],
                "lineList": [],
                "areaList": [{"coordinateList": [{"latitude": 1, "longitude": 1}]}],
            },
            "areaCoverageDepthContractVersion": 1,
            "coverageDepthPolicy": "spatial_capture_depth",
            "requiredCoverageDepth": 1,
            "coverageDepthDetails": [
                {
                    "coverageDepth": 0,
                    "remainingCaptureCount": 1,
                    "remainingAreaM2": remaining_area_m2,
                    "isDone": False,
                }
            ],
            "coverageObservationDetails": [],
            "coverageDepthSatisfied": False,
        }

    def test_higher_cycle_can_replace_smaller_previous_remaining_area(self) -> None:
        central = self._entry(0, 10.0)
        incoming = self._entry(1, 10_000.0)

        self.assertFalse(
            mission_area_replan_store._central_should_replace_incoming(
                central,
                incoming,
            )
        )
        self.assertTrue(
            mission_area_replan_store._central_should_update(
                central,
                incoming,
            )
        )

    def test_lower_cycle_is_rejected_as_stale(self) -> None:
        central = self._entry(2, 10_000.0)
        incoming = self._entry(1, 10.0)

        self.assertTrue(
            mission_area_replan_store._central_should_replace_incoming(
                central,
                incoming,
            )
        )
        self.assertFalse(
            mission_area_replan_store._central_should_update(
                central,
                incoming,
            )
        )


if __name__ == "__main__":
    unittest.main()
