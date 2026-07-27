# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest
from types import SimpleNamespace

from shapely.geometry import box

from modules.monitoring.logic.capture_gate import evaluate_capture_gate
from modules.monitoring.logic.mission_coverage import MissionCoverageDefinition
from modules.monitoring.logic.mission_area_progress_monitor import (
    _area_capture_progress_allowed,
    _area_sensor_offset_bypass_allowed,
)
from modules.monitoring.logic.mission_progress import (
    MissionMeta,
    MissionProgressTracker,
    _coverage_sensor_offset_bypass_allowed,
)


class _IdentityTransformer:
    @staticmethod
    def transform(longitude: float, latitude: float) -> tuple[float, float]:
        return float(longitude) * 100_000.0, float(latitude) * 100_000.0


def _coordinate(latitude: float, longitude: float) -> dict[str, float]:
    return {"latitude": float(latitude), "longitude": float(longitude)}


def _agent_state(*, filming: object = 1, include_filming: bool = True) -> dict[str, object]:
    state: dict[str, object] = {
        "flying": 1,
        "sensor_operation_mode": 2,
        "coordinate": _coordinate(37.0, 127.0),
        # About 2.2 km east: beyond the default 1.5 km safety limit.
        "sensor_center_coordinate": _coordinate(37.0, 127.025),
    }
    if include_filming:
        state["filming"] = filming
    return state


class CaptureGateAreaOffsetTests(unittest.TestCase):
    def test_explicit_filming_requires_active_state_one(self) -> None:
        active = _agent_state(filming=1)
        active["sensor_center_coordinate"] = active["coordinate"]
        self.assertTrue(evaluate_capture_gate(active).allowed)

        off = dict(active, filming=0)
        self.assertEqual(evaluate_capture_gate(off).reason, "filming_off")

        finished = dict(active, filming=2)
        decision = evaluate_capture_gate(finished)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "filming_not_active")

    def test_missing_filming_keeps_legacy_sensor_mode_rules(self) -> None:
        legacy = _agent_state(include_filming=False)
        legacy["sensor_center_coordinate"] = legacy["coordinate"]
        self.assertTrue(evaluate_capture_gate(legacy).allowed)

        legacy_mode_zero = dict(legacy, sensor_operation_mode=0)
        self.assertTrue(evaluate_capture_gate(legacy_mode_zero).allowed)

        non_sweep = dict(legacy, sensor_operation_mode=4)
        self.assertFalse(evaluate_capture_gate(non_sweep).allowed)
        self.assertEqual(evaluate_capture_gate(non_sweep).reason, "sensor_mode_4")

    def test_far_sensor_offset_is_blocked_unless_caller_safely_opts_in(self) -> None:
        state = _agent_state(filming=1)
        self.assertEqual(evaluate_capture_gate(state).reason, "sensor_far_stare")
        self.assertTrue(
            evaluate_capture_gate(
                state,
                allow_sensor_offset_bypass=True,
            ).allowed
        )

    def test_area_offset_bypass_requires_same_pass_sweep_and_real_overlap(self) -> None:
        assignment = box(0.0, 0.0, 100.0, 100.0)
        forward = SimpleNamespace(
            mission_type="area",
            sweep_waypoint_ids={101, 102},
            assignment_geometry=assignment,
        )
        return_pass = SimpleNamespace(
            mission_type="area",
            sweep_waypoint_ids={201, 202},
            assignment_geometry=assignment,
        )
        inside_footprint = box(25.0, 25.0, 75.0, 75.0)
        outside_footprint = box(150.0, 150.0, 200.0, 200.0)

        self.assertTrue(
            _area_sensor_offset_bypass_allowed(
                forward,
                current_waypoint_id=101,
                footprint_geometry=inside_footprint,
            )
        )
        # A turn/re-entry waypoint is not a sweep waypoint.
        self.assertFalse(
            _area_sensor_offset_bypass_allowed(
                forward,
                current_waypoint_id=150,
                footprint_geometry=inside_footprint,
            )
        )
        # A waypoint owned by the other pass cannot unlock this pass.
        self.assertFalse(
            _area_sensor_offset_bypass_allowed(
                return_pass,
                current_waypoint_id=101,
                footprint_geometry=inside_footprint,
            )
        )
        self.assertFalse(
            _area_sensor_offset_bypass_allowed(
                forward,
                current_waypoint_id=101,
                footprint_geometry=outside_footprint,
            )
        )

        far_state = _agent_state(filming=1)
        safe_bypass = _area_sensor_offset_bypass_allowed(
            forward,
            current_waypoint_id=101,
            footprint_geometry=inside_footprint,
        )
        self.assertTrue(
            evaluate_capture_gate(
                far_state,
                allow_sensor_offset_bypass=safe_bypass,
            ).allowed
        )
        unsafe_bypass = _area_sensor_offset_bypass_allowed(
            forward,
            current_waypoint_id=150,
            footprint_geometry=inside_footprint,
        )
        self.assertEqual(
            evaluate_capture_gate(
                far_state,
                allow_sensor_offset_bypass=unsafe_bypass,
            ).reason,
            "sensor_far_stare",
        )

    def test_area_progress_rejects_finished_filming_even_on_sweep_waypoint(self) -> None:
        state = SimpleNamespace(mission_type="area", sweep_waypoint_ids={101})
        self.assertFalse(
            _area_capture_progress_allowed(
                state,
                current_waypoint_id=101,
                filming_value=2,
                sensor_operation_mode=2,
            )
        )
        self.assertTrue(
            _area_capture_progress_allowed(
                state,
                current_waypoint_id=101,
                filming_value=1,
                sensor_operation_mode=2,
            )
        )

    def test_actual_footprint_tracker_bypasses_offset_only_on_planned_sweep_overlap(self) -> None:
        assignment = box(1900.0, -100.0, 2100.0, 100.0)
        definition = MissionCoverageDefinition(
            planned_area_m2=float(assignment.area),
            assignment_geometry=assignment,
            transformer=_IdentityTransformer(),
        )
        meta = MissionMeta(
            mission_id=101,
            aircraft_id=4,
            input_id=4,
            package_id=None,
            path_id=4001,
            planned_seconds=10.0,
            waypoint_ids=[101, 150, 201],
            waypoint_eta_cumulative={101: 0.0, 150: 5.0, 201: 10.0},
            waypoint_index={101: 0, 150: 1, 201: 2},
            waypoint_sweep_coords={
                101: [
                    {"latitude": 0.0, "longitude": 0.019},
                    {"latitude": 0.0, "longitude": 0.021},
                ],
                201: [
                    {"latitude": 0.0, "longitude": 0.021},
                    {"latitude": 0.0, "longitude": 0.019},
                ],
            },
            coverage_pass_by_waypoint_id={101: "forward", 201: "reverse"},
            coverage_pass_order=("forward", "reverse"),
        )
        inside_footprint = box(1950.0, -50.0, 2050.0, 50.0)
        outside_footprint = box(3000.0, -50.0, 3100.0, 50.0)
        self.assertTrue(
            _coverage_sensor_offset_bypass_allowed(
                definition,
                meta,
                current_waypoint_id=101,
                footprint_geometry=inside_footprint,
            )
        )
        self.assertFalse(
            _coverage_sensor_offset_bypass_allowed(
                definition,
                meta,
                current_waypoint_id=150,
                footprint_geometry=inside_footprint,
            )
        )
        self.assertFalse(
            _coverage_sensor_offset_bypass_allowed(
                definition,
                meta,
                current_waypoint_id=101,
                footprint_geometry=outside_footprint,
            )
        )

        tracker = MissionProgressTracker()
        tracker._mission_coverage_defs = {101: definition}
        tracker._mission_meta = {101: meta}
        far_active_state = {
            "current_waypoint_id": 101,
            "flying": 1,
            "filming": 1,
            "sensor_operation_mode": 2,
            "coordinate": _coordinate(0.0, 0.0),
            "sensor_center_coordinate": _coordinate(0.0, 0.02),
            "footprint_corners": [
                {"latitude": 0.0005, "longitude": 0.0195},
                {"latitude": 0.0005, "longitude": 0.0205},
                {"latitude": -0.0005, "longitude": 0.0205},
                {"latitude": -0.0005, "longitude": 0.0195},
            ],
        }
        tracker._update_mission_coverage(101, far_active_state, timestamp_ms=10_000)
        covered = tracker._mission_coverage_state[101].covered_area_m2_by_pass
        self.assertGreater(covered.get("forward", 0.0), 0.0)

        before = float(covered["forward"])
        tracker._update_mission_coverage(
            101,
            dict(far_active_state, filming=2),
            timestamp_ms=10_200,
        )
        self.assertEqual(
            tracker._mission_coverage_state[101].covered_area_m2_by_pass["forward"],
            before,
        )


if __name__ == "__main__":
    unittest.main()
