from __future__ import annotations

import unittest
from unittest.mock import patch

from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _active_tracking_unavailable_by_input_for_plan,
    _assign_targets_to_uav_watchers,
    _remaining_plan_uav_ids,
)


def _plan_data() -> dict:
    return {
        "aircraftList": [
            {"aircraftID": aircraft_id}
            for aircraft_id in (1, 2, 3, 4, 5, 6)
        ]
    }


def _agent_index() -> dict[int, dict]:
    return {
        4: {"coordinate": {"latitude": 38.00, "longitude": 127.00}},
        5: {"coordinate": {"latitude": 38.01, "longitude": 127.01}},
        6: {"coordinate": {"latitude": 38.02, "longitude": 127.02}},
    }


class AttackTrackingAvailabilityTests(unittest.TestCase):
    def test_two_tracking_uavs_leave_only_one_search_uav(self) -> None:
        targets = [
            {
                "target_id": 8,
                "watcher_id": 5,
                "coordinate": {"latitude": 38.01, "longitude": 127.01},
            },
            {
                "target_id": 9,
                "watcher_id": 6,
                "coordinate": {"latitude": 38.02, "longitude": 127.02},
            },
        ]

        with patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "list_active_tracking_assignments",
            return_value=[],
        ):
            assigned, tracking_ids = _assign_targets_to_uav_watchers(
                targets,
                plan_data=_plan_data(),
                agent_index=_agent_index(),
            )

        self.assertEqual(len(assigned), 2)
        self.assertEqual(tracking_ids, {5, 6})
        self.assertEqual(
            _remaining_plan_uav_ids(_plan_data(), set(tracking_ids)),
            [4],
        )

    def test_existing_tracker_omitted_from_followup_batch_stays_unavailable(self) -> None:
        active_assignment = {
            "aircraft_id": 5,
            "attack_plan_id": 700000010,
            "current_input_mission_id": 70000008,
            "target_id": 8,
        }
        new_target = {
            "target_id": 9,
            "watcher_id": 6,
            "coordinate": {"latitude": 38.02, "longitude": 127.02},
        }

        with patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "list_active_tracking_assignments",
            return_value=[active_assignment],
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "resolve_plan_lineage_ids",
            return_value={700000010},
        ):
            assigned, newly_tracking_ids = _assign_targets_to_uav_watchers(
                [new_target],
                plan_data=_plan_data(),
                agent_index=_agent_index(),
            )
            unavailable_by_input, active_tracking_ids = (
                _active_tracking_unavailable_by_input_for_plan(
                    source_plan_id=700000010,
                    plan_data=_plan_data(),
                )
            )

        self.assertEqual([row["watcher_id"] for row in assigned], [6])
        self.assertEqual(newly_tracking_ids, {6})
        self.assertEqual(unavailable_by_input, {70000008: {5}})
        self.assertEqual(active_tracking_ids, {5})
        self.assertEqual(
            _remaining_plan_uav_ids(
                _plan_data(),
                set(newly_tracking_ids) | set(active_tracking_ids),
            ),
            [4],
        )

    def test_active_assignment_from_unrelated_plan_is_not_excluded(self) -> None:
        unrelated_assignment = {
            "aircraft_id": 5,
            "attack_plan_id": 700000001,
            "current_input_mission_id": 70000008,
            "target_id": 8,
        }

        with patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "list_active_tracking_assignments",
            return_value=[unrelated_assignment],
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "resolve_plan_lineage_ids",
            return_value={700000010, 700000009},
        ):
            unavailable_by_input, active_tracking_ids = (
                _active_tracking_unavailable_by_input_for_plan(
                    source_plan_id=700000010,
                    plan_data=_plan_data(),
                )
            )

        self.assertEqual(unavailable_by_input, {})
        self.assertEqual(active_tracking_ids, set())


if __name__ == "__main__":
    unittest.main()
