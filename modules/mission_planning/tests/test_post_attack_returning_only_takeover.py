from __future__ import annotations

import unittest
from unittest.mock import patch

from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _allow_post_attack_active_only_replan,
    _evaluate_rejoin_group,
)


_PLAN_ID = 700000015
_INPUT_ID = 70000003


def _tracking_assignment(aircraft_id: int) -> dict[str, object]:
    return {
        "active": True,
        "attack_plan_id": _PLAN_ID,
        "current_input_mission_id": _INPUT_ID,
        "aircraft_id": int(aircraft_id),
    }


class PostAttackReturningOnlyTakeoverTests(unittest.TestCase):
    def test_partition_preservation_cannot_fall_through_to_active_only_replan(self) -> None:
        self.assertFalse(
            _allow_post_attack_active_only_replan(
                "ongoing_tracker_partition_preserved"
            )
        )

    def _evaluate(
        self,
        *,
        snapshot_completed: bool,
        has_remaining_geometry: bool,
        tracking_aircraft_ids: tuple[int, ...] = (5, 6),
    ) -> tuple[dict[str, object], list[str]]:
        messages: list[str] = []
        with (
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_aircraft_ids_for_input_mission",
                return_value={4, 5, 6},
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "list_active_tracking_assignments",
                return_value=[
                    _tracking_assignment(aircraft_id)
                    for aircraft_id in tracking_aircraft_ids
                ],
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_summarize_active_group_progress",
                return_value={
                    "active_progress_by_aircraft": {},
                    "active_progress_aircraft_ids": [],
                    "active_progress_sample_count": 0,
                    "active_avg_progress_percent": None,
                },
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_remaining_snapshot_explicitly_completed",
                return_value=bool(snapshot_completed),
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_has_remaining_snapshot_geometry",
                return_value=bool(has_remaining_geometry),
            ),
        ):
            result = _evaluate_rejoin_group(
                current_plan_id=_PLAN_ID,
                current_input_id=_INPUT_ID,
                group_assignments=[
                    {
                        "aircraft_id": 4,
                        "current_input_mission_id": _INPUT_ID,
                    }
                ],
                agent_state_map={},
                config={},
                emit=messages.append,
            )
        return result, messages

    def test_returning_uav_keeps_its_partition_while_other_uavs_are_tracking(self) -> None:
        result, messages = self._evaluate(
            snapshot_completed=False,
            has_remaining_geometry=True,
        )

        self.assertFalse(result["replan_needed"])
        self.assertEqual(result["skip_reason"], "ongoing_tracker_partition_preserved")
        self.assertNotIn("returning_only_takeover", result)
        self.assertEqual(result["remaining_snapshot_state"], "partition_preserved")
        self.assertEqual(result["available_aircraft_ids"], [4])
        self.assertEqual(result["active_aircraft_ids"], [])
        self.assertEqual(result["returning_aircraft_ids"], [4])
        self.assertEqual(result["ongoing_tracking_aircraft_ids"], [5, 6])
        self.assertTrue(any("AREA-OWNERSHIP" in message for message in messages))

    def test_active_uav_is_not_given_tracker_owned_aggregate_area(self) -> None:
        result, messages = self._evaluate(
            snapshot_completed=False,
            has_remaining_geometry=True,
            tracking_aircraft_ids=(6,),
        )

        self.assertFalse(result["replan_needed"])
        self.assertEqual(result["skip_reason"], "ongoing_tracker_partition_preserved")
        self.assertEqual(result["active_aircraft_ids"], [5])
        self.assertEqual(result["returning_aircraft_ids"], [4])
        self.assertEqual(result["ongoing_tracking_aircraft_ids"], [6])
        self.assertTrue(any("AREA-OWNERSHIP" in message for message in messages))

    def test_last_tracker_release_can_resume_normal_remaining_area_replan(self) -> None:
        with (
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_select_rejoin_reference_coordinate",
                return_value={
                    "latitude": 38.0,
                    "longitude": 127.0,
                    "altitude": 500.0,
                },
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_estimate_group_remaining_eta_s",
                return_value=600,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_estimate_turn_aware_eta_s",
                return_value=5,
            ),
        ):
            result, messages = self._evaluate(
                snapshot_completed=False,
                has_remaining_geometry=True,
                tracking_aircraft_ids=(),
            )

        self.assertTrue(result["replan_needed"])
        self.assertIsNone(result["skip_reason"])
        self.assertEqual(result["ongoing_tracking_aircraft_ids"], [])
        self.assertEqual(result["active_aircraft_ids"], [5, 6])
        self.assertFalse(any("AREA-OWNERSHIP" in message for message in messages))

    def test_explicitly_completed_snapshot_skips_returning_only_takeover(self) -> None:
        result, messages = self._evaluate(
            snapshot_completed=True,
            has_remaining_geometry=True,
        )

        self.assertFalse(result["replan_needed"])
        self.assertEqual(result["skip_reason"], "remaining_snapshot_completed")
        self.assertEqual(result["remaining_snapshot_state"], "completed")
        self.assertNotIn("returning_only_takeover", result)
        self.assertEqual(result["active_aircraft_ids"], [])
        self.assertEqual(result["returning_aircraft_ids"], [4])
        self.assertEqual(result["ongoing_tracking_aircraft_ids"], [5, 6])
        self.assertTrue(any("explicitly reports mission completion" in message for message in messages))

    def test_missing_remaining_geometry_stays_unavailable(self) -> None:
        result, messages = self._evaluate(
            snapshot_completed=False,
            has_remaining_geometry=False,
        )

        self.assertFalse(result["replan_needed"])
        self.assertEqual(result["skip_reason"], "remaining_snapshot_unavailable")
        self.assertEqual(result["remaining_snapshot_state"], "unavailable")
        self.assertNotIn("returning_only_takeover", result)
        self.assertEqual(result["active_aircraft_ids"], [])
        self.assertEqual(result["returning_aircraft_ids"], [4])
        self.assertEqual(result["ongoing_tracking_aircraft_ids"], [5, 6])
        self.assertTrue(any("remaining snapshot geometry unavailable" in message for message in messages))


if __name__ == "__main__":
    unittest.main()
