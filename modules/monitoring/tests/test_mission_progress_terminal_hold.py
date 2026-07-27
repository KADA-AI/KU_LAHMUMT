from __future__ import annotations

import unittest

from modules.monitoring.logic.mission_progress import MissionProgressTracker


MISSION_ID = 900000184
PATH_ID = 500000023
NEW_WAYPOINT_ID = 12452
STALE_WAYPOINT_ID = 12375


def _terminal_hold_view() -> dict[str, object]:
    return {
        "mission_plan_id": 700000010,
        "input_missions": [
            {
                "input_mission_id": 70000000,
                "is_done": False,
            }
        ],
        "uav_entries": [
            {
                "aircraft_id": 5,
                "individual_mission_package_id": 800000048,
                "current_individual_mission_id": MISSION_ID,
                "missions": [
                    {
                        "individual_mission_id": MISSION_ID,
                        "input_id": 70000000,
                        "path_id": PATH_ID,
                        "eta_seconds": 27.0,
                        "is_done": False,
                        "post_attack_boundary_hold": True,
                        "waypoints": [
                            {
                                "waypoint_id": NEW_WAYPOINT_ID,
                                "eta": 27.0,
                            }
                        ],
                    }
                ],
            }
        ],
    }


def _agent_state(*, waypoint_id: int | None, flying: int) -> dict[str, int | None]:
    return {
        "aircraft_id": 5,
        "current_waypoint_id": waypoint_id,
        "flying": flying,
        "filming": 2,
        "flight_mode": 8,
    }


def _tracker() -> MissionProgressTracker:
    tracker = MissionProgressTracker()
    tracker.set_system_mode(3)
    tracker.reset(_terminal_hold_view())
    return tracker


class TerminalHoldProgressTests(unittest.TestCase):
    def test_stale_previous_waypoint_does_not_complete_terminal_hold_after_grace(self) -> None:
        tracker = _tracker()

        tracker.update(
            1_000,
            [_agent_state(waypoint_id=STALE_WAYPOINT_ID, flying=2)],
        )
        snapshot = tracker.update(
            12_000,
            [_agent_state(waypoint_id=STALE_WAYPOINT_ID, flying=2)],
        )

        self.assertFalse(snapshot["mission_progress"][MISSION_ID]["done"])
        self.assertEqual(snapshot["new_completed_individual"], [])
        self.assertEqual(snapshot["new_completed_waypoints"], [])

    def test_exact_new_waypoint_allows_terminal_hold_completion(self) -> None:
        tracker = _tracker()

        tracker.update(
            1_000,
            [_agent_state(waypoint_id=STALE_WAYPOINT_ID, flying=2)],
        )
        tracker.update(
            12_000,
            [_agent_state(waypoint_id=STALE_WAYPOINT_ID, flying=2)],
        )
        snapshot = tracker.update(
            12_200,
            [_agent_state(waypoint_id=NEW_WAYPOINT_ID, flying=2)],
        )

        self.assertTrue(snapshot["mission_progress"][MISSION_ID]["done"])
        self.assertEqual(
            snapshot["new_completed_individual"],
            [{"mission_id": MISSION_ID, "package_id": 800000048}],
        )
        self.assertEqual(
            snapshot["new_completed_waypoints"],
            [
                {
                    "mission_id": MISSION_ID,
                    "path_id": PATH_ID,
                    "waypoint_ids": [NEW_WAYPOINT_ID],
                }
            ],
        )

    def test_previously_observed_terminal_waypoint_allows_completion_when_id_clears(self) -> None:
        tracker = _tracker()

        tracker.update(
            1_000,
            [_agent_state(waypoint_id=NEW_WAYPOINT_ID, flying=1)],
        )
        snapshot = tracker.update(
            1_200,
            [_agent_state(waypoint_id=None, flying=2)],
        )

        self.assertTrue(snapshot["mission_progress"][MISSION_ID]["done"])
        self.assertEqual(
            snapshot["new_completed_individual"],
            [{"mission_id": MISSION_ID, "package_id": 800000048}],
        )


if __name__ == "__main__":
    unittest.main()
