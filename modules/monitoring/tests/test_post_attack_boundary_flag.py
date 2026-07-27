from __future__ import annotations

import unittest

from modules.monitoring.logic.mission_update import _has_post_attack_boundary_hold


class PostAttackBoundaryFlagTests(unittest.TestCase):
    def test_accepts_planner_attack_completion_flag(self) -> None:
        self.assertTrue(
            _has_post_attack_boundary_hold({"attackCompletionBoundaryHold": True})
        )

    def test_accepts_existing_monitoring_flag(self) -> None:
        self.assertTrue(
            _has_post_attack_boundary_hold({"postAttackBoundaryHold": True})
        )

    def test_false_and_missing_flags_stay_false(self) -> None:
        self.assertFalse(
            _has_post_attack_boundary_hold(
                {"attackCompletionBoundaryHold": False},
                {"post_attack_boundary_hold": "false"},
            )
        )


if __name__ == "__main__":
    unittest.main()
