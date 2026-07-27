from __future__ import annotations

import unittest

from modules.mission_planning.replanning.input_refresh_progress import (
    attach_input_refresh_current_input_id,
    infer_started_input_mission_id,
    input_refresh_current_input_id,
    input_refresh_snapshot_whitelist,
    parallel_snapshot_safety_reasons,
)


def _context(current_input_id: int | None = None) -> dict:
    detail = {
        "trigger": "0201",
        "triggerType": "inputRefresh",
    }
    if current_input_id is not None:
        detail["currentInputMissionID"] = int(current_input_id)
    return {"replan_detail": detail}


class InputRefreshProgressTests(unittest.TestCase):
    def test_materialized_progress_snapshot_does_not_block_parallel_variants(self) -> None:
        reasons = parallel_snapshot_safety_reasons(
            snapshot_apply_result={"applied": 1, "marked_done": 0},
            collapse_apply_result={"groupCount": 1, "mutated": True},
            filtered_payload_materialized=True,
        )

        self.assertEqual(reasons, [])

    def test_unpersisted_progress_snapshot_keeps_sequential_safety_fallback(self) -> None:
        reasons = parallel_snapshot_safety_reasons(
            snapshot_apply_result={"applied": 1, "marked_done": 0},
            collapse_apply_result={"groupCount": 1, "mutated": True},
            filtered_payload_materialized=False,
        )

        self.assertEqual(
            reasons,
            ["remaining_snapshot_mutated", "mission_collapse_mutated"],
        )

    def test_refresh_snapshot_scope_contains_only_current_mission(self) -> None:
        ctx = _context(70000000)

        scope = input_refresh_snapshot_whitelist(
            ctx=ctx,
            staged={},
            mission_whitelist={70000000, 70000001, 70000015},
        )

        self.assertEqual(scope, {70000000})

    def test_refresh_scope_never_falls_back_to_all_when_current_is_missing(self) -> None:
        scope = input_refresh_snapshot_whitelist(
            ctx=_context(),
            staged={},
            mission_whitelist={70000000, 70000001},
        )

        self.assertEqual(scope, {-1})

    def test_started_snapshot_inference_picks_only_progressed_line(self) -> None:
        payload = {
            "inputMissionList": [
                {"inputMissionID": 70000000},
                {"inputMissionID": 70000001},
                {"inputMissionID": 70000015},
            ]
        }
        snapshot = {
            "missions": [
                {
                    "inputMissionID": 70000000,
                    "missionType": "line",
                    "coveragePercent": 24,
                    "isDone": False,
                },
                {
                    "inputMissionID": 70000001,
                    "missionType": "line",
                    "coveragePercent": 0,
                    "isDone": False,
                },
                {
                    "inputMissionID": 70000015,
                    "missionType": "area",
                    "coveragePercent": 0,
                    "isDone": False,
                },
            ]
        }

        self.assertEqual(
            infer_started_input_mission_id(snapshot, payload),
            70000000,
        )

    def test_started_snapshot_inference_refuses_ambiguous_progress(self) -> None:
        payload = {
            "inputMissionList": [
                {"inputMissionID": 70000000},
                {"inputMissionID": 70000001},
            ]
        }
        snapshot = {
            "missions": [
                {
                    "inputMissionID": 70000000,
                    "missionType": "line",
                    "coveragePercent": 24,
                    "isDone": False,
                },
                {
                    "inputMissionID": 70000001,
                    "missionType": "line",
                    "coveragePercent": 12,
                    "isDone": False,
                },
            ]
        }

        self.assertIsNone(infer_started_input_mission_id(snapshot, payload))

    def test_inferred_current_id_is_attached_to_refresh_context_only(self) -> None:
        ctx = _context()
        staged = {
            "replan_detail": {
                "trigger": "0401",
                "triggerType": "agentUnavailable",
            }
        }

        updated = attach_input_refresh_current_input_id(70000000, ctx, staged)

        self.assertEqual(updated, 1)
        self.assertEqual(input_refresh_current_input_id(ctx, staged), 70000000)
        self.assertTrue(ctx["replan_detail"]["preserveCurrentMissionProgress"])
        self.assertNotIn("currentInputMissionID", staged["replan_detail"])


if __name__ == "__main__":
    unittest.main()
