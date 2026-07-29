from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303 as d0303_artifacts,
)
from modules.mission_planning.MissionPlanner.runtime_settings import (
    canonicalize_runtime_payload,
    get_runtime_float,
    runtime_override,
)
from modules.mission_planning.pipelines.next_collab_path_builder import (
    _group_area_sweep_items_by_spacing,
    _select_line_sweep_items_by_spacing,
)
from modules.mission_planning.pipelines.mission_path_trim import (
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.runtime.next_collab_line_runner import (
    _derive_initial_line_deployment_reference,
    _orient_polyline_xy_to_direction,
    coord_to_xy,
)
from modules.mission_planning.runtime import next_collab_line_runner
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.type_decider.logic import (
    apply_logic_type_decider,
)
from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    _resolve_line_deployment_coordinate_list_from_templates,
)
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    build_line_scan_remaining_detail,
    has_line_remaining_geometry,
)
from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _attack_source_path_can_skip_resume_split,
    _attack_point_between_friendly_and_enemy,
    _attack_point_inside_line_coverage,
    _build_uav_attack_completion_hold_waypoint,
    _build_uav_attack_resume_package,
    _load_attack_line_coverage_corridors,
    _predict_replan_resume_anchor,
    _release_first_attack_followup_input_group,
    _resolve_attack_source_plan_id,
    _select_attack_standoff_inside_constraints,
    _split_done_resume_path,
)
from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _POST_ATTACK_COMPLETE_HOLD_SECONDS,
    _apply_post_attack_terminal_hold,
    _active_current_input_path_all_done,
    _active_progress_is_complete,
    _allow_post_attack_active_only_replan,
    _block_post_attack_followups_until_next_collab,
    _estimate_active_done_hold_seconds,
    _include_active_completion_boundary_hold,
    _line_rejoin_snapshot_conflicts_with_live_progress,
    _remaining_snapshot_explicitly_completed,
    _recover_tracking_assignments_from_current_plan,
    _should_preserve_active_current_mission,
    _trim_waypoints_for_exact_sweep_progress,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    _build_remaining_input_mission_for_collaborative_replan,
    _prepare_uav_collaborative_resume_replan,
    _replace_selected_current_input_execution_chain,
    _sync_resume_mission_info_with_waypoints,
)
from modules.monitoring.logic.line_scan_progress_monitor import (
    LineScanProgressMonitor,
    _LineMissionRuntime,
    _common_input_coverage,
    _seed_runtime_common_coverage,
)
from modules.monitoring.logic.mission_line_progress import (
    LineSweepState,
    build_line_sweep_definition,
)


def _line_items(*distances_m: float) -> list[dict]:
    return [
        {
            "anchorXY": (float(distance_m), 0.0),
            "sweepXY": [(float(distance_m), 0.0), (float(distance_m), 100.0)],
            "sweepIndex": index,
        }
        for index, distance_m in enumerate(distances_m)
    ]


def _line_progress_entry(*, plan_id: int, covered_end_m: float) -> dict:
    coords = [
        {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
        {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
    ]
    return {
        "missionPlanID": int(plan_id),
        "inputMissionID": 70000000,
        "aircraftID": 4,
        "missionID": 900000001 + int(plan_id),
        "pathID": 400000000 + int(plan_id),
        "sourceLineWidthM": 1000.0,
        "sourceCoordinateList": coords,
        "lineList": [
            {
                "coordinateList": coords,
                "width": 1000.0,
                "plannedLengthM": 1000.0,
                "coveredIntervals": [
                    {"startM": 0.0, "endM": float(covered_end_m)},
                ],
            }
        ],
    }


class LineReplanRegressionTests(unittest.TestCase):
    def test_single_line_carrier_uses_nested_capture_progress_for_resume(
        self,
    ) -> None:
        sweep = [
            {
                "latitude": 38.0 + (index * 0.00001),
                "longitude": 127.0,
                "altitude": 0,
            }
            for index in range(95)
        ]
        carrier = {
            "waypointID": 35328,
            "coordinate": {
                "latitude": 38.0,
                "longitude": 127.01,
                "altitude": 1400,
            },
            "isDone": False,
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": sweep,
                    "interpolationPoints": 2,
                    "searchSpeed": 20.0,
                },
            },
        }
        self.assertFalse(
            _attack_source_path_can_skip_resume_split([carrier])
        )

        _done, resume, _removed = _split_done_resume_path(
            {
                "aircraftID": 5,
                "pathID": 500000246,
                "waypointList": [carrier],
            },
            artifacts=SimpleNamespace(
                path_id=500000246,
                current_waypoint_id=35328,
                previous_waypoint_id=None,
            ),
            sweep_progress={
                500000246: {
                    "bufferPoints": 30,
                    "progressPoints": 30,
                    "sweepPointCount": 95,
                }
            },
            emit=lambda _message: None,
            force_nonempty_resume=True,
            preserve_line_carrier_coordinates=True,
            synchronize_line_search_to_geometry=True,
        )

        self.assertEqual(len(resume), 1)
        self.assertEqual(
            len(
                resume[0]["filmingProperty"]["lineSearch"][
                    "coordinateList"
                ]
            ),
            65,
        )
        self.assertFalse(resume[0]["isDone"])

    def test_type2_attack_diversion_keeps_unconfirmed_buffer_prefix(self) -> None:
        sweep = [
            {
                "latitude": 38.0 + (index * 0.00001),
                "longitude": 127.0,
                "altitude": 0,
            }
            for index in range(96)
        ]
        carrier = {
            "waypointID": 40953,
            "coordinate": {
                "latitude": 38.0,
                "longitude": 127.01,
                "altitude": 1400,
            },
            "isDone": False,
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": sweep,
                    "interpolationPoints": 2,
                    "searchSpeed": 20.0,
                },
            },
        }

        _done, resume, _removed = _split_done_resume_path(
            {
                "aircraftID": 5,
                "pathID": 500000236,
                "waypointList": [carrier],
            },
            artifacts=SimpleNamespace(
                path_id=500000236,
                current_waypoint_id=40953,
                previous_waypoint_id=None,
            ),
            sweep_progress={
                500000236: {
                    "progress_points": 0,
                    "buffer_points": 8,
                    "sweep_point_count": 96,
                    "seconds_per_point": 0.625,
                }
            },
            emit=lambda _message: None,
            force_nonempty_resume=True,
            preserve_line_carrier_coordinates=True,
            synchronize_line_search_to_geometry=True,
            exact_sweep_point_progress=True,
        )

        remaining = resume[0]["filmingProperty"]["lineSearch"]["coordinateList"]
        self.assertEqual(len(remaining), 96)
        self.assertEqual(remaining[0], sweep[0])
        self.assertFalse(resume[0]["isDone"])

    def test_type2_post_attack_resume_keeps_unconfirmed_buffer_prefix(self) -> None:
        sweep = [
            {
                "latitude": 38.0 + (index * 0.00001),
                "longitude": 127.0,
                "altitude": 0,
            }
            for index in range(184)
        ]
        waypoints = [
            {
                "waypointID": 42052,
                "coordinate": {
                    "latitude": 38.0,
                    "longitude": 127.01,
                    "altitude": 1400,
                },
                "isDone": False,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": sweep,
                        "interpolationPoints": 2,
                        "searchSpeed": 20.0,
                    },
                },
            }
        ]

        trimmed, removed = _trim_waypoints_for_exact_sweep_progress(
            waypoints,
            {
                "progress_points": 18,
                "buffer_points": 23,
                "sweep_point_count": 184,
                "seconds_per_point": 0.875,
            },
            reference_coord={
                "latitude": 38.00006,
                "longitude": 127.01,
                "altitude": 1400,
            },
            allow_line_scan_sweep_point_trim=True,
            preserve_carrier_coordinates=True,
        )

        remaining = trimmed[0]["filmingProperty"]["lineSearch"]["coordinateList"]
        self.assertEqual(removed, 18)
        self.assertEqual(len(remaining), 166)
        self.assertEqual(remaining[0], sweep[18])
        self.assertFalse(trimmed[0]["isDone"])

    def test_completed_type2_line_releases_only_next_followup_input_group(
        self,
    ) -> None:
        missions = [
            {
                "individualMissionID": 900000001,
                "relatedMission": {"inputMissionID": 5},
                "executionBlockedUntilNextCollab": True,
            },
            {
                "individualMissionID": 900000002,
                "relatedMission": {"inputMissionID": 5},
                "executionBlockedUntilNextCollab": True,
            },
            {
                "individualMissionID": 900000003,
                "relatedMission": {"inputMissionID": 6},
                "executionBlockedUntilNextCollab": True,
            },
        ]

        cleared, released_input_id = (
            _release_first_attack_followup_input_group(missions)
        )

        self.assertEqual(cleared, 2)
        self.assertEqual(released_input_id, 5)
        self.assertNotIn("executionBlockedUntilNextCollab", missions[0])
        self.assertNotIn("executionBlockedUntilNextCollab", missions[1])
        self.assertTrue(missions[2]["executionBlockedUntilNextCollab"])

    def test_ambiguous_turn_trend_does_not_expand_the_reported_radius(self) -> None:
        """A near-straight EWLS trend is not a 14 km manoeuvre constraint.

        This is the captured Type-2 P3/UAV4 failure: the direction estimator
        explicitly returned sign zero, but its small residual rate replaced a
        valid 706 m radius and made every branch-entry tangent impossible.
        """

        [row] = next_collab_line_runner._normalize_aircraft_rows(
            [
                {
                    "aircraftID": 4,
                    "coordinate": {
                        "latitude": 38.05578622924647,
                        "longitude": 127.34830536550338,
                        "altitude": 1434,
                    },
                    "headingDeg": 208.5630340576172,
                    "speedMps": 57.984676361083984,
                    "turnRadiusM": 706.2581378663889,
                    "idealTurnRadiusM": 706.2581378663889,
                    "lineTrendHeadingDeg": 208.54772960534353,
                    "lineTrendTurnRateDps": -0.22713342653693072,
                    "lineTrendTurnSign": 0,
                    "lineTurnDirectionConfidence": 0.5224591467381422,
                }
            ],
            mission_center_xy=None,
        )

        self.assertAlmostEqual(float(row["turnRadiusM"]), 706.2581378663889)
        self.assertEqual(
            row["turnRadiusSource"],
            "reported_unreliable_trend_direction",
        )

    def test_straight_line_activation_prediction_does_not_require_turn_confidence(
        self,
    ) -> None:
        current = {
            "latitude": 38.0,
            "longitude": 127.0,
            "altitude": 1400,
        }
        predicted = {
            "latitude": 38.0,
            "longitude": 127.004,
            "altitude": 1400,
        }
        [row] = next_collab_line_runner._normalize_aircraft_rows(
            [
                {
                    "aircraftID": 4,
                    "coordinate": current,
                    "currentCoordinate": current,
                    "headingDeg": 90.0,
                    "speedMps": 40.0,
                    "linePredictedEntryCoordinate": predicted,
                    "linePredictedHeadingDeg": 90.0,
                    "linePredictionLeadS": 10.5,
                    "linePredictionModel": "constant-velocity",
                    "lineTurnDataAgeS": 0.2,
                    "lineTurnDirectionConfidence": 0.0,
                }
            ],
            mission_center_xy=None,
        )

        self.assertTrue(row["predictionUsed"])
        self.assertEqual(row["predictionModel"], "constant-velocity")
        self.assertAlmostEqual(
            float(row["planningPositionXY"][0]),
            float(coord_to_xy(predicted)[0]),
            places=3,
        )
        self.assertAlmostEqual(
            float(row["planningPositionXY"][1]),
            float(coord_to_xy(predicted)[1]),
            places=3,
        )
        [stale_row] = next_collab_line_runner._normalize_aircraft_rows(
            [
                {
                    "aircraftID": 4,
                    "coordinate": current,
                    "currentCoordinate": current,
                    "headingDeg": 90.0,
                    "speedMps": 40.0,
                    "linePredictedEntryCoordinate": predicted,
                    "linePredictedHeadingDeg": 90.0,
                    "linePredictionLeadS": 10.5,
                    "linePredictionModel": "constant-velocity",
                    "lineTurnDataAgeS": 6.0,
                    "lineTurnDirectionConfidence": 0.0,
                }
            ],
            mission_center_xy=None,
        )
        self.assertFalse(stale_row["predictionUsed"])
        self.assertAlmostEqual(
            float(stale_row["planningPositionXY"][0]),
            float(coord_to_xy(current)[0]),
            places=3,
        )
        direct_resume_anchor = _predict_replan_resume_anchor(
            current,
            {
                "heading": 270.0,
                "speed": 40.0,
                "linePredictedEntryCoordinate": predicted,
                "linePredictionLeadS": 10.5,
                "lineTurnDataAgeS": 0.2,
            },
        )
        self.assertIsNotNone(direct_resume_anchor)
        self.assertAlmostEqual(
            float(direct_resume_anchor["longitude"]),
            float(predicted["longitude"]),
            places=6,
        )

    def test_attack_type2_line_resume_keeps_all_carriers_during_activation_delay(
        self,
    ) -> None:
        first_sweep = [
            {
                "latitude": 38.001 + (index * 0.00001),
                "longitude": 126.99,
                "altitude": 0,
            }
            for index in range(88)
        ]
        future_sweep = [
            {
                "latitude": 38.01 + (index * 0.00001),
                "longitude": 126.98,
                "altitude": 0,
            }
            for index in range(130)
        ]
        current = {
            "latitude": 38.0,
            "longitude": 127.0,
            "altitude": 1400,
        }
        predicted = {
            "latitude": 38.0,
            "longitude": 127.003,
            "altitude": 1400,
        }
        original_first_carrier = {
            "latitude": 38.0,
            "longitude": 127.01,
            "altitude": 1500,
        }
        original_future_carrier = {
            "latitude": 38.0,
            "longitude": 127.03,
            "altitude": 1500,
        }
        source_path = {
            "aircraftID": 4,
            "pathID": 400000001,
            "waypointList": [
                {
                    "waypointID": 10,
                    "coordinate": dict(current),
                    "filmingProperty": {"operationMode": 1},
                    "isDone": True,
                },
                {
                    "waypointID": 11,
                    "coordinate": dict(original_first_carrier),
                    "filmingProperty": {
                        "operationMode": 2,
                        "lineSearch": {
                            "coordinateList": first_sweep,
                            "interpolationPoints": 2,
                            "searchSpeed": 20.0,
                        },
                    },
                    "isDone": False,
                },
                {
                    "waypointID": 12,
                    "coordinate": dict(original_future_carrier),
                    "filmingProperty": {
                        "operationMode": 2,
                        "lineSearch": {
                            "coordinateList": future_sweep,
                            "interpolationPoints": 2,
                            "searchSpeed": 20.0,
                        },
                    },
                    "isDone": False,
                },
            ],
        }
        allocated = iter(range(5000, 5020))
        timing: dict = {}

        _done, resume, _removed = _split_done_resume_path(
            source_path,
            artifacts=SimpleNamespace(
                path_id=400000001,
                current_waypoint_id=11,
                previous_waypoint_id=10,
            ),
            sweep_progress={
                400000001: {
                    "bufferPoints": 75,
                    "progressPoints": 75,
                    "sweepPointCount": 218,
                }
            },
            emit=lambda _message: None,
            append_replan_anchor=True,
            replan_coordinate=current,
            resume_trim_anchor_coord=predicted,
            waypoint_id_provider=lambda: next(allocated),
            timing=timing,
            preserve_line_carrier_coordinates=True,
            synchronize_line_search_to_geometry=True,
        )

        self.assertEqual(len(resume), 2)
        self.assertEqual(
            len(resume[0]["filmingProperty"]["lineSearch"]["coordinateList"]),
            13,
        )
        self.assertEqual(
            len(resume[1]["filmingProperty"]["lineSearch"]["coordinateList"]),
            130,
        )
        self.assertAlmostEqual(
            resume[0]["coordinate"]["latitude"],
            original_first_carrier["latitude"],
            places=7,
        )
        self.assertAlmostEqual(
            resume[0]["coordinate"]["longitude"],
            original_first_carrier["longitude"],
            places=7,
        )
        self.assertEqual(resume[0]["coordinate"]["altitude"], current["altitude"])
        self.assertEqual(resume[1]["coordinate"], original_future_carrier)
        realign_timing = timing["realign_line_search_waypoints_to_first_sweep"]
        self.assertEqual(realign_timing["reanchoredWaypoints"], 0)
        self.assertTrue(realign_timing["carrierCoordinatesPreserved"])
        self.assertFalse(realign_timing["activationAnchorEnabled"])
        self.assertFalse(realign_timing["activationAnchorApplied"])

    def test_tracking_resume_preserves_carrier_without_attack_time_activation_anchor(
        self,
    ) -> None:
        first_carrier = {
            "latitude": 38.047971,
            "longitude": 127.384717,
            "altitude": 1352,
        }
        second_carrier = {
            "latitude": 38.054501,
            "longitude": 127.409464,
            "altitude": 1568,
        }
        sweep_a = [
            {
                "latitude": 38.05 - (index * 0.0007),
                "longitude": 127.36 + (index * 0.005),
                "altitude": 200,
            }
            for index in range(6)
        ]
        sweep_b = [
            {
                "latitude": 38.0466 + (index * 0.0024),
                "longitude": 127.3857 + (index * 0.0044),
                "altitude": 190,
            }
            for index in range(6)
        ]

        def _waypoint(waypoint_id: int, carrier: dict, sweep: list[dict]) -> dict:
            return {
                "waypointID": waypoint_id,
                "coordinate": dict(carrier),
                "isDone": False,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": list(sweep),
                        "interpolationPoints": 2,
                        "searchSpeed": 20.0,
                    },
                },
            }

        timing: dict = {}
        _done, resume, _removed = _split_done_resume_path(
            {
                "pathID": 400000017,
                "aircraftID": 4,
                "waypointList": [
                    _waypoint(11168, first_carrier, sweep_a),
                    _waypoint(11169, second_carrier, sweep_b),
                ],
            },
            artifacts=SimpleNamespace(
                path_id=400000017,
                current_waypoint_id=11168,
                previous_waypoint_id=11167,
            ),
            sweep_progress={},
            emit=lambda _message: None,
            append_replan_anchor=True,
            replan_coordinate={
                "latitude": 38.0412,
                "longitude": 127.3737,
                "altitude": 1427,
            },
            resume_trim_anchor_coord={
                "latitude": 38.0439,
                "longitude": 127.3739,
                "altitude": 1427,
            },
            timing=timing,
            preserve_line_carrier_coordinates=True,
            apply_line_activation_anchor=False,
            synchronize_line_search_to_geometry=True,
        )

        self.assertAlmostEqual(
            resume[0]["coordinate"]["latitude"], first_carrier["latitude"], places=7
        )
        self.assertAlmostEqual(
            resume[0]["coordinate"]["longitude"], first_carrier["longitude"], places=7
        )
        self.assertEqual(resume[0]["coordinate"]["altitude"], 1427)
        self.assertEqual(resume[1]["coordinate"], second_carrier)
        realign_timing = timing["realign_line_search_waypoints_to_first_sweep"]
        self.assertTrue(realign_timing["carrierCoordinatesPreserved"])
        self.assertFalse(realign_timing["activationAnchorEnabled"])
        self.assertFalse(realign_timing["activationAnchorApplied"])

    def test_sweep_trim_one_point_fallback_does_not_move_preserved_carrier(
        self,
    ) -> None:
        original_carrier = {
            "latitude": 38.0,
            "longitude": 127.02,
            "altitude": 1400,
        }
        sweep_coords = [
            {
                "latitude": 38.001 + (index * 0.0001),
                "longitude": 126.99,
                "altitude": 0,
            }
            for index in range(6)
        ]
        waypoint = {
            "waypointID": 10,
            "coordinate": dict(original_carrier),
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": sweep_coords,
                    "interpolationPoints": 2,
                },
            },
        }

        trimmed, removed = trim_waypoints_by_sweep_points(
            [waypoint],
            5,
            preserve_waypoints=True,
            preserve_carrier_coordinates=True,
        )

        self.assertEqual(removed, 5)
        self.assertEqual(len(trimmed), 1)
        self.assertEqual(trimmed[0]["coordinate"], original_carrier)
        self.assertNotIn("lineSearch", trimmed[0]["filmingProperty"])
        self.assertEqual(trimmed[0]["filmingProperty"]["operationMode"], 1)

    def test_attack_resume_publishes_remaining_source_line_not_scan_zigzag(self) -> None:
        source = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 120},
            {"latitude": 38.04, "longitude": 127.0, "altitude": 120},
        ]
        remaining_zigzag = [
            {"latitude": 38.030, "longitude": 126.9977, "altitude": 1000},
            {"latitude": 38.032, "longitude": 127.0023, "altitude": 1000},
            {"latitude": 38.034, "longitude": 126.9977, "altitude": 1000},
            {"latitude": 38.036, "longitude": 127.0023, "altitude": 1000},
            {"latitude": 38.038, "longitude": 126.9977, "altitude": 1000},
            {"latitude": 38.040, "longitude": 127.0023, "altitude": 1000},
        ]
        mission = {
            "individualMissionInfo": {
                "individualMissionType": 6,
                # Collaborative parent corridor width.  The UAV owns only the
                # 500 m strip in lineList and attack resume must not inflate it
                # back to this full width.
                "sourceLineWidthM": 1500.0,
                "coordinateList": source,
                "lineList": [{"coordinateList": source, "width": 500}],
                "areaList": [],
            }
        }
        waypoints = [
            {
                "waypointID": 1,
                "filmingProperty": {
                    "lineSearch": {
                        "coordinateList": remaining_zigzag,
                        "width": 500,
                    }
                },
            }
        ]
        original_waypoints = json.loads(json.dumps(waypoints))

        self.assertTrue(_sync_resume_mission_info_with_waypoints(mission, waypoints))

        # The aircraft must continue flying the original executable zigzag.
        self.assertEqual(waypoints, original_waypoints)
        info = mission["individualMissionInfo"]
        self.assertEqual(info["lineGeometrySource"], "projected_source_centerline")
        self.assertEqual(info["sourceCoordinateList"], source)
        self.assertEqual(info["sourceLineWidthM"], 1500.0)
        self.assertEqual(info["assignedLineWidthM"], 500.0)
        self.assertEqual(len(info["lineList"]), 1)
        self.assertEqual(info["lineList"][0]["width"], 500)
        published = info["lineList"][0]["coordinateList"]
        self.assertEqual(len(published), 2)
        self.assertAlmostEqual(published[0]["latitude"], 38.030, places=6)
        self.assertAlmostEqual(published[-1]["latitude"], 38.040, places=6)
        self.assertTrue(all(abs(row["longitude"] - 127.0) < 1e-8 for row in published))
        self.assertEqual(info["coordinateList"], published)

    def test_chained_attack_resume_keeps_canonical_source_line(self) -> None:
        source = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 120},
            {"latitude": 38.04, "longitude": 127.0, "altitude": 120},
        ]
        previously_published_zigzag = [
            {"latitude": 38.030, "longitude": 126.9977, "altitude": 1000},
            {"latitude": 38.040, "longitude": 127.0023, "altitude": 1000},
        ]
        final_zigzag = [
            {"latitude": 38.036, "longitude": 126.9977, "altitude": 1000},
            {"latitude": 38.038, "longitude": 127.0023, "altitude": 1000},
            {"latitude": 38.040, "longitude": 126.9977, "altitude": 1000},
        ]
        mission = {
            "individualMissionInfo": {
                "individualMissionType": 6,
                "sourceCoordinateList": source,
                "sourceLineWidthM": 500.0,
                "coordinateList": previously_published_zigzag,
                "lineList": [
                    {"coordinateList": previously_published_zigzag, "width": 500}
                ],
                "areaList": [],
            }
        }
        waypoints = [
            {
                "waypointID": 2,
                "filmingProperty": {
                    "lineSearch": {
                        "coordinateList": final_zigzag,
                        "width": 500,
                    }
                },
            }
        ]

        self.assertTrue(_sync_resume_mission_info_with_waypoints(mission, waypoints))

        published = mission["individualMissionInfo"]["lineList"][0]["coordinateList"]
        self.assertEqual(len(published), 2)
        self.assertAlmostEqual(published[0]["latitude"], 38.036, places=6)
        self.assertAlmostEqual(published[-1]["latitude"], 38.040, places=6)
        self.assertTrue(all(abs(row["longitude"] - 127.0) < 1e-8 for row in published))

    def test_next_collab_line_retries_only_the_missing_piece(self) -> None:
        mission = {
            "inputMissionID": 4,
            "inputMissionType": 1,
            "regionType": 3,
            "missionDetail": {
                "lineList": [
                    {
                        "width": 1000,
                        "coordinateList": [
                            {"latitude": 37.8, "longitude": 127.8, "altitude": 0},
                            {"latitude": 37.85, "longitude": 127.85, "altitude": 0},
                        ],
                    }
                ],
                "areaList": None,
            },
        }
        aircraft = [
            {
                "aircraftID": aircraft_id,
                "coordinate": {
                    "latitude": 37.79 + (index * 0.005),
                    "longitude": 127.79 + (index * 0.005),
                    "altitude": 1000,
                },
                "headingDeg": 45,
            }
            for index, aircraft_id in enumerate((4, 5, 6))
        ]
        original_make_path_row = next_collab_line_runner._HeadlessLinePlanner._make_path_row
        failed_piece_indices: set[int] = set()

        def fail_one_primary_candidate(
            planner: object,
            piece: SplitPiece,
            overlay: dict,
            *,
            allow_alt_targets: bool = False,
        ) -> dict | None:
            piece_index = int(piece.piece_index or 0)
            if piece_index == 2 and not allow_alt_targets:
                failed_piece_indices.add(piece_index)
                return None
            return original_make_path_row(
                planner,
                piece,
                overlay,
                allow_alt_targets=allow_alt_targets,
            )

        logs: list[str] = []
        with (
            patch.object(next_collab_line_runner, "_line_plan_cache_enabled", return_value=False),
            patch.object(
                next_collab_line_runner._HeadlessLinePlanner,
                "_make_path_row",
                new=fail_one_primary_candidate,
            ),
        ):
            result = next_collab_line_runner.run_next_collab_line_plan(
                target_mission=mission,
                aircraft_entries=aircraft,
                planning_mode={"package_type": 1},
                log=logs.append,
            )

        self.assertEqual(failed_piece_indices, {2})
        self.assertEqual(len(result.split_result.pieces), 3)
        self.assertEqual(len(result.expected_paths), 3)
        self.assertEqual(
            {int(row["pieceIndex"]) for row in result.expected_paths},
            {1, 2, 3},
        )
        self.assertTrue(
            any("retrying missing path rows" in line and "P2" in line for line in logs)
        )

    def test_next_collab_line_never_returns_a_partial_piece_set(self) -> None:
        mission = {
            "inputMissionID": 4,
            "inputMissionType": 1,
            "regionType": 3,
            "missionDetail": {
                "lineList": [
                    {
                        "width": 1000,
                        "coordinateList": [
                            {"latitude": 37.8, "longitude": 127.8, "altitude": 0},
                            {"latitude": 37.85, "longitude": 127.85, "altitude": 0},
                        ],
                    }
                ],
                "areaList": None,
            },
        }
        aircraft = [
            {
                "aircraftID": aircraft_id,
                "coordinate": {
                    "latitude": 37.79 + (index * 0.005),
                    "longitude": 127.79 + (index * 0.005),
                    "altitude": 1000,
                },
                "headingDeg": 45,
            }
            for index, aircraft_id in enumerate((4, 5, 6))
        ]
        original_make_path_row = next_collab_line_runner._HeadlessLinePlanner._make_path_row

        def fail_one_piece_completely(
            planner: object,
            piece: SplitPiece,
            overlay: dict,
            *,
            allow_alt_targets: bool = False,
        ) -> dict | None:
            if int(piece.piece_index or 0) == 2:
                return None
            return original_make_path_row(
                planner,
                piece,
                overlay,
                allow_alt_targets=allow_alt_targets,
            )

        with (
            patch.object(next_collab_line_runner, "_line_plan_cache_enabled", return_value=False),
            patch.object(
                next_collab_line_runner._HeadlessLinePlanner,
                "_make_path_row",
                new=fail_one_piece_completely,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "partial path set.*P2"):
                next_collab_line_runner.run_next_collab_line_plan(
                    target_mission=mission,
                    aircraft_entries=aircraft,
                    planning_mode={"package_type": 1},
                )

    def test_prior_collab_removes_all_selected_reexecute_line_siblings(self) -> None:
        def mission(
            mission_id: int,
            input_id: int,
            *,
            related_type: int = 1,
        ) -> dict:
            return {
                "individualMissionID": int(mission_id),
                "relatedMission": {
                    "inputMissionID": int(input_id),
                    "relatedMissionType": int(related_type),
                },
            }

        rebuilt, removed_ids = _replace_selected_current_input_execution_chain(
            [
                mission(900000001, 3),
                mission(900000101, 4),
                mission(900000102, 4),
                mission(900000103, 4, related_type=2),
                mission(900000104, 4),
                mission(900000201, 5),
            ],
            target_index=2,
            current_input_id=4,
            replacement_missions=[
                mission(900000301, 4, related_type=2),
                mission(900000302, 4),
            ],
        )

        self.assertEqual(removed_ids, [900000101, 900000102, 900000104])
        self.assertEqual(
            [int(row["individualMissionID"]) for row in rebuilt],
            [900000001, 900000301, 900000302, 900000103, 900000201],
        )

    def test_prior_collab_line_rejects_partial_uav_imp_build_before_write(self) -> None:
        current_input = {
            "inputMissionID": 4,
            "inputMissionType": 1,
            "isDone": False,
            "missionDetail": {
                "lineList": [
                    {
                        "width": 1000,
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0},
                            {"latitude": 38.01, "longitude": 127.01},
                        ],
                    }
                ]
            },
        }
        prepared = SimpleNamespace(
            replacement_by_aircraft={
                4: [{"pathID": 400000001}],
                5: [{"pathID": 500000001}],
            },
            generated_fp_by_path={
                400000001: {"pathID": 400000001, "waypointList": []},
                500000001: {"pathID": 500000001, "waypointList": []},
            },
            id_reservation={},
            planner_workflow="line_prediction_path",
            timing_ms={},
            source_cache={},
            mission_mode="line",
        )
        messages: list[str] = []
        with (
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "_build_remaining_input_mission_for_collaborative_replan",
                return_value=(current_input, None),
            ),
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "_aircraft_ids_for_input_mission",
                return_value={4, 5},
            ),
            patch(
                "modules.mission_planning.replanning.triggers.next_collab.pipeline."
                "prepare_next_collab_input_replacements",
                return_value=prepared,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "_build_collaborative_remaining_imp_update_payload",
                side_effect=[SimpleNamespace(imp_id=800000001), None],
            ),
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline.write_json"
            ) as write_json_mock,
        ):
            result = _prepare_uav_collaborative_resume_replan(
                source_plan_id=700000001,
                current_input_id=4,
                unavailable_aircraft_ids=set(),
                agent_state_map={
                    4: {"coordinate": {"latitude": 38.0, "longitude": 127.0}},
                    5: {"coordinate": {"latitude": 38.001, "longitude": 127.001}},
                },
                now_ms=1234,
                emit=messages.append,
                log_prefix="[TEST]",
                drop_prefix_missions=False,
            )

        self.assertIsNone(result)
        write_json_mock.assert_not_called()
        self.assertTrue(
            any("rejected before write" in message for message in messages)
        )

    def test_initial_line_uses_regular_entry_without_close_takeover_special_case(self) -> None:
        line_start = {"latitude": 38.0, "longitude": 127.0, "altitude": 1000}
        mission = {
            "aircraftID": 4,
            "pathID": 400000001,
            "individualMissionID": 800000001,
            "relatedMission": {"inputMissionID": 70000000},
            "individualMissionInfo": {
                "individualMissionType": 6,
                "patternType": 8,
                "SEP": 500.0,
                "routeOffsetSepM": 500.0,
                "FOV": 7.2,
                "lineList": [
                    {
                        "width": 3000.0,
                        "coordinateList": [
                            line_start,
                            {"latitude": 38.03, "longitude": 127.0, "altitude": 1000},
                        ],
                    }
                ],
            },
        }
        takeover = {"latitude": 38.0002, "longitude": 127.0001, "altitude": 1000}
        ref0203 = {
            "takeOverInfoList": [
                {"aircraftID": 4, "coordinate": takeover},
            ]
        }

        with patch.object(d0303_artifacts, "_dem_alt", return_value=0.0):
            [flight_plan] = d0303_artifacts.build_flight_plans(
                [mission],
                ref0203=ref0203,
            )

        waypoints = flight_plan["waypointList"]
        self.assertGreaterEqual(len(waypoints), 2)
        first_filming = waypoints[0]["filmingProperty"]
        self.assertEqual(first_filming["operationMode"], d0303_artifacts.OPMODE_POINT)
        self.assertNotEqual(waypoints[0]["coordinate"], takeover)
        # 진입 WP는 회랑에서 SEP × line_route_offset_scale 만큼만 떨어진다.
        # scale=1.0 이던 시절에는 SEP 전체(500 m)가 이격이 되어 첫 촬영이
        # 계속 멀리서 이뤄졌다.  이제는 그 비율만큼 회랑에 붙어야 한다.
        entry_gap_m = d0303_artifacts._dist_between_coords(
            waypoints[0]["coordinate"], line_start
        )
        offset_scale = float(
            get_runtime_float("line_route_offset_scale", 1.0, None)
        )
        expected_gap_m = 500.0 * offset_scale
        self.assertGreater(entry_gap_m, 0.0)
        self.assertLess(abs(entry_gap_m - expected_gap_m), 0.25 * expected_gap_m)
        self.assertTrue(
            any(
                (waypoint.get("filmingProperty") or {}).get("lineSearch")
                for waypoint in waypoints[1:]
            )
        )
        path_fovs = {
            float((waypoint.get("filmingProperty") or {}).get("fieldOfView"))
            for waypoint in waypoints
            if (waypoint.get("filmingProperty") or {}).get("fieldOfView")
        }
        self.assertEqual(len(path_fovs), 1)
        self.assertEqual(
            mission["individualMissionInfo"]["FOV"],
            round(next(iter(path_fovs)), 3),
        )

    def test_initial_line_route_waypoints_respect_1500m_interval(self) -> None:
        """Regression for path 400000001: a 4.45 km LINE became one WP leg."""

        mission = {
            "aircraftID": 4,
            "pathID": 400000001,
            "individualMissionID": 900000025,
            "relatedMission": {"inputMissionID": 1},
            "individualMissionInfo": {
                "individualMissionType": 6,
                "patternType": 8,
                "BEARING": 1.991542380594808,
                "SPEED": 144.0,
                "FOV": 5.74,
                "SEP": 249.32800284318068,
                "routeOffsetSepM": 249.32800284318068,
                "lineList": [
                    {
                        "width": 497.0,
                        "coordinateList": [
                            {
                                "latitude": 37.98371885561281,
                                "longitude": 127.29642204839983,
                                "altitude": 0,
                            },
                            {
                                "latitude": 38.02381772561281,
                                "longitude": 127.29819208839983,
                                "altitude": 0,
                            },
                        ],
                    }
                ],
            },
        }

        with (
            patch.object(d0303_artifacts, "_dem_alt", return_value=0.0),
            runtime_override({"values": {"uav_wp_interval_m": 1500.0}}),
        ):
            [flight_plan] = d0303_artifacts.build_flight_plans(
                [mission],
                wp_alloc=d0303_artifacts._WPAllocator(start=1),
                cruise_speed=40.0,
            )

        waypoints = flight_plan["waypointList"]
        self.assertEqual(len(waypoints), 4)
        gaps_m = [
            d0303_artifacts._dist_between_coords(
                waypoints[index - 1]["coordinate"],
                waypoints[index]["coordinate"],
            )
            for index in range(1, len(waypoints))
        ]
        self.assertGreater(sum(gaps_m), 4400.0)
        self.assertLessEqual(max(gaps_m), 1500.0)
        self.assertEqual(
            [
                int((waypoint.get("filmingProperty") or {}).get("operationMode", 0))
                for waypoint in waypoints
            ],
            [d0303_artifacts.OPMODE_POINT] + [d0303_artifacts.OPMODE_LINE] * 3,
        )
        line_coord_counts = [
            len(
                (((waypoint.get("filmingProperty") or {}).get("lineSearch") or {}).get("coordinateList") or [])
            )
            for waypoint in waypoints[1:]
        ]
        self.assertTrue(all(count > 0 for count in line_coord_counts))
        self.assertEqual(sum(line_coord_counts), 230)

    def test_route_resampler_splits_any_leg_over_configured_interval(self) -> None:
        samples = d0303_artifacts._resample_polyline_xy(
            [(0.0, 0.0), (1800.0, 0.0)],
            1500.0,
        )
        self.assertEqual(samples, [(0.0, 0.0), (1500.0, 0.0), (1800.0, 0.0)])

    def test_control_transfer_route_also_respects_line_wp_interval(self) -> None:
        samples = d0303_artifacts._resample_route_coordinates(
            [
                {"latitude": 37.99402641, "longitude": 127.376421},
                {"latitude": 37.96350607, "longitude": 127.31004444},
            ],
            1500.0,
        )
        gaps_m = [
            d0303_artifacts._dist_between_coords(samples[index - 1], samples[index])
            for index in range(1, len(samples))
        ]
        self.assertEqual(len(samples), 6)
        self.assertLessEqual(max(gaps_m), 1500.0)

    def test_type_decider_keeps_line_when_replan_mirrors_coords_to_coordinate_list(self) -> None:
        coords = [
            {"latitude": 38.02, "longitude": 127.29, "altitude": 95},
            {"latitude": 38.05, "longitude": 127.27, "altitude": 95},
        ]
        piece = SplitPiece(
            parent_order=1,
            mission_id=70000000,
            mission_type=1,
            piece_index=1,
            data={
                "Geometry": "Area",
                "Centerline": list(coords),
                "coordinateList": [
                    {"latitude": 38.019, "longitude": 127.289, "altitude": 95},
                    {"latitude": 38.051, "longitude": 127.269, "altitude": 95},
                    {"latitude": 38.052, "longitude": 127.271, "altitude": 95},
                ],
            },
            assigned_uav=4,
        )
        split_result = SplitRunResult(uav_count=1, uav_ids=[4], pieces=[piece])
        cmpk = {
            "inputMissionPackageType": 1,
            "availableAircraftList": [{"aircraftID": 4}],
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "inputMissionType": 1,
                    "missionDetail": {
                        # inputRefresh stores this progress-trimmed copy in
                        # addition to the still-authoritative lineList.
                        "coordinateList": list(coords),
                        "lineList": [{"width": 1000, "coordinateList": list(coords)}],
                        "areaList": None,
                    },
                }
            ],
        }

        report = apply_logic_type_decider(split_result, cmpk)

        self.assertEqual(piece.data["individualMissionType"], 6)
        self.assertEqual(piece.data["patternType"], 8)
        self.assertIn("line", piece.data["typeDecider"]["rule"])
        self.assertEqual(report["imTypeCounts"], {6: 1})

    def test_type_decider_preserves_true_coordinate_only_mission(self) -> None:
        coords = [
            {"latitude": 38.02, "longitude": 127.29, "altitude": 95},
            {"latitude": 38.05, "longitude": 127.27, "altitude": 95},
        ]
        piece = SplitPiece(
            parent_order=1,
            mission_id=70000000,
            mission_type=1,
            piece_index=1,
            data={"coordinateList": list(coords)},
            assigned_uav=4,
        )
        split_result = SplitRunResult(uav_count=1, uav_ids=[4], pieces=[piece])
        cmpk = {
            "inputMissionPackageType": 1,
            "availableAircraftList": [{"aircraftID": 4}],
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "inputMissionType": 1,
                    "missionDetail": {
                        "coordinateList": list(coords),
                        "lineList": None,
                        "areaList": None,
                    },
                }
            ],
        }

        apply_logic_type_decider(split_result, cmpk)

        self.assertEqual(piece.data["individualMissionType"], 5)
        self.assertEqual(piece.data["patternType"], 1)

    def test_attack_point_must_stay_between_lah_and_target(self) -> None:
        lah = {"latitude": 38.0, "longitude": 127.0, "altitude": 1000}
        target = {"latitude": 38.02, "longitude": 127.0, "altitude": 0}

        self.assertTrue(
            _attack_point_between_friendly_and_enemy(
                {"latitude": 38.01, "longitude": 127.0},
                lah,
                target,
            )
        )
        self.assertFalse(
            _attack_point_between_friendly_and_enemy(
                {"latitude": 38.03, "longitude": 127.0},
                lah,
                target,
            )
        )
        self.assertFalse(
            _attack_point_between_friendly_and_enemy(
                {"latitude": 37.99, "longitude": 127.0},
                lah,
                target,
            )
        )
        self.assertFalse(
            _attack_point_between_friendly_and_enemy(
                # Projection is between the endpoints, but this point is
                # roughly 1.7 km sideways from the actual attack corridor.
                {"latitude": 38.01, "longitude": 127.02},
                lah,
                target,
            )
        )

    def test_attack_point_uses_completed_or_current_mission_zone(self) -> None:
        line_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100},
            {"latitude": 38.03, "longitude": 127.0, "altitude": 100},
        ]
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "isDone": False,
                    "missionDetail": {
                        "lineList": [{"width": 1000, "coordinateList": line_coords}],
                        "areaList": [],
                    },
                }
            ]
        }
        target = {"latitude": 38.02, "longitude": 127.0, "altitude": 0}
        lah = {"latitude": 37.995, "longitude": 127.0, "altitude": 1000}

        with (
            patch(
                "modules.mission_planning.replanning.triggers.attack.pipeline."
                "_load_input_plan_for_source_plan",
                return_value=input_plan,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.attack.pipeline.load_sweep_progress",
                return_value={},
            ),
        ):
            zones, summary = _load_attack_line_coverage_corridors(
                source_plan_id=700000001,
                watcher_id=6,
                target_coord=target,
            )

        self.assertEqual(summary.get("reason"), "first_discovered_mission_zone_inside")
        self.assertTrue(summary.get("requireInsideDiscoveredMissionZone"))
        self.assertEqual(len(zones), 1)
        selected = _select_attack_standoff_inside_constraints(
            lah,
            target,
            min_standoff_m=300.0,
            preferred_standoff_m=2000.0,
            line_coverage_corridors=zones,
        )
        self.assertIsNotNone(selected)
        selected_coord = selected["coordinate"]
        self.assertTrue(_attack_point_between_friendly_and_enemy(selected_coord, lah, target))
        self.assertTrue(_attack_point_inside_line_coverage(selected_coord, zones))
        self.assertTrue(selected.get("mission_zone_constrained"))

    def test_attack_zone_uses_only_the_mission_that_contains_the_detected_target(self) -> None:
        first_line = [
            {"latitude": 38.0, "longitude": 127.0},
            {"latitude": 38.02, "longitude": 127.0},
        ]
        detected_line = [
            {"latitude": 38.0, "longitude": 127.05},
            {"latitude": 38.03, "longitude": 127.05},
        ]
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "isDone": True,
                    "missionDetail": {
                        "lineList": [{"width": 800, "coordinateList": first_line}],
                    },
                },
                {
                    "inputMissionID": 70000001,
                    "isDone": False,
                    "missionDetail": {
                        "lineList": [{"width": 800, "coordinateList": detected_line}],
                    },
                },
            ]
        }
        progress = {
            "old": {
                "missionPlanID": 700000001,
                "aircraftID": 6,
                "inputMissionID": 70000000,
            },
            "current": {
                "missionPlanID": 700000001,
                "aircraftID": 6,
                "inputMissionID": 70000001,
            },
        }
        target = {"latitude": 38.02, "longitude": 127.05, "altitude": 0}

        with (
            patch(
                "modules.mission_planning.replanning.triggers.attack.pipeline."
                "_load_input_plan_for_source_plan",
                return_value=input_plan,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.attack.pipeline.load_sweep_progress",
                return_value=progress,
            ),
        ):
            zones, summary = _load_attack_line_coverage_corridors(
                source_plan_id=700000001,
                watcher_id=6,
                target_coord=target,
            )

        self.assertEqual(summary.get("inputMissionIDs"), [70000001])
        self.assertEqual(summary.get("targetMissionIndex"), 1)
        self.assertFalse(summary.get("requireInsideDiscoveredMissionZone"))
        self.assertTrue(summary.get("allowBeforeDiscoveredMissionZone"))
        self.assertEqual(len(zones), 1)
        self.assertEqual(zones[0].get("inputMissionID"), 70000001)

    def test_first_mission_attack_point_is_forced_inside_detected_area(self) -> None:
        target = {"latitude": 38.02, "longitude": 127.0, "altitude": 0}
        lah = {"latitude": 37.99, "longitude": 127.0, "altitude": 1000}
        area = {
            "zoneType": "area",
            "inputMissionID": 70000000,
            "coordinateList": [
                {"latitude": 38.015, "longitude": 126.995},
                {"latitude": 38.015, "longitude": 127.005},
                {"latitude": 38.025, "longitude": 127.005},
                {"latitude": 38.025, "longitude": 126.995},
            ],
        }

        selected = _select_attack_standoff_inside_constraints(
            lah,
            target,
            min_standoff_m=300.0,
            preferred_standoff_m=2000.0,
            line_coverage_corridors=[area],
            require_inside_mission_zone=True,
        )

        self.assertIsNotNone(selected)
        self.assertTrue(
            _attack_point_inside_line_coverage(
                selected["coordinate"],
                [area],
                tolerance_m=0.0,
            )
        )
        self.assertEqual(selected.get("mission_zone_requirement"), "inside")

    def test_attack_point_does_not_retreat_when_lah_is_already_inside_minimum_standoff(self) -> None:
        lah = {"latitude": 38.0, "longitude": 127.0, "altitude": 1000}
        target = {"latitude": 38.005, "longitude": 127.0, "altitude": 0}

        selected = _select_attack_standoff_inside_constraints(
            lah,
            target,
            min_standoff_m=3000.0,
            preferred_standoff_m=9000.0,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected["coordinate"], lah)
        self.assertEqual(
            selected.get("selection_mode"),
            "current_position_inside_min_standoff_no_retreat",
        )
        self.assertTrue(selected.get("minimum_standoff_bypassed"))
        self.assertTrue(selected.get("no_retreat"))

    def test_post_attack_path_completion_requires_every_current_path_done(self) -> None:
        imp = {
            "individualMissionList": [
                {"pathID": 4001, "relatedMission": {"inputMissionID": 101}},
                {"pathID": 4002, "relatedMission": {"inputMissionID": 101}},
            ]
        }
        path_payloads = {
            4001: {"waypointList": [{"waypointID": 1, "isDone": True}]},
            4002: {"waypointList": [{"waypointID": 2, "isDone": False}]},
        }

        with (
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_load_imp_package_for_aircraft_cached",
                return_value=imp,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "_load_path_payload",
                side_effect=lambda path_id, **_kwargs: path_payloads[int(path_id)],
            ),
        ):
            self.assertFalse(
                _active_current_input_path_all_done(
                    source_plan_id=7001,
                    current_input_id=101,
                    aircraft_id=4,
                )
            )
            path_payloads[4002]["waypointList"][0]["isDone"] = True
            self.assertTrue(
                _active_current_input_path_all_done(
                    source_plan_id=7001,
                    current_input_id=101,
                    aircraft_id=4,
                )
            )

    def test_post_attack_completion_hold_is_a_fixed_acknowledgement_window(self) -> None:
        hold_seconds = _estimate_active_done_hold_seconds(
            current_plan_id=700000001,
            current_input_id=101,
            evaluation={"max_return_eta_s": 300},
            group_assignments=[],
            agent_state_map={},
            config={},
        )
        waypoint = {
            "eta": 63,
            "waypointPassType": 3,
            "isDone": True,
        }

        # Production hands the estimated window straight through, so the test
        # does the same rather than a value the pipeline never produces.
        _apply_post_attack_terminal_hold(
            [waypoint],
            hold_seconds=hold_seconds,
            orientation_coordinate={
                "latitude": 38.0,
                "longitude": 127.0,
                "altitude": 300.0,
            },
        )

        # A completion-boundary hold is an acknowledgement window, not a
        # rendezvous ETA, so it stays invariant against the 300 s return ETA
        # above.  Assert against the shared constant rather than a literal.
        expected_hold_s = int(_POST_ATTACK_COMPLETE_HOLD_SECONDS)
        self.assertEqual(hold_seconds, expected_hold_s)
        self.assertEqual(waypoint["waypointPassType"], 2)
        self.assertEqual(waypoint["loiterProperty"]["time"], expected_hold_s)
        self.assertFalse(waypoint["isDone"])

    def test_0303_uses_separate_runtime_wp_intervals_for_line_and_area(self) -> None:
        runtime_payload = {
            "values": {
                "uav_wp_interval_m": 2000.0,
                "area_wp_interval_m": 1000.0,
            }
        }
        with (
            patch.object(d0303_artifacts, "SWEEP_ROUTE_WP_SPACING_M", 1200.0),
            patch.object(d0303_artifacts, "AREA_SWEEP_ROUTE_WP_SPACING_M", 1200.0),
            runtime_override(runtime_payload),
        ):
            self.assertEqual(d0303_artifacts._line_route_wp_spacing_m(), 2000.0)
            self.assertEqual(d0303_artifacts._area_route_wp_spacing_m(), 1000.0)

    def test_area_spacing_merges_short_tail_at_terminal_position(self) -> None:
        items = [
            {
                "anchorXY": (distance_m, 0.0),
                "progressM": distance_m,
                "sweepIndex": index,
            }
            for index, distance_m in enumerate((0.0, 1000.0, 1300.0))
        ]

        groups = _group_area_sweep_items_by_spacing(items, spacing_m=1000.0)

        self.assertEqual(len(groups), 1)
        self.assertEqual(
            [round(float(item["progressM"])) for item in groups[0]],
            [0, 1000, 1300],
        )
        self.assertEqual(round(float(groups[0][-1]["progressM"])), 1300)

    def test_0303_honors_runtime_area_dubins_disable(self) -> None:
        with (
            patch.object(d0303_artifacts, "AREA_DUBINS_ENTRY_LINKS_ENABLED", True),
            patch.object(d0303_artifacts, "_DubinsPoint2D", object()),
            patch.object(d0303_artifacts, "_compute_turn_link", object()),
            runtime_override({"values": {"area_dubins_entry_links_enabled": False}}),
        ):
            self.assertFalse(d0303_artifacts._dubins_link_available())

    def test_input_refresh_initial_geometry_flag_survives_canonicalization(self) -> None:
        payload = canonicalize_runtime_payload(
            {"values": {"input_refresh_preserve_initial_geometry_enabled": True}}
        )
        self.assertTrue(
            payload["values"]["input_refresh_preserve_initial_geometry_enabled"]
        )

    def test_post_attack_newer_exact_snapshot_blocks_backward_line_expansion(self) -> None:
        def line_detail(start_lon: float, *, timestamp_ms: int | None = None) -> dict:
            detail = {
                "coordinateList": [
                    {"latitude": 38.0, "longitude": float(start_lon)},
                    {"latitude": 38.0, "longitude": 127.01},
                ],
                "lineList": [
                    {
                        "width": 1000.0,
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": float(start_lon)},
                            {"latitude": 38.0, "longitude": 127.01},
                        ],
                    }
                ],
                "areaList": [],
            }
            if timestamp_ms is not None:
                detail["lineScanLatestTimestampMs"] = int(timestamp_ms)
            return detail

        live = line_detail(127.006, timestamp_ms=1000)
        snapshot = line_detail(127.008)

        self.assertTrue(
            _line_rejoin_snapshot_conflicts_with_live_progress(
                live_line_detail=live,
                snapshot_guard={
                    "detail": snapshot,
                    "coveragePercent": 67.0,
                    "snapshotTimestampMs": 1200,
                },
                evaluation={"active_avg_progress_percent": 53.0},
            )
        )
        self.assertFalse(
            _line_rejoin_snapshot_conflicts_with_live_progress(
                live_line_detail=live,
                snapshot_guard={
                    "detail": snapshot,
                    "coveragePercent": 67.0,
                    "snapshotTimestampMs": 900,
                },
                evaluation={"active_avg_progress_percent": 53.0},
            )
        )

    def test_line_monitor_rejects_attack_and_transit_children_of_line_input(self) -> None:
        source_row = {
            "width": 1000.0,
            "coordinateList": [
                {"latitude": 38.0, "longitude": 127.0},
                {"latitude": 38.0, "longitude": 127.01},
            ],
        }
        for mission_type, pattern_type in ((1, 1), (7, 10)):
            artifact = {
                "individual_mission_type": mission_type,
                "pattern_type": pattern_type,
                "coordinate_list": list(source_row["coordinateList"]),
                "line_list": [],
                "input_line_list": [source_row],
                "sweep_line_coordinate_lists": [],
                "sweep_point_count": 0,
            }
            self.assertIsNone(build_line_sweep_definition(artifact))

    def test_line_monitor_accepts_individual_line_or_linesearch_geometry(self) -> None:
        coords = [
            {"latitude": 38.0, "longitude": 127.0},
            {"latitude": 38.0, "longitude": 127.01},
        ]
        explicit_line = {
            "line_list": [{"width": 1000.0, "coordinateList": coords}],
            "input_line_list": [{"width": 1000.0, "coordinateList": coords}],
        }
        line_search_only = {
            "line_list": [],
            "input_line_list": [{"width": 1000.0, "coordinateList": coords}],
            "sweep_line_coordinate_lists": [coords],
            "sweep_point_count": 2,
        }
        self.assertIsNotNone(build_line_sweep_definition(explicit_line))
        self.assertIsNotNone(build_line_sweep_definition(line_search_only))

    def test_line_monitor_does_not_fall_back_to_pending_line_during_attack(self) -> None:
        monitor = LineScanProgressMonitor()
        monitor._missions = {41: SimpleNamespace(aircraft_id=4)}
        monitor._aircraft_current_mission = {4: 99}
        monitor._waypoint_to_mission = {4: {}}

        self.assertIsNone(monitor._resolve_mission_id(4, current_wp=999, flying=1))

    def test_source_reversed_line_carry_preserves_the_same_physical_prefix(self) -> None:
        forward = [
            {"latitude": 38.0, "longitude": 127.0},
            {"latitude": 38.0, "longitude": 127.01},
        ]
        reversed_row = list(reversed(forward))

        def runtime(aircraft_id: int, coords: list[dict]) -> _LineMissionRuntime:
            definition = build_line_sweep_definition(
                {
                    "line_list": [{"width": 500.0, "coordinateList": coords}],
                    "input_line_list": [{"width": 1000.0, "coordinateList": coords}],
                }
            )
            self.assertIsNotNone(definition)
            return _LineMissionRuntime(
                aircraft_id=aircraft_id,
                mission_id=900000000 + aircraft_id,
                input_id=70000000,
                path_id=(aircraft_id * 100000000) + 1,
                sweep_point_count=10,
                line_def=definition,
                requires_filming_completion=True,
                is_current=True,
                source_line_rows=[{"width": 1000.0, "coordinateList": coords}],
                source_line_width_m=1000.0,
                source_coordinate_list=coords,
            )

        first = runtime(4, forward)
        second = runtime(5, reversed_row)
        first_len = float(first.line_def.planned_length_m)
        second_len = float(second.line_def.planned_length_m)
        first.state = LineSweepState(
            covered_intervals_by_line={0: [(0.0, first_len * 0.3)]},
            covered_length_m=first_len * 0.3,
        )
        # The same physical source prefix is the local suffix of a reversed row.
        second.state = LineSweepState(
            covered_intervals_by_line={0: [(second_len * 0.7, second_len)]},
            covered_length_m=second_len * 0.3,
        )

        carry = _common_input_coverage([first, second])
        self.assertIsNotNone(carry)
        self.assertEqual(carry.coordinate_space, "source_normalized")
        self.assertEqual(len(carry.normalized_intervals), 1)
        self.assertAlmostEqual(carry.normalized_intervals[0][0], 0.0, places=3)
        self.assertAlmostEqual(carry.normalized_intervals[0][1], 0.3, places=2)

        target = runtime(6, reversed_row)
        _seed_runtime_common_coverage(target, carry)
        target_intervals = target.state.covered_intervals_by_line[0]
        self.assertAlmostEqual(target_intervals[0][0] / target.line_def.planned_length_m, 0.7, places=2)
        self.assertAlmostEqual(target_intervals[0][1] / target.line_def.planned_length_m, 1.0, places=3)

    def test_successive_line_replan_keeps_current_assignment_domain(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
        ]
        assigned_coords = [
            {"latitude": 38.0, "longitude": 127.004, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
        ]
        progress = {
            400000004: {
                "missionPlanID": 700000004,
                "inputMissionID": 70000000,
                "aircraftID": 4,
                "missionID": 900000004,
                "pathID": 400000004,
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
                "lineList": [
                    {
                        "coordinateList": assigned_coords,
                        "width": 1000.0,
                        "plannedLengthM": 600.0,
                        "coveredIntervals": [{"startM": 0.0, "endM": 200.0}],
                    }
                ],
            }
        }

        detail = build_line_scan_remaining_detail(
            progress,
            source_plan_id=700000004,
            input_mission_id=70000000,
            source_detail={
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
            },
        )

        self.assertEqual(len(detail.get("lineList") or []), 1)
        remaining = detail["lineList"][0]["coordinateList"]
        # The old 0-40% prefix is outside plan 700000004's assignment and
        # must never become a second remaining fragment.
        self.assertGreater(float(remaining[0]["longitude"]), 127.0055)
        self.assertAlmostEqual(float(remaining[-1]["longitude"]), 127.01, places=6)

    def test_successive_reversed_line_maps_covered_prefix_to_source_suffix(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
        ]
        progress = {
            400000004: {
                "missionPlanID": 700000004,
                "inputMissionID": 70000000,
                "aircraftID": 4,
                "missionID": 900000004,
                "pathID": 400000004,
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
                "lineList": [
                    {
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
                            {"latitude": 38.0, "longitude": 127.004, "altitude": 100.0},
                        ],
                        "width": 1000.0,
                        "plannedLengthM": 600.0,
                        "coveredIntervals": [{"startM": 0.0, "endM": 200.0}],
                    }
                ],
            }
        }

        detail = build_line_scan_remaining_detail(
            progress,
            source_plan_id=700000004,
            input_mission_id=70000000,
            source_detail={
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
            },
        )

        self.assertEqual(len(detail.get("lineList") or []), 1)
        remaining = detail["lineList"][0]["coordinateList"]
        self.assertAlmostEqual(float(remaining[0]["longitude"]), 127.004, places=5)
        self.assertLess(float(remaining[-1]["longitude"]), 127.0085)
        self.assertGreater(float(remaining[-1]["longitude"]), 127.0075)

    def test_common_line_coverage_ignores_another_aircraft_outside_domain(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
        ]

        def completed_entry(aircraft_id: int, start_lon: float, end_lon: float) -> dict:
            row_coords = [
                {"latitude": 38.0, "longitude": float(start_lon), "altitude": 100.0},
                {"latitude": 38.0, "longitude": float(end_lon), "altitude": 100.0},
            ]
            return {
                "missionPlanID": 700000004,
                "inputMissionID": 70000000,
                "aircraftID": int(aircraft_id),
                "missionID": 900000000 + int(aircraft_id),
                "pathID": (int(aircraft_id) * 100000000) + 4,
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
                "lineList": [
                    {
                        "coordinateList": row_coords,
                        "width": 500.0,
                        "plannedLengthM": 900.0,
                        "coveredIntervals": [{"startM": 0.0, "endM": 900.0}],
                    }
                ],
            }

        detail = build_line_scan_remaining_detail(
            {
                400000004: completed_entry(4, 127.0, 127.009),
                500000004: completed_entry(5, 127.001, 127.01),
            },
            source_plan_id=700000004,
            input_mission_id=70000000,
            source_detail={
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
            },
            common_aircraft_coverage=True,
        )

        self.assertTrue(detail.get("lineRemainingCompleted"))
        self.assertFalse(has_line_remaining_geometry(detail))

    def test_common_line_coverage_waits_for_every_requested_aircraft(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
        ]
        completed_entry = {
            "missionPlanID": 700000004,
            "inputMissionID": 70000000,
            "aircraftID": 4,
            "missionID": 900000004,
            "pathID": 400000004,
            "sourceLineWidthM": 1000.0,
            "sourceCoordinateList": source_coords,
            "lineList": [
                {
                    "coordinateList": source_coords,
                    "width": 500.0,
                    "plannedLengthM": 1000.0,
                    "coveredIntervals": [{"startM": 0.0, "endM": 1000.0}],
                }
            ],
        }

        detail = build_line_scan_remaining_detail(
            {400000004: completed_entry},
            source_plan_id=700000004,
            input_mission_id=70000000,
            aircraft_ids=[4, 5],
            source_detail={
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
            },
            common_aircraft_coverage=True,
        )

        self.assertFalse(detail.get("lineRemainingCompleted"))
        self.assertTrue(has_line_remaining_geometry(detail))
        self.assertGreater(float(detail.get("lineRemainingLengthM") or 0.0), 800.0)

    def test_common_line_coverage_discards_tiny_detached_scrap(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 38.0, "longitude": 127.06, "altitude": 100.0},
        ]

        def progress_entry(aircraft_id: int) -> dict:
            return {
                "missionPlanID": 700000005,
                "inputMissionID": 70000000,
                "aircraftID": int(aircraft_id),
                "missionID": 900000000 + int(aircraft_id),
                "pathID": (int(aircraft_id) * 100000000) + 5,
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
                "lineList": [
                    {
                        "coordinateList": source_coords,
                        "width": 500.0,
                        "plannedLengthM": 5000.0,
                        # The 0-65 m disagreement is the exact shape that used
                        # to become three tiny collaborative assignments.
                        "coveredIntervals": [{"startM": 65.0, "endM": 2000.0}],
                    }
                ],
            }

        detail = build_line_scan_remaining_detail(
            {
                400000005: progress_entry(4),
                500000005: progress_entry(5),
            },
            source_plan_id=700000005,
            input_mission_id=70000000,
            source_detail={
                "sourceLineWidthM": 1000.0,
                "sourceCoordinateList": source_coords,
            },
            common_aircraft_coverage=True,
        )

        self.assertEqual(detail.get("lineRemainingFragmentCount"), 1)
        self.assertEqual(detail.get("lineRemainingDiscardedFragmentCount"), 1)
        self.assertGreater(float(detail.get("lineRemainingDiscardedFragmentLengthM") or 0.0), 60.0)
        self.assertIn("short_fragment_suppressed", str(detail.get("lineRemainingPolicy")))
        remaining = detail["lineList"][0]
        self.assertGreater(float(remaining.get("remainingStartM") or 0.0), 2000.0)
        self.assertGreater(float(remaining.get("remainingLengthM") or 0.0), 2500.0)

    def test_completed_current_line_does_not_fall_back_to_older_remaining_geometry(self) -> None:
        progress = {
            400000002: _line_progress_entry(plan_id=700000002, covered_end_m=400.0),
            400000004: _line_progress_entry(plan_id=700000004, covered_end_m=1000.0),
        }
        source_detail = {
            "sourceLineWidthM": 1000.0,
            "coordinateList": [
                {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
                {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
            ],
        }

        old_detail = build_line_scan_remaining_detail(
            progress,
            source_plan_id=700000002,
            input_mission_id=70000000,
            source_detail=source_detail,
        )
        current_detail = build_line_scan_remaining_detail(
            progress,
            source_plan_id=700000004,
            input_mission_id=70000000,
            source_detail=source_detail,
        )

        self.assertTrue(has_line_remaining_geometry(old_detail))
        self.assertFalse(has_line_remaining_geometry(current_detail))
        self.assertTrue(current_detail.get("lineRemainingCompleted"))
        self.assertEqual(current_detail.get("lineScanRequestedSourcePlanID"), 700000004)

    def test_completed_line_progress_without_exact_snapshot_stays_complete(self) -> None:
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "inputMissionType": 1,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0},
                            {"latitude": 38.0, "longitude": 127.01},
                        ],
                    },
                }
            ]
        }
        completed_detail = {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "lineRemainingCompleted": True,
            "lineRemainingPolicy": "centerline_interval_union",
            "lineScanContributingAircraftIDs": [4, 5],
        }

        with patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_input_plan_for_source_plan",
            return_value=input_plan,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.load_line_scan_remaining_detail",
            return_value=completed_detail,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline."
            "mission_area_replan_store.load_replan_ready_snapshot_entry",
            return_value=None,
        ) as snapshot_loader:
            current, next_mission = _build_remaining_input_mission_for_collaborative_replan(
                source_plan_id=700000004,
                current_input_id=70000000,
            )

        snapshot_loader.assert_called_once_with(
            700000004,
            70000000,
            allow_latest=False,
            allow_latest_area=True,
            audit_context="prior_collaborative_resume_remaining_input",
        )
        self.assertIsNone(next_mission)
        self.assertIsNotNone(current)
        self.assertTrue(current.get("isDone"))
        self.assertEqual(current.get("lineTakeoverSource"), "line_scan_progress_completed")

    def test_area_remaining_snapshot_retains_latest_fallback(self) -> None:
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000001,
                    "inputMissionType": 2,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0},
                            {"latitude": 38.0, "longitude": 127.01},
                            {"latitude": 38.01, "longitude": 127.01},
                        ],
                        "areaList": [],
                    },
                }
            ]
        }

        with patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_input_plan_for_source_plan",
            return_value=input_plan,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline."
            "mission_area_replan_store.load_replan_ready_snapshot_entry",
            return_value=None,
        ) as snapshot_loader:
            current, next_mission = _build_remaining_input_mission_for_collaborative_replan(
                source_plan_id=700000004,
                current_input_id=70000001,
            )

        snapshot_loader.assert_called_once_with(
            700000004,
            70000001,
            allow_latest=True,
            allow_latest_area=True,
            audit_context="prior_collaborative_resume_remaining_input",
        )
        self.assertIsNone(current)
        self.assertIsNone(next_mission)

    def test_exact_snapshot_remaining_line_overrides_false_completed_progress(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0},
            {"latitude": 38.0, "longitude": 127.01},
        ]
        remaining_coords = [
            {"latitude": 38.0, "longitude": 127.004},
            {"latitude": 38.0, "longitude": 127.01},
        ]
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "inputMissionType": 1,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": source_coords,
                        "lineList": [{"width": 1000, "coordinateList": source_coords}],
                    },
                }
            ]
        }
        completed_detail = {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "lineRemainingCompleted": True,
            "lineRemainingPolicy": "centerline_interval_union",
            "lineScanContributingAircraftIDs": [4, 6],
        }
        snapshot_entry = {
            "inputMissionID": 70000000,
            "missionType": "line",
            "isDone": False,
            "aircraftIDs": [4, 6],
            "sourceLineWidthM": 1000.0,
            "sourceCoordinateList": source_coords,
            "remainingDetail": {
                "coordinateList": remaining_coords,
                "lineList": [{"width": 1000, "coordinateList": remaining_coords}],
                "areaList": [],
            },
        }

        with patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_input_plan_for_source_plan",
            return_value=input_plan,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.load_line_scan_remaining_detail",
            return_value=completed_detail,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline."
            "mission_area_replan_store.load_replan_ready_snapshot_entry",
            return_value={
                "entry": snapshot_entry,
                "exact": True,
                "snapshotMissionPlanID": 700000004,
            },
        ):
            current, next_mission = _build_remaining_input_mission_for_collaborative_replan(
                source_plan_id=700000004,
                current_input_id=70000000,
                unavailable_aircraft_ids={5, 6},
            )

        self.assertIsNone(next_mission)
        self.assertIsNotNone(current)
        self.assertFalse(current.get("isDone"))
        self.assertEqual(
            current.get("lineCompletionConflictResolution"),
            "exact_snapshot_remaining",
        )
        self.assertEqual(current.get("lineTakeoverSource"), "mission_area_replan_snapshot")
        self.assertEqual(current.get("lineTakeoverUnavailableAircraftIDs"), [5, 6])
        remaining = current["missionDetail"]["lineList"][0]["coordinateList"]
        self.assertAlmostEqual(float(remaining[0]["longitude"]), 127.004, places=6)
        self.assertAlmostEqual(float(remaining[-1]["longitude"]), 127.01, places=6)

    def test_older_snapshot_cannot_revive_completed_current_line(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0},
            {"latitude": 38.0, "longitude": 127.01},
        ]
        input_plan = {
            "inputMissionList": [
                {
                    "inputMissionID": 70000000,
                    "inputMissionType": 1,
                    "isDone": False,
                    "missionDetail": {
                        "coordinateList": source_coords,
                        "lineList": [{"width": 1000, "coordinateList": source_coords}],
                    },
                }
            ]
        }
        completed_detail = {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "lineRemainingCompleted": True,
            "lineRemainingPolicy": "centerline_interval_union",
            "lineScanContributingAircraftIDs": [4],
        }
        older_entry = {
            "inputMissionID": 70000000,
            "missionType": "line",
            "isDone": False,
            "remainingDetail": {
                "coordinateList": source_coords,
                "lineList": [{"width": 1000, "coordinateList": source_coords}],
                "areaList": [],
            },
        }

        with patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline._load_input_plan_for_source_plan",
            return_value=input_plan,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.load_line_scan_remaining_detail",
            return_value=completed_detail,
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline."
            "mission_area_replan_store.load_replan_ready_snapshot_entry",
            return_value={
                "entry": older_entry,
                "exact": False,
                "snapshotMissionPlanID": 700000002,
            },
        ):
            current, next_mission = _build_remaining_input_mission_for_collaborative_replan(
                source_plan_id=700000004,
                current_input_id=70000000,
            )

        self.assertIsNone(next_mission)
        self.assertIsNotNone(current)
        self.assertTrue(current.get("isDone"))
        self.assertEqual(current.get("lineTakeoverSource"), "line_scan_progress_completed")
        self.assertNotIn("lineCompletionConflictResolution", current)

    def test_follow_up_attack_uses_current_plan_not_previous_attack_lineage(self) -> None:
        source_plan_id = _resolve_attack_source_plan_id(
            {
                "sourceMissionPlanID": 700000002,
                "currentMissionPlanID": 700000004,
            },
            {
                "sourceMissionPlanID": 700000002,
                "currentMissionPlanID": 700000004,
                "followUpAttackMode": True,
            },
        )

        self.assertEqual(source_plan_id, 700000004)

    def test_attack_request_current_plan_overrides_stale_staged_context(self) -> None:
        source_plan_id = _resolve_attack_source_plan_id(
            {
                "sourceMissionPlanID": 700000002,
                "currentMissionPlanID": 700000002,
            },
            {
                "sourceMissionPlanID": 700000004,
                "currentMissionPlanID": 700000004,
            },
        )

        self.assertEqual(source_plan_id, 700000004)

    def test_post_attack_missing_current_snapshot_does_not_revive_line(self) -> None:
        self.assertFalse(
            _allow_post_attack_active_only_replan("remaining_snapshot_unavailable")
        )

    def test_post_attack_completed_snapshot_is_not_reported_as_unavailable(self) -> None:
        snapshot = {
            "exact": True,
            "entry": {
                "inputMissionID": 70000008,
                "missionType": "line",
                "isDone": True,
                "remainingAreaM2": 0.0,
                "remainingDetail": {
                    "coordinateList": [],
                    "lineList": [],
                    "areaList": [],
                },
            },
        }
        with patch(
            "modules.mission_planning.replanning.triggers.post_attack.pipeline."
            "mission_area_replan_store.load_snapshot_entry",
            return_value=snapshot,
        ):
            self.assertTrue(
                _remaining_snapshot_explicitly_completed(700000032, 70000008)
            )

    def test_post_attack_other_safe_skip_can_still_replan_active_group(self) -> None:
        self.assertTrue(_allow_post_attack_active_only_replan("rejoin_reference_unavailable"))

    def test_line_progress_100_creates_completion_loiter_boundary(self) -> None:
        self.assertTrue(_active_progress_is_complete(100))
        self.assertTrue(
            _include_active_completion_boundary_hold(
                100,
                preserve_current_mission=False,
                pending_area_pass_reassignment=False,
            )
        )

    def test_line_progress_below_100_does_not_force_completion_loiter(self) -> None:
        self.assertFalse(_active_progress_is_complete(99))
        self.assertFalse(
            _include_active_completion_boundary_hold(
                99,
                preserve_current_mission=False,
                pending_area_pass_reassignment=False,
            )
        )
        self.assertFalse(
            _include_active_completion_boundary_hold(
                99,
                preserve_current_mission=True,
                pending_area_pass_reassignment=False,
            )
        )

    def test_authoritative_completion_does_not_revive_progress_99_line(self) -> None:
        self.assertTrue(
            _should_preserve_active_current_mission(
                path_all_done=True,
                completed_by_on_mission=False,
                progress_percent=99,
                remaining_snapshot_completed=False,
            )
        )
        self.assertFalse(
            _should_preserve_active_current_mission(
                path_all_done=True,
                completed_by_on_mission=False,
                progress_percent=99,
                remaining_snapshot_completed=True,
            )
        )

    def test_attack_exhausted_line_discards_forced_flyby_for_five_second_loiter(
        self,
    ) -> None:
        capture_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 0},
            {"latitude": 38.001, "longitude": 127.0, "altitude": 0},
        ]
        source_path = {
            "aircraftID": 4,
            "pathID": 400000001,
            "waypointList": [
                {
                    "waypointID": 10,
                    "coordinate": {
                        "latitude": 38.0005,
                        "longitude": 127.001,
                        "altitude": 1200,
                    },
                    "filmingProperty": {
                        "operationMode": 2,
                        "lineSearch": {
                            "coordinateList": capture_coords,
                            "searchSpeed": 20,
                        },
                    },
                    "isDone": True,
                },
                {
                    "waypointID": 11,
                    "coordinate": {
                        "latitude": 38.2,
                        "longitude": 127.2,
                        "altitude": 1200,
                    },
                    "waypointPassType": 1,
                    "filmingProperty": {"operationMode": 1},
                    "isDone": False,
                },
            ],
        }
        allocated = iter(range(5000, 5010))

        _done, resume, _removed = _split_done_resume_path(
            source_path,
            artifacts=SimpleNamespace(
                path_id=400000001,
                current_waypoint_id=11,
                previous_waypoint_id=10,
            ),
            sweep_progress=None,
            emit=lambda _message: None,
            force_nonempty_resume=True,
            waypoint_id_provider=lambda: next(allocated),
        )

        self.assertEqual(resume, [])
        attack_coord = {
            "latitude": 38.01,
            "longitude": 127.02,
            "altitude": 1100,
        }
        hold = _build_uav_attack_completion_hold_waypoint(
            source_path,
            waypoint_id=6000,
            fallback_coordinate=attack_coord,
        )
        self.assertIsNotNone(hold)
        self.assertEqual(hold["coordinate"], attack_coord)
        self.assertEqual(hold["waypointPassType"], 2)
        self.assertEqual(hold["loiterProperty"]["time"], 5)
        self.assertEqual(hold["loiterProperty"]["radius"], 180)
        self.assertTrue(hold["noCaptureCompletionLoiter"])
        self.assertNotIn("lineSearch", hold["filmingProperty"])

    def test_attack_empty_resume_creates_last_capture_point_loiter(self) -> None:
        waypoint = _build_uav_attack_completion_hold_waypoint(
            {
                "waypointList": [
                    {
                        "waypointID": 10,
                        "coordinate": {
                            "latitude": 38.0,
                            "longitude": 127.0,
                            "altitude": 1500,
                        },
                        "filmingProperty": {
                            "operationMode": 2,
                            "lineSearch": {
                                "coordinateList": [
                                    {
                                        "latitude": 38.01,
                                        "longitude": 127.02,
                                        "altitude": 200,
                                    }
                                ]
                            },
                        },
                    }
                ]
            },
            waypoint_id=99,
            hold_seconds=300,
        )

        self.assertIsNotNone(waypoint)
        self.assertEqual(waypoint["waypointID"], 99)
        self.assertEqual(
            waypoint["coordinate"],
            {"latitude": 38.01, "longitude": 127.02, "altitude": 1500},
        )
        self.assertEqual(waypoint["waypointPassType"], 2)
        self.assertFalse(waypoint["isDone"])
        self.assertEqual(waypoint["loiterProperty"]["time"], 300)
        self.assertNotIn("lineSearch", waypoint["filmingProperty"])
        self.assertTrue(waypoint["attackCompletionBoundaryHold"])

    def test_attack_completed_uav_retains_next_input_mission_blocked(self) -> None:
        class _FakeIds:
            def __init__(self) -> None:
                self.path_id = 500000100
                self.individual_id = 900000100
                self.waypoint_id = 9900

            def next_paths(self, _aircraft_id: int, count: int) -> list[int]:
                values = list(range(self.path_id, self.path_id + count))
                self.path_id += count
                return values

            def next_individuals(self, count: int) -> list[int]:
                values = list(range(self.individual_id, self.individual_id + count))
                self.individual_id += count
                return values

            def next_individual(self) -> int:
                return self.next_individuals(1)[0]

            def next_path(self, aircraft_id: int) -> int:
                return self.next_paths(aircraft_id, 1)[0]

            def next_waypoint(self) -> int:
                value = self.waypoint_id
                self.waypoint_id += 1
                return value

        current_mission = {
            "individualMissionID": 900000001,
            "isDone": False,
            "relatedMission": {"inputMissionID": 70000001, "relatedMissionType": 1},
            "individualMissionInfo": {
                "individualMissionType": 6,
                "patternType": 8,
                "targetID": None,
                "lineList": [{"coordinateList": []}],
            },
            "pathID": 500000001,
        }
        next_mission = {
            "individualMissionID": 900000002,
            "isDone": False,
            "relatedMission": {"inputMissionID": 70000002, "relatedMissionType": 1},
            "individualMissionInfo": {"individualMissionType": 3, "targetID": None},
            "pathID": 500000002,
        }
        source_waypoint = {
            "waypointID": 10,
            "coordinate": {"latitude": 38.0, "longitude": 127.0, "altitude": 1500},
            "isDone": True,
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": [
                        {"latitude": 38.01, "longitude": 127.02, "altitude": 200}
                    ]
                },
            },
        }
        imp_data = {
            "individualMissionPackageID": 800000001,
            "individualMissionList": [current_mission, next_mission],
        }
        fp_data = {
            "pathID": 500000001,
            "aircraftID": 5,
            "individualMissionID": 900000001,
            "waypointList": [source_waypoint],
        }

        with tempfile.TemporaryDirectory() as tmp_dir, patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "_split_done_resume_path",
            return_value=([source_waypoint], [], 10),
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "_attack_follow_up_can_preserve",
            return_value=True,
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "_validate_generated_artifact_write_entries"
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "_write_or_defer_attack_json_entries",
            side_effect=lambda entries, **_kwargs: ([], list(entries)),
        ), patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "db_paths.get_db_subpath",
            side_effect=lambda kind, filename=None: (
                Path(tmp_dir) / kind / filename
                if filename
                else Path(tmp_dir) / kind
            ),
        ):
            result = _build_uav_attack_resume_package(
                descriptor={"aircraft_id": 5, "label": "uav_resume_5"},
                new_imp_id=800000010,
                imp_data=imp_data,
                fp_data=fp_data,
                target_mission_template=current_mission,
                target_index=0,
                ctx={"mission_ids": [70000001]},
                state={
                    "coordinate": {
                        "latitude": 38.0,
                        "longitude": 127.0,
                        "altitude": 1500,
                    }
                },
                artifacts=SimpleNamespace(source_plan_id=700000009),
                emit=lambda _message: None,
                now_ms=1,
                done_input_ids=set(),
                id_reservation=_FakeIds(),
                defer_write=True,
            )

        self.assertIsNotNone(result)
        self.assertTrue(result["completionBoundaryHold"])
        self.assertEqual(result["followUpMissionCount"], 1)
        rebuilt = imp_data["individualMissionList"]
        self.assertEqual(len(rebuilt), 3)
        self.assertTrue(rebuilt[0]["isDone"])
        self.assertTrue(rebuilt[1]["attackCompletionBoundaryHold"])
        self.assertNotIn("executionBlockedUntilNextCollab", rebuilt[1])
        self.assertEqual(
            rebuilt[1]["relatedMission"]["inputMissionID"],
            70000001,
        )
        self.assertEqual(
            rebuilt[2]["relatedMission"]["inputMissionID"],
            70000002,
        )
        self.assertTrue(rebuilt[2]["executionBlockedUntilNextCollab"])

    def test_completed_collaboration_blocks_followups_until_next_handoff(self) -> None:
        self.assertTrue(
            _block_post_attack_followups_until_next_collab(
                current_mission_completed=True,
                pending_area_pass_reassignment=False,
            )
        )
        self.assertFalse(
            _block_post_attack_followups_until_next_collab(
                current_mission_completed=False,
                pending_area_pass_reassignment=False,
            )
        )
        self.assertTrue(
            _block_post_attack_followups_until_next_collab(
                current_mission_completed=False,
                pending_area_pass_reassignment=False,
                type2_branch_group=True,
            )
        )

    def test_post_attack_recovers_tracking_branch_from_selected_plan(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            for kind in ("MissionPlan", "IndividualMissionPlan", "FlightPath"):
                (root / kind).mkdir(parents=True, exist_ok=True)

            (root / "MissionPlan" / "700000011.json").write_text(
                json.dumps(
                    {
                        "aircraftList": [
                            {"aircraftID": 4, "individualMissionPackageID": 800000047}
                        ]
                    }
                ),
                encoding="utf-8",
            )
            (root / "IndividualMissionPlan" / "800000047.json").write_text(
                json.dumps(
                    {
                        "individualMissionList": [
                            {
                                "individualMissionID": 900000131,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 70000001},
                                "individualMissionInfo": {
                                    "individualMissionType": 1,
                                    "targetID": 9,
                                },
                                "pathID": 400000022,
                            },
                            {
                                "individualMissionID": 900000132,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 70000001},
                                "individualMissionInfo": {
                                    "individualMissionType": 7,
                                    "targetID": None,
                                },
                                "pathID": 400000023,
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            for path_id, latitude in ((400000022, 38.0), (400000023, 38.1)):
                (root / "FlightPath" / f"{path_id}.json").write_text(
                    json.dumps(
                        {
                            "waypointList": [
                                {
                                    "waypointID": path_id,
                                    "isDone": False,
                                    "coordinate": {
                                        "latitude": latitude,
                                        "longitude": 127.0,
                                        "altitude": 1500,
                                    },
                                }
                            ]
                        }
                    ),
                    encoding="utf-8",
                )

            def _db_path(kind: str, filename: str | None = None) -> Path:
                directory = root / kind
                return directory / filename if filename else directory

            with patch(
                "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                "db_paths.get_db_subpath",
                side_effect=_db_path,
            ):
                recovered = _recover_tracking_assignments_from_current_plan(
                    current_plan_id=700000011,
                    target_id=9,
                    watcher_id=4,
                )

        self.assertEqual(len(recovered), 1)
        self.assertEqual(recovered[0]["aircraft_id"], 4)
        self.assertEqual(recovered[0]["tracking_path_id"], 400000022)
        self.assertEqual(recovered[0]["resume_path_id"], 400000023)
        self.assertEqual(recovered[0]["current_input_mission_id"], 70000001)
        self.assertTrue(recovered[0]["recovered_from_plan_artifacts"])

    def test_line_spacing_absorbs_short_trailing_group(self) -> None:
        selected = _select_line_sweep_items_by_spacing(
            _line_items(0.0, 1000.0, 2000.0, 3000.0, 4000.0, 4760.0),
            {"lineRouteWpSpacingM": 2000.0},
            reference_xy=None,
        )

        self.assertEqual(
            [round(float(item["anchorXY"][0])) for item in selected],
            [0, 2000, 4760],
        )

    def test_line_spacing_keeps_exact_full_groups(self) -> None:
        selected = _select_line_sweep_items_by_spacing(
            _line_items(0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0),
            {"lineRouteWpSpacingM": 2000.0},
            reference_xy=None,
        )

        self.assertEqual(
            [round(float(item["anchorXY"][0])) for item in selected],
            [0, 2000, 4000, 6000],
        )

    def test_line_deployment_lock_prevents_entry_position_reversal(self) -> None:
        selected = _select_line_sweep_items_by_spacing(
            _line_items(0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0),
            {
                "lineRouteWpSpacingM": 2000.0,
                "lineDeploymentDirectionLocked": True,
            },
            reference_xy=(6000.0, 0.0),
        )

        self.assertEqual(
            [round(float(item["anchorXY"][0])) for item in selected],
            [0, 2000, 4000, 6000],
        )

    def test_line_deployment_lock_keeps_trimmed_start_without_restoring_prefix(self) -> None:
        selected = _select_line_sweep_items_by_spacing(
            _line_items(3000.0, 4000.0, 5000.0, 6000.0, 7000.0),
            {
                "lineRouteWpSpacingM": 2000.0,
                "lineDeploymentDirectionLocked": True,
            },
            reference_xy=(7000.0, 0.0),
        )

        self.assertEqual(
            [round(float(item["anchorXY"][0])) for item in selected],
            [3000, 5000, 7000],
        )

    def test_trimmed_line_fragment_is_oriented_to_initial_deployment(self) -> None:
        oriented = _orient_polyline_xy_to_direction(
            [(80.0, 0.0), (50.0, 0.0), (20.0, 0.0)],
            direction_xy=[(0.0, 0.0), (100.0, 0.0)],
        )

        self.assertEqual(oriented, [(20.0, 0.0), (50.0, 0.0), (80.0, 0.0)])

    def test_initial_line_lock_uses_first_generated_execution_direction_not_raw_input_order(self) -> None:
        source_coords = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100},
            {"latitude": 38.0, "longitude": 127.1, "altitude": 100},
        ]
        piece = SplitPiece(
            parent_order=0,
            mission_id=70000000,
            mission_type=1,
            piece_index=0,
            data={"Centerline": source_coords},
            assigned_uav=4,
        )

        direction_xy, deployment_coords = _derive_initial_line_deployment_reference(
            source_coords,
            piece,
            {"waypointStartXY": coord_to_xy(source_coords[-1])},
        )

        self.assertEqual(
            [round(float(coord["longitude"]), 6) for coord in deployment_coords],
            [127.1, 127.0],
        )
        self.assertGreater(float(direction_xy[0][0]), float(direction_xy[-1][0]))

    def test_line_deployment_template_prefers_explicit_first_plan_reference(self) -> None:
        explicit = [
            {"latitude": 38.0, "longitude": 127.0, "altitude": 100},
            {"latitude": 38.0, "longitude": 127.2, "altitude": 100},
        ]
        records = {
            4: [
                {
                    "mission": {
                        "individualMissionInfo": {
                            "lineDeploymentCoordinateList": explicit,
                            "lineList": [
                                {
                                    "coordinateList": list(reversed(explicit)),
                                }
                            ],
                        }
                    },
                    "flightPath": {
                        "waypointList": [
                            self._line_search_waypoint(127.2),
                            self._line_search_waypoint(127.0),
                        ]
                    },
                }
            ]
        }

        resolved = _resolve_line_deployment_coordinate_list_from_templates(records, [4])

        self.assertEqual(
            [float(coord["longitude"]) for coord in resolved],
            [127.0, 127.2],
        )

    def test_legacy_line_template_uses_actual_flight_path_order(self) -> None:
        records = {
            4: [
                {
                    "mission": {
                        "individualMissionInfo": {
                            "lineList": [
                                {
                                    "coordinateList": [
                                        {"latitude": 38.0, "longitude": 127.2, "altitude": 100},
                                        {"latitude": 38.0, "longitude": 127.0, "altitude": 100},
                                    ]
                                }
                            ]
                        }
                    },
                    "flightPath": {
                        "waypointList": [
                            self._line_search_waypoint(127.0),
                            self._line_search_waypoint(127.2),
                        ]
                    },
                }
            ]
        }

        resolved = _resolve_line_deployment_coordinate_list_from_templates(records, [4])

        self.assertEqual(
            [round(float(coord["longitude"]), 6) for coord in resolved],
            [127.0, 127.2],
        )

    @staticmethod
    def _line_search_waypoint(center_longitude: float) -> dict:
        return {
            "filmingProperty": {
                "lineSearch": {
                    "coordinateList": [
                        {"latitude": 37.999, "longitude": float(center_longitude), "altitude": 100},
                        {"latitude": 38.001, "longitude": float(center_longitude), "altitude": 100},
                    ]
                }
            }
        }


if __name__ == "__main__":
    unittest.main()
