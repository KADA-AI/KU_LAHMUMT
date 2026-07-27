from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from modules.mission_status_monitoring.service import MissionStatusService


def _feature(
    *,
    input_id: int,
    mission_kind: str = "area",
    role: str | None = None,
    geometry_source: str = "remainingDetail",
) -> dict:
    properties = {
        "inputMissionID": input_id,
        "missionKind": mission_kind,
        "geometrySource": geometry_source,
        "isDone": 0,
    }
    if role is not None:
        properties["visualizationRole"] = role
    return {
        "type": "Feature",
        "geometry": {
            "type": "Polygon",
            "coordinates": [
                [
                    [127.0, 38.0],
                    [127.01, 38.0],
                    [127.01, 38.01],
                    [127.0, 38.0],
                ]
            ],
        },
        "properties": properties,
    }


class RemainingAreaVisualizationTests(unittest.TestCase):
    def _snapshot_path(self, root: Path, plan_id: int) -> Path:
        path = (
            root
            / "DSS_Internal"
            / "mission_area_replan"
            / f"mission_area_snapshot_{plan_id}.json"
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}", encoding="utf-8")
        return path

    def test_separates_live_area_from_line_depth_and_pass_features(self):
        plan_id = 700000101
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._snapshot_path(root, plan_id)
            live_area = _feature(input_id=101)
            line_ribbon = _feature(
                input_id=102,
                mission_kind="line",
                geometry_source="lineRemainingDetail",
            )
            depth = _feature(input_id=103, role="coverageDepth")
            depth["properties"]["coverageDepth"] = 1
            pass_area = _feature(input_id=103, role="coveragePassAttribution")
            pass_area["properties"]["coveragePass"] = "forward"
            payload = {
                "available": True,
                "missionPlanID": plan_id,
                "dataRevision": "revision-1",
                "featureCollection": {
                    "type": "FeatureCollection",
                    "features": [live_area, line_ribbon, depth, pass_area],
                },
            }
            service = MissionStatusService(integration=None)
            service._mission_signature = "mission-1"

            with (
                patch(
                    "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
                    return_value=root,
                ),
                patch(
                    "modules.mission_status_monitoring.service.build_remaining_area_snapshot",
                    return_value=payload,
                ),
            ):
                result = service._remaining_coverage_geometry(plan_id)

            self.assertEqual(1, len(result["remainingAreas"]["features"]))
            self.assertEqual("remainingDetail", result["remainingAreas"]["features"][0]["properties"]["geometrySource"])
            self.assertEqual(1, len(result["coverageDepth"]["features"]))
            self.assertEqual(1, len(result["coveragePassAttribution"]["features"]))

    def test_legacy_remaining_area_does_not_fabricate_complete_depth(self):
        plan_id = 700000102
        input_id = 201
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._snapshot_path(root, plan_id)
            payload = {
                "available": True,
                "missionPlanID": plan_id,
                "dataRevision": "revision-2",
                "featureCollection": {
                    "type": "FeatureCollection",
                    "features": [_feature(input_id=input_id)],
                },
            }
            service = MissionStatusService(integration=None)
            service._mission_signature = "mission-2"
            service._mission_geometry["inputAreas"] = {
                "type": "FeatureCollection",
                "features": [_feature(input_id=input_id)],
            }

            with (
                patch(
                    "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
                    return_value=root,
                ),
                patch(
                    "modules.mission_status_monitoring.service.build_remaining_area_snapshot",
                    return_value=payload,
                ),
            ):
                result = service._remaining_coverage_geometry(plan_id)

            self.assertEqual([], result["coverageDepth"]["features"])

    def test_exact_plan_guard_clears_mismatched_snapshot(self):
        requested_plan_id = 700000103
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._snapshot_path(root, requested_plan_id)
            payload = {
                "available": True,
                "missionPlanID": requested_plan_id - 1,
                "dataRevision": "stale",
                "featureCollection": {
                    "type": "FeatureCollection",
                    "features": [_feature(input_id=301)],
                },
            }
            service = MissionStatusService(integration=None)
            service._mission_signature = "mission-3"

            with (
                patch(
                    "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
                    return_value=root,
                ),
                patch(
                    "modules.mission_status_monitoring.service.build_remaining_area_snapshot",
                    return_value=payload,
                ),
            ):
                result = service._remaining_coverage_geometry(requested_plan_id)

            self.assertEqual("", result["revision"])
            self.assertEqual([], result["remainingAreas"]["features"])

    def test_unchanged_snapshot_is_not_reparsed_each_poll(self):
        plan_id = 700000104
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshot_path = self._snapshot_path(root, plan_id)
            payload = {
                "available": True,
                "missionPlanID": plan_id,
                "dataRevision": "revision-4",
                "featureCollection": {
                    "type": "FeatureCollection",
                    "features": [_feature(input_id=401)],
                },
            }
            service = MissionStatusService(integration=None)
            service._mission_signature = "mission-4"

            with (
                patch(
                    "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
                    return_value=root,
                ),
                patch(
                    "modules.mission_status_monitoring.service.build_remaining_area_snapshot",
                    return_value=payload,
                ) as build,
            ):
                first = service._remaining_coverage_geometry(plan_id)
                second = service._remaining_coverage_geometry(plan_id)
                snapshot_path.write_text('{"changed":true}', encoding="utf-8")
                third = service._remaining_coverage_geometry(plan_id)

            self.assertIs(first, second)
            self.assertIsNot(second, third)
            self.assertEqual(2, build.call_count)

    def test_plan_transition_response_replaces_live_area_with_empty_collection(self):
        service = MissionStatusService(integration=None)
        service._mission_signature = "mission-a"
        service._mission = {"missionPlanID": 700000201}
        live_area = _feature(input_id=501)

        def remaining(plan_id: int) -> dict:
            features = [live_area] if plan_id == 700000201 else []
            return {
                "revision": f"revision-{plan_id}",
                "remainingAreas": {
                    "type": "FeatureCollection",
                    "features": features,
                },
                "coverageDepth": {"type": "FeatureCollection", "features": []},
                "coveragePassAttribution": {
                    "type": "FeatureCollection",
                    "features": [],
                },
            }

        with patch.object(service, "_remaining_coverage_geometry", side_effect=remaining):
            first = service.mission()
            service._mission_signature = "mission-b"
            service._mission = {"missionPlanID": 700000202}
            second = service.mission(first["signature"])

        self.assertEqual(1, len(first["geojson"]["remainingAreas"]["features"]))
        self.assertTrue(second["changed"])
        self.assertEqual([], second["geojson"]["remainingAreas"]["features"])


if __name__ == "__main__":
    unittest.main()
