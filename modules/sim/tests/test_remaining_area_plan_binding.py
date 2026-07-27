from __future__ import annotations

import tempfile
import unittest
from http import HTTPStatus
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from modules.sim.mission import remaining_area_loader
from modules.sim.runtime.sim_service import SimulationService
from modules.sim.server import http_server


def _line_mission(*, done: bool) -> dict:
    return {
        "inputMissionID": 70000001,
        "missionType": "line",
        "isDone": done,
        "sourceLineWidthM": 300.0,
        "remainingDetail": {
            "lineList": [
                {
                    "width": 300.0,
                    "coordinateList": [
                        {"latitude": 38.0, "longitude": 127.0},
                        {"latitude": 38.0, "longitude": 127.01},
                    ],
                }
            ]
        },
    }


def _area_ring(x0: float, y0: float, x1: float, y1: float) -> dict:
    return {
        "isHole": False,
        "coordinateList": [
            {"longitude": x0, "latitude": y0},
            {"longitude": x1, "latitude": y0},
            {"longitude": x1, "latitude": y1},
            {"longitude": x0, "latitude": y1},
        ],
    }


class RemainingAreaLoaderPlanBindingTest(unittest.TestCase):
    def test_owner_only_area_fallback_is_one_shared_logical_feature(self) -> None:
        mission = {
            "inputMissionID": 3,
            "missionType": "area",
            "isDone": False,
            "remainingDetail": {"areaList": []},
            "aircraftIDs": [4, 5, 6],
            "areaOwnershipDetails": [
                {
                    "aircraftID": 4,
                    "remainingDetail": {
                        "areaList": [_area_ring(127.0, 38.0, 127.01, 38.01)]
                    },
                },
                {
                    "aircraftID": 5,
                    "remainingDetail": {
                        "areaList": [_area_ring(127.01, 38.0, 127.02, 38.01)]
                    },
                },
            ],
        }

        features = remaining_area_loader._features_from_snapshot(
            {"missionPlanID": 44, "missions": [mission]}
        )

        self.assertEqual(len(features), 1)
        self.assertEqual(
            features[0]["properties"]["geometrySource"],
            "areaOwnershipUnionFallback",
        )
        self.assertIsNone(features[0]["properties"]["aircraftID"])
        self.assertIn(features[0]["geometry"]["type"], {"Polygon", "MultiPolygon"})

    def test_missing_plan_id_never_selects_latest_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            snapshot_dir = Path(temp_dir)
            (snapshot_dir / "mission_area_snapshot_999.json").write_text(
                "{}", encoding="utf-8"
            )
            with mock.patch.object(
                remaining_area_loader, "_snapshot_dir", return_value=snapshot_dir
            ), mock.patch.object(
                remaining_area_loader.mission_area_replan_store, "load_snapshot"
            ) as load_snapshot:
                snapshot, path, mtime = remaining_area_loader._load_snapshot(None)

        self.assertIsNone(snapshot)
        self.assertIsNone(path)
        self.assertIsNone(mtime)
        load_snapshot.assert_not_called()

    def test_explicit_plan_id_loads_only_its_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            snapshot_dir = Path(temp_dir)
            expected_path = snapshot_dir / "mission_area_snapshot_44.json"
            expected_path.write_text("{}", encoding="utf-8")
            expected_snapshot = {"missionPlanID": 44, "missions": []}
            with mock.patch.object(
                remaining_area_loader, "_snapshot_dir", return_value=snapshot_dir
            ), mock.patch.object(
                remaining_area_loader.mission_area_replan_store,
                "load_snapshot",
                return_value=expected_snapshot,
            ) as load_snapshot:
                snapshot, path, mtime = remaining_area_loader._load_snapshot(44)

        self.assertEqual(snapshot, expected_snapshot)
        self.assertEqual(path, expected_path)
        self.assertIsInstance(mtime, float)
        load_snapshot.assert_called_once_with(44)

    def test_completed_line_is_not_rendered(self) -> None:
        completed = remaining_area_loader._features_from_snapshot(
            {"missionPlanID": 44, "missions": [_line_mission(done=True)]}
        )
        active = remaining_area_loader._features_from_snapshot(
            {"missionPlanID": 44, "missions": [_line_mission(done=False)]}
        )

        self.assertEqual(completed, [])
        self.assertGreater(len(active), 0)
        self.assertTrue(
            all(
                feature.get("properties", {}).get("missionKind") == "line"
                for feature in active
            )
        )


class RemainingAreaServerPlanBindingTest(unittest.TestCase):
    def _handler(self, path: str, sim: object):
        handler = object.__new__(http_server.MapRequestHandler)
        handler.path = path
        handler.server = SimpleNamespace(sim=sim)
        handler._send_json = mock.Mock()
        return handler

    def test_omitted_query_uses_simulator_applied_plan(self) -> None:
        sim = SimpleNamespace(
            current_mission_plan_id=mock.Mock(return_value=73),
        )
        handler = self._handler("/api/sim/remaining_areas", sim)
        response = {"ok": True, "missionPlanID": 73}

        with mock.patch.object(
            http_server, "build_remaining_area_snapshot", return_value=response
        ) as build_snapshot:
            handler._serve_sim_get("/api/sim/remaining_areas")

        sim.current_mission_plan_id.assert_called_once_with()
        build_snapshot.assert_called_once_with(mission_plan_id=73)
        handler._send_json.assert_called_once_with(response)

    def test_explicit_query_does_not_use_simulator_fallback(self) -> None:
        sim = SimpleNamespace(
            current_mission_plan_id=mock.Mock(return_value=73),
        )
        handler = self._handler("/api/sim/remaining_areas?missionPlanID=91", sim)

        with mock.patch.object(
            http_server,
            "build_remaining_area_snapshot",
            return_value={"ok": True, "missionPlanID": 91},
        ) as build_snapshot:
            handler._serve_sim_get("/api/sim/remaining_areas")

        sim.current_mission_plan_id.assert_not_called()
        build_snapshot.assert_called_once_with(mission_plan_id=91)

    def test_invalid_explicit_query_is_rejected(self) -> None:
        sim = SimpleNamespace(current_mission_plan_id=mock.Mock(return_value=73))
        handler = self._handler("/api/sim/remaining_areas?missionPlanID=invalid", sim)

        with mock.patch.object(http_server, "build_remaining_area_snapshot") as build_snapshot:
            handler._serve_sim_get("/api/sim/remaining_areas")

        build_snapshot.assert_not_called()
        handler._send_json.assert_called_once_with(
            {"ok": False, "error": "missionPlanID must be an integer."},
            HTTPStatus.BAD_REQUEST,
        )


class SimulationMissionPlanStateTest(unittest.TestCase):
    def test_current_plan_id_is_exposed_in_state_snapshot(self) -> None:
        service = SimulationService()
        self.addCleanup(service.shutdown)
        service._loaded_mission_plan_id = 81

        self.assertEqual(service.current_mission_plan_id(), 81)
        self.assertEqual(service.build_snapshot()["missionPlanID"], 81)


if __name__ == "__main__":
    unittest.main()
