from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

from modules.mission_status_monitoring.receiver import ReadOnly0401Integration
from modules.mission_status_monitoring.service import MissionStatusService


class _Integration:
    def __init__(self, packets: list[dict]) -> None:
        self._packets = list(packets)

    def drain_0602_events(self) -> list[dict]:
        packets = list(self._packets)
        self._packets.clear()
        return packets


def _context() -> dict:
    return {
        "timestamp": 837_919_604_350,
        "positions": {
            "4": {
                "latitude": 37.914,
                "longitude": 128.164,
                "altitude": 744,
            }
        },
    }


def _packet(payload: dict) -> dict:
    return {
        "arrivalUnixMs": 1_784_604_404_500,
        "payload": payload,
        "aircraftContext": _context(),
    }


def _payload() -> dict:
    return {
        "timestamp": 837_919_604_398,
        "source": "UCC",
        "uavCommandModeType": 3,
        "aircraftID": 4,
        "flightModeCommand": {
            "flightMode": 7,
            "pathFollowing": {"startWaypointID": 50},
        },
        "filmingModeCommand": {
            "operationMode": 2,
            "sensorType": 1,
            "fieldOfView": 3.2,
            "lineSearch": {"coordinateList": []},
        },
    }


def test_receiver_subscribes_to_0602() -> None:
    assert "0602" in ReadOnly0401Integration._READ_MESSAGE_IDS


def test_0602_command_is_persisted_with_position_and_icd_labels(
    tmp_path: Path,
) -> None:
    integration = _Integration([_packet(_payload())])

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        service = MissionStatusService(integration=integration)
        service._ensure_commands_loaded(str(tmp_path.resolve()))
        service._consume_0602_events()

        rows = list(service._uav_commands)
        assert len(rows) == 1
        row = rows[0]
        assert row["timeKst"] == "12:26:44.398"
        assert row["uavLabel"] == "UAV1"
        assert row["flightCommandText"] == "7 : 경로이동비행"
        assert row["filmingCommandText"] == "2 : 구간탐색 모드"
        assert row["sensorTypeName"] == "EO"
        assert row["position"] == {
            "latitude": 37.914,
            "longitude": 128.164,
            "altitude": 744.0,
        }
        assert row["positionTimestamp"] == 837_919_604_350

        history_path = (
            tmp_path / "DSS_Internal" / "mission_status_0602_commands.jsonl"
        )
        persisted = [
            json.loads(line)
            for line in history_path.read_text(encoding="utf-8").splitlines()
        ]
        assert persisted == rows


def test_duplicate_0602_packet_is_recorded_once(tmp_path: Path) -> None:
    payload = _payload()
    integration = _Integration([_packet(payload), _packet(payload)])

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        service = MissionStatusService(integration=integration)
        service._ensure_commands_loaded(str(tmp_path.resolve()))
        service._consume_0602_events()

    assert len(service._uav_commands) == 1
