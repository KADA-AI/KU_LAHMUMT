from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

from modules.mission_status_monitoring.footprint_context import (
    build_0401_footprint_context,
)
from modules.mission_status_monitoring.receiver import ReadOnly0401Integration
from modules.mission_status_monitoring.service import MissionStatusService


class _Integration:
    def __init__(self, packets: list[dict]) -> None:
        self._packets = list(packets)

    def drain_0402_events(self) -> list[dict]:
        packets = list(self._packets)
        self._packets.clear()
        return packets


def _footprint_context(*aircraft_ids: int) -> dict:
    corners = [
        {"latitude": 37.91, "longitude": 128.16, "altitude": 744},
        {"latitude": 37.91, "longitude": 128.17, "altitude": 744},
        {"latitude": 37.90, "longitude": 128.17, "altitude": 744},
        {"latitude": 37.90, "longitude": 128.16, "altitude": 744},
    ]
    return {
        "timestamp": 837_919_604_350,
        "byAircraft": {str(aircraft_id): corners for aircraft_id in aircraft_ids},
    }


def _packet(
    payload: dict,
    arrival_unix_ms: int = 1_784_604_404_500,
    footprint_context: dict | None = None,
) -> dict:
    return {
        "arrivalUnixMs": arrival_unix_ms,
        "payload": payload,
        "footprintContext": footprint_context or {},
    }


def test_receiver_subscribes_to_0402() -> None:
    assert "0402" in ReadOnly0401Integration._READ_MESSAGE_IDS


def test_0401_footprint_context_keeps_detection_polygon() -> None:
    raw_0401 = {
        "timestamp": 837_919_604_350,
        "agentStateList": [
            {
                "aircraftID": 6,
                "isUnmanned": True,
                "coordinate": {
                    "latitude": 37.914,
                    "longitude": 128.164,
                    "altitude": 744,
                },
                "unmannedInfo": {
                    "sensorInfo": {
                        "footprintCornerList": _footprint_context(6)["byAircraft"]["6"]
                    }
                },
            }
        ],
    }

    context = build_0401_footprint_context(raw_0401)

    assert context["timestamp"] == 837_919_604_350
    assert len(context["byAircraft"]["6"]) == 4
    assert context["positions"]["6"] == {
        "latitude": 37.914,
        "longitude": 128.164,
        "altitude": 744.0,
    }


def test_roi_and_first_target_detection_are_persisted_with_precise_kst_time(
    tmp_path: Path,
) -> None:
    payload = {
        "timestamp": 837_919_604_398,
        "source": "DSC",
        "roiInfo": {
            "aircraftID": 4,
            "coordinate": {
                "latitude": 37.676833153,
                "longitude": 128.091691868,
                "altitude": 609,
            },
            "fov": 2.4,
        },
        "targetList": [
            {
                "targetID": 50,
                "targetType": 1,
                "coordinate": {
                    "latitude": 37.914016678,
                    "longitude": 128.164728837,
                    "altitude": 744,
                },
                "watcher": {"aircraftID": 6},
                "targetInFrame": True,
                "isDestroyed": False,
                "threat": 0.9,
            }
        ],
    }
    integration = _Integration(
        [_packet(payload, footprint_context=_footprint_context(4, 6))]
    )

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        service = MissionStatusService(integration=integration)
        service._ensure_discoveries_loaded(str(tmp_path.resolve()))
        service._consume_0402_events()

        rows = list(service._discoveries)
        assert [row["kind"] for row in rows] == ["TARGET", "ROI"]
        assert rows[0]["targetID"] == 50
        assert rows[0]["watcherID"] == 6
        assert rows[0]["timeKst"] == "12:26:44.398"
        assert rows[0]["messageTimestamp"] == 837_919_604_398
        assert rows[0]["footprintTimestamp"] == 837_919_604_350
        assert rows[0]["footprintTimestampUnix"] == 1_784_604_404_350
        assert len(rows[0]["footprint"]) == 4
        assert rows[1]["aircraftID"] == 4
        assert len(rows[1]["footprint"]) == 4

        history_path = (
            tmp_path / "DSS_Internal" / "mission_status_0402_discoveries.jsonl"
        )
        persisted = [
            json.loads(line)
            for line in history_path.read_text(encoding="utf-8").splitlines()
        ]
        assert len(persisted) == 2
        assert {row["kind"] for row in persisted} == {"ROI", "TARGET"}
        assert all(len(row["footprint"]) == 4 for row in persisted)


def test_continuous_target_in_frame_updates_do_not_duplicate_discovery(
    tmp_path: Path,
) -> None:
    first = {
        "timestamp": 837_919_604_398,
        "targetList": [
            {
                "targetID": 50,
                "targetType": 1,
                "coordinate": {"latitude": 37.9, "longitude": 128.1},
                "targetInFrame": True,
            }
        ],
    }
    repeated = {
        "timestamp": 837_919_605_398,
        "targetList": [
            {
                "targetID": 50,
                "targetType": 1,
                "coordinate": {"latitude": 37.900001, "longitude": 128.100001},
                "targetInFrame": True,
            }
        ],
    }
    integration = _Integration([_packet(first), _packet(repeated)])

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        service = MissionStatusService(integration=integration)
        service._ensure_discoveries_loaded(str(tmp_path.resolve()))
        service._consume_0402_events()

    rows = [row for row in service._discoveries if row.get("kind") == "TARGET"]
    assert len(rows) == 1


def test_string_false_target_flags_do_not_create_false_detection(
    tmp_path: Path,
) -> None:
    payload = {
        "timestamp": 837_919_604_398,
        "targetList": [
            {
                "targetID": 50,
                "coordinate": {"latitude": 37.9, "longitude": 128.1},
                "targetInFrame": "false",
                "isDestroyed": "false",
            }
        ],
    }

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        service = MissionStatusService(integration=_Integration([_packet(payload)]))
        service._ensure_discoveries_loaded(str(tmp_path.resolve()))
        service._consume_0402_events()

    assert not service._discoveries


def test_new_target_sample_after_restart_is_recorded_once(
    tmp_path: Path,
) -> None:
    def target_payload(timestamp: int) -> dict:
        return {
            "timestamp": timestamp,
            "targetList": [
                {
                    "targetID": 50,
                    "coordinate": {"latitude": 37.9, "longitude": 128.1},
                    "targetInFrame": True,
                }
            ],
        }

    with patch(
        "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
        return_value=tmp_path,
    ):
        first_service = MissionStatusService(
            integration=_Integration([_packet(target_payload(837_919_604_398))])
        )
        first_service._ensure_discoveries_loaded(str(tmp_path.resolve()))
        first_service._consume_0402_events()

        restarted_service = MissionStatusService(
            integration=_Integration(
                [
                    _packet(target_payload(837_919_604_398)),
                    _packet(target_payload(837_919_605_398)),
                ]
            )
        )
        restarted_service._ensure_discoveries_loaded(str(tmp_path.resolve()))
        restarted_service._consume_0402_events()

    rows = [
        row
        for row in restarted_service._discoveries
        if row.get("kind") == "TARGET"
    ]
    assert len(rows) == 2
    assert {row["messageTimestamp"] for row in rows} == {
        837_919_604_398,
        837_919_605_398,
    }
