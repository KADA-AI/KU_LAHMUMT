from __future__ import annotations

import json
from pathlib import Path

from modules.mission_planning.runtime import json_io


def _flight_path_payload() -> dict:
    return {
        "pathID": 123,
        "aircraftID": 4,
        "source": "MMR/test",
        "waypointList": [
            {
                "waypointID": 456,
                "latitude": 35.12345678901234,
                "longitude": 128.98765432109876,
                "altitude": 150.0,
                "description": "경로/촬영",
            }
        ],
    }


def test_flightpath_fast_serializer_preserves_json_values() -> None:
    payload = _flight_path_payload()
    encoded = json_io.serialize_json_payload(
        Path("FlightPath") / "123.json",
        payload,
    )

    assert json.loads(encoded.decode("utf-8")) == payload


def test_flightpath_fast_serializer_matches_historical_bytes_for_normal_payload() -> None:
    payload = _flight_path_payload()
    encoded = json_io.serialize_json_payload(
        Path("FlightPath") / "123.json",
        payload,
    )
    expected = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")

    assert encoded == expected


def test_compact_flightpath_fast_serializer_matches_historical_bytes() -> None:
    payload = _flight_path_payload()
    encoded = json_io.serialize_json_payload(
        Path("FlightPath") / "123.json",
        payload,
        pretty=False,
    )
    expected = json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")

    assert encoded == expected


def test_non_flightpath_artifact_keeps_historical_serializer(monkeypatch) -> None:
    payload = {"missionPlanID": 700000001, "aircraftList": []}

    class _UnexpectedFastSerializer:
        @staticmethod
        def dumps(*_args, **_kwargs):
            raise AssertionError("ujson must remain FlightPath-only")

    monkeypatch.setattr(json_io, "_ujson", _UnexpectedFastSerializer())
    encoded = json_io.serialize_json_payload(
        Path("MissionPlan") / "700000001.json",
        payload,
    )

    assert encoded == json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")


def test_optional_fast_serializer_failure_falls_back(monkeypatch) -> None:
    payload = _flight_path_payload()

    class _FailingFastSerializer:
        @staticmethod
        def dumps(*_args, **_kwargs):
            raise TypeError("unsupported optional serializer")

    monkeypatch.setattr(json_io, "_ujson", _FailingFastSerializer())
    encoded = json_io.serialize_json_payload(
        Path("FlightPath") / "123.json",
        payload,
    )

    assert encoded == json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
