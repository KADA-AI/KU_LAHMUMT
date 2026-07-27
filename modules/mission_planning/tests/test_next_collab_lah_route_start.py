from __future__ import annotations

import math

from modules.mission_planning.replanning.triggers.next_collab import pipeline


def test_live_lah_coordinates_supply_next_collab_route_starts(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline.agent_status_snapshot,
        "load_agent_status_snapshot",
        lambda: {
            "agent_states": [
                {
                    "aircraftID": 1,
                    "mannedInfo": {
                        "coordinate": {
                            "latitude": 37.1,
                            "longitude": 127.1,
                            "altitude": 500,
                        }
                    },
                },
                {
                    "aircraftID": 2,
                    "coordinate": {
                        "latitude": 37.2,
                        "longitude": 127.2,
                        "altitude": 600,
                    },
                },
            ]
        },
    )

    starts = pipeline._load_lah_route_start_coordinates(
        {
            1: {
                "latitude": 38.0,
                "longitude": 128.0,
                "altitude": 700,
            }
        }
    )

    # Explicit next-collab entry data wins; the latest 0401 fills missing LAHs.
    assert starts[1]["latitude"] == 38.0
    assert starts[2]["latitude"] == 37.2
    assert 3 not in starts


def test_moving_lah_route_start_is_projected_ten_seconds_ahead(monkeypatch) -> None:
    monkeypatch.setattr(
        pipeline.agent_status_snapshot,
        "load_agent_status_snapshot",
        lambda: {
            "agent_states": [
                {
                    "aircraftID": 2,
                    "coordinate": {
                        "latitude": 37.0,
                        "longitude": 127.0,
                        "altitude": 600,
                    },
                    "velocity": {"heading": 90.0, "speed": 50.0},
                }
            ]
        },
    )
    monkeypatch.setattr(
        pipeline,
        "get_runtime_float",
        lambda _key, default: default,
    )

    starts = pipeline._load_lah_route_start_coordinates()

    projected = starts[2]
    east_m = (
        (float(projected["longitude"]) - 127.0)
        * 111_320.0
        * math.cos(math.radians(37.0))
    )
    assert 495.0 <= east_m <= 505.0
    assert abs(float(projected["latitude"]) - 37.0) < 1e-5
    assert projected["altitude"] == 600
