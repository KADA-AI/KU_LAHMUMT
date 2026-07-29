from __future__ import annotations

from modules.mission_status_monitoring.service import MissionStatusService


def test_path_geometry_keeps_input_mission_identity_for_map_focus() -> None:
    service = MissionStatusService()
    mission = {
        "inputMissionPlans": [
            {
                "inputMissionList": [
                    {
                        "inputMissionID": 41,
                        "inputMissionType": 3,
                        "regionType": 6,
                        "missionDetail": {
                            "areaList": [
                                {
                                    "coordinateList": [
                                        {"latitude": 35.0, "longitude": 128.0},
                                        {"latitude": 35.0, "longitude": 128.01},
                                        {"latitude": 35.01, "longitude": 128.01},
                                    ]
                                }
                            ]
                        },
                    }
                ]
            }
        ],
        "pathMissionIndex": {
            "7001": {
                "inputMissionID": 41,
                "individualMissionID": 8101,
            }
        },
        "features": [
            {
                "aircraftId": 4,
                "pathId": 7001,
                "coords": [[128.0, 35.0], [128.01, 35.01]],
            }
        ],
    }

    geometry, _bounds = service._build_geometry(mission)

    path = geometry["paths"]["features"][0]
    assert path["properties"]["pathID"] == 7001
    assert path["properties"]["inputMissionID"] == 41
    assert path["properties"]["individualMissionID"] == 8101

