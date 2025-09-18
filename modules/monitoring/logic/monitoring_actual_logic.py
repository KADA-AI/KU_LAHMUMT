# logic/monitoring_actual_logic.py

from data.message_models import (
    AgentStatusModel,
    MissionProgressBodyModel,
    IndividualMissionProgressStatusModel,
    IndividualMissionIDModel,
)
from datetime import datetime, timezone

from dataclasses import asdict


def run_monitoring_procedure(data_401: AgentStatusModel):
    """
    401 데이터를 기반으로 임무 수행 상태를 판단하고,
    0501 메시지 본문을 생성합니다.
    """
    if not data_401 or not data_401.agentStateList:
        return None

    mission_status = 1  # 1: 정상
    for agent_state in data_401.agentStateList:
        if agent_state.health == 0:  # health가 0이면 비정상으로 판단
            mission_status = 2  # 2: 비정상
            break

    timestamp = int(
        (
            datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
        ).total_seconds()
        * 1000
    )

    individualMissionProgressStatus = []

    for agent_state in data_401.agentStateList:
        if agent_state.isUnmanned == 1:
            data = IndividualMissionProgressStatusModel(
                aircraftID=agent_state.aircraftID,
                currentIndividualMission=IndividualMissionIDModel(
                    individualMissionID=34567
                ),
                currentIndividualMissionProgress=agent_state.unmannedInfo.currentWaypointID.waypointID
                * 100
                // 100,
            )
            individualMissionProgressStatus.append(data)

    data = MissionProgressBodyModel(
        timestamp=timestamp,
        currentMissionPlanID=12345,  # 전체임무계획
        currentInputMissionID=23456,  # 협업기저임무?
        individualMissionProgressStatusList=individualMissionProgressStatus,
    )

    body_0501 = asdict(data)
    return body_0501
