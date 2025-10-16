# logic/monitoring_actual_logic.py

from typing import Any, Dict, List, Optional

from data.message_models import (
    AgentStatusModel,
    MissionProgressBodyModel,
    IndividualMissionProgressStatusModel,
    IndividualMissionIDModel,
)
from datetime import datetime, timezone

from dataclasses import asdict


def run_monitoring_procedure(
    data_401: AgentStatusModel,
    plan_context: Optional[Dict[str, Any]] = None,
    mission_plan_id: Optional[int] = None,
):
    """
    401 메시지를 기반으로 임무 진행 상황을 계산하고,
    0501 MissionProgress 메시지 본문을 생성한다.
    """
    if not data_401 or not data_401.agentStateList:
        return None

    plan_context = plan_context or {}
    plan_aircraft: Dict[int, Dict[str, Any]] = plan_context.get("aircraft") or {}
    plan_input_ids: List[int] = plan_context.get("inputMissionIDs") or []
    plan_input_package_id = plan_context.get("inputMissionPackageID")
    try:
        if plan_input_package_id is not None:
            plan_input_package_id = int(plan_input_package_id)
    except (TypeError, ValueError):
        plan_input_package_id = None

    timestamp = int(
        (
            datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
        ).total_seconds()
        * 1000
    )

    individual_mission_progress: List[IndividualMissionProgressStatusModel] = []

    def _progress_entries_for_agent(agent_state, agent_plan):
        aircraft_id = int(getattr(agent_state, "aircraftID", 0))
        entries: List[IndividualMissionProgressStatusModel] = []
        if getattr(agent_state, "isUnmanned", 0) != 1:
            return entries

        unmanned_info = getattr(agent_state, "unmannedInfo", None)
        current_wp = None
        if unmanned_info and getattr(unmanned_info, "currentWaypointID", None):
            current_wp = getattr(unmanned_info.currentWaypointID, "waypointID", None)
            try:
                if current_wp is not None:
                    current_wp = int(current_wp)
            except (TypeError, ValueError):
                pass

        if not agent_plan:
            progress_value = int(current_wp or 0)
            entries.append(
                IndividualMissionProgressStatusModel(
                    aircraftID=aircraft_id,
                    currentIndividualMission=IndividualMissionIDModel(
                        individualMissionID=0
                    ),
                    currentIndividualMissionProgress=progress_value,
                )
            )
            return entries

        missions: List[Dict[str, Any]] = agent_plan.get("missions") or []
        waypoint_map: Dict[int, Any] = agent_plan.get("waypoint_map") or {}
        if not missions:
            progress_value = int(current_wp or 0)
            entries.append(
                IndividualMissionProgressStatusModel(
                    aircraftID=aircraft_id,
                    currentIndividualMission=IndividualMissionIDModel(
                        individualMissionID=0
                    ),
                    currentIndividualMissionProgress=progress_value,
                )
            )
            return entries

        current_idx = None
        current_pos = None
        if current_wp is None:
            current_idx = len(missions)
        elif current_wp in waypoint_map:
            current_idx, current_pos = waypoint_map[current_wp]

        for idx, mission in enumerate(missions):
            mission_id = int(mission.get("individualMissionID") or 0)
            total_waypoints = len(mission.get("waypoints") or []) or 1

            if current_idx is None:
                progress = 0
            elif current_idx >= len(missions):
                progress = 100
            elif idx < current_idx:
                progress = 100
            elif idx == current_idx:
                if current_pos is None:
                    progress = 0
                else:
                    progress = int(round((current_pos + 1) * 100 / total_waypoints))
            else:
                progress = 0

            progress = max(0, min(progress, 100))
            entries.append(
                IndividualMissionProgressStatusModel(
                    aircraftID=aircraft_id,
                    currentIndividualMission=IndividualMissionIDModel(
                        individualMissionID=mission_id
                    ),
                    currentIndividualMissionProgress=progress,
                )
            )
        return entries

    for agent_state in data_401.agentStateList:
        try:
            aid = int(getattr(agent_state, "aircraftID", 0))
        except (TypeError, ValueError):
            aid = getattr(agent_state, "aircraftID", 0)
        agent_plan = plan_aircraft.get(aid)
        individual_mission_progress.extend(
            _progress_entries_for_agent(agent_state, agent_plan)
        )

    mission_plan_value = mission_plan_id
    if mission_plan_value is None:
        mission_plan_value = plan_context.get("missionPlanID")
    mission_plan_value = int(mission_plan_value) if mission_plan_value else 0

    current_input_mission_id = plan_input_package_id or (plan_input_ids[0] if plan_input_ids else None)
    try:
        current_input_mission_id = int(current_input_mission_id) if current_input_mission_id is not None else 0
    except (TypeError, ValueError):
        current_input_mission_id = 0

    data = MissionProgressBodyModel(
        source="MSM",
        timestamp=timestamp,
        currentMissionPlanID=mission_plan_value,
        currentInputMissionID=current_input_mission_id,
        individualMissionProgressStatusList=individual_mission_progress,
    )

    body_0501 = asdict(data)
    return body_0501
