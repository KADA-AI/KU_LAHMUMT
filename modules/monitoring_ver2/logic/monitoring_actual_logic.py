# logic/monitoring_actual_logic.py

from typing import Any, Dict, List, Optional, Tuple

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
    
) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    401 메시지를 기반으로 임무 진행 상황을 계산하고,
    0501 MissionProgress 메시지 본문을 생성한다.
    """
    if not data_401 or not data_401.agentStateList:
        return None, []

    plan_context = plan_context or {}
    plan_aircraft: Dict[int, Dict[str, Any]] = plan_context.get("aircraft") or {}
    plan_input_ids: List[int] = plan_context.get("inputMissionIDs") or []
    plan_input_package_id = plan_context.get("inputMissionPackageID")
    try:
        if plan_input_package_id is not None:
            plan_input_package_id = int(plan_input_package_id)
    except (TypeError, ValueError):
        plan_input_package_id = None

    raw_current_input_id = (
        plan_input_package_id if plan_input_package_id is not None else (plan_input_ids[0] if plan_input_ids else None)
    )
    try:
        current_input_mission_id_int = (
            int(raw_current_input_id) if raw_current_input_id is not None else None
        )
    except (TypeError, ValueError):
        current_input_mission_id_int = None
    active_input_mission_id = current_input_mission_id_int
    plan_active_input = plan_context.get("activeInputMissionID")
    try:
        if plan_active_input is not None:
            plan_active_input = int(plan_active_input)
    except (TypeError, ValueError):
        plan_active_input = None
    if plan_active_input is not None:
        active_input_mission_id = plan_active_input

    timestamp = int(
        (
            datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
        ).total_seconds()
        * 1000
    )

    individual_mission_progress: List[IndividualMissionProgressStatusModel] = []
    mission_progress_snapshots: List[Dict[str, Any]] = []

    def _progress_entries_for_agent(agent_state, agent_plan):
        aircraft_id = int(getattr(agent_state, "aircraftID", 0))
        entries: List[IndividualMissionProgressStatusModel] = []
        mission_progress: List[Dict[str, Any]] = []
        if getattr(agent_state, "isUnmanned", 0) != 1:
            return entries, mission_progress

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
            mission_progress.append(
                {
                    "aircraftID": aircraft_id,
                    "individualMissionID": 0,
                    "pathID": None,
                    "inputMissionID": None,
                    "progress": progress_value,
                }
            )
            return entries, mission_progress

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
            mission_progress.append(
                {
                    "aircraftID": aircraft_id,
                    "individualMissionID": 0,
                    "pathID": None,
                    "inputMissionID": None,
                    "progress": progress_value,
                }
            )
            return entries, mission_progress

        current_idx = None
        current_pos = None
        if current_wp is None:
            current_idx = len(missions)
        elif current_wp in waypoint_map:
            current_idx, current_pos = waypoint_map[current_wp]

        progress_list: List[tuple[int, int, Optional[int]]] = []
        for idx, mission in enumerate(missions):
            mission_id = int(mission.get("individualMissionID") or 0)
            total_waypoints = len(mission.get("waypoints") or []) or 1
            mission_input_id = mission.get("inputMissionID")
            try:
                mission_input_id = (
                    int(mission_input_id) if mission_input_id is not None else None
                )
            except (TypeError, ValueError):
                mission_input_id = None

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
            progress_list.append((mission_id, progress, mission_input_id))
            mission_progress.append(
                {
                    "aircraftID": aircraft_id,
                    "individualMissionID": mission_id,
                    "pathID": mission.get("pathID"),
                    "inputMissionID": mission.get("inputMissionID"),
                    "progress": progress,
                }
            )

        chosen_id = 0
        chosen_progress = 0
        chosen_entry: Optional[tuple[int, int]] = None
        matching_fallback: Optional[tuple[int, int]] = None
        nonmatching_fallback: Optional[tuple[int, int]] = None

        for mid, prog, mission_input_id in progress_list:
            matches_active = (
                active_input_mission_id is None
                or mission_input_id is None
                or mission_input_id == active_input_mission_id
            )
            entry = (mid, prog)
            if matches_active:
                matching_fallback = entry
                if prog < 100:
                    chosen_entry = entry
                    break
            else:
                if nonmatching_fallback is None:
                    nonmatching_fallback = entry

        if chosen_entry is not None:
            chosen_id, chosen_progress = chosen_entry
        elif matching_fallback is not None:
            chosen_id, chosen_progress = matching_fallback
        elif nonmatching_fallback is not None:
            chosen_id, chosen_progress = nonmatching_fallback
        elif progress_list:
            chosen_id, chosen_progress = progress_list[-1][:2]

        entries.append(
            IndividualMissionProgressStatusModel(
                aircraftID=aircraft_id,
                currentIndividualMission=IndividualMissionIDModel(
                    individualMissionID=chosen_id
                ),
                currentIndividualMissionProgress=chosen_progress,
            )
        )
        return entries, mission_progress

    for agent_state in data_401.agentStateList:
        try:
            aid = int(getattr(agent_state, "aircraftID", 0))
        except (TypeError, ValueError):
            aid = getattr(agent_state, "aircraftID", 0)
        agent_plan = plan_aircraft.get(aid)
        entries, mission_snapshot = _progress_entries_for_agent(
            agent_state, agent_plan
        )
        individual_mission_progress.extend(entries)
        mission_progress_snapshots.extend(mission_snapshot)

    mission_plan_value = mission_plan_id
    if mission_plan_value is None:
        mission_plan_value = plan_context.get("missionPlanID")
    mission_plan_value = int(mission_plan_value) if mission_plan_value else 0

    output_input_mission_id = (
        active_input_mission_id
        if active_input_mission_id is not None
        else current_input_mission_id_int
    )
    if output_input_mission_id is None:
        current_input_mission_id = 0
    else:
        current_input_mission_id = output_input_mission_id

    data = MissionProgressBodyModel(
        source="MSM",
        timestamp=timestamp,
        currentMissionPlanID=mission_plan_value,
        currentInputMissionID=current_input_mission_id,
        individualMissionProgressStatusList=individual_mission_progress,
    )

    body_0501 = asdict(data)
    return body_0501, mission_progress_snapshots
