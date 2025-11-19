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


def _calculate_offset_progress(
    current_position: Optional[int],
    total_waypoints: int,
) -> int:
    """Calculate mission progress while skipping the very first waypoint."""
    if current_position is None:
        return 0
    try:
        pos_value = int(current_position)
    except (TypeError, ValueError):
        return 0
    if total_waypoints <= 1:
        return 100 if pos_value > 0 else 0
    effective_segments = max(total_waypoints - 1, 1)
    if pos_value <= 0:
        return 0
    capped_position = min(pos_value, effective_segments)
    return int(round((capped_position * 100) / effective_segments))


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

    raw_current_input_id = None
    if plan_input_ids:
        raw_current_input_id = plan_input_ids[0]
    elif plan_input_package_id is not None:
        raw_current_input_id = plan_input_package_id

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

        is_unmanned_raw = getattr(agent_state, "isUnmanned", 0)
        try:
            is_unmanned = int(is_unmanned_raw) == 1
        except (TypeError, ValueError):
            is_unmanned = bool(is_unmanned_raw)

        unmanned_info = None
        current_wp = None

        if is_unmanned:
            unmanned_info = getattr(agent_state, "unmannedInfo", None)
            if unmanned_info and getattr(unmanned_info, "currentWaypointID", None):
                current_wp = getattr(unmanned_info.currentWaypointID, "waypointID", None)
                try:
                    if current_wp is not None:
                        current_wp = int(current_wp)
                except (TypeError, ValueError):
                    pass
        else:
            # 유인기 1~3번은 모니터링 대상에서 제외
            if aircraft_id in (1, 2, 3):
                return entries, mission_progress
            # 유인/준비 상태인 경우에도 진행률 추적을 위해 WP를 0으로 간주
            current_wp = 0

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
                    "missionIndex": -1,
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
                    "missionIndex": -1,
                    "progress": progress_value,
                }
            )
            return entries, mission_progress

        current_idx = None
        current_pos = None
        if current_wp is None:
            current_idx = None
        elif current_wp in waypoint_map:
            current_idx, current_pos = waypoint_map[current_wp]
        else:
            # waypoint_map은 mission_context를 초기화한 시점의 경로만 포함한다.
            # 동일한 path를 공유하고 있지만 해당 waypoint가 빠져 있는 경우를 대비해
            # 우선 pathID가 동일한 임무를 찾아 fallback으로 진행률을 계산한다.
            fallback_idx = None
            fallback_pos = None
            if current_wp is not None:
                current_path_id = None
                matches = []
                for idx, mission in enumerate(missions):
                    path_id = mission.get("pathID")
                    if path_id is not None:
                        try:
                            path_id = int(path_id)
                        except (TypeError, ValueError):
                            path_id = None
                    if path_id is not None and path_id == current_path_id:
                        matches.append((idx, mission))
                    elif current_path_id is None:
                        # 첫 번째 매칭용
                        current_path_id = path_id
                        if current_path_id is not None:
                            matches.append((idx, mission))
                if not matches:
                    # path 정보를 활용하지 못할 때 waypoint 숫자가 증가하는 임무를 fallback
                    for idx, mission in enumerate(missions):
                        waypoints = mission.get("waypoints") or []
                        try:
                            waypoints_int = [int(wp) for wp in waypoints]
                        except (TypeError, ValueError):
                            waypoints_int = []
                        if waypoints_int and min(waypoints_int) <= current_wp <= max(
                            waypoints_int
                        ):
                            matches.append((idx, mission))
                if matches:
                    match_idx, mission = matches[0]
                    waypoints = mission.get("waypoints") or []
                    try:
                        waypoint_ints = [int(wp) for wp in waypoints]
                    except (TypeError, ValueError):
                        waypoint_ints = []
                    if waypoint_ints:
                        for pos, wp in enumerate(waypoint_ints):
                            if wp >= current_wp:
                                fallback_pos = pos
                                break
                        if fallback_pos is None:
                            fallback_pos = len(waypoint_ints)
                        fallback_idx = match_idx
            if fallback_idx is not None:
                current_idx, current_pos = fallback_idx, fallback_pos
            elif current_wp is not None:
                # Unable to map waypoint to any mission; reset progress to zero.
                current_idx = None
                current_pos = None

        # Waypoint ID 0은 특정 운용 모드에서 "미진입" 상태로 사용되므로 진행률을 0으로 강제.
        force_zero_progress = current_wp == 0

        if current_wp == 0:
            mission_to_check: Optional[Dict[str, Any]] = None
            if current_idx is not None and 0 <= current_idx < len(missions):
                mission_to_check = missions[current_idx]
            else:
                for mission in missions:
                    waypoints = mission.get("waypoints") or []
                    if 0 in waypoints:
                        mission_to_check = mission
                        break
            if mission_to_check is not None:
                waypoints = mission_to_check.get("waypoints") or []
                if waypoints and all(wp == 0 for wp in waypoints):
                    current_idx = None
                    current_pos = None

        progress_list: List[tuple[int, int, Optional[int]]] = []
        for idx, mission in enumerate(missions):
            mission_id = int(mission.get("individualMissionID") or 0)
            waypoints = mission.get("waypoints") or []
            waypoint_count = len(waypoints)
            safe_waypoint_total = waypoint_count if waypoint_count > 0 else 1
            mission_input_id = mission.get("inputMissionID")
            try:
                mission_input_id = (
                    int(mission_input_id) if mission_input_id is not None else None
                )
            except (TypeError, ValueError):
                mission_input_id = None

            is_transit_stage = False

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
                    progress = _calculate_offset_progress(
                        current_pos, safe_waypoint_total
                    )
                    if (
                        not force_zero_progress
                        and waypoint_count > 1
                        and current_pos <= 0
                    ):
                        is_transit_stage = True
            else:
                progress = 0

            if force_zero_progress:
                progress = 0
                is_transit_stage = False

            if (
                current_wp == 0
                and progress >= 100
                and (mission.get("waypoints") or [])
                and all(wp == 0 for wp in mission.get("waypoints") or [])
            ):
                progress = 0

            progress = max(0, min(progress, 100))
            progress_list.append((mission_id, progress, mission_input_id))
            mission_progress.append(
                {
                    "aircraftID": aircraft_id,
                    "individualMissionID": mission_id,
                    "pathID": mission.get("pathID"),
                    "inputMissionID": mission.get("inputMissionID"),
                    "missionIndex": idx,
                    "progress": progress,
                    "isTransitStage": is_transit_stage,
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
            health_code = getattr(agent_state, "health", 1)
            if health_code is not None and int(health_code) == 2:
                continue
        except Exception:
            pass
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
