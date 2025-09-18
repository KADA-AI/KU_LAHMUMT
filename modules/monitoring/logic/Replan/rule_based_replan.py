"""
규칙 기반 재계획 여부 판단 모듈
"""


class RuleBasedReplan:
    def __init__(self):
        pass

    def check_all_rules(self, combined_data):
        """통합된 데이터를 기반으로 모든 규칙을 확인하고 재계획 트리거 목록을 반환합니다."""
        triggers = []

        # 1. 개별 무인기 소실 판단 (0401 AgentState)
        agent_state_obj = combined_data.get("agent_state")
        if agent_state_obj and hasattr(agent_state_obj, 'agentStateList'):
            agent_state_list = agent_state_obj.agentStateList
            for uav_state in agent_state_list:
                # health: 0=비정상, 1=정상, 2=통신두절(소실)
                if hasattr(uav_state, 'health') and uav_state.health == 2:
                    vehicle_id = f"UAV {getattr(uav_state, 'aircraftID', 'Unknown')}"
                    triggers.append({
                        "MissionPlanningStatus": "무인기 임무 제외",
                        "ReplanReason": f"{vehicle_id} 소실",
                        "Priority": 1, # Highest priority
                        "ReplanType": "무인기 임무 제외",
                    })

        # 2. 개별 무인기 선행 임무 시작 판단 (0202 PriorMissionInfo)
        prior_mission_info_obj = combined_data.get("prior_mission_info")
        if prior_mission_info_obj and hasattr(prior_mission_info_obj, 'priorMissionList'):
            prior_mission_list = prior_mission_info_obj.priorMissionList
            if prior_mission_list:
                triggers.append({
                    "MissionPlanningStatus": "선행 임무 시작",
                    "ReplanReason": "선행 임무 시작",
                    "Priority": 2,
                    "ReplanType": "선행 임무 기반 임무 할당",
                })

        # 3. 임무 대기/재시작/종료 운용자 강제 명령 입력 판단 (0802 MandatoryCommand)
        mandatory_command_obj = combined_data.get("mandatory_command")
        if mandatory_command_obj:
            command_type = getattr(mandatory_command_obj, "mandatoryType", None)
            aircraft_id = getattr(mandatory_command_obj, "aircraftID", None)

            if command_type == 1:  # 1: 강제대기
                triggers.append({
                    "MissionPlanningStatus": "임무 대기",
                    "ReplanReason": f"운용자 강제 대기 명령 (대상: {aircraft_id})",
                    "Priority": 1,
                    "ReplanType": "임무 대기",
                })
            elif command_type == 2:  # 2: 강제귀환
                triggers.append({
                    "MissionPlanningStatus": "전체 임무 재계획",
                    "ReplanReason": f"운용자 강제 귀환 명령 (대상: {aircraft_id})",
                    "Priority": 1,
                    "ReplanType": "전체 임무 재계획",
                })
            elif command_type == 3:  # 3: 강제임무복귀
                triggers.append({
                    "MissionPlanningStatus": "임무 재시작",
                    "ReplanReason": f"운용자 강제 임무 복귀 명령 (대상: {aircraft_id})",
                    "Priority": 1,
                    "ReplanType": "임무 재시작",
                })

        return triggers