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
        agent_state_list = combined_data.get("agent_state", {}).get("agentStateList", [])
        for uav in agent_state_list:
            # health: 0=비정상, 1=정상, 2=통신두절(소실)
            if uav.get("health") == 2:
                vehicle_id = f"UAV {uav.get('aircraftID', 'Unknown')}"
                triggers.append({
                    "MissionPlanningStatus": "무인기 임무 제외",
                    "ReplanReason": f"{vehicle_id} 소실",
                    "Priority": 1, # Highest priority
                    "ReplanType": "무인기 임무 제외",
                })

        # 2. 개별 무인기 선행 임무 시작 판단 (0202 PriorMissionInfo)
        prior_mission_list = combined_data.get("prior_mission_info", {}).get("priorMissionList", [])
        if prior_mission_list:
            triggers.append({
                "MissionPlanningStatus": "선행 임무 시작",
                "ReplanReason": "선행 임무 시작",
                "Priority": 2,
                "ReplanType": "선행 임무 기반 임무 할당",
            })

        # 3. 임무 대기/재시작/종료 운용자 강제 명령 입력 판단 (0802 MandatoryCommand)
        mandatory_command = combined_data.get("mandatory_command", {})
        command_type = mandatory_command.get("mandatoryType")
        aircraft_id = mandatory_command.get("aircraftID")

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