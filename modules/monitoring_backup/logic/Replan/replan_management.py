# logic/Replan/replan_management.py

class ReplanManager:
    def __init__(self):
        # 필요한 초기화 수행
        self.triggers = []

    def manage_replan(self, agent_state, mandatory_command, prior_mission_info):
        """
        재계획 프로세스를 관리하는 메인 메서드
        """
        # 1. 재계획 판단
        is_replan_needed, judgment_reason = self._judge_replan_necessity(agent_state, mandatory_command)

        if not is_replan_needed:
            return None, None

        # 2. 재계획 트리거 관리
        trigger = self._manage_triggers(judgment_reason)
        self.triggers.append(trigger)

        # 3. 재계획 수준 결정 및 재계획 요청 메시지 생성 (여기서는 요청 내용만 반환)
        replan_request_content, replan_level = self._determine_replan_level_and_request(trigger)

        return replan_request_content, trigger, replan_level

    def _judge_replan_necessity(self, agent_state, mandatory_command):
        """
        규칙 기반으로 재계획 필요성을 판단합니다.
        - 예: 유인기/무인기 상태, 임무 수행 상태, 위협 정보 등을 기반으로 판단
        """
        # === 규칙 기반 판단 로직 구현 ===
        # 예시: 강제 명령(0802)이 있으면 재계획 필요
        if mandatory_command:
            return True, "Mandatory command received"

        # 예시: 에이전트 상태가 특정 임계값을 넘으면 재계획 필요
        # if agent_state['health'] < 50:
        #     return True, "Agent health critical"

        return False, None

    def _manage_triggers(self, reason):
        """
        재계획 트리거를 생성하고 관리합니다.
        """
        # === 트리거 관리 로직 구현 ===
        # 예시: 판단 이유를 기반으로 트리거 객체 생성
        trigger = {
            "timestamp": "...",
            "reason": reason,
            "type": "RuleBased"
        }
        return trigger

    def _determine_replan_level_and_request(self, trigger):
        """
        트리거에 따라 재계획 수준을 결정하고,
        송신할 재계획 요청 메시지의 내용을 생성합니다.
        """
        # === 재계획 수준 결정 및 요청 생성 로직 ===
        # 예시: 트리거 이유에 따라 다른 재계획 수준과 요청 내용 결정
        if "Mandatory command" in trigger['reason']:
            replan_level = 1 # 예: 상위 수준 재계획
            request_content = "Executing mandatory command."
        else:
            replan_level = 3 # 예: 하위 수준 재계획
            request_content = "Responding to critical agent state."

        # 실제 0902 메시지 생성은 replan_actual_logic.py에서 처리하도록
        # 여기서는 재계획 요청에 들어갈 핵심 내용과 재계획 수준을 반환
        return request_content, replan_level