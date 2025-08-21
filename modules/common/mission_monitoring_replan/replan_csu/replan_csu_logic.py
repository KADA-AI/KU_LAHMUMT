# c:\Users\HJW\Documents\Dev\MUMT\nFusion\mission_monitoring_replan\replan_csu\replan_csu_logic.py
"""
재계획 CSU 로직: 재계획
CSC로부터 전달받은 데이터를 기반으로 재계획 관련 로직을 수행합니다.
UI나 NodeMessenger에 직접 의존하지 않습니다.
"""

import time  # 예시용


def _perform_replan_calculation(data: dict) -> dict:
    """
    재계획 계산 로직 예시
    """
    print(f"[ReplanCSULogic] 재계획 계산 시작 (데이터 키: {list(data.keys())})")
    # 실제 재계획 알고리즘 구현
    time.sleep(0.2)  # 작업 시뮬레이션
    trigger_info_data = (
        data.get("trigger_info") if data.get("trigger_info") is not None else {}
    )
    replan_plan = {
        "plan_id": f"replan_{int(time.time())}",
        "status": "calculated",
        "proposed_actions": ["action_A", "action_B"],
        "based_on_data": trigger_info_data.get("reason"),
    }
    print(f"[ReplanCSULogic] 재계획 계산 완료: {replan_plan}")
    return replan_plan


class ReplanCSUHandler:
    def __init__(self):
        self._replan_state = {}  # 재계획 CSU 내부 상태
        print("[ReplanCSUHandler] 재계획 CSU 로직 핸들러 초기화됨.")

    def run_replan(self, csc_provided_data: dict) -> dict:
        """
        CSC로부터 데이터를 받아 재계획 로직을 실행합니다.
        """
        self._replan_state["last_input"] = csc_provided_data
        return _perform_replan_calculation(csc_provided_data)
