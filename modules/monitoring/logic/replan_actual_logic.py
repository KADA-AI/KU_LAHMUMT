# logic/replan_actual_logic.py: 실제 재계획 판단 로직을 수행하는 함수를 정의합니다.

from logic.Replan.replan_management import ReplanManager
from data.logic_storage import LogicStorage

def run_replan_procedure(manager):
    """
    실제 재계획 판단 로직의 시작점입니다.
    ReplanManager를 사용하여 재계획 프로세스를 관리합니다.
    """
    manager._log("REPLAN_PROCEDURE", "INFO", "실제 재계획 판단 로직 실행 시작.")

    # ReplanManager 인스턴스 생성
    replan_manager = ReplanManager()

    # 재계획 프로세스 실행
    final_replan_output, _ = replan_manager.manage_replan()

    # 최종 결과 저장
    logic_storage = LogicStorage()
    logic_storage.set_data(
        "final_replan_output",
        final_replan_output,
    )
    
    manager._log("REPLAN_PROCEDURE", "INFO", f"최종 재계획 결과: {final_replan_output}")
    manager._log("REPLAN_PROCEDURE", "INFO", "실제 재계획 판단 로직 실행 종료.")