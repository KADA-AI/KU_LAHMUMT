# logic/replan_logic_part.py: '재계획 판단' 도메인에 대한 세부 비즈니스 로직을 구현합니다.

from .replan_actual_logic import run_replan_procedure
# 새로 만든 유틸리티 파일에서 로깅 함수를 가져옵니다.
from .logic_utils import log_to_file

class ReplanLogic:
    def __init__(self, manager):
        self.manager = manager

    def execute(self, mode_override=None):
        """시스템 모드를 확인하고, 3 (임무수행 모드)일 경우에만 재계획 로직을 실행합니다."""
        system_mode = mode_override if mode_override is not None else self.manager.logic_store.get_data("SystemMode")
        
        log_msg = f"--- [ReplanLogic.execute] Current SystemMode is {system_mode}. Checking if it's 3. ---"
        log_to_file(log_msg)
        print(log_msg)
        
        if system_mode == 3:
            log_msg = "--- [ReplanLogic.execute] SystemMode is 3. Calling run_replan_procedure. ---"
            log_to_file(log_msg)
            print(log_msg)
            run_replan_procedure(self.manager)