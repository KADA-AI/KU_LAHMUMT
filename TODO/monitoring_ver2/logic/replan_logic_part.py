# logic/replan_logic_part.py: '재계획 판단' 도메인에 대한 세부 비즈니스 로직을 구현합니다.

from .replan_actual_logic import run_replan_procedure

class ReplanLogic:
    def __init__(self, manager):
        self.manager = manager

    def execute(self, mode_override=None):
        """시스템 모드를 확인하고, 3 (임무수행 모드)일 경우에만 재계획 로직을 실행합니다."""
        system_mode = mode_override if mode_override is not None else self.manager.logic_store.get_data("SystemMode")
        
        if system_mode == 3:
            run_replan_procedure(self.manager)
