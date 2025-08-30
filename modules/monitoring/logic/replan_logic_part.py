# logic/replan_logic_part.py

class ReplanLogic:
    def __init__(self, manager):
        self.manager = manager

    def execute(self):
        """시스템 모드를 확인하고, 'replan'일 경우에만 로직을 실행합니다."""
        system_mode = self.manager.logic_store.get_data("system_mode")
        
        if system_mode == "replan":
            # 향후 실제 재계획 판단 로직 추가
            self.manager._log("REPLAN_LOGIC", "EXEC", "재계획 판단 로직 실행됨.")
