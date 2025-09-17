import time

from .input_manager import csc_data_store, update_data_store
from .rule_based_replan import RuleBasedReplan
from .priority_based_replan import PriorityBasedReplan
from .trigger_management import TriggerManagement
from .statistical_replan import StatisticalReplan
from data.logic_model import FinalReplanOutput

# from .dbnn_monitor import DBNNMonitor


class ReplanManager:
    def __init__(self):
        self.rule_based_replan = RuleBasedReplan()
        self.priority_based_replan = PriorityBasedReplan()
        self.statistical_replan = StatisticalReplan()

    def manage_replan(self):
        # 1. 데이터 저장소에서 최신 데이터로 업데이트
        update_data_store()

        if csc_data_store is None:
            return FinalReplanOutput(
                new_plan={"status": "error", "reason": "data_store_not_loaded"},
                replan_status="ERROR",
                final_replan_type=None,
            ), None

        # 2. 규칙 기반 재계획에 필요한 모든 데이터를 추출하여 하나의 딕셔너리로 통합
        combined_data = {
            "agent_state": csc_data_store["latest_0401_agent_state"],
            "mandatory_command": csc_data_store["latest_0802_mandatory_command"],
            "prior_mission_info": csc_data_store["latest_0202_prior_mission_info"],
        }

        # 규칙 기반 재계획 여부 판단
        rule_trigger = self.rule_based_replan.check_all_rules(combined_data)

        # 통계적 기반 재계획 판단 (현재는 더미 데이터 사용)
        dummy_dbnn_result = {
            "Schedule_Adherence_Risk": 0.1,
            "Sustainability_Risk": 0.1,
            "Operational_Risk": 0.1,
            "Collision_Risk": 0.1,
            "Enemy_Risk": 0.1,
            "Probability_to_Kill": 0.1,
            "Mission_Success_Rate": 0.1,
        }
        stats_trigger = self.statistical_replan.analyze_statistics(
            dummy_dbnn_result
        )

        # 우선순위 기반 재계획
        final_trigger = self.priority_based_replan.determine_priority(
            rule_trigger, stats_trigger
        )
        
        final_replan_type = final_trigger.get("ReplanType") if final_trigger else None

        # 최종 재계획 결과 생성
        if final_trigger:
            final_replan_output = FinalReplanOutput(
                new_plan={"status": "replan_needed"},
                replan_status="TRIGGERED",
                final_replan_type=final_replan_type,
            )
        else:
            final_replan_output = FinalReplanOutput(
                new_plan={"status": "no_change"},
                replan_status="COMPLETED",
                final_replan_type=None,
            )
        
        return final_replan_output, final_trigger


class RPCSUManagementModule:

    def __init__(self):
        self.replan_manager = ReplanManager()
        self.trigger_management = TriggerManagement()
        print("🚀 재계획 관리 모듈 실행 시작. (종료하려면 Ctrl+C를 누르세요)")

    def execute(self):
        try:
            while True:
                final_output, final_trigger = self.replan_manager.manage_replan()
                if final_trigger:
                    self.trigger_management.manage_triggers(final_trigger)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n🛑 프로그램 실행이 중단되었습니다.")